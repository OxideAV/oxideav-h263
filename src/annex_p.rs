//! Annex P — Reference Picture Resampling (RPR).
//!
//! This module implements the §P.3 resampling process that warps the
//! previous decoded reference picture into a "warped" picture for use
//! as the motion-compensation reference of the current picture. Two
//! invocation paths exist:
//!
//! * **Implicit** (§P.1): when `PLUSPTYPE` is present, the Reference
//!   Picture Resampling bit is *not* set, and the current picture is an
//!   INTER / B / EP picture or Improved PB-frame whose picture size
//!   differs from the temporally previous encoded picture. The
//!   resampling runs with warping parameters all zero, fill mode = clip
//!   and displacement accuracy = 1/16-pixel (`WDA = "11"`). This is the
//!   "predictively-encoded change in picture resolution" case (e.g. a
//!   CIF → 4CIF transition).
//! * **Explicit** (§P.2): the RPR bit *is* set and an `RPRP` field in
//!   the picture header carries a §P.2.1 `WDA`, eight §P.2.2 warping
//!   parameters (Table D.3 VLC), a §P.2.3 fill mode, and an optional
//!   §P.2.4 fill colour.
//!
//! ## Algorithm (§P.3 / §P.4.2)
//!
//! The eight integer warping parameters
//! `w_x0, w_0y, w_xx, w_yx, w_xy, w_yy, w_xxy, w_yxy` define the corner
//! displacements `u^{00}, u^{H0}, u^{0V}, u^{HV}` in 1/32-pixel units
//! (luminance) per §P.3. From the corners, the per-line displacement
//! endpoints `u_xL(j), u_yL(j), u_xR(j), u_yR(j)` are computed by
//! one-dimensional interpolation across the virtual-frame height
//! `SV'`. The §P.4.2 raster loop then walks each line accumulating the
//! transformed sub-pixel position and runs the §P.3 four-tap bilinear
//! `filter` against the reference, clamping out-of-range samples per
//! the §P.2.3 fill mode.
//!
//! All arithmetic is integer; `///` (truncation toward negative
//! infinity) is realised with arithmetic shifts / [`i64::div_euclid`],
//! and `//` (round-half-away-from-zero) with the module's
//! `div_round_half_away` helper.
//!
//! ## Scope
//!
//! The general warp engine in this module handles arbitrary warping
//! parameters, all four fill modes, and both `WDA` accuracies. The
//! picture driver wires the **implicit** path (the common
//! resolution-change case) into end-to-end reconstruction; the explicit
//! `RPRP` field is parsed by [`crate::plus_ptype`] and threaded through
//! as [`RprParams`].

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// §P.2.3 / Table P.1 — fill-mode action for pixels whose computed
/// reference location lies outside the reference picture area.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillMode {
    /// `00` — a fill colour is sent in the §P.2.4 `Y_FILL` / `CB_FILL` /
    /// `CR_FILL` fields and used for out-of-range samples.
    Color,
    /// `01` — luminance samples become `Y = 16`, chrominance
    /// `CB = CR = 128`.
    Black,
    /// `10` — luminance and chrominance both become `128`.
    Gray,
    /// `11` — out-of-range coordinates are clipped to the border (edge
    /// extrapolation, as in Unrestricted Motion Vector mode).
    Clip,
}

impl FillMode {
    /// Decode the two §P.2.3 fill-mode bits.
    pub fn from_bits(bits: u8) -> FillMode {
        match bits & 0b11 {
            0b00 => FillMode::Color,
            0b01 => FillMode::Black,
            0b10 => FillMode::Gray,
            _ => FillMode::Clip,
        }
    }

    /// The luminance fill value for the non-clip modes. (`Color` carries
    /// its own value; this returns the default until that value is
    /// substituted in by the caller.)
    fn luma_fill(self, color_y: u8) -> i64 {
        match self {
            FillMode::Black => 16,
            FillMode::Gray => 128,
            FillMode::Color => i64::from(color_y),
            FillMode::Clip => 0, // unused for clip
        }
    }

    /// The chrominance fill value for the non-clip modes.
    fn chroma_fill(self, color_c: u8) -> i64 {
        match self {
            FillMode::Black => 128,
            FillMode::Gray => 128,
            FillMode::Color => i64::from(color_c),
            FillMode::Clip => 0, // unused for clip
        }
    }
}

/// §P.2.1 — Warping Displacement Accuracy: selects the sub-pixel
/// granularity `P` of the per-pixel displacements.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wda {
    /// `WDA = "10"` — half-pixel accuracy, `P = 2`.
    Half,
    /// `WDA = "11"` (or absent / implicit) — 1/16-pixel accuracy,
    /// `P = 16`.
    Sixteenth,
}

impl Wda {
    /// The accuracy divisor `P` (§P.3): `P = 2` for half-pixel,
    /// `P = 16` for 1/16-pixel.
    pub fn p(self) -> i64 {
        match self {
            Wda::Half => 2,
            Wda::Sixteenth => 16,
        }
    }
}

/// Fully-resolved §P.2 reference-picture-resampling parameters that
/// control a single warp of the reference picture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RprParams {
    /// §P.2.1 displacement accuracy.
    pub wda: Wda,
    /// §P.2.2 eight integer warping parameters
    /// `[w_x0, w_0y, w_xx, w_yx, w_xy, w_yy, w_xxy, w_yxy]` (half-pixel
    /// luminance offsets, range -4095..=4095).
    pub warp: [i32; 8],
    /// §P.2.3 fill mode.
    pub fill: FillMode,
    /// §P.2.4 fill colour `Y` (used only when `fill == Color`).
    pub fill_y: u8,
    /// §P.2.4 fill colour `CB`.
    pub fill_cb: u8,
    /// §P.2.4 fill colour `CR`.
    pub fill_cr: u8,
    /// §P.3 `RCRPR` rounding control (the picture's `RTYPE` bit; `0` or
    /// `1`).
    pub rcrpr: i64,
}

impl RprParams {
    /// The §P.1 *implicit* parameter set: zero warping, clip fill,
    /// 1/16-pixel accuracy. `rcrpr` is the current picture's `RTYPE`
    /// bit.
    pub fn implicit(rcrpr: bool) -> RprParams {
        RprParams {
            wda: Wda::Sixteenth,
            warp: [0; 8],
            fill: FillMode::Clip,
            fill_y: 0,
            fill_cb: 0,
            fill_cr: 0,
            rcrpr: rcrpr as i64,
        }
    }
}

/// §P.2.2 / §D.3 — read one Table D.3 reversible motion-vector
/// codeword, returning its **signed** half-pixel value in
/// `-4095..=4095`.
///
/// The table is regularly constructed (§D, "Table D.3 is a regularly
/// constructed reversible table"). The codeword interleaves the binary
/// magnitude with continue/terminate markers, then a sign bit:
///
/// * The single bit `1` codes magnitude `0` (no sign bit).
/// * Otherwise the codeword opens with `0`, then a sequence of
///   `(bit, marker)` pairs. While `marker == 1` the `bit` is the next
///   data bit of the magnitude (which carries an implicit leading
///   `1`). The pair whose `marker == 0` ends the codeword and its
///   `bit` is the sign (`0` positive, `1` negative).
///
/// Worked §D.3 example: `-13` (`s = 1`, magnitude `1101`) is encoded
/// `0 x2 1 x1 1 x0 1 s 0` = `0 11 01 11 10`.
pub fn read_table_d3(reader: &mut BitReader<'_>) -> Result<i32> {
    // First bit: "1" => the codeword is just "1", magnitude 0. No sign
    // bit follows for the zero codeword.
    let first = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if first {
        return Ok(0);
    }
    // Codeword began with "0": the magnitude has an implicit leading
    // "1". Read (bit, marker) pairs. While marker == 1, `bit` extends
    // the magnitude; the marker == 0 pair carries the sign and ends.
    let mut magnitude: i64 = 1;
    let sign;
    loop {
        let bit = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        let marker = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        if marker {
            magnitude = (magnitude << 1) | (bit as i64);
            if magnitude > 4095 {
                return Err(Error::BadMvdCode);
            }
        } else {
            sign = bit;
            break;
        }
    }
    Ok(if sign {
        -(magnitude as i32)
    } else {
        magnitude as i32
    })
}

/// §P.2 — parse the `RPRP` field of the picture header into
/// [`RprParams`]. `size_changed` is true when the current picture has a
/// different size than the reference (it does not affect parsing here
/// but documents that the warping parameters are still explicit when
/// present). `rcrpr` is the current picture's `RTYPE` bit.
///
/// This handles the INTER / B / Improved-PB picture case (eight warping
/// parameters + emulation-prevention bits + fill mode + optional fill
/// colour). The EP-picture SNR/spatial cases (§P.2.2 paragraph 2),
/// which reuse or refine the lower-layer parameters, are not parsed
/// here.
pub fn parse_rprp(reader: &mut BitReader<'_>, rcrpr: bool) -> Result<RprParams> {
    // §P.2.1 — WDA (2 bits). "10" => half, "11" => 1/16, else reserved.
    let wda_bits = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)?;
    let wda = match wda_bits {
        0b10 => Wda::Half,
        0b11 => Wda::Sixteenth,
        _ => return Err(Error::PlusPtypeReservedField),
    };

    // §P.2.2 — eight warping parameters in the order
    //   w_x0, w_0y, w_xx, w_yx, w_xy, w_yy, w_xxy, w_yxy
    // sent like UMV MVD pairs (Table D.3), with an emulation-prevention
    // "1" after a pair whose two codewords were both the all-zero (value
    // +1 / magnitude 0) codeword.
    let mut warp = [0i32; 8];
    let mut i = 0usize;
    while i < 8 {
        let a = read_table_d3(reader)?;
        let b = read_table_d3(reader)?;
        warp[i] = a;
        warp[i + 1] = b;
        // The "value +1" all-zero codeword corresponds to magnitude 0
        // here (read_table_d3 returns 0 for the "1" codeword). When both
        // members of the pair were the zero codeword, a single "1"
        // emulation-prevention bit follows (§P.2.2 / §D.3).
        if a == 0 && b == 0 {
            let epb = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if !epb {
                return Err(Error::PlusPtypeReservedField);
            }
        }
        i += 2;
    }

    // §P.2.3 — FILL_MODE (2 bits).
    let fill_bits = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)? as u8;
    let fill = FillMode::from_bits(fill_bits);

    // §P.2.4 — fill colour (26 bits) present only when fill mode = color.
    let (fill_y, fill_cb, fill_cr) = if fill == FillMode::Color {
        let y = reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8;
        // CB_EPB (1 bit, = 1).
        let cb_epb = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        if !cb_epb {
            return Err(Error::PlusPtypeReservedField);
        }
        let cb = reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8;
        // CR_EPB (1 bit, = 1).
        let cr_epb = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        if !cr_epb {
            return Err(Error::PlusPtypeReservedField);
        }
        let cr = reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8;
        (y, cb, cr)
    } else {
        (0, 0, 0)
    };

    Ok(RprParams {
        wda,
        warp,
        fill,
        fill_y,
        fill_cb,
        fill_cr,
        rcrpr: rcrpr as i64,
    })
}

/// Integer division `//` of §P.3: rounds the quotient to the nearest
/// integer and rounds half-integer values **away from zero**.
fn div_round_half_away(num: i64, den: i64) -> i64 {
    debug_assert!(den > 0);
    if num >= 0 {
        (num + den / 2) / den
    } else {
        -((-num + den / 2) / den)
    }
}

/// Integer division `///` of §P.3: truncation toward **negative
/// infinity** (Euclidean-style floor division for positive divisor).
fn div_floor(num: i64, den: i64) -> i64 {
    debug_assert!(den > 0);
    num.div_euclid(den)
}

/// Smallest power of two `>= n` (the §P.3 virtual-frame dimension
/// `H' = 2^m >= H`).
fn next_pow2_ge(n: i64) -> i64 {
    let mut v: i64 = 1;
    while v < n {
        v <<= 1;
    }
    v
}

/// A single reference plane (luminance or one chrominance plane) to be
/// resampled, with its width / height in samples.
#[derive(Debug, Clone, Copy)]
pub struct ResamplePlane<'a> {
    /// Row-major sample buffer.
    pub data: &'a [u8],
    /// Plane width in samples.
    pub width: usize,
    /// Plane height in samples.
    pub height: usize,
}

/// §P.3 — resample one plane of the reference picture to the current
/// picture's plane size, returning the warped plane.
///
/// `s` is the §P.3 field parameter: `2` for luminance, `1` for
/// chrominance. `(cur_w, cur_h)` is the current picture's plane size
/// and `(ref_w, ref_h)` is the reference picture's plane size. The
/// corner displacements are derived from `params.warp` per §P.3 in
/// 1/32-pixel luminance units (1/64-pixel for chrominance via the
/// shared `u^{..}` parameters and the `S` factor).
#[allow(clippy::too_many_arguments)]
pub fn resample_plane(
    plane: &ResamplePlane<'_>,
    params: &RprParams,
    s: i64,
    cur_w: usize,
    cur_h: usize,
    ref_w: usize,
    ref_h: usize,
    chroma: bool,
) -> Vec<u8> {
    // §P.3 picture sizes are the *luminance* H, V, H_R, V_R. The corner
    // u^{..} displacements are defined in luminance terms; the same
    // parameters serve chrominance via the S factor. We therefore drive
    // the geometry from the luminance dimensions.
    let h = (cur_w as i64) * if chroma { 2 } else { 1 };
    let v = (cur_h as i64) * if chroma { 2 } else { 1 };
    let hr = (ref_w as i64) * if chroma { 2 } else { 1 };
    let vr = (ref_h as i64) * if chroma { 2 } else { 1 };

    let [wx0, w0y, wxx, wyx, wxy, wyy, wxxy, wyxy] = params.warp.map(i64::from);

    // §P.3 — corner displacements in 1/32-pixel accuracy (luminance).
    let ux00 = 16 * wx0;
    let uy00 = 16 * w0y;
    let uxh0 = 16 * (wx0 + wxx + 2 * (hr - h));
    let uyh0 = 16 * (w0y + wyx);
    let ux0v = 16 * (wx0 + wxy);
    let uy0v = 16 * (w0y + wyy + 2 * (vr - v));
    let uxhv = 16 * (wx0 + wxx + wxy + wxxy + 2 * (hr - h));
    let uyhv = 16 * (w0y + wyx + wyy + wyxy + 2 * (vr - v));

    // §P.3 — virtual frame H', V' = smallest powers of two >= H, V.
    let hp = next_pow2_ge(h);
    let vp = next_pow2_ge(v);

    // §P.3 — bilinear extrapolation of the corner displacements to the
    // virtual points (0,0), (H',0), (0,V'), (H',V'). "//" =
    // round-half-away-from-zero.
    let uxlt = ux00;
    let uylt = uy00;
    let uxrt = div_round_half_away((h - hp) * ux00 + hp * uxh0, h);
    let uyrt = div_round_half_away((h - hp) * uy00 + hp * uyh0, h);
    let uxlb = div_round_half_away((v - vp) * ux00 + vp * ux0v, v);
    let uylb = div_round_half_away((v - vp) * uy00 + vp * uy0v, v);
    let uxrb = div_round_half_away(
        (v - vp) * ((h - hp) * ux00 + hp * uxh0) + vp * ((h - hp) * ux0v + hp * uxhv),
        h * v,
    );
    let uyrb = div_round_half_away(
        (v - vp) * ((h - hp) * uy00 + hp * uyh0) + vp * ((h - hp) * uy0v + hp * uyhv),
        h * v,
    );

    let p = params.wda.p();
    let out_w = cur_w as i64;
    let out_h = cur_h as i64;

    let mut out = vec![0u8; cur_w * cur_h];

    // Per §P.3 the per-line endpoints u_xL(j), u_yL(j), u_xR(j),
    // u_yR(j) interpolate the LT/LB and RT/RB virtual displacements
    // across the virtual height S*V'. The §P.4.2 raster loop then walks
    // the line accumulating the transformed position. We compute the
    // endpoints per line then run the line loop.
    let svp = s * vp;
    let d = 64 * hp / p; // = 64 H' / P
    let two_m = hp; // H' = 2^m

    for j in 0..out_h {
        // u_xL(j) etc. — §P.3 one-dimensional interpolation, "//".
        let uxl = div_round_half_away((svp - 2 * j - 1) * uxlt + (2 * j + 1) * uxlb, svp);
        let uyl = div_round_half_away((svp - 2 * j - 1) * uylt + (2 * j + 1) * uylb, svp);
        let uxr = div_round_half_away((svp - 2 * j - 1) * uxrt + (2 * j + 1) * uxrb, svp);
        let uyr = div_round_half_away((svp - 2 * j - 1) * uyrt + (2 * j + 1) * uyrb, svp);

        // §P.4.2 raster loop accumulators.
        let aix = d * p + 2 * (uxr - uxl);
        let aiy = 2 * (uyr - uyl);
        let mut ax = uxl * s * two_m + (uxr - uxl) + d / 2;
        let mut ay = j * d * p + uyl * s * two_m + (uyr - uyl) + d / 2;

        for i in 0..out_w {
            let ir_pos = div_floor(ax, d);
            let jr_pos = div_floor(ay, d);
            let ir = div_floor(ir_pos, p);
            let jr = div_floor(jr_pos, p);
            let phi_x = ir_pos - ir * p;
            let phi_y = jr_pos - jr * p;

            let value = filter(plane, params, s, hr, vr, chroma, ir, jr, phi_x, phi_y, p);
            out[(j * out_w + i) as usize] = value as u8;
            ax += aix;
            ay += aiy;
        }
    }

    out
}

/// §P.3 — the four-tap bilinear `filter`, sampling the reference plane
/// with the §P.2.3 fill mode applied for out-of-range coordinates.
#[allow(clippy::too_many_arguments)]
fn filter(
    plane: &ResamplePlane<'_>,
    params: &RprParams,
    s: i64,
    hr: i64,
    vr: i64,
    chroma: bool,
    x0: i64,
    y0: i64,
    phi_x: i64,
    phi_y: i64,
    p: i64,
) -> i64 {
    // The reference plane sampling extent in this field's coordinate
    // system is S*H_R/2 by S*V_R/2 (= ref_w by ref_h). prior_sample
    // (§P.4.2) clips or fills.
    let s_hr2 = s * hr / 2;
    let s_vr2 = s * vr / 2;

    let fill_value = if chroma {
        // The caller drives chroma planes with one fill value; we pass
        // the relevant CB/CR through `fill_cb` (Cb) — Cr planes reuse
        // chroma_fill with fill_cr supplied by the caller via params.
        params.fill.chroma_fill(params.fill_cb)
    } else {
        params.fill.luma_fill(params.fill_y)
    };

    let sample = |ip: i64, jp: i64| -> i64 {
        match params.fill {
            FillMode::Clip => {
                let ic = ip.clamp(0, s_hr2 - 1);
                let jc = jp.clamp(0, s_vr2 - 1);
                i64::from(plane.data[(jc as usize) * plane.width + (ic as usize)])
            }
            _ => {
                if ip < 0 || ip > s_hr2 - 1 || jp < 0 || jp > s_vr2 - 1 {
                    fill_value
                } else {
                    i64::from(plane.data[(jp as usize) * plane.width + (ip as usize)])
                }
            }
        }
    };

    let p2 = p * p;
    let acc = (p - phi_y) * ((p - phi_x) * sample(x0, y0) + phi_x * sample(x0 + 1, y0))
        + phi_y * ((p - phi_x) * sample(x0, y0 + 1) + phi_x * sample(x0 + 1, y0 + 1))
        + p2 / 2
        - 1
        + params.rcrpr;
    // "/" division by truncation; acc is non-negative for valid sample
    // ranges so plain integer division truncates toward zero == floor.
    let result = acc / p2;
    result.clamp(0, 255)
}

/// Resample a full YUV 4:2:0 reference (`ref_*`) to the current
/// picture's plane sizes, returning the three warped planes
/// `(y, cb, cr)`. The caller passes the resolved [`RprParams`] (implicit
/// or explicit). Chrominance uses the same warping parameters with the
/// §P.3 field factor `S = 1` and its own fill colour.
#[allow(clippy::too_many_arguments)]
pub fn resample_yuv(
    ref_y: &[u8],
    ref_cb: &[u8],
    ref_cr: &[u8],
    ref_luma_w: usize,
    ref_luma_h: usize,
    cur_luma_w: usize,
    cur_luma_h: usize,
    params: &RprParams,
) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
    let y = resample_plane(
        &ResamplePlane {
            data: ref_y,
            width: ref_luma_w,
            height: ref_luma_h,
        },
        params,
        2,
        cur_luma_w,
        cur_luma_h,
        ref_luma_w,
        ref_luma_h,
        false,
    );
    let cb = resample_plane(
        &ResamplePlane {
            data: ref_cb,
            width: ref_luma_w / 2,
            height: ref_luma_h / 2,
        },
        params,
        1,
        cur_luma_w / 2,
        cur_luma_h / 2,
        ref_luma_w / 2,
        ref_luma_h / 2,
        true,
    );
    // For Cr we substitute the Cr fill colour. A scoped clone keeps the
    // single-fill-value `filter` contract while supplying CR_FILL.
    let cr_params = RprParams {
        fill_cb: params.fill_cr,
        ..*params
    };
    let cr = resample_plane(
        &ResamplePlane {
            data: ref_cr,
            width: ref_luma_w / 2,
            height: ref_luma_h / 2,
        },
        &cr_params,
        1,
        cur_luma_w / 2,
        cur_luma_h / 2,
        ref_luma_w / 2,
        ref_luma_h / 2,
        true,
    );
    (y, cb, cr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn d3_zero_codeword() {
        // "1" => magnitude 0, no sign bit.
        let data = [0b1000_0000u8];
        let mut r = BitReader::new(&data);
        assert_eq!(read_table_d3(&mut r).unwrap(), 0);
    }

    #[test]
    fn d3_value_one_positive() {
        // Magnitude 1, positive: Table D.3 row "1" => code "0 s 0" with
        // s = 0. Bits: 0 (open) 0 (sign) 0 (terminate) = "000".
        let data = [0b0000_0000u8];
        let mut r = BitReader::new(&data);
        assert_eq!(read_table_d3(&mut r).unwrap(), 1);
    }

    #[test]
    fn d3_value_one_negative() {
        // Magnitude 1, negative: "0 s 0" with s = 1 => "010".
        let data = [0b0100_0000u8];
        let mut r = BitReader::new(&data);
        assert_eq!(read_table_d3(&mut r).unwrap(), -1);
    }

    #[test]
    fn d3_worked_example_minus_13() {
        // §D.3 worked example: –13 encodes as "0 11 01 11 10" =
        // bits 011011110 (9 bits).
        let data = [0b0110_1111u8, 0b0000_0000u8];
        let mut r = BitReader::new(&data);
        assert_eq!(read_table_d3(&mut r).unwrap(), -13);
    }

    #[test]
    fn div_helpers_match_spec() {
        // // rounds half away from zero.
        assert_eq!(div_round_half_away(5, 2), 3);
        assert_eq!(div_round_half_away(-5, 2), -3);
        assert_eq!(div_round_half_away(4, 2), 2);
        // /// truncates toward negative infinity.
        assert_eq!(div_floor(-1, 16), -1);
        assert_eq!(div_floor(-16, 16), -1);
        assert_eq!(div_floor(15, 16), 0);
    }

    #[test]
    fn next_pow2() {
        assert_eq!(next_pow2_ge(176), 256);
        assert_eq!(next_pow2_ge(256), 256);
        assert_eq!(next_pow2_ge(144), 256);
        assert_eq!(next_pow2_ge(1), 1);
    }

    #[test]
    fn identity_resample_is_near_lossless() {
        // Zero warping, same size: the warp should map each pixel close
        // to itself (a pure phase-aligned identity). With WDA=1/16 and
        // implicit params, a flat plane must reproduce exactly.
        let w = 16usize;
        let h = 16usize;
        let plane: Vec<u8> = vec![100u8; w * h];
        let params = RprParams::implicit(false);
        let out = resample_plane(
            &ResamplePlane {
                data: &plane,
                width: w,
                height: h,
            },
            &params,
            2,
            w,
            h,
            w,
            h,
            false,
        );
        // A constant plane warps to the same constant everywhere.
        assert!(out.iter().all(|&p| p == 100));
    }

    #[test]
    fn upsample_doubles_dimensions() {
        // A 8x8 reference upsampled to 16x16: a constant plane stays
        // constant (the bilinear filter of a flat field is flat).
        let rw = 8usize;
        let rh = 8usize;
        let plane: Vec<u8> = vec![80u8; rw * rh];
        let params = RprParams::implicit(false);
        let out = resample_plane(
            &ResamplePlane {
                data: &plane,
                width: rw,
                height: rh,
            },
            &params,
            2,
            16,
            16,
            rw,
            rh,
            false,
        );
        assert_eq!(out.len(), 16 * 16);
        assert!(out.iter().all(|&p| p == 80));
    }

    #[test]
    fn fill_mode_bits() {
        assert_eq!(FillMode::from_bits(0b00), FillMode::Color);
        assert_eq!(FillMode::from_bits(0b01), FillMode::Black);
        assert_eq!(FillMode::from_bits(0b10), FillMode::Gray);
        assert_eq!(FillMode::from_bits(0b11), FillMode::Clip);
    }
}
