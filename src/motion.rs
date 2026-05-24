//! H.263 P-frame motion compensation (§6.1) and INTER-block
//! reconstruction (§6.3.1 summation + §6.3.2 clip).
//!
//! Implements the default (non-Annex-D, non-Annex-F) prediction mode
//! of ITU-T Recommendation H.263 (01/2005) §6.1:
//!
//! * **§6.1.1 differential motion vectors** — the macroblock vector
//!   is obtained by adding the median predictor (taken from the three
//!   surrounding macroblocks per Figure 12) to the [`Mvd`] difference
//!   decoded from Table 14. Each VLC word for MVD represents a *pair*
//!   of difference values; only one of the pair yields a component in
//!   the permitted range `[-16, 15.5]` (= `[-32, 31]` in half-pel
//!   units, a span of 64). The reconstruction therefore wraps the
//!   `predictor + difference` sum into that 64-wide window.
//! * **Table 18 chrominance vector derivation** — the chroma vector
//!   is the macroblock luma vector divided by two, with the resulting
//!   quarter-pel position rounded towards the nearest half-pel
//!   position per Table 18.
//! * **§6.1.2 half-pixel interpolation** — half-pel sample values are
//!   found by bilinear interpolation per Figure 13 with `RCONTROL`
//!   (the rounding-type control bit, implied `0` in baseline H.263).
//! * **§6.3.1 summation** — INTER reconstruction = motion-compensated
//!   prediction + inverse-transformed residual, pixel by pixel.
//! * **§6.3.2 clipping** — the summed result is clipped to `[0, 255]`.
//!
//! The reference-picture access here uses the §D.1 edge-replication
//! rule: a pixel referenced outside the coded picture area is replaced
//! by the nearest edge pixel ("limiting the motion vector to the last
//! full-pixel position inside the coded picture area"). This is the
//! always-on boundary behaviour, and it is exactly the §D.1 rule the
//! Unrestricted Motion Vector mode relies on when a vector points
//! outside the picture.
//!
//! Annex D §D.2 — the **Unrestricted Motion Vector mode** with
//! PLUSPTYPE *absent* from the picture header — is wired through
//! [`reconstruct_mv_component_umv`] / [`reconstruct_mv_umv`]: the
//! per-component range is widened from the default `[-32, 31]` to
//! `[-63, 63]` half-pel, with the §D.2 predictor-dependent selection of
//! the Table-14 difference pair. The wider PLUSPTYPE / UUI ranges of
//! Tables D.1 / D.2 and the Table-D.3 reversible VLC remain gated on
//! the not-yet-decoded extended-PTYPE header.

use crate::block::COEFFS_PER_BLOCK;
use crate::idct::BLOCK_DIM;
use crate::macroblock::Mvd;

/// §6.1.1 permitted motion-vector component range, lower bound, in
/// **half-pel units** (spec "Vector" value `-16`).
pub const MV_HALF_MIN: i32 = -32;
/// §6.1.1 permitted motion-vector component range, upper bound, in
/// **half-pel units** (spec "Vector" value `+15.5`).
pub const MV_HALF_MAX: i32 = 31;
/// Width of the permitted range (number of distinct half-pel
/// components), used as the wrap modulus in [`reconstruct_mv_component`].
pub const MV_HALF_SPAN: i32 = MV_HALF_MAX - MV_HALF_MIN + 1; // 64

/// A reconstructed motion vector, in **half-pel units** (a value of
/// `1` denotes half a pixel of displacement). Positive `dx`/`dy`
/// indicate prediction from pixels to the right of / below the pixels
/// being predicted (§6.1.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MotionVector {
    /// Horizontal component, half-pel units.
    pub dx_half: i32,
    /// Vertical component, half-pel units.
    pub dy_half: i32,
}

impl MotionVector {
    /// Construct from explicit half-pel components.
    pub const fn new(dx_half: i32, dy_half: i32) -> Self {
        Self { dx_half, dy_half }
    }
}

/// §6.1.1 reconstruction of one motion-vector component.
///
/// The macroblock vector component is `predictor + difference`, with
/// the caveat that "each VLC word for MVD represents a pair of
/// difference values; only one of the pair will yield a macroblock
/// vector component falling within the permitted range `[-16, 15.5]`".
/// In half-pel units that range is `[-32, 31]` — a window of
/// [`MV_HALF_SPAN`] (= 64) distinct values. The reconstruction wraps
/// the raw sum into that window:
///
/// ```text
///   v = predictor + difference
///   while v < -32 { v += 64 }
///   while v >  31 { v -= 64 }
/// ```
///
/// Both `predictor` and `difference` are in half-pel units; the
/// returned component is in `[-32, 31]`.
pub fn reconstruct_mv_component(predictor: i32, difference: i32) -> i32 {
    let mut v = predictor + difference;
    // The raw sum can be at most |predictor| + |difference| ≤
    // 31 + 32 = 63 from the window edge, so a single wrap on each
    // side always lands inside, but the loop form is robust and
    // matches the spec's "the value in range" selection exactly.
    while v < MV_HALF_MIN {
        v += MV_HALF_SPAN;
    }
    while v > MV_HALF_MAX {
        v -= MV_HALF_SPAN;
    }
    v
}

/// §6.1.1 reconstruction of a full motion vector from its predictor
/// and the decoded [`Mvd`] (Table 14, half-pel units).
pub fn reconstruct_mv(predictor: MotionVector, mvd: Mvd) -> MotionVector {
    MotionVector {
        dx_half: reconstruct_mv_component(predictor.dx_half, mvd.dx_half as i32),
        dy_half: reconstruct_mv_component(predictor.dy_half, mvd.dy_half as i32),
    }
}

// ---- Annex D §D.2 Unrestricted Motion Vector mode (non-PLUSPTYPE) ---

/// Annex D §D.2 extended motion-vector component range, lower bound, in
/// **half-pel units** (spec "Vector" value `-31.5`). Applies in the
/// Unrestricted Motion Vector mode when PLUSPTYPE is *absent* from the
/// picture header.
pub const MV_UMV_HALF_MIN: i32 = -63;
/// Annex D §D.2 extended motion-vector component range, upper bound, in
/// **half-pel units** (spec "Vector" value `+31.5`).
pub const MV_UMV_HALF_MAX: i32 = 63;

/// Annex D §D.2 reconstruction of one motion-vector component in the
/// **Unrestricted Motion Vector mode** with PLUSPTYPE *absent* from the
/// picture header.
///
/// In this mode the per-component range is extended from the default
/// `[-32, 31]` (spec `[-16, 15.5]`) to `[-63, 63]` half-pel
/// (spec `[-31.5, 31.5]`). `difference` is the Table-14 "Vector"
/// column value already decoded into `[-32, 31]` half-pel; that value
/// stands for a *pair* of differences `{difference, difference ± 64}`
/// (the two members differ by [`MV_HALF_SPAN`]). §D.2 selects which
/// member to add to the predictor:
///
/// * If the predictor `Pc` lies in `[-31, 32]` half-pel
///   (spec `[-15.5, 16]`), "only the first column of vector
///   differences applies" — the component is `Pc + difference`, with
///   no wrap. The result is guaranteed to land in `[Pc-32, Pc+31]`,
///   which is inside `[-63, 63]`.
/// * Otherwise (predictor outside `[-31, 32]`), the member of the pair
///   is chosen that yields a component inside `[-63, 63]` **with the
///   same sign as the predictor**, where zero counts as either sign.
///   Concretely (§D.2):
///   - `-63 ≤ Pc ≤ -32` ⇒ result in `[-63, 0]`,
///   - `33 ≤ Pc ≤ 63` ⇒ result in `[0, 63]`.
///
/// Both `predictor` and `difference` are half-pel; the returned
/// component is in `[-63, 63]`.
pub fn reconstruct_mv_component_umv(predictor: i32, difference: i32) -> i32 {
    // §D.2: predictor inside [-31, 32] -> the first column applies
    // directly (no pair selection, no wrap).
    if (-31..=32).contains(&predictor) {
        return predictor + difference;
    }

    // Predictor outside [-31, 32]: the Table-14 codeword denotes the
    // pair {difference, difference ± MV_HALF_SPAN}; pick the member
    // whose `predictor + member` lands inside [-63, 63] with the same
    // sign as the predictor (zero allowed for either sign).
    let alt = if difference >= 0 {
        difference - MV_HALF_SPAN
    } else {
        difference + MV_HALF_SPAN
    };
    let candidates = [predictor + difference, predictor + alt];
    let want_nonneg = predictor > 0;
    for &mvc in &candidates {
        if !(MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX).contains(&mvc) {
            continue;
        }
        // Same sign as the predictor, with zero permitted either way.
        let ok = if want_nonneg { mvc >= 0 } else { mvc <= 0 };
        if ok {
            return mvc;
        }
    }
    // Per §D.2 exactly one candidate satisfies the constraints; this
    // fallback (clamp into range) keeps the function total against
    // malformed predictor/difference combinations.
    (predictor + difference).clamp(MV_UMV_HALF_MIN, MV_UMV_HALF_MAX)
}

/// Annex D §D.2 reconstruction of a full motion vector in the
/// Unrestricted Motion Vector mode (PLUSPTYPE absent), applying
/// [`reconstruct_mv_component_umv`] per component.
pub fn reconstruct_mv_umv(predictor: MotionVector, mvd: Mvd) -> MotionVector {
    MotionVector {
        dx_half: reconstruct_mv_component_umv(predictor.dx_half, mvd.dx_half as i32),
        dy_half: reconstruct_mv_component_umv(predictor.dy_half, mvd.dy_half as i32),
    }
}

/// §6.1.1 median predictor for one component.
///
/// "For each component, the predictor is the median value of the three
/// candidate predictors for this component." (Figure 12 supplies the
/// three candidates MV1 / MV2 / MV3; the border decision rules that
/// select their values are the caller's responsibility.)
pub fn median3(a: i32, b: i32, c: i32) -> i32 {
    // median = a + b + c - max - min
    let max = a.max(b).max(c);
    let min = a.min(b).min(c);
    a + b + c - max - min
}

/// §6.1.1 median predictor for a full motion vector (per-component
/// median of the three candidate predictors).
pub fn predict_mv_median(mv1: MotionVector, mv2: MotionVector, mv3: MotionVector) -> MotionVector {
    MotionVector {
        dx_half: median3(mv1.dx_half, mv2.dx_half, mv3.dx_half),
        dy_half: median3(mv1.dy_half, mv2.dy_half, mv3.dy_half),
    }
}

/// Table 18 modification of one quarter-pixel chrominance vector
/// component towards the nearest half-pixel position.
///
/// The chrominance vector is the luminance macroblock vector divided
/// by two (§6.1.1): a luma half-pel component `l` corresponds to a
/// chroma quarter-pel value `l / 2` (quarter-pel units). Table 18
/// maps quarter-pixel positions to the nearest half-pixel position:
///
/// ```text
///   quarter-pel fractional part   0   1/4  1/2  3/4   1
///   resulting   fractional part   0   1/2  1/2  1/2   1
/// ```
///
/// In half-pel units, a luma component `l` (half-pel) gives a chroma
/// component whose magnitude is `round_to_table18(l)`. Concretely, the
/// integer part of `l/2` (in pixels) is preserved and any non-zero
/// fractional remainder collapses to a single half-pel step.
///
/// We compute this in half-pel chroma units (so the returned value is
/// directly usable as a half-pel displacement on the chroma plane).
/// `l` is the luma component in half-pel units; the result is the
/// chroma component in half-pel units.
pub fn chroma_mv_component(l: i32) -> i32 {
    // l is in luma half-pel units. l/2 (rounded toward zero in
    // quarter-pel terms) gives the chroma displacement in quarter-pel
    // units; Table 18 then snaps the quarter-pel fraction to the
    // nearest half. Work with sign separated so truncation behaves
    // symmetrically about zero (Table 18 is symmetric for ±).
    let sign = if l < 0 { -1 } else { 1 };
    let mag = l.unsigned_abs() as i32; // luma half-pel magnitude
                                       // quarter-pel magnitude = mag (because 1 luma half-pel == 1 chroma
                                       // quarter-pel after the /2). Express in chroma half-pel units:
                                       //   full pixels  = mag / 4   (each pixel is 4 quarter-pels)
                                       //   remainder    = mag % 4   (0..3 quarter-pels)
                                       // Table 18: remainder 0 -> +0 half-pel, remainder 1/2/3 -> +1 half-pel.
    let full_pixels = mag / 4;
    let rem = mag % 4;
    let frac_half = if rem == 0 { 0 } else { 1 };
    sign * (full_pixels * 2 + frac_half)
}

/// Table 18 chroma vector for a full luma motion vector.
pub fn chroma_mv(luma: MotionVector) -> MotionVector {
    MotionVector {
        dx_half: chroma_mv_component(luma.dx_half),
        dy_half: chroma_mv_component(luma.dy_half),
    }
}

/// A non-owning view of one reference-picture plane (luma or one
/// chroma channel) in row-major `u8` layout. Used as the source for
/// motion-compensated prediction.
#[derive(Debug, Clone, Copy)]
pub struct RefPlane<'a> {
    /// Row-major samples, `width * height` long.
    pub samples: &'a [u8],
    /// Plane width in pixels.
    pub width: usize,
    /// Plane height in pixels.
    pub height: usize,
}

impl<'a> RefPlane<'a> {
    /// Construct a plane view, asserting the sample buffer matches the
    /// declared dimensions.
    pub fn new(samples: &'a [u8], width: usize, height: usize) -> Self {
        debug_assert_eq!(samples.len(), width * height);
        Self {
            samples,
            width,
            height,
        }
    }

    /// Fetch a sample with §D.1 edge replication: coordinates outside
    /// the coded picture area are clamped to the nearest edge pixel.
    #[inline]
    fn at(&self, x: i32, y: i32) -> i32 {
        let cx = x.clamp(0, self.width as i32 - 1) as usize;
        let cy = y.clamp(0, self.height as i32 - 1) as usize;
        self.samples[cy * self.width + cx] as i32
    }
}

/// `RCONTROL` for §6.1.2 bilinear interpolation. In baseline H.263 it
/// is implied `0`; the rounding-type bit (Annex M / extended-PTYPE)
/// can set it to `1`. We accept it as a parameter so the caller can
/// pass the right value when extended PTYPE is decoded, but baseline
/// callers pass `0`.
pub const RCONTROL_DEFAULT: i32 = 0;

/// §6.1.2 / Figure 13 — fetch one motion-compensated sample at the
/// given half-pel position `(hx, hy)` (in half-pel units) from a
/// reference plane.
///
/// The integer pixel position is `(hx >> 1, hy >> 1)`; the low bit of
/// each component selects the sub-pixel phase. Per Figure 13 with
/// integer neighbours A/B/C/D (A = top-left integer pixel, B = A's
/// right neighbour, C = A's lower neighbour, D = the diagonal):
///
/// ```text
///   a = A                                  (no fractional offset)
///   b = (A + B + 1 - RCONTROL) / 2         (horizontal half-pel)
///   c = (A + C + 1 - RCONTROL) / 2         (vertical half-pel)
///   d = (A + B + C + D + 2 - RCONTROL) / 4 (diagonal half-pel)
/// ```
///
/// where `/` is truncating integer division. Edge replication (§D.1)
/// is applied when a neighbour falls outside the plane.
#[inline]
fn sample_half_pel(plane: &RefPlane<'_>, hx: i32, hy: i32, rcontrol: i32) -> i32 {
    let ix = hx >> 1; // floor toward -inf for negatives is fine: see note
    let iy = hy >> 1;
    let fx = hx & 1;
    let fy = hy & 1;
    let a = plane.at(ix, iy);
    match (fx, fy) {
        (0, 0) => a,
        (1, 0) => {
            let b = plane.at(ix + 1, iy);
            (a + b + 1 - rcontrol) / 2
        }
        (0, 1) => {
            let c = plane.at(ix, iy + 1);
            (a + c + 1 - rcontrol) / 2
        }
        _ => {
            let b = plane.at(ix + 1, iy);
            let c = plane.at(ix, iy + 1);
            let d = plane.at(ix + 1, iy + 1);
            (a + b + c + d + 2 - rcontrol) / 4
        }
    }
}

/// §6.1 + §6.1.2 — build the 8×8 motion-compensated prediction block
/// for a block whose top-left corner is at integer pixel position
/// `(block_x, block_y)` in the **current** picture, using the motion
/// vector `mv` (half-pel units) against the reference plane.
///
/// For each output pixel `(px, py)` in the 8×8 block, the half-pel
/// source position is
/// `((block_x + px) * 2 + mv.dx_half, (block_y + py) * 2 + mv.dy_half)`.
/// The sample is fetched via §6.1.2 bilinear interpolation with edge
/// replication.
pub fn motion_compensate_block(
    plane: &RefPlane<'_>,
    block_x: usize,
    block_y: usize,
    mv: MotionVector,
    rcontrol: i32,
) -> [u8; COEFFS_PER_BLOCK] {
    let mut out = [0u8; COEFFS_PER_BLOCK];
    for py in 0..BLOCK_DIM {
        for px in 0..BLOCK_DIM {
            let hx = ((block_x + px) as i32) * 2 + mv.dx_half;
            let hy = ((block_y + py) as i32) * 2 + mv.dy_half;
            let s = sample_half_pel(plane, hx, hy, rcontrol);
            // Reference samples are 8-bit, and bilinear averaging keeps
            // the result inside [0, 255]; clamp defensively.
            out[py * BLOCK_DIM + px] = s.clamp(0, 255) as u8;
        }
    }
    out
}

/// §6.3.1 summation + §6.3.2 clip for an INTER block.
///
/// Takes the motion-compensated 8×8 `prediction` (from
/// [`motion_compensate_block`]) and the 8×8 inverse-transformed
/// residual `residual` (the §6.2.4 IDCT output, signed, in
/// `[-256, 255]`), and forms the reconstructed sample block:
///
/// ```text
///   rec(x, y) = clip( prediction(x, y) + residual(x, y) )   §6.3.1 + §6.3.2
/// ```
///
/// The clip pins values below 0 to 0 and above 255 to 255 (§6.3.2).
pub fn reconstruct_inter_block(
    prediction: &[u8; COEFFS_PER_BLOCK],
    residual: &[i16; COEFFS_PER_BLOCK],
) -> [u8; COEFFS_PER_BLOCK] {
    let mut out = [0u8; COEFFS_PER_BLOCK];
    for ((dst, &p), &r) in out.iter_mut().zip(prediction.iter()).zip(residual.iter()) {
        let sum = p as i32 + r as i32;
        *dst = sum.clamp(0, 255) as u8;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- §6.1.1 MV component reconstruction ----------------------

    /// Zero predictor + zero difference => zero vector.
    #[test]
    fn mv_component_zero() {
        assert_eq!(reconstruct_mv_component(0, 0), 0);
    }

    /// In-range sum stays as-is.
    #[test]
    fn mv_component_in_range_no_wrap() {
        assert_eq!(reconstruct_mv_component(4, 6), 10);
        assert_eq!(reconstruct_mv_component(-10, 5), -5);
        assert_eq!(reconstruct_mv_component(MV_HALF_MAX, 0), MV_HALF_MAX);
        assert_eq!(reconstruct_mv_component(MV_HALF_MIN, 0), MV_HALF_MIN);
    }

    /// Sum above +31 wraps down by 64 (the "other element of the
    /// pair" selection): predictor 31 + difference 5 = 36 -> 36 - 64
    /// = -28, which is the in-range value.
    #[test]
    fn mv_component_wraps_high() {
        assert_eq!(reconstruct_mv_component(31, 5), 36 - 64);
        // Boundary: 31 + 1 = 32 -> -32.
        assert_eq!(reconstruct_mv_component(31, 1), -32);
    }

    /// Sum below -32 wraps up by 64.
    #[test]
    fn mv_component_wraps_low() {
        assert_eq!(reconstruct_mv_component(-32, -5), -37 + 64);
        // Boundary: -32 + (-1) = -33 -> 31.
        assert_eq!(reconstruct_mv_component(-32, -1), 31);
    }

    /// Every (predictor, difference) pair lands inside [-32, 31].
    #[test]
    fn mv_component_always_in_range() {
        for p in MV_HALF_MIN..=MV_HALF_MAX {
            for d in MV_HALF_MIN..=MV_HALF_MAX {
                let v = reconstruct_mv_component(p, d);
                assert!(
                    (MV_HALF_MIN..=MV_HALF_MAX).contains(&v),
                    "p={p} d={d} -> {v} out of range"
                );
            }
        }
    }

    /// `reconstruct_mv` applies the per-component rule.
    #[test]
    fn reconstruct_full_mv() {
        let pred = MotionVector::new(31, -32);
        let mvd = Mvd {
            dx_half: 1,
            dy_half: -1,
        };
        let mv = reconstruct_mv(pred, mvd);
        assert_eq!(mv, MotionVector::new(-32, 31));
    }

    // ---- Annex D §D.2 UMV component reconstruction ---------------

    /// §D.2: predictor inside [-31, 32] uses the first column directly
    /// (no wrap), so the component can leave the default [-32, 31]
    /// window and reach the extended [-63, 63] range.
    #[test]
    fn umv_predictor_in_range_no_wrap() {
        // Default mode would wrap 32+31=63 down to -1; UMV keeps it.
        assert_eq!(reconstruct_mv_component_umv(32, 31), 63);
        // Symmetric low end: -31 + (-32) = -63 (in range, no wrap).
        assert_eq!(reconstruct_mv_component_umv(-31, -32), -63);
        // Ordinary interior sum is unchanged.
        assert_eq!(reconstruct_mv_component_umv(4, 6), 10);
        assert_eq!(reconstruct_mv_component_umv(0, 0), 0);
        // Boundary predictors of the "first column" range.
        assert_eq!(reconstruct_mv_component_umv(32, 0), 32);
        assert_eq!(reconstruct_mv_component_umv(-31, 0), -31);
    }

    /// §D.2: predictor below -31 forces a non-positive component with
    /// the same (negative) sign as the predictor. The pair member that
    /// satisfies `-63 ≤ MVc ≤ 0` is selected.
    #[test]
    fn umv_predictor_below_range_selects_nonpositive() {
        // Pc = -40. difference = 31 (Vector column). The pair is
        // {31, 31-64=-33}. Pc+31 = -9 (in [-63,0]) — selected.
        assert_eq!(reconstruct_mv_component_umv(-40, 31), -9);
        // difference = -32: pair {-32, -32+64=32}. Pc-32 = -72 (out),
        // Pc+32 = -8 (in [-63,0]) — selected.
        assert_eq!(reconstruct_mv_component_umv(-40, -32), -8);
        // Result always non-positive and in range for every difference.
        for d in -32..=31 {
            let v = reconstruct_mv_component_umv(-50, d);
            assert!(
                (MV_UMV_HALF_MIN..=0).contains(&v),
                "Pc=-50 d={d} -> {v} not in [-63, 0]"
            );
        }
    }

    /// §D.2: predictor above 32 forces a non-negative component with
    /// the same (positive) sign as the predictor.
    #[test]
    fn umv_predictor_above_range_selects_nonnegative() {
        // Pc = 40. difference = -32: pair {-32, 32}. Pc-32 = 8 (in
        // [0,63]) — selected; Pc+(-32)=8 actually, let the impl decide.
        let v = reconstruct_mv_component_umv(40, -32);
        assert!((0..=MV_UMV_HALF_MAX).contains(&v), "{v} not in [0,63]");
        // Result always non-negative and in range for every difference.
        for d in -32..=31 {
            let v = reconstruct_mv_component_umv(50, d);
            assert!(
                (0..=MV_UMV_HALF_MAX).contains(&v),
                "Pc=50 d={d} -> {v} not in [0, 63]"
            );
        }
    }

    /// Every (predictor, difference) pair in the UMV space yields a
    /// component inside the extended [-63, 63] window.
    #[test]
    fn umv_component_always_in_extended_range() {
        for p in MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX {
            for d in MV_HALF_MIN..=MV_HALF_MAX {
                let v = reconstruct_mv_component_umv(p, d);
                assert!(
                    (MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX).contains(&v),
                    "p={p} d={d} -> {v} out of extended range"
                );
            }
        }
    }

    /// `reconstruct_mv_umv` applies the §D.2 rule per component.
    #[test]
    fn umv_full_vector() {
        // dx: predictor 32 (in range) + 31 -> 63 (no wrap).
        // dy: predictor -50 (below range) + 0 -> 0.
        let pred = MotionVector::new(32, -50);
        let mvd = Mvd {
            dx_half: 31,
            dy_half: 0,
        };
        let mv = reconstruct_mv_umv(pred, mvd);
        assert_eq!(mv.dx_half, 63);
        assert!((MV_UMV_HALF_MIN..=0).contains(&mv.dy_half));
    }

    /// In the predictor "first column" range, UMV and the default
    /// reconstruction agree whenever the default sum does not wrap.
    #[test]
    fn umv_matches_default_when_no_wrap() {
        for p in -31..=32 {
            for d in MV_HALF_MIN..=MV_HALF_MAX {
                let sum = p + d;
                if (MV_HALF_MIN..=MV_HALF_MAX).contains(&sum) {
                    assert_eq!(
                        reconstruct_mv_component_umv(p, d),
                        reconstruct_mv_component(p, d),
                        "p={p} d={d}"
                    );
                }
            }
        }
    }

    // ---- §6.1.1 median predictor ---------------------------------

    #[test]
    fn median3_basic() {
        assert_eq!(median3(1, 2, 3), 2);
        assert_eq!(median3(3, 1, 2), 2);
        assert_eq!(median3(-5, 0, 5), 0);
        assert_eq!(median3(7, 7, 1), 7);
        assert_eq!(median3(4, 4, 4), 4);
    }

    #[test]
    fn predict_median_per_component() {
        let m = predict_mv_median(
            MotionVector::new(1, -3),
            MotionVector::new(5, -1),
            MotionVector::new(3, -2),
        );
        assert_eq!(m, MotionVector::new(3, -2));
    }

    // ---- Table 18 chroma vector derivation -----------------------

    /// Quarter-pel position 0 (luma even half-pel that is a whole even
    /// pixel) -> chroma whole pixel, no fractional part.
    #[test]
    fn chroma_zero_fraction() {
        // luma 0 half-pel -> chroma 0
        assert_eq!(chroma_mv_component(0), 0);
        // luma 4 half-pel = 2 full luma pixels -> chroma 1 full pixel
        // = 2 chroma half-pel.
        assert_eq!(chroma_mv_component(4), 2);
        assert_eq!(chroma_mv_component(-4), -2);
        // luma 8 half-pel = 4 luma pixels -> chroma 2 pixels = 4 half-pel.
        assert_eq!(chroma_mv_component(8), 4);
    }

    /// Table 18: quarter-pel 1/4, 1/2, 3/4 all collapse to 1/2
    /// (one chroma half-pel step) plus the integer part.
    #[test]
    fn chroma_fraction_snaps_to_half() {
        // luma 1 half-pel: mag=1 -> full=0, rem=1 -> 0*2 + 1 = 1.
        assert_eq!(chroma_mv_component(1), 1);
        // luma 2 half-pel: mag=2 -> full=0, rem=2 -> 1.
        assert_eq!(chroma_mv_component(2), 1);
        // luma 3 half-pel: mag=3 -> full=0, rem=3 -> 1.
        assert_eq!(chroma_mv_component(3), 1);
        // luma 5 half-pel: mag=5 -> full=1, rem=1 -> 2+1 = 3.
        assert_eq!(chroma_mv_component(5), 3);
        // luma 6 half-pel: mag=6 -> full=1, rem=2 -> 3.
        assert_eq!(chroma_mv_component(6), 3);
        // luma 7 half-pel: mag=7 -> full=1, rem=3 -> 3.
        assert_eq!(chroma_mv_component(7), 3);
        // negatives mirror.
        assert_eq!(chroma_mv_component(-1), -1);
        assert_eq!(chroma_mv_component(-7), -3);
    }

    #[test]
    fn chroma_full_vector() {
        let c = chroma_mv(MotionVector::new(4, -3));
        assert_eq!(c, MotionVector::new(2, -1));
    }

    // ---- §6.1.2 half-pel interpolation ---------------------------

    fn plane_4x4() -> Vec<u8> {
        // A simple ascending plane:
        //   0  1  2  3
        //  10 11 12 13
        //  20 21 22 23
        //  30 31 32 33
        vec![
            0, 1, 2, 3, //
            10, 11, 12, 13, //
            20, 21, 22, 23, //
            30, 31, 32, 33,
        ]
    }

    /// Integer (full-pel) position returns the pixel itself.
    #[test]
    fn half_pel_integer_position() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Integer pixel (1, 2) -> half position (2, 4).
        assert_eq!(sample_half_pel(&p, 2, 4, 0), 21);
        assert_eq!(sample_half_pel(&p, 0, 0, 0), 0);
    }

    /// Horizontal half-pel `b = (A + B + 1) / 2` with RCONTROL=0.
    #[test]
    fn half_pel_horizontal() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Between pixel (0,0)=0 and (1,0)=1: half x at hx=1,hy=0.
        // b = (0 + 1 + 1 - 0) / 2 = 1.
        assert_eq!(sample_half_pel(&p, 1, 0, 0), 1);
        // Between (1,1)=11 and (2,1)=12: hx=3,hy=2.
        // b = (11 + 12 + 1) / 2 = 12.
        assert_eq!(sample_half_pel(&p, 3, 2, 0), 12);
    }

    /// RCONTROL=1 biases rounding downward: (0+1+1-1)/2 = 0.
    #[test]
    fn half_pel_horizontal_rcontrol1() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        assert_eq!(sample_half_pel(&p, 1, 0, 1), 0);
    }

    /// Vertical half-pel `c = (A + C + 1) / 2`.
    #[test]
    fn half_pel_vertical() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Between (0,0)=0 and (0,1)=10: hx=0,hy=1.
        // c = (0 + 10 + 1) / 2 = 5.
        assert_eq!(sample_half_pel(&p, 0, 1, 0), 5);
    }

    /// Diagonal half-pel `d = (A + B + C + D + 2) / 4`.
    #[test]
    fn half_pel_diagonal() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Around (0,0): A=0, B=1, C=10, D=11. hx=1,hy=1.
        // d = (0 + 1 + 10 + 11 + 2) / 4 = 24 / 4 = 6.
        assert_eq!(sample_half_pel(&p, 1, 1, 0), 6);
    }

    /// Edge replication: sampling outside the plane clamps to the
    /// nearest edge pixel (§D.1).
    #[test]
    fn half_pel_edge_replication() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Negative integer position clamps to (0,0)=0.
        assert_eq!(sample_half_pel(&p, -10, -10, 0), 0);
        // Far past the right/bottom clamps to (3,3)=33.
        assert_eq!(sample_half_pel(&p, 100, 100, 0), 33);
        // Half position straddling the right edge: integer x=3 (last
        // col), B clamps to same column -> b = (3 + 3 + 1)/2 = 3.
        assert_eq!(sample_half_pel(&p, 7, 0, 0), 3);
    }

    // ---- §6.1 block-level motion compensation --------------------

    /// Zero vector against a flat reference returns that flat block.
    #[test]
    fn mc_block_zero_vector_flat() {
        let buf = vec![128u8; 16 * 16];
        let p = RefPlane::new(&buf, 16, 16);
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(0, 0), 0);
        assert!(pred.iter().all(|&v| v == 128));
    }

    /// Integer-pel vector shifts the source window; a full-pel right
    /// shift of 2 (dx_half = 4) at block (0,0) reads pixels starting
    /// at column 2.
    #[test]
    fn mc_block_integer_shift() {
        // 16x8 ramp where pixel value == column.
        let mut buf = vec![0u8; 16 * 8];
        for y in 0..8 {
            for x in 0..16 {
                buf[y * 16 + x] = x as u8;
            }
        }
        let p = RefPlane::new(&buf, 16, 8);
        // dx_half = 4 -> +2 px. Block (0,0) pixel (0,0) reads col 2.
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(4, 0), 0);
        // Row 0 of the prediction should read columns 2..10.
        for (x, &px) in pred.iter().take(BLOCK_DIM).enumerate() {
            assert_eq!(px, (x + 2) as u8, "col {x}");
        }
    }

    /// Half-pel right shift averages adjacent columns.
    #[test]
    fn mc_block_half_pel_shift() {
        // Reference: even columns = 0, odd columns = 100.
        let mut buf = vec![0u8; 16 * 8];
        for y in 0..8 {
            for x in 0..16 {
                buf[y * 16 + x] = if x % 2 == 0 { 0 } else { 100 };
            }
        }
        let p = RefPlane::new(&buf, 16, 8);
        // dx_half = 1 -> +0.5 px. Pixel (0,0): A=col0=0, B=col1=100.
        // b = (0 + 100 + 1) / 2 = 50.
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(1, 0), 0);
        assert_eq!(pred[0], 50);
        // Pixel (1,0): A=col1=100, B=col2=0 -> (100+0+1)/2 = 50.
        assert_eq!(pred[1], 50);
    }

    // ---- §6.3.1 + §6.3.2 INTER summation -------------------------

    /// Prediction + zero residual returns the prediction.
    #[test]
    fn inter_recon_zero_residual() {
        let pred = [100u8; COEFFS_PER_BLOCK];
        let res = [0i16; COEFFS_PER_BLOCK];
        let rec = reconstruct_inter_block(&pred, &res);
        assert!(rec.iter().all(|&v| v == 100));
    }

    /// Residual adds to prediction pixel-by-pixel.
    #[test]
    fn inter_recon_adds_residual() {
        let mut pred = [50u8; COEFFS_PER_BLOCK];
        let mut res = [0i16; COEFFS_PER_BLOCK];
        pred[0] = 50;
        res[0] = 30;
        res[1] = -20;
        let rec = reconstruct_inter_block(&pred, &res);
        assert_eq!(rec[0], 80);
        assert_eq!(rec[1], 30);
    }

    /// §6.3.2 clipping at both extremes.
    #[test]
    fn inter_recon_clips_to_picture_range() {
        let mut pred = [0u8; COEFFS_PER_BLOCK];
        let mut res = [0i16; COEFFS_PER_BLOCK];
        // pixel 0: prediction 250 + residual 50 = 300 -> 255.
        pred[0] = 250;
        res[0] = 50;
        // pixel 1: prediction 10 + residual -200 = -190 -> 0.
        pred[1] = 10;
        res[1] = -200;
        let rec = reconstruct_inter_block(&pred, &res);
        assert_eq!(rec[0], 255);
        assert_eq!(rec[1], 0);
    }

    /// End-to-end: a flat reference + a half-pel vector + a small DC
    /// residual reconstructs a uniformly shifted, biased block.
    #[test]
    fn inter_end_to_end_flat_plus_dc() {
        let buf = vec![100u8; 16 * 16];
        let p = RefPlane::new(&buf, 16, 16);
        // Half-pel vector on a flat plane is still 100 everywhere.
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(1, 1), 0);
        assert!(pred.iter().all(|&v| v == 100));
        // Add a uniform +20 residual.
        let res = [20i16; COEFFS_PER_BLOCK];
        let rec = reconstruct_inter_block(&pred, &res);
        assert!(rec.iter().all(|&v| v == 120));
    }
}
