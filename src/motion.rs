//! Motion-vector VLC encode/decode + median predictor + half-pel interpolator
//! for H.263 baseline P-pictures.
//!
//! Baseline H.263 uses `f_code == 1` (no Annex D unrestricted MV), so
//! the MV magnitude is transmitted as a direct VLC codeword (Table 14/H.263,
//! identical to MPEG-4 `ff_mvtab`) followed by a sign bit (when magnitude > 0).
//! There is no `motion_residual` field. The reconstructed differential `d` is
//! added to the median predictor and folded into the valid half-pel range
//! `[-16, +15.5]` = `[-32, +31]` halfpel units.
//!
//! 1-MV mode only (no Annex F 4MV / OBMC). Each P-MB carries a single 2-D
//! motion vector in luma half-pel units; chroma vectors are derived by
//! `luma_mv_to_chroma` which maps the luma MV (possibly half-pel) into the
//! coarser chroma grid per Table 7-15 of H.263.
//!
//! # Coordinate system
//! All MVs are in **luma half-pel units**. Integer pel shift is `mv_half * 2`,
//! so `(mv_x_half=2, mv_y_half=0)` means "shift source by 1 luma pel to the
//! right". `(1, 0)` is a half-pel position that requires bilinear
//! interpolation.
//!
//! # Annex D — Unrestricted Motion Vectors
//! When the picture header signals UMV (PTYPE bit 10 without PLUSPTYPE, or
//! OPPTYPE bit 5 in a PLUSPTYPE form) the MV component range is extended to
//! `[-31.5, 31.5]` pels = `[-63, 63]` halfpel for baseline-form streams, and
//! motion vectors may point outside the picture — the interpolator replicates
//! edge samples to form out-of-picture references (§D.1). The MVD VLC is
//! still Table 14, but its *interpretation* changes per §D.2: if the
//! median predictor lies in `[-31, +32]` halfpel (= `[-15.5, +16]` pel), the
//! decoded differential `d` is used directly (same as baseline). If the
//! predictor is outside that range, the decoder picks among `{d, d+64, d-64}`
//! whichever produces a reconstructed component that lies in `[-63, +63]`
//! halfpel **and** has the same sign as the predictor (zero counts as
//! either sign). For PLUSPTYPE-form Annex D, Table D.3 is used instead;
//! that path is implemented by [`decode_mvd_plusptype_umv`] / [`encode_mvd_plusptype_umv`]
//! (the "regular-structure MVD VLC" of Table D.3/H.263). Under PLUSPTYPE
//! the reconstructed vector is simply `predictor + differential` (no
//! sign-of-predictor cascade) — §D.2 replaces that rule with a direct
//! range limit per Tables D.1 / D.2 when UUI = "1" or an unlimited
//! range (picture size only) when UUI = "01".
//!
//! Cross-checked against libavcodec's `h263dec.c` MV parsing + `h263.c`
//! median predictor.

use oxideav_core::bits::BitReader;
use oxideav_core::Result;
use oxideav_mpeg4video::tables::{mv as mv_tab, vlc};

use oxideav_core::bits::BitWriter;

/// Valid half-pel MV range for baseline H.263 (f_code == 1): each component
/// lies in `[-32, +31]` half-pel units, i.e. `[-16, +15.5]` luma pels.
pub const MV_RANGE_MIN_HALF: i32 = -32;
pub const MV_RANGE_MAX_HALF: i32 = 31;

/// Valid half-pel MV range when Annex D (UMV) is signalled without PLUSPTYPE.
/// Each component may land anywhere in `[-63, +63]` half-pel units (i.e.
/// `[-31.5, +31.5]` pels). The §D.2 "sign-of-predictor" constraint still
/// restricts which half of the range a given predictor can reach, but the
/// wrapped / absolute value itself lives in this extended interval.
pub const MV_RANGE_UMV_MIN_HALF: i32 = -63;
pub const MV_RANGE_UMV_MAX_HALF: i32 = 63;

/// Fold a reconstructed MV component into the valid half-pel domain.
///
/// H.263 §5.3.7.3: if the predictor + decoded differential falls outside
/// `[-32, +31]` (half-pel units), add or subtract `64` to bring it back in.
pub fn wrap_mv_component(v: i32) -> i32 {
    let range = 32;
    let mut m = v;
    if m < -range {
        m += 2 * range;
    } else if m >= range {
        m -= 2 * range;
    }
    m
}

/// Apply the §D.2 reconstruction rule for Annex D without PLUSPTYPE.
///
/// Given a predictor `pred_half` (halfpel units) and a decoded differential
/// magnitude `mag_half` (positive) with `sign` `+1` or `-1`, pick the signed
/// differential among `{d, d+64, d-64}` whose sum with `pred_half` yields a
/// component in `[-63, 63]`. When multiple candidates qualify (possible when
/// the predictor is near the baseline boundary), prefer the candidate with
/// the same sign as `pred_half` per §D.2; `0` counts as either sign.
///
/// Returns the reconstructed absolute MV component in halfpel units.
pub fn reconstruct_umv_component(pred_half: i32, mag: i32, sign: i32) -> i32 {
    let raw_diff = mag * sign; // positive sign → `-diff` as the VLC sign bit semantics
                               // Candidates per §D.2.
    let candidates = [raw_diff, raw_diff + 64, raw_diff - 64];
    // Filter to those that land inside the extended UMV range.
    let mut best: Option<i32> = None;
    for &d in &candidates {
        let v = pred_half + d;
        if !(MV_RANGE_UMV_MIN_HALF..=MV_RANGE_UMV_MAX_HALF).contains(&v) {
            continue;
        }
        // §D.2 "same sign as predictor (including zero)" rule when the
        // predictor is outside `[-31, +32]`. When the predictor is inside
        // that baseline range, the spec says "only the first column
        // applies" → the raw, unshifted `d` is the answer and no wrapping
        // should be needed unless the raw differential magnitude itself is
        // already in [−32, +31]: the first `d` candidate is then the hit.
        if (-31..=32).contains(&pred_half) {
            // Baseline interpretation — the raw `d` (candidates[0]) wins;
            // keep it only if in range.
            if d == raw_diff {
                return v;
            } else {
                continue;
            }
        }
        // Predictor outside baseline range: pick candidate whose sign
        // matches the predictor (zero counts as same-sign).
        let same_sign = match pred_half.cmp(&0) {
            std::cmp::Ordering::Greater => v >= 0,
            std::cmp::Ordering::Less => v <= 0,
            std::cmp::Ordering::Equal => true,
        };
        if same_sign {
            return v;
        }
        // Remember as fallback.
        if best.is_none() {
            best = Some(v);
        }
    }
    // Fallback: any in-range candidate. This shouldn't happen on conforming
    // streams but guards against rounding.
    best.unwrap_or_else(|| wrap_mv_component(pred_half + raw_diff))
}

/// Reverse map for the MV magnitude VLC (`mv_tab::table()`). Magnitude index →
/// `(bits, code)`. Lifted from FFmpeg `ff_mvtab`; mirrored in
/// `oxideav_mpeg4video::tables::mv` but kept private there. We inline the
/// codewords here because the encoder path needs them and the decode table
/// doesn't expose them.
const MV_ENC_VLC: [(u8, u32); 33] = [
    (1, 1),   // 0
    (2, 1),   // 1
    (3, 1),   // 2
    (4, 1),   // 3
    (6, 3),   // 4
    (7, 5),   // 5
    (7, 4),   // 6
    (7, 3),   // 7
    (9, 11),  // 8
    (9, 10),  // 9
    (9, 9),   // 10
    (10, 17), // 11
    (10, 16), // 12
    (10, 15), // 13
    (10, 14), // 14
    (10, 13), // 15
    (10, 12), // 16
    (10, 11), // 17
    (10, 10), // 18
    (10, 9),  // 19
    (10, 8),  // 20
    (10, 7),  // 21
    (10, 6),  // 22
    (10, 5),  // 23
    (10, 4),  // 24
    (11, 7),  // 25
    (11, 6),  // 26
    (11, 5),  // 27
    (11, 4),  // 28
    (11, 3),  // 29
    (11, 2),  // 30
    (12, 3),  // 31
    (12, 2),  // 32
];

/// Decode one MV component from the bitstream, given the predictor (in luma
/// half-pel units). Returns the reconstructed absolute MV component.
///
/// The decoded symbol is the motion-code magnitude; for a nonzero magnitude we
/// also read a sign bit. The MV differential is the signed motion-code value
/// (no `motion_residual` because f_code == 1). The reconstructed vector is
/// `predictor + diff` folded into `[-32, +31]` via `wrap_mv_component`.
pub fn decode_mv_component(br: &mut BitReader<'_>, predictor_half: i32) -> Result<i32> {
    decode_mv_component_umv(br, predictor_half, false)
}

/// Decode one MV component with optional Annex D Unrestricted Motion Vector
/// interpretation. When `umv` is `true`, the §D.2 "sign-of-predictor" rule
/// is applied and the output range widens to `[-63, +63]` halfpel
/// (`MV_RANGE_UMV_*`). When `false`, behaviour matches
/// [`decode_mv_component`] / baseline H.263 (range folded to `[-32, +31]`).
pub fn decode_mv_component_umv(
    br: &mut BitReader<'_>,
    predictor_half: i32,
    umv: bool,
) -> Result<i32> {
    let magnitude = vlc::decode(br, mv_tab::table())? as i32;
    if magnitude == 0 {
        return Ok(if umv {
            // §D.2: with UMV and zero magnitude, the result is just the
            // predictor (no wrap needed — predictor is already in range).
            predictor_half
        } else {
            wrap_mv_component(predictor_half)
        });
    }
    let sign_bit = br.read_u1()? as i32;
    // VLC sign: `1` = negative differential, `0` = positive.
    let sign_dir = if sign_bit == 1 { -1 } else { 1 };
    if umv {
        Ok(reconstruct_umv_component(
            predictor_half,
            magnitude,
            sign_dir,
        ))
    } else {
        let diff = magnitude * sign_dir;
        Ok(wrap_mv_component(predictor_half + diff))
    }
}

/// Decode one MVD component under Annex D PLUSPTYPE — the "regular-structure
/// MVD VLC" of Table D.3/H.263.
///
/// Returns the signed differential in half-pel units. The reconstructed MV
/// component is simply `predictor + differential` (no wrap, no
/// sign-of-predictor cascade — §D.2 replaces that rule with a direct range
/// limit per Tables D.1/D.2).
///
/// Table D.3 codewords are built as:
///   * value 0   →  `1` (no sign bit)
///   * value > 0 →  `0` followed by, for each x-bit after the leading 1
///     (high to low), the pair `<x_i> 1`; then `<s> 0`
///     where `s = 0` is positive, `s = 1` is negative.
///
/// Length = `1` for zero, `2k + 3` for value with `k` x-bits (`abs` in
/// `[2^k, 2^(k+1))` once `k >= 1`; value 1 has `k = 0`).
pub fn decode_mvd_table_d3(br: &mut BitReader<'_>) -> Result<i32> {
    // First bit: `1` means value 0, `0` means nonzero and more bits follow.
    let first = br.read_u1()?;
    if first == 1 {
        return Ok(0);
    }
    // Collect alternating (x, continue) pairs. `continue == 1` → `x` is a
    // data bit; `continue == 0` → `x` is the sign bit and the code ends.
    let mut xbits: Vec<u32> = Vec::with_capacity(12);
    let sign_bit;
    // Guard against a pathological bitstream that keeps claiming
    // "continue"; Table D.3 goes up to 2047 (11 x-bits) and at most 12
    // x-bits for the warping parameter profile in §O. We stop at 16.
    loop {
        if xbits.len() > 16 {
            return Err(oxideav_core::Error::invalid(
                "h263 Annex D Table D.3: MVD VLC exceeded maximum length",
            ));
        }
        let b = br.read_u1()?;
        let cont = br.read_u1()?;
        if cont == 0 {
            sign_bit = b;
            break;
        }
        xbits.push(b);
    }
    // Assemble the absolute value: leading `1` followed by `xbits` from
    // high to low (in the order they were read).
    let mut abs: i32 = 1;
    for x in &xbits {
        abs = (abs << 1) | (*x as i32);
    }
    let signed = if sign_bit == 1 { -abs } else { abs };
    Ok(signed)
}

/// Emit the Table D.3/H.263 codeword for a signed MVD component.
///
/// See [`decode_mvd_table_d3`] for the VLC structure. For value 0 the
/// codeword is a single `1` bit (no sign).
pub fn encode_mvd_table_d3(bw: &mut BitWriter, diff: i32) {
    if diff == 0 {
        bw.write_bits(1, 1);
        return;
    }
    let sign_bit: u32 = if diff < 0 { 1 } else { 0 };
    let abs = diff.unsigned_abs();
    // Build the x-bits (all bits below the leading 1) from high to low.
    // leading = position of the leading 1-bit.
    let leading = 31 - abs.leading_zeros();
    // x-bits are positions `leading-1 .. 0`.
    bw.write_bits(0, 1); // value != 0
    if leading > 0 {
        for k in (0..leading).rev() {
            let x = (abs >> k) & 1;
            bw.write_bits(x, 1);
            bw.write_bits(1, 1); // continue
        }
    }
    // Terminating pair: `<s> <0>`.
    bw.write_bits(sign_bit, 1);
    bw.write_bits(0, 1);
}

/// Decode one MV component under Annex D + PLUSPTYPE (Table D.3). Reads the
/// Table D.3 codeword from `br` and returns the **reconstructed** component
/// in half-pel units: `predictor + differential`.
///
/// When `limit` is `Some((min, max))` the reconstructed component is checked
/// against the UUI = "1" range from Tables D.1/D.2; out-of-range values
/// yield `Error::invalid`. `None` selects UUI = "01" behaviour — unlimited
/// range (modulo picture size, which we do not enforce here).
pub fn decode_mv_component_plusptype_umv(
    br: &mut BitReader<'_>,
    predictor_half: i32,
    limit: Option<(i32, i32)>,
) -> Result<i32> {
    let diff = decode_mvd_table_d3(br)?;
    let mv = predictor_half + diff;
    if let Some((lo, hi)) = limit {
        if mv < lo || mv > hi {
            return Err(oxideav_core::Error::invalid(format!(
                "h263 Annex D PLUSPTYPE: reconstructed MV {mv} halfpel out of range [{lo}, {hi}]"
            )));
        }
    }
    Ok(mv)
}

/// Decode one MVD component as a **pure differential** (Table 14 magnitude +
/// sign, no wrap, no sign-of-predictor cascade). Used by the Annex G
/// PB-frames `MVDB` decode (§5.3.9 / §G.4) where the predictor is the
/// scaled forward vector and the codeword carries the offset directly.
///
/// Returns the signed differential in half-pel units in `[-32, +32]`.
pub fn decode_mvd_pure_differential(br: &mut BitReader<'_>) -> Result<i32> {
    let magnitude = vlc::decode(br, mv_tab::table())? as i32;
    if magnitude == 0 {
        return Ok(0);
    }
    let sign_bit = br.read_u1()? as i32;
    let sign_dir = if sign_bit == 1 { -1 } else { 1 };
    Ok(magnitude * sign_dir)
}

/// Emit one MVD component as a pure differential — Table 14 magnitude +
/// sign with no wrap. Mirrors [`decode_mvd_pure_differential`] and is used
/// by the Annex G PB-frames `MVDB` encode path.
pub fn encode_mvd_pure_differential(bw: &mut BitWriter, diff: i32) {
    let mag = diff.unsigned_abs() as usize;
    debug_assert!(mag <= 32, "MVDB differential out of range");
    let (bits, code) = MV_ENC_VLC[mag];
    bw.write_bits(code, bits as u32);
    if mag > 0 {
        let sign: u32 = if diff < 0 { 1 } else { 0 };
        bw.write_bits(sign, 1);
    }
}

/// Decode the MVD-pair start-code-emulation-prevention bit (§D.2 last
/// paragraph): "if a pair equals (0.5, 0.5) six consecutive zeros are
/// produced. To prevent start code emulation, this occurrence shall be
/// followed by one bit set to '1'."
///
/// The "(0.5, 0.5)" refers to +1 half-pel on each axis; both components'
/// Table D.3 codes are then `000` each → six zeros concatenated. We detect
/// that case by inspecting the *decoded differentials* rather than the raw
/// bits. Call this helper after decoding both horizontal + vertical
/// components of an MVD (or MVD2-4 under Annex F) when PLUSPTYPE+UMV is
/// active.
pub fn consume_mvd_pair_sce_bit(
    br: &mut BitReader<'_>,
    diff_horiz: i32,
    diff_vert: i32,
) -> Result<()> {
    if diff_horiz == 1 && diff_vert == 1 {
        // Expect a single `1` bit; any other value implies a malformed
        // stream. We tolerate a `0` here as well (some encoders elide the
        // bit when the resulting byte happens not to reach the start-code
        // pattern) but don't advertise that leniency.
        let b = br.read_u1()?;
        if b != 1 {
            return Err(oxideav_core::Error::invalid(
                "h263 Annex D Table D.3: start-code-emulation stuffing bit after (+1,+1) pair was not 1",
            ));
        }
    }
    Ok(())
}

/// Return the UUI = "1" MV-component range `(min, max)` in half-pel units
/// for a picture whose luma dimension (width for horizontal, height for
/// vertical) is `dim` samples. Mirrors Tables D.1 / D.2 of H.263.
///
/// * `dim ≤ 352`  → `[-32, +31.5]` pel = `[-64, +63]` halfpel
/// * `dim ≤ 704`  → `[-64, +63.5]` pel = `[-128, +127]` halfpel
/// * `dim ≤ 1408` → `[-128, +127.5]` pel = `[-256, +255]` halfpel
/// * larger       → `[-256, +255.5]` pel = `[-512, +511]` halfpel
pub fn uui_limit_range_halfpel(dim: u32) -> (i32, i32) {
    if dim <= 352 {
        (-64, 63)
    } else if dim <= 704 {
        (-128, 127)
    } else if dim <= 1408 {
        (-256, 255)
    } else {
        (-512, 511)
    }
}

/// Encode one MV component into `bw`, given the predictor.
///
/// The emitted differential is `mv - predictor`, which may need to be folded
/// into `[-32, +31]` (half-pel units) to select the shortest codeword — if the
/// predictor is near the boundary, the "wrap-around" differential can be
/// smaller in magnitude than the straightforward one. We pick whichever of the
/// two candidates has the smaller absolute value (ties broken toward the
/// non-wrapped form, which matches FFmpeg).
pub fn encode_mv_component(bw: &mut BitWriter, mv_half: i32, predictor_half: i32) {
    let raw_diff = mv_half - predictor_half;
    // Candidate: fold diff into the signed range [-32, +31].
    let folded = {
        let mut d = raw_diff;
        while d < -32 {
            d += 64;
        }
        while d > 31 {
            d -= 64;
        }
        d
    };
    // Verify the encoded vector round-trips through the decoder — the decoder
    // computes `wrap(predictor + diff)`, so we pick the smallest-magnitude
    // `diff` in `[-32, +31]` that yields `mv_half` after wrap.
    debug_assert_eq!(wrap_mv_component(predictor_half + folded), mv_half);
    let diff = folded;
    let mag = diff.unsigned_abs() as usize;
    debug_assert!(mag <= 32);
    let (bits, code) = MV_ENC_VLC[mag];
    bw.write_bits(code, bits as u32);
    if mag > 0 {
        let sign: u32 = if diff < 0 { 1 } else { 0 };
        bw.write_bits(sign, 1);
    }
}

/// Encode one MV component under Annex D (Unrestricted Motion Vectors,
/// baseline-PTYPE form). Selects the magnitude+sign whose decode through
/// [`reconstruct_umv_component`] yields `mv_half`.
///
/// The reconstructed component lives in `[-63, +63]` halfpel units. The
/// VLC magnitude itself is still bounded to 32 (Table 14), so the §D.2
/// "{d, d+64, d-64}" cascade is what extends the reach; this function picks
/// the candidate `(mag, sign)` whose decode matches `mv_half` and whose VLC
/// codeword is shortest. When two candidates produce the same codeword
/// length, the non-wrapped form (the one closest to `mv_half - predictor`)
/// wins to mirror the encoder's `encode_mv_component` tie-break.
///
/// Caller is responsible for ensuring `mv_half ∈ [-63, +63]` and
/// `predictor_half ∈ [-63, +63]` halfpel.
pub fn encode_mv_component_umv(bw: &mut BitWriter, mv_half: i32, predictor_half: i32) {
    debug_assert!((MV_RANGE_UMV_MIN_HALF..=MV_RANGE_UMV_MAX_HALF).contains(&mv_half));
    debug_assert!((MV_RANGE_UMV_MIN_HALF..=MV_RANGE_UMV_MAX_HALF).contains(&predictor_half));
    // Fast-path: when the predictor lies in the baseline range `[-31, +32]`
    // halfpel, §D.2 explicitly says "only the first column" applies — i.e.
    // the decoder uses the raw differential (no wrap, no sign cascade). In
    // that regime the UMV encoder must match the baseline encoder byte-for-
    // byte so streams without UMV-grade MVs round-trip identically.
    if (-31..=32).contains(&predictor_half) {
        let raw_diff = mv_half - predictor_half;
        // Try the raw differential first (no wrap). If `|raw_diff| <= 32` it
        // is a valid Table-14 magnitude and we emit it directly.
        if (-32..=32).contains(&raw_diff) {
            let mag = raw_diff.unsigned_abs() as usize;
            let (bits, code) = MV_ENC_VLC[mag];
            bw.write_bits(code, bits as u32);
            if mag > 0 {
                let sign_bit: u32 = if raw_diff < 0 { 1 } else { 0 };
                bw.write_bits(sign_bit, 1);
            }
            return;
        }
        // Out-of-baseline mv with predictor in baseline range: §D.2 says the
        // raw differential rule applies, so the magnitude *is* `|raw_diff|`
        // even though it exceeds the table. This shouldn't happen because
        // `raw_diff > 32` ↔ `mv_half > predictor_half + 32`, and with
        // `predictor_half ≤ 32` and `mv_half ≤ 63` the max `raw_diff` is
        // 63 - (-31) = 94, which is unreachable through the §D.2 candidate
        // set. We fall through to the general enumerator just in case.
    }
    // General case: enumerate every legal (mag, sign) and find the shortest
    // codeword whose decode reproduces `mv_half`. `mag == 0` yields
    // `predictor_half` only.
    let mut best: Option<(u8, i32, i32)> = None; // (codeword bits, mag, sign)
    for mag in 0..=32i32 {
        let signs: &[i32] = if mag == 0 { &[1] } else { &[1, -1] };
        for &sign in signs {
            let v = reconstruct_umv_component(predictor_half, mag, sign);
            if v != mv_half {
                continue;
            }
            // codeword length: VLC magnitude bits, plus 1 sign bit when mag > 0.
            let len = MV_ENC_VLC[mag as usize].0 + if mag > 0 { 1 } else { 0 };
            // Tie-break: prefer the candidate whose `mag * sign_dir` is
            // closer to the raw `(mv_half - predictor_half)` modulo 64.
            let diff_raw = mv_half - predictor_half;
            let here_diff = mag * if sign == 1 { 1 } else { -1 };
            let prefer = match best {
                None => true,
                Some((bl, bm, bs)) => {
                    if len < bl {
                        true
                    } else if len > bl {
                        false
                    } else {
                        let prev_diff = bm * if bs == 1 { 1 } else { -1 };
                        (here_diff - diff_raw).abs() < (prev_diff - diff_raw).abs()
                    }
                }
            };
            if prefer {
                best = Some((len, mag, sign));
            }
        }
    }
    let (_len, mag, sign) =
        best.expect("encode_mv_component_umv: no codeword reproduces mv_half through §D.2");
    let (bits, code) = MV_ENC_VLC[mag as usize];
    bw.write_bits(code, bits as u32);
    if mag > 0 {
        let sign_bit: u32 = if sign == -1 { 1 } else { 0 };
        bw.write_bits(sign_bit, 1);
    }
}

/// Per-MB motion-vector slot (luma half-pel units). One vector per MB in 1MV
/// mode — all four luma blocks share it. The value is also used for the
/// median predictor of subsequent MBs.
///
/// When Advanced Prediction (Annex F) is active, `mvs4` carries a per-8×8-block
/// vector (raster: `[top-left, top-right, bottom-left, bottom-right]`,
/// matching Figure 5's luminance block numbering 1..=4). For 1MV mode MBs all
/// four entries are copies of `mv` — this matches the spec's
/// "if only one motion vector is transmitted... this is defined as four
/// vectors with the same value" (§F.2). The picture-level predictor for a
/// subsequent MB's block uses the per-block neighbour (block 2 / block 4 /
/// block 3 depending on position; see [`predict_mv_block`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MbMotion {
    pub mv: (i32, i32),
    /// Per-8×8-luma-block motion vectors, indexed `0..4` in Figure 5 order.
    /// Always populated — in 1MV mode all four entries hold the same value
    /// as `mv`.
    pub mvs4: [(i32, i32); 4],
    /// True iff this MB was coded (intra or inter). A non-coded (skipped) MB
    /// contributes `(0, 0)` to future MV predictors per §5.3.4.
    pub coded: bool,
    /// True iff this MB is intra-in-P — in which case its MV is (0,0) and
    /// shouldn't propagate. We still keep it as a neighbour with MV (0,0).
    pub intra: bool,
    /// True iff the MB carried 4 per-block motion vectors (Annex F 4MV mode).
    pub four_mv: bool,
}

impl MbMotion {
    /// Construct a 1-MV `MbMotion` where all four per-block slots are copies
    /// of `mv`. Used by every decode / encode path that doesn't explicitly
    /// handle Annex F 4MV mode.
    pub fn mv1(mv: (i32, i32), coded: bool, intra: bool) -> Self {
        Self {
            mv,
            mvs4: [mv; 4],
            coded,
            intra,
            four_mv: false,
        }
    }

    /// Construct a 4-MV `MbMotion` for Annex F (Advanced Prediction). `mv` is
    /// the first block's MV (conventionally used as the summary MV when
    /// downstream code still thinks in 1MV terms — e.g. for median-predictor
    /// fallback when a neighbour wasn't 4MV-aware).
    pub fn mv4(mvs4: [(i32, i32); 4]) -> Self {
        Self {
            mv: mvs4[0],
            mvs4,
            coded: true,
            intra: false,
            four_mv: true,
        }
    }
}

/// Raster grid of per-MB motion vectors, queried by the median predictor.
#[derive(Clone, Debug)]
pub struct MvGrid {
    pub mb_w: usize,
    pub mb_h: usize,
    /// `[mb_y * mb_w + mb_x] -> MbMotion`.
    pub mvs: Vec<MbMotion>,
}

impl MvGrid {
    pub fn new(mb_w: usize, mb_h: usize) -> Self {
        Self {
            mb_w,
            mb_h,
            mvs: vec![MbMotion::default(); mb_w * mb_h],
        }
    }

    pub fn get(&self, mb_x: usize, mb_y: usize) -> MbMotion {
        self.mvs[mb_y * self.mb_w + mb_x]
    }

    pub fn set(&mut self, mb_x: usize, mb_y: usize, m: MbMotion) {
        self.mvs[mb_y * self.mb_w + mb_x] = m;
    }

    /// Safe lookup that returns `None` when `(mb_x, mb_y)` is outside the
    /// grid. Used by the OBMC path.
    pub fn get_opt(&self, mb_x: isize, mb_y: isize) -> Option<MbMotion> {
        if mb_x < 0 || mb_y < 0 {
            return None;
        }
        let (x, y) = (mb_x as usize, mb_y as usize);
        if x >= self.mb_w || y >= self.mb_h {
            return None;
        }
        Some(self.get(x, y))
    }
}

/// Compute the median motion-vector predictor for the current MB (§5.3.7.3
/// figure 8 of H.263; identical to MPEG-4 1MV case). For baseline 1-MV H.263
/// we take the three neighbours:
/// * MV1 = left neighbour
/// * MV2 = top neighbour
/// * MV3 = top-right neighbour
///
/// Unavailable neighbours (picture edge) are substituted per spec:
/// * If only MV1 is unavailable → all three set to `(0,0)`.
/// * Else if MV2 is unavailable → MV2 = MV3 = MV1.
/// * Else if MV3 is unavailable → MV3 = `(0,0)`.
///
/// Non-coded neighbours contribute `(0,0)` as their MV (per §5.3.4) — this is
/// already what the `MbMotion::default()` gives.
pub fn predict_mv(grid: &MvGrid, mb_x: usize, mb_y: usize) -> (i32, i32) {
    predict_mv_with_gob_mask(grid, mb_x, mb_y, false)
}

/// Variant of [`predict_mv`] that treats the "above" neighbour row as
/// unavailable for predictor purposes when `gob_top_row` is `true`. Per
/// §6.1.1 rule 3, a non-empty GOB header forces MV2 and MV3 → MV1 (which,
/// combined with rule 4 for picture edges, collapses to all-zero at a
/// corner). The actual MV values remain in `grid` for OBMC lookups (§F.3
/// explicitly allows cross-GOB remote MVs outside Slice Structured mode).
pub fn predict_mv_with_gob_mask(
    grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    gob_top_row: bool,
) -> (i32, i32) {
    let get = |x: usize, y: usize| -> (i32, i32) {
        if x >= grid.mb_w || y >= grid.mb_h {
            (0, 0)
        } else {
            grid.get(x, y).mv
        }
    };

    let mv1 = if mb_x > 0 {
        Some(get(mb_x - 1, mb_y))
    } else {
        None
    };
    let above_available = mb_y > 0 && !gob_top_row;
    let mv2 = if above_available {
        Some(get(mb_x, mb_y - 1))
    } else {
        None
    };
    let mv3 = if above_available && mb_x + 1 < grid.mb_w {
        Some(get(mb_x + 1, mb_y - 1))
    } else {
        None
    };

    let (mv1, mv2, mv3) = match (mv1, mv2, mv3) {
        (None, _, _) => ((0, 0), (0, 0), (0, 0)),
        (Some(a), None, _) => (a, a, a),
        (Some(a), Some(b), None) => (a, b, (0, 0)),
        (Some(a), Some(b), Some(c)) => (a, b, c),
    };

    (median3(mv1.0, mv2.0, mv3.0), median3(mv1.1, mv2.1, mv3.1))
}

/// Compute the median predictor for one 8×8 luminance block inside macroblock
/// `(mb_x, mb_y)` when Annex F (4MV / Advanced Prediction) is active. Per
/// §F.2 Figure F.1 the candidate predictors MV1, MV2 and MV3 are redefined
/// for each of the four blocks (Figure 5 numbering, `block_idx = 0..=3`):
///
/// * `block_idx == 0` (top-left): MV1 = left MB's block 1 (bottom-right
///   position of that MB's 2×2 block grid); MV2 = above MB's block 2
///   (bottom-left); MV3 = above-right MB's block 2.
/// * `block_idx == 1` (top-right): MV1 = same MB's block 0; MV2 = above MB's
///   block 3 (bottom-right); MV3 = above-right MB's block 2.
/// * `block_idx == 2` (bottom-left): MV1 = left MB's block 3 (bottom-right);
///   MV2 = same MB's block 0; MV3 = same MB's block 1.
/// * `block_idx == 3` (bottom-right): MV1 = same MB's block 2; MV2 = same
///   MB's block 1; MV3 = the above-right-of-this-block position which lies in
///   the right-neighbour MB's block 2 — that MB hasn't been decoded yet, so
///   per §6.1.1 rule 4 MV3 is considered outside the picture and treated
///   as unavailable (substituted per the standard `{None-cascade}` rules).
///
/// Missing neighbours follow the §6.1.1 cascade: if MV1 is unavailable all
/// three collapse to `(0,0)`; if only MV2 missing then MV2 = MV3 = MV1; if
/// only MV3 missing then MV3 = `(0,0)`.
///
/// Intra / skipped neighbours contribute `(0,0)` per §6.1.1 rule 1, which is
/// already the stored value for [`MbMotion::default()`] (the non-coded slot).
pub fn predict_mv_block(grid: &MvGrid, mb_x: usize, mb_y: usize, block_idx: usize) -> (i32, i32) {
    predict_mv_block_with_gob_mask(grid, mb_x, mb_y, block_idx, false)
}

/// GOB-boundary variant of [`predict_mv_block`]: when `gob_top_row` is
/// `true`, the "above" neighbour row is treated as unavailable for the
/// per-block predictor (same substitution as the 1MV path does in
/// [`predict_mv_with_gob_mask`]).
pub fn predict_mv_block_with_gob_mask(
    grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    gob_top_row: bool,
) -> (i32, i32) {
    let get_block = |mx: usize, my: usize, bi: usize| -> (i32, i32) {
        if mx >= grid.mb_w || my >= grid.mb_h {
            return (0, 0);
        }
        let m = grid.get(mx, my);
        if !m.coded || m.intra {
            return (0, 0);
        }
        m.mvs4[bi]
    };

    let above_available_row = mb_y > 0 && !gob_top_row;

    // Determine which of the four 8×8 blocks the current block is.
    // block_idx ordering: 0=top-left, 1=top-right, 2=bot-left, 3=bot-right.
    let (mv1, mv2, mv3) = match block_idx {
        0 => {
            // Left MB's block 1 (top-right): same MB-row, MB_x-1.
            let left = if mb_x > 0 {
                Some(get_block(mb_x - 1, mb_y, 1))
            } else {
                None
            };
            // Above MB's block 2 (bottom-left): same MB-column, MB_y-1.
            let above = if above_available_row {
                Some(get_block(mb_x, mb_y - 1, 2))
            } else {
                None
            };
            // Above-right MB's block 2 (bottom-left of that MB).
            let above_right = if above_available_row && mb_x + 1 < grid.mb_w {
                Some(get_block(mb_x + 1, mb_y - 1, 2))
            } else {
                None
            };
            (left, above, above_right)
        }
        1 => {
            // Left = block 0 of the same MB (always available after block 0
            // is decoded — all four block vectors are decoded sequentially
            // in the MB layer before the OBMC pass).
            let left = Some(grid.get(mb_x, mb_y).mvs4[0]);
            // Above MB's block 3 (bottom-right).
            let above = if above_available_row {
                Some(get_block(mb_x, mb_y - 1, 3))
            } else {
                None
            };
            // Above-right MB's block 2 (bottom-left of above-right MB).
            let above_right = if above_available_row && mb_x + 1 < grid.mb_w {
                Some(get_block(mb_x + 1, mb_y - 1, 2))
            } else {
                None
            };
            (left, above, above_right)
        }
        2 => {
            // Left MB's block 3 (bottom-right).
            let left = if mb_x > 0 {
                Some(get_block(mb_x - 1, mb_y, 3))
            } else {
                None
            };
            // Above = block 0 of same MB.
            let above = Some(grid.get(mb_x, mb_y).mvs4[0]);
            // Above-right = block 1 of same MB.
            let above_right = Some(grid.get(mb_x, mb_y).mvs4[1]);
            (left, above, above_right)
        }
        3 => {
            // Left = block 2 of same MB.
            let left = Some(grid.get(mb_x, mb_y).mvs4[2]);
            // Above = block 1 of same MB.
            let above = Some(grid.get(mb_x, mb_y).mvs4[1]);
            // Above-right is in the right-neighbour MB which hasn't been
            // decoded yet — treat as unavailable (set to zero per §6.1.1
            // rule 4).
            let above_right: Option<(i32, i32)> = None;
            (left, above, above_right)
        }
        _ => unreachable!("block_idx must be 0..=3"),
    };

    let (mv1, mv2, mv3) = match (mv1, mv2, mv3) {
        (None, _, _) => ((0, 0), (0, 0), (0, 0)),
        (Some(a), None, _) => (a, a, a),
        (Some(a), Some(b), None) => (a, b, (0, 0)),
        (Some(a), Some(b), Some(c)) => (a, b, c),
    };

    (median3(mv1.0, mv2.0, mv3.0), median3(mv1.1, mv2.1, mv3.1))
}

/// Compute MVDCHR (§F.2): sum the four luma per-block MVs, divide by 8, and
/// apply Table F.1 rounding towards the nearest half-pixel (sixteenth-pixel →
/// half-pixel). Returns the chroma MV in luma half-pel units (i.e. already
/// matching the chroma-plane coordinate system used by `predict_block`).
pub fn chroma_mv_4mv(mvs: &[(i32, i32); 4]) -> (i32, i32) {
    // Each luma MV is in half-pel units. Sum/8 gives sixteenth-pel chroma
    // (luma-half divided by 8 = quarter-pel, then ÷2 for chroma → eighth-pel
    // in luma half-pel units; the exact derivation is given by Table F.1).
    //
    // Table F.1 maps a sixteenth-pel value s = sum(mvs)/8 to a half-pel
    // chroma coordinate as:
    //   s mod 16:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
    //   out half:  0  0  0  1  1  1  1  1  1  1  1  1  1  1  2  2
    // i.e. effectively "quantise the fractional sixteenth towards the
    // nearest half" with a threshold at 3/16 and 14/16.
    let map = |sum: i32| -> i32 {
        // sum = sixteenth-pel * 1 (sum of luma halfpels).
        // chroma half-pel = floor(sum/16)*2 + round_table[sum mod 16].
        // Negative sums must map symmetrically: the standard does so by
        // treating `sum` as a signed value and the rounding is applied to
        // the 16-residue (computed on the positive-wrapped value).
        let div = sum.div_euclid(16); // signed floor-div.
        let rem = sum.rem_euclid(16) as usize; // 0..=15
        const TABLE: [i32; 16] = [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2];
        div * 2 + TABLE[rem]
    };

    let sx = mvs[0].0 + mvs[1].0 + mvs[2].0 + mvs[3].0;
    let sy = mvs[0].1 + mvs[1].1 + mvs[2].1 + mvs[3].1;
    (map(sx), map(sy))
}

/// OBMC weighting matrix `H0` for the current-block MV (§F.3, Figure F.2).
/// Row-major 8×8 matrix — index `[j * 8 + i]` for column `i`, row `j`.
#[rustfmt::skip]
pub const OBMC_H0: [[u8; 8]; 8] = [
    [4, 5, 5, 5, 5, 5, 5, 4],
    [5, 5, 5, 5, 5, 5, 5, 5],
    [5, 5, 6, 6, 6, 6, 5, 5],
    [5, 5, 6, 6, 6, 6, 5, 5],
    [5, 5, 6, 6, 6, 6, 5, 5],
    [5, 5, 6, 6, 6, 6, 5, 5],
    [5, 5, 5, 5, 5, 5, 5, 5],
    [4, 5, 5, 5, 5, 5, 5, 4],
];

/// OBMC weighting matrix `H1` for the top/bottom remote MV (§F.3, Figure F.3).
#[rustfmt::skip]
pub const OBMC_H1: [[u8; 8]; 8] = [
    [2, 2, 2, 2, 2, 2, 2, 2],
    [1, 1, 2, 2, 2, 2, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 2, 2, 2, 2, 1, 1],
    [2, 2, 2, 2, 2, 2, 2, 2],
];

/// OBMC weighting matrix `H2` for the left/right remote MV (§F.3, Figure F.4).
#[rustfmt::skip]
pub const OBMC_H2: [[u8; 8]; 8] = [
    [2, 1, 1, 1, 1, 1, 1, 2],
    [2, 2, 1, 1, 1, 1, 2, 2],
    [2, 2, 1, 1, 1, 1, 2, 2],
    [2, 2, 1, 1, 1, 1, 2, 2],
    [2, 2, 1, 1, 1, 1, 2, 2],
    [2, 2, 1, 1, 1, 1, 2, 2],
    [2, 2, 1, 1, 1, 1, 2, 2],
    [2, 1, 1, 1, 1, 1, 1, 2],
];

fn median3(a: i32, b: i32, c: i32) -> i32 {
    if a > b {
        if b > c {
            b
        } else if a > c {
            c
        } else {
            a
        }
    } else if a > c {
        a
    } else if b > c {
        c
    } else {
        b
    }
}

/// Convert a luma half-pel MV component to the matching chroma half-pel MV
/// component per H.263 Table 7-15 (baseline 1MV 4:2:0). Identical to the
/// mapping used by `oxideav_mpeg4video::mc::luma_mv_to_chroma`.
pub fn luma_to_chroma_mv(luma_half: i32) -> i32 {
    let int_part = luma_half >> 2;
    let half_bit = if luma_half & 3 != 0 { 1 } else { 0 };
    int_part * 2 + half_bit
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_mv_component_zero() {
        for pred in [-32, -16, -1, 0, 1, 16, 31] {
            let mut bw = BitWriter::new();
            encode_mv_component(&mut bw, pred, pred);
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            let got = decode_mv_component(&mut br, pred).unwrap();
            assert_eq!(got, pred, "round-trip pred={pred}");
        }
    }

    #[test]
    fn round_trip_mv_component_nonzero() {
        for mv in -32..=31 {
            for pred in [-32, -8, 0, 8, 31] {
                let mut bw = BitWriter::new();
                encode_mv_component(&mut bw, mv, pred);
                let bytes = bw.finish();
                let mut br = BitReader::new(&bytes);
                let got = decode_mv_component(&mut br, pred).unwrap();
                assert_eq!(got, mv, "round-trip mv={mv}, pred={pred}: got {got}");
            }
        }
    }

    #[test]
    fn chroma_mv_mapping_matches_spec() {
        // Sanity — same table as the mpeg4video helper.
        assert_eq!(luma_to_chroma_mv(0), 0);
        assert_eq!(luma_to_chroma_mv(1), 1);
        assert_eq!(luma_to_chroma_mv(2), 1);
        assert_eq!(luma_to_chroma_mv(3), 1);
        assert_eq!(luma_to_chroma_mv(4), 2);
        assert_eq!(luma_to_chroma_mv(-1), -1);
        assert_eq!(luma_to_chroma_mv(-3), -1);
        assert_eq!(luma_to_chroma_mv(-4), -2);
    }

    #[test]
    fn predict_mv_edges() {
        // All-zero grid: any position predicts (0,0).
        let grid = MvGrid::new(4, 4);
        for (x, y) in [(0, 0), (3, 0), (0, 3), (3, 3)] {
            assert_eq!(predict_mv(&grid, x, y), (0, 0));
        }
    }

    #[test]
    fn predict_mv_median() {
        let mut grid = MvGrid::new(3, 3);
        // Place known MVs at neighbours of (1,1): left=(4,0), top=(6,0), top-right=(8,0).
        grid.set(0, 1, MbMotion::mv1((4, 0), true, false));
        grid.set(1, 0, MbMotion::mv1((6, 0), true, false));
        grid.set(2, 0, MbMotion::mv1((8, 0), true, false));
        // median(4, 6, 8) = 6.
        assert_eq!(predict_mv(&grid, 1, 1), (6, 0));
    }

    #[test]
    fn wrap_boundary() {
        assert_eq!(wrap_mv_component(31), 31);
        assert_eq!(wrap_mv_component(32), -32);
        assert_eq!(wrap_mv_component(-32), -32);
        assert_eq!(wrap_mv_component(-33), 31);
    }

    /// Annex D §D.2 — when the predictor is inside the baseline range
    /// `[-31, +32]` halfpel the UMV decode collapses to the plain "predictor
    /// plus signed magnitude" form (no wrap, no sign-of-predictor games).
    #[test]
    fn umv_inside_baseline_range_is_transparent() {
        // magnitude=8, positive sign, predictor=0 → diff=+8 → 8.
        for (pred, mag, sign_bit, want) in [
            (0i32, 8i32, 0u32, 8i32),
            (0, 8, 1, -8),
            (15, 10, 0, 25),
            (-20, 5, 1, -25),
        ] {
            let mut bw = BitWriter::new();
            // encode the magnitude via the regular encoder — same VLC table.
            encode_mv_component(&mut bw, want, pred);
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            let got = decode_mv_component_umv(&mut br, pred, true).unwrap();
            assert_eq!(got, want, "pred={pred} mag={mag} sign={sign_bit}");
        }
    }

    /// Annex D §D.2 — predictor out of `[-31, +32]` → reconstruction picks
    /// the `{d, d+64, d-64}` candidate that stays in `[-63, +63]` *and*
    /// matches the predictor's sign.
    #[test]
    fn umv_predictor_out_of_range_picks_sign_matching_candidate() {
        // Predictor = 40 (outside baseline range). We want to encode an MV of
        // 50 (halfpel); raw_diff = 10 → sum 50 (positive, in range, same sign
        // as predictor). Easy case: the first candidate already works.
        let pred: i32 = 40;
        let want: i32 = 50;
        let diff_raw: i32 = want - pred; // 10
        let mut bw = BitWriter::new();
        // Encode a positive differential of +10 via the raw MV VLC path.
        let mag = diff_raw.unsigned_abs() as usize;
        let (bits, code) = MV_ENC_VLC[mag];
        bw.write_bits(code, bits as u32);
        bw.write_bits(0, 1); // positive sign
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let got = decode_mv_component_umv(&mut br, pred, true).unwrap();
        assert_eq!(got, want);
    }

    /// Annex D Table D.3 — value 0 is a single `1` bit, no sign.
    #[test]
    fn table_d3_zero_round_trip() {
        let mut bw = BitWriter::new();
        encode_mvd_table_d3(&mut bw, 0);
        let bytes = bw.finish();
        // Padded to one byte: `1000 0000` = 0x80.
        assert_eq!(bytes, vec![0x80]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_mvd_table_d3(&mut br).unwrap(), 0);
    }

    /// Annex D Table D.3 — spec example: the motion vector difference -13
    /// is encoded as `0 11 01 11 10` = `0 1 1 0 1 1 1 1 0` (9 bits).
    #[test]
    fn table_d3_spec_example_minus_13() {
        let mut bw = BitWriter::new();
        encode_mvd_table_d3(&mut bw, -13);
        let bytes = bw.finish();
        // 9-bit code `011011110` padded to 16 bits: `01101111 00000000`.
        assert_eq!(bytes, vec![0b01101111, 0x00]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_mvd_table_d3(&mut br).unwrap(), -13);
    }

    /// Annex D Table D.3 — round-trip the full `±2047` range. That's the
    /// 11-x-bit bucket (code length 25 bits) which covers more than the
    /// QCIF UUI=1 range — good enough for conformance.
    #[test]
    fn table_d3_full_range_round_trip() {
        for v in [
            -2047, -1024, -512, -100, -2, -1, 0, 1, 2, 100, 511, 1023, 2047,
        ] {
            let mut bw = BitWriter::new();
            encode_mvd_table_d3(&mut bw, v);
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            assert_eq!(decode_mvd_table_d3(&mut br).unwrap(), v, "round-trip v={v}");
        }
    }

    /// Annex D Table D.3 — consecutive round-trip at byte boundary (exercise
    /// bit-reader's multi-code accumulation).
    #[test]
    fn table_d3_sequential_codes_round_trip() {
        let seq = [0i32, 1, -1, 5, -13, 42, -100, 2047];
        let mut bw = BitWriter::new();
        for v in &seq {
            encode_mvd_table_d3(&mut bw, *v);
        }
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        for v in &seq {
            assert_eq!(decode_mvd_table_d3(&mut br).unwrap(), *v);
        }
    }

    /// MVD-pair SCE stuffing bit (§D.2 end): when both components are
    /// `+1` halfpel, the encoder emits six zero bits (`000 000`) and
    /// the decoder must consume a `1` bit afterward.
    #[test]
    fn mvd_pair_sce_stuff_present_for_plus_one_pair() {
        // Emit two `+1` codes + a stuffing `1` + a `0` marker.
        let mut bw = BitWriter::new();
        encode_mvd_table_d3(&mut bw, 1);
        encode_mvd_table_d3(&mut bw, 1);
        bw.write_bits(1, 1); // SCE stuffing bit.
        bw.write_bits(0, 8); // filler.
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let dx = decode_mvd_table_d3(&mut br).unwrap();
        let dy = decode_mvd_table_d3(&mut br).unwrap();
        assert_eq!(dx, 1);
        assert_eq!(dy, 1);
        consume_mvd_pair_sce_bit(&mut br, dx, dy).unwrap();
    }

    /// PLUSPTYPE MV-component helper: decodes via Table D.3 and adds to
    /// predictor; enforces UUI="1" limits when provided.
    #[test]
    fn plusptype_umv_component_uui1_limits_enforced() {
        // QCIF (176) → UUI=1 range [-64, 63] halfpel.
        let limit = Some(uui_limit_range_halfpel(176));
        assert_eq!(limit, Some((-64, 63)));
        // encode diff = +30 with predictor = +10 → 40, in range.
        let mut bw = BitWriter::new();
        encode_mvd_table_d3(&mut bw, 30);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let mv = decode_mv_component_plusptype_umv(&mut br, 10, limit).unwrap();
        assert_eq!(mv, 40);

        // encode diff = +60, predictor = +10 → 70, out of range for QCIF.
        let mut bw = BitWriter::new();
        encode_mvd_table_d3(&mut bw, 60);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let err = decode_mv_component_plusptype_umv(&mut br, 10, limit).unwrap_err();
        assert!(
            format!("{err}").contains("out of range"),
            "expected range diagnostic, got {err}"
        );
    }

    /// UUI=01 (unlimited): the component decoder should accept any
    /// `predictor + diff` value without a range check.
    #[test]
    fn plusptype_umv_component_uui01_unlimited() {
        let mut bw = BitWriter::new();
        encode_mvd_table_d3(&mut bw, 500);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let mv = decode_mv_component_plusptype_umv(&mut br, 0, None).unwrap();
        assert_eq!(mv, 500);
    }

    /// Annex D §D.1 — reconstruct_umv_component picks the wrap that keeps
    /// the result inside `[-63, 63]` when the straightforward sum would
    /// overflow the extended range.
    #[test]
    fn umv_wraps_to_extended_range_when_out_of_bounds() {
        // pred = 40 halfpel, diff = +30 → 70 overflows 63; candidate d-64 =
        // -34 yields 6 (positive, matches predictor's sign, in range).
        // Actually: {30, 94, -34} → viable in range & positive: just -34 → 6.
        let got = reconstruct_umv_component(40, 30, 1);
        assert_eq!(got, 6);

        // pred = -40, diff = -30 → -70 overflows; candidate +34 → -6 (neg,
        // matches predictor sign).
        let got = reconstruct_umv_component(-40, 30, -1);
        assert_eq!(got, -6);
    }

    /// Round 12 — `encode_mv_component_umv` round-trips through
    /// `decode_mv_component_umv` for every reachable (pred, mv) pair across
    /// the extended `[-63, +63]` halfpel range. The set of reachable MVs
    /// from a given predictor is defined by §D.2: the encoder picks one of
    /// `{d, d+64, d-64}` with `|d| ≤ 32`, decodes to `pred + d`.
    #[test]
    fn umv_encoder_round_trips_extended_range() {
        for pred in [-63, -40, -32, -1, 0, 1, 16, 32, 40, 63] {
            // Build the set of MVs we can actually encode. Then make sure
            // every one decodes back to the same value.
            let mut reachable: Vec<i32> = Vec::new();
            for mag in 0..=32i32 {
                for sign in [1i32, -1] {
                    if mag == 0 && sign == -1 {
                        continue;
                    }
                    let v = reconstruct_umv_component(pred, mag, sign);
                    if (MV_RANGE_UMV_MIN_HALF..=MV_RANGE_UMV_MAX_HALF).contains(&v) {
                        reachable.push(v);
                    }
                }
            }
            reachable.sort();
            reachable.dedup();
            for &mv in &reachable {
                let mut bw = BitWriter::new();
                encode_mv_component_umv(&mut bw, mv, pred);
                let bytes = bw.finish();
                let mut br = BitReader::new(&bytes);
                let got = decode_mv_component_umv(&mut br, pred, true).unwrap();
                assert_eq!(got, mv, "pred={pred}, mv={mv}: got {got}");
            }
        }
    }

    /// Round 12 — when the predictor is in the §D.2 baseline range
    /// `[-31, +32]` halfpel AND the raw differential `mv - pred` already
    /// fits in `[-32, +32]` (so the baseline encoder doesn't need to fold),
    /// the UMV encoder produces the exact same bytes as the baseline
    /// encoder. In other regimes the two encoders may legitimately differ
    /// (baseline relies on the folded `wrap()` interpretation, which UMV
    /// decoders don't apply).
    #[test]
    fn umv_encoder_matches_baseline_inside_baseline_range() {
        for pred in [-31, -8, 0, 8, 31, 32] {
            for mv in -32..=31i32 {
                let raw = mv - pred;
                // Baseline encoder folds raw to `[-32, +31]`; the UMV
                // encoder can't take advantage of that fold (UMV decode
                // would interpret the wrap differently). Skip the boundary
                // case where folding would diverge.
                if !(-32..=31).contains(&raw) {
                    continue;
                }
                let mut a = BitWriter::new();
                encode_mv_component(&mut a, mv, pred);
                let mut b = BitWriter::new();
                encode_mv_component_umv(&mut b, mv, pred);
                assert_eq!(
                    a.finish(),
                    b.finish(),
                    "UMV encoder must match baseline for pred={pred}, mv={mv}"
                );
            }
        }
    }

    /// Round 12 — predictor at the extended-range boundary (`pred=40`):
    /// the encoder must select the wrap that lands in `[-63, +63]` AND
    /// matches the predictor's sign per §D.2.
    #[test]
    fn umv_encoder_handles_predictor_outside_baseline() {
        // pred = 40 (outside `[-31, +32]`). MV = 6 is reachable via mag=30,
        // sign_dir=-1 → raw_diff=-30; pred + (-30+64)=74 (out of range);
        // pred + (-30) = 10 — wait that's mv=10. Let me pick a concrete
        // (pred, mv) pair we know §D.2 makes reachable: pred=40, mv=6 is
        // produced by mag=30 sign_dir=-1 with the d-64 candidate (=-34),
        // pred+(-34)=6 (positive → same sign as 40, in range). Verified by
        // the existing `umv_wraps_to_extended_range_when_out_of_bounds`
        // test.
        let mut bw = BitWriter::new();
        encode_mv_component_umv(&mut bw, 6, 40);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let got = decode_mv_component_umv(&mut br, 40, true).unwrap();
        assert_eq!(got, 6);
    }
}
