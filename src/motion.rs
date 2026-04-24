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
//! either sign). For PLUSPTYPE-form Annex D, Table D.3 is used instead —
//! that path is not yet supported by this crate.
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

/// Per-MB motion-vector slot (luma half-pel units). One vector per MB in 1MV
/// mode — all four luma blocks share it. The value is also used for the
/// median predictor of subsequent MBs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MbMotion {
    pub mv: (i32, i32),
    /// True iff this MB was coded (intra or inter). A non-coded (skipped) MB
    /// contributes `(0, 0)` to future MV predictors per §5.3.4.
    pub coded: bool,
    /// True iff this MB is intra-in-P — in which case its MV is (0,0) and
    /// shouldn't propagate. We still keep it as a neighbour with MV (0,0).
    pub intra: bool,
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
    let mv2 = if mb_y > 0 {
        Some(get(mb_x, mb_y - 1))
    } else {
        None
    };
    let mv3 = if mb_y > 0 && mb_x + 1 < grid.mb_w {
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
        grid.set(
            0,
            1,
            MbMotion {
                mv: (4, 0),
                coded: true,
                intra: false,
            },
        );
        grid.set(
            1,
            0,
            MbMotion {
                mv: (6, 0),
                coded: true,
                intra: false,
            },
        );
        grid.set(
            2,
            0,
            MbMotion {
                mv: (8, 0),
                coded: true,
                intra: false,
            },
        );
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
}
