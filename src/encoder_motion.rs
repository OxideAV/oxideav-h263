//! H.263 **motion estimation** for the baseline encoder.
//!
//! Searches for a per-macroblock motion vector that minimises the
//! prediction error against the reference frame, and replicates the
//! decoder's §6.1.1 median-predictor bookkeeping so the emitted
//! `MVD = MV − predictor` reconstructs to exactly the chosen `MV`.
//!
//! ## Predictor replay
//!
//! The decoder derives each macroblock's MV predictor from the median
//! of three neighbour candidates (Figure 12). For the baseline
//! single-video-picture-segment stream the encoder emits (no GOB headers
//! after GOB 0, all macroblocks in segment 0), the border rules reduce
//! to:
//!
//! * MV1 = left neighbour's MV (zero at the left picture edge),
//! * MV2 = above neighbour's MV (= MV1 at the top picture edge),
//! * MV3 = above-right neighbour's MV (zero at the right picture edge,
//!   = MV1 at the top edge),
//!
//! with INTRA / not-coded neighbours contributing a zero candidate.
//! [`MvGrid`] tracks the running MV state so [`MvGrid::predict`] matches
//! the decoder's `predict_mv` exactly for this stream shape.
//!
//! ## Search
//!
//! [`estimate_motion`] runs an integer-then-half-pel diamond/full search
//! over a bounded window and returns the MV (half-pel units) whose
//! motion-compensated prediction has the lowest sum of absolute
//! differences against the source macroblock, with a small bias toward
//! the predictor (so static regions keep MVD = 0 and stay cheap). The
//! motion compensation reuses the decoder's
//! [`crate::motion::motion_compensate_block`], so the prediction the
//! search scores is bit-identical to what the decoder will reconstruct.

use crate::motion::{
    median3, motion_compensate_block, reconstruct_mv_component_umv, MotionVector, RefPlane,
    MV_HALF_MAX, MV_HALF_MIN, MV_UMV_HALF_MAX, MV_UMV_HALF_MIN, RCONTROL_DEFAULT,
};
use crate::picture::YuvFrame;

/// One entry in the encoder's running MV grid: the macroblock's
/// reconstructed luma MV (half-pel) plus whether it was INTRA / skipped
/// (both contribute a zero predictor candidate).
#[derive(Debug, Clone, Copy)]
struct MvGridEntry {
    mv: MotionVector,
    zero_candidate: bool,
}

impl MvGridEntry {
    const ZERO: MvGridEntry = MvGridEntry {
        mv: MotionVector {
            dx_half: 0,
            dy_half: 0,
        },
        zero_candidate: true,
    };
}

/// The encoder's running motion-vector grid, mirroring the decoder's
/// `MbGridEntry` bookkeeping for the baseline single-segment stream so
/// the median predictor can be replayed during encode.
#[derive(Debug, Clone)]
pub struct MvGrid {
    entries: Vec<MvGridEntry>,
    cols: usize,
    rows: usize,
    /// When the encoder emits a GOB header for **every** GOB after the
    /// first (the §5.2 non-empty-header stream shape), each GOB is its
    /// own §6.1.1 video picture segment: the top macroblock row of
    /// every GOB gets the rule-3 "MV2 = MV3 = MV1" border treatment.
    /// `Some(k)` records the §4.2.1 `mb_rows_per_gob`; `None` keeps the
    /// single-segment behaviour (no headers after GOB 0).
    rows_per_gob: Option<usize>,
}

impl MvGrid {
    /// A fresh grid for a picture `mb_cols × mb_rows` macroblocks
    /// encoded as a **single segment** (no GOB headers after GOB 0).
    pub fn new(mb_cols: usize, mb_rows: usize) -> Self {
        MvGrid {
            entries: vec![MvGridEntry::ZERO; mb_cols * mb_rows],
            cols: mb_cols,
            rows: mb_rows,
            rows_per_gob: None,
        }
    }

    /// A fresh grid for a picture whose encoder emits a GOB header for
    /// every GOB after the first, each GOB covering `mb_rows_per_gob`
    /// macroblock rows (§4.2.1: 1 for sub-QCIF..CIF, 2 for 4CIF, 4 for
    /// 16CIF). Matches the decoder's `predict_mv` when
    /// `gob_header_present` holds for every non-zero GOB.
    pub fn with_gob_headers(mb_cols: usize, mb_rows: usize, mb_rows_per_gob: usize) -> Self {
        MvGrid {
            entries: vec![MvGridEntry::ZERO; mb_cols * mb_rows],
            cols: mb_cols,
            rows: mb_rows,
            rows_per_gob: Some(mb_rows_per_gob.max(1)),
        }
    }

    fn get(&self, col: isize, row: isize) -> Option<MvGridEntry> {
        if col < 0 || row < 0 || col as usize >= self.cols || row as usize >= self.rows {
            None
        } else {
            Some(self.entries[row as usize * self.cols + col as usize])
        }
    }

    /// The §6.1.1 median predictor for the macroblock at `(col, row)`,
    /// matching the decoder's `predict_mv` for a baseline single-segment
    /// picture (no GOB headers, every MB in segment 0).
    pub fn predict(&self, col: usize, row: usize) -> MotionVector {
        let cand = |e: Option<MvGridEntry>| -> MotionVector {
            match e {
                Some(entry) if !entry.zero_candidate => entry.mv,
                _ => MotionVector::new(0, 0),
            }
        };

        // MV1 — left neighbour; zero at the left edge.
        let mv1 = if col == 0 {
            MotionVector::new(0, 0)
        } else {
            cand(self.get(col as isize - 1, row as isize))
        };

        // Top-border test: the picture top edge, plus — when every GOB
        // carries a header — the top macroblock row of each GOB (the
        // §6.1.1 rule-3 "outside the GOB at the top" case the decoder
        // applies whenever `gob_header_present` held for that GOB).
        let top_border = row == 0 || self.rows_per_gob.is_some_and(|k| row % k == 0);

        // MV2 — above neighbour; = MV1 at the top border.
        let mv2 = if top_border {
            mv1
        } else {
            cand(self.get(col as isize, row as isize - 1))
        };

        // MV3 — above-right neighbour; zero at the right edge, = MV1 at
        // the top border.
        let mv3 = if col + 1 >= self.cols {
            MotionVector::new(0, 0)
        } else if top_border {
            mv1
        } else {
            cand(self.get(col as isize + 1, row as isize - 1))
        };

        MotionVector {
            dx_half: median3(mv1.dx_half, mv2.dx_half, mv3.dx_half),
            dy_half: median3(mv1.dy_half, mv2.dy_half, mv3.dy_half),
        }
    }

    /// Record a macroblock's reconstructed MV (an INTER vector).
    pub fn set_inter(&mut self, col: usize, row: usize, mv: MotionVector) {
        self.entries[row * self.cols + col] = MvGridEntry {
            mv,
            zero_candidate: false,
        };
    }

    /// Record an INTRA or skipped macroblock (zero predictor candidate).
    pub fn set_zero_candidate(&mut self, col: usize, row: usize) {
        self.entries[row * self.cols + col] = MvGridEntry::ZERO;
    }
}

/// Annex D §D.2 — the Table-14 MVD component the encoder must emit so
/// the decoder's [`reconstruct_mv_component_umv`] recovers `mv` from
/// `predictor`, or `None` when `mv` is **not reachable** from that
/// predictor in the Unrestricted Motion Vector mode (PLUSPTYPE absent).
///
/// §D.2 restricts the reachable set per component (half-pel units):
///
/// * predictor `Pc ∈ [-31, 32]` — only `MVc ∈ [Pc − 32, Pc + 31]`
///   ("only values that are within a range of `[-16, 15.5]` around the
///   predictor ... can be reached if the predictor is in the range
///   `[-15.5, 16]`"),
/// * `Pc ∈ [-63, -32]` — `MVc ∈ [-63, 0]`,
/// * `Pc ∈ [33, 63]` — `MVc ∈ [0, 63]`.
///
/// The emitted difference is a plain Table-14 codeword in `[-32, 31]`;
/// the decoder's §D.2 pair-selection maps it back to `mv`. The inverse
/// is found by scanning the 64 possible Table-14 values and checking
/// them against the decoder's own reconstruction, so the round-trip is
/// exact by construction (`reconstruct_mv_component_umv(predictor, d)
/// == mv` for the returned `d`).
pub fn umv_mvd_component_for(mv: i32, predictor: i32) -> Option<i8> {
    if !(MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX).contains(&mv) {
        return None;
    }
    // Fast path: the direct difference, when Table-14-representable.
    let direct = mv - predictor;
    if (MV_HALF_MIN..=MV_HALF_MAX).contains(&direct)
        && reconstruct_mv_component_umv(predictor, direct) == mv
    {
        return Some(direct as i8);
    }
    // Otherwise the codeword denotes the pair {d, d ± 64}; try the
    // member of the pair that lands in the Table-14 window.
    let paired = if direct > MV_HALF_MAX {
        direct - crate::motion::MV_HALF_SPAN
    } else {
        direct + crate::motion::MV_HALF_SPAN
    };
    if (MV_HALF_MIN..=MV_HALF_MAX).contains(&paired)
        && reconstruct_mv_component_umv(predictor, paired) == mv
    {
        return Some(paired as i8);
    }
    None
}

/// Annex D §D.2 — the full [`crate::macroblock::Mvd`] for a motion
/// vector under the Unrestricted Motion Vector mode, or `None` when
/// either component is unreachable from its predictor (see
/// [`umv_mvd_component_for`]).
pub fn umv_mvd_for(mv: MotionVector, predictor: MotionVector) -> Option<crate::macroblock::Mvd> {
    let dx = umv_mvd_component_for(mv.dx_half, predictor.dx_half)?;
    let dy = umv_mvd_component_for(mv.dy_half, predictor.dy_half)?;
    Some(crate::macroblock::Mvd {
        dx_half: dx,
        dy_half: dy,
    })
}

/// Sum of absolute differences between a 16×16 macroblock of `source`
/// (luma) at pixel origin `(mb_x, mb_y)` and the motion-compensated
/// prediction from `reference` under motion vector `mv` (half-pel).
fn macroblock_sad(
    source: &YuvFrame,
    reference_plane: &RefPlane<'_>,
    mb_x: usize,
    mb_y: usize,
    mv: MotionVector,
) -> u32 {
    let lw = source.luma_width;
    let mut sad = 0u32;
    // Four 8×8 luma blocks of the 16×16 macroblock.
    for blk in 0..4 {
        let bx = mb_x + (blk % 2) * 8;
        let by = mb_y + (blk / 2) * 8;
        let pred = motion_compensate_block(reference_plane, bx, by, mv, RCONTROL_DEFAULT);
        for row in 0..8 {
            for col in 0..8 {
                let s = source.y[(by + row) * lw + (bx + col)] as i32;
                let p = pred[row * 8 + col] as i32;
                sad += (s - p).unsigned_abs();
            }
        }
    }
    sad
}

/// Estimate the best motion vector (half-pel) for the macroblock at grid
/// position `(mb_col, mb_row)` of `source` against `reference`, biased
/// toward `predictor` so static content keeps `MVD = 0`.
///
/// `search_half` bounds the search window: integer MVs are searched in
/// `±search_half` whole pixels around the predictor's integer part,
/// then a half-pel refinement checks the eight half-pel neighbours of
/// the integer optimum. The returned MV is clamped to the baseline
/// `[-32, 31]` half-pel component range so the resulting `MVD` is
/// representable in Table 14.
///
/// `lambda` weights the predictor bias: the score is `SAD + lambda ×
/// (|dx − pdx| + |dy − pdy|)`, so a vector is only chosen over the
/// predictor when it reduces SAD by more than its MVD cost.
pub fn estimate_motion(
    source: &YuvFrame,
    reference: &YuvFrame,
    mb_col: usize,
    mb_row: usize,
    predictor: MotionVector,
    search_half: i32,
    lambda: u32,
) -> MotionVector {
    let clamp = |mv: MotionVector| -> MotionVector {
        MotionVector {
            dx_half: mv.dx_half.clamp(MV_HALF_MIN, MV_HALF_MAX),
            dy_half: mv.dy_half.clamp(MV_HALF_MIN, MV_HALF_MAX),
        }
    };
    // In the default prediction mode every MV in [-32, 31] half-pel is
    // reachable from any predictor (the §6.1.1 wrap), so the candidate
    // filter is a plain range clamp.
    search_best(
        source,
        reference,
        mb_col,
        mb_row,
        predictor,
        search_half,
        lambda,
        &clamp,
        &|_| true,
    )
}

/// Annex D §D.2 variant of [`estimate_motion`] for the **Unrestricted
/// Motion Vector mode** (PLUSPTYPE absent).
///
/// The candidate window is the extended `[-63, 63]` half-pel range, but
/// a candidate is admitted only when **both** components are reachable
/// from `predictor` under the §D.2 rules (see
/// [`umv_mvd_component_for`]) — so the returned MV is always exactly
/// codable with a Table-14 MVD pair. With a zero predictor the
/// reachable window matches the default mode; as neighbouring
/// macroblocks accumulate large vectors the predictor grows and the
/// window slides out to ±31.5 pixels, which is how §D.2 reaches the
/// extended range in practice.
pub fn estimate_motion_umv(
    source: &YuvFrame,
    reference: &YuvFrame,
    mb_col: usize,
    mb_row: usize,
    predictor: MotionVector,
    search_half: i32,
    lambda: u32,
) -> MotionVector {
    let clamp = |mv: MotionVector| -> MotionVector {
        MotionVector {
            dx_half: mv.dx_half.clamp(MV_UMV_HALF_MIN, MV_UMV_HALF_MAX),
            dy_half: mv.dy_half.clamp(MV_UMV_HALF_MIN, MV_UMV_HALF_MAX),
        }
    };
    let representable = |mv: MotionVector| -> bool {
        umv_mvd_component_for(mv.dx_half, predictor.dx_half).is_some()
            && umv_mvd_component_for(mv.dy_half, predictor.dy_half).is_some()
    };
    search_best(
        source,
        reference,
        mb_col,
        mb_row,
        predictor,
        search_half,
        lambda,
        &clamp,
        &representable,
    )
}

/// Shared integer-then-half-pel SAD search. `clamp` folds a raw
/// candidate into the mode's component range; `admit` rejects
/// candidates whose MVD is not representable from `predictor` (always
/// true in the default mode, the §D.2 reachability test in UMV mode).
#[allow(clippy::too_many_arguments)]
fn search_best(
    source: &YuvFrame,
    reference: &YuvFrame,
    mb_col: usize,
    mb_row: usize,
    predictor: MotionVector,
    search_half: i32,
    lambda: u32,
    clamp: &dyn Fn(MotionVector) -> MotionVector,
    admit: &dyn Fn(MotionVector) -> bool,
) -> MotionVector {
    let y_ref = RefPlane::new(&reference.y, reference.luma_width, reference.luma_height);
    let mb_x = mb_col * 16;
    let mb_y = mb_row * 16;

    let cost = |mv: MotionVector| -> u32 {
        let sad = macroblock_sad(source, &y_ref, mb_x, mb_y, mv);
        let mvbits = (mv.dx_half - predictor.dx_half).unsigned_abs()
            + (mv.dy_half - predictor.dy_half).unsigned_abs();
        sad + lambda * mvbits
    };

    // Start from the predictor (always reachable: MVD = 0) and the zero
    // vector; pick the cheaper.
    let mut best = clamp(predictor);
    let mut best_cost = cost(best);
    let zero = MotionVector::new(0, 0);
    if admit(zero) {
        let zc = cost(zero);
        if zc < best_cost {
            best = zero;
            best_cost = zc;
        }
    }

    // Integer-pel full search around the integer part of the predictor.
    let pdx_int = predictor.dx_half / 2;
    let pdy_int = predictor.dy_half / 2;
    for dy in -search_half..=search_half {
        for dx in -search_half..=search_half {
            let mv = clamp(MotionVector {
                dx_half: (pdx_int + dx) * 2,
                dy_half: (pdy_int + dy) * 2,
            });
            if !admit(mv) {
                continue;
            }
            let c = cost(mv);
            if c < best_cost {
                best_cost = c;
                best = mv;
            }
        }
    }

    // Half-pel refinement around the integer optimum.
    for dy in -1..=1 {
        for dx in -1..=1 {
            if dx == 0 && dy == 0 {
                continue;
            }
            let mv = clamp(MotionVector {
                dx_half: best.dx_half + dx,
                dy_half: best.dy_half + dy,
            });
            if !admit(mv) {
                continue;
            }
            let c = cost(mv);
            if c < best_cost {
                best_cost = c;
                best = mv;
            }
        }
    }

    best
}

/// Compute the `MVD` (half-pel) the encoder must emit so the decoder
/// reconstructs `mv` from `predictor` under the baseline §6.1.1 wrap.
///
/// The decoder computes `component = wrap(predictor + MVD)` into
/// `[-32, 31]`. To recover `mv` we set `MVD = mv − predictor` reduced
/// into `[-32, 31]` (the same window), which inverts the wrap because
/// the search clamps `mv` into `[-32, 31]` so `mv − predictor` is within
/// `[-63, 63]` and a single wrap suffices.
pub fn mvd_for(mv: MotionVector, predictor: MotionVector) -> crate::macroblock::Mvd {
    let reduce = |delta: i32| -> i8 {
        let mut v = delta;
        while v < MV_HALF_MIN {
            v += crate::motion::MV_HALF_SPAN;
        }
        while v > MV_HALF_MAX {
            v -= crate::motion::MV_HALF_SPAN;
        }
        v as i8
    };
    crate::macroblock::Mvd {
        dx_half: reduce(mv.dx_half - predictor.dx_half),
        dy_half: reduce(mv.dy_half - predictor.dy_half),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::reconstruct_mv;

    fn gradient_frame(lw: usize, lh: usize, shift: i32) -> YuvFrame {
        let cw = lw / 2;
        let ch = lh / 2;
        let mut y = vec![0u8; lw * lh];
        for row in 0..lh {
            for col in 0..lw {
                let c = (col as i32 + shift).rem_euclid(lw as i32) as usize;
                y[row * lw + col] = (40 + (c + row) % 160) as u8;
            }
        }
        YuvFrame {
            y,
            cb: vec![128u8; cw * ch],
            cr: vec![128u8; cw * ch],
            luma_width: lw,
            luma_height: lh,
        }
    }

    /// `mvd_for` inverts the decoder's wrap: reconstruct_mv(predictor,
    /// mvd_for(mv, predictor)) == mv for clamped vectors.
    #[test]
    fn mvd_round_trips_through_reconstruct() {
        for pdx in [-20, -4, 0, 6, 30] {
            for pdy in [-30, -1, 0, 12, 28] {
                let predictor = MotionVector::new(pdx, pdy);
                for mvx in [MV_HALF_MIN, -10, 0, 5, MV_HALF_MAX] {
                    for mvy in [MV_HALF_MIN, -3, 0, 9, MV_HALF_MAX] {
                        let mv = MotionVector::new(mvx, mvy);
                        let mvd = mvd_for(mv, predictor);
                        let recon = reconstruct_mv(predictor, mvd);
                        assert_eq!(recon, mv, "predictor={:?} mv={:?}", predictor, mv);
                    }
                }
            }
        }
    }

    /// A static frame (reference == source) estimates a zero MV.
    #[test]
    fn static_frame_estimates_zero_mv() {
        let frame = gradient_frame(176, 144, 0);
        let mv = estimate_motion(&frame, &frame, 3, 3, MotionVector::new(0, 0), 4, 4);
        assert_eq!(mv, MotionVector::new(0, 0));
    }

    /// A horizontally-shifted frame estimates a non-zero horizontal MV
    /// that reduces SAD relative to the zero vector.
    #[test]
    fn shifted_frame_estimates_nonzero_mv() {
        let reference = gradient_frame(176, 144, 0);
        let source = gradient_frame(176, 144, 2); // shifted right by 2 px
        let y_ref = RefPlane::new(&reference.y, reference.luma_width, reference.luma_height);
        let mb_col = 4;
        let mb_row = 4;
        let mb_x = mb_col * 16;
        let mb_y = mb_row * 16;
        let zero_sad = macroblock_sad(&source, &y_ref, mb_x, mb_y, MotionVector::new(0, 0));
        let mv = estimate_motion(
            &source,
            &reference,
            mb_col,
            mb_row,
            MotionVector::new(0, 0),
            4,
            1,
        );
        let mv_sad = macroblock_sad(&source, &y_ref, mb_x, mb_y, mv);
        assert!(mv_sad <= zero_sad, "estimated MV did not reduce SAD");
        // `source` samples the pattern at `col + 2`, so the matching
        // content sits at `col + 2` in the (unshifted) reference — the
        // best vector points right by 2 px (+4 half-pel).
        assert_eq!(mv.dx_half, 4, "expected dx ≈ +4 half-pel, got {:?}", mv);
    }

    // ---- Annex D §D.2 UMV MVD inverse -----------------------------

    /// For every predictor, every §D.2-reachable component round-trips
    /// through the decoder's reconstruction, and every unreachable one
    /// is refused.
    #[test]
    fn umv_mvd_component_inverts_reconstruction_exhaustively() {
        use crate::motion::{reconstruct_mv_component_umv, MV_UMV_HALF_MAX, MV_UMV_HALF_MIN};
        for pred in MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX {
            // §D.2 reachable set for this predictor.
            let reachable = |mv: i32| -> bool {
                if (-31..=32).contains(&pred) {
                    (pred - 32..=pred + 31).contains(&mv)
                } else if pred > 32 {
                    (0..=MV_UMV_HALF_MAX).contains(&mv)
                } else {
                    (MV_UMV_HALF_MIN..=0).contains(&mv)
                }
            };
            for mv in MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX {
                match umv_mvd_component_for(mv, pred) {
                    Some(d) => {
                        assert!(
                            reachable(mv),
                            "pred={pred} mv={mv}: not reachable but coded"
                        );
                        assert_eq!(
                            reconstruct_mv_component_umv(pred, d as i32),
                            mv,
                            "pred={pred} mv={mv} d={d}: decoder disagrees"
                        );
                    }
                    None => {
                        assert!(!reachable(mv), "pred={pred} mv={mv}: reachable but refused");
                    }
                }
            }
        }
    }

    /// The full-vector inverse composes the per-component rule; a
    /// mixed reachable/unreachable pair is refused as a whole.
    #[test]
    fn umv_mvd_full_vector() {
        use crate::motion::reconstruct_mv_umv;
        let pred = MotionVector::new(40, -2);
        let mv = MotionVector::new(56, 20);
        let mvd = umv_mvd_for(mv, pred).expect("reachable pair");
        assert_eq!(reconstruct_mv_umv(pred, mvd), mv);
        // dy = 40 is outside [-34, 29] around pred.dy = -2 → refused.
        assert!(umv_mvd_for(MotionVector::new(56, 40), pred).is_none());
    }

    /// With a large predictor the UMV estimator reaches a vector beyond
    /// the default ±31-half window (the §D.2 extension), while the
    /// default-mode estimator clamps to [-32, 31].
    #[test]
    fn umv_estimator_reaches_extended_range() {
        // Reference: gradient. Source: the same pattern shifted right
        // by 20 px, so the matching content sits 20 px right in the
        // reference (best MV = +40 half-pel).
        let reference = gradient_frame(176, 144, 0);
        let source = gradient_frame(176, 144, 20);
        let predictor = MotionVector::new(40, 0); // grown via neighbours
        let mv = estimate_motion_umv(&source, &reference, 4, 4, predictor, 4, 1);
        assert_eq!(mv.dx_half, 40, "UMV search missed the +40 optimum: {mv:?}");
        assert_eq!(mv.dy_half, 0);
        // The default-mode estimator cannot represent +40: it stays
        // within the baseline window.
        let base = estimate_motion(&source, &reference, 4, 4, predictor, 4, 1);
        assert!(
            base.dx_half <= MV_HALF_MAX,
            "default mode exceeded its range: {base:?}"
        );
    }

    /// Every vector the UMV estimator returns is exactly codable: the
    /// MVD exists and reconstructs to the returned vector.
    #[test]
    fn umv_estimator_returns_codable_vectors() {
        use crate::motion::reconstruct_mv_umv;
        let reference = gradient_frame(176, 144, 0);
        let source = gradient_frame(176, 144, 7);
        for (pdx, pdy) in [(0, 0), (31, 0), (-40, 6), (50, -50)] {
            let predictor = MotionVector::new(pdx, pdy);
            let mv = estimate_motion_umv(&source, &reference, 2, 3, predictor, 3, 2);
            let mvd = umv_mvd_for(mv, predictor)
                .unwrap_or_else(|| panic!("uncodable MV {mv:?} from predictor {predictor:?}"));
            assert_eq!(reconstruct_mv_umv(predictor, mvd), mv);
        }
    }

    /// The predictor grid matches the decoder's reduced border rules:
    /// top-left MB predicts zero; a left neighbour propagates.
    #[test]
    fn predictor_grid_border_rules() {
        let mut grid = MvGrid::new(11, 9);
        // Top-left MB: all neighbours outside -> zero.
        assert_eq!(grid.predict(0, 0), MotionVector::new(0, 0));
        // Set (0,0) MV and check (1,0) sees it via MV1 = left, and at
        // the top row MV2 = MV3 = MV1, so the median is MV1.
        grid.set_inter(0, 0, MotionVector::new(6, -4));
        assert_eq!(grid.predict(1, 0), MotionVector::new(6, -4));
    }

    /// With a header on every GOB, each GOB top row is a §6.1.1 rule-3
    /// border: MV2/MV3 collapse onto MV1 and the above row's vectors do
    /// not leak across the boundary.
    #[test]
    fn predictor_grid_gob_segments() {
        // k = 1 (QCIF-like): every row is a GOB top.
        let mut grid = MvGrid::with_gob_headers(11, 9, 1);
        grid.set_inter(4, 0, MotionVector::new(10, 10));
        // (4,1): MV1 = left (unset -> zero); above (4,0) must NOT
        // contribute (different GOB): MV2 = MV3 = MV1 = 0.
        assert_eq!(grid.predict(4, 1), MotionVector::new(0, 0));
        // Left neighbour still propagates within the row.
        grid.set_inter(3, 1, MotionVector::new(-8, 2));
        assert_eq!(grid.predict(4, 1), MotionVector::new(-8, 2));

        // k = 2 (4CIF-like): row 1 is inside GOB 0, row 2 opens GOB 1.
        let mut grid2 = MvGrid::with_gob_headers(11, 8, 2);
        grid2.set_inter(4, 0, MotionVector::new(10, 10));
        grid2.set_inter(5, 0, MotionVector::new(10, 10));
        // (4,1): above row is the same GOB -> MV2/MV3 contribute, and
        // the median of (0, 10, 10) is 10.
        assert_eq!(grid2.predict(4, 1), MotionVector::new(10, 10));
        // (4,2): GOB boundary -> above row must not leak.
        grid2.set_inter(4, 1, MotionVector::new(10, 10));
        grid2.set_inter(5, 1, MotionVector::new(10, 10));
        assert_eq!(grid2.predict(4, 2), MotionVector::new(0, 0));
    }
}
