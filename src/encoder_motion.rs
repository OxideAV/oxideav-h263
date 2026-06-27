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
    median3, motion_compensate_block, MotionVector, RefPlane, MV_HALF_MAX, MV_HALF_MIN,
    RCONTROL_DEFAULT,
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
}

impl MvGrid {
    /// A fresh grid for a picture `mb_cols × mb_rows` macroblocks.
    pub fn new(mb_cols: usize, mb_rows: usize) -> Self {
        MvGrid {
            entries: vec![MvGridEntry::ZERO; mb_cols * mb_rows],
            cols: mb_cols,
            rows: mb_rows,
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

        // Top-border test (no GOB headers -> only the picture top edge).
        let top_border = row == 0;

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
    let y_ref = RefPlane::new(&reference.y, reference.luma_width, reference.luma_height);
    let mb_x = mb_col * 16;
    let mb_y = mb_row * 16;

    let cost = |mv: MotionVector| -> u32 {
        let sad = macroblock_sad(source, &y_ref, mb_x, mb_y, mv);
        let mvbits = (mv.dx_half - predictor.dx_half).unsigned_abs()
            + (mv.dy_half - predictor.dy_half).unsigned_abs();
        sad + lambda * mvbits
    };

    let clamp = |mv: MotionVector| -> MotionVector {
        MotionVector {
            dx_half: mv.dx_half.clamp(MV_HALF_MIN, MV_HALF_MAX),
            dy_half: mv.dy_half.clamp(MV_HALF_MIN, MV_HALF_MAX),
        }
    };

    // Start from the predictor and the zero vector; pick the cheaper.
    let mut best = clamp(predictor);
    let mut best_cost = cost(best);
    let zero = MotionVector::new(0, 0);
    let zc = cost(zero);
    if zc < best_cost {
        best = zero;
        best_cost = zc;
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
}
