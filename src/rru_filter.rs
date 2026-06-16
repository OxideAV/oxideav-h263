//! Annex Q.7 — block boundary filter for the Reduced-Resolution Update
//! mode.
//!
//! ITU-T Recommendation H.263 (01/2005) §Q.7 defines a filter run along
//! the edges of the *16 × 16 reconstructed blocks* (not the 8 × 8 block
//! edges of the baseline Annex J filter) after the reduced-resolution
//! texture has been up-sampled (§Q.6) and added to the prediction
//! (§Q.2.2.3). Filtering happens on the complete reconstructed image
//! data before that data is stored in the picture store for future
//! prediction.
//!
//! There are two variants:
//!
//! * **§Q.7.1 — default filter.** A two-tap weighted average across the
//!   16 × 16 block boundary. If `A` and `B` are the two samples on a
//!   line straddling the edge (`A` in `block1`, `B` in the neighbouring
//!   `block2` to the right of or below `block1`), then
//!
//!   ```text
//!       A1 = (3 * A + B + 2) / 4
//!       B1 = (A + 3 * B + 2) / 4
//!   ```
//!
//!   with `/` denoting division by truncation (§Q.7.1). This is the same
//!   1-D interpolation kernel the §Q.6.2 boundary up-sampler uses, here
//!   applied to neighbouring reconstructed samples rather than reduced
//!   samples.
//!
//! * **§Q.7.2 — Deblocking Filter mode.** When Annex J is also active,
//!   the §J.3 four-tap deblocking filter is run on the 16 × 16 block
//!   boundary pixels instead of the §Q.7.1 filter, with the single
//!   modification that `STRENGTH = +∞`. With infinite strength
//!   `UpDownRamp(x, ∞) = x`, so `d1 = d = (A − 4B + 4C − D) / 8` and the
//!   §J.3 formula collapses to
//!
//!   ```text
//!       B1 = clip(B + d1)
//!       C1 = clip(C − d1)
//!       A1 = A − d2
//!       D1 = D + d2
//!       d1 = (A − 4B + 4C − D) / 8
//!       d2 = clipd1((A − D) / 4, d1 / 2)
//!   ```
//!
//!   exactly as the spec restates it in §Q.7.2. This reuses the
//!   [`crate::deblock`] J.3 primitives with the published
//!   [`STRENGTH_RRU_INFINITE`] sentinel.
//!
//! ## Filter-on condition and ordering
//!
//! For both variants a boundary edge is filtered when at least one of
//! the two adjoining macroblocks is coded
//! (`COD == 0 || MB-type == INTRA`) — identical wording to the §J.3
//! condition. The order of edges is "identical to the description
//! provided in J.3": every horizontal edge first, then every vertical
//! edge, so the vertical pass sees the already-modified samples.
//!
//! No filtering is performed across picture edges. The slice-edge
//! (Annex K) and ISD GOB-boundary (Annex R) skips are surfaced to the
//! caller through the per-edge condition closure, exactly as in
//! [`crate::deblock::deblock_plane`].
//!
//! ## Scope
//!
//! This module provides the §Q.7 *pixel-domain* filter as a pure
//! plane-level primitive (and per-edge helpers). The surrounding 32 × 32
//! RRU macroblock decode pipeline (§Q.2 / §Q.4 / §Q.5) is not wired here;
//! callers that reconstruct an RRU picture invoke [`rru_filter_plane`]
//! on each reconstructed plane before storing it as a reference.

use crate::deblock::{apply_edge_samples, Sample, STRENGTH_RRU_INFINITE};

/// Side length, in pixels, of an RRU reconstructed block. The §Q.7
/// boundary filter runs along the edges of these 16 × 16 blocks (the
/// up-sampled luminance / chrominance blocks of §Q.6), in contrast to
/// the 8 × 8 blocks of the baseline §J.3 deblocking filter.
pub const RRU_BLOCK_DIM: usize = 16;

/// §Q.7.1 default two-tap boundary kernel. Given the sample `near`
/// (weight 3) and its neighbour `far` (weight 1) across the boundary,
/// returns `(3 * near + far + 2) / 4` with truncating division (§Q.7.1).
///
/// The `+ 2` rounding offset makes the division round-to-nearest for
/// non-negative numerators; samples are in `[0, 255]` so the numerator
/// is always non-negative and the result stays in `[0, 255]`.
#[inline]
pub fn rru_default_tap(near: Sample, far: Sample) -> Sample {
    ((3 * near as i32 + far as i32 + 2) / 4) as Sample
}

/// Which §Q.7 filter variant to run on a boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RruFilterMode {
    /// §Q.7.1 default block boundary filter (two-tap average). Used when
    /// the Deblocking Filter mode (Annex J) is **not** active alongside
    /// Reduced-Resolution Update mode.
    Default,
    /// §Q.7.2 filter: the §J.3 four-tap deblocking filter with
    /// `STRENGTH = +∞`, used when Annex J is active together with the
    /// Reduced-Resolution Update mode.
    Deblocking,
}

/// Per-edge decision the §Q.7 plane driver consults for each candidate
/// 16 × 16 block boundary. Mirrors [`crate::deblock::EdgeCondition`] but
/// without a strength field — §Q.7.1 has no strength and §Q.7.2 fixes
/// `STRENGTH = +∞`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RruEdgeCondition {
    /// At least one of the two adjoining macroblocks is coded
    /// (`COD == 0 || MB-type == INTRA`): the boundary is filtered.
    Filter,
    /// Neither adjoining block is coded, or the edge is a picture /
    /// slice / ISD-segment boundary §Q.7 forbids filtering across: the
    /// boundary is left untouched.
    Skip,
}

/// §Q.7.1 — filter one vertical 16-pixel boundary with the default
/// two-tap kernel. `block1_right_col` is the x of the rightmost column
/// of `block1`; `block2`'s leftmost column is `block1_right_col + 1`.
/// The edge spans the 16 rows `edge_top_row .. edge_top_row + 16`.
fn default_filter_vertical_edge(
    plane: &mut [Sample],
    stride: usize,
    block1_right_col: usize,
    edge_top_row: usize,
) {
    let b2_col = block1_right_col + 1;
    for r in 0..RRU_BLOCK_DIM {
        let row_base = (edge_top_row + r) * stride;
        let a = plane[row_base + block1_right_col]; // block1 (near for A1)
        let b = plane[row_base + b2_col]; // block2 (near for B1)
        plane[row_base + block1_right_col] = rru_default_tap(a, b); // A1
        plane[row_base + b2_col] = rru_default_tap(b, a); // B1
    }
}

/// §Q.7.1 — filter one horizontal 16-pixel boundary with the default
/// two-tap kernel. The boundary sits between row `block1_bottom_row`
/// (in `block1`) and `block1_bottom_row + 1` (in `block2`).
fn default_filter_horizontal_edge(
    plane: &mut [Sample],
    stride: usize,
    block1_bottom_row: usize,
    edge_left_col: usize,
) {
    let b1_base = block1_bottom_row * stride;
    let b2_base = (block1_bottom_row + 1) * stride;
    for c in 0..RRU_BLOCK_DIM {
        let x = edge_left_col + c;
        let a = plane[b1_base + x]; // block1 (near for A1)
        let b = plane[b2_base + x]; // block2 (near for B1)
        plane[b1_base + x] = rru_default_tap(a, b); // A1
        plane[b2_base + x] = rru_default_tap(b, a); // B1
    }
}

/// §Q.7.2 — filter one vertical 16-pixel boundary with the §J.3
/// four-tap filter at `STRENGTH = +∞`. The four samples per line are
/// `A = block1_right_col-1`, `B = block1_right_col`,
/// `C = block1_right_col+1` (= block2 leftmost), `D = block1_right_col+2`.
fn deblock_filter_vertical_edge(
    plane: &mut [Sample],
    stride: usize,
    block1_right_col: usize,
    edge_top_row: usize,
) {
    for r in 0..RRU_BLOCK_DIM {
        let row_base = (edge_top_row + r) * stride;
        let mut line = [
            plane[row_base + block1_right_col - 1],
            plane[row_base + block1_right_col],
            plane[row_base + block1_right_col + 1],
            plane[row_base + block1_right_col + 2],
        ];
        apply_edge_samples(&mut line, STRENGTH_RRU_INFINITE);
        plane[row_base + block1_right_col - 1] = line[0];
        plane[row_base + block1_right_col] = line[1];
        plane[row_base + block1_right_col + 1] = line[2];
        plane[row_base + block1_right_col + 2] = line[3];
    }
}

/// §Q.7.2 — filter one horizontal 16-pixel boundary with the §J.3
/// four-tap filter at `STRENGTH = +∞`. Per column the four samples are
/// rows `block1_bottom_row-1 .. block1_bottom_row+2`.
fn deblock_filter_horizontal_edge(
    plane: &mut [Sample],
    stride: usize,
    block1_bottom_row: usize,
    edge_left_col: usize,
) {
    for c in 0..RRU_BLOCK_DIM {
        let x = edge_left_col + c;
        let mut line = [
            plane[(block1_bottom_row - 1) * stride + x],
            plane[block1_bottom_row * stride + x],
            plane[(block1_bottom_row + 1) * stride + x],
            plane[(block1_bottom_row + 2) * stride + x],
        ];
        apply_edge_samples(&mut line, STRENGTH_RRU_INFINITE);
        plane[(block1_bottom_row - 1) * stride + x] = line[0];
        plane[block1_bottom_row * stride + x] = line[1];
        plane[(block1_bottom_row + 1) * stride + x] = line[2];
        plane[(block1_bottom_row + 2) * stride + x] = line[3];
    }
}

/// §Q.7 plane-level block boundary filter for the Reduced-Resolution
/// Update mode.
///
/// Filters the 16 × 16 reconstructed-block edges of one picture plane
/// (luma or chroma) per the §Q.7 ordering inherited from §J.3: every
/// horizontal edge first, then every vertical edge. `mode` selects the
/// §Q.7.1 default two-tap filter or the §Q.7.2 §J.3-with-infinite-
/// strength filter.
///
/// Picture-edge skip is built in (the rightmost / bottom block row /
/// column has no neighbour, so its outer edge is never filtered). The
/// slice / ISD-segment skips are the caller's, surfaced through
/// `condition_for_edge`, which is invoked once per candidate edge with
/// the two `(block_col, block_row)` 16 × 16-block coordinates (block1
/// then block2, block2 to the right of or below block1) and returns an
/// [`RruEdgeCondition`].
///
/// `width` and `height` are in pixels and must be multiples of 16
/// (RRU reconstructs in 32 × 32 macroblocks, so plane dimensions are
/// always 16 × 16-block aligned). `stride` is the row stride in pixels.
///
/// # Panics
///
/// Panics if `width` or `height` is not a multiple of 16, or if the
/// `plane` buffer is shorter than `stride * height`.
pub fn rru_filter_plane<F>(
    plane: &mut [Sample],
    width: usize,
    height: usize,
    stride: usize,
    mode: RruFilterMode,
    mut condition_for_edge: F,
) where
    F: FnMut((usize, usize), (usize, usize)) -> RruEdgeCondition,
{
    assert!(
        width % RRU_BLOCK_DIM == 0,
        "rru_filter_plane: width must be a multiple of 16, got {}",
        width
    );
    assert!(
        height % RRU_BLOCK_DIM == 0,
        "rru_filter_plane: height must be a multiple of 16, got {}",
        height
    );
    assert!(
        plane.len() >= stride * height,
        "rru_filter_plane: plane buffer too small ({}) for stride*height = {}",
        plane.len(),
        stride * height
    );

    let blocks_w = width / RRU_BLOCK_DIM;
    let blocks_h = height / RRU_BLOCK_DIM;

    // §Q.7 / §J.3 ordering: all horizontal edges first.
    for b_row in 0..blocks_h.saturating_sub(1) {
        let block1_bottom_row = (b_row + 1) * RRU_BLOCK_DIM - 1;
        for b_col in 0..blocks_w {
            if condition_for_edge((b_col, b_row), (b_col, b_row + 1)) == RruEdgeCondition::Filter {
                let edge_left_col = b_col * RRU_BLOCK_DIM;
                match mode {
                    RruFilterMode::Default => default_filter_horizontal_edge(
                        plane,
                        stride,
                        block1_bottom_row,
                        edge_left_col,
                    ),
                    RruFilterMode::Deblocking => deblock_filter_horizontal_edge(
                        plane,
                        stride,
                        block1_bottom_row,
                        edge_left_col,
                    ),
                }
            }
        }
    }

    // Then every vertical edge, on the already-modified samples.
    for b_col in 0..blocks_w.saturating_sub(1) {
        let block1_right_col = (b_col + 1) * RRU_BLOCK_DIM - 1;
        for b_row in 0..blocks_h {
            if condition_for_edge((b_col, b_row), (b_col + 1, b_row)) == RruEdgeCondition::Filter {
                let edge_top_row = b_row * RRU_BLOCK_DIM;
                match mode {
                    RruFilterMode::Default => {
                        default_filter_vertical_edge(plane, stride, block1_right_col, edge_top_row)
                    }
                    RruFilterMode::Deblocking => {
                        deblock_filter_vertical_edge(plane, stride, block1_right_col, edge_top_row)
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Always-filter condition for tests over fully-coded planes.
    fn always(_: (usize, usize), _: (usize, usize)) -> RruEdgeCondition {
        RruEdgeCondition::Filter
    }

    /// Never-filter condition (every edge skipped).
    fn never(_: (usize, usize), _: (usize, usize)) -> RruEdgeCondition {
        RruEdgeCondition::Skip
    }

    /// §Q.7.1 two-tap kernel: worked numerator checks including the
    /// truncating division and the `+ 2` rounding offset.
    #[test]
    fn default_tap_matches_spec_formula() {
        // A=100, B=120: A1=(300+120+2)/4=422/4=105; B1=(100+360+2)/4=462/4=115.
        assert_eq!(rru_default_tap(100, 120), 105);
        assert_eq!(rru_default_tap(120, 100), 115);
        // Equal samples are unchanged: (3k+k+2)/4 = (4k+2)/4 = k for the
        // sample range (the +2 < 4 offset never carries).
        for k in [0u8, 1, 17, 128, 255] {
            assert_eq!(rru_default_tap(k, k), k, "equal-sample identity at {k}");
        }
    }

    /// §Q.7.1: a flat plane is a fixed point of the default filter — no
    /// boundary moves a constant value.
    #[test]
    fn default_filter_constant_plane_unchanged() {
        let (w, h) = (32usize, 32usize);
        let mut plane = vec![73u8; w * h];
        let before = plane.clone();
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Default, always);
        assert_eq!(plane, before, "constant plane must be unchanged");
    }

    /// §Q.7.1: a single vertical step between two 16×16 blocks is
    /// smoothed only on the two columns straddling the boundary, by the
    /// two-tap kernel; all other columns are untouched.
    #[test]
    fn default_filter_smooths_only_boundary_columns() {
        let (w, h) = (32usize, 16usize);
        // Left block all 40, right block all 80.
        let mut plane = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                plane[y * w + x] = if x < 16 { 40 } else { 80 };
            }
        }
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Default, always);
        for y in 0..h {
            // Column 15 (block1 last) and 16 (block2 first) filtered.
            assert_eq!(plane[y * w + 15], rru_default_tap(40, 80)); // (120+80+2)/4=50
            assert_eq!(plane[y * w + 16], rru_default_tap(80, 40)); // (240+40+2)/4=70
                                                                    // Interior columns away from the boundary are untouched.
            assert_eq!(plane[y * w + 14], 40);
            assert_eq!(plane[y * w + 17], 80);
            assert_eq!(plane[y * w], 40);
            assert_eq!(plane[y * w + 31], 80);
        }
    }

    /// §Q.7.1: a single horizontal step between vertically adjacent
    /// 16×16 blocks is smoothed only on the two rows straddling the
    /// boundary.
    #[test]
    fn default_filter_smooths_only_boundary_rows() {
        let (w, h) = (16usize, 32usize);
        let mut plane = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                plane[y * w + x] = if y < 16 { 30 } else { 90 };
            }
        }
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Default, always);
        for x in 0..w {
            assert_eq!(plane[15 * w + x], rru_default_tap(30, 90)); // (90+90+2)/4=45
            assert_eq!(plane[16 * w + x], rru_default_tap(90, 30)); // (270+30+2)/4=75
            assert_eq!(plane[14 * w + x], 30);
            assert_eq!(plane[17 * w + x], 90);
        }
    }

    /// Skip condition suppresses all filtering: a stepped plane stays
    /// exactly as it was.
    #[test]
    fn skip_condition_leaves_plane_untouched() {
        let (w, h) = (32usize, 16usize);
        let mut plane = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                plane[y * w + x] = if x < 16 { 40 } else { 80 };
            }
        }
        let before = plane.clone();
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Default, never);
        assert_eq!(plane, before);
    }

    /// Picture-edge skip: with a single 16×16 block there is no
    /// neighbour, so no edge is ever a candidate and the plane is
    /// unchanged regardless of the condition.
    #[test]
    fn single_block_plane_has_no_internal_edges() {
        let (w, h) = (16usize, 16usize);
        let mut plane: Vec<u8> = (0..(w * h) as u32).map(|v| (v % 256) as u8).collect();
        let before = plane.clone();
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Default, always);
        assert_eq!(plane, before, "no internal 16×16 edge in a 16×16 plane");
    }

    /// §Q.7.2: with `STRENGTH = +∞` the §J.3 filter has `d1 = d`. A flat
    /// plane has `d = 0` everywhere, so it is a fixed point.
    #[test]
    fn deblock_mode_constant_plane_unchanged() {
        let (w, h) = (32usize, 32usize);
        let mut plane = vec![128u8; w * h];
        let before = plane.clone();
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Deblocking, always);
        assert_eq!(plane, before);
    }

    /// §Q.7.2 vertical-edge worked example. A=B=block1=100, C=D=block2
    /// =120 across the boundary (samples at cols 14,15 | 16,17 of a
    /// 32-wide plane). With STRENGTH=∞:
    ///   d  = (A−4B+4C−D)/8 = (100−400+480−120)/8 = 60/8 = 7
    ///   d2 = clipd1((A−D)/4, d1/2) = clipd1((100−120)/4, 3)
    ///      = clipd1(-5, 3) = -3
    ///   B1 = clip(100 + 7) = 107; C1 = clip(120 − 7) = 113
    ///   A1 = 100 − (−3) = 103; D1 = 120 + (−3) = 117
    #[test]
    fn deblock_mode_vertical_edge_worked_example() {
        let (w, h) = (32usize, 16usize);
        let mut plane = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                plane[y * w + x] = if x < 16 { 100 } else { 120 };
            }
        }
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Deblocking, always);
        for y in 0..h {
            assert_eq!(plane[y * w + 14], 103, "A1"); // A
            assert_eq!(plane[y * w + 15], 107, "B1"); // B
            assert_eq!(plane[y * w + 16], 113, "C1"); // C
            assert_eq!(plane[y * w + 17], 117, "D1"); // D
                                                      // Beyond the four-pixel filter window: untouched.
            assert_eq!(plane[y * w + 13], 100);
            assert_eq!(plane[y * w + 18], 120);
        }
    }

    /// Ordering guard (§Q.7 / §J.3): horizontal edges are filtered
    /// before vertical edges. We assert the driver completes a 2×2 block
    /// grid without panicking and that a corner pixel shared by a
    /// horizontal and a vertical boundary reflects both passes (it is
    /// modified, not left at either single-pass value). This pins the
    /// two-pass structure.
    #[test]
    fn default_filter_runs_both_passes_on_2x2_grid() {
        let (w, h) = (32usize, 32usize);
        let mut plane = vec![0u8; w * h];
        // Four distinct quadrants so every internal boundary has a step.
        for y in 0..h {
            for x in 0..w {
                let q = (if y < 16 { 0 } else { 2 }) + (if x < 16 { 0 } else { 1 });
                plane[y * w + x] = [20u8, 60, 100, 200][q];
            }
        }
        let before = plane.clone();
        rru_filter_plane(&mut plane, w, h, w, RruFilterMode::Default, always);
        // The pixel at (row 15, col 15) is block1's bottom-right; it is
        // touched by the horizontal pass (as A on the horizontal edge)
        // and then by the vertical pass (as A on the vertical edge), so
        // it must differ from the original and from a single-pass result.
        assert_ne!(plane[15 * w + 15], before[15 * w + 15]);
        // Centre interior of a block (row 7, col 7) is far from any
        // 16×16 boundary and stays at its quadrant value.
        assert_eq!(plane[7 * w + 7], 20);
    }
}
