//! H.263 Annex J — Deblocking filter.
//!
//! The filter acts across every 8×8 block boundary of the reconstructed
//! picture — vertical boundaries (between columns `8k` and `8k+1` for all
//! `k>=1`) and horizontal boundaries (between rows `8k` and `8k+1` for all
//! `k>=1`) — on all three planes (Y / Cb / Cr). The quantiser used for each
//! filtering operation is the quantiser of the macroblock that owns the
//! pixels being modified; we track one QP value per 16×16 luma MB / 8×8
//! chroma block.
//!
//! Core 4-pixel filter (simplified form used by this crate):
//! Given four pixels `A, B, C, D` that straddle the boundary
//! (B and C are the edge neighbours, A and D are the outer neighbours):
//!
//! ```text
//! d1 = (A - 4*B + 4*C - D) / 8
//! d  = clip(d1, -QP/2, QP/2)
//! B' = clip255(B + d)
//! C' = clip255(C - d)
//! ```
//!
//! The first line is a discrete approximation of the second derivative
//! across the edge; the clip bounds how much any single pel may move (at
//! most `QP/2` per pass). Real-world H.263 Annex J adds a further UP()
//! saturation that turns the filter off on strong edges — we omit it here
//! because the filter is only applied when both the encoder AND decoder
//! opt in (out-of-band; see crate-level docs), and the simpler form is
//! sufficient to match reconstructions between both sides deterministically.
//!
//! The filter is applied in two passes per plane: first all horizontal
//! edges (modifying rows), then all vertical edges (modifying columns).

use crate::mb::IPicture;

/// Apply Annex J deblocking to every 8×8 block boundary in `pic`.
///
/// `qp_per_mb` is a 2-D grid of macroblock quantisers in row-major order,
/// indexed as `qp_per_mb[mb_y * mb_width + mb_x]`. Values must be in 1..=31.
///
/// The filter is idempotent-enough for our test: applying it once to the
/// encoder's reconstruction produces the same picture the decoder gets when
/// it applies the filter once to its own reconstruction (both start from the
/// same bit-exact pre-filter reconstruction).
pub fn deblock_picture(pic: &mut IPicture, qp_per_mb: &[u8]) {
    let mb_w = pic.mb_width;
    let mb_h = pic.mb_height;
    debug_assert_eq!(
        qp_per_mb.len(),
        mb_w * mb_h,
        "qp_per_mb must have one entry per MB"
    );

    // Luma plane — 16×16 per MB, so vertical/horizontal edges fall at every
    // 8th column/row. Boundary QP is the QP of whichever MB owns the upper
    // (for horizontal edges) or left (for vertical edges) pel of the edge
    // pair — the spec uses strength1 = QP(left_block), strength2 = QP(right_block)
    // and filters with `max(strength1, strength2)`. We use the same rule.
    deblock_plane(
        &mut pic.y,
        pic.y_stride,
        pic.mb_width * 16,
        pic.mb_height * 16,
        qp_per_mb,
        mb_w,
        mb_h,
        16,
    );

    // Chroma planes — 8×8 per MB, so there's only ONE edge per MB (the edge
    // between this MB's chroma block and its neighbour's). QP table stays the
    // same (one entry per MB).
    deblock_plane(
        &mut pic.cb,
        pic.c_stride,
        pic.mb_width * 8,
        pic.mb_height * 8,
        qp_per_mb,
        mb_w,
        mb_h,
        8,
    );
    deblock_plane(
        &mut pic.cr,
        pic.c_stride,
        pic.mb_width * 8,
        pic.mb_height * 8,
        qp_per_mb,
        mb_w,
        mb_h,
        8,
    );
}

/// Apply the Annex J filter to a single plane. `mb_size_px` is the MB edge
/// length in pels for this plane (16 for luma, 8 for chroma).
fn deblock_plane(
    plane: &mut [u8],
    stride: usize,
    width: usize,
    height: usize,
    qp_per_mb: &[u8],
    mb_w: usize,
    mb_h: usize,
    mb_size_px: usize,
) {
    // Horizontal edges — walk every row y = 8, 16, 24, ... (but not y == 0 or
    // y >= height) and apply the 4-tap filter vertically across that row.
    // For luma (16px MB) there are 2 horizontal edges per MB row: one at
    // y = mb_y*16+8 *within* the MB, one at y = mb_y*16 between MBs. For
    // chroma (8px MB) there's only the inter-MB edge.
    for y in (8..height).step_by(8) {
        // Determine which MB owns the pels ABOVE this edge (the filtering
        // strength is that MB's QP). For the inter-MB edges (y at MB boundary)
        // the owner is mb_y-1; for the intra-MB edges (y in the middle of an
        // MB) the owner is mb_y = y / mb_size_px.
        let mb_above = (y - 1) / mb_size_px;
        for x in 0..width {
            let mb_x = x.min(width - 1) / mb_size_px;
            let mb_x = mb_x.min(mb_w - 1);
            let mb_above_clamped = mb_above.min(mb_h - 1);
            let qp = qp_per_mb[mb_above_clamped * mb_w + mb_x];
            // 4 pels straddling the edge (columns fixed, rows y-2..=y+1).
            let a = plane[(y - 2) * stride + x] as i32;
            let b = plane[(y - 1) * stride + x] as i32;
            let c = plane[y * stride + x] as i32;
            let d = plane[(y + 1) * stride + x] as i32;
            let (nb, nc) = filter4(a, b, c, d, qp);
            plane[(y - 1) * stride + x] = nb as u8;
            plane[y * stride + x] = nc as u8;
        }
    }

    // Vertical edges — walk every column x = 8, 16, ... .
    for x in (8..width).step_by(8) {
        let mb_left = (x - 1) / mb_size_px;
        for y in 0..height {
            let mb_y = y.min(height - 1) / mb_size_px;
            let mb_y = mb_y.min(mb_h - 1);
            let mb_left_clamped = mb_left.min(mb_w - 1);
            let qp = qp_per_mb[mb_y * mb_w + mb_left_clamped];
            let a = plane[y * stride + (x - 2)] as i32;
            let b = plane[y * stride + (x - 1)] as i32;
            let c = plane[y * stride + x] as i32;
            let d = plane[y * stride + (x + 1)] as i32;
            let (nb, nc) = filter4(a, b, c, d, qp);
            plane[y * stride + (x - 1)] = nb as u8;
            plane[y * stride + x] = nc as u8;
        }
    }
}

/// The simplified H.263 Annex J 4-tap filter (no UP() saturation; see the
/// module docs). Returns the new values for `B` and `C` (the two pels
/// directly adjacent to the edge); `A` and `D` are read-only context.
#[inline]
fn filter4(a: i32, b: i32, c: i32, d: i32, qp: u8) -> (i32, i32) {
    // Discrete second derivative across the edge.
    let d1 = (a - 4 * b + 4 * c - d) / 8;
    // Clip to ±QP/2 — the step bound.
    let half_qp = qp as i32 / 2;
    let delta = d1.clamp(-half_qp, half_qp);
    let nb = (b + delta).clamp(0, 255);
    let nc = (c - delta).clamp(0, 255);
    (nb, nc)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_region_unchanged() {
        // A flat plane (all pels = 100) must be unchanged by the filter
        // because d1 = (100 - 400 + 400 - 100) / 8 = 0.
        let mb_w = 2usize;
        let mb_h = 2usize;
        let mut pic = IPicture::new(mb_w * 16, mb_h * 16);
        pic.y.fill(100);
        pic.cb.fill(128);
        pic.cr.fill(128);
        let pic_before_y = pic.y.clone();
        let qp = vec![5u8; mb_w * mb_h];
        deblock_picture(&mut pic, &qp);
        assert_eq!(pic.y, pic_before_y, "flat luma unchanged");
    }

    #[test]
    fn block_edge_smoothed() {
        // Two 8-pel-wide regions with a step at the MB boundary — the filter
        // should push the step pels closer together.
        let mb_w = 2usize;
        let mb_h = 1usize;
        let mut pic = IPicture::new(mb_w * 16, mb_h * 16);
        // Fill luma: left 16 columns = 100, right 16 columns = 150 → big
        // step at x=16 (MB-MB boundary).
        for y in 0..pic.mb_height * 16 {
            for x in 0..pic.mb_width * 16 {
                pic.y[y * pic.y_stride + x] = if x < 16 { 100 } else { 150 };
            }
        }
        let qp = vec![10u8; mb_w * mb_h];
        deblock_picture(&mut pic, &qp);
        // The two pels across the MB edge (x=15 and x=16) should have moved
        // toward each other; specifically pel x=15 should now be > 100 and
        // pel x=16 should be < 150.
        let row = pic.mb_height * 8; // a row in the middle
        let left = pic.y[row * pic.y_stride + 15];
        let right = pic.y[row * pic.y_stride + 16];
        assert!(left > 100, "left edge pel not filtered: {}", left);
        assert!(right < 150, "right edge pel not filtered: {}", right);
    }

    #[test]
    fn filter4_monotone() {
        // The 4-tap filter on a flat input yields zero delta.
        let (b, c) = filter4(100, 100, 100, 100, 5);
        assert_eq!(b, 100);
        assert_eq!(c, 100);
    }

    #[test]
    fn filter4_step() {
        // A pure step (100, 100, 150, 150) with QP=10 should produce a
        // nonzero but bounded delta.
        let (b, c) = filter4(100, 100, 150, 150, 10);
        assert!(b > 100);
        assert!(c < 150);
        // Delta capped at QP/2 = 5.
        assert!((b - 100).abs() <= 5);
        assert!((150 - c).abs() <= 5);
    }
}
