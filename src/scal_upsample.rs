//! Annex O §O.6 — spatial-scalability reference-picture up-sampling.
//!
//! Before a reference-layer picture is used to predict an EI- or
//! EP-picture in a *spatial* enhancement layer (§O.1.3), it is
//! interpolated by a factor of two horizontally, vertically, or both.
//! This module implements the three §O.6 cases as pure plane / frame
//! transforms:
//!
//! * **2-D** (Figures O.8 / O.9) — both dimensions doubled.
//! * **1-D horizontal** (Figures O.10 / O.11) — width doubled.
//! * **1-D vertical** — height doubled (the horizontal filter applied
//!   along columns, "analogous" per §O.6).
//!
//! ## Sample geometry
//!
//! The reduced-resolution input samples sit at the **centres** of the
//! output 2×2 cells, exactly as for the Annex Q §Q.6 prediction-error
//! up-sample (the spec notes the §O.6 filter "is the same technique used
//! in Annex Q and in some cases in Annex P"). Writing the `W × H` input
//! plane as `s[i][j]` and the up-sampled output as `o[y][x]`:
//!
//! * **2-D interior (Figure O.8).** Four adjacent input samples
//!   `A = s[i][j]`, `B = s[i][j+1]`, `C = s[i+1][j]`, `D = s[i+1][j+1]`
//!   bound a 2×2 output square at `o[2i+1][2j+1]`:
//!
//!   ```text
//!     a = o[2i+1][2j+1] = (9A + 3B + 3C +  D + 8) / 16
//!     b = o[2i+1][2j+2] = (3A + 9B +  C + 3D + 8) / 16
//!     c = o[2i+2][2j+1] = (3A +  B + 9C + 3D + 8) / 16
//!     d = o[2i+2][2j+2] = ( A + 3B + 3C + 9D + 8) / 16
//!   ```
//!
//! * **2-D boundary (Figure O.9).** The outermost output ring has no
//!   input sample beyond the picture edge, so it copies / 1-D
//!   interpolates from the in-picture samples: corners copy the nearest
//!   sample (`a = A`); along an edge between adjacent samples `A`, `B`
//!   the two output pixels are `(3A + B + 2) / 4` (nearest `A`) and
//!   `(A + 3B + 2) / 4` (nearest `B`).
//!
//! * **1-D (Figures O.10 / O.11).** Between two adjacent samples `A`,
//!   `B` the two output pixels are `(3A + B + 2) / 4` and
//!   `(A + 3B + 2) / 4`; the output pixel beyond the last sample at a
//!   picture boundary copies that sample (`a = A`, Figure O.11).
//!
//! `/` is truncating division throughout §O.6; the `+ 8` / `+ 2`
//! rounding offsets make it round-to-nearest for the non-negative
//! sample values (`[0, 255]`), so plain `i32` `/` is exact.

/// §O.6 interior 2-D weight (Figure O.8). `s = [A, B, C, D]`, `w` the
/// per-sample weights (summing to 16); `+ 8` then truncating `/ 16`.
#[inline]
fn interior(s: [i32; 4], w: [i32; 4]) -> i32 {
    (w[0] * s[0] + w[1] * s[1] + w[2] * s[2] + w[3] * s[3] + 8) / 16
}

/// §O.6 1-D weight (Figures O.9–O.11). The pixel nearest `near` carries
/// weight 3; `+ 2` then truncating `/ 4`.
#[inline]
fn one_d(near: i32, far: i32) -> i32 {
    (3 * near + far + 2) / 4
}

/// Clamp an index to the last valid row/column (picture-edge
/// replication for the rightmost / bottom output cell, which has no
/// `j+1` / `i+1` neighbour).
#[inline]
fn clamp_idx(i: usize, max: usize) -> usize {
    if i + 1 < max {
        i + 1
    } else {
        i
    }
}

/// §O.6 / Figures O.8–O.9 — up-sample a `width × height` 8-bit plane by
/// a factor of two in **both** dimensions, returning a
/// `(2·width) × (2·height)` plane.
///
/// `plane` is row-major (`plane[i * width + j]`). Panics if
/// `plane.len() != width * height` or either dimension is zero.
pub fn upsample_plane_2d(plane: &[u8], width: usize, height: usize) -> Vec<u8> {
    assert_eq!(plane.len(), width * height, "plane length must be W*H");
    assert!(width > 0 && height > 0, "plane dimensions must be non-zero");

    let ow = width * 2;
    let oh = height * 2;
    let mut out = vec![0u8; ow * oh];
    let s = |i: usize, j: usize| -> i32 { plane[i * width + j] as i32 };
    let mut put = |y: usize, x: usize, v: i32| {
        out[y * ow + x] = v.clamp(0, 255) as u8;
    };

    // Interior: each input cell (i, j) fills the 2×2 output square whose
    // top-left is (2i+1, 2j+1). The right/bottom edges reuse the last
    // input sample (edge replication) so the whole output is covered.
    for i in 0..height {
        for j in 0..width {
            let a = s(i, j);
            let b = s(i, clamp_idx(j, width));
            let c = s(clamp_idx(i, height), j);
            let d = s(clamp_idx(i, height), clamp_idx(j, width));
            let q = [a, b, c, d];
            let oy = 2 * i + 1;
            let ox = 2 * j + 1;
            put(oy, ox, interior(q, [9, 3, 3, 1]));
            if ox + 1 < ow {
                put(oy, ox + 1, interior(q, [3, 9, 1, 3]));
            }
            if oy + 1 < oh {
                put(oy + 1, ox, interior(q, [3, 1, 9, 3]));
            }
            if oy + 1 < oh && ox + 1 < ow {
                put(oy + 1, ox + 1, interior(q, [1, 3, 3, 9]));
            }
        }
    }

    // Top / bottom edge rows (y = 0 and y = oh-1): 1-D interpolate along
    // x between adjacent input samples of the nearest input row; the
    // first/last output column copies the nearest sample (Figure O.9).
    for &(out_row, in_row) in &[(0usize, 0usize), (oh - 1, height - 1)] {
        put(out_row, 0, s(in_row, 0));
        put(out_row, ow - 1, s(in_row, width - 1));
        for j in 0..width.saturating_sub(1) {
            let a = s(in_row, j);
            let b = s(in_row, j + 1);
            put(out_row, 2 * j + 1, one_d(a, b));
            put(out_row, 2 * j + 2, one_d(b, a));
        }
    }

    // Left / right edge columns (x = 0 and x = ow-1): 1-D interpolate
    // along y between adjacent input samples of the nearest input
    // column; the first/last output row already copied above.
    for &(out_col, in_col) in &[(0usize, 0usize), (ow - 1, width - 1)] {
        for i in 0..height.saturating_sub(1) {
            let a = s(i, in_col);
            let b = s(i + 1, in_col);
            put(2 * i + 1, out_col, one_d(a, b));
            put(2 * i + 2, out_col, one_d(b, a));
        }
    }

    out
}

/// §O.6 / Figures O.10–O.11 — up-sample a plane by a factor of two
/// **horizontally** (width doubled, height unchanged).
pub fn upsample_plane_1d_horizontal(plane: &[u8], width: usize, height: usize) -> Vec<u8> {
    assert_eq!(plane.len(), width * height, "plane length must be W*H");
    assert!(width > 0 && height > 0, "plane dimensions must be non-zero");

    let ow = width * 2;
    let mut out = vec![0u8; ow * height];
    for i in 0..height {
        let row = &plane[i * width..i * width + width];
        let orow = &mut out[i * ow..i * ow + ow];
        // First output pixel copies the first sample (Figure O.11).
        orow[0] = row[0];
        orow[ow - 1] = row[width - 1];
        for j in 0..width - 1 {
            let a = row[j] as i32;
            let b = row[j + 1] as i32;
            orow[2 * j + 1] = one_d(a, b).clamp(0, 255) as u8;
            orow[2 * j + 2] = one_d(b, a).clamp(0, 255) as u8;
        }
    }
    out
}

/// §O.6 — up-sample a plane by a factor of two **vertically** (height
/// doubled, width unchanged). The horizontal filter applied along
/// columns ("analogous", §O.6).
pub fn upsample_plane_1d_vertical(plane: &[u8], width: usize, height: usize) -> Vec<u8> {
    assert_eq!(plane.len(), width * height, "plane length must be W*H");
    assert!(width > 0 && height > 0, "plane dimensions must be non-zero");

    let oh = height * 2;
    let mut out = vec![0u8; width * oh];
    let s = |i: usize, j: usize| -> i32 { plane[i * width + j] as i32 };
    for j in 0..width {
        out[j] = s(0, j) as u8;
        out[(oh - 1) * width + j] = s(height - 1, j) as u8;
        for i in 0..height - 1 {
            let a = s(i, j);
            let b = s(i + 1, j);
            out[(2 * i + 1) * width + j] = one_d(a, b).clamp(0, 255) as u8;
            out[(2 * i + 2) * width + j] = one_d(b, a).clamp(0, 255) as u8;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A uniform plane up-samples to a uniform plane of twice the size
    /// in both dimensions: every §O.6 weight set sums to its divisor, so
    /// a constant input is preserved exactly (interior, edges, corners).
    #[test]
    fn uniform_2d_is_preserved() {
        let w = 4;
        let h = 3;
        let plane = vec![137u8; w * h];
        let up = upsample_plane_2d(&plane, w, h);
        assert_eq!(up.len(), (2 * w) * (2 * h));
        assert!(up.iter().all(|&p| p == 137), "uniform 2-D upsample");
    }

    /// 2-D upsample doubles each dimension.
    #[test]
    fn upsample_2d_dimensions() {
        let up = upsample_plane_2d(&[0u8; 8 * 6], 8, 6);
        assert_eq!(up.len(), 16 * 12);
    }

    /// 1-D horizontal: a two-sample row [0, 100] produces, between the
    /// boundary copies, the §O.10 pair (3·0+100+2)/4 = 25 and
    /// (0+3·100+2)/4 = 75. The boundary pixels copy the ends.
    #[test]
    fn horizontal_1d_two_sample_interpolation() {
        let up = upsample_plane_1d_horizontal(&[0, 100], 2, 1);
        // ow = 4. o[0] = 0 (copy), o[1] = 25, o[2] = 75, o[3] = 100.
        assert_eq!(up, vec![0, 25, 75, 100]);
    }

    /// 1-D vertical mirrors the horizontal filter along columns.
    #[test]
    fn vertical_1d_two_sample_interpolation() {
        // A 1×2 column [0; 100].
        let up = upsample_plane_1d_vertical(&[0, 100], 1, 2);
        // oh = 4, width 1: rows 0=0, 1=25, 2=75, 3=100.
        assert_eq!(up, vec![0, 25, 75, 100]);
    }

    /// A horizontal ramp up-samples monotonically (no overshoot / clamp
    /// artefacts) and the interior interpolation lands between the
    /// bounding samples.
    #[test]
    fn horizontal_ramp_is_monotone() {
        let w = 4;
        let plane: Vec<u8> = (0..w).map(|j| (j * 60) as u8).collect();
        let up = upsample_plane_1d_horizontal(&plane, w, 1);
        for k in 0..up.len() - 1 {
            assert!(up[k] <= up[k + 1], "ramp must stay monotone at {k}");
        }
        assert_eq!(up[0], 0);
        assert_eq!(*up.last().unwrap(), 180);
    }
}
