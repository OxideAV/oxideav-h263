//! Annex Q.6 — upsampling of the reduced-resolution reconstructed
//! prediction error (8×8 → 16×16).
//!
//! ITU-T Recommendation H.263 (01/2005) Annex Q describes the optional
//! Reduced-Resolution Update (RRU) mode. In this mode the DCT / texture
//! data describes 8×8 blocks of a *reduced-resolution* version of the
//! picture (§Q.1); to produce the final full-resolution image the
//! decoded 8×8 reduced-resolution reconstructed prediction-error block
//! is up-sampled to a 16×16 reconstructed prediction-error block before
//! it is added to the (already full-resolution) motion-compensated
//! prediction (§Q.2.2.2 / §Q.2.2.3).
//!
//! Per §Q.6 the up-sampling filter is *closed within a block*: "filtering
//! is closed within a block which enables to perform an individual
//! up-sampling on block basis". Only the pixels that belong to the
//! corresponding 8×8 block are used; no data from neighbouring blocks
//! crosses the 16×16 boundary. This module therefore takes a single
//! signed 8×8 prediction-error block and returns a signed 16×16 block.
//!
//! ## Sample geometry (Figure Q.7)
//!
//! The eight reduced-resolution samples along each axis sit at the
//! centres of the 2×2 output cells. Writing the reduced-resolution input
//! as `s[i][j]` (`i`, `j` in `0..8`) and the 16×16 output as `o[y][x]`
//! (`y`, `x` in `0..16`):
//!
//! * **Interior (§Q.6.1, Figure Q.8).** Four adjacent reduced samples
//!   `A = s[i][j]`, `B = s[i][j+1]`, `C = s[i+1][j]`, `D = s[i+1][j+1]`
//!   bound a 2×2 square of output pixels:
//!
//!   ```text
//!     a = o[2i+1][2j+1] = (9A + 3B + 3C +  D + 8) / 16
//!     b = o[2i+1][2j+2] = (3A + 9B +  C + 3D + 8) / 16
//!     c = o[2i+2][2j+1] = (3A +  B + 9C + 3D + 8) / 16
//!     d = o[2i+2][2j+2] = ( A + 3B + 3C + 9D + 8) / 16
//!   ```
//!
//!   For `i`, `j` in `0..7` these cover output rows `1..=14` and
//!   columns `1..=14`.
//!
//! * **Boundary (§Q.6.2, Figure Q.9).** The outermost ring (rows `0`
//!   and `15`, columns `0` and `15`) has no reduced sample beyond the
//!   block edge, so it copies / 1-D interpolates from the in-block
//!   samples:
//!     * corners copy the nearest reduced sample (`a = A`);
//!     * along an edge between adjacent reduced samples `A`, `B`:
//!       `(3·A + B + 2) / 4` for the pixel nearest `A` and
//!       `(A + 3·B + 2) / 4` for the pixel nearest `B`.
//!
//! `/` denotes division by truncation throughout §Q.6 (the figures say
//! so explicitly), and the `+ 8` / `+ 2` are the rounding offsets that
//! make the division round-to-nearest for non-negative numerators. The
//! prediction-error samples are signed (the §6.2.4 inverse transform
//! ranges over `[-256, +255]`); "division by truncation" means
//! truncation toward zero, which is exactly the behaviour of Rust's `/`
//! operator on `i32`, so all weights below divide with plain `/`.

/// Side length of the reduced-resolution input block.
pub const RRU_IN_DIM: usize = 8;
/// Side length of the up-sampled output block.
pub const RRU_OUT_DIM: usize = 16;

/// Number of samples in a reduced-resolution input block.
pub const RRU_IN_LEN: usize = RRU_IN_DIM * RRU_IN_DIM;
/// Number of samples in an up-sampled output block.
pub const RRU_OUT_LEN: usize = RRU_OUT_DIM * RRU_OUT_DIM;

/// §Q.6.1 interior up-sampling weight. `s = [A, B, C, D]` are the four
/// bounding reduced samples and `w = [wA, wB, wC, wD]` the per-output
/// weights (which always sum to 16); the `+ 8` is the rounding offset
/// and division by 16 truncates per §Q.6.
#[inline]
fn interior(s: [i32; 4], w: [i32; 4]) -> i32 {
    (w[0] * s[0] + w[1] * s[1] + w[2] * s[2] + w[3] * s[3] + 8) / 16
}

/// §Q.6.2 boundary 1-D up-sampling weight, with the `+ 2` rounding
/// offset and truncating division by 4. `near` carries weight 3.
#[inline]
fn boundary(near: i32, far: i32) -> i32 {
    (3 * near + far + 2) / 4
}

/// Annex Q.6 — up-sample an 8×8 reduced-resolution reconstructed
/// prediction-error block to a 16×16 reconstructed prediction-error
/// block.
///
/// Input and output are both row-major signed blocks
/// (`input[i * 8 + j]`, `output[y * 16 + x]`). The filter is closed
/// within the block (§Q.6): no neighbouring-block samples are consulted,
/// so this function is a pure spatial transform of one block.
pub fn upsample_prediction_error(input: &[i16; RRU_IN_LEN]) -> [i16; RRU_OUT_LEN] {
    // Read a reduced-resolution sample as i32 for the arithmetic.
    let s = |i: usize, j: usize| -> i32 { input[i * RRU_IN_DIM + j] as i32 };

    let mut out = [0i16; RRU_OUT_LEN];
    let mut put = |y: usize, x: usize, v: i32| {
        out[y * RRU_OUT_DIM + x] = v as i16;
    };

    // --- Interior (§Q.6.1, Figure Q.8) ---------------------------------
    // Output rows 1..=14, columns 1..=14, filled in 2×2 output cells
    // keyed on the four bounding reduced samples.
    for i in 0..RRU_IN_DIM - 1 {
        for j in 0..RRU_IN_DIM - 1 {
            let a = s(i, j);
            let b = s(i, j + 1);
            let c = s(i + 1, j);
            let d = s(i + 1, j + 1);
            let s = [a, b, c, d];
            let oy = 2 * i + 1;
            let ox = 2 * j + 1;
            // a (nearest A): weights 9,3,3,1
            put(oy, ox, interior(s, [9, 3, 3, 1]));
            // b (nearest B): weights 3,9,1,3
            put(oy, ox + 1, interior(s, [3, 9, 1, 3]));
            // c (nearest C): weights 3,1,9,3
            put(oy + 1, ox, interior(s, [3, 1, 9, 3]));
            // d (nearest D): weights 1,3,3,9
            put(oy + 1, ox + 1, interior(s, [1, 3, 3, 9]));
        }
    }

    // --- Boundary (§Q.6.2, Figure Q.9) ---------------------------------
    // Corners copy the nearest reduced sample (a = A).
    put(0, 0, s(0, 0));
    put(0, RRU_OUT_DIM - 1, s(0, RRU_IN_DIM - 1));
    put(RRU_OUT_DIM - 1, 0, s(RRU_IN_DIM - 1, 0));
    put(
        RRU_OUT_DIM - 1,
        RRU_OUT_DIM - 1,
        s(RRU_IN_DIM - 1, RRU_IN_DIM - 1),
    );

    // Top and bottom edges (rows 0 and 15): 1-D interpolate along the
    // column axis between adjacent reduced samples in the edge row.
    for j in 0..RRU_IN_DIM - 1 {
        let ox = 2 * j + 1;
        // Top row, samples s[0][j], s[0][j+1].
        let (a, b) = (s(0, j), s(0, j + 1));
        put(0, ox, boundary(a, b));
        put(0, ox + 1, boundary(b, a));
        // Bottom row, samples s[7][j], s[7][j+1].
        let (a, b) = (s(RRU_IN_DIM - 1, j), s(RRU_IN_DIM - 1, j + 1));
        put(RRU_OUT_DIM - 1, ox, boundary(a, b));
        put(RRU_OUT_DIM - 1, ox + 1, boundary(b, a));
    }

    // Left and right edges (cols 0 and 15): 1-D interpolate along the
    // row axis between adjacent reduced samples in the edge column.
    for i in 0..RRU_IN_DIM - 1 {
        let oy = 2 * i + 1;
        // Left column, samples s[i][0], s[i+1][0].
        let (a, c) = (s(i, 0), s(i + 1, 0));
        put(oy, 0, boundary(a, c));
        put(oy + 1, 0, boundary(c, a));
        // Right column, samples s[i][7], s[i+1][7].
        let (a, c) = (s(i, RRU_IN_DIM - 1), s(i + 1, RRU_IN_DIM - 1));
        put(oy, RRU_OUT_DIM - 1, boundary(a, c));
        put(oy + 1, RRU_OUT_DIM - 1, boundary(c, a));
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A flat *non-negative* constant block up-samples to that same
    /// constant everywhere. The interior weights sum to 16 and the
    /// boundary weights to 4, matching their divisors, so each weighted
    /// sum is `16k + 8` (interior) or `4k + 2` (boundary); for `k ≥ 0`
    /// the truncating division recovers `k` exactly because the `+ 8` /
    /// `+ 2` offsets are strictly below the divisor.
    #[test]
    fn nonnegative_constant_block_upsamples_to_constant() {
        for &k in &[0i16, 1, 7, 100, 255] {
            let input = [k; RRU_IN_LEN];
            let out = upsample_prediction_error(&input);
            assert!(
                out.iter().all(|&v| v == k),
                "constant {k} did not round-trip: {:?}",
                &out[..16]
            );
        }
    }

    /// For a *negative* constant the spec's division by truncation
    /// (toward zero, §Q.6) biases the result toward zero rather than
    /// preserving the constant: e.g. an interior pixel is
    /// `(16k + 8) / 16` truncated toward zero, which equals `k` only
    /// when `k ≥ 0`. This test pins the spec-exact behaviour for a few
    /// negative constants instead of (wrongly) demanding round-trip.
    #[test]
    fn negative_constant_block_truncates_toward_zero() {
        // k = -50: interior (16*-50 + 8)/16 = -792/16 = -49 (trunc);
        //          boundary (4*-50 + 2)/4 = -198/4 = -49.
        let input = [-50i16; RRU_IN_LEN];
        let out = upsample_prediction_error(&input);
        let at = |y: usize, x: usize| out[y * RRU_OUT_DIM + x];
        // Corner is an exact copy (a = A), so it stays at -50.
        assert_eq!(at(0, 0), -50);
        // An interior pixel and an edge pixel both bias to -49.
        assert_eq!(at(1, 1), -49);
        assert_eq!(at(0, 1), -49);
        // -16 divides evenly: interior (16*-16 + 8)/16 = -248/16 = -15
        // (trunc toward zero), so even "clean" multiples bias up by the
        // rounding offset on the negative side.
        let input = [-16i16; RRU_IN_LEN];
        let out = upsample_prediction_error(&input);
        let at = |y: usize, x: usize| out[y * RRU_OUT_DIM + x];
        assert_eq!(at(0, 0), -16); // exact corner copy
        assert_eq!(at(1, 1), -15); // (16*-16 + 8)/16 = -15
    }

    /// Corners copy the nearest reduced sample exactly (§Q.6.2 `a = A`).
    #[test]
    fn corners_copy_nearest_reduced_sample() {
        let mut input = [0i16; RRU_IN_LEN];
        input[0] = 11; // s[0][0]
        input[RRU_IN_DIM - 1] = 22; // s[0][7]
        input[(RRU_IN_DIM - 1) * RRU_IN_DIM] = 33; // s[7][0]
        input[RRU_IN_LEN - 1] = 44; // s[7][7]
        let out = upsample_prediction_error(&input);
        assert_eq!(out[0], 11); // o[0][0]
        assert_eq!(out[RRU_OUT_DIM - 1], 22); // o[0][15]
        assert_eq!(out[(RRU_OUT_DIM - 1) * RRU_OUT_DIM], 33); // o[15][0]
        assert_eq!(out[RRU_OUT_LEN - 1], 44); // o[15][15]
    }

    /// Worked interior example (§Q.6.1, Figure Q.8). Place a single
    /// distinguishable value at each of the four bounding samples of the
    /// top-left interior cell (A=s[0][0], B=s[0][1], C=s[1][0],
    /// D=s[1][1]) and check the four interior weights individually.
    #[test]
    fn interior_cell_matches_figure_q8_weights() {
        // A=16, B=0, C=0, D=0 ⇒ a=(9*16+8)/16=9, b=(3*16+8)/16=3,
        // c=(3*16+8)/16=3, d=(1*16+8)/16=1.
        let mut input = [0i16; RRU_IN_LEN];
        input[0] = 16; // s[0][0] = A
        let out = upsample_prediction_error(&input);
        // The interior 2×2 cell for (i=0, j=0) lands at output
        // (1,1),(1,2),(2,1),(2,2).
        let at = |y: usize, x: usize| out[y * RRU_OUT_DIM + x];
        assert_eq!(at(1, 1), (9 * 16 + 8) / 16); // a = 9
        assert_eq!(at(1, 2), (3 * 16 + 8) / 16); // b = 3
        assert_eq!(at(2, 1), (3 * 16 + 8) / 16); // c = 3
        assert_eq!(at(2, 2), (16 + 8) / 16); // d = (1*16 + 8)/16 = 1
    }

    /// Full Figure Q.8 numerator check with all four corners distinct.
    #[test]
    fn interior_cell_full_weight_combination() {
        let (av, bv, cv, dv) = (40i32, 12, 8, 4);
        let mut input = [0i16; RRU_IN_LEN];
        input[0] = av as i16; // A = s[0][0]
        input[1] = bv as i16; // B = s[0][1]
        input[RRU_IN_DIM] = cv as i16; // C = s[1][0]
        input[RRU_IN_DIM + 1] = dv as i16; // D = s[1][1]
        let out = upsample_prediction_error(&input);
        let at = |y: usize, x: usize| out[y * RRU_OUT_DIM + x] as i32;
        assert_eq!(at(1, 1), (9 * av + 3 * bv + 3 * cv + dv + 8) / 16);
        assert_eq!(at(1, 2), (3 * av + 9 * bv + cv + 3 * dv + 8) / 16);
        assert_eq!(at(2, 1), (3 * av + bv + 9 * cv + 3 * dv + 8) / 16);
        assert_eq!(at(2, 2), (av + 3 * bv + 3 * cv + 9 * dv + 8) / 16);
    }

    /// Top-edge 1-D interpolation (§Q.6.2). Between A=s[0][0] and
    /// B=s[0][1]: b=(3A+B+2)/4 at o[0][1], c=(A+3B+2)/4 at o[0][2].
    #[test]
    fn top_edge_matches_figure_q9_weights() {
        let (av, bv) = (20i32, 8);
        let mut input = [0i16; RRU_IN_LEN];
        input[0] = av as i16; // s[0][0]
        input[1] = bv as i16; // s[0][1]
        let out = upsample_prediction_error(&input);
        let at = |y: usize, x: usize| out[y * RRU_OUT_DIM + x] as i32;
        assert_eq!(at(0, 0), av); // corner copy
        assert_eq!(at(0, 1), (3 * av + bv + 2) / 4);
        assert_eq!(at(0, 2), (av + 3 * bv + 2) / 4);
    }

    /// Left-edge 1-D interpolation (§Q.6.2). Between A=s[0][0] and
    /// C=s[1][0]: d=(3A+C+2)/4 at o[1][0], e=(A+3C+2)/4 at o[2][0].
    #[test]
    fn left_edge_matches_figure_q9_weights() {
        let (av, cv) = (20i32, 8);
        let mut input = [0i16; RRU_IN_LEN];
        input[0] = av as i16; // s[0][0]
        input[RRU_IN_DIM] = cv as i16; // s[1][0]
        let out = upsample_prediction_error(&input);
        let at = |y: usize, x: usize| out[y * RRU_OUT_DIM + x] as i32;
        assert_eq!(at(0, 0), av); // corner copy
        assert_eq!(at(1, 0), (3 * av + cv + 2) / 4);
        assert_eq!(at(2, 0), (av + 3 * cv + 2) / 4);
    }

    /// Every output sample must be filled exactly once (no gaps, no
    /// double-writes leaving a stale default). We detect this by
    /// up-sampling a block whose only zero is impossible to confuse with
    /// an unwritten default: use a strictly positive constant and assert
    /// no output is the `0` default.
    #[test]
    fn every_output_position_is_written() {
        let input = [7i16; RRU_IN_LEN];
        let out = upsample_prediction_error(&input);
        assert!(
            out.iter().all(|&v| v == 7),
            "an unwritten (or mis-written) output position remained 0"
        );
    }

    /// Negative-sample truncation behaves as division-by-truncation
    /// toward zero (Rust `i32 /`). For A=C=-4 on the left edge:
    /// (3*-4 + -4 + 2)/4 = (-14)/4 = -3 (truncated toward zero), not -4
    /// (floor). Guards the truncation semantics §Q.6 calls for.
    #[test]
    fn negative_samples_truncate_toward_zero() {
        // A=-4, far=-2 on top edge: (3*-4 + -2 + 2)/4 = -12/4 = -3.
        let (av, bv) = (-4i32, -2);
        let mut input = [0i16; RRU_IN_LEN];
        input[0] = av as i16;
        input[1] = bv as i16;
        let out = upsample_prediction_error(&input);
        let at = |y: usize, x: usize| out[y * RRU_OUT_DIM + x] as i32;
        // (3*-4 + -2 + 2)/4 = -12/4 = -3.
        assert_eq!(at(0, 1), -3);
        // (-4 + 3*-2 + 2)/4 = -8/4 = -2.
        assert_eq!(at(0, 2), -2);
    }
}
