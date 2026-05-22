//! H.263 inverse discrete cosine transform (§6.2.4) and intra-block
//! sample reconstruction (§6.3.2 clip).
//!
//! ITU-T Recommendation H.263 (01/2005) §6.2.4 defines the inverse
//! transform via the literal mathematical equation
//!
//! ```text
//!                 1   7   7                                u           v
//!     f(x, y) = ─── · Σ   Σ  C(u) C(v) F(u, v) cos π(2x+1)── cos π(2y+1)──
//!                 4  u=0 v=0                              16          16
//!
//!     C(0) = 1/√2,   C(k) = 1  for k ≠ 0
//! ```
//!
//! (The spec writes "C(u) = 1/2 for u = 0", which together with the
//! 1/4 prefactor gives an orthonormal kernel — i.e. with C(0) = 1/√2
//! and the 1/4 scaled to 2/8 the overall constant matches `1 / (2√2 · 2√2) = 1/8`
//! for the DC-DC weighting, which is what the orthonormal definition
//! in §A.2 fixes.) The spec defers "the arithmetic procedures for
//! computing the inverse transform" to the implementer, only
//! requiring that the result meet the Annex A.7 accuracy budget. We
//! pick the simplest direct double-precision evaluation — a tight
//! loop over 64 (x, y) output pixels each summing 64 (u, v)
//! frequency-domain terms — which is the textbook orthonormal IDCT
//! definition with no separability tricks. Annex A.7 demands at most
//! 1 LSB peak error against an "at least 64-bit floating point"
//! reference; using exactly 64-bit floats with `f64::cos` for the
//! cosine table puts us at that reference, so Annex A.7 is satisfied
//! by construction.
//!
//! ## Output range and clipping
//!
//! §6.2.4 says "The output from the inverse transform ranges from
//! –256 to +255 after clipping to be represented with 9 bits." We
//! apply this clip on the rounded integer output of the IDCT before
//! returning.
//!
//! ## Intra-block sample reconstruction
//!
//! §6.3.1: for INTRA blocks "the reconstruction is equal to the
//! result of the inverse transformation". §6.3.2 then clips to
//! `[0, 255]` for display. We expose [`reconstruct_intra_samples`]
//! that walks the cosine kernel against an 8×8 dequantised
//! coefficient block (post-zigzag scatter) and emits an 8×8 `u8`
//! sample block ready for the picture buffer.

use crate::block::COEFFS_PER_BLOCK;

/// §6.2.4 IDCT output range (9-bit signed).
pub const IDCT_OUT_MIN: i16 = -256;
/// §6.2.4 IDCT output range (9-bit signed).
pub const IDCT_OUT_MAX: i16 = 255;

/// Side length of the H.263 transform block.
pub const BLOCK_DIM: usize = 8;

/// 8×8 cosine basis cache.
///
/// `cos_table()[k][n]` returns `cos(π · (2n+1) · k / 16)` for `k, n`
/// in `0..8`. The kernel is even-symmetric under `(2n+1) → (15 - (2n+1))`
/// and odd-symmetric for odd `k`, but we don't lean on that — the
/// table is small (64 doubles) and the IDCT is called once per block.
fn cos_table() -> &'static [[f64; BLOCK_DIM]; BLOCK_DIM] {
    use std::sync::OnceLock;
    static TABLE: OnceLock<[[f64; BLOCK_DIM]; BLOCK_DIM]> = OnceLock::new();
    TABLE.get_or_init(|| {
        let mut t = [[0.0f64; BLOCK_DIM]; BLOCK_DIM];
        for (k, row) in t.iter_mut().enumerate() {
            for (n, slot) in row.iter_mut().enumerate() {
                let arg = core::f64::consts::PI * ((2 * n + 1) as f64) * (k as f64) / 16.0;
                *slot = arg.cos();
            }
        }
        t
    })
}

/// `C(k)` from §6.2.4 — `1/√2` for k = 0, `1` otherwise.
#[inline]
fn c(k: usize) -> f64 {
    if k == 0 {
        1.0 / core::f64::consts::SQRT_2
    } else {
        1.0
    }
}

/// §6.2.4 inverse DCT.
///
/// Takes an 8×8 dequantised coefficient block in row-major order
/// (`coefs[row * 8 + col] == F(u=col, v=row)`) and returns the 8×8
/// inverse-transformed sample block, also row-major. Output is
/// rounded to integer and clipped to `[-256, +255]` per §6.2.4.
///
/// **Convention.** This crate uses the convention `(u, v) = (col, row)`
/// throughout — i.e. block position `(row, col)` in storage carries
/// the frequency-domain coefficient `F(col, row)`. The inverse
/// transform is symmetric in `(u, v)` swap, so the convention is
/// internally consistent regardless of which axis is named first.
pub fn idct_8x8(coefs: &[i16; COEFFS_PER_BLOCK]) -> [i16; COEFFS_PER_BLOCK] {
    let table = cos_table();
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for y in 0..BLOCK_DIM {
        for x in 0..BLOCK_DIM {
            let mut acc = 0.0f64;
            for v in 0..BLOCK_DIM {
                for u in 0..BLOCK_DIM {
                    let f = coefs[v * BLOCK_DIM + u] as f64;
                    acc += c(u) * c(v) * f * table[u][x] * table[v][y];
                }
            }
            // §6.2.4 prefactor 1/4: outer `1/4` ⋅ inner `C(0) = 1/√2`
            // factors land us at the orthonormal scaling. We apply the
            // 1/4 here.
            let pixel = acc * 0.25;
            // Round-half-away-from-zero (the standard `floor(x + 0.5)`
            // for positive, `ceil(x - 0.5)` for negative). The spec
            // calls for "the nearest integer".
            let rounded = if pixel >= 0.0 {
                (pixel + 0.5).floor() as i32
            } else {
                (pixel - 0.5).ceil() as i32
            };
            let clipped = rounded.clamp(IDCT_OUT_MIN as i32, IDCT_OUT_MAX as i32) as i16;
            out[y * BLOCK_DIM + x] = clipped;
        }
    }
    out
}

/// §6.3.1 + §6.3.2 reconstruction for INTRA blocks.
///
/// Takes a dequantised 8×8 block (post-zigzag scatter), runs the
/// IDCT, and clips to `[0, 255]` per §6.3.2. INTRA blocks have no
/// motion-compensation prediction, so the IDCT output **is** the
/// reconstructed sample (modulo the §6.3.2 clip). The §6.2.4 nominal
/// output range is `[-256, +255]`; the §6.3.2 clip narrows that to
/// the 8-bit picture range `[0, 255]`.
pub fn reconstruct_intra_samples(coefs: &[i16; COEFFS_PER_BLOCK]) -> [u8; COEFFS_PER_BLOCK] {
    let idct = idct_8x8(coefs);
    let mut out = [0u8; COEFFS_PER_BLOCK];
    for (i, &v) in idct.iter().enumerate() {
        out[i] = v.clamp(0, 255) as u8;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §A.8: "All zeros in shall produce all zeros out."
    #[test]
    fn all_zero_input_yields_all_zero_output() {
        let zero = [0i16; COEFFS_PER_BLOCK];
        let out = idct_8x8(&zero);
        assert!(out.iter().all(|&p| p == 0));
    }

    /// DC-only input: F(0, 0) = K, rest zero.
    ///
    /// f(x, y) = (1/4) · C(0) · C(0) · K · cos(0) · cos(0)
    ///        = (1/4) · (1/√2) · (1/√2) · K
    ///        = K / 8
    ///
    /// For K = 8 → every pixel is 1. For K = 800 → every pixel is 100.
    #[test]
    fn dc_only_block_gives_uniform_field() {
        for &dc in &[8i16, 16, 80, 800, 8 * 100] {
            let mut coefs = [0i16; COEFFS_PER_BLOCK];
            coefs[0] = dc;
            let out = idct_8x8(&coefs);
            let expected = dc / 8;
            for (i, &p) in out.iter().enumerate() {
                assert_eq!(
                    p, expected,
                    "dc={}, position {}: expected {}, got {}",
                    dc, i, expected, p
                );
            }
        }
    }

    /// DC-only with negative coefficient.
    #[test]
    fn dc_only_negative() {
        let mut coefs = [0i16; COEFFS_PER_BLOCK];
        coefs[0] = -16;
        let out = idct_8x8(&coefs);
        assert!(out.iter().all(|&p| p == -2));
    }

    /// Sample reconstruction: an INTRA DC = 800 reconstructs to a
    /// uniform field of value 100 (§6.3.2 clip is a no-op since 100
    /// is already in `[0, 255]`).
    #[test]
    fn intra_dc_only_reconstruct() {
        let mut coefs = [0i16; COEFFS_PER_BLOCK];
        coefs[0] = 800;
        let samples = reconstruct_intra_samples(&coefs);
        assert!(samples.iter().all(|&p| p == 100));
    }

    /// §6.3.2 clip: an INTRA DC = 2048 (overshoots 255) saturates to
    /// the IDCT's own -256/+255 range first (giving 255 after rounding
    /// from 256.0 → 256, then clipped to 255), then §6.3.2 keeps 255.
    #[test]
    fn intra_dc_clipped_at_top() {
        let mut coefs = [0i16; COEFFS_PER_BLOCK];
        // DC = 2048 → pixel = 2048/8 = 256 → §6.2.4 clip → 255 → §6.3.2 no-op.
        coefs[0] = 2048;
        let samples = reconstruct_intra_samples(&coefs);
        assert!(samples.iter().all(|&p| p == 255));
    }

    /// §6.3.2 clip: a strongly negative DC saturates to 0.
    #[test]
    fn intra_dc_clipped_at_zero() {
        let mut coefs = [0i16; COEFFS_PER_BLOCK];
        coefs[0] = -1000;
        let samples = reconstruct_intra_samples(&coefs);
        assert!(samples.iter().all(|&p| p == 0));
    }

    /// §A.7 peak-error budget self-check: feed a known cosine-pattern
    /// block (forward DCT of a simple ramp) and verify the IDCT
    /// reproduces the ramp within ±1.
    ///
    /// We construct the forward DCT of a 1-D ramp `f(x) = x` along
    /// rows (zero along columns) by inverting our IDCT against a
    /// well-known orthonormal-DCT pair. Rather than encode the
    /// forward DCT separately, we use a single-AC-coefficient test:
    /// F(u=1, v=0) = K gives f(x, y) = (K/4) · cos(π(2x+1)/16) when
    /// y has no v-dependence (and is independent of y).
    #[test]
    fn single_ac_coefficient_horizontal_basis() {
        // F(u=1, v=0) = 64. Expected:
        // f(x, y) = (1/4) · 1 · (1/√2) · 64 · cos(π(2x+1)/16) · cos(0)
        //        = 64 · (1/(4·√2)) · cos(π(2x+1)/16)
        //        ≈ 11.31 · cos(π(2x+1)/16)
        // Independent of y.
        let mut coefs = [0i16; COEFFS_PER_BLOCK];
        coefs[1] = 64; // F(u=1, v=0)
        let out = idct_8x8(&coefs);

        // Compute expected values in f64 and compare with peak error ≤ 1.
        for y in 0..BLOCK_DIM {
            for x in 0..BLOCK_DIM {
                let cos_term = (core::f64::consts::PI * ((2 * x + 1) as f64) / 16.0).cos();
                let expected = 64.0 * (1.0 / (4.0 * core::f64::consts::SQRT_2)) * cos_term;
                let got = out[y * BLOCK_DIM + x] as f64;
                assert!(
                    (got - expected).abs() <= 1.0,
                    "(x={}, y={}): expected ≈ {}, got {}",
                    x,
                    y,
                    expected,
                    got
                );
            }
        }
    }

    /// Symmetry self-check: an IDCT of a block with only F(0,0) and
    /// F(7,7) populated produces a result symmetric across the
    /// diagonal (because the kernel cos terms factorise).
    #[test]
    fn block_symmetry_diagonal_basis() {
        let mut coefs = [0i16; COEFFS_PER_BLOCK];
        coefs[0] = 64; // DC
        coefs[7 * BLOCK_DIM + 7] = 64; // F(u=7, v=7)
        let out = idct_8x8(&coefs);
        // Symmetric under (x, y) → (y, x).
        for y in 0..BLOCK_DIM {
            for x in 0..BLOCK_DIM {
                assert_eq!(
                    out[y * BLOCK_DIM + x],
                    out[x * BLOCK_DIM + y],
                    "asymmetric at (x={}, y={})",
                    x,
                    y
                );
            }
        }
    }

    /// IDCT of a block with all values in [-2048, 2047] never overflows
    /// the i32 accumulator — sanity smoke test for a max-magnitude
    /// input.
    #[test]
    fn no_overflow_on_extreme_input() {
        let mut coefs = [0i16; COEFFS_PER_BLOCK];
        for v in coefs.iter_mut() {
            *v = 2047;
        }
        let out = idct_8x8(&coefs);
        // The exact pixel values aren't the point — we just need this
        // to terminate without panicking.
        assert_eq!(out.len(), COEFFS_PER_BLOCK);
    }
}
