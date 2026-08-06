//! H.263 **forward** discrete cosine transform and forward quantisation
//! — the encode-side inverse of [`crate::idct`] (§6.2.4) and
//! [`crate::dequant`] (§6.2.1).
//!
//! The transform itself is non-normative on the encode side: the spec
//! (§6.2.4 / Annex A) only constrains the *inverse* transform's
//! accuracy. We use the exact orthonormal forward DCT that inverts the
//! decoder's `f64` IDCT, so an unquantised forward → inverse round-trip
//! reproduces the input block to within the IDCT's own rounding.
//!
//! ## Forward DCT (§6.2.4 inverse)
//!
//! The decoder's IDCT is the orthonormal kernel
//!
//! ```text
//!   f(x,y) = (1/4) Σ_u Σ_v C(u) C(v) F(u,v) cos(π(2x+1)u/16) cos(π(2y+1)v/16)
//! ```
//!
//! with `C(0) = 1/√2`, `C(k≠0) = 1`. The matching forward transform is
//!
//! ```text
//!   F(u,v) = (1/4) C(u) C(v) Σ_x Σ_y f(x,y) cos(π(2x+1)u/16) cos(π(2y+1)v/16)
//! ```
//!
//! which is the transpose/orthonormal dual; composing forward then
//! inverse is the identity (modulo float rounding), so a residual that
//! is quantised to zero reconstructs to zero and a residual that
//! survives quantisation reconstructs to its dequantised approximation.
//!
//! ## Forward quantisation (§6.2.1 inverse)
//!
//! §6.2.1 reconstructs a non-zero quantised LEVEL `L` to
//!
//! ```text
//!   |REC| = QUANT·(2|L|+1)        QUANT odd
//!   |REC| = QUANT·(2|L|+1) − 1    QUANT even
//! ```
//!
//! The forward direction is non-normative; we use the conventional
//! H.263-style dead-zone quantiser
//!
//! ```text
//!   |L| = floor(|F| / (2·QUANT))
//! ```
//!
//! which places the reconstruction levels symmetrically around the
//! transform value and maps small coefficients (`|F| < 2·QUANT`) to
//! zero (the dead zone). The INTRA DC coefficient bypasses this and is
//! handled by [`quantise_intradc`] against Table 15.

use crate::block::COEFFS_PER_BLOCK;
use crate::idct::BLOCK_DIM;

/// `cos(π·(2n+1)·k/16)` cache, identical to the decoder's IDCT basis.
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

/// `C(k)` — `1/√2` for k = 0, `1` otherwise (matches the IDCT).
#[inline]
fn c(k: usize) -> f64 {
    if k == 0 {
        1.0 / core::f64::consts::SQRT_2
    } else {
        1.0
    }
}

/// §6.2.4 forward DCT (the orthonormal dual of [`crate::idct::idct_8x8`]).
///
/// Takes an 8×8 spatial block in row-major order (`samples[y*8 + x]`)
/// and returns the 8×8 frequency-domain coefficient block, also
/// row-major with the crate convention `out[v*8 + u] = F(u, v)`.
/// Coefficients are returned as rounded `f64` values without
/// quantisation; the caller quantises with [`forward_quantise_block`].
pub fn fdct_8x8(samples: &[i16; COEFFS_PER_BLOCK]) -> [f64; COEFFS_PER_BLOCK] {
    let table = cos_table();
    let mut out = [0.0f64; COEFFS_PER_BLOCK];
    for v in 0..BLOCK_DIM {
        for u in 0..BLOCK_DIM {
            let mut acc = 0.0f64;
            for y in 0..BLOCK_DIM {
                for x in 0..BLOCK_DIM {
                    let f = samples[y * BLOCK_DIM + x] as f64;
                    acc += f * table[u][x] * table[v][y];
                }
            }
            out[v * BLOCK_DIM + u] = 0.25 * c(u) * c(v) * acc;
        }
    }
    out
}

/// The maximum LEVEL magnitude a baseline (non-Modified-Quantization)
/// bitstream can represent: Table 16 tops out well below it and the
/// §5.4.2 ESCAPE LEVEL field is 8-bit two's complement with `0x00`
/// and `0x80` forbidden, so `|LEVEL| ≤ 127`. A conformant encoder
/// must not produce an event outside this range; the forward
/// quantiser saturates to it (the reconstruction merely clips at the
/// largest representable magnitude — the same policy as the Annex I
/// planner's `MAX_LEVEL_BASELINE` clamp).
const MAX_AC_LEVEL_BASELINE: i32 = 127;

/// Forward-quantise a single AC coefficient `coef` (frequency-domain
/// value) to a signed quantised LEVEL using the dead-zone rule
/// `|L| = floor(|coef| / (2·quant))`.
///
/// `quant` is the §5.2.6 / §5.3.6 step size (`1..=31`). Returns the
/// signed LEVEL; small coefficients in the `|coef| < 2·quant` dead zone
/// quantise to zero, and the magnitude saturates at the §5.4.2
/// baseline-representable maximum of 127 (sharp content at very fine
/// quantisers would otherwise demand an unrepresentable event).
pub fn quantise_ac_coefficient(coef: f64, quant: u8) -> i16 {
    let q = quant.clamp(1, 31) as f64;
    let mag = (coef.abs() / (2.0 * q)).floor();
    let level = (mag as i32).min(MAX_AC_LEVEL_BASELINE);
    if level == 0 {
        return 0;
    }
    if coef < 0.0 {
        (-level) as i16
    } else {
        level as i16
    }
}

/// Quantise an INTRA DC coefficient (the §6.2.4 DC value `F(0, 0)`,
/// which for the orthonormal forward DCT equals `8 × mean(samples)`)
/// into a legal Table 15 reconstruction level.
///
/// Returns the nearest reconstruction level the decoder can produce:
/// a multiple of 8 in `8..=2032`, or `1024` (the `0xFF` special case).
/// The value is clamped into `[8, 2032]` before rounding so it is
/// always a legal INTRADC the encoder can emit with
/// [`crate::encoder_vlc::write_intradc`].
pub fn quantise_intradc(dc: f64) -> i16 {
    // Round to the nearest multiple of 8.
    let rounded = (dc / 8.0).round() * 8.0;
    let clamped = rounded.clamp(8.0, 2032.0) as i16;
    // 1024 has two codes (the natural 128 slot is forbidden, the 0xFF
    // slot reconstructs to exactly 1024); both reconstruct to 1024, and
    // the emitter maps 1024 → 0xFF, so leaving the value at 1024 is
    // correct.
    clamped
}

/// Forward-quantise a full 8×8 frequency-domain block into the
/// scan-order quantised coefficient array consumed by the encoder's
/// TCOEF emitter.
///
/// `is_intra` controls the DC slot: for INTRA blocks the caller handles
/// the DC via [`quantise_intradc`] / Table 15 separately, so this
/// function quantises only AC slots (DC slot left zero); for INTER
/// blocks the DC slot is an ordinary AC coefficient and is quantised
/// like the rest.
///
/// The returned array is in **zigzag scan order** (`out[0]` = DC slot),
/// matching [`crate::block::H263Block::coefficients`]. The input
/// `block` is in **block position** order (`block[v*8 + u]`), so the
/// inverse of the decoder's [`crate::ZIGZAG_TO_BLOCK_POS`] scatter is
/// applied here.
pub fn forward_quantise_block(
    block: &[f64; COEFFS_PER_BLOCK],
    quant: u8,
    is_intra: bool,
) -> [i16; COEFFS_PER_BLOCK] {
    use crate::block::ZIGZAG_TO_BLOCK_POS;
    let mut scan = [0i16; COEFFS_PER_BLOCK];
    let start = if is_intra { 1 } else { 0 };
    for (scan_idx, slot) in scan
        .iter_mut()
        .enumerate()
        .take(COEFFS_PER_BLOCK)
        .skip(start)
    {
        let block_pos = ZIGZAG_TO_BLOCK_POS[scan_idx] as usize;
        *slot = quantise_ac_coefficient(block[block_pos], quant);
    }
    scan
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::idct::idct_8x8;

    /// A flat (DC-only) block: the forward DCT puts all energy in F(0,0)
    /// = 8·value, and the IDCT recovers the flat field.
    #[test]
    fn fdct_flat_block_is_dc_only() {
        let samples = [50i16; COEFFS_PER_BLOCK];
        let f = fdct_8x8(&samples);
        // DC = 8 * 50 = 400.
        assert!((f[0] - 400.0).abs() < 1e-6, "DC = {}", f[0]);
        // All AC ≈ 0.
        for &coef in f[1..].iter() {
            assert!(coef.abs() < 1e-6, "AC leak {}", coef);
        }
    }

    /// Forward DCT then IDCT reproduces the input block (within IDCT
    /// rounding) for an arbitrary smooth block.
    #[test]
    fn fdct_idct_round_trip_identity() {
        let mut samples = [0i16; COEFFS_PER_BLOCK];
        for y in 0..8 {
            for x in 0..8 {
                samples[y * 8 + x] = (16 * x + 8 * y) as i16; // 0..=176
            }
        }
        let f = fdct_8x8(&samples);
        // Round the float coefficients to integers and inverse-transform.
        let mut fi = [0i16; COEFFS_PER_BLOCK];
        for (o, &v) in fi.iter_mut().zip(f.iter()) {
            *o = v.round() as i16;
        }
        let back = idct_8x8(&fi);
        for (i, (&a, &b)) in samples.iter().zip(back.iter()).enumerate() {
            assert!((a - b).abs() <= 1, "pixel {} : {} vs {}", i, a, b);
        }
    }

    /// Dead-zone quantiser: |coef| < 2·quant maps to zero.
    #[test]
    fn quantise_ac_dead_zone() {
        // quant=4 -> dead zone is |coef| < 8.
        assert_eq!(quantise_ac_coefficient(7.0, 4), 0);
        assert_eq!(quantise_ac_coefficient(-7.9, 4), 0);
        assert_eq!(quantise_ac_coefficient(8.0, 4), 1);
        assert_eq!(quantise_ac_coefficient(-8.0, 4), -1);
        assert_eq!(quantise_ac_coefficient(16.0, 4), 2);
    }

    /// INTRADC quantiser rounds to a legal Table 15 multiple of 8.
    #[test]
    fn quantise_intradc_rounds_to_multiple_of_8() {
        assert_eq!(quantise_intradc(400.0), 400);
        assert_eq!(quantise_intradc(403.0), 400);
        assert_eq!(quantise_intradc(405.0), 408);
        // Clamp range.
        assert_eq!(quantise_intradc(0.0), 8);
        assert_eq!(quantise_intradc(5000.0), 2032);
    }

    /// A quantised-then-dequantised AC value reconstructs to roughly the
    /// original transform coefficient (within one quant step).
    #[test]
    fn forward_quant_then_dequant_approximates() {
        use crate::block::H263Block;
        use crate::dequant::dequantise_ac;
        for quant in [1u8, 3, 8, 15, 31] {
            for coef in [-200.0f64, -33.0, 9.0, 47.0, 130.0] {
                let level = quantise_ac_coefficient(coef, quant);
                if level == 0 {
                    // dead-zoned; original was within 2·quant of zero.
                    assert!(coef.abs() < 2.0 * quant as f64);
                    continue;
                }
                let mut b = H263Block::empty();
                b.coefficients[5] = level;
                dequantise_ac(&mut b, quant, false);
                let rec = b.coefficients[5] as f64;
                // Reconstruction is within ~2·quant of the source.
                assert!(
                    (rec - coef).abs() <= 2.0 * quant as f64 + 1.0,
                    "quant={} coef={} level={} rec={}",
                    quant,
                    coef,
                    level,
                    rec
                );
            }
        }
    }
}
