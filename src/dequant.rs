//! H.263 inverse quantisation (§6.1, §6.2.1, §6.2.2).
//!
//! Implements the H.261-style modulo-2-oddifier inverse-quant rule from
//! ITU-T Recommendation H.263 (01/2005) §6.2.1, plus the §6.2.2
//! reconstruction-level clip and the §6.2.3 zigzag-to-block-position
//! permutation (Figure 14). The INTRADC reconstruction itself was
//! already applied at parse time per Table 15 (see [`crate::block`]);
//! this module leaves that DC slot alone and only touches the AC
//! coefficients.
//!
//! ## §6.1 / §6.2.1 inverse-quant formula
//!
//! For every non-zero AC coefficient with bitstream `LEVEL`:
//!
//! ```text
//!   |REC| = QUANT · (2 · |LEVEL| + 1)         if QUANT is odd
//!   |REC| = QUANT · (2 · |LEVEL| + 1) - 1     if QUANT is even
//!   REC   = sign(LEVEL) · |REC|
//! ```
//!
//! For `LEVEL == 0` the reconstruction level is zero; the formula is
//! not applied (Section 6.2.1 paragraph 1). The §6.2.2 clip pins each
//! AC reconstruction level to `[-2048, 2047]`.
//!
//! ## §6.2.3 zigzag → 8×8 placement
//!
//! The parser leaves coefficients in scan order (`coefficients[0]` =
//! DC, `coefficients[63]` = bottom-right AC). [`scatter_into_block`]
//! applies the [`crate::ZIGZAG_TO_BLOCK_POS`] permutation to produce
//! an 8×8 row-major block ready for IDCT.

use crate::block::{H263Block, COEFFS_PER_BLOCK, ZIGZAG_TO_BLOCK_POS};

/// §6.2.2 AC reconstruction-level clip bounds.
pub const AC_REC_MIN: i16 = -2048;
/// §6.2.2 AC reconstruction-level clip bounds.
pub const AC_REC_MAX: i16 = 2047;

/// Apply §6.2.1 inverse quantisation to a block's AC coefficients
/// **in place**.
///
/// The DC slot (`coefficients[0]`) is preserved verbatim: for INTRA
/// blocks it holds the INTRADC reconstruction level already applied
/// by [`crate::parse_block`] (see Table 15); for INTER blocks the DC
/// slot is just another AC coefficient (no separate INTRADC on the
/// wire) and would be processed under the standard formula. Per the
/// spec text, "the reconstruction level of INTRADC is given by Table
/// 15" — i.e. INTRA's DC bypasses the formula — so we apply the
/// formula starting at slot 1 for INTRA blocks and slot 0 for INTER
/// blocks.
///
/// `quant` must be in `1..=31` (the GQUANT / DQUANT legal range from
/// §5.2.6 / §5.3.6); zero or out-of-range values are clamped (the
/// caller is expected to have already validated against
/// [`crate::Error::InvalidQuantiser`]).
pub fn dequantise_ac(block: &mut H263Block, quant: u8, is_intra: bool) {
    let q = quant.clamp(1, 31) as i32;
    let q_even = (q & 1) == 0;
    let start = if is_intra { 1 } else { 0 };
    for slot in start..COEFFS_PER_BLOCK {
        let level = block.coefficients[slot] as i32;
        if level == 0 {
            continue;
        }
        let abs_level = level.unsigned_abs() as i32;
        let mut abs_rec = q * (2 * abs_level + 1);
        if q_even {
            abs_rec -= 1;
        }
        let rec = if level < 0 { -abs_rec } else { abs_rec };
        // §6.2.2 clip to [-2048, +2047].
        let clipped = rec.clamp(AC_REC_MIN as i32, AC_REC_MAX as i32);
        block.coefficients[slot] = clipped as i16;
    }
}

/// §6.2.3 / Figure 14 zigzag → 8×8 scatter.
///
/// Reads the parser's scan-order coefficient array and writes them
/// into a row-major 8×8 array (`out[row * 8 + col]`) ready for the
/// §6.2.4 inverse transform.
pub fn scatter_into_block(scan_order: &[i16; COEFFS_PER_BLOCK]) -> [i16; COEFFS_PER_BLOCK] {
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for (scan_idx, &coef) in scan_order.iter().enumerate() {
        let block_pos = ZIGZAG_TO_BLOCK_POS[scan_idx] as usize;
        out[block_pos] = coef;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::H263Block;

    fn block_with_coefs(coefs: &[(usize, i16)]) -> H263Block {
        let mut b = H263Block::empty();
        for &(i, v) in coefs {
            b.coefficients[i] = v;
        }
        b
    }

    /// §6.2.1: LEVEL = 0 stays zero regardless of QUANT parity.
    #[test]
    fn zero_level_stays_zero() {
        for q in 1..=31u8 {
            let mut b = H263Block::empty();
            dequantise_ac(&mut b, q, false);
            assert!(b.coefficients.iter().all(|&c| c == 0));
        }
    }

    /// §6.2.1 odd-QUANT formula: |REC| = QUANT * (2|LEVEL| + 1).
    /// Spot-check: QUANT = 1, LEVEL = 1 → |REC| = 1 * 3 = 3.
    #[test]
    fn odd_quant_simple() {
        let mut b = block_with_coefs(&[(0, 1)]);
        dequantise_ac(&mut b, 1, false);
        assert_eq!(b.coefficients[0], 3);
    }

    /// §6.2.1 odd-QUANT, LEVEL = -1 → |REC| = 3, sign-flipped = -3.
    #[test]
    fn odd_quant_negative_level() {
        let mut b = block_with_coefs(&[(0, -1)]);
        dequantise_ac(&mut b, 1, false);
        assert_eq!(b.coefficients[0], -3);
    }

    /// §6.2.1 odd-QUANT, LEVEL = 5, QUANT = 7 → 7 * 11 = 77.
    #[test]
    fn odd_quant_general() {
        let mut b = block_with_coefs(&[(3, 5)]);
        dequantise_ac(&mut b, 7, false);
        assert_eq!(b.coefficients[3], 77);
    }

    /// §6.2.1 even-QUANT formula: |REC| = QUANT * (2|LEVEL| + 1) - 1.
    /// Spot-check: QUANT = 2, LEVEL = 1 → |REC| = 2 * 3 - 1 = 5.
    #[test]
    fn even_quant_simple() {
        let mut b = block_with_coefs(&[(0, 1)]);
        dequantise_ac(&mut b, 2, false);
        assert_eq!(b.coefficients[0], 5);
    }

    /// §6.2.1 even-QUANT, LEVEL = -1, QUANT = 4 → 4 * 3 - 1 = 11, signed -11.
    #[test]
    fn even_quant_negative_level() {
        let mut b = block_with_coefs(&[(0, -1)]);
        dequantise_ac(&mut b, 4, false);
        assert_eq!(b.coefficients[0], -11);
    }

    /// §6.2.1 disallows even-valued numbers per the spec note ("Note
    /// that this process disallows even valued numbers"). Verify that
    /// every reconstructed |REC| is odd for both parities.
    #[test]
    fn rec_levels_are_always_odd() {
        for q in 1..=31u8 {
            for level in -10i16..=10 {
                if level == 0 {
                    continue;
                }
                let mut b = block_with_coefs(&[(2, level)]);
                dequantise_ac(&mut b, q, false);
                let rec = b.coefficients[2];
                assert!(
                    rec.unsigned_abs() & 1 == 1,
                    "QUANT={} LEVEL={} REC={} is even",
                    q,
                    level,
                    rec
                );
            }
        }
    }

    /// INTRA blocks: slot 0 (the INTRADC reconstruction level) must
    /// be preserved verbatim — it is not run through the AC formula.
    #[test]
    fn intra_preserves_dc_slot() {
        let mut b = block_with_coefs(&[(0, 256), (1, 1)]);
        dequantise_ac(&mut b, 5, true);
        assert_eq!(b.coefficients[0], 256, "DC must be untouched");
        // AC slot 1: QUANT=5 (odd), |LEVEL|=1 → 5 * 3 = 15.
        assert_eq!(b.coefficients[1], 15);
    }

    /// INTER blocks: slot 0 is an AC coefficient and goes through the
    /// formula like any other.
    #[test]
    fn inter_processes_slot_zero() {
        let mut b = block_with_coefs(&[(0, 2)]);
        dequantise_ac(&mut b, 3, false);
        // QUANT=3 (odd), |LEVEL|=2 → 3 * 5 = 15.
        assert_eq!(b.coefficients[0], 15);
    }

    /// §6.2.2 clip: a LEVEL big enough to overshoot 2047 must be
    /// clipped. QUANT=31 (odd), LEVEL=127 → 31 * 255 = 7905 → clip 2047.
    #[test]
    fn rec_clipped_at_positive_limit() {
        let mut b = block_with_coefs(&[(5, 127)]);
        dequantise_ac(&mut b, 31, false);
        assert_eq!(b.coefficients[5], 2047);
    }

    /// §6.2.2 clip: negative overshoot pinned to -2048.
    #[test]
    fn rec_clipped_at_negative_limit() {
        let mut b = block_with_coefs(&[(5, -127)]);
        dequantise_ac(&mut b, 31, false);
        assert_eq!(b.coefficients[5], -2048);
    }

    /// All-zero scan input produces an all-zero scattered block.
    #[test]
    fn scatter_zero_input() {
        let scan = [0i16; COEFFS_PER_BLOCK];
        let out = scatter_into_block(&scan);
        assert!(out.iter().all(|&c| c == 0));
    }

    /// Scatter places scan-position 0 at block (0, 0).
    #[test]
    fn scatter_dc_goes_to_top_left() {
        let mut scan = [0i16; COEFFS_PER_BLOCK];
        scan[0] = 42;
        let out = scatter_into_block(&scan);
        assert_eq!(out[0], 42);
        assert!(out[1..].iter().all(|&c| c == 0));
    }

    /// Scatter places scan-position 63 at block (7, 7) per Figure 14
    /// row 7 column 7 == coefficient 64 (1-based).
    #[test]
    fn scatter_last_goes_to_bottom_right() {
        let mut scan = [0i16; COEFFS_PER_BLOCK];
        scan[63] = -7;
        let out = scatter_into_block(&scan);
        assert_eq!(out[63], -7);
        assert!(out[..63].iter().all(|&c| c == 0));
    }

    /// Scatter applied with a unique value at each scan slot lands
    /// every value at a unique block position (the permutation is
    /// bijective).
    #[test]
    fn scatter_is_a_permutation() {
        let mut scan = [0i16; COEFFS_PER_BLOCK];
        for (i, slot) in scan.iter_mut().enumerate() {
            *slot = (i as i16) + 1;
        }
        let out = scatter_into_block(&scan);
        let mut seen = [false; COEFFS_PER_BLOCK];
        for &v in out.iter() {
            let idx = (v as usize) - 1;
            assert!(!seen[idx], "value {} appears twice", v);
            seen[idx] = true;
        }
        assert!(seen.iter().all(|s| *s));
    }
}
