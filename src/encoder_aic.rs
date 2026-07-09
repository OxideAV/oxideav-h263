//! Annex I — Advanced INTRA Coding: §I.3 block-level *encoder*.
//!
//! This is the encoder counterpart of the decoder's §I.3 INTRA path
//! (`picture::decode_intra_macroblock_aic`). It plans and emits a single
//! AIC INTRA 8×8 block: forward DCT of the source samples, coefficient-
//! domain prediction against the already-reconstructed neighbour blocks
//! (`RecA'` above, `RecB'` to the left), the §I.3 modified forward
//! quantisation, the §I.3 / Figure I.2 scan selection, and the Table I.2
//! separate INTRA-coefficient VLC ([`crate::intra_tcoef::write_intra_tcoef_event`]).
//!
//! ## Closed loop
//!
//! The plan step reconstructs the block through the exact decoder
//! primitive [`aic_intra_reconstruct_coefficients`] using the chosen
//! LEVELs, so the `RecC'(u,v)` array it returns is bit-identical to what
//! the decoder will produce from the emitted bits. The macroblock-grid
//! driver threads that array forward as the next block's neighbour, so
//! encoder and decoder never drift.
//!
//! ## §I.3 absorbed INTRADC
//!
//! In AIC mode INTRADC is not a separate FLC — the DC coefficient is the
//! slot-0 event of the Table I.2 stream (§I.3 line 4214). The block's
//! CBP bit (CBPY for luma, CBPC for chroma) is the sole "all-zero"
//! signal: when clear, no coefficients (DC included) are on the wire and
//! the decoder reconstructs the block from the predictor alone. This
//! module therefore reports each block's coded/not-coded state up to the
//! macroblock encoder, which packs the CBPY / CBPC codewords.

use crate::aic::{scan_for_intra_mode, IntraMode};
use crate::aic_predict::{
    aic_intra_reconstruct_coefficients, reconstruct_intra_block_aic, Neighbour,
};
use crate::block::{H263Block, COEFFS_PER_BLOCK};
use crate::fdct::fdct_8x8;
use crate::intra_tcoef::{write_intra_tcoef_event, IntraTcoefEvent};
use crate::Result;
use oxideav_core::bits::BitWriter;

/// The maximum LEVEL magnitude representable without Modified
/// Quantization mode — the §5.4.2 / §I.3 ESCAPE LEVEL field is 8-bit
/// two's complement with `0x80` forbidden, so the magnitude tops out at
/// 127.
const MAX_LEVEL_BASELINE: i32 = 127;

/// The maximum LEVEL magnitude representable via the §T.4 EXTENDED-LEVEL
/// 11-bit two's-complement field (used only under Modified Quantization
/// mode, §T.5 rule 2).
const MAX_LEVEL_EXTENDED: i32 = 1023;

/// A planned AIC INTRA block: whether it carries coefficients (the CBP
/// bit), the chosen scan-order LEVELs, and the reconstructed `RecC'(u,v)`
/// block-position array to thread forward as a neighbour predictor.
#[derive(Debug, Clone)]
pub struct AicBlockPlan {
    /// The block's CBP bit: `true` iff at least one LEVEL is non-zero and
    /// the Table I.2 event stream must be emitted. When `false` the block
    /// is reconstructed from the predictor alone (§I.3).
    pub coded: bool,
    /// The chosen quantised LEVELs in **scan order** (the scan selected
    /// by the macroblock's INTRA_MODE). Only meaningful when `coded`.
    pub levels: [i16; COEFFS_PER_BLOCK],
    /// The reconstructed `RecC'(u,v)` array in **block position** layout
    /// (`index = v * 8 + u`) — bit-identical to the decoder's output for
    /// these LEVELs. Fed to downstream blocks as `RecA'` / `RecB'`.
    pub rec: [i32; COEFFS_PER_BLOCK],
}

/// Plan one Annex I §I.3 INTRA block.
///
/// `source` holds the 8×8 spatial samples in block-position (row-major)
/// order, each in `0..=255` widened to `i16`. `mode` is the macroblock's
/// INTRA_MODE. `quant` is the block's quantiser (`1..=31`; luma QUANT or
/// the §T.3 chroma `QUANT_C`). `block_a` / `block_b` are the
/// already-reconstructed `RecA'` (above) / `RecB'` (left) neighbours, or
/// [`Neighbour::None`] per the §I.3 availability rules. `modified_quant`
/// widens the representable LEVEL range via §T.4 EXTENDED-LEVEL.
///
/// The plan is a lossy but closed-loop choice: LEVELs are computed by
/// forward-quantising the DCT residual against the predictor the decoder
/// will add, then the block is reconstructed through the exact decoder
/// primitive so the returned `RecC'` matches the wire.
#[must_use]
pub fn plan_intra_block_aic(
    source: &[i16; COEFFS_PER_BLOCK],
    mode: IntraMode,
    quant: u8,
    block_a: Neighbour<'_>,
    block_b: Neighbour<'_>,
    modified_quant: bool,
) -> AicBlockPlan {
    // §6.2.4 forward DCT → block-position coefficients.
    let coef = fdct_8x8(source);

    // The predictor the decoder adds for each block position: reconstruct
    // with a zero residual. AC slots collapse to clipAC(predictor); the
    // DC slot to oddifyclipDC(predictor). This is what a LEVEL of zero
    // would reconstruct to, so the residual we must quantise for slot bp
    // is `coef[bp] - pred[bp]`.
    let zero = [0i32; COEFFS_PER_BLOCK];
    let pred = reconstruct_intra_block_aic(&zero, mode, block_a, block_b);

    let scan = scan_for_intra_mode(mode);
    let q = quant.clamp(1, 31) as i32;
    let step = 2 * q;
    let max_level = if modified_quant {
        MAX_LEVEL_EXTENDED
    } else {
        MAX_LEVEL_BASELINE
    };

    let mut levels = [0i16; COEFFS_PER_BLOCK];
    let mut any = false;
    for (scan_pos, slot) in levels.iter_mut().enumerate() {
        let bp = scan[scan_pos] as usize;
        let residual = coef[bp] - pred[bp] as f64;
        // Round-to-nearest quantisation of the residual.
        let mut level = (residual / step as f64).round() as i32;
        level = level.clamp(-max_level, max_level);
        if level != 0 {
            any = true;
        }
        *slot = level as i16;
    }

    // Reconstruct through the exact decoder primitive so `rec` matches the
    // wire regardless of the (lossy) LEVEL choice above.
    let rec = if any {
        let mut block = H263Block::empty();
        block.coefficients = levels;
        aic_intra_reconstruct_coefficients(&block, mode, quant, block_a, block_b)
    } else {
        // All-zero LEVELs: the decoder reconstructs from the predictor
        // alone, which is exactly `pred`.
        pred
    };

    AicBlockPlan {
        coded: any,
        levels,
        rec,
    }
}

/// Emit the Table I.2 event stream for a coded AIC INTRA block plan.
///
/// Walks the scan-order LEVELs, emitting one Table-I.2 `(LAST, RUN,
/// LEVEL)` event per non-zero coefficient (RUN = intervening zeros, LAST
/// on the final non-zero), via [`write_intra_tcoef_event`]. A
/// not-coded plan (`plan.coded == false`) emits nothing — its absence is
/// carried by the CBP bit the macroblock encoder packs.
///
/// Errors only if a chosen LEVEL somehow falls outside the representable
/// range (it never does — [`plan_intra_block_aic`] clamps to it), so the
/// [`Result`] is a defensive pass-through of [`write_intra_tcoef_event`].
pub fn write_intra_block_aic(
    w: &mut BitWriter,
    plan: &AicBlockPlan,
    modified_quant: bool,
) -> Result<()> {
    if !plan.coded {
        return Ok(());
    }

    // Index of the last non-zero coefficient (guaranteed to exist because
    // `coded` implies at least one non-zero LEVEL).
    let last_nz = plan
        .levels
        .iter()
        .rposition(|&l| l != 0)
        .expect("coded plan has a non-zero level");

    let mut run: u8 = 0;
    for (scan_pos, &level) in plan.levels.iter().enumerate() {
        if level == 0 {
            run += 1;
            continue;
        }
        let event = IntraTcoefEvent {
            last: scan_pos == last_nz,
            run,
            level,
        };
        write_intra_tcoef_event(w, event, modified_quant)?;
        run = 0;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block_aic::parse_intra_block_aic;
    use oxideav_core::bits::BitReader;

    fn finish_aligned(mut w: BitWriter) -> Vec<u8> {
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    /// A flat block whose DC is far from the 1024 no-neighbour fallback
    /// predictor (mode 0): the encoder must code a DC LEVEL, and the
    /// reconstruction reproduces the flat field within IDCT tolerance.
    #[test]
    fn flat_block_mode0_reconstructs_dc() {
        let source = [200i16; COEFFS_PER_BLOCK];
        let plan = plan_intra_block_aic(
            &source,
            IntraMode::DcOnly,
            8,
            Neighbour::None,
            Neighbour::None,
            false,
        );
        assert!(plan.coded, "DC far from the 1024 fallback must be coded");
        let samples = crate::aic_predict::aic_intra_reconstruct_samples(&plan.rec);
        for &p in samples.iter() {
            assert!((p as i32 - 200).abs() <= 2, "pixel {p} far from 200");
        }
    }

    /// A flat block whose DC coincides with the 1024 fallback predictor
    /// (mode 0, no neighbours): the residual quantises to zero, so the
    /// block is legitimately not coded yet still reconstructs to ~128
    /// from the predictor alone (§I.3).
    #[test]
    fn flat_block_matching_fallback_is_not_coded() {
        let source = [128i16; COEFFS_PER_BLOCK]; // fdct DC = 1024 ≈ fallback
        let plan = plan_intra_block_aic(
            &source,
            IntraMode::DcOnly,
            8,
            Neighbour::None,
            Neighbour::None,
            false,
        );
        assert!(!plan.coded);
        let samples = crate::aic_predict::aic_intra_reconstruct_samples(&plan.rec);
        for &p in samples.iter() {
            assert!((p as i32 - 128).abs() <= 2, "pixel {p} far from 128");
        }
    }

    /// The emitted Table I.2 stream, parsed by the decoder's
    /// `parse_intra_block_aic`, reconstructs the *same* `RecC'` the
    /// encoder planned — the closed-loop invariant.
    #[test]
    fn emitted_stream_reconstructs_planned_rec() {
        // A non-flat gradient block exercises multiple AC coefficients.
        let mut source = [0i16; COEFFS_PER_BLOCK];
        for y in 0..8 {
            for x in 0..8 {
                source[y * 8 + x] = (40 + 12 * x + 6 * y) as i16;
            }
        }
        for &mode in &[
            IntraMode::DcOnly,
            IntraMode::VerticalDcAc,
            IntraMode::HorizontalDcAc,
        ] {
            let quant = 6;
            let plan = plan_intra_block_aic(
                &source,
                mode,
                quant,
                Neighbour::None,
                Neighbour::None,
                false,
            );
            assert!(plan.coded);

            let mut w = BitWriter::new();
            write_intra_block_aic(&mut w, &plan, false).unwrap();
            let bytes = finish_aligned(w);

            let mut r = BitReader::new(&bytes);
            let block = parse_intra_block_aic(&mut r, true, false).unwrap();
            let rec = aic_intra_reconstruct_coefficients(
                &block,
                mode,
                quant,
                Neighbour::None,
                Neighbour::None,
            );
            assert_eq!(rec, plan.rec, "mode {mode:?} closed-loop mismatch");
        }
    }

    /// A block whose residual is entirely inside the dead zone plans as
    /// not-coded (CBP bit 0) and reconstructs from the predictor alone.
    #[test]
    fn zero_residual_block_is_not_coded() {
        // With an available neighbour DC equal to the block's DC and no
        // AC energy, the residual quantises to all zero.
        let source = [10i16; COEFFS_PER_BLOCK];
        // fdct DC of a flat 10 block = 80. Provide a neighbour whose DC
        // is 80 so the predicted DC matches and the residual is ~0.
        let mut a = [0i32; COEFFS_PER_BLOCK];
        a[0] = 80;
        let plan = plan_intra_block_aic(
            &source,
            IntraMode::DcOnly,
            8,
            Neighbour::Available(&a),
            Neighbour::None,
            false,
        );
        assert!(!plan.coded, "matched-DC block should not be coded");
        // rec equals the pure-predictor reconstruction.
        let zero = [0i32; COEFFS_PER_BLOCK];
        let pred = reconstruct_intra_block_aic(
            &zero,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        assert_eq!(plan.rec, pred);
    }

    /// Under Modified Quantization mode a LEVEL beyond the baseline ±127
    /// range is emitted via the §T.4 EXTENDED-ESCAPE path and round-trips
    /// through the decoder's `parse_intra_block_aic`. Constructed
    /// deterministically so the extended path is always exercised.
    #[test]
    fn extended_level_round_trips_under_mq() {
        let mode = IntraMode::DcOnly;
        let quant = 4;
        // Hand-build a plan with a >127 magnitude LEVEL at a mid scan
        // position, then compute its true reconstruction.
        let mut levels = [0i16; COEFFS_PER_BLOCK];
        levels[0] = 60; // DC
        levels[5] = 300; // extended-range AC
        levels[9] = -200; // extended-range AC, negative
        let mut block = H263Block::empty();
        block.coefficients = levels;
        let rec = aic_intra_reconstruct_coefficients(
            &block,
            mode,
            quant,
            Neighbour::None,
            Neighbour::None,
        );
        let plan = AicBlockPlan {
            coded: true,
            levels,
            rec,
        };
        assert!(plan
            .levels
            .iter()
            .any(|&l| (l as i32).abs() > MAX_LEVEL_BASELINE));

        let mut w = BitWriter::new();
        write_intra_block_aic(&mut w, &plan, true).unwrap();
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let parsed = parse_intra_block_aic(&mut r, true, true).unwrap();
        let rec2 = aic_intra_reconstruct_coefficients(
            &parsed,
            mode,
            quant,
            Neighbour::None,
            Neighbour::None,
        );
        assert_eq!(rec2, plan.rec);
    }
}
