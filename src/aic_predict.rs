//! Annex I §I.3 — Advanced INTRA Coding DC/AC prediction reconstruction.
//!
//! When Advanced INTRA Coding (Annex I, AIC) is in use, an INTRA block's
//! final reconstructed coefficient values `RecC'(u,v)` are obtained by
//! adding an `INTRA_MODE`-dependent prediction contribution sourced from
//! the **already reconstructed** block immediately above (`RecA'`) and /
//! or the block immediately to the left (`RecB'`), then running the
//! result through `clipAC` for AC slots and `oddifyclipDC` for the DC
//! slot. The three INTRA_MODE rules are spelled out in §I.3, page 79:
//!
//! * **Mode 0** ([`crate::aic::IntraMode::DcOnly`]) — DC-only prediction.
//!   AC slots are not predicted. The DC slot is predicted from the
//!   average of `RecA'(0,0)` and `RecB'(0,0)` when both blocks A and B
//!   are INTRA and in the same video picture segment, from a single
//!   neighbour's DC if only one of A / B is available, and from the
//!   fixed value 1024 if neither is available.
//! * **Mode 1** ([`crate::aic::IntraMode::VerticalDcAc`]) — DC and the
//!   first row of AC coefficients are predicted from block A (the block
//!   above). If A is not available the prediction degenerates to a
//!   fixed-1024 DC predictor and no AC predictor.
//! * **Mode 2** ([`crate::aic::IntraMode::HorizontalDcAc`]) — DC and the
//!   first column of AC coefficients are predicted from block B (the
//!   block to the left). If B is not available, same fixed-1024 / no-AC
//!   degeneration as Mode 1.
//!
//! The §I.3 "same video picture segment" availability rule (page 78)
//! lives in the decode driver and is surfaced here as the
//! [`Neighbour::None`] / [`Neighbour::Available`] tag the caller passes
//! per neighbour. From this module's perspective an absent neighbour and
//! a present-but-INTER neighbour are equivalent — both surface as
//! [`Neighbour::None`].
//!
//! ## Layout convention
//!
//! All coefficient arrays here are **block-position** (`u, v`) layouts:
//! `index = v * 8 + u`, with `u = 0..=7` the horizontal index and
//! `v = 0..=7` the vertical index. This is the same convention as the
//! Figure 14 / Figure I.2 scan-target tables in [`crate::block`] and
//! [`crate::aic`], and it is **not** the zigzag-scan order that
//! [`crate::block_aic::parse_intra_block_aic`] produces. Callers must
//! scatter the parsed zigzag-order `LEVEL` values through the
//! [`crate::aic::scan_for_intra_mode`] permutation and then run
//! [`crate::aic_dequant::aic_dequant_coefficient`] per slot before
//! invoking the reconstruction here.
//!
//! `RecA'(u,v)` and `RecB'(u,v)` are the **final** reconstructed values
//! of the neighbour blocks — the caller is expected to have already run
//! the §I.3 reconstruction on them. They are passed in block-position
//! layout. Only the (0,0) slot of each is consulted for Mode 0; Mode 1
//! consults `RecA'(u, 0)` for `u = 0..=7`; Mode 2 consults `RecB'(0, v)`
//! for `v = 0..=7`. The other slots are read but multiplied by zero
//! (the prediction step contributes 0 for unpredicted slots) — they
//! never alter the output.
//!
//! ## Range
//!
//! The output of [`reconstruct_intra_block_aic`] is the §I.3 `RecC'`
//! array post-`clipAC` / `oddifyclipDC`: every AC slot is in
//! `[crate::aic_dequant::AIC_AC_REC_MIN, crate::aic_dequant::AIC_AC_REC_MAX]`
//! (`-2048..=+2047`) and the DC slot is in
//! `[crate::aic_dequant::AIC_DC_REC_MIN, crate::aic_dequant::AIC_DC_REC_MAX]`
//! (`0..=+2047`). The intermediate DC sum (residual + neighbour DC +
//! possibly `+1024`) is computed in `i32` and can transiently exceed
//! `i16`; the final clip pins it back into range.
//!
//! ## Deliberately out of scope (for this round)
//!
//! * The §I.3 "same video picture segment" availability test (within
//!   the picture; same GOB or no GOB header; same slice in Slice
//!   Structured mode). The caller computes this and passes the result
//!   as [`Neighbour::None`] / [`Neighbour::Available`].
//! * The macroblock-grid driver that walks the picture, accumulates
//!   reconstructed `RecA'` / `RecB'` arrays, dispatches the AIC scan,
//!   inverse-quant, prediction, and IDCT in sequence. That driver
//!   composes this module with [`crate::aic`], [`crate::aic_dequant`],
//!   [`crate::block_aic`], and [`crate::idct`]; it has its own round.
//! * The IDCT step — the output of this module is still
//!   frequency-domain coefficients. [`crate::idct::idct_8x8`] consumes
//!   them.

use crate::aic::IntraMode;
use crate::aic_dequant::{clip_ac, oddify_clip_dc};
use crate::block::COEFFS_PER_BLOCK;

/// Predictor source for one of the two §I.3 reference blocks (block A
/// immediately above, or block B immediately to the left of the current
/// block).
///
/// The two §I.3 conditions for a neighbour block to contribute a
/// predictor are (page 78):
///
/// 1. The neighbour is an **INTRA**-coded block, **and**
/// 2. The neighbour is in the **same video picture segment** as the
///    current block.
///
/// If both conditions are met the caller supplies the neighbour's
/// final-reconstructed `RecA'` / `RecB'` array; otherwise the caller
/// passes [`Neighbour::None`] and this module falls back to the §I.3
/// "no predictor" branches (1024 for DC, 0 for AC).
#[derive(Debug, Clone, Copy)]
pub enum Neighbour<'a> {
    /// Neighbour is unavailable: either out of picture, not INTRA-coded,
    /// or in a different video picture segment than the current block.
    None,
    /// Neighbour is INTRA-coded and in the same video picture segment.
    /// The contained array is the neighbour's final reconstructed
    /// `RecA'(u,v)` (for block A) or `RecB'(u,v)` (for block B) in
    /// block-position layout (`index = v * 8 + u`).
    Available(&'a [i32; COEFFS_PER_BLOCK]),
}

impl<'a> Neighbour<'a> {
    /// True when the neighbour is [`Neighbour::Available`].
    #[must_use]
    pub fn is_available(&self) -> bool {
        matches!(self, Neighbour::Available(_))
    }

    /// Read slot `(u, v)` from the neighbour, returning `0` when the
    /// neighbour is unavailable. Used by the per-mode AC predictor
    /// helpers; not normally called by external code.
    #[inline]
    fn slot(&self, u: usize, v: usize) -> i32 {
        match self {
            Neighbour::None => 0,
            Neighbour::Available(arr) => arr[v * 8 + u],
        }
    }
}

/// §I.3 "no-neighbour" fallback DC predictor.
///
/// The fixed value 1024 is used when neither block A nor block B is
/// available as an INTRA predictor in Mode 0, when block A is
/// unavailable in Mode 1, or when block B is unavailable in Mode 2.
/// (§I.3 page 78 — third paragraph of the Mode-0 description and the
/// "else" branches of Modes 1 and 2.)
pub const AIC_FALLBACK_DC_PREDICTOR: i32 = 1024;

/// Apply the §I.3 INTRA prediction reconstruction to a single INTRA
/// block's dequantized residual array.
///
/// `rec_c_residual` is the `RecC(u,v)` residual array (block-position
/// layout, `index = v * 8 + u`) — i.e. the output of
/// [`crate::aic_dequant::aic_dequant_coefficient`] applied slot-by-slot
/// to the parsed [`crate::block_aic::parse_intra_block_aic`] events,
/// after the [`crate::aic::scan_for_intra_mode`] permutation from
/// zigzag-scan order to block-position order has been applied.
///
/// `block_a` is the §I.3 `RecA'` neighbour (block immediately above
/// the current block) and `block_b` is `RecB'` (block immediately to
/// the left); both as [`Neighbour`] tags so the caller can encode the
/// "same video picture segment" availability rule.
///
/// `mode` is the INTRA_MODE decoded by [`crate::aic::decode_intra_mode`]
/// for the current macroblock.
///
/// Returns the final `RecC'(u,v)` array (block-position layout) with
/// `clipAC` applied to every AC slot and `oddifyclipDC` applied to the
/// DC slot, per §I.3 page 79.
#[must_use]
pub fn reconstruct_intra_block_aic(
    rec_c_residual: &[i32; COEFFS_PER_BLOCK],
    mode: IntraMode,
    block_a: Neighbour<'_>,
    block_b: Neighbour<'_>,
) -> [i32; COEFFS_PER_BLOCK] {
    match mode {
        IntraMode::DcOnly => reconstruct_mode0(rec_c_residual, block_a, block_b),
        IntraMode::VerticalDcAc => reconstruct_mode1(rec_c_residual, block_a),
        IntraMode::HorizontalDcAc => reconstruct_mode2(rec_c_residual, block_b),
    }
}

/// §I.3 Mode 0 — DC-only prediction reconstruction.
///
/// Per §I.3 page 79:
///
/// * Every AC slot `(u, v) != (0, 0)` is reconstructed as
///   `clipAC( RecC(u, v) )` — no AC prediction.
/// * The DC slot is reconstructed as `oddifyclipDC( tempDC )` with
///   `tempDC` computed by one of four branches:
///   * Both A and B available — `tempDC = RecC(0,0) + (RecA'(0,0) + RecB'(0,0)) / 2`
///     (truncating integer division).
///   * Only A available — `tempDC = RecC(0,0) + RecA'(0,0)`.
///   * Only B available — `tempDC = RecC(0,0) + RecB'(0,0)`.
///   * Neither available — `tempDC = RecC(0,0) + 1024`.
fn reconstruct_mode0(
    rec_c_residual: &[i32; COEFFS_PER_BLOCK],
    block_a: Neighbour<'_>,
    block_b: Neighbour<'_>,
) -> [i32; COEFFS_PER_BLOCK] {
    let mut out = [0i32; COEFFS_PER_BLOCK];
    // AC slots: clipAC( RecC(u, v) ).
    for idx in 1..COEFFS_PER_BLOCK {
        out[idx] = clip_ac(rec_c_residual[idx]);
    }
    // DC slot: oddifyclipDC( residual + predictor ).
    let rec_c00 = rec_c_residual[0];
    let temp_dc = match (block_a, block_b) {
        (Neighbour::Available(a), Neighbour::Available(b)) => {
            // §I.3: "average (with truncation) of the DC coefficients of
            // blocks A and B" — `(a + b) / 2` with truncation toward
            // zero, matching the spec's "/ defined as division by
            // truncation" convention (§I.3 page 79).
            let a00 = a[0];
            let b00 = b[0];
            rec_c00 + (a00 + b00) / 2
        }
        (Neighbour::Available(a), Neighbour::None) => rec_c00 + a[0],
        (Neighbour::None, Neighbour::Available(b)) => rec_c00 + b[0],
        (Neighbour::None, Neighbour::None) => rec_c00 + AIC_FALLBACK_DC_PREDICTOR,
    };
    out[0] = oddify_clip_dc(temp_dc);
    out
}

/// §I.3 Mode 1 — DC and AC prediction from the block above.
///
/// Per §I.3 page 79 (block A available branch):
///
/// * `tempDC = RecC(0,0) + RecA'(0,0)`
/// * `RecC'(u, 0) = clipAC( RecC(u, 0) + RecA'(u, 0) )` for `u = 1..=7`.
/// * `RecC'(u, v) = clipAC( RecC(u, v) )` for `v = 1..=7`, `u = 0..=7`.
/// * `RecC'(0, 0) = oddifyclipDC( tempDC )`.
///
/// Block-A-unavailable branch:
///
/// * `tempDC = RecC(0,0) + 1024`.
/// * `RecC'(u, v) = clipAC( RecC(u, v) )` for `(u, v) != (0, 0)`.
/// * `RecC'(0, 0) = oddifyclipDC( tempDC )`.
fn reconstruct_mode1(
    rec_c_residual: &[i32; COEFFS_PER_BLOCK],
    block_a: Neighbour<'_>,
) -> [i32; COEFFS_PER_BLOCK] {
    let mut out = [0i32; COEFFS_PER_BLOCK];
    // First row v = 0: AC slots u = 1..=7 get the RecA'(u, 0) predictor
    // when A is available, else no predictor.
    let a_available = block_a.is_available();
    for u in 1..8 {
        let residual = rec_c_residual[u];
        let predictor = if a_available { block_a.slot(u, 0) } else { 0 };
        out[u] = clip_ac(residual + predictor);
    }
    // Rows v = 1..=7: no AC prediction in Mode 1.
    for v in 1..8 {
        for u in 0..8 {
            let idx = v * 8 + u;
            out[idx] = clip_ac(rec_c_residual[idx]);
        }
    }
    // DC slot: oddifyclipDC( residual + predictor ).
    let predictor = if a_available {
        block_a.slot(0, 0)
    } else {
        AIC_FALLBACK_DC_PREDICTOR
    };
    out[0] = oddify_clip_dc(rec_c_residual[0] + predictor);
    out
}

/// §I.3 Mode 2 — DC and AC prediction from the block to the left.
///
/// Per §I.3 page 79 (block B available branch):
///
/// * `tempDC = RecC(0,0) + RecB'(0,0)`
/// * `RecC'(0, v) = clipAC( RecC(0, v) + RecB'(0, v) )` for `v = 1..=7`.
/// * `RecC'(u, v) = clipAC( RecC(u, v) )` for `u = 1..=7`, `v = 0..=7`.
/// * `RecC'(0, 0) = oddifyclipDC( tempDC )`.
///
/// Block-B-unavailable branch:
///
/// * `tempDC = RecC(0,0) + 1024`.
/// * `RecC'(u, v) = clipAC( RecC(u, v) )` for `(u, v) != (0, 0)`.
/// * `RecC'(0, 0) = oddifyclipDC( tempDC )`.
fn reconstruct_mode2(
    rec_c_residual: &[i32; COEFFS_PER_BLOCK],
    block_b: Neighbour<'_>,
) -> [i32; COEFFS_PER_BLOCK] {
    let mut out = [0i32; COEFFS_PER_BLOCK];
    let b_available = block_b.is_available();
    // First column u = 0: AC slots v = 1..=7 get the RecB'(0, v) predictor
    // when B is available, else no predictor.
    for v in 1..8 {
        let idx = v * 8;
        let residual = rec_c_residual[idx];
        let predictor = if b_available { block_b.slot(0, v) } else { 0 };
        out[idx] = clip_ac(residual + predictor);
    }
    // Columns u = 1..=7: no AC prediction in Mode 2.
    for v in 0..8 {
        for u in 1..8 {
            let idx = v * 8 + u;
            out[idx] = clip_ac(rec_c_residual[idx]);
        }
    }
    // DC slot: oddifyclipDC( residual + predictor ).
    let predictor = if b_available {
        block_b.slot(0, 0)
    } else {
        AIC_FALLBACK_DC_PREDICTOR
    };
    out[0] = oddify_clip_dc(rec_c_residual[0] + predictor);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aic_dequant::{AIC_AC_REC_MAX, AIC_AC_REC_MIN, AIC_DC_REC_MAX, AIC_DC_REC_MIN};

    /// Build a residual array with a single non-zero slot at
    /// block-position `(u, v)`. Convenience for tests.
    fn one_slot(u: usize, v: usize, value: i32) -> [i32; COEFFS_PER_BLOCK] {
        let mut r = [0i32; COEFFS_PER_BLOCK];
        r[v * 8 + u] = value;
        r
    }

    /// Build a constant-DC neighbour reference array.
    fn neighbour_with_dc(dc: i32) -> [i32; COEFFS_PER_BLOCK] {
        let mut r = [0i32; COEFFS_PER_BLOCK];
        r[0] = dc;
        r
    }

    /// Mode 0, no neighbours: DC residual passes through
    /// `oddifyclipDC(residual + 1024)`; AC slots pass through `clipAC`.
    #[test]
    fn mode0_no_neighbours_uses_1024_predictor() {
        let residual = one_slot(0, 0, 5);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::None,
            Neighbour::None,
        );
        // residual + 1024 = 1029, which is odd, so oddifyclipDC returns 1029.
        assert_eq!(out[0], 1029);
        // AC slots are all zero (residual is zero, clipAC(0) = 0).
        for (idx, &v) in out.iter().enumerate().skip(1) {
            assert_eq!(v, 0, "idx={idx}");
        }
    }

    /// Mode 0, only block A available: DC predictor is RecA'(0,0) alone.
    #[test]
    fn mode0_only_block_a() {
        let residual = one_slot(0, 0, 10);
        let a = neighbour_with_dc(100);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        // residual + 100 = 110, even → oddifyclipDC bumps to 111.
        assert_eq!(out[0], 111);
    }

    /// Mode 0, only block B available: DC predictor is RecB'(0,0) alone.
    #[test]
    fn mode0_only_block_b() {
        let residual = one_slot(0, 0, 7);
        let b = neighbour_with_dc(200);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::None,
            Neighbour::Available(&b),
        );
        // residual + 200 = 207, odd → oddifyclipDC returns 207.
        assert_eq!(out[0], 207);
    }

    /// Mode 0, both A and B available: DC predictor is `(A + B) / 2`
    /// with truncation toward zero. Spec: §I.3 page 79.
    #[test]
    fn mode0_both_neighbours_averages_with_truncation() {
        let residual = one_slot(0, 0, 0);
        let a = neighbour_with_dc(100);
        let b = neighbour_with_dc(50);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::Available(&b),
        );
        // (100 + 50) / 2 = 75; residual is 0; tempDC = 75; odd → 75.
        assert_eq!(out[0], 75);
    }

    /// Mode 0, both A and B available: truncation rounds toward zero
    /// for odd sums.
    #[test]
    fn mode0_both_neighbours_truncation_toward_zero() {
        let residual = one_slot(0, 0, 1);
        let a = neighbour_with_dc(101); // 101 + 0 = 101, /2 truncates to 50
        let b = neighbour_with_dc(0);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::Available(&b),
        );
        // (101 + 0) / 2 = 50 (truncate); residual = 1; tempDC = 51; odd → 51.
        assert_eq!(out[0], 51);
    }

    /// Mode 0 never touches AC slots from the neighbours: a neighbour
    /// AC value at (3, 4) must NOT alter the current block's (3, 4)
    /// output (which is just `clipAC(residual)`).
    #[test]
    fn mode0_ac_slots_are_clipped_residuals_only() {
        let residual = one_slot(3, 4, 100);
        let mut a = [0i32; COEFFS_PER_BLOCK];
        a[4 * 8 + 3] = 999; // garbage at the same (u, v) slot
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        assert_eq!(out[4 * 8 + 3], 100);
    }

    /// Mode 1, block A available: DC and the first row of AC slots are
    /// predicted from `RecA'(u, 0)`; the rest are bare-residual clips.
    #[test]
    fn mode1_block_a_available_predicts_dc_and_first_row() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[0] = 10; // DC residual
        residual[1] = 20; // (1, 0) AC residual
        residual[7] = 30; // (7, 0) AC residual
        residual[8] = 40; // (0, 1) — second row, should NOT be predicted
        let mut a = [0i32; COEFFS_PER_BLOCK];
        a[0] = 50; // RecA'(0, 0)
        a[1] = 60; // RecA'(1, 0)
        a[7] = 70; // RecA'(7, 0)
        a[8] = 999; // RecA'(0, 1) — should be ignored in Mode 1
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::VerticalDcAc,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        // DC: 10 + 50 = 60, even → oddifyclipDC(60) = 61.
        assert_eq!(out[0], 61);
        // (1, 0): clipAC(20 + 60) = 80.
        assert_eq!(out[1], 80);
        // (7, 0): clipAC(30 + 70) = 100.
        assert_eq!(out[7], 100);
        // (0, 1): clipAC(40) = 40 (no AC prediction beyond the first row).
        assert_eq!(out[8], 40);
    }

    /// Mode 1, block A NOT available: DC predictor is 1024, no AC
    /// prediction. AC slots are pure `clipAC(residual)`.
    #[test]
    fn mode1_block_a_unavailable_falls_back_to_1024() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[0] = 5;
        residual[1] = 100;
        residual[8] = 50;
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::VerticalDcAc,
            Neighbour::None,
            Neighbour::None,
        );
        // DC: 5 + 1024 = 1029, odd → oddifyclipDC = 1029.
        assert_eq!(out[0], 1029);
        // (1, 0): clipAC(100) = 100.
        assert_eq!(out[1], 100);
        // (0, 1): clipAC(50) = 50.
        assert_eq!(out[8], 50);
    }

    /// Mode 2, block B available: DC and the first column of AC slots
    /// are predicted from `RecB'(0, v)`; the rest are bare-residual
    /// clips.
    #[test]
    fn mode2_block_b_available_predicts_dc_and_first_column() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[0] = 10; // DC
        residual[8] = 20; // (0, 1) AC residual
        residual[7 * 8] = 30; // (0, 7) AC residual
        residual[1] = 40; // (1, 0) — first row, should NOT be predicted in Mode 2
        let mut b = [0i32; COEFFS_PER_BLOCK];
        b[0] = 50; // RecB'(0, 0)
        b[8] = 60; // RecB'(0, 1)
        b[7 * 8] = 70; // RecB'(0, 7)
        b[1] = 999; // RecB'(1, 0) — should be ignored in Mode 2
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::HorizontalDcAc,
            Neighbour::None,
            Neighbour::Available(&b),
        );
        // DC: 10 + 50 = 60, even → 61.
        assert_eq!(out[0], 61);
        // (0, 1): clipAC(20 + 60) = 80.
        assert_eq!(out[8], 80);
        // (0, 7): clipAC(30 + 70) = 100.
        assert_eq!(out[7 * 8], 100);
        // (1, 0): clipAC(40) = 40 (no AC prediction beyond the first column).
        assert_eq!(out[1], 40);
    }

    /// Mode 2, block B NOT available: DC predictor is 1024, no AC
    /// prediction.
    #[test]
    fn mode2_block_b_unavailable_falls_back_to_1024() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[0] = 3;
        residual[8] = 50;
        residual[1] = 100;
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::HorizontalDcAc,
            Neighbour::None,
            Neighbour::None,
        );
        // DC: 3 + 1024 = 1027, odd → 1027.
        assert_eq!(out[0], 1027);
        // (0, 1): clipAC(50) = 50.
        assert_eq!(out[8], 50);
        // (1, 0): clipAC(100) = 100.
        assert_eq!(out[1], 100);
    }

    /// `clipAC` is applied to AC outputs: a residual + predictor that
    /// exceeds +2047 must saturate to +2047.
    #[test]
    fn ac_output_saturates_at_upper_clip() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[1] = 2000;
        let mut a = [0i32; COEFFS_PER_BLOCK];
        a[1] = 1000;
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::VerticalDcAc,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        // 2000 + 1000 = 3000 > 2047, saturates.
        assert_eq!(out[1], AIC_AC_REC_MAX);
    }

    /// `clipAC` lower saturation: a residual + predictor that drops
    /// below -2048 must saturate to -2048.
    #[test]
    fn ac_output_saturates_at_lower_clip() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[8] = -2000;
        let mut b = [0i32; COEFFS_PER_BLOCK];
        b[8] = -1000;
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::HorizontalDcAc,
            Neighbour::None,
            Neighbour::Available(&b),
        );
        // -2000 + -1000 = -3000 < -2048, saturates.
        assert_eq!(out[8], AIC_AC_REC_MIN);
    }

    /// `oddifyclipDC` is applied to the DC output: an even sum gets
    /// bumped to odd before clipping; the DC clip range is `[0, 2047]`
    /// not `[-2048, 2047]`.
    #[test]
    fn dc_output_oddifies_and_clips_to_dc_range() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[0] = -500;
        let a = neighbour_with_dc(-1000);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        // -500 + -1000 = -1500, even → oddifyclipDC(-1500) = clipDC(-1499) = 0.
        assert_eq!(out[0], AIC_DC_REC_MIN);
    }

    /// DC output upper clip: a large positive sum saturates to 2047
    /// (with potential parity bump).
    #[test]
    fn dc_output_upper_clip() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[0] = 2000;
        let a = neighbour_with_dc(2000);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        // 2000 + 2000 = 4000, even → 4001 → clipDC = 2047.
        assert_eq!(out[0], AIC_DC_REC_MAX);
    }

    /// All-zero residual + no neighbours: DC = oddifyclipDC(0 + 1024) =
    /// 1025 (1024 is even, bumped to 1025); all AC = 0.
    #[test]
    fn all_zero_residual_no_neighbours() {
        let residual = [0i32; COEFFS_PER_BLOCK];
        for mode in [
            IntraMode::DcOnly,
            IntraMode::VerticalDcAc,
            IntraMode::HorizontalDcAc,
        ] {
            let out =
                reconstruct_intra_block_aic(&residual, mode, Neighbour::None, Neighbour::None);
            assert_eq!(out[0], 1025, "mode={mode:?}");
            for (idx, &v) in out.iter().enumerate().skip(1) {
                assert_eq!(v, 0, "mode={mode:?} idx={idx}");
            }
        }
    }

    /// `Neighbour::None` from "block is INTER or in different segment"
    /// is observationally identical to "block doesn't exist": no
    /// availability bit leaks through the API.
    #[test]
    fn neighbour_none_is_observationally_identical_regardless_of_reason() {
        let mut residual = [0i32; COEFFS_PER_BLOCK];
        residual[0] = 12;
        let out_a = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::None,
            Neighbour::None,
        );
        let out_b = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::None,
            Neighbour::None,
        );
        assert_eq!(out_a, out_b);
    }

    /// `Neighbour::is_available` reflects the variant.
    #[test]
    fn neighbour_is_available_query() {
        let arr = [0i32; COEFFS_PER_BLOCK];
        assert!(!Neighbour::None.is_available());
        assert!(Neighbour::Available(&arr).is_available());
    }

    /// Mode 1 with A available but residual zero everywhere: output AC
    /// first row equals `clipAC(RecA'(u, 0))` for `u = 1..=7`, output DC
    /// equals `oddifyclipDC(RecA'(0, 0))`.
    #[test]
    fn mode1_zero_residual_passes_predictor_through() {
        let residual = [0i32; COEFFS_PER_BLOCK];
        let mut a = [0i32; COEFFS_PER_BLOCK];
        a[0] = 7; // DC predictor — odd
        a[1] = 11;
        a[2] = -50;
        a[7] = 2047;
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::VerticalDcAc,
            Neighbour::Available(&a),
            Neighbour::None,
        );
        assert_eq!(out[0], 7);
        assert_eq!(out[1], 11);
        assert_eq!(out[2], -50);
        assert_eq!(out[7], 2047);
        // Slots outside the first row are zero (residual is 0, clipAC(0) = 0).
        for v in 1..8 {
            for u in 0..8 {
                assert_eq!(out[v * 8 + u], 0, "v={v} u={u}");
            }
        }
    }

    /// Mode 2 with B available but residual zero everywhere: output AC
    /// first column equals `clipAC(RecB'(0, v))` for `v = 1..=7`, output
    /// DC equals `oddifyclipDC(RecB'(0, 0))`.
    #[test]
    fn mode2_zero_residual_passes_predictor_through() {
        let residual = [0i32; COEFFS_PER_BLOCK];
        let mut b = [0i32; COEFFS_PER_BLOCK];
        b[0] = 9; // DC predictor — odd
        b[8] = 11;
        b[2 * 8] = -50;
        b[7 * 8] = -2048;
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::HorizontalDcAc,
            Neighbour::None,
            Neighbour::Available(&b),
        );
        assert_eq!(out[0], 9);
        assert_eq!(out[8], 11);
        assert_eq!(out[2 * 8], -50);
        assert_eq!(out[7 * 8], -2048);
        // Slots outside the first column are zero.
        for v in 0..8 {
            for u in 1..8 {
                assert_eq!(out[v * 8 + u], 0, "v={v} u={u}");
            }
        }
    }

    /// Cross-mode invariant: all output AC values lie within
    /// `[AIC_AC_REC_MIN, AIC_AC_REC_MAX]`, all DC outputs lie within
    /// `[AIC_DC_REC_MIN, AIC_DC_REC_MAX]`. Exhaustive over a sample of
    /// residual / neighbour patterns.
    #[test]
    fn output_ranges_respect_clip_bounds() {
        let extreme_residual = [3000i32; COEFFS_PER_BLOCK];
        let extreme_a = [3000i32; COEFFS_PER_BLOCK];
        let extreme_b = [-3000i32; COEFFS_PER_BLOCK];
        for mode in [
            IntraMode::DcOnly,
            IntraMode::VerticalDcAc,
            IntraMode::HorizontalDcAc,
        ] {
            let out = reconstruct_intra_block_aic(
                &extreme_residual,
                mode,
                Neighbour::Available(&extreme_a),
                Neighbour::Available(&extreme_b),
            );
            assert!(
                (AIC_DC_REC_MIN..=AIC_DC_REC_MAX).contains(&out[0]),
                "mode={mode:?} dc={}",
                out[0]
            );
            for (idx, &val) in out.iter().enumerate().skip(1) {
                assert!(
                    (AIC_AC_REC_MIN..=AIC_AC_REC_MAX).contains(&val),
                    "mode={mode:?} idx={idx} val={val}",
                );
            }
        }
    }

    /// The §I.3 "neither neighbour" Mode 0 branch and the §I.3 Mode 1
    /// "block A unavailable" branch produce the same DC output for a
    /// given residual (both use the `+1024` fallback). Same for Mode 2.
    #[test]
    fn fallback_dc_predictor_is_consistent_across_modes() {
        let residual = one_slot(0, 0, 17);
        let out0 = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::None,
            Neighbour::None,
        );
        let out1 = reconstruct_intra_block_aic(
            &residual,
            IntraMode::VerticalDcAc,
            Neighbour::None,
            Neighbour::None,
        );
        let out2 = reconstruct_intra_block_aic(
            &residual,
            IntraMode::HorizontalDcAc,
            Neighbour::None,
            Neighbour::None,
        );
        // All three should produce the same DC: oddifyclipDC(17 + 1024) = 1041
        // (1041 is odd, no bump).
        assert_eq!(out0[0], 1041);
        assert_eq!(out1[0], 1041);
        assert_eq!(out2[0], 1041);
    }

    /// `AIC_FALLBACK_DC_PREDICTOR` constant matches the §I.3 spec
    /// value `1024`. Sanity guard for the public constant.
    #[test]
    fn fallback_dc_predictor_constant_is_1024() {
        assert_eq!(AIC_FALLBACK_DC_PREDICTOR, 1024);
    }

    /// Mode 0 truncation-toward-zero spot-check: a negative sum from the
    /// `(A + B) / 2` average rounds toward zero (Rust's `i32 / i32`
    /// matches the spec's "/" truncation convention).
    #[test]
    fn mode0_truncation_handles_negative_sums() {
        // (A, B) = (-3, 0): sum = -3, /2 = -1 (Rust truncates toward 0).
        let residual = one_slot(0, 0, 0);
        let a = neighbour_with_dc(-3);
        let b = neighbour_with_dc(0);
        let out = reconstruct_intra_block_aic(
            &residual,
            IntraMode::DcOnly,
            Neighbour::Available(&a),
            Neighbour::Available(&b),
        );
        // (-3 + 0) / 2 = -1 in truncation-toward-zero; tempDC = -1.
        // oddifyclipDC(-1) = clipDC(-1) (odd, no bump) = 0.
        assert_eq!(out[0], 0);
    }
}
