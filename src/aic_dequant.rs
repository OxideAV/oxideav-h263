//! Annex I §I.3 — Advanced INTRA Coding modified inverse quantization
//! and reconstruction-clip primitives.
//!
//! Implements the §I.3 reconstruction-without-dead-zone formula for
//! INTRA coefficient residuals, plus the two §I.3 clipping helpers and
//! the DC "oddification" rule. These are pure transformations that
//! operate on a single coefficient value at a time. They are the
//! companion to the round-14 Table I.2 separate INTRA-coefficient VLC
//! ([`crate::intra_tcoef`]); together they cover the §I.3 path from
//! parsed `(RUN, LEVEL)` event to a reconstructed coefficient residual
//! `RecC(u,v)`.
//!
//! What this module does **not** yet provide (deferred until the
//! macroblock-grid driver supplies neighbour blocks):
//!
//! * The §I.3 DC/AC prediction reconstruction itself (the three
//!   INTRA_MODE-dependent rules that add `RecA'(u,v)` / `RecB'(u,v)`
//!   contributions to `RecC(u,v)` before the final clip). The
//!   reconstruction equations need the live neighbour blocks plus the
//!   §I.3 "same video picture segment" availability rule, which both
//!   belong in the decode driver. The primitives in this module are
//!   the building blocks the driver will compose.
//! * The §I.3 line-4214 reframing of MCBPC / CBPY — i.e. the rule that
//!   in Advanced INTRA Coding mode a zero INTRADC is no longer signalled
//!   as a separate field and instead joins the per-block coefficient
//!   stream. That reframing is a parser-side concern, not a dequant one.
//!
//! ## §I.3 reconstruction formula (no dead-zone)
//!
//! For every coefficient slot (DC and AC alike) the AIC reconstruction
//! residual is computed without the §6.2.1 odd-fier dead-zone:
//!
//! ```text
//!   RecC(u,v) = 2 · QUANT · LEVEL(u,v)              u = 0..=7, v = 0..=7
//! ```
//!
//! `LEVEL(u,v)` carries the parsed two's-complement sign. The output is
//! the residual *before* prediction-addition and *before* the final
//! `oddifyclipDC` / `clipAC` step (see [`oddify_clip_dc`] / [`clip_ac`]).
//!
//! Compare with the round-1 baseline §6.2.1 formula
//! (see [`crate::dequant::dequantise_ac`]):
//!
//! ```text
//!   |REC| = QUANT · (2 · |LEVEL| + 1)            (odd QUANT)
//!   |REC| = QUANT · (2 · |LEVEL| + 1) - 1        (even QUANT)
//! ```
//!
//! The Annex I formula is strictly smaller in magnitude (no `+1` /
//! `-1`) and is signed-symmetric (linear in `LEVEL`, no parity
//! dependence on `QUANT`).
//!
//! ## §I.3 clipping (DC range vs AC range)
//!
//! After prediction is added, the spec specifies two clip functions:
//!
//! * `clipAC(x)` clips to `[-2048, +2047]` — applied to every
//!   coefficient except the DC slot (post-prediction).
//! * `clipDC(x)` clips to `[0, +2047]` — applied to the DC slot
//!   (post-prediction, post-oddification).
//!
//! The DC slot additionally goes through an "oddification" step that
//! protects against IDCT-mismatch sensitivity around the (0,0) / (0,4)
//! / (4,0) / (4,4) basis-pattern resonances:
//!
//! ```text
//!   oddifyclipDC(x):
//!       if x is even  -> clipDC(x + 1)
//!       else          -> clipDC(x)
//! ```
//!
//! (Spec §I.3, page 78 / page 79.)

/// §I.3 AC reconstruction-clip lower bound (`clipAC` range start).
pub const AIC_AC_REC_MIN: i32 = -2048;

/// §I.3 AC reconstruction-clip upper bound (`clipAC` range end).
pub const AIC_AC_REC_MAX: i32 = 2047;

/// §I.3 DC reconstruction-clip lower bound (`clipDC` range start).
///
/// Note the asymmetry: DC clips to a *non-negative* range, while AC
/// clips to a signed range. The DC oddification step runs *before*
/// clipping, so `oddify_clip_dc` may bump a `-1` value up to `0`.
pub const AIC_DC_REC_MIN: i32 = 0;

/// §I.3 DC reconstruction-clip upper bound (`clipDC` range end).
pub const AIC_DC_REC_MAX: i32 = 2047;

/// §I.3 modified inverse-quantisation formula
/// `RecC(u,v) = 2 · QUANT · LEVEL(u,v)`.
///
/// `quant` must be in `1..=31` per the §5.2.6 / §5.3.6 GQUANT / DQUANT
/// legal range; out-of-range values are clamped to that interval
/// (defence-in-depth — the caller is expected to have validated against
/// [`crate::Error::InvalidQuantiser`]). `level` is the parsed two's-
/// complement signed coefficient; the output preserves its sign exactly.
///
/// The output is the §I.3 `RecC(u,v)` *residual*. Per §I.3 (page 79),
/// the final coefficient value `RecC'(u,v)` is obtained by adding an
/// INTRA_MODE-dependent predictor sourced from the block above
/// (`RecA'`) and / or the block to the left (`RecB'`) and then running
/// the result through `clipAC` (AC slots) or `oddifyclipDC` (the DC
/// slot, post-summation). Those final steps need the live neighbour
/// blocks, which the macroblock-grid driver supplies.
///
/// Returned as `i32` because intermediate sums during DC prediction
/// (`RecC(0,0) + RecA'(0,0) + 1024`) can transiently exceed `i16` while
/// the prediction step composes; [`clip_ac`] / [`oddify_clip_dc`] pin
/// the final value back into a representable range.
#[inline]
#[must_use]
pub fn aic_dequant_coefficient(level: i16, quant: u8) -> i32 {
    let q = quant.clamp(1, 31) as i32;
    2 * q * (level as i32)
}

/// §I.3 `clipAC(x)` — clip a reconstructed AC coefficient to the
/// signed range `[-2048, +2047]`.
///
/// Applied per §I.3 to every coefficient *except* the DC slot, after
/// the §I.3 prediction-residual sum and *not* to the prediction-mode-0
/// AC slots (which are not predicted in mode 0 and therefore have no
/// predictor added — the clip is still applied to the bare residual,
/// per the spec text "Mode 0: RecC'(u,v) = clipAC(RecC(u,v))").
#[inline]
#[must_use]
pub fn clip_ac(x: i32) -> i32 {
    x.clamp(AIC_AC_REC_MIN, AIC_AC_REC_MAX)
}

/// §I.3 `clipDC(x)` — clip a DC coefficient to the non-negative range
/// `[0, +2047]`.
///
/// Internal helper for [`oddify_clip_dc`]; the spec never invokes
/// `clipDC` without first running `oddify`, so the public surface is
/// [`oddify_clip_dc`].
#[inline]
#[must_use]
fn clip_dc(x: i32) -> i32 {
    x.clamp(AIC_DC_REC_MIN, AIC_DC_REC_MAX)
}

/// §I.3 `oddifyclipDC(x)` — the combined "oddification + clipDC" step
/// applied to the DC coefficient (slot `(0, 0)`) after the §I.3
/// prediction-residual sum.
///
/// ```text
///   if x is even -> clipDC(x + 1)
///   if x is odd  -> clipDC(x)
/// ```
///
/// The spec text (§I.3 page 78) motivates the oddification: a DC value
/// of the form `8k + 4` IDCTs to a constant `k + 0.5`, which rounds
/// inconsistently between conforming IDCTs. Forcing the DC slot to an
/// odd value at the dequant stage breaks that resonance.
///
/// Note the clip happens *after* the parity adjustment: an even DC of
/// `2047` becomes `clipDC(2048) = 2047` (it is not bumped past the
/// upper clip), and an even DC of `-1` becomes `clipDC(0) = 0`. This
/// matches the spec's literal expansion `result = clipDC(x + 1)`.
#[inline]
#[must_use]
pub fn oddify_clip_dc(x: i32) -> i32 {
    let adjusted = if (x & 1) == 0 { x + 1 } else { x };
    clip_dc(adjusted)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §I.3 formula spot-check: QUANT = 1, LEVEL = 1 → 2 · 1 · 1 = 2.
    #[test]
    fn dequant_simple() {
        assert_eq!(aic_dequant_coefficient(1, 1), 2);
    }

    /// §I.3 formula is sign-symmetric: LEVEL = -1 → -2.
    #[test]
    fn dequant_negative_level() {
        assert_eq!(aic_dequant_coefficient(-1, 1), -2);
    }

    /// LEVEL = 0 always reconstructs to 0 regardless of QUANT.
    #[test]
    fn dequant_zero_level() {
        for q in 1u8..=31 {
            assert_eq!(aic_dequant_coefficient(0, q), 0, "q={q}");
        }
    }

    /// §I.3 reconstruction is strictly even-valued (linear in LEVEL,
    /// pre-multiplied by 2). Contrast with §6.2.1 baseline which is
    /// strictly odd-valued by construction.
    #[test]
    fn dequant_always_even() {
        for q in 1u8..=31 {
            for level in -127i16..=127 {
                let r = aic_dequant_coefficient(level, q);
                assert!((r & 1) == 0, "q={q} level={level} rec={r} should be even",);
            }
        }
    }

    /// §I.3 is linear in LEVEL: doubling LEVEL doubles the output, sign
    /// flip of LEVEL flips the output sign.
    #[test]
    fn dequant_linear_in_level() {
        for q in [1u8, 7, 16, 31] {
            for level in [1i16, 3, 17, 127] {
                let a = aic_dequant_coefficient(level, q);
                let b = aic_dequant_coefficient(2 * level, q);
                assert_eq!(b, 2 * a, "q={q} level={level}");
                let c = aic_dequant_coefficient(-level, q);
                assert_eq!(c, -a, "q={q} level={level}");
            }
        }
    }

    /// §I.3 is linear in QUANT: at fixed LEVEL, doubling QUANT doubles
    /// the output magnitude.
    #[test]
    fn dequant_linear_in_quant() {
        for level in [-50i16, -1, 1, 50, 127] {
            for q in [1u8, 2, 5, 7, 11] {
                let a = aic_dequant_coefficient(level, q);
                let b = aic_dequant_coefficient(level, 2 * q);
                assert_eq!(b, 2 * a, "q={q} level={level}");
            }
        }
    }

    /// §I.3 spot-check at the maximum legal QUANT + max-magnitude
    /// in-spec LEVEL: QUANT = 31, LEVEL = 127 → 2 · 31 · 127 = 7874.
    /// Note this exceeds the post-clip range; the dequant primitive
    /// itself does *not* clip — `clipAC` does that downstream after
    /// prediction is added.
    #[test]
    fn dequant_extreme() {
        assert_eq!(aic_dequant_coefficient(127, 31), 7874);
        assert_eq!(aic_dequant_coefficient(-127, 31), -7874);
    }

    /// §I.3 vs §6.2.1: the Annex I residual is strictly smaller in
    /// magnitude than the baseline H.261-style odd-fier output, for
    /// every legal QUANT × non-zero LEVEL pair. (The baseline adds
    /// `QUANT` or `QUANT - 1` of "dead-zone" gap on top of the linear
    /// `2 · QUANT · |LEVEL|` term.)
    #[test]
    fn aic_residual_smaller_than_baseline() {
        for q in 1u8..=31 {
            for level in 1i16..=127 {
                let aic = aic_dequant_coefficient(level, q).unsigned_abs();
                // Baseline §6.2.1 |REC| (positive LEVEL branch).
                let q32 = q as u32;
                let baseline = q32 * (2 * level as u32 + 1) - if q & 1 == 0 { 1 } else { 0 };
                assert!(
                    aic < baseline,
                    "q={q} level={level} aic={aic} baseline={baseline}",
                );
            }
        }
    }

    /// §I.3 QUANT clamp: zero or out-of-range QUANT is clamped to
    /// `1..=31` (defence in depth).
    #[test]
    fn dequant_quant_clamped() {
        // QUANT = 0 clamps to 1.
        assert_eq!(aic_dequant_coefficient(5, 0), aic_dequant_coefficient(5, 1));
        // QUANT = 200 clamps to 31.
        assert_eq!(
            aic_dequant_coefficient(5, 200),
            aic_dequant_coefficient(5, 31)
        );
    }

    /// `clip_ac` is the identity inside the in-range interval.
    #[test]
    fn clip_ac_identity_inside_range() {
        for x in [AIC_AC_REC_MIN, -1024, -1, 0, 1, 1024, AIC_AC_REC_MAX] {
            assert_eq!(clip_ac(x), x, "x={x}");
        }
    }

    /// `clip_ac` pins values above the upper bound to +2047.
    #[test]
    fn clip_ac_upper_saturation() {
        assert_eq!(clip_ac(AIC_AC_REC_MAX + 1), AIC_AC_REC_MAX);
        assert_eq!(clip_ac(10_000), AIC_AC_REC_MAX);
    }

    /// `clip_ac` pins values below the lower bound to -2048.
    #[test]
    fn clip_ac_lower_saturation() {
        assert_eq!(clip_ac(AIC_AC_REC_MIN - 1), AIC_AC_REC_MIN);
        assert_eq!(clip_ac(-10_000), AIC_AC_REC_MIN);
    }

    /// `oddify_clip_dc` leaves odd values untouched inside the range.
    #[test]
    fn oddify_clip_dc_odd_inside_range() {
        for x in [1, 3, 5, 7, 11, 1023, 1025, 2045, 2047] {
            assert_eq!(oddify_clip_dc(x), x, "x={x}");
        }
    }

    /// `oddify_clip_dc` bumps even values by +1 then clips.
    #[test]
    fn oddify_clip_dc_even_bumped() {
        assert_eq!(oddify_clip_dc(0), 1);
        assert_eq!(oddify_clip_dc(2), 3);
        assert_eq!(oddify_clip_dc(1024), 1025);
        assert_eq!(oddify_clip_dc(2046), 2047);
    }

    /// `oddify_clip_dc` upper saturation: an even input above 2047
    /// is bumped by +1 and then clipped down to 2047; an odd input
    /// above 2047 is clipped directly to 2047.
    #[test]
    fn oddify_clip_dc_upper_saturation() {
        // Even input above upper clip after bump: 2048 -> bump to 2049 -> clip to 2047.
        assert_eq!(oddify_clip_dc(2048), AIC_DC_REC_MAX);
        // Even input far above: 5000 -> bump to 5001 -> clip to 2047.
        assert_eq!(oddify_clip_dc(5000), AIC_DC_REC_MAX);
        // Odd input above: 5001 -> no bump -> clip to 2047.
        assert_eq!(oddify_clip_dc(5001), AIC_DC_REC_MAX);
    }

    /// `oddify_clip_dc` lower saturation: even inputs below the clip
    /// are bumped by +1 then clipped to 0; odd inputs (including -1)
    /// are clipped directly to 0.
    #[test]
    fn oddify_clip_dc_lower_saturation() {
        assert_eq!(oddify_clip_dc(-1), AIC_DC_REC_MIN);
        assert_eq!(oddify_clip_dc(-2), AIC_DC_REC_MIN);
        assert_eq!(oddify_clip_dc(-1000), AIC_DC_REC_MIN);
        assert_eq!(oddify_clip_dc(-999), AIC_DC_REC_MIN);
    }

    /// `oddify_clip_dc` invariant: every output in the in-range
    /// interval is odd. Outputs at the boundaries (0 or 2047) escape
    /// the parity guarantee because the clip happens last; 0 is even,
    /// 2047 is odd. The spec's intent is "no IDCT-mismatch-prone
    /// even DC values *that survive the clip*" — within
    /// `1..=2047`, every output is odd.
    #[test]
    fn oddify_clip_dc_in_range_outputs_are_odd_or_boundary() {
        // Sweep every potentially-meaningful x, including the boundary.
        for x in -100i32..=3000 {
            let y = oddify_clip_dc(x);
            assert!(
                (AIC_DC_REC_MIN..=AIC_DC_REC_MAX).contains(&y),
                "x={x} y={y} out of range",
            );
            // Either y is odd, or y is at the lower clip boundary (0)
            // because oddification of -1 hits clipDC(-1) -> 0.
            assert!(
                y == AIC_DC_REC_MIN || (y & 1) == 1,
                "x={x} y={y} expected odd in range",
            );
        }
    }

    /// Cross-check `oddify_clip_dc` against the spec's literal
    /// pseudocode (verbatim translation from §I.3 page 78).
    #[test]
    fn oddify_clip_dc_matches_spec_pseudocode() {
        for x in -3000i32..=3000 {
            let expected = if (x & 1) == 0 {
                clip_dc(x + 1)
            } else {
                clip_dc(x)
            };
            assert_eq!(oddify_clip_dc(x), expected, "x={x}");
        }
    }

    /// `clip_dc` (internal) sanity: identity in-range, saturates outside.
    #[test]
    fn clip_dc_basic() {
        assert_eq!(clip_dc(0), 0);
        assert_eq!(clip_dc(AIC_DC_REC_MAX), AIC_DC_REC_MAX);
        assert_eq!(clip_dc(1234), 1234);
        assert_eq!(clip_dc(-1), 0);
        assert_eq!(clip_dc(-9999), 0);
        assert_eq!(clip_dc(AIC_DC_REC_MAX + 1), AIC_DC_REC_MAX);
        assert_eq!(clip_dc(99999), AIC_DC_REC_MAX);
    }
}
