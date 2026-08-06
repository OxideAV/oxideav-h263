//! Encoder **rate control** — an Annex B HRD-regulated bit-budget loop.
//!
//! Two pure components compose into the closed-loop rate-controlled
//! sequence encoder
//! ([`crate::encoder::encode_sequence_rate_controlled`]):
//!
//! * [`HrdModel`] — the **Annex B Hypothetical Reference Decoder**
//!   buffer simulation. Bits arrive at the channel rate `R`; the
//!   buffer is examined at picture clock intervals (§B.4) and the
//!   earliest complete coded picture is removed instantaneously;
//!   *immediately after removal the occupancy must be less than
//!   `B = 4 · Rmax / PCF`* (§B.2 / §B.4). The model reports whether a
//!   picture kept the stream conformant, and how much slack the §B.4
//!   inequality `d(n+1) ≥ b(n) + ∫R − B` leaves the next picture.
//! * [`RateController`] — a reactive per-picture quantiser governor
//!   over a virtual buffer: after each picture the fullness moves by
//!   `actual − target` bits and the next picture's QUANT follows the
//!   fullness proportionally (clamped to `1..=31` and to a bounded
//!   per-picture step so quality moves smoothly).
//!
//! The sequence encoder pairs them: the controller picks QUANT, the
//! picture is coded by the existing INTRA / motion-INTER encoders, and
//! the HRD model verifies §B.4 conformance — when a picture would
//! leave the buffer at or above `B` (§B.4: the picture undershot the
//! channel, growing the backlog) it is re-encoded at a **finer** QUANT
//! (spending more bits) until it conforms or the QUANT floor / the
//! re-encode budget is hit, in which case the violation is reported in
//! the stats rather than silently ignored.

/// Annex B Hypothetical Reference Decoder parameters.
///
/// `bits_per_tick` is `R / PCF` — the channel bits that arrive during
/// one picture clock interval (§B.1 pins encoder and HRD to the same
/// picture clock). `b_max` is the §B.2 bound `B = 4 · Rmax / PCF`
/// (a minimum; a larger negotiated value may be supplied).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HrdParams {
    /// Channel bits arriving per picture clock interval (`R / PCF`).
    pub bits_per_tick: u64,
    /// §B.2 `B` — the occupancy that must not be reached immediately
    /// after a picture's removal (§B.4).
    pub b_max: u64,
}

impl HrdParams {
    /// The §B.2 minimum-`B` parameter set for a constant-bit-rate
    /// channel of `rate_bps` bits per second at `pcf` pictures per
    /// second: `bits_per_tick = R / PCF`, `B = 4 · R / PCF`.
    pub fn for_cbr(rate_bps: u64, pcf: u64) -> HrdParams {
        let bits_per_tick = rate_bps / pcf.max(1);
        HrdParams {
            bits_per_tick,
            b_max: 4 * bits_per_tick,
        }
    }
}

/// Outcome of feeding one coded picture through the [`HrdModel`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HrdPictureOutcome {
    /// Buffer occupancy in bits immediately after this picture's
    /// removal (§B.4's `b(n)`).
    pub occupancy_after_removal: u64,
    /// Whether the §B.4 requirement held: occupancy after removal is
    /// **less than** `B`.
    pub conformant: bool,
    /// Picture clock intervals that elapsed before the picture was
    /// complete in the buffer (1 when the picture's bits fit one
    /// interval's channel budget plus the carried occupancy).
    pub ticks_waited: u64,
}

/// The Annex B HRD buffer simulation (encoder-side mirror).
///
/// §B.3 — the buffer starts empty. Each call to
/// [`HrdModel::push_picture`] models the arrival of one coded
/// picture's bits on the channel and its §B.4 removal at the first
/// picture clock examination at which it is complete.
#[derive(Debug, Clone, Copy)]
pub struct HrdModel {
    params: HrdParams,
    /// Current occupancy in bits (after the most recent removal).
    occupancy: u64,
    /// Bits still to arrive for pictures already pushed (channel
    /// backlog beyond the buffered `occupancy`).
    max_after_removal: u64,
}

impl HrdModel {
    /// An initially-empty HRD (§B.3).
    pub fn new(params: HrdParams) -> HrdModel {
        HrdModel {
            params,
            occupancy: 0,
            max_after_removal: 0,
        }
    }

    /// The §B.4 occupancy just after the most recent removal.
    pub fn occupancy(&self) -> u64 {
        self.occupancy
    }

    /// The worst (largest) §B.4 post-removal occupancy seen so far.
    pub fn max_occupancy_after_removal(&self) -> u64 {
        self.max_after_removal
    }

    /// Model one coded picture of `bits` bits on a saturated CBR
    /// channel: the buffer holds `occupancy` already-arrived bits (this
    /// picture's bits arrive first, in bitstream order); the §B.4
    /// examinations run every picture clock interval and the picture is
    /// removed at the first examination at which it is complete
    /// (at least one interval — one picture is removed per
    /// examination). Immediately after removal the residual occupancy
    /// is checked against `B` — the on-wire form of §B.4's
    /// `d(n+1) ≥ b(n) + ∫R − B` requirement.
    pub fn push_picture(&mut self, bits: u64) -> HrdPictureOutcome {
        // Channel intervals until the picture is complete in the
        // buffer: bits still to arrive, over R/PCF per interval,
        // minimum one examination interval.
        let missing = bits.saturating_sub(self.occupancy);
        let ticks = missing.div_ceil(self.params.bits_per_tick.max(1)).max(1);
        // Arrived-then-removed accounting (saturated channel).
        let occupancy_after_removal = self.occupancy + ticks * self.params.bits_per_tick - bits;
        self.occupancy = occupancy_after_removal;
        self.max_after_removal = self.max_after_removal.max(occupancy_after_removal);
        HrdPictureOutcome {
            occupancy_after_removal,
            conformant: occupancy_after_removal < self.params.b_max,
            ticks_waited: ticks,
        }
    }
}

/// Reactive per-picture quantiser governor over a virtual buffer.
///
/// The virtual buffer fullness `W` integrates the per-picture budget
/// error (`actual − target`); the next QUANT is the base QUANT scaled
/// by the fullness (`QP · (1 + W / (2T))`), clamped to `1..=31` and to
/// a per-picture step of ±4 so quality moves smoothly. A deeply
/// underfull buffer (more than one picture's budget in hand) is
/// clamped to `−T` — banked bits do not accumulate without bound.
#[derive(Debug, Clone, Copy)]
pub struct RateController {
    /// Target bits per coded picture.
    target: i64,
    /// Virtual buffer fullness (signed; positive = over budget).
    fullness: i64,
    /// QUANT chosen for the next picture.
    quant: u8,
}

impl RateController {
    /// A controller aiming at `target_bits_per_picture`, starting at
    /// `initial_quant` (clamped to `1..=31`).
    pub fn new(target_bits_per_picture: u32, initial_quant: u8) -> RateController {
        RateController {
            target: (target_bits_per_picture.max(1)) as i64,
            fullness: 0,
            quant: initial_quant.clamp(1, 31),
        }
    }

    /// The QUANT to use for the next picture.
    pub fn next_quant(&self) -> u8 {
        self.quant
    }

    /// Current virtual-buffer fullness in bits (positive = over
    /// budget).
    pub fn fullness(&self) -> i64 {
        self.fullness
    }

    /// Record a coded picture of `bits` bits and derive the next
    /// picture's QUANT.
    pub fn update(&mut self, bits: u64) {
        // The virtual buffer integrates the budget error, capped on
        // both sides so a single burst (an INTRA picture) or a long
        // cheap stretch cannot pin the controller for many pictures:
        // banked underspend tops out at one picture budget, backlog at
        // two (the §B.2-flavoured "a few picture intervals" window).
        self.fullness =
            (self.fullness + bits as i64 - self.target).clamp(-self.target, 2 * self.target);
        // Proportional QUANT reaction: scale by (1 + W / 2T), bounded
        // to a ±4 step and the legal 1..=31 range.
        let scaled = (self.quant as i64) * (2 * self.target + self.fullness) / (2 * self.target);
        let stepped = scaled.clamp(self.quant as i64 - 4, self.quant as i64 + 4);
        // A saturated buffer must keep pressure on even when the
        // proportional term rounds to zero change.
        let nudged = if self.fullness >= 2 * self.target && stepped == self.quant as i64 {
            stepped + 1
        } else if self.fullness <= -self.target && stepped == self.quant as i64 {
            stepped - 1
        } else {
            stepped
        };
        self.quant = nudged.clamp(1, 31) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §B.2 — the CBR helper derives `bits_per_tick = R / PCF` and
    /// `B = 4 · R / PCF`.
    #[test]
    fn hrd_params_cbr_derivation() {
        let p = HrdParams::for_cbr(64_000, 25);
        assert_eq!(p.bits_per_tick, 2560);
        assert_eq!(p.b_max, 10240);
    }

    /// A stream of exactly-budget pictures never accumulates occupancy:
    /// each picture completes in one tick and leaves the buffer empty.
    #[test]
    fn hrd_on_budget_pictures_leave_buffer_empty() {
        let mut hrd = HrdModel::new(HrdParams::for_cbr(64_000, 25));
        for _ in 0..10 {
            let out = hrd.push_picture(2560);
            assert!(out.conformant);
            assert_eq!(out.occupancy_after_removal, 0);
            assert_eq!(out.ticks_waited, 1);
        }
    }

    /// An oversized picture waits for the channel (multiple ticks) but
    /// the buffer stays conformant; small pictures after it drain the
    /// backlog.
    #[test]
    fn hrd_oversized_picture_waits_for_channel() {
        let mut hrd = HrdModel::new(HrdParams::for_cbr(64_000, 25));
        // 3.5 ticks of channel bits.
        let out = hrd.push_picture(8960);
        assert_eq!(out.ticks_waited, 4);
        // 4 ticks arrived = 10240 bits; picture removed 8960 → 1280
        // left, below B.
        assert_eq!(out.occupancy_after_removal, 1280);
        assert!(out.conformant);
        // A tiny follow-up picture drains: 1280 + 2560 arrives ≥ 640.
        let out = hrd.push_picture(640);
        assert!(out.conformant);
        assert_eq!(out.ticks_waited, 1);
    }

    /// Sustained tiny pictures on a fat channel pile up occupancy until
    /// the §B.4 bound trips — the non-conformance is reported, not
    /// masked.
    #[test]
    fn hrd_reports_overflow() {
        let params = HrdParams {
            bits_per_tick: 1000,
            b_max: 2000,
        };
        let mut hrd = HrdModel::new(params);
        let mut tripped = false;
        for _ in 0..8 {
            let out = hrd.push_picture(100);
            tripped |= !out.conformant;
        }
        assert!(
            tripped,
            "900-bit/tick surplus must cross B = 2000 within 8 pictures"
        );
    }

    /// The controller pushes QUANT up when pictures overshoot and back
    /// down when they undershoot, never leaving `1..=31`.
    #[test]
    fn controller_tracks_budget() {
        let mut rc = RateController::new(10_000, 10);
        // Persistent 2× overshoot → QUANT must rise.
        for _ in 0..6 {
            let q = rc.next_quant();
            rc.update(20_000);
            assert!(rc.next_quant() >= q);
        }
        assert!(rc.next_quant() > 10, "QUANT should have risen");
        // Persistent deep undershoot → QUANT must fall back.
        for _ in 0..12 {
            rc.update(1_000);
        }
        assert!(rc.next_quant() < 31);
        let low = rc.next_quant();
        assert!(low < 10 + 4, "QUANT should be easing back, got {low}");
        // Bounds hold under extreme inputs.
        let mut rc = RateController::new(100, 30);
        for _ in 0..20 {
            rc.update(1_000_000);
        }
        assert_eq!(rc.next_quant(), 31);
        let mut rc = RateController::new(1_000_000, 2);
        for _ in 0..20 {
            rc.update(10);
        }
        assert_eq!(rc.next_quant(), 1);
    }
}
