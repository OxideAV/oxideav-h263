//! H.263 Annex I §I.3 — INTRA block parsing with absorbed INTRADC.
//!
//! When Advanced INTRA Coding (Annex I, AIC) is in use, the §5.4
//! block-layer layout changes: INTRADC is no longer carried as a
//! separate 8-bit FLC (§5.4.1, Table 15) before the AC TCOEF events.
//! Per §I.3:
//!
//! > "When Advanced INTRA coding is in use, INTRADC transform
//! > coefficients are no longer handled as a separate case, but are
//! > instead treated in the same way as the AC coefficients in regard
//! > to MCBPC and CBPY. This means that a zero INTRADC will not be
//! > coded as a LEVEL, but will simply increase the run for the
//! > following AC coefficients."
//!
//! The block decode is therefore purely a sequence of Table I.2
//! `(LAST, RUN, LEVEL)` events ([`crate::intra_tcoef::decode_intra_tcoef_event`])
//! starting at scan position 0 — the DC slot is just slot 0 of the
//! coefficient buffer and is filled by the first event whose
//! cumulative-RUN lands on it (or stays zero if no event ever does).
//!
//! Two consequences of "INTRADC treated like AC for CBPY/MCBPC":
//!
//! * The block context collapses to a single boolean: whether the
//!   block has any coefficient events at all. In §5.4 + AIC, this is
//!   driven by the relevant CBPY (luma 0..=3) or CBPC (chroma 4 / 5)
//!   bit, with no separate "INTRADC always present for INTRA"
//!   carve-out — when the CBP bit is 0 the block is all zeros (a
//!   zero DC is signalled by the CBP bit, not by an FLC = 0 event).
//! * The first event's RUN can be 0 (DC carries a non-zero LEVEL) or
//!   any 0..=63 (DC is zero, the first coded coefficient is at scan
//!   position RUN). The §5.4.2 baseline `(RUN, LEVEL)` interpretation
//!   is unchanged; only the slot-0 reservation goes away.
//!
//! ## Composing with the §I.3 coefficient pipeline
//!
//! The output of this parser is bitstream-order `LEVEL` integers in
//! zigzag scan position order (per the round-14 `intra_tcoef` event
//! API and the round-4 `block` output convention). The §I.3
//! coefficient pipeline downstream still requires three more steps
//! before the values can be inverse-transformed:
//!
//! 1. Apply the §I.3 modified inverse-quantization formula `RecC =
//!    2 · QUANT · LEVEL` slot-by-slot (the round-17
//!    [`crate::aic_dequant::aic_dequant_coefficient`] primitive).
//! 2. Add the per-INTRA_MODE prediction contributions from RecA'(u,v)
//!    / RecB'(u,v) (the macroblock-grid driver supplies the live
//!    neighbour blocks; not in this module).
//! 3. Apply `clipAC` for AC slots and `oddifyclipDC` for the DC slot
//!    ([`crate::aic_dequant::clip_ac`] / [`crate::aic_dequant::oddify_clip_dc`]).
//! 4. Scatter from zigzag-scan order into 8×8 block position using
//!    the §I.3 scan selected by [`crate::aic::scan_for_intra_mode`]
//!    (zigzag for `DcOnly`, alternate-horizontal for `VerticalDcAc`,
//!    alternate-vertical for `HorizontalDcAc`) — NOT
//!    [`crate::block::ZIGZAG_TO_BLOCK_POS`] when AIC is in use.
//!
//! This round wires step 0: the event-stream → zigzag-order `LEVEL`
//! array. Steps 1–4 already have their primitives staged; the
//! macroblock-grid driver round will compose them.
//!
//! ## Deliberately out of scope (for this round)
//!
//! * The macroblock-grid driver dispatch: which INTRA blocks of a
//!   §I-mode picture go through this parser vs. the round-4
//!   [`crate::block::parse_block`] path (i.e. when to set the AIC
//!   flag) lives in `picture` once the PLUSPTYPE-gated driver lands.
//! * The DC/AC prediction reconstruction itself (needs neighbour
//!   blocks).
//!
//! ## Annex T interaction
//!
//! When the picture header signals both Advanced INTRA Coding and
//! Modified Quantization mode, [`parse_intra_block_aic`] threads the
//! caller's `modified_quant` flag into [`decode_intra_tcoef_event`]:
//! the §5.4.2 / §I.3 ESCAPE LEVEL `1000 0000` then becomes the §T.4
//! EXTENDED-ESCAPE marker rather than a forbidden code (§T.5 rule 2
//! extends the mechanism to the Table I.2 VLC). With the flag clear
//! the baseline rule applies and `0x80` stays forbidden.

// Tests in this module write bitstream events using the same
// MSB-first nibble grouping the spec prints (e.g. `0000 011` for the
// 7-bit ESCAPE prefix). Suppress clippy's power-of-two grouping
// suggestion so the literals stay auditable against the §I.3 / §5.4.2
// spec text.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::block::{H263Block, COEFFS_PER_BLOCK};
use crate::intra_tcoef::decode_intra_tcoef_event;
use crate::{Error, Result};

/// Parse a single Annex I §I.3 INTRA block — the AIC variant of
/// [`crate::block::parse_block`] in which INTRADC is absorbed into the
/// event stream.
///
/// `has_coefficients` is the relevant CBP bit for this block (CBPY for
/// luma 0..=3, CBPC for chroma 4 / 5). When `false`, the block is
/// all-zero per §I.3 (the CBP bit being 0 is now the sole signal that
/// the DC is also 0 — there is no FLC = 0 path). When `true`, the
/// parser decodes Table I.2 events until it consumes the terminating
/// LAST=1 event.
///
/// On success the reader is left at the first bit after the
/// terminating event, or at the input position if `has_coefficients
/// == false`. On error the reader position is unspecified.
///
/// The returned [`H263Block`] holds the parsed `LEVEL` integers in
/// zigzag-scan-position order (the convention of [`H263Block`]).
/// `had_intradc` is set to `false` — no separate FLC was consumed —
/// regardless of whether slot 0 carries a non-zero LEVEL after parsing.
///
/// ## §I.3 scan-position rules
///
/// * The scan position starts at 0 (the DC slot). The first event's
///   RUN advances the position by RUN before the event's LEVEL is
///   written — so an event `(RUN=0, LEVEL=v)` lands `v` in the DC
///   slot, and an event `(RUN=5, LEVEL=v)` lands `v` in scan
///   position 5 with slots 0..=4 left at zero.
/// * Each subsequent non-terminating event advances the position by
///   `RUN + 1`, writes its LEVEL into the resulting slot. (The `+1`
///   in `scan_pos += 1` between events comes from the §5.4.2
///   convention that RUN is the count of zero coefficients
///   *strictly* between successive non-zero LEVELs.)
/// * If the cumulative position would land at or past slot 64, the
///   block is malformed: [`Error::BadTcoefRunOverflow`].
///
/// These rules are identical to the §5.4.2 baseline post-INTRADC
/// loop in [`crate::block::parse_block`] — the only difference is
/// the starting position (0 here, 1 in the baseline INTRA case).
pub fn parse_intra_block_aic(
    reader: &mut BitReader<'_>,
    has_coefficients: bool,
    modified_quant: bool,
) -> Result<H263Block> {
    let mut block = H263Block::empty();

    if !has_coefficients {
        // §I.3 / §5.3.2 + §5.3.5 — CBPY/CBPC bit = 0 in AIC mode
        // means "this block carries no coefficients at all", DC
        // included.
        return Ok(block);
    }

    // §I.3 — Table I.2 event stream starting at scan position 0
    // (the DC slot is the first slot the event stream may write to).
    let mut scan_pos: usize = 0;

    loop {
        // §T.4 / §T.5 rule 2 — when Modified Quantization mode is in
        // use the Table I.2 ESCAPE LEVEL `1000 0000` is the
        // EXTENDED-ESCAPE marker (AC magnitude > 127); otherwise it is
        // forbidden, exactly as in the baseline coefficient parser.
        let event = decode_intra_tcoef_event(reader, modified_quant)?;
        // Advance past `RUN` zero coefficients, then write LEVEL into
        // the resulting slot. Before the first event runs, scan_pos
        // is 0 — so the very first event's RUN is the count of zero
        // coefficients (including the DC slot) before the first
        // non-zero LEVEL.
        scan_pos = scan_pos
            .checked_add(event.run as usize)
            .ok_or(Error::BadTcoefRunOverflow)?;
        if scan_pos >= COEFFS_PER_BLOCK {
            return Err(Error::BadTcoefRunOverflow);
        }
        block.coefficients[scan_pos] = event.level;
        block.tcoef_event_count += 1;
        if event.last {
            break;
        }
        scan_pos += 1;
        if scan_pos >= COEFFS_PER_BLOCK {
            // Next event would have RUN >= 0 at scan_pos, but slot 63
            // is already filled — no room left.
            return Err(Error::BadTcoefRunOverflow);
        }
    }

    Ok(block)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Pad a BitWriter to a byte boundary and finish it into a Vec.
    fn finish_aligned(mut w: BitWriter) -> Vec<u8> {
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    /// Build a Table-I.2 ESCAPE event with the given `(last, run,
    /// level)` triple and return the encoded bytes. Used to inject
    /// arbitrary `(RUN, LEVEL)` test cases without scanning the table
    /// for a matching regular row.
    ///
    /// The ESCAPE event layout (§I.3 reusing §5.4.2): 7-bit prefix
    /// `0000 011`, 1-bit LAST, 6-bit RUN, 8-bit two's-complement
    /// LEVEL. The caller is responsible for keeping `level` clear of
    /// the forbidden baseline codes `0x00` and `0x80`.
    fn encode_escape_event(last: bool, run: u8, level: i8) -> Vec<u8> {
        let mut w = BitWriter::new();
        // ESCAPE prefix: 0000 011 (7 bits)
        w.write_u32(0b0000_011, 7);
        // LAST (1 bit)
        w.write_bit(last);
        // RUN (6 bits)
        w.write_u32(run as u32, 6);
        // LEVEL (8 bits, two's complement)
        w.write_u32((level as u8) as u32, 8);
        finish_aligned(w)
    }

    /// Encode the Table-I.2 row `(LAST=1, RUN=0, |LEVEL|=1)` — code
    /// `0111s` from row 58 — with a chosen sign bit. Used as a
    /// terminating event after another event has placed the DC.
    fn encode_row_l1_r0_lvl1(sign: bool) -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_u32(0b0111, 4);
        w.write_bit(sign);
        finish_aligned(w)
    }

    /// CBPY/CBPC bit zero → block is all-zero, no bits consumed.
    #[test]
    fn no_coefficients_yields_empty_block() {
        // Arrange a buffer that would error if consumed (high bit = a
        // forbidden ESCAPE prefix start) — the parser must not touch
        // it.
        let bytes = vec![0xFFu8; 4];
        let mut reader = BitReader::new(&bytes);
        let block =
            parse_intra_block_aic(&mut reader, false, false).expect("no-events path is infallible");
        assert!(!block.had_intradc, "AIC never sets had_intradc");
        assert_eq!(block.tcoef_event_count, 0);
        assert_eq!(block.coefficients, [0; COEFFS_PER_BLOCK]);
        // The reader should still be at bit 0 — nothing was consumed.
        // (Cross-check: ask for 1 bit and confirm the value is the
        // leading bit of 0xFF.)
        let next = reader
            .read_bit()
            .expect("buffer was not touched, so 1 bit should remain");
        assert!(next);
    }

    /// Single terminating event at the DC slot — confirms the §I.3
    /// absorbed-INTRADC semantics: DC carries a `LEVEL` (not the
    /// 8-bit FLC of §5.4.1), the slot-0 reservation is gone.
    #[test]
    fn single_event_dc_via_absorbed_intradc() {
        // Encode LAST=1 RUN=0 LEVEL=+1 (row 58 of Table I.2, code
        // `0111s` with sign bit 0). Per §I.3 this single event sets
        // the DC slot to +1 and terminates the block.
        let bytes = encode_row_l1_r0_lvl1(false);
        let mut reader = BitReader::new(&bytes);
        let block = parse_intra_block_aic(&mut reader, true, false)
            .expect("well-formed single-event block");
        assert!(!block.had_intradc);
        assert_eq!(block.tcoef_event_count, 1);
        assert_eq!(block.coefficients[0], 1);
        // Everything past slot 0 stays zero.
        for (idx, &c) in block.coefficients.iter().enumerate().skip(1) {
            assert_eq!(c, 0, "slot {} should be zero", idx);
        }
    }

    /// §I.3 line 4216: "a zero INTRADC will not be coded as a LEVEL,
    /// but will simply increase the run for the following AC
    /// coefficients." Cover that explicitly with an event whose
    /// `RUN=3` skips the DC slot (and slots 1, 2) before placing
    /// `LEVEL=+1` at scan position 3.
    #[test]
    fn zero_dc_is_signalled_by_run() {
        // Use ESCAPE to get exact (RUN=3, LEVEL=+1, LAST=1).
        let bytes = encode_escape_event(true, 3, 1);
        let mut reader = BitReader::new(&bytes);
        let block = parse_intra_block_aic(&mut reader, true, false)
            .expect("escape-event block with RUN=3 is well-formed");
        assert!(!block.had_intradc);
        assert_eq!(block.tcoef_event_count, 1);
        assert_eq!(
            block.coefficients[0], 0,
            "DC slot should be zero — §I.3 absorbed RUN"
        );
        assert_eq!(block.coefficients[1], 0);
        assert_eq!(block.coefficients[2], 0);
        assert_eq!(block.coefficients[3], 1, "LEVEL lands at scan slot 3");
        for (idx, &c) in block.coefficients.iter().enumerate().skip(4) {
            assert_eq!(c, 0, "slot {} should be zero", idx);
        }
    }

    /// Two events: DC-bearing (RUN=0 LEVEL=+1) followed by an AC
    /// event (RUN=2 LEVEL=-1 LAST=1). Confirms the inter-event
    /// `scan_pos += 1` advance is correct after slot 0.
    #[test]
    fn dc_then_ac_event_sequence() {
        // Event 1: row 0 of Table I.2 — code 10s with sign 0 →
        // (LAST=0, RUN=0, LEVEL=+1).
        // Event 2: ESCAPE with (LAST=1, RUN=2, LEVEL=-1).
        let mut bytes = Vec::new();
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        w.write_bit(false);
        // ESCAPE prefix and tail.
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(2, 6);
        w.write_u32(((-1i8) as u8) as u32, 8);
        bytes.extend(finish_aligned(w));
        let mut reader = BitReader::new(&bytes);
        let block = parse_intra_block_aic(&mut reader, true, false).expect("two-event block");
        assert_eq!(block.tcoef_event_count, 2);
        assert_eq!(block.coefficients[0], 1, "DC from first event");
        assert_eq!(block.coefficients[1], 0);
        assert_eq!(block.coefficients[2], 0);
        // After event 1 at slot 0, scan_pos advances to 1; event 2's
        // RUN=2 lands LEVEL at slot 1 + 2 = 3.
        assert_eq!(block.coefficients[3], -1, "AC LEVEL at slot 3");
    }

    /// Boundary: an event with cumulative scan position landing
    /// exactly at slot 63 (the last AC) is well-formed and
    /// terminating. (Single ESCAPE event with RUN=63, LAST=1.)
    #[test]
    fn event_at_slot_63_is_well_formed() {
        let bytes = encode_escape_event(true, 63, 1);
        let mut reader = BitReader::new(&bytes);
        let block =
            parse_intra_block_aic(&mut reader, true, false).expect("RUN=63 lands LEVEL at slot 63");
        assert_eq!(block.coefficients[63], 1);
        assert_eq!(block.coefficients[0], 0);
        assert_eq!(block.tcoef_event_count, 1);
    }

    /// Overflow guard: a non-terminating event at slot 63 leaves no
    /// room for any successor → next iteration must trip the
    /// post-write `scan_pos += 1; scan_pos >= 64` check before any
    /// extra event is decoded.
    #[test]
    fn non_terminating_event_at_slot_63_overflows() {
        // ESCAPE LAST=0 RUN=63 LEVEL=+1 — places LEVEL at slot 63 but
        // does not terminate, so the parser must error rather than try
        // to read another event into a non-existent slot 64.
        let bytes = encode_escape_event(false, 63, 1);
        let mut reader = BitReader::new(&bytes);
        let err = parse_intra_block_aic(&mut reader, true, false).expect_err("overflow expected");
        assert_eq!(err, Error::BadTcoefRunOverflow);
    }

    /// Overflow guard: a single event whose RUN alone exceeds the
    /// 64-slot block → `scan_pos.checked_add(run) >= 64` fires.
    /// (ESCAPE RUN field is 6 bits → max 63, so a single event can
    /// land exactly at slot 63 but cannot exceed it; this test
    /// constructs the overflow by combining a DC event with a second
    /// event whose RUN sum overruns.)
    #[test]
    fn cumulative_run_overflow_errors() {
        // Event 1: row 0 — LAST=0 RUN=0 LEVEL=+1 (lands DC).
        // Event 2: ESCAPE LAST=0 RUN=63 LEVEL=+1 (would land at slot
        // 0 + 1 + 63 = 64 → overflow at the pre-write check).
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        w.write_bit(false);
        w.write_u32(0b0000_011, 7);
        w.write_bit(false);
        w.write_u32(63, 6);
        w.write_u32(1, 8);
        let bytes = finish_aligned(w);
        let mut reader = BitReader::new(&bytes);
        let err = parse_intra_block_aic(&mut reader, true, false).expect_err("cumulative overflow");
        assert_eq!(err, Error::BadTcoefRunOverflow);
    }

    /// Truncated input: bitstream ends inside a regular VLC prefix.
    #[test]
    fn truncated_input_returns_unexpected_eof() {
        // Provide only the first two bits of row 0's code (`10`)
        // without the sign bit.
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        // No sign bit; reader will hit EOF when `finalise_event` tries
        // to read it.
        let bytes = w.finish();
        let mut reader = BitReader::new(&bytes);
        let err = parse_intra_block_aic(&mut reader, true, false).expect_err("EOF mid-event");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// Forbidden ESCAPE LEVEL `0x00` propagates as
    /// [`Error::BadTcoefEscapeLevel`] from `decode_intra_tcoef_event`
    /// (proving the AIC parser routes errors through correctly).
    #[test]
    fn forbidden_escape_level_propagates() {
        // ESCAPE prefix + LAST=1 + RUN=0 + LEVEL=0x00 (forbidden).
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0, 8);
        let bytes = finish_aligned(w);
        let mut reader = BitReader::new(&bytes);
        let err = parse_intra_block_aic(&mut reader, true, false).expect_err("forbidden LEVEL");
        assert_eq!(err, Error::BadTcoefEscapeLevel);
    }

    /// Sign-extension cross-check: ESCAPE LEVEL = -128 is FORBIDDEN
    /// in baseline (`0x80`) — confirm the parser rejects it.
    #[test]
    fn escape_level_0x80_is_forbidden() {
        let bytes = encode_escape_event_unchecked(true, 0, 0x80);
        let mut reader = BitReader::new(&bytes);
        let err = parse_intra_block_aic(&mut reader, true, false).expect_err("0x80 is forbidden");
        assert_eq!(err, Error::BadTcoefEscapeLevel);
    }

    /// ESCAPE LEVEL = -127 (0x81) IS allowed and decodes correctly.
    #[test]
    fn escape_level_negative_127_decodes() {
        // -127i8 == 0x81. Use the unchecked helper so we go through
        // the LEVEL forbidden-code path of the parser (and don't trip
        // on the i8 conversion path).
        let bytes = encode_escape_event(true, 0, -127);
        let mut reader = BitReader::new(&bytes);
        let block = parse_intra_block_aic(&mut reader, true, false).expect("0x81 is legal");
        assert_eq!(block.coefficients[0], -127);
    }

    /// ESCAPE LEVEL = +127 (0x7F) — the largest positive baseline
    /// LEVEL — round-trips through the DC slot.
    #[test]
    fn escape_level_positive_127_decodes() {
        let bytes = encode_escape_event(true, 0, 127);
        let mut reader = BitReader::new(&bytes);
        let block = parse_intra_block_aic(&mut reader, true, false).expect("+127 is legal");
        assert_eq!(block.coefficients[0], 127);
    }

    /// Unchecked helper for tests that need to inject a forbidden
    /// LEVEL byte directly (the safe helper above doesn't take a
    /// `u8`).
    fn encode_escape_event_unchecked(last: bool, run: u8, level_byte: u8) -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(last);
        w.write_u32(run as u32, 6);
        w.write_u32(level_byte as u32, 8);
        finish_aligned(w)
    }

    /// Many-event integration: build a block carrying 8 events at
    /// distributed scan positions (DC + AC), verify each event lands
    /// at its expected slot.
    #[test]
    fn many_event_distribution() {
        // Sequence of `(run, level, last)`:
        let events: &[(u8, i8, bool)] = &[
            (0, 5, false),   // slot 0 (DC) = +5
            (1, -3, false),  // slot 0 + 1 + 1 = 2 → -3
            (4, 2, false),   // slot 2 + 1 + 4 = 7 → +2
            (10, -1, false), // slot 7 + 1 + 10 = 18 → -1
            (0, 7, false),   // slot 18 + 1 = 19 → +7
            (20, 1, false),  // slot 19 + 1 + 20 = 40 → +1
            (5, -2, false),  // slot 40 + 1 + 5 = 46 → -2
            (16, 3, true),   // slot 46 + 1 + 16 = 63 → +3, LAST
        ];
        let mut w = BitWriter::new();
        for &(run, level, last) in events {
            // All events use ESCAPE for explicit (run, level) control.
            w.write_u32(0b0000_011, 7);
            w.write_bit(last);
            w.write_u32(run as u32, 6);
            w.write_u32(((level as u8) as u32) & 0xFF, 8);
        }
        let bytes = finish_aligned(w);
        let mut reader = BitReader::new(&bytes);
        let block = parse_intra_block_aic(&mut reader, true, false).expect("8-event block");
        let expected_slots: &[(usize, i16)] = &[
            (0, 5),
            (2, -3),
            (7, 2),
            (18, -1),
            (19, 7),
            (40, 1),
            (46, -2),
            (63, 3),
        ];
        for &(slot, value) in expected_slots {
            assert_eq!(
                block.coefficients[slot], value,
                "slot {} should carry LEVEL {}",
                slot, value
            );
        }
        // Every other slot should be zero.
        let nonzero: std::collections::HashSet<usize> =
            expected_slots.iter().map(|&(s, _)| s).collect();
        for (idx, &c) in block.coefficients.iter().enumerate() {
            if !nonzero.contains(&idx) {
                assert_eq!(c, 0, "slot {} should be zero", idx);
            }
        }
        assert_eq!(block.tcoef_event_count, events.len() as u32);
    }

    /// `had_intradc` is always `false` in AIC mode — even when slot 0
    /// carries a non-zero LEVEL. This contrasts with the baseline
    /// §5.4.1 path which always sets it `true` for INTRA blocks.
    #[test]
    fn had_intradc_always_false_in_aic() {
        let bytes = encode_row_l1_r0_lvl1(true); // LAST=1 RUN=0 LEVEL=-1
        let mut reader = BitReader::new(&bytes);
        let block = parse_intra_block_aic(&mut reader, true, false).expect("single-event block");
        assert!(
            !block.had_intradc,
            "AIC never sets had_intradc, even with non-zero DC"
        );
        assert_eq!(block.coefficients[0], -1);
    }

    /// Spec-text cross-check: §I.3 line 4216 — "a zero INTRADC will
    /// not be coded as a LEVEL, but will simply increase the run."
    /// Compare an all-zero-DC block whose first non-zero AC is at
    /// scan slot 1 (RUN=1) with an all-zero-DC block whose first
    /// non-zero AC is at scan slot 3 (RUN=3); both must leave the DC
    /// slot at 0.
    #[test]
    fn spec_4216_zero_dc_via_run_invariant() {
        for &(run, expected_slot) in &[(1u8, 1usize), (3u8, 3usize), (7u8, 7usize)] {
            let bytes = encode_escape_event(true, run, 1);
            let mut reader = BitReader::new(&bytes);
            let block = parse_intra_block_aic(&mut reader, true, false).expect("legal block");
            assert_eq!(
                block.coefficients[0], 0,
                "DC slot must stay 0 when RUN > 0 swallows it"
            );
            assert_eq!(
                block.coefficients[expected_slot], 1,
                "LEVEL=+1 lands at slot {}",
                expected_slot
            );
        }
    }
}
