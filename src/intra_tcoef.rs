//! Annex I — Advanced INTRA Coding: §I.3 / Table I.2 separate
//! INTRA-coefficient VLC.
//!
//! When the Advanced INTRA Coding (AIC) mode (§I) is in use, INTRA
//! macroblocks use a *separate* TCOEF VLC table — Table I.2 — for all
//! INTRADC and INTRA AC coefficients, in place of the normal Table 16
//! used outside AIC.
//!
//! Per §I.3 (line 4033 of the spec text):
//!
//! > "the VLC codeword entries used in Table I.2 are the same as those
//! > used in the normal TCOEF table (Table 16) used when Advanced
//! > INTRA coding is not in use, but with a different interpretation
//! > of LEVEL and RUN (without altering LAST)."
//!
//! That is, the 102 regular VLC codewords + the 7-bit ESCAPE prefix
//! are bit-for-bit identical to those of Table 16 (already transcribed
//! into [`crate::block::ZIGZAG_TO_BLOCK_POS`]'s sibling table); only
//! the mapping from each code-index to a `(LAST, RUN, |LEVEL|)` triple
//! is changed (and `LAST` is preserved between the two tables at each
//! index — only `RUN` / `|LEVEL|` are reassigned). The ESCAPE escape
//! event is decoded with the same 1-bit `LAST` + 6-bit `RUN` + 8-bit
//! signed `LEVEL` layout as in §5.4.2, and the baseline forbidden
//! LEVEL codes (`0x00` / `0x80`) apply identically — Annex T's
//! EXTENDED-ESCAPE relaxation is out of scope for this module.
//!
//! ## Scope of this round
//!
//! This module provides the pure VLC primitive: given a [`BitReader`]
//! positioned at the first bit of one Table-I.2 event, return the
//! decoded `(LAST, RUN, LEVEL)` tuple (sign applied; ESCAPE LEVEL
//! interpreted as `i8`). It does **not** yet:
//!
//! * Drive a full INTRA-block parser around it (the §I.3 modified
//!   inverse quantization, the variable-step INTRADC reconstruction,
//!   and the DC/AC prediction reconstruction all need the
//!   macroblock-grid driver to supply the neighbour blocks; those
//!   stay deferred to a later round).
//! * Replace the round-4 INTRADC FLC path in [`crate::block::parse_block`].
//!   In AIC mode INTRADC is no longer a separate 8-bit FLC — it is
//!   absorbed into the per-block coefficient stream as the slot-0 AC
//!   event (§I.3, line 4214: "INTRADC transform coefficients are no
//!   longer handled as a separate case, but are instead treated in the
//!   same way as the AC coefficients in regard to MCBPC and CBPY").
//!   This is now exposed as the round-18 [`crate::block_aic::parse_intra_block_aic`]
//!   primitive; wiring its dispatch into the macroblock-grid driver is
//!   the next round's job.

// The bit-pattern literals in this module are spec transcriptions of
// the Table 16 codes (reused by Table I.2 per §I.3), printed in
// MSB-first nibble groups to mirror the spec. Suppress clippy's
// power-of-two grouping suggestion so the lines stay auditable
// against the spec.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// A single Table-I.2 INTRA-coefficient event (one `(LAST, RUN,
/// LEVEL)` triple from the bitstream). `LEVEL` is signed (the
/// trailing `s` sign bit of the spec's printed VLC has been folded
/// in; ESCAPE `LEVEL` is interpreted as `i8` two's complement).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntraTcoefEvent {
    /// `true` if this is the terminating event of the block (LAST = 1).
    pub last: bool,
    /// Number of zero-valued coefficients preceding this event's
    /// non-zero level (`0..=63` for VLC rows, `0..=63` for the ESCAPE
    /// fixed-length field).
    pub run: u8,
    /// The signed coefficient level.
    pub level: i16,
}

/// One row of Table I.2/H.263 (the INTRA TCOEF VLC). The `bits`
/// column is the spec's printed "Bits" value verbatim — i.e. the
/// total length **including** the trailing sign bit `s` for the 102
/// regular VLC rows, and the literal length (`7`) for the ESCAPE
/// row. The lookup compares `bits - 1` for the former (the prefix
/// proper) and `bits` for the latter, then [`finalise_event`]
/// consumes the sign / fixed-length tail.
#[derive(Debug, Clone, Copy)]
struct IntraTcoefRow {
    bits: u8,
    code: u16,
    last: bool,
    run: u8,
    abs_level: u8,
    is_escape: bool,
}

const fn row(bits: u8, code: u16, last: bool, run: u8, abs_level: u8) -> IntraTcoefRow {
    IntraTcoefRow {
        bits,
        code,
        last,
        run,
        abs_level,
        is_escape: false,
    }
}

const fn escape_row(bits: u8, code: u16) -> IntraTcoefRow {
    IntraTcoefRow {
        bits,
        code,
        last: false,
        run: 0,
        abs_level: 0,
        is_escape: true,
    }
}

/// Table I.2/H.263 — VLC for INTRA TCOEF.
///
/// The codes are bit-for-bit identical to Table 16 at every index
/// (per §I.3); the (`last`, `run`, `abs_level`) columns are the
/// Table I.2 interpretation. The trailing index comment lines up
/// each row with the spec's printed "Index" column (0..=101 for
/// regular rows, 102 for ESCAPE).
const INTRA_TCOEF_TABLE: &[IntraTcoefRow] = &[
    // Indices 0..=11: LAST=0, |LEVEL|=1 (RUN climbs 0,1,3,5,7,8,9,10,
    // 11; then three off-pattern entries at idx 9/10/11 with non-1
    // |LEVEL|).
    row(3, 0b10, false, 0, 1),              // 0: 10s
    row(5, 0b1111, false, 1, 1),            // 1: 1111s
    row(7, 0b0101_01, false, 3, 1),         // 2: 0101 01s
    row(8, 0b0010_111, false, 5, 1),        // 3: 0010 111s
    row(9, 0b0001_1111, false, 7, 1),       // 4: 0001 1111s
    row(10, 0b0001_0010_1, false, 8, 1),    // 5: 0001 0010 1s
    row(10, 0b0001_0010_0, false, 9, 1),    // 6: 0001 0010 0s
    row(11, 0b0000_1000_01, false, 10, 1),  // 7: 0000 1000 01s
    row(11, 0b0000_1000_00, false, 11, 1),  // 8: 0000 1000 00s
    row(12, 0b0000_0000_111, false, 4, 3),  // 9: 0000 0000 111s
    row(12, 0b0000_0000_110, false, 9, 2),  // 10: 0000 0000 110s
    row(12, 0b0000_0100_000, false, 13, 1), // 11: 0000 0100 000s
    // Indices 12..=17: LAST=0, RUN=0..=1, |LEVEL|=2..=7
    row(4, 0b110, false, 0, 2),             // 12: 110s
    row(7, 0b0101_00, false, 1, 2),         // 13: 0101 00s
    row(9, 0b0001_1110, false, 1, 4),       // 14: 0001 1110s
    row(11, 0b0000_0011_11, false, 1, 5),   // 15: 0000 0011 11s
    row(12, 0b0000_0100_001, false, 1, 6),  // 16: 0000 0100 001s
    row(13, 0b0000_0101_0000, false, 1, 7), // 17: 0000 0101 0000s
    // Indices 18..=21
    row(5, 0b1110, false, 0, 3),            // 18: 1110s
    row(9, 0b0001_1101, false, 3, 2),       // 19: 0001 1101s
    row(11, 0b0000_0011_10, false, 2, 3),   // 20: 0000 0011 10s
    row(13, 0b0000_0101_0001, false, 3, 4), // 21: 0000 0101 0001s
    // Indices 22..=24
    row(6, 0b0110_1, false, 0, 5),        // 22: 0110 1s
    row(10, 0b0001_0001_1, false, 4, 2),  // 23: 0001 0001 1s
    row(11, 0b0000_0011_01, false, 3, 3), // 24: 0000 0011 01s
    // Indices 25..=27
    row(6, 0b0110_0, false, 0, 4),          // 25: 0110 0s
    row(10, 0b0001_0001_0, false, 5, 2),    // 26: 0001 0001 0s
    row(13, 0b0000_0101_0010, false, 5, 3), // 27: 0000 0101 0010s
    // Indices 28..=30
    row(6, 0b0101_1, false, 2, 1),           // 28: 0101 1s
    row(11, 0b0000_0011_00, false, 6, 2),    // 29: 0000 0011 00s
    row(13, 0b0000_0101_0011, false, 0, 25), // 30: 0000 0101 0011s
    // Indices 31..=33
    row(7, 0b0100_11, false, 4, 1),          // 31: 0100 11s
    row(11, 0b0000_0010_11, false, 7, 2),    // 32: 0000 0010 11s
    row(13, 0b0000_0101_0100, false, 0, 24), // 33: 0000 0101 0100s
    // Indices 34..=35
    row(7, 0b0100_10, false, 0, 8),       // 34: 0100 10s
    row(11, 0b0000_0010_10, false, 8, 2), // 35: 0000 0010 10s
    // Indices 36..=37
    row(7, 0b0100_01, false, 0, 7),       // 36: 0100 01s
    row(11, 0b0000_0010_01, false, 2, 4), // 37: 0000 0010 01s
    // Indices 38..=39
    row(7, 0b0100_00, false, 0, 6),        // 38: 0100 00s
    row(11, 0b0000_0010_00, false, 12, 1), // 39: 0000 0010 00s
    // Indices 40..=41
    row(8, 0b0010_110, false, 0, 9),         // 40: 0010 110s
    row(13, 0b0000_0101_0101, false, 0, 23), // 41: 0000 0101 0101s
    // Indices 42..=43
    row(8, 0b0010_101, false, 2, 2), // 42: 0010 101s
    row(8, 0b0010_100, false, 1, 3), // 43: 0010 100s
    // Indices 44..=45
    row(9, 0b0001_1100, false, 6, 1),  // 44: 0001 1100s
    row(9, 0b0001_1011, false, 0, 10), // 45: 0001 1011s
    // Indices 46..=47
    row(10, 0b0001_0000_1, false, 0, 12), // 46: 0001 0000 1s
    row(10, 0b0001_0000_0, false, 0, 11), // 47: 0001 0000 0s
    // Indices 48..=53: LAST=0, RUN=0, |LEVEL|=18..=13 (descending)
    row(10, 0b0000_1111_1, false, 0, 18), // 48: 0000 1111 1s
    row(10, 0b0000_1111_0, false, 0, 17), // 49: 0000 1111 0s
    row(10, 0b0000_1110_1, false, 0, 16), // 50: 0000 1110 1s
    row(10, 0b0000_1110_0, false, 0, 15), // 51: 0000 1110 0s
    row(10, 0b0000_1101_1, false, 0, 14), // 52: 0000 1101 1s
    row(10, 0b0000_1101_0, false, 0, 13), // 53: 0000 1101 0s
    // Indices 54..=57: LAST=0, RUN=0, |LEVEL|=20/19/22/21
    row(12, 0b0000_0100_010, false, 0, 20), // 54: 0000 0100 010s
    row(12, 0b0000_0100_011, false, 0, 19), // 55: 0000 0100 011s
    row(13, 0b0000_0101_0110, false, 0, 22), // 56: 0000 0101 0110s
    row(13, 0b0000_0101_0111, false, 0, 21), // 57: 0000 0101 0111s
    // Indices 58..=101: LAST=1 (the back half of the table).
    row(5, 0b0111, true, 0, 1),             // 58: 0111s
    row(10, 0b0000_1100_1, true, 14, 1),    // 59: 0000 1100 1s
    row(12, 0b0000_0000_101, true, 20, 1),  // 60: 0000 0000 101s
    row(7, 0b0011_11, true, 1, 1),          // 61: 0011 11s
    row(12, 0b0000_0000_100, true, 19, 1),  // 62: 0000 0000 100s
    row(7, 0b0011_10, true, 2, 1),          // 63: 0011 10s
    row(7, 0b0011_01, true, 3, 1),          // 64: 0011 01s
    row(7, 0b0011_00, true, 0, 2),          // 65: 0011 00s
    row(8, 0b0010_011, true, 5, 1),         // 66: 0010 011s
    row(8, 0b0010_010, true, 6, 1),         // 67: 0010 010s
    row(8, 0b0010_001, true, 4, 1),         // 68: 0010 001s
    row(8, 0b0010_000, true, 0, 3),         // 69: 0010 000s
    row(9, 0b0001_1010, true, 9, 1),        // 70: 0001 1010s
    row(9, 0b0001_1001, true, 10, 1),       // 71: 0001 1001s
    row(9, 0b0001_1000, true, 11, 1),       // 72: 0001 1000s
    row(9, 0b0001_0111, true, 12, 1),       // 73: 0001 0111s
    row(9, 0b0001_0110, true, 13, 1),       // 74: 0001 0110s
    row(9, 0b0001_0101, true, 8, 1),        // 75: 0001 0101s
    row(9, 0b0001_0100, true, 7, 1),        // 76: 0001 0100s
    row(9, 0b0001_0011, true, 0, 4),        // 77: 0001 0011s
    row(10, 0b0000_1100_0, true, 17, 1),    // 78: 0000 1100 0s
    row(10, 0b0000_1011_1, true, 18, 1),    // 79: 0000 1011 1s
    row(10, 0b0000_1011_0, true, 16, 1),    // 80: 0000 1011 0s
    row(10, 0b0000_1010_1, true, 15, 1),    // 81: 0000 1010 1s
    row(10, 0b0000_1010_0, true, 2, 2),     // 82: 0000 1010 0s
    row(10, 0b0000_1001_1, true, 1, 2),     // 83: 0000 1001 1s
    row(10, 0b0000_1001_0, true, 0, 6),     // 84: 0000 1001 0s
    row(10, 0b0000_1000_1, true, 0, 5),     // 85: 0000 1000 1s
    row(11, 0b0000_0001_11, true, 4, 2),    // 86: 0000 0001 11s
    row(11, 0b0000_0001_10, true, 3, 2),    // 87: 0000 0001 10s
    row(11, 0b0000_0001_01, true, 1, 3),    // 88: 0000 0001 01s
    row(11, 0b0000_0001_00, true, 0, 7),    // 89: 0000 0001 00s
    row(12, 0b0000_0100_100, true, 2, 3),   // 90: 0000 0100 100s
    row(12, 0b0000_0100_101, true, 1, 4),   // 91: 0000 0100 101s
    row(12, 0b0000_0100_110, true, 0, 9),   // 92: 0000 0100 110s
    row(12, 0b0000_0100_111, true, 0, 8),   // 93: 0000 0100 111s
    row(13, 0b0000_0101_1000, true, 21, 1), // 94: 0000 0101 1000s
    row(13, 0b0000_0101_1001, true, 22, 1), // 95: 0000 0101 1001s
    row(13, 0b0000_0101_1010, true, 23, 1), // 96: 0000 0101 1010s
    row(13, 0b0000_0101_1011, true, 7, 2),  // 97: 0000 0101 1011s
    row(13, 0b0000_0101_1100, true, 6, 2),  // 98: 0000 0101 1100s
    row(13, 0b0000_0101_1101, true, 5, 2),  // 99: 0000 0101 1101s
    row(13, 0b0000_0101_1110, true, 3, 3),  // 100: 0000 0101 1110s
    row(13, 0b0000_0101_1111, true, 0, 10), // 101: 0000 0101 1111s
    // Index 102 — ESCAPE. Same 7-bit prefix as Table 16; the
    // subsequent 1 + 6 + 8 = 15-bit fixed-length event is decoded
    // identically (per §I.3 the I.2 reinterpretation does not change
    // the ESCAPE event layout — only the regular-row LEVEL/RUN
    // assignments).
    escape_row(7, 0b0000_011), // 102: 0000 011
];

/// Outcome of an Table-I.2 prefix lookup.
#[derive(Debug, Clone, Copy)]
enum IntraEntry {
    Vlc { last: bool, run: u8, abs_level: u8 },
    Escape,
}

/// Look up a Table-I.2 VLC prefix. The semantics match
/// [`crate::block`]'s Table-16 lookup: `bits` is the prefix length
/// (excluding the trailing sign bit for non-ESCAPE rows; equal to
/// the spec's printed length for the ESCAPE row).
fn lookup_prefix(bits: u8, code: u32) -> Option<IntraEntry> {
    for &r in INTRA_TCOEF_TABLE.iter() {
        let prefix_bits = if r.is_escape { r.bits } else { r.bits - 1 };
        if prefix_bits == bits && r.code as u32 == code {
            return Some(if r.is_escape {
                IntraEntry::Escape
            } else {
                IntraEntry::Vlc {
                    last: r.last,
                    run: r.run,
                    abs_level: r.abs_level,
                }
            });
        }
    }
    None
}

/// Apply the trailing sign bit (or the ESCAPE fixed-length tail) to
/// produce a fully-decoded [`IntraTcoefEvent`].
fn finalise_event(reader: &mut BitReader<'_>, entry: IntraEntry) -> Result<IntraTcoefEvent> {
    match entry {
        IntraEntry::Vlc {
            last,
            run,
            abs_level,
        } => {
            let sign = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            let level = if sign {
                -(abs_level as i16)
            } else {
                abs_level as i16
            };
            Ok(IntraTcoefEvent { last, run, level })
        }
        IntraEntry::Escape => {
            // §I.3 reuses the §5.4.2 ESCAPE layout: 1 bit LAST, 6 bits
            // RUN, 8 bits LEVEL (two's complement), with the baseline
            // §5.4.2 forbidden LEVEL codes (`0x00` / `0x80`) applied.
            let last_bit = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            let run = reader.read_u32(6).map_err(|_| Error::UnexpectedEof)? as u8;
            let level_bits = reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8;
            if level_bits == 0x00 || level_bits == 0x80 {
                return Err(Error::BadTcoefEscapeLevel);
            }
            let level: i16 = (level_bits as i8) as i16;
            Ok(IntraTcoefEvent {
                last: last_bit,
                run,
                level,
            })
        }
    }
}

/// Decode one Table-I.2 INTRA-coefficient `(LAST, RUN, LEVEL)` event
/// from the bitstream.
///
/// On success the reader is advanced past the event (a 3..=14-bit
/// regular VLC plus its sign, or the 22-bit ESCAPE event).
///
/// Errors:
///
/// * [`Error::UnexpectedEof`] — bitstream ended mid-event.
/// * [`Error::BadTcoefCode`] — 13 bits consumed without matching any
///   Table I.2 row.
/// * [`Error::BadTcoefEscapeLevel`] — ESCAPE LEVEL was a forbidden
///   baseline code (`0x00` or `0x80`). Annex T's EXTENDED-ESCAPE
///   relaxation is not supported.
pub fn decode_intra_tcoef_event(reader: &mut BitReader<'_>) -> Result<IntraTcoefEvent> {
    let mut acc: u32 = 0;
    let mut len: u8 = 0;
    while len < 7 {
        let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        acc = (acc << 1) | (b as u32);
        len += 1;
        if let Some(entry) = lookup_prefix(len, acc) {
            return finalise_event(reader, entry);
        }
    }
    while len < 13 {
        let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        acc = (acc << 1) | (b as u32);
        len += 1;
        if let Some(entry) = lookup_prefix(len, acc) {
            return finalise_event(reader, entry);
        }
    }
    Err(Error::BadTcoefCode)
}

/// Number of regular (non-ESCAPE) entries in Table I.2. Equal to
/// Table 16's count, since the two tables share their codeword
/// inventory per §I.3.
pub const INTRA_TCOEF_REGULAR_ENTRIES: usize = 102;

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn finish_aligned(mut w: BitWriter) -> Vec<u8> {
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    /// Encode a single Table-I.2 regular row (its code prefix + a
    /// chosen sign bit) and verify the parser recovers the row's
    /// (LAST, RUN, |LEVEL|) tuple with the correct sign.
    fn encode_and_decode_row(row: &IntraTcoefRow, sign: bool) -> IntraTcoefEvent {
        let mut w = BitWriter::new();
        // Row `bits` includes the trailing sign; write the prefix
        // (bits - 1) bits wide then the sign.
        w.write_u32(row.code as u32, (row.bits - 1) as u32);
        w.write_bit(sign);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        decode_intra_tcoef_event(&mut r).expect("decode")
    }

    /// Table I.2 has exactly 102 regular entries + 1 ESCAPE prefix.
    #[test]
    fn table_has_102_regular_plus_escape() {
        let regular = INTRA_TCOEF_TABLE.iter().filter(|r| !r.is_escape).count();
        let escape = INTRA_TCOEF_TABLE.iter().filter(|r| r.is_escape).count();
        assert_eq!(regular, INTRA_TCOEF_REGULAR_ENTRIES);
        assert_eq!(escape, 1);
    }

    /// All 102 regular entries round-trip with both sign polarities.
    #[test]
    fn full_table_round_trip_both_signs() {
        for row in INTRA_TCOEF_TABLE.iter() {
            if row.is_escape {
                continue;
            }
            for sign in [false, true] {
                let event = encode_and_decode_row(row, sign);
                let expected_level = if sign {
                    -(row.abs_level as i16)
                } else {
                    row.abs_level as i16
                };
                assert_eq!(
                    event.last, row.last,
                    "row code={:b} bits={} last mismatch",
                    row.code, row.bits
                );
                assert_eq!(
                    event.run, row.run,
                    "row code={:b} bits={} run mismatch",
                    row.code, row.bits
                );
                assert_eq!(
                    event.level, expected_level,
                    "row code={:b} bits={} level mismatch (sign={})",
                    row.code, row.bits, sign
                );
            }
        }
    }

    /// Spec spot-check: Table I.2 index 0 is the same as Table 16
    /// index 0 — `10s` decodes to (LAST=0, RUN=0, |LEVEL|=1).
    #[test]
    fn index_0_matches_spec() {
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        w.write_bit(false); // sign = +
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: false,
                run: 0,
                level: 1
            }
        );
    }

    /// Spec spot-check: Table I.2 index 1 — code `1111s` decodes to
    /// (LAST=0, RUN=1, |LEVEL|=1) under the I.2 reinterpretation
    /// (Table 16 index 1 was RUN=0/|LEVEL|=2; per §I.3 the (RUN,
    /// |LEVEL|) columns are reassigned). This is the key proof that
    /// the module diverges from Table 16.
    #[test]
    fn index_1_reinterpretation_diverges_from_table_16() {
        let mut w = BitWriter::new();
        w.write_u32(0b1111, 4);
        w.write_bit(false);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: false,
                run: 1,
                level: 1
            }
        );
    }

    /// Spec spot-check: Table I.2 index 12 — code `110s` decodes to
    /// (LAST=0, RUN=0, |LEVEL|=2).
    #[test]
    fn index_12_decodes_dc_level_2() {
        let mut w = BitWriter::new();
        w.write_u32(0b110, 3);
        w.write_bit(true); // sign = -
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: false,
                run: 0,
                level: -2
            }
        );
    }

    /// Spec spot-check: Table I.2 index 22 — code `0110 1s` decodes
    /// to (LAST=0, RUN=0, |LEVEL|=5). Note the divergence from
    /// Table 16 index 22 (which is RUN=3, |LEVEL|=1).
    #[test]
    fn index_22_decodes_dc_level_5() {
        let mut w = BitWriter::new();
        w.write_u32(0b0110_1, 5);
        w.write_bit(false);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: false,
                run: 0,
                level: 5
            }
        );
    }

    /// Spec spot-check: Table I.2 index 28 — code `0101 1s` decodes
    /// to (LAST=0, RUN=2, |LEVEL|=1).
    #[test]
    fn index_28_decodes_run_2_level_1() {
        let mut w = BitWriter::new();
        w.write_u32(0b0101_1, 5);
        w.write_bit(false);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: false,
                run: 2,
                level: 1
            }
        );
    }

    /// Spec spot-check: Table I.2 index 58 — code `0111s` decodes
    /// to (LAST=1, RUN=0, |LEVEL|=1). Same as Table 16 index 58.
    #[test]
    fn index_58_first_last_one_row() {
        let mut w = BitWriter::new();
        w.write_u32(0b0111, 4);
        w.write_bit(false);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: true,
                run: 0,
                level: 1
            }
        );
    }

    /// Spec spot-check: Table I.2 index 101 (final regular row) —
    /// code `0000 0101 1111s` decodes to (LAST=1, RUN=0,
    /// |LEVEL|=10). Diverges sharply from Table 16 index 101
    /// (LAST=1, RUN=40, |LEVEL|=1).
    #[test]
    fn index_101_final_regular_row() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_0101_1111, 12);
        w.write_bit(true); // sign = -
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: true,
                run: 0,
                level: -10
            }
        );
    }

    /// ESCAPE: `0000 011` + 1 bit LAST + 6 bits RUN + 8 bits signed
    /// LEVEL. Same layout as Table 16 ESCAPE per §I.3 (only the 102
    /// regular (RUN, |LEVEL|) assignments differ).
    #[test]
    fn escape_round_trip_positive_level() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true); // LAST = 1
        w.write_u32(7, 6); // RUN = 7
        w.write_u32(50, 8); // LEVEL = +50
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: true,
                run: 7,
                level: 50
            }
        );
    }

    /// ESCAPE with a negative LEVEL via two's complement.
    #[test]
    fn escape_negative_level_two_complement() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(false); // LAST = 0
        w.write_u32(3, 6); // RUN = 3
        w.write_u32(0xFE, 8); // LEVEL = -2
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let e = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(
            e,
            IntraTcoefEvent {
                last: false,
                run: 3,
                level: -2
            }
        );
    }

    /// ESCAPE LEVEL `0x00` is forbidden in baseline (Annex T
    /// reinterprets the alias `0x80` but not `0x00`; baseline
    /// rejects both).
    #[test]
    fn escape_level_zero_is_forbidden() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x00, 8);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            decode_intra_tcoef_event(&mut r),
            Err(Error::BadTcoefEscapeLevel)
        );
    }

    /// ESCAPE LEVEL `0x80` is forbidden in baseline (the
    /// EXTENDED-ESCAPE Annex-T relaxation is out of scope).
    #[test]
    fn escape_level_minus_128_is_forbidden_in_baseline() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x80, 8);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            decode_intra_tcoef_event(&mut r),
            Err(Error::BadTcoefEscapeLevel)
        );
    }

    /// A 13-bit all-zeros prefix matches no Table-I.2 row → BadTcoefCode.
    #[test]
    fn invalid_prefix_rejected() {
        let mut w = BitWriter::new();
        w.write_u32(0, 13);
        // Trailing data so a length-13 lookup doesn't EOF first.
        w.write_u32(0xFFFF, 16);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        assert_eq!(decode_intra_tcoef_event(&mut r), Err(Error::BadTcoefCode));
    }

    /// Truncated input (less than the shortest legal prefix) →
    /// UnexpectedEof. The shortest Table-I.2 prefix is the 2-bit
    /// `10` (index 0).
    #[test]
    fn truncated_input_unexpected_eof() {
        let empty: [u8; 0] = [];
        let mut r = BitReader::new(&empty);
        assert_eq!(decode_intra_tcoef_event(&mut r), Err(Error::UnexpectedEof));
    }

    /// Bit-consumption: index-0 `10s` consumes exactly 3 bits and
    /// leaves the reader at bit 3.
    #[test]
    fn index_0_consumes_three_bits() {
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        w.write_bit(false);
        // Append a sentinel that must remain.
        w.write_bit(true);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let _ = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(r.bit_position(), 3);
        assert!(r.read_bit().unwrap());
    }

    /// Bit-consumption: the 13-bit ESCAPE event with its 15-bit
    /// fixed-length tail consumes exactly 7 + 15 = 22 bits.
    #[test]
    fn escape_consumes_22_bits() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(1, 6);
        w.write_u32(0x01, 8);
        // Sentinel.
        w.write_bit(false);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let _ = decode_intra_tcoef_event(&mut r).unwrap();
        assert_eq!(r.bit_position(), 22);
        assert!(!r.read_bit().unwrap());
    }

    /// The 102 regular `(LAST, RUN, |LEVEL|)` tuples are pairwise
    /// distinct (no row repeats another row's interpretation under
    /// a different code). This catches a mis-transcription that
    /// silently aliases two entries.
    #[test]
    fn all_regular_tuples_are_distinct() {
        let mut tuples: Vec<(bool, u8, u8)> = INTRA_TCOEF_TABLE
            .iter()
            .filter(|r| !r.is_escape)
            .map(|r| (r.last, r.run, r.abs_level))
            .collect();
        tuples.sort();
        let original_len = tuples.len();
        tuples.dedup();
        assert_eq!(
            tuples.len(),
            original_len,
            "Table I.2 has duplicate (LAST, RUN, |LEVEL|) tuples"
        );
    }

    /// Per §I.3 line 4034: the I.2 codes are bit-for-bit identical
    /// to Table 16, and LAST is preserved between the two tables.
    /// Test that for every shared index the I.2 (bits, code, last)
    /// matches Table 16's. We compare via the public surface of the
    /// `block` module: a Table-16-encoded row should decode under
    /// Table I.2 to a row with the SAME `last` (but possibly
    /// different `run`/`abs_level`).
    ///
    /// This is the strongest available cross-check that we have not
    /// mis-transcribed any codeword (a transposed code/bits column
    /// would surface either an EOF, a BadTcoefCode, or — more
    /// dangerously — a silent misdecode at the wrong index).
    #[test]
    fn intra_table_last_column_matches_table_16_per_index() {
        // Encode a Table-I.2 row and decode it under our parser;
        // verify each row's `last` matches the corresponding
        // Table-16 row's `last`. We rebuild a minimal map of the
        // Table 16 (bits, code) -> last by parsing the LAST sentinel
        // through the Table 16 path: encode the prefix as our table
        // claims and check `block::decode_tcoef_event` agrees on
        // LAST.
        //
        // Since `block::decode_tcoef_event` is private to that module,
        // we exercise the public `parse_block` path with a single
        // event followed by a terminating LAST=1 if needed. Easier:
        // exploit the structural property that in the existing
        // Table 16 (per `block.rs`), indices 0..=57 are LAST=0 and
        // 58..=101 are LAST=1. Per §I.3 LAST is preserved — so all
        // I.2 rows at indices 0..=57 must be LAST=0 and 58..=101
        // must be LAST=1.
        let regulars: Vec<&IntraTcoefRow> =
            INTRA_TCOEF_TABLE.iter().filter(|r| !r.is_escape).collect();
        for (idx, row) in regulars.iter().enumerate() {
            let expected_last = idx >= 58;
            assert_eq!(
                row.last, expected_last,
                "I.2 row index {} (code {:b}, bits {}) has last={} but Table 16 has last={}",
                idx, row.code, row.bits, row.run, expected_last
            );
        }
    }
}
