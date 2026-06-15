//! H.263 block-layer parsing (§5.4).
//!
//! This module implements the structural decode of the block-layer
//! data as defined in ITU-T Recommendation H.263 (01/2005) §5.4.
//! Per Figure 11/H.263 each of the six blocks of a macroblock
//! comprises:
//!
//! ```text
//!   INTRADC   TCOEF   TCOEF   ...   TCOEF (LAST=1)
//! ```
//!
//! Round 4 (post orphan-rebuild) decodes the **non-PB-frame, non-Annex-T,
//! non-Annex-I baseline** subset of those fields:
//!
//! * §5.4.1 — **INTRADC** (8 bits, FLC). Present for *every* block of
//!   an INTRA macroblock (MB types 3 / 4). The 8-bit code maps to a
//!   reconstruction level per Table 15: codes `0000 0000` and
//!   `1000 0000` are forbidden, and code `1111 1111` represents
//!   reconstruction level 1024 (the slot where the natural-binary
//!   `1000 0000` would have been). All other codes `(code << 3)` give
//!   the reconstruction level.
//! * §5.4.2 — **TCOEF** (variable length, Table 16). 102 VLC
//!   code-points each followed by a single sign bit; one fall-through
//!   ESCAPE prefix (`0000 011`, 7 bits) followed by a fixed-length
//!   22-bit total event of `LAST (1 bit) || RUN (6 bits) || LEVEL
//!   (8 bits, signed two's-complement, codes `0000 0000` and
//!   `1000 0000` forbidden in baseline)`.
//!
//! TCOEF events are accumulated into a 64-entry `coefficients` array
//! laid out in **zigzag scan position order** (`coefficients[0]` is
//! the DC coefficient — for INTRA blocks this slot holds the decoded
//! INTRADC level pre-IDCT, scaled per Table 15 — `coefficients[63]`
//! is the bottom-right AC). The zigzag-to-block-position mapping
//! itself is defined by [`ZIGZAG_TO_BLOCK_POS`] below per Figure 14.
//! Round 4 stops at coefficient collection — dequantisation
//! (§6.2.1), zigzag-to-block placement (§6.2.3 / Figure 14),
//! and the inverse transform (§6.2.4) are all deferred.
//!
//! ## Deliberately deferred
//!
//! * §6.2.1 — Inverse quantisation. The coefficients we collect are
//!   the raw `LEVEL` integers from the bitstream, sign-applied but
//!   **not** scaled by QUANT. The dequantisation `|REC| = QUANT ·
//!   (2 · |LEVEL| + 1) [- 1 if QUANT even]` will be applied in a
//!   later round once we have a frame-yielding decoder.
//! * §6.2.3 — Block placement. The output array is in *zigzag scan
//!   position order*, not in 8×8 block layout. The caller can apply
//!   [`ZIGZAG_TO_BLOCK_POS`] to scatter into an 8×8 buffer.
//! * §6.2.4 — Inverse DCT.
//! * Annex I (Advanced INTRA Coding) — uses alternate scans and
//!   places INTRADC as a regular AC-coded value.
//! * Annex T (Modified Quantization) — extends the LEVEL field to
//!   allow the `1000 0000` code as an EXTENDED-ESCAPE prefix.
//! * PB-frames B-block data (§5.4 last paragraph + Annex G).
//!
//! ## Composing with macroblock layer
//!
//! [`parse_block`] takes a [`BlockContext`] describing whether the
//! block has an INTRADC prefix and whether any TCOEFs follow. The
//! caller derives those two booleans from the surrounding
//! macroblock's MB type (INTRA → INTRADC present) and the relevant
//! CBP bit (CBPY for luma blocks 1-4, CBPC for chrominance blocks
//! 5-6). The CBP bit's semantics per §5.3.2 + §5.3.5 are:
//!
//! * For INTER macroblocks — the CBP bit is `1` iff *any* coefficient
//!   is present in this block.
//! * For INTRA macroblocks — INTRADC is *always* present; the CBP
//!   bit only governs whether *additional* TCOEF (AC) coefficients
//!   follow the INTRADC.

// All numeric literals in this module are bit-pattern transcriptions
// of ITU-T H.263 Tables 15 and 16. Like macroblock.rs, we group
// digits to mirror the spec's printed MSB-first layout
// (e.g. "0000 1000 01s" → `0b0000_1000_01`) rather than the
// power-of-two grouping clippy prefers; that grouping is what makes
// the lines auditable against the spec.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Number of coefficients in an H.263 8x8 transform block.
pub const COEFFS_PER_BLOCK: usize = 64;

/// Zigzag-position → block-position lookup per Figure 14/H.263.
///
/// The spec prints Figure 14 using 1-based indices (coefficient 1
/// is DC at block position `(0, 0)`, coefficient 64 is the last AC at
/// position `(7, 7)`). We convert to 0-based: `ZIGZAG_TO_BLOCK_POS[i]`
/// gives the 0..=63 block position (`row * 8 + col`) of the i-th
/// coefficient in scan order, for `i` in 0..=63.
///
/// Reading Figure 14 row-by-row:
///
/// ```text
///   row 0:  1   2   6   7  15  16  28  29
///   row 1:  3   5   8  14  17  27  30  43
///   row 2:  4   9  13  18  26  31  42  44
///   row 3: 10  12  19  25  32  41  45  54
///   row 4: 11  20  24  33  40  46  53  55
///   row 5: 21  23  34  39  47  52  56  61
///   row 6: 22  35  38  48  51  57  60  62
///   row 7: 36  37  49  50  58  59  63  64
/// ```
///
/// Building the inverse: for each 1-based scan index `n` find its
/// row/col in the figure and store `(row * 8 + col)` at index
/// `n - 1`.
pub const ZIGZAG_TO_BLOCK_POS: [u8; COEFFS_PER_BLOCK] = [
    // Scan-index 1..8
    0, 1, 8, 16, 9, 2, 3, 10, // 1, 2, 3, 4, 5, 6, 7, 8
    // Scan-index 9..16
    17, 24, 32, 25, 18, 11, 4, 5, // 9..16
    // Scan-index 17..24
    12, 19, 26, 33, 40, 48, 41, 34, // 17..24
    // Scan-index 25..32
    27, 20, 13, 6, 7, 14, 21, 28, // 25..32
    // Scan-index 33..40
    35, 42, 49, 56, 57, 50, 43, 36, // 33..40
    // Scan-index 41..48
    29, 22, 15, 23, 30, 37, 44, 51, // 41..48
    // Scan-index 49..56
    58, 59, 52, 45, 38, 31, 39, 46, // 49..56
    // Scan-index 57..64
    53, 60, 61, 54, 47, 55, 62, 63, // 57..64
];

/// Per-block context the parser needs from the surrounding macroblock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BlockContext {
    /// `true` if INTRADC (§5.4.1) is on the wire for this block.
    /// Per §5.4: present for *every* block when the macroblock is
    /// INTRA (MB type 3 or 4). Absent otherwise.
    pub has_intradc: bool,
    /// `true` if any TCOEF (§5.4.2) events follow. Derived by the
    /// caller from the CBP bit for this block (CBPYn for luma 1-4,
    /// CBPC5 for Cb, CBPC6 for Cr). For INTER blocks the bit is the
    /// presence of *any* coefficient; for INTRA blocks the bit is
    /// the presence of *non-INTRADC* (i.e. AC) coefficients.
    pub has_coefficients: bool,
    /// `true` if Annex T Modified Quantization mode (§T.4) is in
    /// use for this block. When set, the §5.4.2 ESCAPE LEVEL field
    /// `1000 0000` is no longer forbidden: it is the
    /// EXTENDED-ESCAPE marker introducing an 11-bit EXTENDED-LEVEL
    /// field (§T.4 / Figure T.1) that represents AC coefficient
    /// magnitudes greater than 127. When clear, the baseline
    /// §5.4.2 rule applies (`1000 0000` is forbidden).
    pub modified_quant: bool,
}

/// A single 8x8 transform-coefficient block as decoded from the
/// bitstream. Coefficients are stored in **zigzag scan position**
/// order (`coefficients[0]` is the DC slot).
///
/// For INTRA blocks the DC slot holds the INTRADC reconstruction
/// level from Table 15 (`8 * code`, or 1024 when the wire was
/// `1111 1111`). For INTER blocks the DC slot is either 0 or the
/// `(LAST=0, RUN=0)` AC LEVEL written by the encoder — the spec
/// doesn't distinguish, since INTER blocks have no INTRADC.
///
/// AC slots hold the raw signed `LEVEL` integers from the bitstream
/// — neither dequantised (§6.2.1) nor placed back into 8x8 layout
/// (§6.2.3 / Figure 14, see [`ZIGZAG_TO_BLOCK_POS`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct H263Block {
    /// The 64 quantised coefficients in zigzag scan order. Slots
    /// past the LAST=1 event of the bitstream stay zero.
    pub coefficients: [i16; COEFFS_PER_BLOCK],
    /// Number of (LAST, RUN, LEVEL) TCOEF events the parser
    /// consumed for this block (the count *includes* the
    /// terminating LAST=1 event, and *excludes* the INTRADC).
    /// Zero for an INTER block with `has_coefficients == false`,
    /// or for an INTRA block whose CBP bit is zero (in which case
    /// only the INTRADC is read).
    pub tcoef_event_count: u32,
    /// `true` if [`BlockContext::has_intradc`] was set and the
    /// 8-bit INTRADC was read off the wire. Convenience for the
    /// caller's debug formatting.
    pub had_intradc: bool,
}

impl H263Block {
    /// Build an all-zero block (no INTRADC, no TCOEFs). Used when
    /// the caller's `BlockContext` skips both.
    pub fn empty() -> Self {
        H263Block {
            coefficients: [0; COEFFS_PER_BLOCK],
            tcoef_event_count: 0,
            had_intradc: false,
        }
    }
}

/// Parse a single H.263 block per §5.4. See module-level docs and
/// [`BlockContext`] for what the parser expects.
///
/// On success the reader is left at the first bit after the
/// block's TCOEF terminator (or at the first bit after the
/// INTRADC if no TCOEFs follow, or at the input position if the
/// block context skipped both). On error the reader position is
/// unspecified.
pub fn parse_block(reader: &mut BitReader<'_>, ctx: BlockContext) -> Result<H263Block> {
    let mut block = H263Block::empty();

    // §5.4.1 — INTRADC (8 bits, FLC) per Table 15.
    if ctx.has_intradc {
        let code = reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8;
        let rec = intradc_reconstruction(code)?;
        block.coefficients[0] = rec;
        block.had_intradc = true;
    }

    // §5.4.2 — TCOEF events (variable length) per Table 16.
    if ctx.has_coefficients {
        // Coefficients accumulate at INTRADC + 1 (slot 1, the first
        // AC zigzag position) for INTRA blocks, and at slot 0 for
        // INTER blocks (no INTRADC means the first AC could be at
        // slot 0 if RUN==0). The first AC slot we may write to is
        // therefore `1` for INTRA blocks and `0` for INTER blocks.
        let mut scan_pos: usize = if ctx.has_intradc { 1 } else { 0 };

        loop {
            let event = decode_tcoef_event(reader, ctx.modified_quant)?;
            // Advance past `RUN` zero coefficients, then write LEVEL.
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
                // The next event would have RUN >= 0 at scan_pos,
                // but we already filled slot 63 — no room left.
                return Err(Error::BadTcoefRunOverflow);
            }
        }
    }

    Ok(block)
}

/// A single (LAST, RUN, LEVEL) TCOEF event decoded from the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TcoefEvent {
    last: bool,
    run: u8,
    level: i16,
}

/// Reconstruction level for an 8-bit INTRADC code per Table 15.
///
/// The mapping is:
///
/// * `0x00` — FORBIDDEN.
/// * `0x80` — FORBIDDEN (the natural slot for value 1024).
/// * `0xFF` — special case: reconstruction level 1024 (uses the
///   `0xFF` slot to free `0x80` for future escape use).
/// * everything else — `level = (code as i16) * 8`.
fn intradc_reconstruction(code: u8) -> Result<i16> {
    match code {
        0x00 | 0x80 => Err(Error::BadIntradcCode),
        0xFF => Ok(1024),
        _ => Ok((code as i16) * 8),
    }
}

/// Decode one (LAST, RUN, LEVEL) TCOEF event per §5.4.2 / Table 16.
///
/// Returns the event with `level` carrying the **signed** integer
/// (the sign bit `s` from the table is folded in; ESCAPE LEVEL is
/// interpreted as two's-complement 8-bit).
fn decode_tcoef_event(reader: &mut BitReader<'_>, modified_quant: bool) -> Result<TcoefEvent> {
    // Decode by reading bits incrementally and matching against the
    // 103-row table (102 VLC entries + 1 ESCAPE prefix). Each row
    // gives `(bit_count, code_msb_left, last, run, abs_level)` where
    // `abs_level == 0` means "ESCAPE — read 1+6+8 = 15 more bits".
    let mut acc: u32 = 0;
    let mut len: u8 = 0;
    while len < 7 {
        let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        acc = (acc << 1) | (b as u32);
        len += 1;
        if let Some(entry) = lookup_tcoef_prefix(len, acc) {
            return finalise_tcoef(reader, entry, modified_quant);
        }
    }
    // Beyond 7 bits — keep reading up to 13.
    while len < 13 {
        let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        acc = (acc << 1) | (b as u32);
        len += 1;
        if let Some(entry) = lookup_tcoef_prefix(len, acc) {
            return finalise_tcoef(reader, entry, modified_quant);
        }
    }
    Err(Error::BadTcoefCode)
}

/// Outcome of a TCOEF table lookup: either a regular VLC entry that
/// still needs its sign bit, or the ESCAPE prefix.
#[derive(Debug, Clone, Copy)]
enum TcoefEntry {
    /// Regular VLC code-point. The caller still has to read the
    /// sign bit and apply the sign.
    Vlc { last: bool, run: u8, abs_level: u8 },
    /// ESCAPE prefix — caller reads 1 bit LAST, 6 bits RUN, 8 bits
    /// LEVEL (two's-complement) and applies forbidden-code checks.
    Escape,
}

/// Lookup a TCOEF VLC prefix in Table 16. Returns `Some(entry)` if
/// the `(bits, code)` pair matches a row's prefix (i.e. the spec's
/// printed bit-count minus the trailing sign bit for non-ESCAPE
/// rows; the ESCAPE row has no trailing sign and its `bits` field
/// matches directly).
fn lookup_tcoef_prefix(bits: u8, code: u32) -> Option<TcoefEntry> {
    for &row in TCOEF_TABLE.iter() {
        // For non-ESCAPE rows the table stores the spec's printed
        // "Bits" column which includes the trailing sign bit; we
        // compare against `bits - 1` to match the prefix only and
        // leave the sign to `finalise_tcoef`.
        let prefix_bits = if row.is_escape {
            row.bits
        } else {
            row.bits - 1
        };
        if prefix_bits == bits && row.code as u32 == code {
            return Some(if row.is_escape {
                TcoefEntry::Escape
            } else {
                TcoefEntry::Vlc {
                    last: row.last,
                    run: row.run,
                    abs_level: row.abs_level,
                }
            });
        }
    }
    None
}

/// Apply the trailing sign bit (or the ESCAPE-mode fixed-length
/// fields) to produce a fully-decoded [`TcoefEvent`].
///
/// `modified_quant` selects the Annex T §T.4 interpretation of the
/// ESCAPE LEVEL field: when set, the 8-bit LEVEL value `1000 0000`
/// is the EXTENDED-ESCAPE marker (rather than a forbidden code) and
/// is followed by an 11-bit EXTENDED-LEVEL field.
fn finalise_tcoef(
    reader: &mut BitReader<'_>,
    entry: TcoefEntry,
    modified_quant: bool,
) -> Result<TcoefEvent> {
    match entry {
        TcoefEntry::Vlc {
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
            Ok(TcoefEvent { last, run, level })
        }
        TcoefEntry::Escape => {
            // §5.4.2: 1 bit LAST, 6 bits RUN, 8 bits LEVEL.
            let last_bit = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            let run = reader.read_u32(6).map_err(|_| Error::UnexpectedEof)? as u8;
            let level_bits = reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8;
            // §5.4.2 baseline forbids `0000 0000` and `1000 0000`.
            // §T.4 (Modified Quantization mode) re-purposes `1000 0000`
            // as the EXTENDED-ESCAPE marker; `0000 0000` stays forbidden.
            if level_bits == 0x00 {
                return Err(Error::BadTcoefEscapeLevel);
            }
            if level_bits == 0x80 {
                if !modified_quant {
                    return Err(Error::BadTcoefEscapeLevel);
                }
                // §T.4 EXTENDED-ESCAPE: an 11-bit EXTENDED-LEVEL field
                // follows, carrying an AC LEVEL with magnitude > 127.
                let wire = reader.read_u32(11).map_err(|_| Error::UnexpectedEof)? as u16;
                let level = extended_level_from_wire(wire);
                return Ok(TcoefEvent {
                    last: last_bit,
                    run,
                    level,
                });
            }
            // Two's complement: bit 7 set means negative.
            let level: i16 = (level_bits as i8) as i16;
            Ok(TcoefEvent {
                last: last_bit,
                run,
                level,
            })
        }
    }
}

/// Decode the §T.4 EXTENDED-LEVEL field into a signed `LEVEL`.
///
/// `wire` holds the 11 bits read MSB-first off the bitstream
/// (only the low 11 bits are significant). Per §T.4 / Figure T.1
/// the encoder takes the least-significant 11 bits of the two's-
/// complement representation of LEVEL and cyclically rotates them
/// **right** by 5 positions to form the wire value (the rotation
/// prevents start-code emulation). Decoding therefore rotates the
/// 11-bit wire value cyclically **left** by 5 positions to recover
/// the 11-bit two's-complement LEVEL, which is then sign-extended.
///
/// Wire layout (MSB→LSB):  b5 b4 b3 b2 b1 b11 b10 b9 b8 b7 b6
/// Recovered LEVEL (MSB→LSB): b11 b10 b9 b8 b7 b6 b5 b4 b3 b2 b1
pub(crate) fn extended_level_from_wire(wire: u16) -> i16 {
    const MASK: u16 = 0x07FF; // low 11 bits
    let w = wire & MASK;
    // Cyclic left-rotate by 5 within an 11-bit field.
    let rotated = ((w << 5) | (w >> (11 - 5))) & MASK;
    // Sign-extend the 11-bit two's-complement value (bit 10 = sign).
    if rotated & 0x0400 != 0 {
        (rotated as i16) - 0x0800
    } else {
        rotated as i16
    }
}

/// One row of Table 16/H.263 (TCOEF VLC). `is_escape` flags the
/// single ESCAPE entry (index 102, code `0000 011`); for that row
/// the `last` / `run` / `abs_level` fields are unused.
///
/// `bits` is the spec's printed "Bits" column verbatim — i.e. the
/// total length **including** the trailing sign bit `s` for regular
/// VLC rows, and the literal length (7) for the ESCAPE row. The
/// lookup function compares `bits - 1` for the former and `bits`
/// for the latter; the sign bit is consumed separately by
/// [`finalise_tcoef`].
#[derive(Debug, Clone, Copy)]
struct TcoefRow {
    bits: u8,
    code: u16,
    last: bool,
    run: u8,
    abs_level: u8,
    is_escape: bool,
}

/// Convenience constructor for a regular (non-ESCAPE) row.
const fn vlc_row(bits: u8, code: u16, last: bool, run: u8, abs_level: u8) -> TcoefRow {
    TcoefRow {
        bits,
        code,
        last,
        run,
        abs_level,
        is_escape: false,
    }
}

/// Convenience constructor for the ESCAPE row.
const fn escape_row(bits: u8, code: u16) -> TcoefRow {
    TcoefRow {
        bits,
        code,
        last: false,
        run: 0,
        abs_level: 0,
        is_escape: true,
    }
}

/// Table 16/H.263 — TCOEF VLC. Index column from the spec is *not*
/// used by the lookup (we search by `(bits, code)`); we keep it in
/// the comment for spec-line alignment. The `bits` value omits the
/// trailing sign bit `s` (which is read separately for non-ESCAPE
/// rows).
const TCOEF_TABLE: &[TcoefRow] = &[
    // idx, last, run, |level|, total bits w/ sign, VLC w/o sign
    // Idx 0..11 — LAST=0, RUN=0
    vlc_row(3, 0b10, false, 0, 1),              // 0: 10s
    vlc_row(5, 0b1111, false, 0, 2),            // 1: 1111s
    vlc_row(7, 0b0101_01, false, 0, 3),         // 2: 0101 01s
    vlc_row(8, 0b0010_111, false, 0, 4),        // 3: 0010 111s
    vlc_row(9, 0b0001_1111, false, 0, 5),       // 4: 0001 1111s
    vlc_row(10, 0b0001_0010_1, false, 0, 6),    // 5: 0001 0010 1s
    vlc_row(10, 0b0001_0010_0, false, 0, 7),    // 6: 0001 0010 0s
    vlc_row(11, 0b0000_1000_01, false, 0, 8),   // 7: 0000 1000 01s
    vlc_row(11, 0b0000_1000_00, false, 0, 9),   // 8: 0000 1000 00s
    vlc_row(12, 0b0000_0000_111, false, 0, 10), // 9: 0000 0000 111s
    vlc_row(12, 0b0000_0000_110, false, 0, 11), // 10: 0000 0000 110s
    vlc_row(12, 0b0000_0100_000, false, 0, 12), // 11: 0000 0100 000s
    // Idx 12..17 — LAST=0, RUN=1
    vlc_row(4, 0b110, false, 1, 1),             // 12: 110s
    vlc_row(7, 0b0101_00, false, 1, 2),         // 13: 0101 00s
    vlc_row(9, 0b0001_1110, false, 1, 3),       // 14: 0001 1110s
    vlc_row(11, 0b0000_0011_11, false, 1, 4),   // 15: 0000 0011 11s
    vlc_row(12, 0b0000_0100_001, false, 1, 5),  // 16: 0000 0100 001s
    vlc_row(13, 0b0000_0101_0000, false, 1, 6), // 17: 0000 0101 0000s
    // Idx 18..21 — LAST=0, RUN=2
    vlc_row(5, 0b1110, false, 2, 1),            // 18: 1110s
    vlc_row(9, 0b0001_1101, false, 2, 2),       // 19: 0001 1101s
    vlc_row(11, 0b0000_0011_10, false, 2, 3),   // 20: 0000 0011 10s
    vlc_row(13, 0b0000_0101_0001, false, 2, 4), // 21: 0000 0101 0001s
    // Idx 22..24 — LAST=0, RUN=3
    vlc_row(6, 0b0110_1, false, 3, 1),        // 22: 0110 1s
    vlc_row(10, 0b0001_0001_1, false, 3, 2),  // 23: 0001 0001 1s
    vlc_row(11, 0b0000_0011_01, false, 3, 3), // 24: 0000 0011 01s
    // Idx 25..27 — LAST=0, RUN=4
    vlc_row(6, 0b0110_0, false, 4, 1),          // 25: 0110 0s
    vlc_row(10, 0b0001_0001_0, false, 4, 2),    // 26: 0001 0001 0s
    vlc_row(13, 0b0000_0101_0010, false, 4, 3), // 27: 0000 0101 0010s
    // Idx 28..30 — LAST=0, RUN=5
    vlc_row(6, 0b0101_1, false, 5, 1),          // 28: 0101 1s
    vlc_row(11, 0b0000_0011_00, false, 5, 2),   // 29: 0000 0011 00s
    vlc_row(13, 0b0000_0101_0011, false, 5, 3), // 30: 0000 0101 0011s
    // Idx 31..33 — LAST=0, RUN=6
    vlc_row(7, 0b0100_11, false, 6, 1),         // 31: 0100 11s
    vlc_row(11, 0b0000_0010_11, false, 6, 2),   // 32: 0000 0010 11s
    vlc_row(13, 0b0000_0101_0100, false, 6, 3), // 33: 0000 0101 0100s
    // Idx 34..35 — LAST=0, RUN=7
    vlc_row(7, 0b0100_10, false, 7, 1),       // 34: 0100 10s
    vlc_row(11, 0b0000_0010_10, false, 7, 2), // 35: 0000 0010 10s
    // Idx 36..37 — LAST=0, RUN=8
    vlc_row(7, 0b0100_01, false, 8, 1),       // 36: 0100 01s
    vlc_row(11, 0b0000_0010_01, false, 8, 2), // 37: 0000 0010 01s
    // Idx 38..39 — LAST=0, RUN=9
    vlc_row(7, 0b0100_00, false, 9, 1),       // 38: 0100 00s
    vlc_row(11, 0b0000_0010_00, false, 9, 2), // 39: 0000 0010 00s
    // Idx 40..41 — LAST=0, RUN=10
    vlc_row(8, 0b0010_110, false, 10, 1),        // 40: 0010 110s
    vlc_row(13, 0b0000_0101_0101, false, 10, 2), // 41: 0000 0101 0101s
    // Idx 42 — LAST=0, RUN=11
    vlc_row(8, 0b0010_101, false, 11, 1), // 42: 0010 101s
    // Idx 43 — LAST=0, RUN=12
    vlc_row(8, 0b0010_100, false, 12, 1), // 43: 0010 100s
    // Idx 44 — LAST=0, RUN=13
    vlc_row(9, 0b0001_1100, false, 13, 1), // 44: 0001 1100s
    // Idx 45 — LAST=0, RUN=14
    vlc_row(9, 0b0001_1011, false, 14, 1), // 45: 0001 1011s
    // Idx 46 — LAST=0, RUN=15
    vlc_row(10, 0b0001_0000_1, false, 15, 1), // 46: 0001 0000 1s
    // Idx 47 — LAST=0, RUN=16
    vlc_row(10, 0b0001_0000_0, false, 16, 1), // 47: 0001 0000 0s
    // Idx 48..57 — LAST=0, RUN=17..26
    vlc_row(10, 0b0000_1111_1, false, 17, 1), // 48: 0000 1111 1s
    vlc_row(10, 0b0000_1111_0, false, 18, 1), // 49: 0000 1111 0s
    vlc_row(10, 0b0000_1110_1, false, 19, 1), // 50: 0000 1110 1s
    vlc_row(10, 0b0000_1110_0, false, 20, 1), // 51: 0000 1110 0s
    vlc_row(10, 0b0000_1101_1, false, 21, 1), // 52: 0000 1101 1s
    vlc_row(10, 0b0000_1101_0, false, 22, 1), // 53: 0000 1101 0s
    vlc_row(12, 0b0000_0100_010, false, 23, 1), // 54: 0000 0100 010s
    vlc_row(12, 0b0000_0100_011, false, 24, 1), // 55: 0000 0100 011s
    vlc_row(13, 0b0000_0101_0110, false, 25, 1), // 56: 0000 0101 0110s
    vlc_row(13, 0b0000_0101_0111, false, 26, 1), // 57: 0000 0101 0111s
    // Idx 58..60 — LAST=1, RUN=0
    vlc_row(5, 0b0111, true, 0, 1),           // 58: 0111s
    vlc_row(10, 0b0000_1100_1, true, 0, 2),   // 59: 0000 1100 1s
    vlc_row(12, 0b0000_0000_101, true, 0, 3), // 60: 0000 0000 101s
    // Idx 61..62 — LAST=1, RUN=1
    vlc_row(7, 0b0011_11, true, 1, 1),        // 61: 0011 11s
    vlc_row(12, 0b0000_0000_100, true, 1, 2), // 62: 0000 0000 100s
    // Idx 63..69 — LAST=1, RUN=2..8 (all |LEVEL| = 1)
    vlc_row(7, 0b0011_10, true, 2, 1),  // 63: 0011 10s
    vlc_row(7, 0b0011_01, true, 3, 1),  // 64: 0011 01s
    vlc_row(7, 0b0011_00, true, 4, 1),  // 65: 0011 00s
    vlc_row(8, 0b0010_011, true, 5, 1), // 66: 0010 011s
    vlc_row(8, 0b0010_010, true, 6, 1), // 67: 0010 010s
    vlc_row(8, 0b0010_001, true, 7, 1), // 68: 0010 001s
    vlc_row(8, 0b0010_000, true, 8, 1), // 69: 0010 000s
    // Idx 70..77 — LAST=1, RUN=9..16 (|LEVEL| = 1)
    vlc_row(9, 0b0001_1010, true, 9, 1),  // 70: 0001 1010s
    vlc_row(9, 0b0001_1001, true, 10, 1), // 71: 0001 1001s
    vlc_row(9, 0b0001_1000, true, 11, 1), // 72: 0001 1000s
    vlc_row(9, 0b0001_0111, true, 12, 1), // 73: 0001 0111s
    vlc_row(9, 0b0001_0110, true, 13, 1), // 74: 0001 0110s
    vlc_row(9, 0b0001_0101, true, 14, 1), // 75: 0001 0101s
    vlc_row(9, 0b0001_0100, true, 15, 1), // 76: 0001 0100s
    vlc_row(9, 0b0001_0011, true, 16, 1), // 77: 0001 0011s
    // Idx 78..85 — LAST=1, RUN=17..24 (|LEVEL| = 1)
    vlc_row(10, 0b0000_1100_0, true, 17, 1), // 78: 0000 1100 0s
    vlc_row(10, 0b0000_1011_1, true, 18, 1), // 79: 0000 1011 1s
    vlc_row(10, 0b0000_1011_0, true, 19, 1), // 80: 0000 1011 0s
    vlc_row(10, 0b0000_1010_1, true, 20, 1), // 81: 0000 1010 1s
    vlc_row(10, 0b0000_1010_0, true, 21, 1), // 82: 0000 1010 0s
    vlc_row(10, 0b0000_1001_1, true, 22, 1), // 83: 0000 1001 1s
    vlc_row(10, 0b0000_1001_0, true, 23, 1), // 84: 0000 1001 0s
    vlc_row(10, 0b0000_1000_1, true, 24, 1), // 85: 0000 1000 1s
    // Idx 86..89 — LAST=1, RUN=25..28 (|LEVEL| = 1)
    vlc_row(11, 0b0000_0001_11, true, 25, 1), // 86: 0000 0001 11s
    vlc_row(11, 0b0000_0001_10, true, 26, 1), // 87: 0000 0001 10s
    vlc_row(11, 0b0000_0001_01, true, 27, 1), // 88: 0000 0001 01s
    vlc_row(11, 0b0000_0001_00, true, 28, 1), // 89: 0000 0001 00s
    // Idx 90..93 — LAST=1, RUN=29..32 (|LEVEL| = 1)
    vlc_row(12, 0b0000_0100_100, true, 29, 1), // 90: 0000 0100 100s
    vlc_row(12, 0b0000_0100_101, true, 30, 1), // 91: 0000 0100 101s
    vlc_row(12, 0b0000_0100_110, true, 31, 1), // 92: 0000 0100 110s
    vlc_row(12, 0b0000_0100_111, true, 32, 1), // 93: 0000 0100 111s
    // Idx 94..101 — LAST=1, RUN=33..40 (|LEVEL| = 1)
    vlc_row(13, 0b0000_0101_1000, true, 33, 1), // 94
    vlc_row(13, 0b0000_0101_1001, true, 34, 1), // 95
    vlc_row(13, 0b0000_0101_1010, true, 35, 1), // 96
    vlc_row(13, 0b0000_0101_1011, true, 36, 1), // 97
    vlc_row(13, 0b0000_0101_1100, true, 37, 1), // 98
    vlc_row(13, 0b0000_0101_1101, true, 38, 1), // 99
    vlc_row(13, 0b0000_0101_1110, true, 39, 1), // 100
    vlc_row(13, 0b0000_0101_1111, true, 40, 1), // 101
    // Idx 102 — ESCAPE (7 bits, no trailing sign on the prefix).
    escape_row(7, 0b0000_011), // 102: 0000 011 (then 1+6+8 fixed)
];

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn intra_ctx_with_coefs() -> BlockContext {
        BlockContext {
            has_intradc: true,
            has_coefficients: true,
            ..Default::default()
        }
    }

    fn intra_ctx_dc_only() -> BlockContext {
        BlockContext {
            has_intradc: true,
            has_coefficients: false,
            ..Default::default()
        }
    }

    fn inter_ctx_with_coefs() -> BlockContext {
        BlockContext {
            has_intradc: false,
            has_coefficients: true,
            ..Default::default()
        }
    }

    fn inter_ctx_empty() -> BlockContext {
        BlockContext {
            has_intradc: false,
            has_coefficients: false,
            ..Default::default()
        }
    }

    /// INTER block context with Annex T Modified Quantization (§T.4)
    /// active, so the ESCAPE LEVEL `1000 0000` is the EXTENDED-ESCAPE
    /// marker rather than a forbidden code.
    fn inter_ctx_mq() -> BlockContext {
        BlockContext {
            has_intradc: false,
            has_coefficients: true,
            modified_quant: true,
        }
    }

    /// Test-side encoder for the §T.4 EXTENDED-LEVEL wire field:
    /// take the low 11 bits of LEVEL's two's-complement form and
    /// cyclically rotate them **right** by 5 (Figure T.1).
    fn extended_level_to_wire(level: i16) -> u16 {
        let v = (level as u16) & 0x07FF;
        ((v >> 5) | (v << (11 - 5))) & 0x07FF
    }

    fn finish_aligned(mut w: BitWriter) -> Vec<u8> {
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    /// Empty INTER block (no CBP bit) → no bits consumed, all zero.
    #[test]
    fn empty_inter_block_consumes_no_bits() {
        let bytes = [0xFFu8; 4];
        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_empty()).expect("parse");
        assert_eq!(r.bit_position(), 0);
        assert_eq!(blk.tcoef_event_count, 0);
        assert!(!blk.had_intradc);
        assert!(blk.coefficients.iter().all(|c| *c == 0));
    }

    /// INTRA block, CBP bit zero → only INTRADC is read (8 bits).
    #[test]
    fn intra_dc_only_block_reads_eight_bits() {
        let mut w = BitWriter::new();
        // INTRADC code 0x01 → reconstruction level 8.
        w.write_u32(0x01, 8);
        // Sentinel bits that must NOT be consumed.
        w.write_u32(0xFFFF_FFFF, 24);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, intra_ctx_dc_only()).expect("parse");
        assert_eq!(r.bit_position(), 8);
        assert!(blk.had_intradc);
        assert_eq!(blk.coefficients[0], 8);
        assert!(blk.coefficients[1..].iter().all(|c| *c == 0));
        assert_eq!(blk.tcoef_event_count, 0);
    }

    /// INTRADC `0xFF` → reconstruction level 1024 (Table 15).
    #[test]
    fn intradc_code_ff_means_1024() {
        let mut w = BitWriter::new();
        w.write_u32(0xFF, 8);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, intra_ctx_dc_only()).expect("parse");
        assert_eq!(blk.coefficients[0], 1024);
    }

    /// INTRADC `0x00` is forbidden per Table 15.
    #[test]
    fn intradc_code_zero_is_forbidden() {
        let mut w = BitWriter::new();
        w.write_u32(0x00, 8);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, intra_ctx_dc_only()).unwrap_err(),
            Error::BadIntradcCode
        );
    }

    /// INTRADC `0x80` is forbidden per Table 15.
    #[test]
    fn intradc_code_eighty_is_forbidden() {
        let mut w = BitWriter::new();
        w.write_u32(0x80, 8);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, intra_ctx_dc_only()).unwrap_err(),
            Error::BadIntradcCode
        );
    }

    /// All non-special INTRADC codes map to `code * 8`. Spot-check
    /// a few values from the spec's Table 15 illustration.
    #[test]
    fn intradc_table15_spot_check() {
        for (code, expect) in [
            (0x01u8, 8i16),
            (0x02, 16),
            (0x03, 24),
            (0x7F, 1016),
            (0x81, 1032),
            (0xFD, 2024),
            (0xFE, 2032),
        ] {
            let mut w = BitWriter::new();
            w.write_u32(code as u32, 8);
            let bytes = finish_aligned(w);
            let mut r = BitReader::new(&bytes);
            let blk = parse_block(&mut r, intra_ctx_dc_only()).expect("parse");
            assert_eq!(blk.coefficients[0], expect, "code 0x{:02x}", code);
        }
    }

    /// Simplest INTER block: a single (LAST=1, RUN=0, LEVEL=+1)
    /// event coded by VLC "0111s" with s=0.
    #[test]
    fn inter_block_single_last_event() {
        let mut w = BitWriter::new();
        // VLC for (LAST=1, RUN=0, |LEVEL|=1) is "0111" + sign 0
        w.write_u32(0b0111, 4);
        w.write_bit(false); // sign = +
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_with_coefs()).expect("parse");
        assert_eq!(blk.coefficients[0], 1);
        assert!(blk.coefficients[1..].iter().all(|c| *c == 0));
        assert_eq!(blk.tcoef_event_count, 1);
        assert!(!blk.had_intradc);
    }

    /// INTRA block: INTRADC then a single (LAST=1, RUN=0, LEVEL=-3)
    /// AC event.
    #[test]
    fn intra_block_dc_plus_one_ac() {
        let mut w = BitWriter::new();
        // INTRADC code 0x05 → recon level 40.
        w.write_u32(0x05, 8);
        // (LAST=1, RUN=0, |LEVEL|=3) → VLC "0000 0000 101s" (11-bit prefix + sign)
        w.write_u32(0b0000_0000_101, 11);
        w.write_bit(true); // sign = -
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, intra_ctx_with_coefs()).expect("parse");
        assert_eq!(blk.coefficients[0], 40);
        assert_eq!(blk.coefficients[1], -3);
        assert!(blk.coefficients[2..].iter().all(|c| *c == 0));
        assert_eq!(blk.tcoef_event_count, 1);
    }

    /// Two AC events separated by RUN.
    /// (LAST=0, RUN=2, |LEVEL|=1) is "1110s", then (LAST=1, RUN=3,
    /// |LEVEL|=1) is "0011 01s". So we expect coefs[2] = +1, coefs[6] = -1.
    #[test]
    fn inter_block_two_events_with_run_gap() {
        let mut w = BitWriter::new();
        // Event 1: 1110 + 0 (sign +)
        w.write_u32(0b1110, 4);
        w.write_bit(false);
        // Event 2: 0011 01 + 1 (sign -)
        w.write_u32(0b0011_01, 6);
        w.write_bit(true);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_with_coefs()).expect("parse");
        assert_eq!(blk.tcoef_event_count, 2);
        assert_eq!(blk.coefficients[2], 1);
        // After event 1, scan_pos is 2; we advance by +1 to 3, then
        // event 2 has RUN=3 → scan_pos becomes 6.
        assert_eq!(blk.coefficients[6], -1);
    }

    /// ESCAPE event: prefix 0000 011, LAST=1, RUN=5, LEVEL=42.
    #[test]
    fn escape_event_round_trip_positive_level() {
        let mut w = BitWriter::new();
        // Prefix "0000 011"
        w.write_u32(0b0000_011, 7);
        // LAST = 1
        w.write_bit(true);
        // RUN = 5 (6 bits)
        w.write_u32(5, 6);
        // LEVEL = 42 (8-bit two's complement)
        w.write_u32(42, 8);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_with_coefs()).expect("parse");
        assert_eq!(blk.tcoef_event_count, 1);
        assert_eq!(blk.coefficients[5], 42);
    }

    /// ESCAPE event with a negative LEVEL (two's complement).
    #[test]
    fn escape_event_negative_level_two_complement() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true); // LAST = 1
        w.write_u32(0, 6); // RUN = 0
                           // LEVEL = -1 → two's complement 0xFF
        w.write_u32(0xFF, 8);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_with_coefs()).expect("parse");
        assert_eq!(blk.coefficients[0], -1);
    }

    /// ESCAPE LEVEL `0000 0000` is forbidden in baseline.
    #[test]
    fn escape_event_level_zero_forbidden() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x00, 8);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, inter_ctx_with_coefs()).unwrap_err(),
            Error::BadTcoefEscapeLevel
        );
    }

    /// ESCAPE LEVEL `1000 0000` is forbidden in baseline (Annex T
    /// would relax this but we're baseline only).
    #[test]
    fn escape_event_level_minus_128_forbidden_in_baseline() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x80, 8);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, inter_ctx_with_coefs()).unwrap_err(),
            Error::BadTcoefEscapeLevel
        );
    }

    // ----------------------------------------------------------------
    // Annex T §T.4 — Modified coefficient range (EXTENDED-ESCAPE /
    // EXTENDED-LEVEL).
    // ----------------------------------------------------------------

    /// Pure-function round-trip: the §T.4 / Figure T.1 wire transform
    /// is its own inverse (rotate-right-5 followed by rotate-left-5),
    /// across the entire 11-bit two's-complement LEVEL range.
    #[test]
    fn extended_level_round_trip_full_range() {
        for level in -1024i16..=1023 {
            let wire = extended_level_to_wire(level);
            assert!(wire <= 0x07FF);
            assert_eq!(
                extended_level_from_wire(wire),
                level,
                "round-trip failed for LEVEL = {level}"
            );
        }
    }

    /// §T.4 worked-shape check: the 5-bit cyclic rotation actually
    /// rearranges the bits per Figure T.1 (it is not the identity).
    /// LEVEL = +1 has two's-complement low-11 bits `000 0000 0001`
    /// (b1 = 1, all others 0); cyclically rotating right by 5 moves
    /// b1 into wire position b1's slot (the 6th from the MSB), giving
    /// wire `000 0100 0000` = 0x40. The decoder's rotate-left-5
    /// inverts it back to +1.
    #[test]
    fn extended_level_rotation_matches_figure_t1() {
        assert_eq!(extended_level_to_wire(1), 0x040);
        assert_eq!(extended_level_from_wire(0x040), 1);
        // LEVEL = -1 → low-11 bits all ones → rotation leaves all ones.
        assert_eq!(extended_level_to_wire(-1), 0x7FF);
        assert_eq!(extended_level_from_wire(0x7FF), -1);
    }

    /// §T.4 end-to-end: with Modified Quantization mode active an
    /// ESCAPE followed by the `1000 0000` EXTENDED-ESCAPE marker and
    /// an 11-bit EXTENDED-LEVEL decodes a magnitude > 127 (here +200).
    #[test]
    fn extended_escape_positive_level_decodes() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7); // ESCAPE prefix
        w.write_bit(true); // LAST = 1
        w.write_u32(0, 6); // RUN = 0
        w.write_u32(0x80, 8); // EXTENDED-ESCAPE marker
        w.write_u32(extended_level_to_wire(200) as u32, 11);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_mq()).expect("parse");
        assert_eq!(blk.coefficients[0], 200);
        assert_eq!(blk.tcoef_event_count, 1);
    }

    /// §T.4 end-to-end with a negative extended LEVEL (-300).
    #[test]
    fn extended_escape_negative_level_decodes() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(3, 6); // RUN = 3
        w.write_u32(0x80, 8); // EXTENDED-ESCAPE marker
        w.write_u32(extended_level_to_wire(-300) as u32, 11);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_mq()).expect("parse");
        // RUN=3 from scan_pos 0 → coefficient lands at slot 3.
        assert_eq!(blk.coefficients[3], -300);
    }

    /// §T.4 extreme: the maximum representable inverse-quantizable
    /// magnitude noted in §T.4 (true coefficient values up to 2040).
    /// LEVEL = +2040 fits the 11-bit signed range (−1024..=1023)? No —
    /// it does not, so the largest in-range positive LEVEL is +1023.
    #[test]
    fn extended_escape_max_in_range_level() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x80, 8);
        w.write_u32(extended_level_to_wire(1023) as u32, 11);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_mq()).expect("parse");
        assert_eq!(blk.coefficients[0], 1023);
    }

    /// §T.4: without Modified Quantization mode the `1000 0000`
    /// ESCAPE LEVEL stays the baseline forbidden code (no 11-bit
    /// field is consumed).
    #[test]
    fn extended_escape_marker_forbidden_without_mq() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x80, 8);
        w.write_u32(0, 11); // bytes that would be EXTENDED-LEVEL
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, inter_ctx_with_coefs()).unwrap_err(),
            Error::BadTcoefEscapeLevel
        );
    }

    /// §T.4: LEVEL `0000 0000` remains forbidden even under Modified
    /// Quantization mode (only `1000 0000` is re-purposed).
    #[test]
    fn extended_escape_level_zero_still_forbidden_under_mq() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x00, 8);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, inter_ctx_mq()).unwrap_err(),
            Error::BadTcoefEscapeLevel
        );
    }

    /// §T.4: a normal (non-marker) ESCAPE LEVEL still decodes as the
    /// baseline 8-bit two's-complement value when Modified
    /// Quantization mode is active (the in-range −127..=127 path).
    #[test]
    fn ordinary_escape_level_unaffected_under_mq() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x7F, 8); // LEVEL = +127
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let blk = parse_block(&mut r, inter_ctx_mq()).expect("parse");
        assert_eq!(blk.coefficients[0], 127);
    }

    /// §T.4: a truncated EXTENDED-LEVEL field (marker present but the
    /// 11 bits run off the end of the buffer) is an EOF.
    #[test]
    fn extended_escape_truncated_extended_level_eof() {
        let mut w = BitWriter::new();
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0x80, 8); // marker, but no EXTENDED-LEVEL follows
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, inter_ctx_mq()).unwrap_err(),
            Error::UnexpectedEof
        );
    }

    /// A RUN that would overflow the 64-slot scan must be rejected.
    /// Encode RUN=63 with LAST=1 starting at INTRA scan_pos=1 →
    /// scan_pos jumps to 64, which is out of range.
    #[test]
    fn tcoef_run_overflow_rejected() {
        let mut w = BitWriter::new();
        // Some valid INTRADC.
        w.write_u32(0x01, 8);
        // ESCAPE event: LAST=1, RUN=63, LEVEL=1.
        w.write_u32(0b0000_011, 7);
        w.write_bit(true);
        w.write_u32(63, 6);
        w.write_u32(1, 8);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, intra_ctx_with_coefs()).unwrap_err(),
            Error::BadTcoefRunOverflow
        );
    }

    /// Walk every row of Table 16's regular (non-ESCAPE) entries and
    /// verify each round-trips through the parser at the expected
    /// scan position with both sign polarities.
    #[test]
    fn tcoef_table_full_102_entry_round_trip() {
        for row in TCOEF_TABLE.iter() {
            if row.is_escape {
                continue;
            }
            for sign in [false, true] {
                // Encode the row as a single-event block. Row's `bits`
                // includes the trailing sign; the *prefix* we write is
                // `bits - 1`, then we add the sign bit separately.
                let mut w = BitWriter::new();
                w.write_u32(row.code as u32, (row.bits - 1) as u32);
                w.write_bit(sign);
                // If LAST = 0 we must follow with a terminating LAST=1
                // event so parse_block exits. Use "0111s" (LAST=1, RUN=0,
                // LEVEL=1, sign=0) since RUN=0 keeps scan_pos in range
                // for the small RUN values in the table.
                if !row.last {
                    w.write_u32(0b0111, 4);
                    w.write_bit(false);
                }
                let bytes = finish_aligned(w);

                let mut r = BitReader::new(&bytes);
                // Pick an INTER context so scan_pos starts at 0; the
                // RUN values in Table 16 max out at 40 which is safely
                // in range without an INTRADC offset.
                let blk = parse_block(&mut r, inter_ctx_with_coefs()).expect("parse");
                let expected_level = if sign {
                    -(row.abs_level as i16)
                } else {
                    row.abs_level as i16
                };
                let scan_pos = row.run as usize;
                assert_eq!(
                    blk.coefficients[scan_pos], expected_level,
                    "row bits={} code={:b} last={} run={} |L|={} sign={}",
                    row.bits, row.code, row.last, row.run, row.abs_level, sign
                );
            }
        }
    }

    /// Zigzag table sanity-checks. Slot 0 is block (0,0); slot 63
    /// is block (7,7); the last slot of each anti-diagonal pair sums
    /// to a fixed total.
    #[test]
    fn zigzag_table_endpoints_correct() {
        assert_eq!(ZIGZAG_TO_BLOCK_POS[0], 0); // DC at block (0,0)
        assert_eq!(ZIGZAG_TO_BLOCK_POS[63], 63); // last at (7,7)
        assert_eq!(ZIGZAG_TO_BLOCK_POS[1], 1); // scan 2 at block (0,1)
        assert_eq!(ZIGZAG_TO_BLOCK_POS[2], 8); // scan 3 at block (1,0)
    }

    /// Zigzag table is a permutation of 0..=63: every block
    /// position appears exactly once.
    #[test]
    fn zigzag_table_is_a_permutation() {
        let mut seen = [false; COEFFS_PER_BLOCK];
        for &p in ZIGZAG_TO_BLOCK_POS.iter() {
            let p = p as usize;
            assert!(p < COEFFS_PER_BLOCK, "out of range {}", p);
            assert!(!seen[p], "block pos {} appears twice", p);
            seen[p] = true;
        }
        assert!(seen.iter().all(|s| *s), "not surjective");
    }

    /// Truncated INTRADC → UnexpectedEof.
    #[test]
    fn truncated_intradc_eof() {
        let bytes = [0xFFu8; 1];
        let mut r = BitReader::new(&bytes);
        // Read 7 bits manually to leave only 1 bit for INTRADC.
        r.read_u32(7).unwrap();
        assert_eq!(
            parse_block(&mut r, intra_ctx_dc_only()).unwrap_err(),
            Error::UnexpectedEof
        );
    }

    /// Table 16 has exactly 102 regular VLC code-points + 1 ESCAPE
    /// prefix, totalling 103 rows.
    #[test]
    fn tcoef_table_has_102_regular_entries_plus_escape() {
        let regular = TCOEF_TABLE.iter().filter(|r| !r.is_escape).count();
        let escape = TCOEF_TABLE.iter().filter(|r| r.is_escape).count();
        assert_eq!(regular, 102);
        assert_eq!(escape, 1);
    }

    /// Compose picture / GOB / MB / block parsers on a single
    /// `BitReader`: parse a QCIF I-frame header, GOB header, one
    /// INTRA MB, then one INTRA block with INTRADC = 16 and no AC.
    #[test]
    fn composes_picture_gob_mb_then_one_dc_only_block() {
        use crate::{
            gob_header::{GBSC_BITS, GBSC_VALUE, GFID_BITS, GN_BITS, GQUANT_BITS},
            parse_gob_layer, parse_macroblock, parse_picture_header, MbContext, PSC_BITS,
            PSC_VALUE,
        };

        let mut w = BitWriter::new();
        // Picture header — QCIF INTRA.
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8); // TR
        w.write_bit(true); // PTYPE bit 1 = 1
        w.write_bit(false); // bit 2 = 0
        w.write_bit(false); // split-screen
        w.write_bit(false); // doc-camera
        w.write_bit(false); // freeze
        w.write_u32(0b010, 3); // QCIF
        w.write_bit(false); // INTRA
        w.write_bit(false); // Annex D
        w.write_bit(false); // Annex E
        w.write_bit(false); // Annex F
        w.write_bit(false); // Annex G
                            // GOB header (QUANT=8).
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(1, GN_BITS);
        w.write_u32(0, GFID_BITS);
        w.write_u32(8, GQUANT_BITS);
        // MB header (INTRA, no DQUANT): MCBPC "1", CBPY "0011" (pattern 0000).
        w.write_bit(true);
        w.write_u32(0b0011, 4);
        // Block: INTRADC = 0x02 → reconstruction level 16.
        w.write_u32(0x02, 8);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let pic = parse_picture_header(&mut r).expect("pic");
        let gob = parse_gob_layer(&mut r).expect("gob");
        let mb = parse_macroblock(
            &mut r,
            MbContext {
                picture_coding_type: pic.coding_type,
                advanced_prediction: pic.advanced_prediction,
                aic_intra_mode: false,
                pb_frames: false,
                pb_annex_m: false,
                quantiser_before: gob.quantiser,
                modified_quant: false,
            },
        )
        .expect("mb");
        // CBPY pattern 0000 → no AC TCOEFs for this luma block;
        // INTRA so INTRADC IS on the wire.
        let block = parse_block(
            &mut r,
            BlockContext {
                has_intradc: mb.mb_type.unwrap().is_intra(),
                has_coefficients: false, // CBPY bit 3 = 0 for block 1
                ..Default::default()
            },
        )
        .expect("block");
        assert_eq!(block.coefficients[0], 16);
        assert_eq!(block.tcoef_event_count, 0);
        assert!(block.had_intradc);
    }

    /// A TCOEF VLC prefix that doesn't match any row → BadTcoefCode.
    /// We use 13 bits of all-zeros which never matches.
    #[test]
    fn invalid_tcoef_prefix_rejected() {
        let mut w = BitWriter::new();
        // 13 zero bits — no Table 16 row matches.
        w.write_u32(0, 13);
        // Plus some trailing bits so EOF doesn't kick in first.
        w.write_u32(0xFFFF, 16);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_block(&mut r, inter_ctx_with_coefs()).unwrap_err(),
            Error::BadTcoefCode
        );
    }
}
