//! H.263 PB-frame B-block layer field parsers (§5.3.3 / §5.3.4).
//!
//! The PB-frame mode (Annex G; signalled by PTYPE bit 13 in §5.1.3
//! or by the PLUSPTYPE picture-type code in §5.1.4.3) inserts two
//! additional macroblock-layer fields between MCBPC and CBPY:
//!
//! ```text
//!   COD   MCBPC   MODB   CBPB   CBPY   DQUANT   MVD   MVD2-4   MVDB   Block Data
//!                  ^^^^   ^^^^                                  ^^^^
//!                  §5.3.3 §5.3.4                                §5.3.9
//! ```
//!
//! This module provides the two pure-parser primitives the future
//! macroblock-layer driver needs to consume them:
//!
//! * [`parse_modb`] — §5.3.3 / Table 11 variable-length codeword that
//!   tells the decoder whether the B-block coefficient pattern (CBPB)
//!   and/or the B-block motion-vector difference (MVDB) are on the
//!   wire for the current macroblock. Returns a [`ModbPresence`] tag
//!   covering the three legal codewords; an unrecognised prefix is a
//!   bitstream error.
//! * [`parse_cbpb`] — §5.3.4 6-bit fixed-length field; bit `5` (MSB)
//!   carries B-block number 1, bit `0` (LSB) carries B-block number
//!   6. Returns the raw 6-bit pattern in the low bits of a `u8`.
//!   [`cbpb_block_present`] is the per-block accessor that consults
//!   the pattern with the Figure-5 / §5.3.4 "utmost left bit ↔ block
//!   number 1" convention.
//!
//! ## Composition with the macroblock driver
//!
//! These are pure-parser primitives. They do not consult any
//! enclosing-picture context (PTYPE / MCBPC MB-type / Annex M vs
//! Annex G mode-selector). The macroblock driver is responsible for
//! gating their invocation per §5.3.3 ("MODB is present for MB-type
//! 0-4 if PTYPE indicates 'PB-frame'") and §5.3.4 ("CBPB is only
//! present in PB-frames mode if indicated by MODB").
//!
//! The Annex M (Improved PB-frames) MODB table is **different** from
//! Table 11 — Annex M defines a 7-entry table (Table M.1) where this
//! module's parser covers only the 3-entry Annex G form. Annex-M
//! support is a separate primitive that the future Annex-M driver
//! will add.
//!
//! Per §5.3.9, MVDB is "a variable length codeword for the horizontal
//! component followed by a variable length codeword for the vertical
//! component of each vector. Variable length codes are given in
//! Table 14." Table 14 is the same MVD VLC the baseline §5.3.7 parser
//! already decodes via [`crate::macroblock::H263Macroblock::mvd`]'s
//! component decoder; no new VLC table lands for MVDB itself. The
//! macroblock driver wires the existing `decode_mvd_component` into
//! MVDB when MODB indicates MVDB presence.

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Field width of the §5.3.4 CBPB Coded Block Pattern for B-blocks.
/// Six bits, one per B-block in the macroblock (four luma + two
/// chroma per §5.3.3 / Figure 5).
pub const CBPB_BITS: u32 = 6;

/// Per-B-block presence signalled by a §5.3.3 / Table 11 MODB
/// codeword. Names follow the §5.3.3 column headers ("CBPB", "MVDB")
/// with the "X" cells in Table 11 collapsed onto the tag variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModbPresence {
    /// Table 11 row 0 — neither CBPB nor MVDB on the wire for this
    /// macroblock's B-blocks. Code `0`, 1 bit.
    None,
    /// Table 11 row 1 — MVDB on the wire, CBPB absent. Code `10`,
    /// 2 bits.
    MvdbOnly,
    /// Table 11 row 2 — both CBPB and MVDB on the wire. Code `11`,
    /// 2 bits.
    CbpbAndMvdb,
}

impl ModbPresence {
    /// `true` iff Table 11 marks CBPB as present ("X" in the CBPB
    /// column). Only [`ModbPresence::CbpbAndMvdb`] does.
    pub fn has_cbpb(self) -> bool {
        matches!(self, ModbPresence::CbpbAndMvdb)
    }

    /// `true` iff Table 11 marks MVDB as present ("X" in the MVDB
    /// column). [`ModbPresence::MvdbOnly`] and
    /// [`ModbPresence::CbpbAndMvdb`].
    pub fn has_mvdb(self) -> bool {
        matches!(self, ModbPresence::MvdbOnly | ModbPresence::CbpbAndMvdb)
    }

    /// Length in bits of the §5.3.3 / Table 11 codeword that produced
    /// this tag. Useful for tests and for any caller that needs the
    /// post-parse bit cursor without re-running the bitreader.
    pub fn code_bits(self) -> u32 {
        match self {
            ModbPresence::None => 1,
            ModbPresence::MvdbOnly | ModbPresence::CbpbAndMvdb => 2,
        }
    }
}

/// Decode a §5.3.3 / Table 11 MODB variable-length codeword.
///
/// The reader is left positioned at the first bit following the
/// MODB code on success. On `Err(Error::UnexpectedEof)` the reader's
/// position is unspecified (1-bit MODB always succeeds on a non-empty
/// reader; only the 2-bit branch can run off the end mid-code).
///
/// Table 11 layout:
///
/// | Index | CBPB | MVDB | Code |
/// |-------|------|------|------|
/// | 0     |      |      | `0`  |
/// | 1     |      | X    | `10` |
/// | 2     | X    | X    | `11` |
///
/// The leading bit `0` immediately resolves [`ModbPresence::None`];
/// the leading bit `1` requires one more bit to disambiguate
/// [`ModbPresence::MvdbOnly`] (`10`) from [`ModbPresence::CbpbAndMvdb`]
/// (`11`). There is no unknown / forbidden prefix shape — every
/// 1- or 2-bit prefix starting from the reader is a legal Table 11
/// codeword. The only error this function returns is
/// [`Error::UnexpectedEof`] when the stream ends after the leading
/// `1` without a second bit.
pub fn parse_modb(reader: &mut BitReader<'_>) -> Result<ModbPresence> {
    let lead = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if !lead {
        return Ok(ModbPresence::None);
    }
    let second = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if second {
        Ok(ModbPresence::CbpbAndMvdb)
    } else {
        Ok(ModbPresence::MvdbOnly)
    }
}

/// Decode a §5.3.4 CBPB 6-bit fixed-length codeword.
///
/// Returns the six bits as a `u8` in the low six bits, with bit `5`
/// (the MSB of the field) carrying B-block number 1's CBPBN per the
/// §5.3.4 "the utmost left bit of CBPB corresponding with block
/// number 1" convention. Use [`cbpb_block_present`] to query an
/// individual block's CBPBN bit by 1-based block number without
/// re-deriving the mapping.
///
/// On `Err(Error::UnexpectedEof)` the reader's position is
/// unspecified (the field is fixed-length so any failure indicates
/// fewer than 6 bits remained).
pub fn parse_cbpb(reader: &mut BitReader<'_>) -> Result<u8> {
    let raw = reader
        .read_u32(CBPB_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    Ok(raw as u8)
}

/// `true` iff the §5.3.4 CBPB pattern marks B-block `block_number`
/// (1-based per Figure 5: 1..=4 luma in Y raster order, 5 = Cb,
/// 6 = Cr) as carrying at least one non-zero coefficient.
///
/// Returns `false` for any `block_number` outside `1..=6` (defensive;
/// the spec's block-numbering domain is exactly six).
pub fn cbpb_block_present(cbpb: u8, block_number: u32) -> bool {
    if !(1..=6).contains(&block_number) {
        return false;
    }
    // Block 1 → bit 5 (MSB of the 6-bit field), block 2 → bit 4, ...,
    // block 6 → bit 0.
    let bit_index = 6 - block_number;
    ((cbpb >> bit_index) & 1) != 0
}

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

    /// Table 11 row 0 (code `0`, 1 bit) decodes to
    /// [`ModbPresence::None`].
    #[test]
    fn modb_code_0_is_none() {
        let mut w = BitWriter::new();
        w.write_bit(false);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("parse");
        assert_eq!(modb, ModbPresence::None);
        assert!(!modb.has_cbpb());
        assert!(!modb.has_mvdb());
        assert_eq!(modb.code_bits(), 1);
        // Reader sits at bit position 1 (the bit after the 1-bit
        // codeword).
        assert_eq!(r.bit_position(), 1);
    }

    /// Table 11 row 1 (code `10`, 2 bits) decodes to
    /// [`ModbPresence::MvdbOnly`].
    #[test]
    fn modb_code_10_is_mvdb_only() {
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("parse");
        assert_eq!(modb, ModbPresence::MvdbOnly);
        assert!(!modb.has_cbpb());
        assert!(modb.has_mvdb());
        assert_eq!(modb.code_bits(), 2);
        assert_eq!(r.bit_position(), 2);
    }

    /// Table 11 row 2 (code `11`, 2 bits) decodes to
    /// [`ModbPresence::CbpbAndMvdb`].
    #[test]
    fn modb_code_11_is_cbpb_and_mvdb() {
        let mut w = BitWriter::new();
        w.write_u32(0b11, 2);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("parse");
        assert_eq!(modb, ModbPresence::CbpbAndMvdb);
        assert!(modb.has_cbpb());
        assert!(modb.has_mvdb());
        assert_eq!(modb.code_bits(), 2);
        assert_eq!(r.bit_position(), 2);
    }

    /// MODB starting with a `1` followed by EOF must yield
    /// [`Error::UnexpectedEof`]. We construct a 1-byte buffer whose
    /// last bit is the leading `1` of MODB and burn the seven
    /// preceding bits, so the leading `1` is read but no more bits
    /// remain in the buffer.
    #[test]
    fn modb_truncated_after_lead_one_returns_eof() {
        let data = [0b0000_0001u8];
        let mut r = BitReader::new(&data);
        r.read_u32(7).expect("seven padding bits");
        let err = parse_modb(&mut r).expect_err("second bit missing");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// MODB on an empty buffer yields [`Error::UnexpectedEof`]
    /// immediately (no bits at all to read).
    #[test]
    fn modb_empty_buffer_returns_eof() {
        let data: [u8; 0] = [];
        let mut r = BitReader::new(&data);
        let err = parse_modb(&mut r).expect_err("empty");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// CBPB all-zero pattern decodes to `0` and leaves the reader
    /// 6 bits in.
    #[test]
    fn cbpb_all_zero_pattern() {
        let mut w = BitWriter::new();
        w.write_u32(0b00_0000, 6);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let cbpb = parse_cbpb(&mut r).expect("parse");
        assert_eq!(cbpb, 0);
        assert_eq!(r.bit_position(), 6);
        // No B-block has a non-zero coefficient.
        for n in 1..=6 {
            assert!(!cbpb_block_present(cbpb, n));
        }
    }

    /// CBPB all-ones pattern decodes to `0x3F` and marks every
    /// B-block as carrying a non-zero coefficient.
    #[test]
    fn cbpb_all_one_pattern() {
        let mut w = BitWriter::new();
        w.write_u32(0b11_1111, 6);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let cbpb = parse_cbpb(&mut r).expect("parse");
        assert_eq!(cbpb, 0b11_1111);
        for n in 1..=6 {
            assert!(cbpb_block_present(cbpb, n));
        }
    }

    /// CBPB single-bit-set patterns isolate each B-block in turn:
    /// the §5.3.4 "utmost left bit ↔ block 1" convention means
    /// block 1 corresponds to bit 5, block 2 to bit 4, …, block 6
    /// to bit 0.
    #[test]
    fn cbpb_single_bit_per_block_isolates_correct_block() {
        for target_block in 1..=6u32 {
            // bit_index = 6 - block_number per §5.3.4.
            let bit_index = 6 - target_block;
            let pattern = 1u32 << bit_index;
            let mut w = BitWriter::new();
            w.write_u32(pattern, 6);
            let data = finish_aligned(w);
            let mut r = BitReader::new(&data);
            let cbpb = parse_cbpb(&mut r).expect("parse");
            assert_eq!(
                cbpb, pattern as u8,
                "block {} pattern round-trip",
                target_block
            );
            for query_block in 1..=6 {
                let expected = query_block == target_block;
                assert_eq!(
                    cbpb_block_present(cbpb, query_block),
                    expected,
                    "block {} present query for target block {} ",
                    query_block,
                    target_block
                );
            }
        }
    }

    /// CBPB block-1 vs block-6 specifically: the §5.3.4 "utmost left
    /// bit corresponds with block number 1" convention pins block 1
    /// to bit 5 (`0b10_0000`) and block 6 to bit 0 (`0b00_0001`).
    /// This test is a deliberate static check of those two endpoints
    /// so a future refactor of [`cbpb_block_present`] cannot silently
    /// invert the mapping.
    #[test]
    fn cbpb_block_1_is_msb_block_6_is_lsb() {
        assert!(cbpb_block_present(0b10_0000, 1));
        assert!(!cbpb_block_present(0b10_0000, 2));
        assert!(!cbpb_block_present(0b00_0001, 1));
        assert!(cbpb_block_present(0b00_0001, 6));
    }

    /// CBPB on a truncated buffer (fewer than 6 bits remaining)
    /// returns [`Error::UnexpectedEof`].
    #[test]
    fn cbpb_truncated_returns_eof() {
        // Two-byte buffer with cursor 11 bits in leaves only 5 bits
        // before EOF — too few for CBPB.
        let data = [0u8; 2];
        let mut r = BitReader::new(&data);
        r.read_u32(11).expect("burn 11 bits");
        let err = parse_cbpb(&mut r).expect_err("five bits left");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// CBPB on an empty buffer returns [`Error::UnexpectedEof`].
    #[test]
    fn cbpb_empty_buffer_returns_eof() {
        let data: [u8; 0] = [];
        let mut r = BitReader::new(&data);
        let err = parse_cbpb(&mut r).expect_err("empty");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// Out-of-range block numbers return `false` (defensive).
    #[test]
    fn cbpb_block_present_out_of_range_is_false() {
        for n in [0u32, 7, 8, 100, u32::MAX] {
            assert!(!cbpb_block_present(0b11_1111, n));
        }
    }

    /// `code_bits()` agrees with the bit advance observed by the
    /// reader for all three Table 11 entries (sanity check that the
    /// reader's bit_position and the tag-reported width stay
    /// consistent).
    #[test]
    fn modb_code_bits_matches_reader_advance() {
        for (raw_code, code_bits, expected) in [
            (0b0u32, 1u32, ModbPresence::None),
            (0b10, 2, ModbPresence::MvdbOnly),
            (0b11, 2, ModbPresence::CbpbAndMvdb),
        ] {
            let mut w = BitWriter::new();
            w.write_u32(raw_code, code_bits);
            let data = finish_aligned(w);
            let mut r = BitReader::new(&data);
            let modb = parse_modb(&mut r).expect("parse");
            assert_eq!(modb, expected);
            assert_eq!(modb.code_bits(), code_bits);
            assert_eq!(r.bit_position() as u32, code_bits);
        }
    }

    /// End-to-end: MODB row 2 (`11`) immediately followed by a CBPB
    /// pattern of `0b10_1010` (blocks 1, 3, 5 carry coefficients)
    /// chained into one reader. After both parses, the reader sits
    /// at bit 8 (2 + 6) — the first bit of whatever follows CBPB
    /// in the macroblock layer (CBPY for the eventual PB-mode
    /// driver).
    #[test]
    fn modb_cbpb_chain_advances_reader_by_8_bits() {
        let mut w = BitWriter::new();
        w.write_u32(0b11, 2); // MODB row 2
        w.write_u32(0b10_1010, 6); // CBPB pattern: blocks 1, 3, 5 set
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("modb");
        assert_eq!(modb, ModbPresence::CbpbAndMvdb);
        assert!(modb.has_cbpb());
        let cbpb = parse_cbpb(&mut r).expect("cbpb");
        assert_eq!(cbpb, 0b10_1010);
        assert!(cbpb_block_present(cbpb, 1));
        assert!(!cbpb_block_present(cbpb, 2));
        assert!(cbpb_block_present(cbpb, 3));
        assert!(!cbpb_block_present(cbpb, 4));
        assert!(cbpb_block_present(cbpb, 5));
        assert!(!cbpb_block_present(cbpb, 6));
        assert_eq!(r.bit_position(), 8);
    }
}
