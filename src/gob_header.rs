//! H.263 Group-of-Blocks (GOB) layer header parsing.
//!
//! This module implements the structural decode of the GOB-layer
//! header as defined in ITU-T Recommendation H.263 (01/2005) §5.2.
//! Per Figure 9/H.263 the on-wire structure is:
//!
//! ```text
//!   GSTUF | GBSC | GN | GSBI | GFID | GQUANT | Macroblock Data
//! ```
//!
//! Round 2 (post orphan-rebuild) implements the header up through
//! GQUANT and stops before macroblock data. Within the header we
//! cover:
//!
//! * §5.2.2 — **GBSC** (Group of Blocks Start Code), 17 bits, value
//!   `0000 0000 0000 0000 1`.
//! * §5.2.3 — **GN** (Group Number), 5 bits fixed-length.
//! * §5.2.5 — **GFID** (GOB Frame ID), 2 bits fixed-length. The spec
//!   says GFID "is present when GBSC is present" — i.e. always — so
//!   we must consume those 2 bits to reach GQUANT correctly. The
//!   value is exposed for round-3+ inter-picture continuity checks.
//! * §5.2.6 — **GQUANT** (Quantizer Information), 5 bits, natural
//!   binary representation of QUANT in the range 1..=31.
//!
//! ## Deliberately deferred
//!
//! * §5.2.1 — **GSTUF**: a variable-length run of zero-bits the
//!   encoder may insert directly before GBSC for byte alignment. The
//!   caller is responsible for skipping any leading GSTUF before
//!   invoking this parser; round-2's contract is "the first bit the
//!   reader returns is the MSB of GBSC".
//! * §5.2.4 — **GSBI** (GOB Sub-Bitstream Indicator): present only
//!   when CPM = "1" in the picture-layer header. Round-1's
//!   [`crate::H263PictureHeader`] does not yet expose CPM (that lives
//!   in the Annex-O optional-field block which is not yet decoded),
//!   so we have no way to know whether GSBI is on the wire. Until
//!   CPM lands, this parser only handles the **CPM = "0"** case (no
//!   GSBI on the wire) and would mis-frame a CPM = "1" bitstream.
//!   Callers that need CPM = "1" support should treat this parser as
//!   off-limits for now.
//! * Macroblock data (§5.3) and everything beyond GQUANT.
//!
//! ## GN value space
//!
//! §5.2.3 defines several GN value ranges for different roles:
//!
//! | GN value | Role |
//! |----------|------|
//! | 0        | Not present in GOB headers (group 0 is signalled by PSC). |
//! | 1..=17   | GOB headers, standard picture formats. |
//! | 1..=24   | GOB headers, custom picture formats. |
//! | 16..=28  | Emulated in slice header (Annex K), CPM = "0". |
//! | 25..=27, 29 | Emulated in slice header (Annex K), CPM = "1". |
//! | 30       | EOSBS marker, not a GOB header. |
//! | 31       | EOS marker, not a GOB header. |
//!
//! Without knowing the picture's source format we accept any GN in
//! `1..=29` (the union of the GOB and slice-header ranges) and
//! reject `0`, `30`, `31` as not-a-GOB-header. The narrower
//! source-format-aware check lives in the eventual picture-level
//! demuxer.

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Length in bits of the Group-of-Blocks Start Code (§5.2.2).
pub const GBSC_BITS: u32 = 17;

/// Value of the Group-of-Blocks Start Code: `0000 0000 0000 0000 1`.
///
/// As a 17-bit unsigned integer that is `0x00001` (decimal 1). GOB
/// start codes *may* be byte aligned; the caller is responsible for
/// having skipped any GSTUF stuffing so that the next bit returned by
/// the reader is the MSB of GBSC.
pub const GBSC_VALUE: u32 = 0x0000_0001;

/// Length in bits of the Group Number field (§5.2.3).
pub const GN_BITS: u32 = 5;

/// Length in bits of the GOB Frame ID field (§5.2.5).
pub const GFID_BITS: u32 = 2;

/// Length in bits of the Quantizer Information field (§5.2.6).
pub const GQUANT_BITS: u32 = 5;

/// Total length in bits of the round-2 GOB-layer header
/// (GBSC + GN + GFID + GQUANT, no GSBI).
pub const GOB_HEADER_BITS_NO_CPM: u32 = GBSC_BITS + GN_BITS + GFID_BITS + GQUANT_BITS;

/// Parsed H.263 GOB-layer header (CPM = "0" branch only).
///
/// The header binds the macroblock rows that follow until the next
/// GBSC or PSC. See module-level docs for what each field means and
/// what is deferred.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GobLayer {
    /// §5.2.3 — Group Number. Valid range here is `1..=29`; see the
    /// module-level table for how that interacts with picture source
    /// format and CPM.
    pub number: u8,
    /// §5.2.6 — Quantizer step parameter QUANT, range `1..=31`.
    pub quantiser: u8,
    /// §5.2.5 — GOB Frame ID, 2-bit value.
    ///
    /// Exposed for future continuity checks against the picture's
    /// previous GFID; round-2 does not enforce that the value is
    /// stable across GOBs of a single picture.
    pub gfid: u8,
    /// Total number of bits consumed for this header (GBSC + GN +
    /// GFID + GQUANT). Fixed at [`GOB_HEADER_BITS_NO_CPM`] for the
    /// round-2 CPM-off case; exposed so that a caller composing
    /// multiple layers can advance a byte/bit cursor without
    /// re-deriving the layout.
    pub header_bits: u32,
}

/// Parse an H.263 GOB-layer header starting at the current position
/// of `reader`.
///
/// The caller is responsible for placing the reader at the first bit
/// of GBSC — i.e. for skipping any leading GSTUF stuffing. On success
/// the reader is left positioned immediately after GQUANT, at the
/// first bit of the GOB's macroblock data.
///
/// On error the reader's position is unspecified.
///
/// ### Errors
///
/// * [`Error::UnexpectedEof`] — the buffer ended before the 29-bit
///   header could be read in full.
/// * [`Error::BadGroupStartCode`] — the leading 17 bits were not
///   equal to [`GBSC_VALUE`].
/// * [`Error::InvalidGroupNumber`] — GN was `0`, `30`, or `31`
///   (none of which is a GOB-layer group number; `0` is reserved
///   to PSC, `30` is EOSBS, `31` is EOS).
/// * [`Error::InvalidQuantiser`] — GQUANT was `0`; §5.2.6 limits
///   QUANT to the natural-binary range `1..=31`.
pub fn parse_gob_layer(reader: &mut BitReader<'_>) -> Result<GobLayer> {
    // §5.2.2 — GBSC (17 bits, value 0x00001).
    let gbsc = reader
        .read_u32(GBSC_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    if gbsc != GBSC_VALUE {
        return Err(Error::BadGroupStartCode);
    }

    // §5.2.3 — GN (5 bits, fixed-length).
    let gn = reader.read_u32(GN_BITS).map_err(|_| Error::UnexpectedEof)? as u8;
    // `read_u32(5)` cannot exceed 31 so the upper-bound check is real.
    if gn == 0 || gn == 30 || gn == 31 {
        return Err(Error::InvalidGroupNumber);
    }

    // §5.2.5 — GFID (2 bits). Always present when GBSC is present.
    // GSBI is *not* read here: round-2 only handles the CPM = "0"
    // case; see module-level docs.
    let gfid = reader
        .read_u32(GFID_BITS)
        .map_err(|_| Error::UnexpectedEof)? as u8;

    // §5.2.6 — GQUANT (5 bits, QUANT range 1..=31).
    let gquant = reader
        .read_u32(GQUANT_BITS)
        .map_err(|_| Error::UnexpectedEof)? as u8;
    if gquant == 0 {
        return Err(Error::InvalidQuantiser);
    }

    Ok(GobLayer {
        number: gn,
        quantiser: gquant,
        gfid,
        header_bits: GOB_HEADER_BITS_NO_CPM,
    })
}

/// Convenience wrapper that constructs a [`BitReader`] over `data`
/// and forwards to [`parse_gob_layer`]. The reader's final position
/// is discarded; use [`parse_gob_layer`] directly when composing
/// multiple layer parses.
pub fn parse_gob_layer_from_bytes(data: &[u8]) -> Result<GobLayer> {
    let mut reader = BitReader::new(data);
    parse_gob_layer(&mut reader)
}

/// Detect whether a GOB-layer header (a GBSC start code, optionally
/// preceded by §5.2.1 GSTUF stuffing) begins at the reader's current
/// position, **consuming the GSTUF bits when a header is found** so a
/// subsequent [`parse_gob_layer`] starts on the GBSC's first bit.
///
/// §5.2 says that for every GOB except number 0 "the GOB header may be
/// empty, depending on the encoder strategy" — an encoder is free to
/// omit the GBSC/GN/GFID/GQUANT entirely and let macroblock data run
/// continuously across the GOB boundary. A decoder that walks the GOB
/// grid therefore cannot assume a header precedes each GOB's first
/// macroblock row; it must look for the start code and fall through to
/// the macroblock layer when none is present. the reference encoder's native H.263
/// encoder, for one, emits no non-empty GOB headers for the standard
/// formats, so every fixture below GOB 0 reaches this path.
///
/// ## Detection rule (§5.2.1 / §5.2.2)
///
/// GBSC is the 17-bit word `0000 0000 0000 0000 1`; its 16 leading
/// zeros cannot occur inside legal macroblock data (start-code
/// emulation is prevented by the bitstream construction), so a run of
/// 16 zero bits unambiguously marks a start code rather than coefficient
/// data. §5.2.1 lets the encoder insert GSTUF — "less than 8 zero-bits"
/// whose last bit is the LSB of a byte — directly before GBSC so that
/// "the start of the GBSC codeword is byte aligned". §5.2.2 phrases the
/// same alignment as optional ("GOB start codes *may* be byte aligned"),
/// so this detector accepts a GBSC that begins:
///
/// 1. immediately at the current bit position (no GSTUF), or
/// 2. at the next byte boundary, provided every skipped bit up to that
///    boundary is zero (a valid GSTUF run of 1..=7 bits).
///
/// On a positive detection the GSTUF bits (case 2) are consumed and the
/// reader is left on the MSB of GBSC; on a negative detection the reader
/// position is unchanged. The 17 bits of GBSC itself are **not**
/// consumed in either case — [`parse_gob_layer`] re-reads them.
///
/// Returns `false` (rather than erroring) when fewer than 16 bits remain,
/// so a caller probing past the final GOB of a picture simply sees "no
/// header".
pub fn gob_header_present(reader: &mut BitReader<'_>) -> bool {
    // Case 1 — GBSC directly at the current position (no GSTUF).
    if peek_is_gbsc(reader) {
        return true;
    }

    // Case 2 — a GSTUF run of 1..=7 zero bits that byte-aligns the GBSC.
    // GSTUF is only meaningful when the reader is not already byte
    // aligned; if it is aligned, case 1 already covered the only legal
    // GBSC position.
    let bit_in_byte = (reader.bit_position() & 7) as u32;
    if bit_in_byte == 0 {
        return false;
    }
    // 1..=7 bits to reach the next byte boundary.
    let stuff = 8 - bit_in_byte;
    // The candidate GSTUF bits must all be zero (§5.2.1).
    let mut probe = *reader;
    match probe.peek_u32(stuff + GBSC_BITS) {
        Ok(bits) => {
            let stuff_bits = bits >> GBSC_BITS; // the leading GSTUF run.
            let gbsc = bits & ((1 << GBSC_BITS) - 1);
            if stuff_bits == 0 && gbsc == GBSC_VALUE {
                // Discard the GSTUF (§5.2.1: "Decoders shall be designed
                // to discard GSTUF"), leaving the reader on GBSC's MSB.
                let _ = reader.skip(stuff);
                true
            } else {
                false
            }
        }
        Err(_) => false,
    }
}

/// Peek the next 17 bits and report whether they equal [`GBSC_VALUE`],
/// without consuming anything. Returns `false` when fewer than 17 bits
/// remain.
fn peek_is_gbsc(reader: &BitReader<'_>) -> bool {
    let mut probe = *reader;
    matches!(probe.peek_u32(GBSC_BITS), Ok(v) if v == GBSC_VALUE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Build a synthetic CPM = "0" GOB header (GBSC + GN + GFID +
    /// GQUANT). Input validity is the test's responsibility — the
    /// helper writes whatever bits the test asks for.
    fn build_gob_header(gn: u32, gfid: u32, gquant: u32) -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(gn, GN_BITS);
        w.write_u32(gfid, GFID_BITS);
        w.write_u32(gquant, GQUANT_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    #[test]
    fn parses_minimal_gob_header_gn1_quant1() {
        let bytes = build_gob_header(1, 0, 1);
        let gob = parse_gob_layer_from_bytes(&bytes).expect("parse");
        assert_eq!(gob.number, 1);
        assert_eq!(gob.gfid, 0);
        assert_eq!(gob.quantiser, 1);
        assert_eq!(gob.header_bits, GOB_HEADER_BITS_NO_CPM);
        assert_eq!(GOB_HEADER_BITS_NO_CPM, 17 + 5 + 2 + 5);
    }

    #[test]
    fn parses_max_legal_quantiser_and_gn() {
        // GN = 17 is the top of the standard-picture-format range;
        // QUANT = 31 is the top of the §5.2.6 range.
        let bytes = build_gob_header(17, 0b11, 31);
        let gob = parse_gob_layer_from_bytes(&bytes).expect("parse");
        assert_eq!(gob.number, 17);
        assert_eq!(gob.gfid, 0b11);
        assert_eq!(gob.quantiser, 31);
    }

    #[test]
    fn parses_custom_format_gn_range_up_to_24() {
        // GN = 24 is only legal under custom picture formats; the
        // parser accepts it (it does not know the picture format).
        let bytes = build_gob_header(24, 0, 5);
        let gob = parse_gob_layer_from_bytes(&bytes).expect("parse");
        assert_eq!(gob.number, 24);
        assert_eq!(gob.quantiser, 5);
    }

    #[test]
    fn parses_slice_emulated_gn_range_up_to_29() {
        // GN = 29 is reserved for Annex-K slice-header emulation
        // under CPM = "1" but is structurally a valid 5-bit code; we
        // accept it for the bit-layer parser and defer the
        // "this isn't really a GOB" decision to a higher layer.
        let bytes = build_gob_header(29, 0, 10);
        let gob = parse_gob_layer_from_bytes(&bytes).expect("parse");
        assert_eq!(gob.number, 29);
        assert_eq!(gob.quantiser, 10);
    }

    #[test]
    fn rejects_gn_zero_as_psc_overlap() {
        let bytes = build_gob_header(0, 0, 1);
        assert_eq!(
            parse_gob_layer_from_bytes(&bytes).unwrap_err(),
            Error::InvalidGroupNumber
        );
    }

    #[test]
    fn rejects_gn_30_eosbs_marker() {
        let bytes = build_gob_header(30, 0, 1);
        assert_eq!(
            parse_gob_layer_from_bytes(&bytes).unwrap_err(),
            Error::InvalidGroupNumber
        );
    }

    #[test]
    fn rejects_gn_31_eos_marker() {
        let bytes = build_gob_header(31, 0, 1);
        assert_eq!(
            parse_gob_layer_from_bytes(&bytes).unwrap_err(),
            Error::InvalidGroupNumber
        );
    }

    #[test]
    fn rejects_gquant_zero() {
        // §5.2.6 — QUANT ranges over 1..=31; the value 0 is not
        // representable as a half-step-size.
        let bytes = build_gob_header(1, 0, 0);
        assert_eq!(
            parse_gob_layer_from_bytes(&bytes).unwrap_err(),
            Error::InvalidQuantiser
        );
    }

    #[test]
    fn rejects_bad_gbsc_prefix() {
        // Flip the trailing "1" of the start code so the prefix
        // becomes 0000 0000 0000 0000 0 — still 17 bits, still
        // readable, but not GBSC.
        let mut w = BitWriter::new();
        w.write_u32(GBSC_VALUE ^ 0b1, GBSC_BITS);
        // Pad with enough zero bits that we don't trip
        // UnexpectedEof before the GBSC check.
        w.write_u32(0, 16);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_gob_layer_from_bytes(&bytes).unwrap_err(),
            Error::BadGroupStartCode
        );
    }

    #[test]
    fn short_buffer_yields_unexpected_eof_inside_gbsc() {
        // Only 8 bits in the buffer; GBSC is 17.
        let bytes = [0u8; 1];
        assert_eq!(
            parse_gob_layer_from_bytes(&bytes).unwrap_err(),
            Error::UnexpectedEof
        );
    }

    #[test]
    fn short_buffer_yields_unexpected_eof_inside_gquant() {
        // 24 bits — GBSC + GN fit but GFID + GQUANT do not.
        let mut w = BitWriter::new();
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(1, GN_BITS);
        // Only 2 more bits in the buffer (so GFID fits but GQUANT
        // is truncated). Pad to 24 bits total (= GBSC+GN+GFID).
        w.write_u32(0, GFID_BITS);
        let bytes = w.finish();
        // After write_u32 the writer aligns nothing; the buffer is
        // exactly 24 bits = 3 bytes.
        assert_eq!(bytes.len(), 3);
        assert_eq!(
            parse_gob_layer_from_bytes(&bytes).unwrap_err(),
            Error::UnexpectedEof
        );
    }

    #[test]
    fn detector_finds_byte_aligned_gbsc_with_gstuf() {
        // Six bits of macroblock data, then a 2-bit GSTUF run that
        // byte-aligns a real GOB header. The detector must report
        // "present" and leave the reader on the GBSC MSB so a follow-up
        // parse_gob_layer succeeds.
        let mut w = BitWriter::new();
        w.write_u32(0b101011, 6); // pretend MB data (must be non-zero-run)
                                  // GSTUF: 2 zero bits to reach the next byte boundary.
        w.write_bit(false);
        w.write_bit(false);
        // Now byte-aligned; emit a real header.
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(3, GN_BITS);
        w.write_u32(0, GFID_BITS);
        w.write_u32(9, GQUANT_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        r.skip(6).expect("skip mb data");
        assert!(gob_header_present(&mut r));
        // The 2 GSTUF bits were consumed; reader is on GBSC's MSB.
        assert_eq!(r.bit_position(), 8);
        let gob = parse_gob_layer(&mut r).expect("parse after detect");
        assert_eq!(gob.number, 3);
        assert_eq!(gob.quantiser, 9);
    }

    #[test]
    fn detector_finds_gbsc_at_current_position_no_gstuf() {
        // GBSC begins exactly at the current (already byte-aligned)
        // position — no GSTUF needed.
        let bytes = build_gob_header(5, 0, 4);
        let mut r = BitReader::new(&bytes);
        assert!(gob_header_present(&mut r));
        // No bits consumed by detection (no GSTUF present here).
        assert_eq!(r.bit_position(), 0);
        let gob = parse_gob_layer(&mut r).expect("parse");
        assert_eq!(gob.number, 5);
    }

    #[test]
    fn detector_reports_absent_when_macroblock_data_follows() {
        // A non-zero bit pattern (no 16-zero run) is plain macroblock
        // data, not a GOB header. The detector must not consume bits.
        let mut w = BitWriter::new();
        // 0xB5... has no run of 16 zeros.
        w.write_u32(0xB5A3_C71D, 32);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert!(!gob_header_present(&mut r));
        assert_eq!(r.bit_position(), 0);
    }

    #[test]
    fn detector_rejects_nonzero_gstuf_bits() {
        // A would-be GSTUF run that contains a 1 bit is not valid
        // stuffing; the detector must NOT treat the following aligned
        // GBSC as a header (that 1 bit is macroblock data).
        let mut w = BitWriter::new();
        w.write_u32(0b100000, 6); // 6 bits, leading 1 — first bit is data
                                  // 2 bits to byte boundary, but make one of them a 1.
        w.write_bit(true);
        w.write_bit(false);
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(0, 7);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        r.skip(6).expect("skip");
        assert!(!gob_header_present(&mut r));
        // No GSTUF consumed.
        assert_eq!(r.bit_position(), 6);
    }

    #[test]
    fn detector_reports_absent_at_end_of_stream() {
        // Fewer than 16 bits remain — probing past the final GOB.
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        r.skip(2).expect("skip");
        assert!(!gob_header_present(&mut r));
    }

    #[test]
    fn header_bits_constant_matches_field_widths() {
        assert_eq!(
            GOB_HEADER_BITS_NO_CPM,
            GBSC_BITS + GN_BITS + GFID_BITS + GQUANT_BITS
        );
        assert_eq!(GOB_HEADER_BITS_NO_CPM, 29);
    }

    #[test]
    fn reader_position_after_parse_is_immediately_after_gquant() {
        // Build a header followed by a 0xFF sentinel byte; the
        // sentinel must still be readable after parse_gob_layer
        // returns.
        let mut w = BitWriter::new();
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(7, GN_BITS);
        w.write_u32(0, GFID_BITS);
        w.write_u32(5, GQUANT_BITS);
        // Pad to byte alignment first so the sentinel byte is
        // distinguishable.
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.write_byte(0xFF);
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let gob = parse_gob_layer(&mut r).expect("parse");
        assert_eq!(gob.number, 7);
        assert_eq!(gob.quantiser, 5);
        // The reader should have consumed exactly 29 bits.
        assert_eq!(r.bit_position(), GOB_HEADER_BITS_NO_CPM as u64);
        // Skip the 3 padding bits to reach the sentinel.
        r.skip(3).expect("skip pad");
        assert_eq!(r.read_u32(8).expect("sentinel"), 0xFF);
    }

    #[test]
    fn composes_with_picture_header_round_trip() {
        // Round-1 picture header followed immediately by a round-2
        // GOB header. Verifies that the GOB parser picks up where the
        // picture parser left off, with no realignment.
        use crate::{parse_picture_header, PSC_BITS, PSC_VALUE};

        let mut w = BitWriter::new();
        // PSC + TR=0 + minimal QCIF intra PTYPE.
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8); // TR
        w.write_bit(true); // PTYPE bit1
        w.write_bit(false); // PTYPE bit2
        w.write_bit(false); // split-screen
        w.write_bit(false); // doc-camera
        w.write_bit(false); // freeze
        w.write_u32(0b010, 3); // source = QCIF
        w.write_bit(false); // coding = INTRA
        w.write_bit(false); // UMV
        w.write_bit(false); // SAC
        w.write_bit(false); // AP
        w.write_bit(false); // PB

        // Now the GOB header for group 1.
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(1, GN_BITS);
        w.write_u32(0b10, GFID_BITS);
        w.write_u32(8, GQUANT_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let pic = parse_picture_header(&mut r).expect("pic");
        assert_eq!(
            pic.source_format,
            crate::H263SourceFormat::Qcif,
            "picture-layer wiring sanity"
        );
        let gob = parse_gob_layer(&mut r).expect("gob");
        assert_eq!(gob.number, 1);
        assert_eq!(gob.gfid, 0b10);
        assert_eq!(gob.quantiser, 8);
    }
}
