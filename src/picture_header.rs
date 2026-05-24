//! H.263 picture-layer header parsing.
//!
//! This module implements the (non-extended-PTYPE) picture header as
//! defined in ITU-T Recommendation H.263 (01/2005) §5.1:
//!
//! * §5.1.1 — Picture Start Code (PSC), 22 bits, value `0x000020`
//!   (`0000 0000 0000 0000 1 00000`); byte-aligned.
//! * §5.1.2 — Temporal Reference (TR), 8 bits at the standard CIF
//!   picture clock frequency. (The 10-bit ETR+TR custom-PCF form lives
//!   inside the extended-PTYPE header, which is deferred.)
//! * §5.1.3 — Type Information (PTYPE), 13 bits when source format is
//!   not `"111"`:
//!   - Bit 1: always `1` (start-code-emulation guard).
//!   - Bit 2: always `0` (distinguishes from ITU-T Rec. H.261).
//!   - Bit 3: Split-screen indicator.
//!   - Bit 4: Document-camera indicator.
//!   - Bit 5: Full-picture freeze release.
//!   - Bits 6-8: Source format (`001` sub-QCIF .. `101` 16CIF; `110`
//!     reserved; `111` selects the extended PTYPE / PLUSPTYPE path).
//!   - Bit 9: Picture coding type, `0` INTRA, `1` INTER.
//!   - Bit 10: Unrestricted Motion Vector mode (Annex D).
//!   - Bit 11: Syntax-based Arithmetic Coding mode (Annex E).
//!   - Bit 12: Advanced Prediction mode (Annex F).
//!   - Bit 13: PB-frames mode (Annex G).
//!
//! Round 1 deliberately stops at PTYPE. Subsequent fields (PQUANT,
//! CPM/PSBI, TRB/DBQUANT, PEI/PSUPP, GOB data, ESTUF/EOS/PSTUF) are
//! out of scope and will be added incrementally. If PTYPE bits 6-8
//! signal `"111"`, the parser returns
//! [`Error::ExtendedPtypeNotSupported`] rather than guessing at the
//! 12-or-30-bit PLUSPTYPE structure.
//!
//! All bit numbering follows the spec's 1-based "Bit N" convention;
//! the implementation reads MSB-first via
//! [`oxideav_core::bits::BitReader`].

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Length in bits of the Picture Start Code (§5.1.1).
pub const PSC_BITS: u32 = 22;

/// Value of the Picture Start Code: `0000 0000 0000 0000 1 00000`.
///
/// As a 22-bit unsigned integer that is `0x000020` (decimal 32). All
/// picture start codes are byte-aligned; the caller is responsible
/// for skipping PSTUF stuffing before invoking the parser.
pub const PSC_VALUE: u32 = 0x0000_0020;

/// Source-format field of PTYPE (§5.1.3, bits 6-8).
///
/// The forbidden code `"000"` is rejected by the parser; the
/// `"111"` "extended PTYPE" path is recognised but not yet decoded
/// (returns [`Error::ExtendedPtypeNotSupported`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum H263SourceFormat {
    /// `"001"` — sub-QCIF (128 × 96 luma).
    SubQcif,
    /// `"010"` — QCIF (176 × 144 luma).
    Qcif,
    /// `"011"` — CIF (352 × 288 luma).
    Cif,
    /// `"100"` — 4CIF (704 × 576 luma).
    Cif4,
    /// `"101"` — 16CIF (1408 × 1152 luma).
    Cif16,
    /// `"110"` — reserved.
    Reserved110,
}

impl H263SourceFormat {
    /// Spec-defined nominal luma dimensions (§4.1, Table 4).
    ///
    /// Returns `None` for the reserved `"110"` value.
    pub fn luma_dimensions(self) -> Option<(u32, u32)> {
        match self {
            H263SourceFormat::SubQcif => Some((128, 96)),
            H263SourceFormat::Qcif => Some((176, 144)),
            H263SourceFormat::Cif => Some((352, 288)),
            H263SourceFormat::Cif4 => Some((704, 576)),
            H263SourceFormat::Cif16 => Some((1408, 1152)),
            H263SourceFormat::Reserved110 => None,
        }
    }

    /// Number of GOBs per picture in the non-Reduced-Resolution case
    /// (§4.2.1): 6 for sub-QCIF, 9 for QCIF, 18 for CIF / 4CIF / 16CIF.
    ///
    /// Returns `None` for the reserved `"110"` value.
    pub fn num_gobs(self) -> Option<u32> {
        match self {
            H263SourceFormat::SubQcif => Some(6),
            H263SourceFormat::Qcif => Some(9),
            H263SourceFormat::Cif => Some(18),
            H263SourceFormat::Cif4 => Some(18),
            H263SourceFormat::Cif16 => Some(18),
            H263SourceFormat::Reserved110 => None,
        }
    }

    /// Number of 16×16 luma macroblock rows that one GOB spans, in the
    /// non-Reduced-Resolution case (§4.2.1 / §4.2): one row for
    /// sub-QCIF / QCIF / CIF, two rows for 4CIF, four rows for 16CIF.
    ///
    /// Returns `None` for the reserved `"110"` value.
    pub fn mb_rows_per_gob(self) -> Option<u32> {
        match self {
            H263SourceFormat::SubQcif | H263SourceFormat::Qcif | H263SourceFormat::Cif => Some(1),
            H263SourceFormat::Cif4 => Some(2),
            H263SourceFormat::Cif16 => Some(4),
            H263SourceFormat::Reserved110 => None,
        }
    }

    /// Number of 16×16 macroblocks across one row of the picture
    /// (luma width / 16). 11 for QCIF, 22 for CIF, etc.
    ///
    /// Returns `None` for the reserved `"110"` value.
    pub fn mbs_per_row(self) -> Option<u32> {
        self.luma_dimensions().map(|(w, _)| w / 16)
    }

    /// Total number of 16×16 macroblocks in the picture
    /// (`mbs_per_row * mb_rows`). Returns `None` for `"110"`.
    pub fn total_macroblocks(self) -> Option<u32> {
        match (self.luma_dimensions(), self.mbs_per_row()) {
            (Some((_, h)), Some(per_row)) => Some(per_row * (h / 16)),
            _ => None,
        }
    }
}

/// Picture coding type (§5.1.3, bit 9).
///
/// The non-extended PTYPE header distinguishes only INTRA and INTER;
/// the richer set of types (Improved PB, B, EI, EP, …) is signalled
/// through MPPTYPE in the extended PTYPE path (§5.1.4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum H263PictureCodingType {
    /// `"0"` — I-picture (INTRA).
    Intra,
    /// `"1"` — P-picture (INTER).
    Inter,
}

/// Parsed H.263 picture-layer header (non-extended-PTYPE branch).
///
/// Field naming mirrors the spec. Boolean fields use `false` for the
/// spec's `"off"` / `"0"` and `true` for `"on"` / `"1"`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct H263PictureHeader {
    /// §5.1.2 — Temporal Reference (8 bits at standard CIF PCF).
    pub temporal_reference: u8,
    /// §5.1.3 bit 3 — Split-screen indicator.
    pub split_screen: bool,
    /// §5.1.3 bit 4 — Document-camera indicator.
    pub document_camera: bool,
    /// §5.1.3 bit 5 — Full-picture freeze release.
    pub freeze_release: bool,
    /// §5.1.3 bits 6-8 — Source format.
    pub source_format: H263SourceFormat,
    /// §5.1.3 bit 9 — Picture coding type (INTRA / INTER).
    pub coding_type: H263PictureCodingType,
    /// §5.1.3 bit 10 — Unrestricted Motion Vector mode (Annex D).
    pub umv_mode: bool,
    /// §5.1.3 bit 11 — Syntax-based Arithmetic Coding mode (Annex E).
    pub sac_mode: bool,
    /// §5.1.3 bit 12 — Advanced Prediction mode (Annex F).
    pub advanced_prediction: bool,
    /// §5.1.3 bit 13 — PB-frames mode (Annex G).
    pub pb_frames: bool,
}

/// Parse an H.263 picture-layer header starting at the current
/// position of `reader`.
///
/// The caller is responsible for placing the reader at the first bit
/// of the PSC — i.e. for skipping any leading PSTUF stuffing such that
/// the next bit is the MSB of the 22-bit start code.
///
/// On success the reader is left positioned immediately after PTYPE.
/// On error the reader's position is unspecified.
pub fn parse_picture_header(reader: &mut BitReader<'_>) -> Result<H263PictureHeader> {
    // §5.1.1 — Picture Start Code (22 bits, value 0x000020).
    let psc = reader
        .read_u32(PSC_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    if psc != PSC_VALUE {
        return Err(Error::BadPictureStartCode);
    }

    // §5.1.2 — Temporal Reference (8 bits at standard CIF PCF).
    let tr = reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8;

    // §5.1.3 — PTYPE bit 1: always 1.
    let ptype_bit1 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if !ptype_bit1 {
        return Err(Error::BadPtypeFixedBits);
    }
    // §5.1.3 — PTYPE bit 2: always 0.
    let ptype_bit2 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if ptype_bit2 {
        return Err(Error::BadPtypeFixedBits);
    }

    // §5.1.3 — PTYPE bits 3..5: split-screen / doc-camera / freeze.
    let split_screen = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let document_camera = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let freeze_release = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;

    // §5.1.3 — PTYPE bits 6..8: source format.
    let source_format_bits = reader.read_u32(3).map_err(|_| Error::UnexpectedEof)?;
    let source_format = match source_format_bits {
        0b000 => return Err(Error::ForbiddenSourceFormat),
        0b001 => H263SourceFormat::SubQcif,
        0b010 => H263SourceFormat::Qcif,
        0b011 => H263SourceFormat::Cif,
        0b100 => H263SourceFormat::Cif4,
        0b101 => H263SourceFormat::Cif16,
        0b110 => H263SourceFormat::Reserved110,
        0b111 => return Err(Error::ExtendedPtypeNotSupported),
        // `read_u32(3)` cannot return more than 7.
        _ => unreachable!(),
    };

    // §5.1.3 — PTYPE bits 9..13: coding-type + optional-mode flags.
    let coding_type_bit = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let coding_type = if coding_type_bit {
        H263PictureCodingType::Inter
    } else {
        H263PictureCodingType::Intra
    };
    let umv_mode = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let sac_mode = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let advanced_prediction = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let pb_frames = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;

    Ok(H263PictureHeader {
        temporal_reference: tr,
        split_screen,
        document_camera,
        freeze_release,
        source_format,
        coding_type,
        umv_mode,
        sac_mode,
        advanced_prediction,
        pb_frames,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Build a synthetic picture header (non-extended PTYPE) per
    /// §5.1.1–§5.1.3 layout. Inputs map 1:1 to the parsed struct;
    /// validity is the test's responsibility.
    #[allow(clippy::too_many_arguments)]
    fn build_header(
        tr: u8,
        split_screen: bool,
        document_camera: bool,
        freeze_release: bool,
        source_format_bits: u32,
        coding_type_inter: bool,
        umv: bool,
        sac: bool,
        ap: bool,
        pb: bool,
    ) -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(tr as u32, 8);
        w.write_bit(true); // PTYPE bit1 = 1
        w.write_bit(false); // PTYPE bit2 = 0
        w.write_bit(split_screen);
        w.write_bit(document_camera);
        w.write_bit(freeze_release);
        w.write_u32(source_format_bits, 3);
        w.write_bit(coding_type_inter);
        w.write_bit(umv);
        w.write_bit(sac);
        w.write_bit(ap);
        w.write_bit(pb);
        // Pad to byte alignment so the test buffer is well-formed.
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    #[test]
    fn parses_minimal_qcif_intra_picture() {
        // QCIF intra, TR=0, all optional flags off.
        let bytes = build_header(
            0, false, false, false, 0b010, false, false, false, false, false,
        );
        let hdr = parse_picture_header_from(&bytes).expect("parse");
        assert_eq!(hdr.temporal_reference, 0);
        assert!(!hdr.split_screen);
        assert!(!hdr.document_camera);
        assert!(!hdr.freeze_release);
        assert_eq!(hdr.source_format, H263SourceFormat::Qcif);
        assert_eq!(hdr.source_format.luma_dimensions(), Some((176, 144)));
        assert_eq!(hdr.coding_type, H263PictureCodingType::Intra);
        assert!(!hdr.umv_mode);
        assert!(!hdr.sac_mode);
        assert!(!hdr.advanced_prediction);
        assert!(!hdr.pb_frames);
    }

    #[test]
    fn parses_cif_inter_picture_with_all_mode_flags_set() {
        // CIF inter, TR=200, every optional-mode flag on.
        let bytes = build_header(200, true, true, true, 0b011, true, true, true, true, true);
        let hdr = parse_picture_header_from(&bytes).expect("parse");
        assert_eq!(hdr.temporal_reference, 200);
        assert!(hdr.split_screen);
        assert!(hdr.document_camera);
        assert!(hdr.freeze_release);
        assert_eq!(hdr.source_format, H263SourceFormat::Cif);
        assert_eq!(hdr.source_format.luma_dimensions(), Some((352, 288)));
        assert_eq!(hdr.coding_type, H263PictureCodingType::Inter);
        assert!(hdr.umv_mode);
        assert!(hdr.sac_mode);
        assert!(hdr.advanced_prediction);
        assert!(hdr.pb_frames);
    }

    #[test]
    fn each_source_format_round_trips_to_spec_dimensions() {
        let cases = [
            (0b001, H263SourceFormat::SubQcif, Some((128, 96))),
            (0b010, H263SourceFormat::Qcif, Some((176, 144))),
            (0b011, H263SourceFormat::Cif, Some((352, 288))),
            (0b100, H263SourceFormat::Cif4, Some((704, 576))),
            (0b101, H263SourceFormat::Cif16, Some((1408, 1152))),
            (0b110, H263SourceFormat::Reserved110, None),
        ];
        for (bits, expected_fmt, expected_dims) in cases {
            let bytes = build_header(
                1, false, false, false, bits, false, false, false, false, false,
            );
            let hdr = parse_picture_header_from(&bytes).expect("parse");
            assert_eq!(hdr.source_format, expected_fmt, "bits={:#05b}", bits);
            assert_eq!(hdr.source_format.luma_dimensions(), expected_dims);
        }
    }

    #[test]
    fn forbidden_source_format_000_is_rejected() {
        let bytes = build_header(
            0, false, false, false, 0b000, false, false, false, false, false,
        );
        assert_eq!(
            parse_picture_header_from(&bytes).unwrap_err(),
            Error::ForbiddenSourceFormat
        );
    }

    #[test]
    fn extended_ptype_111_returns_specific_error() {
        // Source-format 111 means "PLUSPTYPE follows"; we do not
        // decode that path in round 1 and want a distinct error rather
        // than misinterpreting subsequent bits.
        let bytes = build_header(
            0, false, false, false, 0b111, false, false, false, false, false,
        );
        assert_eq!(
            parse_picture_header_from(&bytes).unwrap_err(),
            Error::ExtendedPtypeNotSupported
        );
    }

    #[test]
    fn bad_psc_is_detected() {
        // Same length as a real PSC but flip a single bit in the
        // start-code prefix.
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE ^ 0b1, PSC_BITS);
        w.write_u32(0, 8);
        w.write_u32(0, 16); // pad enough that EOF doesn't pre-empt the PSC error
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_picture_header_from(&bytes).unwrap_err(),
            Error::BadPictureStartCode
        );
    }

    #[test]
    fn ptype_bit2_violation_is_rejected() {
        // Construct a buffer where PTYPE bit 2 is 1 (spec says always 0).
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8); // TR
        w.write_bit(true); // PTYPE bit1 = 1 (correct)
        w.write_bit(true); // PTYPE bit2 = 1 (illegal)
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_u32(0b010, 3);
        w.write_u32(0, 5);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_picture_header_from(&bytes).unwrap_err(),
            Error::BadPtypeFixedBits
        );
    }

    #[test]
    fn short_buffer_yields_unexpected_eof() {
        // Just the PSC bits and nothing else.
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_picture_header_from(&bytes).unwrap_err(),
            Error::UnexpectedEof
        );
    }

    fn parse_picture_header_from(data: &[u8]) -> Result<H263PictureHeader> {
        let mut r = BitReader::new(data);
        parse_picture_header(&mut r)
    }
}
