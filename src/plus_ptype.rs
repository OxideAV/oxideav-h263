//! H.263 extended-PTYPE (PLUSPTYPE) picture-header parsing.
//!
//! This module implements the optional extended-PTYPE picture header as
//! defined in ITU-T Recommendation H.263 (01/2005) §5.1.4 through
//! §5.1.18 — the "optional PLUSPTYPE-related fields" block of Figure 8,
//! which is located immediately after PTYPE when bits 6-8 of PTYPE are
//! `"111"` (§5.1.3).
//!
//! Per Figure 8/H.263 the on-wire order of the block is:
//!
//! ```text
//!   PLUSPTYPE CPM PSBI CPFMT EPAR CPCFC ETR UUI SSS
//!     ELNUM RLNUM RPSMF TRPI TRP BCI BCM
//!       RPRP ...
//! ```
//!
//! `PLUSPTYPE` itself (§5.1.4) is up to three subfields:
//!
//! * §5.1.4.1 — **UFEP** (Update Full Extended PTYPE), 3 bits. `"001"`
//!   means the 18-bit optional part (OPPTYPE) follows; `"000"` means it
//!   does not (the modes are inferred from a prior `UFEP = "001"`).
//! * §5.1.4.2 — **OPPTYPE** (Optional part), 18 bits, present only when
//!   `UFEP = "001"`: source format, custom-PCF, and the Annex
//!   D/E/F/I/J/K/N/R/S/T mode flags, plus the bit-15 start-code-emulation
//!   guard and three reserved zero bits.
//! * §5.1.4.3 — **MPPTYPE** (Mandatory part), 9 bits, always present:
//!   the picture-type code (I / P / Improved-PB / B / EI / EP), the RPR
//!   and RRU mode flags, the rounding type, two reserved bits, and a
//!   bit-9 start-code-emulation guard.
//!
//! After PLUSPTYPE, the deterministic-width fields are parsed in the
//! Figure-8 order. Their presence rules (§5.1.5–§5.1.18):
//!
//! * §5.1.20 / §5.1.7 / §5.1.4.7 — **CPM** (1 bit) always present here
//!   (it follows PLUSPTYPE when PLUSPTYPE is present); **PSBI** (2 bits)
//!   only when `CPM = "1"`.
//! * §5.1.5 — **CPFMT** (23 bits): present only when OPPTYPE signalled a
//!   custom source format (OPPTYPE source-format `"110"`) and
//!   `UFEP = "001"`.
//! * §5.1.6 — **EPAR** (16 bits): present only when CPFMT is present and
//!   its PAR code is the extended value `"1111"`.
//! * §5.1.7 — **CPCFC** (8 bits): present only when OPPTYPE signalled a
//!   custom PCF and `UFEP = "001"`.
//! * §5.1.8 — **ETR** (2 bits): present whenever a custom PCF is in use
//!   (regardless of UFEP). With `UFEP = "000"` the custom-PCF state is
//!   inferred from a prior picture; this parser keys ETR presence off the
//!   caller-supplied [`InheritedExtendedState::custom_pcf`].
//! * §5.1.9 — **UUI** (1 or 2 bits): present only when the UMV mode is
//!   on in OPPTYPE and `UFEP = "001"`.
//! * §5.1.10 — **SSS** (2 bits): present only when the Slice Structured
//!   mode is on in OPPTYPE and `UFEP = "001"`.
//! * §5.1.11–§5.1.18 — the scalability / reference-picture-selection
//!   fields ELNUM, RLNUM, RPSMF, TRPI, TRP, BCI, BCM, RPRP. These carry
//!   variable-length and externally-negotiated sub-bitstreams (Annexes
//!   N, O, P) that are not staged for byte-level parsing here. The
//!   parser stops with [`PlusPtypeUnsupported`] if the corresponding
//!   mode bits are set, rather than mis-framing the remaining header.
//!
//! All bit numbering follows the spec's 1-based "Bit N" convention; the
//! implementation reads MSB-first via [`oxideav_core::bits::BitReader`].

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Length in bits of UFEP (§5.1.4.1).
pub const UFEP_BITS: u32 = 3;
/// Length in bits of OPPTYPE (§5.1.4.2).
pub const OPPTYPE_BITS: u32 = 18;
/// Length in bits of MPPTYPE (§5.1.4.3).
pub const MPPTYPE_BITS: u32 = 9;
/// Length in bits of CPFMT (§5.1.5).
pub const CPFMT_BITS: u32 = 23;
/// Length in bits of EPAR (§5.1.6).
pub const EPAR_BITS: u32 = 16;
/// Length in bits of CPCFC (§5.1.7).
pub const CPCFC_BITS: u32 = 8;
/// Length in bits of ETR (§5.1.8).
pub const ETR_BITS: u32 = 2;
/// Length in bits of SSS (§5.1.10).
pub const SSS_BITS: u32 = 2;

/// UFEP value indicating the full optional part (OPPTYPE) is present.
pub const UFEP_FULL: u32 = 0b001;
/// UFEP value indicating only MPPTYPE is present (modes inferred).
pub const UFEP_MANDATORY_ONLY: u32 = 0b000;

/// OPPTYPE source-format code `"110"` — custom source format (CPFMT
/// follows when UFEP is `"001"`).
pub const OPPTYPE_SRCFMT_CUSTOM: u32 = 0b110;

/// CPFMT PAR-code value `"1111"` — extended PAR, EPAR follows (§5.1.5,
/// Table 5).
pub const PAR_CODE_EXTENDED: u32 = 0b1111;

/// State carried from a prior `UFEP = "001"` picture header that a
/// `UFEP = "000"` header inherits (§5.1.4.4 / §5.1.8).
///
/// When `UFEP = "000"` the OPPTYPE bits are absent, so whether a custom
/// PCF is in use (which gates the §5.1.8 ETR field) cannot be read from
/// the current header — it is inferred from the last `UFEP = "001"`
/// picture. The caller supplies that inherited state; for a fresh
/// stream where no prior `UFEP = "001"` was seen, the spec default of
/// "no custom PCF" applies (`Default`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InheritedExtendedState {
    /// Whether a custom picture clock frequency was last signalled in
    /// use (gates §5.1.8 ETR presence when `UFEP = "000"`).
    pub custom_pcf: bool,
}

/// Picture-type code from MPPTYPE bits 1-3 (§5.1.4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlusPictureType {
    /// `"000"` — I-picture (INTRA).
    Intra,
    /// `"001"` — P-picture (INTER).
    Inter,
    /// `"010"` — Improved PB-frame (Annex M).
    ImprovedPb,
    /// `"011"` — B-picture (Annex O).
    BPicture,
    /// `"100"` — EI-picture (Annex O).
    EiPicture,
    /// `"101"` — EP-picture (Annex O).
    EpPicture,
}

/// OPPTYPE source format (§5.1.4.2, bits 1-3). Mirrors the §5.1.3 set
/// but adds the `"110"` custom-source-format code that the baseline
/// PTYPE does not have.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlusSourceFormat {
    /// `"001"` — sub-QCIF.
    SubQcif,
    /// `"010"` — QCIF.
    Qcif,
    /// `"011"` — CIF.
    Cif,
    /// `"100"` — 4CIF.
    Cif4,
    /// `"101"` — 16CIF.
    Cif16,
    /// `"110"` — custom source format (CPFMT carries the dimensions).
    Custom,
}

impl PlusSourceFormat {
    /// Spec-defined nominal luma dimensions (§4.1, Table 4) for the
    /// fixed formats. Returns `None` for [`PlusSourceFormat::Custom`]
    /// (the dimensions come from CPFMT, not this field).
    pub fn luma_dimensions(self) -> Option<(u32, u32)> {
        match self {
            PlusSourceFormat::SubQcif => Some((128, 96)),
            PlusSourceFormat::Qcif => Some((176, 144)),
            PlusSourceFormat::Cif => Some((352, 288)),
            PlusSourceFormat::Cif4 => Some((704, 576)),
            PlusSourceFormat::Cif16 => Some((1408, 1152)),
            PlusSourceFormat::Custom => None,
        }
    }
}

/// The §5.1.4.2 optional part of PLUSPTYPE (OPPTYPE, 18 bits), decoded
/// when `UFEP = "001"`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Opptype {
    /// Bits 1-3 — source format.
    pub source_format: PlusSourceFormat,
    /// Bit 4 — custom PCF in use (`true`) vs. standard CIF PCF.
    pub custom_pcf: bool,
    /// Bit 5 — Unrestricted Motion Vector mode (Annex D).
    pub umv: bool,
    /// Bit 6 — Syntax-based Arithmetic Coding mode (Annex E).
    pub sac: bool,
    /// Bit 7 — Advanced Prediction mode (Annex F).
    pub advanced_prediction: bool,
    /// Bit 8 — Advanced INTRA Coding mode (Annex I).
    pub advanced_intra: bool,
    /// Bit 9 — Deblocking Filter mode (Annex J).
    pub deblocking: bool,
    /// Bit 10 — Slice Structured mode (Annex K).
    pub slice_structured: bool,
    /// Bit 11 — Reference Picture Selection mode (Annex N).
    pub reference_picture_selection: bool,
    /// Bit 12 — Independent Segment Decoding mode (Annex R).
    pub independent_segment_decoding: bool,
    /// Bit 13 — Alternative INTER VLC mode (Annex S).
    pub alternative_inter_vlc: bool,
    /// Bit 14 — Modified Quantization mode (Annex T).
    pub modified_quantization: bool,
}

/// The §5.1.4.3 mandatory part of PLUSPTYPE (MPPTYPE, 9 bits), always
/// present when PLUSPTYPE is present.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpptype {
    /// Bits 1-3 — picture type.
    pub picture_type: PlusPictureType,
    /// Bit 4 — Reference Picture Resampling mode (Annex P).
    pub reference_picture_resampling: bool,
    /// Bit 5 — Reduced-Resolution Update mode (Annex Q).
    pub reduced_resolution_update: bool,
    /// Bit 6 — Rounding Type (RTYPE), §6.1.2.
    pub rounding_type: bool,
}

/// §5.1.5 Custom Picture Format (CPFMT, 23 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CustomPictureFormat {
    /// Bits 1-4 — Pixel Aspect Ratio code (Table 5).
    pub par_code: u8,
    /// Bits 5-13 — Picture Width Indication. Pixels per line is
    /// `(width_indication + 1) * 4`.
    pub width_indication: u16,
    /// Bits 15-23 — Picture Height Indication. Number of lines is
    /// `height_indication * 4`.
    pub height_indication: u16,
}

impl CustomPictureFormat {
    /// Luma width in pixels: `(PWI + 1) * 4` (§5.1.5).
    pub fn luma_width(self) -> u32 {
        (self.width_indication as u32 + 1) * 4
    }

    /// Luma height in lines: `PHI * 4` (§5.1.5).
    pub fn luma_height(self) -> u32 {
        self.height_indication as u32 * 4
    }
}

/// §5.1.6 Extended Pixel Aspect Ratio (EPAR, 16 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExtendedPar {
    /// Bits 1-8 — PAR width (`"0"` forbidden).
    pub width: u8,
    /// Bits 9-16 — PAR height (`"0"` forbidden).
    pub height: u8,
}

/// §5.1.7 Custom Picture Clock Frequency Code (CPCFC, 8 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CustomPcf {
    /// Bit 1 — clock conversion code: `false` → factor 1000, `true` →
    /// 1001.
    pub conversion_1001: bool,
    /// Bits 2-8 — clock divisor (`"0"` forbidden).
    pub divisor: u8,
}

/// §5.1.9 Unlimited Unrestricted Motion Vectors Indicator (UUI).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Uui {
    /// `"1"` — motion-vector range limited per Tables D.1 / D.2.
    Limited,
    /// `"01"` — range limited only by picture size.
    Unlimited,
}

/// §5.1.10 Slice Structured Submode bits (SSS, 2 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SliceStructuredSubmode {
    /// Bit 1 — `true` → rectangular slices, `false` → free-running.
    pub rectangular: bool,
    /// Bit 2 — `true` → arbitrary slice ordering, `false` → sequential.
    pub arbitrary_order: bool,
}

/// A fully parsed extended-PTYPE (PLUSPTYPE) picture header.
///
/// Fields that are absent on the wire (because their presence rule did
/// not fire) are `None`. The five baseline PTYPE indicator bits
/// (split-screen, document-camera, freeze-release) are **not** part of
/// the PLUSPTYPE path: when PTYPE bits 6-8 are `"111"` the bits beyond
/// the source-format field are the PLUSPTYPE codeword itself, so this
/// struct begins at UFEP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlusPtypeHeader {
    /// §5.1.4.1 — UFEP value (`0b000` or `0b001`).
    pub ufep: u8,
    /// §5.1.4.2 — optional part, present only when `ufep == 0b001`.
    pub opptype: Option<Opptype>,
    /// §5.1.4.3 — mandatory part, always present.
    pub mpptype: Mpptype,
    /// §5.1.20 — CPM flag (1 bit), always present in the PLUSPTYPE path.
    pub cpm: bool,
    /// §5.1.21 — PSBI (2-bit sub-bitstream number), present iff CPM.
    pub psbi: Option<u8>,
    /// §5.1.5 — Custom Picture Format, present iff a custom source
    /// format was signalled and `ufep == 0b001`.
    pub cpfmt: Option<CustomPictureFormat>,
    /// §5.1.6 — Extended PAR, present iff CPFMT carries the extended
    /// PAR code.
    pub epar: Option<ExtendedPar>,
    /// §5.1.7 — Custom PCF code, present iff a custom PCF was signalled
    /// and `ufep == 0b001`.
    pub cpcfc: Option<CustomPcf>,
    /// §5.1.8 — Extended TR, present iff a custom PCF is in use.
    pub etr: Option<u8>,
    /// §5.1.9 — UUI, present iff UMV mode on and `ufep == 0b001`.
    pub uui: Option<Uui>,
    /// §5.1.10 — Slice Structured submode, present iff SS mode on and
    /// `ufep == 0b001`.
    pub sss: Option<SliceStructuredSubmode>,
}

impl PlusPtypeHeader {
    /// Effective source format. When `opptype` carried a fixed format,
    /// returns its [`PlusSourceFormat`]; for a custom format the caller
    /// should read [`CustomPictureFormat`] from [`Self::cpfmt`]. When
    /// `UFEP = "000"` there is no OPPTYPE and the source format is
    /// inherited (returns `None`).
    pub fn source_format(&self) -> Option<PlusSourceFormat> {
        self.opptype.map(|o| o.source_format)
    }

    /// Whether a custom PCF is in use, accounting for the inherited
    /// state when `UFEP = "000"`. With `UFEP = "001"` it is the OPPTYPE
    /// bit-4 value; with `UFEP = "000"` it is `inherited.custom_pcf`.
    pub fn custom_pcf(&self, inherited: InheritedExtendedState) -> bool {
        match self.opptype {
            Some(o) => o.custom_pcf,
            None => inherited.custom_pcf,
        }
    }
}

/// Parse the OPPTYPE 18-bit field (§5.1.4.2). The reader must be
/// positioned at OPPTYPE bit 1.
fn parse_opptype(reader: &mut BitReader<'_>) -> Result<Opptype> {
    let raw = reader
        .read_u32(OPPTYPE_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    // MSB-first: bit 1 is the most-significant of the 18.
    let bit = |n: u32| -> bool { (raw >> (OPPTYPE_BITS - n)) & 1 == 1 };

    let source_format = match (raw >> (OPPTYPE_BITS - 3)) & 0b111 {
        0b000 => return Err(Error::PlusPtypeReservedField),
        0b001 => PlusSourceFormat::SubQcif,
        0b010 => PlusSourceFormat::Qcif,
        0b011 => PlusSourceFormat::Cif,
        0b100 => PlusSourceFormat::Cif4,
        0b101 => PlusSourceFormat::Cif16,
        0b110 => PlusSourceFormat::Custom,
        0b111 => return Err(Error::PlusPtypeReservedField),
        _ => unreachable!(),
    };

    // Bit 15 must be "1" (start-code-emulation guard); bits 16-18
    // reserved "0".
    if !bit(15) {
        return Err(Error::PlusPtypeReservedField);
    }
    if bit(16) || bit(17) || bit(18) {
        return Err(Error::PlusPtypeReservedField);
    }

    Ok(Opptype {
        source_format,
        custom_pcf: bit(4),
        umv: bit(5),
        sac: bit(6),
        advanced_prediction: bit(7),
        advanced_intra: bit(8),
        deblocking: bit(9),
        slice_structured: bit(10),
        reference_picture_selection: bit(11),
        independent_segment_decoding: bit(12),
        alternative_inter_vlc: bit(13),
        modified_quantization: bit(14),
    })
}

/// Parse the MPPTYPE 9-bit field (§5.1.4.3). The reader must be
/// positioned at MPPTYPE bit 1.
fn parse_mpptype(reader: &mut BitReader<'_>) -> Result<Mpptype> {
    let raw = reader
        .read_u32(MPPTYPE_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    let bit = |n: u32| -> bool { (raw >> (MPPTYPE_BITS - n)) & 1 == 1 };

    let picture_type = match (raw >> (MPPTYPE_BITS - 3)) & 0b111 {
        0b000 => PlusPictureType::Intra,
        0b001 => PlusPictureType::Inter,
        0b010 => PlusPictureType::ImprovedPb,
        0b011 => PlusPictureType::BPicture,
        0b100 => PlusPictureType::EiPicture,
        0b101 => PlusPictureType::EpPicture,
        0b110 | 0b111 => return Err(Error::PlusPtypeReservedField),
        _ => unreachable!(),
    };

    // Bit 9 must be "1" (start-code-emulation guard); bits 7-8 reserved
    // "0".
    if bit(7) || bit(8) {
        return Err(Error::PlusPtypeReservedField);
    }
    if !bit(9) {
        return Err(Error::PlusPtypeReservedField);
    }

    Ok(Mpptype {
        picture_type,
        reference_picture_resampling: bit(4),
        reduced_resolution_update: bit(5),
        rounding_type: bit(6),
    })
}

/// Parse the extended-PTYPE picture-header fields that follow PTYPE's
/// source-format `"111"` indicator.
///
/// The caller must position `reader` at the first bit of UFEP — i.e.
/// immediately after consuming PTYPE bits 1-8 (with bits 6-8 == `"111"`).
/// `inherited` supplies the state a `UFEP = "000"` header needs (the
/// custom-PCF flag that gates ETR); pass [`InheritedExtendedState::default`]
/// for a fresh stream.
///
/// Returns [`Error::PlusPtypeUnsupported`] when the header signals one
/// of the variable-length / externally-negotiated sub-bitstreams
/// (reference-picture-selection, slice-structured, scalability layers,
/// or reference-picture-resampling) whose byte-level layout is not
/// staged here, rather than mis-framing the remaining header.
pub fn parse_plus_ptype(
    reader: &mut BitReader<'_>,
    inherited: InheritedExtendedState,
) -> Result<PlusPtypeHeader> {
    // §5.1.4.1 — UFEP.
    let ufep = reader
        .read_u32(UFEP_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    let opptype = match ufep {
        UFEP_MANDATORY_ONLY => None,
        UFEP_FULL => Some(parse_opptype(reader)?),
        // Values other than "000" / "001" are reserved (§5.1.4.1).
        _ => return Err(Error::PlusPtypeReservedField),
    };

    // §5.1.4.3 — MPPTYPE (always present).
    let mpptype = parse_mpptype(reader)?;

    // §5.1.4.7 / §5.1.20 — CPM follows immediately after PLUSPTYPE.
    let cpm = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    // §5.1.21 — PSBI present iff CPM.
    let psbi = if cpm {
        Some(reader.read_u32(2).map_err(|_| Error::UnexpectedEof)? as u8)
    } else {
        None
    };

    // §5.1.5 — CPFMT present iff custom source format and UFEP == 001.
    let custom_source = opptype
        .map(|o| o.source_format == PlusSourceFormat::Custom)
        .unwrap_or(false);
    let cpfmt = if custom_source {
        Some(parse_cpfmt(reader)?)
    } else {
        None
    };

    // §5.1.6 — EPAR present iff CPFMT carries the extended PAR code.
    let epar = match cpfmt {
        Some(c) if c.par_code as u32 == PAR_CODE_EXTENDED => Some(parse_epar(reader)?),
        _ => None,
    };

    // §5.1.7 — CPCFC present iff custom PCF signalled and UFEP == 001.
    let opptype_custom_pcf = opptype.map(|o| o.custom_pcf).unwrap_or(false);
    let cpcfc = if opptype_custom_pcf {
        Some(parse_cpcfc(reader)?)
    } else {
        None
    };

    // §5.1.8 — ETR present iff a custom PCF is in use (regardless of
    // UFEP). With UFEP == 001 that is the OPPTYPE bit; with UFEP == 000
    // it is the inherited state.
    let custom_pcf_in_use = match opptype {
        Some(o) => o.custom_pcf,
        None => inherited.custom_pcf,
    };
    let etr = if custom_pcf_in_use {
        Some(
            reader
                .read_u32(ETR_BITS)
                .map_err(|_| Error::UnexpectedEof)? as u8,
        )
    } else {
        None
    };

    // §5.1.9 — UUI present iff UMV on and UFEP == 001.
    let umv_on = opptype.map(|o| o.umv).unwrap_or(false);
    let uui = if umv_on {
        Some(parse_uui(reader)?)
    } else {
        None
    };

    // §5.1.10 — SSS present iff Slice Structured on and UFEP == 001.
    let ss_on = opptype.map(|o| o.slice_structured).unwrap_or(false);
    let sss = if ss_on {
        let raw = reader
            .read_u32(SSS_BITS)
            .map_err(|_| Error::UnexpectedEof)?;
        Some(SliceStructuredSubmode {
            rectangular: (raw >> 1) & 1 == 1,
            arbitrary_order: raw & 1 == 1,
        })
    } else {
        None
    };

    // §5.1.11–§5.1.18 — the scalability / RPS / RPR sub-bitstreams that
    // follow SSS carry variable-length and externally-negotiated layout
    // (Annexes N, O, P) which is not staged for parsing here. Slice
    // structuring (SSS) itself is fully parsed above; the refusals below
    // cover only the layered fields. Refuse rather than mis-frame the
    // remaining header.
    let rps_on = opptype
        .map(|o| o.reference_picture_selection)
        .unwrap_or(false);
    if rps_on {
        return Err(Error::PlusPtypeUnsupported);
    }
    if mpptype.reference_picture_resampling {
        return Err(Error::PlusPtypeUnsupported);
    }
    // EI / EP / B picture types are the scalability layers of Annex O,
    // whose ELNUM / RLNUM fields follow here; their layered decode is
    // out of scope for this header parser.
    if matches!(
        mpptype.picture_type,
        PlusPictureType::BPicture | PlusPictureType::EiPicture | PlusPictureType::EpPicture
    ) {
        return Err(Error::PlusPtypeUnsupported);
    }

    Ok(PlusPtypeHeader {
        ufep: ufep as u8,
        opptype,
        mpptype,
        cpm,
        psbi,
        cpfmt,
        epar,
        cpcfc,
        etr,
        uui,
        sss,
    })
}

/// Parse CPFMT (§5.1.5, 23 bits). Reader at CPFMT bit 1.
fn parse_cpfmt(reader: &mut BitReader<'_>) -> Result<CustomPictureFormat> {
    let raw = reader
        .read_u32(CPFMT_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    // Bits 1-4 PAR code, 5-13 PWI, 14 SCE guard "1", 15-23 PHI.
    let par_code = ((raw >> (CPFMT_BITS - 4)) & 0b1111) as u8;
    if par_code == 0 {
        // PAR code "0000" is forbidden (Table 5).
        return Err(Error::PlusPtypeReservedField);
    }
    let width_indication = ((raw >> (CPFMT_BITS - 13)) & 0x1FF) as u16;
    let sce_guard = (raw >> (CPFMT_BITS - 14)) & 1;
    if sce_guard != 1 {
        return Err(Error::PlusPtypeReservedField);
    }
    let height_indication = (raw & 0x1FF) as u16;
    if height_indication == 0 {
        // PHI range is [1, 288] (§5.1.5).
        return Err(Error::PlusPtypeReservedField);
    }
    Ok(CustomPictureFormat {
        par_code,
        width_indication,
        height_indication,
    })
}

/// Parse EPAR (§5.1.6, 16 bits). Reader at EPAR bit 1.
fn parse_epar(reader: &mut BitReader<'_>) -> Result<ExtendedPar> {
    let raw = reader
        .read_u32(EPAR_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    let width = ((raw >> 8) & 0xFF) as u8;
    let height = (raw & 0xFF) as u8;
    if width == 0 || height == 0 {
        // PAR width / height "0" is forbidden (§5.1.6).
        return Err(Error::PlusPtypeReservedField);
    }
    Ok(ExtendedPar { width, height })
}

/// Parse CPCFC (§5.1.7, 8 bits). Reader at CPCFC bit 1.
fn parse_cpcfc(reader: &mut BitReader<'_>) -> Result<CustomPcf> {
    let raw = reader
        .read_u32(CPCFC_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    let conversion_1001 = (raw >> 7) & 1 == 1;
    let divisor = (raw & 0x7F) as u8;
    if divisor == 0 {
        // Clock divisor "0" is forbidden (§5.1.7).
        return Err(Error::PlusPtypeReservedField);
    }
    Ok(CustomPcf {
        conversion_1001,
        divisor,
    })
}

/// Parse UUI (§5.1.9, 1 or 2 bits). Reader at UUI bit 1.
fn parse_uui(reader: &mut BitReader<'_>) -> Result<Uui> {
    // "1" -> Limited; "01" -> Unlimited.
    let first = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if first {
        return Ok(Uui::Limited);
    }
    let second = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if second {
        Ok(Uui::Unlimited)
    } else {
        // "00" is not a defined UUI codeword.
        Err(Error::PlusPtypeReservedField)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Write OPPTYPE (18 bits) given each field; the SCE guard (bit 15)
    /// is set and reserved bits 16-18 cleared so the result is a valid
    /// codeword unless the test overrides `src_bits`.
    #[allow(clippy::too_many_arguments)]
    fn write_opptype(
        w: &mut BitWriter,
        src_bits: u32,
        custom_pcf: bool,
        umv: bool,
        sac: bool,
        ap: bool,
        aic: bool,
        df: bool,
        ss: bool,
        rps: bool,
        isd: bool,
        aiv: bool,
        mq: bool,
    ) {
        w.write_u32(src_bits, 3);
        w.write_bit(custom_pcf);
        w.write_bit(umv);
        w.write_bit(sac);
        w.write_bit(ap);
        w.write_bit(aic);
        w.write_bit(df);
        w.write_bit(ss);
        w.write_bit(rps);
        w.write_bit(isd);
        w.write_bit(aiv);
        w.write_bit(mq);
        w.write_bit(true); // bit 15 SCE guard
        w.write_bit(false); // bit 16
        w.write_bit(false); // bit 17
        w.write_bit(false); // bit 18
    }

    /// Write MPPTYPE (9 bits): picture-type code + RPR + RRU + RTYPE,
    /// reserved bits 7-8 cleared, bit 9 SCE guard set.
    fn write_mpptype(w: &mut BitWriter, ptype_bits: u32, rpr: bool, rru: bool, rtype: bool) {
        w.write_u32(ptype_bits, 3);
        w.write_bit(rpr);
        w.write_bit(rru);
        w.write_bit(rtype);
        w.write_bit(false); // bit 7
        w.write_bit(false); // bit 8
        w.write_bit(true); // bit 9 SCE guard
    }

    fn parse(bytes: &[u8], inherited: InheritedExtendedState) -> Result<PlusPtypeHeader> {
        let mut r = BitReader::new(bytes);
        parse_plus_ptype(&mut r, inherited)
    }

    #[test]
    fn full_ufep_qcif_inter_minimal() {
        // UFEP=001, OPPTYPE QCIF all modes off, MPPTYPE P-picture,
        // CPM=0.
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b010, false, false, false, false, false, false, false, false, false, false,
            false,
        );
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(false); // CPM
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let hdr = parse(&w.finish(), InheritedExtendedState::default()).expect("parse");
        assert_eq!(hdr.ufep, 0b001);
        let op = hdr.opptype.expect("opptype present");
        assert_eq!(op.source_format, PlusSourceFormat::Qcif);
        assert!(!op.umv && !op.advanced_prediction && !op.modified_quantization);
        assert_eq!(hdr.mpptype.picture_type, PlusPictureType::Inter);
        assert!(!hdr.cpm);
        assert!(hdr.psbi.is_none());
        assert!(hdr.cpfmt.is_none());
        assert!(hdr.etr.is_none());
        assert!(hdr.uui.is_none());
        assert_eq!(hdr.source_format(), Some(PlusSourceFormat::Qcif));
    }

    #[test]
    fn full_ufep_all_optype_modes_on_intra() {
        // UFEP=001, OPPTYPE CIF with AP/AIC/DF/AIV/MQ on (no UMV/SS/RPS
        // to avoid follow-on fields / refusals), MPPTYPE I-picture.
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b011, false, false, false, true, true, true, false, false, false, true, true,
        );
        write_mpptype(&mut w, 0b000, false, false, false);
        w.write_bit(false); // CPM
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let hdr = parse(&w.finish(), InheritedExtendedState::default()).expect("parse");
        let op = hdr.opptype.unwrap();
        assert_eq!(op.source_format, PlusSourceFormat::Cif);
        assert!(op.advanced_prediction);
        assert!(op.advanced_intra);
        assert!(op.deblocking);
        assert!(op.alternative_inter_vlc);
        assert!(op.modified_quantization);
        assert!(!op.umv);
        assert_eq!(hdr.mpptype.picture_type, PlusPictureType::Intra);
    }

    #[test]
    fn cpm_set_pulls_psbi() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b010, false, false, false, false, false, false, false, false, false, false,
            false,
        );
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(true); // CPM
        w.write_u32(0b10, 2); // PSBI = 2
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let hdr = parse(&w.finish(), InheritedExtendedState::default()).expect("parse");
        assert!(hdr.cpm);
        assert_eq!(hdr.psbi, Some(2));
    }

    #[test]
    fn custom_format_pulls_cpfmt_and_epar() {
        // Custom source format -> CPFMT; PAR code 1111 (extended) ->
        // EPAR. PWI=87 -> width=(87+1)*4=352; PHI=72 -> height=288.
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w,
            OPPTYPE_SRCFMT_CUSTOM,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
        );
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(false); // CPM
                            // CPFMT: par=1111, PWI=87, guard=1, PHI=72.
        w.write_u32(PAR_CODE_EXTENDED, 4);
        w.write_u32(87, 9);
        w.write_bit(true); // SCE guard
        w.write_u32(72, 9);
        // EPAR: width=64, height=45.
        w.write_u32(64, 8);
        w.write_u32(45, 8);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let hdr = parse(&w.finish(), InheritedExtendedState::default()).expect("parse");
        let cp = hdr.cpfmt.expect("cpfmt");
        assert_eq!(cp.par_code, 0b1111);
        assert_eq!(cp.luma_width(), 352);
        assert_eq!(cp.luma_height(), 288);
        let ep = hdr.epar.expect("epar");
        assert_eq!(ep.width, 64);
        assert_eq!(ep.height, 45);
        assert_eq!(hdr.source_format(), Some(PlusSourceFormat::Custom));
    }

    #[test]
    fn custom_pcf_pulls_cpcfc_and_etr() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b010, true, // custom PCF
            false, false, false, false, false, false, false, false, false, false,
        );
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(false); // CPM
                            // CPCFC: conversion=1 (1001), divisor=30.
        w.write_bit(true);
        w.write_u32(30, 7);
        // ETR (2 bits) since custom PCF in use.
        w.write_u32(0b10, 2);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let hdr = parse(&w.finish(), InheritedExtendedState::default()).expect("parse");
        let c = hdr.cpcfc.expect("cpcfc");
        assert!(c.conversion_1001);
        assert_eq!(c.divisor, 30);
        assert_eq!(hdr.etr, Some(0b10));
    }

    #[test]
    fn umv_pulls_uui_limited_and_unlimited() {
        for (write_unlimited, expect) in [(false, Uui::Limited), (true, Uui::Unlimited)] {
            let mut w = BitWriter::new();
            w.write_u32(UFEP_FULL, 3);
            write_opptype(
                &mut w, 0b010, false, true, // UMV on
                false, false, false, false, false, false, false, false, false,
            );
            write_mpptype(&mut w, 0b001, false, false, false);
            w.write_bit(false); // CPM
            if write_unlimited {
                w.write_bit(false);
                w.write_bit(true); // "01"
            } else {
                w.write_bit(true); // "1"
            }
            while !w.is_byte_aligned() {
                w.write_bit(false);
            }
            let hdr = parse(&w.finish(), InheritedExtendedState::default()).expect("parse");
            assert_eq!(hdr.uui, Some(expect));
        }
    }

    #[test]
    fn ufep_zero_omits_opptype_and_uses_inherited_pcf() {
        // UFEP=000 -> no OPPTYPE. With inherited custom_pcf=true, ETR is
        // present (2 bits) immediately after CPM.
        let mut w = BitWriter::new();
        w.write_u32(UFEP_MANDATORY_ONLY, 3);
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(false); // CPM
        w.write_u32(0b01, 2); // ETR
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let inherited = InheritedExtendedState { custom_pcf: true };
        let hdr = parse(&w.finish(), inherited).expect("parse");
        assert_eq!(hdr.ufep, 0b000);
        assert!(hdr.opptype.is_none());
        assert_eq!(hdr.etr, Some(0b01));
        assert!(hdr.source_format().is_none());
        assert!(hdr.custom_pcf(inherited));
    }

    #[test]
    fn ufep_zero_no_inherited_pcf_omits_etr() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_MANDATORY_ONLY, 3);
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(false); // CPM
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let hdr = parse(&w.finish(), InheritedExtendedState::default()).expect("parse");
        assert!(hdr.etr.is_none());
        assert!(!hdr.custom_pcf(InheritedExtendedState::default()));
    }

    #[test]
    fn reserved_ufep_rejected() {
        let mut w = BitWriter::new();
        w.write_u32(0b010, 3); // reserved UFEP
        w.write_u32(0, 16);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        assert_eq!(
            parse(&w.finish(), InheritedExtendedState::default()).unwrap_err(),
            Error::PlusPtypeReservedField
        );
    }

    #[test]
    fn opptype_missing_sce_guard_rejected() {
        // OPPTYPE bit 15 must be "1"; clear it.
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        w.write_u32(0b010, 3); // QCIF
        for _ in 0..11 {
            w.write_bit(false); // bits 4-14
        }
        w.write_bit(false); // bit 15 (illegal)
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_u32(0, 16);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        assert_eq!(
            parse(&w.finish(), InheritedExtendedState::default()).unwrap_err(),
            Error::PlusPtypeReservedField
        );
    }

    #[test]
    fn mpptype_reserved_picture_type_rejected() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b010, false, false, false, false, false, false, false, false, false, false,
            false,
        );
        write_mpptype(&mut w, 0b110, false, false, false); // reserved type
        w.write_bit(false);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        assert_eq!(
            parse(&w.finish(), InheritedExtendedState::default()).unwrap_err(),
            Error::PlusPtypeReservedField
        );
    }

    #[test]
    fn rps_mode_is_unsupported() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b010, false, false, false, false, false, false, false, true, // RPS on
            false, false, false,
        );
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(false);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        assert_eq!(
            parse(&w.finish(), InheritedExtendedState::default()).unwrap_err(),
            Error::PlusPtypeUnsupported
        );
    }

    #[test]
    fn rpr_mode_is_unsupported() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b010, false, false, false, false, false, false, false, false, false, false,
            false,
        );
        write_mpptype(&mut w, 0b001, true, false, false); // RPR on
        w.write_bit(false);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        assert_eq!(
            parse(&w.finish(), InheritedExtendedState::default()).unwrap_err(),
            Error::PlusPtypeUnsupported
        );
    }

    #[test]
    fn b_picture_type_is_unsupported() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w, 0b010, false, false, false, false, false, false, false, false, false, false,
            false,
        );
        write_mpptype(&mut w, 0b011, false, false, false); // B-picture
        w.write_bit(false);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        assert_eq!(
            parse(&w.finish(), InheritedExtendedState::default()).unwrap_err(),
            Error::PlusPtypeUnsupported
        );
    }

    #[test]
    fn cpfmt_forbidden_par_code_rejected() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3);
        write_opptype(
            &mut w,
            OPPTYPE_SRCFMT_CUSTOM,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
        );
        write_mpptype(&mut w, 0b001, false, false, false);
        w.write_bit(false); // CPM
        w.write_u32(0b0000, 4); // forbidden PAR code
        w.write_u32(87, 9);
        w.write_bit(true);
        w.write_u32(72, 9);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        assert_eq!(
            parse(&w.finish(), InheritedExtendedState::default()).unwrap_err(),
            Error::PlusPtypeReservedField
        );
    }

    #[test]
    fn short_buffer_is_unexpected_eof() {
        let mut w = BitWriter::new();
        w.write_u32(UFEP_FULL, 3); // promises OPPTYPE but nothing follows
        let bytes = w.finish();
        assert_eq!(
            parse(&bytes, InheritedExtendedState::default()).unwrap_err(),
            Error::UnexpectedEof
        );
    }
}
