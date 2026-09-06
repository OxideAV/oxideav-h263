//! H.263 macroblock-layer parsing (§5.3).
//!
//! This module implements the structural decode of the macroblock-layer
//! header as defined in ITU-T Recommendation H.263 (01/2005) §5.3.
//! Per Figure 10/H.263 the on-wire fields, in order, are:
//!
//! ```text
//!   COD   MCBPC   MODB   CBPB   CBPY   DQUANT   MVD   MVD2-4   MVDB   Block Data
//! ```
//!
//! Round 3 (post orphan-rebuild) decodes the **non-PB-frame baseline**
//! subset of those fields:
//!
//! * §5.3.1 — **COD** (1 bit). Present only when the enclosing picture
//!   is *not* of type INTRA. `0` means the macroblock is coded; `1`
//!   means "skipped" (decoder treats it as an INTER MB with zero
//!   motion vector and no residual).
//! * §5.3.2 — **MCBPC** (variable length, Table 7 for I-pictures,
//!   Table 8 for P-pictures). Combines the macroblock type
//!   (`0..=5` plus a stuffing code) with `CBPC` — the two-bit
//!   coded-block pattern for the U / V chrominance blocks.
//! * §5.3.5 — **CBPY** (variable length, Table 12). Four-bit
//!   coded-block pattern for the four luminance blocks; the table's
//!   `CBPY(INTRA)` column is the natural-binary value, and the
//!   `CBPY(INTER)` column is its bitwise complement. The parser
//!   returns the natural-binary `CBPY(INTRA)` pattern; callers
//!   complement when the surrounding macroblock is INTER per the
//!   per-MB-type table.
//! * §5.3.6 — **DQUANT** (2 bits, baseline form). Differential
//!   adjustment to QUANT per Table 13. Only present for MB types
//!   `1`, `4`, `5`. The Modified Quantization mode (Annex T)
//!   replaces this with a variable-length code; round 3 does not
//!   handle the Annex-T form.
//! * §5.3.7 — **MVD** (variable length). The horizontal component is
//!   decoded, then the vertical component, each reported in
//!   **half-pel units** as a signed integer. The default form is the
//!   Table 14 VLC (`[-32, +31]`, spec value `× 2`); when the
//!   Unrestricted Motion Vector mode is used with PLUSPTYPE present
//!   ([`MbContext::umv_table_d3`]) each pair is instead two §D.2 /
//!   Table D.3 reversible codewords (`[-4095, +4095]`) with the
//!   six-zero emulation-prevention rule. For MB types that carry MVD
//!   only (`0`, `1`, `3` in PB-INTRA), one `Mvd` is returned. The
//!   Advanced Prediction / Deblocking-Filter MVD2-4 follow-on vectors
//!   (§5.3.8) are decoded when the MB type is `2` or `5`.
//!
//! ## PB-frames mode (Annex G)
//!
//! When [`MbContext::pb_frames`] is set (PTYPE bit 13), the parser
//! additionally consumes the §5.3 Table 10 / Figure 10 PB-frame
//! fields: MODB (§5.3.3, Table 11) for every coded non-stuffing
//! macroblock, CBPB (§5.3.4) when MODB indicates it, MVD also for
//! INTRA macroblock types 3 / 4 (§5.3.7: "in PB-frames mode also
//! for INTRA macroblocks" — the vector is used for B-block
//! prediction only, §G.2), and MVDB (§5.3.9) after MVD2-4 when MODB
//! indicates it. The Annex M Improved-PB MODB form (Table M.1) is a
//! separate parser ([`crate::pb_layer::parse_modb_annex_m`]) not
//! gated by this flag.
//!
//! ## Deliberately deferred
//!
//! * Annex-T variable-length DQUANT.
//! * Annex-O B/EI/EP picture macroblocks.
//! * Block Data (§5.4) and everything after the macroblock header.
//!
//! ## Composing with picture / GOB layers
//!
//! [`parse_macroblock`] takes a [`MbContext`] describing the
//! enclosing picture's coding type, advanced-prediction-mode flag,
//! and the current QUANT. The caller threads those through from
//! the picture-layer header (`H263PictureHeader::coding_type`,
//! `advanced_prediction`) and the GOB-layer header (`GobLayer::quantiser`).
//! The parser returns an [`H263Macroblock`] describing what it
//! consumed, and the reader is left positioned at the start of
//! the macroblock's block-data region (which is out of scope for
//! round 3).

// All numeric literals in this module are bit-pattern transcriptions of
// the ITU-T H.263 spec's MSB-first VLC tables (Tables 7, 8, 12, 14).
// We deliberately group digits to mirror the spec's printed layout
// (e.g. "0000 0000 0010 1" -> `0b0_0000_0000_0010_1`) rather than the
// power-of-two grouping clippy prefers; that grouping is what makes the
// lines auditable against the spec.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::aic::{decode_intra_mode, IntraMode};
use crate::pb_layer::{
    parse_cbpb, parse_modb, parse_modb_annex_m, parse_mvdb, BpbCodingMode, ModbAnnexM, ModbPresence,
};
use crate::{Error, H263PictureCodingType, Result};

/// Picture-level context the macroblock parser needs.
///
/// Threaded from the picture-layer header and the current GOB-layer
/// header. The macroblock parser must know:
///
/// * the picture coding type (selects MCBPC table; controls COD
///   presence);
/// * whether Advanced Prediction mode is signalled in PTYPE
///   (selects whether MVD2-4 follow the primary MVD for INTER4V
///   macroblocks);
/// * the current QUANT (only used to thread back through
///   [`H263Macroblock::quantiser_after`] — the parser does not
///   re-emit it raw).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MbContext {
    /// Enclosing picture's coding type. Selects Table 7 (INTRA →
    /// I-picture) or Table 8 (INTER → P-picture) for MCBPC and
    /// gates COD presence.
    pub picture_coding_type: H263PictureCodingType,
    /// PTYPE bit 12 — Advanced Prediction mode (Annex F). When
    /// `true`, INTER4V macroblocks (MB type 2) carry MVD2-4 after
    /// the primary MVD (§5.3.8).
    pub advanced_prediction: bool,
    /// PLUSPTYPE OPPTYPE bit 11 — Deblocking Filter mode (Annex J).
    /// Per §5.3.8 / Table J.1, Deblocking Filter mode "includes the
    /// ability to use four motion vectors per macroblock" without the
    /// OBMC element, so MVD2-4 follow the primary MVD for INTER4V
    /// macroblocks under DF mode just as they do under Advanced
    /// Prediction. Independent of [`MbContext::advanced_prediction`]
    /// (Table J.1 lists DF-only and AP-only rows that both enable four
    /// vectors).
    pub deblocking_filter: bool,
    /// Annex I §I.2 — Advanced INTRA Coding mode. When `true`, an
    /// `INTRA_MODE` VLC (Table I.1) is read between MCBPC and CBPY
    /// for every INTRA macroblock (MB type 3 or 4). The decoded
    /// mode is surfaced on [`H263Macroblock::intra_mode`].
    pub aic_intra_mode: bool,
    /// PTYPE bit 13 — PB-frames mode (Annex G). When `true`, the
    /// §5.3 / Figure 10 / Table 10 PB-frame fields are parsed: MODB
    /// (§5.3.3) after MCBPC for every coded non-stuffing macroblock,
    /// CBPB (§5.3.4) between MODB and CBPY when MODB indicates it,
    /// MVD (§5.3.7) also for INTRA macroblock types 3 / 4 ("in
    /// PB-frames mode also for INTRA macroblocks"), and MVDB
    /// (§5.3.9) after MVD2-4 when MODB indicates it. This is the
    /// Annex G form (Table 11 MODB); the Annex M Improved-PB form
    /// (Table M.1) is a separate parser
    /// ([`crate::pb_layer::parse_modb_annex_m`]) selected by
    /// [`MbContext::pb_annex_m`].
    pub pb_frames: bool,
    /// PLUSPTYPE picture-type `"010"` — Improved PB-frames mode
    /// (Annex M). When `true` (which requires [`MbContext::pb_frames`]
    /// also set, since the §5.3 layer fields are shared), MODB is the
    /// §M.4 / Table M.1 6-entry form parsed by
    /// [`crate::pb_layer::parse_modb_annex_m`] and surfaced on
    /// [`H263Macroblock::annex_m_modb`]; CBPB / MVDB presence is gated
    /// by [`ModbAnnexM::has_cbpb`] / [`ModbAnnexM::has_mvdb`] instead
    /// of the Table 11 [`ModbPresence`] accessors. When `false` the
    /// Annex G Table 11 form is used.
    pub pb_annex_m: bool,
    /// Current QUANT from the most recent GOB-layer header (or
    /// the picture-layer's PQUANT in the no-GOB case). Used to
    /// compute [`H263Macroblock::quantiser_after`] after any
    /// DQUANT differential.
    pub quantiser_before: u8,
    /// PLUSPTYPE OPPTYPE bit 14 — Modified Quantization mode
    /// (Annex T). When `true`, the §5.3.6 DQUANT field is the §T.2
    /// variable-length form (two- or six-bit, parsed by
    /// [`crate::annex_t::parse_modified_dquant`]) instead of the
    /// baseline 2-bit Table 13 differential. The resulting
    /// [`H263Macroblock::quantiser_after`] is the §T.2 / Table T.1 /
    /// §T.2.2 new QUANT directly; [`H263Macroblock::dquant`] carries
    /// the signed change (`new − prior`) for callers that track the
    /// differential. The §T.3 chrominance `QUANT_C` derivation
    /// ([`crate::annex_t::quant_c_from_quant`]) is applied by the
    /// dequant stage, not the parser.
    pub modified_quant: bool,
    /// §5.3.7 / §D.2 — Unrestricted Motion Vector mode with
    /// **PLUSPTYPE present**: "motion vectors are coded using
    /// Table D.3 instead of Table 14". When `true`, every motion
    /// vector difference pair (MVD, each of MVD2-4, MVDB) is read as
    /// two Table D.3 reversible codewords (horizontal then vertical),
    /// and per §D.2 a pair equal to `(+0.5, +0.5)` — six consecutive
    /// zero bits on the wire — is followed by one emulation-prevention
    /// bit that shall be `"1"` (the zero-difference codeword). When
    /// `false`, MVDs are the §5.3.7 Table 14 codewords (both the
    /// default prediction mode and the PLUSPTYPE-absent Annex D form —
    /// those differ only in reconstruction, not parsing).
    pub umv_table_d3: bool,
}

/// Macroblock type from MCBPC (§5.3.2, Tables 7-9).
///
/// The numeric values are the spec's "MB type" column. Stuffing
/// is a control code, not a real macroblock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MbType {
    /// Type 0 — INTER. P-pictures only.
    Inter,
    /// Type 1 — INTER+Q. P-pictures only.
    InterQ,
    /// Type 2 — INTER4V (four 8×8 motion vectors). P-pictures.
    Inter4V,
    /// Type 3 — INTRA. I-pictures and P-pictures.
    Intra,
    /// Type 4 — INTRA+Q. I-pictures and P-pictures.
    IntraQ,
    /// Type 5 — INTER4V+Q. P-pictures only, and only when
    /// PLUSPTYPE is present and Advanced Prediction or Deblocking
    /// Filter mode is in use. Round 3 surfaces the code-point but
    /// rejects it because PLUSPTYPE is not yet decoded.
    Inter4VQ,
    /// MCBPC "stuffing" code (Tables 7/8). The macroblock header
    /// terminates here; the caller should not advance macroblock
    /// counters.
    Stuffing,
}

impl MbType {
    /// Whether DQUANT (§5.3.6) follows MCBPC for this type.
    /// Per Table 9: types 1 (INTER+Q), 4 (INTRA+Q), 5 (INTER4V+Q).
    pub fn has_dquant(self) -> bool {
        matches!(self, MbType::InterQ | MbType::IntraQ | MbType::Inter4VQ)
    }

    /// Whether a primary MVD (§5.3.7) follows for this type.
    /// Per Table 9 (P-picture rows): types 0, 1, 2, 5 in INTER
    /// pictures. INTRA MBs carry MVD only inside PB-frames mode,
    /// which round 3 does not handle.
    pub fn has_mvd(self) -> bool {
        matches!(
            self,
            MbType::Inter | MbType::InterQ | MbType::Inter4V | MbType::Inter4VQ
        )
    }

    /// Whether MVD2-4 (§5.3.8) follows the primary MVD, given the
    /// picture-level Advanced Prediction and Deblocking Filter flags.
    /// Per §5.3.8 / Table J.1: the four-vector elements (and therefore
    /// MVD2-4) are present for INTER4V types (2 / 5) when **either**
    /// Advanced Prediction mode (Annex F) **or** Deblocking Filter mode
    /// (Annex J, PLUSPTYPE OPPTYPE bit 11) is active — DF mode "includes
    /// the ability to use four motion vectors per macroblock" without the
    /// OBMC element.
    pub fn has_mvd2_4(self, advanced_prediction: bool, deblocking_filter: bool) -> bool {
        (advanced_prediction || deblocking_filter)
            && matches!(self, MbType::Inter4V | MbType::Inter4VQ)
    }

    /// Whether the macroblock is "INTRA-coded" for CBPY-complement
    /// purposes (§5.3.5: INTER macroblocks use the complement of
    /// the natural-binary CBPY pattern).
    pub fn is_intra(self) -> bool {
        matches!(self, MbType::Intra | MbType::IntraQ)
    }
}

/// Decoded primary or secondary motion vector difference.
///
/// Components are in **half-pel units**: the spec's "Vector
/// Differences" column scaled by 2 to keep the type integral.
/// The Table 14 form spans `[-32, +31]` (i.e. spec range
/// `-16 .. +15.5` in half-pel quanta of `0.5`); the Table D.3
/// reversible form read when the Unrestricted Motion Vector mode is
/// used with PLUSPTYPE present (§5.3.7 / §D.2,
/// [`MbContext::umv_table_d3`]) spans `[-4095, +4095]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mvd {
    /// Horizontal component, half-pel units, signed.
    pub dx_half: i16,
    /// Vertical component, half-pel units, signed.
    pub dy_half: i16,
}

/// Parsed H.263 macroblock-layer header (round-3 baseline subset).
///
/// See module-level docs for what fields are populated under which
/// MB types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct H263Macroblock {
    /// `false` if the macroblock is "skipped" (COD = 1 in the
    /// spec's wording, but inverted here so that the natural Rust
    /// convention "true = present" applies). I-picture MBs are
    /// always coded; the parser sets this to `true` for them.
    pub coded: bool,
    /// MCBPC macroblock type. `None` only when `coded == false`
    /// (the skip path).
    pub mb_type: Option<MbType>,
    /// MCBPC 2-bit chrominance coded-block pattern: bit 1
    /// (`0b10`) is CBPC5 (Cb), bit 0 (`0b01`) is CBPC6 (Cr).
    /// `None` when `coded == false` or when `mb_type` is Stuffing.
    pub cbpc: Option<u8>,
    /// CBPY 4-bit luminance coded-block pattern in the spec's
    /// **CBPY(INTRA)** orientation — bit 3 is block 1, bit 0 is
    /// block 4 of Figure 5. INTER macroblocks have the actual
    /// coded pattern equal to `cbpy ^ 0b1111`. `None` when MCBPC
    /// is Stuffing or the macroblock is skipped.
    pub cbpy: Option<u8>,
    /// DQUANT differential value in `{-2, -1, +1, +2}`. `None`
    /// when DQUANT is not signalled for this MB type.
    pub dquant: Option<i8>,
    /// QUANT after applying any DQUANT, clipped to `1..=31` per
    /// §5.3.6. Equal to `context.quantiser_before` when no DQUANT
    /// is signalled.
    pub quantiser_after: u8,
    /// Primary motion vector difference (§5.3.7). `None` when the
    /// MB type does not carry MVD.
    pub mvd: Option<Mvd>,
    /// MVD2..MVD4 (§5.3.8) when Advanced Prediction mode is in
    /// use and the MB type carries them (INTER4V / INTER4V+Q).
    /// Always all-or-nothing — either three entries or empty.
    pub mvd234: [Option<Mvd>; 3],
    /// Annex I §I.2 — `INTRA_MODE` from Table I.1. `Some` iff
    /// [`MbContext::aic_intra_mode`] is set AND the macroblock is
    /// INTRA-coded (MCBPC type 3 or 4); the same mode applies to
    /// every block of the macroblock. `None` for all non-AIC paths
    /// and for INTER macroblocks in AIC pictures.
    pub intra_mode: Option<IntraMode>,
    /// §5.3.3 MODB (Table 11). `Some` iff
    /// [`MbContext::pb_frames`] is set, [`MbContext::pb_annex_m`] is
    /// clear (Annex G form), and the macroblock is coded and not
    /// stuffing (per Table 10, the stuffing row carries COD + MCBPC
    /// only). `None` for all non-PB paths and for Annex M pictures
    /// (whose MODB is surfaced on [`H263Macroblock::annex_m_modb`]).
    pub modb: Option<ModbPresence>,
    /// §M.4 / Table M.1 MODB for an Improved PB-frame. `Some` iff
    /// both [`MbContext::pb_frames`] and [`MbContext::pb_annex_m`] are
    /// set and the macroblock is coded and not stuffing. Carries the
    /// §M.2 coding mode (bidirectional / forward / backward) plus the
    /// CBPB / MVDB presence flags. `None` for all non-Annex-M paths.
    pub annex_m_modb: Option<ModbAnnexM>,
    /// §5.3.4 CBPB — the 6-bit B-block coded-block pattern, bit 5
    /// = B-block 1 … bit 0 = B-block 6 (Figure 5 numbering).
    /// `Some` iff MODB indicated CBPB presence.
    pub cbpb: Option<u8>,
    /// §5.3.9 MVDB — the B-macroblock motion-vector delta pair, in
    /// half-pel units like [`H263Macroblock::mvd`]. `Some` iff MODB
    /// indicated MVDB presence.
    pub mvdb: Option<Mvd>,
}

/// Parse an H.263 macroblock header starting at the current
/// position of `reader`. See the module-level docs and
/// [`MbContext`] for what the parser expects and what it returns.
///
/// On success the reader is left at the first bit of the
/// macroblock's block-data region (which is out of scope for
/// round 3). On error the reader's position is unspecified.
pub fn parse_macroblock(reader: &mut BitReader<'_>, ctx: MbContext) -> Result<H263Macroblock> {
    if ctx.quantiser_before == 0 || ctx.quantiser_before > 31 {
        return Err(Error::InvalidQuantiser);
    }

    // §5.3.1 — COD (1 bit), only in non-INTRA pictures.
    let coded = match ctx.picture_coding_type {
        H263PictureCodingType::Intra => true,
        H263PictureCodingType::Inter => {
            let cod = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            // COD = 0 => coded, COD = 1 => skipped.
            !cod
        }
    };

    if !coded {
        // Skipped macroblock: nothing else on the wire for this MB.
        // In PB-frames mode too — Table 10's "Not coded" row carries
        // COD only (the B-part of a skipped PB-macroblock is
        // predicted with zero vectors and no residual, Annex G).
        return Ok(H263Macroblock {
            coded: false,
            mb_type: None,
            cbpc: None,
            cbpy: None,
            dquant: None,
            quantiser_after: ctx.quantiser_before,
            mvd: None,
            mvd234: [None; 3],
            intra_mode: None,
            modb: None,
            annex_m_modb: None,
            cbpb: None,
            mvdb: None,
        });
    }

    // §5.3.2 — MCBPC (variable length, Table 7 vs Table 8).
    let (mb_type, cbpc) = decode_mcbpc(reader, ctx.picture_coding_type)?;

    if matches!(mb_type, MbType::Stuffing) {
        // Per §5.3.2: "When MCBPC = Stuffing, the remaining part
        // of the macroblock layer is skipped." Surface the type
        // so callers know not to advance the MB counter, but
        // populate nothing else. The Table 10 stuffing row carries
        // COD + MCBPC only — no MODB even in PB-frames mode.
        return Ok(H263Macroblock {
            coded: true,
            mb_type: Some(MbType::Stuffing),
            cbpc: None,
            cbpy: None,
            dquant: None,
            quantiser_after: ctx.quantiser_before,
            mvd: None,
            mvd234: [None; 3],
            intra_mode: None,
            modb: None,
            annex_m_modb: None,
            cbpb: None,
            mvdb: None,
        });
    }

    // §I.2 — INTRA_MODE field (Table I.1) lives between MCBPC and
    // CBPY for INTRA macroblocks when Advanced INTRA Coding is on.
    // Outside AIC, or for INTER macroblocks in an AIC picture, no
    // bits are read here.
    let intra_mode = if ctx.aic_intra_mode && mb_type.is_intra() {
        Some(decode_intra_mode(reader)?)
    } else {
        None
    };

    // §5.3.3 — MODB (Table 11), between MCBPC and CBPY per
    // Figure 10. Present for every coded non-stuffing macroblock in
    // PB-frames mode ("MODB is present for MB-type 0-4 if PTYPE
    // indicates 'PB-frame'"; type 5 cannot occur under Annex G
    // because it requires PLUSPTYPE, which §G.1 bars — Table 10
    // nonetheless lists MODB for it, so no type gate is needed).
    // Annex M (Improved PB-frames) replaces Table 11 with the §M.4
    // Table M.1 6-entry form; otherwise the Annex G Table 11 form is
    // read. Only one of the two MODB fields is ever populated.
    let (modb, annex_m_modb) = if ctx.pb_frames {
        if ctx.pb_annex_m {
            (None, Some(parse_modb_annex_m(reader)?))
        } else {
            (Some(parse_modb(reader)?), None)
        }
    } else {
        (None, None)
    };

    // §5.3.4 — CBPB (6-bit FLC), between MODB and CBPY per
    // Figure 10, only when MODB indicates it. Annex G consults
    // Table 11 ([`ModbPresence::has_cbpb`]); Annex M consults Table M.1
    // ([`ModbAnnexM::has_cbpb`]).
    let cbpb = if modb.is_some_and(|m| m.has_cbpb()) || annex_m_modb.is_some_and(|m| m.has_cbpb()) {
        Some(parse_cbpb(reader)?)
    } else {
        None
    };

    // §5.3.5 — CBPY (variable length, Table 12).
    let cbpy = decode_cbpy(reader)?;

    // §5.3.6 — DQUANT. Two forms: the baseline 2-bit Table 13
    // differential, or — when Modified Quantization mode (Annex T)
    // is in use — the §T.2 variable-length form (two- or six-bit)
    // parsed by [`crate::annex_t::parse_modified_dquant`].
    let (dquant, quantiser_after) = if mb_type.has_dquant() {
        if ctx.modified_quant {
            // §T.2 — the field directly yields the new QUANT; carry
            // the signed change (new − prior) on `dquant` for
            // differential-tracking callers.
            let md = crate::annex_t::parse_modified_dquant(reader, ctx.quantiser_before)?;
            let diff = md.new_quant as i16 - ctx.quantiser_before as i16;
            (Some(diff as i8), md.new_quant)
        } else {
            let raw = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)?;
            let diff = match raw {
                0b00 => -1,
                0b01 => -2,
                0b10 => 1,
                0b11 => 2,
                _ => unreachable!("read_u32(2) <= 3"),
            };
            let next = (ctx.quantiser_before as i16 + diff as i16).clamp(1, 31) as u8;
            (Some(diff), next)
        }
    } else {
        (None, ctx.quantiser_before)
    };

    // §5.3.7 — MVD (variable length). Horizontal first, then
    // vertical: Table 14 codewords, or the Table D.3 reversible
    // codewords when the Unrestricted Motion Vector mode is used with
    // PLUSPTYPE present (§D.2, `ctx.umv_table_d3`). "MVD is included
    // for all INTER macroblocks (in PB-frames mode also for INTRA
    // macroblocks)" — the PB-mode INTRA vector is used only for
    // predicting B-blocks (§G.2).
    //
    // Annex M narrows the INTRA case: "in this mode (and only in this
    // mode) [bidirectional prediction], Motion Vector Data (MVD) of the
    // PB-macroblock must be included even if the P-macroblock is INTRA
    // coded" (§M.2.1) — a forward- or backward-mode INTRA macroblock
    // carries no MVD.
    let intra_pb_mvd = ctx.pb_frames
        && mb_type.is_intra()
        && !annex_m_modb.is_some_and(|m| !matches!(m.coding_mode(), BpbCodingMode::Bidirectional));
    let mvd = if mb_type.has_mvd() || intra_pb_mvd {
        Some(read_mvd_pair(reader, ctx.umv_table_d3)?)
    } else {
        None
    };

    // §5.3.8 — MVD2-4 (Advanced Prediction). "The codewords MVD2-4
    // are never used for INTRA" (§G.2) — has_mvd2_4 is false for
    // INTRA types, so no PB-mode adjustment is needed here.
    let mut mvd234 = [None; 3];
    if mb_type.has_mvd2_4(ctx.advanced_prediction, ctx.deblocking_filter) {
        for slot in mvd234.iter_mut() {
            *slot = Some(read_mvd_pair(reader, ctx.umv_table_d3)?);
        }
    }

    // §5.3.9 — MVDB, last header field per Figure 10, only when
    // MODB indicates it. Under Annex M MVDB is a forward motion
    // vector (§M.2.2) rather than the Annex G bidirectional-vector
    // enhancement, but its on-wire form is identical (two MVD
    // codewords); the §M.2.2 interpretation is applied by the
    // decoder, not the parser. §D.2 — with PLUSPTYPE present, UMV
    // mode switches MVDB to Table D.3 like every other MV pair.
    let mvdb = if modb.is_some_and(|m| m.has_mvdb()) || annex_m_modb.is_some_and(|m| m.has_mvdb()) {
        if ctx.umv_table_d3 {
            Some(read_mvd_pair(reader, true)?)
        } else {
            Some(parse_mvdb(reader)?)
        }
    } else {
        None
    };

    Ok(H263Macroblock {
        coded: true,
        mb_type: Some(mb_type),
        cbpc: Some(cbpc),
        cbpy: Some(cbpy),
        dquant,
        quantiser_after,
        mvd,
        mvd234,
        intra_mode,
        modb,
        annex_m_modb,
        cbpb,
        mvdb,
    })
}

/// Decode the MCBPC VLC per §5.3.2 / Tables 7 (I-picture) and 8
/// (P-picture). Returns `(MbType, CBPC)` where CBPC is the 2-bit
/// chrominance pattern (`0b10` = CBPC5/Cb, `0b01` = CBPC6/Cr).
///
/// MCBPC codes are all of the form *n leading zeros* + terminating
/// `1` + optional fixed-length suffix. We use [`BitReader::read_unary`]
/// to consume the run + the `1`, then dispatch on the zero-count
/// and read any suffix bits per the per-table layout below.
/// Test-only re-export of the private MCBPC decoder so the encoder's
/// VLC round-trip tests can verify `write_mcbpc_*` against the decode
/// side directly (without fabricating a whole macroblock layer).
#[cfg(test)]
pub(crate) fn decode_mcbpc_for_test(
    reader: &mut BitReader<'_>,
    picture: H263PictureCodingType,
) -> Result<(MbType, u8)> {
    decode_mcbpc(reader, picture)
}

fn decode_mcbpc(
    reader: &mut BitReader<'_>,
    picture: H263PictureCodingType,
) -> Result<(MbType, u8)> {
    let lz = reader.read_unary().map_err(|_| Error::UnexpectedEof)?;
    match picture {
        H263PictureCodingType::Intra => decode_mcbpc_i(reader, lz),
        H263PictureCodingType::Inter => decode_mcbpc_p(reader, lz),
    }
}

/// Table 7 — I-picture MCBPC. Codes, grouped by leading-zero count:
///
/// | code            | idx | mb type | cbpc | lz | suffix |
/// |-----------------|-----|---------|------|----|--------|
/// | `1`             |  0  | INTRA   | 00   | 0  | -      |
/// | `010`           |  2  | INTRA   | 10   | 1  | `0`    |
/// | `011`           |  3  | INTRA   | 11   | 1  | `1`    |
/// | `001`           |  1  | INTRA   | 01   | 2  | -      |
/// | `0001`          |  4  | INTRA+Q | 00   | 3  | -      |
/// | `000010`        |  6  | INTRA+Q | 10   | 4  | `0`    |
/// | `000011`        |  7  | INTRA+Q | 11   | 4  | `1`    |
/// | `000001`        |  5  | INTRA+Q | 01   | 5  | -      |
/// | `0000 0000 1`   |  8  | Stuffing| -    | 8  | -      |
fn decode_mcbpc_i(reader: &mut BitReader<'_>, lz: u32) -> Result<(MbType, u8)> {
    Ok(match lz {
        0 => (MbType::Intra, 0b00),
        1 => {
            // suffix bit picks between "010" (cbpc 10) and "011" (cbpc 11).
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            (MbType::Intra, if b { 0b11 } else { 0b10 })
        }
        2 => {
            // "001" — only one code-point here: idx 1, type 3, cbpc 01.
            (MbType::Intra, 0b01)
        }
        3 => {
            // "0001" — idx 4, type 4, cbpc 00.
            (MbType::IntraQ, 0b00)
        }
        4 => {
            // suffix bit picks between "000010" (cbpc 10) and
            // "000011" (cbpc 11). The terminating "1" of the
            // unary is the bit before this suffix.
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            (MbType::IntraQ, if b { 0b11 } else { 0b10 })
        }
        5 => {
            // "000001" — idx 5, type 4, cbpc 01.
            (MbType::IntraQ, 0b01)
        }
        8 => (MbType::Stuffing, 0),
        _ => return Err(Error::BadMcbpcCode),
    })
}

/// Table 8 — P-picture MCBPC. Codes, grouped by leading-zero count
/// (zeros precede the terminating `1` that `read_unary` consumes):
///
/// | code              | idx | mb type    | cbpc | lz | suffix |
/// |-------------------|-----|------------|------|----|--------|
/// | `1`               |  0  | INTER      | 00   | 0  | -      |
/// | `011`             |  4  | INTER+Q    | 00   | 1  | `1`    |
/// | `010`             |  8  | INTER4V    | 00   | 1  | `0`    |
/// | `0011`            |  1  | INTER      | 01   | 2  | `1`    |
/// | `0010`            |  2  | INTER      | 10   | 2  | `0`    |
/// | `00011`           | 12  | INTRA      | 00   | 3  | `1`    |
/// | `000100`          | 16  | INTRA+Q    | 00   | 3  | `00`   |
/// | `000101`          |  3  | INTER      | 11   | 3  | `01`   |
/// | `0000111`         |  5  | INTER+Q    | 01   | 4  | `11`   |
/// | `0000110`         |  6  | INTER+Q    | 10   | 4  | `10`   |
/// | `0000101`         |  9  | INTER4V    | 01   | 4  | `01`   |
/// | `0000100`         | 10  | INTER4V    | 10   | 4  | `00`   |
/// | `0000011`         | 15  | INTRA      | 11   | 5  | `1`    |
/// | `00000101`        | 11  | INTER4V    | 11   | 5  | `01`   |
/// | `00000100`        | 13  | INTRA      | 01   | 5  | `00`   |
/// | `00000011`        | 14  | INTRA      | 10   | 6  | `1`    |
/// | `000000101`       |  7  | INTER+Q    | 11   | 6  | `01`   |
/// | `000000100`       | 17  | INTRA+Q    | 01   | 6  | `00`   |
/// | `0000000011`      | 18  | INTRA+Q    | 10   | 7  | `1`    |
/// | `0000000010`      | 19  | INTRA+Q    | 11   | 7  | `0`    |
/// | `000000001`       | 20  | Stuffing   | -    | 8  | -      |
/// | `00000000010`     | 21  | INTER4V+Q  | 00   | 9  | `0`    |
/// | `0000000001100`   | 22  | INTER4V+Q  | 01   | 9  | `100`  |
/// | `0000000001110`   | 23  | INTER4V+Q  | 10   | 9  | `110`  |
/// | `0000000001111`   | 24  | INTER4V+Q  | 11   | 9  | `111`  |
///
/// The `lz=9` bucket disambiguates on a single bit first: `0`
/// selects idx 21 (1-bit suffix), `1` selects one of idx 22-24
/// (3-bit total suffix).
fn decode_mcbpc_p(reader: &mut BitReader<'_>, lz: u32) -> Result<(MbType, u8)> {
    Ok(match lz {
        0 => (MbType::Inter, 0b00),
        1 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                (MbType::InterQ, 0b00)
            } else {
                (MbType::Inter4V, 0b00)
            }
        }
        2 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                (MbType::Inter, 0b01)
            } else {
                (MbType::Inter, 0b10)
            }
        }
        3 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                (MbType::Intra, 0b00) // "00011" idx 12
            } else {
                let b2 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
                if b2 {
                    (MbType::Inter, 0b11) // "000101" idx 3
                } else {
                    (MbType::IntraQ, 0b00) // "000100" idx 16
                }
            }
        }
        4 => {
            let b0 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            let b1 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            match (b0, b1) {
                (true, true) => (MbType::InterQ, 0b01),    // "0000111" idx 5
                (true, false) => (MbType::InterQ, 0b10),   // "0000110" idx 6
                (false, true) => (MbType::Inter4V, 0b01),  // "0000101" idx 9
                (false, false) => (MbType::Inter4V, 0b10), // "0000100" idx 10
            }
        }
        5 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                (MbType::Intra, 0b11) // "0000011" idx 15
            } else {
                let b2 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
                if b2 {
                    (MbType::Inter4V, 0b11) // "00000101" idx 11
                } else {
                    (MbType::Intra, 0b01) // "00000100" idx 13
                }
            }
        }
        6 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                (MbType::Intra, 0b10) // "00000011" idx 14
            } else {
                let b2 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
                if b2 {
                    (MbType::InterQ, 0b11) // "000000101" idx 7
                } else {
                    (MbType::IntraQ, 0b01) // "000000100" idx 17
                }
            }
        }
        7 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                (MbType::IntraQ, 0b10) // "0000000011" idx 18
            } else {
                (MbType::IntraQ, 0b11) // "0000000010" idx 19
            }
        }
        8 => (MbType::Stuffing, 0),
        9 => {
            // Type-5 codes; valid only under PLUSPTYPE +
            // AdvPred/Deblock. Round 3 surfaces the code-points
            // and the caller's MbContext check will gate use.
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if !b {
                (MbType::Inter4VQ, 0b00) // "0000 0000 010" idx 21
            } else {
                let s = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)?;
                match s {
                    0b00 => (MbType::Inter4VQ, 0b01), // "...0110 0" idx 22
                    0b10 => (MbType::Inter4VQ, 0b10), // "...0111 0" idx 23
                    0b11 => (MbType::Inter4VQ, 0b11), // "...0111 1" idx 24
                    _ => return Err(Error::BadMcbpcCode),
                }
            }
        }
        _ => return Err(Error::BadMcbpcCode),
    })
}

/// Decode the CBPY VLC per §5.3.5 / Table 12. Returns the spec's
/// `CBPY(INTRA)` natural-binary 4-bit pattern; INTER macroblocks
/// take the bitwise complement at the caller.
///
/// Bit 3 of the returned value corresponds to luminance block 1,
/// bit 0 to block 4 (Figure 5).
///
/// Codes, grouped by leading-zero count (after `read_unary`
/// consumes the run + terminating `1`):
///
/// | code      | idx | pattern (INTRA) | lz | suffix |
/// |-----------|-----|-----------------|----|--------|
/// | `11`      | 15  | `1111`          | 0  | `1`    |
/// | `1000`    | 13  | `1101`          | 0  | `000`  |
/// | `1001`    |  3  | `0011`          | 0  | `001`  |
/// | `1010`    | 11  | `1011`          | 0  | `010`  |
/// | `1011`    |  7  | `0111`          | 0  | `011`  |
/// | `0100`    | 12  | `1100`          | 1  | `00`   |
/// | `0101`    | 10  | `1010`          | 1  | `01`   |
/// | `0110`    | 14  | `1110`          | 1  | `10`   |
/// | `0111`    |  5  | `0101`          | 1  | `11`   |
/// | `0011`    |  0  | `0000`          | 2  | `1`    |
/// | `00101`   |  1  | `0001`          | 2  | `01`   |
/// | `00100`   |  2  | `0010`          | 2  | `00`   |
/// | `00011`   |  4  | `0100`          | 3  | `1`    |
/// | `00010`   |  8  | `1000`          | 3  | `0`    |
/// | `000011`  |  9  | `1001`          | 4  | `1`    |
/// | `000010`  |  6  | `0110`          | 4  | `0`    |
pub(crate) fn decode_cbpy(reader: &mut BitReader<'_>) -> Result<u8> {
    let lz = reader.read_unary().map_err(|_| Error::UnexpectedEof)?;
    Ok(match lz {
        0 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                0b1111
            } else {
                let s = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)?;
                match s {
                    0b00 => 0b1101,
                    0b01 => 0b0011,
                    0b10 => 0b1011,
                    0b11 => 0b0111,
                    _ => unreachable!(),
                }
            }
        }
        1 => {
            let s = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)?;
            match s {
                0b00 => 0b1100,
                0b01 => 0b1010,
                0b10 => 0b1110,
                0b11 => 0b0101,
                _ => unreachable!(),
            }
        }
        2 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                0b0000
            } else {
                let b2 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
                if b2 {
                    0b0001
                } else {
                    0b0010
                }
            }
        }
        3 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                0b0100
            } else {
                0b1000
            }
        }
        4 => {
            let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
            if b {
                0b1001
            } else {
                0b0110
            }
        }
        _ => return Err(Error::BadCbpyCode),
    })
}

/// Decode a single MVD component per §5.3.7 / Table 14. The return
/// value is in **half-pel units** as a signed integer; legal range
/// is `[-32, +31]` corresponding to spec "Vector" `-16 .. +15.5`.
///
/// Table 14 maps 64 indices; each index has both a "Vector" and
/// "Differences" interpretation (the latter wraps around when the
/// reconstructed vector would fall outside the displacement range
/// — that selection is the *caller's* responsibility against the
/// motion-vector predictor). Round 3 returns the natural-range
/// half-pel `Vector` column; mapping to the wrap-around branch is
/// deferred to the (future) MV-reconstruction stage.
/// Read one motion-vector-difference **pair** (horizontal component
/// then vertical component, §5.3.7) in the entropy form the picture
/// header selects:
///
/// * `table_d3 == false` — two Table 14 codewords
///   ([`decode_mvd_component`]), each in `[-32, +31]` half-pel.
/// * `table_d3 == true` — §D.2, Unrestricted Motion Vector mode with
///   PLUSPTYPE present: two Table D.3 reversible codewords
///   ([`crate::annex_p::read_table_d3`]), each in `[-4095, +4095]`
///   half-pel. "If a pair equals (0.5, 0.5) six consecutive zeros
///   are produced. To prevent start code emulation, this occurrence
///   shall be followed by one bit set to '1'" — the pair `(+1, +1)`
///   (half-pel units, two `"000"` codewords) is followed by an
///   emulation-prevention bit that must read `"1"` (the
///   zero-difference codeword); a `"0"` there is a malformed stream.
pub(crate) fn read_mvd_pair(reader: &mut BitReader<'_>, table_d3: bool) -> Result<Mvd> {
    if !table_d3 {
        return Ok(Mvd {
            dx_half: decode_mvd_component(reader)? as i16,
            dy_half: decode_mvd_component(reader)? as i16,
        });
    }
    let dx = crate::annex_p::read_table_d3(reader)?;
    let dy = crate::annex_p::read_table_d3(reader)?;
    // §D.2 emulation prevention: the (+0.5, +0.5) pair — value +1 in
    // half-pel units for both components, codeword "000" twice — is
    // followed by one bit set to "1".
    if dx == 1 && dy == 1 {
        let epb = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        if !epb {
            return Err(Error::BadMvdCode);
        }
    }
    Ok(Mvd {
        dx_half: dx as i16,
        dy_half: dy as i16,
    })
}

/// §5.3.6 — read the baseline 2-bit DQUANT differential (Table 13) and
/// apply it to `quant_before`, clipping the result to `1..=31`. Shared
/// by the baseline driver and the Annex O scalability driver (the
/// scalability "+ Q" macroblock rows use the same §5.3.6 / Table 13
/// form per §O.4.5).
pub(crate) fn read_dquant_baseline(reader: &mut BitReader<'_>, quant_before: u8) -> Result<u8> {
    let raw = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)?;
    let diff: i16 = match raw {
        0b00 => -1,
        0b01 => -2,
        0b10 => 1,
        0b11 => 2,
        _ => unreachable!("read_u32(2) <= 3"),
    };
    Ok((quant_before as i16 + diff).clamp(1, 31) as u8)
}

pub(crate) fn decode_mvd_component(reader: &mut BitReader<'_>) -> Result<i8> {
    // 64-row literal transcription of Table 14, kept in
    // index-order so the row layout matches the printed spec
    // line-by-line. Decoder reads the codeword bits
    // incrementally (1..=13 bits) and matches against the
    // table at each step.
    const MVD_TABLE: &[(u8, u16, i8)] = &[
        // (bit_count, code_msb_left, half_pel_value)
        // idx 0..63 -> spec values -16, -15.5, -15, ... 0 ... +15.5
        (13, 0b0_0000_0000_0010_1, -32), // idx 0
        (13, 0b0_0000_0000_0011_1, -31), // idx 1
        (12, 0b0000_0000_0101, -30),     // idx 2
        (12, 0b0000_0000_0111, -29),     // idx 3
        (12, 0b0000_0000_1001, -28),     // idx 4
        (12, 0b0000_0000_1011, -27),     // idx 5
        (12, 0b0000_0000_1101, -26),     // idx 6
        (12, 0b0000_0000_1111, -25),     // idx 7
        (11, 0b0000_0001_001, -24),      // idx 8
        (11, 0b0000_0001_011, -23),      // idx 9
        (11, 0b0000_0001_101, -22),      // idx 10
        (11, 0b0000_0001_111, -21),      // idx 11
        (11, 0b0000_0010_001, -20),      // idx 12
        (11, 0b0000_0010_011, -19),      // idx 13
        (11, 0b0000_0010_101, -18),      // idx 14
        (11, 0b0000_0010_111, -17),      // idx 15
        (11, 0b0000_0011_001, -16),      // idx 16
        (11, 0b0000_0011_011, -15),      // idx 17
        (11, 0b0000_0011_101, -14),      // idx 18
        (11, 0b0000_0011_111, -13),      // idx 19
        (11, 0b0000_0100_001, -12),      // idx 20
        (11, 0b0000_0100_011, -11),      // idx 21
        (10, 0b0000_0100_11, -10),       // idx 22
        (10, 0b0000_0101_01, -9),        // idx 23
        (10, 0b0000_0101_11, -8),        // idx 24
        (8, 0b0000_0111, -7),            // idx 25
        (8, 0b0000_1001, -6),            // idx 26
        (8, 0b0000_1011, -5),            // idx 27
        (7, 0b000_0111, -4),             // idx 28
        (5, 0b0001_1, -3),               // idx 29
        (4, 0b0011, -2),                 // idx 30
        (3, 0b011, -1),                  // idx 31
        (1, 0b1, 0),                     // idx 32
        (3, 0b010, 1),                   // idx 33
        (4, 0b0010, 2),                  // idx 34
        (5, 0b0001_0, 3),                // idx 35
        (7, 0b000_0110, 4),              // idx 36
        (8, 0b0000_1010, 5),             // idx 37
        (8, 0b0000_1000, 6),             // idx 38
        (8, 0b0000_0110, 7),             // idx 39
        (10, 0b0000_0101_10, 8),         // idx 40
        (10, 0b0000_0101_00, 9),         // idx 41
        (10, 0b0000_0100_10, 10),        // idx 42
        (11, 0b0000_0100_010, 11),       // idx 43
        (11, 0b0000_0100_000, 12),       // idx 44
        (11, 0b0000_0011_110, 13),       // idx 45
        (11, 0b0000_0011_100, 14),       // idx 46
        (11, 0b0000_0011_010, 15),       // idx 47
        (11, 0b0000_0011_000, 16),       // idx 48
        (11, 0b0000_0010_110, 17),       // idx 49
        (11, 0b0000_0010_100, 18),       // idx 50
        (11, 0b0000_0010_010, 19),       // idx 51
        (11, 0b0000_0010_000, 20),       // idx 52
        (11, 0b0000_0001_110, 21),       // idx 53
        (11, 0b0000_0001_100, 22),       // idx 54
        (11, 0b0000_0001_010, 23),       // idx 55
        (11, 0b0000_0001_000, 24),       // idx 56
        (12, 0b0000_0000_1110, 25),      // idx 57
        (12, 0b0000_0000_1100, 26),      // idx 58
        (12, 0b0000_0000_1010, 27),      // idx 59
        (12, 0b0000_0000_1000, 28),      // idx 60
        (12, 0b0000_0000_0110, 29),      // idx 61
        (12, 0b0000_0000_0100, 30),      // idx 62
        (13, 0b0_0000_0000_0011_0, 31),  // idx 63
    ];

    // Read up to 13 bits one-by-one, building a running prefix.
    // After each read, scan MVD_TABLE for an entry whose
    // `bits` matches the current prefix-length AND whose `code`
    // matches the bits. If found, return its half-pel value.
    let mut acc: u32 = 0;
    let mut len: u8 = 0;
    while len < 13 {
        let b = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        acc = (acc << 1) | (b as u32);
        len += 1;
        for &(bits, code, half) in MVD_TABLE {
            if bits == len && (code as u32) == acc {
                return Ok(half);
            }
        }
    }
    Err(Error::BadMvdCode)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::H263PictureCodingType;
    use oxideav_core::bits::BitWriter;

    /// Build a synthetic bitstream containing exactly the macroblock
    /// header described by the arguments, then byte-align.
    fn intra_picture_ctx(q: u8) -> MbContext {
        MbContext {
            picture_coding_type: H263PictureCodingType::Intra,
            advanced_prediction: false,
            deblocking_filter: false,
            aic_intra_mode: false,
            pb_frames: false,
            pb_annex_m: false,
            quantiser_before: q,
            modified_quant: false,
            umv_table_d3: false,
        }
    }

    fn inter_picture_ctx(q: u8, advanced: bool) -> MbContext {
        MbContext {
            picture_coding_type: H263PictureCodingType::Inter,
            advanced_prediction: advanced,
            deblocking_filter: false,
            aic_intra_mode: false,
            pb_frames: false,
            pb_annex_m: false,
            quantiser_before: q,
            modified_quant: false,
            umv_table_d3: false,
        }
    }

    fn finish_aligned(mut w: BitWriter) -> Vec<u8> {
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    /// §5.3.7 / §D.2 — an INTER macroblock in a UMV + PLUSPTYPE
    /// picture carries its MVD as two Table D.3 reversible codewords;
    /// differences far outside the Table 14 window parse exactly.
    #[test]
    fn umv_plus_mvd_reads_table_d3() {
        for (dx, dy) in [
            (0i16, 0i16),
            (127, -128),
            (-255, 254),
            (63, -1),
            (-4095, 4095),
        ] {
            let mut w = BitWriter::new();
            w.write_bit(false); // COD
            w.write_bit(true); // MCBPC "1" (INTER, cbpc 00)
            w.write_u32(0b0011, 4); // CBPY pattern 0000
            crate::annex_p::write_table_d3(&mut w, dx as i32).unwrap();
            crate::annex_p::write_table_d3(&mut w, dy as i32).unwrap();
            if dx == 1 && dy == 1 {
                w.write_bit(true);
            }
            let bytes = finish_aligned(w);
            let mut r = BitReader::new(&bytes);
            let ctx = MbContext {
                umv_table_d3: true,
                ..inter_picture_ctx(8, false)
            };
            let mb = parse_macroblock(&mut r, ctx).expect("parse");
            assert_eq!(
                mb.mvd,
                Some(Mvd {
                    dx_half: dx,
                    dy_half: dy
                }),
                "({dx}, {dy})"
            );
        }
    }

    /// §D.2 — the (+0.5, +0.5) pair (six consecutive zeros) carries a
    /// mandatory emulation-prevention "1"; its absence is a malformed
    /// stream, and the bit is consumed so following fields stay
    /// aligned.
    #[test]
    fn umv_plus_mvd_pair_of_plus_ones_needs_emulation_prevention_bit() {
        let ctx = MbContext {
            umv_table_d3: true,
            ..inter_picture_ctx(8, false)
        };

        // Well-formed: EPB "1" after the pair; a Table-14-style trailing
        // pattern must still parse from the very next bit.
        let mut w = BitWriter::new();
        w.write_bit(false); // COD
        w.write_bit(true); // MCBPC "1"
        w.write_u32(0b0011, 4); // CBPY 0000
        w.write_u32(0b000_000, 6); // (+1, +1) pair
        w.write_bit(true); // EPB
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, ctx).expect("parse");
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: 1,
                dy_half: 1
            })
        );

        // Malformed: EPB "0".
        let mut w = BitWriter::new();
        w.write_bit(false);
        w.write_bit(true);
        w.write_u32(0b0011, 4);
        w.write_u32(0b000_000, 6);
        w.write_bit(false); // EPB must be "1"
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_macroblock(&mut r, ctx).unwrap_err(),
            Error::BadMvdCode
        );
    }

    /// §5.3.8 + §D.2 — MVD2-4 of an INTER4V macroblock also switch to
    /// Table D.3 when UMV is on with PLUSPTYPE present.
    #[test]
    fn umv_plus_mvd234_read_table_d3() {
        // MCBPC for INTER4V (type 2), cbpc 00 — Table 8 code "010".
        let mut w = BitWriter::new();
        w.write_bit(false); // COD
        w.write_u32(0b010, 3); // MCBPC INTER4V
        w.write_u32(0b0011, 4); // CBPY 0000
        let pairs = [(70i32, -3i32), (1, 1), (-100, 200), (0, -4095)];
        for &(dx, dy) in &pairs {
            crate::annex_p::write_table_d3(&mut w, dx).unwrap();
            crate::annex_p::write_table_d3(&mut w, dy).unwrap();
            if dx == 1 && dy == 1 {
                w.write_bit(true); // §D.2 EPB
            }
        }
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        let ctx = MbContext {
            umv_table_d3: true,
            ..inter_picture_ctx(8, true)
        };
        let mb = parse_macroblock(&mut r, ctx).expect("parse");
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: 70,
                dy_half: -3
            })
        );
        let got: Vec<(i16, i16)> = mb
            .mvd234
            .iter()
            .map(|m| {
                let m = m.expect("MVD2-4 present");
                (m.dx_half, m.dy_half)
            })
            .collect();
        assert_eq!(got, vec![(1, 1), (-100, 200), (0, -4095)]);
    }

    /// I-picture, INTRA MB, all-zero block patterns, no DQUANT.
    /// MCBPC code "1" (idx 0). CBPY code for pattern 0000 is
    /// "0011" (idx 0). No MVD.
    #[test]
    fn intra_picture_minimal_mb_no_dquant() {
        let mut w = BitWriter::new();
        // MCBPC "1"
        w.write_bit(true);
        // CBPY "0011" (idx 0, pattern 0000)
        w.write_u32(0b0011, 4);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, intra_picture_ctx(8)).expect("parse");
        assert!(mb.coded);
        assert_eq!(mb.mb_type, Some(MbType::Intra));
        assert_eq!(mb.cbpc, Some(0b00));
        assert_eq!(mb.cbpy, Some(0b0000));
        assert_eq!(mb.dquant, None);
        assert_eq!(mb.quantiser_after, 8);
        assert_eq!(mb.mvd, None);
        assert!(mb.mvd234.iter().all(|m| m.is_none()));
    }

    /// I-picture, INTRA+Q MB, with DQUANT differential +1.
    #[test]
    fn intra_picture_intraq_with_dquant_plus_one() {
        let mut w = BitWriter::new();
        // MCBPC "0001" (idx 4, type 4 / cbpc 00)
        w.write_u32(0b0001, 4);
        // CBPY for pattern 1111 (idx 15) is "11"
        w.write_u32(0b11, 2);
        // DQUANT "10" -> +1
        w.write_u32(0b10, 2);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, intra_picture_ctx(8)).expect("parse");
        assert_eq!(mb.mb_type, Some(MbType::IntraQ));
        assert_eq!(mb.cbpc, Some(0b00));
        assert_eq!(mb.cbpy, Some(0b1111));
        assert_eq!(mb.dquant, Some(1));
        assert_eq!(mb.quantiser_after, 9);
    }

    /// I-picture, INTRA+Q MB, DQUANT differential -2 clamps QUANT
    /// to 1.
    #[test]
    fn intra_picture_dquant_clamps_to_one() {
        let mut w = BitWriter::new();
        w.write_u32(0b0001, 4); // MCBPC idx 4
        w.write_u32(0b11, 2); // CBPY 1111
        w.write_u32(0b01, 2); // DQUANT -2
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, intra_picture_ctx(2)).expect("parse");
        assert_eq!(mb.dquant, Some(-2));
        assert_eq!(mb.quantiser_after, 1);
    }

    /// Annex T (§T.2.1) Modified Quantization mode: an INTRA+Q
    /// macroblock whose DQUANT is the small-step codeword `"11"`
    /// resolves the new QUANT through Table T.1 (prior 11 → +2 → 13),
    /// not the baseline 2-bit differential, and consumes only 2 bits.
    #[test]
    fn intra_picture_modified_quant_small_step() {
        let mut w = BitWriter::new();
        w.write_u32(0b0001, 4); // MCBPC idx 4 (type 4 / cbpc 00)
        w.write_u32(0b11, 2); // CBPY 1111
        w.write_u32(0b11, 2); // §T.2.1 DQUANT "11"
        let bytes = finish_aligned(w);

        let mut ctx = intra_picture_ctx(11);
        ctx.modified_quant = true;
        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, ctx).expect("parse");
        assert_eq!(mb.mb_type, Some(MbType::IntraQ));
        // Table T.1: prior 11, "11" → +2 → new QUANT 13.
        assert_eq!(mb.quantiser_after, 13);
        assert_eq!(mb.dquant, Some(2));
    }

    /// Annex T (§T.2.2) Modified Quantization mode: the arbitrary-
    /// selection DQUANT form (`0` + 5 bits) sets a brand-new QUANT
    /// directly, independent of the prior QUANT, consuming 6 bits.
    #[test]
    fn intra_picture_modified_quant_arbitrary() {
        let mut w = BitWriter::new();
        w.write_u32(0b0001, 4); // MCBPC idx 4
        w.write_u32(0b11, 2); // CBPY 1111
        w.write_u32(0b0_01111, 6); // §T.2.2 "001111" → new QUANT 15
        let bytes = finish_aligned(w);

        let mut ctx = intra_picture_ctx(2);
        ctx.modified_quant = true;
        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, ctx).expect("parse");
        assert_eq!(mb.quantiser_after, 15);
        // Signed change carried on dquant: 15 − 2 = +13.
        assert_eq!(mb.dquant, Some(13));
    }

    /// The Modified Quantization flag only changes DQUANT decoding for
    /// MB types that actually carry DQUANT; a plain INTRA macroblock
    /// (no DQUANT) is unaffected and keeps the GOB QUANT.
    #[test]
    fn modified_quant_flag_inert_without_dquant() {
        let mut w = BitWriter::new();
        w.write_u32(0b1, 1); // MCBPC "1" (idx 0, type 3 / cbpc 00)
        w.write_u32(0b0011, 4); // CBPY 0000
        let bytes = finish_aligned(w);

        let mut ctx = intra_picture_ctx(8);
        ctx.modified_quant = true;
        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, ctx).expect("parse");
        assert_eq!(mb.mb_type, Some(MbType::Intra));
        assert_eq!(mb.dquant, None);
        assert_eq!(mb.quantiser_after, 8);
    }

    /// I-picture, INTRA+Q MB, DQUANT differential +2 clamps QUANT
    /// to 31.
    #[test]
    fn intra_picture_dquant_clamps_to_thirty_one() {
        let mut w = BitWriter::new();
        w.write_u32(0b0001, 4);
        w.write_u32(0b11, 2);
        w.write_u32(0b11, 2); // DQUANT +2
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, intra_picture_ctx(30)).expect("parse");
        assert_eq!(mb.dquant, Some(2));
        assert_eq!(mb.quantiser_after, 31);
    }

    /// I-picture, MCBPC stuffing terminates the MB. Subsequent
    /// bits in the buffer must remain unconsumed.
    #[test]
    fn intra_picture_mcbpc_stuffing_is_recognised() {
        let mut w = BitWriter::new();
        // MCBPC stuffing "0000 0000 1" (9 bits, idx 8 in Table 7).
        w.write_u32(0b0_0000_0001, 9);
        // Sentinel: another "1" that must NOT be consumed.
        w.write_bit(true);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, intra_picture_ctx(5)).expect("parse");
        assert_eq!(mb.mb_type, Some(MbType::Stuffing));
        assert!(mb.cbpc.is_none());
        assert!(mb.cbpy.is_none());
        assert!(mb.dquant.is_none());
        assert_eq!(mb.quantiser_after, 5);
        // Reader must be at exactly 9 bits consumed.
        assert_eq!(r.bit_position(), 9);
    }

    /// P-picture, COD = 1 -> skipped MB. Subsequent buffer bits
    /// remain unconsumed.
    #[test]
    fn inter_picture_skipped_macroblock() {
        let mut w = BitWriter::new();
        w.write_bit(true); // COD = 1 -> skipped
        w.write_bit(true); // sentinel
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, inter_picture_ctx(10, false)).expect("parse");
        assert!(!mb.coded);
        assert_eq!(mb.mb_type, None);
        assert_eq!(mb.mvd, None);
        assert_eq!(mb.quantiser_after, 10);
        // Only COD consumed.
        assert_eq!(r.bit_position(), 1);
    }

    /// P-picture, COD = 0, MCBPC "1" -> INTER MB type 0 / cbpc 00,
    /// CBPY pattern 0011 idx 0, MVD (0, 0).
    #[test]
    fn inter_picture_minimal_inter_mb_zero_mvd() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC "1"
        w.write_u32(0b0011, 4); // CBPY idx 0
                                // MVD horizontal "1" (idx 32, vector 0)
        w.write_bit(true);
        // MVD vertical "1"
        w.write_bit(true);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, inter_picture_ctx(10, false)).expect("parse");
        assert!(mb.coded);
        assert_eq!(mb.mb_type, Some(MbType::Inter));
        assert_eq!(mb.cbpc, Some(0b00));
        assert_eq!(mb.cbpy, Some(0b0000));
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: 0,
                dy_half: 0
            })
        );
        assert_eq!(mb.dquant, None);
    }

    /// P-picture, INTER4V with Advanced Prediction → MVD2-4 follow.
    /// MCBPC "010" (idx 8, type 2 cbpc 00).
    #[test]
    fn inter_picture_inter4v_with_adv_pred_pulls_four_mvds() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_u32(0b010, 3); // MCBPC idx 8
        w.write_u32(0b11, 2); // CBPY idx 15 pattern 1111
                              // Four MVDs (each = (0,0) = "1","1")
        for _ in 0..4 {
            w.write_bit(true); // dx = 0
            w.write_bit(true); // dy = 0
        }
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, inter_picture_ctx(12, true)).expect("parse");
        assert_eq!(mb.mb_type, Some(MbType::Inter4V));
        assert_eq!(mb.cbpy, Some(0b1111));
        assert!(mb.mvd.is_some());
        for slot in &mb.mvd234 {
            assert_eq!(
                *slot,
                Some(Mvd {
                    dx_half: 0,
                    dy_half: 0
                })
            );
        }
    }

    /// Without Advanced Prediction the INTER4V MB carries only
    /// one MVD even though the type would otherwise pull MVD2-4.
    #[test]
    fn inter4v_without_adv_pred_has_only_primary_mvd() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD
        w.write_u32(0b010, 3); // MCBPC idx 8 type 2 cbpc 00
        w.write_u32(0b0011, 4); // CBPY idx 0 pattern 0000
        w.write_bit(true);
        w.write_bit(true); // MVD (0,0)
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, inter_picture_ctx(12, false)).expect("parse");
        assert_eq!(mb.mb_type, Some(MbType::Inter4V));
        assert!(mb.mvd.is_some());
        for slot in &mb.mvd234 {
            assert!(slot.is_none());
        }
    }

    /// MVD vector decode round-trips a non-zero half-pel value.
    /// "011" (idx 31) is -1 half-pel; "010" (idx 33) is +1 half-pel.
    #[test]
    fn mvd_component_table_basic_round_trip() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC "1"
        w.write_u32(0b0011, 4); // CBPY pattern 0000
                                // MVD dx = +1 (code "010"), dy = -1 (code "011")
        w.write_u32(0b010, 3);
        w.write_u32(0b011, 3);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, inter_picture_ctx(8, false)).expect("parse");
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: 1,
                dy_half: -1
            })
        );
    }

    /// MVD vector decode at the table extremes.
    /// idx 0 (-16) = "0000 0000 0010 1" (13 bits).
    /// idx 63 (+15.5) = "0000 0000 0011 0" (13 bits).
    #[test]
    fn mvd_component_table_extremes() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD
        w.write_bit(true); // MCBPC "1"
        w.write_u32(0b0011, 4); // CBPY pattern 0000
                                // dx = -16 (half -32)
        w.write_u32(0b0_0000_0000_0010_1, 13);
        // dy = +15.5 (half +31)
        w.write_u32(0b0_0000_0000_0011_0, 13);
        let bytes = finish_aligned(w);

        let mut r = BitReader::new(&bytes);
        let mb = parse_macroblock(&mut r, inter_picture_ctx(8, false)).expect("parse");
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: -32,
                dy_half: 31,
            })
        );
    }

    /// All Table 14 MVD codes round-trip to the expected half-pel
    /// value. Iterates the full 64-row table and synthesises a
    /// minimal INTER MB header for each row.
    #[test]
    fn mvd_component_table_full_64_round_trip() {
        // Re-derive the (bits, code, half) triples here so the
        // test stays independent of the impl table.
        let table: &[(u8, u16, i8)] = &[
            (13, 0b0_0000_0000_0010_1, -32),
            (13, 0b0_0000_0000_0011_1, -31),
            (12, 0b0000_0000_0101, -30),
            (12, 0b0000_0000_0111, -29),
            (12, 0b0000_0000_1001, -28),
            (12, 0b0000_0000_1011, -27),
            (12, 0b0000_0000_1101, -26),
            (12, 0b0000_0000_1111, -25),
            (11, 0b0000_0001_001, -24),
            (11, 0b0000_0001_011, -23),
            (11, 0b0000_0001_101, -22),
            (11, 0b0000_0001_111, -21),
            (11, 0b0000_0010_001, -20),
            (11, 0b0000_0010_011, -19),
            (11, 0b0000_0010_101, -18),
            (11, 0b0000_0010_111, -17),
            (11, 0b0000_0011_001, -16),
            (11, 0b0000_0011_011, -15),
            (11, 0b0000_0011_101, -14),
            (11, 0b0000_0011_111, -13),
            (11, 0b0000_0100_001, -12),
            (11, 0b0000_0100_011, -11),
            (10, 0b0000_0100_11, -10),
            (10, 0b0000_0101_01, -9),
            (10, 0b0000_0101_11, -8),
            (8, 0b0000_0111, -7),
            (8, 0b0000_1001, -6),
            (8, 0b0000_1011, -5),
            (7, 0b000_0111, -4),
            (5, 0b0001_1, -3),
            (4, 0b0011, -2),
            (3, 0b011, -1),
            (1, 0b1, 0),
            (3, 0b010, 1),
            (4, 0b0010, 2),
            (5, 0b0001_0, 3),
            (7, 0b000_0110, 4),
            (8, 0b0000_1010, 5),
            (8, 0b0000_1000, 6),
            (8, 0b0000_0110, 7),
            (10, 0b0000_0101_10, 8),
            (10, 0b0000_0101_00, 9),
            (10, 0b0000_0100_10, 10),
            (11, 0b0000_0100_010, 11),
            (11, 0b0000_0100_000, 12),
            (11, 0b0000_0011_110, 13),
            (11, 0b0000_0011_100, 14),
            (11, 0b0000_0011_010, 15),
            (11, 0b0000_0011_000, 16),
            (11, 0b0000_0010_110, 17),
            (11, 0b0000_0010_100, 18),
            (11, 0b0000_0010_010, 19),
            (11, 0b0000_0010_000, 20),
            (11, 0b0000_0001_110, 21),
            (11, 0b0000_0001_100, 22),
            (11, 0b0000_0001_010, 23),
            (11, 0b0000_0001_000, 24),
            (12, 0b0000_0000_1110, 25),
            (12, 0b0000_0000_1100, 26),
            (12, 0b0000_0000_1010, 27),
            (12, 0b0000_0000_1000, 28),
            (12, 0b0000_0000_0110, 29),
            (12, 0b0000_0000_0100, 30),
            (13, 0b0_0000_0000_0011_0, 31),
        ];
        for &(bits, code, half) in table {
            let mut w = BitWriter::new();
            w.write_bit(false); // COD
            w.write_bit(true); // MCBPC "1"
            w.write_u32(0b0011, 4); // CBPY pattern 0000
            w.write_u32(code as u32, bits as u32);
            w.write_bit(true); // dy = 0 (idx 32)
            let bytes = finish_aligned(w);
            let mut r = BitReader::new(&bytes);
            let mb = parse_macroblock(&mut r, inter_picture_ctx(8, false)).expect("parse");
            assert_eq!(
                mb.mvd,
                Some(Mvd {
                    dx_half: half as i16,
                    dy_half: 0,
                }),
                "code {:b} bits {} half {}",
                code,
                bits,
                half
            );
        }
    }

    /// All Table 12 CBPY codes round-trip to the expected
    /// natural-binary INTRA pattern.
    #[test]
    fn cbpy_table_full_round_trip() {
        // (bits, code, pattern)
        let table: &[(u8, u16, u8)] = &[
            (4, 0b0011, 0b0000),   // idx 0
            (5, 0b00101, 0b0001),  // idx 1
            (5, 0b00100, 0b0010),  // idx 2
            (4, 0b1001, 0b0011),   // idx 3
            (5, 0b00011, 0b0100),  // idx 4
            (4, 0b0111, 0b0101),   // idx 5
            (6, 0b000010, 0b0110), // idx 6
            (4, 0b1011, 0b0111),   // idx 7
            (5, 0b00010, 0b1000),  // idx 8
            (6, 0b000011, 0b1001), // idx 9
            (4, 0b0101, 0b1010),   // idx 10
            (4, 0b1010, 0b1011),   // idx 11
            (4, 0b0100, 0b1100),   // idx 12
            (4, 0b1000, 0b1101),   // idx 13
            (4, 0b0110, 0b1110),   // idx 14
            (2, 0b11, 0b1111),     // idx 15
        ];
        for &(bits, code, pattern) in table {
            // Wrap each CBPY entry in a minimal INTRA MB:
            // MCBPC "1" (idx 0, type 3 cbpc 00) then this CBPY.
            let mut w = BitWriter::new();
            w.write_bit(true); // MCBPC "1"
            w.write_u32(code as u32, bits as u32);
            let bytes = finish_aligned(w);
            let mut r = BitReader::new(&bytes);
            let mb = parse_macroblock(&mut r, intra_picture_ctx(8)).expect("parse");
            assert_eq!(
                mb.cbpy,
                Some(pattern),
                "code {:b} bits {} pattern {:b}",
                code,
                bits,
                pattern
            );
        }
    }

    /// All Table 7 MCBPC (I-picture) codes round-trip to the
    /// expected (mb_type, cbpc) pair. Indices 0..=8.
    #[test]
    fn mcbpc_i_picture_table_full_round_trip() {
        // (bits, code, mb_type, cbpc) — Table 7.
        let table: &[(u8, u16, MbType, u8)] = &[
            (1, 0b1, MbType::Intra, 0b00),
            (3, 0b001, MbType::Intra, 0b01),
            (3, 0b010, MbType::Intra, 0b10),
            (3, 0b011, MbType::Intra, 0b11),
            (4, 0b0001, MbType::IntraQ, 0b00),
            (6, 0b000001, MbType::IntraQ, 0b01),
            (6, 0b000010, MbType::IntraQ, 0b10),
            (6, 0b000011, MbType::IntraQ, 0b11),
            (9, 0b000000001, MbType::Stuffing, 0),
        ];
        for &(bits, code, ty, cbpc) in table {
            let mut w = BitWriter::new();
            w.write_u32(code as u32, bits as u32);
            // Pad with a CBPY "11" (idx 15) so non-stuffing rows
            // can complete the header.
            if !matches!(ty, MbType::Stuffing) {
                w.write_u32(0b11, 2);
                if ty.has_dquant() {
                    w.write_u32(0b10, 2); // DQUANT +1
                }
            }
            let bytes = finish_aligned(w);
            let mut r = BitReader::new(&bytes);
            let mb = parse_macroblock(&mut r, intra_picture_ctx(8)).expect("parse");
            assert_eq!(mb.mb_type, Some(ty), "code {:b}", code);
            if matches!(ty, MbType::Stuffing) {
                assert!(mb.cbpc.is_none());
            } else {
                assert_eq!(mb.cbpc, Some(cbpc), "code {:b}", code);
            }
        }
    }

    /// All Table 8 MCBPC (P-picture) indices 0..=20 (i.e. types
    /// 0..=4 + stuffing). Indices 21..=24 (type 5) require Annex
    /// F/J which is not in this round; they're exercised
    /// structurally in [`mcbpc_p_picture_type5_codes_decode_but_caller_must_gate`].
    #[test]
    fn mcbpc_p_picture_table_indices_0_through_20() {
        // (bits, code, mb_type, cbpc) — Table 8 indices 0..=20.
        let table: &[(u8, u16, MbType, u8)] = &[
            (1, 0b1, MbType::Inter, 0b00),
            (4, 0b0011, MbType::Inter, 0b01),
            (4, 0b0010, MbType::Inter, 0b10),
            (6, 0b000101, MbType::Inter, 0b11),
            (3, 0b011, MbType::InterQ, 0b00),
            (7, 0b0000111, MbType::InterQ, 0b01),
            (7, 0b0000110, MbType::InterQ, 0b10),
            (9, 0b000000101, MbType::InterQ, 0b11),
            (3, 0b010, MbType::Inter4V, 0b00),
            (7, 0b0000101, MbType::Inter4V, 0b01),
            (7, 0b0000100, MbType::Inter4V, 0b10),
            (8, 0b00000101, MbType::Inter4V, 0b11),
            (5, 0b00011, MbType::Intra, 0b00),
            (8, 0b00000100, MbType::Intra, 0b01),
            (8, 0b00000011, MbType::Intra, 0b10),
            (7, 0b0000011, MbType::Intra, 0b11),
            (6, 0b000100, MbType::IntraQ, 0b00),
            (9, 0b000000100, MbType::IntraQ, 0b01),
            (9, 0b000000011, MbType::IntraQ, 0b10),
            (9, 0b000000010, MbType::IntraQ, 0b11),
            (9, 0b000000001, MbType::Stuffing, 0),
        ];
        for &(bits, code, ty, cbpc) in table {
            let mut w = BitWriter::new();
            w.write_bit(false); // COD = 0 -> coded
            w.write_u32(code as u32, bits as u32);
            if !matches!(ty, MbType::Stuffing) {
                // CBPY idx 0 (pattern 0000) = "0011" (4 bits)
                w.write_u32(0b0011, 4);
                if ty.has_dquant() {
                    w.write_u32(0b10, 2); // DQUANT +1
                }
                if ty.has_mvd() {
                    w.write_bit(true); // MVD dx = 0
                    w.write_bit(true); // MVD dy = 0
                }
            }
            let bytes = finish_aligned(w);
            let mut r = BitReader::new(&bytes);
            let mb = parse_macroblock(&mut r, inter_picture_ctx(8, false)).expect("parse");
            assert_eq!(mb.mb_type, Some(ty), "code {:b}", code);
            if matches!(ty, MbType::Stuffing) {
                assert!(mb.cbpc.is_none());
            } else {
                assert_eq!(mb.cbpc, Some(cbpc), "code {:b}", code);
            }
        }
    }

    /// Type-5 MCBPC code-points decode structurally; downstream
    /// gating is the caller's job. This test just confirms the
    /// VLC dispatcher reaches Inter4VQ for all four sub-codes.
    #[test]
    fn mcbpc_p_picture_type5_codes_decode_but_caller_must_gate() {
        // (bits, code, cbpc) for indices 21..=24.
        let table: &[(u8, u16, u8)] = &[
            (11, 0b00000000010, 0b00),   // idx 21
            (13, 0b0000000001100, 0b01), // idx 22
            (13, 0b0000000001110, 0b10), // idx 23
            (13, 0b0000000001111, 0b11), // idx 24
        ];
        for &(bits, code, cbpc) in table {
            let mut w = BitWriter::new();
            w.write_bit(false); // COD = 0
            w.write_u32(code as u32, bits as u32);
            // CBPY idx 0, DQUANT +1, MVD (0,0), MVD2-4 (0,0)*3
            w.write_u32(0b0011, 4);
            w.write_u32(0b10, 2);
            for _ in 0..4 {
                w.write_bit(true);
                w.write_bit(true);
            }
            let bytes = finish_aligned(w);
            let mut r = BitReader::new(&bytes);
            let mb = parse_macroblock(&mut r, inter_picture_ctx(8, true)).expect("parse");
            assert_eq!(mb.mb_type, Some(MbType::Inter4VQ), "code {:b}", code);
            assert_eq!(mb.cbpc, Some(cbpc));
            assert_eq!(mb.dquant, Some(1));
            assert!(mb.mvd.is_some());
            for slot in &mb.mvd234 {
                assert!(slot.is_some());
            }
        }
    }

    /// Buffer shorter than the MCBPC code: UnexpectedEof.
    #[test]
    fn truncated_mcbpc_yields_unexpected_eof() {
        // I-picture; only 4 zero bits — not enough to terminate
        // the longest stuffing prefix.
        let bytes = [0u8; 1];
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_macroblock(&mut r, intra_picture_ctx(8)).unwrap_err(),
            Error::UnexpectedEof
        );
    }

    /// MCBPC bucket with no valid code-point → BadMcbpcCode.
    /// I-picture, lz=6: no idx; the parser must reject.
    #[test]
    fn invalid_mcbpc_bucket_rejected() {
        let mut w = BitWriter::new();
        // Six zeros + 1 -> lz=6, no I-picture row uses it.
        w.write_u32(0b0_000000_1, 8);
        // Pad with extra bits so EOF doesn't preempt.
        w.write_u32(0, 16);
        let bytes = finish_aligned(w);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_macroblock(&mut r, intra_picture_ctx(8)).unwrap_err(),
            Error::BadMcbpcCode
        );
    }

    /// Context quantiser out of range is rejected before any read.
    #[test]
    fn quantiser_out_of_range_rejected() {
        let bytes = [0xFFu8; 4];
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_macroblock(&mut r, intra_picture_ctx(0)).unwrap_err(),
            Error::InvalidQuantiser
        );
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_macroblock(&mut r, intra_picture_ctx(32)).unwrap_err(),
            Error::InvalidQuantiser
        );
    }

    /// Composes with the GOB-layer header: parse_gob_layer then
    /// parse_macroblock on a single reader, verifying that the
    /// macroblock parser picks up exactly where the GOB header
    /// left off.
    #[test]
    fn composes_with_gob_layer_header_then_one_intra_mb() {
        use crate::{
            gob_header::{GBSC_BITS, GBSC_VALUE, GFID_BITS, GN_BITS, GQUANT_BITS},
            parse_gob_layer, parse_picture_header, PSC_BITS, PSC_VALUE,
        };

        let mut w = BitWriter::new();
        // Picture header — QCIF intra.
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8); // TR
        w.write_bit(true);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_u32(0b010, 3); // QCIF
        w.write_bit(false); // INTRA
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        // GOB header.
        w.write_u32(GBSC_VALUE, GBSC_BITS);
        w.write_u32(1, GN_BITS);
        w.write_u32(0, GFID_BITS);
        w.write_u32(5, GQUANT_BITS); // QUANT=5
                                     // One intra MB: MCBPC "1", CBPY "0011" (pattern 0000).
        w.write_bit(true);
        w.write_u32(0b0011, 4);
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
                deblocking_filter: false,
                aic_intra_mode: false,
                pb_frames: false,
                pb_annex_m: false,
                quantiser_before: gob.quantiser,
                modified_quant: false,
                umv_table_d3: false,
            },
        )
        .expect("mb");
        assert_eq!(mb.mb_type, Some(MbType::Intra));
        assert_eq!(mb.cbpy, Some(0b0000));
        assert_eq!(mb.quantiser_after, 5);
    }

    // ---- PB-frames mode (§5.3 Table 10 / Figure 10) ----------------

    fn pb_picture_ctx(q: u8) -> MbContext {
        MbContext {
            picture_coding_type: H263PictureCodingType::Inter,
            advanced_prediction: false,
            deblocking_filter: false,
            aic_intra_mode: false,
            pb_frames: true,
            pb_annex_m: false,
            quantiser_before: q,
            modified_quant: false,
            umv_table_d3: false,
        }
    }

    /// Annex M (Improved PB-frames) macroblock context: Table M.1 MODB
    /// instead of Table 11.
    fn improved_pb_picture_ctx(q: u8) -> MbContext {
        MbContext {
            picture_coding_type: H263PictureCodingType::Inter,
            advanced_prediction: false,
            deblocking_filter: false,
            aic_intra_mode: false,
            pb_frames: true,
            pb_annex_m: true,
            quantiser_before: q,
            modified_quant: false,
            umv_table_d3: false,
        }
    }

    /// PB-mode INTER macroblock with MODB row 0 (`0`): no CBPB, no
    /// MVDB. Wire: COD `0`, MCBPC `1` (type 0, cbpc 00), MODB `0`,
    /// CBPY `11` (CBPY(INTRA) = 1111 → INTER pattern 0000), then MVD
    /// `1` `1` — 7 bits total.
    #[test]
    fn pb_inter_mb_modb_none_parses_mvd_only() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC type 0, cbpc 00
        w.write_bit(false); // MODB row 0
        w.write_u32(0b11, 2); // CBPY
        w.write_bit(true); // MVD dx = 0
        w.write_bit(true); // MVD dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, pb_picture_ctx(8)).expect("mb");
        assert!(mb.coded);
        assert_eq!(mb.mb_type, Some(MbType::Inter));
        assert_eq!(mb.modb, Some(ModbPresence::None));
        assert_eq!(mb.cbpb, None);
        assert_eq!(mb.mvdb, None);
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: 0,
                dy_half: 0
            })
        );
        assert_eq!(r.bit_position(), 7);
    }

    /// PB-mode INTER macroblock with MODB row 2 (`11`): CBPB and
    /// MVDB both on the wire, in Figure 10 order — MODB and CBPB
    /// between MCBPC and CBPY, MVDB after MVD. CBPB `101010` lights
    /// B-blocks 1 / 3 / 5; MVDB pair (+1, −1) half-pel (Table 14
    /// codes `010` / `011`). Total 1 + 1 + 2 + 6 + 2 + 2 + 6 = 20
    /// bits.
    #[test]
    fn pb_inter_mb_modb_cbpb_and_mvdb_full_layer() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC type 0, cbpc 00
        w.write_u32(0b11, 2); // MODB row 2
        w.write_u32(0b101010, 6); // CBPB: blocks 1, 3, 5
        w.write_u32(0b11, 2); // CBPY
        w.write_bit(true); // MVD dx = 0
        w.write_bit(true); // MVD dy = 0
        w.write_u32(0b010, 3); // MVDB dx = +1 half-pel
        w.write_u32(0b011, 3); // MVDB dy = -1 half-pel
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, pb_picture_ctx(8)).expect("mb");
        assert_eq!(mb.modb, Some(ModbPresence::CbpbAndMvdb));
        assert_eq!(mb.cbpb, Some(0b101010));
        assert_eq!(
            mb.mvdb,
            Some(Mvd {
                dx_half: 1,
                dy_half: -1
            })
        );
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: 0,
                dy_half: 0
            })
        );
        assert_eq!(r.bit_position(), 20);
    }

    /// §5.3.7 / §G.2: "MVD is included for all INTER macroblocks (in
    /// PB-frames mode also for INTRA macroblocks)" — an INTRA
    /// macroblock (Table 8 type 3, code `0001 1`) in a PB-mode
    /// P-picture carries MODB, MVD (here +2 half-pel horizontal,
    /// Table 14 code `0010`) and — via MODB row 1 (`10`) — MVDB, but
    /// never MVD2-4 ("The codewords MVD2-4 are never used for
    /// INTRA", §G.2). Total 1 + 5 + 2 + 4 + 5 + 2 = 19 bits.
    #[test]
    fn pb_intra_mb_carries_mvd_and_mvdb() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_u32(0b00011, 5); // MCBPC type 3 (INTRA), cbpc 00
        w.write_u32(0b10, 2); // MODB row 1: MVDB only
        w.write_u32(0b0011, 4); // CBPY: CBPY(INTRA) = 0000
        w.write_u32(0b0010, 4); // MVD dx = +2 half-pel
        w.write_bit(true); // MVD dy = 0
        w.write_bit(true); // MVDB dx = 0
        w.write_bit(true); // MVDB dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, pb_picture_ctx(8)).expect("mb");
        assert_eq!(mb.mb_type, Some(MbType::Intra));
        assert_eq!(mb.modb, Some(ModbPresence::MvdbOnly));
        assert_eq!(mb.cbpb, None);
        assert_eq!(
            mb.mvd,
            Some(Mvd {
                dx_half: 2,
                dy_half: 0
            })
        );
        assert_eq!(
            mb.mvdb,
            Some(Mvd {
                dx_half: 0,
                dy_half: 0
            })
        );
        assert_eq!(mb.mvd234, [None; 3]);
        assert_eq!(r.bit_position(), 19);
    }

    /// A skipped macroblock (COD = 1) in PB-frames mode carries no
    /// further fields (Table 10 "Not coded" row: COD only).
    #[test]
    fn pb_skipped_mb_carries_no_pb_fields() {
        let mut w = BitWriter::new();
        w.write_bit(true); // COD = 1
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, pb_picture_ctx(8)).expect("mb");
        assert!(!mb.coded);
        assert_eq!(mb.modb, None);
        assert_eq!(mb.cbpb, None);
        assert_eq!(mb.mvdb, None);
        assert_eq!(r.bit_position(), 1);
    }

    /// Annex M (Improved PB-frames): MODB is the §M.4 Table M.1 form,
    /// surfaced on `annex_m_modb` (not `modb`). Table M.1 row 0
    /// (code `0`, bidirectional, no CBPB / no MVDB) parses with no
    /// CBPB / MVDB on the wire. Total: COD `0` + MCBPC `1` + MODB `0`
    /// + CBPY `11` + MVD `1` `1` = 7 bits.
    #[test]
    fn improved_pb_inter_mb_modb_row0_bidirectional() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC type 0, cbpc 00
        w.write_bit(false); // Table M.1 row 0 (code `0`)
        w.write_u32(0b11, 2); // CBPY
        w.write_bit(true); // MVD dx = 0
        w.write_bit(true); // MVD dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, improved_pb_picture_ctx(8)).expect("mb");
        assert_eq!(mb.modb, None, "Annex G MODB not populated under Annex M");
        assert_eq!(mb.annex_m_modb, Some(ModbAnnexM::BidirNoCbpbNoMvdb));
        assert_eq!(mb.cbpb, None);
        assert_eq!(mb.mvdb, None);
        assert_eq!(r.bit_position(), 7);
    }

    /// Annex M Table M.1 row 3 (code `1110`, forward, CBPB + MVDB both
    /// present): the parser reads CBPB between MODB and CBPY and MVDB
    /// after MVD, gating both on the Table M.1 accessors. CBPB
    /// `110000` lights B-blocks 1 / 2; MVDB (+2, 0) (Table 14 `0010` /
    /// `1`). Total: 1 + 1 + 4 + 6 + 2 + 1 + 1 + 4 + 1 = 21 bits.
    #[test]
    fn improved_pb_inter_mb_modb_row3_forward_cbpb_mvdb() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC type 0, cbpc 00
        w.write_u32(0b1110, 4); // Table M.1 row 3 (code `1110`)
        w.write_u32(0b110000, 6); // CBPB: blocks 1, 2
        w.write_u32(0b11, 2); // CBPY
        w.write_bit(true); // MVD dx = 0
        w.write_bit(true); // MVD dy = 0
        w.write_u32(0b0010, 4); // MVDB dx = +2 half-pel
        w.write_bit(true); // MVDB dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, improved_pb_picture_ctx(8)).expect("mb");
        assert_eq!(mb.annex_m_modb, Some(ModbAnnexM::ForwardCbpbMvdb));
        assert_eq!(mb.cbpb, Some(0b110000));
        assert_eq!(
            mb.mvdb,
            Some(Mvd {
                dx_half: 2,
                dy_half: 0
            })
        );
        assert_eq!(r.bit_position(), 21);
    }

    /// Annex M Table M.1 row 4 (code `11110`, backward, no CBPB / no
    /// MVDB): the 5-bit codeword is consumed and no CBPB / MVDB follow.
    /// Total: 1 + 1 + 5 + 2 + 1 + 1 = 11 bits.
    #[test]
    fn improved_pb_inter_mb_modb_row4_backward() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC type 0, cbpc 00
        w.write_u32(0b11110, 5); // Table M.1 row 4 (code `11110`)
        w.write_u32(0b11, 2); // CBPY
        w.write_bit(true); // MVD dx = 0
        w.write_bit(true); // MVD dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, improved_pb_picture_ctx(8)).expect("mb");
        assert_eq!(mb.annex_m_modb, Some(ModbAnnexM::BackwardNoCbpbNoMvdb));
        assert_eq!(mb.cbpb, None);
        assert_eq!(mb.mvdb, None);
        assert_eq!(r.bit_position(), 11);
    }

    /// §M.2.1 — under Annex M an INTRA P-macroblock carries MVD only in
    /// the bidirectional mode. Backward row 4: COD (1) + MCBPC INTRA
    /// `00011` (5) + MODB `11110` (5) + CBPY `0011` (4) = 15 bits, no
    /// MVD; bidirectional row 0: COD + MCBPC + MODB `0` + CBPY + MVD
    /// (1 + 1) = 13 bits with the vector present.
    #[test]
    fn improved_pb_intra_mb_mvd_only_in_bidirectional_mode() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_u32(0b00011, 5); // MCBPC type 3 (INTRA), cbpc 00
        w.write_u32(0b11110, 5); // Table M.1 row 4 (backward)
        w.write_u32(0b0011, 4); // CBPY (INTRA) pattern 0000
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, improved_pb_picture_ctx(8)).expect("mb");
        assert_eq!(mb.mb_type, Some(MbType::Intra));
        assert_eq!(mb.annex_m_modb, Some(ModbAnnexM::BackwardNoCbpbNoMvdb));
        assert_eq!(
            mb.mvd, None,
            "§M.2.1: no MVD outside the bidirectional mode"
        );
        assert_eq!(r.bit_position(), 15);

        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_u32(0b00011, 5); // MCBPC type 3 (INTRA), cbpc 00
        w.write_bit(false); // Table M.1 row 0 (bidirectional)
        w.write_u32(0b0011, 4); // CBPY (INTRA) pattern 0000
        w.write_bit(true); // MVD dx = 0
        w.write_bit(true); // MVD dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, improved_pb_picture_ctx(8)).expect("mb");
        assert_eq!(mb.annex_m_modb, Some(ModbAnnexM::BidirNoCbpbNoMvdb));
        assert!(
            mb.mvd.is_some(),
            "§M.2.1: MVD present for the bidirectional INTRA case"
        );
        assert_eq!(r.bit_position(), 13);
    }

    /// Outside PB-frames mode the INTER macroblock wire layout has no
    /// MODB / CBPB / MVDB — the new fields stay `None` and no extra
    /// bits are consumed (COD + MCBPC + CBPY + MVD = 6 bits).
    #[test]
    fn non_pb_inter_mb_has_no_pb_fields() {
        let mut w = BitWriter::new();
        w.write_bit(false); // COD = 0
        w.write_bit(true); // MCBPC type 0, cbpc 00
        w.write_u32(0b11, 2); // CBPY
        w.write_bit(true); // MVD dx = 0
        w.write_bit(true); // MVD dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mb = parse_macroblock(&mut r, inter_picture_ctx(8, false)).expect("mb");
        assert_eq!(mb.modb, None);
        assert_eq!(mb.cbpb, None);
        assert_eq!(mb.mvdb, None);
        assert_eq!(r.bit_position(), 6);
    }
}
