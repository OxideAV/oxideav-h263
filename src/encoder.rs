//! H.263 baseline **picture-layer** encode — the §5.1 / §5.2 inverse of
//! [`crate::picture::decode_picture_no_gob0_header`].
//!
//! Assembles a complete baseline INTRA (I-) picture from a planar 4:2:0
//! [`crate::picture::YuvFrame`]: the §5.1 picture header (PSC, TR,
//! PTYPE, PQUANT, CPM, PEI) followed by the §5.2.2 GOB-0-elided
//! macroblock stream. The encoder emits a **single video-picture
//! segment** with no GOB headers after GOB 0 (the §5.2 "optional GOB
//! header" form a real encoder uses for the standard formats), so every
//! macroblock runs at the picture PQUANT.
//!
//! The output decodes end-to-end through
//! [`crate::picture::decode_picture_no_gob0_header`] and
//! [`crate::picture::decode_sequence`], and the reconstructed frame
//! approximates the source to a quantiser-dependent tolerance (the
//! transform + dead-zone quantiser are lossy but round-trip-consistent
//! with the decoder).
//!
//! This is the first end-to-end encode path in the crate; the §5.1.3
//! optional-mode flags (UMV / SAC / AP / PB) are all emitted as `0`
//! (baseline), and only the source formats with fixed §4.1 dimensions
//! (sub-QCIF .. 16CIF) are supported. A frame whose dimensions do not
//! match a standard source format is rejected.

use crate::aic::{write_intra_mode, IntraMode};
use crate::aic_predict::Neighbour;
use crate::block::COEFFS_PER_BLOCK;
use crate::encoder_aic::{plan_intra_block_aic, write_intra_block_aic, AicBlockPlan};
use crate::encoder_mb::{encode_intra_macroblock, MacroblockSamples};
use crate::encoder_vlc::{write_cbpy, write_mcbpc_i};
use crate::macroblock::MbType;
use crate::picture::YuvFrame;
use crate::picture_header::{H263SourceFormat, PSC_VALUE};
use crate::{Error, Result};
use oxideav_core::bits::BitWriter;

pub use crate::encoder_deblock::{
    encode_inter_picture_deblock, encode_intra_picture_deblock, DeblockConfig,
};
pub use crate::encoder_pb::{
    encode_improved_pb_picture, encode_improved_pb_picture_stats, encode_pb_picture_ap,
    encode_pb_picture_ap_stats, ImprovedPbConfig, ImprovedPbStats,
};
pub use crate::encoder_rc::{
    encode_inter_picture_adaptive, encode_intra_picture_adaptive, AdaptiveQuantConfig,
    AdaptiveQuantPicture,
};

/// Map standard luma dimensions to the §5.1.3 source-format selector.
pub(crate) fn source_format_for(luma_w: usize, luma_h: usize) -> Option<H263SourceFormat> {
    match (luma_w, luma_h) {
        (128, 96) => Some(H263SourceFormat::SubQcif),
        (176, 144) => Some(H263SourceFormat::Qcif),
        (352, 288) => Some(H263SourceFormat::Cif),
        (704, 576) => Some(H263SourceFormat::Cif4),
        (1408, 1152) => Some(H263SourceFormat::Cif16),
        _ => None,
    }
}

/// §5.1.3 — the 3-bit source-format code.
fn source_format_bits(fmt: H263SourceFormat) -> u32 {
    match fmt {
        H263SourceFormat::SubQcif => 0b001,
        H263SourceFormat::Qcif => 0b010,
        H263SourceFormat::Cif => 0b011,
        H263SourceFormat::Cif4 => 0b100,
        H263SourceFormat::Cif16 => 0b101,
        H263SourceFormat::Reserved110 => 0b110,
    }
}

/// Extract the six 8×8 sample blocks of the macroblock at grid position
/// `(mb_col, mb_row)` from a planar 4:2:0 frame.
///
/// Luma blocks are the 2×2 grid Y1..Y4; chroma is a single 8×8 block
/// each (the macroblock's 16×16 luma maps to 8×8 chroma at 4:2:0).
pub(crate) fn extract_macroblock(
    frame: &YuvFrame,
    mb_col: usize,
    mb_row: usize,
) -> MacroblockSamples {
    let lw = frame.luma_width;
    let cw = frame.chroma_width();
    let mut mb = MacroblockSamples {
        luma: [[0i16; COEFFS_PER_BLOCK]; 4],
        cb: [0i16; COEFFS_PER_BLOCK],
        cr: [0i16; COEFFS_PER_BLOCK],
    };
    // Luma: four 8×8 blocks in a 2×2 layout.
    let mb_x = mb_col * 16;
    let mb_y = mb_row * 16;
    for (blk, blk_samples) in mb.luma.iter_mut().enumerate() {
        let bx = mb_x + (blk % 2) * 8;
        let by = mb_y + (blk / 2) * 8;
        for row in 0..8 {
            for col in 0..8 {
                let px = bx + col;
                let py = by + row;
                blk_samples[row * 8 + col] = frame.y[py * lw + px] as i16;
            }
        }
    }
    // Chroma: one 8×8 block each at (mb_col*8, mb_row*8).
    let cx = mb_col * 8;
    let cy = mb_row * 8;
    for row in 0..8 {
        for col in 0..8 {
            let idx = (cy + row) * cw + (cx + col);
            mb.cb[row * 8 + col] = frame.cb[idx] as i16;
            mb.cr[row * 8 + col] = frame.cr[idx] as i16;
        }
    }
    mb
}

/// The §5.1.3 PTYPE optional-mode flags the encoder can raise (bits
/// 10–13). Baseline pictures leave every flag `false`.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct PtypeFlags {
    /// Bit 10 — Annex D Unrestricted Motion Vector mode.
    pub(crate) umv: bool,
    /// Bit 11 — Annex E Syntax-based Arithmetic Coding mode.
    pub(crate) sac: bool,
    /// Bit 12 — Annex F Advanced Prediction mode.
    pub(crate) advanced_prediction: bool,
}

/// Write the §5.1 baseline picture header (PSC, TR, PTYPE, PQUANT,
/// CPM=0, PEI=0). `is_inter` selects the §5.1.3 picture coding-type bit
/// (INTRA = 0, INTER = 1); `flags` raises the optional-mode PTYPE bits
/// (SAC stays 0 on this path). `pb_fields` — `Some((trb, dbquant))` —
/// raises the PTYPE bit-13 PB-frames flag and emits the §5.1.22 TRB +
/// §5.1.23 DBQUANT fields between CPM and PEI.
pub(crate) fn write_picture_header(
    w: &mut BitWriter,
    fmt: H263SourceFormat,
    quant: u8,
    tr: u8,
    is_inter: bool,
    flags: PtypeFlags,
    pb_fields: Option<(u8, u8)>,
) {
    // §5.1.1 — Picture Start Code (22 bits, 0x000020).
    w.write_bits(PSC_VALUE, 22);
    // §5.1.2 — Temporal Reference (8 bits).
    w.write_bits(tr as u32, 8);
    // §5.1.3 — PTYPE. bit1 = 1, bit2 = 0, then split/doc/freeze = 0,
    // source-format (3), coding-type, then the UMV/SAC/AP/PB flags.
    w.write_bit(true); // bit 1
    w.write_bit(false); // bit 2
    w.write_bit(false); // split-screen
    w.write_bit(false); // document-camera
    w.write_bit(false); // freeze-release
    w.write_bits(source_format_bits(fmt), 3);
    w.write_bit(is_inter); // coding-type: 0 INTRA / 1 INTER
    w.write_bit(flags.umv); // UMV (Annex D)
    w.write_bit(flags.sac); // SAC (Annex E)
    w.write_bit(flags.advanced_prediction); // AP (Annex F)
    w.write_bit(pb_fields.is_some()); // PB (Annex G)
                                      // §5.1.19 — PQUANT (5 bits).
    w.write_bits(quant as u32, 5);
    // §5.1.20 — CPM (1 bit, 0 = single bitstream).
    w.write_bit(false);
    // §5.1.22 TRB (3 bits) + §5.1.23 DBQUANT (2 bits) — PB-frames only.
    if let Some((trb, dbquant)) = pb_fields {
        w.write_bits(trb as u32, 3);
        w.write_bits(dbquant as u32, 2);
    }
    // §5.1.24 — PEI (1 bit, 0 = no PSUPP extension).
    w.write_bit(false);
}

/// Write a §5.2 GOB header for GOB `gn` (CPM = 0 stream): §5.2.1 GSTUF
/// zero-stuffing to the next byte boundary, the 17-bit GBSC, the 5-bit
/// Group Number, the 2-bit GOB Frame ID and the 5-bit GQUANT.
fn write_gob_header(w: &mut BitWriter, gn: u32, gfid: u8, gquant: u8) {
    use crate::gob_header::{GBSC_BITS, GBSC_VALUE, GFID_BITS, GN_BITS, GQUANT_BITS};
    // §5.2.1 — GSTUF: zero bits until the GBSC is byte aligned.
    w.align_to_byte_zero();
    w.write_bits(GBSC_VALUE, GBSC_BITS);
    w.write_bits(gn, GN_BITS);
    w.write_bits(gfid as u32 & 0b11, GFID_BITS);
    w.write_bits(gquant as u32, GQUANT_BITS);
}

/// The optional-mode set an extended-PTYPE (PLUSPTYPE / H.263+)
/// picture header signals on the wire, mapped onto the §5.1.4.2
/// OPPTYPE mode bits the crate's decoder stages.
///
/// Every mode defaults to *off*; [`PlusModes::default`] therefore
/// describes a plain H.263+ baseline picture (all OPPTYPE mode bits
/// zero). Mode bits the decoder refuses (SAC, RPS, ISD, custom PCF,
/// custom source formats) are not exposed — the writer cannot emit a
/// header the crate's own decoder would reject.
#[derive(Debug, Clone, Copy, Default)]
pub struct PlusModes {
    /// OPPTYPE bit 5 — Annex D Unrestricted Motion Vector mode. The
    /// §5.1.9 UUI codeword `"1"` (motion-vector range limited per
    /// Tables D.1/D.2) is emitted alongside.
    pub umv: bool,
    /// OPPTYPE bit 7 — Annex F Advanced Prediction mode.
    pub advanced_prediction: bool,
    /// OPPTYPE bit 8 — Annex I Advanced INTRA Coding mode.
    pub advanced_intra: bool,
    /// OPPTYPE bit 9 — Annex J Deblocking Filter mode.
    pub deblocking: bool,
    /// OPPTYPE bit 10 — Annex K Slice Structured mode, with the
    /// §5.1.10 SSS submode bits that follow CPM on the wire.
    pub slice_structured: Option<crate::plus_ptype::SliceStructuredSubmode>,
    /// OPPTYPE bit 13 — Annex S Alternative INTER VLC mode.
    pub alt_inter_vlc: bool,
    /// OPPTYPE bit 14 — Annex T Modified Quantization mode.
    pub modified_quant: bool,
    /// §5.1.20 / §5.1.21 — `Some(psbi)` emits CPM = "1" followed by
    /// the 2-bit Picture Sub-Bitstream Indicator (`0..=3`, Annex C
    /// Continuous Presence Multipoint). `None` emits CPM = "0".
    pub cpm_psbi: Option<u8>,
    /// MPPTYPE bit 5 — Annex Q Reduced-Resolution Update mode.
    pub rru: bool,
    /// OPPTYPE bit 12 — Annex R Independent Segment Decoding mode.
    pub independent_segment_decoding: bool,
    /// OPPTYPE bit 17 — Annex V Data-Partitioned Slice mode (requires
    /// [`Self::slice_structured`] per §V.3).
    pub data_partitioned_slices: bool,
    /// `Some((trb, dbquant))` makes the picture an **Annex M Improved
    /// PB-frame**: MPPTYPE picture type `"010"` (§5.1.4.3), with the
    /// §5.1.22 TRB (3 bits at the standard picture clock frequency,
    /// `1..=7`) and §5.1.23 DBQUANT (`0..=3`) fields emitted after
    /// PQUANT. Requires `is_inter` (the P-part is a P-picture, §M.1).
    pub improved_pb: Option<(u8, u8)>,
}

/// Write an extended-PTYPE (PLUSPTYPE, §5.1.4) picture header: PSC, TR,
/// PTYPE bits 1-5 + the `"111"` extended escape, UFEP `"001"`, the
/// 18-bit OPPTYPE, the 9-bit MPPTYPE, CPM = 0, the conditional §5.1.9
/// UUI / §5.1.10 SSS fields, §5.1.19 PQUANT and §5.1.24 PEI = 0.
///
/// The writer always emits the full-update `UFEP = "001"` form (legal
/// on every picture, and the only form a stateless decoder can accept),
/// with:
///
/// * a standard §5.1.4.2 source format (custom CPFMT formats are not
///   staged — [`Error::NotImplemented`] for
///   [`H263SourceFormat::Reserved110`]),
/// * standard CIF picture clock frequency (OPPTYPE bit 4 = 0),
/// * MPPTYPE picture type `"000"` (I) or `"001"` (P) per `is_inter`,
///   RPR / RRU off and RTYPE = 0,
/// * CPM = 0 (no PSBI), no scalability / RPS / RPR trailing fields.
///
/// After this header the picture body follows immediately: the GOB-0
/// macroblock stream for the GOB layout (§5.2.2 first-GOB header
/// elision), or the first slice's reduced header (§K.2.2) when
/// [`PlusModes::slice_structured`] is signalled.
///
/// The emitted header parses back through
/// [`crate::picture_header::parse_picture_layer`] and decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence`, which
/// auto-activate the wire-signalled Annex I / J / S / T modes without
/// any caller-side [`crate::picture::DecodeOptions`].
pub fn write_plus_picture_header(
    w: &mut BitWriter,
    fmt: H263SourceFormat,
    quant: u8,
    tr: u8,
    is_inter: bool,
    modes: PlusModes,
) -> Result<()> {
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if matches!(fmt, H263SourceFormat::Reserved110) {
        return Err(Error::NotImplemented);
    }
    // §5.1.1 — Picture Start Code (22 bits).
    w.write_bits(PSC_VALUE, 22);
    // §5.1.2 — Temporal Reference (8 bits).
    w.write_bits(tr as u32, 8);
    // §5.1.3 — PTYPE bits 1-5 (bit1 = 1, bit2 = 0, split-screen /
    // document-camera / freeze-release = 0), then bits 6-8 = "111":
    // the extended-PTYPE escape (§5.1.4).
    w.write_bit(true);
    w.write_bit(false);
    w.write_bit(false);
    w.write_bit(false);
    w.write_bit(false);
    w.write_bits(0b111, 3);
    // §5.1.4.1 — UFEP "001": the full optional part follows.
    w.write_bits(0b001, 3);
    // §5.1.4.2 — OPPTYPE (18 bits). Bit n sits at shift 18-n.
    let mut opptype: u32 = 0;
    opptype |= source_format_bits(fmt) << 15; // bits 1-3 — source format
    if modes.umv {
        opptype |= 1 << 13; // bit 5 — Annex D
    }
    if modes.advanced_prediction {
        opptype |= 1 << 11; // bit 7 — Annex F
    }
    if modes.advanced_intra {
        opptype |= 1 << 10; // bit 8 — Annex I
    }
    if modes.deblocking {
        opptype |= 1 << 9; // bit 9 — Annex J
    }
    if modes.slice_structured.is_some() {
        opptype |= 1 << 8; // bit 10 — Annex K
    }
    if modes.independent_segment_decoding {
        opptype |= 1 << 6; // bit 12 — Annex R
    }
    if modes.data_partitioned_slices {
        if modes.slice_structured.is_none() {
            // §V.3 — the SS mode shall be indicated whenever DPS is.
            return Err(Error::NotImplemented);
        }
        opptype |= 1 << 1; // bit 17 — Annex V
    }
    if modes.alt_inter_vlc {
        opptype |= 1 << 5; // bit 13 — Annex S
    }
    if modes.modified_quant {
        opptype |= 1 << 4; // bit 14 — Annex T
    }
    opptype |= 1 << 3; // bit 15 — "1", start-code-emulation guard
    w.write_bits(opptype, 18);
    // §5.1.4.3 — MPPTYPE (9 bits): picture type, RPR / RRU / RTYPE = 0,
    // reserved "00", bit 9 = "1".
    let mut mpptype: u32 = 0;
    if let Some((trb, dbquant)) = modes.improved_pb {
        // §5.1.4.3 — "010": Improved PB-frame (Annex M). Its P-part is a
        // P-picture (§M.1), so an INTRA request is contradictory.
        if !is_inter {
            return Err(Error::NotImplemented);
        }
        if trb == 0 || trb > 7 || dbquant > 3 {
            return Err(Error::BadPbTemporalReference);
        }
        mpptype |= 0b010 << 6;
    } else if is_inter {
        mpptype |= 0b001 << 6; // "001" P-picture ("000" is I)
    }
    if modes.rru {
        mpptype |= 1 << 4; // bit 5 — Annex Q Reduced-Resolution Update
    }
    mpptype |= 1; // bit 9 — start-code-emulation guard
    w.write_bits(mpptype, 9);
    // §5.1.4.7 / §5.1.20 — CPM, immediately after PLUSPTYPE; §5.1.21 —
    // PSBI (2 bits) follows a set CPM bit.
    match modes.cpm_psbi {
        Some(psbi) => {
            if psbi > 3 {
                return Err(Error::BadSliceSsbiCode);
            }
            w.write_bit(true);
            w.write_bits(psbi as u32, 2);
        }
        None => w.write_bit(false),
    }
    // §5.1.9 — UUI, present iff UMV mode on (UFEP = "001"): "1" =
    // motion-vector range per Tables D.1/D.2 (the crate's §D.2 range).
    if modes.umv {
        w.write_bit(true);
    }
    // §5.1.10 — SSS, present iff Slice Structured mode on.
    if let Some(sss) = modes.slice_structured {
        w.write_bit(sss.rectangular);
        w.write_bit(sss.arbitrary_order);
    }
    // §5.1.19 — PQUANT (5 bits).
    w.write_bits(quant as u32, 5);
    // §5.1.22 / §5.1.23 — TRB (3 bits, standard PCF) and DBQUANT
    // (2 bits) follow PQUANT when the picture is an Improved PB-frame.
    if let Some((trb, dbquant)) = modes.improved_pb {
        w.write_bits(trb as u32, 3);
        w.write_bits(dbquant as u32, 2);
    }
    // §5.1.24 — PEI = 0 (no PSUPP extension).
    w.write_bit(false);
    Ok(())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as a baseline H.263 **INTRA**
/// (I-) picture with a §5.2 **GOB header on every GOB after the
/// first**, carrying a per-GOB quantiser.
///
/// `gob_quant(gn)` supplies the quantiser for GOB `gn` (each must be in
/// `1..=31`): GOB 0 runs at `gob_quant(0)`, which doubles as the
/// picture-header PQUANT (§5.2.2 — GOB 0 never carries a header), and
/// every later GOB opens with GSTUF + GBSC + GN + GFID + GQUANT
/// priming its own quantiser (§5.2.6). Unlike the §5.3.6 DQUANT path,
/// GQUANT can jump anywhere in `1..=31` between GOBs — coarse-grained
/// rate control with resynchronisation points.
///
/// The output decodes through
/// [`crate::picture::decode_picture_no_gob0_header`] (whose
/// `gob_header_present` probe accepts the §5.2 optional headers) and
/// [`crate::picture::decode_sequence`].
pub fn encode_intra_picture_gobs<F>(frame: &YuvFrame, tr: u8, mut gob_quant: F) -> Result<Vec<u8>>
where
    F: FnMut(usize) -> u8,
{
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let rows_per_gob = crate::picture::PictureLayout::for_source_format(fmt)
        .ok_or(Error::NotImplemented)?
        .mb_rows_per_gob as usize;

    let pquant = gob_quant(0);
    if pquant == 0 || pquant > 31 {
        return Err(Error::InvalidQuantiser);
    }

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        pquant,
        tr,
        /* is_inter */ false,
        PtypeFlags::default(),
        None,
    );

    let gfid = tr & 0b11;
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    let mut quant = pquant;
    for mb_row in 0..mb_rows {
        if mb_row > 0 && mb_row % rows_per_gob == 0 {
            let gn = mb_row / rows_per_gob;
            quant = gob_quant(gn);
            if quant == 0 || quant > 31 {
                return Err(Error::InvalidQuantiser);
            }
            write_gob_header(&mut w, gn as u32, gfid, quant);
        }
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_intra_macroblock(
                &mut w, &mb, quant, /* write_cod */ false, /* picture_is_inter */ false,
            )?;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an Annex K **Slice
/// Structured** H.263+ **INTRA** picture: the §5.1.4 PLUSPTYPE header
/// signals the OPPTYPE bit-10 SS mode with the §5.1.10 SSS
/// free-running / sequential-order submode, and the picture body is a
/// sequence of §K.2 slices — the reduced-header first slice (§K.2.2,
/// running at the §5.1.19 PQUANT) followed by SSTUF + SSC-delimited
/// slices, each `mb_rows_per_slice` macroblock rows tall and carrying
/// its own §K.2.7 SQUANT.
///
/// `slice_quant(slice_index)` supplies each slice's quantiser
/// (`1..=31`; slice 0's value doubles as PQUANT). Like GOB-level
/// GQUANT, SQUANT can jump anywhere in `1..=31` between slices —
/// coarse-grained rate control with byte-aligned resynchronisation
/// points — but the slice layout is decoupled from the §4.2.1 GOB
/// grid: any `1..=mb_rows` row count per slice is legal.
///
/// Each slice is its own §6.1.1 / §I.3 video picture segment. INTRA
/// macroblocks carry no cross-macroblock prediction in the baseline
/// coding (absolute Table-15 INTRADC per block), so the segmentation
/// needs no encoder-side state; the emitted macroblock stream at a
/// given QUANT is bit-identical to the GOB-layer form's.
///
/// The output is self-describing and decodes through
/// [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`
/// (the Annex K routing engages from the wire).
pub fn encode_intra_picture_slices<F>(
    frame: &YuvFrame,
    tr: u8,
    mb_rows_per_slice: usize,
    mut slice_quant: F,
) -> Result<Vec<u8>>
where
    F: FnMut(usize) -> u8,
{
    use crate::plus_ptype::SliceStructuredSubmode;
    use crate::slice_header::{write_first_slice_header, write_slice_layer, SliceHeaderContext};

    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    if mb_rows_per_slice == 0 || mb_rows_per_slice > mb_rows {
        return Err(Error::UnsupportedPictureGeometry);
    }

    let pquant = slice_quant(0);
    if pquant == 0 || pquant > 31 {
        return Err(Error::InvalidQuantiser);
    }

    let sss = SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    };
    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        pquant,
        tr,
        /* is_inter */ false,
        PlusModes {
            slice_structured: Some(sss),
            ..PlusModes::default()
        },
    )?;

    // §K.2 slice-header context: free-running submode, CPM / RRU off.
    let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
    let gfid = tr & 0b11;

    // §K.2.2 — the slice following the picture header uses the reduced
    // form (no SSC / SQUANT; it runs at PQUANT) and starts at MBA 0.
    write_first_slice_header(&mut w, &ctx, 0, None)?;

    let mut quant = pquant;
    for mb_row in 0..mb_rows {
        if mb_row > 0 && mb_row % mb_rows_per_slice == 0 {
            let slice_index = mb_row / mb_rows_per_slice;
            quant = slice_quant(slice_index);
            if quant == 0 || quant > 31 {
                return Err(Error::InvalidQuantiser);
            }
            let mba = (mb_row * mb_cols) as u32;
            write_slice_layer(&mut w, &ctx, mba, quant, gfid, None)?;
        }
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_intra_macroblock(
                &mut w, &mb, quant, /* write_cod */ false, /* picture_is_inter */ false,
            )?;
        }
    }

    // §5.1.28 — PSTUF.
    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode an Annex K Slice-Structured H.263+ INTRA picture inside a
/// **Continuous Presence Multipoint Sub-Bitstream** (CPM = "1",
/// §5.1.20 / Annex C): the picture header carries the §5.1.21 PSBI
/// for `sub_bitstream` (`0..=3`) and every non-first slice header
/// carries the matching §K.2.4 SSBI Table-K.1 codeword. The slice
/// layout matches [`encode_intra_picture_slices`] (free-running,
/// row-aligned, per-slice SQUANT via `slice_quant`).
///
/// Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence` with
/// `DecodeOptions::default()` (the decoder stages the
/// single-Sub-Bitstream CPM case — every SSBI must match PSBI, which
/// this encoder guarantees).
pub fn encode_intra_picture_slices_cpm<F>(
    frame: &YuvFrame,
    tr: u8,
    mb_rows_per_slice: usize,
    sub_bitstream: u8,
    mut slice_quant: F,
) -> Result<Vec<u8>>
where
    F: FnMut(usize) -> u8,
{
    use crate::plus_ptype::SliceStructuredSubmode;
    use crate::slice_header::{
        write_first_slice_header, write_slice_layer_cpm, SliceHeaderContext,
    };

    if sub_bitstream > 3 {
        return Err(Error::BadSliceSsbiCode);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    if mb_rows_per_slice == 0 || mb_rows_per_slice > mb_rows {
        return Err(Error::UnsupportedPictureGeometry);
    }

    let pquant = slice_quant(0);
    if pquant == 0 || pquant > 31 {
        return Err(Error::InvalidQuantiser);
    }

    let sss = SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    };
    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        pquant,
        tr,
        /* is_inter */ false,
        PlusModes {
            slice_structured: Some(sss),
            cpm_psbi: Some(sub_bitstream),
            ..PlusModes::default()
        },
    )?;

    // §K.2 slice-header context with CPM on (SSBI on every non-first
    // slice header).
    let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), true, false);
    let gfid = tr & 0b11;
    write_first_slice_header(&mut w, &ctx, 0, None)?;

    let mut quant = pquant;
    for mb_row in 0..mb_rows {
        if mb_row > 0 && mb_row % mb_rows_per_slice == 0 {
            let slice_index = mb_row / mb_rows_per_slice;
            quant = slice_quant(slice_index);
            if quant == 0 || quant > 31 {
                return Err(Error::InvalidQuantiser);
            }
            let mba = (mb_row * mb_cols) as u32;
            write_slice_layer_cpm(&mut w, &ctx, sub_bitstream, mba, quant, gfid, None)?;
        }
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_intra_macroblock(
                &mut w, &mb, quant, /* write_cod */ false, /* picture_is_inter */ false,
            )?;
        }
    }

    // §5.1.28 — PSTUF.
    w.align_to_byte_zero();
    Ok(w.finish())
}

// ---------------------------------------------------------------------
// Annex Q — Reduced-Resolution Update encoders.
// ---------------------------------------------------------------------

/// Down-sample one 16 × 16 source region (top-left at `(x0, y0)` of a
/// `stride`-wide plane) to an 8 × 8 reduced-resolution block by 2 × 2
/// averaging with round-to-nearest — the encoder-side inverse of the
/// §Q.6 up-sampling (an encoder choice; §Q does not normify it).
fn rru_downsample_16(plane: &[u8], stride: usize, x0: usize, y0: usize) -> [i16; COEFFS_PER_BLOCK] {
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for j in 0..8 {
        for i in 0..8 {
            let a = plane[(y0 + 2 * j) * stride + x0 + 2 * i] as i32;
            let b = plane[(y0 + 2 * j) * stride + x0 + 2 * i + 1] as i32;
            let c = plane[(y0 + 2 * j + 1) * stride + x0 + 2 * i] as i32;
            let d = plane[(y0 + 2 * j + 1) * stride + x0 + 2 * i + 1] as i32;
            out[j * 8 + i] = ((a + b + c + d + 2) >> 2) as i16;
        }
    }
    out
}

/// As [`rru_downsample_16`] but over a signed 16 × 16 residual block.
fn rru_downsample_residual(res: &[i32; 256]) -> [i16; COEFFS_PER_BLOCK] {
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for j in 0..8 {
        for i in 0..8 {
            let a = res[(2 * j) * 16 + 2 * i];
            let b = res[(2 * j) * 16 + 2 * i + 1];
            let c = res[(2 * j + 1) * 16 + 2 * i];
            let d = res[(2 * j + 1) * 16 + 2 * i + 1];
            // Arithmetic-shift average (floors negatives consistently
            // with the Implementors' Guide §Q.6 rounding).
            out[j * 8 + i] = ((a + b + c + d + 2) >> 2) as i16;
        }
    }
    out
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex Q
/// Reduced-Resolution Update INTRA** picture: the PLUSPTYPE header
/// raises the §5.1.4.3 MPPTYPE RRU bit and the picture body is the
/// standard §5.3 / §5.4 macroblock syntax over the §Q.1 32 × 32
/// macroblock grid — each 16 × 16 source region is down-sampled to an
/// 8 × 8 reduced-resolution block, forward-transformed and quantised
/// by the exact baseline INTRA stage. The decoder up-samples per §Q.6
/// and runs the §Q.7 boundary filter, so the round-trip is inherently
/// low-passed (RRU trades detail for rate, §Q.1).
///
/// Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence` with
/// `DecodeOptions::default()`. The source dimensions must be one of
/// the five standard formats; for formats whose reference size is not
/// divisible by 32 (sub-QCIF / QCIF / 4CIF) the source is §Q.3-style
/// edge-extended to the coded size before down-sampling.
pub fn encode_intra_picture_rru(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    use crate::encoder_mb::{encode_intra_macroblock, MacroblockSamples};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let geo = crate::picture::rru_geometry_for_display(frame.luma_width, frame.luma_height);
    let ext = crate::picture::extend_frame_rru(frame, geo.0, geo.1);
    let (hc, vc) = geo;
    let mb_cols = hc / 32;
    let mb_rows = vc / 32;

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ false,
        PlusModes {
            rru: true,
            ..PlusModes::default()
        },
    )?;

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb_x = mb_col * 32;
            let mb_y = mb_row * 32;
            let c_x = mb_col * 16;
            let c_y = mb_row * 16;
            // Down-sample the four 16 × 16 luma regions and the two
            // 16 × 16 chroma regions to the reduced 8 × 8 blocks.
            let mut samples = MacroblockSamples {
                luma: [[0i16; COEFFS_PER_BLOCK]; 4],
                cb: [0i16; COEFFS_PER_BLOCK],
                cr: [0i16; COEFFS_PER_BLOCK],
            };
            for blk in 0..4 {
                let bx = mb_x + (blk % 2) * 16;
                let by = mb_y + (blk / 2) * 16;
                samples.luma[blk] = rru_downsample_16(&ext.y, hc, bx, by);
            }
            samples.cb = rru_downsample_16(&ext.cb, hc / 2, c_x, c_y);
            samples.cr = rru_downsample_16(&ext.cr, hc / 2, c_x, c_y);
            encode_intra_macroblock(
                &mut w, &samples, quant, /* write_cod */ false,
                /* picture_is_inter */ false,
            )?;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex Q
/// Reduced-Resolution Update INTER** picture predicted from
/// `reference` (the previously **decoded** reconstruction at the
/// reference size, §Q.2.3): 32 × 32 macroblocks with one §Q.4
/// pseudo-motion vector each — the search runs directly in the
/// pseudo-vector domain over `± search_pseudo_half` half-pel steps
/// around the pseudo-predictor, so every candidate expands to a legal
/// half-integer-or-zero actual vector — and per-16 × 16-sub-block
/// residuals down-sampled to 8 × 8, transformed and quantised by the
/// baseline INTER stage. A zero-vector macroblock with no surviving
/// residual is skipped.
///
/// Self-describing; decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence`. The
/// closed-loop GOP convention applies: pass the decoder's own
/// reconstruction of the previous picture as `reference` so encoder
/// and decoder never drift (the §Q.7 boundary filter is part of the
/// decoded reference).
pub fn encode_inter_picture_rru(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_pseudo_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_rru_impl(frame, reference, quant, tr, search_pseudo_half, false)
}

/// As [`encode_inter_picture_rru`], but with the **Annex D
/// Unrestricted Motion Vector mode** signalled alongside RRU: §Q.4 —
/// the pseudo motion vector is `pseudo-PC + difference` with the
/// difference coded per §D.2 as a **Table D.3** reversible pair, and
/// the §5.1.9 UUI `"1"` (Limited) selection bounds the *pseudo*
/// vectors by the Tables-D.1/D.2 picture-format range ("in the
/// Reduced-Resolution Update mode, the specified range applies to the
/// pseudo motion vectors" — so the actual motion reach is roughly
/// doubled). Self-describing; decodes with `DecodeOptions::default()`.
pub fn encode_inter_picture_rru_umv(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_pseudo_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_rru_impl(frame, reference, quant, tr, search_pseudo_half, true)
}

fn encode_inter_picture_rru_impl(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_pseudo_half: i32,
    umv: bool,
) -> Result<Vec<u8>> {
    use crate::motion::{rru_actual_mv, rru_pseudo_mv, MotionVector};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let (hc, vc) = crate::picture::rru_geometry_for_display(frame.luma_width, frame.luma_height);
    let ext_src = crate::picture::extend_frame_rru(frame, hc, vc);
    let ext_ref = crate::picture::extend_frame_rru(reference, hc, vc);
    let mb_cols = hc / 32;
    let mb_rows = vc / 32;

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PlusModes {
            rru: true,
            umv,
            ..PlusModes::default()
        },
    )?;

    let y_ref = crate::motion::RefPlane::new(&ext_ref.y, hc, vc);
    let cb_ref = crate::motion::RefPlane::new(&ext_ref.cb, hc / 2, vc / 2);
    let cr_ref = crate::motion::RefPlane::new(&ext_ref.cr, hc / 2, vc / 2);

    // §6.1.1 predictor grid over the actual vectors (replayed like the
    // decoder's).
    let mut grid = crate::encoder_motion::MvGrid::new(mb_cols, mb_rows);

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb_x = mb_col * 32;
            let mb_y = mb_row * 32;
            let c_x = mb_col * 16;
            let c_y = mb_row * 16;

            // §Q.4 — the predictor over actual vectors, converted to
            // the pseudo domain; the search walks pseudo candidates.
            let pc = grid.predict(mb_col, mb_row);
            let pseudo_pc = rru_pseudo_mv(pc);

            let sad_for = |actual: MotionVector| -> u32 {
                let mut sad = 0u32;
                for blk in 0..4 {
                    let bx = mb_x + (blk % 2) * 16;
                    let by = mb_y + (blk / 2) * 16;
                    let pred =
                        crate::picture::rru_motion_compensate_16_pub(&y_ref, bx, by, actual, 0);
                    for j in 0..16 {
                        for i in 0..16 {
                            let sv = ext_src.y[(by + j) * hc + bx + i] as i32;
                            sad += (sv - pred[j * 16 + i] as i32).unsigned_abs();
                        }
                    }
                }
                sad
            };

            let mut best_pseudo = MotionVector::new(0, 0);
            let mut best_cost = sad_for(MotionVector::new(0, 0));
            let lambda = 2 * quant as u32;
            // Pseudo-domain candidate window: the baseline [-32, 31]
            // half-pel window (§Q.4 item 2), widened to the
            // Tables-D.1/D.2 range under UMV (§D.2 — the range applies
            // to the pseudo vectors; UUI = "1" is emitted).
            let (px_min, px_max) = if umv {
                crate::motion::umv_plus_horizontal_range_half(frame.luma_width as u32)
            } else {
                (-32, 31)
            };
            let (py_min, py_max) = if umv {
                crate::motion::umv_plus_vertical_range_half(frame.luma_height as u32)
            } else {
                (-32, 31)
            };
            for dy in -search_pseudo_half..=search_pseudo_half {
                for dx in -search_pseudo_half..=search_pseudo_half {
                    let cand = MotionVector {
                        dx_half: pseudo_pc.dx_half + dx,
                        dy_half: pseudo_pc.dy_half + dy,
                    };
                    if !(px_min..=px_max).contains(&cand.dx_half)
                        || !(py_min..=py_max).contains(&cand.dy_half)
                    {
                        continue;
                    }
                    let actual = rru_actual_mv(cand);
                    let mvbits = (cand.dx_half - pseudo_pc.dx_half).unsigned_abs()
                        + (cand.dy_half - pseudo_pc.dy_half).unsigned_abs();
                    let cost = sad_for(actual) + lambda * mvbits;
                    if cost < best_cost {
                        best_cost = cost;
                        best_pseudo = cand;
                    }
                }
            }
            // The zero pseudo-vector must stay reachable through the
            // Table-14 wrap; every candidate the loop admitted is.
            let pseudo_mv = best_pseudo;
            let mv = rru_actual_mv(pseudo_mv);
            let chroma_vec = crate::motion::chroma_mv(mv);

            // Residuals per 16 × 16 sub-block, down-sampled to 8 × 8.
            let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(4);
            for blk in 0..4 {
                let bx = mb_x + (blk % 2) * 16;
                let by = mb_y + (blk / 2) * 16;
                let pred = crate::picture::rru_motion_compensate_16_pub(&y_ref, bx, by, mv, 0);
                let mut res = [0i32; 256];
                for j in 0..16 {
                    for i in 0..16 {
                        res[j * 16 + i] =
                            ext_src.y[(by + j) * hc + bx + i] as i32 - pred[j * 16 + i] as i32;
                    }
                }
                let reduced = rru_downsample_residual(&res);
                luma_enc.push(crate::encoder_block::encode_inter_block(&reduced, quant));
            }
            let mut chroma_enc: Vec<crate::encoder_block::EncodedInterBlock> =
                Vec::with_capacity(2);
            for (plane, rp) in [(&ext_src.cb, &cb_ref), (&ext_src.cr, &cr_ref)] {
                let pred =
                    crate::picture::rru_motion_compensate_16_pub(rp, c_x, c_y, chroma_vec, 0);
                let mut res = [0i32; 256];
                for j in 0..16 {
                    for i in 0..16 {
                        res[j * 16 + i] =
                            plane[(c_y + j) * (hc / 2) + c_x + i] as i32 - pred[j * 16 + i] as i32;
                    }
                }
                let reduced = rru_downsample_residual(&res);
                chroma_enc.push(crate::encoder_block::encode_inter_block(&reduced, quant));
            }

            let any_coeffs =
                luma_enc.iter().any(|e| e.has_coeffs) || chroma_enc.iter().any(|e| e.has_coeffs);
            let is_zero = mv.dx_half == 0 && mv.dy_half == 0;
            if !any_coeffs && is_zero {
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
                grid.set_zero_candidate(mb_col, mb_row);
                continue;
            }

            // Emit: COD + MCBPC(INTER) + CBPY + MVD (pseudo-domain
            // difference, Table 14 wrap) + blocks.
            let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                luma_enc[0].clone(),
                luma_enc[1].clone(),
                luma_enc[2].clone(),
                luma_enc[3].clone(),
            ];
            if umv {
                // §Q.4 / §D.2 — the Table D.3 difference is the plain
                // pseudo-domain `pseudo_mv − pseudo_pc`.
                let mvd = crate::macroblock::Mvd {
                    dx_half: (pseudo_mv.dx_half - pseudo_pc.dx_half) as i16,
                    dy_half: (pseudo_mv.dy_half - pseudo_pc.dy_half) as i16,
                };
                crate::encoder_mb::encode_inter_macroblock_umv_plus(
                    &mut w,
                    &luma_arr,
                    &chroma_enc[0],
                    &chroma_enc[1],
                    mvd,
                )?;
            } else {
                let mvd = crate::encoder_motion::mvd_for(pseudo_mv, pseudo_pc);
                crate::encoder_mb::encode_inter_macroblock(
                    &mut w,
                    &luma_arr,
                    &chroma_enc[0],
                    &chroma_enc[1],
                    mvd,
                )?;
            }
            grid.set_inter(mb_col, mb_row, mv);
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode an Annex K **Rectangular Slice submode** H.263+ INTRA
/// picture: the picture is tiled into full-height vertical stripes
/// `stripe_width_mbs` macroblocks wide (the right-most stripe takes
/// the remainder), each stripe one §K.2 slice whose header carries the
/// §K.2.8 SWI field (`SWI = width − 1`) and whose macroblocks run in
/// scanning order **within the rectangle** (§K.1 submode 1). Each
/// slice is its own §6.1.1 / §I.3 video picture segment.
///
/// With `arbitrary_order` set, the **Arbitrary Slice Ordering**
/// submode (§K.1 submode 2) is signalled too and the stripes are
/// emitted right-to-left — an out-of-order bitstream whose first
/// (reduced-header) slice is *not* the MBA-0 slice, exercising the
/// §K.1 "not necessarily the slice starting with macroblock 0" rule.
/// Baseline INTRA macroblocks carry no cross-macroblock prediction,
/// so the emitted stream reconstructs identically in either order.
///
/// The output is self-describing (SSS carries both submode bits) and
/// decodes through [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`.
pub fn encode_intra_picture_slices_rect(
    frame: &YuvFrame,
    quant: u8,
    tr: u8,
    stripe_width_mbs: usize,
    arbitrary_order: bool,
) -> Result<Vec<u8>> {
    encode_picture_slices_rect_impl(frame, None, quant, tr, stripe_width_mbs, arbitrary_order)
}

/// Encode an Annex K **Rectangular Slice submode** H.263+ INTER (P-)
/// picture predicted from `reference` with zero motion vectors — the
/// rectangular-stripe counterpart of the free-running
/// [`encode_inter_picture_slices`]. Stripe layout, SWI emission and
/// the optional Arbitrary Slice Ordering right-to-left emission match
/// [`encode_intra_picture_slices_rect`].
///
/// Every coded macroblock carries `MVD = (0, 0)`; since all
/// reconstructed vectors are zero, the §6.1.1 per-segment predictor is
/// zero at every macroblock regardless of the stripe scan order, so
/// the emitted MVD reconstructs exactly. A macroblock with no
/// surviving residual is skipped (COD = 1).
pub fn encode_inter_picture_slices_rect(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    stripe_width_mbs: usize,
    arbitrary_order: bool,
) -> Result<Vec<u8>> {
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    encode_picture_slices_rect_impl(
        frame,
        Some(reference),
        quant,
        tr,
        stripe_width_mbs,
        arbitrary_order,
    )
}

/// Shared body of the Rectangular-Slice stripe encoders.
/// `reference = None` encodes an INTRA picture; `Some(_)` a zero-MV
/// INTER picture.
fn encode_picture_slices_rect_impl(
    frame: &YuvFrame,
    reference: Option<&YuvFrame>,
    quant: u8,
    tr: u8,
    stripe_width_mbs: usize,
    arbitrary_order: bool,
) -> Result<Vec<u8>> {
    use crate::plus_ptype::SliceStructuredSubmode;
    use crate::slice_header::{write_first_slice_header, write_slice_layer, SliceHeaderContext};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    if stripe_width_mbs == 0 || stripe_width_mbs > mb_cols {
        return Err(Error::UnsupportedPictureGeometry);
    }

    let sss = SliceStructuredSubmode {
        rectangular: true,
        arbitrary_order,
    };
    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ reference.is_some(),
        PlusModes {
            slice_structured: Some(sss),
            ..PlusModes::default()
        },
    )?;

    let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
    let gfid = tr & 0b11;

    // Stripe geometry: full-height vertical rectangles.
    let stripe_count = mb_cols.div_ceil(stripe_width_mbs);
    let stripe_of = |i: usize| -> (usize, usize) {
        let col0 = i * stripe_width_mbs;
        (col0, stripe_width_mbs.min(mb_cols - col0))
    };
    // ASO: emit the stripes right-to-left; the first (reduced-header)
    // slice is then the right-most stripe.
    let order: Vec<usize> = if arbitrary_order {
        (0..stripe_count).rev().collect()
    } else {
        (0..stripe_count).collect()
    };

    for (emit_index, &stripe) in order.iter().enumerate() {
        let (col0, width) = stripe_of(stripe);
        let mba = col0 as u32;
        if emit_index == 0 {
            // §K.2.2 — the slice following the picture header uses the
            // reduced form (no SSC / SQUANT; it runs at PQUANT).
            write_first_slice_header(&mut w, &ctx, mba, Some(width as u32))?;
        } else {
            write_slice_layer(&mut w, &ctx, mba, quant, gfid, Some(width as u32))?;
        }
        // §K.1 — macroblocks in scanning order within the rectangle.
        for mb_row in 0..mb_rows {
            for mb_col in col0..col0 + width {
                let src = extract_macroblock(frame, mb_col, mb_row);
                match reference {
                    None => {
                        encode_intra_macroblock(
                            &mut w, &src, quant, /* write_cod */ false,
                            /* picture_is_inter */ false,
                        )?;
                    }
                    Some(reference) => {
                        let refmb = extract_macroblock(reference, mb_col, mb_row);
                        let luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = (0..4)
                            .map(|blk| {
                                let residual = residual_of(&src.luma[blk], &refmb.luma[blk]);
                                crate::encoder_block::encode_inter_block(&residual, quant)
                            })
                            .collect();
                        let cb_enc = crate::encoder_block::encode_inter_block(
                            &residual_of(&src.cb, &refmb.cb),
                            quant,
                        );
                        let cr_enc = crate::encoder_block::encode_inter_block(
                            &residual_of(&src.cr, &refmb.cr),
                            quant,
                        );
                        let any_coeffs = luma_enc.iter().any(|e| e.has_coeffs)
                            || cb_enc.has_coeffs
                            || cr_enc.has_coeffs;
                        if !any_coeffs {
                            crate::encoder_mb::encode_skipped_macroblock(&mut w);
                            continue;
                        }
                        let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                            luma_enc[0].clone(),
                            luma_enc[1].clone(),
                            luma_enc[2].clone(),
                            luma_enc[3].clone(),
                        ];
                        crate::encoder_mb::encode_inter_macroblock(
                            &mut w,
                            &luma_arr,
                            &cb_enc,
                            &cr_enc,
                            crate::macroblock::Mvd {
                                dx_half: 0,
                                dy_half: 0,
                            },
                        )?;
                    }
                }
            }
        }
    }

    // §5.1.28 — PSTUF.
    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode an Annex K **Slice Structured** + Annex I **Advanced INTRA
/// Coding** H.263+ INTRA picture: §K.2 slices every
/// `mb_rows_per_slice` macroblock rows, each macroblock coded with the
/// §I.2 per-macroblock INTRA_MODE decision and §I.3 DC/AC prediction —
/// where each slice is its own §I.3 **video picture segment**, so a
/// predictor candidate in a different slice is unavailable (the
/// page-78 availability rule), exactly mirroring the decoder's
/// per-segment `AicState`. Both SS and AIC are signalled in PLUSPTYPE;
/// the stream decodes with `DecodeOptions::default()`.
pub fn encode_intra_picture_slices_aic(
    frame: &YuvFrame,
    quant: u8,
    tr: u8,
    mb_rows_per_slice: usize,
) -> Result<Vec<u8>> {
    encode_intra_picture_slices_aic_core(frame, quant, tr, mb_rows_per_slice, false)
}

/// As [`encode_intra_picture_slices_aic`], with Annex T **Modified
/// Quantization** mode also active and signalled (the §T.3 chroma
/// `QUANT_C` step and the §T.4 / §T.5-rule-2 EXTENDED-ESCAPE on the
/// Table-I.2 VLC) — the AIC / MQ / Slice-Structured mode set of the
/// `advanced-intra-coding` conformance fixture, now producible as well
/// as decodable.
pub fn encode_intra_picture_slices_aic_mq(
    frame: &YuvFrame,
    quant: u8,
    tr: u8,
    mb_rows_per_slice: usize,
) -> Result<Vec<u8>> {
    encode_intra_picture_slices_aic_core(frame, quant, tr, mb_rows_per_slice, true)
}

/// Shared body of the Annex K AIC slice encoders: PLUSPTYPE (SS + AIC
/// (+ MQ)), reduced first slice header, then per-slice §K.2 headers
/// with the picture QUANT as every SQUANT; the AIC planning threads a
/// per-slice segment id through the neighbour grid.
fn encode_intra_picture_slices_aic_core(
    frame: &YuvFrame,
    quant: u8,
    tr: u8,
    mb_rows_per_slice: usize,
    modified_quant: bool,
) -> Result<Vec<u8>> {
    use crate::plus_ptype::SliceStructuredSubmode;
    use crate::slice_header::{write_first_slice_header, write_slice_layer, SliceHeaderContext};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    if mb_rows_per_slice == 0 || mb_rows_per_slice > mb_rows {
        return Err(Error::UnsupportedPictureGeometry);
    }

    // §T.3 — chrominance QUANT_C under Modified Quantization mode.
    let chroma_quant = if modified_quant {
        crate::annex_t::quant_c_from_quant(quant)?
    } else {
        quant
    };

    let sss = SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    };
    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ false,
        PlusModes {
            advanced_intra: true,
            modified_quant,
            slice_structured: Some(sss),
            ..PlusModes::default()
        },
    )?;

    let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
    let gfid = tr & 0b11;
    write_first_slice_header(&mut w, &ctx, 0, None)?;

    let mut grid = AicEncodeGrid::new(mb_cols, mb_rows);
    for mb_row in 0..mb_rows {
        let slice_index = mb_row / mb_rows_per_slice;
        if mb_row > 0 && mb_row % mb_rows_per_slice == 0 {
            let mba = (mb_row * mb_cols) as u32;
            write_slice_layer(&mut w, &ctx, mba, quant, gfid, None)?;
        }
        let params = AicParams {
            quant,
            chroma_quant,
            modified_quant,
            segment: slice_index as u32,
        };
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_choose_macroblock_aic(&mut w, &mb, params, None, mb_col, mb_row, &mut grid)?;
        }
    }

    // §5.1.28 — PSTUF.
    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as a baseline H.263 **INTRA**
/// (I-) picture at the given `quant` and 8-bit temporal reference `tr`.
///
/// The frame's luma dimensions must match a standard §4.1 source format
/// (sub-QCIF .. 16CIF); otherwise [`Error::NotImplemented`] is returned.
/// `quant` must be in `1..=31` ([`Error::InvalidQuantiser`] otherwise).
///
/// The returned bytes are a complete byte-aligned H.263 picture
/// (terminated with PSTUF zero-padding to the next byte boundary) that
/// decodes through [`crate::picture::decode_picture_no_gob0_header`].
pub fn encode_intra_picture(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ false,
        PtypeFlags::default(),
        None,
    );

    // §5.2.2 — macroblock stream, no GOB headers (single segment, GOB-0
    // elided, every later GOB header omitted). The macroblocks run
    // left-to-right, top-to-bottom over the whole picture.
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            // I-picture INTRA macroblock: no COD, Table 7 MCBPC.
            encode_intra_macroblock(
                &mut w, &mb, quant, /* write_cod */ false, /* picture_is_inter */ false,
            )?;
        }
    }

    // §5.1.28 — PSTUF: pad to the next byte boundary with zero bits so
    // the picture (and any following PSC) is byte-aligned.
    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex E Syntax-based
/// Arithmetic Coding INTRA** picture at the given `quant` and temporal
/// reference `tr`.
///
/// The picture header is the baseline §5.1 form with PTYPE bit 11
/// (SAC) set; the macroblock stream is the same single-segment §5.2.2
/// layout as [`encode_intra_picture`] but with every macroblock- and
/// block-layer symbol arithmetic-coded under its §E.7 / §E.8 model
/// through [`crate::sac::SacEncoder`] (the §E.5 stuffing rule keeps
/// the coded stream free of start-code emulation). The §E.6
/// `encoder_flush` terminates the arithmetic interval before the
/// closing §5.1.28 PSTUF.
///
/// The forward transform / quantisation stage is shared with the VLC
/// encoder, so the SAC picture reconstructs **byte-identically** to
/// the [`encode_intra_picture`] output of the same source — only the
/// entropy layer differs. Decodes through
/// [`crate::picture::decode_picture_sac`] (and [`decode_sequence`]).
pub fn encode_intra_picture_sac(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    use crate::sac::{encode_intra_macroblock_sac, SacEncoder};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ false,
        PtypeFlags {
            sac: true,
            ..PtypeFlags::default()
        },
        None,
    );

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    {
        // §E.5 — the zero-run counter spans the header/arithmetic
        // boundary: the header tail is PQUANT's trailing zeros + CPM=0
        // + PEI=0.
        let mut enc = SacEncoder::with_zero_run(&mut w, 2 + quant.trailing_zeros());
        for mb_row in 0..mb_rows {
            for mb_col in 0..mb_cols {
                let mb = extract_macroblock(frame, mb_col, mb_row);
                encode_intra_macroblock_sac(
                    &mut enc, &mb, quant, /* write_cod */ false,
                    /* picture_is_inter */ false,
                )?;
            }
        }
        // §E.6 — flush the arithmetic interval before the header/PSTUF
        // string that follows.
        enc.flush();
    }

    // §5.1.28 — PSTUF.
    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex E SAC INTER**
/// (P-) picture predicted from `reference` with zero motion vectors —
/// the arithmetic-coded mirror of [`encode_inter_picture`].
///
/// Each macroblock's residual against the co-located reference block
/// is transformed and quantised by the exact VLC-encoder stage; a
/// macroblock with no surviving residual is skipped (the COD = "1"
/// symbol). Decodes through [`crate::picture::decode_picture_sac`]
/// against the same reference, reconstructing byte-identically to the
/// [`encode_inter_picture`] output of the same source.
pub fn encode_inter_picture_sac(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
) -> Result<Vec<u8>> {
    use crate::sac::{encode_inter_macroblock_sac, encode_skipped_macroblock_sac, SacEncoder};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PtypeFlags {
            sac: true,
            ..PtypeFlags::default()
        },
        None,
    );

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    {
        // §E.5 — the zero-run counter spans the header/arithmetic
        // boundary: the header tail is PQUANT's trailing zeros + CPM=0
        // + PEI=0.
        let mut enc = SacEncoder::with_zero_run(&mut w, 2 + quant.trailing_zeros());
        for mb_row in 0..mb_rows {
            for mb_col in 0..mb_cols {
                let src = extract_macroblock(frame, mb_col, mb_row);
                let refmb = extract_macroblock(reference, mb_col, mb_row);

                let luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = (0..4)
                    .map(|blk| {
                        let residual = residual_of(&src.luma[blk], &refmb.luma[blk]);
                        crate::encoder_block::encode_inter_block(&residual, quant)
                    })
                    .collect();
                let cb_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cb, &refmb.cb),
                    quant,
                );
                let cr_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cr, &refmb.cr),
                    quant,
                );

                let any_coeffs =
                    luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;
                if !any_coeffs {
                    encode_skipped_macroblock_sac(&mut enc);
                    continue;
                }

                let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                    luma_enc[0].clone(),
                    luma_enc[1].clone(),
                    luma_enc[2].clone(),
                    luma_enc[3].clone(),
                ];
                encode_inter_macroblock_sac(
                    &mut enc,
                    &luma_arr,
                    &cb_enc,
                    &cr_enc,
                    crate::macroblock::Mvd {
                        dx_half: 0,
                        dy_half: 0,
                    },
                )?;
            }
        }
        enc.flush();
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex E SAC INTER**
/// (P-) picture with **motion estimation** — the arithmetic-coded
/// mirror of [`encode_inter_picture_motion`] (single-segment framing).
///
/// Each macroblock's vector is estimated by
/// [`crate::encoder_motion::estimate_motion`] around the §6.1.1 median
/// predictor (replayed through [`crate::encoder_motion::MvGrid`], so
/// every emitted MVD symbol reconstructs to exactly the searched
/// vector), the residual is computed against the motion-compensated
/// prediction, and the classic intra-refresh heuristic converts a
/// macroblock whose residual energy exceeds its own AC energy into a
/// P-picture INTRA macroblock. A zero-vector macroblock with no
/// surviving residual is skipped. Decodes through
/// [`crate::picture::decode_picture_sac`] and
/// [`crate::picture::decode_sequence`].
pub fn encode_inter_picture_motion_sac(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    use crate::sac::{
        encode_inter_macroblock_sac, encode_intra_macroblock_sac, encode_skipped_macroblock_sac,
        SacEncoder,
    };

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PtypeFlags {
            sac: true,
            ..PtypeFlags::default()
        },
        None,
    );

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let mut grid = crate::encoder_motion::MvGrid::new(mb_cols, mb_rows);
    let lambda = 2 * quant as u32;

    {
        // §E.5 — the zero-run counter spans the header/arithmetic
        // boundary: the header tail is PQUANT's trailing zeros + CPM=0
        // + PEI=0.
        let mut enc = SacEncoder::with_zero_run(&mut w, 2 + quant.trailing_zeros());
        for mb_row in 0..mb_rows {
            for mb_col in 0..mb_cols {
                let predictor = grid.predict(mb_col, mb_row);
                let mv = crate::encoder_motion::estimate_motion(
                    frame,
                    reference,
                    mb_col,
                    mb_row,
                    predictor,
                    search_half,
                    lambda,
                );

                let mb_x = mb_col * 16;
                let mb_y = mb_row * 16;
                let c_x = mb_col * 8;
                let c_y = mb_row * 8;
                let chroma_mv = crate::motion::chroma_mv(mv);

                let src = extract_macroblock(frame, mb_col, mb_row);
                let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> =
                    Vec::with_capacity(4);
                let mut inter_sad = 0u32;
                for blk in 0..4 {
                    let bx = mb_x + (blk % 2) * 8;
                    let by = mb_y + (blk / 2) * 8;
                    let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                    let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                    for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                        *d = p as i16;
                    }
                    for (&s, &p) in src.luma[blk].iter().zip(pred_i16.iter()) {
                        inter_sad += (s as i32 - p as i32).unsigned_abs();
                    }
                    let residual = residual_of(&src.luma[blk], &pred_i16);
                    luma_enc.push(crate::encoder_block::encode_inter_block(&residual, quant));
                }
                let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_mv);
                let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_mv);
                let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
                let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
                for i in 0..COEFFS_PER_BLOCK {
                    cb_pred_i[i] = cb_pred[i] as i16;
                    cr_pred_i[i] = cr_pred[i] as i16;
                }
                let cb_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cb, &cb_pred_i),
                    quant,
                );
                let cr_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cr, &cr_pred_i),
                    quant,
                );

                let any_coeffs =
                    luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;
                let is_zero_mv = mv.dx_half == 0 && mv.dy_half == 0;

                if !any_coeffs && is_zero_mv {
                    encode_skipped_macroblock_sac(&mut enc);
                    grid.set_zero_candidate(mb_col, mb_row);
                    continue;
                }

                // Intra-refresh heuristic — same decision rule as the
                // VLC motion encoder.
                let intra_sad: u32 = src
                    .luma
                    .iter()
                    .map(|blk| {
                        let mean = blk.iter().map(|&v| v as i32).sum::<i32>() / 64;
                        blk.iter()
                            .map(|&v| (v as i32 - mean).unsigned_abs())
                            .sum::<u32>()
                    })
                    .sum();
                if intra_sad + 256 < inter_sad {
                    encode_intra_macroblock_sac(
                        &mut enc, &src, quant, /* write_cod */ true,
                        /* picture_is_inter */ true,
                    )?;
                    grid.set_zero_candidate(mb_col, mb_row);
                    continue;
                }

                let mvd = crate::encoder_motion::mvd_for(mv, predictor);
                let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                    luma_enc[0].clone(),
                    luma_enc[1].clone(),
                    luma_enc[2].clone(),
                    luma_enc[3].clone(),
                ];
                encode_inter_macroblock_sac(&mut enc, &luma_arr, &cb_enc, &cr_enc, mvd)?;
                grid.set_inter(mb_col, mb_row, mv);
            }
        }
        enc.flush();
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Length of the run of `"0"` bits a fixed-length header string ends
/// in — the §E.5 stuffing-filter seed for the arithmetic segment that
/// follows it. `fields` lists the header fields after PTYPE in wire
/// order as `(value, bit_width)` pairs.
fn header_tail_zero_run(fields: &[(u32, u32)]) -> u32 {
    let mut run = 0u32;
    'outer: for &(value, bits) in fields.iter().rev() {
        for i in 0..bits {
            if (value >> i) & 1 == 0 {
                run += 1;
            } else {
                break 'outer;
            }
        }
    }
    run
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex E SAC + Annex F
/// Advanced Prediction** INTER picture — the arithmetic-coded mirror
/// of [`encode_inter_picture_ap`]: PTYPE signals both SAC (bit 11) and
/// AP (bit 12), every macroblock carries four §F.2 motion vectors
/// (MVD + MVD2-4 under the §E.7 `cumf_MVD` model) and the residual is
/// taken against the exact §F.3 OBMC prediction the decoder
/// reconstructs with.
///
/// The transform / quantiser stage is shared with the VLC AP encoder,
/// so the SAC and VLC AP pictures of the same source reconstruct
/// **byte-identically** through their respective drivers
/// ([`crate::picture::decode_picture_sac`] /
/// [`crate::picture::decode_picture`]-family).
pub fn encode_inter_picture_ap_sac(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    use crate::encoder_motion::{estimate_block_motion, mvd_for, Mv4Grid};
    use crate::motion::{chroma_mv_4mv, LumaBlockIndex, Mb4Mv, MotionVector, RemoteMv};
    use crate::sac::{encode_inter4v_macroblock_sac, SacEncoder};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let lambda = 2 * quant as u32;

    // ---- Pass 1: per-block motion estimation with §F.2 predictor
    // replay (identical to the VLC AP encoder). ----------------------
    let mut grid4 = Mv4Grid::new(mb_cols, mb_rows);
    let mut field: Vec<Mb4Mv> = Vec::with_capacity(mb_cols * mb_rows);
    let mut mvds_field: Vec<[crate::macroblock::Mvd; 4]> = Vec::with_capacity(mb_cols * mb_rows);
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mut cur: Mb4Mv = [MotionVector::new(0, 0); 4];
            let mut mvds = [crate::macroblock::Mvd {
                dx_half: 0,
                dy_half: 0,
            }; 4];
            for &blk in &LumaBlockIndex::ALL {
                let blk_i = blk.index();
                let bx = mb_col * 16 + (blk_i % 2) * 8;
                let by = mb_row * 16 + (blk_i / 2) * 8;
                let predictor = grid4.predict_block(mb_col, mb_row, blk, &cur);
                let mv =
                    estimate_block_motion(frame, reference, bx, by, predictor, search_half, lambda);
                cur[blk_i] = mv;
                mvds[blk_i] = mvd_for(mv, predictor);
            }
            grid4.set(mb_col, mb_row, cur);
            field.push(cur);
            mvds_field.push(mvds);
        }
    }

    // ---- Pass 2: §F.3 OBMC prediction + residual coding through the
    // arithmetic coder. ----------------------------------------------
    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PtypeFlags {
            sac: true,
            advanced_prediction: true,
            ..PtypeFlags::default()
        },
        None,
    );

    let y_ref = crate::motion::RefPlane::new(&reference.y, lw, lh);
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();

    {
        // §E.5 — the header tail is PQUANT's trailing zeros + CPM = 0
        // + PEI = 0.
        let mut enc = SacEncoder::with_zero_run(&mut w, 2 + quant.trailing_zeros());
        for mb_row in 0..mb_rows {
            for mb_col in 0..mb_cols {
                let idx = mb_row * mb_cols + mb_col;
                let cur = field[idx];
                let mvds = mvds_field[idx];
                let above = (mb_row > 0).then(|| field[idx - mb_cols]);
                let left = (mb_col > 0).then(|| field[idx - 1]);
                let right = (mb_col + 1 < mb_cols).then(|| field[idx + 1]);

                // §F.3 remote-vector tags per block (every macroblock
                // in this stream is coded INTER).
                let remote = |nb: Option<Mb4Mv>, cell: LumaBlockIndex| -> RemoteMv {
                    match nb {
                        Some(m) => RemoteMv::Vector(m[cell.index()]),
                        None => RemoteMv::Current,
                    }
                };
                let tags = |blk: LumaBlockIndex| -> (RemoteMv, RemoteMv, RemoteMv, RemoteMv) {
                    match blk {
                        LumaBlockIndex::B1 => (
                            remote(above, LumaBlockIndex::B3),
                            RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                            remote(left, LumaBlockIndex::B2),
                            RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
                        ),
                        LumaBlockIndex::B2 => (
                            remote(above, LumaBlockIndex::B4),
                            RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
                            RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                            remote(right, LumaBlockIndex::B1),
                        ),
                        LumaBlockIndex::B3 => (
                            RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                            RemoteMv::Current,
                            remote(left, LumaBlockIndex::B4),
                            RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
                        ),
                        LumaBlockIndex::B4 => (
                            RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
                            RemoteMv::Current,
                            RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                            remote(right, LumaBlockIndex::B3),
                        ),
                    }
                };

                let src = extract_macroblock(frame, mb_col, mb_row);
                let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> =
                    Vec::with_capacity(4);
                for &blk in &LumaBlockIndex::ALL {
                    let blk_i = blk.index();
                    let bx = mb_col * 16 + (blk_i % 2) * 8;
                    let by = mb_row * 16 + (blk_i / 2) * 8;
                    let (r_top, r_bot, s_left, s_right) = tags(blk);
                    let pred = crate::motion::obmc_predict_block(
                        &y_ref,
                        bx,
                        by,
                        cur[blk_i],
                        r_top,
                        r_bot,
                        s_left,
                        s_right,
                        crate::motion::RCONTROL_DEFAULT,
                    );
                    let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                    for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                        *d = p as i16;
                    }
                    let residual = residual_of(&src.luma[blk_i], &pred_i16);
                    luma_enc.push(crate::encoder_block::encode_inter_block(&residual, quant));
                }

                // §F.2 chroma: sum-of-four / Table F.1 vector, plain
                // half-pel motion compensation (no OBMC).
                let chroma_vec = chroma_mv_4mv(&cur);
                let c_x = mb_col * 8;
                let c_y = mb_row * 8;
                let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_vec);
                let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_vec);
                let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
                let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
                for i in 0..COEFFS_PER_BLOCK {
                    cb_pred_i[i] = cb_pred[i] as i16;
                    cr_pred_i[i] = cr_pred[i] as i16;
                }
                let cb_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cb, &cb_pred_i),
                    quant,
                );
                let cr_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cr, &cr_pred_i),
                    quant,
                );

                let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                    luma_enc[0].clone(),
                    luma_enc[1].clone(),
                    luma_enc[2].clone(),
                    luma_enc[3].clone(),
                ];
                encode_inter4v_macroblock_sac(&mut enc, &luma_arr, &cb_enc, &cr_enc, &mvds)?;
            }
        }
        enc.flush();
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode an Annex G **PB-frame** through the **Annex E arithmetic
/// coder** — the SAC mirror of [`encode_pb_picture`]: PTYPE signals
/// both SAC (bit 11) and PB-frames (bit 13), and every §5.3 /
/// Figure 10 field (COD, MCBPC, MODB, CBPB, CBPY, MVD, MVDB) plus the
/// twelve block payloads is an §E.7-modelled arithmetic symbol.
///
/// The P-part motion estimation, PREC reconstruction (§G.5) and the
/// §G.4 bidirectional B-prediction are byte-for-byte the VLC PB
/// encoder's, so the SAC and VLC PB-frames of the same sources
/// reconstruct **identically** in both parts. Decodes through
/// [`crate::picture::decode_pb_picture_sac`] and — inside an
/// elementary stream — [`crate::picture::decode_sequence`].
pub fn encode_pb_picture_sac(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<Vec<u8>> {
    use crate::pb_layer::{pb_b_predict_macroblock, pb_bquant, ModbPresence, PbBReferencePlanes};
    use crate::sac::{
        encode_cbpb_sac, encode_cbpy_sac, encode_cod, encode_mcbpc_p_sac, encode_modb_sac,
        encode_mvd_component_sac, encode_skipped_macroblock_sac, write_block_sac, SacEncoder,
    };

    if cfg.quant == 0 || cfg.quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if cfg.trb == 0 || cfg.trb > 7 || cfg.dbquant > 3 {
        return Err(Error::BadPbTemporalReference);
    }
    let trd = i32::from(tr_p.wrapping_sub(prev_tr));
    if trd == 0 || i32::from(cfg.trb) >= trd {
        return Err(Error::BadPbTemporalReference);
    }
    if p_source.luma_width != reference.luma_width
        || p_source.luma_height != reference.luma_height
        || b_source.luma_width != reference.luma_width
        || b_source.luma_height != reference.luma_height
    {
        return Err(Error::NotImplemented);
    }
    let fmt = source_format_for(p_source.luma_width, p_source.luma_height)
        .ok_or(Error::NotImplemented)?;

    let quant = cfg.quant;
    let bquant = pb_bquant(cfg.dbquant, quant);
    let trb = i32::from(cfg.trb);

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr_p,
        /* is_inter */ true,
        PtypeFlags {
            sac: true,
            ..PtypeFlags::default()
        },
        Some((cfg.trb, cfg.dbquant)),
    );

    let lw = p_source.luma_width;
    let lh = p_source.luma_height;
    let cw = p_source.chroma_width();
    let ch = p_source.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let mut grid = crate::encoder_motion::MvGrid::new(mb_cols, mb_rows);
    let lambda = 2 * quant as u32;

    let prev_y = crate::motion::RefPlane::new(&reference.y, lw, lh);
    let prev_cb = crate::motion::RefPlane::new(&reference.cb, cw, ch);
    let prev_cr = crate::motion::RefPlane::new(&reference.cr, cw, ch);

    {
        // §E.5 — the PB header tail is PQUANT + CPM = 0 + TRB +
        // DBQUANT + PEI = 0 (§E.6 string 1).
        let seed = header_tail_zero_run(&[
            (quant as u32, 5),
            (0, 1),
            (cfg.trb as u32, 3),
            (cfg.dbquant as u32, 2),
            (0, 1),
        ]);
        let mut enc = SacEncoder::with_zero_run(&mut w, seed);

        for mb_row in 0..mb_rows {
            for mb_col in 0..mb_cols {
                let mb_x = mb_col * 16;
                let mb_y = mb_row * 16;
                let c_x = mb_col * 8;
                let c_y = mb_row * 8;

                // ---- P-part: motion estimation + residual coding
                // (identical to the VLC PB encoder). ------------------
                let predictor = grid.predict(mb_col, mb_row);
                let mv = crate::encoder_motion::estimate_motion(
                    p_source,
                    reference,
                    mb_col,
                    mb_row,
                    predictor,
                    cfg.search_half,
                    lambda,
                );
                let chroma_mv = crate::motion::chroma_mv(mv);
                let src = extract_macroblock(p_source, mb_col, mb_row);

                let mut luma_pred: Vec<[u8; COEFFS_PER_BLOCK]> = Vec::with_capacity(4);
                let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> =
                    Vec::with_capacity(4);
                for blk in 0..4 {
                    let bx = mb_x + (blk % 2) * 8;
                    let by = mb_y + (blk / 2) * 8;
                    let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                    let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                    for (d, &pv) in pred_i16.iter_mut().zip(pred.iter()) {
                        *d = pv as i16;
                    }
                    luma_enc.push(crate::encoder_block::encode_inter_block(
                        &residual_of(&src.luma[blk], &pred_i16),
                        quant,
                    ));
                    luma_pred.push(pred);
                }
                let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_mv);
                let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_mv);
                let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
                let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
                for i in 0..COEFFS_PER_BLOCK {
                    cb_pred_i[i] = cb_pred[i] as i16;
                    cr_pred_i[i] = cr_pred[i] as i16;
                }
                let cb_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cb, &cb_pred_i),
                    quant,
                );
                let cr_enc = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cr, &cr_pred_i),
                    quant,
                );

                let any_p =
                    luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;
                let is_zero_mv = mv.dx_half == 0 && mv.dy_half == 0;

                // ---- PREC (§G.5). -----------------------------------
                let recon_block = |enc: &crate::encoder_block::EncodedInterBlock,
                                   pred: &[u8; COEFFS_PER_BLOCK],
                                   q: u8|
                 -> [u8; COEFFS_PER_BLOCK] {
                    if enc.has_coeffs {
                        let block = crate::block::H263Block {
                            coefficients: enc.scan,
                            tcoef_event_count: 0,
                            had_intradc: false,
                        };
                        crate::reconstruct_inter_block_with_prediction(&block, q, pred)
                    } else {
                        *pred
                    }
                };
                let mut prec_y = [0u8; 256];
                for blk in 0..4 {
                    let samples = recon_block(&luma_enc[blk], &luma_pred[blk], quant);
                    let ox = (blk % 2) * 8;
                    let oy = (blk / 2) * 8;
                    for j in 0..8 {
                        prec_y[(oy + j) * 16 + ox..(oy + j) * 16 + ox + 8]
                            .copy_from_slice(&samples[j * 8..j * 8 + 8]);
                    }
                }
                let prec_cb = recon_block(&cb_enc, &cb_pred, quant);
                let prec_cr = recon_block(&cr_enc, &cr_pred, quant);

                // ---- B-part: §G.4 + §G.5 prediction, MVDB = 0. ------
                let planes = PbBReferencePlanes {
                    prev_y,
                    prev_cb,
                    prev_cr,
                    prec_y: crate::motion::RefPlane::new(&prec_y, 16, 16),
                    prec_cb: crate::motion::RefPlane::new(&prec_cb, 8, 8),
                    prec_cr: crate::motion::RefPlane::new(&prec_cr, 8, 8),
                };
                let b_pred = pb_b_predict_macroblock(
                    &planes,
                    mb_x,
                    mb_y,
                    &[mv; 4],
                    None,
                    trb,
                    trd,
                    crate::motion::RCONTROL_DEFAULT,
                );

                let b_src = extract_macroblock(b_source, mb_col, mb_row);
                let mut b_enc: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(6);
                for blk in 0..4 {
                    let ox = (blk % 2) * 8;
                    let oy = (blk / 2) * 8;
                    let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                    for j in 0..8 {
                        for i in 0..8 {
                            pred_i16[j * 8 + i] = b_pred.luma[oy + j][ox + i] as i16;
                        }
                    }
                    b_enc.push(crate::encoder_block::encode_inter_block(
                        &residual_of(&b_src.luma[blk], &pred_i16),
                        bquant,
                    ));
                }
                let mut b_cb_pred = [0i16; COEFFS_PER_BLOCK];
                let mut b_cr_pred = [0i16; COEFFS_PER_BLOCK];
                for j in 0..8 {
                    for i in 0..8 {
                        b_cb_pred[j * 8 + i] = b_pred.cb[j][i] as i16;
                        b_cr_pred[j * 8 + i] = b_pred.cr[j][i] as i16;
                    }
                }
                b_enc.push(crate::encoder_block::encode_inter_block(
                    &residual_of(&b_src.cb, &b_cb_pred),
                    bquant,
                ));
                b_enc.push(crate::encoder_block::encode_inter_block(
                    &residual_of(&b_src.cr, &b_cr_pred),
                    bquant,
                ));
                let any_b = b_enc.iter().any(|e| e.has_coeffs);

                // ---- Skip / emit. -----------------------------------
                if !any_p && !any_b && is_zero_mv {
                    encode_skipped_macroblock_sac(&mut enc);
                    grid.set_zero_candidate(mb_col, mb_row);
                    continue;
                }

                // COD = 0; MCBPC (Table 8, INTER type 0).
                encode_cod(&mut enc, true);
                let mut cbpc = 0u8;
                if cb_enc.has_coeffs {
                    cbpc |= 0b10;
                }
                if cr_enc.has_coeffs {
                    cbpc |= 0b01;
                }
                encode_mcbpc_p_sac(&mut enc, crate::macroblock::MbType::Inter, cbpc)?;

                // §5.3.3 MODB + §5.3.4 CBPB.
                if any_b {
                    encode_modb_sac(&mut enc, ModbPresence::CbpbAndMvdb);
                    let mut cbpb = 0u8;
                    for (blk, e) in b_enc.iter().enumerate() {
                        if e.has_coeffs {
                            cbpb |= 1 << (6 - (blk + 1));
                        }
                    }
                    encode_cbpb_sac(&mut enc, cbpb);
                } else {
                    encode_modb_sac(&mut enc, ModbPresence::None);
                }

                // §5.3.5 CBPY (INTER complement).
                let mut cbpy_intra = 0u8;
                for (blk, e) in luma_enc.iter().enumerate() {
                    if e.has_coeffs {
                        cbpy_intra |= 1 << (3 - blk);
                    }
                }
                encode_cbpy_sac(&mut enc, cbpy_intra ^ 0b1111, false)?;

                // §5.3.7 MVD.
                let mvd = crate::encoder_motion::mvd_for(mv, predictor);
                encode_mvd_component_sac(&mut enc, mvd.dx_half)?;
                encode_mvd_component_sac(&mut enc, mvd.dy_half)?;

                // §5.3.9 MVDB = (0, 0) when MODB carries it.
                if any_b {
                    encode_mvd_component_sac(&mut enc, 0)?;
                    encode_mvd_component_sac(&mut enc, 0)?;
                }

                // §G.3 — six P-blocks, then six B-blocks.
                for e in luma_enc.iter() {
                    if e.has_coeffs {
                        write_block_sac(&mut enc, None, &e.scan, true, false)?;
                    }
                }
                if cb_enc.has_coeffs {
                    write_block_sac(&mut enc, None, &cb_enc.scan, true, false)?;
                }
                if cr_enc.has_coeffs {
                    write_block_sac(&mut enc, None, &cr_enc.scan, true, false)?;
                }
                for e in b_enc.iter() {
                    if e.has_coeffs {
                        write_block_sac(&mut enc, None, &e.scan, true, false)?;
                    }
                }

                grid.set_inter(mb_col, mb_row, mv);
            }
        }
        enc.flush();
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **extended-PTYPE (H.263+)
/// INTRA** picture: the §5.1.4 PLUSPTYPE header (UFEP `"001"`, no
/// optional modes) followed by the same single-segment §5.2.2
/// macroblock stream [`encode_intra_picture`] emits.
///
/// The output is self-describing on the wire and decodes through
/// [`crate::picture::decode_picture_layer`] and
/// [`crate::picture::decode_sequence`] (extended-PTYPE dispatch) with
/// `DecodeOptions::default()`.
pub fn encode_intra_picture_plus(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ false,
        PlusModes::default(),
    )?;

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_intra_macroblock(
                &mut w, &mb, quant, /* write_cod */ false, /* picture_is_inter */ false,
            )?;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encoder-side mirror of the decoder's `AicState` §I.3 neighbour grid.
///
/// Holds the reconstructed `RecC'(u,v)` block-position arrays keyed by
/// the same grid coordinates the decoder uses (`luma_block_grid_pos`):
/// luma at `2·mb_cols × 2·mb_rows`, chroma at `mb_cols × mb_rows`.
///
/// Mirrors the §I.3 page-78 availability rule at video-picture-segment
/// granularity: each committed macroblock records the segment id it was
/// encoded in, and a neighbour whose segment differs from the current
/// macroblock's collapses to [`Neighbour::None`] — exactly the
/// decoder's `AicBlockMeta` treatment. Single-segment pictures (the
/// header-less I-picture form) pass segment `0` everywhere, which
/// reduces to "every in-bounds encoded block is available".
struct AicEncodeGrid {
    luma_block_cols: usize,
    mb_cols: usize,
    /// `RecC'` per luma block; `None` until the block has been encoded.
    luma: Vec<Option<[i32; COEFFS_PER_BLOCK]>>,
    cb: Vec<Option<[i32; COEFFS_PER_BLOCK]>>,
    cr: Vec<Option<[i32; COEFFS_PER_BLOCK]>>,
    /// §6.1.1 / §I.3 video-picture-segment id per **macroblock**
    /// (`u32::MAX` = not yet encoded). All six blocks of a macroblock
    /// share one segment, so MB granularity suffices.
    segment: Vec<u32>,
}

impl AicEncodeGrid {
    fn new(mb_cols: usize, mb_rows: usize) -> Self {
        let luma_block_cols = 2 * mb_cols;
        let luma_block_rows = 2 * mb_rows;
        AicEncodeGrid {
            luma_block_cols,
            mb_cols,
            luma: vec![None; luma_block_cols * luma_block_rows],
            cb: vec![None; mb_cols * mb_rows],
            cr: vec![None; mb_cols * mb_rows],
            segment: vec![u32::MAX; mb_cols * mb_rows],
        }
    }

    /// Whether the macroblock owning luma-block-grid column `bx` / row
    /// `by` was encoded in `segment`.
    fn luma_block_in_segment(&self, bx: usize, by: usize, segment: u32) -> bool {
        self.segment[(by / 2) * self.mb_cols + (bx / 2)] == segment
    }

    /// The `RecA'` (above) / `RecB'` (left) luma neighbour arrays at grid
    /// position `(bx, by)` for a macroblock in `segment`, copied out so
    /// the caller can build [`Neighbour`] tags without borrowing the
    /// grid while it mutates. A neighbour from a different video picture
    /// segment is unavailable (§I.3 page-78 rule).
    fn luma_neighbours(
        &self,
        bx: usize,
        by: usize,
        segment: u32,
    ) -> (
        Option<[i32; COEFFS_PER_BLOCK]>,
        Option<[i32; COEFFS_PER_BLOCK]>,
    ) {
        let above = if by > 0 && self.luma_block_in_segment(bx, by - 1, segment) {
            self.luma[(by - 1) * self.luma_block_cols + bx]
        } else {
            None
        };
        let left = if bx > 0 && self.luma_block_in_segment(bx - 1, by, segment) {
            self.luma[by * self.luma_block_cols + (bx - 1)]
        } else {
            None
        };
        (above, left)
    }

    /// Chroma-plane analogue of [`Self::luma_neighbours`]: the grid is
    /// at macroblock granularity, so the segment check indexes
    /// `segments` directly.
    fn chroma_neighbours(
        planes: &[Option<[i32; COEFFS_PER_BLOCK]>],
        segments: &[u32],
        col: usize,
        row: usize,
        cols: usize,
        segment: u32,
    ) -> (
        Option<[i32; COEFFS_PER_BLOCK]>,
        Option<[i32; COEFFS_PER_BLOCK]>,
    ) {
        let above = if row > 0 && segments[(row - 1) * cols + col] == segment {
            planes[(row - 1) * cols + col]
        } else {
            None
        };
        let left = if col > 0 && segments[row * cols + (col - 1)] == segment {
            planes[row * cols + (col - 1)]
        } else {
            None
        };
        (above, left)
    }
}

/// Build a [`Neighbour`] tag borrowing `arr` when present.
fn neigh(arr: &Option<[i32; COEFFS_PER_BLOCK]>) -> Neighbour<'_> {
    match arr {
        Some(a) => Neighbour::Available(a),
        None => Neighbour::None,
    }
}

/// A fully-planned AIC INTRA macroblock: the six block plans plus the
/// chosen INTRA_MODE. Produced by [`plan_macroblock_aic`] without
/// mutating the grid, so several candidate modes can be planned and
/// compared before one is committed.
struct MbAicPlan {
    mode: IntraMode,
    luma: [AicBlockPlan; 4],
    cb: AicBlockPlan,
    cr: AicBlockPlan,
    /// Whether the block streams are emitted under Annex T Modified
    /// Quantization mode (§T.4 EXTENDED-ESCAPE enabled on the wire).
    modified_quant: bool,
}

/// The picture-constant §I / §T parameters threaded into every AIC
/// macroblock plan: the luma QUANT, the §T.3 chroma `QUANT_C` (equal to
/// QUANT outside Modified Quantization mode), and the MQ flag.
#[derive(Debug, Clone, Copy)]
struct AicParams {
    quant: u8,
    chroma_quant: u8,
    modified_quant: bool,
    /// §6.1.1 / §I.3 video-picture-segment id the macroblock is encoded
    /// in (0 for single-segment pictures; the slice index in Annex K
    /// Slice-Structured pictures).
    segment: u32,
}

/// Plan one AIC INTRA macroblock with a fixed INTRA_MODE, reading the
/// §I.3 neighbour predictors from `grid` **without** mutating it.
///
/// The four luma blocks chain intra-macroblock: Y2's above is Y0, Y3's
/// left is Y2, Y4's above/left are Y1/Y3 — those reconstructions come
/// from a local scratch (not yet in the grid), while the top row / left
/// column of the macroblock read the already-committed neighbours from
/// `grid` (Figure-5 order, matching the decoder). Chroma dequantises at
/// the §T.3 `QUANT_C` under Modified Quantization mode.
fn plan_macroblock_aic(
    grid: &AicEncodeGrid,
    mb: &MacroblockSamples,
    params: AicParams,
    mode: IntraMode,
    mb_col: usize,
    mb_row: usize,
) -> MbAicPlan {
    let AicParams {
        quant,
        chroma_quant,
        modified_quant,
        segment,
    } = params;
    // Local reconstructions of this macroblock's four luma blocks, filled
    // as we plan them so later blocks see earlier ones as neighbours.
    let mut local: [Option<[i32; COEFFS_PER_BLOCK]>; 4] = [None; 4];
    let mut luma: [Option<AicBlockPlan>; 4] = [None, None, None, None];

    for blk in 0..4 {
        let bx = 2 * mb_col + (blk & 1);
        let by = 2 * mb_row + (blk >> 1);
        // Above neighbour: an intra-macroblock block (Y0/Y1) when this is a
        // bottom-row block, else the committed grid block above.
        let above = if (blk >> 1) == 1 {
            local[blk - 2]
        } else {
            grid.luma_neighbours(bx, by, segment).0
        };
        // Left neighbour: an intra-macroblock block (Y0/Y2) when this is a
        // right-column block, else the committed grid block to the left.
        let left = if (blk & 1) == 1 {
            local[blk - 1]
        } else {
            grid.luma_neighbours(bx, by, segment).1
        };
        let plan = plan_intra_block_aic(
            &mb.luma[blk],
            mode,
            quant,
            neigh(&above),
            neigh(&left),
            modified_quant,
        );
        local[blk] = Some(plan.rec);
        luma[blk] = Some(plan);
    }

    let (cb_above, cb_left) = AicEncodeGrid::chroma_neighbours(
        &grid.cb,
        &grid.segment,
        mb_col,
        mb_row,
        grid.mb_cols,
        segment,
    );
    let cb = plan_intra_block_aic(
        &mb.cb,
        mode,
        chroma_quant,
        neigh(&cb_above),
        neigh(&cb_left),
        modified_quant,
    );

    let (cr_above, cr_left) = AicEncodeGrid::chroma_neighbours(
        &grid.cr,
        &grid.segment,
        mb_col,
        mb_row,
        grid.mb_cols,
        segment,
    );
    let cr = plan_intra_block_aic(
        &mb.cr,
        mode,
        chroma_quant,
        neigh(&cr_above),
        neigh(&cr_left),
        modified_quant,
    );

    MbAicPlan {
        mode,
        luma: luma.map(|p| p.unwrap()),
        cb,
        cr,
        modified_quant,
    }
}

/// Emit a planned AIC INTRA macroblock (the exact inverse of
/// `decode_intra_macroblock_aic`): MCBPC(INTRA) → §I.2 INTRA_MODE
/// (Table I.1) → CBPY → six Table-I.2 block streams (Y1..Y4, Cb, Cr).
/// The CBP bits carry each block's §I.3 coded/not-coded state (absorbed
/// INTRADC — a not-coded block reconstructs from the predictor alone).
fn write_macroblock_aic(w: &mut BitWriter, plan: &MbAicPlan) -> Result<()> {
    // §5.3.2 — MCBPC(INTRA): CBPC bit 0b10 = Cb coded, 0b01 = Cr coded.
    let mut cbpc = 0u8;
    if plan.cb.coded {
        cbpc |= 0b10;
    }
    if plan.cr.coded {
        cbpc |= 0b01;
    }
    write_mcbpc_i(w, MbType::Intra, cbpc)?;

    // §I.2 — INTRA_MODE (Table I.1), between MCBPC and CBPY.
    write_intra_mode(w, plan.mode);

    // §5.3.5 — CBPY (INTRA orientation): bit (3 - blk) set when luma
    // block blk is coded (in AIC the bit gates the whole block).
    let mut cbpy = 0u8;
    for (blk, p) in plan.luma.iter().enumerate() {
        if p.coded {
            cbpy |= 1 << (3 - blk);
        }
    }
    write_cbpy(w, cbpy)?;

    // §5.4 / §I.3 — six Table-I.2 block streams in order Y1..Y4, Cb, Cr.
    let mq = plan.modified_quant;
    for p in &plan.luma {
        write_intra_block_aic(w, p, mq)?;
    }
    write_intra_block_aic(w, &plan.cb, mq)?;
    write_intra_block_aic(w, &plan.cr, mq)?;
    Ok(())
}

/// Store a planned macroblock's reconstructions into `grid` so downstream
/// macroblocks pick them up as §I.3 neighbours, recording the video
/// picture segment the macroblock was encoded in.
fn commit_macroblock_aic(
    grid: &mut AicEncodeGrid,
    plan: &MbAicPlan,
    mb_col: usize,
    mb_row: usize,
    segment: u32,
) {
    for (blk, p) in plan.luma.iter().enumerate() {
        let bx = 2 * mb_col + (blk & 1);
        let by = 2 * mb_row + (blk >> 1);
        grid.luma[by * grid.luma_block_cols + bx] = Some(p.rec);
    }
    grid.cb[mb_row * grid.mb_cols + mb_col] = Some(plan.cb.rec);
    grid.cr[mb_row * grid.mb_cols + mb_col] = Some(plan.cr.rec);
    grid.segment[mb_row * grid.mb_cols + mb_col] = segment;
}

/// The estimated wire cost (in bits) of a planned AIC macroblock — used
/// by the per-macroblock INTRA_MODE decision to pick the cheapest mode.
/// Measured exactly by emitting into a scratch writer.
fn macroblock_aic_bit_cost(plan: &MbAicPlan) -> u64 {
    let mut scratch = BitWriter::new();
    // write_macroblock_aic only errors on unrepresentable fields, which a
    // valid plan never has; treat an error as "infinite" cost.
    if write_macroblock_aic(&mut scratch, plan).is_err() {
        return u64::MAX;
    }
    scratch.bit_position()
}

/// Plan, choose the cheapest INTRA_MODE for, emit, and commit one AIC
/// INTRA macroblock. When `fixed_mode` is `Some`, that mode is used;
/// otherwise all three §I.2 modes are planned and the one with the
/// smallest exact bit cost wins (§I.3 directional prediction pays off on
/// content with a dominant orientation).
fn encode_choose_macroblock_aic(
    w: &mut BitWriter,
    mb: &MacroblockSamples,
    params: AicParams,
    fixed_mode: Option<IntraMode>,
    mb_col: usize,
    mb_row: usize,
    grid: &mut AicEncodeGrid,
) -> Result<()> {
    let plan = match fixed_mode {
        Some(mode) => plan_macroblock_aic(grid, mb, params, mode, mb_col, mb_row),
        None => {
            let mut best: Option<(u64, MbAicPlan)> = None;
            for mode in [
                IntraMode::DcOnly,
                IntraMode::VerticalDcAc,
                IntraMode::HorizontalDcAc,
            ] {
                let candidate = plan_macroblock_aic(grid, mb, params, mode, mb_col, mb_row);
                let cost = macroblock_aic_bit_cost(&candidate);
                let improves = match &best {
                    None => true,
                    Some((best_cost, _)) => cost < *best_cost,
                };
                if improves {
                    best = Some((cost, candidate));
                }
            }
            best.unwrap().1
        }
    };
    write_macroblock_aic(w, &plan)?;
    commit_macroblock_aic(grid, &plan, mb_col, mb_row, params.segment);
    Ok(())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an Annex I **Advanced INTRA
/// Coding** (§I) INTRA picture: every macroblock is INTRA, coded with
/// the §I.2 per-macroblock INTRA_MODE, the §I.3 coefficient-domain DC/AC
/// prediction from reconstructed neighbours, the §I.3 modified
/// quantisation and the Table I.2 separate INTRA-coefficient VLC.
///
/// `quant` is the picture quantiser (`1..=31`), `tr` the §5.1.2 Temporal
/// Reference, and `mode` the INTRA_MODE applied to every macroblock (the
/// §I.3 DC-only / vertical / horizontal prediction + scan selection).
///
/// The picture header is a plain baseline INTRA header — the §I mode is
/// **not** signalled on the wire (a baseline PTYPE cannot carry it), so
/// the stream must be decoded with `DecodeOptions { aic: true, .. }`
/// (matching the crate's decoder convention). The output is a single
/// video-picture segment (§5.2.2 GOB-0 elided, no later GOB headers), so
/// every in-bounds neighbour is an available §I.3 predictor; the closed
/// loop (each block reconstructed through the exact decoder primitive)
/// keeps encoder and decoder bit-identical.
///
/// Decodes through
/// [`crate::picture::decode_picture_no_gob0_header`] /
/// [`crate::picture::decode_sequence`] with `aic` set.
pub fn encode_intra_picture_aic(
    frame: &YuvFrame,
    quant: u8,
    tr: u8,
    mode: IntraMode,
) -> Result<Vec<u8>> {
    encode_intra_picture_aic_core(frame, quant, tr, Some(mode), false, /* plus */ false)
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an Annex I **Advanced INTRA
/// Coding** (§I) INTRA picture with a **per-macroblock INTRA_MODE
/// decision**: each macroblock is planned under all three §I.2 modes
/// (DC-only / vertical / horizontal) and the mode with the smallest exact
/// wire cost is emitted, so §I.3 directional prediction is spent only
/// where a macroblock's content has a dominant orientation.
///
/// Otherwise identical to [`encode_intra_picture_aic`] (single
/// video-picture segment, closed-loop neighbour reconstruction, decode
/// with `DecodeOptions { aic: true, .. }`).
pub fn encode_intra_picture_aic_auto(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    encode_intra_picture_aic_core(frame, quant, tr, None, false, /* plus */ false)
}

/// Encode an Annex I **Advanced INTRA Coding** picture with Annex T
/// **Modified Quantization** mode active: chrominance coefficients
/// dequantise at the §T.3 `QUANT_C` step (Table T.2) and the §T.4 /
/// §T.5-rule-2 EXTENDED-ESCAPE mechanism widens the Table-I.2 LEVEL range
/// beyond ±127. Uses the per-macroblock INTRA_MODE decision.
///
/// The stream must be decoded with
/// `DecodeOptions { aic: true, modified_quant: true, .. }` — neither §I
/// nor §T is signalled on the baseline PTYPE wire (both are H.263+
/// PLUSPTYPE-gated modes; the on-wire form is
/// [`encode_intra_picture_aic_mq_plus`]).
pub fn encode_intra_picture_aic_mq(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    encode_intra_picture_aic_core(frame, quant, tr, None, true, /* plus */ false)
}

/// Encode an Annex I **Advanced INTRA Coding** picture with the §I mode
/// **signalled on the wire**: the §5.1.4 PLUSPTYPE header carries the
/// OPPTYPE bit-8 AIC flag, so the stream is self-describing and decodes
/// through [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`
/// (the decoder auto-activates §I from OPPTYPE). Uses the
/// per-macroblock INTRA_MODE decision; the macroblock stream is
/// bit-identical to [`encode_intra_picture_aic_auto`]'s.
pub fn encode_intra_picture_aic_plus(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    encode_intra_picture_aic_core(frame, quant, tr, None, false, /* plus */ true)
}

/// Encode an Annex I **Advanced INTRA Coding** + Annex T **Modified
/// Quantization** picture with both modes **signalled on the wire**
/// (PLUSPTYPE OPPTYPE bits 8 and 14). Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`.
/// The macroblock stream is bit-identical to
/// [`encode_intra_picture_aic_mq`]'s.
pub fn encode_intra_picture_aic_mq_plus(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    encode_intra_picture_aic_core(frame, quant, tr, None, true, /* plus */ true)
}

/// Shared body of the AIC INTRA picture encoders: picture header
/// (baseline PTYPE, or the §5.1.4 PLUSPTYPE form with the §I / §T mode
/// bits when `plus` is set), then an all-INTRA macroblock stream
/// (single video-picture segment). `fixed_mode` selects a picture-wide
/// INTRA_MODE or the per-macroblock rate decision (`None`);
/// `modified_quant` activates Annex T (§T.3 chroma `QUANT_C`, §T.4
/// EXTENDED-ESCAPE).
fn encode_intra_picture_aic_core(
    frame: &YuvFrame,
    quant: u8,
    tr: u8,
    fixed_mode: Option<IntraMode>,
    modified_quant: bool,
    plus: bool,
) -> Result<Vec<u8>> {
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    // §T.3 — chrominance QUANT_C (Table T.2) under Modified Quantization
    // mode; identical to QUANT otherwise.
    let chroma_quant = if modified_quant {
        crate::annex_t::quant_c_from_quant(quant)?
    } else {
        quant
    };
    let params = AicParams {
        quant,
        chroma_quant,
        modified_quant,
        segment: 0,
    };

    let mut w = BitWriter::new();
    if plus {
        write_plus_picture_header(
            &mut w,
            fmt,
            quant,
            tr,
            /* is_inter */ false,
            PlusModes {
                advanced_intra: true,
                modified_quant,
                ..PlusModes::default()
            },
        )?;
    } else {
        write_picture_header(
            &mut w,
            fmt,
            quant,
            tr,
            /* is_inter */ false,
            PtypeFlags::default(),
            None,
        );
    }

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    let mut grid = AicEncodeGrid::new(mb_cols, mb_rows);
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_choose_macroblock_aic(
                &mut w, &mb, params, fixed_mode, mb_col, mb_row, &mut grid,
            )?;
        }
    }

    // §5.1.28 — PSTUF.
    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Clamp a desired per-macroblock target quantiser into a §5.3.6
/// reachable step given the running QUANT: DQUANT can only move QUANT by
/// `{-2, -1, +1, +2}` per macroblock (and stays in `1..=31`). Returns the
/// reachable `(new_quant, dquant)` closest to `target` (no change yields
/// `dquant = None`).
fn reach_quant(current: u8, target: u8) -> (u8, Option<i8>) {
    let target = target.clamp(1, 31);
    if target == current {
        return (current, None);
    }
    let diff = (target as i16 - current as i16).clamp(-2, 2) as i8;
    let next = (current as i16 + diff as i16).clamp(1, 31) as u8;
    if next == current {
        (current, None)
    } else {
        (next, Some(next as i8 - current as i8))
    }
}

/// Encode a planar 4:2:0 [`YuvFrame`] as a baseline H.263 **INTRA**
/// (I-) picture with a **per-macroblock quantiser map** driven through
/// §5.3.6 DQUANT (the INTRA+Q macroblock type).
///
/// `pquant` is the picture-header PQUANT (the QUANT GOB 0 / the first
/// macroblock runs at). `target_quant(mb_col, mb_row)` returns the
/// desired quantiser for each macroblock; the encoder walks the
/// macroblocks in raster order tracking the running QUANT and emits a
/// DQUANT differential whenever the target differs and is reachable in
/// one `{-2,-1,+1,+2}` step (the §5.3.6 constraint), quantising that
/// macroblock at the reached QUANT. This is the foundation of rate
/// control: coarser quantisation where the eye is less sensitive, finer
/// where detail matters, all within the single video-picture segment.
///
/// The output decodes through [`crate::picture::decode_picture_no_gob0_header`].
pub fn encode_intra_picture_dquant<F>(
    frame: &YuvFrame,
    pquant: u8,
    tr: u8,
    mut target_quant: F,
) -> Result<Vec<u8>>
where
    F: FnMut(usize, usize) -> u8,
{
    if pquant == 0 || pquant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        pquant,
        tr,
        /* is_inter */ false,
        PtypeFlags::default(),
        None,
    );

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    let mut current = pquant;
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            let (next, dquant) = reach_quant(current, target_quant(mb_col, mb_row));
            crate::encoder_mb::encode_intra_macroblock_dq(
                &mut w, &mb, next, /* write_cod */ false, /* picture_is_inter */ false,
                dquant,
            )?;
            current = next;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Per-element signed residual `source − prediction` of two 8×8 sample
/// blocks (range roughly `[-255, 255]`).
pub(crate) fn residual_of(
    source: &[i16; COEFFS_PER_BLOCK],
    prediction: &[i16; COEFFS_PER_BLOCK],
) -> [i16; COEFFS_PER_BLOCK] {
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for ((o, &s), &p) in out.iter_mut().zip(source.iter()).zip(prediction.iter()) {
        *o = s - p;
    }
    out
}

/// Encode a planar 4:2:0 [`YuvFrame`] as a baseline H.263 **INTER**
/// (P-) picture predicted from `reference` (the previous reconstructed
/// frame) with **zero motion vectors**.
///
/// Every macroblock is INTER-coded with `MVD = 0`: because the encoder
/// emits a single video-picture segment and the §6.1.1 median predictor
/// over an all-zero-MV neighbourhood is zero, the decoder reconstructs
/// `MV = predictor + MVD = 0` for every macroblock, so the prediction is
/// the co-located reference block. The residual (`source − reference`)
/// is forward-transformed, quantised and coded per block; macroblocks
/// whose residual quantises away entirely (and whose chroma is also
/// zero) are emitted as **skipped** (COD = 1) for compactness.
///
/// Zero-MV P-pictures are exact for static content and a correct (if
/// not rate-optimal) encoding for moving content; true motion estimation
/// is a later milestone. `reference` must share the frame's dimensions.
pub fn encode_inter_picture(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
) -> Result<Vec<u8>> {
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PtypeFlags::default(),
        None,
    );

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let src = extract_macroblock(frame, mb_col, mb_row);
            let refmb = extract_macroblock(reference, mb_col, mb_row);

            // Residual = source − prediction (zero-MV prediction is the
            // co-located reference block).
            let luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = (0..4)
                .map(|blk| {
                    let residual = residual_of(&src.luma[blk], &refmb.luma[blk]);
                    crate::encoder_block::encode_inter_block(&residual, quant)
                })
                .collect();
            let cb_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cb, &refmb.cb), quant);
            let cr_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cr, &refmb.cr), quant);

            let any_coeffs =
                luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;

            if !any_coeffs {
                // No residual survives quantisation — emit a skipped MB
                // (COD = 1). The decoder copies the co-located reference
                // block with a zero MV, which is exactly our prediction.
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
                continue;
            }

            let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                luma_enc[0].clone(),
                luma_enc[1].clone(),
                luma_enc[2].clone(),
                luma_enc[3].clone(),
            ];
            crate::encoder_mb::encode_inter_macroblock(
                &mut w,
                &luma_arr,
                &cb_enc,
                &cr_enc,
                crate::macroblock::Mvd {
                    dx_half: 0,
                    dy_half: 0,
                },
            )?;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Fetch the 8×8 block at pixel origin `(x0, y0)` from a plane with the
/// given row stride into a flat `[u8; 64]`.
pub(crate) fn motion_compensated_block(
    plane: &[u8],
    width: usize,
    height: usize,
    x0: usize,
    y0: usize,
    mv: crate::motion::MotionVector,
) -> [u8; COEFFS_PER_BLOCK] {
    let rp = crate::motion::RefPlane::new(plane, width, height);
    crate::motion::motion_compensate_block(&rp, x0, y0, mv, crate::motion::RCONTROL_DEFAULT)
}

/// Encode a planar 4:2:0 [`YuvFrame`] as a baseline H.263 **INTER**
/// (P-) picture with **motion estimation** against `reference`.
///
/// Each macroblock's luma motion vector is estimated by
/// [`crate::encoder_motion::estimate_motion`] (SAD over a `search_half`
/// integer window + half-pel refinement, biased toward the §6.1.1
/// median predictor so static regions keep `MVD = 0`). The encoder
/// replicates the decoder's predictor bookkeeping via
/// [`crate::encoder_motion::MvGrid`], so the emitted `MVD` reconstructs
/// to exactly the chosen MV. The residual is computed against the
/// **motion-compensated** prediction (bit-identical to the decoder's),
/// forward-transformed, quantised and coded; a macroblock with a zero
/// MV and no surviving residual is skipped.
///
/// `reference` must share the frame's dimensions. The chroma vector is
/// derived from the luma MV by the decoder (Table 18), so the encoder
/// computes the chroma residual against the same Table-18 chroma
/// prediction.
pub fn encode_inter_picture_motion(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ false,
        InterFraming::Single,
        /* plus */ false,
        /* isd */ false,
    )
}

/// As [`encode_inter_picture_motion`], but with an **extended-PTYPE
/// (H.263+) picture header**: the §5.1.4 PLUSPTYPE form (UFEP `"001"`,
/// MPPTYPE picture type `"001"` P-picture, no optional modes) replaces
/// the baseline PTYPE. Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] (extended-PTYPE dispatch) with
/// `DecodeOptions::default()`; the macroblock stream is bit-identical
/// to [`encode_inter_picture_motion`]'s.
pub fn encode_inter_picture_plus(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ false,
        InterFraming::Single,
        /* plus */ true,
        /* isd */ false,
    )
}

/// Encode a motion-estimated **INTER** (P-) picture in Annex K **Slice
/// Structured** mode: an H.263+ P-picture whose body is §K.2 slices
/// every `mb_rows_per_slice` macroblock rows (free-running submode,
/// signalled in PLUSPTYPE), each slice its own §6.1.1 video picture
/// segment — the encoder replays the decoder's per-segment
/// median-predictor treatment (`MV2 = MV3 = MV1` at every slice-top
/// row), so every emitted MVD reconstructs to exactly the searched
/// vector. Otherwise identical to [`encode_inter_picture_motion`]
/// (SAD + half-pel estimation, skip / INTRA-refresh decisions).
///
/// Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`
/// (the Annex K slice routing engages from the wire).
pub fn encode_inter_picture_slices(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
    mb_rows_per_slice: usize,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ false,
        InterFraming::Slices {
            rows: mb_rows_per_slice,
        },
        /* plus */ true,
        /* isd */ false,
    )
}

/// As [`encode_inter_picture_motion`], but emitting a §5.2 **GOB
/// header** (GSTUF byte alignment + GBSC + GN + GFID + GQUANT) for
/// every GOB after the first — the resync-friendly stream shape. Each
/// GOB is then its own §6.1.1 video picture segment: the encoder
/// applies the rule-3 top-border predictor treatment at every GOB's
/// first macroblock row, exactly as the decoder does when
/// `gob_header_present` holds.
pub fn encode_inter_picture_gobs(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ false,
        InterFraming::GobHeaders,
        /* plus */ false,
        /* isd */ false,
    )
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an H.263 **INTER** (P-)
/// picture in the **Annex D Unrestricted Motion Vector mode**
/// (PLUSPTYPE absent): the PTYPE bit-10 UMV flag is raised and each
/// macroblock's motion vector is estimated over the extended §D.2
/// `[-31.5, 31.5]`-pixel range via
/// [`crate::encoder_motion::estimate_motion_umv`].
///
/// Two Annex D effects apply relative to the default mode:
///
/// * **§D.2 range extension** — a vector component beyond ±16 pixels is
///   reachable once the median predictor has grown past the first
///   column of Table 14 (the encoder's per-candidate reachability
///   filter guarantees every emitted MVD reconstructs to exactly the
///   searched vector through the decoder's §D.2 pair selection);
/// * **§D.1 motion vectors over picture boundaries** — vectors may
///   reference pixels outside the coded picture area; both the
///   encoder's prediction and the decoder's use the same edge-replicated
///   sampling, so the round-trip stays consistent.
///
/// Fast-moving content (beyond ±16 pixels/frame) reconstructs with far
/// less residual than the default mode can manage. `reference` must
/// share the frame's dimensions.
pub fn encode_inter_picture_umv(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ true,
        InterFraming::Single,
        /* plus */ false,
        /* isd */ false,
    )
}

/// As [`encode_inter_picture_umv`], but with an **extended-PTYPE
/// (H.263+) picture header**: the OPPTYPE bit-5 UMV flag signals Annex
/// D on the wire and the §5.1.9 UUI codeword `"1"` selects the
/// Tables-D.1/D.2 limited range.
///
/// Per §5.3.7 / §D.2, with PLUSPTYPE present the motion vector
/// differences are coded with the **Table D.3 reversible codes**
/// (single-valued `mv − predictor`, six-zero emulation-prevention
/// rule) instead of Table 14, and each component is bounded by the
/// Tables-D.1/D.2 picture-size range and the §D.1.1 15-pixel border
/// rule rather than the PLUSPTYPE-absent predictor-window rule — so
/// the macroblock stream deliberately differs from
/// [`encode_inter_picture_umv`]'s. Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`.
pub fn encode_inter_picture_umv_plus(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ true,
        InterFraming::Single,
        /* plus */ true,
        /* isd */ false,
    )
}

/// As [`encode_inter_picture_umv_plus`], but in Annex K **Slice
/// Structured** framing: §K.2 slices every `mb_rows_per_slice`
/// macroblock rows, each its own §6.1.1 video picture segment (the
/// per-slice rule-3 predictor treatment is replayed by the estimator),
/// with the Annex D UMV mode signalled in OPPTYPE + §5.1.9 UUI and the
/// motion vector differences coded per §5.3.7 / §D.2 with the
/// **Table D.3** reversible codes — the mode pairing of the staged
/// UMV+ conformance stream. Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`.
pub fn encode_inter_picture_umv_slices(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
    mb_rows_per_slice: usize,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ true,
        InterFraming::Slices {
            rows: mb_rows_per_slice,
        },
        /* plus */ true,
        /* isd */ false,
    )
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an H.263 **INTER** (P-)
/// picture in the **Annex F Advanced Prediction mode**: the PTYPE
/// bit-12 AP flag is raised and every macroblock is coded **INTER4V**
/// with four per-8×8-block motion vectors (§5.3.8 / §F.2), predicted
/// through the §F.3 **overlapped block motion compensation** blend the
/// decoder reconstructs with.
///
/// The encoder runs two passes:
///
/// 1. **Estimation** (raster order): each luminance block's vector is
///    searched around its §F.2 / Figure-F.1 median predictor
///    ([`crate::encoder_motion::Mv4Grid`] replays the decoder's
///    derivation, including the intra-macroblock candidate threading),
///    and the four MVDs are recorded.
/// 2. **Reconstruction + coding**: with the full motion field known,
///    each block's §F.3 OBMC prediction is computed exactly as the
///    decoder will (the right-half remote vectors read the macroblock
///    to the right — available now), the residual is transformed,
///    quantised and coded, and the macroblock is emitted.
///
/// Every macroblock is coded (no skip): a skipped macroblock would be
/// reconstructed by the decoder as a plain zero-vector copy rather
/// than the OBMC blend, which only coincides for all-zero
/// neighbourhoods — the always-coded form keeps encoder and decoder
/// predictions bit-identical everywhere at a small COD/MCBPC/MVD
/// overhead. Chrominance is predicted with the §F.2 / Table-F.1
/// sum-of-four vector (no OBMC). `reference` must share the frame's
/// dimensions.
pub fn encode_inter_picture_ap(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_ap_impl(frame, reference, quant, tr, search_half, false)
}

/// As [`encode_inter_picture_ap`], but combined with the **Annex D
/// Unrestricted Motion Vector mode on an extended-PTYPE (H.263+)
/// header**: OPPTYPE signals AP + UMV (+ §5.1.9 UUI `"1"`), each 8×8
/// block's vector is searched over the Tables-D.1/D.2 range under the
/// §D.1.1 border bound, and all four MVD pairs are written per §5.3.7
/// / §5.3.8 / §D.2 as **Table D.3** reversible codewords (the plain
/// single-valued `mv − predictor` per block — no reachability
/// constraint, so far-flung blocks are codable directly). The §F.3
/// OBMC prediction is unchanged. Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence` with
/// `DecodeOptions::default()`.
pub fn encode_inter_picture_ap_umv_plus(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_ap_impl(frame, reference, quant, tr, search_half, true)
}

fn encode_inter_picture_ap_impl(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
    umv_plus: bool,
) -> Result<Vec<u8>> {
    use crate::encoder_motion::{
        estimate_block_motion, estimate_block_motion_umv_plus, mvd_for, Mv4Grid,
    };
    use crate::motion::{chroma_mv_4mv, LumaBlockIndex, Mb4Mv, MotionVector, RemoteMv};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let lambda = 2 * quant as u32;

    // ---- Pass 1: per-block motion estimation with §F.2 predictor
    // replay. --------------------------------------------------------
    let mut grid4 = Mv4Grid::new(mb_cols, mb_rows);
    let mut field: Vec<Mb4Mv> = Vec::with_capacity(mb_cols * mb_rows);
    let mut mvds_field: Vec<[crate::macroblock::Mvd; 4]> = Vec::with_capacity(mb_cols * mb_rows);
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mut cur: Mb4Mv = [MotionVector::new(0, 0); 4];
            let mut mvds = [crate::macroblock::Mvd {
                dx_half: 0,
                dy_half: 0,
            }; 4];
            for &blk in &LumaBlockIndex::ALL {
                let blk_i = blk.index();
                let bx = mb_col * 16 + (blk_i % 2) * 8;
                let by = mb_row * 16 + (blk_i / 2) * 8;
                let predictor = grid4.predict_block(mb_col, mb_row, blk, &cur);
                let mv = if umv_plus {
                    estimate_block_motion_umv_plus(
                        frame,
                        reference,
                        bx,
                        by,
                        predictor,
                        search_half,
                        lambda,
                    )
                } else {
                    estimate_block_motion(frame, reference, bx, by, predictor, search_half, lambda)
                };
                cur[blk_i] = mv;
                mvds[blk_i] = if umv_plus {
                    // §D.2 with PLUSPTYPE: the Table D.3 difference is
                    // the plain single-valued `mv − predictor`.
                    crate::macroblock::Mvd {
                        dx_half: (mv.dx_half - predictor.dx_half) as i16,
                        dy_half: (mv.dy_half - predictor.dy_half) as i16,
                    }
                } else {
                    mvd_for(mv, predictor)
                };
            }
            grid4.set(mb_col, mb_row, cur);
            field.push(cur);
            mvds_field.push(mvds);
        }
    }

    // ---- Pass 2: §F.3 OBMC prediction + residual coding. -------------
    let mut w = BitWriter::new();
    if umv_plus {
        // H.263+ header: OPPTYPE AP + UMV, §5.1.9 UUI = "1" (Limited).
        write_plus_picture_header(
            &mut w,
            fmt,
            quant,
            tr,
            /* is_inter */ true,
            PlusModes {
                advanced_prediction: true,
                umv: true,
                ..PlusModes::default()
            },
        )?;
    } else {
        write_picture_header(
            &mut w,
            fmt,
            quant,
            tr,
            /* is_inter */ true,
            PtypeFlags {
                advanced_prediction: true,
                ..PtypeFlags::default()
            },
            None,
        );
    }

    let y_ref = crate::motion::RefPlane::new(&reference.y, lw, lh);
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let idx = mb_row * mb_cols + mb_col;
            let cur = field[idx];
            let mvds = mvds_field[idx];
            let above = (mb_row > 0).then(|| field[idx - mb_cols]);
            let left = (mb_col > 0).then(|| field[idx - 1]);
            let right = (mb_col + 1 < mb_cols).then(|| field[idx + 1]);

            // §F.3 remote-vector tags per block. Every macroblock in
            // this stream is coded INTER, so a present neighbour
            // contributes its actual vector; an off-picture neighbour
            // is replaced by the current vector, and the bottom
            // remotes of B3/B4 are always the current vector.
            let remote = |nb: Option<Mb4Mv>, cell: LumaBlockIndex| -> RemoteMv {
                match nb {
                    Some(m) => RemoteMv::Vector(m[cell.index()]),
                    None => RemoteMv::Current,
                }
            };
            let tags = |blk: LumaBlockIndex| -> (RemoteMv, RemoteMv, RemoteMv, RemoteMv) {
                match blk {
                    LumaBlockIndex::B1 => (
                        remote(above, LumaBlockIndex::B3),
                        RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                        remote(left, LumaBlockIndex::B2),
                        RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
                    ),
                    LumaBlockIndex::B2 => (
                        remote(above, LumaBlockIndex::B4),
                        RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
                        RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                        remote(right, LumaBlockIndex::B1),
                    ),
                    LumaBlockIndex::B3 => (
                        RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                        RemoteMv::Current,
                        remote(left, LumaBlockIndex::B4),
                        RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
                    ),
                    LumaBlockIndex::B4 => (
                        RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
                        RemoteMv::Current,
                        RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                        remote(right, LumaBlockIndex::B3),
                    ),
                }
            };

            let src = extract_macroblock(frame, mb_col, mb_row);
            let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(4);
            for &blk in &LumaBlockIndex::ALL {
                let blk_i = blk.index();
                let bx = mb_col * 16 + (blk_i % 2) * 8;
                let by = mb_row * 16 + (blk_i / 2) * 8;
                let (r_top, r_bot, s_left, s_right) = tags(blk);
                let pred = crate::motion::obmc_predict_block(
                    &y_ref,
                    bx,
                    by,
                    cur[blk_i],
                    r_top,
                    r_bot,
                    s_left,
                    s_right,
                    crate::motion::RCONTROL_DEFAULT,
                );
                let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                    *d = p as i16;
                }
                let residual = residual_of(&src.luma[blk_i], &pred_i16);
                luma_enc.push(crate::encoder_block::encode_inter_block(&residual, quant));
            }

            // §F.2 chroma: sum-of-four / Table F.1 vector, plain
            // half-pel motion compensation (no OBMC).
            let chroma_vec = chroma_mv_4mv(&cur);
            let c_x = mb_col * 8;
            let c_y = mb_row * 8;
            let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_vec);
            let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_vec);
            let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
            let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
            for i in 0..COEFFS_PER_BLOCK {
                cb_pred_i[i] = cb_pred[i] as i16;
                cr_pred_i[i] = cr_pred[i] as i16;
            }
            let cb_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cb, &cb_pred_i), quant);
            let cr_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cr, &cr_pred_i), quant);

            let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                luma_enc[0].clone(),
                luma_enc[1].clone(),
                luma_enc[2].clone(),
                luma_enc[3].clone(),
            ];
            if umv_plus {
                crate::encoder_mb::encode_inter4v_macroblock_umv_plus(
                    &mut w, &luma_arr, &cb_enc, &cr_enc, &mvds,
                )?;
            } else {
                crate::encoder_mb::encode_inter4v_macroblock(
                    &mut w, &luma_arr, &cb_enc, &cr_enc, &mvds,
                )?;
            }
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex K Slice-Structured
/// plus Annex F Advanced Prediction** H.263+ P-picture: the PLUSPTYPE
/// header signals both modes, the picture body is §K.2 free-running
/// slices every `mb_rows_per_slice` macroblock rows, and every
/// macroblock carries four §F.2 motion vectors predicted through the
/// §F.3 OBMC blend.
///
/// Both §K.1 confinement rules are replayed on the encode side: the
/// §6.1.1 / §F.2 candidate predictors treat the slice top row as a
/// rule-3 border ([`crate::encoder_motion::Mv4Grid::with_row_segments`])
/// and the §F.3 remote vectors of blocks in a different slice are
/// substituted with the current block's vector — exactly what the
/// slice decode driver reconstructs with, so the round-trip is exact
/// at the prediction level.
///
/// Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence` with
/// `DecodeOptions::default()`.
pub fn encode_inter_picture_ap_slices(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
    mb_rows_per_slice: usize,
) -> Result<Vec<u8>> {
    use crate::encoder_motion::{estimate_block_motion, mvd_for, Mv4Grid};
    use crate::motion::{chroma_mv_4mv, LumaBlockIndex, Mb4Mv, MotionVector, RemoteMv};
    use crate::plus_ptype::SliceStructuredSubmode;
    use crate::slice_header::{write_first_slice_header, write_slice_layer, SliceHeaderContext};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    if mb_rows_per_slice == 0 || mb_rows_per_slice > mb_rows {
        return Err(Error::UnsupportedPictureGeometry);
    }
    let lambda = 2 * quant as u32;

    // ---- Pass 1: per-block motion estimation with §F.2 predictor
    // replay under the per-slice §6.1.1 segmentation. -----------------
    let mut grid4 = Mv4Grid::with_row_segments(mb_cols, mb_rows, mb_rows_per_slice);
    let mut field: Vec<Mb4Mv> = Vec::with_capacity(mb_cols * mb_rows);
    let mut mvds_field: Vec<[crate::macroblock::Mvd; 4]> = Vec::with_capacity(mb_cols * mb_rows);
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mut cur: Mb4Mv = [MotionVector::new(0, 0); 4];
            let mut mvds = [crate::macroblock::Mvd {
                dx_half: 0,
                dy_half: 0,
            }; 4];
            for &blk in &LumaBlockIndex::ALL {
                let blk_i = blk.index();
                let bx = mb_col * 16 + (blk_i % 2) * 8;
                let by = mb_row * 16 + (blk_i / 2) * 8;
                let predictor = grid4.predict_block(mb_col, mb_row, blk, &cur);
                let mv =
                    estimate_block_motion(frame, reference, bx, by, predictor, search_half, lambda);
                cur[blk_i] = mv;
                mvds[blk_i] = mvd_for(mv, predictor);
            }
            grid4.set(mb_col, mb_row, cur);
            field.push(cur);
            mvds_field.push(mvds);
        }
    }

    // ---- Pass 2: §F.3 OBMC prediction (slice-confined remotes) +
    // residual coding under the §K.2 slice framing. -------------------
    let sss = SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    };
    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PlusModes {
            advanced_prediction: true,
            slice_structured: Some(sss),
            ..PlusModes::default()
        },
    )?;
    let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
    write_first_slice_header(&mut w, &ctx, 0, None)?;
    let gfid = tr & 0b11;

    let y_ref = crate::motion::RefPlane::new(&reference.y, lw, lh);
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();

    for mb_row in 0..mb_rows {
        if mb_row > 0 && mb_row % mb_rows_per_slice == 0 {
            let mba = (mb_row * mb_cols) as u32;
            write_slice_layer(&mut w, &ctx, mba, quant, gfid, None)?;
        }
        // §F.3 slice rule — an above / below neighbour in a different
        // slice substitutes the current vector; with row-aligned
        // slices the left / right neighbours are always same-slice.
        let above_in_slice = mb_row % mb_rows_per_slice != 0;
        let below_in_slice = (mb_row + 1) % mb_rows_per_slice != 0;
        for mb_col in 0..mb_cols {
            let idx = mb_row * mb_cols + mb_col;
            let cur = field[idx];
            let mvds = mvds_field[idx];
            let above = (mb_row > 0 && above_in_slice).then(|| field[idx - mb_cols]);
            let left = (mb_col > 0).then(|| field[idx - 1]);
            let right = (mb_col + 1 < mb_cols).then(|| field[idx + 1]);
            // §F.3 last-sentence rule makes the B3/B4 bottom remotes
            // Current regardless, so `below_in_slice` only documents
            // that no additional case arises for row-aligned slices.
            let _ = below_in_slice;

            let remote = |nb: Option<Mb4Mv>, cell: LumaBlockIndex| -> RemoteMv {
                match nb {
                    Some(m) => RemoteMv::Vector(m[cell.index()]),
                    None => RemoteMv::Current,
                }
            };
            let tags = |blk: LumaBlockIndex| -> (RemoteMv, RemoteMv, RemoteMv, RemoteMv) {
                match blk {
                    LumaBlockIndex::B1 => (
                        remote(above, LumaBlockIndex::B3),
                        RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                        remote(left, LumaBlockIndex::B2),
                        RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
                    ),
                    LumaBlockIndex::B2 => (
                        remote(above, LumaBlockIndex::B4),
                        RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
                        RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                        remote(right, LumaBlockIndex::B1),
                    ),
                    LumaBlockIndex::B3 => (
                        RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                        RemoteMv::Current,
                        remote(left, LumaBlockIndex::B4),
                        RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
                    ),
                    LumaBlockIndex::B4 => (
                        RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
                        RemoteMv::Current,
                        RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                        remote(right, LumaBlockIndex::B3),
                    ),
                }
            };

            let src = extract_macroblock(frame, mb_col, mb_row);
            let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(4);
            for &blk in &LumaBlockIndex::ALL {
                let blk_i = blk.index();
                let bx = mb_col * 16 + (blk_i % 2) * 8;
                let by = mb_row * 16 + (blk_i / 2) * 8;
                let (r_top, r_bot, s_left, s_right) = tags(blk);
                let pred = crate::motion::obmc_predict_block(
                    &y_ref,
                    bx,
                    by,
                    cur[blk_i],
                    r_top,
                    r_bot,
                    s_left,
                    s_right,
                    crate::motion::RCONTROL_DEFAULT,
                );
                let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                    *d = p as i16;
                }
                let residual = residual_of(&src.luma[blk_i], &pred_i16);
                luma_enc.push(crate::encoder_block::encode_inter_block(&residual, quant));
            }

            let chroma_vec = chroma_mv_4mv(&cur);
            let c_x = mb_col * 8;
            let c_y = mb_row * 8;
            let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_vec);
            let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_vec);
            let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
            let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
            for i in 0..COEFFS_PER_BLOCK {
                cb_pred_i[i] = cb_pred[i] as i16;
                cr_pred_i[i] = cr_pred[i] as i16;
            }
            let cb_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cb, &cb_pred_i), quant);
            let cr_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cr, &cr_pred_i), quant);

            let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                luma_enc[0].clone(),
                luma_enc[1].clone(),
                luma_enc[2].clone(),
                luma_enc[3].clone(),
            ];
            crate::encoder_mb::encode_inter4v_macroblock(
                &mut w, &luma_arr, &cb_enc, &cr_enc, &mvds,
            )?;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Segment framing for the motion-estimated INTER picture encode.
#[derive(Debug, Clone, Copy)]
enum InterFraming {
    /// Single video-picture segment (§5.2.2 GOB-0 elided, no later GOB
    /// headers).
    Single,
    /// §5.2 GOB header on every GOB after the first, each GOB its own
    /// §6.1.1 segment (baseline-PTYPE stream shape).
    GobHeaders,
    /// Annex K free-running slices every `rows` macroblock rows, each
    /// slice its own §6.1.1 segment. Requires the PLUSPTYPE header
    /// (`plus = true`) — the SS mode bit lives in OPPTYPE.
    Slices { rows: usize },
}

/// Shared motion-estimated INTER picture encode: the default-mode
/// (§6.1.1 wrap, `umv = false`) and Annex D UMV (`umv = true`) paths
/// differ only in the PTYPE bit-10 flag, the estimator range and the
/// MVD derivation; `framing` selects the single-segment, §5.2
/// every-GOB-header or Annex K slice stream shape (with the matching
/// per-segment predictor treatment).
#[allow(clippy::too_many_arguments)]
/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex R Independent
/// Segment Decoding** H.263+ **INTRA** picture: the §5.1.4 PLUSPTYPE
/// header raises the OPPTYPE bit-12 ISD flag and the picture body
/// carries a §5.2 GOB header on every GOB after the first, so each
/// GOB is one video picture segment (§R.2) with a byte-aligned
/// resynchronisation point. An INTRA picture reads no reference, so
/// the ISD emission reduces to the mode bit + the per-GOB headers
/// (whose constant per-picture layout satisfies §R.3.2 for the
/// following P-pictures); the AIC / MV predictor confinement to
/// segments is inherent in the per-GOB header segmentation.
///
/// Decodes through [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`].
pub fn encode_intra_picture_isd(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let rows_per_gob = crate::picture::PictureLayout::for_source_format(fmt)
        .ok_or(Error::NotImplemented)?
        .mb_rows_per_gob as usize;

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ false,
        PlusModes {
            independent_segment_decoding: true,
            ..PlusModes::default()
        },
    )?;

    let gfid = tr & 0b11;
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    for mb_row in 0..mb_rows {
        // §R.2 — a non-empty GOB header at the top of every GOB after
        // GOB 0 makes each GOB its own video picture segment.
        if mb_row > 0 && mb_row % rows_per_gob == 0 {
            write_gob_header(&mut w, (mb_row / rows_per_gob) as u32, gfid, quant);
        }
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_intra_macroblock(
                &mut w, &mb, quant, /* write_cod */ false, /* picture_is_inter */ false,
            )?;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex R Independent
/// Segment Decoding** H.263+ **INTER** (P-) picture predicted from
/// `reference`.
///
/// The PLUSPTYPE header raises the OPPTYPE bit-12 ISD flag together
/// with the bit-5 **Annex D UMV** flag (Table D.3 motion coding under
/// the §5.1.9 UUI Tables-D.1/D.2 range), and a §5.2 GOB header opens
/// every GOB after the first — each GOB one video picture segment.
/// UMV matters: §R.2 rule 4 prohibits motion vectors that reference
/// data outside the current segment *unless* Annex D / F / J / O is
/// in use, in which case "the borders of the current video picture
/// segment in the prior picture are extrapolated as described in
/// Annex D". With UMV on, this encoder searches each segment against
/// an edge-replicated band view of the reference
/// ([`band_replicated_reference`]) — byte-identical to the decoder's
/// banded fetch — so over-boundary vectors are both legal and
/// closed-loop exact.
///
/// §R.3.2 (segment shapes constant from picture to picture) holds by
/// construction: every picture this pair of entry points emits uses
/// the uniform one-header-per-GOB segmentation.
///
/// Decodes through [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`]; predict the next picture from
/// the decoder's reconstruction for a drift-free closed loop.
pub fn encode_inter_picture_isd(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
) -> Result<Vec<u8>> {
    encode_inter_picture_motion_impl(
        frame,
        reference,
        quant,
        tr,
        search_half,
        /* umv */ true,
        InterFraming::GobHeaders,
        /* plus */ true,
        /* isd */ true,
    )
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex V Data-Partitioned
/// Slice** H.263+ **INTRA** picture: the PLUSPTYPE header raises the
/// OPPTYPE bit-10 Slice Structured and bit-17 DPS flags, and each
/// free-running slice of `mb_rows_per_slice` macroblock rows carries
/// the §V.2 partitioned layout — the Table V.1 RVLC COD + MCBPC
/// headers for the whole slice, the §V.2.2 Header Marker, and the
/// coefficient layer (CBPY + INTRADC/TCOEF block data per §V.2.6; an
/// INTRA picture has no motion-vector partition, so no §V.2.5 MVM is
/// emitted).
///
/// Decodes through [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`].
pub fn encode_intra_picture_dps(
    frame: &YuvFrame,
    quant: u8,
    tr: u8,
    mb_rows_per_slice: usize,
) -> Result<Vec<u8>> {
    use crate::annex_v::{
        write_dps_mb_header, DpsMbHeader, HEADER_MARKER, HEADER_MARKER_BITS, TABLE_V1_INTRA,
    };
    use crate::encoder_vlc::write_cbpy;
    use crate::slice_header::{write_first_slice_header, write_slice_layer, SliceHeaderContext};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;
    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    if mb_rows_per_slice == 0 || mb_rows_per_slice > mb_rows {
        return Err(Error::UnsupportedPictureGeometry);
    }
    let sss = crate::plus_ptype::SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    };

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ false,
        PlusModes {
            slice_structured: Some(sss),
            data_partitioned_slices: true,
            ..PlusModes::default()
        },
    )?;
    let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
    write_first_slice_header(&mut w, &ctx, 0, None)?;
    let gfid = tr & 0b11;

    for slice_row0 in (0..mb_rows).step_by(mb_rows_per_slice) {
        if slice_row0 > 0 {
            let mba = (slice_row0 * mb_cols) as u32;
            write_slice_layer(&mut w, &ctx, mba, quant, gfid, None)?;
        }
        let rows = mb_rows_per_slice.min(mb_rows - slice_row0);

        // Phase 1 — transform + quantise every macroblock of the
        // slice so the HD partition can carry its CBPC up front.
        let mut planned = Vec::with_capacity(rows * mb_cols);
        for r in 0..rows {
            for c in 0..mb_cols {
                let src = extract_macroblock(frame, c, slice_row0 + r);
                let luma: Vec<crate::encoder_block::EncodedIntraBlock> = src
                    .luma
                    .iter()
                    .map(|b| crate::encoder_block::encode_intra_block(b, quant))
                    .collect();
                let cb = crate::encoder_block::encode_intra_block(&src.cb, quant);
                let cr = crate::encoder_block::encode_intra_block(&src.cr, quant);
                planned.push((luma, cb, cr));
            }
        }

        // HD partition (§V.2.1) + Header Marker (§V.2.2).
        for (_, cb, cr) in &planned {
            let cbpc = (u8::from(cb.has_ac) << 1) | u8::from(cr.has_ac);
            write_dps_mb_header(
                &mut w,
                TABLE_V1_INTRA,
                DpsMbHeader::Intra { cbpc, quant: false },
            )?;
        }
        w.write_bits(HEADER_MARKER, HEADER_MARKER_BITS);

        // Coefficient partition (§V.2.6): CBPY + the six blocks per
        // macroblock, slice order.
        for (luma, cb, cr) in &planned {
            let mut cbpy = 0u8;
            for (i, b) in luma.iter().enumerate() {
                if b.has_ac {
                    cbpy |= 1 << (3 - i);
                }
            }
            write_cbpy(&mut w, cbpy)?;
            for b in luma {
                crate::encoder_block::write_intra_block(&mut w, b.dc_level, &b.scan, b.has_ac)?;
            }
            for b in [cb, cr] {
                crate::encoder_block::write_intra_block(&mut w, b.dc_level, &b.scan, b.has_ac)?;
            }
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Encode a planar 4:2:0 [`YuvFrame`] as an **Annex V Data-Partitioned
/// Slice** H.263+ **INTER** (P-) picture predicted from `reference`:
/// per free-running slice, the Table V.2 RVLC COD + MCBPC headers
/// (skipped / INTER classes), the §V.2.2 Header Marker, the §V.2.3
/// motion-vector partition — every coded macroblock's vector as Table
/// D.3 codewords over the single §V.2.3.2 prediction thread (first
/// predictor zero), with the §V.2.3.3 per-codeword emulation rule,
/// the redundant §V.2.4 LMVV and the §V.2.5 Motion Vector Marker —
/// then the §V.2.6 coefficient layer.
///
/// Predict the next picture from the decoder's reconstruction for a
/// drift-free closed loop.
pub fn encode_inter_picture_dps(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
    mb_rows_per_slice: usize,
) -> Result<Vec<u8>> {
    use crate::annex_v::{
        write_dps_mb_header, DpsMbHeader, MvdEmulationState, HEADER_MARKER, HEADER_MARKER_BITS,
        MOTION_VECTOR_MARKER, MOTION_VECTOR_MARKER_BITS, TABLE_V2_INTER,
    };
    use crate::encoder_vlc::write_cbpy;
    use crate::slice_header::{write_first_slice_header, write_slice_layer, SliceHeaderContext};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;
    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    if mb_rows_per_slice == 0 || mb_rows_per_slice > mb_rows {
        return Err(Error::UnsupportedPictureGeometry);
    }
    let sss = crate::plus_ptype::SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    };

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PlusModes {
            slice_structured: Some(sss),
            data_partitioned_slices: true,
            ..PlusModes::default()
        },
    )?;
    let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
    write_first_slice_header(&mut w, &ctx, 0, None)?;
    let gfid = tr & 0b11;
    let lambda = 2 * quant as u32;

    /// Per-macroblock plan: either skipped or a coded INTER
    /// macroblock with its vector and quantised blocks (boxed — the
    /// coded payload is ~1 KiB next to the unit Skip variant).
    enum Plan {
        Skip,
        Inter(Box<InterPlan>),
    }
    struct InterPlan {
        mv: crate::motion::MotionVector,
        luma: [crate::encoder_block::EncodedInterBlock; 4],
        cb: crate::encoder_block::EncodedInterBlock,
        cr: crate::encoder_block::EncodedInterBlock,
    }

    for slice_row0 in (0..mb_rows).step_by(mb_rows_per_slice) {
        if slice_row0 > 0 {
            let mba = (slice_row0 * mb_cols) as u32;
            write_slice_layer(&mut w, &ctx, mba, quant, gfid, None)?;
        }
        let rows = mb_rows_per_slice.min(mb_rows - slice_row0);

        // Phase 1 — motion-estimate and quantise the whole slice.
        // §V.2.3.2: the prediction thread runs over the *coded*
        // macroblocks of the slice, first predictor zero.
        let mut plans: Vec<Plan> = Vec::with_capacity(rows * mb_cols);
        let mut thread_prev = crate::motion::MotionVector::new(0, 0);
        for r in 0..rows {
            let mb_row = slice_row0 + r;
            for mb_col in 0..mb_cols {
                let mv = crate::encoder_motion::estimate_motion(
                    frame,
                    reference,
                    mb_col,
                    mb_row,
                    thread_prev,
                    search_half,
                    lambda,
                );
                let mb_x = mb_col * 16;
                let mb_y = mb_row * 16;
                let c_x = mb_col * 8;
                let c_y = mb_row * 8;
                let src = extract_macroblock(frame, mb_col, mb_row);
                let mut luma: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(4);
                for blk in 0..4 {
                    let bx = mb_x + (blk % 2) * 8;
                    let by = mb_y + (blk / 2) * 8;
                    let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                    let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                    for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                        *d = p as i16;
                    }
                    let residual = residual_of(&src.luma[blk], &pred_i16);
                    luma.push(crate::encoder_block::encode_inter_block(&residual, quant));
                }
                let chroma_mv = crate::motion::chroma_mv(mv);
                let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_mv);
                let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_mv);
                let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
                let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
                for i in 0..COEFFS_PER_BLOCK {
                    cb_pred_i[i] = cb_pred[i] as i16;
                    cr_pred_i[i] = cr_pred[i] as i16;
                }
                let cb = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cb, &cb_pred_i),
                    quant,
                );
                let cr = crate::encoder_block::encode_inter_block(
                    &residual_of(&src.cr, &cr_pred_i),
                    quant,
                );

                let any = luma.iter().any(|e| e.has_coeffs) || cb.has_coeffs || cr.has_coeffs;
                if mv.dx_half == 0 && mv.dy_half == 0 && !any {
                    plans.push(Plan::Skip);
                } else {
                    let luma: [crate::encoder_block::EncodedInterBlock; 4] =
                        luma.try_into().expect("four luma blocks");
                    plans.push(Plan::Inter(Box::new(InterPlan { mv, luma, cb, cr })));
                    thread_prev = mv;
                }
            }
        }

        // HD partition (§V.2.1) + HM (§V.2.2).
        for plan in &plans {
            let entry = match plan {
                Plan::Skip => DpsMbHeader::Skipped,
                Plan::Inter(p) => DpsMbHeader::Inter {
                    cbpc: (u8::from(p.cb.has_coeffs) << 1) | u8::from(p.cr.has_coeffs),
                    quant: false,
                },
            };
            write_dps_mb_header(&mut w, TABLE_V2_INTER, entry)?;
        }
        w.write_bits(HEADER_MARKER, HEADER_MARKER_BITS);

        // MV partition (§V.2.3–§V.2.5).
        let coded_mvs: Vec<crate::motion::MotionVector> = plans
            .iter()
            .filter_map(|p| match p {
                Plan::Inter(p) => Some(p.mv),
                Plan::Skip => None,
            })
            .collect();
        if !coded_mvs.is_empty() {
            let mut emu = MvdEmulationState::new();
            let mut prev = crate::motion::MotionVector::new(0, 0);
            for &mv in &coded_mvs {
                emu.write_component(&mut w, mv.dx_half - prev.dx_half)?;
                emu.write_component(&mut w, mv.dy_half - prev.dy_half)?;
                prev = mv;
            }
            if coded_mvs.len() >= 2 {
                let last = *coded_mvs.last().expect("non-empty");
                emu.write_component(&mut w, last.dx_half)?;
                emu.write_component(&mut w, last.dy_half)?;
            }
            w.write_bits(MOTION_VECTOR_MARKER, MOTION_VECTOR_MARKER_BITS);
        }

        // Coefficient partition (§V.2.6).
        for plan in &plans {
            let Plan::Inter(p) = plan else {
                continue;
            };
            let InterPlan { luma, cb, cr, .. } = p.as_ref();
            let mut coded_pattern = 0u8;
            for (i, b) in luma.iter().enumerate() {
                if b.has_coeffs {
                    coded_pattern |= 1 << (3 - i);
                }
            }
            // §5.3.5 — INTER macroblocks carry the complement pattern.
            write_cbpy(&mut w, coded_pattern ^ 0b1111)?;
            for b in luma {
                if b.has_coeffs {
                    crate::encoder_block::write_inter_block_coeffs(&mut w, &b.scan)?;
                }
            }
            for b in [cb, cr] {
                if b.has_coeffs {
                    crate::encoder_block::write_inter_block_coeffs(&mut w, &b.scan)?;
                }
            }
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Annex R §R.2 rule 4 — the reference view a video picture segment is
/// allowed to predict from: the segment's own luma band `top..bottom`
/// kept verbatim, every row outside it replaced by the nearest band
/// edge row (and the same, at half resolution, for the chrominance
/// planes). Sampling this frame with the ordinary §D.1 edge-replicated
/// fetch is byte-identical to clamping the row coordinate into the
/// band — the decoder's banded-fetch treatment of segment boundaries
/// as picture boundaries.
fn band_replicated_reference(reference: &YuvFrame, top: usize, bottom: usize) -> YuvFrame {
    let mut out = reference.clone();
    let replicate = |plane: &mut [u8], width: usize, height: usize, top: usize, bottom: usize| {
        let bottom = bottom.min(height).max(1);
        let top = top.min(bottom - 1);
        let (top_row, bot_row) = (top, bottom - 1);
        for row in 0..height {
            let src_row = row.clamp(top_row, bot_row);
            if src_row != row {
                let (src_off, dst_off) = (src_row * width, row * width);
                plane.copy_within(src_off..src_off + width, dst_off);
            }
        }
    };
    let (lw, lh) = (reference.luma_width, reference.luma_height);
    let (cw, ch) = (reference.chroma_width(), reference.chroma_height());
    replicate(&mut out.y, lw, lh, top, bottom);
    replicate(&mut out.cb, cw, ch, top / 2, bottom.div_ceil(2));
    replicate(&mut out.cr, cw, ch, top / 2, bottom.div_ceil(2));
    out
}

#[allow(clippy::too_many_arguments)]
fn encode_inter_picture_motion_impl(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
    umv: bool,
    framing: InterFraming,
    plus: bool,
    // Annex R — emit the OPPTYPE ISD bit and confine motion
    // estimation + prediction to each GOB segment's band (the
    // reference rows outside the segment are edge-replicated, exactly
    // the decoder's §R.2 rule-4 border extrapolation). Requires
    // `plus` and [`InterFraming::GobHeaders`].
    isd: bool,
) -> Result<Vec<u8>> {
    use crate::plus_ptype::SliceStructuredSubmode;
    use crate::slice_header::{write_first_slice_header, write_slice_layer, SliceHeaderContext};

    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;
    let mb_rows_total = frame.luma_height / 16;
    // Annex K slices ride the PLUSPTYPE header only, and the slice
    // height must tile the picture's MB rows.
    let sss = match framing {
        InterFraming::Slices { rows } => {
            if !plus {
                return Err(Error::NotImplemented);
            }
            if rows == 0 || rows > mb_rows_total {
                return Err(Error::UnsupportedPictureGeometry);
            }
            Some(SliceStructuredSubmode {
                rectangular: false,
                arbitrary_order: false,
            })
        }
        _ => None,
    };

    if isd && (!plus || !matches!(framing, InterFraming::GobHeaders)) {
        // Annex R is signalled in OPPTYPE (PLUSPTYPE only), and this
        // encoder stages it on the GOB segmentation.
        return Err(Error::NotImplemented);
    }

    let mut w = BitWriter::new();
    if plus {
        write_plus_picture_header(
            &mut w,
            fmt,
            quant,
            tr,
            /* is_inter */ true,
            PlusModes {
                umv,
                slice_structured: sss,
                independent_segment_decoding: isd,
                ..PlusModes::default()
            },
        )?;
    } else {
        write_picture_header(
            &mut w,
            fmt,
            quant,
            tr,
            /* is_inter */ true,
            PtypeFlags {
                umv,
                ..PtypeFlags::default()
            },
            None,
        );
    }

    // §K.2 slice-header context + the §K.2.2 reduced first-slice header
    // (MBA 0) for the slice framing.
    let slice_ctx =
        sss.map(|sss| SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false));
    if let Some(ctx) = slice_ctx.as_ref() {
        write_first_slice_header(&mut w, ctx, 0, None)?;
    }

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    // §4.2.1 — macroblock rows per GOB for the standard source formats
    // (1 for sub-QCIF..CIF, 2 for 4CIF, 4 for 16CIF).
    let rows_per_gob = layout.mb_rows_per_gob as usize;
    // The per-segment predictor treatment: GOB headers and row-aligned
    // slices both reduce to "rule-3 top border every k rows", which
    // `MvGrid::with_gob_headers` replays exactly (the decoder's
    // segment-id check collapses to the same row test for row-aligned
    // uniform segments).
    let mut grid = match framing {
        InterFraming::Single => crate::encoder_motion::MvGrid::new(mb_cols, mb_rows),
        InterFraming::GobHeaders => {
            crate::encoder_motion::MvGrid::with_gob_headers(mb_cols, mb_rows, rows_per_gob)
        }
        InterFraming::Slices { rows } => {
            crate::encoder_motion::MvGrid::with_gob_headers(mb_cols, mb_rows, rows)
        }
    };
    let gfid = tr & 0b11;
    // λ in SAD units per half-pel of MVD; a small bias keeps static
    // regions on MVD = 0 without over-penalising real motion.
    let lambda = 2 * quant as u32;

    // Annex R — per-segment edge-replicated reference view; rebuilt at
    // each segment top when `isd` is set.
    let mut seg_ref_storage: Option<YuvFrame> = None;
    for mb_row in 0..mb_rows {
        match framing {
            InterFraming::Single => {}
            // §5.2 — a GOB header before the first macroblock row of
            // every GOB after GOB 0 (which is always header-less,
            // §5.2.2).
            InterFraming::GobHeaders => {
                if mb_row > 0 && mb_row % rows_per_gob == 0 {
                    write_gob_header(&mut w, (mb_row / rows_per_gob) as u32, gfid, quant);
                }
            }
            // §K.2 — SSTUF + SSC slice header at every slice start
            // after the first (reduced-header) slice.
            InterFraming::Slices { rows } => {
                if mb_row > 0 && mb_row % rows == 0 {
                    let ctx = slice_ctx.as_ref().expect("slice framing has a context");
                    let mba = (mb_row * mb_cols) as u32;
                    write_slice_layer(&mut w, ctx, mba, quant, gfid, None)?;
                }
            }
        }
        // Annex R — at each segment top, materialize the reference
        // view this segment is allowed to see: its own band with the
        // out-of-band rows edge-replicated (§R.2 rule 4 treats the
        // segment borders like picture borders, so a UMV vector
        // reaching past them predicts from the replicated edge —
        // byte-identical to the decoder's banded fetch clamp).
        if isd && mb_row % rows_per_gob == 0 {
            let top = mb_row * 16;
            let bottom = ((mb_row + rows_per_gob) * 16).min(lh);
            seg_ref_storage = Some(band_replicated_reference(reference, top, bottom));
        }
        let seg_ref: &YuvFrame = seg_ref_storage.as_ref().unwrap_or(reference);
        for mb_col in 0..mb_cols {
            let predictor = grid.predict(mb_col, mb_row);
            let mv = if umv && plus {
                // §D.2 with PLUSPTYPE: single-valued Table D.3
                // differences — every candidate in the Tables-D.1/D.2
                // window (∩ the §D.1.1 border bound) is codable.
                crate::encoder_motion::estimate_motion_umv_plus(
                    frame,
                    seg_ref,
                    mb_col,
                    mb_row,
                    predictor,
                    search_half,
                    lambda,
                )
            } else if umv {
                crate::encoder_motion::estimate_motion_umv(
                    frame,
                    seg_ref,
                    mb_col,
                    mb_row,
                    predictor,
                    search_half,
                    lambda,
                )
            } else {
                crate::encoder_motion::estimate_motion(
                    frame,
                    seg_ref,
                    mb_col,
                    mb_row,
                    predictor,
                    search_half,
                    lambda,
                )
            };

            let mb_x = mb_col * 16;
            let mb_y = mb_row * 16;
            let c_x = mb_col * 8;
            let c_y = mb_row * 8;
            let chroma_mv = crate::motion::chroma_mv(mv);

            // Build the motion-compensated prediction + residual per block.
            let src = extract_macroblock(frame, mb_col, mb_row);
            let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(4);
            for blk in 0..4 {
                let bx = mb_x + (blk % 2) * 8;
                let by = mb_y + (blk / 2) * 8;
                let pred = motion_compensated_block(&seg_ref.y, lw, lh, bx, by, mv);
                let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                    *d = p as i16;
                }
                let residual = residual_of(&src.luma[blk], &pred_i16);
                luma_enc.push(crate::encoder_block::encode_inter_block(&residual, quant));
            }
            let cb_pred = motion_compensated_block(&seg_ref.cb, cw, ch, c_x, c_y, chroma_mv);
            let cr_pred = motion_compensated_block(&seg_ref.cr, cw, ch, c_x, c_y, chroma_mv);
            let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
            let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
            for i in 0..COEFFS_PER_BLOCK {
                cb_pred_i[i] = cb_pred[i] as i16;
                cr_pred_i[i] = cr_pred[i] as i16;
            }
            let cb_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cb, &cb_pred_i), quant);
            let cr_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cr, &cr_pred_i), quant);

            let any_coeffs =
                luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;
            let is_zero_mv = mv.dx_half == 0 && mv.dy_half == 0;

            if !any_coeffs && is_zero_mv {
                // Skipped MB: COD = 1. The decoder copies the co-located
                // reference block with a zero MV (= our prediction). The
                // grid records a zero candidate (skipped MB).
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
                grid.set_zero_candidate(mb_col, mb_row);
                continue;
            }

            // §5.3.2 INTRA/INTER mode decision. When the motion-compensated
            // residual energy exceeds the macroblock's own AC energy
            // (variance about the per-block mean), an INTRA macroblock
            // costs fewer bits and reconstructs more faithfully than a
            // large INTER residual — the classic H.263 intra-refresh
            // heuristic. The INTRA candidate is recorded as a zero
            // predictor candidate (§6.1.1 rule 1, outside PB-frames mode).
            let inter_sad: u32 = {
                let mut s = 0u32;
                for blk in 0..4 {
                    let bx = mb_x + (blk % 2) * 8;
                    let by = mb_y + (blk / 2) * 8;
                    let pred = motion_compensated_block(&seg_ref.y, lw, lh, bx, by, mv);
                    for row in 0..8 {
                        for col in 0..8 {
                            let sv = frame.y[(by + row) * lw + (bx + col)] as i32;
                            let pv = pred[row * 8 + col] as i32;
                            s += (sv - pv).unsigned_abs();
                        }
                    }
                }
                s
            };
            let intra_sad: u32 = src
                .luma
                .iter()
                .map(|blk| {
                    let mean = blk.iter().map(|&v| v as i32).sum::<i32>() / 64;
                    blk.iter()
                        .map(|&v| (v as i32 - mean).unsigned_abs())
                        .sum::<u32>()
                })
                .sum();

            if intra_sad + 256 < inter_sad {
                // Code as an INTRA macroblock in the P-picture (Table 8
                // INTRA MCBPC, with COD). No motion vector is carried.
                encode_intra_macroblock(
                    &mut w, &src, quant, /* write_cod */ true,
                    /* picture_is_inter */ true,
                )?;
                grid.set_zero_candidate(mb_col, mb_row);
                continue;
            }

            let mvd = if umv && plus {
                // §D.2 with PLUSPTYPE: the Table D.3 difference is the
                // plain `mv − predictor` (single-valued, no pair
                // selection).
                crate::macroblock::Mvd {
                    dx_half: (mv.dx_half - predictor.dx_half) as i16,
                    dy_half: (mv.dy_half - predictor.dy_half) as i16,
                }
            } else if umv {
                // The UMV estimator only returns §D.2-reachable vectors,
                // so the inverse always exists.
                crate::encoder_motion::umv_mvd_for(mv, predictor).ok_or(Error::BadMvdCode)?
            } else {
                crate::encoder_motion::mvd_for(mv, predictor)
            };
            let luma_arr: [crate::encoder_block::EncodedInterBlock; 4] = [
                luma_enc[0].clone(),
                luma_enc[1].clone(),
                luma_enc[2].clone(),
                luma_enc[3].clone(),
            ];
            if umv && plus {
                crate::encoder_mb::encode_inter_macroblock_umv_plus(
                    &mut w, &luma_arr, &cb_enc, &cr_enc, mvd,
                )?;
            } else {
                crate::encoder_mb::encode_inter_macroblock(
                    &mut w, &luma_arr, &cb_enc, &cr_enc, mvd,
                )?;
            }
            grid.set_inter(mb_col, mb_row, mv);
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Configuration for [`encode_pb_picture`].
#[derive(Debug, Clone, Copy)]
pub struct PbConfig {
    /// Quantiser for the P-blocks (`1..=31`); the B-blocks run at the
    /// §5.1.23 / Table-6 BQUANT derived from it and `dbquant`.
    pub quant: u8,
    /// §5.1.22 TRB (3-bit form, `1..=7`): the number of
    /// non-transmitted pictures between the previous anchor and the
    /// B-picture, plus one.
    pub trb: u8,
    /// §5.1.23 DBQUANT (`0..=3`): selects the Table-6 QUANT → BQUANT
    /// relation.
    pub dbquant: u8,
    /// Motion-search window for the P-part (±whole pixels).
    pub search_half: i32,
    /// §5.3.9 MVDB refinement window for the B-part, in **half-pel**
    /// units around the §G.4 scaled prediction (`0` disables the
    /// search: every macroblock keeps `MVD = 0`). Each candidate
    /// delta must keep the per-component forward vector MVF inside
    /// the §G.4 permitted range `[-16, 15.5]` (half-pel `[-32, 31]`)
    /// — the constraint that disambiguates the Table 14 codeword's
    /// value pair for any decoder — and be Table 14 codable itself.
    pub b_search_half: i32,
}

impl Default for PbConfig {
    fn default() -> Self {
        PbConfig {
            quant: 8,
            trb: 1,
            dbquant: 0,
            search_half: 8,
            b_search_half: 2,
        }
    }
}

/// Encode an Annex G **PB-frame**: one picture unit carrying a
/// P-picture (`p_source`, predicted from `reference`) and a B-picture
/// (`b_source`, temporally between `reference` and `p_source`,
/// predicted bidirectionally per §G.4 / §G.5).
///
/// Per macroblock the P-part is motion-estimated and coded exactly
/// like [`encode_inter_picture_motion`]; the encoder then
/// reconstructs the P-macroblock (PREC, §G.5) the way the decoder
/// will, forms the §G.4 bidirectional B-prediction with the
/// TRB/TRD-scaled vectors (`MVDB = 0`), and codes the B-residual at
/// the Table-6 BQUANT wherever it survives quantisation (MODB `"11"`
/// with a zero MVDB + CBPB; MODB `"0"` when no B-block is lit). A
/// macroblock with a zero vector, no P-residual and no B-residual is
/// skipped (COD = 1).
///
/// `tr_p` is the §5.1.2 Temporal Reference of the P-part; `prev_tr`
/// is the reference picture's TR (their difference mod 256 is the
/// §G.4 TRD, which must be non-zero and greater than
/// [`PbConfig::trb`]). The output decodes through
/// [`crate::picture::decode_pb_picture_no_gob0_header`] and — inside
/// an elementary stream — [`crate::picture::decode_sequence`], which
/// splices the decoded pair in display order (B before P).
pub fn encode_pb_picture(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<Vec<u8>> {
    use crate::pb_layer::{pb_b_predict_macroblock, pb_bquant, PbBReferencePlanes};

    if cfg.quant == 0 || cfg.quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if cfg.trb == 0 || cfg.trb > 7 || cfg.dbquant > 3 {
        return Err(Error::BadPbTemporalReference);
    }
    let trd = i32::from(tr_p.wrapping_sub(prev_tr));
    if trd == 0 || i32::from(cfg.trb) >= trd {
        return Err(Error::BadPbTemporalReference);
    }
    if p_source.luma_width != reference.luma_width
        || p_source.luma_height != reference.luma_height
        || b_source.luma_width != reference.luma_width
        || b_source.luma_height != reference.luma_height
    {
        return Err(Error::NotImplemented);
    }
    let fmt = source_format_for(p_source.luma_width, p_source.luma_height)
        .ok_or(Error::NotImplemented)?;

    let quant = cfg.quant;
    let bquant = pb_bquant(cfg.dbquant, quant);
    let trb = i32::from(cfg.trb);

    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        quant,
        tr_p,
        /* is_inter */ true,
        PtypeFlags::default(),
        Some((cfg.trb, cfg.dbquant)),
    );

    let lw = p_source.luma_width;
    let lh = p_source.luma_height;
    let cw = p_source.chroma_width();
    let ch = p_source.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let mut grid = crate::encoder_motion::MvGrid::new(mb_cols, mb_rows);
    let lambda = 2 * quant as u32;

    let prev_y = crate::motion::RefPlane::new(&reference.y, lw, lh);
    let prev_cb = crate::motion::RefPlane::new(&reference.cb, cw, ch);
    let prev_cr = crate::motion::RefPlane::new(&reference.cr, cw, ch);

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb_x = mb_col * 16;
            let mb_y = mb_row * 16;
            let c_x = mb_col * 8;
            let c_y = mb_row * 8;

            // ---- P-part: motion estimation + residual coding. -------
            let predictor = grid.predict(mb_col, mb_row);
            let mv = crate::encoder_motion::estimate_motion(
                p_source,
                reference,
                mb_col,
                mb_row,
                predictor,
                cfg.search_half,
                lambda,
            );
            let chroma_mv = crate::motion::chroma_mv(mv);
            let src = extract_macroblock(p_source, mb_col, mb_row);

            let mut luma_pred: Vec<[u8; COEFFS_PER_BLOCK]> = Vec::with_capacity(4);
            let mut luma_enc: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(4);
            for blk in 0..4 {
                let bx = mb_x + (blk % 2) * 8;
                let by = mb_y + (blk / 2) * 8;
                let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                for (d, &pv) in pred_i16.iter_mut().zip(pred.iter()) {
                    *d = pv as i16;
                }
                luma_enc.push(crate::encoder_block::encode_inter_block(
                    &residual_of(&src.luma[blk], &pred_i16),
                    quant,
                ));
                luma_pred.push(pred);
            }
            let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_mv);
            let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_mv);
            let mut cb_pred_i = [0i16; COEFFS_PER_BLOCK];
            let mut cr_pred_i = [0i16; COEFFS_PER_BLOCK];
            for i in 0..COEFFS_PER_BLOCK {
                cb_pred_i[i] = cb_pred[i] as i16;
                cr_pred_i[i] = cr_pred[i] as i16;
            }
            let cb_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cb, &cb_pred_i), quant);
            let cr_enc =
                crate::encoder_block::encode_inter_block(&residual_of(&src.cr, &cr_pred_i), quant);

            let any_p =
                luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;
            let is_zero_mv = mv.dx_half == 0 && mv.dy_half == 0;

            // ---- PREC (§G.5): the decoder-reconstructed P-macroblock.
            let recon_block = |enc: &crate::encoder_block::EncodedInterBlock,
                               pred: &[u8; COEFFS_PER_BLOCK],
                               q: u8|
             -> [u8; COEFFS_PER_BLOCK] {
                if enc.has_coeffs {
                    let block = crate::block::H263Block {
                        coefficients: enc.scan,
                        tcoef_event_count: 0,
                        had_intradc: false,
                    };
                    crate::reconstruct_inter_block_with_prediction(&block, q, pred)
                } else {
                    *pred
                }
            };
            let mut prec_y = [0u8; 256];
            for blk in 0..4 {
                let samples = recon_block(&luma_enc[blk], &luma_pred[blk], quant);
                let ox = (blk % 2) * 8;
                let oy = (blk / 2) * 8;
                for j in 0..8 {
                    prec_y[(oy + j) * 16 + ox..(oy + j) * 16 + ox + 8]
                        .copy_from_slice(&samples[j * 8..j * 8 + 8]);
                }
            }
            let prec_cb = recon_block(&cb_enc, &cb_pred, quant);
            let prec_cr = recon_block(&cr_enc, &cr_pred, quant);

            // ---- B-part: §G.4 + §G.5 bidirectional prediction with
            // MVDB = 0, residual at BQUANT. ---------------------------
            let planes = PbBReferencePlanes {
                prev_y,
                prev_cb,
                prev_cr,
                prec_y: crate::motion::RefPlane::new(&prec_y, 16, 16),
                prec_cb: crate::motion::RefPlane::new(&prec_cb, 8, 8),
                prec_cr: crate::motion::RefPlane::new(&prec_cr, 8, 8),
            };
            let b_src = extract_macroblock(b_source, mb_col, mb_row);

            // §5.3.9 / §G.4 — MVDB refinement: search a small window
            // of delta vectors, keeping only deltas that are Table 14
            // codable and whose per-component MVF stays inside the
            // §G.4 permitted range (the in-range value is what any
            // decoder selects from the Table 14 pair, since the pair
            // mate sits exactly 64 half-pels away — outside a range
            // only 64 wide). SAD of the B-luma against the §G.5
            // blended prediction decides; the zero delta (MVD = 0)
            // competes with a flat bias so static content never pays
            // the MODB/MVDB bits.
            let b_sad_of = |pred: &crate::pb_layer::PbBMacroblockPrediction| -> u32 {
                let mut sad = 0u32;
                for blk in 0..4 {
                    let ox = (blk % 2) * 8;
                    let oy = (blk / 2) * 8;
                    for j in 0..8 {
                        for i in 0..8 {
                            let sv = b_src.luma[blk][j * 8 + i] as i32;
                            let pv = pred.luma[oy + j][ox + i] as i32;
                            sad += (sv - pv).unsigned_abs();
                        }
                    }
                }
                sad
            };
            let mvf_in_range = |p_comp: i32, delta: i32| -> bool {
                let mvf = (trb * p_comp) / trd + delta;
                (-32..=31).contains(&mvf)
            };
            let predict_at = |delta: Option<crate::macroblock::Mvd>| {
                pb_b_predict_macroblock(
                    &planes,
                    mb_x,
                    mb_y,
                    &[mv; 4],
                    delta,
                    trb,
                    trd,
                    crate::motion::RCONTROL_DEFAULT,
                )
            };
            let mut b_pred = predict_at(None);
            let mut best_delta: Option<crate::macroblock::Mvd> = None;
            if cfg.b_search_half > 0 {
                let bw = cfg.b_search_half;
                let mut best_cost = b_sad_of(&b_pred);
                for dy in -bw..=bw {
                    for dx in -bw..=bw {
                        if (dx == 0 && dy == 0)
                            || !(-32..=31).contains(&dx)
                            || !(-32..=31).contains(&dy)
                            || !mvf_in_range(mv.dx_half, dx)
                            || !mvf_in_range(mv.dy_half, dy)
                        {
                            continue;
                        }
                        let delta = crate::macroblock::Mvd {
                            dx_half: dx as i16,
                            dy_half: dy as i16,
                        };
                        let cand = predict_at(Some(delta));
                        let cost = b_sad_of(&cand)
                            + lambda * (dx.unsigned_abs() + dy.unsigned_abs())
                            + 4 * lambda;
                        if cost < best_cost {
                            best_cost = cost;
                            best_delta = Some(delta);
                            b_pred = cand;
                        }
                    }
                }
            }

            let mut b_enc: Vec<crate::encoder_block::EncodedInterBlock> = Vec::with_capacity(6);
            for blk in 0..4 {
                let ox = (blk % 2) * 8;
                let oy = (blk / 2) * 8;
                let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                for j in 0..8 {
                    for i in 0..8 {
                        pred_i16[j * 8 + i] = b_pred.luma[oy + j][ox + i] as i16;
                    }
                }
                b_enc.push(crate::encoder_block::encode_inter_block(
                    &residual_of(&b_src.luma[blk], &pred_i16),
                    bquant,
                ));
            }
            let mut b_cb_pred = [0i16; COEFFS_PER_BLOCK];
            let mut b_cr_pred = [0i16; COEFFS_PER_BLOCK];
            for j in 0..8 {
                for i in 0..8 {
                    b_cb_pred[j * 8 + i] = b_pred.cb[j][i] as i16;
                    b_cr_pred[j * 8 + i] = b_pred.cr[j][i] as i16;
                }
            }
            b_enc.push(crate::encoder_block::encode_inter_block(
                &residual_of(&b_src.cb, &b_cb_pred),
                bquant,
            ));
            b_enc.push(crate::encoder_block::encode_inter_block(
                &residual_of(&b_src.cr, &b_cr_pred),
                bquant,
            ));
            let any_b = b_enc.iter().any(|e| e.has_coeffs);

            // ---- Skip / emit. ---------------------------------------
            if !any_p && !any_b && is_zero_mv && best_delta.is_none() {
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
                grid.set_zero_candidate(mb_col, mb_row);
                continue;
            }

            // COD = 0; MCBPC (Table 8, INTER type 0).
            w.write_bit(false);
            let mut cbpc = 0u8;
            if cb_enc.has_coeffs {
                cbpc |= 0b10;
            }
            if cr_enc.has_coeffs {
                cbpc |= 0b01;
            }
            crate::encoder_vlc::write_mcbpc_p(&mut w, crate::macroblock::MbType::Inter, cbpc)?;

            // §5.3.3 MODB (Table 11): "0" = no CBPB/MVDB, "10" = MVDB
            // only, "11" = CBPB + MVDB.
            let has_mvdb = any_b || best_delta.is_some();
            if any_b {
                w.write_bit(true);
                w.write_bit(true);
                // §5.3.4 CBPB — block N lights bit (6 − N).
                let mut cbpb = 0u8;
                for (blk, e) in b_enc.iter().enumerate() {
                    if e.has_coeffs {
                        cbpb |= 1 << (6 - (blk + 1));
                    }
                }
                w.write_bits(cbpb as u32, 6);
            } else if has_mvdb {
                w.write_bit(true);
                w.write_bit(false);
            } else {
                w.write_bit(false);
            }

            // §5.3.5 CBPY (INTER complement).
            let mut cbpy_intra = 0u8;
            for (blk, e) in luma_enc.iter().enumerate() {
                if e.has_coeffs {
                    cbpy_intra |= 1 << (3 - blk);
                }
            }
            crate::encoder_vlc::write_cbpy(&mut w, cbpy_intra ^ 0b1111)?;

            // §5.3.7 MVD.
            let mvd = crate::encoder_motion::mvd_for(mv, predictor);
            crate::encoder_vlc::write_mvd_component(&mut w, mvd.dx_half)?;
            crate::encoder_vlc::write_mvd_component(&mut w, mvd.dy_half)?;

            // §5.3.9 MVDB — the searched delta (zero when only CBPB
            // forced MODB).
            if has_mvdb {
                let d = best_delta.unwrap_or(crate::macroblock::Mvd {
                    dx_half: 0,
                    dy_half: 0,
                });
                crate::encoder_vlc::write_mvd_component(&mut w, d.dx_half)?;
                crate::encoder_vlc::write_mvd_component(&mut w, d.dy_half)?;
            }

            // §G.3 — six P-blocks, then six B-blocks.
            for e in luma_enc.iter() {
                if e.has_coeffs {
                    crate::encoder_block::write_inter_block_coeffs(&mut w, &e.scan)?;
                }
            }
            if cb_enc.has_coeffs {
                crate::encoder_block::write_inter_block_coeffs(&mut w, &cb_enc.scan)?;
            }
            if cr_enc.has_coeffs {
                crate::encoder_block::write_inter_block_coeffs(&mut w, &cr_enc.scan)?;
            }
            for e in b_enc.iter() {
                if e.has_coeffs {
                    crate::encoder_block::write_inter_block_coeffs(&mut w, &e.scan)?;
                }
            }

            grid.set_inter(mb_col, mb_row, mv);
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// §5.1.27 — the byte-aligned End Of Sequence marker: the 22-bit
/// codeword `0000 0000 0000 0000 1 11111` followed by two ESTUF-style
/// zero bits completing the byte. Appending it to a byte-aligned
/// elementary stream (every picture ends PSTUF-padded, §5.1.28) keeps
/// the alignment invariant.
pub const EOS_BYTES: [u8; 3] = [0x00, 0x00, 0xFC];

/// Configuration for [`encode_sequence`] — the closed-loop GOP encoder.
#[derive(Debug, Clone, Copy)]
pub struct GopConfig {
    /// Quantiser for every picture (`1..=31`).
    pub quant: u8,
    /// An INTRA picture every `intra_period` frames (frame 0 is always
    /// INTRA). `0` means "only the first frame is INTRA" (an infinite
    /// GOP); `1` means all-INTRA.
    pub intra_period: usize,
    /// Motion-search window for P-pictures (±whole pixels around the
    /// predictor).
    pub search_half: i32,
    /// Encode P-pictures in the Annex D Unrestricted Motion Vector
    /// mode (extended §D.2 range + §D.1 over-boundary vectors).
    pub umv: bool,
    /// Append the §5.1.27 End Of Sequence marker after the last
    /// picture.
    pub eos: bool,
    /// Encode every picture in the Annex J **Deblocking Filter mode**
    /// (H.263+ headers, OPPTYPE bit 9): the closed loop predicts from
    /// the §J.3-filtered reconstruction, P-pictures use the mode's
    /// §F.2 predictors and may carry four vectors per macroblock
    /// ([`Self::four_mv`]). Composes with [`Self::umv`] (Table D.3
    /// difference coding).
    pub deblock: bool,
    /// Allow four motion vectors per macroblock (INTER4V) in
    /// deblocking-mode P-pictures (ignored without [`Self::deblock`]).
    pub four_mv: bool,
}

impl Default for GopConfig {
    fn default() -> Self {
        GopConfig {
            quant: 8,
            intra_period: 12,
            search_half: 8,
            umv: false,
            eos: false,
            deblock: false,
            four_mv: true,
        }
    }
}

/// Encode a sequence of frames as an H.263 elementary stream with a
/// classic **I + P GOP structure**, closed-loop.
///
/// Frame 0 (and every `intra_period`-th frame after it) is coded as a
/// baseline INTRA picture; every other frame is a motion-estimated
/// INTER picture predicted from the **decoder's reconstruction** of
/// the previous picture — the encoder decodes its own output picture
/// by picture (via [`crate::picture::decode_picture_no_gob0_header`])
/// so its prediction reference is bit-identical to what any conformant
/// decoder holds, eliminating encoder–decoder drift over arbitrarily
/// long sequences. The §5.1.2 Temporal Reference increments modulo 256
/// from `tr0`.
///
/// The resulting stream decodes through
/// [`crate::picture::decode_sequence`] into `frames.len()` pictures;
/// with [`GopConfig::eos`] set, the §5.1.27 End Of Sequence codeword
/// (byte-aligned, [`EOS_BYTES`]) terminates the stream — decoders
/// ignore it (it is not a Picture Start Code).
pub fn encode_sequence(frames: &[YuvFrame], cfg: &GopConfig, tr0: u8) -> Result<Vec<u8>> {
    use crate::picture::{decode_picture_no_gob0_header, DecodeOptions};

    if cfg.quant == 0 || cfg.quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let mut out = Vec::new();
    let mut recon: Option<YuvFrame> = None;
    for (i, frame) in frames.iter().enumerate() {
        let tr = tr0.wrapping_add(i as u8);
        let force_intra = recon.is_none() || (cfg.intra_period != 0 && i % cfg.intra_period == 0);
        let bytes = if force_intra {
            if cfg.deblock {
                encode_intra_picture_deblock(frame, cfg.quant, tr)?
            } else {
                encode_intra_picture(frame, cfg.quant, tr)?
            }
        } else {
            let reference = recon.as_ref().expect("recon present for P-picture");
            if cfg.deblock {
                let dcfg = DeblockConfig {
                    search_half: cfg.search_half,
                    four_mv: cfg.four_mv,
                    umv: cfg.umv,
                };
                encode_inter_picture_deblock(frame, reference, cfg.quant, tr, &dcfg)?
            } else if cfg.umv {
                encode_inter_picture_umv(frame, reference, cfg.quant, tr, cfg.search_half)?
            } else {
                encode_inter_picture_motion(frame, reference, cfg.quant, tr, cfg.search_half)?
            }
        };
        // Closed loop: the next picture predicts from the *decoded*
        // reconstruction of this one, exactly like the decoder will.
        // Deblocking-mode pictures go through the extended-header
        // driver, which applies the §J.3 filter from OPPTYPE, so the
        // reference held here is the filtered one.
        let prior = if force_intra { None } else { recon.as_ref() };
        let decoded = if cfg.deblock {
            crate::picture::decode_picture_layer(&bytes, prior, DecodeOptions::default())?
        } else {
            decode_picture_no_gob0_header(&bytes, prior, DecodeOptions::default())?
        };
        out.extend_from_slice(&bytes);
        recon = Some(decoded);
    }
    if cfg.eos {
        // §5.1.27 — EOS, byte-aligned (the stream is already a
        // multiple of 8 bits after each picture's PSTUF).
        out.extend_from_slice(&EOS_BYTES);
    }
    Ok(out)
}

/// Configuration for [`encode_sequence_rate_controlled`].
#[derive(Debug, Clone, Copy)]
pub struct RateControlConfig {
    /// Bit budget per coded picture (the [`crate::rate_control::RateController`]
    /// target). For a CBR channel of `R` bits/s at `PCF` pictures/s
    /// this is `R / PCF`.
    pub target_bits_per_picture: u32,
    /// QUANT for the first picture; the controller adapts from here.
    pub initial_quant: u8,
    /// An INTRA picture every `intra_period` frames (frame 0 always).
    /// `0` = only frame 0 is INTRA.
    pub intra_period: usize,
    /// Motion-search window for P-pictures (±whole pixels).
    pub search_half: i32,
    /// Annex B HRD parameters to regulate against; `None` runs the
    /// virtual-buffer controller without the HRD conformance loop.
    pub hrd: Option<crate::rate_control::HrdParams>,
    /// Regulate **inside** each picture as well: encode pictures with
    /// the §5.3.6 per-macroblock DQUANT governors
    /// ([`crate::encoder_rc::encode_intra_picture_adaptive`] /
    /// [`crate::encoder_rc::encode_inter_picture_adaptive`]) aimed at
    /// this picture budget, instead of one QUANT per picture.
    pub mb_adaptive: bool,
    /// Maximum re-encodes of a single picture when it lands outside
    /// the regulation bounds (HRD violation → finer QUANT; > 4× budget
    /// → coarser QUANT). `0` disables re-encoding.
    pub max_reencodes: u8,
}

impl Default for RateControlConfig {
    fn default() -> Self {
        RateControlConfig {
            target_bits_per_picture: 24_000,
            initial_quant: 10,
            intra_period: 12,
            search_half: 8,
            hrd: None,
            mb_adaptive: false,
            max_reencodes: 1,
        }
    }
}

/// Output of [`encode_sequence_rate_controlled`]: the elementary
/// stream plus the per-picture measurements the regulation produced.
#[derive(Debug, Clone)]
pub struct RateControlledStream {
    /// The H.263 elementary stream (decodes through
    /// [`crate::picture::decode_sequence`]).
    pub bytes: Vec<u8>,
    /// Coded size of each picture in bits.
    pub picture_bits: Vec<u32>,
    /// QUANT each picture was finally coded at.
    pub picture_quants: Vec<u8>,
    /// Whether every picture kept the Annex B §B.4 requirement
    /// (`true` when no HRD parameters were supplied).
    pub hrd_conformant: bool,
    /// Largest §B.4 post-removal occupancy observed (0 without HRD).
    pub hrd_max_occupancy: u64,
}

/// Encode a sequence of frames as a **rate-controlled** I + P GOP
/// elementary stream: the closed-loop structure of
/// [`encode_sequence`] (each P-picture predicts from the decoder's
/// reconstruction) with the per-picture QUANT chosen by a
/// [`crate::rate_control::RateController`] virtual-buffer loop aiming
/// at [`RateControlConfig::target_bits_per_picture`], optionally
/// regulated against the **Annex B Hypothetical Reference Decoder**:
///
/// * each coded picture is fed through the
///   [`crate::rate_control::HrdModel`] CBR simulation; a picture whose
///   removal would leave the buffer at or above `B` (§B.4 — the
///   picture undershot the channel so far that the buffer backlog
///   crossed the bound) is re-encoded at a **finer** QUANT (more
///   bits), up to [`RateControlConfig::max_reencodes`] times;
/// * a picture overshooting 4× the budget is re-encoded at a
///   **coarser** QUANT, bounding worst-case channel latency.
///
/// Any residual §B.4 violation after the re-encode budget is spent is
/// *reported* on [`RateControlledStream::hrd_conformant`] rather than
/// silently ignored.
pub fn encode_sequence_rate_controlled(
    frames: &[YuvFrame],
    cfg: &RateControlConfig,
    tr0: u8,
) -> Result<RateControlledStream> {
    use crate::picture::{decode_picture_no_gob0_header, DecodeOptions};
    use crate::rate_control::{HrdModel, RateController};

    if cfg.initial_quant == 0 || cfg.initial_quant > 31 {
        return Err(Error::InvalidQuantiser);
    }

    let mut rc = RateController::new(cfg.target_bits_per_picture, cfg.initial_quant);
    let mut hrd = cfg.hrd.map(HrdModel::new);
    let mut out = Vec::new();
    let mut picture_bits = Vec::with_capacity(frames.len());
    let mut picture_quants = Vec::with_capacity(frames.len());
    let mut hrd_conformant = true;
    let mut recon: Option<YuvFrame> = None;

    for (i, frame) in frames.iter().enumerate() {
        let tr = tr0.wrapping_add(i as u8);
        let force_intra = recon.is_none() || (cfg.intra_period != 0 && i % cfg.intra_period == 0);

        let mut quant = rc.next_quant();
        let mut bytes;
        let mut reencodes = cfg.max_reencodes;
        loop {
            bytes = if cfg.mb_adaptive {
                let acfg = crate::encoder_rc::AdaptiveQuantConfig {
                    target_bits: cfg.target_bits_per_picture,
                    initial_quant: quant,
                    search_half: cfg.search_half,
                };
                if force_intra {
                    crate::encoder_rc::encode_intra_picture_adaptive(frame, &acfg, tr)?.bytes
                } else {
                    let reference = recon.as_ref().expect("recon present for P-picture");
                    crate::encoder_rc::encode_inter_picture_adaptive(frame, reference, &acfg, tr)?
                        .bytes
                }
            } else if force_intra {
                encode_intra_picture(frame, quant, tr)?
            } else {
                let reference = recon.as_ref().expect("recon present for P-picture");
                encode_inter_picture_motion(frame, reference, quant, tr, cfg.search_half)?
            };
            let bits = bytes.len() as u64 * 8;

            // Probe the regulation bounds on scratch state; commit only
            // the final encode.
            let hrd_violation = hrd
                .as_ref()
                .map(|h| !{
                    let mut probe = *h;
                    probe.push_picture(bits).conformant
                })
                .unwrap_or(false);
            let overshoot = bits > 4 * cfg.target_bits_per_picture as u64;

            if reencodes == 0 || (!hrd_violation && !overshoot) {
                break;
            }
            reencodes -= 1;
            if hrd_violation {
                // §B.4 — the picture is too small for the channel
                // backlog; spend more bits (finer QUANT).
                if quant == 1 {
                    break;
                }
                quant = quant.saturating_sub(4).max(1);
            } else {
                // Latency bound: too many channel intervals for one
                // picture; spend fewer bits (coarser QUANT).
                if quant == 31 {
                    break;
                }
                quant = (quant + 4).min(31);
            }
        }

        let bits = bytes.len() as u64 * 8;
        if let Some(h) = hrd.as_mut() {
            let outcome = h.push_picture(bits);
            hrd_conformant &= outcome.conformant;
        }
        rc.update(bits);
        picture_bits.push(bits as u32);
        picture_quants.push(quant);

        // Closed loop: predict the next picture from the decoded
        // reconstruction of this one.
        let decoded = decode_picture_no_gob0_header(
            &bytes,
            if force_intra { None } else { recon.as_ref() },
            DecodeOptions::default(),
        )?;
        out.extend_from_slice(&bytes);
        recon = Some(decoded);
    }

    let hrd_max_occupancy = hrd.map(|h| h.max_occupancy_after_removal()).unwrap_or(0);
    Ok(RateControlledStream {
        bytes: out,
        picture_bits,
        picture_quants,
        hrd_conformant,
        hrd_max_occupancy,
    })
}

/// Encode a sequence of frames as an all-INTRA H.263 elementary stream.
///
/// Each frame is encoded as a baseline I-picture (via
/// [`encode_intra_picture`]) at the same `quant`, byte-aligned and
/// concatenated; the §5.1.2 Temporal Reference is assigned modulo 256
/// in presentation order starting from `tr0`. Every frame must share the
/// same standard source-format dimensions.
///
/// The resulting stream decodes through
/// [`crate::picture::decode_sequence`] into the same number of frames.
/// All-INTRA is the simplest valid stream (no inter-frame prediction);
/// P-picture encoding will let later frames reference their predecessor.
pub fn encode_intra_sequence(frames: &[YuvFrame], quant: u8, tr0: u8) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    for (i, frame) in frames.iter().enumerate() {
        let tr = tr0.wrapping_add(i as u8);
        let pic = encode_intra_picture(frame, quant, tr)?;
        out.extend_from_slice(&pic);
    }
    Ok(out)
}

/// Encode a sequence of frames as an all-INTRA Annex I **Advanced INTRA
/// Coding** elementary stream.
///
/// Each frame is encoded as a §I AIC I-picture (via
/// [`encode_intra_picture_aic_auto`], per-macroblock INTRA_MODE decision)
/// at the same `quant`, byte-aligned and concatenated; the §5.1.2
/// Temporal Reference is assigned modulo 256 in presentation order from
/// `tr0`.
///
/// The stream decodes through [`crate::picture::decode_sequence`] when
/// called with `DecodeOptions { aic: true, .. }` — each picture is a
/// baseline-PTYPE INTRA picture whose §I mode is carried by the decode
/// option (a baseline PTYPE cannot signal it on the wire).
pub fn encode_intra_sequence_aic(frames: &[YuvFrame], quant: u8, tr0: u8) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    for (i, frame) in frames.iter().enumerate() {
        let tr = tr0.wrapping_add(i as u8);
        let pic = encode_intra_picture_aic_auto(frame, quant, tr)?;
        out.extend_from_slice(&pic);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::{decode_picture_no_gob0_header, DecodeOptions};
    use crate::picture_header::{parse_picture_layer, H263PictureLayer};
    use crate::plus_ptype::{
        InheritedExtendedState, PlusPictureType, PlusSourceFormat, SliceStructuredSubmode, Uui,
    };
    use oxideav_core::bits::BitReader;

    /// Emit a PLUSPTYPE header and parse it back through the picture-layer
    /// parser, returning the extended header + a reader positioned at
    /// PQUANT.
    fn plus_header_round_trip(
        fmt: H263SourceFormat,
        quant: u8,
        tr: u8,
        is_inter: bool,
        modes: PlusModes,
    ) -> Vec<u8> {
        let mut w = BitWriter::new();
        write_plus_picture_header(&mut w, fmt, quant, tr, is_inter, modes).unwrap();
        w.align_to_byte_zero();
        w.finish()
    }

    #[test]
    fn plus_header_intra_no_modes_parses_back() {
        let bytes =
            plus_header_round_trip(H263SourceFormat::Qcif, 7, 42, false, PlusModes::default());
        let mut r = BitReader::new(&bytes);
        let layer = parse_picture_layer(&mut r, InheritedExtendedState::default()).unwrap();
        let ext = match layer {
            H263PictureLayer::Extended(e) => e,
            H263PictureLayer::Baseline(_) => panic!("expected extended header"),
        };
        assert_eq!(ext.prefix.temporal_reference, 42);
        assert_eq!(ext.plus.ufep, 0b001);
        let opp = ext.plus.opptype.expect("UFEP=001 carries OPPTYPE");
        assert_eq!(opp.source_format, PlusSourceFormat::Qcif);
        assert!(!opp.custom_pcf);
        assert!(!opp.umv && !opp.sac && !opp.advanced_prediction);
        assert!(!opp.advanced_intra && !opp.deblocking && !opp.slice_structured);
        assert!(!opp.reference_picture_selection && !opp.independent_segment_decoding);
        assert!(!opp.alternative_inter_vlc && !opp.modified_quantization);
        assert_eq!(ext.plus.mpptype.picture_type, PlusPictureType::Intra);
        assert!(!ext.plus.mpptype.reference_picture_resampling);
        assert!(!ext.plus.mpptype.reduced_resolution_update);
        assert!(!ext.plus.mpptype.rounding_type);
        assert!(!ext.plus.cpm);
        assert_eq!(ext.plus.uui, None);
        assert_eq!(ext.plus.sss, None);
        // §5.1.19 PQUANT + §5.1.24 PEI follow the parsed block.
        assert_eq!(r.read_u32(5).unwrap(), 7);
        assert!(!r.read_bit().unwrap());
    }

    #[test]
    fn plus_header_inter_aic_mq_parses_back() {
        let modes = PlusModes {
            advanced_intra: true,
            modified_quant: true,
            ..PlusModes::default()
        };
        let bytes = plus_header_round_trip(H263SourceFormat::Cif, 31, 3, true, modes);
        let mut r = BitReader::new(&bytes);
        let layer = parse_picture_layer(&mut r, InheritedExtendedState::default()).unwrap();
        let ext = match layer {
            H263PictureLayer::Extended(e) => e,
            H263PictureLayer::Baseline(_) => panic!("expected extended header"),
        };
        let opp = ext.plus.opptype.unwrap();
        assert_eq!(opp.source_format, PlusSourceFormat::Cif);
        assert!(opp.advanced_intra);
        assert!(opp.modified_quantization);
        assert!(!opp.umv && !opp.advanced_prediction && !opp.slice_structured);
        assert_eq!(ext.plus.mpptype.picture_type, PlusPictureType::Inter);
        assert_eq!(r.read_u32(5).unwrap(), 31);
        assert!(!r.read_bit().unwrap());
    }

    #[test]
    fn plus_header_umv_emits_limited_uui() {
        let modes = PlusModes {
            umv: true,
            ..PlusModes::default()
        };
        let bytes = plus_header_round_trip(H263SourceFormat::SubQcif, 12, 0, true, modes);
        let mut r = BitReader::new(&bytes);
        let layer = parse_picture_layer(&mut r, InheritedExtendedState::default()).unwrap();
        let ext = match layer {
            H263PictureLayer::Extended(e) => e,
            H263PictureLayer::Baseline(_) => panic!("expected extended header"),
        };
        assert!(ext.plus.opptype.unwrap().umv);
        assert_eq!(ext.plus.uui, Some(Uui::Limited));
        assert_eq!(r.read_u32(5).unwrap(), 12);
    }

    #[test]
    fn plus_header_slice_structured_sss_round_trips() {
        for &(rect, aso) in &[(false, false), (false, true), (true, false), (true, true)] {
            let modes = PlusModes {
                slice_structured: Some(SliceStructuredSubmode {
                    rectangular: rect,
                    arbitrary_order: aso,
                }),
                ..PlusModes::default()
            };
            let bytes = plus_header_round_trip(H263SourceFormat::Qcif, 5, 9, false, modes);
            let mut r = BitReader::new(&bytes);
            let layer = parse_picture_layer(&mut r, InheritedExtendedState::default()).unwrap();
            let ext = match layer {
                H263PictureLayer::Extended(e) => e,
                H263PictureLayer::Baseline(_) => panic!("expected extended header"),
            };
            assert!(ext.plus.opptype.unwrap().slice_structured);
            assert_eq!(
                ext.plus.sss,
                Some(SliceStructuredSubmode {
                    rectangular: rect,
                    arbitrary_order: aso,
                })
            );
            assert_eq!(r.read_u32(5).unwrap(), 5);
        }
    }

    #[test]
    fn plus_header_rejects_bad_inputs() {
        let mut w = BitWriter::new();
        assert!(matches!(
            write_plus_picture_header(
                &mut w,
                H263SourceFormat::Qcif,
                0,
                0,
                false,
                PlusModes::default()
            ),
            Err(Error::InvalidQuantiser)
        ));
        assert!(matches!(
            write_plus_picture_header(
                &mut w,
                H263SourceFormat::Reserved110,
                8,
                0,
                false,
                PlusModes::default()
            ),
            Err(Error::NotImplemented)
        ));
    }

    /// Build a deterministic test frame with a smooth gradient on each
    /// plane (so the encoder exercises both DC and AC paths).
    fn gradient_frame(lw: usize, lh: usize) -> YuvFrame {
        let cw = lw / 2;
        let ch = lh / 2;
        let mut y = vec![0u8; lw * lh];
        for row in 0..lh {
            for col in 0..lw {
                y[row * lw + col] = (40 + (col + row) % 160) as u8;
            }
        }
        let mut cb = vec![0u8; cw * ch];
        let mut cr = vec![0u8; cw * ch];
        for row in 0..ch {
            for col in 0..cw {
                cb[row * cw + col] = (90 + (col % 60)) as u8;
                cr[row * cw + col] = (110 + (row % 50)) as u8;
            }
        }
        YuvFrame {
            y,
            cb,
            cr,
            luma_width: lw,
            luma_height: lh,
        }
    }

    /// A flat grey frame encodes and decodes back to (almost) flat grey.
    #[test]
    fn flat_grey_qcif_round_trips() {
        let frame = YuvFrame::grey(176, 144);
        let bytes = encode_intra_picture(&frame, 8, 0).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.luma_width, 176);
        assert_eq!(decoded.luma_height, 144);
        // 128 -> DC 1024 (the 0xFF special) -> reconstruct 128.
        assert!(decoded.y.iter().all(|&p| p == 128), "luma not flat 128");
        assert!(decoded.cb.iter().all(|&p| p == 128));
        assert!(decoded.cr.iter().all(|&p| p == 128));
    }

    /// A gradient QCIF frame round-trips with bounded reconstruction
    /// error (the encoder + decoder are lossy but consistent).
    #[test]
    fn gradient_qcif_round_trips_within_tolerance() {
        let frame = gradient_frame(176, 144);
        let quant = 4;
        let bytes = encode_intra_picture(&frame, quant, 7).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default()).unwrap();

        // Compute mean absolute luma error.
        let mut sum = 0u64;
        let mut max = 0u32;
        for (a, b) in frame.y.iter().zip(decoded.y.iter()) {
            let e = (*a as i32 - *b as i32).unsigned_abs();
            sum += e as u64;
            max = max.max(e);
        }
        let mae = sum as f64 / frame.y.len() as f64;
        assert!(mae < 8.0, "luma MAE too high: {}", mae);
        assert!(max <= 40, "luma max error too high: {}", max);
    }

    /// A per-macroblock quantiser map driven through DQUANT decodes
    /// end-to-end and stays within INTRA tolerance. The left half of the
    /// picture is coded fine (low QUANT), the right half coarse (high
    /// QUANT); the encoder ramps QUANT in {-2,-1,+1,+2} steps via DQUANT.
    #[test]
    fn intra_picture_dquant_round_trips() {
        let frame = gradient_frame(176, 144);
        let pquant = 4;
        let mb_cols = 176 / 16;
        let bytes =
            encode_intra_picture_dquant(
                &frame,
                pquant,
                0,
                |col, _row| {
                    if col < mb_cols / 2 {
                        4
                    } else {
                        16
                    }
                },
            )
            .unwrap();
        let decoded =
            decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default()).unwrap();
        assert_eq!((decoded.luma_width, decoded.luma_height), (176, 144));
        // Whole-frame mean error stays bounded even with the coarse half.
        let mut sum = 0u64;
        for (a, b) in frame.y.iter().zip(decoded.y.iter()) {
            sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mae = sum as f64 / frame.y.len() as f64;
        assert!(mae < 16.0, "DQUANT-mapped INTRA luma MAE too high: {}", mae);
    }

    /// `reach_quant` only moves QUANT by a legal §5.3.6 step and clamps
    /// to 1..=31.
    #[test]
    fn reach_quant_respects_step_and_clamp() {
        assert_eq!(reach_quant(10, 10), (10, None));
        assert_eq!(reach_quant(10, 12), (12, Some(2)));
        assert_eq!(reach_quant(10, 11), (11, Some(1)));
        assert_eq!(reach_quant(10, 8), (8, Some(-2)));
        // Target far above: only +2 reachable in one step.
        assert_eq!(reach_quant(10, 31), (12, Some(2)));
        // At the ceiling: +1 would exceed 31, so clamp keeps movement legal.
        assert_eq!(reach_quant(31, 31), (31, None));
        assert_eq!(reach_quant(1, 1), (1, None));
    }

    /// sub-QCIF dimensions are accepted.
    #[test]
    fn sub_qcif_supported() {
        let frame = YuvFrame::grey(128, 96);
        let bytes = encode_intra_picture(&frame, 10, 0).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default()).unwrap();
        assert_eq!((decoded.luma_width, decoded.luma_height), (128, 96));
    }

    /// CIF dimensions are accepted and round-trip.
    #[test]
    fn cif_gradient_round_trips() {
        let frame = gradient_frame(352, 288);
        let bytes = encode_intra_picture(&frame, 6, 0).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default()).unwrap();
        assert_eq!((decoded.luma_width, decoded.luma_height), (352, 288));
    }

    /// Non-standard dimensions are rejected.
    #[test]
    fn non_standard_dimensions_rejected() {
        let frame = YuvFrame::grey(160, 120);
        assert!(matches!(
            encode_intra_picture(&frame, 8, 0),
            Err(Error::NotImplemented)
        ));
    }

    /// Out-of-range quantiser is rejected.
    #[test]
    fn bad_quant_rejected() {
        let frame = YuvFrame::grey(176, 144);
        assert!(matches!(
            encode_intra_picture(&frame, 0, 0),
            Err(Error::InvalidQuantiser)
        ));
        assert!(matches!(
            encode_intra_picture(&frame, 32, 0),
            Err(Error::InvalidQuantiser)
        ));
    }

    /// The encoded stream is byte-aligned and begins with a PSC.
    #[test]
    fn output_starts_with_psc_and_is_byte_aligned() {
        let frame = YuvFrame::grey(176, 144);
        let bytes = encode_intra_picture(&frame, 8, 0).unwrap();
        // First two bytes are 0x00 0x00, third byte top two bits are 10.
        assert_eq!(bytes[0], 0x00);
        assert_eq!(bytes[1], 0x00);
        assert_eq!(bytes[2] & 0b1100_0000, 0b1000_0000);
    }

    /// A static P-picture (identical to its reference) encodes to an
    /// all-skipped frame and reconstructs to the reference exactly.
    #[test]
    fn static_inter_picture_is_lossless() {
        // Reference is the *decoded* I-picture, so the P-frame predicts
        // from exactly what the decoder will hold.
        let src = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&src, 6, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        // P-frame whose source equals the reference -> zero residual.
        let p_bytes = encode_inter_picture(&recon_ref, &recon_ref, 6, 1).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&p_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        assert_eq!(decoded.y, recon_ref.y, "static P luma must equal reference");
        assert_eq!(decoded.cb, recon_ref.cb);
        assert_eq!(decoded.cr, recon_ref.cr);
    }

    /// A moving P-picture round-trips against its reconstructed
    /// reference with bounded error.
    #[test]
    fn moving_inter_picture_round_trips_within_tolerance() {
        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        // frame1 = frame0 brightened by a small constant (a residual the
        // INTER path must carry).
        let mut frame1 = frame0.clone();
        for p in frame1.y.iter_mut() {
            *p = (*p as i32 + 20).min(255) as u8;
        }

        let p_bytes = encode_inter_picture(&frame1, &recon_ref, 5, 1).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&p_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();

        let mut sum = 0u64;
        for (a, b) in frame1.y.iter().zip(decoded.y.iter()) {
            sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mae = sum as f64 / frame1.y.len() as f64;
        assert!(mae < 10.0, "INTER luma MAE too high: {}", mae);
    }

    /// A horizontally-translated frame: motion-compensated INTER
    /// encoding round-trips with bounded error and beats zero-motion
    /// encoding on the same translated content.
    #[test]
    fn motion_compensated_inter_round_trips_and_beats_zero_motion() {
        // Reference = decoded I-picture of frame0.
        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        // frame1 = frame0 translated left by 2 px (content moves), built
        // from the *reconstructed* reference so the only error source is
        // the residual quantiser.
        let lw = 176;
        let lh = 144;
        let mut frame1 = recon_ref.clone();
        for row in 0..lh {
            for col in 0..lw {
                let srccol = (col + 2).min(lw - 1);
                frame1.y[row * lw + col] = recon_ref.y[row * lw + srccol];
            }
        }

        // Motion-compensated encode.
        let mc_bytes = encode_inter_picture_motion(&frame1, &recon_ref, 5, 1, 4).unwrap();
        let mc_decoded =
            decode_picture_no_gob0_header(&mc_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        let mut mc_sum = 0u64;
        for (a, b) in frame1.y.iter().zip(mc_decoded.y.iter()) {
            mc_sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mc_mae = mc_sum as f64 / frame1.y.len() as f64;

        // Zero-motion encode of the same content.
        let zm_bytes = encode_inter_picture(&frame1, &recon_ref, 5, 1).unwrap();

        // Motion compensation should reconstruct accurately (the shifted
        // content is exactly available in the reference).
        assert!(mc_mae < 6.0, "MC luma MAE too high: {}", mc_mae);
        // And it should be more compact than zero-motion (fewer residual
        // bits for translational motion).
        assert!(
            mc_bytes.len() <= zm_bytes.len(),
            "MC stream ({}) not smaller than zero-motion ({})",
            mc_bytes.len(),
            zm_bytes.len()
        );
    }

    /// An INTRA picture with a header on every GOB reconstructs
    /// **identically** to the header-less single-segment encode at the
    /// same quantiser (the headers change framing, not coefficients),
    /// and the stream is longer by the header bits.
    #[test]
    fn intra_gob_headers_reconstruct_identically() {
        let frame = gradient_frame(176, 144);
        let plain = encode_intra_picture(&frame, 6, 3).unwrap();
        let gobs = encode_intra_picture_gobs(&frame, 3, |_| 6).unwrap();
        assert!(
            gobs.len() > plain.len(),
            "GOB headers should add bytes ({} vs {})",
            gobs.len(),
            plain.len()
        );
        let d_plain =
            decode_picture_no_gob0_header(&plain, None, DecodeOptions::default()).unwrap();
        let d_gobs = decode_picture_no_gob0_header(&gobs, None, DecodeOptions::default()).unwrap();
        assert_eq!(d_plain.y, d_gobs.y);
        assert_eq!(d_plain.cb, d_gobs.cb);
        assert_eq!(d_plain.cr, d_gobs.cr);
    }

    /// A per-GOB quantiser map (fine top half, coarse bottom half)
    /// primes each GOB's QUANT from its GQUANT and decodes end-to-end
    /// within tolerance — unlike DQUANT, GQUANT may jump arbitrarily.
    #[test]
    fn intra_gob_quant_map_round_trips() {
        let frame = gradient_frame(176, 144);
        // QCIF: 9 GOBs of one MB row each. Top 4 fine, bottom 5 coarse
        // (a jump 4 -> 20 that DQUANT could not express in one step).
        let bytes = encode_intra_picture_gobs(&frame, 0, |gn| if gn < 4 { 4 } else { 20 }).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default()).unwrap();
        let lw = 176;
        // The fine half is tight; the whole frame stays bounded.
        let mut top_sum = 0u64;
        for row in 0..64 {
            for col in 0..lw {
                top_sum += (frame.y[row * lw + col] as i32 - decoded.y[row * lw + col] as i32)
                    .unsigned_abs() as u64;
            }
        }
        let top_mae = top_sum as f64 / (64 * lw) as f64;
        assert!(top_mae < 8.0, "fine-GOB luma MAE too high: {}", top_mae);
        let mut sum = 0u64;
        for (a, b) in frame.y.iter().zip(decoded.y.iter()) {
            sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mae = sum as f64 / frame.y.len() as f64;
        assert!(mae < 20.0, "whole-frame luma MAE too high: {}", mae);
    }

    /// An out-of-range per-GOB quantiser is rejected.
    #[test]
    fn intra_gob_bad_quant_rejected() {
        let frame = YuvFrame::grey(176, 144);
        assert!(matches!(
            encode_intra_picture_gobs(&frame, 0, |_| 0),
            Err(Error::InvalidQuantiser)
        ));
        assert!(matches!(
            encode_intra_picture_gobs(&frame, 0, |gn| if gn == 3 { 32 } else { 8 }),
            Err(Error::InvalidQuantiser)
        ));
    }

    /// A motion-compensated P-picture with per-GOB headers round-trips:
    /// the encoder's per-GOB predictor segmentation matches the
    /// decoder's rule-3 treatment of header-carrying GOBs, so every
    /// MVD reconstructs to the searched vector.
    #[test]
    fn inter_gob_headers_round_trip() {
        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();
        let frame1 = translate_left(&recon_ref, 3);

        let p_bytes = encode_inter_picture_gobs(&frame1, &recon_ref, 5, 1, 4).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&p_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        let mut sum = 0u64;
        for (a, b) in frame1.y.iter().zip(decoded.y.iter()) {
            sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mae = sum as f64 / frame1.y.len() as f64;
        assert!(mae < 6.0, "GOB-header INTER luma MAE too high: {}", mae);
    }

    /// A static Advanced-Prediction P-picture is lossless: every
    /// macroblock is coded INTER4V with zero vectors, the §F.3 OBMC
    /// blend of all-zero vectors is the plain co-located copy, and the
    /// residual quantises to zero — so the decoder reproduces the
    /// reference exactly. The PTYPE AP flag is on the wire.
    #[test]
    fn ap_static_picture_is_lossless() {
        use crate::picture_header::parse_picture_header;
        use oxideav_core::bits::BitReader;

        let src = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&src, 6, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        let p_bytes = encode_inter_picture_ap(&recon_ref, &recon_ref, 6, 1, 2).unwrap();
        let mut r = BitReader::new(&p_bytes);
        let header = parse_picture_header(&mut r).unwrap();
        assert!(header.advanced_prediction, "PTYPE AP flag not set");
        assert!(!header.umv_mode);

        let decoded =
            decode_picture_no_gob0_header(&p_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        assert_eq!(decoded.y, recon_ref.y, "static AP luma must be lossless");
        assert_eq!(decoded.cb, recon_ref.cb);
        assert_eq!(decoded.cr, recon_ref.cr);
    }

    /// Divergent intra-macroblock motion (the right 8-pixel half of
    /// every macroblock shifts, the left half stays) — exactly what
    /// four vectors per macroblock exist for. The AP encode
    /// round-trips within tolerance and spends far fewer bits than the
    /// zero-motion encoder on the same content.
    #[test]
    fn ap_shear_content_round_trips_and_beats_zero_motion() {
        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        // Right half of every macroblock samples 3 px to the right;
        // left half is static. Per-8×8-block vectors capture this,
        // a single MV per macroblock cannot.
        let lw = 176;
        let lh = 144;
        let mut frame1 = recon_ref.clone();
        for row in 0..lh {
            for col in 0..lw {
                let shift = if col % 16 >= 8 { 3 } else { 0 };
                let src = (col + shift).min(lw - 1);
                frame1.y[row * lw + col] = recon_ref.y[row * lw + src];
            }
        }

        let ap_bytes = encode_inter_picture_ap(&frame1, &recon_ref, 5, 1, 4).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&ap_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        let mut sum = 0u64;
        for (a, b) in frame1.y.iter().zip(decoded.y.iter()) {
            sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mae = sum as f64 / frame1.y.len() as f64;
        assert!(mae < 8.0, "AP shear luma MAE too high: {}", mae);

        // Zero-motion coding of the same content has to carry the
        // whole divergence as residual.
        let zm_bytes = encode_inter_picture(&frame1, &recon_ref, 5, 1).unwrap();
        assert!(
            ap_bytes.len() < zm_bytes.len(),
            "AP stream ({}) not smaller than zero-motion ({})",
            ap_bytes.len(),
            zm_bytes.len()
        );
    }

    /// A translated frame round-trips through the AP encoder (uniform
    /// motion — all four vectors of each macroblock converge, OBMC
    /// remotes agree across macroblocks).
    #[test]
    fn ap_translated_frame_round_trips() {
        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();
        let frame1 = translate_left(&recon_ref, 2);

        let ap_bytes = encode_inter_picture_ap(&frame1, &recon_ref, 5, 1, 4).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&ap_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        let mut sum = 0u64;
        for (a, b) in frame1.y.iter().zip(decoded.y.iter()) {
            sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mae = sum as f64 / frame1.y.len() as f64;
        assert!(mae < 6.0, "AP translated luma MAE too high: {}", mae);
    }

    /// Translate a frame's content left by `shift` luma pixels with
    /// edge replication (the rightmost column repeats), on all planes.
    fn translate_left(frame: &YuvFrame, shift: usize) -> YuvFrame {
        let lw = frame.luma_width;
        let lh = frame.luma_height;
        let cw = frame.chroma_width();
        let ch = frame.chroma_height();
        let mut out = frame.clone();
        for row in 0..lh {
            for col in 0..lw {
                let src = (col + shift).min(lw - 1);
                out.y[row * lw + col] = frame.y[row * lw + src];
            }
        }
        let cshift = shift / 2;
        for row in 0..ch {
            for col in 0..cw {
                let src = (col + cshift).min(cw - 1);
                out.cb[row * cw + col] = frame.cb[row * cw + src];
                out.cr[row * cw + col] = frame.cr[row * cw + src];
            }
        }
        out
    }

    /// A 20-pixel translation — beyond the default ±16-pixel MV range —
    /// round-trips through the Annex D UMV encoder with low error and
    /// fewer bits than the default-mode encoder needs for the same
    /// content, and the stream carries the PTYPE UMV flag.
    #[test]
    fn umv_inter_large_shift_round_trips_and_beats_default() {
        use crate::picture_header::parse_picture_header;
        use oxideav_core::bits::BitReader;

        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        // Content moves 20 px — matching pixels sit 20 px to the right
        // in the reference (best MV = +40 half-pel, outside [-32, 31]).
        let frame1 = translate_left(&recon_ref, 20);

        let umv_bytes = encode_inter_picture_umv(&frame1, &recon_ref, 5, 1, 22).unwrap();
        let base_bytes = encode_inter_picture_motion(&frame1, &recon_ref, 5, 1, 22).unwrap();

        // The UMV stream signals Annex D in PTYPE.
        let mut r = BitReader::new(&umv_bytes);
        let header = parse_picture_header(&mut r).unwrap();
        assert!(header.umv_mode, "PTYPE UMV flag not set");
        assert!(!header.advanced_prediction);

        let decoded =
            decode_picture_no_gob0_header(&umv_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        let mut sum = 0u64;
        for (a, b) in frame1.y.iter().zip(decoded.y.iter()) {
            sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
        }
        let mae = sum as f64 / frame1.y.len() as f64;
        assert!(mae < 8.0, "UMV luma MAE too high: {}", mae);

        // Beyond-range motion costs the default mode much more bits
        // (large residuals / intra refresh); UMV codes it as motion.
        assert!(
            umv_bytes.len() < base_bytes.len(),
            "UMV stream ({}) not smaller than default-mode stream ({})",
            umv_bytes.len(),
            base_bytes.len()
        );
    }

    /// A static UMV P-picture is all-skipped and lossless, exactly like
    /// the default mode (the UMV flag changes only MV interpretation).
    #[test]
    fn umv_static_inter_picture_is_lossless() {
        let src = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&src, 6, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        let p_bytes = encode_inter_picture_umv(&recon_ref, &recon_ref, 6, 1, 4).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&p_bytes, Some(&recon_ref), DecodeOptions::default())
                .unwrap();
        assert_eq!(decoded.y, recon_ref.y);
        assert_eq!(decoded.cb, recon_ref.cb);
        assert_eq!(decoded.cr, recon_ref.cr);
    }

    /// When the reference is unrelated to the source, the P-picture
    /// mode decision picks INTRA macroblocks (Table 8 INTRA), and the
    /// result reconstructs the *source* (not the reference) within the
    /// INTRA tolerance.
    #[test]
    fn unrelated_inter_picture_falls_back_to_intra() {
        // Reference is a mid-grey field; source is a bright gradient
        // with nothing in common — INTER prediction is useless.
        let reference = YuvFrame::grey(176, 144);
        let source = gradient_frame(176, 144);

        let p_bytes = encode_inter_picture_motion(&source, &reference, 5, 1, 2).unwrap();
        let decoded =
            decode_picture_no_gob0_header(&p_bytes, Some(&reference), DecodeOptions::default())
                .unwrap();

        // The decoded frame should track the SOURCE (via INTRA refresh),
        // not the grey reference.
        let mut src_sum = 0u64;
        let mut ref_sum = 0u64;
        for i in 0..source.y.len() {
            src_sum += (source.y[i] as i32 - decoded.y[i] as i32).unsigned_abs() as u64;
            ref_sum += (reference.y[i] as i32 - decoded.y[i] as i32).unsigned_abs() as u64;
        }
        let src_mae = src_sum as f64 / source.y.len() as f64;
        let ref_mae = ref_sum as f64 / source.y.len() as f64;
        assert!(
            src_mae < 8.0,
            "decoded should track source, MAE {}",
            src_mae
        );
        assert!(
            src_mae < ref_mae,
            "decoded closer to reference ({}) than source ({})",
            ref_mae,
            src_mae
        );
    }

    /// A mismatched reference is rejected.
    #[test]
    fn inter_mismatched_reference_rejected() {
        let frame = YuvFrame::grey(176, 144);
        let bad_ref = YuvFrame::grey(128, 96);
        assert!(matches!(
            encode_inter_picture(&frame, &bad_ref, 8, 1),
            Err(Error::NotImplemented)
        ));
    }

    /// A static PB-frame (B == P == reference) is all-skipped and both
    /// parts reconstruct the reference exactly.
    #[test]
    fn pb_static_pair_is_lossless() {
        use crate::picture::decode_pb_picture_no_gob0_header;
        let src = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&src, 6, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

        let cfg = PbConfig {
            quant: 6,
            trb: 1,
            dbquant: 0,
            search_half: 2,
            b_search_half: 0,
        };
        let pb_bytes = encode_pb_picture(&recon_ref, &recon_ref, &recon_ref, 2, 0, &cfg).unwrap();
        let pair =
            decode_pb_picture_no_gob0_header(&pb_bytes, &recon_ref, 0, DecodeOptions::default())
                .unwrap();
        assert_eq!(
            pair.p_frame.y, recon_ref.y,
            "static P-part must be lossless"
        );
        assert_eq!(
            pair.b_frame.y, recon_ref.y,
            "static B-part must be lossless"
        );
        assert_eq!(pair.p_frame.cb, recon_ref.cb);
        assert_eq!(pair.b_frame.cr, recon_ref.cr);
    }

    /// A translating three-frame set (reference, B at the midpoint, P):
    /// the PB-frame round-trips with both parts tracking their sources
    /// — the B-part via the §G.4 scaled vectors plus coded B-residual.
    #[test]
    fn pb_translating_pair_round_trips() {
        use crate::picture::decode_pb_picture_no_gob0_header;
        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();
        // B halfway (1 px), P at 2 px: linear motion, TRB/TRD = 1/2.
        let b_src = translate_left(&recon_ref, 1);
        let p_src = translate_left(&recon_ref, 2);

        let cfg = PbConfig {
            quant: 5,
            trb: 1,
            dbquant: 0,
            search_half: 3,
            b_search_half: 0,
        };
        let pb_bytes = encode_pb_picture(&p_src, &b_src, &recon_ref, 2, 0, &cfg).unwrap();
        let pair =
            decode_pb_picture_no_gob0_header(&pb_bytes, &recon_ref, 0, DecodeOptions::default())
                .unwrap();

        let mae = |a: &YuvFrame, b: &YuvFrame| -> f64 {
            let mut sum = 0u64;
            for (x, y) in a.y.iter().zip(b.y.iter()) {
                sum += (*x as i32 - *y as i32).unsigned_abs() as u64;
            }
            sum as f64 / a.y.len() as f64
        };
        let p_mae = mae(&p_src, &pair.p_frame);
        let b_mae = mae(&b_src, &pair.b_frame);
        assert!(p_mae < 6.0, "P-part luma MAE too high: {p_mae}");
        assert!(b_mae < 8.0, "B-part luma MAE too high: {b_mae}");
    }

    /// An I + PB elementary stream decodes through `decode_sequence`
    /// into three frames in display order [I, B, P].
    #[test]
    fn pb_stream_decodes_in_display_order() {
        use crate::picture::decode_sequence;
        let frame0 = gradient_frame(176, 144);
        let i_bytes = encode_intra_picture(&frame0, 5, 0).unwrap();
        let recon_ref =
            decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();
        let b_src = translate_left(&recon_ref, 1);
        let p_src = translate_left(&recon_ref, 2);

        let cfg = PbConfig {
            quant: 5,
            trb: 1,
            dbquant: 0,
            search_half: 3,
            b_search_half: 0,
        };
        let pb_bytes = encode_pb_picture(&p_src, &b_src, &recon_ref, 2, 0, &cfg).unwrap();
        let mut stream = i_bytes.clone();
        stream.extend_from_slice(&pb_bytes);

        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.len(), 3, "expected [I, B, P]");
        // Display order: the middle frame is the B-part (closest to
        // b_src), the last is the P-part.
        let mae = |a: &YuvFrame, b: &YuvFrame| -> f64 {
            let mut sum = 0u64;
            for (x, y) in a.y.iter().zip(b.y.iter()) {
                sum += (*x as i32 - *y as i32).unsigned_abs() as u64;
            }
            sum as f64 / a.y.len() as f64
        };
        assert!(mae(&b_src, &decoded[1]) < 8.0, "middle frame should be B");
        assert!(mae(&p_src, &decoded[2]) < 6.0, "last frame should be P");
        assert!(
            mae(&b_src, &decoded[1]) < mae(&p_src, &decoded[1]),
            "middle decoded frame closer to P than to B — order wrong"
        );
    }

    /// PB parameter validation: TRB and TRD constraints.
    #[test]
    fn pb_bad_parameters_rejected() {
        let f = YuvFrame::grey(176, 144);
        let bad_trb = PbConfig {
            trb: 0,
            ..PbConfig::default()
        };
        assert!(matches!(
            encode_pb_picture(&f, &f, &f, 2, 0, &bad_trb),
            Err(Error::BadPbTemporalReference)
        ));
        // TRD = 0 (same TR).
        assert!(matches!(
            encode_pb_picture(&f, &f, &f, 5, 5, &PbConfig::default()),
            Err(Error::BadPbTemporalReference)
        ));
        // TRB >= TRD.
        let cfg = PbConfig {
            trb: 2,
            ..PbConfig::default()
        };
        assert!(matches!(
            encode_pb_picture(&f, &f, &f, 2, 0, &cfg),
            Err(Error::BadPbTemporalReference)
        ));
    }

    /// A six-frame I+P GOP (intra_period = 3) round-trips: the stream
    /// holds six pictures, each decoded frame tracks its source within
    /// tolerance, and the closed loop keeps late P-frames as accurate
    /// as early ones (no drift).
    #[test]
    fn gop_sequence_round_trips_without_drift() {
        use crate::picture::decode_sequence;
        // Six frames translating 1 px per frame.
        let base = gradient_frame(176, 144);
        let frames: Vec<YuvFrame> = (0..6)
            .map(|k| {
                let lw = 176;
                let mut f = base.clone();
                for row in 0..144 {
                    for col in 0..lw {
                        let src = (col + k).min(lw - 1);
                        f.y[row * lw + col] = base.y[row * lw + src];
                    }
                }
                f
            })
            .collect();

        let cfg = GopConfig {
            quant: 5,
            intra_period: 3,
            search_half: 3,
            ..GopConfig::default()
        };
        let stream = encode_sequence(&frames, &cfg, 0).unwrap();
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.len(), 6, "expected 6 decoded frames");
        let mut maes = Vec::new();
        for (src, dec) in frames.iter().zip(decoded.iter()) {
            let mut sum = 0u64;
            for (a, b) in src.y.iter().zip(dec.y.iter()) {
                sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
            }
            maes.push(sum as f64 / src.y.len() as f64);
        }
        for (i, mae) in maes.iter().enumerate() {
            assert!(*mae < 8.0, "frame {i} luma MAE too high: {mae}");
        }
        // Closed loop: the last P-frame is not meaningfully worse than
        // the first (drift would grow the error monotonically).
        assert!(
            maes[5] < maes[1] + 4.0,
            "drift suspected: MAE grew from {} to {}",
            maes[1],
            maes[5]
        );
    }

    /// The §5.1.27 EOS marker terminates the stream byte-aligned and
    /// is transparent to the sequence decoder.
    #[test]
    fn gop_sequence_eos_appended_and_transparent() {
        use crate::picture::decode_sequence;
        let frames = vec![gradient_frame(176, 144), YuvFrame::grey(176, 144)];
        let cfg = GopConfig {
            quant: 6,
            intra_period: 1, // all-INTRA
            eos: true,
            ..GopConfig::default()
        };
        let stream = encode_sequence(&frames, &cfg, 0).unwrap();
        assert!(
            stream.ends_with(&EOS_BYTES),
            "stream must end with the byte-aligned EOS codeword"
        );
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.len(), 2);
        assert!(decoded[1].y.iter().all(|&p| p == 128));
    }

    /// The UMV P-picture path threads through the GOP driver.
    #[test]
    fn gop_sequence_umv_path_round_trips() {
        use crate::picture::decode_sequence;
        let base = gradient_frame(176, 144);
        let frames: Vec<YuvFrame> = (0..3)
            .map(|k| {
                let lw = 176;
                let mut f = base.clone();
                for row in 0..144 {
                    for col in 0..lw {
                        let src = (col + 4 * k).min(lw - 1);
                        f.y[row * lw + col] = base.y[row * lw + src];
                    }
                }
                f
            })
            .collect();
        let cfg = GopConfig {
            quant: 5,
            intra_period: 0, // I then P, P
            search_half: 5,
            umv: true,
            ..GopConfig::default()
        };
        let stream = encode_sequence(&frames, &cfg, 0).unwrap();
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.len(), 3);
        for (i, (src, dec)) in frames.iter().zip(decoded.iter()).enumerate() {
            let mut sum = 0u64;
            for (a, b) in src.y.iter().zip(dec.y.iter()) {
                sum += (*a as i32 - *b as i32).unsigned_abs() as u64;
            }
            let mae = sum as f64 / src.y.len() as f64;
            assert!(mae < 8.0, "UMV GOP frame {i} MAE too high: {mae}");
        }
    }

    /// An all-INTRA sequence of three frames decodes back to three
    /// frames through `decode_sequence`, each TR incrementing.
    #[test]
    fn intra_sequence_round_trips_to_n_frames() {
        use crate::picture::decode_sequence;
        let frames = vec![
            gradient_frame(176, 144),
            YuvFrame::grey(176, 144),
            gradient_frame(176, 144),
        ];
        let stream = encode_intra_sequence(&frames, 5, 0).unwrap();
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.len(), 3, "expected 3 decoded frames");
        for d in &decoded {
            assert_eq!((d.luma_width, d.luma_height), (176, 144));
        }
        // The flat-grey middle frame reconstructs to flat 128.
        assert!(decoded[1].y.iter().all(|&p| p == 128));
    }
}
