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

/// Map standard luma dimensions to the §5.1.3 source-format selector.
fn source_format_for(luma_w: usize, luma_h: usize) -> Option<H263SourceFormat> {
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
fn extract_macroblock(frame: &YuvFrame, mb_col: usize, mb_row: usize) -> MacroblockSamples {
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
struct PtypeFlags {
    /// Bit 10 — Annex D Unrestricted Motion Vector mode.
    umv: bool,
    /// Bit 12 — Annex F Advanced Prediction mode.
    advanced_prediction: bool,
}

/// Write the §5.1 baseline picture header (PSC, TR, PTYPE, PQUANT,
/// CPM=0, PEI=0). `is_inter` selects the §5.1.3 picture coding-type bit
/// (INTRA = 0, INTER = 1); `flags` raises the optional-mode PTYPE bits
/// (SAC stays 0 on this path). `pb_fields` — `Some((trb, dbquant))` —
/// raises the PTYPE bit-13 PB-frames flag and emits the §5.1.22 TRB +
/// §5.1.23 DBQUANT fields between CPM and PEI.
fn write_picture_header(
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
    w.write_bit(false); // SAC (Annex E)
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

/// Encoder-side mirror of the decoder's `AicState` §I.3 neighbour grid,
/// for a **single video picture segment** (a header-less I-picture, so
/// every already-encoded in-bounds block is an available `RecA'` /
/// `RecB'` predictor — no segment-id bookkeeping is required).
///
/// Holds the reconstructed `RecC'(u,v)` block-position arrays keyed by
/// the same grid coordinates the decoder uses (`luma_block_grid_pos`):
/// luma at `2·mb_cols × 2·mb_rows`, chroma at `mb_cols × mb_rows`.
struct AicEncodeGrid {
    luma_block_cols: usize,
    mb_cols: usize,
    /// `RecC'` per luma block; `None` until the block has been encoded.
    luma: Vec<Option<[i32; COEFFS_PER_BLOCK]>>,
    cb: Vec<Option<[i32; COEFFS_PER_BLOCK]>>,
    cr: Vec<Option<[i32; COEFFS_PER_BLOCK]>>,
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
        }
    }

    /// The `RecA'` (above) / `RecB'` (left) luma neighbour arrays at grid
    /// position `(bx, by)`, copied out so the caller can build
    /// [`Neighbour`] tags without borrowing the grid while it mutates.
    fn luma_neighbours(
        &self,
        bx: usize,
        by: usize,
    ) -> (
        Option<[i32; COEFFS_PER_BLOCK]>,
        Option<[i32; COEFFS_PER_BLOCK]>,
    ) {
        let above = if by > 0 {
            self.luma[(by - 1) * self.luma_block_cols + bx]
        } else {
            None
        };
        let left = if bx > 0 {
            self.luma[by * self.luma_block_cols + (bx - 1)]
        } else {
            None
        };
        (above, left)
    }

    fn chroma_neighbours(
        planes: &[Option<[i32; COEFFS_PER_BLOCK]>],
        col: usize,
        row: usize,
        cols: usize,
    ) -> (
        Option<[i32; COEFFS_PER_BLOCK]>,
        Option<[i32; COEFFS_PER_BLOCK]>,
    ) {
        let above = if row > 0 {
            planes[(row - 1) * cols + col]
        } else {
            None
        };
        let left = if col > 0 {
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
}

/// Plan one AIC INTRA macroblock with a fixed INTRA_MODE, reading the
/// §I.3 neighbour predictors from `grid` **without** mutating it.
///
/// The four luma blocks chain intra-macroblock: Y2's above is Y0, Y3's
/// left is Y2, Y4's above/left are Y1/Y3 — those reconstructions come
/// from a local scratch (not yet in the grid), while the top row / left
/// column of the macroblock read the already-committed neighbours from
/// `grid` (Figure-5 order, matching the decoder).
fn plan_macroblock_aic(
    grid: &AicEncodeGrid,
    mb: &MacroblockSamples,
    quant: u8,
    mode: IntraMode,
    mb_col: usize,
    mb_row: usize,
) -> MbAicPlan {
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
            grid.luma_neighbours(bx, by).0
        };
        // Left neighbour: an intra-macroblock block (Y0/Y2) when this is a
        // right-column block, else the committed grid block to the left.
        let left = if (blk & 1) == 1 {
            local[blk - 1]
        } else {
            grid.luma_neighbours(bx, by).1
        };
        let plan = plan_intra_block_aic(
            &mb.luma[blk],
            mode,
            quant,
            neigh(&above),
            neigh(&left),
            false,
        );
        local[blk] = Some(plan.rec);
        luma[blk] = Some(plan);
    }

    let (cb_above, cb_left) =
        AicEncodeGrid::chroma_neighbours(&grid.cb, mb_col, mb_row, grid.mb_cols);
    let cb = plan_intra_block_aic(
        &mb.cb,
        mode,
        quant,
        neigh(&cb_above),
        neigh(&cb_left),
        false,
    );

    let (cr_above, cr_left) =
        AicEncodeGrid::chroma_neighbours(&grid.cr, mb_col, mb_row, grid.mb_cols);
    let cr = plan_intra_block_aic(
        &mb.cr,
        mode,
        quant,
        neigh(&cr_above),
        neigh(&cr_left),
        false,
    );

    MbAicPlan {
        mode,
        luma: luma.map(|p| p.unwrap()),
        cb,
        cr,
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
    for p in &plan.luma {
        write_intra_block_aic(w, p, false)?;
    }
    write_intra_block_aic(w, &plan.cb, false)?;
    write_intra_block_aic(w, &plan.cr, false)?;
    Ok(())
}

/// Store a planned macroblock's reconstructions into `grid` so downstream
/// macroblocks pick them up as §I.3 neighbours.
fn commit_macroblock_aic(grid: &mut AicEncodeGrid, plan: &MbAicPlan, mb_col: usize, mb_row: usize) {
    for (blk, p) in plan.luma.iter().enumerate() {
        let bx = 2 * mb_col + (blk & 1);
        let by = 2 * mb_row + (blk >> 1);
        grid.luma[by * grid.luma_block_cols + bx] = Some(p.rec);
    }
    grid.cb[mb_row * grid.mb_cols + mb_col] = Some(plan.cb.rec);
    grid.cr[mb_row * grid.mb_cols + mb_col] = Some(plan.cr.rec);
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
    quant: u8,
    fixed_mode: Option<IntraMode>,
    mb_col: usize,
    mb_row: usize,
    grid: &mut AicEncodeGrid,
) -> Result<()> {
    let plan = match fixed_mode {
        Some(mode) => plan_macroblock_aic(grid, mb, quant, mode, mb_col, mb_row),
        None => {
            let mut best: Option<(u64, MbAicPlan)> = None;
            for mode in [
                IntraMode::DcOnly,
                IntraMode::VerticalDcAc,
                IntraMode::HorizontalDcAc,
            ] {
                let candidate = plan_macroblock_aic(grid, mb, quant, mode, mb_col, mb_row);
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
    commit_macroblock_aic(grid, &plan, mb_col, mb_row);
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

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    let mut grid = AicEncodeGrid::new(mb_cols, mb_rows);
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_choose_macroblock_aic(
                &mut w,
                &mb,
                quant,
                Some(mode),
                mb_col,
                mb_row,
                &mut grid,
            )?;
        }
    }

    // §5.1.28 — PSTUF.
    w.align_to_byte_zero();
    Ok(w.finish())
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

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    let mut grid = AicEncodeGrid::new(mb_cols, mb_rows);
    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_choose_macroblock_aic(&mut w, &mb, quant, None, mb_col, mb_row, &mut grid)?;
        }
    }

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
fn residual_of(
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
fn motion_compensated_block(
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
        /* gob_headers */ false,
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
        /* gob_headers */ true,
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
        /* gob_headers */ false,
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
    use crate::encoder_motion::{estimate_block_motion, mvd_for, Mv4Grid};
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

    // ---- Pass 2: §F.3 OBMC prediction + residual coding. -------------
    let mut w = BitWriter::new();
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
            crate::encoder_mb::encode_inter4v_macroblock(
                &mut w, &luma_arr, &cb_enc, &cr_enc, &mvds,
            )?;
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

/// Shared motion-estimated INTER picture encode: the default-mode
/// (§6.1.1 wrap, `umv = false`) and Annex D UMV (`umv = true`) paths
/// differ only in the PTYPE bit-10 flag, the estimator range and the
/// MVD derivation; `gob_headers` selects the §5.2 every-GOB-header
/// stream shape (with the per-GOB predictor segmentation).
fn encode_inter_picture_motion_impl(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    search_half: i32,
    umv: bool,
    gob_headers: bool,
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
        PtypeFlags {
            umv,
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
    // §4.2.1 — macroblock rows per GOB for the standard source formats
    // (1 for sub-QCIF..CIF, 2 for 4CIF, 4 for 16CIF).
    let rows_per_gob = crate::picture::PictureLayout::for_source_format(fmt)
        .ok_or(Error::NotImplemented)?
        .mb_rows_per_gob as usize;
    let mut grid = if gob_headers {
        crate::encoder_motion::MvGrid::with_gob_headers(mb_cols, mb_rows, rows_per_gob)
    } else {
        crate::encoder_motion::MvGrid::new(mb_cols, mb_rows)
    };
    let gfid = tr & 0b11;
    // λ in SAD units per half-pel of MVD; a small bias keeps static
    // regions on MVD = 0 without over-penalising real motion.
    let lambda = 2 * quant as u32;

    for mb_row in 0..mb_rows {
        // §5.2 — a GOB header before the first macroblock row of every
        // GOB after GOB 0 (which is always header-less, §5.2.2).
        if gob_headers && mb_row > 0 && mb_row % rows_per_gob == 0 {
            write_gob_header(&mut w, (mb_row / rows_per_gob) as u32, gfid, quant);
        }
        for mb_col in 0..mb_cols {
            let predictor = grid.predict(mb_col, mb_row);
            let mv = if umv {
                crate::encoder_motion::estimate_motion_umv(
                    frame,
                    reference,
                    mb_col,
                    mb_row,
                    predictor,
                    search_half,
                    lambda,
                )
            } else {
                crate::encoder_motion::estimate_motion(
                    frame,
                    reference,
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
                let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                    *d = p as i16;
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
                    let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
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

            let mvd = if umv {
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
            crate::encoder_mb::encode_inter_macroblock(&mut w, &luma_arr, &cb_enc, &cr_enc, mvd)?;
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
}

impl Default for PbConfig {
    fn default() -> Self {
        PbConfig {
            quant: 8,
            trb: 1,
            dbquant: 0,
            search_half: 8,
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

            // ---- Skip / emit. ---------------------------------------
            if !any_p && !any_b && is_zero_mv {
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

            // §5.3.3 MODB (Table 11): "0" = no CBPB/MVDB; "11" = both.
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

            // §5.3.9 MVDB = (0, 0) when MODB carries it.
            if any_b {
                crate::encoder_vlc::write_mvd_component(&mut w, 0)?;
                crate::encoder_vlc::write_mvd_component(&mut w, 0)?;
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
}

impl Default for GopConfig {
    fn default() -> Self {
        GopConfig {
            quant: 8,
            intra_period: 12,
            search_half: 8,
            umv: false,
            eos: false,
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
            encode_intra_picture(frame, cfg.quant, tr)?
        } else {
            let reference = recon.as_ref().expect("recon present for P-picture");
            if cfg.umv {
                encode_inter_picture_umv(frame, reference, cfg.quant, tr, cfg.search_half)?
            } else {
                encode_inter_picture_motion(frame, reference, cfg.quant, tr, cfg.search_half)?
            }
        };
        // Closed loop: the next picture predicts from the *decoded*
        // reconstruction of this one, exactly like the decoder will.
        let decoded = decode_picture_no_gob0_header(
            &bytes,
            if force_intra { None } else { recon.as_ref() },
            DecodeOptions::default(),
        )?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::{decode_picture_no_gob0_header, DecodeOptions};

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
