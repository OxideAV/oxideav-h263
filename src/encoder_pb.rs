//! Annex M **Improved PB-frames** encoder (§M.1 – §M.4).
//!
//! An Improved PB-frame is one picture unit carrying a P-picture and a
//! BPB-picture (the "B-part", §M.1), signalled by the PLUSPTYPE
//! MPPTYPE picture type `"010"` (§5.1.4.3). Its P-part is coded exactly
//! like [`crate::encoder::encode_pb_picture`]'s; the difference is the
//! B-part, where every BPB-macroblock chooses one of the three §M.2
//! prediction modes through the Table M.1 MODB codeword:
//!
//! * **bidirectional** (§M.2.1) — the §G.4 / §G.5 scaled-vector
//!   composition with `MVD = 0` (no MVDB on the wire);
//! * **forward** (§M.2.2) — a single 16 × 16 forward vector into the
//!   previous reference picture, coded as MVDB against the §M.2.2
//!   left-neighbour predictor;
//! * **backward** (§M.2.3) — the prediction is PREC (§G.5), no vector
//!   data.
//!
//! The mode decision is rate-biased SAD (the luminance SAD of the
//! B-source against each candidate prediction plus λ × the MODB / MVDB
//! bits); the residual of the chosen prediction is coded at the
//! Table-6 BQUANT where it survives quantisation (CBPB). The output
//! decodes through
//! [`crate::picture::decode_improved_pb_picture_with_inherited`] and —
//! inside an elementary stream — [`crate::picture::decode_sequence`],
//! which splices the decoded pair in display order (BPB before P).

use crate::block::COEFFS_PER_BLOCK;
use crate::encoder::{
    extract_macroblock, motion_compensated_block, residual_of, source_format_for,
    write_plus_picture_header, PlusModes,
};
use crate::encoder_block::{encode_inter_block, write_inter_block_coeffs, EncodedInterBlock};
use crate::encoder_motion::{estimate_motion, mvd_for, MvGrid};
use crate::encoder_vlc::{write_cbpy, write_mcbpc_p, write_mvd_component};
use crate::macroblock::MbType;
use crate::motion::{chroma_mv, MotionVector, RefPlane, RCONTROL_DEFAULT};
use crate::pb_layer::{
    pb_b_predict_macroblock, pb_bquant, write_modb_annex_m, BpbCodingMode, ModbAnnexM,
    PbBMacroblockPrediction, PbBReferencePlanes,
};
use crate::picture::YuvFrame;
use crate::{Error, Result};
use oxideav_core::bits::BitWriter;

/// Configuration for [`encode_improved_pb_picture`].
#[derive(Debug, Clone, Copy)]
pub struct ImprovedPbConfig {
    /// Quantiser for the P-blocks (`1..=31`); the BPB-blocks run at the
    /// §5.1.23 / Table-6 BQUANT derived from it and `dbquant`.
    pub quant: u8,
    /// §5.1.22 TRB (3-bit form, `1..=7`).
    pub trb: u8,
    /// §5.1.23 DBQUANT (`0..=3`).
    pub dbquant: u8,
    /// Motion-search window for the P-part (±whole pixels).
    pub search_half: i32,
    /// §M.2.2 forward-vector search window for the BPB-part (±whole
    /// pixels around the left-neighbour predictor). `0` disables the
    /// forward mode altogether (only bidirectional / backward compete).
    pub forward_search_half: i32,
    /// Allow the §M.2.3 backward mode (prediction = PREC) to compete.
    pub allow_backward: bool,
}

impl Default for ImprovedPbConfig {
    fn default() -> Self {
        ImprovedPbConfig {
            quant: 8,
            trb: 1,
            dbquant: 0,
            search_half: 8,
            forward_search_half: 4,
            allow_backward: true,
        }
    }
}

/// Per-picture mode census returned by
/// [`encode_improved_pb_picture_stats`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ImprovedPbStats {
    /// Macroblocks emitted with COD = 1 (their BPB-part is the
    /// zero-vector bidirectional prediction, §M.2.1).
    pub skipped: usize,
    /// Coded macroblocks whose BPB-part took the §M.2.1 mode.
    pub bidirectional: usize,
    /// Coded macroblocks whose BPB-part took the §M.2.2 mode.
    pub forward: usize,
    /// Coded macroblocks whose BPB-part took the §M.2.3 mode.
    pub backward: usize,
    /// Macroblocks with at least one CBPB-lit BPB-block.
    pub b_residual: usize,
}

/// Encode an Annex M **Improved PB-frame**: one picture unit carrying a
/// P-picture (`p_source`, predicted from `reference`) and a BPB-picture
/// (`b_source`, temporally between `reference` and `p_source`).
///
/// See the [module documentation](self) for the per-macroblock mode
/// decision. `tr_p` is the §5.1.2 Temporal Reference of the P-part;
/// `prev_tr` is the reference picture's TR (their difference mod 256 is
/// the §G.4 TRD, which must be non-zero and greater than
/// [`ImprovedPbConfig::trb`]).
///
/// # Errors
///
/// * [`Error::InvalidQuantiser`] — `cfg.quant` outside `1..=31`.
/// * [`Error::BadPbTemporalReference`] — TRB / DBQUANT out of range or
///   `TRB >= TRD`.
/// * [`Error::NotImplemented`] — the three frames disagree in geometry
///   or the size is not a standard §5.1.3 source format.
pub fn encode_improved_pb_picture(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &ImprovedPbConfig,
) -> Result<Vec<u8>> {
    encode_improved_pb_picture_stats(p_source, b_source, reference, tr_p, prev_tr, cfg)
        .map(|(bytes, _)| bytes)
}

/// As [`encode_improved_pb_picture`], additionally returning the
/// per-picture [`ImprovedPbStats`] mode census.
pub fn encode_improved_pb_picture_stats(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &ImprovedPbConfig,
) -> Result<(Vec<u8>, ImprovedPbStats)> {
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
    let mut stats = ImprovedPbStats::default();

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr_p,
        /* is_inter */ true,
        PlusModes {
            improved_pb: Some((cfg.trb, cfg.dbquant)),
            ..PlusModes::default()
        },
    )?;

    let lw = p_source.luma_width;
    let lh = p_source.luma_height;
    let cw = p_source.chroma_width();
    let ch = p_source.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let mut grid = MvGrid::new(mb_cols, mb_rows);
    let lambda = 2 * quant as u32;

    let prev_y = RefPlane::new(&reference.y, lw, lh);
    let prev_cb = RefPlane::new(&reference.cb, cw, ch);
    let prev_cr = RefPlane::new(&reference.cr, cw, ch);

    for mb_row in 0..mb_rows {
        // §M.2.2 — the forward-vector predictor restarts at the far-left
        // edge of every macroblock row (the decoder resets it per row).
        let mut left_forward: Option<MotionVector> = None;
        for mb_col in 0..mb_cols {
            let mb_x = mb_col * 16;
            let mb_y = mb_row * 16;
            let c_x = mb_col * 8;
            let c_y = mb_row * 8;

            // ---- P-part: motion estimation + residual coding. -------
            let predictor = grid.predict(mb_col, mb_row);
            let mv = estimate_motion(
                p_source,
                reference,
                mb_col,
                mb_row,
                predictor,
                cfg.search_half,
                lambda,
            );
            let chroma_vec = chroma_mv(mv);
            let src = extract_macroblock(p_source, mb_col, mb_row);

            let mut luma_pred: Vec<[u8; COEFFS_PER_BLOCK]> = Vec::with_capacity(4);
            let mut luma_enc: Vec<EncodedInterBlock> = Vec::with_capacity(4);
            for blk in 0..4 {
                let bx = mb_x + (blk % 2) * 8;
                let by = mb_y + (blk / 2) * 8;
                let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                luma_enc.push(encode_inter_block(
                    &residual_of(&src.luma[blk], &to_i16(&pred)),
                    quant,
                ));
                luma_pred.push(pred);
            }
            let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_vec);
            let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_vec);
            let cb_enc = encode_inter_block(&residual_of(&src.cb, &to_i16(&cb_pred)), quant);
            let cr_enc = encode_inter_block(&residual_of(&src.cr, &to_i16(&cr_pred)), quant);

            let any_p =
                luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;
            let is_zero_mv = mv.dx_half == 0 && mv.dy_half == 0;

            // ---- PREC (§G.5): the decoder-reconstructed P-macroblock.
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

            // ---- BPB-part: §M.2 mode decision. ----------------------
            let planes = PbBReferencePlanes {
                prev_y,
                prev_cb,
                prev_cr,
                prec_y: RefPlane::new(&prec_y, 16, 16),
                prec_cb: RefPlane::new(&prec_cb, 8, 8),
                prec_cr: RefPlane::new(&prec_cr, 8, 8),
            };
            let b_src = extract_macroblock(b_source, mb_col, mb_row);
            let b_sad_of = |pred: &PbBMacroblockPrediction| -> u32 {
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

            // §M.2.1 — bidirectional, MVD = 0 (Table M.1 rows 0 / 1).
            let bidir = pb_b_predict_macroblock(
                &planes,
                mb_x,
                mb_y,
                &[mv; 4],
                None,
                trb,
                trd,
                RCONTROL_DEFAULT,
            );
            let mut best_mode = BpbCodingMode::Bidirectional;
            let mut best_cost = b_sad_of(&bidir) + lambda;
            let mut best_pred = bidir;
            let mut forward_choice: Option<(MotionVector, crate::macroblock::Mvd)> = None;

            // §M.2.2 — forward: one 16 × 16 vector into the previous
            // reference, coded against the left-neighbour predictor.
            if cfg.forward_search_half > 0 {
                let fwd_predictor = left_forward.unwrap_or_default();
                let fwd_mv = estimate_motion(
                    b_source,
                    reference,
                    mb_col,
                    mb_row,
                    fwd_predictor,
                    cfg.forward_search_half,
                    lambda,
                );
                let fwd_pred = forward_prediction(&planes, mb_x, mb_y, fwd_mv);
                let mvdb = mvd_for(fwd_mv, fwd_predictor);
                let bits = 3
                    + (mvdb.dx_half.unsigned_abs() as u32)
                    + (mvdb.dy_half.unsigned_abs() as u32)
                    + 2;
                let cost = b_sad_of(&fwd_pred) + lambda * bits;
                if cost < best_cost {
                    best_cost = cost;
                    best_mode = BpbCodingMode::Forward;
                    best_pred = fwd_pred;
                    forward_choice = Some((fwd_mv, mvdb));
                }
            }

            // §M.2.3 — backward: the prediction is PREC itself.
            if cfg.allow_backward {
                let bwd_pred = backward_prediction(&prec_y, &prec_cb, &prec_cr);
                let cost = b_sad_of(&bwd_pred) + lambda * 5;
                if cost < best_cost {
                    best_cost = cost;
                    best_mode = BpbCodingMode::Backward;
                    best_pred = bwd_pred;
                    forward_choice = None;
                }
            }
            let _ = best_cost;

            // Residual of the chosen prediction at BQUANT.
            let mut b_enc: Vec<EncodedInterBlock> = Vec::with_capacity(6);
            for blk in 0..4 {
                let ox = (blk % 2) * 8;
                let oy = (blk / 2) * 8;
                let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
                for j in 0..8 {
                    for i in 0..8 {
                        pred_i16[j * 8 + i] = best_pred.luma[oy + j][ox + i] as i16;
                    }
                }
                b_enc.push(encode_inter_block(
                    &residual_of(&b_src.luma[blk], &pred_i16),
                    bquant,
                ));
            }
            let mut b_cb_pred = [0i16; COEFFS_PER_BLOCK];
            let mut b_cr_pred = [0i16; COEFFS_PER_BLOCK];
            for j in 0..8 {
                for i in 0..8 {
                    b_cb_pred[j * 8 + i] = best_pred.cb[j][i] as i16;
                    b_cr_pred[j * 8 + i] = best_pred.cr[j][i] as i16;
                }
            }
            b_enc.push(encode_inter_block(
                &residual_of(&b_src.cb, &b_cb_pred),
                bquant,
            ));
            b_enc.push(encode_inter_block(
                &residual_of(&b_src.cr, &b_cr_pred),
                bquant,
            ));
            let any_b = b_enc.iter().any(|e| e.has_coeffs);

            // ---- Skip / emit. ---------------------------------------
            // A skipped macroblock (COD = 1) carries no MODB: its
            // BPB-part is the zero-vector bidirectional prediction and
            // the §M.2.2 predictor state is left untouched.
            if !any_p && !any_b && is_zero_mv && matches!(best_mode, BpbCodingMode::Bidirectional) {
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
                grid.set_zero_candidate(mb_col, mb_row);
                stats.skipped += 1;
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
            write_mcbpc_p(&mut w, MbType::Inter, cbpc)?;

            // §M.4 / Table M.1 MODB.
            write_modb_annex_m(&mut w, ModbAnnexM::from_parts(best_mode, any_b));
            if any_b {
                // §5.3.4 CBPB — block N lights bit (6 − N).
                let mut cbpb = 0u8;
                for (blk, e) in b_enc.iter().enumerate() {
                    if e.has_coeffs {
                        cbpb |= 1 << (6 - (blk + 1));
                    }
                }
                w.write_bits(cbpb as u32, 6);
                stats.b_residual += 1;
            }

            // §5.3.5 CBPY (INTER complement).
            let mut cbpy_intra = 0u8;
            for (blk, e) in luma_enc.iter().enumerate() {
                if e.has_coeffs {
                    cbpy_intra |= 1 << (3 - blk);
                }
            }
            write_cbpy(&mut w, cbpy_intra ^ 0b1111)?;

            // §5.3.7 MVD.
            let mvd = mvd_for(mv, predictor);
            write_mvd_component(&mut w, mvd.dx_half)?;
            write_mvd_component(&mut w, mvd.dy_half)?;

            // §5.3.9 MVDB — forward mode only (Table M.1 rows 2 / 3).
            match best_mode {
                BpbCodingMode::Forward => {
                    let (fwd_mv, mvdb) = forward_choice.expect("forward mode carries a vector");
                    write_mvd_component(&mut w, mvdb.dx_half)?;
                    write_mvd_component(&mut w, mvdb.dy_half)?;
                    left_forward = Some(fwd_mv);
                    stats.forward += 1;
                }
                BpbCodingMode::Backward => {
                    left_forward = None;
                    stats.backward += 1;
                }
                BpbCodingMode::Bidirectional => {
                    stats.bidirectional += 1;
                }
            }

            // §G.3 — six P-blocks, then six BPB-blocks.
            for e in luma_enc.iter() {
                if e.has_coeffs {
                    write_inter_block_coeffs(&mut w, &e.scan)?;
                }
            }
            if cb_enc.has_coeffs {
                write_inter_block_coeffs(&mut w, &cb_enc.scan)?;
            }
            if cr_enc.has_coeffs {
                write_inter_block_coeffs(&mut w, &cr_enc.scan)?;
            }
            for e in b_enc.iter() {
                if e.has_coeffs {
                    write_inter_block_coeffs(&mut w, &e.scan)?;
                }
            }

            grid.set_inter(mb_col, mb_row, mv);
        }
    }

    w.align_to_byte_zero();
    Ok((w.finish(), stats))
}

fn to_i16(pred: &[u8; COEFFS_PER_BLOCK]) -> [i16; COEFFS_PER_BLOCK] {
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for (d, &p) in out.iter_mut().zip(pred.iter()) {
        *d = p as i16;
    }
    out
}

/// The decoder-side reconstruction of one INTER block: prediction plus
/// the dequantised residual (§6.3.1) and the §6.3.2 clip, or the bare
/// prediction when no coefficient survived.
fn recon_block(
    enc: &EncodedInterBlock,
    pred: &[u8; COEFFS_PER_BLOCK],
    quant: u8,
) -> [u8; COEFFS_PER_BLOCK] {
    if enc.has_coeffs {
        let block = crate::block::H263Block {
            coefficients: enc.scan,
            tcoef_event_count: 0,
            had_intradc: false,
        };
        crate::reconstruct_inter_block_with_prediction(&block, quant, pred)
    } else {
        *pred
    }
}

/// §M.2.2 — the forward-only BPB prediction: the four luma blocks
/// fetched with one 16 × 16 vector from the previous reference and the
/// two chroma blocks with its Table-18 chroma vector.
fn forward_prediction(
    planes: &PbBReferencePlanes<'_>,
    mb_x: usize,
    mb_y: usize,
    forward_mv: MotionVector,
) -> PbBMacroblockPrediction {
    let mut luma = [[0u8; 16]; 16];
    for n in 0..4 {
        let nh = n & 1;
        let nv = n >> 1;
        let block = crate::motion::motion_compensate_block(
            &planes.prev_y,
            mb_x + nh * 8,
            mb_y + nv * 8,
            forward_mv,
            RCONTROL_DEFAULT,
        );
        for j in 0..8 {
            luma[nv * 8 + j][nh * 8..nh * 8 + 8].copy_from_slice(&block[j * 8..j * 8 + 8]);
        }
    }
    let chroma_vec = chroma_mv(forward_mv);
    let (cx, cy) = (mb_x / 2, mb_y / 2);
    let cb_flat = crate::motion::motion_compensate_block(
        &planes.prev_cb,
        cx,
        cy,
        chroma_vec,
        RCONTROL_DEFAULT,
    );
    let cr_flat = crate::motion::motion_compensate_block(
        &planes.prev_cr,
        cx,
        cy,
        chroma_vec,
        RCONTROL_DEFAULT,
    );
    let mut cb = [[0u8; 8]; 8];
    let mut cr = [[0u8; 8]; 8];
    for j in 0..8 {
        cb[j].copy_from_slice(&cb_flat[j * 8..j * 8 + 8]);
        cr[j].copy_from_slice(&cr_flat[j * 8..j * 8 + 8]);
    }
    PbBMacroblockPrediction { luma, cb, cr }
}

/// §M.2.3 — the backward BPB prediction is PREC.
fn backward_prediction(
    prec_y: &[u8; 256],
    prec_cb: &[u8; COEFFS_PER_BLOCK],
    prec_cr: &[u8; COEFFS_PER_BLOCK],
) -> PbBMacroblockPrediction {
    let mut luma = [[0u8; 16]; 16];
    for (j, row) in luma.iter_mut().enumerate() {
        row.copy_from_slice(&prec_y[j * 16..j * 16 + 16]);
    }
    let mut cb = [[0u8; 8]; 8];
    let mut cr = [[0u8; 8]; 8];
    for j in 0..8 {
        cb[j].copy_from_slice(&prec_cb[j * 8..j * 8 + 8]);
        cr[j].copy_from_slice(&prec_cr[j * 8..j * 8 + 8]);
    }
    PbBMacroblockPrediction { luma, cb, cr }
}
