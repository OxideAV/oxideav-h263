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

use crate::block::COEFFS_PER_BLOCK;
use crate::encoder_mb::{encode_intra_macroblock, MacroblockSamples};
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

/// Write the §5.1 baseline picture header (PSC, TR, PTYPE all-baseline,
/// PQUANT, CPM=0, PEI=0). `is_inter` selects the §5.1.3 picture
/// coding-type bit (INTRA = 0, INTER = 1); all optional-mode flags are
/// emitted as 0.
fn write_picture_header(
    w: &mut BitWriter,
    fmt: H263SourceFormat,
    quant: u8,
    tr: u8,
    is_inter: bool,
) {
    // §5.1.1 — Picture Start Code (22 bits, 0x000020).
    w.write_bits(PSC_VALUE, 22);
    // §5.1.2 — Temporal Reference (8 bits).
    w.write_bits(tr as u32, 8);
    // §5.1.3 — PTYPE. bit1 = 1, bit2 = 0, then split/doc/freeze = 0,
    // source-format (3), coding-type, then UMV/SAC/AP/PB = 0.
    w.write_bit(true); // bit 1
    w.write_bit(false); // bit 2
    w.write_bit(false); // split-screen
    w.write_bit(false); // document-camera
    w.write_bit(false); // freeze-release
    w.write_bits(source_format_bits(fmt), 3);
    w.write_bit(is_inter); // coding-type: 0 INTRA / 1 INTER
    w.write_bit(false); // UMV (Annex D)
    w.write_bit(false); // SAC (Annex E)
    w.write_bit(false); // AP  (Annex F)
    w.write_bit(false); // PB  (Annex G)
                        // §5.1.19 — PQUANT (5 bits).
    w.write_bits(quant as u32, 5);
    // §5.1.20 — CPM (1 bit, 0 = single bitstream).
    w.write_bit(false);
    // §5.1.24 — PEI (1 bit, 0 = no PSUPP extension).
    w.write_bit(false);
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
    write_picture_header(&mut w, fmt, quant, tr, /* is_inter */ false);

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
    write_picture_header(&mut w, fmt, quant, tr, /* is_inter */ true);

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
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;

    let mut w = BitWriter::new();
    write_picture_header(&mut w, fmt, quant, tr, /* is_inter */ true);

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let mut grid = crate::encoder_motion::MvGrid::new(mb_cols, mb_rows);
    // λ in SAD units per half-pel of MVD; a small bias keeps static
    // regions on MVD = 0 without over-penalising real motion.
    let lambda = 2 * quant as u32;

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

            let mvd = crate::encoder_motion::mvd_for(mv, predictor);
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
