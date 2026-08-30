//! Within-picture **adaptive quantisation** — the §5.3.6 per-macroblock
//! DQUANT primitives driven by a bit-budget governor.
//!
//! The frame-level controller ([`crate::rate_control::RateController`])
//! picks one QUANT per picture; these encoders regulate *inside* the
//! picture as well: while walking the macroblocks in scanning order the
//! governor compares the bits already written against the pro-rata
//! share of the picture budget and steps QUANT through the `+Q`
//! macroblock types (INTRA+Q / INTER+Q, Table 8/9 with the 2-bit
//! Table 12 DQUANT field) — at most ±2 per macroblock, exactly the
//! §5.3.6 differential range, staying in the legal `1..=31`. A skipped
//! macroblock (COD = 1) carries no DQUANT, so the governor holds its
//! QUANT across skips; a macroblock whose QUANT is unchanged uses the
//! plain INTER / INTRA types with no DQUANT overhead.
//!
//! Both encoders emit plain **baseline** pictures — DQUANT is §5.3
//! machinery, no optional mode is signalled — decodable by
//! [`crate::picture::decode_picture_no_gob0_header`] /
//! [`crate::picture::decode_sequence`], and both report the
//! per-macroblock QUANT trace so callers (and tests) can see the
//! regulation. [`crate::encoder::encode_sequence_rate_controlled`]
//! drives them from its per-picture budget when
//! [`crate::encoder::RateControlConfig::mb_adaptive`] is set.

use crate::encoder::{
    extract_macroblock, motion_compensated_block, residual_of, source_format_for,
    write_picture_header, PtypeFlags,
};
use crate::encoder_block::{encode_inter_block, EncodedInterBlock};
use crate::encoder_mb::{
    encode_inter_macroblock_dq, encode_intra_macroblock_dq, encode_skipped_macroblock,
};
use crate::encoder_motion::{estimate_motion, mvd_for, MvGrid};
use crate::motion::chroma_mv;
use crate::picture::YuvFrame;
use crate::{Error, Result};
use oxideav_core::bits::BitWriter;

/// Configuration for the adaptive-quantisation picture encoders.
#[derive(Debug, Clone, Copy)]
pub struct AdaptiveQuantConfig {
    /// Bit budget for the whole coded picture (header included).
    pub target_bits: u32,
    /// QUANT of the first macroblock (also the header PQUANT), `1..=31`.
    pub initial_quant: u8,
    /// Motion-search window for the INTER encoder (± whole pixels).
    pub search_half: i32,
}

impl Default for AdaptiveQuantConfig {
    fn default() -> Self {
        AdaptiveQuantConfig {
            target_bits: 24_000,
            initial_quant: 10,
            search_half: 8,
        }
    }
}

/// An adaptively-quantised picture: the coded bytes plus the QUANT each
/// macroblock was quantised at (the governor's trace, in scanning
/// order — a skipped macroblock reports the QUANT it held).
#[derive(Debug, Clone)]
pub struct AdaptiveQuantPicture {
    /// The coded picture (byte-aligned, PSTUF-padded).
    pub bytes: Vec<u8>,
    /// Per-macroblock QUANT in scanning order.
    pub mb_quants: Vec<u8>,
}

/// The governor: given the bits spent so far, the pro-rata budget at
/// this macroblock and the current QUANT, pick the next macroblock's
/// QUANT within the §5.3.6 ±2 differential and the legal range.
///
/// The reaction threshold is one macroblock's budget share: drifting
/// less than one share leaves QUANT alone (no DQUANT overhead for
/// noise), one-to-three shares steps by 1, more steps by 2.
fn govern(spent: u64, ideal: u64, mb_share: u64, quant: u8) -> u8 {
    let share = mb_share.max(1) as i64;
    let err = spent as i64 - ideal as i64;
    let step: i8 = if err > 3 * share {
        2
    } else if err > share {
        1
    } else if err < -3 * share {
        -2
    } else if err < -share {
        -1
    } else {
        0
    };
    (quant as i8 + step).clamp(1, 31) as u8
}

/// Encode a baseline **INTRA** picture with per-macroblock adaptive
/// quantisation: the governor steps QUANT through INTRA+Q macroblocks
/// (§5.3.6 DQUANT) to hold the picture near
/// [`AdaptiveQuantConfig::target_bits`]. Decodes through
/// [`crate::picture::decode_picture_no_gob0_header`] / `decode_sequence`.
pub fn encode_intra_picture_adaptive(
    frame: &YuvFrame,
    cfg: &AdaptiveQuantConfig,
    tr: u8,
) -> Result<AdaptiveQuantPicture> {
    if cfg.initial_quant == 0 || cfg.initial_quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let fmt =
        source_format_for(frame.luma_width, frame.luma_height).ok_or(Error::NotImplemented)?;
    let mut w = BitWriter::new();
    write_picture_header(
        &mut w,
        fmt,
        cfg.initial_quant,
        tr,
        /* is_inter */ false,
        PtypeFlags::default(),
        None,
    );

    let mb_cols = frame.luma_width / 16;
    let mb_rows = frame.luma_height / 16;
    let total = (mb_cols * mb_rows) as u64;
    let mb_share = cfg.target_bits as u64 / total.max(1);
    let mut quant = cfg.initial_quant;
    let mut mb_quants = Vec::with_capacity(mb_cols * mb_rows);

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let idx = (mb_row * mb_cols + mb_col) as u64;
            let next = if idx == 0 {
                quant
            } else {
                govern(
                    w.bit_position(),
                    cfg.target_bits as u64 * idx / total,
                    mb_share,
                    quant,
                )
            };
            let dquant = (next != quant).then_some((next as i8) - (quant as i8));
            quant = next;
            let mb = extract_macroblock(frame, mb_col, mb_row);
            encode_intra_macroblock_dq(
                &mut w, &mb, quant, /* write_cod */ false, /* picture_is_inter */ false,
                dquant,
            )?;
            mb_quants.push(quant);
        }
    }
    w.align_to_byte_zero();
    Ok(AdaptiveQuantPicture {
        bytes: w.finish(),
        mb_quants,
    })
}

/// Encode a baseline motion-estimated **INTER** picture with
/// per-macroblock adaptive quantisation: identical macroblock walk to
/// [`crate::encoder::encode_inter_picture_motion`] (SAD + half-pel
/// search with §6.1.1 predictor replay, COD = 1 skips, §5.3.2 INTRA
/// refresh), but the governor steps QUANT through the `+Q` macroblock
/// types to hold the picture near [`AdaptiveQuantConfig::target_bits`].
/// Skipped macroblocks carry no DQUANT, so QUANT holds across them.
/// Decodes through [`crate::picture::decode_picture_no_gob0_header`] /
/// `decode_sequence`.
pub fn encode_inter_picture_adaptive(
    frame: &YuvFrame,
    reference: &YuvFrame,
    cfg: &AdaptiveQuantConfig,
    tr: u8,
) -> Result<AdaptiveQuantPicture> {
    if cfg.initial_quant == 0 || cfg.initial_quant > 31 {
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
        cfg.initial_quant,
        tr,
        /* is_inter */ true,
        PtypeFlags::default(),
        None,
    );

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let total = (mb_cols * mb_rows) as u64;
    let mb_share = cfg.target_bits as u64 / total.max(1);
    let mut grid = MvGrid::new(mb_cols, mb_rows);
    let mut quant = cfg.initial_quant;
    let mut mb_quants = Vec::with_capacity(mb_cols * mb_rows);

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let idx = (mb_row * mb_cols + mb_col) as u64;
            let next = if idx == 0 {
                quant
            } else {
                govern(
                    w.bit_position(),
                    cfg.target_bits as u64 * idx / total,
                    mb_share,
                    quant,
                )
            };

            let predictor = grid.predict(mb_col, mb_row);
            let lambda = 2 * next as u32;
            let mv = estimate_motion(
                frame,
                reference,
                mb_col,
                mb_row,
                predictor,
                cfg.search_half,
                lambda,
            );

            let mb_x = mb_col * 16;
            let mb_y = mb_row * 16;
            let src = extract_macroblock(frame, mb_col, mb_row);
            let mut luma_enc: Vec<EncodedInterBlock> = Vec::with_capacity(4);
            let mut inter_sad = 0u32;
            for blk in 0..4 {
                let bx = mb_x + (blk % 2) * 8;
                let by = mb_y + (blk / 2) * 8;
                let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                let mut pred_i16 = [0i16; 64];
                for (d, &p) in pred_i16.iter_mut().zip(pred.iter()) {
                    *d = p as i16;
                }
                for row in 0..8 {
                    for col in 0..8 {
                        let sv = frame.y[(by + row) * lw + (bx + col)] as i32;
                        inter_sad += (sv - pred[row * 8 + col] as i32).unsigned_abs();
                    }
                }
                let residual = residual_of(&src.luma[blk], &pred_i16);
                luma_enc.push(encode_inter_block(&residual, next));
            }
            let chroma_vec = chroma_mv(mv);
            let c_x = mb_col * 8;
            let c_y = mb_row * 8;
            let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_vec);
            let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_vec);
            let mut cb_pred_i = [0i16; 64];
            let mut cr_pred_i = [0i16; 64];
            for i in 0..64 {
                cb_pred_i[i] = cb_pred[i] as i16;
                cr_pred_i[i] = cr_pred[i] as i16;
            }
            let cb_enc = encode_inter_block(&residual_of(&src.cb, &cb_pred_i), next);
            let cr_enc = encode_inter_block(&residual_of(&src.cr, &cr_pred_i), next);
            let any_coeffs =
                luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;

            if !any_coeffs && mv.dx_half == 0 && mv.dy_half == 0 {
                // COD = 1 — no DQUANT is carried, QUANT holds.
                encode_skipped_macroblock(&mut w);
                grid.set_zero_candidate(mb_col, mb_row);
                mb_quants.push(quant);
                continue;
            }

            // The macroblock is coded: commit the governor's step and
            // its DQUANT differential (±2 max by construction).
            let dquant = (next != quant).then_some((next as i8) - (quant as i8));
            quant = next;

            // §5.3.2 INTRA refresh (same heuristic as the fixed-QUANT
            // motion encoder).
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
                encode_intra_macroblock_dq(
                    &mut w, &src, quant, /* write_cod */ true,
                    /* picture_is_inter */ true, dquant,
                )?;
                grid.set_zero_candidate(mb_col, mb_row);
                mb_quants.push(quant);
                continue;
            }

            let mvd = mvd_for(mv, predictor);
            let luma_arr: [EncodedInterBlock; 4] = [
                luma_enc[0].clone(),
                luma_enc[1].clone(),
                luma_enc[2].clone(),
                luma_enc[3].clone(),
            ];
            encode_inter_macroblock_dq(&mut w, &luma_arr, &cb_enc, &cr_enc, mvd, dquant)?;
            grid.set_inter(mb_col, mb_row, mv);
            mb_quants.push(quant);
        }
    }
    w.align_to_byte_zero();
    Ok(AdaptiveQuantPicture {
        bytes: w.finish(),
        mb_quants,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::{decode_picture_no_gob0_header, DecodeOptions};

    fn textured(lw: usize, lh: usize, seed: usize) -> YuvFrame {
        let mut y = vec![0u8; lw * lh];
        for r in 0..lh {
            for c in 0..lw {
                // Busy top half, flat bottom half: the governor must
                // coarsen while the busy rows overspend, then relax.
                let v = if r < lh / 2 {
                    (r * 7 + c * 5 + seed * 13 + ((r / 4) + (c / 4)) * 29) % 256
                } else {
                    120
                };
                y[r * lw + c] = v as u8;
            }
        }
        YuvFrame {
            y,
            cb: vec![100; lw * lh / 4],
            cr: vec![150; lw * lh / 4],
            luma_width: lw,
            luma_height: lh,
        }
    }

    #[test]
    fn adaptive_intra_regulates_and_round_trips() {
        let f = textured(176, 144, 1);
        let tight = AdaptiveQuantConfig {
            target_bits: 20_000,
            initial_quant: 6,
            search_half: 0,
        };
        let pic = encode_intra_picture_adaptive(&f, &tight, 0).unwrap();
        // Legal trace: every step within ±2, all quants 1..=31.
        for pair in pic.mb_quants.windows(2) {
            assert!((pair[0] as i16 - pair[1] as i16).abs() <= 2);
        }
        assert!(pic.mb_quants.iter().all(|&q| (1..=31).contains(&q)));
        // The busy half overspends at QUANT 6 → the governor must have
        // coarsened somewhere.
        assert!(
            pic.mb_quants.iter().any(|&q| q > 6),
            "governor never stepped"
        );
        // Regulation: within +25 % of the budget (a fixed QUANT 6
        // picture of this content is far larger).
        assert!(
            (pic.bytes.len() as u32 * 8) < tight.target_bits + tight.target_bits / 4,
            "{} bits vs target {}",
            pic.bytes.len() * 8,
            tight.target_bits
        );
        let dec = decode_picture_no_gob0_header(&pic.bytes, None, DecodeOptions::default());
        assert!(dec.is_ok());
    }

    #[test]
    fn adaptive_inter_regulates_skips_hold_quant_and_round_trips() {
        let base = textured(176, 144, 2);
        let i = crate::encoder::encode_intra_picture(&base, 8, 0).unwrap();
        let recon = decode_picture_no_gob0_header(&i, None, DecodeOptions::default()).unwrap();
        // New content in the busy half only — the flat half skips.
        let next = textured(176, 144, 5);
        let cfg = AdaptiveQuantConfig {
            target_bits: 8_000,
            initial_quant: 8,
            search_half: 4,
        };
        let pic = encode_inter_picture_adaptive(&next, &recon, &cfg, 1).unwrap();
        for pair in pic.mb_quants.windows(2) {
            assert!((pair[0] as i16 - pair[1] as i16).abs() <= 2);
        }
        assert!(
            (pic.bytes.len() as u32 * 8) < cfg.target_bits + cfg.target_bits / 4,
            "{} bits vs target {}",
            pic.bytes.len() * 8,
            cfg.target_bits
        );
        let dec = decode_picture_no_gob0_header(&pic.bytes, Some(&recon), DecodeOptions::default());
        assert!(dec.is_ok());
    }

    #[test]
    fn static_content_stays_on_one_quant() {
        let base = textured(128, 96, 3);
        let i = crate::encoder::encode_intra_picture(&base, 10, 0).unwrap();
        let recon = decode_picture_no_gob0_header(&i, None, DecodeOptions::default()).unwrap();
        let cfg = AdaptiveQuantConfig {
            target_bits: 4_000,
            initial_quant: 10,
            search_half: 4,
        };
        // Re-encoding the reconstruction against itself: all skips, no
        // DQUANT anywhere, the trace is flat.
        let pic = encode_inter_picture_adaptive(&recon, &recon, &cfg, 1).unwrap();
        assert!(pic.mb_quants.iter().all(|&q| q == 10));
        assert!(pic.bytes.len() <= 16);
    }
}
