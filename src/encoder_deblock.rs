//! Annex J **Deblocking Filter mode** — the encoder arm.
//!
//! An H.263+ picture with OPPTYPE bit 9 set runs the §J.3 block-edge
//! filter inside the coding loop: the decoder filters the reconstructed
//! picture (prediction + reconstructed prediction error, clipped per
//! §6.3.2) *before* it becomes the next picture's reference, and §J.3
//! requires the encoder to do the same so both sides predict from the
//! identical filtered picture. This crate's encoder obtains its
//! reference by decoding its own output with the crate decoder (see
//! [`crate::encoder::encode_sequence`]), so the filtered reference is
//! produced by the exact decoder primitive; what the write side adds is
//! the wire signalling and the mode's motion rules.
//!
//! Per Table J.1 the Deblocking Filter mode on its own switches on
//! three of the five Annex D / F / J elements: motion vectors over
//! picture boundaries (§D.1), **four motion vectors per macroblock**
//! (§F.2 — the INTER4V macroblock types become legal) and the §J.3
//! edge filter; the §F.3 overlapped block motion compensation stays
//! **off** unless Advanced Prediction is also signalled. The vector
//! predictors therefore follow §F.2 / Figure F.1 for every macroblock
//! (a one-vector macroblock counts as four equal vectors), which the
//! encoder replays with [`crate::encoder_motion::Mv4Grid`].
//!
//! Three entry points:
//!
//! * [`encode_intra_picture_deblock`] — an I-picture whose OPPTYPE
//!   signals the mode (the filter also applies to I-pictures, §J.1);
//! * [`encode_inter_picture_deblock`] — a P-picture with a per-macroblock
//!   one-vector / four-vector decision ([`DeblockConfig::four_mv`]),
//!   optional Annex D extended range ([`DeblockConfig::umv`], Table D.3
//!   difference coding as PLUSPTYPE is present), the §5.3.2 INTRA
//!   refresh decision and COD = 1 skipping;
//! * the [`crate::encoder::GopConfig::deblock`] switch of the
//!   closed-loop sequence encoders, which routes every picture through
//!   these two and predicts from the filtered reconstruction.

use crate::block::COEFFS_PER_BLOCK;
use crate::encoder::{
    extract_macroblock, motion_compensated_block, residual_of, source_format_for,
    write_plus_picture_header, PlusModes,
};
use crate::encoder_block::{encode_inter_block, EncodedInterBlock};
use crate::encoder_mb::{
    encode_inter4v_macroblock, encode_inter4v_macroblock_umv_plus, encode_inter_macroblock,
    encode_inter_macroblock_umv_plus, encode_intra_macroblock, encode_skipped_macroblock,
};
use crate::encoder_motion::{
    estimate_block_motion, estimate_block_motion_umv_plus, estimate_motion,
    estimate_motion_umv_plus, mvd_for, Mv4Grid,
};
use crate::macroblock::Mvd;
use crate::motion::{chroma_mv, chroma_mv_4mv, LumaBlockIndex, Mb4Mv, MotionVector};
use crate::picture::YuvFrame;
use crate::{Error, Result};
use oxideav_core::bits::BitWriter;

/// Configuration for [`encode_inter_picture_deblock`].
#[derive(Debug, Clone, Copy)]
pub struct DeblockConfig {
    /// Motion-search window (± whole pixels around the §F.2 predictor).
    pub search_half: i32,
    /// Allow the four-vector (INTER4V) macroblock type the mode makes
    /// legal (Table J.1): each macroblock picks one or four vectors by
    /// a SAD + vector-cost comparison. `false` codes one vector per
    /// macroblock throughout (still under the §F.2 predictor rules).
    pub four_mv: bool,
    /// Also signal Annex D Unrestricted Motion Vector mode (OPPTYPE
    /// bit 5 + UUI `"1"`): vectors are searched over the Tables
    /// D.1/D.2 range and every difference is written as a Table D.3
    /// codeword (§5.3.7 — PLUSPTYPE is present).
    pub umv: bool,
}

impl Default for DeblockConfig {
    fn default() -> Self {
        DeblockConfig {
            search_half: 8,
            four_mv: true,
            umv: false,
        }
    }
}

/// Encode an **INTRA** picture in Annex J Deblocking Filter mode: the
/// H.263+ header signals OPPTYPE bit 9, and the macroblock stream is
/// the plain §5.3 INTRA stream (bit-identical to
/// [`crate::encoder::encode_intra_picture_plus`]'s). Decoders apply
/// the §J.3 filter to the reconstruction, so the picture a
/// deblocking-mode P-picture must predict from is the decoded (filtered)
/// output of [`crate::picture::decode_picture_layer`], not the
/// unfiltered macroblock reconstruction.
pub fn encode_intra_picture_deblock(frame: &YuvFrame, quant: u8, tr: u8) -> Result<Vec<u8>> {
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
        PlusModes {
            deblocking: true,
            ..PlusModes::default()
        },
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

/// SAD of the 8×8 luma block at `(bx, by)` against its prediction.
fn block_sad(frame: &YuvFrame, bx: usize, by: usize, pred: &[u8; COEFFS_PER_BLOCK]) -> u32 {
    let lw = frame.luma_width;
    let mut s = 0u32;
    for row in 0..8 {
        for col in 0..8 {
            let sv = frame.y[(by + row) * lw + (bx + col)] as i32;
            s += (sv - pred[row * 8 + col] as i32).unsigned_abs();
        }
    }
    s
}

fn u8_to_i16(p: &[u8; COEFFS_PER_BLOCK]) -> [i16; COEFFS_PER_BLOCK] {
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for (d, &s) in out.iter_mut().zip(p.iter()) {
        *d = s as i16;
    }
    out
}

/// Half-pel vector-difference cost proxy (`|dx| + |dy|` of the MVD).
fn mvd_cost(mv: MotionVector, predictor: MotionVector) -> u32 {
    (mv.dx_half - predictor.dx_half).unsigned_abs()
        + (mv.dy_half - predictor.dy_half).unsigned_abs()
}

/// Encode a **P-picture** in Annex J Deblocking Filter mode (see the
/// module documentation for the mode's Table J.1 element set).
///
/// `reference` must be the **filtered** reconstruction of the previous
/// picture — what [`crate::picture::decode_picture_layer`] returns for
/// a deblocking-mode picture. Each macroblock:
///
/// 1. searches one 16×16 vector biased toward its §F.2 block-1
///    predictor; with [`DeblockConfig::four_mv`] it also searches the
///    four 8×8 block vectors (each toward its own §F.2 predictor,
///    threading the already-chosen earlier blocks as Figure F.1
///    candidates) and keeps whichever set has the lower
///    `SAD + λ · |MVD|` cost, the four-vector set carrying a fixed
///    MCBPC-length penalty;
/// 2. applies the §5.3.2 INTRA-refresh decision (INTRA when the motion
///    residual exceeds the block's own AC energy);
/// 3. codes the residual against the plain (non-overlapped) §6.1
///    block prediction — OBMC is off in this mode — with the Table 18
///    chroma vector for one-vector macroblocks and the Table F.1
///    vector for four-vector ones, exactly as the decoder derives them;
/// 4. skips (COD = 1) a zero-vector macroblock with no surviving
///    residual.
///
/// The output is self-describing: it decodes through
/// [`crate::picture::decode_picture_layer`] /
/// [`crate::picture::decode_sequence`] with `DecodeOptions::default()`,
/// which activates the §J.3 filter from OPPTYPE.
pub fn encode_inter_picture_deblock(
    frame: &YuvFrame,
    reference: &YuvFrame,
    quant: u8,
    tr: u8,
    cfg: &DeblockConfig,
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
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PlusModes {
            deblocking: true,
            umv: cfg.umv,
            ..PlusModes::default()
        },
    )?;

    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = frame.chroma_width();
    let ch = frame.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let lambda = 2 * quant as u32;
    // The INTER4V MCBPC codewords are longer than their INTER
    // counterparts and three extra MVD pairs follow; a fixed λ-scaled
    // penalty keeps four vectors for macroblocks that earn them.
    let four_mv_penalty = 6 * lambda;

    let mut grid = Mv4Grid::new(mb_cols, mb_rows);
    let zero4: Mb4Mv = [MotionVector::new(0, 0); 4];

    for mb_row in 0..mb_rows {
        for mb_col in 0..mb_cols {
            let mb_x = mb_col * 16;
            let mb_y = mb_row * 16;

            // ── one 16×16 vector toward the §F.2 block-1 predictor ──
            let pred1 = grid.predict_block(mb_col, mb_row, LumaBlockIndex::B1, &zero4);
            let mv1 = if cfg.umv {
                estimate_motion_umv_plus(
                    frame,
                    reference,
                    mb_col,
                    mb_row,
                    pred1,
                    cfg.search_half,
                    lambda,
                )
            } else {
                estimate_motion(
                    frame,
                    reference,
                    mb_col,
                    mb_row,
                    pred1,
                    cfg.search_half,
                    lambda,
                )
            };
            let mut preds1: Vec<[u8; COEFFS_PER_BLOCK]> = Vec::with_capacity(4);
            let mut cost1 = lambda * mvd_cost(mv1, pred1);
            for blk in 0..4 {
                let bx = mb_x + (blk % 2) * 8;
                let by = mb_y + (blk / 2) * 8;
                let p = motion_compensated_block(&reference.y, lw, lh, bx, by, mv1);
                cost1 += block_sad(frame, bx, by, &p);
                preds1.push(p);
            }

            // ── optionally four 8×8 vectors, each toward its own predictor ──
            let mut choose_four = false;
            let mut cur4: Mb4Mv = [mv1; 4];
            let mut mvds4 = [Mvd {
                dx_half: 0,
                dy_half: 0,
            }; 4];
            let mut preds4: Vec<[u8; COEFFS_PER_BLOCK]> = Vec::with_capacity(4);
            if cfg.four_mv {
                let mut cost4 = four_mv_penalty;
                let mut cur: Mb4Mv = zero4;
                for &blk in &LumaBlockIndex::ALL {
                    let blk_i = blk.index();
                    let bx = mb_x + (blk_i % 2) * 8;
                    let by = mb_y + (blk_i / 2) * 8;
                    let predictor = grid.predict_block(mb_col, mb_row, blk, &cur);
                    let mv = if cfg.umv {
                        estimate_block_motion_umv_plus(
                            frame,
                            reference,
                            bx,
                            by,
                            predictor,
                            cfg.search_half,
                            lambda,
                        )
                    } else {
                        estimate_block_motion(
                            frame,
                            reference,
                            bx,
                            by,
                            predictor,
                            cfg.search_half,
                            lambda,
                        )
                    };
                    cur[blk_i] = mv;
                    mvds4[blk_i] = if cfg.umv {
                        Mvd {
                            dx_half: (mv.dx_half - predictor.dx_half) as i16,
                            dy_half: (mv.dy_half - predictor.dy_half) as i16,
                        }
                    } else {
                        mvd_for(mv, predictor)
                    };
                    let p = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                    cost4 += block_sad(frame, bx, by, &p) + lambda * mvd_cost(mv, predictor);
                    preds4.push(p);
                }
                if cost4 < cost1 {
                    choose_four = true;
                    cur4 = cur;
                }
            }

            let src = extract_macroblock(frame, mb_col, mb_row);
            let luma_preds = if choose_four { &preds4 } else { &preds1 };
            let luma_enc: Vec<EncodedInterBlock> = (0..4)
                .map(|blk| {
                    let residual = residual_of(&src.luma[blk], &u8_to_i16(&luma_preds[blk]));
                    encode_inter_block(&residual, quant)
                })
                .collect();
            let chroma_vec = if choose_four {
                chroma_mv_4mv(&cur4)
            } else {
                chroma_mv(mv1)
            };
            let c_x = mb_col * 8;
            let c_y = mb_row * 8;
            let cb_pred = motion_compensated_block(&reference.cb, cw, ch, c_x, c_y, chroma_vec);
            let cr_pred = motion_compensated_block(&reference.cr, cw, ch, c_x, c_y, chroma_vec);
            let cb_enc = encode_inter_block(&residual_of(&src.cb, &u8_to_i16(&cb_pred)), quant);
            let cr_enc = encode_inter_block(&residual_of(&src.cr, &u8_to_i16(&cr_pred)), quant);
            let any_coeffs =
                luma_enc.iter().any(|e| e.has_coeffs) || cb_enc.has_coeffs || cr_enc.has_coeffs;

            if !choose_four && !any_coeffs && mv1.dx_half == 0 && mv1.dy_half == 0 {
                // COD = 1 — the decoder copies the co-located reference
                // macroblock (zero vector), which is our prediction.
                encode_skipped_macroblock(&mut w);
                grid.set(mb_col, mb_row, zero4);
                continue;
            }

            // §5.3.2 INTRA refresh: when the motion-compensated residual
            // energy exceeds the macroblock's own AC energy, an INTRA
            // macroblock is cheaper and reconstructs more faithfully.
            let inter_sad: u32 = (0..4)
                .map(|blk| {
                    let bx = mb_x + (blk % 2) * 8;
                    let by = mb_y + (blk / 2) * 8;
                    block_sad(frame, bx, by, &luma_preds[blk])
                })
                .sum();
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
                encode_intra_macroblock(
                    &mut w, &src, quant, /* write_cod */ true,
                    /* picture_is_inter */ true,
                )?;
                // §6.1.1 rule 1 — an INTRA macroblock is a zero candidate.
                grid.set(mb_col, mb_row, zero4);
                continue;
            }

            let luma_arr: [EncodedInterBlock; 4] = [
                luma_enc[0].clone(),
                luma_enc[1].clone(),
                luma_enc[2].clone(),
                luma_enc[3].clone(),
            ];
            if choose_four {
                if cfg.umv {
                    encode_inter4v_macroblock_umv_plus(
                        &mut w, &luma_arr, &cb_enc, &cr_enc, &mvds4,
                    )?;
                } else {
                    encode_inter4v_macroblock(&mut w, &luma_arr, &cb_enc, &cr_enc, &mvds4)?;
                }
                grid.set(mb_col, mb_row, cur4);
            } else {
                let mvd = if cfg.umv {
                    Mvd {
                        dx_half: (mv1.dx_half - pred1.dx_half) as i16,
                        dy_half: (mv1.dy_half - pred1.dy_half) as i16,
                    }
                } else {
                    mvd_for(mv1, pred1)
                };
                if cfg.umv {
                    encode_inter_macroblock_umv_plus(&mut w, &luma_arr, &cb_enc, &cr_enc, mvd)?;
                } else {
                    encode_inter_macroblock(&mut w, &luma_arr, &cb_enc, &cr_enc, mvd)?;
                }
                grid.set(mb_col, mb_row, [mv1; 4]);
            }
        }
    }

    w.align_to_byte_zero();
    Ok(w.finish())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::{decode_picture_layer, DecodeOptions};
    use crate::picture_header::{parse_picture_layer, H263PictureLayer};
    use crate::plus_ptype::InheritedExtendedState;
    use oxideav_core::bits::BitReader;

    fn textured(lw: usize, lh: usize, seed: usize) -> YuvFrame {
        let mut y = vec![0u8; lw * lh];
        for r in 0..lh {
            for c in 0..lw {
                let checker = if ((r / 8) + (c / 8)) % 2 == 0 { 30 } else { 0 };
                y[r * lw + c] = ((r * 5 + c * 3 + seed * 11 + checker) % 256) as u8;
            }
        }
        YuvFrame {
            y,
            cb: (0..lw * lh / 4).map(|i| (90 + i % 50) as u8).collect(),
            cr: (0..lw * lh / 4).map(|i| (160 - (i % 40)) as u8).collect(),
            luma_width: lw,
            luma_height: lh,
        }
    }

    fn shifted(f: &YuvFrame, dx: usize, dy: usize) -> YuvFrame {
        let lw = f.luma_width;
        let lh = f.luma_height;
        let mut out = f.clone();
        for r in 0..lh {
            for c in 0..lw {
                out.y[r * lw + c] = f.y[((r + lh - dy) % lh) * lw + (c + lw - dx) % lw];
            }
        }
        let cw = lw / 2;
        let ch = lh / 2;
        for r in 0..ch {
            for c in 0..cw {
                let s = ((r + ch - dy / 2) % ch) * cw + (c + cw - dx / 2) % cw;
                out.cb[r * cw + c] = f.cb[s];
                out.cr[r * cw + c] = f.cr[s];
            }
        }
        out
    }

    fn psnr(a: &[u8], b: &[u8]) -> f64 {
        let mse: f64 = a
            .iter()
            .zip(b.iter())
            .map(|(&x, &y)| {
                let d = x as f64 - y as f64;
                d * d
            })
            .sum::<f64>()
            / a.len() as f64;
        if mse == 0.0 {
            99.0
        } else {
            10.0 * (255.0f64 * 255.0 / mse).log10()
        }
    }

    fn opptype_of(bytes: &[u8]) -> crate::plus_ptype::Opptype {
        let mut r = BitReader::new(bytes);
        match parse_picture_layer(&mut r, InheritedExtendedState::default()).unwrap() {
            H263PictureLayer::Extended(e) => e.plus.opptype.unwrap(),
            H263PictureLayer::Baseline(_) => panic!("baseline header"),
        }
    }

    #[test]
    fn intra_deblock_picture_signals_annex_j_and_decodes() {
        let f = textured(176, 144, 1);
        let bytes = encode_intra_picture_deblock(&f, 10, 3).unwrap();
        let opp = opptype_of(&bytes);
        assert!(opp.deblocking && !opp.advanced_prediction && !opp.umv);
        let dec = decode_picture_layer(&bytes, None, DecodeOptions::default()).unwrap();
        assert!(psnr(&f.y, &dec.y) > 30.0);
    }

    #[test]
    fn inter_deblock_one_and_four_vector_forms_round_trip() {
        let base = textured(176, 144, 2);
        let next = shifted(&base, 3, 2);
        let i = encode_intra_picture_deblock(&base, 8, 0).unwrap();
        let recon = decode_picture_layer(&i, None, DecodeOptions::default()).unwrap();
        for (four_mv, umv) in [(false, false), (true, false), (false, true), (true, true)] {
            let cfg = DeblockConfig {
                search_half: 6,
                four_mv,
                umv,
            };
            let p = encode_inter_picture_deblock(&next, &recon, 8, 1, &cfg).unwrap();
            let opp = opptype_of(&p);
            assert!(opp.deblocking);
            assert_eq!(opp.umv, umv);
            let dec = decode_picture_layer(&p, Some(&recon), DecodeOptions::default()).unwrap();
            let q = psnr(&next.y, &dec.y);
            assert!(q > 32.0, "four_mv={four_mv} umv={umv}: PSNR {q:.2}");
        }
    }

    #[test]
    fn static_inter_deblock_picture_is_all_skipped() {
        let base = textured(128, 96, 4);
        let i = encode_intra_picture_deblock(&base, 12, 0).unwrap();
        let recon = decode_picture_layer(&i, None, DecodeOptions::default()).unwrap();
        // Re-encoding the *reconstruction* against itself: every
        // macroblock has a zero vector and no residual → COD = 1 for
        // all, and the picture is the header plus stuffing.
        let p =
            encode_inter_picture_deblock(&recon, &recon, 12, 1, &DeblockConfig::default()).unwrap();
        assert!(p.len() <= 16, "static P-picture is {} bytes", p.len());
        let dec = decode_picture_layer(&p, Some(&recon), DecodeOptions::default()).unwrap();
        assert_eq!(dec.y, recon.y);
    }

    #[test]
    fn four_vector_search_engages_on_divergent_motion() {
        // Left half moves right, right half moves down — a 16×16
        // vector cannot fit macroblocks straddling the seam, so the
        // four-vector form must beat the one-vector form on rate at
        // equal quality (or better quality at equal rate).
        let base = textured(176, 144, 6);
        let lw = 176;
        let lh = 144;
        let mut next = base.clone();
        for r in 0..lh {
            for c in 0..lw {
                let (sr, sc) = if c < lw / 2 {
                    (r, (c + lw - 4) % lw)
                } else {
                    ((r + lh - 4) % lh, c)
                };
                next.y[r * lw + c] = base.y[sr * lw + sc];
            }
        }
        let i = encode_intra_picture_deblock(&base, 8, 0).unwrap();
        let recon = decode_picture_layer(&i, None, DecodeOptions::default()).unwrap();
        let one = encode_inter_picture_deblock(
            &next,
            &recon,
            8,
            1,
            &DeblockConfig {
                search_half: 6,
                four_mv: false,
                umv: false,
            },
        )
        .unwrap();
        let four = encode_inter_picture_deblock(
            &next,
            &recon,
            8,
            1,
            &DeblockConfig {
                search_half: 6,
                four_mv: true,
                umv: false,
            },
        )
        .unwrap();
        let d1 = decode_picture_layer(&one, Some(&recon), DecodeOptions::default()).unwrap();
        let d4 = decode_picture_layer(&four, Some(&recon), DecodeOptions::default()).unwrap();
        let (q1, q4) = (psnr(&next.y, &d1.y), psnr(&next.y, &d4.y));
        assert!(
            four.len() < one.len() || q4 > q1,
            "4V {} bytes / {q4:.2} dB vs 1V {} bytes / {q1:.2} dB",
            four.len(),
            one.len()
        );
    }
}
