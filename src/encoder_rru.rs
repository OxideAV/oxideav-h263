//! Annex Q Reduced-Resolution Update INTER-picture encoder with the
//! mode combinations of §Q: Unrestricted Motion Vectors (§Q.4 Table
//! D.3 pseudo-vector differences), Advanced Prediction (§Q.5 enlarged
//! overlapped motion compensation, four §F.2 vectors per 32 × 32
//! macroblock), Deblocking Filter (§Q.7.2; four vectors without the
//! overlap) and the Annex K slice layer (the Tables K.2 / K.3 "RRU
//! mode" field widths, macroblock addresses on the 32 × 32 grid).
//!
//! The picture is coded in two passes: the first chooses every
//! macroblock's vectors (a search in the pseudo-vector domain, so each
//! candidate expands to a legal half-integer-or-zero vector, §Q.4),
//! the second forms the predictions with the whole vector field known
//! — the §F.3 remote vectors of a block include the macroblock to its
//! right — and codes the down-sampled residuals.

use oxideav_core::bits::BitWriter;

use crate::encoder::{
    PlusModes, rru_downsample_16, rru_downsample_residual, write_plus_picture_header,
};
use crate::encoder_block::{EncodedInterBlock, encode_inter_block};
use crate::encoder_motion::{Mv4Grid, mvd_for};
use crate::macroblock::Mvd;
use crate::mode_rules::{ModeRule, ModeSet};
use crate::motion::{
    LumaBlockIndex, Mb4Mv, MotionVector, RefPlane, RemoteMv, chroma_mv, chroma_mv_4mv,
    obmc_predict_block_16, rru_actual_mv, rru_pseudo_mv,
};
use crate::picture::{YuvFrame, rru_motion_compensate_16_pub};
use crate::plus_ptype::{PlusPictureType, SliceStructuredSubmode};
use crate::slice_header::{SliceHeaderContext, write_first_slice_header, write_slice_layer};
use crate::{Error, Result};

/// Options of [`encode_inter_picture_rru_with`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct RruInterConfig {
    /// QUANT (1..=31).
    pub quant: u8,
    /// Search half-width around the pseudo-predictor, in half-pel
    /// steps of the pseudo-vector domain.
    pub search_pseudo_half: i32,
    /// Annex D: Table D.3 pseudo-vector differences under the UUI
    /// "1" range (§Q.4).
    pub umv: bool,
    /// Annex J: the §Q.7.2 block boundary filter variant.
    pub deblock: bool,
    /// Annex F: §Q.5 enlarged overlapped motion compensation.
    pub advanced_prediction: bool,
    /// Let a macroblock carry four vectors (one per 16 × 16 luminance
    /// block) when that lowers the estimated cost. Requires
    /// `advanced_prediction` or `deblock` (§F.2).
    pub four_mv: bool,
    /// Annex K: start a new slice every this many 32 × 32 macroblock
    /// rows (`0` = no slice layer, one video picture segment).
    pub slice_rows: usize,
    /// Code a macroblock INTRA when its best motion-compensated match
    /// is worse than its own deviation from its mean (uncovered
    /// content, scene changes).
    pub intra_decision: bool,
}

impl RruInterConfig {
    /// Defaults: the given QUANT, a ±2 pseudo-half-pel search, every
    /// optional mode off.
    pub fn new(quant: u8) -> Self {
        RruInterConfig {
            quant,
            search_pseudo_half: 2,
            umv: false,
            deblock: false,
            advanced_prediction: false,
            four_mv: false,
            slice_rows: 0,
            intra_decision: false,
        }
    }
    /// Set [`RruInterConfig::search_pseudo_half`].
    #[must_use]
    pub fn with_search_pseudo_half(mut self, v: i32) -> Self {
        self.search_pseudo_half = v;
        self
    }
    /// Set [`RruInterConfig::umv`].
    #[must_use]
    pub fn with_umv(mut self, v: bool) -> Self {
        self.umv = v;
        self
    }
    /// Set [`RruInterConfig::deblock`].
    #[must_use]
    pub fn with_deblock(mut self, v: bool) -> Self {
        self.deblock = v;
        self
    }
    /// Set [`RruInterConfig::advanced_prediction`].
    #[must_use]
    pub fn with_advanced_prediction(mut self, v: bool) -> Self {
        self.advanced_prediction = v;
        self
    }
    /// Set [`RruInterConfig::four_mv`].
    #[must_use]
    pub fn with_four_mv(mut self, v: bool) -> Self {
        self.four_mv = v;
        self
    }
    /// Set [`RruInterConfig::slice_rows`].
    #[must_use]
    pub fn with_slice_rows(mut self, v: usize) -> Self {
        self.slice_rows = v;
        self
    }
    /// Set [`RruInterConfig::intra_decision`].
    #[must_use]
    pub fn with_intra_decision(mut self, v: bool) -> Self {
        self.intra_decision = v;
        self
    }
}

/// Per-picture counts returned by [`encode_inter_picture_rru_stats`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct RruInterStats {
    /// Skipped (COD = 1) macroblocks.
    pub skipped: usize,
    /// Coded one-vector macroblocks.
    pub one_vector: usize,
    /// Coded four-vector (INTER4V) macroblocks.
    pub four_vector: usize,
    /// INTRA macroblocks.
    pub intra: usize,
    /// Slices emitted (`1` without the slice layer).
    pub slices: usize,
}

/// Encode `frame` as an **Annex Q Reduced-Resolution Update INTER**
/// picture predicted from `reference` (the previously **decoded**
/// reconstruction, §Q.2.3) with the mode set in `cfg`.
///
/// Self-describing (PLUSPTYPE, UFEP = "001"); decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence`. The
/// source must be one of the five standard formats.
///
/// # Errors
///
/// [`Error::ModeCombination`] when `cfg` asks for a combination the
/// Recommendation forbids (four vectors without Advanced Prediction
/// or Deblocking Filter, §F.2); [`Error::InvalidQuantiser`];
/// [`Error::NotImplemented`] for non-standard picture sizes.
pub fn encode_inter_picture_rru_with(
    frame: &YuvFrame,
    reference: &YuvFrame,
    tr: u8,
    cfg: &RruInterConfig,
) -> Result<Vec<u8>> {
    encode_inter_picture_rru_stats(frame, reference, tr, cfg).map(|(bytes, _)| bytes)
}

/// [`encode_inter_picture_rru_with`] that also reports the
/// macroblock-mode counts.
pub fn encode_inter_picture_rru_stats(
    frame: &YuvFrame,
    reference: &YuvFrame,
    tr: u8,
    cfg: &RruInterConfig,
) -> Result<(Vec<u8>, RruInterStats)> {
    let quant = cfg.quant;
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if cfg.four_mv && !(cfg.advanced_prediction || cfg.deblock) {
        return Err(Error::ModeCombination(ModeRule::FourVectorsWithoutApOrDf));
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt = crate::encoder::source_format_for(frame.luma_width, frame.luma_height)
        .ok_or(Error::NotImplemented)?;
    let sss = (cfg.slice_rows > 0).then_some(SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    });
    ModeSet::new(PlusPictureType::Inter, true)
        .with_reduced_resolution_update(true)
        .with_umv(cfg.umv)
        .with_deblocking(cfg.deblock)
        .with_advanced_prediction(cfg.advanced_prediction)
        .with_four_vectors(cfg.four_mv)
        .with_slice_structured(sss)
        .check()?;

    let (hr, vr) = (
        frame.luma_width.div_ceil(16) * 16,
        frame.luma_height.div_ceil(16) * 16,
    );
    let (hc, vc) = crate::picture::rru_geometry_for_display(frame.luma_width, frame.luma_height);
    let ext_src = crate::picture::extend_frame_rru(frame, hc, vc);
    let ext_ref = crate::picture::extend_frame_rru(reference, hc, vc);
    let mb_cols = hc / 32;
    let mb_rows = vc / 32;
    let mb_count = mb_cols * mb_rows;
    let ap = cfg.advanced_prediction;
    let slice_rows = if cfg.slice_rows == 0 {
        mb_rows
    } else {
        cfg.slice_rows
    };

    let y_ref = RefPlane::new(&ext_ref.y, hc, vc);
    let cb_ref = RefPlane::new(&ext_ref.cb, hc / 2, vc / 2);
    let cr_ref = RefPlane::new(&ext_ref.cr, hc / 2, vc / 2);

    // Pseudo-domain candidate window: the [-16, 15.5]-pel default
    // (§Q.4 item 2), or the Tables D.1 / D.2 range under UMV (§D.2 —
    // "the specified range applies to the pseudo motion vectors").
    let (px_min, px_max) = if cfg.umv {
        crate::motion::umv_plus_horizontal_range_half(frame.luma_width as u32)
    } else {
        (-32, 31)
    };
    let (py_min, py_max) = if cfg.umv {
        crate::motion::umv_plus_vertical_range_half(frame.luma_height as u32)
    } else {
        (-32, 31)
    };
    // Without UMV / AP / DF a vector shall not reference pixels
    // outside the picture; §Q.3 leaves the extension area undefined
    // then, so every *stored* pixel (inside HR × VR) must be predicted
    // from inside HR × VR.
    let confined = !(cfg.umv || ap || cfg.deblock);
    let inside = |bx: usize, by: usize, size: usize, mv: MotionVector| -> bool {
        if !confined {
            return true;
        }
        let ok = |p0: usize, limit: usize, v: i32| -> bool {
            if p0 >= limit {
                // Entirely in the extension area: never stored.
                return true;
            }
            let last = (p0 + size).min(limit) as i32 - 1;
            let lo = 2 * p0 as i32 + v;
            let hi = 2 * last + v;
            lo >= 0 && hi <= 2 * (limit as i32 - 1)
        };
        ok(bx, hr, mv.dx_half) && ok(by, vr, mv.dy_half)
    };

    let block_sad = |bx: usize, by: usize, mv: MotionVector| -> u32 {
        let pred = rru_motion_compensate_16_pub(&y_ref, bx, by, mv, 0);
        let mut sad = 0u32;
        for (j, prow) in pred.as_chunks::<16>().0.iter().enumerate() {
            let srow = &ext_src.y[(by + j) * hc + bx..(by + j) * hc + bx + 16];
            for (&s, &p) in srow.iter().zip(prow) {
                sad += (s as i32 - p as i32).unsigned_abs();
            }
        }
        sad
    };
    let lambda = 2 * quant as u32;
    // Search the pseudo window around `pseudo_pc` for the region made
    // of `blocks` (16 × 16 luminance block origins).
    let search = |blocks: &[(usize, usize)],
                  extent: (usize, usize, usize),
                  pseudo_pc: MotionVector|
     -> (MotionVector, u32) {
        let cost_of = |pseudo: MotionVector| -> Option<u32> {
            let actual = rru_actual_mv(pseudo);
            if !inside(extent.0, extent.1, extent.2, actual) {
                return None;
            }
            let bits = (pseudo.dx_half - pseudo_pc.dx_half).unsigned_abs()
                + (pseudo.dy_half - pseudo_pc.dy_half).unsigned_abs();
            Some(
                blocks
                    .iter()
                    .map(|&(bx, by)| block_sad(bx, by, actual))
                    .sum::<u32>()
                    + lambda * bits,
            )
        };
        // The zero vector is always legal and is the fallback.
        let zero = MotionVector::new(0, 0);
        let mut best = zero;
        let mut best_cost = cost_of(zero).unwrap_or(u32::MAX);
        let s = cfg.search_pseudo_half;
        for dy in -s..=s {
            for dx in -s..=s {
                let cand = MotionVector::new(pseudo_pc.dx_half + dx, pseudo_pc.dy_half + dy);
                if !(px_min..=px_max).contains(&cand.dx_half)
                    || !(py_min..=py_max).contains(&cand.dy_half)
                {
                    continue;
                }
                if let Some(c) = cost_of(cand)
                    && c < best_cost
                {
                    best_cost = c;
                    best = cand;
                }
            }
        }
        (best, best_cost)
    };
    let difference = |pseudo: MotionVector, pseudo_pc: MotionVector| -> Mvd {
        if cfg.umv {
            // §Q.4 / §D.2 — the Table D.3 difference is the plain
            // pseudo-domain `pseudo − pseudo-PC`.
            Mvd {
                dx_half: (pseudo.dx_half - pseudo_pc.dx_half) as i16,
                dy_half: (pseudo.dy_half - pseudo_pc.dy_half) as i16,
            }
        } else {
            mvd_for(pseudo, pseudo_pc)
        }
    };

    // ---- pass 1: the vector field ------------------------------------
    let mut grid = Mv4Grid::with_row_segments(mb_cols, mb_rows, slice_rows);
    let mut field: Vec<Mb4Mv> = Vec::with_capacity(mb_count);
    let mut four: Vec<bool> = Vec::with_capacity(mb_count);
    let mut mvds: Vec<[Mvd; 4]> = Vec::with_capacity(mb_count);
    let mut intra: Vec<bool> = Vec::with_capacity(mb_count);
    let zero = MotionVector::new(0, 0);
    for row in 0..mb_rows {
        for col in 0..mb_cols {
            let (mb_x, mb_y) = (col * 32, row * 32);
            let origins: [(usize, usize); 4] =
                std::array::from_fn(|b| (mb_x + (b % 2) * 16, mb_y + (b / 2) * 16));

            // One vector: §F.2 defines the candidates as for block 1.
            let pc1 = rru_pseudo_mv(grid.predict_block(col, row, LumaBlockIndex::B1, &[zero; 4]));
            let (p1, cost1) = search(&origins, (mb_x, mb_y, 32), pc1);
            let mut choice: Mb4Mv = [rru_actual_mv(p1); 4];
            let mut choice_mvds = [difference(p1, pc1); 4];
            let mut is_four = false;
            let mut chosen_cost = cost1;

            if cfg.four_mv {
                let mut cur: Mb4Mv = [zero; 4];
                let mut cur_mvds = choice_mvds;
                let mut cost4 = 0u32;
                for &blk in &LumaBlockIndex::ALL {
                    let b = blk.index();
                    let pc = rru_pseudo_mv(grid.predict_block(col, row, blk, &cur));
                    let (p, c) = search(&origins[b..=b], (origins[b].0, origins[b].1, 16), pc);
                    cur[b] = rru_actual_mv(p);
                    cur_mvds[b] = difference(p, pc);
                    cost4 = cost4.saturating_add(c);
                }
                // Three extra vector differences and the longer MCBPC
                // must pay for themselves.
                if cost4.saturating_add(lambda * 8) < cost1 {
                    choice = cur;
                    choice_mvds = cur_mvds;
                    is_four = true;
                    chosen_cost = cost4;
                }
            }
            // INTRA when the macroblock is closer to its own mean than
            // to its best prediction (half a level per pixel of bias
            // towards INTER, whose residual is usually cheaper).
            let mut is_intra = false;
            if cfg.intra_decision {
                let rows = (mb_y..mb_y + 32).map(|y| &ext_src.y[y * hc + mb_x..y * hc + mb_x + 32]);
                let sum: u32 = rows.clone().flatten().map(|&s| s as u32).sum();
                let mean = (sum + 512) / 1024;
                let activity: u32 = rows.flatten().map(|&s| (s as u32).abs_diff(mean)).sum();
                if activity + 512 < chosen_cost {
                    is_intra = true;
                    is_four = false;
                    choice = [zero; 4];
                }
            }
            grid.set(col, row, choice);
            field.push(choice);
            four.push(is_four);
            mvds.push(choice_mvds);
            intra.push(is_intra);
        }
    }

    // ---- pass 2: prediction, residual, bitstream ----------------------
    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PlusModes {
            rru: true,
            umv: cfg.umv,
            deblocking: cfg.deblock,
            advanced_prediction: ap,
            slice_structured: sss,
            ..PlusModes::default()
        },
    )?;
    let ctx = SliceHeaderContext {
        picture_width: frame.luma_width as u32,
        picture_height: frame.luma_height as u32,
        cpm: false,
        rectangular_slices: false,
        rru: true,
    };
    if sss.is_some() {
        write_first_slice_header(&mut w, &ctx, 0, None)?;
    }
    let gfid = tr & 0b11;
    let mut stats = RruInterStats {
        slices: 1,
        ..RruInterStats::default()
    };
    let segment_of = |row: usize| row / slice_rows;

    for row in 0..mb_rows {
        if sss.is_some() && row > 0 && row % slice_rows == 0 {
            write_slice_layer(&mut w, &ctx, (row * mb_cols) as u32, quant, gfid, None)?;
            stats.slices += 1;
        }
        for col in 0..mb_cols {
            let idx = row * mb_cols + col;
            let cur = field[idx];
            let (mb_x, mb_y) = (col * 32, row * 32);
            let (c_x, c_y) = (col * 16, row * 16);

            // §F.3 remote vectors: every macroblock of this picture is
            // INTER (a skipped one carries the zero vector, which is
            // also what the not-coded rule substitutes), so a remote
            // is the neighbouring block's vector, or the current one
            // at the picture border and — §K.1 rule 3 — at a slice
            // border.
            if intra[idx] {
                let mut samples = crate::encoder_mb::MacroblockSamples {
                    luma: [[0i16; 64]; 4],
                    cb: [0i16; 64],
                    cr: [0i16; 64],
                };
                for (b, blk) in samples.luma.iter_mut().enumerate() {
                    *blk =
                        rru_downsample_16(&ext_src.y, hc, mb_x + (b % 2) * 16, mb_y + (b / 2) * 16);
                }
                samples.cb = rru_downsample_16(&ext_src.cb, hc / 2, c_x, c_y);
                samples.cr = rru_downsample_16(&ext_src.cr, hc / 2, c_x, c_y);
                crate::encoder_mb::encode_intra_macroblock(
                    &mut w, &samples, quant, /* write_cod */ true,
                    /* picture_is_inter */ true,
                )?;
                stats.intra += 1;
                continue;
            }
            // An INTRA neighbour's remote vector is the current one.
            let above =
                (row > 0 && segment_of(row - 1) == segment_of(row) && !intra[idx - mb_cols])
                    .then(|| field[idx - mb_cols]);
            let left = (col > 0 && !intra[idx - 1]).then(|| field[idx - 1]);
            let right = (col + 1 < mb_cols && !intra[idx + 1]).then(|| field[idx + 1]);
            let remote = |nb: Option<Mb4Mv>, cell: usize| -> RemoteMv {
                nb.map_or(RemoteMv::Current, |m| RemoteMv::Vector(m[cell]))
            };
            let remotes = |b: usize| -> (RemoteMv, RemoteMv, RemoteMv, RemoteMv) {
                match b {
                    0 => (
                        remote(above, 2),
                        RemoteMv::Vector(cur[2]),
                        remote(left, 1),
                        RemoteMv::Vector(cur[1]),
                    ),
                    1 => (
                        remote(above, 3),
                        RemoteMv::Vector(cur[3]),
                        RemoteMv::Vector(cur[0]),
                        remote(right, 0),
                    ),
                    2 => (
                        RemoteMv::Vector(cur[0]),
                        RemoteMv::Current,
                        remote(left, 3),
                        RemoteMv::Vector(cur[3]),
                    ),
                    _ => (
                        RemoteMv::Vector(cur[1]),
                        RemoteMv::Current,
                        RemoteMv::Vector(cur[2]),
                        remote(right, 2),
                    ),
                }
            };

            let code = |pred: &[u8; 256],
                        plane: &[u8],
                        stride: usize,
                        x0: usize,
                        y0: usize|
             -> EncodedInterBlock {
                let mut res = [0i32; 256];
                for (j, (rrow, prow)) in res
                    .as_chunks_mut::<16>()
                    .0
                    .iter_mut()
                    .zip(pred.as_chunks::<16>().0)
                    .enumerate()
                {
                    let srow = &plane[(y0 + j) * stride + x0..(y0 + j) * stride + x0 + 16];
                    for ((r, &s), &p) in rrow.iter_mut().zip(srow).zip(prow) {
                        *r = s as i32 - p as i32;
                    }
                }
                encode_inter_block(&rru_downsample_residual(&res), quant)
            };

            let luma: [EncodedInterBlock; 4] = std::array::from_fn(|b| {
                let bx = mb_x + (b % 2) * 16;
                let by = mb_y + (b / 2) * 16;
                let pred = if ap {
                    let (r_top, r_bot, s_left, s_right) = remotes(b);
                    obmc_predict_block_16(&y_ref, bx, by, cur[b], r_top, r_bot, s_left, s_right, 0)
                } else {
                    rru_motion_compensate_16_pub(&y_ref, bx, by, cur[b], 0)
                };
                code(&pred, &ext_src.y, hc, bx, by)
            });
            let chroma_vec = if four[idx] {
                chroma_mv_4mv(&cur)
            } else {
                chroma_mv(cur[0])
            };
            let cb_pred = rru_motion_compensate_16_pub(&cb_ref, c_x, c_y, chroma_vec, 0);
            let cr_pred = rru_motion_compensate_16_pub(&cr_ref, c_x, c_y, chroma_vec, 0);
            let cb = code(&cb_pred, &ext_src.cb, hc / 2, c_x, c_y);
            let cr = code(&cr_pred, &ext_src.cr, hc / 2, c_x, c_y);

            let any = luma.iter().any(|e| e.has_coeffs) || cb.has_coeffs || cr.has_coeffs;
            if four[idx] {
                stats.four_vector += 1;
                if cfg.umv {
                    crate::encoder_mb::encode_inter4v_macroblock_umv_plus(
                        &mut w, &luma, &cb, &cr, &mvds[idx],
                    )?;
                } else {
                    crate::encoder_mb::encode_inter4v_macroblock(
                        &mut w, &luma, &cb, &cr, &mvds[idx],
                    )?;
                }
            } else if !any && cur[0] == zero {
                stats.skipped += 1;
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
            } else {
                stats.one_vector += 1;
                if cfg.umv {
                    crate::encoder_mb::encode_inter_macroblock_umv_plus(
                        &mut w,
                        &luma,
                        &cb,
                        &cr,
                        mvds[idx][0],
                    )?;
                } else {
                    crate::encoder_mb::encode_inter_macroblock(
                        &mut w,
                        &luma,
                        &cb,
                        &cr,
                        mvds[idx][0],
                    )?;
                }
            }
        }
    }

    w.align_to_byte_zero();
    Ok((w.finish(), stats))
}
