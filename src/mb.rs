//! H.263 macroblock-level decoding for I- and P-pictures.
//!
//! For an I-picture, per-MB decode sequence (§5.3):
//! 1. **MCBPC** (Table 14/H.263) — picks `(mb_type, cbpc)`.
//!    `mb_type = 3` (Intra) or `4` (IntraQ); `cbpc` is the 2-bit chroma CBP.
//! 2. **CBPY** (Table 13/H.263, intra variant) — 4-bit luma CBP. We treat
//!    the decoded value directly as the bit-pattern of "block has AC
//!    coefficients" flags (matching `oxideav-mpeg4video`'s I-VOP convention,
//!    cross-checked against the `h263-rs` table).
//! 3. **DQUANT** — 2 signed bits, present iff `mb_type == IntraQ`. Adjusts
//!    QUANT by `[-1, -2, 1, 2]` for codes `0..=3`.
//! 4. **Per-block** (Y0..Y3, Cb, Cr): 8-bit INTRADC + optional TCOEF.
//!
//! For a P-picture, per-MB decode sequence (§5.3.1 / §5.3.5):
//! 1. **COD** — 1 bit. `1` means "not coded" → the MB is copied verbatim from
//!    the reference at the same position with MV(0,0).
//! 2. **MCBPC** (Table 16/H.263 inter) — picks `(mb_type, cbpc)`. We accept
//!    `Inter`, `InterQ`, `Intra`, `IntraQ`; `Inter4MV` / `Inter4MV+Q` are
//!    rejected (Annex F advanced prediction — out of scope).
//! 3. **CBPY** — for inter, bit-inverted of Table 13; for intra embedded in
//!    P, the raw (non-inverted) pattern.
//! 4. **DQUANT** — only if mb_type is `*Q`.
//! 5. **MV** — 2 half-pel components via `motion::decode_mv_component` using
//!    the median predictor over decoded neighbours.
//! 6. **Per-block texture**:
//!    * Inter: TCOEF at scan index 0 (no INTRADC), dequantise → IDCT →
//!      residual; add to the half-pel motion-compensated predictor then clip.
//!    * Intra-in-P: same path as I-pictures (INTRADC + AC).

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};
use oxideav_mpeg4video::tables::{cbpy, mcbpc, vlc};

use crate::block::{decode_ac, decode_intradc, idct_and_clip};
use crate::interp::predict_block;
use crate::motion::{
    chroma_mv_4mv, consume_mvd_pair_sce_bit, decode_mv_component_umv, luma_to_chroma_mv,
    predict_mv_block_with_gob_mask, predict_mv_with_gob_mask, MbMotion, MvGrid, OBMC_H0, OBMC_H1,
    OBMC_H2,
};

/// Signed `dquant` adjustment — Table 12/H.263. 2-bit code indexes `[-1, -2, 1, 2]`.
const DQUANT_DELTA: [i32; 4] = [-1, -2, 1, 2];

/// Per-picture MV-decode mode, plumbed from the picture-header UMV flags to
/// the MB decoder.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UmvMode {
    /// UMV off — Table 14 VLC, range folded into `[-32, +31]` halfpel.
    #[default]
    Off,
    /// UMV on, baseline PTYPE form (no PLUSPTYPE): Table 14 VLC + §D.2
    /// sign-of-predictor reconstruction. See
    /// [`crate::motion::decode_mv_component_umv`].
    BaselinePtype,
    /// UMV on, PLUSPTYPE form: Table D.3 VLC + direct reconstruction
    /// (`predictor + differential`) + MVD-pair SCE stuffing bit after
    /// `(+1,+1)` halfpel pairs.
    ///
    /// `h_limit` / `v_limit` are the per-axis UMV range for UUI="1"
    /// (Tables D.1 / D.2 of H.263) or `None` for UUI="01" (range
    /// unlimited except by picture size, which we do not enforce here).
    PlusPtype {
        h_limit: Option<(i32, i32)>,
        v_limit: Option<(i32, i32)>,
    },
}

impl UmvMode {
    /// Convert a decoded [`crate::picture::PictureHeader`] into the
    /// MV-decode mode used by `decode_p_mb` / `decode_p_mb_pass1`.
    pub fn from_header(hdr: &crate::picture::PictureHeader) -> Self {
        if !hdr.umv_mode {
            return UmvMode::Off;
        }
        if !hdr.plusptype {
            return UmvMode::BaselinePtype;
        }
        if hdr.uui_unlimited {
            UmvMode::PlusPtype {
                h_limit: None,
                v_limit: None,
            }
        } else {
            UmvMode::PlusPtype {
                h_limit: Some(crate::motion::uui_limit_range_halfpel(hdr.width)),
                v_limit: Some(crate::motion::uui_limit_range_halfpel(hdr.height)),
            }
        }
    }
}

/// Reconstructed I-picture: three pel planes (Y, Cb, Cr), MB-aligned, stride
/// equal to MB-aligned width.
#[derive(Clone)]
pub struct IPicture {
    pub width: usize,
    pub height: usize,
    pub mb_width: usize,
    pub mb_height: usize,
    pub y: Vec<u8>,
    pub cb: Vec<u8>,
    pub cr: Vec<u8>,
    pub y_stride: usize,
    pub c_stride: usize,
}

impl IPicture {
    pub fn new(width: usize, height: usize) -> Self {
        let mb_w = width.div_ceil(16);
        let mb_h = height.div_ceil(16);
        let y_stride = mb_w * 16;
        let c_stride = mb_w * 8;
        let y_h = mb_h * 16;
        let c_h = mb_h * 8;
        Self {
            width,
            height,
            mb_width: mb_w,
            mb_height: mb_h,
            y_stride,
            c_stride,
            y: vec![0u8; y_stride * y_h],
            cb: vec![0u8; c_stride * c_h],
            cr: vec![0u8; c_stride * c_h],
        }
    }
}

/// Decode one I-picture intra macroblock. Returns the (possibly updated)
/// quantiser.
pub fn decode_intra_mb(
    br: &mut BitReader<'_>,
    mb_x: usize,
    mb_y: usize,
    quant_in: u32,
    pic: &mut IPicture,
) -> Result<u32> {
    // 1. MCBPC — loop over stuffing.
    let mcbpc_v = loop {
        let v = vlc::decode(br, mcbpc::i_table())?;
        if v != mcbpc::STUFFING {
            break v;
        }
    };
    let (is_intra_q, cbpc) = if mcbpc_v < 4 {
        (false, mcbpc_v)
    } else if mcbpc_v < 8 {
        (true, mcbpc_v - 4)
    } else {
        return Err(Error::invalid("h263 MB: invalid MCBPC value"));
    };

    // 2. CBPY (intra variant — direct, no XOR).
    let cbpy = vlc::decode(br, cbpy::table())?;

    // 3. DQUANT.
    let mut quant = quant_in;
    if is_intra_q {
        let d = br.read_u32(2)? as usize;
        let new_q = (quant as i32) + DQUANT_DELTA[d];
        quant = new_q.clamp(1, 31) as u32;
    }

    // 4. Per-block decode.
    // CBPY bit 3 (MSB) -> block 0, bit 0 (LSB) -> block 3 (per spec ordering).
    let luma_coded = [
        (cbpy >> 3) & 1 != 0,
        (cbpy >> 2) & 1 != 0,
        (cbpy >> 1) & 1 != 0,
        cbpy & 1 != 0,
    ];
    let chroma_coded = [(cbpc >> 1) & 1 != 0, cbpc & 1 != 0];

    for block_idx in 0..6usize {
        let coded = if block_idx < 4 {
            luma_coded[block_idx]
        } else {
            chroma_coded[block_idx - 4]
        };
        decode_one_intra_block(br, block_idx, coded, mb_x, mb_y, quant, pic)?;
    }

    Ok(quant)
}

fn decode_one_intra_block(
    br: &mut BitReader<'_>,
    block_idx: usize,
    has_ac: bool,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    pic: &mut IPicture,
) -> Result<()> {
    // INTRADC always present for intra blocks.
    let dc = decode_intradc(br)?;
    let mut coeffs = [0i32; 64];
    coeffs[0] = dc;

    if has_ac {
        decode_ac(br, &mut coeffs, 1, quant)?;
    }

    // Saturate the DC coefficient to spec range.
    coeffs[0] = coeffs[0].clamp(-2048, 2047);

    // IDCT + clip.
    let mut out = [0u8; 64];
    idct_and_clip(&mut coeffs, &mut out);

    write_block_to_picture(pic, block_idx, mb_x, mb_y, &out);
    Ok(())
}

/// Write the 8×8 reconstructed block into the picture buffer.
fn write_block_to_picture(
    pic: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    out: &[u8; 64],
) {
    let (plane, stride, px, py) = block_dst(pic, block_idx, mb_x, mb_y);
    for dy in 0..8 {
        for dx in 0..8 {
            plane[(py + dy) * stride + (px + dx)] = out[dy * 8 + dx];
        }
    }
}

/// Block-layout helper: return the plane slice + stride + top-left pel for
/// block `block_idx` (0..=5) of MB `(mb_x, mb_y)`.
fn block_dst(
    pic: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
) -> (&mut [u8], usize, usize, usize) {
    match block_idx {
        0 => (pic.y.as_mut_slice(), pic.y_stride, mb_x * 16, mb_y * 16),
        1 => (pic.y.as_mut_slice(), pic.y_stride, mb_x * 16 + 8, mb_y * 16),
        2 => (pic.y.as_mut_slice(), pic.y_stride, mb_x * 16, mb_y * 16 + 8),
        3 => (
            pic.y.as_mut_slice(),
            pic.y_stride,
            mb_x * 16 + 8,
            mb_y * 16 + 8,
        ),
        4 => (pic.cb.as_mut_slice(), pic.c_stride, mb_x * 8, mb_y * 8),
        5 => (pic.cr.as_mut_slice(), pic.c_stride, mb_x * 8, mb_y * 8),
        _ => unreachable!(),
    }
}

/// Per-MB decoded state retained after the first pass of a P-picture body, so
/// the second pass (Annex F OBMC) can apply the overlapped motion-compensation
/// predictor with full knowledge of the neighbour MVs before writing final
/// pels into `pic`.
///
/// The residual blocks are stored in signed-IDCT form (pre-addition to the
/// predictor); `coded[k]` says whether block `k` (0..4 luma, 4..6 chroma)
/// carries a residual at all. A non-coded block contributes zero residual to
/// the (pure MC) output.
#[derive(Clone)]
pub struct PMbInfo {
    pub coded: bool,
    pub intra: bool,
    /// Residual pels (IDCT output, signed) for each of the 6 blocks. Zeroes
    /// when `residual_present[k]` is false.
    pub residual: Vec<i16>,
    pub residual_present: [bool; 6],
    /// Intra path reconstructed pels (post-clip). When `intra` is true these
    /// blocks have already been written to `pic` during pass 1 and pass 2
    /// leaves them alone.
    pub intra_done: bool,
}

impl PMbInfo {
    pub fn empty_skipped() -> Self {
        Self {
            coded: false,
            intra: false,
            residual: vec![0i16; 6 * 64],
            residual_present: [false; 6],
            intra_done: false,
        }
    }

    pub fn residual_block(&self, k: usize) -> &[i16] {
        &self.residual[k * 64..(k + 1) * 64]
    }

    pub fn residual_block_mut(&mut self, k: usize) -> &mut [i16] {
        &mut self.residual[k * 64..(k + 1) * 64]
    }
}

/// Pass 1: decode one P-picture macroblock. Populates `mv_grid` and returns a
/// [`PMbInfo`] holding residuals so that [`apply_p_mb_reconstruction`] can do
/// the actual pixel write later. When `advanced_prediction` is `true`, the
/// MCBPC Inter4MV / Inter4MVQ codes are accepted and three extra MV codewords
/// (MVD2..MVD4, §5.3.8) are read.
///
/// Intra MBs are written to `pic` directly (no OBMC dependence) — they
/// don't need pass 2.
///
/// Returns the (possibly updated) quantiser.
#[allow(clippy::too_many_arguments)]
pub fn decode_p_mb_pass1(
    br: &mut BitReader<'_>,
    mb_x: usize,
    mb_y: usize,
    quant_in: u32,
    pic: &mut IPicture,
    mv_grid: &mut MvGrid,
    umv: UmvMode,
    advanced_prediction: bool,
    gob_top_row: bool,
) -> Result<(u32, PMbInfo)> {
    // 1. COD flag (§5.3.1). 1 → not_coded: MV (0,0), no residual. Pass 2 still
    //    runs OBMC on this MB when AP is active (§F.3 note).
    let cod = br.read_u1()?;
    if cod == 1 {
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        return Ok((quant_in, PMbInfo::empty_skipped()));
    }

    // 2. MCBPC inter.
    let mcbpc_v = loop {
        let v = vlc::decode(br, mcbpc::p_table())?;
        if v != mcbpc::INTER_STUFFING {
            break v;
        }
    };
    let (mb_type, cbpc) = mcbpc::decompose_inter(mcbpc_v);
    use mcbpc::PMbType;

    let is_4mv = matches!(mb_type, PMbType::Inter4MV | PMbType::Inter4MVQ);
    if is_4mv && !advanced_prediction {
        return Err(Error::unsupported(
            "h263 P-MB: Inter4MV present but Advanced Prediction mode not signalled",
        ));
    }
    let is_intra = matches!(mb_type, PMbType::Intra | PMbType::IntraQ);
    let needs_dquant = matches!(
        mb_type,
        PMbType::InterQ | PMbType::IntraQ | PMbType::Inter4MVQ
    );

    // 3. CBPY.
    let cbpy_raw = vlc::decode(br, cbpy::table())?;
    let cbpy = if is_intra { cbpy_raw } else { cbpy_raw ^ 0xF };

    // 4. DQUANT.
    let mut quant = quant_in;
    if needs_dquant {
        const DQUANT_DELTA: [i32; 4] = [-1, -2, 1, 2];
        let d = br.read_u32(2)? as usize;
        let new_q = (quant as i32) + DQUANT_DELTA[d];
        quant = new_q.clamp(1, 31) as u32;
    }

    // 5. Motion vectors.
    if is_intra {
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
    } else if is_4mv {
        // Decode 4 per-block MVs using the §F.2 Fig F.1 redefined predictors.
        // Each per-block MV is stored into `mv_grid` as it's decoded, so that
        // later blocks in the same MB can see it (blocks 1..3 reference
        // block 0 / block 0/1 / block 2 within the same MB).
        let mut mvs = [(0i32, 0i32); 4];
        // Start with the stored MB entry being 4-MV-mode so `predict_mv_block`
        // picks up partial in-MB neighbours correctly.
        mv_grid.set(mb_x, mb_y, MbMotion::mv4([(0, 0); 4]));
        for b in 0..4 {
            let (px, py) = predict_mv_block_with_gob_mask(mv_grid, mb_x, mb_y, b, gob_top_row);
            let (mvx, mvy) = decode_mv_pair(br, px, py, umv)?;
            mvs[b] = (mvx, mvy);
            // Update partial mvs4 so subsequent block predictors see it.
            let mut cur = mv_grid.get(mb_x, mb_y);
            cur.mvs4[b] = (mvx, mvy);
            cur.mv = mvs[0]; // summary MV = block 0's.
            mv_grid.set(mb_x, mb_y, cur);
        }
        mv_grid.set(mb_x, mb_y, MbMotion::mv4(mvs));
    } else {
        let (px, py) = predict_mv_with_gob_mask(mv_grid, mb_x, mb_y, gob_top_row);
        let (mvx, mvy) = decode_mv_pair(br, px, py, umv)?;
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((mvx, mvy), true, false));
    }

    // 6. Per-block texture.
    let luma_coded = [
        (cbpy >> 3) & 1 != 0,
        (cbpy >> 2) & 1 != 0,
        (cbpy >> 1) & 1 != 0,
        cbpy & 1 != 0,
    ];
    let chroma_coded = [(cbpc >> 1) & 1 != 0, cbpc & 1 != 0];

    if is_intra {
        // Intra blocks — decode and write directly to `pic` (no OBMC needed).
        for block_idx in 0..6usize {
            let coded = if block_idx < 4 {
                luma_coded[block_idx]
            } else {
                chroma_coded[block_idx - 4]
            };
            decode_one_intra_block_in_p(br, block_idx, coded, mb_x, mb_y, quant, pic)?;
        }
        return Ok((
            quant,
            PMbInfo {
                coded: true,
                intra: true,
                residual: vec![0i16; 6 * 64],
                residual_present: [false; 6],
                intra_done: true,
            },
        ));
    }

    // Inter path — decode each block's AC residual into the PMbInfo buffer;
    // do NOT touch `pic` yet. Pass 2 applies MC + residual.
    let mut info = PMbInfo {
        coded: true,
        intra: false,
        residual: vec![0i16; 6 * 64],
        residual_present: [false; 6],
        intra_done: false,
    };
    for block_idx in 0..6usize {
        let coded = if block_idx < 4 {
            luma_coded[block_idx]
        } else {
            chroma_coded[block_idx - 4]
        };
        if !coded {
            continue;
        }
        let mut coeffs = [0i32; 64];
        decode_ac(br, &mut coeffs, 0, quant)?;
        let mut resid = [0i32; 64];
        crate::block::idct_signed(&mut coeffs, &mut resid);
        let dst = info.residual_block_mut(block_idx);
        for (i, &v) in resid.iter().enumerate() {
            dst[i] = v.clamp(-4096, 4095) as i16;
        }
        info.residual_present[block_idx] = true;
    }
    Ok((quant, info))
}

/// Pass 2: apply motion compensation (with OBMC when `advanced_prediction` is
/// on) + residual addition to produce final pels in `pic`.
pub fn apply_p_mb_reconstruction(
    mb_x: usize,
    mb_y: usize,
    pic: &mut IPicture,
    reference: &IPicture,
    mv_grid: &MvGrid,
    info: &PMbInfo,
    advanced_prediction: bool,
) {
    // Intra MBs were written in pass 1.
    if info.intra_done {
        return;
    }

    let motion = mv_grid.get(mb_x, mb_y);

    // Luma blocks (0..=3).
    for block_idx in 0..4usize {
        let mut pred = [0u8; 64];
        if advanced_prediction {
            obmc_luma_block(
                &mut pred, reference, mv_grid, mb_x, mb_y, block_idx, &motion,
            );
        } else {
            // Plain single-MV MC.
            let (sub_x, sub_y) = block_offset(block_idx);
            let blk_px = (mb_x * 16 + sub_x) as i32;
            let blk_py = (mb_y * 16 + sub_y) as i32;
            let (mvx, mvy) = motion.mvs4[block_idx];
            let ref_y_h = reference.y.len() / reference.y_stride;
            predict_block(
                &reference.y,
                reference.y_stride,
                reference.y_stride as i32,
                ref_y_h as i32,
                blk_px,
                blk_py,
                mvx,
                mvy,
                8,
                &mut pred,
                8,
            );
        }
        let (plane, stride, px, py) = block_dst(pic, block_idx, mb_x, mb_y);
        if info.residual_present[block_idx] {
            let resid = info.residual_block(block_idx);
            for j in 0..8 {
                for i in 0..8 {
                    let s = pred[j * 8 + i] as i32 + resid[j * 8 + i] as i32;
                    plane[(py + j) * stride + (px + i)] = s.clamp(0, 255) as u8;
                }
            }
        } else {
            for j in 0..8 {
                for i in 0..8 {
                    plane[(py + j) * stride + (px + i)] = pred[j * 8 + i];
                }
            }
        }
    }

    // Chroma — single MVDCHR derived from the luma 4MV (F.2) or the 1MV path.
    let (cmx, cmy) = if advanced_prediction && motion.four_mv {
        chroma_mv_4mv(&motion.mvs4)
    } else {
        (
            luma_to_chroma_mv(motion.mv.0),
            luma_to_chroma_mv(motion.mv.1),
        )
    };

    let ref_c_h = reference.cb.len() / reference.c_stride;
    for (plane_idx, block_idx) in (4..6usize).enumerate() {
        let blk_px = (mb_x * 8) as i32;
        let blk_py = (mb_y * 8) as i32;
        let mut pred = [0u8; 64];
        let (ref_plane, ref_stride) = if plane_idx == 0 {
            (&reference.cb, reference.c_stride)
        } else {
            (&reference.cr, reference.c_stride)
        };
        predict_block(
            ref_plane,
            ref_stride,
            ref_stride as i32,
            ref_c_h as i32,
            blk_px,
            blk_py,
            cmx,
            cmy,
            8,
            &mut pred,
            8,
        );
        let (plane, stride, px, py) = block_dst(pic, block_idx, mb_x, mb_y);
        if info.residual_present[block_idx] {
            let resid = info.residual_block(block_idx);
            for j in 0..8 {
                for i in 0..8 {
                    let s = pred[j * 8 + i] as i32 + resid[j * 8 + i] as i32;
                    plane[(py + j) * stride + (px + i)] = s.clamp(0, 255) as u8;
                }
            }
        } else {
            for j in 0..8 {
                for i in 0..8 {
                    plane[(py + j) * stride + (px + i)] = pred[j * 8 + i];
                }
            }
        }
    }
}

/// §F.3 — build the overlapped-motion-compensated luma predictor for one 8×8
/// block. Weights from `OBMC_H0` / `OBMC_H1` / `OBMC_H2` are applied to three
/// separate half-pel predictions (current block's MV, plus two "remote"
/// neighbour MVs) and the combined result is rounded by `(+ 4) / 8`.
#[allow(clippy::too_many_arguments)]
fn obmc_luma_block(
    dst: &mut [u8; 64],
    reference: &IPicture,
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    motion: &MbMotion,
) {
    let ref_y_h = reference.y.len() / reference.y_stride;
    let ref_w = reference.y_stride as i32;
    let ref_h = ref_y_h as i32;

    let (sub_x, sub_y) = block_offset(block_idx);
    let blk_px = (mb_x * 16 + sub_x) as i32;
    let blk_py = (mb_y * 16 + sub_y) as i32;

    // Build three predictions: q (current MV), r (top/bottom remote MV), s
    // (left/right remote MV). Each is a full 8×8 block.
    let mv0 = motion.mvs4[block_idx];
    let mut q_pred = [0u8; 64];
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv0.0,
        mv0.1,
        8,
        &mut q_pred,
        8,
    );

    // "Top or bottom" remote MV per pixel column: in the upper 4 rows of the
    // block we use the MV of the block ABOVE; in the lower 4 rows we use the
    // MV of the block BELOW. Since the weighting matrix H1 is symmetric
    // top/bottom (§F.3), we compute two separate predictions (r_top, r_bot)
    // using the top-neighbour MV and bottom-neighbour MV respectively and
    // pick per row in the weighted sum.
    let mv_top = obmc_remote_mv_vertical(mv_grid, mb_x, mb_y, block_idx, mv0, VerticalSide::Top);
    let mv_bot = obmc_remote_mv_vertical(mv_grid, mb_x, mb_y, block_idx, mv0, VerticalSide::Bottom);
    let mut r_top_pred = [0u8; 64];
    let mut r_bot_pred = [0u8; 64];
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_top.0,
        mv_top.1,
        8,
        &mut r_top_pred,
        8,
    );
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_bot.0,
        mv_bot.1,
        8,
        &mut r_bot_pred,
        8,
    );

    // Left / right remote MV, likewise split into two predictions.
    let mv_left =
        obmc_remote_mv_horizontal(mv_grid, mb_x, mb_y, block_idx, mv0, HorizontalSide::Left);
    let mv_right =
        obmc_remote_mv_horizontal(mv_grid, mb_x, mb_y, block_idx, mv0, HorizontalSide::Right);
    let mut s_left_pred = [0u8; 64];
    let mut s_right_pred = [0u8; 64];
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_left.0,
        mv_left.1,
        8,
        &mut s_left_pred,
        8,
    );
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_right.0,
        mv_right.1,
        8,
        &mut s_right_pred,
        8,
    );

    for j in 0..8usize {
        for i in 0..8usize {
            let h0 = OBMC_H0[j][i] as i32;
            let h1 = OBMC_H1[j][i] as i32;
            let h2 = OBMC_H2[j][i] as i32;
            let q = q_pred[j * 8 + i] as i32;
            let r = if j < 4 {
                r_top_pred[j * 8 + i] as i32
            } else {
                r_bot_pred[j * 8 + i] as i32
            };
            let s = if i < 4 {
                s_left_pred[j * 8 + i] as i32
            } else {
                s_right_pred[j * 8 + i] as i32
            };
            let v = (q * h0 + r * h1 + s * h2 + 4) / 8;
            dst[j * 8 + i] = v.clamp(0, 255) as u8;
        }
    }
}

enum VerticalSide {
    Top,
    Bottom,
}

enum HorizontalSide {
    Left,
    Right,
}

/// Remote vertical-neighbour MV for OBMC per §F.3. For each 8×8 block `k` of
/// the current MB, pick the MV of the block immediately above (`Top`) or
/// below (`Bottom`) at the 8×8 block granularity.
///
/// Fall-back rules (§F.3):
/// * If the neighbour MB was not coded → remote MV = (0, 0).
/// * If the neighbour block was intra coded → remote MV = current block MV.
/// * If the neighbour block is outside the picture → remote MV = current block MV.
/// * For blocks 2 and 3 (bottom row of the MB) with `Bottom` side, the
///   §F.3 last-paragraph rule forces the remote MV to be the current
///   block's MV.
fn obmc_remote_mv_vertical(
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    cur_mv: (i32, i32),
    side: VerticalSide,
) -> (i32, i32) {
    // Block rows inside MB: block 0/1 on top row, 2/3 on bottom row.
    let in_top_row = block_idx < 2;

    // Spec rule: for bottom-of-MB blocks (2 and 3) and Bottom-side neighbour,
    // force remote MV = current MV.
    if !in_top_row && matches!(side, VerticalSide::Bottom) {
        return cur_mv;
    }
    // Symmetric rule for top-of-MB blocks with Top-side neighbour when it
    // would reach into the row ABOVE the current MB. We still look up the
    // actual neighbour there though (the spec only special-cases the
    // "neighbour in MB below" rule explicitly, but does also describe
    // substitution for any missing neighbour).

    match side {
        VerticalSide::Top => {
            if in_top_row {
                // Neighbour lives in MB (mb_x, mb_y-1). The block physically
                // adjacent to our current block (which is in the TOP row of
                // this MB) is the BOTTOM-row block of that MB with the same
                // column — i.e. block 2 for column 0, block 3 for column 1.
                if mb_y == 0 {
                    return cur_mv;
                }
                let nb = mv_grid.get(mb_x, mb_y - 1);
                neighbour_mv(nb, sibling_block_in_row(block_idx, false), cur_mv)
            } else {
                // Block 2 or 3 — top neighbour is block 0 or 1 of the SAME MB
                // (same column, top row).
                let cur_mb = mv_grid.get(mb_x, mb_y);
                cur_mb.mvs4[sibling_block_in_row(block_idx, true)]
            }
        }
        VerticalSide::Bottom => {
            // We return here for the in_top_row case only; bottom-row was
            // handled at the top of the function.
            debug_assert!(in_top_row);
            // Neighbour is block 2 or 3 of same MB — look in our own MB's
            // already-decoded entry.
            let cur_mb = mv_grid.get(mb_x, mb_y);
            cur_mb.mvs4[sibling_block_in_row(block_idx, false)]
        }
    }
}

/// Remote horizontal-neighbour MV for OBMC per §F.3.
fn obmc_remote_mv_horizontal(
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    cur_mv: (i32, i32),
    side: HorizontalSide,
) -> (i32, i32) {
    // Block cols inside MB: blocks 0/2 on left, 1/3 on right.
    let in_left_col = block_idx == 0 || block_idx == 2;

    match side {
        HorizontalSide::Left => {
            if in_left_col {
                // Current block is in LEFT column of this MB. Its left
                // neighbour physically is the RIGHT column of MB (mb_x-1),
                // same row — block 1 for top row, block 3 for bottom row.
                if mb_x == 0 {
                    return cur_mv;
                }
                let nb = mv_grid.get(mb_x - 1, mb_y);
                neighbour_mv(nb, sibling_block_in_col(block_idx, false), cur_mv)
            } else {
                // Blocks 1 or 3 — left neighbour is blocks 0 or 2 in same MB
                // (same row, left column).
                let cur_mb = mv_grid.get(mb_x, mb_y);
                cur_mb.mvs4[sibling_block_in_col(block_idx, true)]
            }
        }
        HorizontalSide::Right => {
            if in_left_col {
                // Blocks 0 or 2 — right neighbour is blocks 1 or 3 in same MB.
                let cur_mb = mv_grid.get(mb_x, mb_y);
                cur_mb.mvs4[sibling_block_in_col(block_idx, false)]
            } else {
                // Blocks 1 or 3 — right neighbour in MB (mb_x+1, mb_y), LEFT
                // column — block 0 for top row, block 2 for bottom row.
                if mb_x + 1 >= mv_grid.mb_w {
                    return cur_mv;
                }
                let nb = mv_grid.get(mb_x + 1, mb_y);
                neighbour_mv(nb, sibling_block_in_col(block_idx, true), cur_mv)
            }
        }
    }
}

/// Pick a neighbour block's MV from an `MbMotion`, applying the §F.3
/// fall-backs for "not coded" (→ zero) and "intra" (→ current MV).
fn neighbour_mv(nb: MbMotion, block_idx: usize, cur_mv: (i32, i32)) -> (i32, i32) {
    if !nb.coded {
        return (0, 0);
    }
    if nb.intra {
        return cur_mv;
    }
    nb.mvs4[block_idx]
}

/// For block `block_idx` in the current MB, return the sibling block index
/// in the other row of the same MB. If `top`, we want the block in the top
/// row (used for Top-side remote MVs when looking up the sibling in MB
/// above — where we want the bottom-row block of THAT MB, hence flipped
/// semantics handled at the caller).
///
/// The mapping is symmetric:
///   col-0, row-0 (block 0) ↔ col-0, row-1 (block 2)
///   col-1, row-0 (block 1) ↔ col-1, row-1 (block 3)
///
/// For a current block on the top row of THIS MB, the "Top" neighbour lives
/// in the MB above → we want THAT MB's bottom-row block in the same column,
/// i.e. block 2 for col-0 / block 3 for col-1. So `want_top = false` in the
/// above-MB indexing when our block is top-row.
fn sibling_block_in_row(block_idx: usize, want_top_of_that_mb: bool) -> usize {
    // Column = block_idx % 2 (0 or 1).
    let col = block_idx & 1;
    if want_top_of_that_mb {
        col // blocks 0 or 1 are top-row.
    } else {
        col + 2 // blocks 2 or 3 are bottom-row.
    }
}

/// Sibling block in the other COLUMN of the same MB (for left/right lookups).
/// If `want_left`, return the left-column sibling (0 or 2).
fn sibling_block_in_col(block_idx: usize, want_left_of_that_mb: bool) -> usize {
    // Row = block_idx / 2 (0 or 1).
    let row = block_idx >> 1;
    if want_left_of_that_mb {
        row * 2 // blocks 0 or 2 are left col.
    } else {
        row * 2 + 1 // blocks 1 or 3 are right col.
    }
}

/// (sub_x, sub_y) offset inside a 16×16 MB for each 8×8 luma block index.
fn block_offset(block_idx: usize) -> (usize, usize) {
    match block_idx {
        0 => (0, 0),
        1 => (8, 0),
        2 => (0, 8),
        3 => (8, 8),
        _ => unreachable!("luma block_idx must be 0..=3"),
    }
}

/// Per-MB output of the PB-frames P-half decode — same shape as
/// [`PMbInfo`] but also carries MODB / CBPB / MVDB data needed to
/// reconstruct the B-half later.
#[derive(Clone)]
pub struct PbMbInfo {
    pub p_info: PMbInfo,
    /// Decoded MODB (always present when COD = 0; absent for skipped MBs).
    pub modb: crate::pb::ModbDecoded,
    /// CBPB bits 1..=6 (left-to-right; MSB = block 1). Present only when
    /// MODB indicated CBPB.
    pub cbpb: u8,
    /// MVDB delta in luma half-pel units. Present only when MODB indicated
    /// MVDB; otherwise `(0, 0)`.
    pub mvdb: (i32, i32),
}

/// PB-frames variant of [`decode_p_mb_pass1`]. After MCBPC, reads MODB
/// (Table 11), then optional CBPB (6 bits) and MVDB (Table 14 differential
/// applied to the §G.4 forward predictor). The B-block residual coding is
/// not yet wired (round-14 scope keeps it absent on the encoder side); when
/// CBPB is non-zero on the wire we currently surface a specific
/// `Unsupported` to flag third-party encoder interop.
#[allow(clippy::too_many_arguments)]
pub fn decode_p_mb_pb(
    br: &mut BitReader<'_>,
    mb_x: usize,
    mb_y: usize,
    quant_in: u32,
    pic: &mut IPicture,
    reference: &IPicture,
    mv_grid: &mut MvGrid,
    umv: UmvMode,
) -> Result<(u32, PbMbInfo)> {
    // 1. COD.
    let cod = br.read_u1()?;
    if cod == 1 {
        // Skipped MB — no MODB per Table 10. MVs default to (0,0); pass-2
        // reconstruction is a pure copy. The B-half for a skipped MB
        // inherits MV = (0, 0) and runs the §G.4 / §G.5 path with that.
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        // Do the local reconstruction so `pic` matches what the
        // single-pass non-PB path would produce.
        apply_p_mb_reconstruction(
            mb_x,
            mb_y,
            pic,
            reference,
            mv_grid,
            &PMbInfo::empty_skipped(),
            false,
        );
        return Ok((
            quant_in,
            PbMbInfo {
                p_info: PMbInfo::empty_skipped(),
                modb: crate::pb::ModbDecoded {
                    cbpb_present: false,
                    mvdb_present: false,
                },
                cbpb: 0,
                mvdb: (0, 0),
            },
        ));
    }

    // 2. MCBPC inter (loop over stuffing).
    let mcbpc_v = loop {
        let v = vlc::decode(br, mcbpc::p_table())?;
        if v != mcbpc::INTER_STUFFING {
            break v;
        }
    };
    let (mb_type, cbpc) = mcbpc::decompose_inter(mcbpc_v);
    use mcbpc::PMbType;

    let is_4mv = matches!(mb_type, PMbType::Inter4MV | PMbType::Inter4MVQ);
    if is_4mv {
        return Err(Error::unsupported(
            "h263 PB-frames: Inter4MV not supported in this round (round-14 scope is 1MV)",
        ));
    }
    let is_intra = matches!(mb_type, PMbType::Intra | PMbType::IntraQ);
    let needs_dquant = matches!(
        mb_type,
        PMbType::InterQ | PMbType::IntraQ | PMbType::Inter4MVQ
    );

    // 3. MODB (PB-frames specific) — between MCBPC and CBPY (§5.3 Fig 10).
    let modb = crate::pb::decode_modb(br)?;

    // 4. CBPB if signalled by MODB.
    let cbpb = if modb.cbpb_present {
        br.read_u32(6)? as u8
    } else {
        0
    };

    // 5. CBPY.
    let cbpy_raw = vlc::decode(br, cbpy::table())?;
    let cbpy = if is_intra { cbpy_raw } else { cbpy_raw ^ 0xF };

    // 6. DQUANT.
    let mut quant = quant_in;
    if needs_dquant {
        const DQUANT_DELTA: [i32; 4] = [-1, -2, 1, 2];
        let d = br.read_u32(2)? as usize;
        let new_q = (quant as i32) + DQUANT_DELTA[d];
        quant = new_q.clamp(1, 31) as u32;
    }

    // 7. MVD (§5.3.7) — single-MV path only in this round.
    if !is_intra {
        let (px, py) = crate::motion::predict_mv(mv_grid, mb_x, mb_y);
        let (mvx, mvy) = decode_mv_pair(br, px, py, umv)?;
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((mvx, mvy), true, false));
    } else {
        // Intra in P with PB-frames: §G.2 says MVD is **also** present for
        // intra MBs (used only for the B-blocks). We must still read it.
        let (px, py) = crate::motion::predict_mv(mv_grid, mb_x, mb_y);
        let (mvx, mvy) = decode_mv_pair(br, px, py, umv)?;
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((mvx, mvy), true, true));
    }

    // 8. MVDB (§5.3.9) if signalled. We read each component as a Table 14
    //    magnitude + sign — this matches the encoder side's emission. The
    //    decoded value is the raw differential (no wrap, no sign-of-
    //    predictor cascade because the predictor is the §G.4 scaled vector,
    //    not a neighbour MV).
    let mvdb = if modb.mvdb_present {
        let dx = crate::motion::decode_mvd_pure_differential(br)?;
        let dy = crate::motion::decode_mvd_pure_differential(br)?;
        (dx, dy)
    } else {
        (0, 0)
    };

    // 9. Per-block P-half texture.
    let luma_coded = [
        (cbpy >> 3) & 1 != 0,
        (cbpy >> 2) & 1 != 0,
        (cbpy >> 1) & 1 != 0,
        cbpy & 1 != 0,
    ];
    let chroma_coded = [(cbpc >> 1) & 1 != 0, cbpc & 1 != 0];

    let p_info = if is_intra {
        for block_idx in 0..6usize {
            let coded = if block_idx < 4 {
                luma_coded[block_idx]
            } else {
                chroma_coded[block_idx - 4]
            };
            decode_one_intra_block_in_p(br, block_idx, coded, mb_x, mb_y, quant, pic)?;
        }
        PMbInfo {
            coded: true,
            intra: true,
            residual: vec![0i16; 6 * 64],
            residual_present: [false; 6],
            intra_done: true,
        }
    } else {
        let mut info = PMbInfo {
            coded: true,
            intra: false,
            residual: vec![0i16; 6 * 64],
            residual_present: [false; 6],
            intra_done: false,
        };
        for block_idx in 0..6usize {
            let coded = if block_idx < 4 {
                luma_coded[block_idx]
            } else {
                chroma_coded[block_idx - 4]
            };
            if !coded {
                continue;
            }
            let mut coeffs = [0i32; 64];
            decode_ac(br, &mut coeffs, 0, quant)?;
            let mut resid = [0i32; 64];
            crate::block::idct_signed(&mut coeffs, &mut resid);
            let dst = info.residual_block_mut(block_idx);
            for (i, &v) in resid.iter().enumerate() {
                dst[i] = v.clamp(-4096, 4095) as i16;
            }
            info.residual_present[block_idx] = true;
        }
        // Apply local reconstruction (no OBMC, single-pass mode).
        apply_p_mb_reconstruction(mb_x, mb_y, pic, reference, mv_grid, &info, false);
        info
    };

    // 10. B-block residual (CBPB-driven). Decode but do not yet apply — the
    //     caller stores it for §G.5 reconstruction after the P-half is fully
    //     done. Round-14 round-trip pairs with an encoder that always emits
    //     CBPB = 0, so this branch is exercised only by third-party streams.
    if cbpb != 0 {
        // We accept CBPB on the wire for spec-completeness; the residual
        // bits are read (so the bitstream stays in sync) but the values are
        // discarded. A future round will plumb them through to the §G.5
        // B-half reconstruction.
        for block_idx in 0..6usize {
            // CBPB block numbering per spec: utmost left bit ↔ block 1
            // (the first luma block). We numbered our blocks 0..=5; the
            // spec numbers them 1..=6, so block_idx 0 maps to bit 5 of
            // CBPB (MSB of the 6-bit field) and block_idx 5 maps to bit 0.
            let bit = (cbpb >> (5 - block_idx)) & 1 != 0;
            if bit {
                let mut coeffs = [0i32; 64];
                // B-blocks use BQUANT (§5.1.23) — for now we decode at the
                // same quantiser. The values are discarded.
                decode_ac(br, &mut coeffs, 0, quant)?;
            }
        }
    }

    Ok((
        quant,
        PbMbInfo {
            p_info,
            modb,
            cbpb,
            mvdb,
        },
    ))
}

/// Legacy one-pass decoder used when Annex F is not enabled. Kept for
/// backwards compatibility with existing callers / tests. Calls
/// `decode_p_mb_pass1` followed by `apply_p_mb_reconstruction` for the
/// current MB only — this is correct without OBMC (the reconstruction is
/// purely local), but would produce wrong output if AP were set. Callers
/// that need AP must drive the two-pass path via the decoder.
#[allow(clippy::too_many_arguments)]
pub fn decode_p_mb(
    br: &mut BitReader<'_>,
    mb_x: usize,
    mb_y: usize,
    quant_in: u32,
    pic: &mut IPicture,
    reference: &IPicture,
    mv_grid: &mut MvGrid,
    umv: UmvMode,
) -> Result<u32> {
    let (quant, info) =
        decode_p_mb_pass1(br, mb_x, mb_y, quant_in, pic, mv_grid, umv, false, false)?;
    apply_p_mb_reconstruction(mb_x, mb_y, pic, reference, mv_grid, &info, false);
    Ok(quant)
}

/// Decode a pair `(mvx, mvy)` for a single MVD codeword, dispatching over
/// [`UmvMode`]. Under PLUSPTYPE+UMV, the MVD-pair start-code-emulation
/// stuffing bit (§D.2 last paragraph) is consumed after both components
/// have been decoded and the differentials happen to be `(+1, +1)` halfpel
/// (i.e. six consecutive zero bits in the code stream).
fn decode_mv_pair(br: &mut BitReader<'_>, px: i32, py: i32, umv: UmvMode) -> Result<(i32, i32)> {
    match umv {
        UmvMode::Off => {
            let mvx = decode_mv_component_umv(br, px, false)?;
            let mvy = decode_mv_component_umv(br, py, false)?;
            Ok((mvx, mvy))
        }
        UmvMode::BaselinePtype => {
            let mvx = decode_mv_component_umv(br, px, true)?;
            let mvy = decode_mv_component_umv(br, py, true)?;
            Ok((mvx, mvy))
        }
        UmvMode::PlusPtype { h_limit, v_limit } => {
            // Table D.3 differentials are decoded directly (no wrap); the
            // reconstructed vector is `predictor + diff`. We re-read the
            // per-component differential to pass the same value into the
            // SCE stuffing check below — component helper returns the
            // reconstructed MV but not the diff, so inline the decode.
            let diff_x = crate::motion::decode_mvd_table_d3(br)?;
            let diff_y = crate::motion::decode_mvd_table_d3(br)?;
            consume_mvd_pair_sce_bit(br, diff_x, diff_y)?;
            let mvx = px + diff_x;
            let mvy = py + diff_y;
            if let Some((lo, hi)) = h_limit {
                if mvx < lo || mvx > hi {
                    return Err(Error::invalid(format!(
                        "h263 Annex D PLUSPTYPE: horizontal MV {mvx} halfpel out of range [{lo}, {hi}]"
                    )));
                }
            }
            if let Some((lo, hi)) = v_limit {
                if mvy < lo || mvy > hi {
                    return Err(Error::invalid(format!(
                        "h263 Annex D PLUSPTYPE: vertical MV {mvy} halfpel out of range [{lo}, {hi}]"
                    )));
                }
            }
            Ok((mvx, mvy))
        }
    }
}

/// Intra block decode when the MB is an embedded intra inside a P-picture.
/// Identical to `decode_one_intra_block` (I-path) — factored as a separate
/// function so the intra-in-P caller doesn't depend on the I-path's private
/// helper name.
fn decode_one_intra_block_in_p(
    br: &mut BitReader<'_>,
    block_idx: usize,
    has_ac: bool,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    pic: &mut IPicture,
) -> Result<()> {
    let dc = decode_intradc(br)?;
    let mut coeffs = [0i32; 64];
    coeffs[0] = dc;

    if has_ac {
        decode_ac(br, &mut coeffs, 1, quant)?;
    }
    coeffs[0] = coeffs[0].clamp(-2048, 2047);

    let mut out = [0u8; 64];
    idct_and_clip(&mut coeffs, &mut out);

    write_block_to_picture(pic, block_idx, mb_x, mb_y, &out);
    Ok(())
}
