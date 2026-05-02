//! H.263 Annex G — PB-frames mode (round 14).
//!
//! A "PB-frame" is two pictures coded as one transmission unit: a P-picture
//! (predicted from the previous decoded P-picture) plus a B-picture
//! (predicted bidirectionally from both the previous P and the new P).
//!
//! Picture-header additions (parsed in [`crate::picture`]):
//!   * PTYPE bit 13 = 1 (PBFR).
//!   * `TRB` (3 bits, §5.1.22) — temporal reference of the B-block relative
//!     to the previous P (TRB-th non-transmitted picture, plus 1 — i.e. TRB
//!     ≥ 1 in any well-formed stream that uses this mode).
//!   * `DBQUANT` (2 bits, §5.1.23) — quantiser-offset code for the B-block,
//!     mapped through Table 6.
//!
//! MB-layer additions (per §5.3.3 / §5.3.4 / §5.3.9, between CBPY and the
//! Block Data):
//!   * `MODB` (1-3 bit VLC, Table 11) — selects whether `CBPB` and / or
//!     `MVDB` are present.
//!   * `CBPB` (6 bits, §5.3.4) — coded-block pattern for the six B-blocks
//!     (Y0..Y3, Cb, Cr). MSB = block 1.
//!   * `MVDB` (variable, §5.3.9) — differential MV that perturbs the §G.4
//!     scaled forward MV, when the B-block's MC needs a non-default vector.
//!
//! §G.4 derives forward `MVF` and backward `MVB` per block (in half-pel
//! units) from the co-located P-MB's MV `MV` plus an optional `MVD` from
//! `MVDB`:
//!
//! ```text
//! MVF = (TRB * MV) / TRD + MVD
//! MVB = ((TRB - TRD) * MV) / TRD              (when MVD == 0)
//! MVB = MVF - MV                              (when MVD != 0)
//! ```
//!
//! `TRD` is the increment of the P-picture temporal reference since the
//! prior P-picture (it is **not** the same as TRB — TRB ≤ TRD because TRB
//! counts the position of the B inside the gap). The `/` is integer
//! truncation toward zero.
//!
//! §G.5 reconstructs the B-block by averaging a forward prediction (from the
//! prior P-picture using `MVF`) with a backward prediction (from the freshly
//! reconstructed P-block using `MVB`). The bidirectional region is the set of
//! pixels for which `MVB` would actually point inside the new P-MB; outside
//! that region only the forward prediction is used.
//!
//! Round-14 scope (encoder): emit MODB = `0` (no CBPB, no MVDB) per MB and
//! DBQUANT = `00` at the picture level. The B-block then has zero MVDB +
//! zero residual — a pure §G.4 / §G.5 bidirectional MC predictor — which
//! still validates every header / body field on the wire and the §G.4 / §G.5
//! derivation. The decoder accepts the full MODB / CBPB / MVDB syntax (so
//! third-party encoders can interoperate), and applies the bidirectional MC
//! into a separate "B-picture" surface that is emitted in display order
//! before the P.

use crate::mb::IPicture;
use crate::motion::{luma_to_chroma_mv, MbMotion, MvGrid};

/// Compute `BQUANT` from `QUANT` and DBQUANT per Table 6/H.263.
/// Result is clipped to `[1, 31]` per §5.1.23.
pub fn bquant_from_quant(quant: u8, dbquant: u8) -> u8 {
    let factor = match dbquant & 0x3 {
        0b00 => 5,
        0b01 => 6,
        0b10 => 7,
        0b11 => 8,
        _ => unreachable!(),
    };
    let raw = (factor * (quant as u32)) / 4;
    raw.clamp(1, 31) as u8
}

/// Result of [`derive_b_block_mvs`] — forward + backward MV (in luma half-pel
/// units) for one 8×8 B-block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BBlockMvs {
    pub mvf: (i32, i32),
    pub mvb: (i32, i32),
}

/// §G.4 — derive `MVF` / `MVB` for one B-block.
///
/// `mv_p` is the co-located P-block's MV (in 1MV mode all four blocks share
/// the same MV; in 4MV mode each B-block uses the matching luma block's MV).
/// `mvd_b` is the §5.3.9 / §G.4 delta from MVDB (zero when MVDB is absent).
/// `trb` is the picture-header §5.1.22 value; `trd` is computed from the
/// per-frame temporal-reference increments (always ≥ TRB on a well-formed
/// stream).
pub fn derive_b_block_mvs(mv_p: (i32, i32), mvd_b: (i32, i32), trb: i32, trd: i32) -> BBlockMvs {
    let trd = trd.max(1);
    let trb = trb.max(0);
    // MVF = (TRB * MV) / TRD + MVD     (truncating division)
    let mvf_x = trunc_div(trb * mv_p.0, trd) + mvd_b.0;
    let mvf_y = trunc_div(trb * mv_p.1, trd) + mvd_b.1;
    let mvb = if mvd_b == (0, 0) {
        // MVB = ((TRB - TRD) * MV) / TRD
        let mvb_x = trunc_div((trb - trd) * mv_p.0, trd);
        let mvb_y = trunc_div((trb - trd) * mv_p.1, trd);
        (mvb_x, mvb_y)
    } else {
        // MVB = MVF - MV
        (mvf_x - mv_p.0, mvf_y - mv_p.1)
    };
    BBlockMvs {
        mvf: (mvf_x, mvf_y),
        mvb,
    }
}

/// Integer division by truncation toward zero (Rust's `/` operator already
/// truncates toward zero for `i32`, matching the spec's "/" for negative
/// operands too).
#[inline]
fn trunc_div(num: i32, den: i32) -> i32 {
    debug_assert!(den != 0);
    num / den
}

/// §G.5 — reconstruct one 8×8 B-block.
///
/// `block_idx` selects the B-block within the MB:
///   * 0..=3 → luma 8×8 (top-left, top-right, bottom-left, bottom-right).
///   * 4 → Cb 8×8.
///   * 5 → Cr 8×8.
///
/// `forward_ref` is the previous decoded P-picture (the "prev P"); `p_recon`
/// is the freshly reconstructed P-half of this PB-frame. `mvs` is the
/// per-block forward + backward MV pair from [`derive_b_block_mvs`].
///
/// `residual` is added to the predictor sample-by-sample and the result is
/// clipped to `[0, 255]`. Pass an all-zero `&[0i16; 64]` when the B-block is
/// not coded (CBPB bit 0 / MODB indicates no CBPB).
///
/// Per §G.5 the prediction has two regions:
///   * For pixels where `MVB` (mapped to the destination 8×8 block coords)
///     points **inside** the P-MB area `(0..=15)` of the freshly reconstructed
///     P-picture, use the average of the forward prediction (from
///     `forward_ref` using `MVF`) and the backward prediction (from `p_recon`
///     using `MVB` relative to the P-MB origin).
///   * For all other pixels, use the forward prediction only.
///
/// The destination is written into `dst` (8×8, row-major).
#[allow(clippy::too_many_arguments)]
pub fn reconstruct_b_block(
    dst: &mut [u8; 64],
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    forward_ref: &IPicture,
    p_recon: &IPicture,
    mvs: BBlockMvs,
    residual: &[i16; 64],
) {
    let mut pred = [0i16; 64];
    predict_b_block(&mut pred, block_idx, mb_x, mb_y, forward_ref, p_recon, mvs);
    for k in 0..64 {
        let p = pred[k] as i32;
        let r = residual[k] as i32;
        dst[k] = (p + r).clamp(0, 255) as u8;
    }
}

/// §G.5 — compute the **prediction-only** (forward + backward bidirectional
/// average inside the §G.5 region, forward-only outside) for one B-block.
///
/// This is the building block for both:
///   * the decoder's [`reconstruct_b_block`] (which adds the parsed residual);
///   * the encoder's B-residual computation (subtracts this prediction from
///     the source pels to get the residual that will be DCT/quantised at
///     BQUANT and emitted under CBPB).
///
/// Output is signed (i16) so the encoder can compute `source - prediction`
/// without saturating at 0 or 255.
pub fn predict_b_block(
    dst: &mut [i16; 64],
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    forward_ref: &IPicture,
    p_recon: &IPicture,
    mvs: BBlockMvs,
) {
    debug_assert!(block_idx < 6);
    let is_chroma = block_idx >= 4;

    // Source plane geometry.
    let (fwd_plane, fwd_stride, p_plane, p_stride, blk_px, blk_py, n, mb_origin_px, mb_origin_py) =
        if is_chroma {
            // Chroma is 4:2:0 — 8×8 block, one per MB. The MB occupies the
            // 8×8 chroma region at (mb_x*8, mb_y*8) of the chroma plane.
            let cw = forward_ref.c_stride;
            let (fp, pp) = match block_idx {
                4 => (&forward_ref.cb, &p_recon.cb),
                5 => (&forward_ref.cr, &p_recon.cr),
                _ => unreachable!(),
            };
            (
                fp.as_slice(),
                cw,
                pp.as_slice(),
                p_recon.c_stride,
                (mb_x * 8) as i32,
                (mb_y * 8) as i32,
                8,
                (mb_x * 8) as i32,
                (mb_y * 8) as i32,
            )
        } else {
            // Luma 8×8 block at the standard sub-MB offset.
            let (sub_x, sub_y) = match block_idx {
                0 => (0, 0),
                1 => (8, 0),
                2 => (0, 8),
                3 => (8, 8),
                _ => unreachable!(),
            };
            (
                forward_ref.y.as_slice(),
                forward_ref.y_stride,
                p_recon.y.as_slice(),
                p_recon.y_stride,
                (mb_x * 16 + sub_x) as i32,
                (mb_y * 16 + sub_y) as i32,
                8,
                (mb_x * 16) as i32,
                (mb_y * 16) as i32,
            )
        };

    let fwd_w = fwd_stride as i32;
    let fwd_h = (forward_ref.y.len() / forward_ref.y_stride.max(1)) as i32;
    let fwd_h = if is_chroma {
        (forward_ref.cb.len() / forward_ref.c_stride.max(1)) as i32
    } else {
        fwd_h
    };
    let p_w = p_stride as i32;
    let p_h = if is_chroma {
        (p_recon.cb.len() / p_recon.c_stride.max(1)) as i32
    } else {
        (p_recon.y.len() / p_recon.y_stride.max(1)) as i32
    };

    // Forward prediction: full 8×8 from `forward_ref` using `mvs.mvf`.
    let mut fwd = [0u8; 64];
    crate::interp::predict_block(
        fwd_plane, fwd_stride, fwd_w, fwd_h, blk_px, blk_py, mvs.mvf.0, mvs.mvf.1, n, &mut fwd,
        n as usize,
    );

    // Backward prediction: from `p_recon` using `mvs.mvb`. The backward
    // prediction is anchored at the **same destination block position**, so
    // it is sampled from `(blk_px + mvb.x/2, blk_py + mvb.y/2)` of `p_recon`.
    let mut bwd = [0u8; 64];
    crate::interp::predict_block(
        p_plane, p_stride, p_w, p_h, blk_px, blk_py, mvs.mvb.0, mvs.mvb.1, n, &mut bwd, n as usize,
    );

    // §G.5 region selection: for each output pixel, decide whether the
    // backward MV maps it into the freshly-reconstructed P-MB region. When
    // it does, average forward + backward (truncating). When it doesn't, use
    // forward only.
    //
    // The spec's procedure (in C-like form) defines the bidirectional region
    // by `[max(0, (-m+1)/2 - sub), min(7, 15 - (m+1)/2 - sub)]` per axis,
    // where `m` is the MV component and `sub` is the block offset within the
    // MB (0 or 8). For chroma the MB is 8×8 so `sub == 0` and the same range
    // formula applies with the 7/15 constants for chroma.
    let (sub_x, sub_y, mb_size): (i32, i32, i32) = if is_chroma {
        (0, 0, 7)
    } else {
        let (sx, sy) = match block_idx {
            0 => (0i32, 0i32),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        (sx, sy, 15)
    };
    let mh = mvs.mvb.0;
    let mv = mvs.mvb.1;
    let i_lo = (((-mh + 1) / 2) - sub_x).max(0);
    let i_hi = (mb_size - ((mh + 1) / 2) - sub_x).min(7);
    let j_lo = (((-mv + 1) / 2) - sub_y).max(0);
    let j_hi = (mb_size - ((mv + 1) / 2) - sub_y).min(7);

    // The above ranges are in **block-local** coordinates (0..=7); compare
    // each pixel's block-local index against them.
    let _ = (mb_origin_px, mb_origin_py); // kept for future-proofing (chroma uses block origin).

    for j in 0..8usize {
        for i in 0..8usize {
            let f = fwd[j * 8 + i] as i32;
            let inside = (i as i32) >= i_lo
                && (i as i32) <= i_hi
                && (j as i32) >= j_lo
                && (j as i32) <= j_hi;
            let pred = if inside {
                let b = bwd[j * 8 + i] as i32;
                (f + b) / 2
            } else {
                f
            };
            dst[j * 8 + i] = pred as i16;
        }
    }
}

/// Per-MB derived B-block MVs (4 luma + chroma derived from the 4 luma sums
/// per §G.4 / Table F.1 chroma rounding).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BMbMvs {
    /// Per-luma-block forward + backward MV.
    pub luma: [BBlockMvs; 4],
    /// Chroma forward + backward MV (Cb and Cr share the same MV).
    pub chroma: BBlockMvs,
}

/// Derive every B-MV needed for one MB. `motion` is the co-located P-MB's
/// motion (1MV or 4MV), `mvd_b` is the per-MB MVDB delta (zero when MVDB is
/// absent — the same delta is applied to all four luma B-blocks per §G.4).
/// `trb` and `trd` are picture-level constants.
pub fn derive_b_mb_mvs(motion: &MbMotion, mvd_b: (i32, i32), trb: i32, trd: i32) -> BMbMvs {
    let mut luma = [BBlockMvs {
        mvf: (0, 0),
        mvb: (0, 0),
    }; 4];
    for b in 0..4 {
        luma[b] = derive_b_block_mvs(motion.mvs4[b], mvd_b, trb, trd);
    }
    // Chroma per §G.4 last paragraph: sum the 4 luma MVs (forward and
    // backward separately) divide by 8, round per Table F.1.
    let sum_fwd = (
        luma[0].mvf.0 + luma[1].mvf.0 + luma[2].mvf.0 + luma[3].mvf.0,
        luma[0].mvf.1 + luma[1].mvf.1 + luma[2].mvf.1 + luma[3].mvf.1,
    );
    let sum_bwd = (
        luma[0].mvb.0 + luma[1].mvb.0 + luma[2].mvb.0 + luma[3].mvb.0,
        luma[0].mvb.1 + luma[1].mvb.1 + luma[2].mvb.1 + luma[3].mvb.1,
    );
    let chroma = BBlockMvs {
        mvf: (
            chroma_round_table_f1(sum_fwd.0),
            chroma_round_table_f1(sum_fwd.1),
        ),
        mvb: (
            chroma_round_table_f1(sum_bwd.0),
            chroma_round_table_f1(sum_bwd.1),
        ),
    };
    BMbMvs { luma, chroma }
}

/// Table F.1 chroma rounding: divide a sum-of-4-luma-MVs by 8 and round the
/// resulting sixteenth-pel value toward the nearest half-pel position. We
/// reuse [`luma_to_chroma_mv`] for the 1MV path and apply the explicit
/// Table F.1 mapping here for the 4MV path. Because both paths converge on
/// `round(sum/8) -> half-pel`, we follow the simple `(sum + 4) >> 3` form
/// that matches §F.2 NOTE 1's "rounded value" + Table F.1's "0/0/0/0/+/+/+/+
/// per remainder mod 8" intent.
#[inline]
fn chroma_round_table_f1(sum: i32) -> i32 {
    // Sum is in luma half-pel units. Dividing by 8 gives a chroma sixteenth-
    // pel value; we then round to the nearest chroma half-pel.
    //
    // Table F.1 ladder (sum mod 16 → chroma half-pel offset relative to the
    // truncated result): 0→0, ±1→0, ±2→±1, ±3→±1, ±4→±1, ±5→±1, ±6→±1, ±7→±1.
    // i.e. anything not within ±1 of a multiple-of-8 rounds outward to the
    // adjacent half-pel.
    //
    // Concretely: q = sum / 8 (truncate toward zero); r = sum - 8*q in
    // `[-7, +7]`. If `|r| <= 1` we keep `q` (round toward nearest); otherwise
    // we step `q` by `sign(r)`. The final chroma half-pel value is `q`.
    let q = sum / 8;
    let r = sum - q * 8;
    if r.abs() <= 1 {
        q
    } else if r > 0 {
        q + 1
    } else {
        q - 1
    }
}

/// Build the **B-half** of a PB-frame.
///
/// `mb_motions` indexes co-located P-MB motion (1MV or 4MV). `mb_mvdbs` is
/// the per-MB MVDB delta — pass `(0, 0)` for MBs whose MODB code didn't
/// carry MVDB.
///
/// `b_residuals` is the per-MB-per-block B-residual buffer (laid out
/// `mb * 6 * 64 + block * 64`). MBs with no CBPB use all-zero residuals.
///
/// `width` / `height` are the picture dimensions.
#[allow(clippy::too_many_arguments)]
pub fn reconstruct_b_picture(
    width: usize,
    height: usize,
    forward_ref: &IPicture,
    p_recon: &IPicture,
    mb_motions: &[MbMotion],
    mb_mvdbs: &[(i32, i32)],
    b_residuals: &[i16],
    trb: i32,
    trd: i32,
) -> IPicture {
    let mut b_pic = IPicture::new(width, height);
    let mb_w = b_pic.mb_width;
    let mb_h = b_pic.mb_height;
    debug_assert_eq!(mb_motions.len(), mb_w * mb_h);
    debug_assert_eq!(mb_mvdbs.len(), mb_w * mb_h);
    debug_assert_eq!(b_residuals.len(), mb_w * mb_h * 6 * 64);

    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let mb_idx = mb_y * mb_w + mb_x;
            let motion = mb_motions[mb_idx];
            // §G.2 — when the P-MB is INTRA, the spec allows it to still carry
            // MVDB for the B-half (the §G.4 vector formula uses MV which must
            // then be zero for an intra MB). We follow the spec's note: for
            // intra MBs, MV defaults to (0,0) but MVDB still applies so the
            // B-half is forward-predicted with a small delta.
            let p_mv = if motion.intra {
                MbMotion::mv1((0, 0), true, false)
            } else {
                motion
            };
            let mvd_b = mb_mvdbs[mb_idx];
            let mvs = derive_b_mb_mvs(&p_mv, mvd_b, trb, trd);

            // For each block, reconstruct into the picture. Use a scratch
            // residual buffer pulled from `b_residuals`.
            for b in 0..4usize {
                let mut dst = [0u8; 64];
                let resid_slice = &b_residuals[(mb_idx * 6 + b) * 64..(mb_idx * 6 + b + 1) * 64];
                let resid_arr: [i16; 64] = resid_slice.try_into().expect("slice len 64");
                reconstruct_b_block(
                    &mut dst,
                    b,
                    mb_x,
                    mb_y,
                    forward_ref,
                    p_recon,
                    mvs.luma[b],
                    &resid_arr,
                );
                // Copy into b_pic.
                let (sub_x, sub_y) = match b {
                    0 => (0, 0),
                    1 => (8, 0),
                    2 => (0, 8),
                    3 => (8, 8),
                    _ => unreachable!(),
                };
                let base_x = mb_x * 16 + sub_x;
                let base_y = mb_y * 16 + sub_y;
                for j in 0..8 {
                    let off = (base_y + j) * b_pic.y_stride + base_x;
                    b_pic.y[off..off + 8].copy_from_slice(&dst[j * 8..j * 8 + 8]);
                }
            }
            for ci in 0..2usize {
                let b = 4 + ci;
                let mut dst = [0u8; 64];
                let resid_slice = &b_residuals[(mb_idx * 6 + b) * 64..(mb_idx * 6 + b + 1) * 64];
                let resid_arr: [i16; 64] = resid_slice.try_into().expect("slice len 64");
                reconstruct_b_block(
                    &mut dst,
                    b,
                    mb_x,
                    mb_y,
                    forward_ref,
                    p_recon,
                    mvs.chroma,
                    &resid_arr,
                );
                let plane = if ci == 0 {
                    &mut b_pic.cb
                } else {
                    &mut b_pic.cr
                };
                let stride = b_pic.c_stride;
                let base_x = mb_x * 8;
                let base_y = mb_y * 8;
                for j in 0..8 {
                    let off = (base_y + j) * stride + base_x;
                    plane[off..off + 8].copy_from_slice(&dst[j * 8..j * 8 + 8]);
                }
            }
        }
    }
    // Suppress dead-code warnings for the helper kept for future paths.
    let _ = luma_to_chroma_mv as fn(i32) -> i32;
    let _ = MvGrid::new(mb_w, mb_h); // unused but keeps the import alive for other callers
    b_pic
}

/// Annex M version of [`reconstruct_b_picture`]. Differs from the Annex G
/// path on three points:
///
/// 1. Per-MB the B-mode is read from `mb_bmodes` and the predictor follows
///    §M.2 (bidir → §G.5 — same as Annex G; forward → §M.2.2 fwd MV from
///    `mb_mvfs`; backward → §M.2.3 PREC pels).
/// 2. The MVDB slot in Annex M carries a *forward MV* (luma half-pel) rather
///    than a perturbing delta — the caller passes that vector unchanged in
///    `mb_mvfs` and the bidir / backward paths ignore it.
/// 3. Chroma vectors for the forward path use the §M.2.2 simple predictor
///    (same as the §G.4 chroma rounding via Table F.1, applied to the per-MB
///    forward MV scaled to chroma sixteenths).
///
/// `b_residuals` layout matches Annex G: per-MB-per-block i16[64], 6 blocks
/// per MB. MBs whose CBPB is zero use all-zero residuals.
#[allow(clippy::too_many_arguments)]
pub fn reconstruct_b_picture_m(
    width: usize,
    height: usize,
    forward_ref: &IPicture,
    p_recon: &IPicture,
    mb_motions: &[MbMotion],
    mb_mvfs: &[(i32, i32)],
    mb_bmodes: &[BMode],
    b_residuals: &[i16],
    trb: i32,
    trd: i32,
) -> IPicture {
    let mut b_pic = IPicture::new(width, height);
    let mb_w = b_pic.mb_width;
    let mb_h = b_pic.mb_height;
    debug_assert_eq!(mb_motions.len(), mb_w * mb_h);
    debug_assert_eq!(mb_mvfs.len(), mb_w * mb_h);
    debug_assert_eq!(mb_bmodes.len(), mb_w * mb_h);
    debug_assert_eq!(b_residuals.len(), mb_w * mb_h * 6 * 64);

    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let mb_idx = mb_y * mb_w + mb_x;
            let bmode = mb_bmodes[mb_idx];
            let motion = mb_motions[mb_idx];
            let p_mv = if motion.intra {
                MbMotion::mv1((0, 0), true, false)
            } else {
                motion
            };

            // For each block, build a signed predictor matching §M.2 dispatch.
            for b in 0..6usize {
                let mut pred = [0i16; 64];
                match bmode {
                    BMode::Bidirectional => {
                        // §M.2.1 → §G.5 path with MVDB = 0 (Annex M's MVDB is
                        // a forward-mode MV, not a §G.4 delta — bidirectional
                        // mode in Annex M is "the prediction defined in
                        // Annex G when MVD = 0").
                        let mvs = if b < 4 {
                            derive_b_block_mvs(p_mv.mvs4[b], (0, 0), trb, trd)
                        } else {
                            // Chroma uses the chroma MV — derived from the
                            // Annex G chroma path inside `derive_b_mb_mvs`.
                            let m = derive_b_mb_mvs(&p_mv, (0, 0), trb, trd);
                            m.chroma
                        };
                        predict_b_block(&mut pred, b, mb_x, mb_y, forward_ref, p_recon, mvs);
                    }
                    BMode::Forward => {
                        // §M.2.2 — single 16×16 forward MV from `mb_mvfs`.
                        // Chroma vectors use the Table F.1 chroma rounding
                        // applied to the per-MB forward MV (same as the §F.2
                        // chroma derivation but with one MV instead of four).
                        let mvf = mb_mvfs[mb_idx];
                        let block_mvf = if b < 4 {
                            mvf
                        } else {
                            // 1MV chroma derivation per §F.2 / Table F.1 NOTE 1:
                            // chroma half-pel = round(luma_halfpel / 2).
                            (
                                crate::motion::luma_to_chroma_mv(mvf.0),
                                crate::motion::luma_to_chroma_mv(mvf.1),
                            )
                        };
                        predict_b_block_forward(&mut pred, b, mb_x, mb_y, forward_ref, block_mvf);
                    }
                    BMode::Backward => {
                        // §M.2.3 — PREC = freshly reconstructed P-MB pels.
                        predict_b_block_backward(&mut pred, b, mb_x, mb_y, p_recon);
                    }
                }

                // Add the per-block B-residual (zero outside CBPB), clip to
                // [0, 255], and write into `b_pic` at this block's
                // destination position.
                let resid_slice = &b_residuals[(mb_idx * 6 + b) * 64..(mb_idx * 6 + b + 1) * 64];
                let mut dst = [0u8; 64];
                for k in 0..64 {
                    let v = pred[k] as i32 + resid_slice[k] as i32;
                    dst[k] = v.clamp(0, 255) as u8;
                }
                write_block_into_picture(&mut b_pic, b, mb_x, mb_y, &dst);
            }
        }
    }
    b_pic
}

/// Helper: copy an 8×8 reconstructed block into `pic` at the destination
/// position implied by `(block_idx, mb_x, mb_y)`.
fn write_block_into_picture(
    pic: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    block: &[u8; 64],
) {
    let is_chroma = block_idx >= 4;
    if is_chroma {
        let plane = match block_idx {
            4 => &mut pic.cb,
            5 => &mut pic.cr,
            _ => unreachable!(),
        };
        let stride = pic.c_stride;
        let bx = mb_x * 8;
        let by = mb_y * 8;
        for j in 0..8 {
            let off = (by + j) * stride + bx;
            plane[off..off + 8].copy_from_slice(&block[j * 8..j * 8 + 8]);
        }
    } else {
        let (sub_x, sub_y) = match block_idx {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let bx = mb_x * 16 + sub_x;
        let by = mb_y * 16 + sub_y;
        let stride = pic.y_stride;
        for j in 0..8 {
            let off = (by + j) * stride + bx;
            pic.y[off..off + 8].copy_from_slice(&block[j * 8..j * 8 + 8]);
        }
    }
}

// ---------------------------------------------------------------------------
// MODB VLC (Table 11/H.263 — Annex G; Table M.1/H.263 — Annex M)
// ---------------------------------------------------------------------------

/// Per-MB BPB-block prediction mode for the **Annex M** (Improved PB-frames)
/// path. Annex G's MODB table only encodes one prediction shape (always
/// bidirectional, optionally tweaked by an MVDB delta); Annex M widens that
/// to three shapes per §M.2:
///
/// * [`BMode::Bidirectional`] — same predictor as Annex G (forward from prior
///   P, backward from new P, averaged inside the §G.5 region). The optional
///   forward MV delta is **not** present (Annex G's MVDB-as-delta semantic
///   does not survive into Annex M; instead Annex M reuses the MVDB slot for
///   a forward MV in the forward-prediction mode).
/// * [`BMode::Forward`] — single 16×16 forward MV from MVDB used to fetch a
///   forward predictor from the prior P. No backward fetch, no §G.5 region
///   averaging. This is the win on a B macroblock where the new P-half is a
///   poor match (e.g. occluded / new content not present in the prior P
///   either, so bidirectional averaging only adds noise).
/// * [`BMode::Backward`] — predictor is the freshly reconstructed co-located
///   P-MB pels (`PREC`). No MV data on the wire. This is the win when the
///   B-frame's content is essentially the new P (small camera pan, B sits
///   close to P in time and the prior-P MC predictor is worse than copying
///   the new P).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BMode {
    Bidirectional,
    Forward,
    Backward,
}

/// Decoded MODB code — pair of `(cbpb_present, mvdb_present)`.
///
/// For Annex M this is augmented with the [`BMode`] selection (see
/// [`ModbMDecoded`]); the legacy Annex G path keeps the simpler shape
/// because its MODB table always implies `BMode::Bidirectional`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModbDecoded {
    pub cbpb_present: bool,
    pub mvdb_present: bool,
}

/// Decode one MODB code from the bitstream.
///
/// Table 11/H.263 — three codes:
/// * `0`  → no CBPB, no MVDB.
/// * `10` → no CBPB, MVDB present.
/// * `11` → CBPB present, MVDB present.
pub fn decode_modb(
    br: &mut oxideav_core::bits::BitReader<'_>,
) -> oxideav_core::Result<ModbDecoded> {
    let b0 = br.read_u1()?;
    if b0 == 0 {
        return Ok(ModbDecoded {
            cbpb_present: false,
            mvdb_present: false,
        });
    }
    let b1 = br.read_u1()?;
    Ok(ModbDecoded {
        cbpb_present: b1 == 1,
        mvdb_present: true,
    })
}

/// Emit the MODB code matching `(cbpb_present, mvdb_present)`.
///
/// Note: `(true, false)` is **not** representable in Table 11 — CBPB without
/// MVDB has no codeword. Encoders that want to send CBPB must also send
/// MVDB (mapping to `11`). This function maps `(true, false)` → `11` to
/// preserve the decoder-side invariant.
pub fn encode_modb(bw: &mut oxideav_core::bits::BitWriter, cbpb_present: bool, mvdb_present: bool) {
    let _ = mvdb_present; // see note above
    if !cbpb_present && !mvdb_present {
        bw.write_bits(0, 1); // "0"
    } else if cbpb_present {
        bw.write_bits(0b11, 2); // "11"
    } else {
        bw.write_bits(0b10, 2); // "10"
    }
}

/// Decoded MODB code (Annex M flavour) — selection of B-mode plus the
/// presence flags for CBPB and MVDB. See [`decode_modb_m`] / [`encode_modb_m`].
///
/// Per Table M.1/H.263 the (mode, cbpb, mvdb) triple takes one of six values:
///
/// | Index | CBPB | MVDB | Code   | Mode          |
/// |-------|------|------|--------|---------------|
/// |   0   |  -   |  -   | `0`    | Bidirectional |
/// |   1   |  x   |  -   | `10`   | Bidirectional |
/// |   2   |  -   |  x   | `110`  | Forward       |
/// |   3   |  x   |  x   | `1110` | Forward       |
/// |   4   |  -   |  -   | `11110`| Backward      |
/// |   5   |  x   |  -   | `11111`| Backward      |
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModbMDecoded {
    pub mode: BMode,
    pub cbpb_present: bool,
    pub mvdb_present: bool,
}

/// Decode one MODB code from the bitstream using the **Annex M** Table M.1
/// VLC (Improved PB-frames). The reader consumes 1..=5 bits.
pub fn decode_modb_m(
    br: &mut oxideav_core::bits::BitReader<'_>,
) -> oxideav_core::Result<ModbMDecoded> {
    // Walk the Table M.1 prefix tree:
    //   "0"     → bidir, no cbpb, no mvdb
    //   "10"    → bidir, cbpb, no mvdb
    //   "110"   → forward, no cbpb, mvdb
    //   "1110"  → forward, cbpb, mvdb
    //   "11110" → backward, no cbpb, no mvdb
    //   "11111" → backward, cbpb, no mvdb
    if br.read_u1()? == 0 {
        return Ok(ModbMDecoded {
            mode: BMode::Bidirectional,
            cbpb_present: false,
            mvdb_present: false,
        });
    }
    if br.read_u1()? == 0 {
        return Ok(ModbMDecoded {
            mode: BMode::Bidirectional,
            cbpb_present: true,
            mvdb_present: false,
        });
    }
    if br.read_u1()? == 0 {
        return Ok(ModbMDecoded {
            mode: BMode::Forward,
            cbpb_present: false,
            mvdb_present: true,
        });
    }
    if br.read_u1()? == 0 {
        return Ok(ModbMDecoded {
            mode: BMode::Forward,
            cbpb_present: true,
            mvdb_present: true,
        });
    }
    // Prefix is "1111" — last bit picks index 4 (no CBPB) vs 5 (CBPB).
    let last = br.read_u1()?;
    Ok(ModbMDecoded {
        mode: BMode::Backward,
        cbpb_present: last == 1,
        mvdb_present: false,
    })
}

/// Emit the Annex M Table M.1 MODB code matching `(mode, cbpb)`.
///
/// MVDB presence is implied by `mode` per Table M.1: it is present iff
/// `mode == BMode::Forward`. The caller does not pass `mvdb_present`
/// separately to make illegal combinations unrepresentable.
pub fn encode_modb_m(bw: &mut oxideav_core::bits::BitWriter, mode: BMode, cbpb_present: bool) {
    match (mode, cbpb_present) {
        (BMode::Bidirectional, false) => bw.write_bits(0b0, 1),
        (BMode::Bidirectional, true) => bw.write_bits(0b10, 2),
        (BMode::Forward, false) => bw.write_bits(0b110, 3),
        (BMode::Forward, true) => bw.write_bits(0b1110, 4),
        (BMode::Backward, false) => bw.write_bits(0b11110, 5),
        (BMode::Backward, true) => bw.write_bits(0b11111, 5),
    }
}

/// Annex M §M.2.3 — backward-only prediction. The predictor is the
/// freshly-reconstructed P-MB pels (`PREC`) at the same destination position
/// in the new P, with no MV data on the wire. Output is signed `i16` so the
/// encoder can compute `source - prediction` without saturating at 0/255.
///
/// `block_idx` selects the BPB-block within the MB (luma 0..=3, chroma 4/5);
/// `(mb_x, mb_y)` is the MB position. The destination block is sampled at
/// the same (`(mb_x, mb_y)` + sub-block offset) coordinates of `p_recon`,
/// matching what §M.2.3 calls "PREC defined in G.5" (i.e. the backward
/// prediction of §G.5 with the backward MV pinned to (0, 0)).
pub fn predict_b_block_backward(
    dst: &mut [i16; 64],
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    p_recon: &IPicture,
) {
    debug_assert!(block_idx < 6);
    let is_chroma = block_idx >= 4;
    if is_chroma {
        let plane = match block_idx {
            4 => &p_recon.cb,
            5 => &p_recon.cr,
            _ => unreachable!(),
        };
        let stride = p_recon.c_stride;
        let bx = mb_x * 8;
        let by = mb_y * 8;
        for j in 0..8 {
            for i in 0..8 {
                dst[j * 8 + i] = plane[(by + j) * stride + (bx + i)] as i16;
            }
        }
    } else {
        let (sub_x, sub_y) = match block_idx {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let bx = mb_x * 16 + sub_x;
        let by = mb_y * 16 + sub_y;
        let stride = p_recon.y_stride;
        for j in 0..8 {
            for i in 0..8 {
                dst[j * 8 + i] = p_recon.y[(by + j) * stride + (bx + i)] as i16;
            }
        }
    }
}

/// Annex M §M.2.2 — forward-only prediction. The forward MV from MVDB
/// (luma half-pel units) selects an 8×8 patch from the **prior P**
/// (`forward_ref`) at this block's destination position offset by the MV.
///
/// `block_idx` is the same as for [`predict_b_block`]; `mvf` is the per-block
/// forward MV (chroma blocks pass the chroma-rounded MV — see
/// [`luma_to_chroma_mv`] / Table F.1).
pub fn predict_b_block_forward(
    dst: &mut [i16; 64],
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    forward_ref: &IPicture,
    mvf: (i32, i32),
) {
    debug_assert!(block_idx < 6);
    let is_chroma = block_idx >= 4;
    let (plane, stride, bx, by, w, h) = if is_chroma {
        let stride = forward_ref.c_stride;
        let plane: &[u8] = match block_idx {
            4 => forward_ref.cb.as_slice(),
            5 => forward_ref.cr.as_slice(),
            _ => unreachable!(),
        };
        let h = (forward_ref.cb.len() / forward_ref.c_stride.max(1)) as i32;
        (
            plane,
            stride,
            (mb_x * 8) as i32,
            (mb_y * 8) as i32,
            stride as i32,
            h,
        )
    } else {
        let (sub_x, sub_y) = match block_idx {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let stride = forward_ref.y_stride;
        let h = (forward_ref.y.len() / forward_ref.y_stride.max(1)) as i32;
        (
            forward_ref.y.as_slice(),
            stride,
            (mb_x * 16 + sub_x) as i32,
            (mb_y * 16 + sub_y) as i32,
            stride as i32,
            h,
        )
    };
    let mut out = [0u8; 64];
    crate::interp::predict_block(plane, stride, w, h, bx, by, mvf.0, mvf.1, 8, &mut out, 8);
    for k in 0..64 {
        dst[k] = out[k] as i16;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::{BitReader, BitWriter};

    #[test]
    fn bquant_table_6() {
        // Spec Table 6 examples — at QUANT = 8 the four DBQUANT codes give
        // 5*8/4=10, 6*8/4=12, 7*8/4=14, 8*8/4=16.
        assert_eq!(bquant_from_quant(8, 0b00), 10);
        assert_eq!(bquant_from_quant(8, 0b01), 12);
        assert_eq!(bquant_from_quant(8, 0b10), 14);
        assert_eq!(bquant_from_quant(8, 0b11), 16);
        // Clip-to-31 path.
        assert_eq!(bquant_from_quant(31, 0b11), 31);
    }

    #[test]
    fn derive_at_midpoint() {
        // TRB = TRD/2, MV = (4, -8) halfpel, MVDB = 0 →
        //   MVF = (TRB * MV) / TRD = (1 * (4, -8)) / 2 = (2, -4)
        //   MVB = ((TRB - TRD) * MV) / TRD = ((-1) * (4, -8)) / 2 = (-2, 4)
        let r = derive_b_block_mvs((4, -8), (0, 0), 1, 2);
        assert_eq!(r.mvf, (2, -4));
        assert_eq!(r.mvb, (-2, 4));
    }

    #[test]
    fn derive_with_mvdb() {
        // TRB = 1, TRD = 2, MV = (4, 0), MVDB = (1, 1) →
        //   MVF = (1*4)/2 + 1 = 3
        //   MVB = MVF - MV = 3 - 4 = -1   (because MVD != 0)
        let r = derive_b_block_mvs((4, 0), (1, 1), 1, 2);
        assert_eq!(r.mvf, (3, 1));
        assert_eq!(r.mvb, (-1, 1));
    }

    #[test]
    fn modb_round_trip() {
        let cases: &[(bool, bool, &[u8])] = &[
            (false, false, &[0]),
            (false, true, &[0b10]),
            (true, true, &[0b11]),
        ];
        for &(cbpb, mvdb, _) in cases {
            let mut bw = BitWriter::with_capacity(8);
            encode_modb(&mut bw, cbpb, mvdb);
            let buf = bw.finish();
            let mut br = BitReader::new(&buf);
            let d = decode_modb(&mut br).unwrap();
            assert_eq!(d.cbpb_present, cbpb);
            // MVDB present is implied by cbpb (per encoder note above).
            let expected_mvdb = mvdb || cbpb;
            assert_eq!(d.mvdb_present, expected_mvdb);
        }
    }

    #[test]
    #[allow(clippy::unusual_byte_groupings)]
    fn modb_m_table_round_trip() {
        // Spec Table M.1/H.263 — verify each of the 6 codewords is emitted
        // and decoded exactly as specified. The unusual byte groupings
        // intentionally separate the codeword bits from the trailing
        // zero-padding so the prefix is human-readable against the spec.
        let cases: &[(BMode, bool, bool, &[u8], u32)] = &[
            // (mode, cbpb, mvdb, byte-prefix, codeword bit length)
            (BMode::Bidirectional, false, false, &[0b0_0000000], 1),
            (BMode::Bidirectional, true, false, &[0b10_000000], 2),
            (BMode::Forward, false, true, &[0b110_00000], 3),
            (BMode::Forward, true, true, &[0b1110_0000], 4),
            (BMode::Backward, false, false, &[0b11110_000], 5),
            (BMode::Backward, true, false, &[0b11111_000], 5),
        ];
        for &(mode, cbpb, mvdb, expected, _) in cases {
            let mut bw = BitWriter::with_capacity(8);
            encode_modb_m(&mut bw, mode, cbpb);
            let buf = bw.finish();
            assert_eq!(
                buf, expected,
                "MODB-M emission for ({mode:?}, cbpb={cbpb}) should match Table M.1"
            );
            let mut br = BitReader::new(&buf);
            let d = decode_modb_m(&mut br).unwrap();
            assert_eq!(d.mode, mode);
            assert_eq!(d.cbpb_present, cbpb);
            assert_eq!(d.mvdb_present, mvdb);
        }
    }

    #[test]
    fn modb_m_distinguishes_from_g() {
        // The Annex G "10" code maps to bidir+MVDB (no CBPB); the Annex M
        // "10" code maps to bidir+CBPB (no MVDB). Asserting they don't
        // alias is the regression check.
        let mut bw_g = BitWriter::with_capacity(8);
        encode_modb(&mut bw_g, false, true);
        let mut bw_m = BitWriter::with_capacity(8);
        encode_modb_m(&mut bw_m, BMode::Bidirectional, true);
        // Both codecs emit the same 2 bits "10" but interpret them differently.
        assert_eq!(bw_g.finish(), bw_m.finish());

        // Decoding "10" through M gives bidir+cbpb (MVDB absent).
        let bytes = vec![0b10_000000u8];
        let mut br = BitReader::new(&bytes);
        let d = decode_modb_m(&mut br).unwrap();
        assert_eq!(d.mode, BMode::Bidirectional);
        assert!(d.cbpb_present);
        assert!(!d.mvdb_present);
    }

    #[test]
    fn predict_backward_copies_p_recon() {
        // §M.2.3 backward prediction is just a copy of the freshly-
        // reconstructed P-MB pels at the same destination position.
        let mut p_recon = IPicture::new(16, 16);
        for j in 0..8 {
            for i in 0..8 {
                p_recon.y[j * 16 + i] = ((j * 16 + i) as u8).wrapping_mul(3);
            }
        }
        let mut dst = [0i16; 64];
        predict_b_block_backward(&mut dst, 0, 0, 0, &p_recon);
        for j in 0..8 {
            for i in 0..8 {
                assert_eq!(dst[j * 8 + i], p_recon.y[j * 16 + i] as i16);
            }
        }
    }

    #[test]
    fn predict_forward_zero_mv_copies_ref() {
        // Forward prediction with MVDB = (0, 0) is just a copy of the prior
        // P at the same position.
        let mut fwd = IPicture::new(16, 16);
        for j in 0..16 {
            for i in 0..16 {
                fwd.y[j * 16 + i] = ((i * 3 + j) % 256) as u8;
            }
        }
        let mut dst = [0i16; 64];
        predict_b_block_forward(&mut dst, 0, 0, 0, &fwd, (0, 0));
        for j in 0..8 {
            for i in 0..8 {
                assert_eq!(dst[j * 8 + i], fwd.y[j * 16 + i] as i16);
            }
        }
    }

    #[test]
    fn chroma_round_table_f1_examples() {
        // Sum of four MVs in luma halfpel; divide by 8 = chroma sixteenth.
        // Round to half-pel.
        assert_eq!(chroma_round_table_f1(0), 0);
        assert_eq!(chroma_round_table_f1(8), 1);
        assert_eq!(chroma_round_table_f1(-8), -1);
        assert_eq!(chroma_round_table_f1(1), 0); // remainder ±1 keeps q.
        assert_eq!(chroma_round_table_f1(-1), 0);
        assert_eq!(chroma_round_table_f1(2), 1); // remainder ±2 → outward.
        assert_eq!(chroma_round_table_f1(-2), -1);
    }
}
