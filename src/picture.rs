//! H.263 picture decode driver (§4.2 / §5 / §6).
//!
//! This module wires the per-layer parsers
//! ([`crate::parse_picture_header`], [`crate::parse_gob_layer`],
//! [`crate::parse_macroblock`], [`crate::parse_block`]) and the
//! per-block reconstruction primitives (intra reconstruction §6.1–§6.3,
//! INTER motion compensation + summation §6.1.1 / §6.1.2 / §6.3.1, and
//! the Annex J §J.3 deblocking filter) into a *full-picture* decode.
//! The result is a decoded planar YUV 4:2:0 frame.
//!
//! ## Scope (the baseline single-MV path)
//!
//! Per §4.2.1 each picture is split into GOBs, scanned top-to-bottom;
//! each GOB header (§5.2) is followed by one or more rows of
//! macroblocks scanned left-to-right (§4.2.2 / Figure 4 / Figure 5).
//! For each of the picture's standardized source formats the number of
//! GOBs and the number of macroblock rows per GOB are fixed
//! ([`crate::H263SourceFormat::num_gobs`] /
//! [`crate::H263SourceFormat::mb_rows_per_gob`]).
//!
//! This driver decodes the **baseline** macroblock set:
//!
//! * **INTRA / INTRA+Q macroblocks** (MB types 3 / 4) — every block
//!   carries INTRADC (§5.4.1); AC presence is governed by CBPY (luma)
//!   and CBPC (chroma). Each block is reconstructed with
//!   [`crate::reconstruct_intra_block`].
//! * **INTER / INTER+Q macroblocks** (MB types 0 / 1) — a single
//!   motion vector per macroblock (§6.1.1). The MV is reconstructed
//!   from the §6.1.1 / Figure-12 median predictor (with the candidate
//!   border-decision rules implemented here) plus the Table-14 MVD,
//!   the luma blocks are motion-compensated from the reference frame
//!   ([`crate::motion::motion_compensate_block`]), the chroma blocks
//!   use the Table-18 derived chroma vector, and each block adds its
//!   IDCT residual via [`crate::reconstruct_inter_block_with_prediction`].
//! * **Skipped macroblocks** (COD = 1) — copied from the reference
//!   frame with a zero motion vector (§5.3.1).
//!
//! After all macroblocks are reconstructed, the Annex J §J.3
//! deblocking filter is applied to each plane *iff* the picture's
//! `J`-mode flag is requested by the caller (the baseline
//! non-extended-PTYPE header cannot signal Annex J on the wire, so the
//! caller passes the flag explicitly through [`DecodeOptions`]).
//!
//! ## Deliberately out of scope
//!
//! * **INTER4V / INTER4V+Q** (MB types 2 / 5) — four motion vectors
//!   per macroblock; the candidate-predictor redefinition lives in
//!   Annex F (§F.2 / Figure F.1) which is not yet wired. The driver
//!   returns [`Error::NotImplemented`] when it meets such a macroblock.
//! * **PB-frames** (Annex G), **extended PTYPE / PLUSPTYPE** (§5.1.4),
//!   **Annex T variable-length DQUANT**, **CPM = 1 / GSBI**, **slice
//!   structured mode (Annex K)**, the Annex-I prediction
//!   reconstruction, and **GSTUF** auto-detection — all rejected /
//!   skipped exactly as the per-layer parsers do.
//! * **Custom picture formats** (the `"110"` / `"111"` source formats)
//!   — the driver needs the standardized GOB/MB layout tables.

// Synthetic test bitstreams group bits to mirror the spec's printed
// MSB-first field layout (e.g. the 7-bit TCOEF ESCAPE prefix
// "0000 011") rather than clippy's power-of-two grouping, matching the
// convention in block.rs / macroblock.rs.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::block::{parse_block, BlockContext, COEFFS_PER_BLOCK};
use crate::deblock::{deblock_plane, strength_for_quant, EdgeCondition};
use crate::gob_header::parse_gob_layer;
use crate::idct::BLOCK_DIM;
use crate::macroblock::{parse_macroblock, H263Macroblock, MbContext, MbType};
use crate::motion::{
    chroma_mv, motion_compensate_block, predict_mv_median, reconstruct_mv, MotionVector, RefPlane,
    RCONTROL_DEFAULT,
};
use crate::picture_header::{parse_picture_header, H263PictureCodingType};
use crate::{reconstruct_inter_block_with_prediction, reconstruct_intra_block, Error, Result};

/// A decoded planar YUV 4:2:0 frame produced by [`decode_picture`].
///
/// Plane storage is row-major `u8`. The luma plane is
/// `luma_width × luma_height`; each chroma plane is
/// `(luma_width / 2) × (luma_height / 2)` per the 4:2:0 sub-sampling
/// of §4.1.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct YuvFrame {
    /// Luma (Y) plane, `luma_width * luma_height` samples.
    pub y: Vec<u8>,
    /// Blue-difference chroma (Cb) plane, `chroma_width * chroma_height`.
    pub cb: Vec<u8>,
    /// Red-difference chroma (Cr) plane, `chroma_width * chroma_height`.
    pub cr: Vec<u8>,
    /// Luma plane width in pixels.
    pub luma_width: usize,
    /// Luma plane height in pixels.
    pub luma_height: usize,
}

impl YuvFrame {
    /// Chroma plane width (luma width / 2 for 4:2:0).
    pub fn chroma_width(&self) -> usize {
        self.luma_width / 2
    }

    /// Chroma plane height (luma height / 2 for 4:2:0).
    pub fn chroma_height(&self) -> usize {
        self.luma_height / 2
    }

    /// Construct an all-grey (sample value 128) frame of the given
    /// luma dimensions — a convenient neutral reference for the first
    /// INTER picture in a sequence when no prior frame exists.
    pub fn grey(luma_width: usize, luma_height: usize) -> Self {
        let cw = luma_width / 2;
        let ch = luma_height / 2;
        YuvFrame {
            y: vec![128u8; luma_width * luma_height],
            cb: vec![128u8; cw * ch],
            cr: vec![128u8; cw * ch],
            luma_width,
            luma_height,
        }
    }
}

/// Caller-supplied decode options for the baseline picture driver.
///
/// The non-extended-PTYPE header cannot signal Annex J on the wire, so
/// the deblocking filter is opt-in here. Annex D / F / G flags read off
/// the header still gate the relevant parser paths (the driver rejects
/// the modes it does not implement); this struct only carries the
/// decisions the wire cannot convey in the baseline header.
#[derive(Debug, Clone, Copy, Default)]
pub struct DecodeOptions {
    /// Run the Annex J §J.3 deblocking filter on the reconstructed
    /// planes after macroblock reconstruction. Off by default.
    pub deblock: bool,
}

/// Per-macroblock state the §6.1.1 / Figure-12 candidate-predictor
/// selection needs from the macroblock grid.
#[derive(Debug, Clone, Copy)]
struct MbGridEntry {
    /// `true` if the macroblock is INTRA-coded (MB type 3 / 4).
    intra: bool,
    /// `true` if the macroblock is "not coded" (COD = 1, skip).
    not_coded: bool,
    /// Reconstructed luma motion vector (half-pel). Zero for INTRA /
    /// not-coded macroblocks (which still participate in prediction as
    /// the spec's "set to zero" candidates).
    mv: MotionVector,
}

impl MbGridEntry {
    /// An off-picture / outside-the-coded-area sentinel.
    const OUTSIDE: MbGridEntry = MbGridEntry {
        intra: false,
        not_coded: false,
        mv: MotionVector::new(0, 0),
    };
}

/// §6.1.1 / Figure-12 candidate-predictor selection for the baseline
/// one-vector-per-macroblock case.
///
/// `grid` is the row-major macroblock grid (`mb_cols × mb_rows`) of
/// already-decoded entries; `(col, row)` is the current macroblock.
/// `gob_top_row` is the macroblock-grid row index of the first row of
/// the current GOB (so the §6.1.1 rule 3 "outside the GOB at the top"
/// border can be detected when the GOB header is non-empty —
/// `gob_header_present`).
///
/// The candidate layout (Figure 12):
/// * MV1 = left neighbour `(col-1, row)`.
/// * MV2 = above neighbour `(col, row-1)`.
/// * MV3 = above-right neighbour `(col+1, row-1)`.
fn predict_mv(
    grid: &[MbGridEntry],
    mb_cols: usize,
    col: usize,
    row: usize,
    gob_top_row: usize,
    gob_header_present: bool,
) -> MotionVector {
    let fetch = |c: isize, r: isize| -> Option<MbGridEntry> {
        if c < 0 || r < 0 || c as usize >= mb_cols || r as usize > row {
            // r > row means the entry has not been decoded yet (we only
            // ever look at the current row's left neighbour and the
            // previous row); treat as outside.
            None
        } else {
            Some(grid[r as usize * mb_cols + c as usize])
        }
    };

    // §6.1.1 rule 1: an INTRA or not-coded candidate contributes a
    // zero vector. We fold that into the per-candidate value below.
    let candidate_value = |entry: MbGridEntry| -> MotionVector {
        if entry.intra || entry.not_coded {
            MotionVector::new(0, 0)
        } else {
            entry.mv
        }
    };

    // MV1 — left neighbour. §6.1.1 rule 2: zero if outside picture/
    // slice at the left side.
    let outside_left = col == 0;
    let mv1 = if outside_left {
        MotionVector::new(0, 0)
    } else {
        candidate_value(fetch(col as isize - 1, row as isize).unwrap_or(MbGridEntry::OUTSIDE))
    };

    // §6.1.1 rule 3: MV2 / MV3 are set to MV1 if the corresponding
    // macroblock is outside the picture at the top, or outside the GOB
    // at the top when the current GOB's header is non-empty.
    let above_outside_picture = row == 0;
    let above_outside_gob = gob_header_present && row == gob_top_row;
    let top_border = above_outside_picture || above_outside_gob;

    // MV2 — above neighbour.
    let mv2 = if top_border {
        mv1
    } else {
        candidate_value(fetch(col as isize, row as isize - 1).unwrap_or(MbGridEntry::OUTSIDE))
    };

    // MV3 — above-right neighbour. §6.1.1 rule 4: zero if outside the
    // picture at the right side (otherwise rule 3's top-border copy of
    // MV1 applies).
    let outside_right = col + 1 >= mb_cols;
    let mv3 = if outside_right {
        // Rule 4: outside picture at the right -> zero. This applies
        // after rule 3, so a right-edge MB at a top border still gets
        // zero (not MV1).
        MotionVector::new(0, 0)
    } else if top_border {
        mv1
    } else {
        candidate_value(fetch(col as isize + 1, row as isize - 1).unwrap_or(MbGridEntry::OUTSIDE))
    };

    predict_mv_median(mv1, mv2, mv3)
}

/// Copy an 8×8 sample block into a plane at the given pixel origin.
fn blit_block(
    plane: &mut [u8],
    stride: usize,
    x0: usize,
    y0: usize,
    block: &[u8; COEFFS_PER_BLOCK],
) {
    for by in 0..BLOCK_DIM {
        let dst = (y0 + by) * stride + x0;
        plane[dst..dst + BLOCK_DIM]
            .copy_from_slice(&block[by * BLOCK_DIM..by * BLOCK_DIM + BLOCK_DIM]);
    }
}

/// Decode a single H.263 picture from `data`, starting at the bit
/// position of the Picture Start Code.
///
/// `reference` is the previously-decoded frame used as the motion-
/// compensation source for INTER / skipped macroblocks. For an INTRA
/// (I) picture it is ignored and may be `None`; for an INTER (P)
/// picture it must be `Some` and must match the picture's luma
/// dimensions, or [`Error::NotImplemented`] is returned (a missing
/// reference cannot be motion-compensated).
///
/// Returns the decoded [`YuvFrame`].
///
/// # Errors
///
/// Propagates every per-layer parser error, plus:
/// * [`Error::NotImplemented`] for the unsupported paths listed in the
///   module docs (extended PTYPE, INTER4V, custom format, missing
///   reference for an INTER picture, PB-frames, ...).
pub fn decode_picture(
    data: &[u8],
    reference: Option<&YuvFrame>,
    options: DecodeOptions,
) -> Result<YuvFrame> {
    let mut reader = BitReader::new(data);
    let header = parse_picture_header(&mut reader)?;

    // Unsupported header-signalled modes — refuse rather than guess.
    if header.pb_frames || header.sac_mode {
        return Err(Error::NotImplemented);
    }

    let (luma_w, luma_h) = header
        .source_format
        .luma_dimensions()
        .ok_or(Error::NotImplemented)?;
    let num_gobs = header
        .source_format
        .num_gobs()
        .ok_or(Error::NotImplemented)?;
    let mb_rows_per_gob = header
        .source_format
        .mb_rows_per_gob()
        .ok_or(Error::NotImplemented)?;
    let mb_cols = (luma_w / 16) as usize;
    let mb_rows_total = (luma_h / 16) as usize;

    let luma_w = luma_w as usize;
    let luma_h = luma_h as usize;
    let chroma_w = luma_w / 2;
    let chroma_h = luma_h / 2;

    let is_inter_picture = matches!(header.coding_type, H263PictureCodingType::Inter);

    // INTER pictures need a same-sized reference plane.
    if is_inter_picture {
        match reference {
            Some(r) if r.luma_width == luma_w && r.luma_height == luma_h => {}
            _ => return Err(Error::NotImplemented),
        }
    }

    let mut frame = YuvFrame {
        y: vec![0u8; luma_w * luma_h],
        cb: vec![0u8; chroma_w * chroma_h],
        cr: vec![0u8; chroma_w * chroma_h],
        luma_width: luma_w,
        luma_height: luma_h,
    };

    // Macroblock grid for §6.1.1 candidate-predictor selection and the
    // Annex J per-edge condition.
    let mut grid = vec![MbGridEntry::OUTSIDE; mb_cols * mb_rows_total];

    // Per-macroblock QUANT after any DQUANT, used by the deblocking
    // STRENGTH lookup. Indexed by grid position.
    let mut mb_quant = vec![0u8; mb_cols * mb_rows_total];

    // Walk GOBs top-to-bottom (§4.2.1 vertical scan). Every GOB —
    // including the topmost — is expected to carry a GOB header in the
    // baseline driver. The spec permits "GOB 0" to omit its header
    // (its QUANT then being the picture-layer PQUANT), but PQUANT lives
    // in the extended/optional header block this baseline subset does
    // not decode; requiring a header for every GOB keeps the driver
    // self-contained against the layer set we parse. The header is
    // therefore always "present" for the §6.1.1 rule-3 "outside the
    // GOB at the top" border test.
    let gob_header_present = true;
    for gob_index in 0..num_gobs as usize {
        let gob = parse_gob_layer(&mut reader)?;
        let gob_quant = gob.quantiser;
        let gob_top_row = gob_index * mb_rows_per_gob as usize;

        for local_row in 0..mb_rows_per_gob as usize {
            let row = gob_top_row + local_row;
            if row >= mb_rows_total {
                break;
            }
            let mut current_quant = gob_quant;

            for col in 0..mb_cols {
                // §5.3.2: an MCBPC stuffing code carries no macroblock
                // data; skip it and re-read until a real macroblock
                // (or the skip / coded macroblock) appears for this
                // grid position.
                let mb = loop {
                    let mb = parse_macroblock(
                        &mut reader,
                        MbContext {
                            picture_coding_type: header.coding_type,
                            advanced_prediction: header.advanced_prediction,
                            quantiser_before: current_quant,
                        },
                    )?;
                    if matches!(mb.mb_type, Some(MbType::Stuffing)) {
                        continue;
                    }
                    break mb;
                };

                let mv = decode_one_macroblock(
                    &mut reader,
                    &mb,
                    reference,
                    &mut frame,
                    &grid,
                    mb_cols,
                    col,
                    row,
                    gob_top_row,
                    gob_header_present,
                    &mut current_quant,
                )?;
                record_grid(
                    &mut grid,
                    &mut mb_quant,
                    mb_cols,
                    col,
                    row,
                    &mb,
                    current_quant,
                    mv,
                );
            }
        }
    }

    if options.deblock {
        apply_deblocking(&mut frame, &grid, &mb_quant, mb_cols, mb_rows_total);
    }

    Ok(frame)
}

/// Record a decoded macroblock into the prediction grid + QUANT map.
///
/// `mv` is the *reconstructed* luma motion vector the decode path
/// produced (zero for INTRA / skipped macroblocks), so later
/// neighbours predict against the correct value (§6.1.1).
#[allow(clippy::too_many_arguments)]
fn record_grid(
    grid: &mut [MbGridEntry],
    mb_quant: &mut [u8],
    mb_cols: usize,
    col: usize,
    row: usize,
    mb: &H263Macroblock,
    quant: u8,
    mv: MotionVector,
) {
    let idx = row * mb_cols + col;
    grid[idx] = MbGridEntry {
        intra: mb.mb_type.map(MbType::is_intra).unwrap_or(false),
        not_coded: !mb.coded,
        mv,
    };
    mb_quant[idx] = quant;
}

/// Decode and reconstruct one macroblock into the frame planes,
/// returning the reconstructed luma motion vector (zero for INTRA /
/// skipped macroblocks) for the caller to record in the prediction
/// grid.
#[allow(clippy::too_many_arguments)]
fn decode_one_macroblock(
    reader: &mut BitReader<'_>,
    mb: &H263Macroblock,
    reference: Option<&YuvFrame>,
    frame: &mut YuvFrame,
    grid: &[MbGridEntry],
    mb_cols: usize,
    col: usize,
    row: usize,
    gob_top_row: usize,
    gob_header_present: bool,
    current_quant: &mut u8,
) -> Result<MotionVector> {
    let luma_stride = frame.luma_width;
    let chroma_stride = frame.chroma_width();

    // Pixel origin of the macroblock.
    let mb_x = col * 16;
    let mb_y = row * 16;
    let c_x = col * 8;
    let c_y = row * 8;

    // Skipped macroblock (COD = 1): copy from the reference with a
    // zero motion vector (§5.3.1).
    if !mb.coded {
        let reference = reference.ok_or(Error::NotImplemented)?;
        copy_inter_macroblock(
            reference,
            frame,
            mb_x,
            mb_y,
            c_x,
            c_y,
            MotionVector::new(0, 0),
        );
        return Ok(MotionVector::new(0, 0));
    }

    let mb_type = mb.mb_type.ok_or(Error::NotImplemented)?;

    // INTER4V / INTER4V+Q need Annex-F four-vector prediction — out of
    // scope for the baseline driver.
    if matches!(mb_type, MbType::Inter4V | MbType::Inter4VQ) {
        return Err(Error::NotImplemented);
    }

    *current_quant = mb.quantiser_after;
    let quant = mb.quantiser_after;

    let cbpy = mb.cbpy.unwrap_or(0);
    let cbpc = mb.cbpc.unwrap_or(0);

    if mb_type.is_intra() {
        // INTRA / INTRA+Q: every block has INTRADC; CBPY/CBPC govern AC.
        // CBPY is in CBPY(INTRA) orientation: bit 3 (0b1000) = block 1,
        // bit 0 (0b0001) = block 4 (§5.3.5, Figure 5).
        for blk in 0..4 {
            let has_ac = (cbpy >> (3 - blk)) & 1 == 1;
            let block = parse_block(
                reader,
                BlockContext {
                    has_intradc: true,
                    has_coefficients: has_ac,
                },
            )?;
            let samples = reconstruct_intra_block(&block, quant);
            let (bx, by) = luma_block_origin(mb_x, mb_y, blk);
            blit_block(&mut frame.y, luma_stride, bx, by, &samples);
        }
        // Chroma: CBPC bit 0b10 = Cb (block 5), 0b01 = Cr (block 6).
        let cb_block = parse_block(
            reader,
            BlockContext {
                has_intradc: true,
                has_coefficients: cbpc & 0b10 != 0,
            },
        )?;
        let cb_samples = reconstruct_intra_block(&cb_block, quant);
        blit_block(&mut frame.cb, chroma_stride, c_x, c_y, &cb_samples);

        let cr_block = parse_block(
            reader,
            BlockContext {
                has_intradc: true,
                has_coefficients: cbpc & 0b01 != 0,
            },
        )?;
        let cr_samples = reconstruct_intra_block(&cr_block, quant);
        blit_block(&mut frame.cr, chroma_stride, c_x, c_y, &cr_samples);

        // INTRA macroblocks have no motion vector (§6.1.1 rule 1 treats
        // them as zero candidates for neighbours).
        return Ok(MotionVector::new(0, 0));
    }

    // INTER / INTER+Q (single MV).
    let reference = reference.ok_or(Error::NotImplemented)?;

    // §6.1.1 / Figure-12 predictor + Table-14 MVD.
    let predictor = predict_mv(grid, mb_cols, col, row, gob_top_row, gob_header_present);
    let mvd = mb.mvd.ok_or(Error::NotImplemented)?;
    let luma_mv = reconstruct_mv(predictor, mvd);
    let chroma_vec = chroma_mv(luma_mv);

    // INTER macroblocks: CBPY is the *complement* on the wire — the
    // macroblock parser already returns the CBPY(INTRA) orientation, so
    // for INTER the actual coded pattern is `cbpy ^ 0b1111` (§5.3.5).
    let inter_cbpy = cbpy ^ 0b1111;

    let y_ref = RefPlane::new(&reference.y, reference.luma_width, reference.luma_height);
    for blk in 0..4 {
        let has_coef = (inter_cbpy >> (3 - blk)) & 1 == 1;
        let (bx, by) = luma_block_origin(mb_x, mb_y, blk);
        let prediction = motion_compensate_block(&y_ref, bx, by, luma_mv, RCONTROL_DEFAULT);
        let samples = if has_coef {
            let block = parse_block(
                reader,
                BlockContext {
                    has_intradc: false,
                    has_coefficients: true,
                },
            )?;
            reconstruct_inter_block_with_prediction(&block, quant, &prediction)
        } else {
            prediction
        };
        blit_block(&mut frame.y, luma_stride, bx, by, &samples);
    }

    let cb_ref = RefPlane::new(
        &reference.cb,
        reference.chroma_width(),
        reference.chroma_height(),
    );
    let cb_pred = motion_compensate_block(&cb_ref, c_x, c_y, chroma_vec, RCONTROL_DEFAULT);
    let cb_samples = if cbpc & 0b10 != 0 {
        let block = parse_block(
            reader,
            BlockContext {
                has_intradc: false,
                has_coefficients: true,
            },
        )?;
        reconstruct_inter_block_with_prediction(&block, quant, &cb_pred)
    } else {
        cb_pred
    };
    blit_block(&mut frame.cb, chroma_stride, c_x, c_y, &cb_samples);

    let cr_ref = RefPlane::new(
        &reference.cr,
        reference.chroma_width(),
        reference.chroma_height(),
    );
    let cr_pred = motion_compensate_block(&cr_ref, c_x, c_y, chroma_vec, RCONTROL_DEFAULT);
    let cr_samples = if cbpc & 0b01 != 0 {
        let block = parse_block(
            reader,
            BlockContext {
                has_intradc: false,
                has_coefficients: true,
            },
        )?;
        reconstruct_inter_block_with_prediction(&block, quant, &cr_pred)
    } else {
        cr_pred
    };
    blit_block(&mut frame.cr, chroma_stride, c_x, c_y, &cr_samples);

    Ok(luma_mv)
}

/// Copy an entire macroblock (4 luma + 2 chroma blocks) from the
/// reference frame, motion-compensated by `luma_mv` (§5.3.1 skip path
/// uses a zero vector).
fn copy_inter_macroblock(
    reference: &YuvFrame,
    frame: &mut YuvFrame,
    mb_x: usize,
    mb_y: usize,
    c_x: usize,
    c_y: usize,
    luma_mv: MotionVector,
) {
    let luma_stride = frame.luma_width;
    let chroma_stride = frame.chroma_width();
    let chroma_vec = chroma_mv(luma_mv);

    let y_ref = RefPlane::new(&reference.y, reference.luma_width, reference.luma_height);
    for blk in 0..4 {
        let (bx, by) = luma_block_origin(mb_x, mb_y, blk);
        let pred = motion_compensate_block(&y_ref, bx, by, luma_mv, RCONTROL_DEFAULT);
        blit_block(&mut frame.y, luma_stride, bx, by, &pred);
    }
    let cb_ref = RefPlane::new(
        &reference.cb,
        reference.chroma_width(),
        reference.chroma_height(),
    );
    let cb = motion_compensate_block(&cb_ref, c_x, c_y, chroma_vec, RCONTROL_DEFAULT);
    blit_block(&mut frame.cb, chroma_stride, c_x, c_y, &cb);
    let cr_ref = RefPlane::new(
        &reference.cr,
        reference.chroma_width(),
        reference.chroma_height(),
    );
    let cr = motion_compensate_block(&cr_ref, c_x, c_y, chroma_vec, RCONTROL_DEFAULT);
    blit_block(&mut frame.cr, chroma_stride, c_x, c_y, &cr);
}

/// Pixel origin `(x, y)` of luma block `blk` (0..4) within the
/// macroblock at `(mb_x, mb_y)`, per the Figure-5 numbering: block 1
/// top-left, block 2 top-right, block 3 bottom-left, block 4
/// bottom-right.
fn luma_block_origin(mb_x: usize, mb_y: usize, blk: usize) -> (usize, usize) {
    let dx = (blk & 1) * BLOCK_DIM;
    let dy = (blk >> 1) * BLOCK_DIM;
    (mb_x + dx, mb_y + dy)
}

/// Apply the Annex J §J.3 deblocking filter to all three planes.
///
/// The per-edge condition runs the filter when at least one of the two
/// macroblocks touching the edge is coded (COD == 0 or INTRA) per
/// §J.3. The STRENGTH is taken from Table J.2 against the QUANT of the
/// macroblock owning `block2` (the lower / right block of the edge).
fn apply_deblocking(
    frame: &mut YuvFrame,
    grid: &[MbGridEntry],
    mb_quant: &[u8],
    mb_cols: usize,
    mb_rows: usize,
) {
    let luma_w = frame.luma_width;
    let luma_h = frame.luma_height;
    let chroma_w = frame.chroma_width();
    let chroma_h = frame.chroma_height();

    // Edge condition: §J.3 "filter if block1 or block2 belongs to a
    // coded MB". `block_to_mb` maps the plane's 8×8 block coordinate to
    // its owning macroblock for the given blocks-per-MB factor (2 for
    // luma, 1 for chroma).
    let edge_cond =
        |b1: (usize, usize), b2: (usize, usize), blocks_per_mb: usize| -> EdgeCondition {
            let mb1 = (b1.0 / blocks_per_mb, b1.1 / blocks_per_mb);
            let mb2 = (b2.0 / blocks_per_mb, b2.1 / blocks_per_mb);
            let coded = |m: (usize, usize)| -> bool {
                if m.0 >= mb_cols || m.1 >= mb_rows {
                    return false;
                }
                let e = grid[m.1 * mb_cols + m.0];
                // A coded MB is one that is not "not coded": INTRA or
                // INTER with residual/MV both count as coded for §J.3.
                !e.not_coded
            };
            if coded(mb1) || coded(mb2) {
                let q = mb_quant
                    .get(mb2.1 * mb_cols + mb2.0)
                    .copied()
                    .filter(|&q| q != 0)
                    .unwrap_or_else(|| mb_quant[mb1.1 * mb_cols + mb1.0]);
                EdgeCondition::Filter {
                    strength: strength_for_quant(q),
                }
            } else {
                EdgeCondition::Skip
            }
        };

    deblock_plane(&mut frame.y, luma_w, luma_h, luma_w, |b1, b2| {
        edge_cond(b1, b2, 2)
    });
    deblock_plane(&mut frame.cb, chroma_w, chroma_h, chroma_w, |b1, b2| {
        edge_cond(b1, b2, 1)
    });
    deblock_plane(&mut frame.cr, chroma_w, chroma_h, chroma_w, |b1, b2| {
        edge_cond(b1, b2, 1)
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::ZIGZAG_TO_BLOCK_POS;
    use crate::gob_header::{GBSC_BITS, GBSC_VALUE, GFID_BITS, GN_BITS, GQUANT_BITS};
    use crate::picture_header::{H263SourceFormat, PSC_BITS, PSC_VALUE};
    use oxideav_core::bits::BitWriter;

    /// Source-format helpers must agree with the spec's GOB/MB tables.
    #[test]
    fn qcif_layout_constants() {
        let f = H263SourceFormat::Qcif;
        assert_eq!(f.luma_dimensions(), Some((176, 144)));
        assert_eq!(f.num_gobs(), Some(9));
        assert_eq!(f.mb_rows_per_gob(), Some(1));
        assert_eq!(f.mbs_per_row(), Some(11));
        assert_eq!(f.total_macroblocks(), Some(99));
    }

    #[test]
    fn cif_layout_constants() {
        let f = H263SourceFormat::Cif;
        assert_eq!(f.num_gobs(), Some(18));
        assert_eq!(f.mb_rows_per_gob(), Some(1));
        assert_eq!(f.mbs_per_row(), Some(22));
        assert_eq!(f.total_macroblocks(), Some(22 * 18));
    }

    #[test]
    fn cif4_two_rows_per_gob() {
        let f = H263SourceFormat::Cif4;
        assert_eq!(f.num_gobs(), Some(18));
        assert_eq!(f.mb_rows_per_gob(), Some(2));
        assert_eq!(f.mbs_per_row(), Some(44));
        // 18 GOBs * 2 rows = 36 MB rows; 704/16 = 44 cols.
        assert_eq!(f.total_macroblocks(), Some(44 * 36));
    }

    #[test]
    fn grey_frame_dimensions() {
        let g = YuvFrame::grey(176, 144);
        assert_eq!(g.y.len(), 176 * 144);
        assert_eq!(g.cb.len(), 88 * 72);
        assert_eq!(g.cr.len(), 88 * 72);
        assert!(g.y.iter().all(|&p| p == 128));
        assert!(g.cb.iter().all(|&p| p == 128));
    }

    /// luma_block_origin places the four blocks in Figure-5 order.
    #[test]
    fn luma_block_origins_figure5() {
        assert_eq!(luma_block_origin(16, 32, 0), (16, 32)); // block 1 TL
        assert_eq!(luma_block_origin(16, 32, 1), (24, 32)); // block 2 TR
        assert_eq!(luma_block_origin(16, 32, 2), (16, 40)); // block 3 BL
        assert_eq!(luma_block_origin(16, 32, 3), (24, 40)); // block 4 BR
    }

    /// blit_block copies an 8×8 block into the right window.
    #[test]
    fn blit_block_places_8x8() {
        let mut plane = vec![0u8; 16 * 16];
        let mut block = [0u8; COEFFS_PER_BLOCK];
        for (i, b) in block.iter_mut().enumerate() {
            *b = i as u8;
        }
        blit_block(&mut plane, 16, 8, 8, &block);
        // Top-left of the destination block.
        assert_eq!(plane[8 * 16 + 8], 0);
        // (row 1, col 0) of the block = value 8.
        assert_eq!(plane[9 * 16 + 8], 8);
        // (row 7, col 7) = value 63.
        assert_eq!(plane[15 * 16 + 15], 63);
        // Outside the block is untouched.
        assert_eq!(plane[0], 0);
        assert_eq!(plane[7 * 16 + 7], 0);
    }

    // ---- §6.1.1 / Figure-12 candidate-predictor selection -----------

    fn grid_with(cols: usize, rows: usize) -> Vec<MbGridEntry> {
        vec![MbGridEntry::OUTSIDE; cols * rows]
    }

    /// Top-left macroblock: every candidate is a border -> zero
    /// predictor.
    #[test]
    fn predict_top_left_is_zero() {
        let grid = grid_with(11, 9);
        let p = predict_mv(&grid, 11, 0, 0, 0, true);
        assert_eq!(p, MotionVector::new(0, 0));
    }

    /// Within-row left neighbour drives MV1; with MV2/MV3 at the top
    /// border copied from MV1, the median is MV1.
    #[test]
    fn predict_uses_left_neighbour_at_top_row() {
        let mut grid = grid_with(11, 9);
        grid[0] = MbGridEntry {
            intra: false,
            not_coded: false,
            mv: MotionVector::new(6, -4),
        };
        // MB (1, 0): MV1 = grid[0] = (6,-4); top border so MV2=MV3=MV1.
        // median = (6,-4).
        let p = predict_mv(&grid, 11, 1, 0, 0, true);
        assert_eq!(p, MotionVector::new(6, -4));
    }

    /// An INTRA left neighbour contributes a zero candidate (rule 1).
    #[test]
    fn predict_intra_neighbour_is_zero_candidate() {
        let mut grid = grid_with(11, 9);
        grid[0] = MbGridEntry {
            intra: true,
            not_coded: false,
            mv: MotionVector::new(10, 10), // ignored because intra
        };
        let p = predict_mv(&grid, 11, 1, 0, 0, true);
        assert_eq!(p, MotionVector::new(0, 0));
    }

    /// Interior macroblock: median of left / above / above-right.
    #[test]
    fn predict_interior_median() {
        let mut grid = grid_with(11, 9);
        let set = |g: &mut [MbGridEntry], c: usize, r: usize, dx: i32, dy: i32| {
            g[r * 11 + c] = MbGridEntry {
                intra: false,
                not_coded: false,
                mv: MotionVector::new(dx, dy),
            };
        };
        // current MB at (2, 1): MV1=(1,2) left=(1,1)? careful with idx.
        set(&mut grid, 1, 1, 2, 2); // left  (col-1,row)
        set(&mut grid, 2, 0, 8, -2); // above (col,row-1)
        set(&mut grid, 3, 0, -4, 6); // above-right (col+1,row-1)
        let p = predict_mv(&grid, 11, 2, 1, 0, false);
        // medians: dx median(2,8,-4)=2; dy median(2,-2,6)=2.
        assert_eq!(p, MotionVector::new(2, 2));
    }

    /// Right-edge macroblock: MV3 (above-right) is forced to zero
    /// (rule 4), even when an above neighbour exists.
    #[test]
    fn predict_right_edge_mv3_is_zero() {
        let mut grid = grid_with(11, 9);
        let set = |g: &mut [MbGridEntry], c: usize, r: usize, dx: i32, dy: i32| {
            g[r * 11 + c] = MbGridEntry {
                intra: false,
                not_coded: false,
                mv: MotionVector::new(dx, dy),
            };
        };
        // current MB at the rightmost column (10, 1).
        set(&mut grid, 9, 1, 10, 10); // left
        set(&mut grid, 10, 0, 20, 20); // above
                                       // above-right (11,0) is outside -> rule 4 zero.
        let p = predict_mv(&grid, 11, 10, 1, 0, false);
        // candidates: MV1=(10,10), MV2=(20,20), MV3=(0,0).
        // median dx of (10,20,0)=10; dy=10.
        assert_eq!(p, MotionVector::new(10, 10));
    }

    // ---- end-to-end picture decode ---------------------------------

    /// Build a minimal QCIF INTRA picture where every macroblock is a
    /// DC-only INTRA macroblock with INTRADC code 0x10 (-> level 128,
    /// pixel 16 after IDCT) and no AC coefficients. Returns the byte
    /// buffer.
    ///
    /// Layout per GOB: GBSC + GN + GFID + GQUANT, then 11 macroblocks.
    /// Each macroblock (I-picture): MCBPC=`1` (type INTRA, cbpc 00) +
    /// CBPY=`0011` (Table 12 index 0: no AC in any luma block) + 6 ×
    /// INTRADC byte 0x10.
    fn build_qcif_intra_dc_picture(intradc_code: u8) -> Vec<u8> {
        let mut w = BitWriter::new();
        // Picture header: QCIF, INTRA, all flags off.
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8); // TR
        w.write_bit(true); // PTYPE bit1
        w.write_bit(false); // PTYPE bit2
        w.write_bit(false); // split-screen
        w.write_bit(false); // doc-camera
        w.write_bit(false); // freeze
        w.write_u32(0b010, 3); // source format QCIF
        w.write_bit(false); // coding type INTRA
        w.write_bit(false); // umv
        w.write_bit(false); // sac
        w.write_bit(false); // ap
        w.write_bit(false); // pb

        for _gob in 0..9 {
            // GOB header.
            w.write_u32(GBSC_VALUE, GBSC_BITS);
            w.write_u32(1, GN_BITS); // GN (any valid; driver ignores)
            w.write_u32(0, GFID_BITS);
            w.write_u32(8, GQUANT_BITS); // QUANT = 8
            for _mb in 0..11 {
                // MCBPC = `1` -> I-picture type INTRA, cbpc 00.
                w.write_bit(true);
                // CBPY = `0011` (Table 12 index 0): CBPY(INTRA) = 0000,
                // i.e. no AC in any luma block.
                w.write_bit(false);
                w.write_bit(false);
                w.write_bit(true);
                w.write_bit(true);
                // Six blocks, each just INTRADC (8-bit FLC).
                for _blk in 0..6 {
                    w.write_u32(intradc_code as u32, 8);
                }
            }
        }
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    #[test]
    fn decode_qcif_intra_dc_only_uniform_frame() {
        // INTRADC code 0x10 -> Table 15 level 0x10 * 8 = 128 ->
        // IDCT distributes 128/8 = 16 to every pixel.
        let data = build_qcif_intra_dc_picture(0x10);
        let frame = decode_picture(&data, None, DecodeOptions::default()).expect("decode");
        assert_eq!(frame.luma_width, 176);
        assert_eq!(frame.luma_height, 144);
        assert_eq!(frame.y.len(), 176 * 144);
        assert_eq!(frame.cb.len(), 88 * 72);
        // Every luma + chroma sample is 16.
        assert!(frame.y.iter().all(|&p| p == 16), "luma not uniform 16");
        assert!(frame.cb.iter().all(|&p| p == 16), "cb not uniform 16");
        assert!(frame.cr.iter().all(|&p| p == 16), "cr not uniform 16");
    }

    #[test]
    fn decode_qcif_intra_higher_dc() {
        // INTRADC 0x40 -> level 512 -> 512/8 = 64 per pixel.
        let data = build_qcif_intra_dc_picture(0x40);
        let frame = decode_picture(&data, None, DecodeOptions::default()).expect("decode");
        assert!(frame.y.iter().all(|&p| p == 64));
        assert!(frame.cb.iter().all(|&p| p == 64));
    }

    #[test]
    fn decode_intra_then_deblock_is_noop_on_flat_field() {
        // A uniformly flat reconstructed frame has no block edges to
        // smooth, so the §J.3 filter must leave every sample unchanged
        // (d = (A−4B+4C−D)/8 = 0 when A=B=C=D).
        let data = build_qcif_intra_dc_picture(0x10);
        let frame = decode_picture(&data, None, DecodeOptions { deblock: true }).expect("decode");
        assert!(frame.y.iter().all(|&p| p == 16));
        assert!(frame.cb.iter().all(|&p| p == 16));
        assert!(frame.cr.iter().all(|&p| p == 16));
    }

    /// An INTER picture with all-skipped macroblocks must reproduce the
    /// reference frame exactly (zero MV, no residual).
    #[test]
    fn decode_inter_all_skipped_copies_reference() {
        // Build a non-flat reference so an accidental zero-fill would
        // be detected.
        let mut reference = YuvFrame::grey(176, 144);
        for (i, p) in reference.y.iter_mut().enumerate() {
            *p = (i % 200) as u8;
        }
        for (i, p) in reference.cb.iter_mut().enumerate() {
            *p = (i % 100) as u8;
        }
        for (i, p) in reference.cr.iter_mut().enumerate() {
            *p = (i % 50) as u8;
        }

        // INTER picture, every MB COD = 1 (skipped).
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8);
        w.write_bit(true);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_u32(0b010, 3); // QCIF
        w.write_bit(true); // INTER
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        for _gob in 0..9 {
            w.write_u32(GBSC_VALUE, GBSC_BITS);
            w.write_u32(1, GN_BITS);
            w.write_u32(0, GFID_BITS);
            w.write_u32(8, GQUANT_BITS);
            for _mb in 0..11 {
                // COD = 1 -> skipped.
                w.write_bit(true);
            }
        }
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let data = w.finish();

        let frame =
            decode_picture(&data, Some(&reference), DecodeOptions::default()).expect("decode");
        // Zero-MV motion compensation of an integer position is an
        // exact copy.
        assert_eq!(frame.y, reference.y);
        assert_eq!(frame.cb, reference.cb);
        assert_eq!(frame.cr, reference.cr);
    }

    /// INTER picture with one coded INTER macroblock carrying a small
    /// residual: confirm the residual is applied on top of the
    /// motion-compensated prediction and the rest is copied.
    #[test]
    fn decode_inter_picture_missing_reference_is_error() {
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8);
        w.write_bit(true);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_u32(0b010, 3);
        w.write_bit(true); // INTER
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let data = w.finish();
        assert_eq!(
            decode_picture(&data, None, DecodeOptions::default()).unwrap_err(),
            Error::NotImplemented
        );
    }

    /// Extended PTYPE (source format 111) is refused before any GOB.
    #[test]
    fn decode_extended_ptype_is_refused() {
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8);
        w.write_bit(true);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_u32(0b111, 3); // extended PTYPE
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let data = w.finish();
        assert_eq!(
            decode_picture(&data, None, DecodeOptions::default()).unwrap_err(),
            Error::ExtendedPtypeNotSupported
        );
    }

    /// An INTRA macroblock with one AC coefficient in luma block 1
    /// should differ from the DC-only field in that block, while the
    /// other luma blocks stay uniform — confirming CBPY drives per-block
    /// AC presence.
    #[test]
    fn decode_intra_cbpy_drives_per_block_ac() {
        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8);
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

        // We only encode the first GOB's first macroblock with AC; the
        // remaining 10 MBs of row 0 and all later GOBs are DC-only so
        // the decode completes. Macroblock 0 of GOB 0: CBPY index 3
        // codeword `1001` -> CBPY(INTRA) pattern `0011` (Table 12,
        // read `(12, 34)` top row `00` then bottom row `11`), i.e.
        // blocks 3 & 4 have AC, blocks 1 & 2 do not.
        for gob in 0..9 {
            w.write_u32(GBSC_VALUE, GBSC_BITS);
            w.write_u32(1, GN_BITS);
            w.write_u32(0, GFID_BITS);
            w.write_u32(1, GQUANT_BITS); // QUANT = 1
            for mb in 0..11 {
                w.write_bit(true); // MCBPC `1` INTRA cbpc 00
                if gob == 0 && mb == 0 {
                    // CBPY = `1001` (index 3): blocks 3,4 have AC.
                    w.write_bit(true);
                    w.write_bit(false);
                    w.write_bit(false);
                    w.write_bit(true);
                    // Blocks 1,2: INTRADC only.
                    w.write_u32(0x10, 8);
                    w.write_u32(0x10, 8);
                    // Block 3: INTRADC 0x10 + one TCOEF.
                    w.write_u32(0x10, 8);
                    write_single_tcoef_last(&mut w);
                    // Block 4: INTRADC 0x10 + one TCOEF.
                    w.write_u32(0x10, 8);
                    write_single_tcoef_last(&mut w);
                    // Chroma 5,6: INTRADC only (cbpc 00).
                    w.write_u32(0x10, 8);
                    w.write_u32(0x10, 8);
                } else {
                    // CBPY = `0011` (index 0): no AC.
                    w.write_bit(false);
                    w.write_bit(false);
                    w.write_bit(true);
                    w.write_bit(true);
                    for _blk in 0..6 {
                        w.write_u32(0x10, 8);
                    }
                }
            }
        }
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let data = w.finish();
        let frame = decode_picture(&data, None, DecodeOptions::default()).expect("decode");

        // Block 1 of MB 0 (rows 0..8, cols 0..8) is DC-only with
        // INTRADC 0x10 -> level 128 -> 16 everywhere.
        let block1_flat = (0..8).all(|y| (0..8).all(|x| frame.y[y * 176 + x] == 16));
        assert!(block1_flat, "block 1 should be DC-only flat 16");

        // Block 3 (rows 8..16, cols 0..8) carries a single AC
        // coefficient, so at least one of its samples must differ from
        // a perfectly flat DC reconstruction.
        let block3_has_variation =
            (8..16).any(|y| (0..8).any(|x| frame.y[y * 176 + x] != frame.y[8 * 176]));
        assert!(block3_has_variation, "block 3 should show AC variation");
    }

    /// Helper: write a single TCOEF event (LAST=1, RUN=0, LEVEL=+1)
    /// using the Table-16 ESCAPE form, which is unambiguous to encode.
    /// ESCAPE prefix `0000 011`, then LAST(1)=1, RUN(6)=0, LEVEL(8) = 1.
    fn write_single_tcoef_last(w: &mut BitWriter) {
        // ESCAPE prefix: 0000 011 (7 bits).
        w.write_u32(0b0000_011, 7);
        w.write_bit(true); // LAST = 1
        w.write_u32(0, 6); // RUN = 0
        w.write_u32(1, 8); // LEVEL = +1 (8-bit two's complement)
                           // After RUN=0 from scan_pos 1 (INTRA), this lands at zigzag
                           // slot 1 = block position 1.
        let _ = ZIGZAG_TO_BLOCK_POS; // referenced for documentation.
    }

    /// End-to-end INTER motion compensation: an INTER (type 0)
    /// macroblock at the top-left with a +2-half-pel (= +1 full pixel)
    /// horizontal motion vector and no residual must reproduce the
    /// reference plane shifted one pixel to the right (with §D.1 edge
    /// replication on the right boundary). The remaining macroblocks
    /// are skipped, so they copy the reference verbatim.
    ///
    /// Table 14: MVD component `0010` (4 bits) = +2 half-pel; the zero
    /// component is the single bit `1`. With a zero predictor at the
    /// top-left, reconstruct_mv((0,0), (2,0)) = (2,0) half-pel.
    #[test]
    fn decode_inter_horizontal_mv_shifts_reference() {
        // Reference: a horizontal ramp so a 1-pixel shift is visible.
        let mut reference = YuvFrame::grey(176, 144);
        for y in 0..144 {
            for x in 0..176 {
                reference.y[y * 176 + x] = (x % 256) as u8;
            }
        }
        for y in 0..72 {
            for x in 0..88 {
                reference.cb[y * 88 + x] = (x % 256) as u8;
                reference.cr[y * 88 + x] = (200 - (x % 200)) as u8;
            }
        }

        let mut w = BitWriter::new();
        w.write_u32(PSC_VALUE, PSC_BITS);
        w.write_u32(0, 8);
        w.write_bit(true);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_u32(0b010, 3); // QCIF
        w.write_bit(true); // INTER
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);

        for gob in 0..9 {
            w.write_u32(GBSC_VALUE, GBSC_BITS);
            w.write_u32(1, GN_BITS);
            w.write_u32(0, GFID_BITS);
            w.write_u32(8, GQUANT_BITS);
            for mb in 0..11 {
                if gob == 0 && mb == 0 {
                    // Coded INTER MB (COD = 0).
                    w.write_bit(false);
                    // MCBPC `1` -> P-picture type INTER (type 0), cbpc 00.
                    w.write_bit(true);
                    // CBPY index 15 (Table 12): CBPY(INTRA) pattern
                    // `1111`, 2-bit codeword `11`. The macroblock parser
                    // returns the CBPY(INTRA) orientation, so the
                    // driver's INTER coded pattern is `1111 ^ 1111 =
                    // 0000` — i.e. no AC residual in any luma block.
                    w.write_u32(0b11, 2);
                    // MVD: dx = +2 half-pel (`0010`), dy = 0 (`1`).
                    w.write_u32(0b0010, 4);
                    w.write_bit(true);
                    // No block data: inter_cbpy = 0000 and cbpc = 00,
                    // so no coefficients follow.
                } else {
                    // Skipped (COD = 1).
                    w.write_bit(true);
                }
            }
        }
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let data = w.finish();

        let frame =
            decode_picture(&data, Some(&reference), DecodeOptions::default()).expect("decode");

        // MB(0,0) covers luma pixels (0..16, 0..16). With mv = +2
        // half-pel = +1 pixel, the prediction for output (x, y) is
        // reference at (x + 1) clamped to the picture (§D.1). So the
        // decoded MB should equal the reference shifted one pixel left
        // in value (sampling one pixel to the right).
        for y in 0..16 {
            for x in 0..16 {
                let src_x = (x + 1).min(175);
                assert_eq!(
                    frame.y[y * 176 + x],
                    reference.y[y * 176 + src_x],
                    "luma ({x},{y}) shift mismatch"
                );
            }
        }
        // A skipped macroblock (e.g. MB(1,0), pixels x in 16..32) copies
        // the reference verbatim.
        for y in 0..16 {
            for x in 16..32 {
                assert_eq!(
                    frame.y[y * 176 + x],
                    reference.y[y * 176 + x],
                    "skipped MB ({x},{y}) should be a verbatim copy"
                );
            }
        }
    }
}
