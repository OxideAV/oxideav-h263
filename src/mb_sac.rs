//! Annex E — MB-layer SAC bridge for I- and P-pictures.
//!
//! Round 13 bridged the §E.2/§E.3 arithmetic coder (in [`crate::sac`]) into
//! the I-picture macroblock loop. Round 14 added the P-picture path: COD,
//! MCBPC (no4MVQ flavour), CBPY (raw — §E.7 indexes Table 12 directly,
//! not the XOR'd VLC representation), DQUANT, MVD components, and the
//! INTER TCOEF1/2/3/r + SIGN + LAST/RUN/LEVEL escape chain.
//!
//! Picture-header layer bytes are still written / read with the VLC bit
//! engine — §E.6 keeps the fixed-length headers (PSC, TR, PTYPE, PQUANT,
//! PEI/PSPARE) outside the PSC_FIFO multiplexer. After the byte-aligned
//! picture header closes, the body switches to SAC. GOB headers (§5.2)
//! are also fixed-length and live outside the PSC_FIFO — when present,
//! the encoder calls `encoder_flush` (§E.7) before the GOB header to
//! drain the arithmetic coder to a byte boundary, then opens a fresh SAC
//! segment after the header; the decoder mirrors with `decoder_reset`
//! (§E.3) at every GBSC.
//!
//! Per §E.7 the integer indices for each model match Tables 7 (MCBPC
//! intra), 8 (MCBPC inter), 12 (CBPY), 13 (DQUANT), 14 (MVD), 15
//! (INTRADC), 16 (TCOEF), and 17 (RUN/LEVEL FLC) of clause 5.
//!
//! Combining SAC with Annex F (Advanced Prediction / 4MV / OBMC) is
//! rejected at the encoder — `cumf_MCBPC_4MVQ` + per-block MVD wiring
//! is the next-round follow-up.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};
use oxideav_mpeg4video::headers::vol::ZIGZAG;

use crate::block::{idct_and_clip, idct_signed};
use crate::mb::IPicture;
use crate::picture::PictureHeader;
use crate::sac::{SacIPictureReader, SacIPictureWriter};
use crate::start_code::find_next_start_code;

/// Decode a SAC-coded I-picture body. `bytes` is the full picture (including
/// PSC) so we can locate the byte boundary between the (VLC-coded)
/// fixed-length header and the (SAC-coded) MB-layer body.
///
/// We re-parse the header through a private `BitReader` solely to find the
/// bit position of the first body byte, then construct a fresh `BitReader`
/// over that tail and prime the SAC decoder there.
pub fn decode_i_picture_sac(hdr: &PictureHeader, bytes: &[u8]) -> Result<IPicture> {
    let mb_w = hdr.width.div_ceil(16) as usize;
    let mb_h = hdr.height.div_ceil(16) as usize;
    let mut pic = IPicture::new(hdr.width as usize, hdr.height as usize);
    let quant = hdr.pquant as u32;

    // Find the byte offset of the picture body. We do this by re-parsing the
    // header with a throwaway bit-reader and noting where it stopped. The
    // header is guaranteed to end on a byte boundary because the encoder
    // pads to a byte before SAC starts (the only post-header field that
    // could be non-byte-aligned is the PEI/PSPARE loop, but the encoder
    // ensures byte alignment before flipping to SAC).
    let body_byte_pos = locate_body_byte_pos(bytes)?;

    // Reject non-empty GOB headers inside SAC bodies for now — the spec
    // allows them (with `decoder_reset` / `encoder_flush` boundaries) but we
    // emit a single SAC segment per picture. If any GOB headers are
    // present the body is malformed (or from a different encoder).
    let trailing = &bytes[body_byte_pos..];
    if let Some(sc) = find_next_start_code(trailing, 0) {
        // Allow the picture-end marker (next picture's PSC) but reject GBSCs
        // inside this body.
        use crate::start_code::{GN_EOS, GN_PICTURE};
        if sc.gn != GN_PICTURE && sc.gn != GN_EOS {
            return Err(Error::unsupported(
                "h263 Annex E SAC: in-body GOB headers not supported \
                 (decoder accepts a single SAC segment per picture; \
                 each GOB boundary would need a fresh decoder_reset)",
            ));
        }
    }

    let mut br = BitReader::new(trailing);
    let mut rdr = SacIPictureReader::new(&mut br)?;

    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            decode_one_intra_mb_sac(&mut rdr, mb_x, mb_y, quant, &mut pic)?;
        }
    }

    Ok(pic)
}

/// Locate the byte position immediately after the picture header,
/// rounding up to the next byte boundary. The H.263 picture header doesn't
/// itself end on a byte boundary in general (a baseline header that emits
/// the PEI=0 terminator after PQUANT+CPM is 50 bits); §E.6 / §E.5 expect
/// the SAC body's PSC_FIFO bytes to be byte-aligned, so the encoder pads
/// to byte and the decoder must skip the same padding.
fn locate_body_byte_pos(bytes: &[u8]) -> Result<usize> {
    let mut br = BitReader::new(bytes);
    let _ = crate::picture::parse_picture_header(&mut br)?;
    let bit_pos = br.bit_position();
    // Round up to the next byte boundary (§E.5 PSC_FIFO bytes are
    // byte-aligned; the encoder writes 0..=7 zero stuffing bits to land on
    // a byte boundary before flipping to SAC).
    let byte_pos = (bit_pos.div_ceil(8)) as usize;
    Ok(byte_pos)
}

/// Decode one I-picture intra macroblock through the SAC bridge. Mirrors
/// `mb::decode_intra_mb` step-for-step but reads SAC symbols instead of
/// VLC tokens.
fn decode_one_intra_mb_sac(
    rdr: &mut SacIPictureReader<'_>,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    pic: &mut IPicture,
) -> Result<()> {
    // 1. MCBPC (intra). Skip stuffing (index 8 in Table 7).
    let mcbpc_v = loop {
        let v = rdr.read_mcbpc_intra()?;
        if v != 8 {
            break v;
        }
    };
    let (is_intra_q, cbpc) = if mcbpc_v < 4 {
        (false, mcbpc_v as u8)
    } else if mcbpc_v < 8 {
        (true, (mcbpc_v - 4) as u8)
    } else {
        return Err(Error::invalid(format!(
            "h263 SAC MCBPC intra: bad value {mcbpc_v}"
        )));
    };

    // 2. CBPY (intra — pattern is direct, no XOR).
    let cbpy = rdr.read_cbpy_intra()?;

    // 3. DQUANT — for round 13 we keep `quant` constant inside the picture
    //    (mirrors the encoder, which never emits IntraQ). When the bitstream
    //    is IntraQ we still consume the 2-bit DQUANT token via SAC (per
    //    §E.7 `cumf_DQUANT`) so the rest of the MB stays aligned.
    let mut q = quant;
    if is_intra_q {
        let d = read_dquant_via(rdr)?;
        // Table 13 deltas.
        const DQUANT_DELTA: [i32; 4] = [-1, -2, 1, 2];
        let new_q = (quant as i32) + DQUANT_DELTA[d];
        q = new_q.clamp(1, 31) as u32;
    }

    // 4. Per-block: INTRADC + TCOEF (intra flavour).
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
        decode_one_intra_block_sac(rdr, block_idx, coded, mb_x, mb_y, q, pic)?;
    }

    Ok(())
}

fn decode_one_intra_block_sac(
    rdr: &mut SacIPictureReader<'_>,
    block_idx: usize,
    has_ac: bool,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    pic: &mut IPicture,
) -> Result<()> {
    // INTRADC always present for intra blocks. SAC-encoded byte → DC level.
    let dc_byte = rdr.read_intradc()?;
    let dc_level: i32 = if dc_byte == 0xFF {
        1024
    } else {
        (dc_byte as i32) << 3
    };
    let mut coeffs = [0i32; 64];
    coeffs[0] = dc_level;

    if has_ac {
        decode_intra_ac_sac(rdr, &mut coeffs, quant)?;
    }

    coeffs[0] = coeffs[0].clamp(-2048, 2047);

    let mut out = [0u8; 64];
    idct_and_clip(&mut coeffs, &mut out);

    write_block_to_picture(pic, block_idx, mb_x, mb_y, &out);
    let _ = idct_signed; // keep symbol referenced for future inter SAC path
    Ok(())
}

/// Decode the AC coefficients of an INTRA block via SAC. AC starts at scan
/// position 1 (DC is in `block[0]` already). The position counter for
/// model-selection (TCOEF1/2/3/r) advances only on *actual* coded events,
/// not on RUN-skipped zeros.
fn decode_intra_ac_sac(
    rdr: &mut SacIPictureReader<'_>,
    block: &mut [i32; 64],
    quant: u32,
) -> Result<()> {
    let mut scan_pos = 1usize;
    let mut event_pos = 1usize; // 1-based: 1=TCOEF1, 2=TCOEF2, 3=TCOEF3, ≥4=TCOEFr
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    loop {
        if scan_pos > 63 {
            return Err(Error::invalid("h263 SAC intra block: AC overrun"));
        }
        let (last, run, level_signed) = rdr.read_tcoef(true, event_pos)?;
        scan_pos = scan_pos.saturating_add(run as usize);
        if scan_pos > 63 {
            return Err(Error::invalid("h263 SAC intra block: AC run overflow"));
        }
        // Dequantise.
        let abs = level_signed.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if level_signed < 0 {
            val = -val;
        }
        let val = val.clamp(-2048, 2047);
        block[ZIGZAG[scan_pos]] = val;
        if last {
            return Ok(());
        }
        scan_pos += 1;
        event_pos += 1;
        if scan_pos > 63 {
            return Ok(());
        }
    }
}

/// Mirror of `mb::write_block_to_picture` (which is private to that module).
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

// ---------------------------------------------------------------------------
// Encoder-side SAC bridge for I-pictures.
// ---------------------------------------------------------------------------

use oxideav_core::bits::BitWriter;
use oxideav_core::frame::VideoFrame;

use crate::dct::fdct8x8;

/// Encode the body of an I-picture as a single SAC segment and append it to
/// `bw` (which must already hold the byte-aligned picture header). The SAC
/// segment itself goes through the PSC_FIFO; once flushed it is appended as
/// a sequence of bytes to the writer.
///
/// The reconstruction is also written into `recon` so the caller can use it
/// as the MC reference for subsequent P-pictures (currently not exercised
/// because P-picture SAC is deferred, but kept for symmetry with the VLC
/// path).
pub fn encode_i_picture_sac_body(
    bw: &mut BitWriter,
    width: u32,
    height: u32,
    pquant: u8,
    frame: &VideoFrame,
    recon: &mut IPicture,
) -> Result<()> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;

    let mut sac = SacIPictureWriter::new();

    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            encode_intra_mb_sac(&mut sac, mb_x, mb_y, pquant, frame, recon)?;
        }
    }

    let body_bytes = sac.finish();
    if !bw.is_byte_aligned() {
        return Err(Error::invalid(
            "h263 SAC encoder: header writer must be byte-aligned before SAC body",
        ));
    }
    for b in &body_bytes {
        bw.write_bits(*b as u32, 8);
    }
    Ok(())
}

fn encode_intra_mb_sac(
    sac: &mut SacIPictureWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    recon: &mut IPicture,
) -> Result<()> {
    // 1. Pull samples for all 6 blocks, run forward DCT + intra quantiser,
    //    build CBP. Mirrors `encoder::encode_intra_mb` for the VLC path.
    let mut blocks = [[0i32; 64]; 6];
    let mut dc_pels = [128u8; 6];
    let mut block_has_ac = [false; 6];

    for b in 0..6 {
        let mut samples = [0.0f32; 64];
        sample_block_for(frame, mb_x, mb_y, b, &mut samples);
        let mut dctf = samples;
        fdct8x8(&mut dctf);
        let (dc_byte, levels, any_ac) = quantise_intra_block(&dctf, quant);
        dc_pels[b] = dc_byte;
        block_has_ac[b] = any_ac;
        blocks[b] = levels;
    }

    // 2. CBPC + CBPY.
    let cbpc: u8 = ((block_has_ac[4] as u8) << 1) | (block_has_ac[5] as u8);
    let cbpy: u8 = ((block_has_ac[0] as u8) << 3)
        | ((block_has_ac[1] as u8) << 2)
        | ((block_has_ac[2] as u8) << 1)
        | (block_has_ac[3] as u8);

    // 3. MCBPC intra — we always emit mb_type=3 (Intra), so the Table 7
    //    index is just `cbpc` (0..=3). Index 4..=7 = IntraQ, 8 = stuffing.
    sac.write_mcbpc_intra(cbpc as usize);

    // 4. CBPY intra — direct pattern.
    sac.write_cbpy_intra(cbpy);

    // 5. Per-block INTRADC + (optionally) AC TCOEF + reconstruction.
    for b in 0..6 {
        sac.write_intradc(dc_pels[b])?;
        if block_has_ac[b] {
            write_block_ac_sac_intra(sac, &blocks[b]);
        }
        reconstruct_intra_block(recon, b, mb_x, mb_y, dc_pels[b], &blocks[b], quant);
    }

    Ok(())
}

/// Mirror of `encoder::quantise_intra_block`.
fn quantise_intra_block(dctf: &[f32; 64], quant: u8) -> (u8, [i32; 64], bool) {
    let dc_round = (dctf[0] / 8.0).round() as i32;
    let dc_clamped = dc_round.clamp(1, 254);
    let dc_byte: u8 = if dc_clamped == 128 {
        0xFF
    } else {
        dc_clamped as u8
    };
    let mut levels = [0i32; 64];
    let q = quant as i32;
    let two_q = 2 * q;
    let bias = q / 4;
    for k in 1..64 {
        let coef = dctf[k];
        let abs_f = coef.abs() as i32;
        let mag = (abs_f + bias) / two_q;
        if mag != 0 {
            let signed = if coef < 0.0 { -mag } else { mag };
            levels[k] = signed.clamp(-127, 127);
        }
    }
    let any_ac = levels.iter().skip(1).any(|&l| l != 0);
    (dc_byte, levels, any_ac)
}

/// Mirror of `encoder::sample_block_for`. Replicated locally so the encoder
/// module doesn't need to expose its private helper.
fn sample_block_for(
    frame: &VideoFrame,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    out: &mut [f32; 64],
) {
    let (plane, stride, base_x, base_y, max_x, max_y) = match block_idx {
        0..=3 => {
            let x = mb_x * 16 + if block_idx & 1 == 1 { 8 } else { 0 };
            let y = mb_y * 16 + if block_idx & 2 == 2 { 8 } else { 0 };
            let p = &frame.planes[0];
            (
                p.data.as_slice(),
                p.stride,
                x,
                y,
                frame.width as usize,
                frame.height as usize,
            )
        }
        4 => {
            let x = mb_x * 8;
            let y = mb_y * 8;
            let p = &frame.planes[1];
            let cw = (frame.width as usize).div_ceil(2);
            let ch = (frame.height as usize).div_ceil(2);
            (p.data.as_slice(), p.stride, x, y, cw, ch)
        }
        5 => {
            let x = mb_x * 8;
            let y = mb_y * 8;
            let p = &frame.planes[2];
            let cw = (frame.width as usize).div_ceil(2);
            let ch = (frame.height as usize).div_ceil(2);
            (p.data.as_slice(), p.stride, x, y, cw, ch)
        }
        _ => unreachable!(),
    };
    for j in 0..8 {
        let yy = (base_y + j).min(max_y.saturating_sub(1));
        for i in 0..8 {
            let xx = (base_x + i).min(max_x.saturating_sub(1));
            out[j * 8 + i] = plane[yy * stride + xx] as f32;
        }
    }
}

/// Emit the AC coefficients of an INTRA block as SAC events. Mirrors
/// `encoder::write_block_ac` but routes through `SacIPictureWriter`. Caller
/// guarantees at least one nonzero in `levels[1..]`.
fn write_block_ac_sac_intra(sac: &mut SacIPictureWriter, levels: &[i32; 64]) {
    let mut nonzero_zz: Vec<(usize, i32)> = Vec::with_capacity(8);
    for zz in 1..64 {
        let nat = ZIGZAG[zz];
        let lv = levels[nat];
        if lv != 0 {
            nonzero_zz.push((zz, lv));
        }
    }
    debug_assert!(!nonzero_zz.is_empty());

    let mut prev_zz: usize = 0;
    let total = nonzero_zz.len();
    for (event_pos, (i, &(zz, lv))) in (1usize..).zip(nonzero_zz.iter().enumerate()) {
        let run = if i == 0 {
            (zz - 1) as u8
        } else {
            (zz - prev_zz - 1) as u8
        };
        let last = i == total - 1;
        sac.write_tcoef(true, event_pos, last, run, lv);
        prev_zz = zz;
    }
}

/// Mirror of `encoder::reconstruct_intra_block`. Uses the same dequant +
/// IDCT path the decoder runs.
fn reconstruct_intra_block(
    recon: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    dc_byte: u8,
    levels: &[i32; 64],
    quant: u8,
) {
    let mut coeffs = dequantise_block(levels, quant, true);
    coeffs[0] = if dc_byte == 0xFF {
        1024
    } else {
        (dc_byte as i32) << 3
    };
    coeffs[0] = coeffs[0].clamp(-2048, 2047);
    let mut out = [0u8; 64];
    idct_and_clip(&mut coeffs, &mut out);
    write_block_into(recon, block_idx, mb_x, mb_y, &out);
}

fn dequantise_block(levels: &[i32; 64], quant: u8, skip_dc: bool) -> [i32; 64] {
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    let mut out = [0i32; 64];
    let start = if skip_dc { 1 } else { 0 };
    for k in start..64 {
        let l = levels[k];
        if l == 0 {
            continue;
        }
        let abs = l.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if l < 0 {
            val = -val;
        }
        out[k] = val.clamp(-2048, 2047);
    }
    out
}

fn write_block_into(
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

/// Read a 2-bit DQUANT code (Table 13 index 0..=3) from a SAC reader. Free
/// function rather than a method on the reader so this module owns the
/// helper that's only used here (the reader stays a minimal wrapper).
fn read_dquant_via(rdr: &mut SacIPictureReader<'_>) -> Result<usize> {
    rdr.decode_with_model(crate::sac::models::DQUANT)
}

// ---------------------------------------------------------------------------
// Round 14 — SAC bridge for P-picture bodies. Encoder and decoder.
//
// The encoder side (`encode_p_picture_sac_body`) mirrors the VLC encoder's
// single-pass path (`encoder::encode_p_mb`) but routes every MB-layer
// element through `SacPPictureWriter`. The reconstruction is written into
// `recon` so the next-picture's MC reference is bit-identical to what the
// decoder produces from the same SAC body.
//
// The decoder side (`decode_p_picture_sac`) mirrors `decoder::decode_p_picture`
// (single-pass non-AP variant) and reads SAC tokens via `SacPPictureReader`.
//
// GOB boundaries are wired through §E.5/§E.6: each non-empty GOB header is
// preceded by `encoder_flush` + a byte-aligned PSC_FIFO drain, and followed
// by a fresh SAC segment with `decoder_reset` (a brand-new
// SacPPictureWriter / SacPPictureReader instance for the next GOB).
// ---------------------------------------------------------------------------

use crate::interp::predict_block;
use crate::motion::{luma_to_chroma_mv, predict_mv, wrap_mv_component, MbMotion, MvGrid};
use crate::sac::{PMcbpcModel, SacPPictureReader, SacPPictureWriter};
use oxideav_mpeg4video::tables::mcbpc::PMbType;

/// Encode the body of a P-picture as one or more SAC segments and append
/// them to `bw`. When `emit_gob_headers` is true, every GOB boundary
/// (one per MB-row group per the source format, except for the first GOB)
/// triggers an `encoder_flush` + GOB header (VLC) + fresh SAC segment,
/// mirroring §E.5. When false, the picture body is emitted as a single
/// SAC segment with no in-body resync points (matches the VLC P-encoder's
/// "no GOB headers in P-pictures" baseline behaviour and so produces a
/// byte-identical reconstruction).
///
/// Reconstruction is written into `recon` for use as the next picture's MC
/// reference.
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_sac_body(
    bw: &mut BitWriter,
    width: u32,
    height: u32,
    pquant: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    recon: &mut IPicture,
    mb_rows_per_gob: u32,
    emit_gob_headers: bool,
) -> Result<()> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;

    let mut mv_grid = MvGrid::new(mb_w, mb_h);
    // Round 14 keeps `cumf_MCBPC_no4MVQ` because we don't emit Annex F or
    // Annex J in the SAC P-path. Per §E.7, the moment AP/DF turns on this
    // selector flips to `cumf_MCBPC_4MVQ`; the encoder does not currently
    // combine SAC with AP/DF.
    let mcbpc_model = PMcbpcModel::No4MvQ;
    let mut sac = SacPPictureWriter::new(mcbpc_model);

    for mb_y in 0..mb_h {
        if emit_gob_headers && mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            // §E.6 / §E.5 boundary: flush the current SAC segment to a
            // byte boundary, emit the GOB header through the VLC bit
            // writer, then start a fresh SAC segment for the next group.
            sac.encoder_flush();
            let segment_bytes = sac.take_byte_aligned_bytes();
            for b in &segment_bytes {
                bw.write_bits(*b as u32, 8);
            }
            // Emit GOB header (already byte-aligned now). `write_gob_header`
            // includes its own pad-to-byte before the GBSC.
            let gn = (mb_y as u32 / mb_rows_per_gob) as u8;
            crate::encoder::write_gob_header_pub(bw, gn, pquant)?;
            // Pad back to byte boundary so the next SAC segment's PSC_FIFO
            // bytes start clean (§E.6 — header layer bypasses PSC_FIFO and
            // the SAC stream is byte-aligned at every fixed-length-header
            // boundary).
            while !bw.is_byte_aligned() {
                bw.write_bits(0, 1);
            }
            // Reset MV predictor (§5.3.7.2 — non-empty GOB header clears the
            // grid).
            mv_grid = MvGrid::new(mb_w, mb_h);
            // Fresh SAC segment.
            sac = SacPPictureWriter::new(mcbpc_model);
        }
        for mb_x in 0..mb_w {
            encode_p_mb_sac(
                &mut sac,
                mb_x,
                mb_y,
                pquant,
                frame,
                reference,
                recon,
                &mut mv_grid,
            )?;
        }
    }

    // Final flush + drain.
    let body_bytes = sac.finish();
    if !bw.is_byte_aligned() {
        return Err(Error::invalid(
            "h263 SAC P encoder: bit writer not byte-aligned before final body drain",
        ));
    }
    for b in &body_bytes {
        bw.write_bits(*b as u32, 8);
    }
    Ok(())
}

/// Encode one P-MB through the SAC writer. Mirrors `encoder::encode_p_mb`
/// in structure: motion estimate → skip / intra / inter decision → emit
/// SAC tokens → reconstruct.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_sac(
    sac: &mut SacPPictureWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    recon: &mut IPicture,
    mv_grid: &mut MvGrid,
) -> Result<()> {
    // 1. Motion-estimate luma 16×16. Reuse the encoder's helper.
    let (mvx, mvy, mv_sad) = crate::encoder::motion_estimate_mb_pub(frame, reference, mb_x, mb_y);

    // Zero-MV SAD for skip decision.
    let zero_sad = crate::encoder::sad_block_pub(
        &frame.planes[0].data,
        frame.planes[0].stride,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        &reference.y,
        reference.y_stride,
        reference.y_stride as i32,
        (reference.y.len() / reference.y_stride) as i32,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        0,
        0,
        16,
    );
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    let can_skip = pmx == 0 && pmy == 0 && zero_sad < mv_sad + 128;

    let decide_mv = if can_skip { (0, 0) } else { (mvx, mvy) };

    // Build predictor.
    let mut y_pred = [0u8; 256];
    let mut u_pred = [0u8; 64];
    let mut v_pred = [0u8; 64];
    crate::encoder::build_mb_predictor_pub(
        reference,
        mb_x,
        mb_y,
        decide_mv.0,
        decide_mv.1,
        &mut y_pred,
        &mut u_pred,
        &mut v_pred,
    );

    let src_y = &frame.planes[0];
    let src_cb = &frame.planes[1];
    let src_cr = &frame.planes[2];
    let mut luma_abs_sum = 0u32;
    for j in 0..16 {
        for i in 0..16 {
            let s = src_y.data[(mb_y * 16 + j) * src_y.stride + (mb_x * 16 + i)] as i32;
            let p = y_pred[j * 16 + i] as i32;
            luma_abs_sum += (s - p).unsigned_abs();
        }
    }

    let intra_variance = crate::encoder::mb_luma_variance_pub(src_y, mb_x, mb_y);
    let try_intra = intra_variance * 5 < luma_abs_sum;

    if can_skip && luma_abs_sum < (quant as u32) * 128 {
        // COD=1 — skipped MB.
        sac.write_cod(false);
        crate::encoder::copy_predictor_to_recon_pub(recon, mb_x, mb_y, &y_pred, &u_pred, &v_pred);
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        return Ok(());
    }

    // COD=0 — coded MB.
    sac.write_cod(true);

    if try_intra {
        encode_p_mb_intra_sac(sac, mb_x, mb_y, quant, frame, recon)?;
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
        return Ok(());
    }

    // Inter path.
    encode_p_mb_inter_sac(
        sac, mb_x, mb_y, quant, src_y, src_cb, src_cr, recon, decide_mv, mv_grid, &y_pred, &u_pred,
        &v_pred,
    )
}

/// Intra encode of a P-MB via SAC. Index 4 (mb_type=1, cbpc=0) base in
/// Table 8 → `4 + cbpc` for cbpc 0..=3.
fn encode_p_mb_intra_sac(
    sac: &mut SacPPictureWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    recon: &mut IPicture,
) -> Result<()> {
    let mut blocks = [[0i32; 64]; 6];
    let mut dc_pels = [128u8; 6];
    let mut block_has_ac = [false; 6];

    for b in 0..6 {
        let mut samples = [0.0f32; 64];
        sample_block_for(frame, mb_x, mb_y, b, &mut samples);
        let mut dctf = samples;
        fdct8x8(&mut dctf);
        let (dc_byte, levels, any_ac) = quantise_intra_block(&dctf, quant);
        dc_pels[b] = dc_byte;
        block_has_ac[b] = any_ac;
        blocks[b] = levels;
    }
    let cbpc: u8 = ((block_has_ac[4] as u8) << 1) | (block_has_ac[5] as u8);
    let cbpy: u8 = ((block_has_ac[0] as u8) << 3)
        | ((block_has_ac[1] as u8) << 2)
        | ((block_has_ac[2] as u8) << 1)
        | (block_has_ac[3] as u8);

    // Table 8 mb_type=1 (Intra) → indices 4..=7. We never emit IntraQ
    // (we hold quant constant inside a picture, same as the I-path).
    let mcbpc_index = 4 + cbpc as usize;
    sac.write_mcbpc(mcbpc_index)?;
    // CBPY for intra-in-P uses the CBPY_intra model — but the §E.7 inter
    // CBPY model is what the table indexes call out. Strictly the spec
    // says "cumf_CBPY_intra in INTRA macroblocks" — including those
    // embedded in P-pictures. We use the intra CBPY model for intra MBs
    // in P-pictures here.
    sac.enc_intra_cbpy(cbpy)?;
    for b in 0..6 {
        sac.enc_intradc(dc_pels[b])?;
        if block_has_ac[b] {
            for ev in tcoef_intra_events(&blocks[b]) {
                sac.write_tcoef_inter_intra_pos(ev);
            }
        }
        reconstruct_intra_block(recon, b, mb_x, mb_y, dc_pels[b], &blocks[b], quant);
    }
    Ok(())
}

/// Inter encode of a P-MB via SAC.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_inter_sac(
    sac: &mut SacPPictureWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    src_y: &oxideav_core::frame::VideoPlane,
    src_cb: &oxideav_core::frame::VideoPlane,
    src_cr: &oxideav_core::frame::VideoPlane,
    recon: &mut IPicture,
    mv: (i32, i32),
    mv_grid: &mut MvGrid,
    y_pred: &[u8; 256],
    u_pred: &[u8; 64],
    v_pred: &[u8; 64],
) -> Result<()> {
    let mut levels_all = [[0i32; 64]; 6];
    let mut has_ac = [false; 6];

    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 16 + sub_y + j;
                let px = mb_x * 16 + sub_x + i;
                let s = src_y.data[py * src_y.stride + px] as i32;
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = crate::encoder::quantise_inter_block_pub(&dctf, quant);
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }
    for (ci, plane) in [(0, src_cb), (1, src_cr)].iter() {
        let pred = if *ci == 0 { u_pred } else { v_pred };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 8 + j;
                let px = mb_x * 8 + i;
                let s = plane.data[py * plane.stride + px] as i32;
                let p = pred[j * 8 + i] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = crate::encoder::quantise_inter_block_pub(&dctf, quant);
        let b = 4 + ci;
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }

    let cbpc: u8 = ((has_ac[4] as u8) << 1) | (has_ac[5] as u8);
    let cbpy_true: u8 = ((has_ac[0] as u8) << 3)
        | ((has_ac[1] as u8) << 2)
        | ((has_ac[2] as u8) << 1)
        | (has_ac[3] as u8);

    // Table 8 mb_type=0 (Inter) → indices 0..=3. We never emit InterQ
    // (we hold quant constant).
    let mcbpc_index = cbpc as usize;
    sac.write_mcbpc(mcbpc_index)?;
    // §E.7: cumf_CBPY for INTER MBs uses Table 12 indexing on the RAW
    // pattern (NOT the XOR'd VLC representation).
    sac.write_cbpy_inter(cbpy_true)?;

    // MV — predictor + diff (folded), emit two MVD components.
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    let diff_x = fold_mvd(mv.0 - pmx);
    let diff_y = fold_mvd(mv.1 - pmy);
    debug_assert_eq!(wrap_mv_component(pmx + diff_x), mv.0);
    debug_assert_eq!(wrap_mv_component(pmy + diff_y), mv.1);
    sac.write_mvd_component(diff_x);
    sac.write_mvd_component(diff_y);

    // AC per coded block.
    for b in 0..6 {
        if has_ac[b] {
            for ev in tcoef_inter_events(&levels_all[b]) {
                sac.write_tcoef_inter(ev.position, ev.last, ev.run, ev.level);
            }
        }
    }

    // Reconstruct (predictor + dequant residual IDCT + clip), like
    // `encode_p_mb_inter`.
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let coeffs = crate::encoder::dequantise_block_pub(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst_for(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
    }
    for ci in 0..2usize {
        let b = 4 + ci;
        let pred = if ci == 0 { u_pred } else { v_pred };
        let coeffs = crate::encoder::dequantise_block_pub(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst_for(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = pred[j * 8 + i] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
    }

    mv_grid.set(mb_x, mb_y, MbMotion::mv1(mv, true, false));
    Ok(())
}

/// Same `block_dst` as in `mb.rs`, replicated here so we don't need to
/// expose it to the world.
fn block_dst_for(
    pic: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
) -> (&mut [u8], usize, usize, usize) {
    block_dst(pic, block_idx, mb_x, mb_y)
}

/// Fold `raw` into `[-32, +31]` halfpel — same wrap as the VLC path's
/// `encode_mv_component`.
fn fold_mvd(raw: i32) -> i32 {
    let mut d = raw;
    while d < -32 {
        d += 64;
    }
    while d > 31 {
        d -= 64;
    }
    d
}

/// Inter TCOEF event triple as the SAC writer wants it.
struct InterEvent {
    position: usize,
    last: bool,
    run: u8,
    level: i32,
}

/// Walk `levels` (natural order) in zig-zag and emit `InterEvent`s. Inter
/// blocks start at scan position 0 (no DC special-case).
fn tcoef_inter_events(levels: &[i32; 64]) -> Vec<InterEvent> {
    let mut nz: Vec<(usize, i32)> = Vec::with_capacity(8);
    for zz in 0..64 {
        let nat = ZIGZAG[zz];
        if levels[nat] != 0 {
            nz.push((zz, levels[nat]));
        }
    }
    let total = nz.len();
    let mut out = Vec::with_capacity(total);
    let mut prev_zz: i32 = -1;
    for (i, &(zz, lv)) in nz.iter().enumerate() {
        let run = (zz as i32 - prev_zz - 1) as u8;
        let last = i == total - 1;
        out.push(InterEvent {
            position: i + 1,
            last,
            run,
            level: lv,
        });
        prev_zz = zz as i32;
    }
    out
}

/// Same as `write_block_ac_sac_intra` upstream but emits a `Vec` of
/// per-event records; intra event positions skip DC (start at scan 1).
fn tcoef_intra_events(levels: &[i32; 64]) -> Vec<IntraEvent> {
    let mut nz: Vec<(usize, i32)> = Vec::with_capacity(8);
    for zz in 1..64 {
        let nat = ZIGZAG[zz];
        if levels[nat] != 0 {
            nz.push((zz, levels[nat]));
        }
    }
    let total = nz.len();
    let mut out = Vec::with_capacity(total);
    let mut prev_zz: usize = 0;
    for (i, &(zz, lv)) in nz.iter().enumerate() {
        let run = (zz - prev_zz - 1) as u8;
        let last = i == total - 1;
        out.push(IntraEvent {
            position: i + 1,
            last,
            run,
            level: lv,
        });
        prev_zz = zz;
    }
    out
}

struct IntraEvent {
    position: usize,
    last: bool,
    run: u8,
    level: i32,
}

// We stash a couple of intra-flavoured helpers on `SacPPictureWriter` to
// keep the surface narrow. They just dispatch to the lower-level encoder
// using the §E.7 INTRA models (CBPY_intra, INTRADC, TCOEF*_intra, SIGN,
// LAST_intra / RUN_intra / LEVEL_intra).
impl SacPPictureWriter {
    /// Encode the CBPY for an intra-in-P MB (§E.7 — uses cumf_CBPY_intra).
    pub fn enc_intra_cbpy(&mut self, cbpy: u8) -> Result<()> {
        if cbpy >= 16 {
            return Err(Error::invalid("SAC CBPY_intra: value out of 0..=15"));
        }
        self.encode_with(crate::sac::models::CBPY_INTRA, cbpy as usize);
        Ok(())
    }

    /// Encode the INTRADC byte for an intra-in-P MB.
    pub fn enc_intradc(&mut self, byte: u8) -> Result<()> {
        let idx = crate::sac::intradc_byte_to_index(byte)
            .ok_or_else(|| Error::invalid("SAC INTRADC P/intra: illegal byte 0x00 / 0x80"))?;
        self.encode_with(crate::sac::models::INTRADC, idx);
        Ok(())
    }

    /// Encode one intra-in-P TCOEF event using the INTRA TCOEF / SIGN /
    /// LAST_intra / RUN_intra / LEVEL_intra models per §E.7.
    fn write_tcoef_inter_intra_pos(&mut self, ev: IntraEvent) {
        let abs = ev.level.unsigned_abs();
        let tcoef_model: &[u32] = match ev.position {
            1 => crate::sac::models::TCOEF1_INTRA,
            2 => crate::sac::models::TCOEF2_INTRA,
            3 => crate::sac::models::TCOEF3_INTRA,
            _ => crate::sac::models::TCOEFR_INTRA,
        };
        let sign_bit: usize = if ev.level < 0 { 1 } else { 0 };
        if abs <= 12 {
            if let Some(idx) = crate::sac::tcoef_lookup_index(ev.last, ev.run, abs as u8) {
                self.encode_with(tcoef_model, idx);
                self.encode_with(crate::sac::models::SIGN, sign_bit);
                return;
            }
        }
        // ESCAPE.
        self.encode_with(tcoef_model, crate::sac::TCOEF_ESCAPE_INDEX);
        self.encode_with(crate::sac::models::LAST_INTRA, if ev.last { 1 } else { 0 });
        self.encode_with(crate::sac::models::RUN_INTRA, ev.run as usize);
        let level_byte: u8 = ev.level.rem_euclid(256) as u8;
        let level_idx = crate::sac::level_byte_to_index(level_byte);
        self.encode_with(crate::sac::models::LEVEL_INTRA, level_idx);
    }

    /// Tiny pass-through used by the intra-in-P helpers above so the
    /// writer's encoder/fifo stay private to `sac.rs`.
    fn encode_with(&mut self, model: &'static [u32], idx: usize) {
        self.encode_symbol_via(model, idx);
    }
}

// ---------------------------------------------------------------------------
// Decoder side — P-picture SAC body driver.
// ---------------------------------------------------------------------------

use crate::start_code::{GN_EOS, GN_PICTURE};

/// Decode a SAC-coded P-picture body. Mirror of `decoder::decode_p_picture`
/// (single-pass, non-AP path) but pulls SAC tokens via `SacPPictureReader`.
///
/// `bytes` is the full picture (PSC + body) so we can locate the post-
/// header byte boundary and any GOB headers.
pub fn decode_p_picture_sac(
    hdr: &PictureHeader,
    bytes: &[u8],
    reference: &IPicture,
) -> Result<IPicture> {
    let mb_w = hdr.width.div_ceil(16) as usize;
    let mb_h = hdr.height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = hdr
        .source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 SAC P: source format has no GOB layout"))?;

    let mut pic = IPicture::new(hdr.width as usize, hdr.height as usize);
    let mut quant = hdr.pquant as u32;

    let body_byte_pos = locate_body_byte_pos(bytes)?;

    // Pre-locate every GBSC in the body (excluding the picture's own PSC).
    let trailing = &bytes[body_byte_pos..];
    let mut gob_offsets: Vec<usize> = Vec::new();
    let mut pos = 0usize;
    while let Some(sc) = find_next_start_code(trailing, pos) {
        if sc.gn != GN_PICTURE && sc.gn != GN_EOS {
            gob_offsets.push(sc.byte_pos);
        }
        pos = sc.byte_pos + 3;
    }
    // Sentinel — past the body.
    gob_offsets.push(trailing.len());

    let mcbpc_model = PMcbpcModel::No4MvQ;
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    // Walk the body in segments. Each segment runs from a byte offset
    // (start of body, or just past a GOB header) to the next GBSC byte
    // offset (sentinel = end of body).
    //
    // For SAC, the encoder calls `encoder_flush` immediately before each
    // GOB header. The decoder must call `decoder_reset` immediately after
    // it consumes the GOB header (i.e., construct a fresh
    // `SacPPictureReader` over the byte tail starting at the next byte
    // after the GOB header).
    let mut seg_start = 0usize;
    let mut next_gob_idx = 0usize;
    let mut mb_y = 0usize;

    while mb_y < mb_h {
        let seg_end = gob_offsets[next_gob_idx];
        let segment = &trailing[seg_start..seg_end];

        // Determine how many MB rows live in this segment. If there are
        // more GOB headers ahead, this segment covers exactly one GOB
        // (`mb_rows_per_gob` rows). Otherwise (last/only segment) it
        // covers all remaining rows — the encoder may have elided every
        // GOB header (the VLC P-encoder does, and our SAC P-encoder does
        // too unless `emit_gob_headers` is set).
        let rows_left = mb_h - mb_y;
        let has_more_gobs = next_gob_idx < gob_offsets.len() - 1;
        let rows_in_seg = if has_more_gobs {
            (mb_rows_per_gob as usize).min(rows_left)
        } else {
            rows_left
        };

        let mut br = BitReader::new(segment);
        let mut rdr = SacPPictureReader::new(&mut br, mcbpc_model)?;

        for _ in 0..rows_in_seg {
            for mb_x in 0..mb_w {
                decode_p_mb_sac(
                    &mut rdr,
                    mb_x,
                    mb_y,
                    &mut quant,
                    &mut pic,
                    reference,
                    &mut mv_grid,
                )?;
            }
            mb_y += 1;
        }

        // Advance to next segment: consume the GBSC if there is one.
        if next_gob_idx < gob_offsets.len() - 1 {
            // The encoder put a GOB header at byte offset `seg_end` of
            // `trailing`. Parse it via a temporary VLC bit reader and
            // refresh `quant`.
            let gob_seg = &trailing[seg_end..];
            let mut gob_br = BitReader::new(gob_seg);
            let gob = crate::gob::parse_gob_header(&mut gob_br, hdr.cpm)?;
            quant = gob.gquant as u32;
            // Reset MV predictor at GOB boundary (§5.3.7.2).
            mv_grid = MvGrid::new(mb_w, mb_h);
            // Compute byte offset just past the GOB header (round up).
            let bit_pos_after = gob_br.bit_position();
            let bytes_consumed = bit_pos_after.div_ceil(8) as usize;
            seg_start = seg_end + bytes_consumed;
            next_gob_idx += 1;
        } else {
            break;
        }
    }

    Ok(pic)
}

fn decode_p_mb_sac(
    rdr: &mut SacPPictureReader<'_>,
    mb_x: usize,
    mb_y: usize,
    quant: &mut u32,
    pic: &mut IPicture,
    reference: &IPicture,
    mv_grid: &mut MvGrid,
) -> Result<()> {
    // 1. COD.
    let coded = rdr.read_cod()?;
    if !coded {
        // Skipped — copy MV(0,0) predictor into pic.
        copy_skipped_mb(pic, reference, mb_x, mb_y);
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        return Ok(());
    }

    // 2. MCBPC — loop over stuffing.
    let mcbpc_v = loop {
        let v = rdr.read_mcbpc()?;
        if v != PMcbpcModel::STUFFING_INDEX {
            break v;
        }
    };
    let (mb_type, cbpc) = decompose_mcbpc_for_no4mvq(mcbpc_v)?;
    let is_intra = matches!(mb_type, PMbType::Intra | PMbType::IntraQ);
    let needs_dquant = matches!(
        mb_type,
        PMbType::InterQ | PMbType::IntraQ | PMbType::Inter4MVQ
    );

    // 3. CBPY — intra MBs use CBPY_intra (raw); inter MBs use CBPY (raw,
    //    NOT XOR'd per §E.7).
    let cbpy = if is_intra {
        rdr.read_cbpy_intra_via()?
    } else {
        rdr.read_cbpy_inter()?
    };

    // 4. DQUANT.
    if needs_dquant {
        let d = rdr.read_dquant()?;
        const DQUANT_DELTA: [i32; 4] = [-1, -2, 1, 2];
        let new_q = (*quant as i32) + DQUANT_DELTA[d];
        *quant = new_q.clamp(1, 31) as u32;
    }

    // 5. MVs. For Inter / InterQ we read 2 MVDs and reconstruct via
    //    median predictor.
    let mb_mv: (i32, i32) = if is_intra {
        // Intra-in-P: MV (0, 0), no MVDs.
        (0, 0)
    } else {
        let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
        let dx = rdr.read_mvd_component()?;
        let dy = rdr.read_mvd_component()?;
        let mvx = wrap_mv_component(pmx + dx);
        let mvy = wrap_mv_component(pmy + dy);
        (mvx, mvy)
    };

    // 6. Per-block AC.
    let luma_coded = [
        (cbpy >> 3) & 1 != 0,
        (cbpy >> 2) & 1 != 0,
        (cbpy >> 1) & 1 != 0,
        cbpy & 1 != 0,
    ];
    let chroma_coded = [(cbpc >> 1) & 1 != 0, cbpc & 1 != 0];

    if is_intra {
        for block_idx in 0..6usize {
            let coded_b = if block_idx < 4 {
                luma_coded[block_idx]
            } else {
                chroma_coded[block_idx - 4]
            };
            decode_one_intra_block_sac_in_p(rdr, block_idx, coded_b, mb_x, mb_y, *quant, pic)?;
        }
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
        return Ok(());
    }

    // Inter MB: build predictor, decode AC residuals, add.
    mv_grid.set(mb_x, mb_y, MbMotion::mv1(mb_mv, true, false));

    for block_idx in 0..4usize {
        let coded_b = luma_coded[block_idx];
        let mut coeffs = [0i32; 64];
        if coded_b {
            decode_inter_ac_sac(rdr, &mut coeffs, *quant)?;
        }
        let (sub_x, sub_y) = match block_idx {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let blk_px = (mb_x * 16 + sub_x) as i32;
        let blk_py = (mb_y * 16 + sub_y) as i32;
        let ref_y_h = reference.y.len() / reference.y_stride;
        let mut pred = [0u8; 64];
        predict_block(
            &reference.y,
            reference.y_stride,
            reference.y_stride as i32,
            ref_y_h as i32,
            blk_px,
            blk_py,
            mb_mv.0,
            mb_mv.1,
            8,
            &mut pred,
            8,
        );
        let mut resid_out = [0i32; 64];
        if coded_b {
            crate::block::idct_signed(&mut coeffs, &mut resid_out);
        }
        let (plane, stride, px, py) = block_dst_for(pic, block_idx, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = pred[j * 8 + i] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
    }

    let cmx = luma_to_chroma_mv(mb_mv.0);
    let cmy = luma_to_chroma_mv(mb_mv.1);
    let ref_c_h = reference.cb.len() / reference.c_stride;
    for ci in 0..2usize {
        let block_idx = 4 + ci;
        let coded_b = chroma_coded[ci];
        let mut coeffs = [0i32; 64];
        if coded_b {
            decode_inter_ac_sac(rdr, &mut coeffs, *quant)?;
        }
        let (ref_plane, ref_stride) = if ci == 0 {
            (&reference.cb, reference.c_stride)
        } else {
            (&reference.cr, reference.c_stride)
        };
        let mut pred = [0u8; 64];
        let blk_px = (mb_x * 8) as i32;
        let blk_py = (mb_y * 8) as i32;
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
        let mut resid_out = [0i32; 64];
        if coded_b {
            crate::block::idct_signed(&mut coeffs, &mut resid_out);
        }
        let (plane, stride, px, py) = block_dst_for(pic, block_idx, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = pred[j * 8 + i] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
    }
    Ok(())
}

/// Decode a no-MV "skipped" P-MB by copying the reference block at the
/// same position with MV=(0,0).
fn copy_skipped_mb(pic: &mut IPicture, reference: &IPicture, mb_x: usize, mb_y: usize) {
    // Luma 16×16.
    let base_x = mb_x * 16;
    let base_y = mb_y * 16;
    for j in 0..16usize {
        let off_dst = (base_y + j) * pic.y_stride + base_x;
        let off_src = (base_y + j) * reference.y_stride + base_x;
        pic.y[off_dst..off_dst + 16].copy_from_slice(&reference.y[off_src..off_src + 16]);
    }
    // Chroma 8×8.
    let cbx = mb_x * 8;
    let cby = mb_y * 8;
    for j in 0..8usize {
        let off_dst = (cby + j) * pic.c_stride + cbx;
        let off_src = (cby + j) * reference.c_stride + cbx;
        pic.cb[off_dst..off_dst + 8].copy_from_slice(&reference.cb[off_src..off_src + 8]);
        pic.cr[off_dst..off_dst + 8].copy_from_slice(&reference.cr[off_src..off_src + 8]);
    }
}

/// Decode an INTRA block embedded in a P-picture using SAC. Mirrors
/// `decode_one_intra_block_sac` from the I-path but uses the P reader.
fn decode_one_intra_block_sac_in_p(
    rdr: &mut SacPPictureReader<'_>,
    block_idx: usize,
    has_ac: bool,
    mb_x: usize,
    mb_y: usize,
    quant: u32,
    pic: &mut IPicture,
) -> Result<()> {
    let dc_byte = rdr.read_intradc_via()?;
    let dc_level: i32 = if dc_byte == 0xFF {
        1024
    } else {
        (dc_byte as i32) << 3
    };
    let mut coeffs = [0i32; 64];
    coeffs[0] = dc_level;
    if has_ac {
        decode_intra_ac_sac_in_p(rdr, &mut coeffs, quant)?;
    }
    coeffs[0] = coeffs[0].clamp(-2048, 2047);
    let mut out = [0u8; 64];
    idct_and_clip(&mut coeffs, &mut out);
    let (plane, stride, px, py) = block_dst_for(pic, block_idx, mb_x, mb_y);
    for dy in 0..8 {
        for dx in 0..8 {
            plane[(py + dy) * stride + (px + dx)] = out[dy * 8 + dx];
        }
    }
    Ok(())
}

/// Inter AC: scan from position 0, read TCOEF events until LAST=1.
fn decode_inter_ac_sac(
    rdr: &mut SacPPictureReader<'_>,
    block: &mut [i32; 64],
    quant: u32,
) -> Result<()> {
    let mut scan_pos = 0usize;
    let mut event_pos = 1usize;
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    loop {
        if scan_pos > 63 {
            return Err(Error::invalid("h263 SAC inter block: AC overrun"));
        }
        let (last, run, level_signed) = rdr.read_tcoef_inter(event_pos)?;
        scan_pos = scan_pos.saturating_add(run as usize);
        if scan_pos > 63 {
            return Err(Error::invalid("h263 SAC inter block: AC run overflow"));
        }
        let abs = level_signed.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if level_signed < 0 {
            val = -val;
        }
        let val = val.clamp(-2048, 2047);
        block[ZIGZAG[scan_pos]] = val;
        if last {
            return Ok(());
        }
        scan_pos += 1;
        event_pos += 1;
        if scan_pos > 63 {
            return Ok(());
        }
    }
}

fn decode_intra_ac_sac_in_p(
    rdr: &mut SacPPictureReader<'_>,
    block: &mut [i32; 64],
    quant: u32,
) -> Result<()> {
    let mut scan_pos = 1usize;
    let mut event_pos = 1usize;
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    loop {
        if scan_pos > 63 {
            return Err(Error::invalid("h263 SAC intra-in-P block: AC overrun"));
        }
        let (last, run, level_signed) = rdr.read_tcoef_intra_via(event_pos)?;
        scan_pos = scan_pos.saturating_add(run as usize);
        if scan_pos > 63 {
            return Err(Error::invalid("h263 SAC intra-in-P block: AC run overflow"));
        }
        let abs = level_signed.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if level_signed < 0 {
            val = -val;
        }
        let val = val.clamp(-2048, 2047);
        block[ZIGZAG[scan_pos]] = val;
        if last {
            return Ok(());
        }
        scan_pos += 1;
        event_pos += 1;
        if scan_pos > 63 {
            return Ok(());
        }
    }
}

/// Map a `cumf_MCBPC_no4MVQ` index back to `(mb_type, cbpc)`. Spec
/// Table 8: indices 0..=3 → Inter, 4..=7 → Intra, 8..=11 → InterQ,
/// 12..=15 → IntraQ, 16..=19 → Inter4MV (rejected here), 20 → stuffing
/// (caller filters), and (in `Mv4Q` model only) 21..=24 → Inter4MV+Q.
fn decompose_mcbpc_for_no4mvq(idx: usize) -> Result<(PMbType, u8)> {
    let cbpc = (idx & 0x3) as u8;
    let group = idx >> 2;
    let ty = match group {
        0 => PMbType::Inter,
        1 => PMbType::Intra,
        2 => PMbType::InterQ,
        3 => PMbType::IntraQ,
        4 => {
            return Err(Error::unsupported(
                "h263 SAC P: Inter4MV in cumf_MCBPC_no4MVQ stream — \
                 Annex F SAC interleave not implemented",
            ));
        }
        _ => {
            return Err(Error::invalid(format!(
                "h263 SAC MCBPC P: bad group {group}"
            )));
        }
    };
    Ok((ty, cbpc))
}

// Reader-side intra-flavour helpers (peer of the writer's `enc_intra_*`).
impl<'a> SacPPictureReader<'a> {
    /// Decode the next CBPY using the INTRA model.
    pub fn read_cbpy_intra_via(&mut self) -> Result<u8> {
        let v = self.decode_with(crate::sac::models::CBPY_INTRA)?;
        if v >= 16 {
            return Err(Error::invalid("SAC CBPY_intra P: index out of range"));
        }
        Ok(v as u8)
    }

    /// Decode the next INTRADC byte for an intra-in-P block.
    pub fn read_intradc_via(&mut self) -> Result<u8> {
        let idx = self.decode_with(crate::sac::models::INTRADC)?;
        crate::sac::intradc_index_to_byte(idx)
            .ok_or_else(|| Error::invalid("SAC INTRADC P/intra: index out of range"))
    }

    /// Decode the next intra-in-P TCOEF event.
    pub fn read_tcoef_intra_via(&mut self, position: usize) -> Result<(bool, u8, i32)> {
        let tcoef_model: &[u32] = match position {
            1 => crate::sac::models::TCOEF1_INTRA,
            2 => crate::sac::models::TCOEF2_INTRA,
            3 => crate::sac::models::TCOEF3_INTRA,
            _ => crate::sac::models::TCOEFR_INTRA,
        };
        let idx = self.decode_with(tcoef_model)?;
        if idx == crate::sac::TCOEF_ESCAPE_INDEX {
            let last_idx = self.decode_with(crate::sac::models::LAST_INTRA)?;
            let run = self.decode_with(crate::sac::models::RUN_INTRA)? as u8;
            let level_idx = self.decode_with(crate::sac::models::LEVEL_INTRA)?;
            let level_byte = crate::sac::level_index_to_byte(level_idx);
            if level_byte == 0 {
                return Err(Error::invalid("SAC TCOEF intra-in-P: level == 0 forbidden"));
            }
            if level_byte == 0x80 {
                return Err(Error::invalid(
                    "SAC TCOEF intra-in-P: level == -128 forbidden",
                ));
            }
            let level = if level_byte & 0x80 != 0 {
                level_byte as i32 - 256
            } else {
                level_byte as i32
            };
            return Ok((last_idx == 1, run, level));
        }
        let (last, run, abs) = crate::sac::tcoef_index_to_event(idx)
            .ok_or_else(|| Error::invalid("SAC TCOEF intra-in-P: bad event index"))?;
        let sign_idx = self.decode_with(crate::sac::models::SIGN)?;
        let level = if sign_idx == 1 {
            -(abs as i32)
        } else {
            abs as i32
        };
        Ok((last, run, level))
    }

    /// Pass-through helper used by the intra-in-P decode methods.
    fn decode_with(&mut self, model: &'static [u32]) -> Result<usize> {
        // We can't reach `dec` / `fifo` directly — they're private to
        // `SacPPictureReader`. Hand-route by reusing the decode_symbol
        // signature via a public hatch on the reader: but the reader
        // already exposes `read_mcbpc` etc. Instead expose a small helper.
        self.decode_symbol_internal(model)
    }
}

// `decode_symbol_internal` is colocated with the reader's other private
// items in `sac.rs` for visibility — see the impl block there.

#[cfg(test)]
mod tests {
    use crate::sac::{intradc_byte_to_index, intradc_index_to_byte};

    /// Every legal INTRADC FLC byte must round-trip through the index map.
    /// This catches off-by-one errors in the spec's "1..=127, 129..=254,
    /// special 0xFF" interleaving.
    #[test]
    fn intradc_index_round_trips_all_legal_bytes() {
        for b in 1u8..=255u8 {
            if b == 0x80 {
                continue; // forbidden
            }
            let idx = intradc_byte_to_index(b)
                .unwrap_or_else(|| panic!("intradc byte 0x{b:02x} → no index"));
            assert!(idx < 254, "INTRADC byte 0x{b:02x} → idx {idx} out of range");
            let back = intradc_index_to_byte(idx).unwrap();
            assert_eq!(
                back, b,
                "INTRADC byte 0x{b:02x} → idx {idx} → 0x{back:02x} (round-trip)"
            );
        }
    }

    /// Reserved bytes must reject.
    #[test]
    fn intradc_reserved_bytes_reject() {
        assert!(intradc_byte_to_index(0x00).is_none());
        assert!(intradc_byte_to_index(0x80).is_none());
    }
}
