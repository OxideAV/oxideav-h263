//! Annex E — MB-layer SAC bridge for I-pictures.
//!
//! Round 13: bridges the §E.2/§E.3 arithmetic coder (in [`crate::sac`]) into
//! the I-picture macroblock loop, replacing the per-MB VLC path with a SAC
//! traversal of MCBPC + CBPY + INTRADC + per-block TCOEF events.
//!
//! Picture-header layer bytes are still written / read with the VLC bit
//! engine — §E.6 keeps the fixed-length headers (PSC, TR, PTYPE, PQUANT,
//! PEI/PSPARE) outside the PSC_FIFO multiplexer. After the byte-aligned
//! picture header closes, the body switches to SAC: the encoder side calls
//! [`SacIPictureWriter`] for every MB and then [`SacIPictureWriter::finish`]
//! to flush the arithmetic coder (§E.7 `encoder_flush`); the decoder side
//! constructs a [`SacIPictureReader`] over the byte-aligned body and reads
//! the same MBs.
//!
//! Per §E.7 the integer indices for each model match Tables 7 (MCBPC intra),
//! 12 (CBPY), 15 (INTRADC), 16 (TCOEF), and 17 (RUN/LEVEL FLC) of clause 5.
//!
//! P-picture SAC bodies (which add the cumf_COD model + cumf_MCBPC_no4MVQ /
//! cumf_MCBPC_4MVQ + cumf_MVD wiring through the median-MV predictor) are
//! the next-round follow-up — the picture-header parse already accepts
//! `sac_mode = true` for P-pictures, but the decoder front-end short-circuits
//! with a specific `Error::Unsupported` until that wiring lands.

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
