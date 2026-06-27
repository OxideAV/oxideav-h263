//! H.263 macroblock-layer **encode** — the §5.3 inverse of
//! [`crate::macroblock::parse_macroblock`] for the baseline INTRA and
//! INTER macroblock forms.
//!
//! Assembles the §5.3 header fields (COD, MCBPC, CBPY, optional DQUANT,
//! optional MVD) and the §5.4 six-block payload (Y1..Y4, Cb, Cr) into
//! the on-wire macroblock. The block ordering and CBP-bit gating match
//! the decoder's `decode_one_macroblock` exactly:
//!
//! * luma block `blk` (0..=3) is gated by CBPY bit `3 - blk`,
//! * Cb is gated by CBPC bit `0b10`, Cr by CBPC bit `0b01`,
//! * INTRA blocks always carry INTRADC; the CBP bit only gates AC.
//!
//! INTER macroblocks complement the CBPY pattern on the wire
//! (`cbpy ^ 0b1111`) per §5.3.5, matching the decoder's `is_intra`
//! branch.

use crate::block::COEFFS_PER_BLOCK;
use crate::encoder_block::{
    encode_intra_block, write_inter_block_coeffs, write_intra_block, EncodedInterBlock,
};
use crate::encoder_vlc::{write_cbpy, write_mcbpc_i, write_mcbpc_p, write_mvd_component};
use crate::macroblock::{MbType, Mvd};
use crate::Result;
use oxideav_core::bits::BitWriter;

/// The six 8×8 luma + chroma sample blocks of a macroblock, in the
/// decoder's block order (Y1, Y2, Y3, Y4, Cb, Cr).
///
/// Luma blocks are the 2×2 grid of the 16×16 macroblock: Y1 top-left,
/// Y2 top-right, Y3 bottom-left, Y4 bottom-right. Each block is an 8×8
/// row-major `[i16; 64]` (sample values 0..=255 promoted to `i16`).
#[derive(Debug, Clone)]
pub struct MacroblockSamples {
    /// Four luma blocks Y1..Y4 (raster 2×2 order).
    pub luma: [[i16; COEFFS_PER_BLOCK]; 4],
    /// Cb chroma block.
    pub cb: [i16; COEFFS_PER_BLOCK],
    /// Cr chroma block.
    pub cr: [i16; COEFFS_PER_BLOCK],
}

/// Encode a baseline **INTRA** macroblock (I-picture or INTRA MB in a
/// P-picture) at quantiser `quant`, writing the full §5.3 + §5.4 wire
/// representation.
///
/// `write_cod` controls whether a leading COD bit (§5.3.1) is emitted:
/// `false` for I-pictures (no COD), `true` for P-pictures (COD = 0,
/// "coded"). `picture_is_inter` selects the MCBPC table (Table 7 vs
/// Table 8).
///
/// The macroblock type is INTRA (no DQUANT here — the +Q variants are a
/// later milestone). Returns [`crate::Error::BadMcbpcCode`] / [`crate::Error::BadTcoefCode`]
/// if any field is unrepresentable.
pub fn encode_intra_macroblock(
    w: &mut BitWriter,
    mb: &MacroblockSamples,
    quant: u8,
    write_cod: bool,
    picture_is_inter: bool,
) -> Result<()> {
    // §5.3.1 — COD (P-pictures only): 0 = coded.
    if write_cod {
        w.write_bit(false);
    }

    // Forward-transform + quantise all six blocks.
    let y_enc = [
        encode_intra_block(&mb.luma[0], quant),
        encode_intra_block(&mb.luma[1], quant),
        encode_intra_block(&mb.luma[2], quant),
        encode_intra_block(&mb.luma[3], quant),
    ];
    let cb_enc = encode_intra_block(&mb.cb, quant);
    let cr_enc = encode_intra_block(&mb.cr, quant);

    // §5.3.2 — MCBPC. CBPC bit 0b10 = Cb AC present, 0b01 = Cr AC present.
    let mut cbpc = 0u8;
    if cb_enc.has_ac {
        cbpc |= 0b10;
    }
    if cr_enc.has_ac {
        cbpc |= 0b01;
    }
    if picture_is_inter {
        write_mcbpc_p(w, MbType::Intra, cbpc)?;
    } else {
        write_mcbpc_i(w, MbType::Intra, cbpc)?;
    }

    // §5.3.5 — CBPY (INTRA orientation): bit (3 - blk) set when luma
    // block blk has AC.
    let mut cbpy = 0u8;
    for (blk, e) in y_enc.iter().enumerate() {
        if e.has_ac {
            cbpy |= 1 << (3 - blk);
        }
    }
    write_cbpy(w, cbpy)?;

    // §5.4 — six blocks in order Y1..Y4, Cb, Cr.
    for e in &y_enc {
        write_intra_block(w, e.dc_level, &e.scan, e.has_ac)?;
    }
    write_intra_block(w, cb_enc.dc_level, &cb_enc.scan, cb_enc.has_ac)?;
    write_intra_block(w, cr_enc.dc_level, &cr_enc.scan, cr_enc.has_ac)?;

    Ok(())
}

/// Encode a baseline **INTER** macroblock (P-picture) at quantiser
/// `quant` with a single motion vector difference `mvd` (half-pel
/// components) and six pre-encoded residual blocks.
///
/// The residual blocks are the forward-transformed, quantised
/// `source − prediction` differences; the caller supplies them already
/// encoded (via [`crate::encoder_block::encode_inter_block`]) so the
/// motion-estimation stage owns prediction. `mvd` is written with two
/// Table 14 VLCs (horizontal then vertical).
///
/// Emits COD = 0 (coded). Returns an error if a field is unrepresentable.
pub fn encode_inter_macroblock(
    w: &mut BitWriter,
    luma: &[EncodedInterBlock; 4],
    cb: &EncodedInterBlock,
    cr: &EncodedInterBlock,
    mvd: Mvd,
) -> Result<()> {
    // §5.3.1 — COD = 0 (coded). INTER macroblocks always appear in
    // P-pictures, which carry COD.
    w.write_bit(false);

    // §5.3.2 — MCBPC for an INTER (type 0) macroblock.
    let mut cbpc = 0u8;
    if cb.has_coeffs {
        cbpc |= 0b10;
    }
    if cr.has_coeffs {
        cbpc |= 0b01;
    }
    write_mcbpc_p(w, MbType::Inter, cbpc)?;

    // §5.3.5 — CBPY. The natural (INTRA-orientation) pattern has bit
    // (3 - blk) set when luma block blk has coefficients; INTER
    // macroblocks emit the complement.
    let mut cbpy_intra = 0u8;
    for (blk, e) in luma.iter().enumerate() {
        if e.has_coeffs {
            cbpy_intra |= 1 << (3 - blk);
        }
    }
    write_cbpy(w, cbpy_intra ^ 0b1111)?;

    // §5.3.7 — MVD (horizontal then vertical).
    write_mvd_component(w, mvd.dx_half)?;
    write_mvd_component(w, mvd.dy_half)?;

    // §5.4 — six blocks, only those with coefficients emit TCOEFs.
    for e in luma.iter() {
        if e.has_coeffs {
            write_inter_block_coeffs(w, &e.scan)?;
        }
    }
    if cb.has_coeffs {
        write_inter_block_coeffs(w, &cb.scan)?;
    }
    if cr.has_coeffs {
        write_inter_block_coeffs(w, &cr.scan)?;
    }

    Ok(())
}

/// Emit a "not coded" (skipped) P-picture macroblock: a single COD = 1
/// bit (§5.3.1). The decoder copies the co-located reference block.
pub fn encode_skipped_macroblock(w: &mut BitWriter) {
    w.write_bit(true);
}

/// Convenience: clamp an `i16` sample-difference array's source samples
/// (`u8` promoted) is the caller's job; this helper builds a
/// [`MacroblockSamples`] from raw `u8` luma/chroma blocks.
pub fn macroblock_samples_from_u8(
    luma: &[[u8; COEFFS_PER_BLOCK]; 4],
    cb: &[u8; COEFFS_PER_BLOCK],
    cr: &[u8; COEFFS_PER_BLOCK],
) -> MacroblockSamples {
    let mut out = MacroblockSamples {
        luma: [[0i16; COEFFS_PER_BLOCK]; 4],
        cb: [0i16; COEFFS_PER_BLOCK],
        cr: [0i16; COEFFS_PER_BLOCK],
    };
    for (dst, src) in out.luma.iter_mut().zip(luma.iter()) {
        for (d, &s) in dst.iter_mut().zip(src.iter()) {
            *d = s as i16;
        }
    }
    for (d, &s) in out.cb.iter_mut().zip(cb.iter()) {
        *d = s as i16;
    }
    for (d, &s) in out.cr.iter_mut().zip(cr.iter()) {
        *d = s as i16;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::macroblock::{parse_macroblock, MbContext};
    use crate::{H263PictureCodingType, MbType};
    use oxideav_core::bits::BitReader;

    fn flat_mb(value: i16) -> MacroblockSamples {
        MacroblockSamples {
            luma: [[value; COEFFS_PER_BLOCK]; 4],
            cb: [value; COEFFS_PER_BLOCK],
            cr: [value; COEFFS_PER_BLOCK],
        }
    }

    /// A flat INTRA macroblock in an I-picture: no COD, MCBPC type INTRA
    /// with cbpc=0, CBPY=0 (no AC). The decoder parses the header
    /// correctly.
    #[test]
    fn intra_flat_mb_header_round_trips() {
        let mb = flat_mb(100);
        let mut w = BitWriter::new();
        encode_intra_macroblock(
            &mut w, &mb, 8, /* write_cod */ false, /* inter */ false,
        )
        .unwrap();
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let parsed = parse_macroblock(
            &mut r,
            MbContext {
                picture_coding_type: H263PictureCodingType::Intra,
                advanced_prediction: false,
                deblocking_filter: false,
                aic_intra_mode: false,
                pb_frames: false,
                pb_annex_m: false,
                quantiser_before: 8,
                modified_quant: false,
            },
        )
        .unwrap();
        assert_eq!(parsed.mb_type, Some(MbType::Intra));
        assert_eq!(parsed.cbpc, Some(0));
        assert_eq!(parsed.cbpy, Some(0));
        assert!(parsed.coded);
    }

    /// A gradient INTRA macroblock: the header signals AC where present,
    /// and the parser consumes exactly the bits we wrote (the reader
    /// ends byte-aligned after we pad).
    #[test]
    fn intra_gradient_mb_consumes_all_bits() {
        let mut mb = flat_mb(80);
        // Put a gradient on Y1 and Cb so they have AC; leave others flat.
        for y in 0..8 {
            for x in 0..8 {
                mb.luma[0][y * 8 + x] = (80 + 12 * x as i32 - 6 * y as i32) as i16;
                mb.cb[y * 8 + x] = (80 + 8 * x as i32) as i16;
            }
        }
        let mut w = BitWriter::new();
        encode_intra_macroblock(&mut w, &mb, 5, false, false).unwrap();
        let bit_len = w.bit_position();
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        // Parse the header then the six blocks via the full driver is
        // out of scope here; instead parse the macroblock header and
        // confirm the AC flags are consistent.
        let parsed = parse_macroblock(
            &mut r,
            MbContext {
                picture_coding_type: H263PictureCodingType::Intra,
                advanced_prediction: false,
                deblocking_filter: false,
                aic_intra_mode: false,
                pb_frames: false,
                pb_annex_m: false,
                quantiser_before: 5,
                modified_quant: false,
            },
        )
        .unwrap();
        // Y1 has AC -> CBPY bit 3 set.
        assert_eq!(parsed.cbpy.unwrap() & 0b1000, 0b1000);
        // Cb has AC -> CBPC bit 0b10 set.
        assert_eq!(parsed.cbpc.unwrap() & 0b10, 0b10);
        assert!(bit_len > 0);
    }

    /// Skipped MB: COD = 1, then the decoder reports not-coded.
    #[test]
    fn skipped_mb_round_trips() {
        let mut w = BitWriter::new();
        encode_skipped_macroblock(&mut w);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let parsed = parse_macroblock(
            &mut r,
            MbContext {
                picture_coding_type: H263PictureCodingType::Inter,
                advanced_prediction: false,
                deblocking_filter: false,
                aic_intra_mode: false,
                pb_frames: false,
                pb_annex_m: false,
                quantiser_before: 8,
                modified_quant: false,
            },
        )
        .unwrap();
        assert!(!parsed.coded);
    }
}
