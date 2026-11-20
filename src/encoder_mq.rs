//! INTER-picture encoder for the two coefficient-layer annexes:
//! **Annex T Modified Quantization** (§T.3 chrominance `QUANT_C`,
//! §T.4 EXTENDED-ESCAPE coefficient range) and **Annex S Alternative
//! INTER VLC** (§S.2 Table I.2 codewords for INTER blocks whenever the
//! decoder can detect them, §S.3 the INTRA CBPY column when both
//! chrominance blocks are coded).
//!
//! One motion vector per macroblock on a PLUSPTYPE header, one video
//! picture segment. The caller supplies the decoder's reconstruction
//! of the previous picture as the reference (closed loop).

use oxideav_core::bits::{BitReader, BitWriter};

use crate::block::{COEFFS_PER_BLOCK, ZIGZAG_TO_BLOCK_POS, parse_inter_block_alt_inter_vlc};
use crate::encoder::{
    PlusModes, extract_macroblock, motion_compensated_block, residual_of, write_plus_picture_header,
};
use crate::encoder_block::{EncodedInterBlock, tcoef_events};
use crate::encoder_motion::{MvGrid, estimate_motion, estimate_motion_umv_plus, mvd_for};
use crate::encoder_vlc::{TcoefEvent, write_cbpy, write_mcbpc_p, write_mvd_component, write_tcoef};
use crate::intra_tcoef::{IntraTcoefEvent, write_intra_tcoef_event};
use crate::macroblock::{MbType, Mvd};
use crate::picture::YuvFrame;
use crate::{Error, Result};

/// Options of [`encode_inter_picture_mq`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct InterVlcConfig {
    /// QUANT (1..=31).
    pub quant: u8,
    /// Motion search half-width in half-pel units.
    pub search_half: i32,
    /// Annex T: chrominance at `QUANT_C`, coefficient levels beyond
    /// ±127 through EXTENDED-ESCAPE when the step allows it (§T.5).
    pub modified_quant: bool,
    /// Annex S: Table I.2 codewords for an INTER block when that is
    /// shorter and detectable; the INTRA CBPY column when both
    /// chrominance blocks are coded.
    pub alt_inter_vlc: bool,
    /// Annex D (Table D.3 vector differences under the UUI "1" range).
    pub umv: bool,
}

impl InterVlcConfig {
    /// Defaults: the given QUANT, a ±7.5-pel search, both annexes off.
    pub fn new(quant: u8) -> Self {
        InterVlcConfig {
            quant,
            search_half: 15,
            modified_quant: false,
            alt_inter_vlc: false,
            umv: false,
        }
    }
    /// Set [`InterVlcConfig::search_half`].
    #[must_use]
    pub fn with_search_half(mut self, v: i32) -> Self {
        self.search_half = v;
        self
    }
    /// Set [`InterVlcConfig::modified_quant`].
    #[must_use]
    pub fn with_modified_quant(mut self, v: bool) -> Self {
        self.modified_quant = v;
        self
    }
    /// Set [`InterVlcConfig::alt_inter_vlc`].
    #[must_use]
    pub fn with_alt_inter_vlc(mut self, v: bool) -> Self {
        self.alt_inter_vlc = v;
        self
    }
    /// Set [`InterVlcConfig::umv`].
    #[must_use]
    pub fn with_umv(mut self, v: bool) -> Self {
        self.umv = v;
        self
    }
}

/// Per-picture counts returned by [`encode_inter_picture_mq_stats`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct InterVlcStats {
    /// Skipped macroblocks.
    pub skipped: usize,
    /// Coded blocks sent with Table 16 codewords.
    pub inter_vlc_blocks: usize,
    /// Coded blocks sent with Table I.2 codewords (§S.2).
    pub intra_vlc_blocks: usize,
    /// Macroblocks whose CBPY used the INTRA column (§S.3).
    pub intra_cbpy: usize,
    /// Coefficients sent through EXTENDED-ESCAPE (§T.4).
    pub extended_levels: usize,
}

/// Largest LEVEL magnitude the mode allows at step `q`: 127 in the
/// default mode and — §T.5 rule 3 — whenever the step is 8 or more;
/// below that the §T.4 extended range, bounded so the reconstruction
/// `q · (2·|LEVEL| + 1)` stays inside the §6.2.2 clip (which also
/// satisfies §T.5 rule 1, `|REC| < 4096`).
fn max_level(q: u8, modified_quant: bool) -> i32 {
    if modified_quant && q < 8 {
        ((2047 / q as i32) - 1) / 2
    } else {
        127
    }
}

/// §6.2.1 forward quantisation of an INTER block with the mode's level
/// range (dead-zone quantiser, as the default-mode stage).
fn quantise_inter(
    freq: &[f64; COEFFS_PER_BLOCK],
    q: u8,
    modified_quant: bool,
) -> EncodedInterBlock {
    let cap = max_level(q, modified_quant);
    let step = 2.0 * q as f64;
    let mut scan = [0i16; COEFFS_PER_BLOCK];
    for (slot, &pos) in scan.iter_mut().zip(ZIGZAG_TO_BLOCK_POS.iter()) {
        let c = freq[pos as usize];
        let level = ((c.abs() / step).floor() as i32).min(cap);
        *slot = if c < 0.0 { -level } else { level } as i16;
    }
    let has_coeffs = scan.iter().any(|&l| l != 0);
    EncodedInterBlock { scan, has_coeffs }
}

/// Append the first `bits` bits of `buf` to `w`.
fn append_bits(w: &mut BitWriter, buf: &[u8], bits: u64) -> Result<()> {
    let mut r = BitReader::new(buf);
    let mut left = bits;
    while left > 0 {
        let n = left.min(32) as u32;
        w.write_bits(r.read_u32(n).map_err(|_| Error::UnexpectedEof)?, n);
        left -= n as u64;
    }
    Ok(())
}

/// One INTER block under Table 16; a level beyond ±127 takes the §T.4
/// EXTENDED-ESCAPE form (ESCAPE, LAST, RUN, `1000 0000`, the 11-bit
/// rotated EXTENDED-LEVEL).
fn write_inter_events(
    w: &mut BitWriter,
    events: &[TcoefEvent],
    extended: &mut usize,
) -> Result<()> {
    for &ev in events {
        if ev.level.unsigned_abs() > 127 {
            // §5.4.2 ESCAPE, "0000 011".
            w.write_bits(0b11, 7);
            w.write_bit(ev.last);
            w.write_bits(ev.run as u32, 6);
            w.write_bits(0x80, 8);
            w.write_bits(crate::block::extended_level_to_wire(ev.level) as u32, 11);
            *extended += 1;
        } else {
            write_tcoef(w, ev)?;
        }
    }
    Ok(())
}

/// Write one coded INTER block, choosing the codeword table per §S.2.1.
fn write_block(
    w: &mut BitWriter,
    scan: &[i16; COEFFS_PER_BLOCK],
    cfg: &InterVlcConfig,
    stats: &mut InterVlcStats,
) -> Result<()> {
    let events = tcoef_events(scan, false);
    if events.is_empty() {
        return Err(Error::BadTcoefCode);
    }
    let mut inter = BitWriter::new();
    let mut extended = 0usize;
    write_inter_events(&mut inter, &events, &mut extended)?;
    let inter_bits = inter.bit_position();

    if cfg.alt_inter_vlc {
        // §S.2.1 — Table I.2 "whenever the decoder can detect its use"
        // and it "results in fewer bits". Detection is the decoder's
        // own §S.2.2 procedure run over the candidate codewords: it
        // must hand back exactly this block.
        let mut intra = BitWriter::new();
        let mut ok = true;
        for ev in &events {
            let iev = IntraTcoefEvent {
                last: ev.last,
                run: ev.run,
                level: ev.level,
            };
            if write_intra_tcoef_event(&mut intra, iev, cfg.modified_quant).is_err() {
                ok = false;
                break;
            }
        }
        let intra_bits = intra.bit_position();
        if ok && intra_bits < inter_bits {
            // Trailing ones keep a short candidate from running the
            // probe out of data instead of out of the block.
            let mut probe = intra.finish();
            probe.extend_from_slice(&[0xFF; 4]);
            let mut r = BitReader::new(&probe);
            let detected = parse_inter_block_alt_inter_vlc(&mut r, cfg.modified_quant)
                .is_ok_and(|b| b.coefficients == *scan && r.bit_position() == intra_bits);
            if detected {
                stats.intra_vlc_blocks += 1;
                stats.extended_levels += extended;
                return append_bits(w, &probe, intra_bits);
            }
        }
    }
    stats.inter_vlc_blocks += 1;
    stats.extended_levels += extended;
    append_bits(w, &inter.finish(), inter_bits)
}

/// Encode `frame` as a PLUSPTYPE **INTER** picture with Annex T and /
/// or Annex S in use, predicted from `reference` (the decoder's
/// reconstruction of the previous picture).
///
/// Self-describing: decodes through
/// [`crate::picture::decode_picture_layer`] / `decode_sequence`.
pub fn encode_inter_picture_mq(
    frame: &YuvFrame,
    reference: &YuvFrame,
    tr: u8,
    cfg: &InterVlcConfig,
) -> Result<Vec<u8>> {
    encode_inter_picture_mq_stats(frame, reference, tr, cfg).map(|(b, _)| b)
}

/// [`encode_inter_picture_mq`] that also reports the per-picture
/// counts.
pub fn encode_inter_picture_mq_stats(
    frame: &YuvFrame,
    reference: &YuvFrame,
    tr: u8,
    cfg: &InterVlcConfig,
) -> Result<(Vec<u8>, InterVlcStats)> {
    let quant = cfg.quant;
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if frame.luma_width != reference.luma_width || frame.luma_height != reference.luma_height {
        return Err(Error::NotImplemented);
    }
    let fmt = crate::encoder::source_format_for(frame.luma_width, frame.luma_height)
        .ok_or(Error::NotImplemented)?;
    // §T.3 — chrominance step.
    let quant_c = if cfg.modified_quant {
        crate::annex_t::quant_c_from_quant(quant)?
    } else {
        quant
    };

    let mut w = BitWriter::new();
    write_plus_picture_header(
        &mut w,
        fmt,
        quant,
        tr,
        /* is_inter */ true,
        PlusModes {
            umv: cfg.umv,
            modified_quant: cfg.modified_quant,
            alt_inter_vlc: cfg.alt_inter_vlc,
            ..PlusModes::default()
        },
    )?;

    let (lw, lh) = (frame.luma_width, frame.luma_height);
    let (cw, ch) = (frame.chroma_width(), frame.chroma_height());
    let (mb_cols, mb_rows) = (lw / 16, lh / 16);
    let mut grid = MvGrid::new(mb_cols, mb_rows);
    let lambda = 2 * quant as u32;
    let mut stats = InterVlcStats::default();
    let to_i16 = |p: &[u8; COEFFS_PER_BLOCK]| -> [i16; COEFFS_PER_BLOCK] { p.map(|v| v as i16) };

    for row in 0..mb_rows {
        for col in 0..mb_cols {
            let predictor = grid.predict(col, row);
            let mv = if cfg.umv {
                estimate_motion_umv_plus(
                    frame,
                    reference,
                    col,
                    row,
                    predictor,
                    cfg.search_half,
                    lambda,
                )
            } else {
                estimate_motion(
                    frame,
                    reference,
                    col,
                    row,
                    predictor,
                    cfg.search_half,
                    lambda,
                )
            };
            let src = extract_macroblock(frame, col, row);
            let luma: [EncodedInterBlock; 4] = std::array::from_fn(|b| {
                let (bx, by) = (col * 16 + (b % 2) * 8, row * 16 + (b / 2) * 8);
                let pred = motion_compensated_block(&reference.y, lw, lh, bx, by, mv);
                let freq = crate::fdct::fdct_8x8(&residual_of(&src.luma[b], &to_i16(&pred)));
                quantise_inter(&freq, quant, cfg.modified_quant)
            });
            let cmv = crate::motion::chroma_mv(mv);
            let chroma = |plane: &[u8], s: &[i16; COEFFS_PER_BLOCK]| -> EncodedInterBlock {
                let pred = motion_compensated_block(plane, cw, ch, col * 8, row * 8, cmv);
                let freq = crate::fdct::fdct_8x8(&residual_of(s, &to_i16(&pred)));
                quantise_inter(&freq, quant_c, cfg.modified_quant)
            };
            let cb = chroma(&reference.cb, &src.cb);
            let cr = chroma(&reference.cr, &src.cr);

            let any = luma.iter().any(|e| e.has_coeffs) || cb.has_coeffs || cr.has_coeffs;
            if !any && mv.dx_half == 0 && mv.dy_half == 0 {
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
                grid.set_zero_candidate(col, row);
                stats.skipped += 1;
                continue;
            }

            // COD, MCBPC, CBPY, MVD.
            w.write_bit(false);
            let cbpc = (cb.has_coeffs as u8) << 1 | cr.has_coeffs as u8;
            write_mcbpc_p(&mut w, MbType::Inter, cbpc)?;
            let cbpy = luma
                .iter()
                .enumerate()
                .fold(0u8, |acc, (b, e)| acc | (e.has_coeffs as u8) << (3 - b));
            // §S.3 — the INTRA column of Table 12 "whenever
            // CBPC5 = CBPC6 = 1"; otherwise the INTER complement.
            if cfg.alt_inter_vlc && cbpc == 0b11 {
                write_cbpy(&mut w, cbpy)?;
                stats.intra_cbpy += 1;
            } else {
                write_cbpy(&mut w, cbpy ^ 0b1111)?;
            }
            if cfg.umv {
                crate::encoder_vlc::write_mvd_pair_d3(
                    &mut w,
                    Mvd {
                        dx_half: (mv.dx_half - predictor.dx_half) as i16,
                        dy_half: (mv.dy_half - predictor.dy_half) as i16,
                    },
                )?;
            } else {
                let mvd = mvd_for(mv, predictor);
                write_mvd_component(&mut w, mvd.dx_half)?;
                write_mvd_component(&mut w, mvd.dy_half)?;
            }
            for e in luma.iter().chain([&cb, &cr]).filter(|e| e.has_coeffs) {
                write_block(&mut w, &e.scan, cfg, &mut stats)?;
            }
            grid.set_inter(col, row, mv);
        }
    }

    w.align_to_byte_zero();
    Ok((w.finish(), stats))
}
