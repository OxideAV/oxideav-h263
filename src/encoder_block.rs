//! H.263 block-layer **encode** — the §5.4 inverse of
//! [`crate::block::parse_block`].
//!
//! Composes the forward transform / quantiser ([`crate::fdct`]) with the
//! VLC emitters ([`crate::encoder_vlc`]) to turn an 8×8 spatial block
//! into its on-wire block-layer representation: an optional INTRADC
//! prefix (§5.4.1, INTRA blocks only) followed by zero or more
//! `(LAST, RUN, LEVEL)` TCOEF events (§5.4.2).
//!
//! The run-length coder walks the 64-entry zigzag-scan coefficient
//! array, emitting a TCOEF event for every non-zero LEVEL with the RUN
//! of zeros that precede it, and marks the final non-zero event with
//! `LAST = 1`. This is the exact inverse of the decoder's
//! accumulate-RUN-then-write-LEVEL loop, so
//! `encode_block(...) ; parse_block(...)` reproduces the quantised
//! coefficients.

use crate::block::COEFFS_PER_BLOCK;
use crate::encoder_vlc::{write_intradc, write_tcoef, TcoefEvent};
use crate::fdct::{fdct_8x8, forward_quantise_block, quantise_intradc};
use crate::{Error, Result};
use oxideav_core::bits::BitWriter;

/// Whether a quantised **scan-order** coefficient array carries any AC
/// (non-DC) energy — the CBP bit for the block.
///
/// For an INTRA block this is "any non-zero coefficient in slots 1..64"
/// (the DC is carried separately as INTRADC); for an INTER block it is
/// "any non-zero coefficient in slots 0..64".
pub fn block_has_ac(scan: &[i16; COEFFS_PER_BLOCK], is_intra: bool) -> bool {
    let start = if is_intra { 1 } else { 0 };
    scan[start..].iter().any(|&c| c != 0)
}

/// Run-length-code a quantised **scan-order** coefficient array into the
/// TCOEF event stream (§5.4.2). The events cover slots `start..64`
/// (`start = 1` for INTRA, `0` for INTER), and the last non-zero event
/// is flagged `LAST = 1`.
///
/// Returns an empty vector when no coefficient in range is non-zero
/// (the caller then clears the CBP bit and emits no TCOEFs for the
/// block).
pub fn tcoef_events(scan: &[i16; COEFFS_PER_BLOCK], is_intra: bool) -> Vec<TcoefEvent> {
    let start = if is_intra { 1 } else { 0 };
    // Collect (scan_index, level) for non-zero coefficients in range.
    let mut nz: Vec<(usize, i16)> = Vec::new();
    for (idx, &lvl) in scan.iter().enumerate().take(COEFFS_PER_BLOCK).skip(start) {
        if lvl != 0 {
            nz.push((idx, lvl));
        }
    }
    let mut events = Vec::with_capacity(nz.len());
    let mut prev = start as i32 - 1; // index just before the first eligible slot
    for (i, &(idx, lvl)) in nz.iter().enumerate() {
        let run = (idx as i32 - prev - 1) as u8;
        let last = i == nz.len() - 1;
        events.push(TcoefEvent {
            last,
            run,
            level: lvl,
        });
        prev = idx as i32;
    }
    events
}

/// Emit an INTER block's §5.4 wire bits from a quantised scan-order
/// array, assuming the caller already determined (via [`block_has_ac`])
/// that the block has coefficients. Writes only the TCOEF events.
///
/// Returns [`Error::BadTcoefCode`] if an event is unrepresentable (RUN
/// or LEVEL out of the §5.4.2 range).
pub fn write_inter_block_coeffs(w: &mut BitWriter, scan: &[i16; COEFFS_PER_BLOCK]) -> Result<()> {
    let events = tcoef_events(scan, /* is_intra = */ false);
    if events.is_empty() {
        // No coefficients — caller should not have set the CBP bit.
        return Err(Error::BadTcoefCode);
    }
    for ev in events {
        write_tcoef(w, ev)?;
    }
    Ok(())
}

/// Emit an INTRA block's §5.4 wire bits: the 8-bit INTRADC (§5.4.1)
/// followed by the AC TCOEF events (§5.4.2) when `has_ac` is set.
///
/// `dc_level` is the Table 15 reconstruction level (slot-0 value); the
/// emitter maps it back to the 8-bit FLC. `scan` carries the quantised
/// AC coefficients in slots 1..64 (slot 0 is ignored). When `has_ac`
/// is `false` no TCOEFs are written (the caller cleared the CBP bit).
pub fn write_intra_block(
    w: &mut BitWriter,
    dc_level: i16,
    scan: &[i16; COEFFS_PER_BLOCK],
    has_ac: bool,
) -> Result<()> {
    write_intradc(w, dc_level)?;
    if has_ac {
        let events = tcoef_events(scan, /* is_intra = */ true);
        if events.is_empty() {
            // has_ac was set but there are no AC coefficients — caller
            // bug; refuse rather than emit a malformed block.
            return Err(Error::BadTcoefCode);
        }
        for ev in events {
            write_tcoef(w, ev)?;
        }
    }
    Ok(())
}

/// One encoded INTRA block: the forward transform + quantisation of an
/// 8×8 spatial block, ready to emit and carrying its CBP (AC-present)
/// bit.
#[derive(Debug, Clone)]
pub struct EncodedIntraBlock {
    /// Table 15 DC reconstruction level (slot-0 value).
    pub dc_level: i16,
    /// Quantised AC coefficients in zigzag scan order (slot 0 unused).
    pub scan: [i16; COEFFS_PER_BLOCK],
    /// `true` when any AC coefficient is non-zero (the CBP bit).
    pub has_ac: bool,
}

/// Forward-transform + quantise an 8×8 INTRA spatial block at `quant`.
///
/// The DC coefficient (`8×mean`) is rounded to a legal Table 15 level;
/// the AC coefficients are dead-zone quantised. The returned
/// [`EncodedIntraBlock`] is emitted with [`write_intra_block`].
pub fn encode_intra_block(samples: &[i16; COEFFS_PER_BLOCK], quant: u8) -> EncodedIntraBlock {
    let freq = fdct_8x8(samples);
    let dc_level = quantise_intradc(freq[0]);
    let scan = forward_quantise_block(&freq, quant, /* is_intra = */ true);
    let has_ac = block_has_ac(&scan, /* is_intra = */ true);
    EncodedIntraBlock {
        dc_level,
        scan,
        has_ac,
    }
}

/// One encoded INTER block: the forward transform + quantisation of an
/// 8×8 residual block, ready to emit and carrying its CBP bit.
#[derive(Debug, Clone)]
pub struct EncodedInterBlock {
    /// Quantised coefficients in zigzag scan order (slot 0 is AC for
    /// INTER blocks).
    pub scan: [i16; COEFFS_PER_BLOCK],
    /// `true` when any coefficient is non-zero (the CBP bit).
    pub has_coeffs: bool,
}

/// Forward-transform + quantise an 8×8 INTER **residual** block (signed
/// difference `source − prediction`, range roughly `[-255, 255]`) at
/// `quant`. The DC slot is an ordinary AC coefficient.
pub fn encode_inter_block(residual: &[i16; COEFFS_PER_BLOCK], quant: u8) -> EncodedInterBlock {
    let freq = fdct_8x8(residual);
    let scan = forward_quantise_block(&freq, quant, /* is_intra = */ false);
    let has_coeffs = block_has_ac(&scan, /* is_intra = */ false);
    EncodedInterBlock { scan, has_coeffs }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::{parse_block, BlockContext};
    use crate::{reconstruct_inter_block_with_prediction, reconstruct_intra_block};
    use oxideav_core::bits::BitReader;

    fn parse_intra(bytes: &[u8], has_ac: bool) -> crate::block::H263Block {
        let mut r = BitReader::new(bytes);
        parse_block(
            &mut r,
            BlockContext {
                has_intradc: true,
                has_coefficients: has_ac,
                modified_quant: false,
            },
        )
        .unwrap()
    }

    fn parse_inter(bytes: &[u8]) -> crate::block::H263Block {
        let mut r = BitReader::new(bytes);
        parse_block(
            &mut r,
            BlockContext {
                has_intradc: false,
                has_coefficients: true,
                modified_quant: false,
            },
        )
        .unwrap()
    }

    /// A flat INTRA block encodes to DC only (no AC), and the decoder
    /// reconstructs a uniform field equal to the input value.
    #[test]
    fn intra_flat_block_round_trips() {
        let samples = [120i16; COEFFS_PER_BLOCK];
        let enc = encode_intra_block(&samples, 8);
        assert!(!enc.has_ac, "flat block should have no AC");
        // DC = 8*120 = 960.
        assert_eq!(enc.dc_level, 960);

        let mut w = BitWriter::new();
        write_intra_block(&mut w, enc.dc_level, &enc.scan, enc.has_ac).unwrap();
        let bytes = w.finish();

        let block = parse_intra(&bytes, enc.has_ac);
        let recon = reconstruct_intra_block(&block, 8);
        // DC 960 -> pixel 960/8 = 120.
        assert!(recon.iter().all(|&p| p == 120), "recon not flat 120");
    }

    /// An INTRA block with a gradient encodes DC + AC and reconstructs
    /// close to the source (within a quant-dependent tolerance).
    #[test]
    fn intra_gradient_block_round_trips_approximately() {
        let mut samples = [0i16; COEFFS_PER_BLOCK];
        for y in 0..8 {
            for x in 0..8 {
                samples[y * 8 + x] = (100 + 10 * x as i32 - 5 * y as i32) as i16;
            }
        }
        let quant = 4;
        let enc = encode_intra_block(&samples, quant);
        assert!(enc.has_ac);

        let mut w = BitWriter::new();
        write_intra_block(&mut w, enc.dc_level, &enc.scan, enc.has_ac).unwrap();
        let bytes = w.finish();

        let block = parse_intra(&bytes, enc.has_ac);
        let recon = reconstruct_intra_block(&block, quant);
        // Mean error should be small.
        let mut max_err = 0i32;
        for (i, &p) in recon.iter().enumerate() {
            let err = (p as i32 - samples[i] as i32).abs();
            max_err = max_err.max(err);
        }
        assert!(max_err <= 16, "max reconstruction error {}", max_err);
    }

    /// An all-zero INTER residual encodes to no coefficients.
    #[test]
    fn inter_zero_residual_has_no_coeffs() {
        let residual = [0i16; COEFFS_PER_BLOCK];
        let enc = encode_inter_block(&residual, 8);
        assert!(!enc.has_coeffs);
    }

    /// A non-trivial INTER residual round-trips: encode, decode, add the
    /// prediction back, and the reconstruction is close to source.
    #[test]
    fn inter_residual_round_trips_approximately() {
        let prediction = [100u8; COEFFS_PER_BLOCK];
        let mut source = [0i16; COEFFS_PER_BLOCK];
        for y in 0..8 {
            for x in 0..8 {
                source[y * 8 + x] = (100 + 20 * ((x + y) % 3) as i32 - 15) as i16;
            }
        }
        let mut residual = [0i16; COEFFS_PER_BLOCK];
        for i in 0..COEFFS_PER_BLOCK {
            residual[i] = source[i] - prediction[i] as i16;
        }
        let quant = 5;
        let enc = encode_inter_block(&residual, quant);
        assert!(enc.has_coeffs);

        let mut w = BitWriter::new();
        write_inter_block_coeffs(&mut w, &enc.scan).unwrap();
        let bytes = w.finish();

        let block = parse_inter(&bytes);
        let recon = reconstruct_inter_block_with_prediction(&block, quant, &prediction);
        let mut max_err = 0i32;
        for (i, &p) in recon.iter().enumerate() {
            max_err = max_err.max((p as i32 - source[i] as i32).abs());
        }
        assert!(max_err <= 24, "max INTER reconstruction error {}", max_err);
    }

    /// The run-length coder produces correctly-spaced events that the
    /// decoder reconstructs bit-exactly (no transform involved).
    #[test]
    fn tcoef_run_length_is_exact() {
        let mut scan = [0i16; COEFFS_PER_BLOCK];
        scan[0] = 5;
        scan[3] = -2;
        scan[10] = 1;
        scan[63] = 7;
        let mut w = BitWriter::new();
        write_inter_block_coeffs(&mut w, &scan).unwrap();
        let bytes = w.finish();
        let block = parse_inter(&bytes);
        assert_eq!(block.coefficients, scan, "scan not reproduced bit-exactly");
    }
}
