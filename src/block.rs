//! H.263 block-level decoding (§5.4 of ITU-T Rec. H.263).
//!
//! For an INTRA macroblock, each of the six 8×8 blocks is encoded as:
//! * **INTRADC** — 8 fixed-length bits. Bitstream values `0x00` and `0x80`
//!   are illegal; `0xFF` decodes to a reconstruction level of 1024 (giving a
//!   gray block of pel value 128). All other values decode to
//!   `intradc << 3` — i.e. the pel-domain DC times 8. This is fed directly
//!   into the IDCT input at position 0.
//! * **TCOEF** — present iff the block's CBP bit is set. Variable-length
//!   `(last, run, level)` triples in zig-zag scan order, terminated by
//!   `last == true`. Codes use Table 16/H.263 (the same table as the
//!   MPEG-4 Part 2 inter TCOEF, Annex B-17). Escape mode is much simpler
//!   than MPEG-4: the 7-bit `0000011` escape prefix is followed by
//!   `last(1) + run(6) + level(8 signed)` — no marker bits, no max-level
//!   trick.
//!
//! Dequantisation uses the H.263 formula (identical to MPEG-4's H.263 mode):
//!   `|F''| = q * (2 * |level| + 1)`, with bit-0 cleared when `q` is even.
//! INTRADC bypasses dequantisation — it's already in the correct domain.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};
use oxideav_mpeg4video::headers::vol::ZIGZAG;
use oxideav_mpeg4video::tables::{
    tcoef::{inter_table, TcoefSym},
    vlc,
};

/// Decode the 8-bit INTRADC value and return the reconstruction level for
/// position `[0]` of the IDCT input.
///
/// Returns `Err` for the two illegal bitstream values `0x00` and `0x80`.
pub fn decode_intradc(br: &mut BitReader<'_>) -> Result<i32> {
    let v = br.read_u32(8)? as u8;
    if v == 0x00 || v == 0x80 {
        return Err(Error::invalid(format!(
            "h263 INTRADC: illegal bitstream value 0x{v:02x}"
        )));
    }
    if v == 0xFF {
        Ok(1024)
    } else {
        Ok((v as i32) << 3)
    }
}

/// Decode the AC coefficients of an 8×8 H.263 block, placing them in zig-zag
/// scan positions of `block`. AC starts at scan index 1 for INTRA blocks (DC
/// is in `block[0]` already, set by the caller from INTRADC) and at scan
/// index 0 for INTER blocks. `start` selects which.
///
/// Coefficients are written in their **dequantised** form so the IDCT can be
/// run directly afterwards.
pub fn decode_ac(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    start: usize,
    quant: u32,
) -> Result<()> {
    let table = inter_table();
    let mut i: usize = start;
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    loop {
        if i > 63 {
            return Err(Error::invalid("h263 block: AC overrun"));
        }
        let sym = vlc::decode(br, table)?;
        let (last, run, level_signed) = match sym {
            TcoefSym::RunLevel {
                last,
                run,
                level_abs,
            } => {
                let sign = br.read_u1()? as i32;
                let l = if sign == 1 {
                    -(level_abs as i32)
                } else {
                    level_abs as i32
                };
                (last, run, l)
            }
            TcoefSym::Escape => {
                // H.263 escape: last(1) + run(6) + level(8 signed).
                let last = br.read_u1()? == 1;
                let run = br.read_u32(6)? as u8;
                let raw = br.read_u32(8)?;
                // 8-bit two's complement; reject 0x80 (forbidden).
                let level: i32 = if raw == 0 {
                    return Err(Error::invalid("h263 block: escape level == 0"));
                } else if raw == 0x80 {
                    return Err(Error::invalid("h263 block: escape level == -128 forbidden"));
                } else if raw & 0x80 != 0 {
                    raw as i32 - 256
                } else {
                    raw as i32
                };
                (last, run, level)
            }
        };
        i = i.saturating_add(run as usize);
        if i > 63 {
            return Err(Error::invalid("h263 block: AC run overflow"));
        }
        // Dequantise: |F''| = q * (2*|level| + 1) - (1 if q even else 0).
        let abs = level_signed.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if level_signed < 0 {
            val = -val;
        }
        let val = val.clamp(-2048, 2047);
        block[ZIGZAG[i]] = val;
        if last {
            return Ok(());
        }
        i += 1;
        if i > 63 {
            // No more room and `last` wasn't set — accept end-of-block anyway.
            return Ok(());
        }
    }
}

/// Variant of [`decode_ac`] for **Annex S — Alternative INTER VLC**.
///
/// Per §S.2 the decoder first parses an INTER block's coefficients with
/// the standard inter VLC (Table 16). If during that parse the running
/// scan position would exceed the 64-coefficient block (i.e. the
/// codewords decoded under the inter table imply RUN values that overflow
/// the block), the codewords are re-interpreted under the INTRA VLC
/// (Table I.2) — which assigns shorter codes to large `|LEVEL|` values
/// at the cost of larger RUN values, so a stream that overflows under
/// INTER may decode cleanly under INTRA.
///
/// This implementation buffers the raw codewords as we go: every
/// `(bits, value)` we read is recorded, then if the inter parse
/// overflows we replay the same bit slice through the intra VLC.
/// The `start` parameter is `0` for inter blocks (no separate INTRADC).
///
/// Returns the number of bits consumed from `br`. The buffered re-parse
/// reads from a fresh `BitReader` over those same bits.
///
/// On success the dequantised coefficients are written into `block` in
/// zig-zag order (matching [`decode_ac`]); the caller IDCTs as usual.
pub fn decode_ac_aiv(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    start: usize,
    quant: u32,
) -> Result<()> {
    // Snapshot before the parse — if INTER overflows we replay through
    // INTRA from the same bit position.
    let saved = *br;
    match try_decode_ac_inter(br, block, start, quant) {
        Ok(()) => Ok(()),
        Err(_) => {
            // Reset state and re-parse with the INTRA table.
            *br = saved;
            for v in block.iter_mut() {
                *v = 0;
            }
            try_decode_ac_intra(br, block, start, quant)
        }
    }
}

/// INTER-table AC decode used by [`decode_ac_aiv`]. Identical to
/// [`decode_ac`] except the run-overflow check returns a sentinel error
/// rather than aborting the whole picture.
fn try_decode_ac_inter(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    start: usize,
    quant: u32,
) -> Result<()> {
    let table = inter_table();
    let mut i: usize = start;
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    loop {
        if i > 63 {
            return Err(Error::invalid(
                "h263 AIV INTER overflow → fall back to INTRA",
            ));
        }
        let sym = vlc::decode(br, table)?;
        let (last, run, level_signed) = match sym {
            TcoefSym::RunLevel {
                last,
                run,
                level_abs,
            } => {
                let sign = br.read_u1()? as i32;
                let l = if sign == 1 {
                    -(level_abs as i32)
                } else {
                    level_abs as i32
                };
                (last, run, l)
            }
            TcoefSym::Escape => {
                let last = br.read_u1()? == 1;
                let run = br.read_u32(6)? as u8;
                let raw = br.read_u32(8)?;
                let level: i32 = if raw == 0 {
                    return Err(Error::invalid("h263 AIV: escape level == 0"));
                } else if raw == 0x80 {
                    return Err(Error::invalid("h263 AIV: escape level == -128 forbidden"));
                } else if raw & 0x80 != 0 {
                    raw as i32 - 256
                } else {
                    raw as i32
                };
                (last, run, level)
            }
        };
        i = i.saturating_add(run as usize);
        if i > 63 {
            return Err(Error::invalid(
                "h263 AIV INTER overflow → fall back to INTRA",
            ));
        }
        let abs = level_signed.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if level_signed < 0 {
            val = -val;
        }
        let val = val.clamp(-2048, 2047);
        block[ZIGZAG[i]] = val;
        if last {
            return Ok(());
        }
        i += 1;
        if i > 63 {
            return Ok(());
        }
    }
}

/// Re-parse path that fires when [`try_decode_ac_inter`] overflows.
/// Reads the same codewords through Table I.2 (INTRA VLC) and writes
/// the dequantised coefficients with the H.263 inter dequantisation
/// formula.
fn try_decode_ac_intra(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    start: usize,
    quant: u32,
) -> Result<()> {
    let mut i: usize = start;
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    loop {
        if i > 63 {
            return Err(Error::invalid(
                "h263 AIV INTRA replay also overflows — bitstream is malformed",
            ));
        }
        let sym = crate::aic::decode_intra_tcoef(br)?;
        let (last, run, level_signed) = match sym {
            crate::aic::IntraTcoefSym::RunLevel {
                last,
                run,
                level_abs,
            } => {
                let sign = br.read_u1()? as i32;
                let l = if sign == 1 {
                    -(level_abs as i32)
                } else {
                    level_abs as i32
                };
                (last, run, l)
            }
            crate::aic::IntraTcoefSym::Escape => {
                let last = br.read_u1()? == 1;
                let run = br.read_u32(6)? as u8;
                let raw = br.read_u32(8)?;
                let level: i32 = if raw == 0 {
                    return Err(Error::invalid("h263 AIV INTRA: escape level == 0"));
                } else if raw == 0x80 {
                    return Err(Error::invalid(
                        "h263 AIV INTRA: escape level == -128 forbidden",
                    ));
                } else if raw & 0x80 != 0 {
                    raw as i32 - 256
                } else {
                    raw as i32
                };
                (last, run, level)
            }
        };
        i = i.saturating_add(run as usize);
        if i > 63 {
            return Err(Error::invalid(
                "h263 AIV INTRA replay also overflows — bitstream is malformed",
            ));
        }
        let abs = level_signed.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if level_signed < 0 {
            val = -val;
        }
        let val = val.clamp(-2048, 2047);
        block[ZIGZAG[i]] = val;
        if last {
            return Ok(());
        }
        i += 1;
        if i > 63 {
            return Ok(());
        }
    }
}

/// Variant of [`decode_ac`] that honours the **§T.4 EXTENDED-ESCAPE**
/// extension under Annex T (Modified Quantization).
///
/// Reads identically to [`decode_ac`] except that, after the 7-bit
/// ESCAPE prefix `0000011`, an 8-bit fixed-length value of `1000_0000`
/// (which is forbidden in baseline H.263) is now interpreted as the
/// EXTENDED-ESCAPE marker. When this marker fires, an additional 11-bit
/// EXTENDED-LEVEL field follows whose bits are cyclically right-rotated
/// by 5 to prevent start-code emulation
/// (see [`crate::mq::unrotate_extended_level`]).
///
/// §T.5 restricts EXTENDED-ESCAPE to `quant < 8` and to `|level| > 127`;
/// we surface §T.5 violations as `Error::invalid` rather than silently
/// accepting the malformed level (the spec mandates these are not used
/// outside the allowed range, and a bitstream that does so is suspect).
pub fn decode_ac_mq(
    br: &mut BitReader<'_>,
    block: &mut [i32; 64],
    start: usize,
    quant: u32,
) -> Result<()> {
    let table = inter_table();
    let mut i: usize = start;
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    loop {
        if i > 63 {
            return Err(Error::invalid("h263 MQ block: AC overrun"));
        }
        let sym = vlc::decode(br, table)?;
        let (last, run, level_signed) = match sym {
            TcoefSym::RunLevel {
                last,
                run,
                level_abs,
            } => {
                let sign = br.read_u1()? as i32;
                let l = if sign == 1 {
                    -(level_abs as i32)
                } else {
                    level_abs as i32
                };
                (last, run, l)
            }
            TcoefSym::Escape => {
                // Standard escape body: last(1) + run(6) + level(8 signed).
                // §T.4 promotes the previously-forbidden level byte
                // 1000_0000 to the EXTENDED-ESCAPE marker — when we see
                // it, read 11 more bits of cyclically-rotated EXTENDED-
                // LEVEL.
                let last = br.read_u1()? == 1;
                let run = br.read_u32(6)? as u8;
                let raw = br.read_u32(8)?;
                if raw == 0x80 {
                    // EXTENDED-ESCAPE.
                    if quant >= 8 {
                        return Err(Error::invalid(format!(
                            "h263 Annex T: EXTENDED-ESCAPE with QUANT={quant} \
                             (§T.5 mandates QUANT < 8)"
                        )));
                    }
                    let wire = br.read_u32(11)?;
                    let lv = crate::mq::unrotate_extended_level(wire);
                    if lv == 0 {
                        return Err(Error::invalid("h263 Annex T EXTENDED-ESCAPE: level == 0"));
                    }
                    if lv.abs() <= 127 {
                        return Err(Error::invalid(format!(
                            "h263 Annex T: EXTENDED-ESCAPE with |level|={} (§T.5 \
                             mandates |level| > 127)",
                            lv.abs()
                        )));
                    }
                    (last, run, lv)
                } else {
                    let level: i32 = if raw == 0 {
                        return Err(Error::invalid("h263 MQ block: escape level == 0"));
                    } else if raw & 0x80 != 0 {
                        raw as i32 - 256
                    } else {
                        raw as i32
                    };
                    (last, run, level)
                }
            }
        };
        i = i.saturating_add(run as usize);
        if i > 63 {
            return Err(Error::invalid("h263 MQ block: AC run overflow"));
        }
        // Dequantise: §6.2.1.
        let abs = level_signed.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if level_signed < 0 {
            val = -val;
        }
        // §T.5 — for any coefficient |REC| < 4096 (we already clip below).
        let val = val.clamp(-4095, 4095);
        block[ZIGZAG[i]] = val;
        if last {
            return Ok(());
        }
        i += 1;
        if i > 63 {
            return Ok(());
        }
    }
}

/// Run the float-domain IDCT on `block` and return clipped 8-bit pel samples
/// (0..=255).
pub fn idct_and_clip(block: &mut [i32; 64], out: &mut [u8; 64]) {
    let mut f = [0.0f32; 64];
    for i in 0..64 {
        f[i] = block[i] as f32;
    }
    oxideav_mpeg4video::block::idct8x8(&mut f);
    for i in 0..64 {
        let v = f[i].round() as i32;
        out[i] = if v < 0 {
            0
        } else if v > 255 {
            255
        } else {
            v as u8
        };
    }
}

/// Run the float-domain IDCT on `block` and return signed residual samples
/// clipped to the spec's inter-residual range `[-256, 255]`. Used by the
/// P-picture inter path where the output is added to a motion-compensated
/// predictor before the final 8-bit clip.
pub fn idct_signed(block: &mut [i32; 64], out: &mut [i32; 64]) {
    let mut f = [0.0f32; 64];
    for i in 0..64 {
        f[i] = block[i] as f32;
    }
    oxideav_mpeg4video::block::idct8x8(&mut f);
    for i in 0..64 {
        let v = f[i].round() as i32;
        out[i] = v.clamp(-256, 255);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intradc_basic() {
        let data = [0x10u8];
        let mut br = BitReader::new(&data);
        assert_eq!(decode_intradc(&mut br).unwrap(), 0x10 << 3);
    }

    #[test]
    fn intradc_special_ff() {
        let data = [0xFFu8];
        let mut br = BitReader::new(&data);
        assert_eq!(decode_intradc(&mut br).unwrap(), 1024);
    }

    #[test]
    fn intradc_zero_is_illegal() {
        let data = [0x00u8];
        let mut br = BitReader::new(&data);
        assert!(decode_intradc(&mut br).is_err());
    }

    #[test]
    fn intradc_0x80_is_illegal() {
        let data = [0x80u8];
        let mut br = BitReader::new(&data);
        assert!(decode_intradc(&mut br).is_err());
    }
}
