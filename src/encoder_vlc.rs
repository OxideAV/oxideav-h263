//! H.263 variable-length-code **encode** primitives — the inverse of
//! the decoder's Table 7/8 (MCBPC), Table 12 (CBPY), Table 14 (MVD),
//! Table 15 (INTRADC) and Table 16 (TCOEF) lookups.
//!
//! Each function here takes a decoded symbol (the same Rust value the
//! decoder produces) and appends its canonical codeword to an
//! [`oxideav_core::bits::BitWriter`] (MSB-first, matching the H.263
//! wire). The codewords are the verbatim spec tables, so the pairing
//!
//! ```text
//!   encode(sym) ; reader.decode() == sym
//! ```
//!
//! holds by construction for every symbol the decoder can produce.
//!
//! All truth is ITU-T Recommendation H.263 (01/2005); the codeword
//! inventories mirror the decoder's tables in [`crate::block`] and
//! [`crate::macroblock`] (the same numeric table data, written in the
//! emit direction).
//!
//! The codeword literals are grouped to match the spec's printed
//! bit-fields (e.g. `0000_011` for the 7-bit ESCAPE prefix), so the
//! `unusual_byte_groupings` lint is allowed module-wide.
#![allow(clippy::unusual_byte_groupings)]

use crate::macroblock::MbType;
use crate::{Error, Result};
use oxideav_core::bits::BitWriter;

/// §5.3.2 / Table 7 — emit the I-picture MCBPC codeword for a macroblock
/// `(mb_type, cbpc)`. `cbpc` is the 2-bit chrominance coded-block
/// pattern (`0b10` = Cb, `0b01` = Cr).
///
/// I-pictures may only carry INTRA / INTRA+Q / Stuffing; any other type
/// is rejected with [`Error::BadMcbpcCode`].
pub fn write_mcbpc_i(w: &mut BitWriter, mb_type: MbType, cbpc: u8) -> Result<()> {
    // (bits, code) — the verbatim Table 7 codewords, indexed by
    // (mb_type, cbpc). See [`crate::macroblock::decode_mcbpc_i`] for the
    // matching decode side.
    let (bits, code): (u32, u32) = match (mb_type, cbpc) {
        (MbType::Intra, 0b00) => (1, 0b1),
        (MbType::Intra, 0b10) => (3, 0b010),
        (MbType::Intra, 0b11) => (3, 0b011),
        (MbType::Intra, 0b01) => (3, 0b001),
        (MbType::IntraQ, 0b00) => (4, 0b0001),
        (MbType::IntraQ, 0b10) => (6, 0b000010),
        (MbType::IntraQ, 0b11) => (6, 0b000011),
        (MbType::IntraQ, 0b01) => (6, 0b000001),
        (MbType::Stuffing, _) => (9, 0b0000_0000_1),
        _ => return Err(Error::BadMcbpcCode),
    };
    w.write_bits(code, bits);
    Ok(())
}

/// §5.3.2 / Table 8 — emit the P-picture MCBPC codeword for a macroblock
/// `(mb_type, cbpc)`.
pub fn write_mcbpc_p(w: &mut BitWriter, mb_type: MbType, cbpc: u8) -> Result<()> {
    // (bits, code) — the verbatim Table 8 codewords, indexed by
    // (mb_type, cbpc). See [`crate::macroblock::decode_mcbpc_p`].
    let (bits, code): (u32, u32) = match (mb_type, cbpc) {
        (MbType::Inter, 0b00) => (1, 0b1),
        (MbType::InterQ, 0b00) => (3, 0b011),
        (MbType::Inter4V, 0b00) => (3, 0b010),
        (MbType::Inter, 0b01) => (4, 0b0011),
        (MbType::Inter, 0b10) => (4, 0b0010),
        (MbType::Intra, 0b00) => (5, 0b00011),
        (MbType::IntraQ, 0b00) => (6, 0b000100),
        (MbType::Inter, 0b11) => (6, 0b000101),
        (MbType::InterQ, 0b01) => (7, 0b0000111),
        (MbType::InterQ, 0b10) => (7, 0b0000110),
        (MbType::Inter4V, 0b01) => (7, 0b0000101),
        (MbType::Inter4V, 0b10) => (7, 0b0000100),
        (MbType::Intra, 0b11) => (7, 0b0000011),
        (MbType::Inter4V, 0b11) => (8, 0b00000101),
        (MbType::Intra, 0b01) => (8, 0b00000100),
        (MbType::Intra, 0b10) => (8, 0b00000011),
        (MbType::InterQ, 0b11) => (9, 0b000000101),
        (MbType::IntraQ, 0b01) => (9, 0b000000100),
        // lz=7 + terminating `1` + 1-bit suffix.
        (MbType::IntraQ, 0b10) => (9, 0b000000011),
        (MbType::IntraQ, 0b11) => (9, 0b000000010),
        // lz=8 + terminating `1`.
        (MbType::Stuffing, _) => (9, 0b000000001),
        // lz=9 + terminating `1` + suffix(es).
        (MbType::Inter4VQ, 0b00) => (11, 0b00000000010),
        (MbType::Inter4VQ, 0b01) => (13, 0b0000000001100),
        (MbType::Inter4VQ, 0b10) => (13, 0b0000000001110),
        (MbType::Inter4VQ, 0b11) => (13, 0b0000000001111),
        _ => return Err(Error::BadMcbpcCode),
    };
    w.write_bits(code, bits);
    Ok(())
}

/// §5.3.5 / Table 12 — emit the CBPY codeword for the **INTRA-orientation**
/// 4-bit pattern `cbpy` (bit 3 = block 1, bit 0 = block 4). INTER
/// macroblocks must complement the pattern (`cbpy ^ 0b1111`) before
/// calling, matching the decoder's `is_intra` complement rule.
pub fn write_cbpy(w: &mut BitWriter, cbpy: u8) -> Result<()> {
    // (bits, code) keyed by the INTRA-orientation pattern, verbatim
    // Table 12. See [`crate::macroblock::decode_cbpy`].
    let (bits, code): (u32, u32) = match cbpy & 0b1111 {
        0b0000 => (4, 0b0011),
        0b0001 => (5, 0b00101),
        0b0010 => (5, 0b00100),
        0b0011 => (4, 0b1001),
        0b0100 => (5, 0b00011),
        0b0101 => (4, 0b0111),
        0b0110 => (6, 0b000010),
        0b0111 => (4, 0b1011),
        0b1000 => (5, 0b00010),
        0b1001 => (6, 0b000011),
        0b1010 => (4, 0b0101),
        0b1011 => (4, 0b1010),
        0b1100 => (4, 0b0100),
        0b1101 => (4, 0b1000),
        0b1110 => (4, 0b0110),
        0b1111 => (2, 0b11),
        _ => unreachable!("cbpy masked to 4 bits"),
    };
    w.write_bits(code, bits);
    Ok(())
}

/// §5.4.1 / Table 15 — emit the 8-bit INTRADC FLC for a DC reconstruction
/// level `rec` (the value the decoder reconstructs into block slot 0).
///
/// `rec` must be a legal Table 15 reconstruction level: an exact
/// multiple of 8 in `8..=2040` (giving a code `1..=255` excluding 128),
/// or the special value `1024` (wire `0xFF`). Other values are rejected
/// with [`Error::BadIntradcCode`].
pub fn write_intradc(w: &mut BitWriter, rec: i16) -> Result<()> {
    let code: u8 = if rec == 1024 {
        0xFF
    } else if rec > 0 && rec % 8 == 0 && (rec / 8) <= 254 {
        let c = (rec / 8) as u8;
        // 0x00 and 0x80 are forbidden Table 15 codes.
        if c == 0x00 || c == 0x80 {
            return Err(Error::BadIntradcCode);
        }
        c
    } else {
        return Err(Error::BadIntradcCode);
    };
    w.write_bits(code as u32, 8);
    Ok(())
}

/// §5.3.7 / Table 14 — emit the MVD-component codeword for a half-pel
/// value `half` in the spec range `[-32, +31]` (i.e. vector `-16 .. +15.5`).
///
/// Out-of-range values are rejected with [`Error::BadMvdCode`].
pub fn write_mvd_component(w: &mut BitWriter, half: i8) -> Result<()> {
    let (bits, code) = mvd_codeword(half).ok_or(Error::BadMvdCode)?;
    w.write_bits(code, bits);
    Ok(())
}

/// Table 14 codeword `(bits, code)` for a half-pel MVD value, or `None`
/// if out of range. The inventory is the verbatim Table 14 transcription
/// (the same data as [`crate::macroblock::decode_mvd_component`]).
fn mvd_codeword(half: i8) -> Option<(u32, u32)> {
    let row: (u32, u32) = match half {
        -32 => (13, 0b0_0000_0000_0010_1),
        -31 => (13, 0b0_0000_0000_0011_1),
        -30 => (12, 0b0000_0000_0101),
        -29 => (12, 0b0000_0000_0111),
        -28 => (12, 0b0000_0000_1001),
        -27 => (12, 0b0000_0000_1011),
        -26 => (12, 0b0000_0000_1101),
        -25 => (12, 0b0000_0000_1111),
        -24 => (11, 0b0000_0001_001),
        -23 => (11, 0b0000_0001_011),
        -22 => (11, 0b0000_0001_101),
        -21 => (11, 0b0000_0001_111),
        -20 => (11, 0b0000_0010_001),
        -19 => (11, 0b0000_0010_011),
        -18 => (11, 0b0000_0010_101),
        -17 => (11, 0b0000_0010_111),
        -16 => (11, 0b0000_0011_001),
        -15 => (11, 0b0000_0011_011),
        -14 => (11, 0b0000_0011_101),
        -13 => (11, 0b0000_0011_111),
        -12 => (11, 0b0000_0100_001),
        -11 => (11, 0b0000_0100_011),
        -10 => (10, 0b0000_0100_11),
        -9 => (10, 0b0000_0101_01),
        -8 => (10, 0b0000_0101_11),
        -7 => (8, 0b0000_0111),
        -6 => (8, 0b0000_1001),
        -5 => (8, 0b0000_1011),
        -4 => (7, 0b000_0111),
        -3 => (5, 0b0001_1),
        -2 => (4, 0b0011),
        -1 => (3, 0b011),
        0 => (1, 0b1),
        1 => (3, 0b010),
        2 => (4, 0b0010),
        3 => (5, 0b0001_0),
        4 => (7, 0b000_0110),
        5 => (8, 0b0000_1010),
        6 => (8, 0b0000_1000),
        7 => (8, 0b0000_0110),
        8 => (10, 0b0000_0101_10),
        9 => (10, 0b0000_0101_00),
        10 => (10, 0b0000_0100_10),
        11 => (11, 0b0000_0100_010),
        12 => (11, 0b0000_0100_000),
        13 => (11, 0b0000_0011_110),
        14 => (11, 0b0000_0011_100),
        15 => (11, 0b0000_0011_010),
        16 => (11, 0b0000_0011_000),
        17 => (11, 0b0000_0010_110),
        18 => (11, 0b0000_0010_100),
        19 => (11, 0b0000_0010_010),
        20 => (11, 0b0000_0010_000),
        21 => (11, 0b0000_0001_110),
        22 => (11, 0b0000_0001_100),
        23 => (11, 0b0000_0001_010),
        24 => (11, 0b0000_0001_000),
        25 => (12, 0b0000_0000_1110),
        26 => (12, 0b0000_0000_1100),
        27 => (12, 0b0000_0000_1010),
        28 => (12, 0b0000_0000_1000),
        29 => (12, 0b0000_0000_0110),
        30 => (12, 0b0000_0000_0100),
        31 => (13, 0b0_0000_0000_0011_0),
        _ => return None,
    };
    Some(row)
}

/// A single (LAST, RUN, LEVEL) TCOEF event to emit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TcoefEvent {
    /// `true` if this is the last event of the block.
    pub last: bool,
    /// Number of zero coefficients preceding the LEVEL (0..=63).
    pub run: u8,
    /// Signed quantised LEVEL (non-zero).
    pub level: i16,
}

/// §5.4.2 / Table 16 — emit a TCOEF event. A `(last, run, |level|)`
/// triple that has a dedicated VLC code-point uses it (followed by the
/// sign bit); anything else takes the ESCAPE form
/// (`0000 011` + 1-bit LAST + 6-bit RUN + 8-bit two's-complement LEVEL).
///
/// `level` must be non-zero. The ESCAPE LEVEL `0x00` and `0x80` are
/// forbidden by §5.4.2, so an ESCAPE LEVEL magnitude must fit signed
/// 8-bit excluding `-128`; out-of-range events are rejected with
/// [`Error::BadTcoefCode`].
pub fn write_tcoef(w: &mut BitWriter, ev: TcoefEvent) -> Result<()> {
    if ev.level == 0 {
        return Err(Error::BadTcoefCode);
    }
    let abs_level = ev.level.unsigned_abs();
    let sign = ev.level < 0;

    // Try the dedicated VLC code-point first.
    if abs_level <= u8::MAX as u16 {
        if let Some((bits, code)) = tcoef_vlc_codeword(ev.last, ev.run, abs_level as u8) {
            w.write_bits(code, bits);
            w.write_bit(sign);
            return Ok(());
        }
    }

    // ESCAPE form. §5.4.2: 7-bit prefix `0000 011`, then 1-bit LAST,
    // 6-bit RUN, 8-bit two's-complement LEVEL.
    if ev.run > 0x3F {
        return Err(Error::BadTcoefCode);
    }
    // The 8-bit LEVEL field is two's complement; `0x00` and `0x80` are
    // forbidden, so the representable magnitudes are `1..=127`.
    if !(1..=127).contains(&abs_level) {
        return Err(Error::BadTcoefCode);
    }
    w.write_bits(0b0000_011, 7);
    w.write_bit(ev.last);
    w.write_bits(ev.run as u32, 6);
    let level_byte = (ev.level as i8) as u8;
    w.write_bits(level_byte as u32, 8);
    Ok(())
}

/// Table 16 dedicated `(bits, code)` codeword (sign bit excluded) for a
/// regular `(last, run, |level|)` event, or `None` if the triple has no
/// dedicated VLC entry (i.e. it must use the ESCAPE form).
///
/// `bits` is the prefix length **without** the trailing sign bit. The
/// inventory is the verbatim Table 16 transcription (the same data as
/// [`crate::block`]'s `TCOEF_TABLE`).
fn tcoef_vlc_codeword(last: bool, run: u8, abs_level: u8) -> Option<(u32, u32)> {
    // Each row is (last, run, |level|, prefix_bits_without_sign, code).
    // The `prefix_bits_without_sign` equals the spec's printed "Bits"
    // column minus 1 (the trailing sign `s`).
    const ROWS: &[(bool, u8, u8, u32, u32)] = &[
        // LAST=0, RUN=0
        (false, 0, 1, 2, 0b10),
        (false, 0, 2, 4, 0b1111),
        (false, 0, 3, 6, 0b0101_01),
        (false, 0, 4, 7, 0b0010_111),
        (false, 0, 5, 8, 0b0001_1111),
        (false, 0, 6, 9, 0b0001_0010_1),
        (false, 0, 7, 9, 0b0001_0010_0),
        (false, 0, 8, 10, 0b0000_1000_01),
        (false, 0, 9, 10, 0b0000_1000_00),
        (false, 0, 10, 11, 0b0000_0000_111),
        (false, 0, 11, 11, 0b0000_0000_110),
        (false, 0, 12, 11, 0b0000_0100_000),
        // LAST=0, RUN=1
        (false, 1, 1, 3, 0b110),
        (false, 1, 2, 6, 0b0101_00),
        (false, 1, 3, 8, 0b0001_1110),
        (false, 1, 4, 10, 0b0000_0011_11),
        (false, 1, 5, 11, 0b0000_0100_001),
        (false, 1, 6, 12, 0b0000_0101_0000),
        // LAST=0, RUN=2
        (false, 2, 1, 4, 0b1110),
        (false, 2, 2, 8, 0b0001_1101),
        (false, 2, 3, 10, 0b0000_0011_10),
        (false, 2, 4, 12, 0b0000_0101_0001),
        // LAST=0, RUN=3
        (false, 3, 1, 5, 0b0110_1),
        (false, 3, 2, 9, 0b0001_0001_1),
        (false, 3, 3, 10, 0b0000_0011_01),
        // LAST=0, RUN=4
        (false, 4, 1, 5, 0b0110_0),
        (false, 4, 2, 9, 0b0001_0001_0),
        (false, 4, 3, 12, 0b0000_0101_0010),
        // LAST=0, RUN=5
        (false, 5, 1, 5, 0b0101_1),
        (false, 5, 2, 10, 0b0000_0011_00),
        (false, 5, 3, 12, 0b0000_0101_0011),
        // LAST=0, RUN=6
        (false, 6, 1, 6, 0b0100_11),
        (false, 6, 2, 10, 0b0000_0010_11),
        (false, 6, 3, 12, 0b0000_0101_0100),
        // LAST=0, RUN=7
        (false, 7, 1, 6, 0b0100_10),
        (false, 7, 2, 10, 0b0000_0010_10),
        // LAST=0, RUN=8
        (false, 8, 1, 6, 0b0100_01),
        (false, 8, 2, 10, 0b0000_0010_01),
        // LAST=0, RUN=9
        (false, 9, 1, 6, 0b0100_00),
        (false, 9, 2, 10, 0b0000_0010_00),
        // LAST=0, RUN=10
        (false, 10, 1, 7, 0b0010_110),
        (false, 10, 2, 12, 0b0000_0101_0101),
        // LAST=0, RUN=11..16 (|level|=1)
        (false, 11, 1, 7, 0b0010_101),
        (false, 12, 1, 7, 0b0010_100),
        (false, 13, 1, 8, 0b0001_1100),
        (false, 14, 1, 8, 0b0001_1011),
        (false, 15, 1, 9, 0b0001_0000_1),
        (false, 16, 1, 9, 0b0001_0000_0),
        // LAST=0, RUN=17..26 (|level|=1)
        (false, 17, 1, 9, 0b0000_1111_1),
        (false, 18, 1, 9, 0b0000_1111_0),
        (false, 19, 1, 9, 0b0000_1110_1),
        (false, 20, 1, 9, 0b0000_1110_0),
        (false, 21, 1, 9, 0b0000_1101_1),
        (false, 22, 1, 9, 0b0000_1101_0),
        (false, 23, 1, 11, 0b0000_0100_010),
        (false, 24, 1, 11, 0b0000_0100_011),
        (false, 25, 1, 12, 0b0000_0101_0110),
        (false, 26, 1, 12, 0b0000_0101_0111),
        // LAST=1, RUN=0
        (true, 0, 1, 4, 0b0111),
        (true, 0, 2, 9, 0b0000_1100_1),
        (true, 0, 3, 11, 0b0000_0000_101),
        // LAST=1, RUN=1
        (true, 1, 1, 6, 0b0011_11),
        (true, 1, 2, 11, 0b0000_0000_100),
        // LAST=1, RUN=2..8 (|level|=1)
        (true, 2, 1, 6, 0b0011_10),
        (true, 3, 1, 6, 0b0011_01),
        (true, 4, 1, 6, 0b0011_00),
        (true, 5, 1, 7, 0b0010_011),
        (true, 6, 1, 7, 0b0010_010),
        (true, 7, 1, 7, 0b0010_001),
        (true, 8, 1, 7, 0b0010_000),
        // LAST=1, RUN=9..16 (|level|=1)
        (true, 9, 1, 8, 0b0001_1010),
        (true, 10, 1, 8, 0b0001_1001),
        (true, 11, 1, 8, 0b0001_1000),
        (true, 12, 1, 8, 0b0001_0111),
        (true, 13, 1, 8, 0b0001_0110),
        (true, 14, 1, 8, 0b0001_0101),
        (true, 15, 1, 8, 0b0001_0100),
        (true, 16, 1, 8, 0b0001_0011),
        // LAST=1, RUN=17..24 (|level|=1)
        (true, 17, 1, 9, 0b0000_1100_0),
        (true, 18, 1, 9, 0b0000_1011_1),
        (true, 19, 1, 9, 0b0000_1011_0),
        (true, 20, 1, 9, 0b0000_1010_1),
        (true, 21, 1, 9, 0b0000_1010_0),
        (true, 22, 1, 9, 0b0000_1001_1),
        (true, 23, 1, 9, 0b0000_1001_0),
        (true, 24, 1, 9, 0b0000_1000_1),
        // LAST=1, RUN=25..28 (|level|=1)
        (true, 25, 1, 10, 0b0000_0001_11),
        (true, 26, 1, 10, 0b0000_0001_10),
        (true, 27, 1, 10, 0b0000_0001_01),
        (true, 28, 1, 10, 0b0000_0001_00),
        // LAST=1, RUN=29..32 (|level|=1)
        (true, 29, 1, 11, 0b0000_0100_100),
        (true, 30, 1, 11, 0b0000_0100_101),
        (true, 31, 1, 11, 0b0000_0100_110),
        (true, 32, 1, 11, 0b0000_0100_111),
        // LAST=1, RUN=33..40 (|level|=1)
        (true, 33, 1, 12, 0b0000_0101_1000),
        (true, 34, 1, 12, 0b0000_0101_1001),
        (true, 35, 1, 12, 0b0000_0101_1010),
        (true, 36, 1, 12, 0b0000_0101_1011),
        (true, 37, 1, 12, 0b0000_0101_1100),
        (true, 38, 1, 12, 0b0000_0101_1101),
        (true, 39, 1, 12, 0b0000_0101_1110),
        (true, 40, 1, 12, 0b0000_0101_1111),
    ];
    for &(l, r, a, bits, code) in ROWS {
        if l == last && r == run && a == abs_level {
            return Some((bits, code));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::{parse_block, BlockContext};
    use crate::macroblock::{decode_cbpy, decode_mvd_component};
    use oxideav_core::bits::BitReader;

    // Round-trip every I-picture MCBPC code through the decoder.
    #[test]
    fn mcbpc_i_round_trips() {
        let cases = [
            (MbType::Intra, 0b00u8),
            (MbType::Intra, 0b10),
            (MbType::Intra, 0b11),
            (MbType::Intra, 0b01),
            (MbType::IntraQ, 0b00),
            (MbType::IntraQ, 0b10),
            (MbType::IntraQ, 0b11),
            (MbType::IntraQ, 0b01),
        ];
        for (ty, cbpc) in cases {
            let mut w = BitWriter::new();
            write_mcbpc_i(&mut w, ty, cbpc).unwrap();
            // Decode via the public parse_macroblock path is heavier; use
            // the internal decode through a fabricated reader by routing
            // a whole INTRA macroblock would be overkill — instead verify
            // bit-length consistency by re-reading the unary prefix.
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            let (dty, dcbpc) = crate::macroblock::decode_mcbpc_for_test(
                &mut r,
                crate::H263PictureCodingType::Intra,
            )
            .unwrap();
            assert_eq!((dty, dcbpc), (ty, cbpc));
        }
    }

    #[test]
    fn mcbpc_p_round_trips() {
        let cases = [
            (MbType::Inter, 0b00u8),
            (MbType::InterQ, 0b00),
            (MbType::Inter4V, 0b00),
            (MbType::Inter, 0b01),
            (MbType::Inter, 0b10),
            (MbType::Intra, 0b00),
            (MbType::IntraQ, 0b00),
            (MbType::Inter, 0b11),
            (MbType::InterQ, 0b01),
            (MbType::InterQ, 0b10),
            (MbType::Inter4V, 0b01),
            (MbType::Inter4V, 0b10),
            (MbType::Intra, 0b11),
            (MbType::Inter4V, 0b11),
            (MbType::Intra, 0b01),
            (MbType::Intra, 0b10),
            (MbType::InterQ, 0b11),
            (MbType::IntraQ, 0b01),
            (MbType::IntraQ, 0b10),
            (MbType::IntraQ, 0b11),
            (MbType::Inter4VQ, 0b00),
            (MbType::Inter4VQ, 0b01),
            (MbType::Inter4VQ, 0b10),
            (MbType::Inter4VQ, 0b11),
        ];
        for (ty, cbpc) in cases {
            let mut w = BitWriter::new();
            write_mcbpc_p(&mut w, ty, cbpc).unwrap();
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            let (dty, dcbpc) = crate::macroblock::decode_mcbpc_for_test(
                &mut r,
                crate::H263PictureCodingType::Inter,
            )
            .unwrap();
            assert_eq!((dty, dcbpc), (ty, cbpc));
        }
    }

    #[test]
    fn cbpy_round_trips_all_16() {
        for pat in 0u8..16 {
            let mut w = BitWriter::new();
            write_cbpy(&mut w, pat).unwrap();
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(decode_cbpy(&mut r).unwrap(), pat);
        }
    }

    #[test]
    fn mvd_round_trips_full_range() {
        for half in -32i8..=31 {
            let mut w = BitWriter::new();
            write_mvd_component(&mut w, half).unwrap();
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(decode_mvd_component(&mut r).unwrap(), half);
        }
    }

    #[test]
    fn intradc_round_trips_legal_levels() {
        // Every legal Table 15 reconstruction level: 8*code for
        // code in 1..=254 excluding 128, plus 1024.
        let mut levels: Vec<i16> = (1..=254i16).filter(|&c| c != 128).map(|c| c * 8).collect();
        levels.push(1024);
        for lvl in levels {
            let mut w = BitWriter::new();
            write_intradc(&mut w, lvl).unwrap();
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            let block = parse_block(
                &mut r,
                BlockContext {
                    has_intradc: true,
                    has_coefficients: false,
                    modified_quant: false,
                },
            )
            .unwrap();
            assert_eq!(block.coefficients[0], lvl);
        }
    }

    #[test]
    fn intradc_rejects_illegal() {
        let mut w = BitWriter::new();
        assert!(write_intradc(&mut w, 7).is_err()); // not multiple of 8
        assert!(write_intradc(&mut w, 0).is_err());
        assert!(write_intradc(&mut w, 2040).is_err()); // code 255 > 254
        assert!(write_intradc(&mut w, -8).is_err()); // negative
    }

    // Round-trip a single TCOEF event by writing it (with a terminating
    // LAST) and parsing the resulting block.
    fn roundtrip_tcoef_block(events: &[TcoefEvent]) -> crate::block::H263Block {
        let mut w = BitWriter::new();
        for &ev in events {
            write_tcoef(&mut w, ev).unwrap();
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
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

    #[test]
    fn tcoef_single_dc_level_round_trips() {
        // (last=true, run=0, level=+1) -> coefficient[0] = 1.
        let block = roundtrip_tcoef_block(&[TcoefEvent {
            last: true,
            run: 0,
            level: 1,
        }]);
        assert_eq!(block.coefficients[0], 1);
        assert_eq!(block.tcoef_event_count, 1);
    }

    #[test]
    fn tcoef_negative_level_round_trips() {
        let block = roundtrip_tcoef_block(&[TcoefEvent {
            last: true,
            run: 3,
            level: -2,
        }]);
        assert_eq!(block.coefficients[3], -2);
    }

    #[test]
    fn tcoef_escape_level_round_trips() {
        // |level| = 50 with run=0, last=true has no dedicated VLC ->
        // ESCAPE form.
        let block = roundtrip_tcoef_block(&[TcoefEvent {
            last: true,
            run: 0,
            level: 50,
        }]);
        assert_eq!(block.coefficients[0], 50);
    }

    #[test]
    fn tcoef_multi_event_round_trips() {
        let block = roundtrip_tcoef_block(&[
            TcoefEvent {
                last: false,
                run: 0,
                level: 3,
            },
            TcoefEvent {
                last: false,
                run: 2,
                level: -1,
            },
            TcoefEvent {
                last: true,
                run: 5,
                level: 1,
            },
        ]);
        assert_eq!(block.coefficients[0], 3);
        assert_eq!(block.coefficients[3], -1);
        assert_eq!(block.coefficients[9], 1);
        assert_eq!(block.tcoef_event_count, 3);
    }
}
