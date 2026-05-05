//! H.263 **Annex T** — Modified Quantization mode.
//!
//! Annex T modifies the quantizer in four ways (§T.1):
//!
//! 1. The DQUANT field becomes a variable-length code: 2 bits for a
//!    "small-step" alteration relative to the prior QUANT (§T.2.1 /
//!    Table T.1), or 6 bits for an arbitrary new QUANT (§T.2.2).
//! 2. Chrominance uses a smaller step size than luminance — controlled
//!    by `QUANT_C`, derived from the luma `QUANT` via Table T.2 (§T.3).
//! 3. The representable LEVEL range is widened past ±127 by an
//!    EXTENDED-ESCAPE code (§T.4): the bit sequence `1000 0000` after
//!    the 7-bit ESCAPE prefix introduces an 11-bit cyclically-rotated
//!    EXTENDED-LEVEL field.
//! 4. Restrictions on which `(LAST, RUN, LEVEL)` combinations may use
//!    the EXTENDED-ESCAPE — only when QUANT/QUANT_C < 8 and only when
//!    `|LEVEL| > 127` (§T.5).
//!
//! Round 25 lands the syntax-recognition path in the decoder + the
//! supporting tables; encoder emit is a follow-up.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

/// §T.2.1 Table T.1 — small-step QUANT alteration. Indexed by
/// `prior_quant` (1..=31). Each entry is `(delta_for_dquant_10,
/// delta_for_dquant_11)`.
const SMALL_STEP_TABLE: [(i32, i32); 32] = [
    (0, 0),   // 0 — invalid
    (2, 1),   // 1
    (-1, 1),  // 2
    (-1, 1),  // 3
    (-1, 1),  // 4
    (-1, 1),  // 5
    (-1, 1),  // 6
    (-1, 1),  // 7
    (-1, 1),  // 8
    (-1, 1),  // 9
    (-1, 1),  // 10
    (-2, 2),  // 11
    (-2, 2),  // 12
    (-2, 2),  // 13
    (-2, 2),  // 14
    (-2, 2),  // 15
    (-2, 2),  // 16
    (-2, 2),  // 17
    (-2, 2),  // 18
    (-2, 2),  // 19
    (-2, 2),  // 20
    (-3, 3),  // 21
    (-3, 3),  // 22
    (-3, 3),  // 23
    (-3, 3),  // 24
    (-3, 3),  // 25
    (-3, 3),  // 26
    (-3, 3),  // 27
    (-3, 3),  // 28
    (-3, 2),  // 29
    (-3, 1),  // 30
    (-3, -5), // 31
];

/// §T.3 Table T.2 — luminance QUANT → chrominance QUANT_C mapping.
/// `quant_c_for_quant(q)` returns `QUANT_C` where `q` is the luma
/// QUANT in `1..=31`. For `q < 1` or `q > 31` the function debug-asserts;
/// release builds clamp.
pub fn quant_c_for_quant(quant: u32) -> u32 {
    debug_assert!((1..=31).contains(&quant), "quant out of range: {quant}");
    let q = quant.clamp(1, 31);
    match q {
        1..=6 => q,
        7..=9 => q - 1,
        10..=11 => 9,
        12..=13 => 10,
        14..=15 => 11,
        16..=18 => 12,
        19..=21 => 13,
        22..=26 => 14,
        27..=31 => 15,
        _ => unreachable!(),
    }
}

/// §T.2 — decode the variable-length DQUANT field. Returns the new
/// `QUANT` value. `prior_quant` must be in `1..=31`.
///
/// The first bit of DQUANT distinguishes the two forms:
///
/// * `1` — small-step (1 more bit follows; codeword `10` or `11` selects
///   the delta from Table T.1 indexed by `prior_quant`).
/// * `0` — arbitrary (5 more bits follow encoding the new `QUANT`
///   directly, valid range `1..=31`; the `00000` value is reserved).
///
/// Returned QUANT is clamped to `1..=31`.
pub fn decode_dquant_mq(br: &mut BitReader<'_>, prior_quant: u32) -> Result<u32> {
    debug_assert!((1..=31).contains(&prior_quant));
    let first = br.read_u1()?;
    if first == 1 {
        let second = br.read_u1()?;
        let (delta_10, delta_11) = SMALL_STEP_TABLE[prior_quant.clamp(1, 31) as usize];
        let delta = if second == 0 { delta_10 } else { delta_11 };
        let new_q = (prior_quant as i32) + delta;
        Ok(new_q.clamp(1, 31) as u32)
    } else {
        let new_q = br.read_u32(5)?;
        if new_q == 0 {
            return Err(Error::invalid(
                "h263 Annex T DQUANT: arbitrary-form new QUANT == 0 is reserved (§5.1.19)",
            ));
        }
        Ok(new_q)
    }
}

/// §T.4 — perform the cyclic 5-bit right rotation that recovers an
/// 11-bit signed LEVEL from the on-the-wire EXTENDED-LEVEL field.
///
/// On the wire the bits are `b5 b4 b3 b2 b1 b11 b10 b9 b8 b7 b6` (the
/// caller will have read those as a single 11-bit unsigned value). The
/// canonical LEVEL bit order is `b11 b10 b9 b8 b7 b6 b5 b4 b3 b2 b1`.
/// We unscramble by left-rotating the on-the-wire bits by 5.
pub fn unrotate_extended_level(wire: u32) -> i32 {
    let w = wire & 0x7FF;
    // Top-5 of canonical = bottom-5 of wire (the rotation moved them).
    // wire = (canonical_top5 << 6) | (canonical_low6 << 0) WAS rotated
    // RIGHT by 5, i.e. canonical = (wire << 5 | wire >> 6) & 0x7FF.
    let canonical = ((w << 5) | (w >> 6)) & 0x7FF;
    // Sign-extend from 11 bits.
    if canonical & 0x400 != 0 {
        canonical as i32 - 0x800
    } else {
        canonical as i32
    }
}

/// Inverse of [`unrotate_extended_level`] — pack a signed LEVEL into the
/// 11-bit on-the-wire form. The caller is responsible for enforcing the
/// §T.5 restrictions (only when `|level| > 127` and only when QUANT < 8).
pub fn rotate_extended_level(level: i32) -> u32 {
    debug_assert!(
        (-1024..=1023).contains(&level),
        "level out of 11-bit range: {level}"
    );
    let canonical = (level & 0x7FF) as u32;
    // Right-rotate canonical by 5 to produce the wire form.
    ((canonical >> 5) | (canonical << 6)) & 0x7FF
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    #[test]
    fn quant_c_table_t2_spec_values() {
        // Spot-check Table T.2 (§T.3).
        assert_eq!(quant_c_for_quant(1), 1);
        assert_eq!(quant_c_for_quant(6), 6);
        assert_eq!(quant_c_for_quant(7), 6);
        assert_eq!(quant_c_for_quant(9), 8);
        assert_eq!(quant_c_for_quant(10), 9);
        assert_eq!(quant_c_for_quant(11), 9);
        assert_eq!(quant_c_for_quant(12), 10);
        assert_eq!(quant_c_for_quant(15), 11);
        assert_eq!(quant_c_for_quant(16), 12);
        assert_eq!(quant_c_for_quant(18), 12);
        assert_eq!(quant_c_for_quant(21), 13);
        assert_eq!(quant_c_for_quant(22), 14);
        assert_eq!(quant_c_for_quant(26), 14);
        assert_eq!(quant_c_for_quant(27), 15);
        assert_eq!(quant_c_for_quant(31), 15);
    }

    #[test]
    fn small_step_dquant_spec_example() {
        // §T.2.1 example: prior QUANT = 29, DQUANT = "11" → delta = +2 →
        // new QUANT = 31.
        let mut bw = BitWriter::new();
        bw.write_bits(0b11, 2);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let q = decode_dquant_mq(&mut br, 29).unwrap();
        assert_eq!(q, 31);
    }

    #[test]
    fn small_step_dquant_table_t1() {
        // QUANT = 1, "11" → delta +1 → new = 2.
        let bytes = bits(&[(0b11, 2)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_dquant_mq(&mut br, 1).unwrap(), 2);

        // QUANT = 1, "10" → delta +2 → new = 3.
        let bytes = bits(&[(0b10, 2)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_dquant_mq(&mut br, 1).unwrap(), 3);

        // QUANT = 31, "11" → delta -5 → new = 26.
        let bytes = bits(&[(0b11, 2)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_dquant_mq(&mut br, 31).unwrap(), 26);

        // QUANT = 15, "10" → delta -2 → new = 13.
        let bytes = bits(&[(0b10, 2)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_dquant_mq(&mut br, 15).unwrap(), 13);
    }

    #[test]
    fn arbitrary_dquant_spec_example() {
        // §T.2.2 example: code "001111" → new QUANT = 15. The first '0'
        // selects the arbitrary form; the next 5 bits "01111" carry the
        // value.
        let bytes = bits(&[(0b001111, 6)]);
        let mut br = BitReader::new(&bytes);
        assert_eq!(decode_dquant_mq(&mut br, 5).unwrap(), 15);
    }

    #[test]
    fn arbitrary_dquant_zero_is_rejected() {
        // First-bit 0, then five 0s = arbitrary new = 0 → reserved.
        let bytes = bits(&[(0b000000, 6)]);
        let mut br = BitReader::new(&bytes);
        let err = decode_dquant_mq(&mut br, 5).unwrap_err();
        assert!(err.to_string().contains("Annex T"));
    }

    #[test]
    fn extended_level_rotation_round_trips_within_11_bits() {
        for &level in &[-1024i32, -512, -200, -127, -1, 0, 1, 127, 200, 1023] {
            let wire = rotate_extended_level(level);
            assert!(wire <= 0x7FF, "wire out of 11-bit range: {wire}");
            let back = unrotate_extended_level(wire);
            assert_eq!(back, level, "level={level} wire=0x{wire:x}");
        }
    }

    fn bits(triples: &[(u32, u32)]) -> Vec<u8> {
        let mut bw = BitWriter::new();
        for &(v, n) in triples {
            bw.write_bits(v, n);
        }
        bw.finish()
    }
}
