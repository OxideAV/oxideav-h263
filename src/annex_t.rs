//! H.263 Annex T — Modified Quantization mode (§T.2 / §T.3).
//!
//! Annex T modifies quantiser operation when the PLUSPTYPE Modified
//! Quantization (MQ) bit (§5.1.4.2 OPPTYPE bit 14) is set. This module
//! implements the two decoder-side primitives that gate cleanly at the
//! wire / dequant boundary:
//!
//! * **§T.2 Modified DQUANT Update.** The DQUANT field (§5.3.6) is no
//!   longer a 2-bit fixed-length differential. It is a variable-length
//!   field of either two or six bits, selected by its first bit:
//!   * first bit `1` (§T.2.1, "Small-step QUANT alteration") — one more
//!     bit follows; the resulting QUANT is looked up in Table T.1 from
//!     the prior QUANT and that bit;
//!   * first bit `0` (§T.2.2, "Arbitrary QUANT selection") — five more
//!     bits follow giving a brand-new QUANT directly per §5.1.19.
//! * **§T.3 Altered chrominance step size.** Chrominance coefficients
//!   are inverse-quantised with `QUANT_C` rather than the luminance
//!   QUANT; the relationship is the Table T.2 lookup
//!   [`quant_c_from_quant`].
//!
//! Out of scope for this module (reported, not guessed): the §T.4
//! EXTENDED-ESCAPE / EXTENDED-LEVEL coefficient-range extension (it
//! lives in the §5.4.2 TCOEF VLC layer, not the DQUANT / dequant
//! boundary) and the §T.5 usage restrictions (encoder-side
//! constraints).
//!
//! All numeric facts are transcribed from ITU-T Recommendation H.263
//! (01/2005) Annex T, Tables T.1 and T.2.

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// §T.2.1 / Table T.1 — small-step QUANT alteration lookup.
///
/// Indexed `[second_bit][prior_quant]`. `second_bit == 0` is the
/// DQUANT codeword `"10"`; `second_bit == 1` is the codeword `"11"`.
/// The stored value is the **resulting** QUANT (the spec table lists
/// the change of QUANT; the resulting value is the prior QUANT plus
/// that change), already clipped into the legal `1..=31` range by the
/// table itself (e.g. prior QUANT 31 with codeword `"11"` maps to a
/// `−5` change → 26). Index 0 is unused: the prior QUANT is always in
/// `1..=31`.
///
/// Worked example from §T.2.1: prior QUANT 29, codeword `"11"`
/// (`second_bit == 1`) → `MODIFIED_QUANT_TAB[1][29] == 31`
/// (the change is `+2`).
const MODIFIED_QUANT_TAB: [[u8; 32]; 2] = [
    // second_bit == 0 → DQUANT codeword "10"
    [
        0, 3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 18, 19, 20, 21, 22,
        23, 24, 25, 26, 27, 28,
    ],
    // second_bit == 1 → DQUANT codeword "11"
    [
        0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27,
        28, 29, 30, 31, 31, 31, 26,
    ],
];

/// §T.3 / Table T.2 — chrominance quantiser `QUANT_C` from the
/// luminance `QUANT`.
///
/// Indexed by `QUANT` directly (`1..=31`). Index 0 is unused (QUANT is
/// always in `1..=31`). Transcribed from Table T.2:
///
/// ```text
///   QUANT 1-6   → QUANT_C = QUANT
///   QUANT 7-9   → QUANT_C = QUANT − 1
///   QUANT 10-11 → 9
///   QUANT 12-13 → 10
///   QUANT 14-15 → 11
///   QUANT 16-18 → 12
///   QUANT 19-21 → 13
///   QUANT 22-26 → 14
///   QUANT 27-31 → 15
/// ```
const QUANT_C_TAB: [u8; 32] = [
    0, // unused
    1, 2, 3, 4, 5, 6, // 1-6: QUANT_C = QUANT
    6, 7, 8, // 7-9: QUANT_C = QUANT − 1
    9, 9, // 10-11
    10, 10, // 12-13
    11, 11, // 14-15
    12, 12, 12, // 16-18
    13, 13, 13, // 19-21
    14, 14, 14, 14, 14, // 22-26
    15, 15, 15, 15, 15, // 27-31
];

/// Result of decoding an Annex T (§T.2) Modified DQUANT field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModifiedDquant {
    /// The new QUANT value (luminance quantiser) in force from this
    /// macroblock onward, always in `1..=31`.
    pub new_quant: u8,
    /// Number of bits consumed from the bitstream (`2` for the §T.2.1
    /// small-step form, `6` for the §T.2.2 arbitrary-selection form).
    pub bits_consumed: u8,
}

/// §T.3 / Table T.2 — derive the chrominance quantiser `QUANT_C` from
/// the luminance `QUANT`.
///
/// `quant` must be in `1..=31` (the §5.1.19 QUANT range); values
/// outside it return [`Error::InvalidQuantiser`]. When the Modified
/// Quantization mode is in use, chrominance coefficients are inverse-
/// quantised with the returned `QUANT_C` (and, if Annex J deblocking is
/// active, the chrominance deblocking filter uses it too — §T.3).
pub fn quant_c_from_quant(quant: u8) -> Result<u8> {
    if quant == 0 || quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    Ok(QUANT_C_TAB[quant as usize])
}

/// §T.2 — parse the Modified Quantization mode DQUANT field.
///
/// The field is variable length, two or six bits, selected by its first
/// bit (§T.2):
///
/// * first bit `1` (§T.2.1): one more bit `b` follows; `new_quant =
///   MODIFIED_QUANT_TAB[b][prior_quant]` (Table T.1). Two bits total.
/// * first bit `0` (§T.2.2): five more bits follow giving a brand-new
///   QUANT directly per §5.1.19. Six bits total. The five-bit value `0`
///   is rejected with [`Error::InvalidQuantiser`] (§5.1.19 limits QUANT
///   to `1..=31`).
///
/// `prior_quant` is the QUANT in force before this macroblock (the
/// GOB-layer / picture-layer QUANT, or the previous macroblock's
/// `quantiser_after`); it must be in `1..=31` or
/// [`Error::InvalidQuantiser`] is returned without consuming any bits.
///
/// On a truncated read [`Error::UnexpectedEof`] is returned.
pub fn parse_modified_dquant(
    reader: &mut BitReader<'_>,
    prior_quant: u8,
) -> Result<ModifiedDquant> {
    if prior_quant == 0 || prior_quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    let first = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if first {
        // §T.2.1 small-step alteration: one more bit selects the
        // Table T.1 column.
        let second = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        let col = usize::from(second);
        let new_quant = MODIFIED_QUANT_TAB[col][prior_quant as usize];
        Ok(ModifiedDquant {
            new_quant,
            bits_consumed: 2,
        })
    } else {
        // §T.2.2 arbitrary selection: five more bits give the new
        // QUANT directly per §5.1.19.
        let raw = reader.read_u32(5).map_err(|_| Error::UnexpectedEof)?;
        if raw == 0 {
            return Err(Error::InvalidQuantiser);
        }
        Ok(ModifiedDquant {
            new_quant: raw as u8,
            bits_consumed: 6,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn reader_from(bits: &[(u32, u32)]) -> Vec<u8> {
        let mut w = BitWriter::new();
        for &(value, n) in bits {
            w.write_u32(value, n);
        }
        w.finish()
    }

    // ---- §T.3 / Table T.2: QUANT_C derivation ----

    /// §T.3 Table T.2 rows 1-6: QUANT_C == QUANT (identity band).
    #[test]
    fn quant_c_identity_band() {
        for q in 1..=6u8 {
            assert_eq!(quant_c_from_quant(q).unwrap(), q);
        }
    }

    /// §T.3 Table T.2 rows 7-9: QUANT_C == QUANT − 1.
    #[test]
    fn quant_c_minus_one_band() {
        assert_eq!(quant_c_from_quant(7).unwrap(), 6);
        assert_eq!(quant_c_from_quant(8).unwrap(), 7);
        assert_eq!(quant_c_from_quant(9).unwrap(), 8);
    }

    /// §T.3 Table T.2 saturating bands: every range pins to the
    /// stated constant.
    #[test]
    fn quant_c_saturating_bands() {
        for q in 10..=11 {
            assert_eq!(quant_c_from_quant(q).unwrap(), 9);
        }
        for q in 12..=13 {
            assert_eq!(quant_c_from_quant(q).unwrap(), 10);
        }
        for q in 14..=15 {
            assert_eq!(quant_c_from_quant(q).unwrap(), 11);
        }
        for q in 16..=18 {
            assert_eq!(quant_c_from_quant(q).unwrap(), 12);
        }
        for q in 19..=21 {
            assert_eq!(quant_c_from_quant(q).unwrap(), 13);
        }
        for q in 22..=26 {
            assert_eq!(quant_c_from_quant(q).unwrap(), 14);
        }
        for q in 27..=31 {
            assert_eq!(quant_c_from_quant(q).unwrap(), 15);
        }
    }

    /// QUANT_C is monotone non-decreasing across the whole `1..=31`
    /// domain (a structural sanity check on the Table T.2 transcription).
    #[test]
    fn quant_c_is_monotone_non_decreasing() {
        let mut prev = quant_c_from_quant(1).unwrap();
        for q in 2..=31u8 {
            let cur = quant_c_from_quant(q).unwrap();
            assert!(
                cur >= prev,
                "QUANT_C decreased at QUANT={q}: {cur} < {prev}"
            );
            prev = cur;
        }
    }

    /// §5.1.19 QUANT domain: 0 and >31 are rejected.
    #[test]
    fn quant_c_rejects_out_of_range() {
        assert_eq!(quant_c_from_quant(0), Err(Error::InvalidQuantiser));
        assert_eq!(quant_c_from_quant(32), Err(Error::InvalidQuantiser));
        assert_eq!(quant_c_from_quant(255), Err(Error::InvalidQuantiser));
    }

    // ---- §T.2.1: small-step QUANT alteration ----

    /// §T.2.1 worked example: prior QUANT 29, codeword "11" → +2 → 31.
    #[test]
    fn small_step_worked_example_quant_29_code_11() {
        // first bit 1, second bit 1.
        let data = reader_from(&[(0b11, 2)]);
        let mut r = BitReader::new(&data);
        let out = parse_modified_dquant(&mut r, 29).unwrap();
        assert_eq!(out.new_quant, 31);
        assert_eq!(out.bits_consumed, 2);
        assert_eq!(r.bit_position(), 2);
    }

    /// §T.2.1 Table T.1: prior QUANT 1, "10" → +2 → 3; "11" → +1 → 2.
    #[test]
    fn small_step_quant_1_both_codes() {
        let data = reader_from(&[(0b10, 2)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 1).unwrap().new_quant, 3);

        let data = reader_from(&[(0b11, 2)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 1).unwrap().new_quant, 2);
    }

    /// §T.2.1 Table T.1 mid-range: prior QUANT 11, "10" → −2 → 9;
    /// "11" → +2 → 13.
    #[test]
    fn small_step_quant_11_both_codes() {
        let data = reader_from(&[(0b10, 2)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 11).unwrap().new_quant, 9);

        let data = reader_from(&[(0b11, 2)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 11).unwrap().new_quant, 13);
    }

    /// §T.2.1 Table T.1 boundary: prior QUANT 31, "10" → −3 → 28;
    /// "11" → −5 → 26 (the lone negative change in the "11" column).
    #[test]
    fn small_step_quant_31_both_codes() {
        let data = reader_from(&[(0b10, 2)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 31).unwrap().new_quant, 28);

        let data = reader_from(&[(0b11, 2)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 31).unwrap().new_quant, 26);
    }

    /// §T.2.1 result is always in the legal `1..=31` QUANT range, for
    /// every prior QUANT and both codewords (Table T.1 self-clips).
    #[test]
    fn small_step_result_always_in_range() {
        for prior in 1..=31u8 {
            for code in [0b10u32, 0b11u32] {
                let data = reader_from(&[(code, 2)]);
                let mut r = BitReader::new(&data);
                let out = parse_modified_dquant(&mut r, prior).unwrap();
                assert!(
                    (1..=31).contains(&out.new_quant),
                    "prior={prior} code={code:#04b} → {} out of range",
                    out.new_quant
                );
            }
        }
    }

    // ---- §T.2.2: arbitrary QUANT selection ----

    /// §T.2.2 worked example: codeword "001111" → new QUANT 15,
    /// regardless of prior QUANT.
    #[test]
    fn arbitrary_worked_example_001111_is_15() {
        for prior in [1u8, 7, 31] {
            // first bit 0, then five bits 0b01111 == 15.
            let data = reader_from(&[(0b0_01111, 6)]);
            let mut r = BitReader::new(&data);
            let out = parse_modified_dquant(&mut r, prior).unwrap();
            assert_eq!(out.new_quant, 15);
            assert_eq!(out.bits_consumed, 6);
            assert_eq!(r.bit_position(), 6);
        }
    }

    /// §T.2.2 endpoints: smallest legal QUANT (1) and largest (31).
    #[test]
    fn arbitrary_selection_endpoints() {
        let data = reader_from(&[(0b0_00001, 6)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 12).unwrap().new_quant, 1);

        let data = reader_from(&[(0b0_11111, 6)]);
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 12).unwrap().new_quant, 31);
    }

    /// §T.2.2 / §5.1.19: an arbitrary-selection value of 0 is illegal.
    #[test]
    fn arbitrary_selection_zero_rejected() {
        let data = reader_from(&[(0b0_00000, 6)]);
        let mut r = BitReader::new(&data);
        assert_eq!(
            parse_modified_dquant(&mut r, 12),
            Err(Error::InvalidQuantiser)
        );
    }

    // ---- error / edge paths ----

    /// Prior QUANT outside `1..=31` is rejected before any bit is read.
    #[test]
    fn rejects_out_of_range_prior_quant() {
        let data = reader_from(&[(0b11, 2)]);
        let mut r = BitReader::new(&data);
        assert_eq!(
            parse_modified_dquant(&mut r, 0),
            Err(Error::InvalidQuantiser)
        );
        // No bits consumed on the rejection.
        assert_eq!(r.bit_position(), 0);

        let mut r = BitReader::new(&data);
        assert_eq!(
            parse_modified_dquant(&mut r, 32),
            Err(Error::InvalidQuantiser)
        );
    }

    /// Empty buffer → EOF on the first bit.
    #[test]
    fn empty_buffer_returns_eof() {
        let data: [u8; 0] = [];
        let mut r = BitReader::new(&data);
        assert_eq!(parse_modified_dquant(&mut r, 12), Err(Error::UnexpectedEof));
    }

    /// §T.2.1: truncated after the leading `1` (no second bit) → EOF.
    /// The buffer is exactly one byte; we pre-advance the reader to its
    /// last bit so the small-step form (first bit `1`, then one more
    /// bit) runs off the end on its second read.
    #[test]
    fn small_step_truncated_returns_eof() {
        // One byte: bit 7 (the last) is `1` so the first read selects
        // the small-step form; there is no eighth-plus bit to follow.
        let data = [0b0000_0001u8];
        let mut r = BitReader::new(&data);
        r.read_u32(7).unwrap(); // consume the seven leading zero bits
        assert_eq!(parse_modified_dquant(&mut r, 12), Err(Error::UnexpectedEof));
        // The first bit (the `1`) was consumed before the EOF.
        assert_eq!(r.bit_position(), 8);
    }

    /// §T.2.2: truncated inside the five-bit arbitrary field → EOF.
    /// The single-byte buffer is positioned so only four bits remain;
    /// the arbitrary form (first bit `0`, then five value bits) runs
    /// off the end inside the five-bit field.
    #[test]
    fn arbitrary_truncated_returns_eof() {
        // One byte; pre-advance to bit 4 so four bits remain. First
        // read (bit 4) must be `0` to select the arbitrary form, then
        // only three of the five value bits are available.
        let data = [0b0000_0000u8];
        let mut r = BitReader::new(&data);
        r.read_u32(4).unwrap(); // consume four leading bits
        assert_eq!(parse_modified_dquant(&mut r, 12), Err(Error::UnexpectedEof));
    }

    /// The §T.2.1 vs §T.2.2 selector is the first bit only: a `0` first
    /// bit always consumes six bits, a `1` first bit always two — even
    /// when the same byte holds both encodings back to back.
    #[test]
    fn two_fields_back_to_back_decode_independently() {
        // Field 1: "11" small-step (prior 5 → +1 → 6).
        // Field 2: "0 01010" arbitrary → 10.
        let data = reader_from(&[(0b11, 2), (0b0_01010, 6)]);
        let mut r = BitReader::new(&data);
        let f1 = parse_modified_dquant(&mut r, 5).unwrap();
        assert_eq!(f1.new_quant, 6);
        assert_eq!(f1.bits_consumed, 2);
        let f2 = parse_modified_dquant(&mut r, f1.new_quant).unwrap();
        assert_eq!(f2.new_quant, 10);
        assert_eq!(f2.bits_consumed, 6);
        assert_eq!(r.bit_position(), 8);
    }
}
