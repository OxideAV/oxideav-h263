//! Annex I — Advanced INTRA Coding mode: scan + prediction-mode layer.
//!
//! This module implements the *scan-and-mode* foundation of the
//! optional Advanced INTRA Coding (AIC) mode (ITU-T H.263, Annex I).
//! AIC alters the decoding of macroblocks of type INTRA only (§I.1);
//! its three coding-efficiency tools are
//!
//! 1. INTRA-block prediction from neighbouring INTRA blocks,
//! 2. modified inverse quantization for INTRA coefficients, and
//! 3. a separate VLC for INTRA coefficients (Table I.2).
//!
//! This round lands the parts that gate the rest:
//!
//! * The **INTRA_MODE** field (§I.2, Table I.1) — a 1-or-2-bit VLC
//!   transmitted once per INTRA macroblock when AIC is in use,
//!   signalling one of three prediction modes. Decoded by
//!   [`decode_intra_mode`] into [`IntraMode`].
//! * The two **alternate DCT scanning patterns** (§I.3, Figure I.2):
//!   the Alternate-Horizontal scan ([`ALT_HORIZONTAL_TO_BLOCK_POS`])
//!   and the Alternate-Vertical scan ([`ALT_VERTICAL_TO_BLOCK_POS`]),
//!   each expressed as a scan-position → block-position permutation in
//!   the same convention as
//!   [`crate::block::ZIGZAG_TO_BLOCK_POS`] (Figure 14).
//! * The §I.3 **scan-selection rule**: prediction-mode → active scan
//!   table ([`scan_for_intra_mode`]). Mode 0 (DC-only) keeps the
//!   Figure-14 zigzag scan; modes 1 and 2 switch to the
//!   alternate-horizontal and alternate-vertical scans respectively.
//!
//! What this module does **not** yet provide (later rounds):
//!
//! * The Table I.2 separate INTRA-coefficient VLC landed in round 14
//!   ([`crate::intra_tcoef::decode_intra_tcoef_event`]); the §I.3
//!   modified inverse-quantisation residual formula
//!   `RecC(u,v) = 2·QUANT·LEVEL(u,v)` and the `oddifyclipDC` / `clipAC`
//!   clipping primitives landed in round 17
//!   ([`crate::aic_dequant`]). The variable-step INTRADC reconstruction
//!   is a parser-side reframing (§I.3 line 4214: INTRADC absorbed into
//!   the per-block coefficient stream) and stays deferred.
//! * The DC/AC prediction reconstruction (modes 0/1/2 add a
//!   predictor sourced from the block above / to the left) and the
//!   "same video picture segment" neighbour-availability rule, which
//!   both need the macroblock-grid driver to supply the neighbouring
//!   reconstructed blocks.

use crate::block::COEFFS_PER_BLOCK;
use crate::{Error, Result};
use oxideav_core::bits::BitReader;

/// The INTRA prediction mode signalled by the §I.2 INTRA_MODE field
/// (Table I.1). One mode is transmitted per INTRA macroblock when the
/// Advanced INTRA Coding mode is in use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntraMode {
    /// Index 0 — "DC Only". Only the DC coefficient is predicted (as
    /// an average of the block above and the block to the left). AC
    /// coefficients carry no predictor. The Figure-14 zigzag scan is
    /// used for every block in the macroblock.
    DcOnly,
    /// Index 1 — "Vertical DC & AC". The vertically adjacent block
    /// (block above) predicts the DC coefficient and the first row of
    /// AC coefficients. The Alternate-Horizontal scan is used.
    VerticalDcAc,
    /// Index 2 — "Horizontal DC & AC". The horizontally adjacent
    /// block (block to the left) predicts the DC coefficient and the
    /// first column of AC coefficients. The Alternate-Vertical scan
    /// is used.
    HorizontalDcAc,
}

impl IntraMode {
    /// The Table I.1 index (`0`, `1`, or `2`) for this mode.
    #[must_use]
    pub fn index(self) -> u8 {
        match self {
            IntraMode::DcOnly => 0,
            IntraMode::VerticalDcAc => 1,
            IntraMode::HorizontalDcAc => 2,
        }
    }
}

/// Decode the §I.2 INTRA_MODE field per Table I.1.
///
/// The VLC is:
///
/// | Index | Mode               | VLC |
/// |-------|--------------------|-----|
/// | 0     | DC Only            | `0` |
/// | 1     | Vertical DC & AC   | `10`|
/// | 2     | Horizontal DC & AC | `11`|
///
/// On success the reader is advanced past the 1 or 2 consumed bits.
/// This field cannot fail to decode for any bit pattern (every
/// 1-or-2-bit prefix maps to a mode), but EOF mid-field surfaces
/// [`Error::UnexpectedEof`].
pub fn decode_intra_mode(reader: &mut BitReader<'_>) -> Result<IntraMode> {
    let first = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if !first {
        // `0` — DC Only.
        return Ok(IntraMode::DcOnly);
    }
    // Leading `1`: read one more bit to disambiguate `10` vs `11`.
    let second = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if second {
        // `11` — Horizontal DC & AC.
        Ok(IntraMode::HorizontalDcAc)
    } else {
        // `10` — Vertical DC & AC.
        Ok(IntraMode::VerticalDcAc)
    }
}

/// Alternate-Horizontal scan → block-position lookup (§I.3,
/// Figure I.2-a).
///
/// `ALT_HORIZONTAL_TO_BLOCK_POS[i]` gives the 0..=63 block position
/// (`row * 8 + col`) of the `i`-th coefficient in this scan order,
/// using the same convention as
/// [`crate::block::ZIGZAG_TO_BLOCK_POS`]. This scan is selected for
/// prediction mode 1 ([`IntraMode::VerticalDcAc`]); it visits the
/// stronger horizontal frequencies before the vertical ones.
///
/// Figure I.2-a prints the scan with 1-based indices placed at each
/// block position; this array is the inverse mapping (for each scan
/// index `n`, the block position holding that index), shifted to
/// 0-based as in Figure 14.
pub const ALT_HORIZONTAL_TO_BLOCK_POS: [u8; COEFFS_PER_BLOCK] = [
    0, 1, 2, 3, 8, 9, 16, 17, // scan 1..8
    10, 11, 4, 5, 6, 7, 15, 14, // scan 9..16
    13, 12, 19, 18, 24, 25, 32, 33, // scan 17..24
    26, 27, 20, 21, 22, 23, 28, 29, // scan 25..32
    30, 31, 34, 35, 40, 41, 48, 49, // scan 33..40
    42, 43, 36, 37, 38, 39, 44, 45, // scan 41..48
    46, 47, 50, 51, 56, 57, 58, 59, // scan 49..56
    52, 53, 54, 55, 60, 61, 62, 63, // scan 57..64
];

/// Alternate-Vertical scan → block-position lookup (§I.3,
/// Figure I.2-b; identical to the ITU-T H.262 alternate scan).
///
/// `ALT_VERTICAL_TO_BLOCK_POS[i]` gives the 0..=63 block position of
/// the `i`-th coefficient in this scan order, using the same
/// convention as [`crate::block::ZIGZAG_TO_BLOCK_POS`]. This scan is
/// selected for prediction mode 2 ([`IntraMode::HorizontalDcAc`]); it
/// visits the stronger vertical frequencies before the horizontal
/// ones.
pub const ALT_VERTICAL_TO_BLOCK_POS: [u8; COEFFS_PER_BLOCK] = [
    0, 8, 16, 24, 1, 9, 2, 10, // scan 1..8
    17, 25, 32, 40, 48, 56, 57, 49, // scan 9..16
    41, 33, 26, 18, 3, 11, 4, 12, // scan 17..24
    19, 27, 34, 42, 50, 58, 35, 43, // scan 25..32
    51, 59, 20, 28, 5, 13, 6, 14, // scan 33..40
    21, 29, 36, 44, 52, 60, 37, 45, // scan 41..48
    53, 61, 22, 30, 7, 15, 23, 31, // scan 49..56
    38, 46, 54, 62, 39, 47, 55, 63, // scan 57..64
];

/// §I.3 scan-selection rule: return the scan-position → block-position
/// permutation to use for an INTRA block under the given prediction
/// mode.
///
/// * [`IntraMode::DcOnly`] → the Figure-14 zigzag scan
///   ([`crate::block::ZIGZAG_TO_BLOCK_POS`]).
/// * [`IntraMode::VerticalDcAc`] → the Alternate-Horizontal scan
///   ([`ALT_HORIZONTAL_TO_BLOCK_POS`]).
/// * [`IntraMode::HorizontalDcAc`] → the Alternate-Vertical scan
///   ([`ALT_VERTICAL_TO_BLOCK_POS`]).
///
/// (Per §I.3, non-INTRA blocks are always zigzag-scanned; this
/// function only governs INTRA-block scan selection.)
#[must_use]
pub fn scan_for_intra_mode(mode: IntraMode) -> &'static [u8; COEFFS_PER_BLOCK] {
    match mode {
        IntraMode::DcOnly => &crate::block::ZIGZAG_TO_BLOCK_POS,
        IntraMode::VerticalDcAc => &ALT_HORIZONTAL_TO_BLOCK_POS,
        IntraMode::HorizontalDcAc => &ALT_VERTICAL_TO_BLOCK_POS,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::ZIGZAG_TO_BLOCK_POS;
    use oxideav_core::bits::BitWriter;

    /// Helper: encode a sequence of bits MSB-first and return the
    /// reader-ready bytes plus a `BitReader`-friendly buffer.
    fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
        let mut w = BitWriter::new();
        for &b in bits {
            w.write_bit(b);
        }
        // Pad to a byte boundary with zeros — the reader only consumes
        // the bits we ask for.
        w.into_bytes()
    }

    #[test]
    fn intra_mode_dc_only_decodes_from_single_zero_bit() {
        let buf = bits_to_bytes(&[false]);
        let mut r = BitReader::new(&buf);
        assert_eq!(decode_intra_mode(&mut r).unwrap(), IntraMode::DcOnly);
    }

    #[test]
    fn intra_mode_vertical_decodes_from_one_zero() {
        let buf = bits_to_bytes(&[true, false]); // 10
        let mut r = BitReader::new(&buf);
        assert_eq!(decode_intra_mode(&mut r).unwrap(), IntraMode::VerticalDcAc);
    }

    #[test]
    fn intra_mode_horizontal_decodes_from_one_one() {
        let buf = bits_to_bytes(&[true, true]); // 11
        let mut r = BitReader::new(&buf);
        assert_eq!(
            decode_intra_mode(&mut r).unwrap(),
            IntraMode::HorizontalDcAc
        );
    }

    /// The `0` codeword consumes exactly one bit; a following bit must
    /// remain for the next field.
    #[test]
    fn intra_mode_dc_only_consumes_exactly_one_bit() {
        // 0 then 1: decode DcOnly, then the remaining bit reads back 1.
        let buf = bits_to_bytes(&[false, true]);
        let mut r = BitReader::new(&buf);
        assert_eq!(decode_intra_mode(&mut r).unwrap(), IntraMode::DcOnly);
        assert!(r.read_bit().unwrap());
    }

    /// The `10`/`11` codewords consume exactly two bits.
    #[test]
    fn intra_mode_two_bit_codes_consume_exactly_two_bits() {
        // 10 then 1.
        let buf = bits_to_bytes(&[true, false, true]);
        let mut r = BitReader::new(&buf);
        assert_eq!(decode_intra_mode(&mut r).unwrap(), IntraMode::VerticalDcAc);
        assert!(r.read_bit().unwrap());

        // 11 then 0.
        let buf = bits_to_bytes(&[true, true, false]);
        let mut r = BitReader::new(&buf);
        assert_eq!(
            decode_intra_mode(&mut r).unwrap(),
            IntraMode::HorizontalDcAc
        );
        assert!(!r.read_bit().unwrap());
    }

    /// EOF mid-field: a buffer holding exactly a leading `1` and
    /// nothing else surfaces UnexpectedEof when the second bit is
    /// demanded, rather than guessing a mode. We position a reader so
    /// only one bit remains, then feed the `1` and let the second
    /// `read_bit` hit the end.
    #[test]
    fn intra_mode_eof_after_leading_one_errors() {
        // One byte = 8 bits; consume 7 leaving 1 bit. Make that last
        // bit a `1` so `decode_intra_mode` reads it as a leading one
        // and then EOFs reaching for the disambiguating second bit.
        let buf = [0b0000_0001u8];
        let mut r = BitReader::new(&buf);
        for _ in 0..7 {
            r.read_bit().unwrap();
        }
        assert_eq!(decode_intra_mode(&mut r), Err(Error::UnexpectedEof));
    }

    /// EOF on the very first bit surfaces UnexpectedEof.
    #[test]
    fn intra_mode_eof_on_empty_buffer() {
        let empty: [u8; 0] = [];
        let mut r = BitReader::new(&empty);
        assert_eq!(decode_intra_mode(&mut r), Err(Error::UnexpectedEof));
    }

    #[test]
    fn intra_mode_index_round_trips() {
        assert_eq!(IntraMode::DcOnly.index(), 0);
        assert_eq!(IntraMode::VerticalDcAc.index(), 1);
        assert_eq!(IntraMode::HorizontalDcAc.index(), 2);
    }

    /// Both alternate scans are permutations of 0..=63 (each block
    /// position appears exactly once), like the Figure-14 zigzag.
    #[test]
    fn alternate_scans_are_permutations() {
        for table in [&ALT_HORIZONTAL_TO_BLOCK_POS, &ALT_VERTICAL_TO_BLOCK_POS] {
            let mut seen = [false; COEFFS_PER_BLOCK];
            for &p in table.iter() {
                let p = p as usize;
                assert!(p < COEFFS_PER_BLOCK, "block pos {} out of range", p);
                assert!(!seen[p], "block pos {} appears twice", p);
                seen[p] = true;
            }
            assert!(seen.iter().all(|&s| s), "not every block position visited");
        }
    }

    /// Scan index 1 (the DC coefficient) lands at block position 0 in
    /// every scan — zigzag and both alternates.
    #[test]
    fn dc_is_first_in_every_scan() {
        assert_eq!(ZIGZAG_TO_BLOCK_POS[0], 0);
        assert_eq!(ALT_HORIZONTAL_TO_BLOCK_POS[0], 0);
        assert_eq!(ALT_VERTICAL_TO_BLOCK_POS[0], 0);
    }

    /// The two alternate scans are distinct from each other and from
    /// the zigzag scan (they only coincide at the DC slot).
    #[test]
    fn scans_differ_off_dc() {
        let h = &ALT_HORIZONTAL_TO_BLOCK_POS;
        let v = &ALT_VERTICAL_TO_BLOCK_POS;
        let z = &ZIGZAG_TO_BLOCK_POS;
        // At least one off-DC position differs between each pair.
        assert!((1..COEFFS_PER_BLOCK).any(|i| h[i] != v[i]));
        assert!((1..COEFFS_PER_BLOCK).any(|i| h[i] != z[i]));
        assert!((1..COEFFS_PER_BLOCK).any(|i| v[i] != z[i]));
    }

    /// The Alternate-Vertical scan is the transpose of the
    /// Alternate-Horizontal scan: §I.3 designs mode 2 (vertical
    /// frequencies first) as the row/column-swapped counterpart of
    /// mode 1 (horizontal frequencies first). For each scan index,
    /// transposing the block position of one scan
    /// (`row*8+col → col*8+row`) yields the other.
    #[test]
    fn alternate_vertical_is_transpose_of_horizontal() {
        for i in 0..COEFFS_PER_BLOCK {
            let h = ALT_HORIZONTAL_TO_BLOCK_POS[i] as usize;
            let (hr, hc) = (h / 8, h % 8);
            let transposed = (hc * 8 + hr) as u8;
            assert_eq!(
                ALT_VERTICAL_TO_BLOCK_POS[i],
                transposed,
                "scan index {} (1-based {}): H pos {} transposed {} != V pos {}",
                i,
                i + 1,
                h,
                transposed,
                ALT_VERTICAL_TO_BLOCK_POS[i]
            );
        }
    }

    /// §I.3 scan-selection rule: mode 0 → zigzag, mode 1 → alt-horiz,
    /// mode 2 → alt-vert (compared by table contents).
    #[test]
    fn scan_selection_matches_spec() {
        assert_eq!(scan_for_intra_mode(IntraMode::DcOnly), &ZIGZAG_TO_BLOCK_POS);
        assert_eq!(
            scan_for_intra_mode(IntraMode::VerticalDcAc),
            &ALT_HORIZONTAL_TO_BLOCK_POS
        );
        assert_eq!(
            scan_for_intra_mode(IntraMode::HorizontalDcAc),
            &ALT_VERTICAL_TO_BLOCK_POS
        );
    }

    /// Spot-check a handful of Figure I.2-a (Alternate-Horizontal)
    /// entries straight from the spec grid. Figure prints 1-based
    /// scan indices at block positions; we verify the inverse.
    /// Grid row 0: `1 2 3 4 11 12 13 14`. So scan index 1 → block
    /// (0,0)=0, scan 2 → (0,1)=1, scan 3 → (0,2)=2, scan 4 → (0,3)=3,
    /// and scan 11 → (0,4)=4.
    #[test]
    fn alt_horizontal_spot_checks() {
        assert_eq!(ALT_HORIZONTAL_TO_BLOCK_POS[0], 0); // scan 1 -> (0,0)
        assert_eq!(ALT_HORIZONTAL_TO_BLOCK_POS[1], 1); // scan 2 -> (0,1)
        assert_eq!(ALT_HORIZONTAL_TO_BLOCK_POS[2], 2); // scan 3 -> (0,2)
        assert_eq!(ALT_HORIZONTAL_TO_BLOCK_POS[3], 3); // scan 4 -> (0,3)
        assert_eq!(ALT_HORIZONTAL_TO_BLOCK_POS[10], 4); // scan 11 -> (0,4)
                                                        // Grid last row: `53 54 55 56 61 62 63 64`. Scan 64 -> (7,7)=63.
        assert_eq!(ALT_HORIZONTAL_TO_BLOCK_POS[63], 63);
    }

    /// Spot-check Figure I.2-b (Alternate-Vertical). Grid col 0 (top
    /// to bottom): `1 2 3 4 11 12 13 14`. So scan 1 → (0,0)=0, scan 2
    /// → (1,0)=8, scan 3 → (2,0)=16, scan 4 → (3,0)=24, scan 11 →
    /// (4,0)=32. Scan 64 → (7,7)=63.
    #[test]
    fn alt_vertical_spot_checks() {
        assert_eq!(ALT_VERTICAL_TO_BLOCK_POS[0], 0); // scan 1 -> (0,0)
        assert_eq!(ALT_VERTICAL_TO_BLOCK_POS[1], 8); // scan 2 -> (1,0)
        assert_eq!(ALT_VERTICAL_TO_BLOCK_POS[2], 16); // scan 3 -> (2,0)
        assert_eq!(ALT_VERTICAL_TO_BLOCK_POS[3], 24); // scan 4 -> (3,0)
        assert_eq!(ALT_VERTICAL_TO_BLOCK_POS[10], 32); // scan 11 -> (4,0)
        assert_eq!(ALT_VERTICAL_TO_BLOCK_POS[63], 63); // scan 64 -> (7,7)
    }
}
