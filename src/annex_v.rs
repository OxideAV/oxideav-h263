//! Annex V — Data-Partitioned Slice (DPS) mode, partition-layer
//! primitives.
//!
//! §V.2 rearranges each Annex K video picture segment so that the
//! macroblock headers of the whole segment come first (the **HD**
//! partition, coded with the reversible COD + MCBPC codes of Tables
//! V.1 / V.2), then every motion vector (the **MV** partition — Table
//! D.3 codewords over a single §V.2.3.2 prediction thread, closed by
//! the redundant §V.2.4 LMVV), then all the coefficient data
//! (§V.2.6). The §V.2.2 Header Marker and §V.2.5 Motion Vector
//! Marker separate the partitions; neither value can occur naturally
//! in its partition, giving a decoder resynchronisation points.
//!
//! This module stages the pure partition-layer pieces — the Table
//! V.1 / V.2 RVLC inventories both directions, the marker constants,
//! and the §V.2.3 motion-vector thread coder with its §V.2.3.3
//! per-codeword start-code-emulation rule (which replaces the §D.2
//! pair rule). The picture-level DPS drivers live in
//! [`crate::picture`] (decode) and [`crate::encoder`] (encode).

use crate::{Error, Result};
use oxideav_core::bits::{BitReader, BitWriter};

/// §V.2.2 — the 9-bit Header Marker `1010 0010 1` that terminates the
/// HD partition.
#[doc(hidden)]
pub const HEADER_MARKER: u32 = 0b1_0100_0101;
/// Bit length of [`HEADER_MARKER`].
#[doc(hidden)]
pub const HEADER_MARKER_BITS: u32 = 9;
/// §V.2.5 — the 10-bit Motion Vector Marker `0000 0000 01` that
/// terminates the MV partition (absent when the segment carries no
/// motion vector data).
#[doc(hidden)]
pub const MOTION_VECTOR_MARKER: u32 = 0b00_0000_0001;
/// Bit length of [`MOTION_VECTOR_MARKER`].
#[doc(hidden)]
pub const MOTION_VECTOR_MARKER_BITS: u32 = 10;

/// One decoded Table V.1 / V.2 HD entry: the macroblock's class plus
/// its CBPC bits (chrominance coded-block pattern, block-5 bit in
/// `0b10`, block-6 bit in `0b01`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DpsMbHeader {
    /// COD = 1 — skipped macroblock (INTER pictures only).
    Skipped,
    /// MB type 0 / 1 — single-vector INTER macroblock; `quant` marks
    /// the "+ Q" form (a DQUANT rides in the coefficient partition).
    Inter {
        /// CBPC bits (blocks 5/6).
        cbpc: u8,
        /// INTER + Q (type 1).
        quant: bool,
    },
    /// MB type 2 / 5 — INTER4V macroblock (four vectors); `quant`
    /// marks the "+ Q" form.
    Inter4v {
        /// CBPC bits (blocks 5/6).
        cbpc: u8,
        /// INTER4V + Q (type 5).
        quant: bool,
    },
    /// MB type 3 / 4 — INTRA macroblock; `quant` marks the "+ Q" form.
    Intra {
        /// CBPC bits (blocks 5/6).
        cbpc: u8,
        /// INTRA + Q (type 4).
        quant: bool,
    },
    /// The stuffing codeword — carries no macroblock.
    Stuffing,
}

impl DpsMbHeader {
    /// Number of motion vectors this macroblock contributes to the
    /// §V.2.3.2 prediction thread.
    pub fn motion_vector_count(self) -> usize {
        match self {
            DpsMbHeader::Inter { .. } => 1,
            DpsMbHeader::Inter4v { .. } => 4,
            _ => 0,
        }
    }

    /// The CBPC bits, `0` for the vector-less classes.
    pub fn cbpc(self) -> u8 {
        match self {
            DpsMbHeader::Inter { cbpc, .. }
            | DpsMbHeader::Inter4v { cbpc, .. }
            | DpsMbHeader::Intra { cbpc, .. } => cbpc,
            _ => 0,
        }
    }

    /// Whether a §5.3.6 DQUANT rides in the coefficient partition.
    pub fn has_dquant(self) -> bool {
        matches!(
            self,
            DpsMbHeader::Inter { quant: true, .. }
                | DpsMbHeader::Inter4v { quant: true, .. }
                | DpsMbHeader::Intra { quant: true, .. }
        )
    }
}

/// One row of a Table V.1 / V.2 inventory: `(code, bits, entry)` with
/// the codeword MSB-aligned within `bits`.
type RvlcRow = (u32, u32, DpsMbHeader);

/// Table V.1/H.263 — COD + MCBPC RVLC inventory for INTRA-picture
/// macroblocks (MB types 3 / 4 plus stuffing).
pub const TABLE_V1_INTRA: &[RvlcRow] = &[
    (
        0b1,
        1,
        DpsMbHeader::Intra {
            cbpc: 0b00,
            quant: false,
        },
    ),
    (
        0b010,
        3,
        DpsMbHeader::Intra {
            cbpc: 0b01,
            quant: false,
        },
    ),
    (
        0b0110,
        4,
        DpsMbHeader::Intra {
            cbpc: 0b10,
            quant: false,
        },
    ),
    (
        0b01110,
        5,
        DpsMbHeader::Intra {
            cbpc: 0b11,
            quant: false,
        },
    ),
    (
        0b00100,
        5,
        DpsMbHeader::Intra {
            cbpc: 0b00,
            quant: true,
        },
    ),
    (
        0b011110,
        6,
        DpsMbHeader::Intra {
            cbpc: 0b01,
            quant: true,
        },
    ),
    (
        0b001100,
        6,
        DpsMbHeader::Intra {
            cbpc: 0b10,
            quant: true,
        },
    ),
    (
        0b0111110,
        7,
        DpsMbHeader::Intra {
            cbpc: 0b11,
            quant: true,
        },
    ),
    (0b0011100, 7, DpsMbHeader::Stuffing),
];

/// Table V.2/H.263 — COD + MCBPC RVLC inventory for INTER-picture
/// macroblocks (skipped + MB types 0..=5 plus stuffing). Note the
/// printed CBPC column order (`00, 10, 01, 11` for most types).
pub const TABLE_V2_INTER: &[RvlcRow] = &[
    (0b1, 1, DpsMbHeader::Skipped),
    (
        0b010,
        3,
        DpsMbHeader::Inter {
            cbpc: 0b00,
            quant: false,
        },
    ),
    (
        0b00100,
        5,
        DpsMbHeader::Inter {
            cbpc: 0b10,
            quant: false,
        },
    ),
    (
        0b011110,
        6,
        DpsMbHeader::Inter {
            cbpc: 0b01,
            quant: false,
        },
    ),
    (
        0b0011100,
        7,
        DpsMbHeader::Inter {
            cbpc: 0b11,
            quant: false,
        },
    ),
    (
        0b01110,
        5,
        DpsMbHeader::Inter {
            cbpc: 0b00,
            quant: true,
        },
    ),
    (
        0b00011000,
        8,
        DpsMbHeader::Inter {
            cbpc: 0b10,
            quant: true,
        },
    ),
    (
        0b011111110,
        9,
        DpsMbHeader::Inter {
            cbpc: 0b01,
            quant: true,
        },
    ),
    (
        0b01111111110,
        11,
        DpsMbHeader::Inter {
            cbpc: 0b11,
            quant: true,
        },
    ),
    (
        0b0110,
        4,
        DpsMbHeader::Inter4v {
            cbpc: 0b00,
            quant: false,
        },
    ),
    (
        0b01111110,
        8,
        DpsMbHeader::Inter4v {
            cbpc: 0b10,
            quant: false,
        },
    ),
    (
        0b00111100,
        8,
        DpsMbHeader::Inter4v {
            cbpc: 0b01,
            quant: false,
        },
    ),
    (
        0b000010000,
        9,
        DpsMbHeader::Inter4v {
            cbpc: 0b11,
            quant: false,
        },
    ),
    (
        0b001100,
        6,
        DpsMbHeader::Intra {
            cbpc: 0b00,
            quant: false,
        },
    ),
    (
        0b0001000,
        7,
        DpsMbHeader::Intra {
            cbpc: 0b11,
            quant: false,
        },
    ),
    (
        0b001111100,
        9,
        DpsMbHeader::Intra {
            cbpc: 0b10,
            quant: false,
        },
    ),
    (
        0b000111000,
        9,
        DpsMbHeader::Intra {
            cbpc: 0b01,
            quant: false,
        },
    ),
    (
        0b0111110,
        7,
        DpsMbHeader::Intra {
            cbpc: 0b00,
            quant: true,
        },
    ),
    (
        0b0011111100,
        10,
        DpsMbHeader::Intra {
            cbpc: 0b11,
            quant: true,
        },
    ),
    (
        0b0001111000,
        10,
        DpsMbHeader::Intra {
            cbpc: 0b10,
            quant: true,
        },
    ),
    (
        0b0000110000,
        10,
        DpsMbHeader::Intra {
            cbpc: 0b01,
            quant: true,
        },
    ),
    (
        0b00111111100,
        11,
        DpsMbHeader::Inter4v {
            cbpc: 0b00,
            quant: true,
        },
    ),
    (
        0b00011111000,
        11,
        DpsMbHeader::Inter4v {
            cbpc: 0b01,
            quant: true,
        },
    ),
    (
        0b00001110000,
        11,
        DpsMbHeader::Inter4v {
            cbpc: 0b10,
            quant: true,
        },
    ),
    (
        0b00000100000,
        11,
        DpsMbHeader::Inter4v {
            cbpc: 0b11,
            quant: true,
        },
    ),
    (0b0111111110, 10, DpsMbHeader::Stuffing),
];

/// Longest codeword length across both inventories.
const MAX_RVLC_BITS: u32 = 11;

/// Read one Table V.1 / V.2 codeword from `reader`.
///
/// # Errors
///
/// [`Error::BadDpsHeaderCode`] when no inventory row matches within
/// [`MAX_RVLC_BITS`]; [`Error::UnexpectedEof`] on starvation.
pub fn read_dps_mb_header(reader: &mut BitReader<'_>, table: &[RvlcRow]) -> Result<DpsMbHeader> {
    let mut code: u32 = 0;
    for len in 1..=MAX_RVLC_BITS {
        code = (code << 1) | reader.read_u32(1).map_err(|_| Error::UnexpectedEof)?;
        for &(c, l, entry) in table {
            if l == len && c == code {
                return Ok(entry);
            }
        }
    }
    Err(Error::BadDpsHeaderCode)
}

/// Write one Table V.1 / V.2 entry.
///
/// # Errors
///
/// [`Error::BadDpsHeaderCode`] when `entry` has no row in `table`
/// (e.g. [`DpsMbHeader::Skipped`] against Table V.1).
pub fn write_dps_mb_header(w: &mut BitWriter, table: &[RvlcRow], entry: DpsMbHeader) -> Result<()> {
    for &(c, l, e) in table {
        if e == entry {
            w.write_bits(c, l);
            return Ok(());
        }
    }
    Err(Error::BadDpsHeaderCode)
}

/// §V.2.3 — the motion-vector partition coder: Table D.3 codewords
/// over one §V.2.3.2 prediction thread, with the §V.2.3.3 per-codeword
/// start-code-emulation rule.
///
/// The rule differs from §D.2: the partition is scanned codeword by
/// codeword (components, not pairs), and an MVD = 0 codeword (`"1"`)
/// is inserted after **any two consecutive** MVD = +1 codewords
/// (`"000"` each); a third consecutive `"000"` starts a new count.
/// The counter spans the whole run of Table D.3 codewords — the MVD
/// data and the §V.2.4 LMVV are contiguous, and the §V.2.5 marker's
/// zero run is exactly what the rule protects.
#[derive(Debug, Clone, Copy, Default)]
#[doc(hidden)]
pub struct MvdEmulationState {
    consecutive_ones: u8,
}

impl MvdEmulationState {
    /// Fresh state for a new MV partition.
    pub fn new() -> Self {
        Self::default()
    }

    /// Read one Table D.3 codeword (half-pel value), consuming any
    /// §V.2.3.3 inserted MVD = 0 codeword.
    pub fn read_component(&mut self, reader: &mut BitReader<'_>) -> Result<i32> {
        let value = crate::annex_p::read_table_d3(reader)?;
        if value == 1 {
            self.consecutive_ones += 1;
            if self.consecutive_ones == 2 {
                // The inserted MVD = 0 codeword ("1") follows; it is
                // not data.
                let epb = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
                if !epb {
                    return Err(Error::BadMvdCode);
                }
                self.consecutive_ones = 0;
            }
        } else {
            self.consecutive_ones = 0;
        }
        Ok(value)
    }

    /// Write one Table D.3 codeword (half-pel value), inserting the
    /// §V.2.3.3 MVD = 0 codeword after every second consecutive
    /// MVD = +1.
    pub fn write_component(&mut self, w: &mut BitWriter, value: i32) -> Result<()> {
        crate::annex_p::write_table_d3(w, value)?;
        if value == 1 {
            self.consecutive_ones += 1;
            if self.consecutive_ones == 2 {
                w.write_bit(true);
                self.consecutive_ones = 0;
            }
        } else {
            self.consecutive_ones = 0;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hd_tables_are_prefix_free_and_round_trip() {
        for table in [TABLE_V1_INTRA, TABLE_V2_INTER] {
            for &(c, l, entry) in table {
                // No codeword is a prefix of another.
                for &(c2, l2, _) in table {
                    if (c, l) == (c2, l2) {
                        continue;
                    }
                    if l2 > l {
                        assert_ne!(
                            c2 >> (l2 - l),
                            c,
                            "{c:0width$b} prefixes {c2:b}",
                            width = l as usize
                        );
                    }
                }
                // Round trip through the bit layer.
                let mut w = BitWriter::new();
                write_dps_mb_header(&mut w, table, entry).expect("write");
                w.write_bits(0b1010, 4); // sentinel
                let bytes = w.finish();
                let mut r = BitReader::new(&bytes);
                assert_eq!(read_dps_mb_header(&mut r, table).expect("read"), entry);
                assert_eq!(r.bit_position(), l as u64);
            }
        }
    }

    #[test]
    fn hd_tables_are_reversible() {
        // RVLC property: the reversed bit pattern of every codeword is
        // also a valid codeword of the same table (what backward
        // decoding relies on). Symmetric codewords map to themselves.
        for table in [TABLE_V1_INTRA, TABLE_V2_INTER] {
            for &(c, l, _) in table {
                let mut rev = 0u32;
                for bit in 0..l {
                    rev = (rev << 1) | ((c >> bit) & 1);
                }
                assert!(
                    table.iter().any(|&(c2, l2, _)| (c2, l2) == (rev, l)),
                    "reverse of {c:0width$b} missing from table",
                    width = l as usize
                );
            }
        }
    }

    #[test]
    fn header_marker_cannot_open_a_codeword_sequence() {
        // The 9-bit HM read at a codeword boundary must not decode as
        // the start of any codeword string: its prefix "1" would be a
        // 1-bit codeword, then "010" (V.1: INTRA cbpc 01; V.2: INTER),
        // then "0010 1" — and no codeword in either table starts
        // "00101". So a decoder peeking HM before each macroblock can
        // never confuse data for the marker.
        for table in [TABLE_V1_INTRA, TABLE_V2_INTER] {
            let tail = 0b00101u32; // remaining 5 bits of HM after "1" + "010"
            for &(c, l, _) in table {
                if l >= 5 {
                    assert_ne!(c >> (l - 5), tail, "codeword starts with HM tail");
                }
            }
        }
    }

    #[test]
    fn mv_thread_emulation_rule_round_trips() {
        // A run of +1 codewords ("000" each) exercises the §V.2.3.3
        // insertion at every second one, including across what §D.2
        // would have treated as pair boundaries.
        let values = [1i32, 1, 1, 1, 1, 0, 1, 1, -3, 1, 1, 2047, -2047, 0];
        let mut w = BitWriter::new();
        let mut enc = MvdEmulationState::new();
        for &v in &values {
            enc.write_component(&mut w, v).expect("write");
        }
        w.write_bits(MOTION_VECTOR_MARKER, MOTION_VECTOR_MARKER_BITS);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let mut dec = MvdEmulationState::new();
        for &v in &values {
            assert_eq!(dec.read_component(&mut r).expect("read"), v);
        }
        assert_eq!(
            r.read_u32(MOTION_VECTOR_MARKER_BITS).expect("marker"),
            MOTION_VECTOR_MARKER
        );
    }

    #[test]
    fn five_consecutive_plus_ones_insert_twice() {
        // "000 000 1 000 000 1 000": insertions after the 2nd and 4th,
        // the 5th starts a new count (§V.2.3.3's "shall be considered
        // the first").
        let mut w = BitWriter::new();
        let mut enc = MvdEmulationState::new();
        for _ in 0..5 {
            enc.write_component(&mut w, 1).expect("write");
        }
        assert_eq!(w.bit_position(), 3 * 5 + 2);
    }
}
