//! H.263 Annex K — Slice Structured mode.
//!
//! The Slice Structured mode (signalled by OPPTYPE bit 10 — `SS` — inside a
//! PLUSPTYPE picture header) replaces the GOB layer with a *slice* layer.
//! Each slice carries its own slice header which acts as a resynchronisation
//! point under bit error / packet loss: a corrupted slice can be discarded
//! and decoding can resume cleanly at the next slice header.
//!
//! Slice layer syntax (§K.2, Figure K.1):
//!
//! ```text
//! SSTUF | SSC(17) | SEPB1(1) | SSBI(4 if CPM) | MBA(N) | SEPB2(1 cond.)
//!       | SQUANT(5) | SWI(N if RS) | SEPB3(1) | GFID(2) | macroblock data
//! ```
//!
//! For the **first** slice of a picture (the slice immediately following PSC)
//! only `MBA` (and `SWI` if Rectangular Slice submode is in use) is present;
//! `SSTUF`, `SSC`, `SSBI`, `SQUANT`, `GFID`, `SEPB1`/`SEPB3`, and `SEPB2` (in
//! most cases) are omitted.
//!
//! Two submodes are signalled in the SSS field of the picture header:
//!   * **Rectangular Slice (RS)** — slices occupy a rectangular region of
//!     width given by `SWI`. When RS is off, slices are arbitrary contiguous
//!     ranges of macroblocks in raster scan order within the whole picture.
//!   * **Arbitrary Slice Ordering (ASO)** — slices may appear in any order
//!     within the bitstream. When ASO is off, slices appear in increasing
//!     `MBA` order.
//!
//! Per-slice MV-prediction reset (§K.1 rule 1) — "the prediction of motion
//! vector values are the same as if a GOB header were present". The decoder
//! treats every slice boundary the same way the existing GOB-boundary path
//! does (`gob_top_row` flag in the §F.2 / §6.1.1 predictor cascade), and the
//! encoder mirrors that by resetting the MV grid at slice boundaries.
//!
//! This module owns the slice-header parser, emitter, and the small bit-width
//! tables (Tables K.2 / K.3). The actual MB-loop integration lives in
//! [`crate::decoder`] and [`crate::encoder`].

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_core::{Error, Result};

use crate::picture::SourceFormat;

/// 17-bit Slice Start Code value, identical bit pattern to GBSC.
/// Disambiguation between GBSC and SSC is by picture-header context:
/// when `PictureHeader::slice_structured` is true, all 17-bit start codes
/// (other than PSC) are SSCs; otherwise they are GBSCs.
pub const SSC_VALUE: u32 = 0b00_0000_0000_0000_0001;
/// Width in bits of SSC.
pub const SSC_BITS: u32 = 17;

/// Field-width helpers — Table K.2 (MBA) and Table K.3 (SWI). `rru`
/// selects Reduced-Resolution Update mode (Annex Q); we don't implement
/// Annex Q yet so callers always pass `rru = false`.
pub fn mba_field_width(format: SourceFormat, rru: bool) -> Result<u32> {
    let w = match (format, rru) {
        (SourceFormat::SubQcif, false) => 6,
        (SourceFormat::SubQcif, true) => 5,
        (SourceFormat::Qcif, false) => 7,
        (SourceFormat::Qcif, true) => 6,
        (SourceFormat::Cif, false) => 9,
        (SourceFormat::Cif, true) => 7,
        (SourceFormat::FourCif, false) => 11,
        (SourceFormat::FourCif, true) => 9,
        (SourceFormat::SixteenCif, false) => 13,
        (SourceFormat::SixteenCif, true) => 11,
        _ => {
            return Err(Error::unsupported(format!(
                "h263 Annex K: MBA field width for {format:?} not specified"
            )));
        }
    };
    Ok(w)
}

/// Field width in bits for the SWI parameter (Table K.3).
pub fn swi_field_width(format: SourceFormat, rru: bool) -> Result<u32> {
    let w = match (format, rru) {
        (SourceFormat::SubQcif, false) => 4,
        (SourceFormat::SubQcif, true) => 3,
        (SourceFormat::Qcif, false) => 4,
        (SourceFormat::Qcif, true) => 3,
        (SourceFormat::Cif, false) => 5,
        (SourceFormat::Cif, true) => 4,
        (SourceFormat::FourCif, false) => 6,
        (SourceFormat::FourCif, true) => 5,
        (SourceFormat::SixteenCif, false) => 7,
        (SourceFormat::SixteenCif, true) => 6,
        _ => {
            return Err(Error::unsupported(format!(
                "h263 Annex K: SWI field width for {format:?} not specified"
            )));
        }
    };
    Ok(w)
}

/// SSS field — 2-bit submode signal carried in the PLUSPTYPE picture header
/// when the SS bit is set. Layout per §5.1.4.6:
/// * bit 1: Rectangular Slice (RS) submode flag — `1` = RS on.
/// * bit 2: Arbitrary Slice Ordering (ASO) submode flag — `1` = ASO on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SssMode {
    pub rectangular_slice: bool,
    pub arbitrary_order: bool,
}

impl SssMode {
    pub fn from_bits(bits: u8) -> Self {
        // Per §5.1.4.6 the 2 SSS bits are sent MSB first: bit 1 = RS, bit 2 = ASO.
        Self {
            rectangular_slice: (bits & 0b10) != 0,
            arbitrary_order: (bits & 0b01) != 0,
        }
    }

    pub fn to_bits(self) -> u32 {
        ((self.rectangular_slice as u32) << 1) | (self.arbitrary_order as u32)
    }
}

/// Parsed slice header (§K.2). `swi` is `Some` only when the Rectangular
/// Slice submode is in use.
#[derive(Clone, Copy, Debug)]
pub struct SliceHeader {
    /// Macroblock address of the first MB in this slice (§K.2.5).
    pub mba: u32,
    /// Quantiser to be used for this slice and onwards until DQUANT modifies
    /// it (§K.2.7). Always present except for the first slice (which inherits
    /// PQUANT from the picture header).
    pub squant: u8,
    /// Slice width in macroblocks (§K.2.8) — only present when the
    /// Rectangular Slice submode is active. The decoded value is
    /// `SWI + 1`, stored here in pre-decoded form (`Some(SWI)`).
    pub swi: Option<u32>,
    /// GOB Frame ID (§5.2.5) — same field as the GOB header's GFID.
    pub gfid: u8,
    /// Slice Sub-Bitstream Indicator (§K.2.4) — only present when CPM=1.
    pub ssbi: Option<u8>,
}

/// Parse a slice header that follows an already-consumed 17-bit SSC
/// codeword. The caller is expected to have aligned to the byte after
/// SSTUF and consumed the SSC itself (so this function reads from
/// SEPB1 onwards). `is_first_slice` is true for the slice that
/// immediately follows the picture start code (no SSTUF / SSC are
/// present in the bitstream for that slice — only the trailing fields).
///
/// `format` selects the MBA/SWI bit widths from Tables K.2/K.3. `cpm`
/// gates the optional SSBI field; `sss` selects whether SWI is present.
pub fn parse_slice_header_body(
    br: &mut BitReader<'_>,
    format: SourceFormat,
    sss: SssMode,
    cpm: bool,
    is_first_slice: bool,
) -> Result<SliceHeader> {
    // SEPB1 — always "1".
    let sepb1 = br.read_u1()?;
    if sepb1 != 1 {
        return Err(Error::invalid(format!(
            "h263 Annex K: SEPB1 != 1 (got {sepb1})"
        )));
    }
    // SSBI — only present when CPM=1, and never for the first slice.
    let ssbi = if cpm && !is_first_slice {
        Some(br.read_u32(4)? as u8)
    } else {
        None
    };

    let mba_w = mba_field_width(format, false)?;
    let mba = br.read_u32(mba_w)?;

    // SEPB2 — included only when:
    //   * For non-first slices: MBA width > 11 with CPM=0, or > 9 with CPM=1.
    //   * For the first slice: only when the Rectangular Slice submode is in use.
    let sepb2_needed = if is_first_slice {
        sss.rectangular_slice
    } else if cpm {
        mba_w > 9
    } else {
        mba_w > 11
    };
    if sepb2_needed {
        let sepb2 = br.read_u1()?;
        if sepb2 != 1 {
            return Err(Error::invalid(format!(
                "h263 Annex K: SEPB2 != 1 (got {sepb2})"
            )));
        }
    }

    // SQUANT — not present for the first slice (PQUANT applies).
    let squant = if !is_first_slice {
        let q = br.read_u32(5)? as u8;
        if q == 0 {
            return Err(Error::invalid("h263 Annex K: SQUANT == 0"));
        }
        q
    } else {
        0
    };

    // SWI — only present when Rectangular Slice submode is in use.
    let swi = if sss.rectangular_slice {
        let w = swi_field_width(format, false)?;
        Some(br.read_u32(w)?)
    } else {
        None
    };

    // SEPB3 — always "1".
    let sepb3 = br.read_u1()?;
    if sepb3 != 1 {
        return Err(Error::invalid(format!(
            "h263 Annex K: SEPB3 != 1 (got {sepb3})"
        )));
    }
    // GFID (2 bits).
    let gfid = br.read_u32(2)? as u8;

    Ok(SliceHeader {
        mba,
        squant,
        swi,
        gfid,
        ssbi,
    })
}

/// Emit a non-first slice header. Caller is responsible for calling
/// [`align_for_ssc`] before this (so SSC is byte-aligned per §K.2.2),
/// and for choosing the slice's first MB (`mba`) and its quantiser
/// (`squant`). When the Rectangular Slice submode is active the caller
/// supplies the slice width via `swi` (`Some(SWI)` — emitted as raw
/// SWI, the decoded width being `SWI+1`).
///
/// Writes: SSC | SEPB1 | (SSBI if CPM) | MBA | (SEPB2 if needed) | SQUANT
/// | (SWI if RS) | SEPB3 | GFID.
pub fn write_slice_header(
    bw: &mut BitWriter,
    format: SourceFormat,
    sss: SssMode,
    cpm: bool,
    mba: u32,
    squant: u8,
    swi: Option<u32>,
    gfid: u8,
    ssbi: Option<u8>,
) -> Result<()> {
    debug_assert!(bw.is_byte_aligned(), "SSC must be byte-aligned (§K.2.2)");
    if cpm && ssbi.is_none() {
        return Err(Error::invalid(
            "h263 Annex K: SSBI required when CPM is on (§K.2.4)",
        ));
    }
    if sss.rectangular_slice && swi.is_none() {
        return Err(Error::invalid(
            "h263 Annex K: SWI required when Rectangular Slice submode is on (§K.2.8)",
        ));
    }
    if squant == 0 || squant > 31 {
        return Err(Error::invalid(format!(
            "h263 Annex K: SQUANT {squant} out of range 1..=31"
        )));
    }
    bw.write_bits(SSC_VALUE, SSC_BITS);
    bw.write_bits(1, 1); // SEPB1
    if let Some(s) = ssbi {
        bw.write_bits(s as u32 & 0xF, 4);
    }
    let mba_w = mba_field_width(format, false)?;
    bw.write_bits(mba & ((1u32 << mba_w) - 1), mba_w);
    let sepb2_needed = if cpm { mba_w > 9 } else { mba_w > 11 };
    if sepb2_needed {
        bw.write_bits(1, 1); // SEPB2
    }
    bw.write_bits(squant as u32, 5);
    if let Some(w) = swi {
        let sw_w = swi_field_width(format, false)?;
        bw.write_bits(w & ((1u32 << sw_w) - 1), sw_w);
    }
    bw.write_bits(1, 1); // SEPB3
    bw.write_bits(gfid as u32 & 0x3, 2);
    Ok(())
}

/// Pad with zero stuffing bits up to a byte boundary so the next 17-bit
/// SSC codeword is byte-aligned (§K.2.1 / §K.2.2).
pub fn align_for_ssc(bw: &mut BitWriter) {
    while !bw.is_byte_aligned() {
        bw.write_bits(0, 1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mba_widths_table_k2() {
        assert_eq!(mba_field_width(SourceFormat::SubQcif, false).unwrap(), 6);
        assert_eq!(mba_field_width(SourceFormat::Qcif, false).unwrap(), 7);
        assert_eq!(mba_field_width(SourceFormat::Cif, false).unwrap(), 9);
        assert_eq!(mba_field_width(SourceFormat::FourCif, false).unwrap(), 11);
        assert_eq!(
            mba_field_width(SourceFormat::SixteenCif, false).unwrap(),
            13
        );
    }

    #[test]
    fn swi_widths_table_k3() {
        assert_eq!(swi_field_width(SourceFormat::SubQcif, false).unwrap(), 4);
        assert_eq!(swi_field_width(SourceFormat::Qcif, false).unwrap(), 4);
        assert_eq!(swi_field_width(SourceFormat::Cif, false).unwrap(), 5);
        assert_eq!(swi_field_width(SourceFormat::FourCif, false).unwrap(), 6);
        assert_eq!(swi_field_width(SourceFormat::SixteenCif, false).unwrap(), 7);
    }

    #[test]
    fn sss_mode_bits_round_trip() {
        for bits in 0..4u8 {
            let m = SssMode::from_bits(bits);
            assert_eq!(m.to_bits() as u8, bits);
        }
    }

    /// Round-trip a non-first slice header for QCIF, no RS, no CPM.
    /// MBA width = 7, SEPB2 omitted, no SWI.
    #[test]
    fn slice_header_round_trip_qcif_no_rs() {
        let sss = SssMode {
            rectangular_slice: false,
            arbitrary_order: false,
        };
        let mut bw = BitWriter::with_capacity(64);
        // We need to emit the SSC byte-aligned. Force alignment then write.
        align_for_ssc(&mut bw);
        write_slice_header(
            &mut bw,
            SourceFormat::Qcif,
            sss,
            false,
            42,
            7,
            None,
            1,
            None,
        )
        .unwrap();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        // Skip the SSC (17 bits).
        let ssc = br.read_u32(SSC_BITS).unwrap();
        assert_eq!(ssc, SSC_VALUE);
        let hdr = parse_slice_header_body(&mut br, SourceFormat::Qcif, sss, false, false).unwrap();
        assert_eq!(hdr.mba, 42);
        assert_eq!(hdr.squant, 7);
        assert_eq!(hdr.swi, None);
        assert_eq!(hdr.gfid, 1);
        assert_eq!(hdr.ssbi, None);
    }

    /// Round-trip a non-first slice header for CIF with RS submode active.
    /// MBA width = 9, SWI present (5 bits).
    #[test]
    fn slice_header_round_trip_cif_with_rs() {
        let sss = SssMode {
            rectangular_slice: true,
            arbitrary_order: false,
        };
        let mut bw = BitWriter::with_capacity(64);
        align_for_ssc(&mut bw);
        write_slice_header(
            &mut bw,
            SourceFormat::Cif,
            sss,
            false,
            120,
            9,
            Some(10),
            2,
            None,
        )
        .unwrap();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let _ssc = br.read_u32(SSC_BITS).unwrap();
        let hdr = parse_slice_header_body(&mut br, SourceFormat::Cif, sss, false, false).unwrap();
        assert_eq!(hdr.mba, 120);
        assert_eq!(hdr.squant, 9);
        assert_eq!(hdr.swi, Some(10));
        assert_eq!(hdr.gfid, 2);
    }

    /// First-slice form: only MBA + SEPB3 + GFID + (SWI if RS) — no SSC, no
    /// SEPB1, no SQUANT. This test exercises only the body-parser given a
    /// hand-made buffer.
    #[test]
    fn first_slice_no_squant_no_ssc() {
        // Build the first-slice body manually:
        //   SEPB1=1 | MBA(7)=0 | SEPB3=1 | GFID(2)=0
        // For QCIF with no RS: 1 + 7 + 1 + 2 = 11 bits → pad to 16 for parsing.
        let mut bw = BitWriter::with_capacity(8);
        bw.write_bits(1, 1); // SEPB1
        bw.write_bits(0, 7); // MBA = 0 (first MB of picture)
                             // No SEPB2 (CPM=0, mba_w=7 not > 11)
                             // No SQUANT (first slice)
                             // No SWI (RS off)
        bw.write_bits(1, 1); // SEPB3
        bw.write_bits(0, 2); // GFID = 0
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let sss = SssMode::default();
        let hdr = parse_slice_header_body(&mut br, SourceFormat::Qcif, sss, false, true).unwrap();
        assert_eq!(hdr.mba, 0);
        assert_eq!(hdr.squant, 0); // not present, default 0
        assert_eq!(hdr.swi, None);
        assert_eq!(hdr.gfid, 0);
    }
}
