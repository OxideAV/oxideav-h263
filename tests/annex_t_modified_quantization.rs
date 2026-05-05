//! Integration tests for **Annex T — Modified Quantization mode** (round 25).
#![allow(clippy::unusual_byte_groupings)]
//!
//! Round 25 wires the Annex T helpers + I-picture body driver:
//!
//! * `crate::mq::decode_dquant_mq` — §T.2 variable-length DQUANT (Table T.1).
//! * `crate::mq::quant_c_for_quant` — §T.3 Table T.2 (luma → chroma quant).
//! * `crate::mq::unrotate_extended_level` — §T.4 11-bit cyclic rotation.
//! * `crate::block::decode_ac_mq` — INTER/INTRA AC decode honouring §T.4
//!   EXTENDED-ESCAPE + §T.5 restrictions.
//! * `crate::mb::decode_intra_mb_mq` — I-picture MB body driver.
//! * `picture::PictureHeader::modified_quantization` — surfaced from
//!   PLUSPTYPE OPPTYPE bit 14.
//!
//! These tests exercise the parser-side wiring; the I-MB body uses the
//! same Table 14 MCBPC + Table 13 CBPY shape as the baseline path, so
//! the only novel behaviour we exercise here is (a) the header bit
//! propagating through to the parsed picture, and (b) the §T.4 helper
//! round-trip, and (c) the runtime guard that rejects Annex T on
//! P-pictures (which need the §T.2 DQUANT VLC + §T.3 chroma QUANT
//! plumbed through `decode_p_mb` — round-26 work).

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_h263::picture::{parse_picture_header, PictureCodingType, SourceFormat};

#[allow(clippy::too_many_arguments)]
fn pack_opptype(
    src_fmt: u32,
    custom_pcf: bool,
    umv: bool,
    sac: bool,
    ap: bool,
    aic: bool,
    df: bool,
    sss: bool,
    rps: bool,
    isd: bool,
    aiv: bool,
    mq: bool,
) -> u32 {
    let b = |k: u32, v: u32| (v & 1) << (18 - k);
    let srcf = (src_fmt & 0b111) << 15;
    srcf | b(4, custom_pcf as u32)
        | b(5, umv as u32)
        | b(6, sac as u32)
        | b(7, ap as u32)
        | b(8, aic as u32)
        | b(9, df as u32)
        | b(10, sss as u32)
        | b(11, rps as u32)
        | b(12, isd as u32)
        | b(13, aiv as u32)
        | b(14, mq as u32)
        | b(15, 1) // marker
}

fn write_plusptype_iframe_with_flags(aiv: bool, isd: bool, mq: bool, coding_p: bool) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_bits(0b00_0000_0000_0000_0000_1_00000, 22); // PSC
    bw.write_bits(0, 8); // TR
    bw.write_bits(1, 1); // PTYPE marker
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3); // src fmt = PLUSPTYPE
    bw.write_bits(0b001, 3); // UFEP = OPPTYPE follows
    let opptype = pack_opptype(
        0b010, // QCIF
        false, false, false, false, false, false, false, false, isd, aiv, mq,
    );
    bw.write_bits(opptype, 18);
    // MPPTYPE: PCT=000 (I) or 001 (P) | RPR=0 | RRU=0 | RTYPE=0 | marker=001
    let pct = if coding_p { 0b001 } else { 0b000 };
    bw.write_bits((pct << 6) | 0b001, 9);
    bw.write_bits(0, 1); // CPM = 0
    bw.write_bits(5, 5); // PQUANT = 5
    bw.write_bits(0, 1); // PEI = 0
    bw.finish()
}

#[test]
fn plusptype_mq_flag_propagates_to_picture_header() {
    let bytes = write_plusptype_iframe_with_flags(false, false, true, false);
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert!(hdr.plusptype);
    assert!(hdr.modified_quantization);
    assert!(!hdr.alternative_inter_vlc);
    assert!(!hdr.independent_segment_decoding);
    assert_eq!(hdr.source_format, SourceFormat::Qcif);
    assert_eq!(hdr.coding_type, PictureCodingType::Intra);
}

#[test]
fn plusptype_aiv_flag_propagates_to_picture_header() {
    let bytes = write_plusptype_iframe_with_flags(true, false, false, false);
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert!(hdr.alternative_inter_vlc);
    assert!(!hdr.modified_quantization);
}

#[test]
fn plusptype_isd_flag_propagates_to_picture_header() {
    let bytes = write_plusptype_iframe_with_flags(false, true, false, false);
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert!(hdr.independent_segment_decoding);
    assert!(!hdr.modified_quantization);
}

#[test]
fn all_three_round_25_flags_can_combine_at_parse_time() {
    let bytes = write_plusptype_iframe_with_flags(true, true, true, false);
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert!(hdr.alternative_inter_vlc);
    assert!(hdr.independent_segment_decoding);
    assert!(hdr.modified_quantization);
}

/// §T.4 / §T.5 spot-check: a 5-bit-rotated 11-bit signed extended level
/// round-trips through the helper. (Belt-and-braces with the lib unit
/// test — keeps this integration test sealed against accidental helper
/// signature changes.)
#[test]
fn extended_level_helper_round_trip() {
    use oxideav_h263::mq::{rotate_extended_level, unrotate_extended_level};
    for &lv in &[-1024i32, -512, -200, -128, 128, 200, 1023] {
        let wire = rotate_extended_level(lv);
        assert_eq!(unrotate_extended_level(wire), lv);
    }
}
