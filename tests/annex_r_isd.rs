//! Integration tests for **Annex R — Independent Segment Decoding**
//! (round 25).
#![allow(clippy::unusual_byte_groupings)]
//!
//! Round 25 wires:
//!
//! * Picture-header recognition: PLUSPTYPE OPPTYPE bit 12 → `hdr.independent_segment_decoding`.
//! * `decoder::decode_one_picture` rejects Annex R + Annex K combinations
//!   that don't carry the §K.1 Rectangular Slice (RS) submode (§R.3.1).
//! * `decoder::decode_one_picture` rejects Annex R + (UMV / AP / Annex J)
//!   until the §R.2.4 out-of-segment MV extrapolation is plumbed (round-26
//!   work).
//!
//! Body-driver pieces still pending: §R.2.4 boundary extrapolation, §R.3.2
//! constraint that segmentation must be the same as the temporal
//! reference picture's segmentation. These are flagged with explicit
//! `Error::unsupported` diagnostics so a stream can't silently produce
//! wrong pels.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_core::{CodecId, Decoder, Packet, TimeBase};
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::picture::parse_picture_header;

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
        | b(15, 1)
}

fn write_plusptype_iframe_isd_with_k_no_rs() -> Vec<u8> {
    // Annex R + Annex K, but SSS = 00 (no RS, no ASO) → must be rejected
    // by §R.3.1.
    let mut bw = BitWriter::new();
    bw.write_bits(0b00_0000_0000_0000_0000_1_00000, 22); // PSC
    bw.write_bits(0, 8); // TR
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3); // PLUSPTYPE
    bw.write_bits(0b001, 3); // UFEP
    let opp = pack_opptype(
        0b010, false, false, false, false, false, false, true, false, true, false, false,
    );
    bw.write_bits(opp, 18);
    bw.write_bits(0b000_0_0_0_001, 9); // MPPTYPE I
    bw.write_bits(0, 1); // CPM = 0
    bw.write_bits(0b00, 2); // SSS = 00 (no RS, no ASO)
    bw.write_bits(5, 5); // PQUANT = 5
    bw.write_bits(0, 1); // PEI = 0
                         // Add some trailing 0 bytes so try_consume_slice_header etc. don't
                         // run off the end if the decoder ever gets that far.
    let mut bytes = bw.finish();
    bytes.extend_from_slice(&[0u8; 64]);
    bytes
}

#[test]
fn isd_flag_propagates_to_picture_header() {
    let mut bw = BitWriter::new();
    bw.write_bits(0b00_0000_0000_0000_0000_1_00000, 22);
    bw.write_bits(0, 8);
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3);
    bw.write_bits(0b001, 3);
    let opp = pack_opptype(
        0b010, false, false, false, false, false, false, false, false, true, false, false,
    );
    bw.write_bits(opp, 18);
    bw.write_bits(0b000_0_0_0_001, 9);
    bw.write_bits(0, 1);
    bw.write_bits(5, 5);
    bw.write_bits(0, 1);
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert!(hdr.independent_segment_decoding);
    assert!(!hdr.slice_structured);
}

/// §R.3.1 — Annex R + Annex K shall use Annex K's Rectangular Slice (RS)
/// submode. With SSS = 00 (no RS) the decoder must reject the picture.
#[test]
fn isd_with_annex_k_no_rs_is_rejected_by_decoder() {
    let bytes = write_plusptype_iframe_isd_with_k_no_rs();
    let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let pkt = Packet::new(0, TimeBase::new(1, 30), bytes);
    // The decoder buffers until the next PSC arrives or `flush()` is
    // called; call flush() so the rejection fires before we look for
    // a frame.
    let _ = dec.send_packet(&pkt);
    let flush_err = dec.flush();
    let recv = dec.receive_frame();
    let combined = format!("flush={flush_err:?} recv={recv:?}");
    assert!(
        combined.contains("R.3.1")
            || combined.contains("Rectangular Slice")
            || combined.contains("Annex R"),
        "expected an Annex R / R.3.1 / RS rejection, got: {combined}"
    );
}

#[test]
fn isd_alone_without_other_modes_parses_cleanly_at_header_level() {
    // ISD on, no UMV/AP/J/K — header parses, decode body would still
    // fail due to no actual MB data but the picture-header parse is
    // what we're validating here.
    let mut bw = BitWriter::new();
    bw.write_bits(0b00_0000_0000_0000_0000_1_00000, 22);
    bw.write_bits(0, 8);
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3);
    bw.write_bits(0b001, 3);
    let opp = pack_opptype(
        0b010, false, false, false, false, false, false, false, false, true, false, false,
    );
    bw.write_bits(opp, 18);
    bw.write_bits(0b000_0_0_0_001, 9);
    bw.write_bits(0, 1);
    bw.write_bits(5, 5);
    bw.write_bits(0, 1);
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert!(hdr.independent_segment_decoding);
}
