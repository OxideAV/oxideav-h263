//! Integration tests for **Annex S — Alternative INTER VLC** (round 25).
#![allow(clippy::unusual_byte_groupings)]
//!
//! Round 25 wires the helper + header recognition; per-MB plumbing is
//! follow-up work. These tests cover:
//!
//! * `block::decode_ac_aiv` round-trips a baseline INTER block bit-
//!   identically to `block::decode_ac` (the §S.2 try-INTER-then-fallback
//!   path commits the first attempt when no overflow occurs).
//! * The PLUSPTYPE header parser surfaces `alternative_inter_vlc`.
//! * The decoder rejects a picture with the AIV flag set with a specific
//!   `Unsupported` diagnostic citing the helper.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_core::{CodecId, Decoder, Packet, TimeBase};
use oxideav_h263::block::{decode_ac, decode_ac_aiv};
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

/// Synthesise a tiny INTER-block bitstream: one Table 16 codeword
/// (LAST=1, RUN=0, |LEVEL|=1, sign=0). The shortest codeword in the
/// table is `0111s` for `(LAST=0, RUN=0, |LEVEL|=1)` and `0011s` for
/// `(LAST=1, RUN=0, |LEVEL|=1)`. We use the latter so a single
/// codeword closes the block.
///
/// On decode the AC array should have a single non-zero coefficient at
/// position 0 (zig-zag[0] = DCT[0,0]) with reconstruction
/// `q * (2*1 + 1) - (q even ? 1 : 0) = q*3 - (q even ? 1 : 0)`.
fn one_coef_inter_block_bits() -> Vec<u8> {
    let mut bw = BitWriter::new();
    // Table 16 (last=1, run=0, level=1) → `0111s`. Per the inter table
    // shipped with oxideav-mpeg4video, this codeword is `00111` —
    // length 5, with the trailing `s` (sign bit) appended.
    //
    // We don't need to know the exact bit pattern: we just need to
    // round-trip _some_ valid INTER block. The simplest is to emit
    // through `decode_ac`'s inverse — but no encoder helper is exposed.
    // So we emit a hand-rolled (LAST=1, RUN=0, LEVEL=+1) pattern using
    // the encoder's `crate::enc_tables::write_tcoef`-equivalent path.
    //
    // For simplicity and to keep this self-contained, we use an
    // ESCAPE codeword: `0000011` (7 bits) + last=1 (1 bit) + run=0
    // (6 bits) + level=+1 (8-bit signed) = 22 bits.
    bw.write_bits(0b0000011, 7);
    bw.write_bits(1, 1); // last
    bw.write_bits(0, 6); // run
    bw.write_bits(1, 8); // level = +1
    bw.finish()
}

#[test]
fn aiv_helper_matches_baseline_for_normal_inter_block() {
    let bytes = one_coef_inter_block_bits();
    let mut block_aiv = [0i32; 64];
    let mut br = BitReader::new(&bytes);
    decode_ac_aiv(&mut br, &mut block_aiv, 0, 5).unwrap();

    let mut block_baseline = [0i32; 64];
    let mut br = BitReader::new(&bytes);
    decode_ac(&mut br, &mut block_baseline, 0, 5).unwrap();

    assert_eq!(
        block_aiv, block_baseline,
        "AIV helper should match baseline AC decode for a non-overflowing block"
    );
    // Sanity: the dequantised level for q=5, |level|=1, sign=+ is
    // 5 * (2*1 + 1) - (q even ? 1 : 0) = 15 - 0 = 15.
    assert_eq!(block_aiv[0], 15);
}

#[test]
fn aiv_flag_propagates_to_picture_header() {
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
        0b010, false, false, false, false, false, false, false, false, false, true, false,
    );
    bw.write_bits(opp, 18);
    bw.write_bits(0b000_0_0_0_001, 9);
    bw.write_bits(0, 1);
    bw.write_bits(5, 5);
    bw.write_bits(0, 1);
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert!(hdr.alternative_inter_vlc);
    assert!(!hdr.modified_quantization);
    assert!(!hdr.independent_segment_decoding);
}

/// The decoder rejects an AIV-flagged picture with a specific
/// `Unsupported` diagnostic — round-25 wires the helper but not the
/// per-MB plumbing.
#[test]
fn aiv_picture_is_rejected_by_decoder_with_specific_diagnostic() {
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
        0b010, false, false, false, false, false, false, false, false, false, true, false,
    );
    bw.write_bits(opp, 18);
    bw.write_bits(0b000_0_0_0_001, 9);
    bw.write_bits(0, 1);
    bw.write_bits(5, 5);
    bw.write_bits(0, 1);
    let mut bytes = bw.finish();
    bytes.extend_from_slice(&[0u8; 64]);

    let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let pkt = Packet::new(0, TimeBase::new(1, 30), bytes);
    let _ = dec.send_packet(&pkt);
    let flush_err = dec.flush();
    let recv = dec.receive_frame();
    let combined = format!("flush={flush_err:?} recv={recv:?}");
    assert!(
        combined.contains("Annex S") || combined.contains("Alternative INTER VLC"),
        "expected Annex S diagnostic, got: {combined}"
    );
}
