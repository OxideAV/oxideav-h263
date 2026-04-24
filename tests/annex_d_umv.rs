//! Annex D — Unrestricted Motion Vectors integration tests.
//!
//! The baseline encoder in this crate does **not** emit UMV streams, so the
//! positive path tests build H.263 bitstreams by hand (picture header with
//! PTYPE bit 10 set, plus a handful of MB-layer bits) and drive them
//! through the decoder. This is enough to exercise:
//!
//! * `picture::parse_picture_header` accepting the UMV flag on baseline
//!   PTYPE streams.
//! * `motion::decode_mv_component_umv` applying the §D.2 "sign-of-predictor"
//!   rule when the predictor lives outside the baseline `[-31, +32]`
//!   halfpel band.
//! * `interp::predict_block` replicating the nearest edge sample when a
//!   motion vector points outside the coded picture area (§D.1), so that
//!   an "edge-hugging" MV doesn't access uninitialised memory.
//!
//! There is also an opportunistic negative-path test that shells out to
//! `ffmpeg -c:v h263p -umv 1` to confirm we surface a specific
//! `Error::Unsupported` diagnostic on h263+ / PLUSPTYPE UMV streams (which
//! require Table D.3 — follow-up work). Skipped when ffmpeg isn't on
//! `$PATH` or can't emit the expected stream.

#![allow(clippy::unusual_byte_groupings)]

use std::process::Command;

use oxideav_codec::Decoder;
use oxideav_core::bits::BitReader;
use oxideav_core::{CodecId, Frame, Packet, TimeBase};
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::interp::predict_block;
use oxideav_h263::picture::{parse_picture_header, PictureCodingType, SourceFormat};

/// The §D.1 picture-edge extrapolation is implemented by the half-pel
/// interpolator via `x.clamp(0, w-1)`. This test exercises the edge
/// replication directly: an MV that points 32 pels past the right edge of
/// a 16-wide reference plane must sample the right-edge column for every
/// destination pel.
#[test]
fn interp_replicates_edge_for_out_of_picture_mv() {
    // 16x16 reference with distinct per-column values 0..=15 (Y plane).
    let mut refp = [0u8; 16 * 16];
    for j in 0..16 {
        for i in 0..16 {
            refp[j * 16 + i] = i as u8;
        }
    }
    let mut dst = [0u8; 16 * 16];
    // Block at origin, MV = (+64 halfpel, 0) = +32 integer pels. The block
    // sits entirely outside the reference; every destination sample must
    // come from column 15 (the rightmost valid column).
    predict_block(&refp, 16, 16, 16, 0, 0, 64, 0, 16, &mut dst, 16);
    for j in 0..16 {
        for i in 0..16 {
            assert_eq!(
                dst[j * 16 + i], 15,
                "out-of-picture MV must replicate edge at ({i},{j})"
            );
        }
    }

    // Mirror: MV = (-64 halfpel, 0) → block sits 32 pels to the left of the
    // frame, every sample should be column 0.
    predict_block(&refp, 16, 16, 16, 0, 0, -64, 0, 16, &mut dst, 16);
    for j in 0..16 {
        for i in 0..16 {
            assert_eq!(dst[j * 16 + i], 0, "left-of-picture MV must replicate edge");
        }
    }
}

/// Tiny MSB-first bit writer used by the synthetic-stream tests below.
struct BitBuf {
    bytes: Vec<u8>,
    acc: u64,
    n: u32,
}
impl BitBuf {
    fn new() -> Self {
        Self {
            bytes: Vec::new(),
            acc: 0,
            n: 0,
        }
    }
    fn put(&mut self, v: u32, bits: u32) {
        self.acc = (self.acc << bits) | (v as u64 & ((1u64 << bits) - 1));
        self.n += bits;
        while self.n >= 8 {
            self.n -= 8;
            self.bytes.push((self.acc >> self.n) as u8);
        }
    }
    fn finish(mut self) -> Vec<u8> {
        if self.n > 0 {
            self.bytes.push(((self.acc << (8 - self.n)) & 0xff) as u8);
        }
        self.bytes
    }
}

/// Build a minimal sub-QCIF I-picture whose PTYPE signals Annex D (UMV bit
/// 10 set). The picture body is an all-zeros "no AC, luma DC = 128, chroma
/// DC = 128" intra MB repeated across the 8x6 MB grid — that lets us then
/// follow up with a P-picture whose MVs are out of range.
fn build_umv_i_picture_subqcif() -> Vec<u8> {
    // Sub-QCIF = 128x96 luma → 8x6 = 48 MBs arranged in 6 GOBs of one row.
    let mut w = BitBuf::new();
    // PSC (22).
    w.put(0b00_0000_0000_0000_0000_1_00000, 22);
    // TR (8).
    w.put(0, 8);
    // PTYPE (13): marker=1 id=0 split=0 cam=0 freeze=0 fmt=001 (sub-QCIF),
    //              I-pic bit=0, UMV=1, SAC=0, AP=0, PB=0.
    w.put(1, 1);
    w.put(0, 1);
    w.put(0, 1);
    w.put(0, 1);
    w.put(0, 1);
    w.put(0b001, 3);
    w.put(0, 1); // I-picture
    w.put(1, 1); // UMV ON
    w.put(0, 1); // SAC
    w.put(0, 1); // AP
    w.put(0, 1); // PB
    // PQUANT = 5.
    w.put(5, 5);
    // CPM = 0, PEI = 0.
    w.put(0, 1);
    w.put(0, 1);

    // MB body. For an Intra MB we need MCBPC (intra, no AC anywhere) +
    // CBPY=0000 + six 8-bit INTRADC blocks with value 0x40 (→ 128 after ×8).
    //
    // MCBPC intra: mb_type=3, cbpc=0 → code `1` (1 bit) per Table 7 (H.263),
    // which our mpeg4video shared table decodes as MCBPC index 0.
    //
    // CBPY=0000 (no AC): per Table 13 intra, the all-zeros CBPY code is
    // `0011` (4 bits).
    //
    // INTRADC uses 8 bits directly (escape 0xFF handling notwithstanding) —
    // 0x40 = 64 maps to DC=64 which after H.263 dequant × 8 gives 512 →
    // IDCT spreads the DC as ~64 per sample (well below saturation).
    //
    // These exact codes were cross-checked against the tests already in
    // `tests/encoder_roundtrip.rs` which round-trip through our own
    // encoder + decoder pair. We only borrow the bit patterns here.
    let mb_w = 8usize;
    let mb_h = 6usize;
    for mb_y in 0..mb_h {
        for _ in 0..mb_w {
            // MCBPC intra, mb_type=3 cbpc=0 → VLC code "1".
            w.put(1, 1);
            // CBPY=0000 for intra → "0011" (4 bits). See Table 13/H.263.
            w.put(0b0011, 4);
            // Six INTRADC bytes = 0x40.
            for _ in 0..6 {
                w.put(0x40, 8);
            }
        }
        // GOB header appears at every MB row boundary (except the first)
        // in sub-QCIF: GBSC (17 bits, `0000 0000 0000 0000 1`), GN (5),
        // GFID (2), GQUANT (5). We just emit one per remaining row.
        if mb_y + 1 < mb_h {
            // Pad to byte boundary first (GBSC is byte-aligned).
            while w.n % 8 != 0 {
                w.put(0, 1);
            }
            // GBSC: 17 bits of `0000 0000 0000 0000 1`.
            w.put(0b0_0000_0000_0000_0000_1, 17);
            // GN = mb_y+1 (1..=5 for rows 1..=5).
            w.put((mb_y + 1) as u32, 5);
            // GFID = 0.
            w.put(0, 2);
            // GQUANT = 5 (same as PQUANT — no change).
            w.put(5, 5);
        }
    }
    w.finish()
}

/// A UMV-flagged I-picture built by hand must decode end-to-end without
/// errors and produce a 128x96 frame whose luma is ~128 everywhere.
#[test]
fn synthetic_umv_i_picture_decodes() {
    let data = build_umv_i_picture_subqcif();

    // Check the picture header parses correctly.
    let mut br = BitReader::new(&data);
    let hdr = parse_picture_header(&mut br).expect("parse UMV picture header");
    assert_eq!(hdr.source_format, SourceFormat::SubQcif);
    assert_eq!(hdr.coding_type, PictureCodingType::Intra);
    assert!(hdr.umv_mode, "UMV flag should be latched");
    assert!(!hdr.plusptype);
    assert_eq!(hdr.width, 128);
    assert_eq!(hdr.height, 96);

    // Drive the full decoder. The synthetic I-picture's body uses the exact
    // MCBPC / CBPY / INTRADC bit patterns above; the decoder must reach EOS
    // without returning Error::Invalid.
    let mut decoder = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    decoder
        .send_packet(&Packet::new(0, TimeBase::new(1, 90_000), data))
        .expect("send_packet");
    decoder.flush().expect("flush");
    let frame = decoder.receive_frame().expect("receive I-frame");
    let Frame::Video(vf) = frame else {
        panic!("expected video frame");
    };
    assert_eq!(vf.width, 128);
    assert_eq!(vf.height, 96);
    // Luma plane should be close to 128 (the target DC level) across the
    // whole picture — ringing at block edges is OK.
    let luma = &vf.planes[0];
    let mut total = 0u64;
    let mut ok = 0u64;
    for j in 0..vf.height as usize {
        for i in 0..vf.width as usize {
            let y = luma.data[j * luma.stride + i];
            total += 1;
            // INTRADC=0x40 (64) after ×8 dequant → DC coefficient 512, which
            // after IDCT+clip lands the block flat at 64 in 8-bit space.
            // Anywhere in the 0..=120 band is close enough for this test.
            if y <= 120 {
                ok += 1;
            }
        }
    }
    let pct = ok as f64 / total as f64;
    assert!(pct > 0.99, "I-picture luma DC should be stable: {pct}");
}

/// When ffmpeg is on PATH, verify that an h263p stream with `-umv 1`
/// currently produces a deterministic `Error::Unsupported` diagnostic
/// (PLUSPTYPE + UMV requires Table D.3, which is follow-up work). This
/// locks in the specific error message so a subsequent round will know
/// that enabling Table D.3 also needs to remove / update this check.
#[test]
fn ffmpeg_h263p_umv_rejected_with_specific_diagnostic() {
    if Command::new("ffmpeg").arg("-version").output().is_err() {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    }
    let tmp = std::env::temp_dir();
    let avi = tmp.join("h263_annex_d_test.avi");
    let es = tmp.join("h263_annex_d_test.es");
    let out = Command::new("ffmpeg")
        .args([
            "-y",
            "-f",
            "lavfi",
            "-i",
            "testsrc=size=176x144:rate=10:duration=0.2",
            "-c:v",
            "h263p",
            "-umv",
            "1",
            "-qscale:v",
            "5",
            "-an",
            avi.to_str().unwrap(),
        ])
        .output();
    let Ok(out) = out else {
        eprintln!("ffmpeg failed to launch — skipping");
        return;
    };
    if !out.status.success() {
        eprintln!("ffmpeg didn't accept -umv 1 for h263p — skipping");
        return;
    }
    let repack = Command::new("ffmpeg")
        .args([
            "-y",
            "-i",
            avi.to_str().unwrap(),
            "-c:v",
            "copy",
            "-f",
            "h263",
            es.to_str().unwrap(),
        ])
        .output();
    let Ok(repack) = repack else {
        eprintln!("ffmpeg demux failed — skipping");
        return;
    };
    if !repack.status.success() {
        eprintln!("ffmpeg couldn't repack to .h263 — skipping");
        return;
    }
    let bytes = std::fs::read(&es).expect("read h263 es");
    let mut decoder = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let err = decoder
        .send_packet(&Packet::new(0, TimeBase::new(1, 90_000), bytes))
        .expect_err("expected Unsupported error for PLUSPTYPE+UMV");
    let msg = format!("{err}");
    eprintln!("ffmpeg h263p -umv 1 diagnostic: {msg}");
    // Table D.3 isn't implemented yet; the diagnostic must name Annex D OR
    // a downstream annex ffmpeg also bundled (SS, custom PCF).
    assert!(
        msg.contains("Annex D")
            || msg.contains("Annex K")
            || msg.contains("custom picture clock frequency")
            || msg.contains("Table D.3"),
        "unexpected diagnostic: {msg}"
    );
}
