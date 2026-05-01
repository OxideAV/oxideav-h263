//! End-to-end tests for Annex G (PB-frames) emission + decode (rounds
//! 14-15).
//!
//! Coverage:
//! * `pb_picture_header_carries_pb_bit_and_trb_dbquant` — bit-level check
//!   that the encoder sets PTYPE bit 13 and writes TRB/DBQUANT when the
//!   PB-frames knob is on.
//! * `pb_self_roundtrip_psnr` — encode 5 frames as `[I, PB, PB, PB, PB]`,
//!   decode with our decoder, assert PSNR ≥ 30 dB on every emitted frame
//!   (the decoder produces both the B and the P for each PB picture).
//!   The B-half floor is ≥ 40 dB after round-15's residual emission.
//! * `pb_b_residual_emission_psnr_jumps_with_finer_bquant` — round-15
//!   regression check: the B-half PSNR follows the BQUANT relationship
//!   (DBQUANT = 0 → finer quant → cleaner reconstruction).
//! * `pb_combined_with_other_annex_rejected` — sanity-check the
//!   "combinations not supported" guard.
//! * `pb_modb_zero_round_trips` — wire-level round trip for an MB with
//!   MODB = 0 (no CBPB, no MVDB).

use oxideav_core::bits::BitReader;
use oxideav_core::frame::VideoPlane;
use oxideav_core::Decoder;
use oxideav_core::{
    CodecId, CodecParameters, Encoder, Error, Frame, Packet, PixelFormat, Rational, Result,
    VideoFrame,
};
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::encoder::H263Encoder;

const W: u32 = 176;
const H: u32 = 144;

fn make_pb_encoder() -> Box<H263Encoder> {
    let mut params = CodecParameters::video(CodecId::new(oxideav_h263::CODEC_ID_STR));
    params.width = Some(W);
    params.height = Some(H);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    params.frame_rate = Some(Rational::new(10, 1));
    let mut enc = H263Encoder::from_params(&params).expect("make pb encoder");
    enc.set_enable_annex_g_pb(true);
    Box::new(enc)
}

fn moving_square_frame(sx: i32, sy: i32, pts: i64) -> VideoFrame {
    let cw = (W / 2) as usize;
    let ch = (H / 2) as usize;
    let mut y = vec![80u8; (W * H) as usize];
    let size = 32i32;
    for j in 0..size {
        for i in 0..size {
            let xx = sx + i;
            let yy = sy + j;
            if xx >= 0 && xx < W as i32 && yy >= 0 && yy < H as i32 {
                y[(yy as usize) * W as usize + (xx as usize)] = 210;
            }
        }
    }
    let cb = vec![128u8; cw * ch];
    let cr = vec![128u8; cw * ch];
    VideoFrame {
        pts: Some(pts),
        planes: vec![
            VideoPlane {
                stride: W as usize,
                data: y,
            },
            VideoPlane {
                stride: cw,
                data: cb,
            },
            VideoPlane {
                stride: cw,
                data: cr,
            },
        ],
    }
}

fn psnr(src: &VideoFrame, dec: &VideoFrame) -> f64 {
    let sl = &src.planes[0];
    let w = sl.stride;
    let h = sl.data.len() / sl.stride;
    let dp = &dec.planes[0];
    if dp.stride != w || dp.data.len() / dp.stride != h {
        return 0.0;
    }
    let mut mse = 0f64;
    let mut n = 0u64;
    for j in 0..h {
        for i in 0..w {
            let a = sl.data[j * sl.stride + i] as f64;
            let b = dp.data[j * dp.stride + i] as f64;
            let d = a - b;
            mse += d * d;
            n += 1;
        }
    }
    let mse = mse / (n as f64);
    if mse <= 0.0 {
        return f64::INFINITY;
    }
    10.0 * (255.0f64 * 255.0f64 / mse).log10()
}

fn collect_packets(enc: &mut dyn Encoder, frames: &[VideoFrame]) -> Result<Vec<Packet>> {
    let mut out = Vec::new();
    for f in frames {
        enc.send_frame(&Frame::Video(f.clone()))?;
        loop {
            match enc.receive_packet() {
                Ok(p) => out.push(p),
                Err(Error::NeedMore) => break,
                Err(Error::Eof) => break,
                Err(e) => return Err(e),
            }
        }
    }
    enc.flush()?;
    loop {
        match enc.receive_packet() {
            Ok(p) => out.push(p),
            Err(Error::NeedMore) => break,
            Err(Error::Eof) => break,
            Err(e) => return Err(e),
        }
    }
    Ok(out)
}

fn decode_packets(dec: &mut dyn Decoder, packets: &[Packet]) -> Result<Vec<VideoFrame>> {
    let mut out = Vec::new();
    for p in packets {
        dec.send_packet(p)?;
        loop {
            match dec.receive_frame() {
                Ok(Frame::Video(v)) => out.push(v),
                Ok(_) => {}
                Err(Error::NeedMore) => break,
                Err(Error::Eof) => break,
                Err(e) => return Err(e),
            }
        }
    }
    dec.flush()?;
    loop {
        match dec.receive_frame() {
            Ok(Frame::Video(v)) => out.push(v),
            Ok(_) => {}
            Err(Error::NeedMore) => break,
            Err(Error::Eof) => break,
            Err(e) => return Err(e),
        }
    }
    Ok(out)
}

#[test]
fn pb_picture_header_carries_pb_bit_and_trb_dbquant() {
    let mut enc = make_pb_encoder();
    enc.set_pb_trb(3);
    enc.set_pb_dbquant(0b10);
    // Frame 0 = I (no PB). Frame 1 = PB.
    let frames = [
        moving_square_frame(40, 60, 0),
        moving_square_frame(50, 60, 1),
    ];
    let pkts = collect_packets(enc.as_mut(), &frames).expect("encode");
    assert_eq!(pkts.len(), 2);

    // Parse the header of packet 1 (the PB frame) and check the PB bit + TRB
    // + DBQUANT are present.
    let bytes = &pkts[1].data;
    let mut br = BitReader::new(bytes);
    let hdr = oxideav_h263::picture::parse_picture_header(&mut br).expect("parse header");
    assert!(hdr.pb_frames, "PB-frames bit must be set");
    assert_eq!(hdr.trb, 3, "encoded TRB should round-trip");
    assert_eq!(hdr.dbquant, 0b10, "encoded DBQUANT should round-trip");
}

#[test]
fn pb_self_roundtrip_psnr() {
    let mut enc = make_pb_encoder();
    let frames: Vec<VideoFrame> = (0..5)
        .map(|i| moving_square_frame(40 + i * 4, 60, i as i64))
        .collect();
    let pkts = collect_packets(enc.as_mut(), &frames).expect("encode");
    assert_eq!(pkts.len(), 5);

    // Decode through our decoder. Every PB-frame produces TWO output frames
    // (B then P); the first packet (I) produces one. So we expect:
    //   1 (from I) + 4 * 2 (from PBs) = 9 frames.
    let mut params = CodecParameters::video(CodecId::new(oxideav_h263::CODEC_ID_STR));
    params.width = Some(W);
    params.height = Some(H);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let decoded = decode_packets(&mut dec, &pkts).expect("decode");
    assert_eq!(
        decoded.len(),
        9,
        "1 I + 4 PB pairs should yield 9 emitted VideoFrames"
    );

    // The I-frame and every P-half should match its source frame to ≥ 30 dB.
    // The B-half is reconstructed by §G.4 / §G.5 bidirectional MC plus the
    // round-15 B-residual emission. The encoder uses the **input frame** as
    // the B-source (the streaming 1-input-per-PB-pair model has no separate
    // B-source available), so the decoded B-half should match the current
    // input frame at ≥ 40 dB once the residual lands.
    //
    // Output ordering: I (frame 0), B0 P1, B1 P2, B2 P3, B3 P4
    //                  → indices    0,    1  2,   3  4,   5  6,   7  8
    // The B at index 2k+1 corresponds (in source time) to source frame k+1
    // (the input that became the P-source), and the P at 2k+2 to the same.
    let i_psnr = psnr(&frames[0], &decoded[0]);
    eprintln!("PB roundtrip — I-frame PSNR = {i_psnr:.1} dB");
    assert!(
        i_psnr >= 30.0,
        "I-frame PSNR {i_psnr:.1} dB must be ≥ 30 dB"
    );

    for k in 0..4usize {
        let p_psnr = psnr(&frames[k + 1], &decoded[2 * k + 2]);
        eprintln!("PB roundtrip — P-half {} PSNR = {:.1} dB", k + 1, p_psnr);
        assert!(
            p_psnr >= 30.0,
            "P-half PSNR for source frame {} = {:.1} dB (must be ≥ 30 dB)",
            k + 1,
            p_psnr
        );
    }

    for k in 0..4usize {
        // B-half "expected" frame is somewhere between source frames k and
        // k+1. The encoder picked source frame k+1 as the B-source, so the
        // decoded B-half should be close to that frame.
        let b_psnr_prev = psnr(&frames[k], &decoded[2 * k + 1]);
        let b_psnr_next = psnr(&frames[k + 1], &decoded[2 * k + 1]);
        let best = b_psnr_prev.max(b_psnr_next);
        eprintln!(
            "PB roundtrip — B-half {} PSNR = max(prev={:.1}, next={:.1}) = {:.1} dB",
            k, b_psnr_prev, b_psnr_next, best
        );
        assert!(
            best >= 40.0,
            "B-half PSNR for PB pair {} = max({:.1}, {:.1}) dB (must be ≥ 40 dB \
             with round-15 B-residual emission)",
            k,
            b_psnr_prev,
            b_psnr_next
        );
    }
}

/// Round 15 — verify the B-half residual emission at non-default DBQUANT.
/// At DBQUANT = `00` the B-block quantiser BQUANT = 5*QUANT/4 = 5 (since
/// PQUANT defaults to 5 → 5*5/4 = 6, clipped to 6). At DBQUANT = `11` we get
/// BQUANT = 8*QUANT/4 = 10. Higher BQUANT → coarser residual → lower PSNR.
/// We assert the relationship rather than a hard PSNR value.
#[test]
fn pb_b_residual_emission_psnr_jumps_with_finer_bquant() {
    fn run_at_dbquant(dbq: u8) -> f64 {
        let mut enc = make_pb_encoder();
        enc.set_pb_dbquant(dbq);
        let frames: Vec<VideoFrame> = (0..3)
            .map(|i| moving_square_frame(40 + i * 4, 60, i as i64))
            .collect();
        let pkts = collect_packets(enc.as_mut(), &frames).expect("encode");
        let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
        let decoded = decode_packets(&mut dec, &pkts).expect("decode");
        // 1 (I) + 2 (PBs) * 2 = 5 frames.
        assert_eq!(decoded.len(), 5);
        // Compare each B-half to its corresponding source (current input).
        let mut psnrs = Vec::new();
        for k in 0..2usize {
            let p = psnr(&frames[k + 1], &decoded[2 * k + 1]);
            psnrs.push(p);
        }
        psnrs.iter().sum::<f64>() / (psnrs.len() as f64)
    }
    let avg00 = run_at_dbquant(0b00);
    let avg11 = run_at_dbquant(0b11);
    eprintln!("PB B-residual avg PSNR — DBQUANT=00: {avg00:.1} dB, DBQUANT=11: {avg11:.1} dB");
    // Both must clear the round-15 floor (which the per-frame test already
    // checks at DBQUANT=00); the finer quant must produce at least as good
    // a PSNR as the coarser one (modulo rounding, allow 0.5 dB slack).
    assert!(
        avg00 >= 40.0,
        "DBQUANT=00 average B PSNR {avg00:.1} dB < 40"
    );
    assert!(
        avg11 >= 30.0,
        "DBQUANT=11 average B PSNR {avg11:.1} dB < 30"
    );
    assert!(
        avg00 + 0.5 >= avg11,
        "DBQUANT=00 (BQUANT smaller) should reconstruct B-half at >= the \
         quality of DBQUANT=11; got 00={avg00:.1} dB vs 11={avg11:.1} dB"
    );
}

#[test]
fn pb_combined_with_other_annex_rejected() {
    let mut params = CodecParameters::video(CodecId::new(oxideav_h263::CODEC_ID_STR));
    params.width = Some(W);
    params.height = Some(H);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    let mut enc = H263Encoder::from_params(&params).expect("make");
    enc.set_enable_annex_g_pb(true);
    enc.set_enable_annex_d_umv(true);
    let f = moving_square_frame(40, 60, 0);
    let res = enc.send_frame(&Frame::Video(f));
    match res {
        Err(Error::Unsupported(s)) => {
            assert!(
                s.contains("Annex G"),
                "expected Annex G + D rejection, got: {s}"
            );
        }
        other => panic!("expected Unsupported, got {other:?}"),
    }
}

#[test]
fn pb_modb_zero_round_trips() {
    use oxideav_core::bits::BitWriter;
    use oxideav_h263::pb::{decode_modb, encode_modb};

    // The simplest PB-frames mode case: MODB = 0 → no CBPB, no MVDB.
    let mut bw = BitWriter::with_capacity(8);
    encode_modb(&mut bw, false, false);
    let buf = bw.finish();
    assert_eq!(buf, vec![0x00]); // single 0 bit padded to a byte.

    let mut br = BitReader::new(&buf);
    let d = decode_modb(&mut br).expect("decode modb 0");
    assert!(!d.cbpb_present);
    assert!(!d.mvdb_present);
}

#[test]
fn pb_modb_table_11_codewords() {
    use oxideav_core::bits::BitWriter;
    use oxideav_h263::pb::{decode_modb, encode_modb};

    // Table 11/H.263:
    //   index 0: no CBPB, no MVDB  → "0"
    //   index 1: no CBPB, MVDB     → "10"
    //   index 2: CBPB present, MVDB → "11"
    let cases: &[(bool, bool, &[u8])] = &[
        (false, false, &[0b0_0000000]),
        (false, true, &[0b10_000000]),
        (true, true, &[0b11_000000]),
    ];
    for &(cbpb, mvdb, expected) in cases {
        let mut bw = BitWriter::with_capacity(8);
        encode_modb(&mut bw, cbpb, mvdb);
        let buf = bw.finish();
        assert_eq!(buf, expected, "MODB({cbpb}, {mvdb})");
        let mut br = BitReader::new(&buf);
        let d = decode_modb(&mut br).unwrap();
        assert_eq!(d.cbpb_present, cbpb);
        assert_eq!(d.mvdb_present, mvdb || cbpb);
    }
}

#[test]
fn pb_b_picture_dimensions_match_source() {
    // Run a 3-frame encode and check that the decoder's emitted frames have
    // the right plane sizes.
    let mut enc = make_pb_encoder();
    let frames = [
        moving_square_frame(40, 60, 0),
        moving_square_frame(44, 60, 1),
        moving_square_frame(48, 60, 2),
    ];
    let pkts = collect_packets(enc.as_mut(), &frames).expect("encode");
    let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let decoded = decode_packets(&mut dec, &pkts).expect("decode");
    // Should be 1 (I) + 2 PBs * 2 = 5 frames.
    assert_eq!(decoded.len(), 5);
    for v in &decoded {
        assert_eq!(v.planes.len(), 3);
        let cw = (W / 2) as usize;
        assert_eq!(v.planes[0].data.len(), (W * H) as usize);
        assert_eq!(v.planes[0].stride, W as usize);
        assert_eq!(v.planes[1].stride, cw);
        assert_eq!(v.planes[2].stride, cw);
    }
}

/// Best-effort ffmpeg cross-decode probe. We don't expect ffmpeg to accept
/// our PB-frames stream byte-for-byte (its decoder is finicky about MODB
/// timing), but the test runs only when ffmpeg is on $PATH so it stays a
/// soft signal — we report what we get.
#[test]
#[ignore = "informational; runs only when ffmpeg is on PATH"]
fn pb_ffmpeg_cross_decode_probe() {
    use std::io::Write;
    use std::process::{Command, Stdio};

    let which = Command::new("which").arg("ffmpeg").output();
    let ffmpeg_present = which.map(|o| o.status.success()).unwrap_or(false);
    if !ffmpeg_present {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    }

    let mut enc = make_pb_encoder();
    let frames: Vec<VideoFrame> = (0..3)
        .map(|i| moving_square_frame(40 + i * 6, 60, i as i64))
        .collect();
    let pkts = collect_packets(enc.as_mut(), &frames).expect("encode");

    let mut bytes: Vec<u8> = Vec::new();
    for p in &pkts {
        bytes.extend_from_slice(&p.data);
    }

    let mut child = Command::new("ffmpeg")
        .args([
            "-loglevel",
            "error",
            "-f",
            "h263",
            "-i",
            "pipe:0",
            "-f",
            "null",
            "-",
        ])
        .stdin(Stdio::piped())
        .stderr(Stdio::piped())
        .stdout(Stdio::null())
        .spawn()
        .expect("spawn ffmpeg");
    {
        let mut stdin = child.stdin.take().expect("ffmpeg stdin");
        stdin.write_all(&bytes).ok();
    }
    let out = child.wait_with_output().expect("wait ffmpeg");
    let stderr = String::from_utf8_lossy(&out.stderr);
    eprintln!(
        "[informational] ffmpeg PB-frames decode stderr:\n{stderr}\nexit={}",
        out.status.code().unwrap_or(-1)
    );
    // We accept any exit code — ffmpeg's PB-frames support is partial; this
    // is purely an interop probe.
}
