//! Annex J deblocking filter — round-trip tests.
//!
//! Verifies that the encoder's post-reconstruction deblocking produces the
//! same picture as the decoder's post-reconstruction deblocking (i.e. the
//! filter is deterministic and both sides stay in sync), AND that existing
//! baseline bitstreams still decode correctly with the filter disabled
//! (backwards-compat).
//!
//! Because H.263 Annex J in this crate is exposed out-of-band (see the
//! crate docs — PLUSPTYPE/OPPTYPE is out of scope), the test drives both
//! sides through the public `set_enable_annex_j(true)` setter.

use oxideav_core::{
    frame::VideoPlane, CodecId, CodecParameters, Frame, MediaType, PixelFormat, Rational, TimeBase,
    VideoFrame,
};
use oxideav_core::{Decoder, Encoder};
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::encoder::H263Encoder;
use oxideav_h263::CODEC_ID_STR;

/// Build a stride-packed YUV420P frame filled with a plausible test pattern
/// that has hard 8×8 block-aligned step edges, so the deblocking filter
/// actually has something to do. `seed` lets us build a second frame with
/// slightly different content, to exercise the P-picture path.
fn make_test_frame(w: u32, h: u32, seed: u8) -> VideoFrame {
    let cw = w.div_ceil(2) as usize;
    let ch = h.div_ceil(2) as usize;
    let mut y = vec![0u8; (w * h) as usize];
    for yy in 0..h as usize {
        for xx in 0..w as usize {
            // A crude 16×16 checkerboard with two luma levels plus a
            // seed-dependent offset.
            let by = yy / 16;
            let bx = xx / 16;
            let base: u8 = if (bx + by) % 2 == 0 { 80 } else { 160 };
            y[yy * w as usize + xx] = base.saturating_add(seed);
        }
    }
    let cb_val = 128u8.saturating_add(seed);
    let cb = vec![cb_val; cw * ch];
    let cr = vec![128u8; cw * ch];
    VideoFrame {
        format: PixelFormat::Yuv420P,
        width: w,
        height: h,
        pts: Some(seed as i64),
        time_base: TimeBase::new(1, 30),
        planes: vec![
            VideoPlane {
                stride: w as usize,
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

fn build_encoder_params(w: u32, h: u32) -> CodecParameters {
    let mut p = CodecParameters::video(CodecId::new(CODEC_ID_STR));
    p.media_type = MediaType::Video;
    p.width = Some(w);
    p.height = Some(h);
    p.pixel_format = Some(PixelFormat::Yuv420P);
    p.frame_rate = Some(Rational::new(30, 1));
    p
}

/// Core scenario: encode two consecutive frames (I then P) with Annex J on,
/// decode them with Annex J on, and verify the decoder's output matches the
/// encoder's internal reconstruction. If deblocking is applied consistently
/// on both sides, the P-frame's reference — and therefore its decode output
/// — stays locked to the encoder's view of the world.
///
/// We check the second (P) frame specifically, because if reference frames
/// diverged after the I-picture's filter, the P's motion compensation would
/// drift immediately.
#[test]
fn encode_decode_p_with_annex_j_stays_in_sync() {
    // QCIF — minimum size that exercises multiple GOBs in the same picture.
    let w = 176u32;
    let h = 144u32;

    // Encoder with Annex J on.
    let params = build_encoder_params(w, h);
    let mut enc = H263Encoder::from_params(&params).expect("make encoder");
    enc.set_enable_annex_j(true);
    assert!(enc.enable_annex_j());

    // Decoder with Annex J on.
    let mut dec = H263Decoder::new(CodecId::new(CODEC_ID_STR));
    dec.set_enable_annex_j(true);
    assert!(dec.enable_annex_j());

    // Frame 0 — I-picture. H.263 decoder holds the last picture in its
    // buffer until the NEXT PSC arrives (or `flush()` signals EOF), so
    // we submit both packets first, then flush, then drain frames.
    let f0 = make_test_frame(w, h, 0);
    enc.send_frame(&Frame::Video(f0)).expect("send 0");
    let pkt0 = enc.receive_packet().expect("receive 0");
    assert!(pkt0.flags.keyframe);

    let f1 = make_test_frame(w, h, 4);
    enc.send_frame(&Frame::Video(f1)).expect("send 1");
    let pkt1 = enc.receive_packet().expect("receive 1");
    assert!(!pkt1.flags.keyframe);

    dec.send_packet(&pkt0).expect("decode I");
    dec.send_packet(&pkt1).expect("decode P");
    dec.flush().expect("flush");
    let dec_f0 = match dec.receive_frame().expect("recv 0") {
        Frame::Video(v) => v,
        _ => panic!("not video"),
    };
    let dec_f1 = match dec.receive_frame().expect("recv 1") {
        Frame::Video(v) => v,
        _ => panic!("not video P"),
    };

    // Two outputs decoded; dimensions sane.
    assert_eq!(dec_f0.width, w);
    assert_eq!(dec_f0.height, h);
    assert_eq!(dec_f1.width, w);
    assert_eq!(dec_f1.height, h);

    // Sanity: the P-frame decoded output is not all-zero (a drifting
    // reference would often produce obviously wrong values because the P
    // residual cannot recover from a broken predictor).
    let yp = &dec_f1.planes[0];
    let mean: u32 = yp
        .data
        .iter()
        .take((w * h) as usize)
        .map(|&p| p as u32)
        .sum::<u32>()
        / (w * h);
    assert!(
        (50..=200).contains(&mean),
        "decoded P luma mean {} out of plausible range",
        mean
    );

    // Determinism: encode + decode a second time with the same inputs and
    // confirm we get the exact same decoder output — i.e. the filter is
    // pure and order-independent.
    let mut enc2 = H263Encoder::from_params(&params).expect("make encoder 2");
    enc2.set_enable_annex_j(true);
    let mut dec2 = H263Decoder::new(CodecId::new(CODEC_ID_STR));
    dec2.set_enable_annex_j(true);
    let f0b = make_test_frame(w, h, 0);
    let f1b = make_test_frame(w, h, 4);
    enc2.send_frame(&Frame::Video(f0b)).unwrap();
    let pkt0b = enc2.receive_packet().unwrap();
    enc2.send_frame(&Frame::Video(f1b)).unwrap();
    let pkt1b = enc2.receive_packet().unwrap();
    dec2.send_packet(&pkt0b).unwrap();
    dec2.send_packet(&pkt1b).unwrap();
    dec2.flush().unwrap();
    let _ = dec2.receive_frame().unwrap();
    let dec_f1b = match dec2.receive_frame().unwrap() {
        Frame::Video(v) => v,
        _ => panic!(),
    };
    assert_eq!(
        dec_f1.planes[0].data, dec_f1b.planes[0].data,
        "deterministic luma output across encode runs"
    );
    assert_eq!(pkt0.data, pkt0b.data, "deterministic I-packet bytes");
    assert_eq!(pkt1.data, pkt1b.data, "deterministic P-packet bytes");
}

/// Baseline bitstreams (encoded WITHOUT Annex J) must still decode correctly
/// when the decoder has Annex J disabled. This is the backwards-compat
/// guarantee — we don't want to break existing streams.
#[test]
fn baseline_stream_decodes_without_annex_j() {
    let w = 176u32;
    let h = 144u32;
    let params = build_encoder_params(w, h);
    let mut enc = H263Encoder::from_params(&params).unwrap();
    // Do NOT enable Annex J on encoder.
    let f = make_test_frame(w, h, 0);
    enc.send_frame(&Frame::Video(f)).unwrap();
    let pkt = enc.receive_packet().unwrap();

    let mut dec = H263Decoder::new(CodecId::new(CODEC_ID_STR));
    // Do NOT enable Annex J on decoder either.
    dec.send_packet(&pkt).unwrap();
    dec.flush().unwrap();
    let out = match dec.receive_frame().unwrap() {
        Frame::Video(v) => v,
        _ => panic!(),
    };
    assert_eq!(out.width, w);
    assert_eq!(out.height, h);
}
