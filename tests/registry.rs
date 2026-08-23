//! End-to-end registry-path tests: the codec must resolve and decode
//! through a fresh `oxideav_core::RuntimeContext` exactly as through
//! the crate's own free functions — the dual-API contract.

use oxideav_core::registry::{Decoder, RuntimeContext};
use oxideav_core::{
    CodecId, CodecOptions, CodecParameters, CodecTag, Error as CoreError, Frame, Packet,
    ProbeContext, TimeBase,
};
use oxideav_h263::encoder::{encode_sequence, GopConfig};
use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(format!("tests/fixtures/{name}/input.h263")).expect("fixture stream")
}

fn registered_context() -> RuntimeContext {
    let mut ctx = RuntimeContext::new();
    oxideav_h263::register(&mut ctx);
    ctx
}

fn video_params() -> CodecParameters {
    CodecParameters::video(CodecId::new("h263"))
}

/// Collect every decoded frame from a registry decoder fed with the
/// given packets, interleaving receive_frame with send_packet the way
/// a pipeline does.
fn run_decoder(dec: &mut dyn Decoder, packets: &[&[u8]]) -> Vec<Frame> {
    let mut out = Vec::new();
    for (i, p) in packets.iter().enumerate() {
        let packet = Packet::new(0, TimeBase::MICROS, p.to_vec()).with_pts(i as i64 * 40_000);
        dec.send_packet(&packet).expect("send_packet");
        loop {
            match dec.receive_frame() {
                Ok(f) => out.push(f),
                Err(CoreError::NeedMore) => break,
                Err(e) => panic!("receive_frame: {e}"),
            }
        }
    }
    dec.flush().expect("flush");
    loop {
        match dec.receive_frame() {
            Ok(f) => out.push(f),
            Err(CoreError::Eof) => break,
            Err(e) => panic!("post-flush receive_frame: {e}"),
        }
    }
    out
}

/// Flatten a framework video frame back to the planar layout of the
/// crate-level `YuvFrame` for comparison.
fn frame_planes(f: &Frame) -> Vec<Vec<u8>> {
    match f {
        Frame::Video(v) => v.image_planes().iter().map(|p| p.data.clone()).collect(),
        other => panic!("expected video frame, got {other:?}"),
    }
}

fn assert_frames_match(registry_frames: &[Frame], direct: &[YuvFrame]) {
    assert_eq!(registry_frames.len(), direct.len(), "frame count");
    for (i, (rf, df)) in registry_frames.iter().zip(direct.iter()).enumerate() {
        let planes = frame_planes(rf);
        assert_eq!(planes.len(), 3, "frame {i} plane count");
        assert_eq!(planes[0], df.y, "frame {i} luma");
        assert_eq!(planes[1], df.cb, "frame {i} Cb");
        assert_eq!(planes[2], df.cr, "frame {i} Cr");
    }
}

#[test]
fn registry_decoder_matches_decode_sequence_whole_stream() {
    let stream = fixture("i-frame-then-p-frame-qcif");
    let direct = decode_sequence(&stream, DecodeOptions::default()).expect("direct decode");
    let ctx = registered_context();
    let mut dec = ctx
        .codecs
        .first_decoder(&video_params())
        .expect("resolve decoder");
    let frames = run_decoder(dec.as_mut(), &[&stream]);
    assert_frames_match(&frames, &direct);
}

#[test]
fn registry_decoder_matches_on_h263p_plusptype_stream() {
    let stream = fixture("h263p-modern");
    let direct = decode_sequence(&stream, DecodeOptions::default()).expect("direct decode");
    let ctx = registered_context();
    let mut dec = ctx
        .codecs
        .first_decoder(&video_params())
        .expect("resolve decoder");
    let frames = run_decoder(dec.as_mut(), &[&stream]);
    assert_frames_match(&frames, &direct);
}

#[test]
fn registry_decoder_handles_arbitrary_packetization() {
    // 7-byte packets shred every picture across many packets; the
    // adapter's PSC re-framing must reassemble them losslessly.
    let stream = fixture("i-frame-then-p-frame-qcif");
    let direct = decode_sequence(&stream, DecodeOptions::default()).expect("direct decode");
    let chunks: Vec<&[u8]> = stream.chunks(7).collect();
    let ctx = registered_context();
    let mut dec = ctx
        .codecs
        .first_decoder(&video_params())
        .expect("resolve decoder");
    let frames = run_decoder(dec.as_mut(), &chunks);
    assert_frames_match(&frames, &direct);
}

#[test]
fn registry_decoder_one_picture_per_packet_yields_frames_without_flush() {
    // Container-style delivery: each packet is exactly one picture.
    // The eager tail decode must yield each frame from its own packet,
    // not one packet late.
    let stream = fixture("i-frame-then-p-frame-qcif");
    let mut boundaries = Vec::new();
    let mut at = 0usize;
    while let Some(p) = oxideav_h263::picture::next_picture_start_code(&stream, at) {
        boundaries.push(p);
        at = p + 1;
    }
    assert!(boundaries.len() >= 2, "fixture holds at least two pictures");
    let mut packets: Vec<&[u8]> = Vec::new();
    for (i, &start) in boundaries.iter().enumerate() {
        let end = boundaries.get(i + 1).copied().unwrap_or(stream.len());
        packets.push(&stream[start..end]);
    }
    let ctx = registered_context();
    let mut dec = ctx
        .codecs
        .first_decoder(&video_params())
        .expect("resolve decoder");
    let mut per_packet_counts = Vec::new();
    for (i, p) in packets.iter().enumerate() {
        let packet = Packet::new(0, TimeBase::MICROS, p.to_vec()).with_pts(i as i64);
        dec.send_packet(&packet).expect("send_packet");
        let mut n = 0;
        loop {
            match dec.receive_frame() {
                Ok(f) => {
                    // The first frame of each picture carries the
                    // packet's PTS.
                    if n == 0 {
                        assert_eq!(f.pts(), Some(i as i64), "packet {i} pts passthrough");
                    }
                    n += 1;
                }
                Err(CoreError::NeedMore) => break,
                Err(e) => panic!("receive_frame: {e}"),
            }
        }
        per_packet_counts.push(n);
    }
    assert!(
        per_packet_counts.iter().all(|&n| n >= 1),
        "every picture packet must yield its frame immediately, got {per_packet_counts:?}"
    );
}

#[test]
fn registry_decoder_reset_decodes_again_from_scratch() {
    let stream = fixture("i-only-qcif-baseline");
    let direct = decode_sequence(&stream, DecodeOptions::default()).expect("direct decode");
    let ctx = registered_context();
    let mut dec = ctx
        .codecs
        .first_decoder(&video_params())
        .expect("resolve decoder");
    let first = run_decoder(dec.as_mut(), &[&stream]);
    assert_frames_match(&first, &direct);
    dec.reset().expect("reset");
    let second = run_decoder(dec.as_mut(), &[&stream]);
    assert_frames_match(&second, &direct);
}

#[test]
fn registry_decoder_honours_decoder_options() {
    // An unknown option key must fail the factory; a known one must
    // change the decode (deblock alters reconstructed pixels).
    let ctx = registered_context();
    let mut params = video_params();
    params.options = CodecOptions::new().set("no_such_option", "1");
    assert!(
        ctx.codecs.first_decoder(&params).is_err(),
        "unknown option must be rejected"
    );

    let stream = fixture("i-frame-then-p-frame-qcif");
    let deblocked_direct = decode_sequence(
        &stream,
        DecodeOptions {
            deblock: true,
            ..DecodeOptions::default()
        },
    )
    .expect("direct deblocked decode");
    let mut params = video_params();
    params.options = CodecOptions::new().set("deblock", "true");
    let mut dec = ctx
        .codecs
        .first_decoder(&params)
        .expect("resolve with options");
    let frames = run_decoder(dec.as_mut(), &[&stream]);
    assert_frames_match(&frames, &deblocked_direct);
}

#[test]
fn registry_tag_and_payload_magic_resolution() {
    let ctx = registered_context();
    for tag in [CodecTag::fourcc(b"H263"), CodecTag::fourcc(b"s263")] {
        let probe = ProbeContext::new(&tag);
        let id = ctx
            .codecs
            .resolve_tag_ref(&probe)
            .unwrap_or_else(|| panic!("tag {tag} must resolve"));
        assert_eq!(id.as_str(), "h263");
    }
    // A raw elementary stream's leading bytes are a byte-aligned PSC.
    let stream = fixture("i-only-qcif-baseline");
    let id = ctx
        .codecs
        .resolve_payload_magic_ref(&stream[..16])
        .expect("payload magic must resolve");
    assert_eq!(id.as_str(), "h263");
}

#[test]
fn macro_entry_point_registers_the_codec() {
    let mut ctx = RuntimeContext::new();
    oxideav_h263::__oxideav_entry(&mut ctx);
    assert!(ctx.codecs.has_decoder(&CodecId::new("h263")));
    assert!(ctx.codecs.has_encoder(&CodecId::new("h263")));
}

#[test]
fn registry_encoder_stream_matches_encode_sequence() {
    // The streaming adapter mirrors encode_sequence's closed loop
    // frame for frame — with identical config + TR seed the
    // concatenated packet payloads must be byte-identical.
    let frames: Vec<YuvFrame> = (0..5)
        .map(|i| {
            let mut f = YuvFrame::grey(176, 144);
            // Moving gradient so P-pictures carry real motion.
            for y in 0..144usize {
                for x in 0..176usize {
                    f.y[y * 176 + x] = ((x * 3 + y * 2 + i * 11) & 0xFF) as u8;
                }
            }
            f
        })
        .collect();
    let cfg = GopConfig {
        quant: 6,
        intra_period: 3,
        search_half: 8,
        umv: false,
        eos: true,
    };
    let direct = encode_sequence(&frames, &cfg, 0).expect("encode_sequence");

    let ctx = registered_context();
    let mut params = video_params();
    params.width = Some(176);
    params.height = Some(144);
    params.options = CodecOptions::new()
        .set("quant", "6")
        .set("gop", "3")
        .set("search", "8")
        .set("eos", "true");
    let mut enc = ctx.codecs.first_encoder(&params).expect("resolve encoder");
    let mut out = Vec::new();
    let mut keyframes = Vec::new();
    for (i, f) in frames.iter().enumerate() {
        let mut vf = oxideav_core::VideoFrame {
            pts: Some(i as i64),
            planes: vec![
                oxideav_core::VideoPlane {
                    stride: 176,
                    data: f.y.clone(),
                },
                oxideav_core::VideoPlane {
                    stride: 88,
                    data: f.cb.clone(),
                },
                oxideav_core::VideoPlane {
                    stride: 88,
                    data: f.cr.clone(),
                },
            ],
        };
        // Exercise the stride-honouring path on one frame: widen the
        // luma stride with per-row padding the adapter must strip.
        if i == 2 {
            let mut padded = Vec::with_capacity(180 * 144);
            for row in f.y.chunks(176) {
                padded.extend_from_slice(row);
                padded.extend_from_slice(&[0xEE; 4]);
            }
            vf.planes[0] = oxideav_core::VideoPlane {
                stride: 180,
                data: padded,
            };
        }
        enc.send_frame(&Frame::Video(vf)).expect("send_frame");
        loop {
            match enc.receive_packet() {
                Ok(p) => {
                    keyframes.push(p.flags.keyframe);
                    out.extend_from_slice(&p.data);
                }
                Err(CoreError::NeedMore) => break,
                Err(e) => panic!("receive_packet: {e}"),
            }
        }
    }
    enc.flush().expect("flush");
    loop {
        match enc.receive_packet() {
            Ok(p) => out.extend_from_slice(&p.data),
            Err(CoreError::Eof) => break,
            Err(e) => panic!("post-flush receive_packet: {e}"),
        }
    }
    assert_eq!(
        out, direct,
        "streaming packets must equal encode_sequence bytes"
    );
    assert_eq!(keyframes, [true, false, false, true, false], "GOP cadence");

    // And the emitted stream round-trips through the registry decoder.
    let mut dec = ctx.codecs.first_decoder(&video_params()).expect("decoder");
    let decoded = run_decoder(dec.as_mut(), &[&out]);
    assert_eq!(decoded.len(), frames.len());
}

#[test]
fn registry_encoder_rejects_bad_geometry_and_options() {
    let ctx = registered_context();
    // Missing dimensions.
    assert!(ctx.codecs.first_encoder(&video_params()).is_err());
    // Non-standard format.
    let mut params = video_params();
    params.width = Some(100);
    params.height = Some(100);
    assert!(ctx.codecs.first_encoder(&params).is_err());
    // Quantiser out of range.
    let mut params = video_params();
    params.width = Some(176);
    params.height = Some(144);
    params.options = CodecOptions::new().set("quant", "32");
    assert!(ctx.codecs.first_encoder(&params).is_err());
}
