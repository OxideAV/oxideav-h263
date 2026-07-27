//! RFC 4629 RTP packetization round-trip tests: the packetizer splits
//! real H.263 elementary streams (both crate-encoded and the vendored
//! conformance fixtures) into payload-budgeted packets, and the
//! depacketizer reassembles a byte-exact stream that still decodes.

use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};
use oxideav_h263::rtp::{
    depacketize_payloads, packetize_stream, parse_payload_header, PacketizeConfig,
};

/// A deterministic gradient frame on every plane.
fn gradient(lw: usize, lh: usize, seed: u8) -> YuvFrame {
    let cw = lw / 2;
    let ch = lh / 2;
    let mut y = vec![0u8; lw * lh];
    for row in 0..lh {
        for col in 0..lw {
            y[row * lw + col] = (32 + (col + row + seed as usize) % 192) as u8;
        }
    }
    let mut cb = vec![0u8; cw * ch];
    let mut cr = vec![0u8; cw * ch];
    for row in 0..ch {
        for col in 0..cw {
            cb[row * cw + col] = (80 + (col % 64)) as u8;
            cr[row * cw + col] = (100 + (row % 56)) as u8;
        }
    }
    YuvFrame {
        y,
        cb,
        cr,
        luma_width: lw,
        luma_height: lh,
    }
}

/// Packetize at the given budget, assert every invariant the RFC
/// requires, and return the reassembled stream.
fn packetize_check(stream: &[u8], max_payload: usize) -> Vec<u8> {
    let payloads = packetize_stream(
        stream,
        PacketizeConfig {
            max_payload,
            ..PacketizeConfig::default()
        },
    )
    .expect("packetize");
    assert!(!payloads.is_empty());
    for (i, p) in payloads.iter().enumerate() {
        assert!(
            p.len() <= max_payload,
            "payload {i} is {} bytes > budget {max_payload}",
            p.len()
        );
        let (header, offset) = parse_payload_header(p).expect("parse payload header");
        assert!(offset < p.len(), "payload {i} carries no bitstream data");
        if header.p {
            // §6.1 — a segment packet's stripped start code means the
            // first data byte has its MSB set ('1' of the start code
            // after 16 zeros).
            assert!(
                p[offset] & 0x80 != 0,
                "payload {i}: P=1 but data does not continue a start code"
            );
        }
        if i == 0 {
            assert!(header.p, "first packet must be a segment packet");
        }
    }
    depacketize_payloads(&payloads).expect("depacketize")
}

/// A single-picture stream fits one large packet; the round trip is
/// byte-exact and the packet is a P=1 picture packet whose payload
/// starts with the '100000' PSC tail.
#[test]
fn single_picture_single_packet() {
    // A flat sub-QCIF keyframe is a few hundred bytes — comfortably a
    // single default-budget payload.
    let src = YuvFrame::grey(128, 96);
    let stream = oxideav_h263::encoder::encode_intra_picture(&src, 7, 0).expect("encode");
    let payloads = packetize_stream(&stream, PacketizeConfig::default()).expect("packetize");
    assert_eq!(payloads.len(), 1);
    let (header, offset) = parse_payload_header(&payloads[0]).unwrap();
    assert!(header.p);
    // '100000' — the six-bit PSC tail (0x80 >> nothing: byte 0x80..0x83
    // region; PSC third byte is 0x80 | TR high bits... the third PSC
    // byte is 1000 00xx: top six bits '100000').
    assert_eq!(payloads[0][offset] & 0xFC, 0x80);
    let rebuilt = depacketize_payloads(&payloads).unwrap();
    assert_eq!(rebuilt, stream);
}

/// A multi-picture GOP stream round-trips byte-exactly across a sweep
/// of payload budgets (forcing pure picture packets, GOB-boundary
/// cuts, and arbitrary Follow-on cuts), and the reassembled stream
/// decodes to the same frames as the original.
#[test]
fn gop_stream_round_trips_at_every_budget() {
    use oxideav_h263::encoder::{encode_sequence, GopConfig};

    let frames = vec![
        gradient(176, 144, 0),
        gradient(176, 144, 12),
        gradient(176, 144, 24),
        gradient(176, 144, 36),
    ];
    let cfg = GopConfig {
        quant: 7,
        intra_period: 2,
        search_half: 4,
        umv: false,
        eos: true,
    };
    let stream = encode_sequence(&frames, &cfg, 0).expect("encode GOP");
    let reference_frames = decode_sequence(&stream, DecodeOptions::default()).expect("decode");

    for budget in [32usize, 64, 200, 512, 1440, 4096] {
        let rebuilt = packetize_check(&stream, budget);
        assert_eq!(rebuilt, stream, "budget {budget}: stream not byte-exact");
        let frames2 = decode_sequence(&rebuilt, DecodeOptions::default()).expect("decode rebuilt");
        assert_eq!(frames2.len(), reference_frames.len());
        for (a, b) in reference_frames.iter().zip(frames2.iter()) {
            assert_eq!(a.y, b.y, "budget {budget}");
        }
    }
}

/// A GOB-headered stream cuts preferentially at the byte-aligned GBSC
/// boundaries: with a budget below the picture size every packet
/// after the first within a picture is still a P=1 segment packet
/// (no Follow-ons needed at all when GOBs fit the budget).
#[test]
fn gob_stream_prefers_segment_cuts() {
    use oxideav_h263::encoder::encode_intra_picture_gobs;

    let src = gradient(176, 144, 9);
    let stream = encode_intra_picture_gobs(&src, 0, |_| 6).expect("encode GOBs");
    // QCIF at q=6: each GOB is well under 600 bytes.
    let payloads = packetize_stream(
        &stream,
        PacketizeConfig {
            max_payload: 600,
            ..PacketizeConfig::default()
        },
    )
    .expect("packetize");
    assert!(payloads.len() > 1, "expected a multi-packet split");
    for (i, p) in payloads.iter().enumerate() {
        let (header, _) = parse_payload_header(p).unwrap();
        assert!(header.p, "packet {i} should cut at a GBSC boundary");
    }
    assert_eq!(depacketize_payloads(&payloads).unwrap(), stream);
}

/// An Annex K slice-structured H.263+ stream likewise cuts at the
/// byte-aligned SSC boundaries and survives the round trip, decoding
/// with default options.
#[test]
fn slice_stream_round_trips() {
    use oxideav_h263::encoder::encode_intra_picture_slices;

    let src = gradient(176, 144, 40);
    let stream = encode_intra_picture_slices(&src, 0, 2, |_| 8).expect("encode slices");
    let payloads = packetize_stream(
        &stream,
        PacketizeConfig {
            max_payload: 500,
            ..PacketizeConfig::default()
        },
    )
    .expect("packetize");
    assert!(payloads.len() > 1);
    for p in &payloads {
        let (header, _) = parse_payload_header(p).unwrap();
        assert!(header.p, "slice cuts should all land on SSC boundaries");
    }
    let rebuilt = depacketize_payloads(&payloads).unwrap();
    assert_eq!(rebuilt, stream);
    let dec = decode_sequence(&rebuilt, DecodeOptions::default()).expect("decode");
    assert_eq!(dec.len(), 1);
}

/// The vendored conformance fixtures (streams produced by a real
/// encoder) round-trip byte-exactly through packetize + depacketize
/// at several budgets, including budgets small enough to force
/// Follow-on packets.
#[test]
fn conformance_fixtures_round_trip() {
    let fixtures: &[&[u8]] = &[
        include_bytes!("fixtures/i-frame-then-p-frame-qcif/input.h263"),
        include_bytes!("fixtures/i-only-cif-baseline/input.h263"),
        include_bytes!("fixtures/h263p-modern/input.h263"),
        include_bytes!("fixtures/slice-structured-mode/input.h263"),
        include_bytes!("fixtures/advanced-intra-coding/input.h263"),
    ];
    for (i, stream) in fixtures.iter().enumerate() {
        for budget in [48usize, 256, 1440] {
            let rebuilt = packetize_check(stream, budget);
            assert_eq!(
                &rebuilt, stream,
                "fixture {i} budget {budget}: not byte-exact"
            );
        }
    }
}

/// §6.1.2 redundant picture header attachment: every GOB-boundary
/// packet of a GOB-headered stream carries PLEN > 0 whose re-assembled
/// bytes parse into the exact same picture-layer fields as the primary
/// picture header; picture packets keep PLEN = 0; the reassembled
/// stream stays byte-exact (the redundant copies are discarded).
#[test]
fn redundant_picture_header_attachment() {
    use oxideav_core::bits::BitReader;
    use oxideav_h263::encoder::encode_intra_picture_gobs;
    use oxideav_h263::picture_header::{parse_picture_layer, H263PictureLayer};
    use oxideav_h263::plus_ptype::InheritedExtendedState;
    use oxideav_h263::rtp::assemble_picture_header;

    let src = gradient(176, 144, 17);
    let stream = encode_intra_picture_gobs(&src, 6, |_| 6).expect("encode GOBs");

    // Parse the primary picture header for reference.
    let primary = {
        let mut r = BitReader::new(&stream);
        match parse_picture_layer(&mut r, InheritedExtendedState::default()).unwrap() {
            H263PictureLayer::Baseline(h) => h,
            H263PictureLayer::Extended(_) => panic!("baseline stream expected"),
        }
    };

    let payloads = packetize_stream(
        &stream,
        PacketizeConfig {
            max_payload: 600,
            attach_picture_header: true,
        },
    )
    .expect("packetize");
    assert!(payloads.len() > 1);

    let mut saw_attached = 0;
    for (i, p) in payloads.iter().enumerate() {
        let (header, _) = parse_payload_header(p).unwrap();
        assert!(header.p);
        if i == 0 {
            // Picture packet: PLEN = 0 (§6.1.1).
            assert!(header.extra_picture_header.is_empty());
            continue;
        }
        // GOB packets: redundant header attached.
        let bytes = assemble_picture_header(&header).expect("PLEN > 0 on GOB packet");
        let mut r = BitReader::new(&bytes);
        let reparsed = match parse_picture_layer(&mut r, InheritedExtendedState::default()) {
            Ok(H263PictureLayer::Baseline(h)) => h,
            other => panic!("redundant header did not reparse: {other:?}"),
        };
        assert_eq!(reparsed, primary, "packet {i} redundant header mismatch");
        saw_attached += 1;
    }
    assert!(saw_attached > 0);

    // Reassembly discards the redundant copies: byte-exact stream.
    assert_eq!(depacketize_payloads(&payloads).unwrap(), stream);
}

/// Redundant headers also attach to H.263+ slice packets (complete
/// UFEP=001 headers), and `redundant_picture_header` computes a PEBIT
/// consistent with the actual header bit length.
#[test]
fn redundant_header_on_h263p_slices() {
    use oxideav_h263::encoder::encode_intra_picture_slices;
    use oxideav_h263::rtp::redundant_picture_header;

    let src = gradient(176, 144, 29);
    let stream = encode_intra_picture_slices(&src, 1, 2, |_| 8).expect("encode slices");

    let (bytes, pebit) = redundant_picture_header(&stream)
        .expect("parse")
        .expect("complete header must be attachable");
    assert!(!bytes.is_empty());
    assert!(pebit < 8);
    // The attached copy starts with the '100000' PSC tail.
    assert_eq!(bytes[0] & 0xFC, 0x80);

    let payloads = packetize_stream(
        &stream,
        PacketizeConfig {
            max_payload: 500,
            attach_picture_header: true,
        },
    )
    .expect("packetize");
    let mut saw_attached = 0;
    for (i, p) in payloads.iter().enumerate() {
        let (header, _) = parse_payload_header(p).unwrap();
        if i > 0 && header.p {
            assert_eq!(header.extra_picture_header, bytes, "packet {i}");
            assert_eq!(header.pebit, pebit);
            saw_attached += 1;
        }
    }
    assert!(saw_attached > 0);
    assert_eq!(depacketize_payloads(&payloads).unwrap(), stream);
}
