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

    // §7 — every Picture Start Code must begin its own packet: the
    // number of P=1 packets whose (stripped) start code is a PSC must
    // equal the number of byte-aligned PSCs in the stream.
    let stream_pscs = (0..stream.len().saturating_sub(2))
        .filter(|&i| stream[i] == 0 && stream[i + 1] == 0 && stream[i + 2] & 0xFC == 0x80)
        .count();
    let packet_pscs = payloads
        .iter()
        .filter(|p| {
            let (header, offset) = parse_payload_header(p).unwrap();
            header.p && p[offset] & 0xFC == 0x80
        })
        .count();
    assert_eq!(
        packet_pscs, stream_pscs,
        "every PSC must begin a picture segment packet"
    );

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
        ..GopConfig::default()
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
        include_bytes!("fixtures/unrestricted-mv-mode/input.h263"),
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

/// RFC 2190 Mode A: a baseline GOB-headered I + P stream packetizes
/// at GOB boundaries with full start codes, the per-picture
/// SRC/I/U/S/A fields mirror each picture's PTYPE, and reassembly is
/// byte-exact with decoded-frame equality.
#[test]
fn rfc2190_mode_a_round_trips() {
    use oxideav_h263::encoder::{encode_inter_picture_gobs, encode_intra_picture_gobs};
    use oxideav_h263::picture::decode_picture_no_gob0_header;
    use oxideav_h263::rtp::{
        depacketize_payloads_rfc2190, packetize_stream_rfc2190, parse_rfc2190_mode_a,
    };

    let f0 = gradient(176, 144, 2);
    let f1 = gradient(176, 144, 7);
    let i_bytes = encode_intra_picture_gobs(&f0, 4, |_| 6).expect("I");
    let anchor =
        decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).expect("dec I");
    let p_bytes = encode_inter_picture_gobs(&f1, &anchor, 6, 5, 4).expect("P");
    let mut stream = i_bytes.clone();
    stream.extend_from_slice(&p_bytes);

    let payloads = packetize_stream_rfc2190(
        &stream,
        PacketizeConfig {
            max_payload: 512,
            ..PacketizeConfig::default()
        },
    )
    .expect("packetize 2190");
    assert!(payloads.len() > 1);

    let mut saw_intra = false;
    let mut saw_inter = false;
    for p in &payloads {
        assert!(p.len() <= 512);
        let (h, offset) = parse_rfc2190_mode_a(p).expect("mode A header");
        // Mode A carries the full start code: 16 zero bits first.
        assert_eq!(&p[offset..offset + 2], &[0, 0]);
        assert_eq!(h.sbit, 0);
        assert_eq!(h.ebit, 0);
        assert_eq!(h.src, 0b010, "QCIF SRC code");
        assert!(!h.pb_frames);
        assert!(!h.umv && !h.sac && !h.advanced_prediction);
        assert_eq!((h.dbq, h.trb, h.tr), (0, 0, 0));
        saw_intra |= !h.inter;
        saw_inter |= h.inter;
    }
    assert!(saw_intra && saw_inter);

    let rebuilt = depacketize_payloads_rfc2190(&payloads).expect("depacketize 2190");
    assert_eq!(rebuilt, stream);
}

/// RFC 2190 Mode A PB-frames: the DBQ / TRB / TR header fields mirror
/// the §5.1.22 / §5.1.23 picture-header fields of a PB-picture; an
/// oversized PB picture fragments at macroblock boundaries with
/// **Mode C** headers carrying the same PB fields, and reassembles
/// byte-exactly.
#[test]
fn rfc2190_mode_a_pb_fields_and_mode_c_fragmentation() {
    use oxideav_h263::encoder::{encode_intra_picture, encode_pb_picture, PbConfig};
    use oxideav_h263::picture::decode_picture_no_gob0_header;
    use oxideav_h263::rtp::{
        depacketize_payloads_rfc2190, packetize_stream_rfc2190, parse_rfc2190_mode_a,
        parse_rfc2190_mode_c,
    };

    let f0 = gradient(128, 96, 0);
    let f1 = gradient(128, 96, 4);
    let f2 = gradient(128, 96, 8);
    let i_bytes = encode_intra_picture(&f0, 6, 0).expect("I");
    let anchor =
        decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).expect("dec I");
    let pb = encode_pb_picture(
        &f2,
        &f1,
        &anchor,
        /* tr_p */ 8,
        /* prev_tr */ 0,
        &PbConfig {
            quant: 6,
            dbquant: 1,
            trb: 4,
            search_half: 4,
        },
    )
    .expect("PB");

    let payloads = packetize_stream_rfc2190(
        &pb,
        PacketizeConfig {
            max_payload: 65_000,
            ..PacketizeConfig::default()
        },
    )
    .expect("packetize PB");
    assert_eq!(payloads.len(), 1);
    let (h, _) = parse_rfc2190_mode_a(&payloads[0]).unwrap();
    assert!(h.pb_frames && h.inter);
    assert_eq!(h.dbq, 1);
    assert_eq!(h.trb, 4);
    assert_eq!(h.tr, 8);
    assert_eq!(depacketize_payloads_rfc2190(&payloads).unwrap(), pb);

    // A single-segment PB picture larger than the budget fragments at
    // macroblock boundaries: the first packet starts at the PSC (Mode
    // A), every later one at a macroblock boundary (Mode C, F=1 P=1,
    // with the PB DBQ/TRB/TR fields mirrored). Bit-level reassembly is
    // byte-exact.
    let payloads = packetize_stream_rfc2190(
        &pb,
        PacketizeConfig {
            max_payload: 64,
            ..PacketizeConfig::default()
        },
    )
    .expect("packetize PB fragments");
    assert!(payloads.len() > 1);
    let mut saw_mode_c = 0usize;
    for (i, p) in payloads.iter().enumerate() {
        assert!(p.len() <= 64, "payload {i} is {} bytes", p.len());
        if p[0] & 0x80 != 0 {
            let (c, _) = parse_rfc2190_mode_c(p).expect("mode C header");
            assert!(c.b.inter);
            assert_eq!((c.dbq, c.trb, c.tr), (1, 4, 8));
            assert!(c.b.quant >= 1 && c.b.quant <= 31);
            saw_mode_c += 1;
        } else {
            assert_eq!(i, 0, "only the PSC packet may be Mode A here");
        }
    }
    assert!(saw_mode_c > 0, "expected Mode C fragments");
    assert_eq!(depacketize_payloads_rfc2190(&payloads).unwrap(), pb);

    // H.263+ (PLUSPTYPE) streams are refused on the legacy format.
    let plus = oxideav_h263::encoder::encode_intra_picture_plus(&f0, 6, 0).expect("plus I");
    assert!(matches!(
        packetize_stream_rfc2190(&plus, PacketizeConfig::default()),
        Err(oxideav_h263::Error::NotImplemented)
    ));
}

/// RFC 2190 Mode B: an oversized baseline I + P stream (single-segment
/// pictures — no GOB headers on the wire) fragments at macroblock
/// boundaries. Every non-first packet of a picture carries the Mode B
/// resumption side channel — GOBN / MBA / QUANT and the §6.1.1
/// motion-vector predictors — which is cross-checked against the
/// crate's own `enumerate_mb_boundaries` ground truth; reassembly is
/// bit-exact and the rebuilt stream decodes to the same frames.
#[test]
fn rfc2190_mode_b_fragments_round_trip() {
    use oxideav_h263::encoder::{encode_inter_picture_motion, encode_intra_picture};
    use oxideav_h263::picture::{decode_picture_no_gob0_header, enumerate_mb_boundaries};
    use oxideav_h263::rtp::{
        depacketize_payloads_rfc2190, packetize_stream_rfc2190, parse_rfc2190_mode_b,
    };

    let f0 = gradient(176, 144, 2);
    let i_bytes = encode_intra_picture(&f0, 4, 0).expect("I");
    let anchor =
        decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).expect("dec I");
    let mut f1 = anchor.clone();
    for row in 0..144 {
        for col in 0..176 {
            let sc = (col + 4).min(175);
            f1.y[row * 176 + col] = anchor.y[row * 176 + sc];
        }
    }
    let p_bytes = encode_inter_picture_motion(&f1, &anchor, 5, 1, 6).expect("P");
    let mut stream = i_bytes.clone();
    stream.extend_from_slice(&p_bytes);

    // Ground truth for the P-picture's macroblock side channel.
    let p_bounds = enumerate_mb_boundaries(&p_bytes).expect("bounds");

    // The busiest single INTRA macroblock of this content spans ~159
    // bytes, so budgets from 192 exercise Mode B fragmentation while
    // always fitting at least one macroblock per packet.
    for &budget in &[192usize, 256, 512] {
        let payloads = packetize_stream_rfc2190(
            &stream,
            PacketizeConfig {
                max_payload: budget,
                ..PacketizeConfig::default()
            },
        )
        .expect("packetize");
        assert!(payloads.len() > 2);

        let mut saw_mode_b = 0usize;
        for p in &payloads {
            assert!(p.len() <= budget);
            if p[0] & 0x80 == 0 {
                continue; // Mode A (PSC packet)
            }
            let (b, _) = parse_rfc2190_mode_b(p).expect("mode B header");
            assert!(!b.sac && !b.advanced_prediction);
            assert_eq!(b.src, 0b010, "QCIF");
            assert!(b.quant >= 1 && b.quant <= 31);
            // The (GOBN, MBA, QUANT, predictor) tuple must appear in
            // the ground-truth side channel of one of the pictures.
            let matches_truth = p_bounds.iter().any(|inf| {
                inf.gobn == b.gobn
                    && inf.mba_in_gob == b.mba
                    && inf.quant == b.quant
                    && inf.pred1 == (b.hmv1 as i16, b.vmv1 as i16)
            });
            let i_bounds = enumerate_mb_boundaries(&i_bytes).expect("bounds I");
            let matches_truth_i = i_bounds
                .iter()
                .any(|inf| inf.gobn == b.gobn && inf.mba_in_gob == b.mba && inf.quant == b.quant);
            assert!(
                matches_truth || matches_truth_i,
                "Mode B header {b:?} not found in side channel"
            );
            saw_mode_b += 1;
        }
        assert!(saw_mode_b > 0, "budget {budget}: expected Mode B packets");

        // Bit-exact reassembly …
        let rebuilt = depacketize_payloads_rfc2190(&payloads).expect("depacketize");
        assert_eq!(rebuilt, stream, "budget {budget}");
        // … and the rebuilt stream decodes to the original frames.
        let frames =
            oxideav_h263::picture::decode_sequence(&rebuilt, DecodeOptions::default()).unwrap();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].y, anchor.y);
    }
}

/// RFC 2190 Mode B / C headers are exact wire inverses, including
/// negative motion-vector predictors (7-bit two's complement).
#[test]
fn rfc2190_mode_b_c_headers_round_trip() {
    use oxideav_h263::rtp::{
        parse_rfc2190_mode_b, parse_rfc2190_mode_c, write_rfc2190_mode_b, write_rfc2190_mode_c,
        Rfc2190ModeB, Rfc2190ModeC,
    };

    let b = Rfc2190ModeB {
        sbit: 5,
        ebit: 2,
        src: 0b011,
        quant: 17,
        gobn: 8,
        mba: 313,
        inter: true,
        umv: true,
        sac: false,
        advanced_prediction: false,
        hmv1: -33,
        vmv1: 63,
        hmv2: -64,
        vmv2: 0,
    };
    let mut bytes = Vec::new();
    write_rfc2190_mode_b(&mut bytes, &b).unwrap();
    bytes.extend_from_slice(&[0xAB, 0xCD]);
    let (parsed, offset) = parse_rfc2190_mode_b(&bytes).unwrap();
    assert_eq!(parsed, b);
    assert_eq!(&bytes[offset..], &[0xAB, 0xCD]);

    let c = Rfc2190ModeC {
        b,
        dbq: 2,
        trb: 5,
        tr: 200,
    };
    let mut bytes = Vec::new();
    write_rfc2190_mode_c(&mut bytes, &c).unwrap();
    bytes.extend_from_slice(&[0x11]);
    let (parsed, offset) = parse_rfc2190_mode_c(&bytes).unwrap();
    assert_eq!(parsed, c);
    assert_eq!(&bytes[offset..], &[0x11]);

    // Mode dispatch: a Mode B parser refuses a Mode C header and vice
    // versa.
    let mut cb = Vec::new();
    write_rfc2190_mode_c(&mut cb, &c).unwrap();
    assert!(parse_rfc2190_mode_b(&cb).is_err());
    let mut bb = Vec::new();
    write_rfc2190_mode_b(&mut bb, &b).unwrap();
    assert!(parse_rfc2190_mode_c(&bb).is_err());
}
