//! DoS-limits fuzz fixtures for `H263Decoder`.
//!
//! Two scenarios:
//!
//! 1. **Picture-header pixel-cap fixture** — a hand-built minimal sub-QCIF
//!    I-picture (128×96, the smallest standard H.263 source format) is fed
//!    to a decoder constructed with
//!    `DecoderLimits::with_max_pixels_per_frame(100)` (i.e. cap below the
//!    actual picture's pixel count). The decoder must reject the picture
//!    with [`Error::ResourceExhausted`] **without** allocating an
//!    `IPicture`. The same fixture decoded under default limits succeeds.
//!
//! 2. **Arena-pool exhaustion fixture** — the decoder's
//!    [`H263Decoder::arena_pool`] is leased N+1 times (N =
//!    `DecoderLimits::max_arenas_in_flight`); the (N+1)th `lease()` must
//!    return [`Error::ResourceExhausted`]. This proves the pool-size cap
//!    is wired through `make_decoder`.
//!
//! The picture bytes match the canonical ffmpeg-emitted PSC + PTYPE +
//! PQUANT bit layout — see the inline comment for each bit's source.

use oxideav_core::packet::PacketFlags;
use oxideav_core::Decoder;
use oxideav_core::{CodecId, CodecParameters, DecoderLimits, Error, Frame, Packet, TimeBase};
use oxideav_h263::decoder::{H263Decoder, DEFAULT_H263_ARENA_BYTES};

/// Build the canonical sub-QCIF I-picture header (no MB body) used by
/// `picture::tests::minimal_subqcif_iframe`. The header alone is enough
/// to exercise the dimension-check path; the absence of MBs means a
/// successful decode-attempt would fail later in MB parse, but our DoS
/// check fires *before* that.
///
/// Bit stream (50 bits, padded with zeros to byte boundary):
///   PSC(22)     = 0000 0000 0000 0000 1 00000
///   TR(8)       = 00000000
///   PTYPE(13)   = 1 0 0 0 0 001 0 0 0 0 0
///   PQUANT(5)   = 00101 (=5)
///   CPM(1)      = 0
///   PEI(1)      = 0
fn minimal_subqcif_iframe_header() -> Vec<u8> {
    vec![0x00, 0x00, 0x80, 0x02, 0x04, 0x05, 0x20]
}

/// Construct a `Packet` carrying the supplied elementary-stream bytes
/// as a keyframe at PTS 0.
fn keyframe_packet(es: Vec<u8>) -> Packet {
    Packet {
        stream_index: 0,
        data: es,
        pts: Some(0),
        dts: Some(0),
        duration: None,
        time_base: TimeBase::new(1, 90_000),
        flags: PacketFlags {
            keyframe: true,
            ..PacketFlags::default()
        },
    }
}

#[test]
fn picture_header_pixel_cap_rejects_oversize_dimensions() {
    // Cap below 128*96 = 12288 pixels. The DoS check reads the just-parsed
    // PictureHeader.{width,height} and compares against this cap.
    let limits = DecoderLimits::default().with_max_pixels_per_frame(100);
    let params =
        CodecParameters::video(CodecId::new(oxideav_h263::CODEC_ID_STR)).with_limits(limits);
    let mut decoder = H263Decoder::with_limits(params.codec_id.clone(), *params.limits());

    let pkt = keyframe_packet(minimal_subqcif_iframe_header());
    // send_packet on its own buffers data until a trailing PSC arrives; the
    // single-picture fixture has no trailing PSC so the decode body is run
    // by flush() (which sets the EOF flag and drains the buffer).
    decoder
        .send_packet(&pkt)
        .expect("send_packet should buffer");
    let r = decoder.flush();
    match r {
        Err(Error::ResourceExhausted(msg)) => {
            assert!(
                msg.contains("128") && msg.contains("96"),
                "diag should name the actual dims, got: {msg}"
            );
        }
        other => panic!("expected ResourceExhausted, got {other:?}"),
    }
}

#[test]
fn picture_header_pixel_cap_passes_under_default_limits() {
    // Under default limits the same picture-header dimension check passes.
    // The decode then fails on MB body parse (no MBs are present), which is
    // a different error variant — we just check it's NOT ResourceExhausted.
    let mut decoder = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let pkt = keyframe_packet(minimal_subqcif_iframe_header());
    decoder.send_packet(&pkt).expect("buffer");
    let r = decoder.flush();
    // Any other outcome (MB parse failure on the empty body, Ok, …) is
    // fine; only ResourceExhausted on the pixel cap would be a regression.
    if let Err(Error::ResourceExhausted(msg)) = r {
        panic!("default limits incorrectly rejected sub-QCIF on pixel cap: {msg}");
    }
}

#[test]
fn arena_pool_exhaustion_returns_resource_exhausted() {
    // Tighten the pool to 2 arenas so the test proves the cap is plumbed.
    let limits = DecoderLimits::default().with_max_arenas_in_flight(2);
    let decoder = H263Decoder::with_limits(CodecId::new(oxideav_h263::CODEC_ID_STR), limits);
    let pool = decoder.arena_pool().clone();

    let _a = pool.lease().expect("first lease");
    let _b = pool.lease().expect("second lease");
    // Third lease must fail — the cap is two.
    let third = pool.lease();
    match third {
        Err(Error::ResourceExhausted(_)) => {}
        Err(other) => panic!("expected ResourceExhausted on 3rd lease, got {other:?}"),
        Ok(_) => panic!("expected ResourceExhausted on 3rd lease, got Ok(_)"),
    }
}

#[test]
fn arena_pool_cap_per_arena_is_bounded_by_h263_default() {
    // A decoder constructed with the workspace default DecoderLimits must
    // size each arena to no more than DEFAULT_H263_ARENA_BYTES. This guards
    // against a regression where the default 1 GiB
    // max_alloc_bytes_per_frame leaked through and ate 8 GiB of address
    // space across the default 8-slot pool.
    let decoder = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let pool = decoder.arena_pool();
    assert!(
        pool.cap_per_arena() as u64 <= DEFAULT_H263_ARENA_BYTES,
        "default per-arena cap = {} exceeds h263 ceiling {}",
        pool.cap_per_arena(),
        DEFAULT_H263_ARENA_BYTES
    );
    assert_eq!(
        pool.max_arenas(),
        DecoderLimits::default().max_arenas_in_flight as usize
    );
}

#[test]
fn make_decoder_factory_honours_codec_parameters_limits() {
    // The registry-facing `make_decoder` factory must read
    // `params.limits()` so server callers that pass a tightened
    // `CodecParameters` actually get a tightened decoder.
    let limits = DecoderLimits::default()
        .with_max_pixels_per_frame(100)
        .with_max_arenas_in_flight(1);
    let params =
        CodecParameters::video(CodecId::new(oxideav_h263::CODEC_ID_STR)).with_limits(limits);
    let mut decoder = oxideav_h263::decoder::make_decoder(&params).expect("factory");

    let pkt = keyframe_packet(minimal_subqcif_iframe_header());
    decoder.send_packet(&pkt).expect("send_packet buffers");
    match decoder.flush() {
        Err(Error::ResourceExhausted(_)) => {} // expected
        other => panic!("factory-produced decoder ignored limits: got {other:?}"),
    }

    // A receive_frame on the failed decoder yields NeedMore (no frames
    // were enqueued) — confirms the rejection happened *before* any
    // IPicture allocation made it into ready_frames.
    let r = decoder.receive_frame();
    assert!(
        matches!(r, Err(Error::NeedMore) | Err(Error::Eof)),
        "expected NeedMore/Eof after rejected packet, got {r:?}"
    );
    // Belt-and-braces: an actually-emitted frame would be Frame::Video.
    if let Ok(Frame::Video(_)) = r {
        panic!("decoder emitted a frame after a rejected oversize packet");
    }
}
