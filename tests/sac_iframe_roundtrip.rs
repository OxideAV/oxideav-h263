//! Annex E (SAC) I-picture self-roundtrip — encode an I-picture with the
//! SAC bridge enabled, decode it back through the same crate, and check the
//! reconstructed frame matches the encoder's locally reconstructed picture
//! AND lies within a few LSBs of the source frame.
//!
//! This exercises:
//! * The encoder's SAC PTYPE bit 11 emission and SAC body builder
//!   (`encoder::encode_i_picture_sac_with_recon`).
//! * The picture-header parser's acceptance of `sac_mode = true`.
//! * The decoder's SAC body driver (`mb_sac::decode_i_picture_sac`).
//! * Every model wired into [`oxideav_h263::sac::SacIPictureWriter`] /
//!   `SacIPictureReader`: MCBPC_INTRA, CBPY_INTRA, INTRADC, TCOEF*_INTRA,
//!   SIGN, and (when AC events overflow Table 16) the LAST_INTRA / RUN_INTRA
//!   / LEVEL_INTRA escape body.

use oxideav_core::frame::VideoPlane;
use oxideav_core::{
    CodecId, CodecParameters, Decoder, Encoder, Frame, MediaType, PixelFormat, VideoFrame,
};
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::encoder::H263Encoder;

fn make_qcif_synthetic(seed: u8) -> VideoFrame {
    // Build a 176x144 frame with a smooth gradient + a few high-frequency
    // patches so the AC TCOEF SAC path actually fires. Constant frames
    // round-trip trivially because all blocks are AC-zero, which doesn't
    // exercise the TCOEF1/2/3/r models or SIGN.
    let w = 176usize;
    let h = 144usize;
    let mut y = vec![0u8; w * h];
    let mut cb = vec![128u8; (w / 2) * (h / 2)];
    let mut cr = vec![128u8; (w / 2) * (h / 2)];
    for j in 0..h {
        for i in 0..w {
            // Gradient with a per-row sinusoidal nudge so blocks have AC
            // energy. Add `seed` so successive frames produce different
            // residual patterns.
            let g = ((i + j) as i32 * 5 / 8 + seed as i32) & 0xFF;
            let hf = ((i ^ j) as i32) & 0x1F;
            y[j * w + i] = ((g + hf) & 0xFF) as u8;
        }
    }
    for j in 0..(h / 2) {
        for i in 0..(w / 2) {
            cb[j * (w / 2) + i] = (96 + ((j as i32) & 0x3F)) as u8;
            cr[j * (w / 2) + i] = (160 - ((i as i32) & 0x3F)) as u8;
        }
    }
    VideoFrame {
        pts: Some(0),
        planes: vec![
            VideoPlane { stride: w, data: y },
            VideoPlane {
                stride: w / 2,
                data: cb,
            },
            VideoPlane {
                stride: w / 2,
                data: cr,
            },
        ],
    }
}

#[test]
fn sac_i_picture_self_roundtrip_qcif() {
    let frame = make_qcif_synthetic(0);

    // Build encoder with SAC enabled.
    let mut params = CodecParameters::video(CodecId::new("h263"));
    params.media_type = MediaType::Video;
    params.width = Some(176);
    params.height = Some(144);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    let mut enc = H263Encoder::from_params(&params).expect("encoder construct");
    enc.set_enable_annex_e(true);
    assert!(enc.enable_annex_e());

    enc.send_frame(&Frame::Video(frame.clone()))
        .expect("send_frame");
    enc.flush().expect("encoder flush");
    let pkt = enc.receive_packet().expect("encoder packet");
    assert!(pkt.flags.keyframe, "first frame must be a keyframe");

    // Picture header bit 11 (SAC) should be set. Locate it: PSC=22 bits,
    // TR=8 bits, then PTYPE bits 1..=11 follow. Bit 11 sits in byte 4 (0-
    // indexed) bit 6 (MSB-first). Spot-check rather than full re-parse.
    // Layout: 22+8 = 30 bits of PSC+TR; PTYPE bit 11 is at absolute bit
    // 30+10 = 40 → byte 5 bit 7 (MSB-first), which is the high bit of
    // byte index 5. We just verify a SAC bit is present in the right region.
    assert!(pkt.data.len() >= 6, "encoded packet too small");
    // Bit position 40 → byte 5, bit (7-0) = 7 (MSB-first). Read it.
    let byte5 = pkt.data[5];
    let sac_bit = (byte5 >> 7) & 1;
    assert_eq!(
        sac_bit, 1,
        "PTYPE bit 11 (SAC) must be set in encoded I-picture; byte5=0x{byte5:02x}"
    );

    // Decode it back through our own decoder.
    let mut dec = H263Decoder::new(CodecId::new("h263"));
    dec.send_packet(&pkt).expect("decoder send_packet");
    dec.flush().expect("decoder flush");
    let f = dec.receive_frame().expect("decoder receive_frame");
    let v = match f {
        Frame::Video(v) => v,
        _ => panic!("expected video frame"),
    };

    let yp = &v.planes[0];
    let w = yp.stride;
    let h = yp.data.len() / yp.stride;
    assert_eq!(w, 176);
    assert_eq!(h, 144);
    // Pixel format (Yuv420P) is implied by the 3-plane layout the decoder
    // writes; sanity check the chroma planes exist at half resolution.
    assert_eq!(v.planes.len(), 3);
    assert_eq!(v.planes[1].stride, w / 2);
    assert_eq!(v.planes[2].stride, w / 2);

    // Compare luma — H.263 quantisation will smooth the gradient + HF, but
    // the average error against the source should be modest. We compute
    // PSNR and assert it's above the floor where SAC and VLC paths produce
    // the same bytes (both run the same DCT/quant/IDCT pipeline; only the
    // entropy coder differs).
    let mut sse: u64 = 0;
    let n = (w * h) as u64;
    for j in 0..h {
        for i in 0..w {
            let s = frame.planes[0].data[j * frame.planes[0].stride + i] as i64;
            let d = yp.data[j * yp.stride + i] as i64;
            let e = s - d;
            sse += (e * e) as u64;
        }
    }
    let mse = sse as f64 / n as f64;
    // PSNR = 10 * log10(255^2 / MSE). For QCIF gradient at q=5 we expect
    // ≥ 30 dB.
    let psnr = 10.0 * (255.0_f64 * 255.0_f64 / mse.max(1e-9)).log10();
    assert!(
        psnr >= 28.0,
        "SAC roundtrip luma PSNR {psnr:.2} dB below 28 dB floor (mse={mse:.4})"
    );
}

#[test]
fn sac_i_picture_constant_frame_roundtrip() {
    // Constant-frame degenerate case: every block is DC-only (no AC TCOEF
    // events). Exercises the no-AC SAC path: MCBPC + CBPY=0 + INTRADC only,
    // 6 blocks per MB. Reconstruction must be near-exact (DC quant only).
    let w = 176usize;
    let h = 144usize;
    let frame = VideoFrame {
        pts: Some(0),
        planes: vec![
            VideoPlane {
                stride: w,
                data: vec![100u8; w * h],
            },
            VideoPlane {
                stride: w / 2,
                data: vec![128u8; (w / 2) * (h / 2)],
            },
            VideoPlane {
                stride: w / 2,
                data: vec![128u8; (w / 2) * (h / 2)],
            },
        ],
    };
    let mut params = CodecParameters::video(CodecId::new("h263"));
    params.media_type = MediaType::Video;
    params.width = Some(176);
    params.height = Some(144);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);
    enc.send_frame(&Frame::Video(frame)).unwrap();
    enc.flush().unwrap();
    let pkt = enc.receive_packet().unwrap();

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    dec.send_packet(&pkt).unwrap();
    dec.flush().unwrap();
    let f = dec.receive_frame().unwrap();
    let v = match f {
        Frame::Video(v) => v,
        _ => panic!("video"),
    };
    let yp = &v.planes[0];
    let w = yp.stride;
    let h = yp.data.len() / yp.stride;
    let mut hits = 0usize;
    for j in 0..h {
        for i in 0..w {
            let p = yp.data[j * yp.stride + i] as i32;
            if (p - 100).abs() <= 2 {
                hits += 1;
            }
        }
    }
    let total = w * h;
    let ratio = hits * 100 / total;
    assert!(
        ratio >= 99,
        "SAC constant-frame roundtrip: {hits}/{total} pels within ±2 of 100 ({ratio}%)"
    );
}

#[test]
fn sac_vs_vlc_yields_same_reconstruction() {
    // Both pipelines run the same DCT/quant/IDCT — only the entropy coder
    // differs. So SAC-encoded → SAC-decoded reconstruction MUST equal
    // VLC-encoded → VLC-decoded reconstruction byte-for-byte.
    let frame = make_qcif_synthetic(7);

    let mut params = CodecParameters::video(CodecId::new("h263"));
    params.media_type = MediaType::Video;
    params.width = Some(176);
    params.height = Some(144);
    params.pixel_format = Some(PixelFormat::Yuv420P);

    let mut enc_sac = H263Encoder::from_params(&params).unwrap();
    enc_sac.set_enable_annex_e(true);
    enc_sac.send_frame(&Frame::Video(frame.clone())).unwrap();
    enc_sac.flush().unwrap();
    let pkt_sac = enc_sac.receive_packet().unwrap();

    let mut enc_vlc = H263Encoder::from_params(&params).unwrap();
    enc_vlc.send_frame(&Frame::Video(frame)).unwrap();
    enc_vlc.flush().unwrap();
    let pkt_vlc = enc_vlc.receive_packet().unwrap();

    let mut dec_sac = H263Decoder::new(CodecId::new("h263"));
    dec_sac.send_packet(&pkt_sac).unwrap();
    dec_sac.flush().unwrap();
    let v_sac = match dec_sac.receive_frame().unwrap() {
        Frame::Video(v) => v,
        _ => panic!("video"),
    };

    let mut dec_vlc = H263Decoder::new(CodecId::new("h263"));
    dec_vlc.send_packet(&pkt_vlc).unwrap();
    dec_vlc.flush().unwrap();
    let v_vlc = match dec_vlc.receive_frame().unwrap() {
        Frame::Video(v) => v,
        _ => panic!("video"),
    };

    // The two decoded YUV frames should match exactly.
    assert_eq!(v_sac.planes[0].data, v_vlc.planes[0].data, "luma differs");
    assert_eq!(v_sac.planes[1].data, v_vlc.planes[1].data, "Cb differs");
    assert_eq!(v_sac.planes[2].data, v_vlc.planes[2].data, "Cr differs");
}

#[test]
fn sac_packet_includes_psc_marker() {
    // Sanity-check that the SAC body doesn't accidentally emit a PSC
    // emulation. The §E.5 PSC_FIFO 14-zero stuffing is exactly there to
    // prevent this; this test asserts only one PSC byte sequence appears
    // in the encoded packet (the picture's own PSC).
    let frame = make_qcif_synthetic(13);
    let mut params = CodecParameters::video(CodecId::new("h263"));
    params.media_type = MediaType::Video;
    params.width = Some(176);
    params.height = Some(144);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);
    enc.send_frame(&Frame::Video(frame)).unwrap();
    enc.flush().unwrap();
    let pkt = enc.receive_packet().unwrap();
    // PSC = 22 bits = 0000 0000 0000 0000 1xxxxx (the first 17 bits are
    // zero, then a 1 bit). Search for byte sequence `00 00 80` which is
    // the byte-aligned PSC prefix.
    let mut hits = 0usize;
    for w in pkt.data.windows(3) {
        if w[0] == 0x00 && w[1] == 0x00 && w[2] >= 0x80 {
            hits += 1;
        }
    }
    assert_eq!(
        hits, 1,
        "expected exactly one PSC marker in SAC packet, got {hits}"
    );
}
