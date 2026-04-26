//! Annex E (SAC) P-picture self-roundtrip — round 14.
#![allow(clippy::needless_range_loop)]
//!
//! Encodes I+P chains with the SAC bridge enabled, decodes them back, and
//! checks (a) the encoder's SAC PTYPE bit 11 is set, (b) the decoded
//! picture matches the encoder's locally reconstructed picture
//! byte-for-byte vs the VLC pipeline running on the same source frames,
//! and (c) GOB-boundary `encoder_flush` / `decoder_reset` round-trips
//! (CIF source produces 17 GOBs of one MB row each, exercising every
//! mid-picture boundary).
//!
//! Targets every model wired by `crate::sac::SacPPictureWriter` /
//! `SacPPictureReader`: cumf_COD, cumf_MCBPC_no4MVQ, cumf_CBPY,
//! cumf_DQUANT, cumf_MVD, INTER cumf_TCOEF1/2/3/r, cumf_SIGN,
//! cumf_LAST + cumf_RUN + cumf_LEVEL ESCAPE.

use oxideav_core::frame::VideoPlane;
use oxideav_core::{
    CodecId, CodecParameters, Decoder, Encoder, Frame, MediaType, PixelFormat, TimeBase, VideoFrame,
};
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::encoder::H263Encoder;

fn make_qcif_synthetic(seed: u8, dx: i32, dy: i32) -> VideoFrame {
    // Pannning + AC-energy synthetic: gradient with HF noise that shifts
    // by (dx, dy) per frame so the P-encoder's motion estimator finds
    // non-zero MVs.
    let w = 176usize;
    let h = 144usize;
    let mut y = vec![0u8; w * h];
    let mut cb = vec![128u8; (w / 2) * (h / 2)];
    let mut cr = vec![128u8; (w / 2) * (h / 2)];
    for j in 0..h {
        for i in 0..w {
            let ii = (i as i32 + dx).rem_euclid(w as i32) as usize;
            let jj = (j as i32 + dy).rem_euclid(h as i32) as usize;
            let g = ((ii + jj) as i32 * 5 / 8 + seed as i32) & 0xFF;
            let hf = ((ii ^ jj) as i32) & 0x1F;
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

fn make_cif_synthetic(seed: u8, dx: i32, dy: i32) -> VideoFrame {
    // CIF is 352x288 = 18 GOBs of 1 MB row each (18 GOB layout from §5.2).
    // Forces multiple `encoder_flush` / `decoder_reset` boundaries.
    let w = 352usize;
    let h = 288usize;
    let mut y = vec![0u8; w * h];
    let mut cb = vec![128u8; (w / 2) * (h / 2)];
    let mut cr = vec![128u8; (w / 2) * (h / 2)];
    for j in 0..h {
        for i in 0..w {
            let ii = (i as i32 + dx).rem_euclid(w as i32) as usize;
            let jj = (j as i32 + dy).rem_euclid(h as i32) as usize;
            let g = ((ii + jj) as i32 * 3 / 4 + seed as i32) & 0xFF;
            let hf = ((ii ^ jj) as i32) & 0x0F;
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

fn make_params(w: u32, h: u32) -> CodecParameters {
    let mut p = CodecParameters::video(CodecId::new("h263"));
    p.media_type = MediaType::Video;
    p.width = Some(w);
    p.height = Some(h);
    p.pixel_format = Some(PixelFormat::Yuv420P);
    p
}

#[test]
fn sac_p_picture_self_roundtrip_qcif() {
    // Encode I + 3 P frames with SAC enabled; decode through the same
    // SAC driver; check decoded YUV matches the source closely (PSNR
    // floor) and that the P-pictures actually carry the SAC PTYPE bit.
    let frames: Vec<VideoFrame> = (0..4u8)
        .map(|f| make_qcif_synthetic(f, (f as i32) * 2, 0))
        .collect();

    let params = make_params(176, 144);
    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);

    for f in &frames {
        enc.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc.flush().unwrap();

    let mut packets = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        packets.push(p);
    }
    assert_eq!(
        packets.len(),
        4,
        "expected 4 packets, got {}",
        packets.len()
    );
    assert!(packets[0].flags.keyframe, "first packet must be keyframe");
    for (i, p) in packets.iter().enumerate().skip(1) {
        assert!(!p.flags.keyframe, "packet {i} should be a P-picture");
    }

    // Verify each P-picture's PTYPE bit 11 (SAC) is set.
    for (i, p) in packets.iter().enumerate() {
        let bit_pos_sac = 30 + 10; // PSC=22, TR=8, then PTYPE bits 1..=11.
        let byte_idx = bit_pos_sac / 8;
        let bit_in_byte = 7 - (bit_pos_sac % 8);
        let v = (p.data[byte_idx] >> bit_in_byte) & 1;
        assert_eq!(v, 1, "packet {i} PTYPE bit 11 (SAC) not set");
    }

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    for p in &packets {
        dec.send_packet(p).unwrap();
    }
    dec.flush().unwrap();

    for i in 0..4 {
        let f = dec.receive_frame().unwrap();
        let v = match f {
            Frame::Video(v) => v,
            _ => panic!("video"),
        };
        let yp = &v.planes[0];
        let w = yp.stride;
        let h = yp.data.len() / yp.stride;
        assert_eq!(w, 176);
        assert_eq!(h, 144);
        let mut sse: u64 = 0;
        let n = (w * h) as u64;
        for j in 0..h {
            for ii in 0..w {
                let s = frames[i].planes[0].data[j * frames[i].planes[0].stride + ii] as i64;
                let d = yp.data[j * yp.stride + ii] as i64;
                let e = s - d;
                sse += (e * e) as u64;
            }
        }
        let mse = sse as f64 / n as f64;
        let psnr = 10.0 * (255.0_f64 * 255.0_f64 / mse.max(1e-9)).log10();
        assert!(
            psnr >= 24.0,
            "frame {i} luma PSNR {psnr:.2} dB below 24 dB floor"
        );
    }
}

#[test]
fn sac_p_picture_matches_vlc_reconstruction_qcif() {
    // The SAC and VLC encode pipelines share DCT/quant/IDCT and motion
    // estimation; only entropy coding differs. So the decoded YUV must
    // match byte-for-byte across both pipelines on the same source.
    let frames: Vec<VideoFrame> = (0..4u8)
        .map(|f| make_qcif_synthetic(f, (f as i32) * 2, 0))
        .collect();
    let params = make_params(176, 144);

    // SAC pipeline.
    let mut enc_sac = H263Encoder::from_params(&params).unwrap();
    enc_sac.set_enable_annex_e(true);
    for f in &frames {
        enc_sac.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc_sac.flush().unwrap();
    let mut sac_pkts = Vec::new();
    while let Ok(p) = enc_sac.receive_packet() {
        sac_pkts.push(p);
    }

    // VLC pipeline.
    let mut enc_vlc = H263Encoder::from_params(&params).unwrap();
    for f in &frames {
        enc_vlc.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc_vlc.flush().unwrap();
    let mut vlc_pkts = Vec::new();
    while let Ok(p) = enc_vlc.receive_packet() {
        vlc_pkts.push(p);
    }

    let mut dec_sac = H263Decoder::new(CodecId::new("h263"));
    for p in &sac_pkts {
        dec_sac.send_packet(p).unwrap();
    }
    dec_sac.flush().unwrap();
    let mut dec_vlc = H263Decoder::new(CodecId::new("h263"));
    for p in &vlc_pkts {
        dec_vlc.send_packet(p).unwrap();
    }
    dec_vlc.flush().unwrap();

    for fi in 0..4 {
        let s = match dec_sac.receive_frame().unwrap() {
            Frame::Video(v) => v,
            _ => panic!("video"),
        };
        let v = match dec_vlc.receive_frame().unwrap() {
            Frame::Video(v) => v,
            _ => panic!("video"),
        };
        assert_eq!(
            s.planes[0].data, v.planes[0].data,
            "frame {fi} luma differs"
        );
        assert_eq!(s.planes[1].data, v.planes[1].data, "frame {fi} Cb differs");
        assert_eq!(s.planes[2].data, v.planes[2].data, "frame {fi} Cr differs");
    }
}

#[test]
fn sac_p_picture_gob_resync_cif() {
    // CIF: 352x288 = 18 MB rows = 18 GOBs (one MB row each per Table A.1).
    // Drive the SAC P-encoder via `encode_p_picture_sac_with_recon_opts`
    // with `emit_gob_headers = true` so every row boundary fires the
    // `encoder_flush` + GOB header + fresh SAC segment path. The decoder
    // mirrors with a `decoder_reset` at each GBSC.
    use oxideav_core::Packet;
    use oxideav_h263::encoder::{
        encode_i_picture_sac_with_recon, encode_p_picture_sac_with_recon_opts,
    };
    use oxideav_h263::picture::SourceFormat;

    let frames: Vec<VideoFrame> = (0..3u8)
        .map(|f| make_cif_synthetic(f, (f as i32) * 4, 0))
        .collect();

    // Encode I + 2 P (with GOB resync) by hand — bypasses `H263Encoder`
    // so we can pass the GOB-resync flag.
    let mut packets: Vec<Packet> = Vec::new();
    let (i_bytes, mut recon) =
        encode_i_picture_sac_with_recon(352, 288, SourceFormat::Cif, 8, 0, &frames[0])
            .expect("encode I");
    packets.push(Packet::new(0, TimeBase::new(1, 30), i_bytes));
    for (idx, f) in frames.iter().enumerate().skip(1) {
        let (p_bytes, p_recon) = encode_p_picture_sac_with_recon_opts(
            352,
            288,
            SourceFormat::Cif,
            8,
            idx as u8,
            f,
            &recon,
            true,  // emit_gob_headers
            false, // enable_annex_j
        )
        .expect("encode P");
        packets.push(Packet::new(0, TimeBase::new(1, 30), p_bytes));
        recon = p_recon;
    }

    // Sanity: each P-picture packet must carry GBSCs at expected
    // positions (we just check there's at least one extra start code in
    // each P packet).
    use oxideav_h263::start_code::iter_start_codes;
    for (i, p) in packets.iter().enumerate().skip(1) {
        let scs: Vec<_> = iter_start_codes(&p.data).collect();
        // PSC + ≥ 1 GBSC.
        assert!(
            scs.len() >= 2,
            "P packet {i} should carry at least one GOB header (got {} start codes)",
            scs.len()
        );
    }

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    for p in &packets {
        dec.send_packet(p).unwrap();
    }
    dec.flush().unwrap();

    for i in 0..3 {
        let f = dec.receive_frame().unwrap();
        let v = match f {
            Frame::Video(v) => v,
            _ => panic!("video"),
        };
        let yp = &v.planes[0];
        let w = yp.stride;
        let h = yp.data.len() / yp.stride;
        assert_eq!(w, 352);
        assert_eq!(h, 288);
        let mut sse: u64 = 0;
        let n = (w * h) as u64;
        for j in 0..h {
            for ii in 0..w {
                let s = frames[i].planes[0].data[j * frames[i].planes[0].stride + ii] as i64;
                let d = yp.data[j * yp.stride + ii] as i64;
                let e = s - d;
                sse += (e * e) as u64;
            }
        }
        let mse = sse as f64 / n as f64;
        let psnr = 10.0 * (255.0_f64 * 255.0_f64 / mse.max(1e-9)).log10();
        assert!(
            psnr >= 24.0,
            "CIF SAC frame {i} luma PSNR {psnr:.2} dB below 24 dB"
        );
    }
}

#[test]
fn sac_p_picture_constant_frame_uses_skip_path() {
    // Constant frame means every P-MB should be skipped (COD=1) — the
    // SAC stream is mostly cumf_COD index-1 symbols. Round-trip must be
    // bit-exact (skipped MB just copies the predictor).
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
    let params = make_params(176, 144);
    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);

    // I + 2 P, all identical → P frames should be ~ all-skipped.
    for _ in 0..3 {
        enc.send_frame(&Frame::Video(frame.clone())).unwrap();
    }
    enc.flush().unwrap();
    let mut pkts = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        pkts.push(p);
    }

    // P packets should be tiny relative to the I packet (skipped MBs
    // emit ~1 SAC bit each).
    for (i, p) in pkts.iter().enumerate().skip(1) {
        assert!(
            p.data.len() < pkts[0].data.len() / 2,
            "skipped-only P packet {i} not significantly smaller than I (P={}, I={})",
            p.data.len(),
            pkts[0].data.len()
        );
    }

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    for p in &pkts {
        dec.send_packet(p).unwrap();
    }
    dec.flush().unwrap();
    for _ in 0..3 {
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
            for ii in 0..w {
                if (yp.data[j * yp.stride + ii] as i32 - 100).abs() <= 2 {
                    hits += 1;
                }
            }
        }
        let total = w * h;
        assert!(
            hits * 100 / total >= 99,
            "constant-frame SAC roundtrip too many off pels: {hits}/{total}"
        );
    }
}
