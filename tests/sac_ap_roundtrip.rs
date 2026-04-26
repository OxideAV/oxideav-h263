//! Annex E (SAC) + Annex F (Advanced Prediction / 4MV / OBMC) round-15
//! roundtrip tests.
#![allow(clippy::needless_range_loop)]
//!
//! Exercises the new combined SAC+AP path:
//!   * encoder sets PTYPE bits 11 (SAC) AND 12 (AP)
//!   * MB layer uses `cumf_MCBPC_4MVQ` (§E.8) with the Inter4MV row
//!     (indices 16..=19) when 4MV is selected
//!   * four MVDs per 4MV macroblock (§F.2 per-block redefined predictor)
//!   * §F.3 OBMC blending applied to local reconstruction so the
//!     decoder's pass-2 OBMC produces the same picture
//!
//! ffmpeg rejects SAC streams with "H.263 SAC not supported", so the
//! verification is internal: SAC encoder → SAC decoder roundtrip with a
//! PSNR floor + match against the encoder's locally reconstructed
//! picture.

use oxideav_core::frame::VideoPlane;
use oxideav_core::{
    CodecId, CodecParameters, Decoder, Encoder, Frame, MediaType, PixelFormat, TimeBase, VideoFrame,
};
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::encoder::H263Encoder;

fn make_params(w: u32, h: u32) -> CodecParameters {
    let mut p = CodecParameters::video(CodecId::new("h263"));
    p.media_type = MediaType::Video;
    p.width = Some(w);
    p.height = Some(h);
    p.pixel_format = Some(PixelFormat::Yuv420P);
    p
}

/// Synthetic QCIF frame with a panning gradient + AC noise so the motion
/// estimator finds non-zero MVs and the 4-MV decision can fire on some
/// macroblocks.
fn make_qcif_panning(seed: u8, dx: i32, dy: i32) -> VideoFrame {
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

#[test]
fn sac_ap_picture_sets_both_ptype_bits() {
    let frames: Vec<VideoFrame> = (0..3u8)
        .map(|f| make_qcif_panning(f, (f as i32) * 2, 0))
        .collect();
    let params = make_params(176, 144);

    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);
    enc.set_enable_annex_f(true);

    for f in &frames {
        enc.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc.flush().unwrap();

    let mut packets = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        packets.push(p);
    }
    assert_eq!(packets.len(), 3, "expected 3 packets");
    assert!(packets[0].flags.keyframe, "first packet must be I");

    // P-pictures must carry PTYPE bit 11 (SAC) AND bit 12 (AP).
    for (i, p) in packets.iter().enumerate().skip(1) {
        let bit_pos_sac = 30 + 10; // PSC=22, TR=8, then PTYPE bits 1..=11.
        let bit_pos_ap = bit_pos_sac + 1;
        let sac = (p.data[bit_pos_sac / 8] >> (7 - (bit_pos_sac % 8))) & 1;
        let ap = (p.data[bit_pos_ap / 8] >> (7 - (bit_pos_ap % 8))) & 1;
        assert_eq!(sac, 1, "P packet {i} PTYPE bit 11 (SAC) not set");
        assert_eq!(ap, 1, "P packet {i} PTYPE bit 12 (AP) not set");
    }
}

#[test]
fn sac_ap_picture_self_roundtrip_qcif() {
    // Encode I + 3 P frames with SAC + AP enabled; decode through the same
    // SAC+AP driver; check decoded YUV matches the source closely (PSNR
    // floor) and the internal MC reference cycle is consistent across
    // frames.
    let frames: Vec<VideoFrame> = (0..4u8)
        .map(|f| make_qcif_panning(f, (f as i32) * 2, 0))
        .collect();
    let params = make_params(176, 144);

    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);
    enc.set_enable_annex_f(true);

    for f in &frames {
        enc.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc.flush().unwrap();

    let mut packets = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        packets.push(p);
    }
    assert_eq!(packets.len(), 4);

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
        // Width/height now live on the stream's CodecParameters; derive them
        // from the luma plane (decoder writes stride == width).
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
            psnr >= 22.0,
            "frame {i} luma PSNR {psnr:.2} dB below 22 dB floor (SAC+AP)"
        );
    }
}

#[test]
fn sac_ap_matches_vlc_ap_reconstruction_qcif() {
    // SAC + AP and VLC + AP share the same DCT, quant, IDCT, motion
    // estimation, AND OBMC reconstruction — only entropy coding differs.
    // The decoded YUV must therefore be byte-for-byte identical.
    let frames: Vec<VideoFrame> = (0..4u8)
        .map(|f| make_qcif_panning(f, (f as i32) * 2, 0))
        .collect();
    let params = make_params(176, 144);

    // SAC + AP pipeline.
    let mut enc_sac = H263Encoder::from_params(&params).unwrap();
    enc_sac.set_enable_annex_e(true);
    enc_sac.set_enable_annex_f(true);
    for f in &frames {
        enc_sac.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc_sac.flush().unwrap();
    let mut sac_pkts = Vec::new();
    while let Ok(p) = enc_sac.receive_packet() {
        sac_pkts.push(p);
    }

    // VLC + AP pipeline.
    let mut enc_vlc = H263Encoder::from_params(&params).unwrap();
    enc_vlc.set_enable_annex_f(true);
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
            "frame {fi} luma differs between SAC+AP and VLC+AP"
        );
        assert_eq!(
            s.planes[1].data, v.planes[1].data,
            "frame {fi} Cb differs between SAC+AP and VLC+AP"
        );
        assert_eq!(
            s.planes[2].data, v.planes[2].data,
            "frame {fi} Cr differs between SAC+AP and VLC+AP"
        );
    }
}

/// Drive the SAC+AP encoder directly to assert that the encoder's local
/// reconstruction matches the decoder's output bit-for-bit (i.e. the
/// encoder's pass-3 OBMC math is identical to what the decoder runs).
#[test]
fn sac_ap_local_recon_matches_decoded_output() {
    use oxideav_core::Packet;
    use oxideav_h263::encoder::{
        encode_i_picture_sac_with_recon, encode_p_picture_sac_ap_with_recon,
    };
    use oxideav_h263::picture::SourceFormat;

    let frames: Vec<VideoFrame> = (0..3u8)
        .map(|f| make_qcif_panning(f, (f as i32) * 2, 0))
        .collect();

    // Encode I, then 2 P frames against the previous recon. We snapshot
    // each recon's pel planes as `ReconSnap` to compare with the decoder
    // later (no need to keep `IPicture` around — it doesn't impl Clone,
    // and we only need flat pels for the equality check).
    struct ReconSnap {
        y: Vec<u8>,
        cb: Vec<u8>,
        cr: Vec<u8>,
        y_stride: usize,
        c_stride: usize,
    }
    let mut packets: Vec<Packet> = Vec::new();
    let mut snapshots: Vec<ReconSnap> = Vec::new();

    let (i_bytes, mut prev) =
        encode_i_picture_sac_with_recon(176, 144, SourceFormat::Qcif, 8, 0, &frames[0])
            .expect("encode I");
    packets.push(Packet::new(0, TimeBase::new(1, 30), i_bytes));
    snapshots.push(ReconSnap {
        y: prev.y.clone(),
        cb: prev.cb.clone(),
        cr: prev.cr.clone(),
        y_stride: prev.y_stride,
        c_stride: prev.c_stride,
    });
    for (idx, f) in frames.iter().enumerate().skip(1) {
        let (p_bytes, p_recon) = encode_p_picture_sac_ap_with_recon(
            176,
            144,
            SourceFormat::Qcif,
            8,
            idx as u8,
            f,
            &prev,
        )
        .expect("encode P SAC+AP");
        packets.push(Packet::new(0, TimeBase::new(1, 30), p_bytes));
        snapshots.push(ReconSnap {
            y: p_recon.y.clone(),
            cb: p_recon.cb.clone(),
            cr: p_recon.cr.clone(),
            y_stride: p_recon.y_stride,
            c_stride: p_recon.c_stride,
        });
        prev = p_recon;
    }

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    for p in &packets {
        dec.send_packet(p).unwrap();
    }
    dec.flush().unwrap();

    for fi in 0..3 {
        let v = match dec.receive_frame().unwrap() {
            Frame::Video(v) => v,
            _ => panic!("video"),
        };
        let snap = &snapshots[fi];
        // Compare luma row-by-row.
        for j in 0..144usize {
            let dec_row = &v.planes[0].data[j * v.planes[0].stride..j * v.planes[0].stride + 176];
            let rec_row = &snap.y[j * snap.y_stride..j * snap.y_stride + 176];
            assert_eq!(
                dec_row, rec_row,
                "frame {fi} row {j} luma differs (encoder recon vs decoder output)"
            );
        }
        for j in 0..72usize {
            let cw = 88usize;
            let dec_cb = &v.planes[1].data[j * v.planes[1].stride..j * v.planes[1].stride + cw];
            let rec_cb = &snap.cb[j * snap.c_stride..j * snap.c_stride + cw];
            assert_eq!(dec_cb, rec_cb, "frame {fi} row {j} Cb differs");
            let dec_cr = &v.planes[2].data[j * v.planes[2].stride..j * v.planes[2].stride + cw];
            let rec_cr = &snap.cr[j * snap.c_stride..j * snap.c_stride + cw];
            assert_eq!(dec_cr, rec_cr, "frame {fi} row {j} Cr differs");
        }
    }
}
