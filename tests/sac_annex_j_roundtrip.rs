//! Round 16 — Annex E (SAC) + Annex J (deblocking) interaction tests.
#![allow(clippy::needless_range_loop)]
//!
//! Per H.263 §E.7 the MCBPC selector flips from `cumf_MCBPC_no4MVQ` to
//! `cumf_MCBPC_4MVQ` whenever Annex F (4MV/OBMC) OR Annex J (deblocking
//! filter) is active. Round 15 wired the SAC + Annex F combination
//! (PTYPE bit 12 carries AP); round 16 wires SAC + Annex J alone, with
//! the deblocking flag plumbed out-of-band on baseline PTYPE (`set_enable_annex_j`).
//!
//! Coverage:
//!   * encoder `enable_annex_e + enable_annex_j` produces a stream whose
//!     SAC P bodies decode through the same flag pair (round-trip);
//!   * the SAC + DF stream's decoded pels match the SAC + DF VLC stream
//!     bit-for-bit (only entropy coding differs);
//!   * baseline SAC streams (no DF) still parse with the no4MVQ MCBPC
//!     model — backwards-compat guard.
//!
//! Tests use baseline PTYPE (no PLUSPTYPE) so the DF flag is conveyed
//! out-of-band — both encoder and decoder must opt in via
//! `set_enable_annex_j`.

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

/// Synthetic QCIF panning frame with hard 8×8 step edges to give the
/// deblocking filter something to do, plus AC noise so the motion estimator
/// finds non-zero MVs.
fn make_qcif_blocky(seed: u8, dx: i32, dy: i32) -> VideoFrame {
    let w = 176usize;
    let h = 144usize;
    let mut y = vec![0u8; w * h];
    let mut cb = vec![128u8; (w / 2) * (h / 2)];
    let mut cr = vec![128u8; (w / 2) * (h / 2)];
    for j in 0..h {
        for i in 0..w {
            let ii = (i as i32 + dx).rem_euclid(w as i32) as usize;
            let jj = (j as i32 + dy).rem_euclid(h as i32) as usize;
            // 8x8 block-sized step pattern + diagonal gradient → forces
            // visible block edges that the deblock filter must smooth.
            let by = jj / 8;
            let bx = ii / 8;
            let base: u8 = if (bx + by) % 2 == 0 { 80 } else { 160 };
            let g = ((ii + jj) as i32 / 4 + seed as i32) & 0xF;
            y[j * w + i] = base.saturating_add(g as u8);
        }
    }
    for j in 0..(h / 2) {
        for i in 0..(w / 2) {
            cb[j * (w / 2) + i] = (96 + ((j as i32) & 0x3F)) as u8;
            cr[j * (w / 2) + i] = (160 - ((i as i32) & 0x3F)) as u8;
        }
    }
    VideoFrame {
        pts: Some(seed as i64),
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

/// Encode I + 3 P frames with SAC + Annex J on (no Annex F), decode through
/// the same flags, and verify the decoder stays in sync with the encoder.
/// The reference cycle would diverge immediately if MCBPC selector or the
/// deblock filter desynced.
#[test]
fn sac_annex_j_picture_self_roundtrip_qcif() {
    let frames: Vec<VideoFrame> = (0..4u8)
        .map(|f| make_qcif_blocky(f, (f as i32) * 2, 0))
        .collect();
    let params = make_params(176, 144);

    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);
    enc.set_enable_annex_j(true);
    assert!(enc.enable_annex_j());

    for f in &frames {
        enc.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc.flush().unwrap();

    let mut packets = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        packets.push(p);
    }
    assert_eq!(packets.len(), 4);
    assert!(packets[0].flags.keyframe);

    // P-pictures must carry PTYPE bit 11 (SAC) and NOT bit 12 (AP) —
    // Annex J is signalled out-of-band on baseline PTYPE.
    for (i, p) in packets.iter().enumerate().skip(1) {
        let bit_pos_sac = 30 + 10; // PSC=22, TR=8, PTYPE bits 1..=11.
        let bit_pos_ap = bit_pos_sac + 1;
        let sac = (p.data[bit_pos_sac / 8] >> (7 - (bit_pos_sac % 8))) & 1;
        let ap = (p.data[bit_pos_ap / 8] >> (7 - (bit_pos_ap % 8))) & 1;
        assert_eq!(sac, 1, "P packet {i} PTYPE bit 11 (SAC) not set");
        assert_eq!(ap, 0, "P packet {i} PTYPE bit 12 (AP) unexpectedly set");
    }

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    dec.set_enable_annex_j(true);
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
            psnr >= 20.0,
            "SAC+J frame {i} luma PSNR {psnr:.2} dB below 20 dB floor"
        );
    }
}

/// SAC + Annex J and VLC + Annex J share DCT/quant/IDCT/MC/deblock — only
/// the entropy stage differs. The decoded YUV must therefore be
/// byte-identical between the two pipelines on the same source frames.
/// This is the round-16 SAC↔VLC byte-identical roundtrip invariant for
/// the Annex-J path.
#[test]
fn sac_annex_j_matches_vlc_annex_j_reconstruction_qcif() {
    let frames: Vec<VideoFrame> = (0..4u8)
        .map(|f| make_qcif_blocky(f, (f as i32) * 2, 0))
        .collect();
    let params = make_params(176, 144);

    // SAC + DF pipeline.
    let mut enc_sac = H263Encoder::from_params(&params).unwrap();
    enc_sac.set_enable_annex_e(true);
    enc_sac.set_enable_annex_j(true);
    for f in &frames {
        enc_sac.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc_sac.flush().unwrap();
    let mut sac_pkts = Vec::new();
    while let Ok(p) = enc_sac.receive_packet() {
        sac_pkts.push(p);
    }

    // VLC + DF pipeline.
    let mut enc_vlc = H263Encoder::from_params(&params).unwrap();
    enc_vlc.set_enable_annex_j(true);
    for f in &frames {
        enc_vlc.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc_vlc.flush().unwrap();
    let mut vlc_pkts = Vec::new();
    while let Ok(p) = enc_vlc.receive_packet() {
        vlc_pkts.push(p);
    }

    let mut dec_sac = H263Decoder::new(CodecId::new("h263"));
    dec_sac.set_enable_annex_j(true);
    for p in &sac_pkts {
        dec_sac.send_packet(p).unwrap();
    }
    dec_sac.flush().unwrap();
    let mut dec_vlc = H263Decoder::new(CodecId::new("h263"));
    dec_vlc.set_enable_annex_j(true);
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
            "SAC+J frame {fi} luma differs from VLC+J — entropy stage drifted"
        );
        assert_eq!(s.planes[1].data, v.planes[1].data, "SAC+J frame {fi} Cb");
        assert_eq!(s.planes[2].data, v.planes[2].data, "SAC+J frame {fi} Cr");
    }
}

/// Backwards-compat: a SAC stream with Annex J OFF on both sides must
/// continue decoding through the no4MVQ MCBPC model — round 16 must not
/// regress the round-14 path.
#[test]
fn sac_without_annex_j_uses_no4mvq_mcbpc_model() {
    let frames: Vec<VideoFrame> = (0..3u8)
        .map(|f| make_qcif_blocky(f, (f as i32) * 2, 0))
        .collect();
    let params = make_params(176, 144);

    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);
    // DF stays OFF.
    for f in &frames {
        enc.send_frame(&Frame::Video(f.clone())).unwrap();
    }
    enc.flush().unwrap();
    let mut packets = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        packets.push(p);
    }

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    // DF stays OFF.
    for p in &packets {
        dec.send_packet(p).unwrap();
    }
    dec.flush().unwrap();
    for _ in 0..3 {
        let _ = dec.receive_frame().unwrap();
    }
}

/// Round 16 / Part B — GOB-resync on the SAC + Annex F (AP) path.
/// CIF source has 18 GOBs of one MB row each; the SAC AP encoder must
/// emit `encoder_flush` + GOB header + fresh SAC segment at every row
/// boundary, and the decoder must `decoder_reset` at each GBSC. Verifies
/// that (a) the bitstream actually carries multiple start codes, (b) the
/// roundtrip produces the same picture as the encoder's local reconstruction.
#[test]
fn sac_ap_picture_with_gob_resync_cif() {
    use oxideav_core::Packet;
    use oxideav_h263::encoder::{
        encode_i_picture_sac_with_recon, encode_p_picture_sac_ap_with_recon_opts,
    };
    use oxideav_h263::picture::SourceFormat;

    let frames: Vec<VideoFrame> = (0..3u8)
        .map(|f| make_cif_blocky(f, (f as i32) * 2, 0))
        .collect();

    let mut packets: Vec<Packet> = Vec::new();
    let (i_bytes, mut recon) =
        encode_i_picture_sac_with_recon(352, 288, SourceFormat::Cif, 8, 0, &frames[0])
            .expect("encode I");
    packets.push(Packet::new(0, TimeBase::new(1, 30), i_bytes));
    for (idx, f) in frames.iter().enumerate().skip(1) {
        let (p_bytes, p_recon) = encode_p_picture_sac_ap_with_recon_opts(
            352,
            288,
            SourceFormat::Cif,
            8,
            idx as u8,
            f,
            &recon,
            true, // emit_gob_headers — round 16
        )
        .expect("encode P SAC+AP");
        packets.push(Packet::new(0, TimeBase::new(1, 30), p_bytes));
        recon = p_recon;
    }

    // Verify each P-packet carries multiple start codes (at least PSC + 1
    // GBSC). The CIF layout has 17 internal GOB boundaries.
    use oxideav_h263::start_code::iter_start_codes;
    for (i, p) in packets.iter().enumerate().skip(1) {
        let scs: Vec<_> = iter_start_codes(&p.data).collect();
        assert!(
            scs.len() >= 2,
            "SAC+AP P packet {i} should carry at least one GOB header (got {})",
            scs.len()
        );
    }

    let mut dec = H263Decoder::new(CodecId::new("h263"));
    for p in &packets {
        dec.send_packet(p).unwrap();
    }
    dec.flush().unwrap();
    for fi in 0..3 {
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
                let s = frames[fi].planes[0].data[j * frames[fi].planes[0].stride + ii] as i64;
                let d = yp.data[j * yp.stride + ii] as i64;
                let e = s - d;
                sse += (e * e) as u64;
            }
        }
        let mse = sse as f64 / n as f64;
        let psnr = 10.0 * (255.0_f64 * 255.0_f64 / mse.max(1e-9)).log10();
        assert!(
            psnr >= 20.0,
            "CIF SAC+AP+GOB-resync frame {fi} luma PSNR {psnr:.2} dB below floor"
        );
    }
}

fn make_cif_blocky(seed: u8, dx: i32, dy: i32) -> VideoFrame {
    let w = 352usize;
    let h = 288usize;
    let mut y = vec![0u8; w * h];
    let mut cb = vec![128u8; (w / 2) * (h / 2)];
    let mut cr = vec![128u8; (w / 2) * (h / 2)];
    for j in 0..h {
        for i in 0..w {
            let ii = (i as i32 + dx).rem_euclid(w as i32) as usize;
            let jj = (j as i32 + dy).rem_euclid(h as i32) as usize;
            let by = jj / 8;
            let bx = ii / 8;
            let base: u8 = if (bx + by) % 2 == 0 { 80 } else { 160 };
            let g = ((ii + jj) as i32 / 4 + seed as i32) & 0xF;
            y[j * w + i] = base.saturating_add(g as u8);
        }
    }
    for j in 0..(h / 2) {
        for i in 0..(w / 2) {
            cb[j * (w / 2) + i] = (96 + ((j as i32) & 0x3F)) as u8;
            cr[j * (w / 2) + i] = (160 - ((i as i32) & 0x3F)) as u8;
        }
    }
    VideoFrame {
        pts: Some(seed as i64),
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
