//! End-to-end **Annex E SAC** integration tests: the arithmetic-coded
//! picture encoders (`encode_intra_picture_sac` /
//! `encode_inter_picture_sac`) driven back through
//! `decode_picture_sac`, plus the VLC-parity pin — SAC and VLC
//! pictures of the same source share the forward-transform /
//! quantisation stage, so their reconstructions must be
//! **byte-identical** (only the entropy layer differs).

use oxideav_h263::encoder::{
    encode_inter_picture, encode_inter_picture_motion, encode_inter_picture_motion_sac,
    encode_inter_picture_sac, encode_intra_picture, encode_intra_picture_sac,
};
use oxideav_h263::picture::{
    decode_picture_no_gob0_header, decode_picture_sac, decode_sequence, DecodeOptions, YuvFrame,
};
use oxideav_h263::Error;

/// A deterministic gradient frame on every plane (the
/// `encode_roundtrip.rs` convention).
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

/// SAC INTRA pictures reconstruct **byte-identically** to the VLC
/// INTRA pictures of the same source at every tested size and
/// quantiser — the transform / quantiser stage is shared, so any
/// divergence would be an entropy-layer bug.
#[test]
fn sac_intra_matches_vlc_reconstruction_exactly() {
    for &(lw, lh) in &[(128usize, 96usize), (176, 144), (352, 288)] {
        for &quant in &[2u8, 4, 13, 31] {
            let src = gradient(lw, lh, 7);
            let vlc = encode_intra_picture(&src, quant, 0).expect("VLC encode");
            let sac = encode_intra_picture_sac(&src, quant, 0).expect("SAC encode");
            let vlc_frame = decode_picture_no_gob0_header(&vlc, None, DecodeOptions::default())
                .expect("VLC decode");
            let sac_frame =
                decode_picture_sac(&sac, None, DecodeOptions::default()).expect("SAC decode");
            assert_eq!(sac_frame.y, vlc_frame.y, "{lw}x{lh} q{quant} luma");
            assert_eq!(sac_frame.cb, vlc_frame.cb, "{lw}x{lh} q{quant} cb");
            assert_eq!(sac_frame.cr, vlc_frame.cr, "{lw}x{lh} q{quant} cr");
        }
    }
}

/// A flat SAC INTRA picture is exact (DC-only blocks, no transform
/// loss).
#[test]
fn sac_flat_intra_is_exact() {
    let src = YuvFrame::grey(176, 144);
    let bytes = encode_intra_picture_sac(&src, 12, 3).expect("encode");
    let decoded = decode_picture_sac(&bytes, None, DecodeOptions::default()).expect("decode");
    assert!(decoded.y.iter().all(|&p| p == 128));
    assert!(decoded.cb.iter().all(|&p| p == 128));
    assert!(decoded.cr.iter().all(|&p| p == 128));
}

/// SAC P-pictures (zero-MV) reconstruct byte-identically to the VLC
/// P-pictures of the same (source, reference) pair.
#[test]
fn sac_inter_matches_vlc_reconstruction_exactly() {
    let lw = 176;
    let lh = 144;
    let frame0 = gradient(lw, lh, 0);
    let i_bytes = encode_intra_picture_sac(&frame0, 5, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");

    // frame1 = recon plus a moving square so P-residuals survive.
    let mut frame1 = recon.clone();
    for row in 40..72 {
        for col in 60..92 {
            frame1.y[row * lw + col] = frame1.y[row * lw + col].wrapping_add(60);
        }
    }

    let vlc_p = encode_inter_picture(&frame1, &recon, 6, 1).expect("VLC encode P");
    let sac_p = encode_inter_picture_sac(&frame1, &recon, 6, 1).expect("SAC encode P");
    let vlc_frame = decode_picture_no_gob0_header(&vlc_p, Some(&recon), DecodeOptions::default())
        .expect("VLC decode P");
    let sac_frame =
        decode_picture_sac(&sac_p, Some(&recon), DecodeOptions::default()).expect("SAC decode P");
    assert_eq!(sac_frame.y, vlc_frame.y);
    assert_eq!(sac_frame.cb, vlc_frame.cb);
    assert_eq!(sac_frame.cr, vlc_frame.cr);
}

/// A perfectly-predicted SAC P-picture (source == reference) skips
/// every macroblock and reconstructs losslessly.
#[test]
fn sac_static_inter_is_lossless() {
    let src = gradient(176, 144, 5);
    let i_bytes = encode_intra_picture_sac(&src, 6, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let p_bytes = encode_inter_picture_sac(&recon, &recon, 6, 1).expect("encode P");
    let p_frame =
        decode_picture_sac(&p_bytes, Some(&recon), DecodeOptions::default()).expect("decode P");
    assert_eq!(p_frame.y, recon.y);
    assert_eq!(p_frame.cb, recon.cb);
    assert_eq!(p_frame.cr, recon.cr);
    // All-skip P-picture: header + 99 COD symbols + flush — a handful
    // of bytes.
    assert!(p_bytes.len() < 32, "all-skip P is {} bytes", p_bytes.len());
}

/// The arithmetic coder earns its keep: over the tested gradient
/// content the SAC INTRA picture is smaller than the VLC picture of
/// the same source at the same quantiser.
#[test]
fn sac_intra_is_smaller_than_vlc_on_gradient_content() {
    for &quant in &[4u8, 8, 13] {
        let src = gradient(176, 144, 7);
        let vlc = encode_intra_picture(&src, quant, 0).expect("VLC encode");
        let sac = encode_intra_picture_sac(&src, quant, 0).expect("SAC encode");
        assert!(
            sac.len() < vlc.len(),
            "q{quant}: SAC {} bytes >= VLC {} bytes",
            sac.len(),
            vlc.len()
        );
    }
}

/// Mode cross-checks: the SAC driver refuses a VLC picture, the VLC
/// drivers refuse an SAC picture (PTYPE bit 11 routes them apart).
#[test]
fn sac_and_vlc_drivers_reject_each_other() {
    let src = gradient(176, 144, 1);
    let vlc = encode_intra_picture(&src, 8, 0).expect("VLC encode");
    let sac = encode_intra_picture_sac(&src, 8, 0).expect("SAC encode");
    assert_eq!(
        decode_picture_sac(&vlc, None, DecodeOptions::default()).unwrap_err(),
        Error::NotImplemented
    );
    assert_eq!(
        decode_picture_no_gob0_header(&sac, None, DecodeOptions::default()).unwrap_err(),
        Error::NotImplemented
    );
}

/// §5.1.4.6 — the SAC driver refuses the barred Annex S / Annex T
/// option combinations outright.
#[test]
fn sac_refuses_barred_mode_combinations() {
    let src = gradient(176, 144, 1);
    let sac = encode_intra_picture_sac(&src, 8, 0).expect("SAC encode");
    for options in [
        DecodeOptions {
            modified_quant: true,
            ..DecodeOptions::default()
        },
        DecodeOptions {
            alt_inter_vlc: true,
            ..DecodeOptions::default()
        },
        DecodeOptions {
            aic: true,
            ..DecodeOptions::default()
        },
    ] {
        assert_eq!(
            decode_picture_sac(&sac, None, options).unwrap_err(),
            Error::NotImplemented
        );
    }
}

/// A **motion-estimated** SAC P-picture reconstructs byte-identically
/// to the VLC motion P-picture of the same (source, reference) pair —
/// the estimator, predictor replay, intra-refresh decision and
/// transform stage are all shared; only the entropy layer differs.
#[test]
fn sac_motion_inter_matches_vlc_reconstruction_exactly() {
    let lw = 176;
    let lh = 144;
    let frame0 = gradient(lw, lh, 0);
    let i_bytes = encode_intra_picture_sac(&frame0, 5, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");

    // frame1 = recon translated left by 3 px — real motion for the
    // estimator to chase (built from recon so the only error source is
    // the residual quantiser).
    let mut frame1 = recon.clone();
    for row in 0..lh {
        for col in 0..lw {
            let srccol = (col + 3).min(lw - 1);
            frame1.y[row * lw + col] = recon.y[row * lw + srccol];
        }
    }

    let vlc_p = encode_inter_picture_motion(&frame1, &recon, 5, 1, 5).expect("VLC encode");
    let sac_p = encode_inter_picture_motion_sac(&frame1, &recon, 5, 1, 5).expect("SAC encode");
    let vlc_frame = decode_picture_no_gob0_header(&vlc_p, Some(&recon), DecodeOptions::default())
        .expect("VLC decode");
    let sac_frame =
        decode_picture_sac(&sac_p, Some(&recon), DecodeOptions::default()).expect("SAC decode");
    assert_eq!(sac_frame.y, vlc_frame.y);
    assert_eq!(sac_frame.cb, vlc_frame.cb);
    assert_eq!(sac_frame.cr, vlc_frame.cr);
    // The moving-content P-picture must actually carry motion coding.
    assert!(sac_p.len() > 24, "suspiciously small SAC P");
}

/// A full SAC elementary stream (I + P + P, concatenated pictures)
/// decodes through the headline `decode_sequence` entry point: the
/// PTYPE bit-11 peek routes each picture to the SAC driver and the
/// reconstruction threads forward as the next reference.
#[test]
fn sac_elementary_stream_decodes_through_decode_sequence() {
    let lw = 176;
    let lh = 144;
    let frame0 = gradient(lw, lh, 0);
    let i_bytes = encode_intra_picture_sac(&frame0, 5, 0).expect("encode I");
    let recon0 = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");

    // P1: a moving square; P2: static (all-skip).
    let mut frame1 = recon0.clone();
    for row in 48..80 {
        for col in 30..62 {
            frame1.y[row * lw + col] = frame1.y[row * lw + col].wrapping_add(50);
        }
    }
    let p1_bytes = encode_inter_picture_motion_sac(&frame1, &recon0, 6, 1, 5).expect("encode P1");
    let recon1 =
        decode_picture_sac(&p1_bytes, Some(&recon0), DecodeOptions::default()).expect("decode P1");
    let p2_bytes = encode_inter_picture_sac(&recon1, &recon1, 6, 2).expect("encode P2");

    let mut stream = Vec::new();
    stream.extend_from_slice(&i_bytes);
    stream.extend_from_slice(&p1_bytes);
    stream.extend_from_slice(&p2_bytes);

    let frames = decode_sequence(&stream, DecodeOptions::default()).expect("decode sequence");
    assert_eq!(frames.len(), 3);
    assert_eq!(frames[0].y, recon0.y);
    assert_eq!(frames[1].y, recon1.y);
    // P2 was perfectly predicted from recon1.
    assert_eq!(frames[2].y, recon1.y);
    assert_eq!(frames[2].cb, recon1.cb);
    assert_eq!(frames[2].cr, recon1.cr);
}

/// A mixed VLC + SAC elementary stream decodes through
/// `decode_sequence`: the per-picture PTYPE bit-11 peek routes each
/// picture to its own entropy layer while the reference threads across
/// the boundary.
#[test]
fn mixed_vlc_and_sac_stream_decodes_through_decode_sequence() {
    let lw = 176;
    let lh = 144;
    let frame0 = gradient(lw, lh, 0);
    // VLC I-picture, then an SAC P-picture predicted from it.
    let i_bytes = encode_intra_picture(&frame0, 5, 0).expect("encode I");
    let recon0 =
        decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let mut frame1 = recon0.clone();
    for row in 16..48 {
        for col in 100..132 {
            frame1.y[row * lw + col] = frame1.y[row * lw + col].wrapping_add(40);
        }
    }
    let p_bytes = encode_inter_picture_sac(&frame1, &recon0, 6, 1).expect("encode SAC P");
    let expect_p =
        decode_picture_sac(&p_bytes, Some(&recon0), DecodeOptions::default()).expect("decode P");

    let mut stream = Vec::new();
    stream.extend_from_slice(&i_bytes);
    stream.extend_from_slice(&p_bytes);
    let frames = decode_sequence(&stream, DecodeOptions::default()).expect("decode sequence");
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].y, recon0.y);
    assert_eq!(frames[1].y, expect_p.y);
}

/// No start-code emulation inside an SAC picture: scanning the encoded
/// bytes for a byte-aligned PSC prefix finds only the picture's own
/// PSC at offset 0 (the §E.5 stuffing rule at work over real content).
#[test]
fn sac_picture_contains_no_emulated_psc() {
    fn psc_hits(bytes: &[u8]) -> Vec<usize> {
        let mut hits = Vec::new();
        for i in 0..bytes.len().saturating_sub(2) {
            if bytes[i] == 0x00
                && bytes[i + 1] == 0x00
                && (bytes[i + 2] & 0b1111_1100) == 0b1000_0000
            {
                hits.push(i);
            }
        }
        hits
    }
    let src = gradient(352, 288, 3);
    let bytes = encode_intra_picture_sac(&src, 2, 0).expect("encode");
    assert_eq!(psc_hits(&bytes), vec![0]);

    // Regression: the §E.5 zero-run counter must span the header /
    // arithmetic boundary. Quantisers whose PQUANT field ends in zeros
    // (tz 1..=4) maximise the header tail — before the seeded-run fix,
    // a P-picture at quant 6 emulated a PSC at byte 6.
    let i_bytes = encode_intra_picture_sac(&gradient(176, 144, 0), 5, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let mut moving = recon.clone();
    for row in 16..48 {
        for col in 100..132 {
            moving.y[row * 176 + col] = moving.y[row * 176 + col].wrapping_add(40);
        }
    }
    for &quant in &[2u8, 4, 6, 8, 16, 24] {
        let p = encode_inter_picture_sac(&moving, &recon, quant, 1).expect("encode P");
        assert_eq!(psc_hits(&p), vec![0], "q{quant} P-picture PSC emulation");
        // And it still decodes.
        decode_picture_sac(&p, Some(&recon), DecodeOptions::default()).expect("decode P");
    }
}

/// Translate a frame's luma content left by `shift` pixels (edge
/// replication), chroma by `shift / 2` (the `encode_roundtrip.rs`
/// convention).
fn translated(frame: &YuvFrame, shift: usize) -> YuvFrame {
    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = lw / 2;
    let ch = lh / 2;
    let mut out = frame.clone();
    for row in 0..lh {
        for col in 0..lw {
            out.y[row * lw + col] = frame.y[row * lw + (col + shift).min(lw - 1)];
        }
    }
    for row in 0..ch {
        for col in 0..cw {
            let src = (col + shift / 2).min(cw - 1);
            out.cb[row * cw + col] = frame.cb[row * cw + src];
            out.cr[row * cw + col] = frame.cr[row * cw + src];
        }
    }
    out
}

/// An **SAC + Advanced Prediction** (INTER4V + §F.3 OBMC) P-picture
/// reconstructs **byte-identically** to the VLC AP picture of the same
/// (source, reference) pair — the two-pass §F.2 estimator, the OBMC
/// prediction and the transform stage are all shared; only the entropy
/// layer differs. Pinned across quantisers.
#[test]
fn sac_ap_matches_vlc_ap_reconstruction_exactly() {
    use oxideav_h263::encoder::{encode_inter_picture_ap, encode_inter_picture_ap_sac};

    let i_bytes = encode_intra_picture_sac(&gradient(176, 144, 0), 6, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    // Sheared content drives divergent per-block vectors (the INTER4V
    // pay-off case).
    let mut sheared = translated(&recon, 2);
    for row in 72..144 {
        for col in 0..176 {
            sheared.y[row * 176 + col] = recon.y[row * 176 + (col + 4).min(175)];
        }
    }

    for &quant in &[4u8, 8, 13] {
        let vlc = encode_inter_picture_ap(&sheared, &recon, quant, 1, 3).expect("VLC AP");
        let sac = encode_inter_picture_ap_sac(&sheared, &recon, quant, 1, 3).expect("SAC AP");
        let vlc_rec = decode_picture_no_gob0_header(&vlc, Some(&recon), DecodeOptions::default())
            .expect("decode VLC AP");
        let sac_rec = decode_picture_sac(&sac, Some(&recon), DecodeOptions::default())
            .expect("decode SAC AP");
        assert_eq!(vlc_rec.y, sac_rec.y, "q{quant} luma");
        assert_eq!(vlc_rec.cb, sac_rec.cb, "q{quant} cb");
        assert_eq!(vlc_rec.cr, sac_rec.cr, "q{quant} cr");
    }
}

/// A static SAC AP picture (source == reference) round-trips
/// losslessly: every INTER4V macroblock estimates a zero vector and
/// codes no residual, and the OBMC blend of all-equal vectors is the
/// plain reference copy.
#[test]
fn sac_ap_static_picture_is_lossless() {
    use oxideav_h263::encoder::encode_inter_picture_ap_sac;

    let i_bytes = encode_intra_picture_sac(&gradient(176, 144, 7), 7, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let p = encode_inter_picture_ap_sac(&recon, &recon, 7, 1, 3).expect("encode AP");
    let dec = decode_picture_sac(&p, Some(&recon), DecodeOptions::default()).expect("decode AP");
    assert_eq!(dec.y, recon.y);
    assert_eq!(dec.cb, recon.cb);
    assert_eq!(dec.cr, recon.cr);
}

/// An **SAC PB-frame** reconstructs **byte-identically** to the VLC
/// PB-frame of the same (P-source, B-source, reference) triple in
/// both parts — the P-part estimator, PREC reconstruction and §G.4 /
/// §G.5 B-prediction are shared.
#[test]
fn sac_pb_matches_vlc_pb_reconstruction_exactly() {
    use oxideav_h263::encoder::{encode_pb_picture, encode_pb_picture_sac, PbConfig};
    use oxideav_h263::picture::{decode_pb_picture_no_gob0_header, decode_pb_picture_sac};

    let i_bytes = encode_intra_picture_sac(&gradient(176, 144, 0), 6, 3).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let fb = translated(&recon, 2);
    let fp = translated(&recon, 4);
    let cfg = PbConfig {
        quant: 6,
        trb: 1,
        dbquant: 1,
        search_half: 3,
    };
    let vlc = encode_pb_picture(&fp, &fb, &recon, 5, 3, &cfg).expect("VLC PB");
    let sac = encode_pb_picture_sac(&fp, &fb, &recon, 5, 3, &cfg).expect("SAC PB");

    let vlc_pair = decode_pb_picture_no_gob0_header(&vlc, &recon, 3, DecodeOptions::default())
        .expect("decode VLC PB");
    let sac_pair =
        decode_pb_picture_sac(&sac, &recon, 3, DecodeOptions::default()).expect("decode SAC PB");
    assert_eq!(vlc_pair.p_frame.y, sac_pair.p_frame.y, "P luma");
    assert_eq!(vlc_pair.p_frame.cb, sac_pair.p_frame.cb, "P cb");
    assert_eq!(vlc_pair.p_frame.cr, sac_pair.p_frame.cr, "P cr");
    assert_eq!(vlc_pair.b_frame.y, sac_pair.b_frame.y, "B luma");
    assert_eq!(vlc_pair.b_frame.cb, sac_pair.b_frame.cb, "B cb");
    assert_eq!(vlc_pair.b_frame.cr, sac_pair.b_frame.cr, "B cr");
}

/// A fully static SAC PB-frame is lossless on both parts (every
/// macroblock skips: zero vector, no P-residual, no B-residual).
#[test]
fn sac_pb_static_is_lossless_both_parts() {
    use oxideav_h263::encoder::{encode_pb_picture_sac, PbConfig};
    use oxideav_h263::picture::decode_pb_picture_sac;

    let i_bytes = encode_intra_picture_sac(&gradient(176, 144, 11), 9, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let cfg = PbConfig {
        quant: 9,
        trb: 1,
        dbquant: 0,
        search_half: 3,
    };
    let pb = encode_pb_picture_sac(&recon, &recon, &recon, 2, 0, &cfg).expect("encode PB");
    let pair = decode_pb_picture_sac(&pb, &recon, 0, DecodeOptions::default()).expect("decode PB");
    assert_eq!(pair.p_frame.y, recon.y);
    assert_eq!(pair.b_frame.y, recon.y);
    assert_eq!(pair.p_frame.cb, recon.cb);
    assert_eq!(pair.b_frame.cb, recon.cb);
    assert_eq!(pair.p_frame.cr, recon.cr);
    assert_eq!(pair.b_frame.cr, recon.cr);
}

/// A pure-SAC elementary stream carrying every staged coding shape —
/// I, then an AP (INTER4V + OBMC) P, then a PB pair — decodes through
/// the headline `decode_sequence` entry point in display order with
/// every frame tracking its source.
#[test]
fn sac_ap_and_pb_stream_decodes_through_decode_sequence() {
    use oxideav_h263::encoder::{encode_inter_picture_ap_sac, encode_pb_picture_sac, PbConfig};

    fn luma_mae(a: &YuvFrame, b: &YuvFrame) -> f64 {
        let sum: u64 =
            a.y.iter()
                .zip(b.y.iter())
                .map(|(&x, &y)| (x as i64 - y as i64).unsigned_abs())
                .sum();
        sum as f64 / a.y.len() as f64
    }

    let f0 = gradient(176, 144, 0);
    let i_bytes = encode_intra_picture_sac(&f0, 5, 0).unwrap();
    let r0 = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).unwrap();

    // SAC AP P (2 px).
    let f1 = translated(&r0, 2);
    let p1 = encode_inter_picture_ap_sac(&f1, &r0, 5, 1, 3).unwrap();
    let r1 = decode_picture_sac(&p1, Some(&r0), DecodeOptions::default()).unwrap();

    // SAC PB pair: B at 3 px, P at 4 px (TR 1 -> 3, TRB 1).
    let fb = translated(&r0, 3);
    let fp = translated(&r0, 4);
    let cfg = PbConfig {
        quant: 5,
        trb: 1,
        dbquant: 0,
        search_half: 3,
    };
    let pb = encode_pb_picture_sac(&fp, &fb, &r1, 3, 1, &cfg).unwrap();

    let mut stream = Vec::new();
    for part in [&i_bytes, &p1, &pb] {
        stream.extend_from_slice(part);
    }

    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 4, "expected I, P(AP), B, P");
    for (i, (src, dec)) in [&f0, &f1, &fb, &fp].iter().zip(decoded.iter()).enumerate() {
        let mae = luma_mae(src, dec);
        assert!(mae < 8.0, "frame {i} luma MAE {mae}");
    }
}

/// The single-picture SAC driver still refuses a PB-frame (the pair
/// decodes through `decode_pb_picture_sac`), and the SAC PB driver
/// refuses a non-PB SAC picture — the two entry points route apart.
#[test]
fn sac_pb_and_single_picture_drivers_route_apart() {
    use oxideav_h263::encoder::{encode_pb_picture_sac, PbConfig};
    use oxideav_h263::picture::decode_pb_picture_sac;

    let i_bytes = encode_intra_picture_sac(&gradient(176, 144, 1), 8, 0).expect("encode I");
    let recon = decode_picture_sac(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let cfg = PbConfig::default();
    let pb = encode_pb_picture_sac(&recon, &recon, &recon, 2, 0, &cfg).expect("encode PB");

    assert_eq!(
        decode_picture_sac(&pb, Some(&recon), DecodeOptions::default()).unwrap_err(),
        Error::NotImplemented
    );
    assert_eq!(
        decode_pb_picture_sac(&i_bytes, &recon, 0, DecodeOptions::default()).unwrap_err(),
        Error::NotImplemented
    );
}
