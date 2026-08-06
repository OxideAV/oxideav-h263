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
