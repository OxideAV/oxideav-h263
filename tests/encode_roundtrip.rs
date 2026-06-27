//! End-to-end **encoder** integration tests driving the public encode
//! API and decoding the result back through the crate's own decoder.
//!
//! These exercise `encode_intra_picture`, `encode_inter_picture`,
//! `encode_inter_picture_motion` and `encode_intra_sequence` over
//! synthetic planar 4:2:0 frames, asserting the reconstructed output
//! matches the source to the round-trip tolerance the lossy transform +
//! dead-zone quantiser permit (a flat block is exact; AC-bearing content
//! is bounded). The encoder is non-normative, so the criterion is
//! "decodes through our decoder and reconstructs within tolerance",
//! exactly as a real H.263 codec round-trips.

use oxideav_h263::encoder::{
    encode_inter_picture, encode_inter_picture_motion, encode_intra_picture, encode_intra_sequence,
};
use oxideav_h263::picture::{
    decode_picture_no_gob0_header, decode_sequence, DecodeOptions, YuvFrame,
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

fn luma_mae(a: &YuvFrame, b: &YuvFrame) -> f64 {
    let sum: u64 =
        a.y.iter()
            .zip(b.y.iter())
            .map(|(x, y)| (*x as i32 - *y as i32).unsigned_abs() as u64)
            .sum();
    sum as f64 / a.y.len() as f64
}

#[test]
fn intra_picture_round_trips_within_tolerance() {
    for &(lw, lh) in &[(128usize, 96usize), (176, 144), (352, 288)] {
        let src = gradient(lw, lh, 0);
        let bytes = encode_intra_picture(&src, 4, 0).expect("encode I");
        let decoded = decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default())
            .expect("decode I");
        assert_eq!((decoded.luma_width, decoded.luma_height), (lw, lh));
        let mae = luma_mae(&src, &decoded);
        assert!(mae < 8.0, "{}x{} INTRA luma MAE {}", lw, lh, mae);
    }
}

#[test]
fn flat_intra_picture_is_exact() {
    let src = YuvFrame::grey(176, 144);
    let bytes = encode_intra_picture(&src, 12, 3).expect("encode");
    let decoded =
        decode_picture_no_gob0_header(&bytes, None, DecodeOptions::default()).expect("decode");
    assert!(decoded.y.iter().all(|&p| p == 128));
    assert!(decoded.cb.iter().all(|&p| p == 128));
    assert!(decoded.cr.iter().all(|&p| p == 128));
}

#[test]
fn static_inter_picture_is_lossless() {
    let src = gradient(176, 144, 5);
    let i_bytes = encode_intra_picture(&src, 6, 0).expect("encode I");
    let recon =
        decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    // A P-frame whose source equals the reconstructed reference is
    // perfectly predicted (all macroblocks skipped).
    let p_bytes = encode_inter_picture(&recon, &recon, 6, 1).expect("encode P");
    let p_decoded = decode_picture_no_gob0_header(&p_bytes, Some(&recon), DecodeOptions::default())
        .expect("decode P");
    assert_eq!(p_decoded.y, recon.y);
    assert_eq!(p_decoded.cb, recon.cb);
    assert_eq!(p_decoded.cr, recon.cr);
}

#[test]
fn motion_compensated_inter_beats_zero_motion_on_translation() {
    let lw = 176;
    let lh = 144;
    let frame0 = gradient(lw, lh, 0);
    let i_bytes = encode_intra_picture(&frame0, 5, 0).expect("encode I");
    let recon =
        decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).expect("decode I");

    // frame1 = recon translated left by 3 px (built from recon so the
    // only error source is the residual quantiser).
    let mut frame1 = recon.clone();
    for row in 0..lh {
        for col in 0..lw {
            let srccol = (col + 3).min(lw - 1);
            frame1.y[row * lw + col] = recon.y[row * lw + srccol];
        }
    }

    let mc = encode_inter_picture_motion(&frame1, &recon, 5, 1, 5).expect("encode MC");
    let zm = encode_inter_picture(&frame1, &recon, 5, 1).expect("encode ZM");

    let mc_decoded = decode_picture_no_gob0_header(&mc, Some(&recon), DecodeOptions::default())
        .expect("decode MC");
    let mae = luma_mae(&frame1, &mc_decoded);
    assert!(mae < 6.0, "motion-compensated luma MAE {}", mae);
    assert!(
        mc.len() <= zm.len(),
        "motion-compensated stream ({}) not smaller than zero-motion ({})",
        mc.len(),
        zm.len()
    );
}

#[test]
fn intra_sequence_decodes_to_all_frames() {
    let frames = vec![
        gradient(176, 144, 0),
        gradient(176, 144, 30),
        YuvFrame::grey(176, 144),
        gradient(176, 144, 60),
    ];
    let stream = encode_intra_sequence(&frames, 7, 0).expect("encode sequence");
    let decoded = decode_sequence(&stream, DecodeOptions::default()).expect("decode sequence");
    assert_eq!(decoded.len(), frames.len());
    for (src, dec) in frames.iter().zip(decoded.iter()) {
        assert_eq!((dec.luma_width, dec.luma_height), (176, 144));
        let mae = luma_mae(src, dec);
        assert!(mae < 8.0, "sequence frame MAE {}", mae);
    }
}

/// An I-frame followed by a motion-compensated P-frame, concatenated as
/// an elementary stream, decodes through `decode_sequence` into two
/// frames (the P-frame referencing the reconstructed I-frame).
#[test]
fn i_then_p_gop_decodes_as_sequence() {
    let lw = 176;
    let lh = 144;
    let frame0 = gradient(lw, lh, 0);

    // Encode the I-frame and decode it to learn the reconstructed
    // reference the P-frame must predict from.
    let i_bytes = encode_intra_picture(&frame0, 6, 0).expect("encode I");
    let recon0 =
        decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).expect("decode I");

    // frame1 = recon0 shifted by 2px; encode against recon0.
    let mut frame1 = recon0.clone();
    for row in 0..lh {
        for col in 0..lw {
            let srccol = (col + 2).min(lw - 1);
            frame1.y[row * lw + col] = recon0.y[row * lw + srccol];
        }
    }
    let p_bytes = encode_inter_picture_motion(&frame1, &recon0, 6, 1, 4).expect("encode P");

    let mut stream = i_bytes.clone();
    stream.extend_from_slice(&p_bytes);

    let decoded = decode_sequence(&stream, DecodeOptions::default()).expect("decode GOP");
    assert_eq!(decoded.len(), 2, "expected I + P = 2 frames");
    // Frame 0 matches the I reconstruction exactly.
    assert_eq!(decoded[0].y, recon0.y);
    // Frame 1 reconstructs the translated content within tolerance.
    let mae = luma_mae(&frame1, &decoded[1]);
    assert!(mae < 6.0, "P-frame luma MAE {}", mae);
}
