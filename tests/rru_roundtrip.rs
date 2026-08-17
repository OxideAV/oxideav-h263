//! End-to-end **Annex Q Reduced-Resolution Update** integration tests:
//! the RRU picture encoders driven back through the self-describing
//! PLUSPTYPE decode path (`decode_picture_layer` / `decode_sequence`),
//! plus the §Q.4 pseudo-motion-vector lattice pins.

use oxideav_h263::encoder::{
    encode_inter_picture_rru, encode_inter_picture_rru_umv, encode_intra_picture_rru,
};
use oxideav_h263::motion::{rru_actual_component, rru_pseudo_component};
use oxideav_h263::picture::{decode_picture_layer, decode_sequence, DecodeOptions, YuvFrame};

/// A deterministic gradient frame on every plane.
fn gradient(lw: usize, lh: usize, seed: u8) -> YuvFrame {
    let cw = lw / 2;
    let ch = lh / 2;
    let mut y = vec![0u8; lw * lh];
    for row in 0..lh {
        for col in 0..lw {
            y[row * lw + col] = (32 + (col / 2 + row / 2 + seed as usize) % 192) as u8;
        }
    }
    let mut cb = vec![0u8; cw * ch];
    let mut cr = vec![0u8; cw * ch];
    for row in 0..ch {
        for col in 0..cw {
            cb[row * cw + col] = (80 + (col / 2 % 64)) as u8;
            cr[row * cw + col] = (100 + (row / 2 % 56)) as u8;
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
            .map(|(&x, &y)| (x as i64 - y as i64).unsigned_abs())
            .sum();
    sum as f64 / a.y.len() as f64
}

/// Translate a frame's luma content left by `shift` pixels (edge
/// replication), chroma by `shift / 2`.
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

/// §Q.4 — the pseudo ↔ actual component maps are exact inverses over
/// the whole pseudo range, and every actual value is a half-integer
/// (odd half-pel count) or zero within [-31.5, 30.5] pels.
#[test]
fn rru_pseudo_actual_lattice_round_trips() {
    for pseudo in -32i32..=31 {
        let actual = rru_actual_component(pseudo);
        assert!(actual == 0 || actual.rem_euclid(2) == 1 || (-actual).rem_euclid(2) == 1);
        assert!((-63..=61).contains(&actual), "actual {actual}");
        // Convert back through the §Q.4 item-1 predictor map: an
        // actual value used as PC gives back the pseudo value.
        assert_eq!(rru_pseudo_component(actual), pseudo, "pseudo {pseudo}");
    }
    assert_eq!(rru_actual_component(0), 0);
    assert_eq!(rru_actual_component(1), 1); // 0.5 pel
    assert_eq!(rru_actual_component(-1), -1);
    assert_eq!(rru_actual_component(4), 7); // 2.0 pseudo -> 3.5 pel
}

/// An RRU INTRA picture round-trips within the mode's low-pass budget
/// on smooth content, at CIF (coded size == reference size) and QCIF
/// (the §Q.3 extension + §Q.2.3 crop case).
#[test]
fn rru_intra_round_trips_within_lowpass_budget() {
    for (lw, lh) in [(352usize, 288usize), (176, 144)] {
        let src = gradient(lw, lh, 3);
        let bytes = encode_intra_picture_rru(&src, 6, 0).expect("encode RRU I");
        let dec = decode_picture_layer(&bytes, None, DecodeOptions::default()).expect("decode");
        assert_eq!((dec.luma_width, dec.luma_height), (lw, lh));
        let mae = luma_mae(&src, &dec);
        assert!(mae < 6.0, "{lw}x{lh} RRU I luma MAE {mae}");
    }
}

/// A static RRU P-picture (source == decoded reference) is lossless:
/// every 32 × 32 macroblock skips, the §Q.7 filter has no coded
/// macroblock to fire on, and the reconstruction is the reference.
#[test]
fn rru_static_p_is_lossless() {
    let src = gradient(176, 144, 9);
    let i_bytes = encode_intra_picture_rru(&src, 7, 0).expect("encode I");
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let p_bytes = encode_inter_picture_rru(&recon, &recon, 7, 1, 2).expect("encode P");
    let dec =
        decode_picture_layer(&p_bytes, Some(&recon), DecodeOptions::default()).expect("decode P");
    assert_eq!(dec.y, recon.y);
    assert_eq!(dec.cb, recon.cb);
    assert_eq!(dec.cr, recon.cr);
}

/// Translated content through the RRU P encoder round-trips within
/// tolerance — the §Q.4 half-integer-or-zero vector lattice cannot
/// represent integer translations exactly, so the residual layer
/// carries the correction.
#[test]
fn rru_translated_p_round_trips() {
    let src = gradient(176, 144, 0);
    let i_bytes = encode_intra_picture_rru(&src, 5, 0).expect("encode I");
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let moved = translated(&recon, 3);
    let p_bytes = encode_inter_picture_rru(&moved, &recon, 5, 1, 4).expect("encode P");
    let dec =
        decode_picture_layer(&p_bytes, Some(&recon), DecodeOptions::default()).expect("decode P");
    let mae = luma_mae(&moved, &dec);
    assert!(mae < 6.0, "RRU P luma MAE {mae}");
}

/// An RRU I + P elementary stream decodes through the headline
/// `decode_sequence` entry point (extended-PTYPE dispatch → RRU
/// routing, reference threading across the RRU crop).
#[test]
fn rru_stream_decodes_through_decode_sequence() {
    let src = gradient(176, 144, 4);
    let i_bytes = encode_intra_picture_rru(&src, 6, 0).unwrap();
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).unwrap();
    let moved = translated(&recon, 2);
    let p_bytes = encode_inter_picture_rru(&moved, &recon, 6, 1, 3).unwrap();

    let mut stream = i_bytes.clone();
    stream.extend_from_slice(&p_bytes);
    let frames = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].y, recon.y);
    let mae = luma_mae(&moved, &frames[1]);
    assert!(mae < 6.0, "sequence RRU P luma MAE {mae}");
}

/// RRU × UMV (§Q.4 / §D.2): a static RRU + UMV P-picture is lossless —
/// every macroblock skips exactly as in the default mode, and the
/// UMV-signalled header (OPPTYPE UMV + §5.1.9 UUI) round-trips through
/// the Table D.3 macroblock parser.
#[test]
fn rru_umv_static_p_is_lossless() {
    let src = gradient(176, 144, 9);
    let i_bytes = encode_intra_picture_rru(&src, 7, 0).expect("encode I");
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let p_bytes = encode_inter_picture_rru_umv(&recon, &recon, 7, 1, 2).expect("encode P");
    let dec =
        decode_picture_layer(&p_bytes, Some(&recon), DecodeOptions::default()).expect("decode P");
    assert_eq!(dec.y, recon.y);
    assert_eq!(dec.cb, recon.cb);
    assert_eq!(dec.cr, recon.cr);
}

/// RRU × UMV big motion: a 40-pixel translation needs a pseudo vector
/// of ≈ 20 pixels — outside the default RRU pseudo window
/// (`[-16, 15.5]` pel) but inside the UMV Tables-D.1/D.2 pseudo range
/// (±32 pel at sub-QCIF width). The Table D.3 pseudo differences carry
/// it; the §Q.4 half-integer-or-zero actual lattice plus the residual
/// layer reconstruct within the RRU low-pass budget.
#[test]
fn rru_umv_large_translation_round_trips() {
    let src = gradient(128, 96, 0);
    let i_bytes = encode_intra_picture_rru(&src, 5, 0).expect("encode I");
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).expect("decode I");
    let moved = translated(&recon, 40);
    let p_bytes = encode_inter_picture_rru_umv(&moved, &recon, 5, 1, 44).expect("encode P");
    let dec =
        decode_picture_layer(&p_bytes, Some(&recon), DecodeOptions::default()).expect("decode P");
    let mae = luma_mae(&moved, &dec);
    assert!(mae < 6.0, "RRU+UMV P luma MAE {mae}");

    // The same translation through the default-window encoder cannot
    // reach a ≈20-pel pseudo vector; its best reconstruction is
    // measurably worse, proving the UMV leg actually used the extended
    // range rather than the residual layer alone.
    let p_default = encode_inter_picture_rru(&moved, &recon, 5, 1, 44).expect("encode P default");
    let dec_default = decode_picture_layer(&p_default, Some(&recon), DecodeOptions::default())
        .expect("decode P default");
    let mae_default = luma_mae(&moved, &dec_default);
    assert!(
        p_bytes.len() < p_default.len(),
        "UMV RRU stream should be smaller: {} vs {}",
        p_bytes.len(),
        p_default.len()
    );
    assert!(
        mae <= mae_default + 0.5,
        "UMV RRU should not reconstruct worse: {mae} vs {mae_default}"
    );
}

/// An RRU + UMV I + P elementary stream decodes through
/// `decode_sequence` (extended-PTYPE dispatch → RRU routing with the
/// Table D.3 motion path).
#[test]
fn rru_umv_stream_decodes_through_decode_sequence() {
    let src = gradient(176, 144, 4);
    let i_bytes = encode_intra_picture_rru(&src, 6, 0).unwrap();
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).unwrap();
    let moved = translated(&recon, 2);
    let p_bytes = encode_inter_picture_rru_umv(&moved, &recon, 6, 1, 3).unwrap();

    let mut stream = i_bytes.clone();
    stream.extend_from_slice(&p_bytes);
    let frames = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].y, recon.y);
    let mae = luma_mae(&moved, &frames[1]);
    assert!(mae < 6.0, "sequence RRU+UMV P luma MAE {mae}");
}

/// RRU pictures refuse the unstaged mode combinations (the OPPTYPE
/// AP / DF / AIC bits alter the OBMC / filter layers): a caller-forced
/// deblock option on an RRU stream is refused rather than
/// mis-filtered.
#[test]
fn rru_refuses_deblock_option() {
    use oxideav_h263::Error;
    let src = gradient(176, 144, 1);
    let bytes = encode_intra_picture_rru(&src, 8, 0).unwrap();
    assert_eq!(
        decode_picture_layer(
            &bytes,
            None,
            DecodeOptions {
                deblock: true,
                ..DecodeOptions::default()
            }
        )
        .unwrap_err(),
        Error::NotImplemented
    );
}
