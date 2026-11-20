//! Annex T Modified Quantization and Annex S Alternative INTER VLC on
//! INTER pictures: encoder → own decoder round trips.

use oxideav_h263::encoder::{encode_inter_picture_plus, encode_intra_picture_plus};
use oxideav_h263::encoder_mq::{InterVlcConfig, encode_inter_picture_mq_stats};
use oxideav_h263::picture::{DecodeOptions, YuvFrame, decode_picture_layer, decode_sequence};

/// Busy content: a fine checkerboard riding on a gradient, plus
/// saturated chrominance bars — many large coefficients per block.
fn busy(lw: usize, lh: usize, t: usize) -> YuvFrame {
    let (cw, ch) = (lw / 2, lh / 2);
    let mut y = vec![0u8; lw * lh];
    for r in 0..lh {
        for c in 0..lw {
            let g = 60 + ((r + c + 5 * t) / 2) % 120;
            let k = if ((r / 2) + ((c + t) / 3)).is_multiple_of(2) {
                70
            } else {
                0
            };
            y[r * lw + c] = (g + k).min(255) as u8;
        }
    }
    let bars = |phase: usize| -> Vec<u8> {
        (0..cw * ch)
            .map(|i| {
                if ((i % cw + phase + t) / 4).is_multiple_of(2) {
                    40
                } else {
                    215
                }
            })
            .collect()
    };
    YuvFrame {
        y,
        cb: bars(0),
        cr: bars(2),
        luma_width: lw,
        luma_height: lh,
    }
}

fn mae(a: &[u8], b: &[u8]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(&x, &y)| x.abs_diff(y) as u64)
        .sum::<u64>() as f64
        / a.len() as f64
}

fn anchor(lw: usize, lh: usize, quant: u8) -> (Vec<u8>, YuvFrame) {
    let i = encode_intra_picture_plus(&busy(lw, lh, 0), quant, 0).unwrap();
    let r = decode_picture_layer(&i, None, DecodeOptions::default()).unwrap();
    (i, r)
}

/// §S.2 / §S.3 change only the codewords: an Alternative INTER VLC
/// picture reconstructs exactly like the default-VLC coding of the
/// same decisions, in fewer bytes on busy content, with both the
/// Table I.2 blocks and the INTRA-column CBPY actually used.
#[test]
fn alt_inter_vlc_is_lossless_recoding_and_smaller() {
    for (lw, lh) in [(176usize, 144usize), (352, 288)] {
        for quant in [2u8, 4, 9] {
            let (i_bytes, recon) = anchor(lw, lh, quant);
            let src = busy(lw, lh, 3);
            let base = InterVlcConfig::new(quant).with_search_half(6);
            let (plain, s0) = encode_inter_picture_mq_stats(&src, &recon, 1, &base).unwrap();
            let (aiv, s1) =
                encode_inter_picture_mq_stats(&src, &recon, 1, &base.with_alt_inter_vlc(true))
                    .unwrap();
            assert_eq!(s0.intra_vlc_blocks + s0.intra_cbpy, 0);
            let d_plain = decode_picture_layer(&plain, Some(&recon), DecodeOptions::default());
            let d_aiv = decode_picture_layer(&aiv, Some(&recon), DecodeOptions::default());
            assert_eq!(d_aiv.unwrap(), d_plain.unwrap(), "{lw}x{lh} q{quant}");
            assert!(aiv.len() <= plain.len(), "{} vs {}", aiv.len(), plain.len());
            eprintln!(
                "{lw}x{lh} q{quant}: default {} B, AIV {} B, {s1:?}",
                plain.len(),
                aiv.len()
            );
            if quant <= 4 {
                assert!(s1.intra_vlc_blocks > 0, "{lw}x{lh} q{quant}: {s1:?}");
                assert!(s1.intra_cbpy > 0, "{lw}x{lh} q{quant}: {s1:?}");
                assert!(aiv.len() < plain.len());
            }
            // The default-mode picture of this encoder is the same
            // picture the plain PLUSPTYPE P encoder produces a decode
            // for: through decode_sequence too.
            let mut stream = i_bytes.clone();
            stream.extend_from_slice(&aiv);
            let frames = decode_sequence(&stream, DecodeOptions::default()).unwrap();
            assert_eq!(frames.len(), 2);
        }
    }
}

/// §T.3 — the chrominance step `QUANT_C` is finer than QUANT above 6:
/// chrominance fidelity improves, luminance is untouched.
#[test]
fn modified_quant_refines_chrominance() {
    let (_, recon) = anchor(176, 144, 20);
    let src = busy(176, 144, 2);
    let base = InterVlcConfig::new(20).with_search_half(6);
    let (plain, _) = encode_inter_picture_mq_stats(&src, &recon, 1, &base).unwrap();
    let (mq, _) =
        encode_inter_picture_mq_stats(&src, &recon, 1, &base.with_modified_quant(true)).unwrap();
    let d_plain = decode_picture_layer(&plain, Some(&recon), DecodeOptions::default()).unwrap();
    let d_mq = decode_picture_layer(&mq, Some(&recon), DecodeOptions::default()).unwrap();
    assert_eq!(d_mq.y, d_plain.y, "luminance step unchanged");
    let (c_plain, c_mq) = (
        mae(&d_plain.cb, &src.cb) + mae(&d_plain.cr, &src.cr),
        mae(&d_mq.cb, &src.cb) + mae(&d_mq.cr, &src.cr),
    );
    eprintln!("chroma MAE: QUANT {c_plain:.3}, QUANT_C {c_mq:.3}");
    assert!(c_mq < c_plain * 0.8, "{c_mq} vs {c_plain}");
}

/// §T.4 — at QUANT 1 a residual coefficient can exceed the ±127 level
/// range; EXTENDED-ESCAPE carries it and the picture comes back nearly
/// exact, where the default range has to clip.
#[test]
fn modified_quant_extended_levels_round_trip() {
    // A flat reference and a hard-edged target: large residual
    // coefficients everywhere.
    let flat = YuvFrame {
        y: vec![0; 176 * 144],
        cb: vec![128; 88 * 72],
        cr: vec![128; 88 * 72],
        luma_width: 176,
        luma_height: 144,
    };
    let i = encode_intra_picture_plus(&flat, 1, 0).unwrap();
    let recon = decode_picture_layer(&i, None, DecodeOptions::default()).unwrap();
    let mut src = flat.clone();
    for (n, v) in src.y.iter_mut().enumerate() {
        let (r, c) = (n / 176, n % 176);
        *v = if (r / 8 + c / 8).is_multiple_of(2) {
            255
        } else {
            0
        };
    }
    let base = InterVlcConfig::new(1).with_search_half(0);
    let (plain, s0) = encode_inter_picture_mq_stats(&src, &recon, 1, &base).unwrap();
    let (mq, s1) = encode_inter_picture_mq_stats(
        &src,
        &recon,
        1,
        &base.with_modified_quant(true).with_alt_inter_vlc(true),
    )
    .unwrap();
    assert_eq!(s0.extended_levels, 0);
    assert!(s1.extended_levels > 0, "{s1:?}");
    let d_plain = decode_picture_layer(&plain, Some(&recon), DecodeOptions::default()).unwrap();
    let d_mq = decode_picture_layer(&mq, Some(&recon), DecodeOptions::default()).unwrap();
    let (e_plain, e_mq) = (mae(&d_plain.y, &src.y), mae(&d_mq.y, &src.y));
    eprintln!("QUANT 1 luma MAE: ±127 range {e_plain:.3}, extended {e_mq:.3}");
    assert!(e_mq < 1.5, "{e_mq}");
    assert!(e_plain > 4.0 * e_mq.max(0.25), "{e_plain} vs {e_mq}");
}

/// Both annexes with UMV, a short chain, against the plain P encoder
/// for rate: the mode set stays source-close over several pictures.
#[test]
fn mq_aiv_umv_chain_stays_close() {
    let (i_bytes, mut reference) = anchor(176, 144, 8);
    let cfg = InterVlcConfig::new(8)
        .with_search_half(8)
        .with_modified_quant(true)
        .with_alt_inter_vlc(true)
        .with_umv(true);
    let mut stream = i_bytes;
    let (mut total, mut total_plain) = (0usize, 0usize);
    for t in 1..=3 {
        let src = busy(176, 144, 2 * t);
        let (p, _) = encode_inter_picture_mq_stats(&src, &reference, t as u8, &cfg).unwrap();
        total += p.len();
        total_plain += encode_inter_picture_plus(&src, &reference, 8, t as u8, 8)
            .unwrap()
            .len();
        reference = decode_picture_layer(&p, Some(&reference), DecodeOptions::default()).unwrap();
        assert!(mae(&reference.y, &src.y) < 6.0);
        stream.extend_from_slice(&p);
    }
    eprintln!("3 P-pictures: MQ+AIV+UMV {total} B, plain {total_plain} B");
    let frames = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(frames.len(), 4);
    assert_eq!(frames[3], reference);
}
