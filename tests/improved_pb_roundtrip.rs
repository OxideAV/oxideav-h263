//! Annex M Improved PB-frames: encoder → own decoder round trips.

use oxideav_h263::encoder::{
    encode_improved_pb_picture, encode_improved_pb_picture_stats, encode_intra_picture,
    ImprovedPbConfig, EOS_BYTES,
};
use oxideav_h263::picture::{decode_improved_pb_picture, decode_sequence, DecodeOptions, YuvFrame};
use oxideav_h263::Error;

fn textured(lw: usize, lh: usize, seed: usize) -> YuvFrame {
    let cw = lw / 2;
    let ch = lh / 2;
    let mut y = vec![0u8; lw * lh];
    for r in 0..lh {
        for c in 0..lw {
            let v = (r * 5 + c * 3 + seed * 11) % 256;
            let checker = if ((r / 8) + (c / 8)) % 2 == 0 { 30 } else { 0 };
            y[r * lw + c] = ((v + checker) % 256) as u8;
        }
    }
    let cb = (0..cw * ch).map(|i| (90 + (i % 50) + seed) as u8).collect();
    let cr = (0..cw * ch)
        .map(|i| (160 - (i % 40) + seed) as u8)
        .collect();
    YuvFrame {
        y,
        cb,
        cr,
        luma_width: lw,
        luma_height: lh,
    }
}

fn translated(frame: &YuvFrame, dx: usize, dy: usize) -> YuvFrame {
    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = lw / 2;
    let ch = lh / 2;
    let mut out = frame.clone();
    for r in 0..lh {
        for c in 0..lw {
            out.y[r * lw + c] = frame.y[((r + lh - dy) % lh) * lw + (c + lw - dx) % lw];
        }
    }
    for r in 0..ch {
        for c in 0..cw {
            let s = ((r + ch - dy / 2) % ch) * cw + (c + cw - dx / 2) % cw;
            out.cb[r * cw + c] = frame.cb[s];
            out.cr[r * cw + c] = frame.cr[s];
        }
    }
    out
}

fn luma_mae(a: &YuvFrame, b: &YuvFrame) -> f64 {
    let n = a.y.len() as f64;
    a.y.iter()
        .zip(b.y.iter())
        .map(|(&x, &y)| x.abs_diff(y) as f64)
        .sum::<f64>()
        / n
}

fn luma_psnr(a: &YuvFrame, b: &YuvFrame) -> f64 {
    let n = a.y.len() as f64;
    let mse =
        a.y.iter()
            .zip(b.y.iter())
            .map(|(&x, &y)| {
                let d = x as f64 - y as f64;
                d * d
            })
            .sum::<f64>()
            / n;
    if mse == 0.0 {
        99.0
    } else {
        10.0 * (255.0f64 * 255.0 / mse).log10()
    }
}

/// A BPB-source whose three horizontal bands favour the three §M.2
/// modes: the top band repeats the reference (forward, zero vector),
/// the middle band sits halfway between reference and P (bidirectional),
/// the bottom band repeats the P-picture (backward = PREC).
fn banded_b(reference: &YuvFrame, p: &YuvFrame, halfway: &YuvFrame) -> YuvFrame {
    let lw = reference.luma_width;
    let lh = reference.luma_height;
    let mut b = halfway.clone();
    for r in 0..lh {
        let src = if r < lh / 3 {
            reference
        } else if r < 2 * lh / 3 {
            halfway
        } else {
            p
        };
        b.y[r * lw..(r + 1) * lw].copy_from_slice(&src.y[r * lw..(r + 1) * lw]);
    }
    let cw = lw / 2;
    for r in 0..lh / 2 {
        let src = if r < lh / 6 {
            reference
        } else if r < lh / 3 {
            halfway
        } else {
            p
        };
        b.cb[r * cw..(r + 1) * cw].copy_from_slice(&src.cb[r * cw..(r + 1) * cw]);
        b.cr[r * cw..(r + 1) * cw].copy_from_slice(&src.cr[r * cw..(r + 1) * cw]);
    }
    b
}

#[test]
fn improved_pb_all_three_modes_round_trip_through_decode_sequence() {
    let base = textured(176, 144, 1);
    let i_bytes = encode_intra_picture(&base, 6, 0).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);

    // P at +4 px, halfway at +2 px (TRB = 1, TRD = 2).
    let p1 = translated(&base, 4, 0);
    let half1 = translated(&base, 2, 0);
    let b1 = banded_b(&r0, &p1, &half1);
    let cfg = ImprovedPbConfig {
        quant: 6,
        trb: 1,
        dbquant: 0,
        search_half: 6,
        forward_search_half: 3,
        allow_backward: true,
    };
    let (ipb1, stats) = encode_improved_pb_picture_stats(&p1, &b1, &r0, 2, 0, &cfg).unwrap();
    eprintln!("Improved-PB #1 mode census: {stats:?}");
    assert!(stats.forward > 0, "forward band must select §M.2.2");
    assert!(stats.bidirectional > 0, "middle band must select §M.2.1");
    assert!(stats.backward > 0, "bottom band must select §M.2.3");

    // Chain a second Improved PB-frame off the decoded P-part.
    let pair1 = decode_improved_pb_picture(&ipb1, &r0, 0, DecodeOptions::default()).unwrap();
    let p2 = translated(&base, 8, 2);
    let half2 = translated(&base, 6, 1);
    let b2 = banded_b(&pair1.p_frame, &p2, &half2);
    let ipb2 = encode_improved_pb_picture(&p2, &b2, &pair1.p_frame, 4, 2, &cfg).unwrap();

    let mut stream = i_bytes.clone();
    stream.extend_from_slice(&ipb1);
    stream.extend_from_slice(&ipb2);
    stream.extend_from_slice(&EOS_BYTES);
    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 5, "I, BPB, P, BPB, P in display order");
    assert_eq!(decoded[1], pair1.b_frame);
    assert_eq!(decoded[2], pair1.p_frame);
    for (name, src, dec) in [
        ("I", &base, &decoded[0]),
        ("BPB1", &b1, &decoded[1]),
        ("P1", &p1, &decoded[2]),
        ("BPB2", &b2, &decoded[3]),
        ("P2", &p2, &decoded[4]),
    ] {
        let mae = luma_mae(src, dec);
        let psnr = luma_psnr(src, dec);
        eprintln!("{name}: luma MAE {mae:.3}, PSNR {psnr:.2} dB");
        assert!(mae < 3.0, "{name}: luma MAE {mae:.3} too high");
    }
}

/// The BPB-part of a static clip is reconstructed exactly: every mode's
/// prediction is a whole-pel copy and the residual is zero.
#[test]
fn improved_pb_static_content_is_lossless_and_mostly_skipped() {
    let base = textured(128, 96, 5);
    let i_bytes = encode_intra_picture(&base, 4, 0).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let cfg = ImprovedPbConfig {
        quant: 4,
        ..ImprovedPbConfig::default()
    };
    let (ipb, stats) = encode_improved_pb_picture_stats(&r0, &r0, &r0, 3, 0, &cfg).unwrap();
    assert_eq!(
        stats.skipped, 48,
        "every macroblock of a static SQCIF clip skips"
    );
    let pair = decode_improved_pb_picture(&ipb, &r0, 0, DecodeOptions::default()).unwrap();
    assert_eq!(pair.p_frame, r0);
    assert_eq!(pair.b_frame, r0);
}

/// Forward-only and backward-only configurations still decode and the
/// census reflects the restriction.
#[test]
fn improved_pb_mode_restrictions_are_honoured() {
    let base = textured(128, 96, 9);
    let p = translated(&base, 4, 0);
    // Halfway content is the bidirectional natural (TRB / TRD = 1 / 2).
    let b_half = translated(&base, 2, 0);
    let no_forward = ImprovedPbConfig {
        quant: 5,
        forward_search_half: 0,
        allow_backward: true,
        ..ImprovedPbConfig::default()
    };
    let (bytes, stats) =
        encode_improved_pb_picture_stats(&p, &b_half, &base, 2, 0, &no_forward).unwrap();
    assert_eq!(stats.forward, 0);
    let pair = decode_improved_pb_picture(&bytes, &base, 0, DecodeOptions::default()).unwrap();
    let mae = luma_mae(&pair.b_frame, &b_half);
    assert!(mae < 3.0, "no-forward BPB luma MAE {mae:.3}");

    // A +1 px BPB is a forward-mode natural (one whole-pel vector).
    let b = translated(&base, 1, 0);

    let no_backward = ImprovedPbConfig {
        quant: 5,
        forward_search_half: 3,
        allow_backward: false,
        ..ImprovedPbConfig::default()
    };
    let (bytes, stats) =
        encode_improved_pb_picture_stats(&p, &b, &base, 2, 0, &no_backward).unwrap();
    assert_eq!(stats.backward, 0);
    assert!(stats.forward > 0, "a +1 px BPB is a forward-mode natural");
    let pair = decode_improved_pb_picture(&bytes, &base, 0, DecodeOptions::default()).unwrap();
    let mae = luma_mae(&pair.b_frame, &b);
    assert!(mae < 3.0, "no-backward BPB luma MAE {mae:.3}");
}

#[test]
fn improved_pb_rejects_bad_parameters() {
    let f = textured(128, 96, 2);
    let bad_q = ImprovedPbConfig {
        quant: 0,
        ..ImprovedPbConfig::default()
    };
    assert_eq!(
        encode_improved_pb_picture(&f, &f, &f, 2, 0, &bad_q).unwrap_err(),
        Error::InvalidQuantiser
    );
    let bad_trb = ImprovedPbConfig {
        trb: 2,
        ..ImprovedPbConfig::default()
    };
    // TRB must be smaller than TRD (= 2 here).
    assert_eq!(
        encode_improved_pb_picture(&f, &f, &f, 2, 0, &bad_trb).unwrap_err(),
        Error::BadPbTemporalReference
    );
    let bad_dbq = ImprovedPbConfig {
        dbquant: 4,
        ..ImprovedPbConfig::default()
    };
    assert_eq!(
        encode_improved_pb_picture(&f, &f, &f, 2, 0, &bad_dbq).unwrap_err(),
        Error::BadPbTemporalReference
    );
    let other = textured(176, 144, 2);
    assert_eq!(
        encode_improved_pb_picture(&other, &f, &f, 2, 0, &ImprovedPbConfig::default()).unwrap_err(),
        Error::NotImplemented
    );
}
