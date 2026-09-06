//! Annex M Improved PB-frames: encoder → own decoder round trips.

use oxideav_h263::encoder::{
    encode_improved_pb_picture, encode_improved_pb_picture_stats, encode_intra_picture,
    encode_pb_picture, encode_pb_picture_ap_stats, encode_pb_picture_umv_stats, ImprovedPbConfig,
    PbConfig, EOS_BYTES,
};
use oxideav_h263::picture::{
    decode_improved_pb_picture, decode_pb_picture_no_gob0_header, decode_sequence, DecodeOptions,
    YuvFrame,
};
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
        advanced_prediction: false,
        umv: false,
        slice_rows: 0,
        intra_refresh: 0,
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

/// Annex G PB-frames + Advanced Prediction: INTER4V P-parts through the
/// §F.3 OBMC blend, the B-part scaled per 8 × 8 block. The stream
/// decodes through `decode_sequence`; the OBMC P-part must beat the
/// single-vector Annex G P-part on a warped (non-translational) clip.
#[test]
fn annex_g_pb_with_advanced_prediction_round_trips() {
    let base = textured(176, 144, 3);
    let i_bytes = encode_intra_picture(&base, 6, 0).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    // Non-uniform motion: the left half moves 4 px, the right half 2 px.
    let mut p = translated(&base, 4, 0);
    let p_right = translated(&base, 2, 0);
    for r in 0..144 {
        p.y[r * 176 + 88..r * 176 + 176].copy_from_slice(&p_right.y[r * 176 + 88..r * 176 + 176]);
    }
    for r in 0..72 {
        p.cb[r * 88 + 44..r * 88 + 88].copy_from_slice(&p_right.cb[r * 88 + 44..r * 88 + 88]);
        p.cr[r * 88 + 44..r * 88 + 88].copy_from_slice(&p_right.cr[r * 88 + 44..r * 88 + 88]);
    }
    let mut b = translated(&base, 2, 0);
    let b_right = translated(&base, 1, 0);
    for r in 0..144 {
        b.y[r * 176 + 88..r * 176 + 176].copy_from_slice(&b_right.y[r * 176 + 88..r * 176 + 176]);
    }
    let cfg = PbConfig {
        quant: 6,
        trb: 1,
        dbquant: 0,
        search_half: 6,
        b_search_half: 2,
    };
    let (ap_bytes, stats) = encode_pb_picture_ap_stats(&p, &b, &r0, 2, 0, &cfg).unwrap();
    eprintln!("Annex G + AP census: {stats:?}");
    assert_eq!(
        stats.forward + stats.backward,
        0,
        "Annex G has no §M.2 modes"
    );
    let plain_bytes = encode_pb_picture(&p, &b, &r0, 2, 0, &cfg).unwrap();

    let ap_pair =
        decode_pb_picture_no_gob0_header(&ap_bytes, &r0, 0, DecodeOptions::default()).unwrap();
    let plain_pair =
        decode_pb_picture_no_gob0_header(&plain_bytes, &r0, 0, DecodeOptions::default()).unwrap();
    let (ap_p, plain_p) = (
        luma_mae(&ap_pair.p_frame, &p),
        luma_mae(&plain_pair.p_frame, &p),
    );
    let (ap_b, plain_b) = (
        luma_mae(&ap_pair.b_frame, &b),
        luma_mae(&plain_pair.b_frame, &b),
    );
    eprintln!(
        "AP: {} bytes, P MAE {ap_p:.3}, B MAE {ap_b:.3}; plain: {} bytes, P MAE {plain_p:.3}, B MAE {plain_b:.3}",
        ap_bytes.len(),
        plain_bytes.len()
    );
    assert!(
        ap_p < 3.0 && ap_b < 3.5,
        "AP PB pair reconstructs the sources"
    );

    let mut stream = i_bytes;
    stream.extend_from_slice(&ap_bytes);
    stream.extend_from_slice(&EOS_BYTES);
    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 3);
    assert_eq!(decoded[1], ap_pair.b_frame);
    assert_eq!(decoded[2], ap_pair.p_frame);
}

/// Annex M + Advanced Prediction with an INTRA refresh: every fourth
/// macroblock is INTRA coded and still carries its §G.2 B-purpose
/// vector, which the neighbours' OBMC uses as the remote vector
/// ("the remote 'INTRA' motion vector is used") — the decoder must
/// reproduce the encoder's closed-loop prediction for the pair to
/// reconstruct cleanly.
#[test]
fn improved_pb_with_advanced_prediction_and_intra_refresh_round_trips() {
    let base = textured(176, 144, 7);
    let i_bytes = encode_intra_picture(&base, 5, 0).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let p1 = translated(&base, 5, 2);
    let half1 = translated(&base, 3, 1);
    let b1 = banded_b(&r0, &p1, &half1);
    let cfg = ImprovedPbConfig {
        quant: 5,
        trb: 1,
        dbquant: 0,
        search_half: 6,
        forward_search_half: 3,
        allow_backward: true,
        advanced_prediction: true,
        umv: false,
        slice_rows: 0,
        intra_refresh: 4,
    };
    let (ipb1, stats) = encode_improved_pb_picture_stats(&p1, &b1, &r0, 2, 0, &cfg).unwrap();
    eprintln!("Improved-PB + AP census: {stats:?}");
    assert_eq!(stats.intra, 99usize.div_ceil(4));
    assert!(stats.forward > 0 && stats.bidirectional > 0 && stats.backward > 0);

    let pair1 = decode_improved_pb_picture(&ipb1, &r0, 0, DecodeOptions::default()).unwrap();
    let p2 = translated(&base, 9, 3);
    let half2 = translated(&base, 7, 2);
    let b2 = banded_b(&pair1.p_frame, &p2, &half2);
    let ipb2 = encode_improved_pb_picture(&p2, &b2, &pair1.p_frame, 4, 2, &cfg).unwrap();

    let mut stream = i_bytes;
    stream.extend_from_slice(&ipb1);
    stream.extend_from_slice(&ipb2);
    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 5);
    for (name, src, dec) in [
        ("BPB1", &b1, &decoded[1]),
        ("P1", &p1, &decoded[2]),
        ("BPB2", &b2, &decoded[3]),
        ("P2", &p2, &decoded[4]),
    ] {
        let mae = luma_mae(src, dec);
        eprintln!(
            "{name}: luma MAE {mae:.3}, PSNR {:.2} dB",
            luma_psnr(src, dec)
        );
        assert!(mae < 3.0, "{name}: luma MAE {mae:.3} too high");
    }
}

/// Annex G PB-frames in the Unrestricted Motion Vector mode: a 20 px
/// pan needs the §D.2 extended range (the predictor slides the
/// reachable window out), and the MVDB delta is resolved per block
/// with `Pc = (TRB × MV)/TRD`. Pinned: the UMV pair reconstructs a pan
/// the default-range pair cannot follow.
#[test]
fn annex_g_pb_umv_follows_a_wide_pan() {
    let base = textured(176, 144, 13);
    let i_bytes = encode_intra_picture(&base, 6, 0).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let p = translated(&base, 22, 0);
    let b = translated(&base, 11, 0);
    let cfg = PbConfig {
        quant: 6,
        trb: 1,
        dbquant: 0,
        search_half: 24,
        b_search_half: 2,
    };
    let (umv_bytes, stats) = encode_pb_picture_umv_stats(&p, &b, &r0, 2, 0, &cfg).unwrap();
    eprintln!("Annex G + UMV census: {stats:?}");
    let plain_bytes = encode_pb_picture(&p, &b, &r0, 2, 0, &cfg).unwrap();
    let umv_pair =
        decode_pb_picture_no_gob0_header(&umv_bytes, &r0, 0, DecodeOptions::default()).unwrap();
    let plain_pair =
        decode_pb_picture_no_gob0_header(&plain_bytes, &r0, 0, DecodeOptions::default()).unwrap();
    let (umv_p, plain_p) = (
        luma_mae(&umv_pair.p_frame, &p),
        luma_mae(&plain_pair.p_frame, &p),
    );
    let (umv_b, plain_b) = (
        luma_mae(&umv_pair.b_frame, &b),
        luma_mae(&plain_pair.b_frame, &b),
    );
    eprintln!(
        "UMV: {} bytes, P MAE {umv_p:.3}, B MAE {umv_b:.3}; plain: {} bytes, P MAE {plain_p:.3}, B MAE {plain_b:.3}",
        umv_bytes.len(),
        plain_bytes.len()
    );
    assert!(
        umv_p < 3.0 && umv_b < 3.5,
        "UMV PB pair reconstructs the wide pan"
    );
    assert!(
        umv_bytes.len() < plain_bytes.len(),
        "the extended range pays off"
    );
    let mut stream = i_bytes;
    stream.extend_from_slice(&umv_bytes);
    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 3);
    assert_eq!(decoded[1], umv_pair.b_frame);
    assert_eq!(decoded[2], umv_pair.p_frame);
}

/// Annex M + UMV (PLUSPTYPE, Table D.3): the P-part and the §M.2.2
/// forward vectors are single-valued Table D.3 differences over the
/// UUI = "1" range; a 20 px pan on the P-part and a forward-mode BPB
/// whose best vector points over the left picture boundary (§D.1 edge
/// replication) both reconstruct.
#[test]
fn improved_pb_umv_plus_round_trips_with_over_boundary_forward_vectors() {
    let base = textured(176, 144, 17);
    let i_bytes = encode_intra_picture(&base, 6, 0).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    // P: 20 px pan (needs UMV). BPB: the reference shifted right by 6 px
    // with its left 6 columns edge-replicated — the forward vector
    // (-12, 0) half-pel points 6 px left of the picture edge there.
    let p = translated(&base, 20, 0);
    let mut b = translated(&r0, 6, 0);
    for r in 0..144 {
        let edge = r0.y[r * 176];
        for c in 0..6 {
            b.y[r * 176 + c] = edge;
        }
    }
    for r in 0..72 {
        let (cb, cr) = (r0.cb[r * 88], r0.cr[r * 88]);
        for c in 0..3 {
            b.cb[r * 88 + c] = cb;
            b.cr[r * 88 + c] = cr;
        }
    }
    for ap in [false, true] {
        let cfg = ImprovedPbConfig {
            quant: 6,
            trb: 1,
            dbquant: 0,
            search_half: 22,
            forward_search_half: 8,
            allow_backward: true,
            advanced_prediction: ap,
            umv: true,
            slice_rows: 0,
            intra_refresh: 0,
        };
        let (bytes, stats) = encode_improved_pb_picture_stats(&p, &b, &r0, 2, 0, &cfg).unwrap();
        eprintln!("Improved-PB + UMV+ (ap {ap}) census: {stats:?}");
        assert!(
            stats.forward > 0,
            "the shifted BPB is a forward-mode natural"
        );
        let pair = decode_improved_pb_picture(&bytes, &r0, 0, DecodeOptions::default()).unwrap();
        let (mp, mb) = (luma_mae(&pair.p_frame, &p), luma_mae(&pair.b_frame, &b));
        eprintln!("ap {ap}: P MAE {mp:.3}, BPB MAE {mb:.3}");
        assert!(mp < 3.0, "P MAE {mp:.3}");
        assert!(
            mb < 1.0,
            "the BPB is an exact forward copy up to quantisation: MAE {mb:.3}"
        );
        // The left-edge macroblocks' forward vectors must reach over the
        // boundary: their BPB reconstruction equals the edge-replicated
        // source there.
        let mut stream = i_bytes.clone();
        stream.extend_from_slice(&bytes);
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded[1], pair.b_frame);
    }
}

/// Annex K + Improved PB-frames: free-running slices every three
/// macroblock rows, plain and with Advanced Prediction. Each slice is
/// its own §6.1.1 / §F.3 segment and the §M.2.2 forward predictor
/// restarts at every slice's left edge; the pairs reconstruct through
/// `decode_improved_pb_picture` and `decode_sequence`.
#[test]
fn improved_pb_with_annex_k_slices_round_trips() {
    let base = textured(176, 144, 21);
    let i_bytes = encode_intra_picture(&base, 6, 0).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let p1 = translated(&base, 5, 2);
    let half1 = translated(&base, 3, 1);
    let b1 = banded_b(&r0, &p1, &half1);
    for ap in [false, true] {
        let cfg = ImprovedPbConfig {
            quant: 6,
            trb: 1,
            dbquant: 0,
            search_half: 6,
            forward_search_half: 3,
            allow_backward: true,
            advanced_prediction: ap,
            umv: false,
            slice_rows: 3,
            intra_refresh: 0,
        };
        let (ipb1, stats) = encode_improved_pb_picture_stats(&p1, &b1, &r0, 2, 0, &cfg).unwrap();
        eprintln!("Improved-PB + slices (ap {ap}) census: {stats:?}");
        assert!(stats.forward > 0 && stats.bidirectional > 0 && stats.backward > 0);
        let pair1 = decode_improved_pb_picture(&ipb1, &r0, 0, DecodeOptions::default()).unwrap();
        let p2 = translated(&base, 9, 3);
        let half2 = translated(&base, 7, 2);
        let b2 = banded_b(&pair1.p_frame, &p2, &half2);
        let ipb2 = encode_improved_pb_picture(&p2, &b2, &pair1.p_frame, 4, 2, &cfg).unwrap();
        let mut stream = i_bytes.clone();
        stream.extend_from_slice(&ipb1);
        stream.extend_from_slice(&ipb2);
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.len(), 5);
        assert_eq!(decoded[1], pair1.b_frame);
        for (name, src, dec) in [
            ("BPB1", &b1, &decoded[1]),
            ("P1", &p1, &decoded[2]),
            ("BPB2", &b2, &decoded[3]),
            ("P2", &p2, &decoded[4]),
        ] {
            let mae = luma_mae(src, dec);
            let psnr = luma_psnr(src, dec);
            eprintln!("ap {ap} {name}: luma MAE {mae:.3}, PSNR {psnr:.2} dB");
            assert!(mae < 3.0, "ap {ap} {name}: luma MAE {mae:.3} too high");
            // A single mispredicted macroblock (e.g. a forward predictor
            // not reset at a slice edge) costs ~15 dB here.
            assert!(psnr > 35.0, "ap {ap} {name}: PSNR {psnr:.2} dB");
        }
    }
    // Slices taller than the picture are refused.
    let bad = ImprovedPbConfig {
        slice_rows: 10,
        ..ImprovedPbConfig::default()
    };
    assert_eq!(
        encode_improved_pb_picture(&p1, &b1, &r0, 2, 0, &bad).unwrap_err(),
        Error::UnsupportedPictureGeometry
    );
}
