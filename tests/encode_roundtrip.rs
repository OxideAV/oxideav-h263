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

use oxideav_h263::aic::IntraMode;
use oxideav_h263::encoder::{
    encode_inter_picture, encode_inter_picture_ap, encode_inter_picture_motion,
    encode_inter_picture_umv, encode_intra_picture, encode_intra_picture_aic,
    encode_intra_picture_aic_auto, encode_intra_picture_aic_mq, encode_intra_sequence,
    encode_intra_sequence_aic, encode_pb_picture, encode_sequence, GopConfig, PbConfig, EOS_BYTES,
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

/// A mixed-mode elementary stream assembled from the round-384 encoder
/// entry points — I, then a baseline P, a UMV P, an Advanced-Prediction
/// (INTER4V + OBMC) P and an Annex G PB pair — decodes end-to-end
/// through `decode_sequence` with every frame tracking its source.
#[test]
fn mixed_mode_stream_decodes_end_to_end() {
    let f0 = gradient(176, 144, 0);
    let i_bytes = encode_intra_picture(&f0, 5, 0).unwrap();
    let r0 = decode_picture_no_gob0_header(&i_bytes, None, DecodeOptions::default()).unwrap();

    // Baseline motion P (1 px).
    let f1 = translated(&r0, 1);
    let p1 = encode_inter_picture_motion(&f1, &r0, 5, 1, 3).unwrap();
    let r1 = decode_picture_no_gob0_header(&p1, Some(&r0), DecodeOptions::default()).unwrap();

    // Annex D UMV P (2 px more).
    let f2 = translated(&r0, 3);
    let p2 = encode_inter_picture_umv(&f2, &r1, 5, 2, 4).unwrap();
    let r2 = decode_picture_no_gob0_header(&p2, Some(&r1), DecodeOptions::default()).unwrap();

    // Annex F AP / INTER4V P (1 px more).
    let f3 = translated(&r0, 4);
    let p3 = encode_inter_picture_ap(&f3, &r2, 5, 3, 3).unwrap();
    let r3 = decode_picture_no_gob0_header(&p3, Some(&r2), DecodeOptions::default()).unwrap();

    // Annex G PB pair: B at 5 px, P at 6 px (TR 3 -> 5, TRB 1).
    let fb = translated(&r0, 5);
    let fp = translated(&r0, 6);
    let cfg = PbConfig {
        quant: 5,
        trb: 1,
        dbquant: 0,
        search_half: 3,
    };
    let pb = encode_pb_picture(&fp, &fb, &r3, 5, 3, &cfg).unwrap();

    let mut stream = Vec::new();
    for part in [&i_bytes, &p1, &p2, &p3, &pb] {
        stream.extend_from_slice(part);
    }
    stream.extend_from_slice(&EOS_BYTES);

    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 6, "expected I, P, P(UMV), P(AP), B, P");
    for (i, (src, dec)) in [&f0, &f1, &f2, &f3, &fb, &fp]
        .iter()
        .zip(decoded.iter())
        .enumerate()
    {
        let mae = luma_mae(src, dec);
        assert!(mae < 8.0, "frame {i} luma MAE too high: {mae}");
    }
}

/// The closed-loop GOP driver round-trips through the public API with
/// an EOS-terminated stream.
#[test]
fn gop_driver_public_api_round_trips() {
    let base = gradient(176, 144, 10);
    let frames: Vec<YuvFrame> = (0..5).map(|k| translated(&base, k)).collect();
    let cfg = GopConfig {
        quant: 6,
        intra_period: 4,
        search_half: 2,
        umv: false,
        eos: true,
    };
    let stream = encode_sequence(&frames, &cfg, 0).unwrap();
    assert!(stream.ends_with(&EOS_BYTES));
    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 5);
    for (i, (src, dec)) in frames.iter().zip(decoded.iter()).enumerate() {
        let mae = luma_mae(src, dec);
        assert!(mae < 8.0, "GOP frame {i} luma MAE too high: {mae}");
    }
}

/// Annex I Advanced INTRA Coding encode → decode round-trip: the
/// `encode_intra_picture_aic` output decodes (with `aic` set) within the
/// lossy transform + §I.3 quantiser tolerance, for every INTRA_MODE and
/// several standard picture sizes / quantisers.
#[test]
fn aic_intra_picture_round_trips_within_tolerance() {
    let opts = DecodeOptions {
        aic: true,
        ..DecodeOptions::default()
    };
    for &mode in &[
        IntraMode::DcOnly,
        IntraMode::VerticalDcAc,
        IntraMode::HorizontalDcAc,
    ] {
        for &(lw, lh) in &[(128usize, 96usize), (176, 144)] {
            for &q in &[8u8, 13] {
                let src = gradient(lw, lh, 3);
                let bytes = encode_intra_picture_aic(&src, q, 0, mode).expect("encode AIC I");
                let decoded =
                    decode_picture_no_gob0_header(&bytes, None, opts).expect("decode AIC I");
                assert_eq!((decoded.luma_width, decoded.luma_height), (lw, lh));
                let mae = luma_mae(&src, &decoded);
                assert!(mae < 10.0, "{lw}x{lh} q{q} {mode:?} AIC luma MAE {mae}");
            }
        }
    }
}

/// A flat AIC INTRA picture reconstructs the exact grey field: with no
/// AC energy every macroblock's DC prediction chains cleanly across the
/// grid, so the round-trip is byte-exact on all three planes.
#[test]
fn aic_flat_picture_is_exact() {
    let opts = DecodeOptions {
        aic: true,
        ..DecodeOptions::default()
    };
    let src = YuvFrame::grey(176, 144);
    for &mode in &[
        IntraMode::DcOnly,
        IntraMode::VerticalDcAc,
        IntraMode::HorizontalDcAc,
    ] {
        let bytes = encode_intra_picture_aic(&src, 10, 2, mode).expect("encode AIC flat");
        let decoded = decode_picture_no_gob0_header(&bytes, None, opts).expect("decode AIC flat");
        assert!(
            decoded.y.iter().all(|&p| p == 128),
            "{mode:?} luma not flat 128"
        );
        assert!(decoded.cb.iter().all(|&p| p == 128), "{mode:?} cb not flat");
        assert!(decoded.cr.iter().all(|&p| p == 128), "{mode:?} cr not flat");
    }
}

/// The per-macroblock INTRA_MODE decision encoder round-trips within
/// tolerance and is never larger than the worst fixed-mode encoding of
/// the same frame (choosing the cheapest mode per macroblock can only
/// help). Also exercised on directional content where mode 1 / 2 pay off.
#[test]
fn aic_auto_mode_round_trips_and_is_not_worse() {
    let opts = DecodeOptions {
        aic: true,
        ..DecodeOptions::default()
    };
    // A frame with strong horizontal banding (rows constant, columns
    // vary) plus a smooth luma gradient.
    let lw = 176;
    let lh = 144;
    let mut src = gradient(lw, lh, 7);
    for row in 0..lh {
        for col in 0..lw {
            // Vertical stripes → strong horizontal AC → mode selection
            // has something to chew on.
            let v = if (col / 4) % 2 == 0 { 40 } else { 200 };
            src.y[row * lw + col] = v;
        }
    }

    let q = 10;
    let auto = encode_intra_picture_aic_auto(&src, q, 0).expect("encode auto");
    let decoded = decode_picture_no_gob0_header(&auto, None, opts).expect("decode auto");
    let mae = luma_mae(&src, &decoded);
    assert!(mae < 10.0, "auto AIC luma MAE {mae}");

    // The auto choice is <= the largest fixed-mode encoding.
    let worst_fixed = [
        IntraMode::DcOnly,
        IntraMode::VerticalDcAc,
        IntraMode::HorizontalDcAc,
    ]
    .iter()
    .map(|&m| encode_intra_picture_aic(&src, q, 0, m).unwrap().len())
    .max()
    .unwrap();
    assert!(
        auto.len() <= worst_fixed,
        "auto {} bytes worse than worst fixed {}",
        auto.len(),
        worst_fixed
    );
}

/// AIC + Annex T Modified Quantization encode → decode round-trip: the
/// stream decodes with both `aic` and `modified_quant` set (§T.3 chroma
/// QUANT_C + §T.4 EXTENDED-ESCAPE) within tolerance, across several
/// quantisers including low ones where the widened LEVEL range matters.
#[test]
fn aic_mq_intra_picture_round_trips() {
    let opts = DecodeOptions {
        aic: true,
        modified_quant: true,
        ..DecodeOptions::default()
    };
    for &q in &[3u8, 8, 16, 25] {
        let src = gradient(176, 144, 4);
        let bytes = encode_intra_picture_aic_mq(&src, q, 0).expect("encode AIC+MQ");
        let decoded = decode_picture_no_gob0_header(&bytes, None, opts).expect("decode AIC+MQ");
        let mae = luma_mae(&src, &decoded);
        assert!(mae < 10.0, "AIC+MQ q{q} luma MAE {mae}");
    }
}

/// A multi-picture AIC elementary stream splits on Picture Start Codes
/// and decodes through `decode_sequence` (with `aic` set) into all
/// frames, each within the §I.3 round-trip tolerance.
#[test]
fn aic_intra_sequence_decodes_all_frames() {
    let opts = DecodeOptions {
        aic: true,
        ..DecodeOptions::default()
    };
    let frames = vec![
        gradient(176, 144, 0),
        gradient(176, 144, 40),
        YuvFrame::grey(176, 144),
        gradient(176, 144, 90),
    ];
    let stream = encode_intra_sequence_aic(&frames, 9, 0).expect("encode AIC sequence");
    let decoded = decode_sequence(&stream, opts).expect("decode AIC sequence");
    assert_eq!(decoded.len(), frames.len());
    for (i, (src, dec)) in frames.iter().zip(decoded.iter()).enumerate() {
        assert_eq!((dec.luma_width, dec.luma_height), (176, 144));
        let mae = luma_mae(src, dec);
        assert!(mae < 10.0, "AIC sequence frame {i} luma MAE {mae}");
    }
}

/// An extended-PTYPE (PLUSPTYPE) INTRA picture from
/// `encode_intra_picture_plus` is self-describing: it decodes through
/// the wire-driven `decode_picture_layer` / `decode_sequence` entry
/// points with `DecodeOptions::default()`, and reconstructs the exact
/// same frame as the baseline-PTYPE form (the macroblock stream is
/// bit-identical, only the header differs).
#[test]
fn plus_intra_picture_is_self_describing() {
    use oxideav_h263::encoder::encode_intra_picture_plus;
    use oxideav_h263::picture::decode_picture_layer;

    let src = gradient(176, 144, 11);
    let plus = encode_intra_picture_plus(&src, 6, 3).expect("encode plus I");
    let base = encode_intra_picture(&src, 6, 3).expect("encode baseline I");

    let from_plus =
        decode_picture_layer(&plus, None, DecodeOptions::default()).expect("decode plus");
    let from_base =
        decode_picture_no_gob0_header(&base, None, DecodeOptions::default()).expect("decode base");
    assert_eq!(from_plus.y, from_base.y);
    assert_eq!(from_plus.cb, from_base.cb);
    assert_eq!(from_plus.cr, from_base.cr);

    // decode_sequence dispatches on the extended-PTYPE escape too.
    let seq = decode_sequence(&plus, DecodeOptions::default()).expect("sequence");
    assert_eq!(seq.len(), 1);
    assert_eq!(seq[0].y, from_base.y);
}

/// The on-wire AIC form (`encode_intra_picture_aic_plus`) decodes with
/// **default** options — the decoder activates §I from the OPPTYPE AIC
/// bit — and reconstructs identically to the baseline-PTYPE AIC stream
/// decoded with the explicit `aic` option.
#[test]
fn plus_aic_picture_signals_aic_on_the_wire() {
    use oxideav_h263::encoder::encode_intra_picture_aic_plus;
    use oxideav_h263::picture::decode_picture_layer;

    let src = gradient(176, 144, 27);
    let plus = encode_intra_picture_aic_plus(&src, 7, 0).expect("encode plus AIC");
    let base = encode_intra_picture_aic_auto(&src, 7, 0).expect("encode baseline AIC");

    let from_plus =
        decode_picture_layer(&plus, None, DecodeOptions::default()).expect("decode plus AIC");
    let from_base = decode_picture_no_gob0_header(
        &base,
        None,
        DecodeOptions {
            aic: true,
            ..DecodeOptions::default()
        },
    )
    .expect("decode base AIC");
    assert_eq!(from_plus.y, from_base.y);
    assert_eq!(from_plus.cb, from_base.cb);
    assert_eq!(from_plus.cr, from_base.cr);
    let mae = luma_mae(&src, &from_plus);
    assert!(mae < 10.0, "plus-AIC luma MAE {mae}");
}

/// The on-wire AIC + Modified Quantization form
/// (`encode_intra_picture_aic_mq_plus`) decodes with default options
/// (§I + §T auto-activated from OPPTYPE bits 8/14), matching the
/// explicit-options decode of the baseline-PTYPE form byte-exactly.
#[test]
fn plus_aic_mq_picture_signals_both_modes_on_the_wire() {
    use oxideav_h263::encoder::encode_intra_picture_aic_mq_plus;
    use oxideav_h263::picture::decode_picture_layer;

    for &q in &[3u8, 12, 25] {
        let src = gradient(176, 144, 4);
        let plus = encode_intra_picture_aic_mq_plus(&src, q, 9).expect("encode plus AIC+MQ");
        let base = encode_intra_picture_aic_mq(&src, q, 9).expect("encode baseline AIC+MQ");

        let from_plus = decode_picture_layer(&plus, None, DecodeOptions::default())
            .expect("decode plus AIC+MQ");
        let from_base = decode_picture_no_gob0_header(
            &base,
            None,
            DecodeOptions {
                aic: true,
                modified_quant: true,
                ..DecodeOptions::default()
            },
        )
        .expect("decode base AIC+MQ");
        assert_eq!(from_plus.y, from_base.y, "q{q} luma mismatch");
        assert_eq!(from_plus.cb, from_base.cb, "q{q} cb mismatch");
        assert_eq!(from_plus.cr, from_base.cr, "q{q} cr mismatch");
    }
}

/// A mixed extended-PTYPE elementary stream — H.263+ I-picture followed
/// by H.263+ motion P-pictures (plain and Annex D UMV signalled via
/// OPPTYPE + UUI) — decodes end-to-end through `decode_sequence` with
/// default options, threading the reference across pictures.
#[test]
fn plus_inter_sequence_round_trips() {
    use oxideav_h263::encoder::{
        encode_inter_picture_plus, encode_inter_picture_umv_plus, encode_intra_picture_plus,
    };

    let f0 = gradient(176, 144, 0);
    // Frame 1: shift the gradient (small motion).
    let f1 = gradient(176, 144, 6);
    // Frame 2: larger shift for the UMV picture.
    let f2 = gradient(176, 144, 60);

    let q = 6;
    let i0 = encode_intra_picture_plus(&f0, q, 0).expect("I");
    let r0 = decode_sequence(&i0, DecodeOptions::default()).expect("dec I")[0].clone();
    let p1 = encode_inter_picture_plus(&f1, &r0, q, 1, 4).expect("P");
    let mut stream = i0.clone();
    stream.extend_from_slice(&p1);
    let r1 = decode_sequence(&stream, DecodeOptions::default()).expect("dec I+P")[1].clone();
    let p2 = encode_inter_picture_umv_plus(&f2, &r1, q, 2, 20).expect("UMV P");
    stream.extend_from_slice(&p2);

    let frames = decode_sequence(&stream, DecodeOptions::default()).expect("dec I+P+UMV");
    assert_eq!(frames.len(), 3);
    for (i, (src, dec)) in [&f0, &f1, &f2].iter().zip(frames.iter()).enumerate() {
        let mae = luma_mae(src, dec);
        assert!(mae < 8.0, "plus sequence frame {i} luma MAE {mae}");
    }
}

/// An Annex K Slice-Structured INTRA picture at a constant quantiser
/// reconstructs **byte-exactly** the same frame as the single-segment
/// baseline encode: INTRA macroblock coding carries no cross-segment
/// state, so only the header/slice framing differs.
#[test]
fn slice_structured_intra_matches_single_segment_reconstruction() {
    use oxideav_h263::encoder::encode_intra_picture_slices;
    use oxideav_h263::picture::decode_picture_layer;

    let src = gradient(176, 144, 5);
    for rows_per_slice in [1usize, 2, 3, 4, 9] {
        let sliced =
            encode_intra_picture_slices(&src, 0, rows_per_slice, |_| 7).expect("encode slices");
        let from_sliced = decode_picture_layer(&sliced, None, DecodeOptions::default())
            .expect("decode slice-structured");
        let base = encode_intra_picture(&src, 7, 0).expect("encode base");
        let from_base = decode_picture_no_gob0_header(&base, None, DecodeOptions::default())
            .expect("decode base");
        assert_eq!(
            from_sliced.y, from_base.y,
            "rows_per_slice {rows_per_slice}"
        );
        assert_eq!(from_sliced.cb, from_base.cb);
        assert_eq!(from_sliced.cr, from_base.cr);
    }
}

/// Per-slice SQUANT drives real quantiser changes: a picture whose top
/// half runs fine (q=2) and bottom half coarse (q=28) shows lower
/// reconstruction error in the fine-quantised slices, and the whole
/// picture stays within the INTRA round-trip tolerance.
#[test]
fn slice_structured_intra_per_slice_squant() {
    use oxideav_h263::encoder::encode_intra_picture_slices;
    use oxideav_h263::picture::decode_picture_layer;

    let src = gradient(176, 144, 33);
    // 9 MB rows; 3-row slices → slices 0..=2 at q 2 / 20 / 28.
    let quants = [2u8, 20, 28];
    let bytes =
        encode_intra_picture_slices(&src, 4, 3, |s| quants[s.min(2)]).expect("encode slices");
    let decoded =
        decode_picture_layer(&bytes, None, DecodeOptions::default()).expect("decode slices");

    // Mean absolute luma error per slice band (48 luma rows each).
    let band_mae = |band: usize| -> f64 {
        let lw = src.luma_width;
        let rows = 48;
        let y0 = band * rows;
        let mut sum = 0u64;
        for row in y0..y0 + rows {
            for col in 0..lw {
                let a = src.y[row * lw + col] as i32;
                let b = decoded.y[row * lw + col] as i32;
                sum += (a - b).unsigned_abs() as u64;
            }
        }
        sum as f64 / (rows * lw) as f64
    };
    let fine = band_mae(0);
    let coarse = band_mae(2);
    assert!(
        fine < coarse,
        "fine-slice MAE {fine} should beat coarse-slice MAE {coarse}"
    );
    assert!(fine < 1.5, "q=2 band MAE {fine}");
    assert!(coarse < 12.0, "q=28 band MAE {coarse}");
}

/// A multi-picture stream of Slice-Structured INTRA pictures decodes
/// through `decode_sequence` with default options (extended-PTYPE
/// dispatch + Annex K routing per picture).
#[test]
fn slice_structured_intra_sequence_decodes() {
    use oxideav_h263::encoder::encode_intra_picture_slices;

    let frames = [gradient(128, 96, 0), gradient(128, 96, 50)];
    let mut stream = Vec::new();
    for (i, f) in frames.iter().enumerate() {
        let pic = encode_intra_picture_slices(f, i as u8, 2, |_| 9).expect("encode");
        stream.extend_from_slice(&pic);
    }
    let decoded = decode_sequence(&stream, DecodeOptions::default()).expect("decode sequence");
    assert_eq!(decoded.len(), 2);
    for (src, dec) in frames.iter().zip(decoded.iter()) {
        let mae = luma_mae(src, dec);
        assert!(mae < 8.0, "slice sequence MAE {mae}");
    }
}

/// A whole-picture single slice AIC encode is bit-equivalent in its
/// macroblock stream to the non-slice on-wire AIC encode: with
/// `mb_rows_per_slice == mb_rows` there is exactly one segment, so the
/// §I.3 neighbour availability matches the single-segment picture and
/// the reconstruction must be byte-exact between the two forms.
#[test]
fn slice_structured_aic_single_slice_matches_plus_aic() {
    use oxideav_h263::encoder::{encode_intra_picture_aic_plus, encode_intra_picture_slices_aic};
    use oxideav_h263::picture::decode_picture_layer;

    let src = gradient(176, 144, 21);
    let sliced = encode_intra_picture_slices_aic(&src, 6, 2, 9).expect("encode AIC slices");
    let plain = encode_intra_picture_aic_plus(&src, 6, 2).expect("encode AIC plus");
    let from_sliced =
        decode_picture_layer(&sliced, None, DecodeOptions::default()).expect("decode AIC slices");
    let from_plain =
        decode_picture_layer(&plain, None, DecodeOptions::default()).expect("decode AIC plus");
    assert_eq!(from_sliced.y, from_plain.y);
    assert_eq!(from_sliced.cb, from_plain.cb);
    assert_eq!(from_sliced.cr, from_plain.cr);
}

/// Multi-slice AIC (+ MQ) pictures decode with default options within
/// a tight tolerance at fine quantisation — a segment-availability
/// mismatch between encoder and decoder would corrupt the §I.3 DC/AC
/// prediction at every slice top and blow the bound.
#[test]
fn slice_structured_aic_mq_round_trips() {
    use oxideav_h263::encoder::{
        encode_intra_picture_slices_aic, encode_intra_picture_slices_aic_mq,
    };
    use oxideav_h263::picture::decode_picture_layer;

    let src = gradient(176, 144, 13);
    for rows_per_slice in [1usize, 2, 3, 4] {
        let aic =
            encode_intra_picture_slices_aic(&src, 4, 0, rows_per_slice).expect("encode AIC slices");
        let dec =
            decode_picture_layer(&aic, None, DecodeOptions::default()).expect("decode AIC slices");
        let mae = luma_mae(&src, &dec);
        assert!(mae < 3.0, "AIC slices ({rows_per_slice} rows) MAE {mae}");

        let mq = encode_intra_picture_slices_aic_mq(&src, 4, 0, rows_per_slice)
            .expect("encode AIC+MQ slices");
        let dec = decode_picture_layer(&mq, None, DecodeOptions::default())
            .expect("decode AIC+MQ slices");
        let mae = luma_mae(&src, &dec);
        assert!(mae < 3.0, "AIC+MQ slices ({rows_per_slice} rows) MAE {mae}");
    }
}

/// A flat AIC + MQ slice-structured picture reconstructs byte-exactly
/// (no AC, the §I.3 closed loop is lossless on flat content), proving
/// the whole slice framing + AIC + MQ pipeline is bit-consistent.
#[test]
fn slice_structured_aic_mq_flat_is_exact() {
    use oxideav_h263::encoder::encode_intra_picture_slices_aic_mq;
    use oxideav_h263::picture::decode_picture_layer;

    let src = YuvFrame::grey(128, 96);
    let bytes = encode_intra_picture_slices_aic_mq(&src, 14, 5, 2).expect("encode");
    let dec = decode_picture_layer(&bytes, None, DecodeOptions::default()).expect("decode");
    assert!(dec.y.iter().all(|&p| p == 128));
    assert!(dec.cb.iter().all(|&p| p == 128));
    assert!(dec.cr.iter().all(|&p| p == 128));
}

/// A translated-content P-picture in Annex K Slice-Structured mode:
/// motion estimation + per-slice segment predictor replay round-trips
/// through the wire-driven decoder with default options, for several
/// slice heights, and a whole-picture single slice reconstructs
/// byte-exactly like the single-segment H.263+ P-picture.
#[test]
fn slice_structured_inter_round_trips() {
    use oxideav_h263::encoder::{encode_inter_picture_plus, encode_inter_picture_slices};
    use oxideav_h263::picture::decode_picture_layer;

    // Reference: gradient. Current: the same gradient shifted right by
    // 3 pixels and down by 1 (global motion).
    let lw = 176;
    let lh = 144;
    let reference = gradient(lw, lh, 10);
    let mut cur = reference.clone();
    for row in 0..lh {
        for col in 0..lw {
            let sr = row.saturating_sub(1);
            let sc = col.saturating_sub(3);
            cur.y[row * lw + col] = reference.y[sr * lw + sc];
        }
    }
    let cw = lw / 2;
    let chh = lh / 2;
    for row in 0..chh {
        for col in 0..cw {
            let sr = row.saturating_sub(1);
            let sc = col.saturating_sub(2);
            cur.cb[row * cw + col] = reference.cb[sr * cw + sc];
            cur.cr[row * cw + col] = reference.cr[sr * cw + sc];
        }
    }

    // Decode the reference I-picture first so both paths predict from
    // the decoder-reconstructed anchor.
    let i_bytes =
        oxideav_h263::encoder::encode_intra_picture_plus(&reference, 5, 0).expect("encode I");
    let anchor = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).expect("decode I");

    for rows_per_slice in [1usize, 3, 9] {
        let p_bytes = encode_inter_picture_slices(&cur, &anchor, 5, 1, 6, rows_per_slice)
            .expect("encode P slices");
        let dec = decode_picture_layer(&p_bytes, Some(&anchor), DecodeOptions::default())
            .expect("decode P slices");
        let mae = luma_mae(&cur, &dec);
        assert!(mae < 4.0, "P slices ({rows_per_slice} rows) MAE {mae}");
    }

    // Single whole-picture slice == single segment: byte-exact parity
    // with the non-slice H.263+ P-picture.
    let sliced = encode_inter_picture_slices(&cur, &anchor, 5, 1, 6, 9).expect("slices");
    let plain = encode_inter_picture_plus(&cur, &anchor, 5, 1, 6).expect("plain");
    let a = decode_picture_layer(&sliced, Some(&anchor), DecodeOptions::default()).unwrap();
    let b = decode_picture_layer(&plain, Some(&anchor), DecodeOptions::default()).unwrap();
    assert_eq!(a.y, b.y);
    assert_eq!(a.cb, b.cb);
    assert_eq!(a.cr, b.cr);
}

/// A full slice-structured GOP — H.263+ AIC I-picture in slices
/// followed by two motion P-pictures in slices — decodes end-to-end
/// through `decode_sequence` with default options (closed loop: each
/// P predicts from the decoder's reconstruction).
#[test]
fn slice_structured_gop_decodes_end_to_end() {
    use oxideav_h263::encoder::{encode_inter_picture_slices, encode_intra_picture_slices_aic};

    let frames = [
        gradient(128, 96, 0),
        gradient(128, 96, 8),
        gradient(128, 96, 16),
    ];
    let q = 5;
    let mut stream = encode_intra_picture_slices_aic(&frames[0], q, 0, 2).expect("I slices");
    let mut recon = decode_sequence(&stream, DecodeOptions::default()).expect("dec I");
    for (i, f) in frames.iter().enumerate().skip(1) {
        let p = encode_inter_picture_slices(f, recon.last().unwrap(), q, i as u8, 5, 3)
            .expect("P slices");
        stream.extend_from_slice(&p);
        recon = decode_sequence(&stream, DecodeOptions::default()).expect("dec seq");
    }
    assert_eq!(recon.len(), 3);
    for (i, (src, dec)) in frames.iter().zip(recon.iter()).enumerate() {
        let mae = luma_mae(src, dec);
        assert!(mae < 6.0, "slice GOP frame {i} MAE {mae}");
    }
}
