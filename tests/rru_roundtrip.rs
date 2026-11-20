//! End-to-end **Annex Q Reduced-Resolution Update** integration tests:
//! the RRU picture encoders driven back through the self-describing
//! PLUSPTYPE decode path (`decode_picture_layer` / `decode_sequence`),
//! plus the §Q.4 pseudo-motion-vector lattice pins.

use oxideav_h263::encoder::{
    encode_inter_picture_rru, encode_inter_picture_rru_deblock, encode_inter_picture_rru_umv,
    encode_intra_picture_plus, encode_intra_picture_rru, encode_intra_picture_rru_deblock,
};
use oxideav_h263::encoder_rru::{
    RruInterConfig, encode_inter_picture_rru_stats, encode_inter_picture_rru_with,
};
use oxideav_h263::motion::{rru_actual_component, rru_pseudo_component};
use oxideav_h263::picture::{DecodeOptions, YuvFrame, decode_picture_layer, decode_sequence};
use oxideav_h263::{Error, ModeRule};

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

/// §5.1.4.5 items 1 / 4 — "the Reduced-Resolution Update mode bit
/// shall not be set in an INTRA picture": the encoder refuses with the
/// typed rule instead of emitting a non-conformant picture.
#[test]
fn rru_intra_picture_is_refused_with_the_rule() {
    let src = gradient(176, 144, 3);
    for r in [
        encode_intra_picture_rru(&src, 6, 0),
        encode_intra_picture_rru_deblock(&src, 6, 0),
    ] {
        assert_eq!(
            r.unwrap_err(),
            Error::ModeCombination(ModeRule::RruInIntraPicture)
        );
    }
    assert_eq!(ModeRule::RruInIntraPicture.clause(), "5.1.4.5 item 4");
}

/// A textured frame: gradient plus a per-16-px checkerboard and a
/// moving bright square, so INTER4V and INTRA decisions have something
/// to bite on.
fn textured(lw: usize, lh: usize, t: usize) -> YuvFrame {
    let mut f = gradient(lw, lh, (t * 2) as u8);
    for r in 0..lh {
        for c in 0..lw {
            let v = &mut f.y[r * lw + c];
            if ((r / 8) + ((c + 3 * t) / 8)).is_multiple_of(2) {
                *v = v.saturating_add(18);
            }
            let (sx, sy) = (40 + 7 * t, 30 + 5 * t);
            if (sx..sx + 28).contains(&c) && (sy..sy + 28).contains(&r) {
                *v = 230;
            }
        }
    }
    f
}

fn decode_p(bytes: &[u8], reference: &YuvFrame) -> YuvFrame {
    decode_picture_layer(bytes, Some(reference), DecodeOptions::default()).expect("decode P")
}

/// INTRA macroblocks inside an RRU P-picture (the legal home of
/// reduced-resolution INTRA texture): content unrelated to the
/// reference is coded INTRA and round-trips within the low-pass
/// budget, at CIF (coded == reference size) and QCIF (§Q.3 extension
/// + §Q.2.3 crop).
#[test]
fn rru_p_picture_intra_macroblocks_round_trip() {
    for (lw, lh) in [(352usize, 288usize), (176, 144)] {
        let flat = YuvFrame {
            y: vec![16; lw * lh],
            cb: vec![128; lw * lh / 4],
            cr: vec![128; lw * lh / 4],
            luma_width: lw,
            luma_height: lh,
        };
        let i_bytes = encode_intra_picture_plus(&flat, 6, 0).unwrap();
        let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).unwrap();
        let src = gradient(lw, lh, 3);
        let cfg = RruInterConfig::new(6).with_intra_decision(true);
        let (bytes, stats) = encode_inter_picture_rru_stats(&src, &recon, 1, &cfg).unwrap();
        assert!(stats.intra > 0, "{lw}x{lh}: no INTRA macroblock chosen");
        let dec = decode_p(&bytes, &recon);
        assert_eq!((dec.luma_width, dec.luma_height), (lw, lh));
        let mae = luma_mae(&src, &dec);
        assert!(mae < 6.0, "{lw}x{lh} RRU P/INTRA luma MAE {mae}");
    }
}

/// Every mode combination of the general RRU P encoder decodes, stays
/// source-close, and — pinned by a second decode — reconstructs
/// identically through `decode_sequence`. Covers RRU + Advanced
/// Prediction (§Q.5 enlarged OBMC, one and four vectors), RRU + Annex
/// K slices, the two together, and their UMV / Deblocking Filter
/// compositions.
#[test]
fn rru_mode_matrix_round_trips() {
    let base = RruInterConfig::new(6).with_search_pseudo_half(6);
    let cases: Vec<(&str, RruInterConfig)> = vec![
        ("plain", base),
        ("ap", base.with_advanced_prediction(true)),
        (
            "ap+4mv",
            base.with_advanced_prediction(true).with_four_mv(true),
        ),
        ("df+4mv", base.with_deblock(true).with_four_mv(true)),
        ("slices", base.with_slice_rows(1)),
        ("slices2", base.with_slice_rows(2)),
        (
            "ap+4mv+slices",
            base.with_advanced_prediction(true)
                .with_four_mv(true)
                .with_slice_rows(1),
        ),
        (
            "ap+4mv+umv",
            base.with_advanced_prediction(true)
                .with_four_mv(true)
                .with_umv(true),
        ),
        (
            "ap+4mv+umv+df+slices+intra",
            base.with_advanced_prediction(true)
                .with_four_mv(true)
                .with_umv(true)
                .with_deblock(true)
                .with_slice_rows(2)
                .with_intra_decision(true),
        ),
    ];
    for (lw, lh) in [(176usize, 144usize), (352, 288)] {
        let i_bytes = encode_intra_picture_plus(&textured(lw, lh, 0), 5, 0).unwrap();
        let recon0 = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).unwrap();
        for (name, cfg) in &cases {
            // Two chained P-pictures: the second predicts from the
            // first one's (filtered, cropped) reconstruction.
            let mut stream = i_bytes.clone();
            let mut reference = recon0.clone();
            let mut any_four = 0;
            let mut slices = 0;
            for t in 1..=2 {
                let src = textured(lw, lh, t);
                let (p, stats) =
                    encode_inter_picture_rru_stats(&src, &reference, t as u8, cfg).unwrap();
                any_four += stats.four_vector;
                slices = stats.slices;
                let dec = decode_p(&p, &reference);
                let mae = luma_mae(&src, &dec);
                assert!(mae < 9.0, "{name} {lw}x{lh} t{t}: luma MAE {mae}");
                stream.extend_from_slice(&p);
                reference = dec;
            }
            let frames = decode_sequence(&stream, DecodeOptions::default()).unwrap();
            assert_eq!(frames.len(), 3, "{name}");
            assert_eq!(frames[2], reference, "{name}: sequence decode drifted");
            if cfg.four_mv {
                assert!(
                    any_four > 0,
                    "{name} {lw}x{lh}: no INTER4V macroblock chosen"
                );
            }
            if cfg.slice_rows > 0 {
                assert_eq!(slices, (lh.div_ceil(32)).div_ceil(cfg.slice_rows), "{name}");
            }
        }
    }
}

/// A static picture is lossless in every RRU mode set: all macroblocks
/// skip (under Advanced Prediction a skipped macroblock is still
/// overlap-predicted, from zero vectors — the reference itself), and
/// no coded macroblock arms the §Q.7 filter.
#[test]
fn rru_static_p_is_lossless_in_every_mode() {
    let i_bytes = encode_intra_picture_plus(&textured(176, 144, 0), 7, 0).unwrap();
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).unwrap();
    for cfg in [
        RruInterConfig::new(7).with_advanced_prediction(true),
        RruInterConfig::new(7)
            .with_advanced_prediction(true)
            .with_four_mv(true)
            .with_slice_rows(1),
        RruInterConfig::new(7).with_slice_rows(2).with_deblock(true),
    ] {
        let (p, stats) = encode_inter_picture_rru_stats(&recon, &recon, 1, &cfg).unwrap();
        assert_eq!(stats.skipped, 30, "{cfg:?}");
        assert_eq!(decode_p(&p, &recon), recon, "{cfg:?}");
    }
}

/// §Q.5 is observable: the same vector field decoded with and without
/// the Advanced Prediction flag differs (overlapped vs plain
/// prediction), and §K / §Q.7 is observable: slicing a picture changes
/// the reconstruction only through the unfiltered slice edges and the
/// confined predictors.
#[test]
fn rru_ap_and_slices_change_the_reconstruction() {
    let i_bytes = encode_intra_picture_plus(&textured(176, 144, 0), 5, 0).unwrap();
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).unwrap();
    let src = textured(176, 144, 2);
    let base = RruInterConfig::new(5).with_search_pseudo_half(6);
    let plain = decode_p(
        &encode_inter_picture_rru_with(&src, &recon, 1, &base).unwrap(),
        &recon,
    );
    let ap = decode_p(
        &encode_inter_picture_rru_with(&src, &recon, 1, &base.with_advanced_prediction(true))
            .unwrap(),
        &recon,
    );
    let sliced = decode_p(
        &encode_inter_picture_rru_with(&src, &recon, 1, &base.with_slice_rows(1)).unwrap(),
        &recon,
    );
    assert_ne!(plain.y, ap.y);
    assert_ne!(plain.y, sliced.y);
}

/// §F.2 — four vectors need Advanced Prediction or Deblocking Filter.
#[test]
fn rru_four_vectors_without_ap_or_df_is_refused() {
    let f = gradient(176, 144, 1);
    let cfg = RruInterConfig::new(6).with_four_mv(true);
    assert_eq!(
        encode_inter_picture_rru_with(&f, &f, 1, &cfg).unwrap_err(),
        Error::ModeCombination(ModeRule::FourVectorsWithoutApOrDf)
    );
}

/// Annex J with RRU (§Q.7.2): the deblocking-filter variant of the
/// block boundary filter — the §J.3 four-tap filter with STRENGTH = +∞
/// on the 16 × 16 block edges — replaces the §Q.7.1 two-tap default.
/// Signalled in OPPTYPE by the `_deblock` encoder, or forced by
/// `DecodeOptions::deblock`; the two filters must yield different,
/// both source-close, reconstructions.
#[test]
fn rru_deblocking_filter_variant_round_trips() {
    let i_bytes = encode_intra_picture_plus(&textured(176, 144, 0), 8, 0).unwrap();
    let recon = decode_picture_layer(&i_bytes, None, DecodeOptions::default()).unwrap();
    let next = textured(176, 144, 2);
    let plain = encode_inter_picture_rru(&next, &recon, 8, 1, 6).unwrap();
    let r_plain = decode_p(&plain, &recon);
    let r_forced = decode_picture_layer(
        &plain,
        Some(&recon),
        DecodeOptions::default().with_deblock(true),
    )
    .unwrap();
    assert!(
        r_forced.y != r_plain.y,
        "§Q.7.2 and §Q.7.1 filter differently"
    );
    assert!(luma_mae(&r_forced, &next) < 9.0);

    // Signalled: P-picture through decode_sequence.
    let p = encode_inter_picture_rru_deblock(&next, &recon, 8, 1, 6).unwrap();
    let mut stream = i_bytes.clone();
    stream.extend_from_slice(&p);
    let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
    assert_eq!(decoded.len(), 2);
    let m_p = luma_mae(&decoded[1], &next);
    assert!(m_p < 9.0, "P MAE {m_p:.3}");
    // A static P is lossless under §Q.7.2 as well.
    let p_static = encode_inter_picture_rru_deblock(&recon, &recon, 8, 2, 6).unwrap();
    assert_eq!(decode_p(&p_static, &recon), recon);
}

/// A static RRU P-picture (source == decoded reference) is lossless:
/// every 32 × 32 macroblock skips, the §Q.7 filter has no coded
/// macroblock to fire on, and the reconstruction is the reference.
#[test]
fn rru_static_p_is_lossless() {
    let src = gradient(176, 144, 9);
    let i_bytes = encode_intra_picture_plus(&src, 7, 0).expect("encode I");
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
    let i_bytes = encode_intra_picture_plus(&src, 5, 0).expect("encode I");
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
    let i_bytes = encode_intra_picture_plus(&src, 6, 0).unwrap();
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
    let i_bytes = encode_intra_picture_plus(&src, 7, 0).expect("encode I");
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
    let i_bytes = encode_intra_picture_plus(&src, 5, 0).expect("encode I");
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
    let i_bytes = encode_intra_picture_plus(&src, 6, 0).unwrap();
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
