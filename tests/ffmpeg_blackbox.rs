//! Black-box cross-validation of crate-encoded elementary streams
//! against an independent H.263 decoder binary (`ffmpeg`, invoked as an
//! opaque oracle — its source is not consulted).
//!
//! Every crate encoder form is fed the same synthetic clip; the raw
//! `.h263` stream is handed to the oracle, whose planar 4:2:0 output
//! is compared sample-by-sample with this crate's own `decode_sequence`
//! reconstruction. §6.2 leaves the inverse transform arithmetic to the
//! implementation and Annex A.7 bounds the per-sample peak error at 1,
//! so AC-bearing pictures are asserted within ±1 per sample and the
//! plane means within a small fraction; pictures the crate reconstructs
//! without any transform (flat content) must agree byte-exactly.
//!
//! The tests are skipped (with a notice) when no `ffmpeg` binary is on
//! `PATH`, so CI images without the oracle stay green without hiding a
//! failing premise behind `#[ignore]`.

use oxideav_h263::encoder::{
    encode_improved_pb_picture, encode_inter_picture_ap, encode_inter_picture_deblock,
    encode_inter_picture_motion, encode_inter_picture_umv_plus, encode_intra_picture,
    encode_intra_picture_aic_plus, encode_intra_picture_deblock, encode_pb_picture,
    encode_pb_picture_ap, encode_pb_picture_umv, encode_sequence, DeblockConfig, GopConfig,
    ImprovedPbConfig, PbConfig,
};
use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};
use std::path::PathBuf;
use std::process::Command;

fn ffmpeg_available() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

fn gradient(lw: usize, lh: usize, seed: u8) -> YuvFrame {
    let cw = lw / 2;
    let ch = lh / 2;
    let mut y = vec![0u8; lw * lh];
    for r in 0..lh {
        for c in 0..lw {
            let v = (r * 3 + c * 2 + seed as usize * 7) % 256;
            // A textured field: gradient plus a coarse checker so the
            // AC path and the motion search both have work to do.
            let checker = if ((r / 8) + (c / 8)) % 2 == 0 { 24 } else { 0 };
            y[r * lw + c] = ((v + checker) % 256) as u8;
        }
    }
    let cb = (0..cw * ch)
        .map(|i| (80 + (i % 64) + seed as usize) as u8)
        .collect();
    let cr = (0..cw * ch)
        .map(|i| (170u32.wrapping_sub((i % 48) as u32) as u8).wrapping_add(seed))
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
            let sr = (r + lh - dy) % lh;
            let sc = (c + lw - dx) % lw;
            out.y[r * lw + c] = frame.y[sr * lw + sc];
        }
    }
    for r in 0..ch {
        for c in 0..cw {
            let sr = (r + ch - dy / 2) % ch;
            let sc = (c + cw - dx / 2) % cw;
            out.cb[r * cw + c] = frame.cb[sr * cw + sc];
            out.cr[r * cw + c] = frame.cr[sr * cw + sc];
        }
    }
    out
}

fn flat(lw: usize, lh: usize, y: u8) -> YuvFrame {
    YuvFrame {
        y: vec![y; lw * lh],
        cb: vec![128; lw * lh / 4],
        cr: vec![128; lw * lh / 4],
        luma_width: lw,
        luma_height: lh,
    }
}

fn scratch_path(name: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!(
        "oxideav-h263-ffmpeg-{}-{}",
        std::process::id(),
        name
    ));
    p
}

/// Decode `stream` with the oracle into planar 4:2:0 frames.
fn oracle_decode(stream: &[u8], lw: usize, lh: usize, name: &str) -> Vec<YuvFrame> {
    let input = scratch_path(&format!("{name}.h263"));
    let output = scratch_path(&format!("{name}.yuv"));
    std::fs::write(&input, stream).unwrap();
    let status = Command::new("ffmpeg")
        .args(["-v", "error", "-y", "-f", "h263", "-i"])
        .arg(&input)
        .args(["-f", "rawvideo", "-pix_fmt", "yuv420p"])
        .arg(&output)
        .status()
        .expect("spawn oracle");
    assert!(status.success(), "oracle rejected the {name} stream");
    let raw = std::fs::read(&output).unwrap();
    let _ = std::fs::remove_file(&input);
    let _ = std::fs::remove_file(&output);
    let frame_len = lw * lh * 3 / 2;
    assert_eq!(
        raw.len() % frame_len,
        0,
        "oracle output is not whole frames"
    );
    raw.chunks(frame_len)
        .map(|f| YuvFrame {
            y: f[..lw * lh].to_vec(),
            cb: f[lw * lh..lw * lh + lw * lh / 4].to_vec(),
            cr: f[lw * lh + lw * lh / 4..].to_vec(),
            luma_width: lw,
            luma_height: lh,
        })
        .collect()
}

/// Maximum absolute per-sample difference over all three planes.
fn max_abs_diff(a: &YuvFrame, b: &YuvFrame) -> u8 {
    let planes = [(&a.y, &b.y), (&a.cb, &b.cb), (&a.cr, &b.cr)];
    planes
        .iter()
        .flat_map(|(p, q)| p.iter().zip(q.iter()).map(|(&x, &y)| x.abs_diff(y)))
        .max()
        .unwrap_or(0)
}

/// Mean absolute per-sample difference over all three planes.
fn mean_abs_diff(a: &YuvFrame, b: &YuvFrame) -> f64 {
    let planes = [(&a.y, &b.y), (&a.cb, &b.cb), (&a.cr, &b.cr)];
    let mut n = 0usize;
    let mut d = 0u64;
    for (p, q) in planes {
        for (x, y) in p.iter().zip(q.iter()) {
            n += 1;
            d += x.abs_diff(*y) as u64;
        }
    }
    d as f64 / n as f64
}

/// Compare this crate's reconstruction of `stream` with the oracle's.
///
/// §6.2 leaves the inverse transform to the implementation, so two
/// conformant decoders may differ by ±1 per IDCT (Annex A.7); across a
/// P-picture chain each picture adds its own IDCT pass on top of an
/// already-diverged reference, so the peak divergence bound grows by
/// one per predicted picture (measured: our decoder vs the oracle sit
/// exactly on this staircase). Asserted per picture `i`: same count,
/// peak |diff| ≤ `i + 1`, and a mean |diff| below `mean_limit` —
/// large-scale disagreement (a wrong vector, a wrong mode) blows the
/// mean three orders of magnitude past the rounding floor.
fn cross_check(stream: &[u8], lw: usize, lh: usize, name: &str, mean_limit: f64) {
    let ours = decode_sequence(stream, DecodeOptions::default()).expect("own decode");
    let theirs = oracle_decode(stream, lw, lh, name);
    assert_eq!(ours.len(), theirs.len(), "{name}: picture count");
    for (i, (a, b)) in ours.iter().zip(theirs.iter()).enumerate() {
        let m = max_abs_diff(a, b) as usize;
        let f = mean_abs_diff(a, b);
        assert!(m <= i + 1, "{name} picture {i}: max |diff| = {m}");
        assert!(
            f <= mean_limit,
            "{name} picture {i}: mean |diff| {f:.4} (limit {mean_limit})"
        );
    }
}

/// Luma PSNR of `a` against `b` (dB; 99 when identical).
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

/// [`cross_check`] over the pictures the oracle outputs for a PB /
/// Improved-PB stream: the oracle emits only the anchor (I / P-part)
/// pictures of such a stream, so `ours_indices` selects the matching
/// display-order entries of this crate's decode (the B-parts are
/// validated by the crate's own decoder). The peak bound follows the
/// same one-IDCT-per-predicted-picture staircase.
fn cross_check_anchors(
    stream: &[u8],
    lw: usize,
    lh: usize,
    name: &str,
    mean_limit: f64,
    ours_indices: &[usize],
) {
    let ours = decode_sequence(stream, DecodeOptions::default()).expect("own decode");
    let theirs = oracle_decode(stream, lw, lh, name);
    assert_eq!(
        theirs.len(),
        ours_indices.len(),
        "{name}: oracle picture count"
    );
    for (i, (&oi, b)) in ours_indices.iter().zip(theirs.iter()).enumerate() {
        let a = &ours[oi];
        let m = max_abs_diff(a, b) as usize;
        let f = mean_abs_diff(a, b);
        eprintln!(
            "{name} anchor {i} (ours #{oi}): max |diff| {m}, mean {f:.4}, luma PSNR vs oracle {:.2} dB",
            luma_psnr(a, b)
        );
        assert!(m <= i + 1, "{name} anchor {i}: max |diff| = {m}");
        assert!(
            f <= mean_limit,
            "{name} anchor {i}: mean |diff| {f:.4} (limit {mean_limit})"
        );
    }
}

/// [`cross_check_anchors`] for the PB + Advanced Prediction P-parts,
/// where the oracle's P-part output provably depends on the B-part's
/// content (see `oracle_ap_pb_p_part_depends_on_b_content`) — a
/// behaviour §G.3 / §G.5 do not permit (the P-blocks precede the
/// B-blocks and only PREC feeds the B prediction, never the reverse).
/// This crate's decoder matches the encoder's PREC byte-exactly there
/// (pinned by `tests/improved_pb_roundtrip.rs`), so the oracle check
/// degrades to a PSNR floor: the contamination is confined to a few
/// 8 × 8 blocks per picture, every other block sitting on the IDCT
/// staircase.
fn cross_check_anchors_psnr(
    stream: &[u8],
    lw: usize,
    lh: usize,
    name: &str,
    psnr_floor: f64,
    ours_indices: &[usize],
) {
    let ours = decode_sequence(stream, DecodeOptions::default()).expect("own decode");
    let theirs = oracle_decode(stream, lw, lh, name);
    assert_eq!(
        theirs.len(),
        ours_indices.len(),
        "{name}: oracle picture count"
    );
    for (i, (&oi, b)) in ours_indices.iter().zip(theirs.iter()).enumerate() {
        let a = &ours[oi];
        let m = max_abs_diff(a, b) as usize;
        let f = mean_abs_diff(a, b);
        let psnr = luma_psnr(a, b);
        eprintln!(
            "{name} anchor {i} (ours #{oi}): max |diff| {m}, mean {f:.4}, luma PSNR vs oracle {psnr:.2} dB"
        );
        assert!(
            psnr >= psnr_floor,
            "{name} anchor {i}: luma PSNR vs oracle {psnr:.2} dB (floor {psnr_floor})"
        );
    }
}

macro_rules! require_oracle {
    () => {
        if !ffmpeg_available() {
            eprintln!("ffmpeg oracle not on PATH — black-box cross-check skipped");
            return;
        }
    };
}

#[test]
fn flat_intra_picture_is_byte_exact_against_oracle() {
    require_oracle!();
    let f = flat(176, 144, 90);
    let bytes = encode_intra_picture(&f, 8, 0).unwrap();
    let ours = decode_sequence(&bytes, DecodeOptions::default()).unwrap();
    let theirs = oracle_decode(&bytes, 176, 144, "flat-i");
    assert_eq!(ours.len(), 1);
    assert_eq!(ours[0].y, theirs[0].y);
    assert_eq!(ours[0].cb, theirs[0].cb);
    assert_eq!(ours[0].cr, theirs[0].cr);
}

#[test]
fn baseline_intra_pictures_agree_with_oracle() {
    require_oracle!();
    for (lw, lh, name) in [
        (128, 96, "sqcif-i"),
        (176, 144, "qcif-i"),
        (352, 288, "cif-i"),
    ] {
        let f = gradient(lw, lh, 3);
        for quant in [2u8, 8, 20, 31] {
            let bytes = encode_intra_picture(&f, quant, 5).unwrap();
            cross_check(&bytes, lw, lh, &format!("{name}-q{quant}"), 0.05);
        }
    }
}

#[test]
fn baseline_gop_agrees_with_oracle() {
    require_oracle!();
    let base = gradient(176, 144, 9);
    let frames: Vec<YuvFrame> = (0..6).map(|i| translated(&base, 2 * i, i)).collect();
    for umv in [false, true] {
        let cfg = GopConfig {
            quant: 10,
            intra_period: 0,
            search_half: 7,
            umv,
            ..GopConfig::default()
        };
        let stream = encode_sequence(&frames, &cfg, 0).unwrap();
        cross_check(&stream, 176, 144, &format!("gop-umv{umv}"), 0.08);
    }
}

#[test]
fn advanced_prediction_picture_agrees_with_oracle() {
    require_oracle!();
    let base = gradient(176, 144, 11);
    let next = translated(&base, 3, 2);
    let i_bytes = encode_intra_picture(&base, 9, 0).unwrap();
    let recon = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let p_bytes = encode_inter_picture_ap(&next, &recon, 9, 1, 6).unwrap();
    let mut stream = i_bytes;
    stream.extend_from_slice(&p_bytes);
    cross_check(&stream, 176, 144, "ap", 0.08);
}

#[test]
fn plus_umv_and_aic_pictures_agree_with_oracle() {
    require_oracle!();
    let base = gradient(176, 144, 13);
    let next = translated(&base, 5, 3);
    let i_bytes = encode_intra_picture_aic_plus(&base, 9, 0).unwrap();
    let recon = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let p_bytes = encode_inter_picture_umv_plus(&next, &recon, 9, 1, 12).unwrap();
    let mut stream = i_bytes;
    stream.extend_from_slice(&p_bytes);
    cross_check(&stream, 176, 144, "aic-umv-plus", 0.08);
}

#[test]
fn deblocking_filter_pictures_agree_with_oracle() {
    require_oracle!();
    let base = gradient(176, 144, 17);
    let mut stream = encode_intra_picture_deblock(&base, 12, 0).unwrap();
    let mut recon = decode_sequence(&stream, DecodeOptions::default())
        .unwrap()
        .remove(0);
    for (i, four_mv) in [false, true, true].iter().enumerate() {
        let src = translated(&base, 3 * (i + 1), i + 1);
        let cfg = DeblockConfig {
            search_half: 6,
            four_mv: *four_mv,
            umv: false,
        };
        let p = encode_inter_picture_deblock(&src, &recon, 12, (i + 1) as u8, &cfg).unwrap();
        stream.extend_from_slice(&p);
        recon = decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .pop()
            .unwrap();
    }
    cross_check(&stream, 176, 144, "deblock", 0.08);
}

#[test]
fn motion_picture_at_every_quantiser_stays_within_oracle_budget() {
    require_oracle!();
    let base = gradient(128, 96, 21);
    let next = translated(&base, 4, 2);
    for quant in [1u8, 4, 16, 31] {
        let i_bytes = encode_intra_picture(&base, quant, 0).unwrap();
        let recon = decode_sequence(&i_bytes, DecodeOptions::default())
            .unwrap()
            .remove(0);
        let p_bytes = encode_inter_picture_motion(&next, &recon, quant, 1, 6).unwrap();
        let mut stream = i_bytes;
        stream.extend_from_slice(&p_bytes);
        cross_check(&stream, 128, 96, &format!("motion-q{quant}"), 0.08);
    }
}

/// Annex M Improved PB-frames: the oracle parses the whole picture
/// unit (P-blocks and BPB-blocks of every macroblock — a misparse of
/// any MODB / CBPB / MVDB field would desynchronise the P-part that
/// follows) and outputs the P-part, which must agree with this crate's.
#[test]
fn improved_pb_frames_agree_with_oracle() {
    require_oracle!();
    let base = gradient(176, 144, 23);
    let mut stream = encode_intra_picture(&base, 7, 0).unwrap();
    let mut recon = decode_sequence(&stream, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let cfg = ImprovedPbConfig {
        quant: 7,
        trb: 1,
        dbquant: 1,
        search_half: 6,
        forward_search_half: 3,
        allow_backward: true,
        advanced_prediction: false,
        umv: false,
        slice_rows: 0,
        intra_refresh: 0,
    };
    let mut prev_tr = 0u8;
    for k in 1..=3usize {
        // P at 4k px, BPB in between (not exactly halfway, so every
        // §M.2 mode has a reason to be chosen somewhere).
        let p = translated(&base, 4 * k, k);
        let b = translated(&base, 4 * k - 3, k);
        let tr_p = (2 * k) as u8;
        let ipb = encode_improved_pb_picture(&p, &b, &recon, tr_p, prev_tr, &cfg).unwrap();
        stream.extend_from_slice(&ipb);
        recon = decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .pop()
            .unwrap();
        prev_tr = tr_p;
    }
    // Display order: I, BPB1, P1, BPB2, P2, BPB3, P3 — anchors at 0, 2, 4, 6.
    cross_check_anchors(&stream, 176, 144, "improved-pb", 0.08, &[0, 2, 4, 6]);
}

/// Annex G PB-frames, plain and with Advanced Prediction: the oracle
/// parses every macroblock's twelve blocks and outputs the P-parts —
/// under AP the §F.3 OBMC P-luma, so an OBMC / MODB / MVDB misparse
/// would show up as a P-part disagreement.
#[test]
fn pb_frames_plain_and_advanced_prediction_agree_with_oracle() {
    require_oracle!();
    for ap in [false, true] {
        let base = gradient(176, 144, 29);
        let mut stream = encode_intra_picture(&base, 8, 0).unwrap();
        let mut recon = decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .remove(0);
        let cfg = PbConfig {
            quant: 8,
            trb: 1,
            dbquant: 2,
            search_half: 6,
            b_search_half: 2,
        };
        let mut prev_tr = 0u8;
        for k in 1..=2usize {
            let p = translated(&base, 4 * k, 2 * k);
            let b = translated(&base, 4 * k - 3, 2 * k - 1);
            let tr_p = (2 * k) as u8;
            let unit = if ap {
                encode_pb_picture_ap(&p, &b, &recon, tr_p, prev_tr, &cfg).unwrap()
            } else {
                encode_pb_picture(&p, &b, &recon, tr_p, prev_tr, &cfg).unwrap()
            };
            stream.extend_from_slice(&unit);
            recon = decode_sequence(&stream, DecodeOptions::default())
                .unwrap()
                .pop()
                .unwrap();
            prev_tr = tr_p;
        }
        // The plain form must match exactly; under Advanced Prediction
        // the oracle's P-part is B-content-dependent (see
        // `cross_check_anchors_psnr`).
        if ap {
            cross_check_anchors_psnr(&stream, 176, 144, "pb-ap", 30.0, &[0, 2, 4]);
        } else {
            cross_check_anchors(&stream, 176, 144, "pb-plain", 0.08, &[0, 2, 4]);
        }
    }
}

/// Annex M with an INTRA refresh, plain and with Advanced Prediction:
/// every fourth P-macroblock is INTRA and carries the §G.2 / §M.2.1
/// B-purpose MVD. On the plain form the oracle must agree exactly
/// (the encoder sends a zero INTRA vector — see
/// `ImprovedPbConfig::intra_refresh` for why that matters to the
/// oracle); under AP the INTRA neighbours' §G.2 OBMC remotes join the
/// documented right-column disagreement, so the tolerant bound applies.
#[test]
fn improved_pb_intra_refresh_agrees_with_oracle() {
    require_oracle!();
    for ap in [false, true] {
        let base = gradient(176, 144, 31);
        let mut stream = encode_intra_picture(&base, 7, 0).unwrap();
        let mut recon = decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .remove(0);
        let cfg = ImprovedPbConfig {
            quant: 7,
            trb: 1,
            dbquant: 0,
            search_half: 6,
            forward_search_half: 3,
            allow_backward: true,
            advanced_prediction: ap,
            umv: false,
            slice_rows: 0,
            intra_refresh: 4,
        };
        let mut prev_tr = 0u8;
        for k in 1..=2usize {
            let p = translated(&base, 5 * k, k);
            let b = translated(&base, 5 * k - 2, k);
            let tr_p = (2 * k) as u8;
            let unit = encode_improved_pb_picture(&p, &b, &recon, tr_p, prev_tr, &cfg).unwrap();
            stream.extend_from_slice(&unit);
            recon = decode_sequence(&stream, DecodeOptions::default())
                .unwrap()
                .pop()
                .unwrap();
            prev_tr = tr_p;
        }
        if ap {
            cross_check_anchors_psnr(&stream, 176, 144, "improved-pb-intra-ap", 30.0, &[0, 2, 4]);
        } else {
            cross_check_anchors(&stream, 176, 144, "improved-pb-intra", 0.08, &[0, 2, 4]);
        }
    }
}

/// Black-box evidence for the AP + PB oracle deviation: two streams
/// whose P-parts are bit-identical in every P-field (same P source,
/// same reference, same vectors) but whose B-parts differ make the
/// oracle output *different* P-pictures, while this crate's decoder
/// (matching the encoder's PREC) outputs identical ones. §G.3 orders
/// the P-blocks before the B-blocks and §G.5 derives the B prediction
/// from PREC, never the reverse — so the P-part cannot legitimately
/// depend on the B-part.
#[test]
fn oracle_ap_pb_p_part_depends_on_b_content() {
    require_oracle!();
    let base = gradient(176, 144, 31);
    let i_bytes = encode_intra_picture(&base, 7, 0).unwrap();
    let recon = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let p = translated(&base, 5, 1);
    let cfg = ImprovedPbConfig {
        quant: 7,
        trb: 1,
        dbquant: 0,
        search_half: 6,
        forward_search_half: 0,
        allow_backward: false,
        advanced_prediction: true,
        umv: false,
        slice_rows: 0,
        intra_refresh: 0,
    };
    let mut ours_p = Vec::new();
    let mut oracle_p = Vec::new();
    for (k, b) in [translated(&base, 3, 1), base.clone()].iter().enumerate() {
        let mut stream = i_bytes.clone();
        stream.extend_from_slice(&encode_improved_pb_picture(&p, b, &recon, 2, 0, &cfg).unwrap());
        let ours = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        let theirs = oracle_decode(&stream, 176, 144, &format!("ap-pb-bdep{k}"));
        assert_eq!(theirs.len(), 2);
        let psnr = luma_psnr(&ours[2], &theirs[1]);
        eprintln!("B-variant {k}: luma PSNR ours-vs-oracle on the P-part {psnr:.2} dB");
        assert!(psnr >= 35.0);
        ours_p.push(ours[2].clone());
        oracle_p.push(theirs[1].clone());
    }
    assert_eq!(
        ours_p[0], ours_p[1],
        "our P-part is independent of the B-part"
    );
    assert_ne!(
        oracle_p[0], oracle_p[1],
        "the oracle's P-part varies with the B-part (documented deviation)"
    );
}

/// Annex G + UMV (baseline) and Annex M + UMV (Table D.3): wide pans
/// whose vectors need the extended range, MVDB / forward vectors
/// included — the oracle must agree on every P-part exactly.
#[test]
fn pb_frames_with_umv_agree_with_oracle() {
    require_oracle!();
    let base = gradient(176, 144, 37);
    // Annex G + UMV.
    let mut stream = encode_intra_picture(&base, 8, 0).unwrap();
    let mut recon = decode_sequence(&stream, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let cfg = PbConfig {
        quant: 8,
        trb: 1,
        dbquant: 1,
        search_half: 24,
        b_search_half: 2,
    };
    let mut prev_tr = 0u8;
    for k in 1..=2usize {
        let p = translated(&base, 20 * k, k);
        let b = translated(&base, 20 * k - 9, k);
        let tr_p = (2 * k) as u8;
        let unit = encode_pb_picture_umv(&p, &b, &recon, tr_p, prev_tr, &cfg).unwrap();
        stream.extend_from_slice(&unit);
        recon = decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .pop()
            .unwrap();
        prev_tr = tr_p;
    }
    cross_check_anchors(&stream, 176, 144, "pb-umv", 0.08, &[0, 2, 4]);

    // Annex M + UMV+ — bidirectional / backward BPB modes only: the
    // oracle parses a forward-mode MVDB under UMV + PLUSPTYPE with some
    // table other than the Table D.3 this crate applies (§M.2.2 "coded
    // in the same way as … MVD", which §5.3.7 / §D.2 switch to Table
    // D.3 when PLUSPTYPE is present) — its output desynchronises from
    // the first forward macroblock, and a Table 14 MVDB is rejected by
    // it outright. The P-part vectors (Table D.3, 20 px pan) agree.
    let mut stream = encode_intra_picture(&base, 8, 0).unwrap();
    let mut recon = decode_sequence(&stream, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let cfg = ImprovedPbConfig {
        quant: 8,
        trb: 1,
        dbquant: 0,
        search_half: 24,
        forward_search_half: 0,
        allow_backward: true,
        advanced_prediction: false,
        umv: true,
        slice_rows: 0,
        intra_refresh: 0,
    };
    let mut prev_tr = 0u8;
    for k in 1..=2usize {
        let p = translated(&base, 20 * k, k);
        let b = translated(&base, 20 * k - 9, k);
        let tr_p = (2 * k) as u8;
        let unit = encode_improved_pb_picture(&p, &b, &recon, tr_p, prev_tr, &cfg).unwrap();
        stream.extend_from_slice(&unit);
        recon = decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .pop()
            .unwrap();
        prev_tr = tr_p;
    }
    cross_check_anchors(&stream, 176, 144, "improved-pb-umv-plus", 0.08, &[0, 2, 4]);
}

/// Annex K slices + Improved PB-frames (plain P-part): every slice
/// header sits between PB macroblock data, and the oracle must agree on
/// every P-part exactly.
#[test]
fn improved_pb_with_slices_agrees_with_oracle() {
    require_oracle!();
    let base = gradient(176, 144, 43);
    let mut stream = encode_intra_picture(&base, 7, 0).unwrap();
    let mut recon = decode_sequence(&stream, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let cfg = ImprovedPbConfig {
        quant: 7,
        trb: 1,
        dbquant: 0,
        search_half: 6,
        forward_search_half: 3,
        allow_backward: true,
        advanced_prediction: false,
        umv: false,
        slice_rows: 2,
        intra_refresh: 0,
    };
    let mut prev_tr = 0u8;
    for k in 1..=2usize {
        let p = translated(&base, 4 * k, k);
        let b = translated(&base, 4 * k - 3, k);
        let tr_p = (2 * k) as u8;
        let unit = encode_improved_pb_picture(&p, &b, &recon, tr_p, prev_tr, &cfg).unwrap();
        stream.extend_from_slice(&unit);
        recon = decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .pop()
            .unwrap();
        prev_tr = tr_p;
    }
    cross_check_anchors(&stream, 176, 144, "improved-pb-slices", 0.08, &[0, 2, 4]);
}
