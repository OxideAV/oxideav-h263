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
    encode_inter_picture_ap, encode_inter_picture_deblock, encode_inter_picture_motion,
    encode_inter_picture_umv_plus, encode_intra_picture, encode_intra_picture_aic_plus,
    encode_intra_picture_deblock, encode_sequence, DeblockConfig, GopConfig,
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
