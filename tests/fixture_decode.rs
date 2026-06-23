//! End-to-end fixture decode tests for the H.263 baseline driver.
//!
//! These drive the public [`decode_sequence`] / [`decode_picture_no_gob0_header`]
//! entry points over real H.263 elementary streams produced by the reference encoder's
//! native H.263 encoder and compare the reconstructed planar 4:2:0 output
//! against the encoder-produced reference YUV that ships beside each
//! stream (`tests/fixtures/<name>/{input.h263,expected.yuv}`).
//!
//! ## Conformance criterion (ITU-T H.263 Annex A)
//!
//! H.263 does **not** mandate a bit-exact inverse DCT. §6.2 states "the
//! arithmetic procedures for computing the inverse transform are not
//! defined, but should meet the error tolerance specified in Annex A",
//! and Annex A.7's first bullet bounds the per-pixel peak error at **1 in
//! magnitude**. Two conforming decoders — ours (an f64 reference-DCT
//! kernel) and the reference encoder's (a fast integer IDCT) — may therefore legally
//! differ by up to ±1 per sample wherever a block carries AC energy.
//!
//! So the assertion for AC-bearing content is "every sample is within ±1
//! of the reference", not byte-equality. The flat-colour `tiny` fixture
//! carries no AC, so its IDCT output is exact and it is additionally
//! checked byte-for-byte against the reference SHA-256 recorded in the
//! fixture's `notes.md`.

use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};

/// Concatenate a frame's Y, Cb, Cr planes into the planar layout the
/// reference `expected.yuv` files use (full Y plane, then Cb, then Cr).
fn frame_bytes(f: &YuvFrame) -> Vec<u8> {
    let mut v = Vec::with_capacity(f.y.len() + f.cb.len() + f.cr.len());
    v.extend_from_slice(&f.y);
    v.extend_from_slice(&f.cb);
    v.extend_from_slice(&f.cr);
    v
}

/// Decode `input.h263`, concatenate the reconstructed frames, and return
/// the resulting planar byte stream.
fn decode_all(input: &[u8]) -> Vec<u8> {
    let frames = decode_sequence(input, DecodeOptions::default()).expect("decode_sequence");
    let mut out = Vec::new();
    for f in &frames {
        out.extend_from_slice(&frame_bytes(f));
    }
    out
}

/// Assert that `got` is the same length as `expected` and that no sample
/// differs by more than the Annex A.7 per-pixel peak-error bound of 1.
fn assert_within_idct_tolerance(name: &str, got: &[u8], expected: &[u8]) {
    assert_eq!(
        got.len(),
        expected.len(),
        "{name}: decoded byte count {} != reference {}",
        got.len(),
        expected.len()
    );
    let mut max_diff = 0i32;
    let mut over = 0usize;
    for (g, e) in got.iter().zip(expected.iter()) {
        let d = (*g as i32 - *e as i32).abs();
        if d > max_diff {
            max_diff = d;
        }
        if d > 1 {
            over += 1;
        }
    }
    assert!(
        over == 0,
        "{name}: {over} sample(s) exceed the Annex A.7 ±1 IDCT tolerance (peak diff {max_diff})"
    );
}

macro_rules! fixture {
    ($name:literal) => {{
        let input = include_bytes!(concat!("fixtures/", $name, "/input.h263"));
        let expected = include_bytes!(concat!("fixtures/", $name, "/expected.yuv"));
        (&input[..], &expected[..])
    }};
}

#[test]
fn tiny_sqcif_i_only_is_byte_exact() {
    // Flat red sub-QCIF keyframe: no AC energy, so the IDCT output is
    // exact and the reconstruction must match the reference byte-for-byte.
    let (input, expected) = fixture!("tiny-i-only-sqcif-baseline");
    let got = decode_all(input);
    assert_eq!(
        got, expected,
        "tiny-i-only-sqcif: flat keyframe must reconstruct byte-exact"
    );
    // The fixture is a single 128x96 sub-QCIF frame (Y + Cb + Cr).
    assert_eq!(got.len(), 128 * 96 + 2 * (64 * 48));
}

#[test]
fn tiny_sqcif_matches_recorded_sha256() {
    // Cross-check the reference bytes themselves against the SHA-256
    // recorded in the fixture's notes.md, so a corrupted vendored
    // expected.yuv can't silently pass the byte-exact comparison.
    let (_input, expected) = fixture!("tiny-i-only-sqcif-baseline");
    let digest = sha256_hex(expected);
    assert_eq!(
        digest, "11da3e53f759280e82f71a46843e55dd3260f0aa0dfb38e79bbea11a4b87205d",
        "tiny-i-only-sqcif expected.yuv SHA-256 mismatch — vendored fixture corrupted?"
    );
}

#[test]
fn qcif_i_only_within_idct_tolerance() {
    let (input, expected) = fixture!("i-only-qcif-baseline");
    let got = decode_all(input);
    assert_within_idct_tolerance("i-only-qcif", &got, expected);
    assert_eq!(got.len(), 176 * 144 + 2 * (88 * 72));
}

#[test]
fn cif_i_only_within_idct_tolerance() {
    let (input, expected) = fixture!("i-only-cif-baseline");
    let got = decode_all(input);
    assert_within_idct_tolerance("i-only-cif", &got, expected);
    assert_eq!(got.len(), 352 * 288 + 2 * (176 * 144));
}

#[test]
fn qcif_i_then_p_frames_within_idct_tolerance() {
    // I + P + P: exercises the multi-picture demuxer (PSC splitting),
    // reference threading, and the P-frame motion-compensation /
    // median-MV-predictor path against real inter-coded content.
    let (input, expected) = fixture!("i-frame-then-p-frame-qcif");
    let frames = decode_sequence(input, DecodeOptions::default()).expect("decode_sequence");
    assert_eq!(
        frames.len(),
        3,
        "stream is one I-frame followed by two P-frames"
    );
    let got = decode_all(input);
    assert_within_idct_tolerance("i-frame-then-p-frame-qcif", &got, expected);
    assert_eq!(got.len(), 3 * (176 * 144 + 2 * (88 * 72)));
}

#[test]
fn qcif_qp_high_i_only_within_idct_tolerance() {
    // Baseline H.263 QCIF keyframe at PQUANT=31 (the maximum quantiser).
    // Exercises the §5.4.2.2 inverse-quant `2*QP+1` / `2*QP` parity at the
    // upper QP boundary where AC coefficients are most heavily attenuated.
    let (input, expected) = fixture!("qp-high");
    let got = decode_all(input);
    assert_within_idct_tolerance("qp-high", &got, expected);
    assert_eq!(got.len(), 176 * 144 + 2 * (88 * 72));
}

#[test]
fn qcif_qp_low_i_only_within_idct_tolerance() {
    // Baseline H.263 QCIF keyframe at PQUANT=2 (the minimum quantiser):
    // near-lossless coding, the largest fixture in the corpus. A decoder
    // that mis-implements de-quantisation diverges most visibly here.
    let (input, expected) = fixture!("qp-low");
    let got = decode_all(input);
    assert_within_idct_tolerance("qp-low", &got, expected);
    assert_eq!(got.len(), 176 * 144 + 2 * (88 * 72));
}

#[test]
fn h263p_modern_plusptype_within_idct_tolerance() {
    // H.263+ (1998) baseline picture-header form: PTYPE source-format =
    // "111" selects the extended PLUSPTYPE header, with every Annex bit
    // cleared except the H.263+ default custom-PCF. Exercises the
    // `decode_sequence` PLUSPTYPE dispatch (extended-header detection +
    // §5.1.4.4 inherited-state threading), the §5.1.7 / §5.1.8 CPCFC / ETR
    // framing, and the §5.2.2 GOB-0-header elision on the extended GOB
    // driver. The stream is one I-frame followed by two P-frames at QCIF.
    let (input, expected) = fixture!("h263p-modern");
    let frames =
        decode_sequence(input, DecodeOptions::default()).expect("decode_sequence h263p-modern");
    assert_eq!(frames.len(), 3, "h263p-modern is I + P + P");
    let got = decode_all(input);
    assert_within_idct_tolerance("h263p-modern", &got, expected);
    assert_eq!(got.len(), 3 * (176 * 144 + 2 * (88 * 72)));
}

#[test]
fn slice_structured_mode_within_idct_tolerance() {
    // H.263+ Annex K Slice-Structured mode (free-running, non-rectangular):
    // each picture replaces the GOB layer with §K.2 slices (SSC + reduced
    // first-slice header + per-slice SEPB/MBA/SQUANT/GFID). Exercises the
    // `decode_sequence` PLUSPTYPE dispatch into the slice driver, the
    // §5.1.24 PEI consumption before the first slice, and the §K.1
    // strictly-increasing-MBA free-running walk. The stream is a QCIF
    // I+P+P sequence with 24 slice headers across the corpus.
    let (input, expected) = fixture!("slice-structured-mode");
    let frames = decode_sequence(input, DecodeOptions::default())
        .expect("decode_sequence slice-structured-mode");
    assert_eq!(frames.len(), 3, "slice-structured-mode is I + P + P");
    let got = decode_all(input);
    assert_within_idct_tolerance("slice-structured-mode", &got, expected);
    assert_eq!(got.len(), 3 * (176 * 144 + 2 * (88 * 72)));
}

// --- minimal SHA-256 (FIPS 180-4), test-only, no external crate ---

fn sha256_hex(data: &[u8]) -> String {
    let h = sha256(data);
    let mut s = String::with_capacity(64);
    for b in h {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

fn sha256(data: &[u8]) -> [u8; 32] {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];

    let mut msg = data.to_vec();
    let bit_len = (data.len() as u64) * 8;
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bit_len.to_be_bytes());

    for chunk in msg.chunks_exact(64) {
        let mut w = [0u32; 64];
        for (i, word) in w.iter_mut().take(16).enumerate() {
            *word = u32::from_be_bytes([
                chunk[i * 4],
                chunk[i * 4 + 1],
                chunk[i * 4 + 2],
                chunk[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }

        let mut a = h[0];
        let mut b = h[1];
        let mut c = h[2];
        let mut d = h[3];
        let mut e = h[4];
        let mut f = h[5];
        let mut g = h[6];
        let mut hh = h[7];

        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }

        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
        h[5] = h[5].wrapping_add(f);
        h[6] = h[6].wrapping_add(g);
        h[7] = h[7].wrapping_add(hh);
    }

    let mut out = [0u8; 32];
    for (i, word) in h.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    out
}

#[test]
fn vendored_fixture_expected_yuv_sha256() {
    // Guard the newly-vendored reference YUVs against silent corruption,
    // the same way `tiny_sqcif_matches_recorded_sha256` does. The digests
    // are the ones recorded in each fixture's docs notes.md.
    for (name, want) in [
        (
            "qp-high",
            "1c8b87565eba7c92153b63cd9a0eaa7c0b5d5d532519b5bc7cb7c1ce351af10c",
        ),
        (
            "qp-low",
            "963c0243f04890c22c4aee11ed373fbebd35192e0ed7d6bd6cf1cb38c5873ede",
        ),
        (
            "h263p-modern",
            "7418cf95376076df42caef4294262da252d02f85205a864abd62d16ff6b36a78",
        ),
        (
            "slice-structured-mode",
            "0633dfd408e39596186b46d5eeca95550840efce77ede281b23a56f3d153f8ac",
        ),
    ] {
        let expected: &[u8] = match name {
            "qp-high" => fixture!("qp-high").1,
            "qp-low" => fixture!("qp-low").1,
            "h263p-modern" => fixture!("h263p-modern").1,
            "slice-structured-mode" => fixture!("slice-structured-mode").1,
            _ => unreachable!(),
        };
        assert_eq!(
            sha256_hex(expected),
            want,
            "{name} expected.yuv SHA-256 mismatch — vendored fixture corrupted?"
        );
    }
}

#[test]
fn sha256_self_test() {
    // FIPS 180-4 worked example: SHA-256("abc").
    assert_eq!(
        sha256_hex(b"abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    );
    assert_eq!(
        sha256_hex(b""),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
}
