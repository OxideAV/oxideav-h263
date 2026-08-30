//! Rate / PSNR ladder over the closed-loop GOP encoder: QCIF and CIF
//! synthetic moving clips encoded at a spread of quantisers, measured
//! end-to-end through the crate decoder. Pins the two properties a
//! sane rate/distortion ladder must have — rate falls and distortion
//! rises monotonically as QP coarsens — and prints the measured table
//! (run with `--nocapture`) so the README numbers stay reproducible.

use oxideav_h263::encoder::{encode_sequence, GopConfig};
use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};

fn textured(lw: usize, lh: usize, seed: usize) -> YuvFrame {
    let mut y = vec![0u8; lw * lh];
    for r in 0..lh {
        for c in 0..lw {
            let checker = if ((r / 8) + (c / 8)) % 2 == 0 { 26 } else { 0 };
            y[r * lw + c] = ((r * 3 + c * 2 + seed * 7 + checker) % 256) as u8;
        }
    }
    YuvFrame {
        y,
        cb: (0..lw * lh / 4).map(|i| (90 + i % 60) as u8).collect(),
        cr: (0..lw * lh / 4).map(|i| (170 - (i % 50)) as u8).collect(),
        luma_width: lw,
        luma_height: lh,
    }
}

fn translated(f: &YuvFrame, dx: usize, dy: usize) -> YuvFrame {
    let lw = f.luma_width;
    let lh = f.luma_height;
    let cw = lw / 2;
    let ch = lh / 2;
    let mut out = f.clone();
    for r in 0..lh {
        for c in 0..lw {
            out.y[r * lw + c] = f.y[((r + lh - dy) % lh) * lw + (c + lw - dx) % lw];
        }
    }
    for r in 0..ch {
        for c in 0..cw {
            let s = ((r + ch - dy / 2) % ch) * cw + (c + cw - dx / 2) % cw;
            out.cb[r * cw + c] = f.cb[s];
            out.cr[r * cw + c] = f.cr[s];
        }
    }
    out
}

fn clip(lw: usize, lh: usize, n: usize) -> Vec<YuvFrame> {
    let base = textured(lw, lh, 5);
    (0..n).map(|i| translated(&base, 2 * i, i)).collect()
}

fn luma_psnr(clips: &[YuvFrame], decs: &[YuvFrame]) -> f64 {
    let mut se = 0f64;
    let mut n = 0usize;
    for (a, b) in clips.iter().zip(decs.iter()) {
        for (&x, &y) in a.y.iter().zip(b.y.iter()) {
            let d = x as f64 - y as f64;
            se += d * d;
            n += 1;
        }
    }
    let mse = se / n as f64;
    10.0 * (255.0f64 * 255.0 / mse).log10()
}

/// One ladder: encode `frames` at each QP, return (bits/picture, PSNR).
fn ladder(frames: &[YuvFrame], qps: &[u8]) -> Vec<(u8, f64, f64)> {
    qps.iter()
        .map(|&qp| {
            let cfg = GopConfig {
                quant: qp,
                intra_period: 0,
                search_half: 7,
                ..GopConfig::default()
            };
            let stream = encode_sequence(frames, &cfg, 0).expect("encode");
            let decs = decode_sequence(&stream, DecodeOptions::default()).expect("decode");
            assert_eq!(decs.len(), frames.len());
            let bits = stream.len() as f64 * 8.0 / frames.len() as f64;
            (qp, bits, luma_psnr(frames, &decs))
        })
        .collect()
}

fn assert_monotone(name: &str, rows: &[(u8, f64, f64)]) {
    println!("{name}:  QP  bits/picture  Y-PSNR(dB)");
    for (qp, bits, psnr) in rows {
        println!("{name}:  {qp:>2}  {bits:>12.0}  {psnr:>9.2}");
    }
    for pair in rows.windows(2) {
        assert!(
            pair[1].1 < pair[0].1,
            "{name}: rate must fall as QP coarsens ({} -> {})",
            pair[0].1,
            pair[1].1
        );
        assert!(
            pair[1].2 < pair[0].2,
            "{name}: PSNR must fall as QP coarsens ({} -> {})",
            pair[0].2,
            pair[1].2
        );
    }
}

#[test]
fn qcif_rate_psnr_ladder_is_monotone() {
    let frames = clip(176, 144, 8);
    let rows = ladder(&frames, &[2, 4, 8, 16, 31]);
    assert_monotone("QCIF", &rows);
    // Anchor the endpoints loosely so a quality regression trips.
    assert!(rows[0].2 > 40.0, "QP 2 Y-PSNR {:.2}", rows[0].2);
    assert!(rows[4].2 > 24.0, "QP 31 Y-PSNR {:.2}", rows[4].2);
}

#[test]
fn cif_rate_psnr_ladder_is_monotone() {
    let frames = clip(352, 288, 4);
    let rows = ladder(&frames, &[2, 8, 31]);
    assert_monotone("CIF", &rows);
    assert!(rows[0].2 > 40.0, "QP 2 Y-PSNR {:.2}", rows[0].2);
}
