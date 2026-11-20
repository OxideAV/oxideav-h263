//! Encode / decode throughput of the closed-loop I + P GOP at CIF and
//! 4CIF, on synthetic moving textured content.
//!
//! `cargo run --release --example throughput [frames]` (set
//! `H263_DUMP_DIR` to keep the streams), or
//! `cargo run --release --example throughput decode <file.h263>`.

use std::time::Instant;

use oxideav_h263::encoder::{GopConfig, encode_sequence};
use oxideav_h263::picture::{DecodeOptions, YuvFrame, decode_sequence};

fn frame(lw: usize, lh: usize, t: usize) -> YuvFrame {
    let (cw, ch) = (lw / 2, lh / 2);
    let mut y = vec![0u8; lw * lh];
    for r in 0..lh {
        for c in 0..lw {
            let (x, v) = (c + 3 * t, r + t);
            let ramp = (x / 2 + v / 3) % 96;
            let tex = ((x * 7) ^ (v * 13)) % 23;
            let disc = {
                let (dx, dy) = (
                    c as i64 - (lw / 2 + 5 * t) as i64,
                    r as i64 - (lh / 2) as i64,
                );
                if dx * dx + dy * dy < ((lh / 5) * (lh / 5)) as i64 {
                    70
                } else {
                    0
                }
            };
            y[r * lw + c] = (40 + ramp + tex + disc).min(255) as u8;
        }
    }
    YuvFrame {
        y,
        cb: (0..cw * ch)
            .map(|i| (110 + ((i % cw + t) / 8) % 30) as u8)
            .collect(),
        cr: (0..cw * ch)
            .map(|i| (130 + ((i / cw + t) / 8) % 30) as u8)
            .collect(),
        luma_width: lw,
        luma_height: lh,
    }
}

fn main() {
    // `throughput decode <file.h263>`: time the decode of an existing
    // elementary stream (best of five passes).
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("decode") {
        let stream = std::fs::read(&args[2]).expect("read stream");
        let mut best = f64::MAX;
        let mut pictures = 0;
        for _ in 0..5 {
            let t = Instant::now();
            pictures = decode_sequence(&stream, DecodeOptions::default())
                .expect("decode")
                .len();
            best = best.min(t.elapsed().as_secs_f64());
        }
        println!(
            "{}: {pictures} pictures, decode {:.1} fps",
            args[2],
            pictures as f64 / best
        );
        return;
    }
    let n: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(12);
    for (name, lw, lh) in [("CIF", 352usize, 288usize), ("4CIF", 704, 576)] {
        let frames: Vec<YuvFrame> = (0..n).map(|t| frame(lw, lh, t)).collect();
        let cfg = GopConfig::default().with_quant(8).with_search_half(14);
        let t0 = Instant::now();
        let stream = encode_sequence(&frames, &cfg, 0).expect("encode");
        let enc = t0.elapsed().as_secs_f64();
        let mut best = f64::MAX;
        let mut decoded = Vec::new();
        for _ in 0..3 {
            let t1 = Instant::now();
            decoded = decode_sequence(&stream, DecodeOptions::default()).expect("decode");
            best = best.min(t1.elapsed().as_secs_f64());
        }
        assert_eq!(decoded.len(), n);
        if let Ok(dir) = std::env::var("H263_DUMP_DIR") {
            std::fs::write(format!("{dir}/{name}.h263"), &stream).expect("dump stream");
        }
        let digest = decoded
            .iter()
            .flat_map(|f| f.y.iter().chain(&f.cb).chain(&f.cr))
            .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
                (h ^ b as u64).wrapping_mul(0x100_0000_01b3)
            });
        println!(
            "{name}: {n} frames, {} bytes; encode {:.1} fps, decode {:.1} fps; digest {digest:016x}",
            stream.len(),
            n as f64 / enc,
            n as f64 / best,
        );
    }
}
