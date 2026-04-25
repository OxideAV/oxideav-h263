//! Diagnostic tool: for a 1-second ffmpeg-produced OBMC stream, dump per-frame
//! per-MB differences between our decoded output and ffmpeg's reference decode.
//!
//! Prints the distribution of pixel differences (`|delta|`) per frame so we can
//! see where the drift starts and whether it has a signed bias.

use std::process::Command;

use oxideav_core::Decoder;
use oxideav_core::{CodecId, Frame, Packet, TimeBase};
use oxideav_h263::decoder::H263Decoder;

#[test]
#[ignore] // Run manually: `cargo test --release --test drift_diag -- --ignored --nocapture`
fn obmc_drift_per_frame_histogram() {
    if Command::new("ffmpeg").arg("-version").output().is_err() {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    }
    let tmp = std::env::temp_dir();
    let avi = tmp.join("h263_drift.avi");
    let es = tmp.join("h263_drift.h263");
    let ref_yuv = tmp.join("h263_drift.yuv");

    let _ = Command::new("ffmpeg")
        .args([
            "-y",
            "-f",
            "lavfi",
            "-i",
            "testsrc2=size=176x144:rate=15:duration=1.0",
            "-c:v",
            "h263",
            "-obmc",
            "1",
            "-qscale:v",
            "5",
            "-an",
            avi.to_str().unwrap(),
        ])
        .output()
        .unwrap();
    let _ = Command::new("ffmpeg")
        .args([
            "-y",
            "-i",
            avi.to_str().unwrap(),
            "-c:v",
            "copy",
            "-f",
            "h263",
            es.to_str().unwrap(),
        ])
        .output()
        .unwrap();
    let _ = Command::new("ffmpeg")
        .args([
            "-y",
            "-i",
            es.to_str().unwrap(),
            "-f",
            "rawvideo",
            "-pix_fmt",
            "yuv420p",
            ref_yuv.to_str().unwrap(),
        ])
        .output()
        .unwrap();

    let h263_bytes = std::fs::read(&es).unwrap();
    let ref_bytes = std::fs::read(&ref_yuv).unwrap();

    let mut decoder = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    decoder
        .send_packet(&Packet::new(0, TimeBase::new(1, 90_000), h263_bytes))
        .unwrap();
    decoder.flush().unwrap();

    let mut frames = Vec::new();
    while let Ok(frame) = decoder.receive_frame() {
        if let Frame::Video(v) = frame {
            frames.push(v);
        }
    }

    let (w, h) = (frames[0].width as usize, frames[0].height as usize);
    let y_size = w * h;
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    let c_size = cw * ch;
    let frame_size = y_size + 2 * c_size;

    for (i, f) in frames.iter().enumerate() {
        let base = i * frame_size;
        let ref_y = &ref_bytes[base..base + y_size];
        let ref_cb = &ref_bytes[base + y_size..base + y_size + c_size];
        let ref_cr = &ref_bytes[base + y_size + c_size..base + frame_size];

        let my_y = &f.planes[0];
        let my_cb = &f.planes[1];
        let my_cr = &f.planes[2];

        let mut histo = [0u32; 64]; // 0..=31 and 32..=63 for larger
        let mut max_diff: i32 = 0;
        let mut sum_diff: i64 = 0;
        let mut n_nonzero: u32 = 0;
        let mut total: u32 = 0;

        // Y plane per-MB
        let mb_w = w / 16;
        let mb_h = h / 16;
        let mut worst_mb = (0, 0, 0u32);

        for mb_y in 0..mb_h {
            for mb_x in 0..mb_w {
                let mut mb_sum = 0u32;
                for j in 0..16 {
                    for ii in 0..16 {
                        let py = mb_y * 16 + j;
                        let px = mb_x * 16 + ii;
                        let r = ref_y[py * w + px] as i32;
                        let m = my_y.data[py * my_y.stride + px] as i32;
                        let d = (m - r).abs();
                        mb_sum += d as u32;
                        let idx = d.min(63) as usize;
                        histo[idx] += 1;
                        if d > max_diff {
                            max_diff = d;
                        }
                        sum_diff += (m - r) as i64;
                        if d != 0 {
                            n_nonzero += 1;
                        }
                        total += 1;
                    }
                }
                if mb_sum > worst_mb.2 {
                    worst_mb = (mb_x, mb_y, mb_sum);
                }
            }
        }
        // Chroma totals
        for j in 0..ch {
            for ii in 0..cw {
                let r = ref_cb[j * cw + ii] as i32;
                let m = my_cb.data[j * my_cb.stride + ii] as i32;
                let d = (m - r).abs();
                if d != 0 {
                    n_nonzero += 1;
                }
                total += 1;
                sum_diff += (m - r) as i64;
                let r = ref_cr[j * cw + ii] as i32;
                let m = my_cr.data[j * my_cr.stride + ii] as i32;
                let d = (m - r).abs();
                if d != 0 {
                    n_nonzero += 1;
                }
                total += 1;
                sum_diff += (m - r) as i64;
            }
        }

        eprintln!(
            "frame {i:2}: total={total}, nonzero={n_nonzero} ({:.2}%), max={max_diff}, sum(signed)={sum_diff}, worst_mb=({},{},{}), hist[0..=6]={:?}, hist[>=7]={}",
            100.0 * n_nonzero as f64 / total as f64,
            worst_mb.0, worst_mb.1, worst_mb.2,
            &histo[0..=6],
            histo[7..].iter().sum::<u32>(),
        );
    }
}
