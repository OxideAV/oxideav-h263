//! Drift diagnostic variant: if we forcibly disable OBMC (use plain MC for
//! all blocks even when the stream signals AP), does the drift change?
//!
//! This helps isolate whether the drift comes from OBMC's averaging (with
//! 4MV predictors) or from non-OBMC parts of the pipeline (IDCT, dequant,
//! half-pel filter, residual addition). Run manually:
//!
//!   cargo test --release --test drift_no_obmc -- --ignored --nocapture

use std::process::Command;

#[test]
#[ignore]
fn compare_with_and_without_obmc() {
    if Command::new("ffmpeg").arg("-version").output().is_err() {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    }
    let tmp = std::env::temp_dir();

    // Generate a 1-second clip WITHOUT the AP bit so the stream is 1MV-per-MB
    // and we're stressing only the plain MC + IDCT path on the drift chain.
    let avi = tmp.join("h263_no_obmc.avi");
    let es = tmp.join("h263_no_obmc.h263");
    let ref_yuv = tmp.join("h263_no_obmc.yuv");
    let _ = Command::new("ffmpeg")
        .args([
            "-y",
            "-f",
            "lavfi",
            "-i",
            "testsrc2=size=176x144:rate=15:duration=1.0",
            "-c:v",
            "h263",
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

    use oxideav_core::Decoder;
    use oxideav_core::{CodecId, Frame, Packet, TimeBase};
    use oxideav_h263::decoder::H263Decoder;

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

    let luma0 = &frames[0].planes[0];
    let (w, h) = (luma0.stride, luma0.data.len() / luma0.stride);
    let y_size = w * h;
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    let c_size = cw * ch;
    let frame_size = y_size + 2 * c_size;

    for (i, f) in frames.iter().enumerate() {
        let base = i * frame_size;
        let ref_y = &ref_bytes[base..base + y_size];
        let my_y = &f.planes[0];
        let mut mse = 0f64;
        let mut n = 0u64;
        let mut max_diff: i32 = 0;
        let mut sum_diff: i64 = 0;
        for j in 0..h {
            for ii in 0..w {
                let r = ref_y[j * w + ii] as i32;
                let m = my_y.data[j * my_y.stride + ii] as i32;
                let d = (m - r).abs();
                if d > max_diff {
                    max_diff = d;
                }
                sum_diff += (m - r) as i64;
                mse += ((m - r) * (m - r)) as f64;
                n += 1;
            }
        }
        let mse = mse / n as f64;
        let psnr = if mse <= 0.0 {
            99.0
        } else {
            10.0 * (255.0 * 255.0 / mse).log10()
        };
        eprintln!("no-OBMC frame {i:2}: Y-only psnr={psnr:.2} dB, max={max_diff}, sum={sum_diff}");
    }
}
