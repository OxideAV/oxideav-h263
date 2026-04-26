//! Drift diagnostic: per-frame, per-MB MAD (mean abs diff) heatmap on the
//! luma plane between our decode and ffmpeg's reference decode.
//!
//! Run manually: `cargo test --release --test drift_mb_heatmap -- --ignored --nocapture`

use std::process::Command;

use oxideav_core::Decoder;
use oxideav_core::{CodecId, Frame, Packet, TimeBase};
use oxideav_h263::decoder::H263Decoder;

#[test]
#[ignore]
fn per_mb_heatmap_no_obmc() {
    heatmap("h263_heat_no_obmc", &[] as &[&str]);
}

#[test]
#[ignore]
fn per_mb_heatmap_obmc() {
    heatmap("h263_heat_obmc", &["-obmc", "1"]);
}

fn heatmap(name: &str, extra: &[&str]) {
    if Command::new("ffmpeg").arg("-version").output().is_err() {
        return;
    }
    let tmp = std::env::temp_dir();
    let avi = tmp.join(format!("{name}.avi"));
    let es = tmp.join(format!("{name}.h263"));
    let ref_yuv = tmp.join(format!("{name}.yuv"));

    let mut enc_args: Vec<String> = vec![
        "-y".into(),
        "-f".into(),
        "lavfi".into(),
        "-i".into(),
        "testsrc2=size=176x144:rate=15:duration=1.0".into(),
        "-c:v".into(),
        "h263".into(),
    ];
    for s in extra {
        enc_args.push(s.to_string());
    }
    enc_args.extend(
        ["-qscale:v", "5", "-an", avi.to_str().unwrap()]
            .iter()
            .map(|s| s.to_string()),
    );

    let _ = Command::new("ffmpeg").args(&enc_args).output().unwrap();
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

    let luma0 = &frames[0].planes[0];
    let (w, h) = (luma0.stride, luma0.data.len() / luma0.stride);
    let y_size = w * h;
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    let c_size = cw * ch;
    let frame_size = y_size + 2 * c_size;

    let mb_w = w / 16;
    let mb_h = h / 16;

    for (i, f) in frames.iter().enumerate() {
        if !(4..=9).contains(&i) {
            continue;
        }
        let base = i * frame_size;
        let ref_y = &ref_bytes[base..base + y_size];
        let my_y = &f.planes[0];
        eprintln!("== {name} frame {i} ==");
        for mb_y in 0..mb_h {
            let mut row_str = String::new();
            for mb_x in 0..mb_w {
                let mut sum = 0u32;
                for j in 0..16 {
                    for ii in 0..16 {
                        let py = mb_y * 16 + j;
                        let px = mb_x * 16 + ii;
                        let r = ref_y[py * w + px] as i32;
                        let m = my_y.data[py * my_y.stride + px] as i32;
                        sum += (m - r).unsigned_abs();
                    }
                }
                let mad = sum / 256;
                row_str.push(match mad {
                    0 => '.',
                    1 => '1',
                    2 => '2',
                    3 => '3',
                    4..=9 => (b'0' + mad as u8) as char,
                    10..=99 => '#',
                    _ => '@',
                });
            }
            eprintln!("  {row_str}");
        }
    }
}
