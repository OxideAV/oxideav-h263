//! Annex I (AIC) ffmpeg interop probe.
//!
//! Encode a single I-picture with AIC enabled, write the raw H.263
//! elementary stream to a temp file, and shell out to ffmpeg to attempt a
//! decode. Marked `#[ignore]` by default — many ffmpeg builds disable
//! H.263+ features by default, so the "interop succeeds" outcome here is
//! informational rather than a correctness gate. Run explicitly with
//! `cargo test -p oxideav-h263 --test aic_ffmpeg_interop -- --ignored`.

use std::io::Write;
use std::process::{Command, Stdio};

use oxideav_core::frame::VideoPlane;
use oxideav_core::{CodecId, CodecParameters, Encoder, Frame, MediaType, PixelFormat, VideoFrame};
use oxideav_h263::encoder::H263Encoder;

fn make_qcif_textured() -> VideoFrame {
    let w = 176usize;
    let h = 144usize;
    let mut y = vec![0u8; w * h];
    for j in 0..h {
        for i in 0..w {
            // Smooth gradient + tiny dither — exercises both DC pred and
            // some AC content under AIC.
            let v = (((i + j) * 200 / (w + h)) as u8).clamp(20, 200);
            y[j * w + i] = v.wrapping_add(((i ^ j) & 3) as u8);
        }
    }
    VideoFrame {
        pts: Some(0),
        planes: vec![
            VideoPlane { stride: w, data: y },
            VideoPlane {
                stride: w / 2,
                data: vec![128u8; (w / 2) * (h / 2)],
            },
            VideoPlane {
                stride: w / 2,
                data: vec![128u8; (w / 2) * (h / 2)],
            },
        ],
    }
}

#[test]
#[ignore = "informational ffmpeg interop probe; FFmpeg may not implement Annex I AIC decode"]
fn aic_iframe_decoded_by_ffmpeg() {
    let frame = make_qcif_textured();
    let mut params = CodecParameters::video(CodecId::new("h263"));
    params.media_type = MediaType::Video;
    params.width = Some(176);
    params.height = Some(144);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_i_aic(true);
    enc.send_frame(&Frame::Video(frame)).unwrap();
    enc.flush().unwrap();
    let pkt = enc.receive_packet().unwrap();

    eprintln!(
        "AIC packet: {} bytes, first 16: {:02x?}",
        pkt.data.len(),
        &pkt.data[..pkt.data.len().min(16)]
    );

    let ff = if std::path::Path::new("/opt/homebrew/bin/ffmpeg").exists() {
        "/opt/homebrew/bin/ffmpeg"
    } else {
        "ffmpeg"
    };

    let mut child = Command::new(ff)
        .args([
            "-loglevel",
            "info",
            "-f",
            "h263",
            "-i",
            "pipe:0",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "yuv420p",
            "-frames:v",
            "1",
            "-y",
            "/tmp/aic_ffmpeg_out.yuv",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn ffmpeg");
    {
        let mut stdin = child.stdin.take().unwrap();
        stdin.write_all(&pkt.data).unwrap();
    }
    let out = child.wait_with_output().expect("ffmpeg wait");
    let stderr = String::from_utf8_lossy(&out.stderr);
    eprintln!("ffmpeg exit: {:?}", out.status.code());
    eprintln!("ffmpeg stderr (last 60 lines):");
    for line in stderr
        .lines()
        .rev()
        .take(60)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
    {
        eprintln!("    {line}");
    }
    // No assertion — this test is informational.
}
