//! Annex E SAC ffmpeg interop probe.
//!
//! Encode a single I-picture with SAC enabled, write the raw H.263
//! elementary stream to a temp file, and shell out to ffmpeg to attempt a
//! decode. Marked `#[ignore]` by default — many ffmpeg builds disable the
//! H.263 SAC decode path or never implemented it (FFmpeg's `h263.c` does
//! NOT decode the SAC body — see `h263_decode_picture_header` rejecting
//! `s->h263_aic` together with PTYPE bit 11 in older builds), so the
//! "interop succeeds" outcome here is informational rather than a
//! correctness gate. Run with `--ignored` to probe explicitly.

use std::io::Write;
use std::process::{Command, Stdio};

use oxideav_core::frame::VideoPlane;
use oxideav_core::{
    CodecId, CodecParameters, Encoder, Frame, MediaType, PixelFormat, VideoFrame,
};
use oxideav_h263::encoder::H263Encoder;

fn make_qcif_constant() -> VideoFrame {
    let w = 176usize;
    let h = 144usize;
    VideoFrame {
        pts: Some(0),
        planes: vec![
            VideoPlane {
                stride: w,
                data: vec![100u8; w * h],
            },
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
#[ignore = "informational ffmpeg interop probe; FFmpeg may not implement Annex E SAC decode"]
fn sac_iframe_decoded_by_ffmpeg() {
    let frame = make_qcif_constant();
    let mut params = CodecParameters::video(CodecId::new("h263"));
    params.media_type = MediaType::Video;
    params.width = Some(176);
    params.height = Some(144);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    let mut enc = H263Encoder::from_params(&params).unwrap();
    enc.set_enable_annex_e(true);
    enc.send_frame(&Frame::Video(frame)).unwrap();
    enc.flush().unwrap();
    let pkt = enc.receive_packet().unwrap();

    eprintln!(
        "SAC packet: {} bytes, first 16: {:02x?}",
        pkt.data.len(),
        &pkt.data[..pkt.data.len().min(16)]
    );

    // Pipe the raw H.263 elementary stream into ffmpeg via stdin and ask
    // for a YUV420P rawvideo decode. We do NOT assert success — record
    // ffmpeg's stderr and ignore the exit code beyond logging it.
    let mut child = Command::new("/opt/homebrew/bin/ffmpeg")
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
            "/tmp/sac_ffmpeg_out.yuv",
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
