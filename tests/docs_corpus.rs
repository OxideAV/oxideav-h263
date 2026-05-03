//! Integration tests against the docs/video/h263/ fixture corpus.
//!
//! Each fixture under `../../docs/video/h263/fixtures/<name>/` carries
//! an `input.h263` (raw H.263 elementary stream — every fixture has
//! one) plus a per-fixture `expected.yuv` byte-for-byte ground truth
//! produced by libavcodec's H.263 decoder. One fixture
//! (`containerless-elementary-vs-3gp`) additionally ships an
//! `input.3gp` whose H.263 elementary payload is byte-identical to the
//! `.h263` raw variant — a separate test exercises the 3GP demux path
//! against the same `expected.yuv`.
//!
//! For every fixture we drive [`H263Decoder`], drain all output frames,
//! repack them into the contiguous yuv420p layout that ffmpeg used for
//! `expected.yuv`, and report per-plane match statistics.
//!
//! Per-fixture classification:
//! * [`Tier::BitExact`]   — must round-trip exactly. Failure = CI red.
//!   (Unused on day one — every fixture starts as ReportOnly per task
//!   brief; promote individual entries here as follow-up rounds confirm
//!   bit-exact decode.)
//! * [`Tier::ReportOnly`] — currently divergent (or not yet decodable);
//!   logged but not asserted. Each carries an inline `TODO(h263-corpus)`
//!   tag so the underlying decoder bug stays grep-able.
//!
//! The trace.txt files under each fixture directory are not consumed by
//! this driver; they exist so anyone bisecting a divergence can `diff`
//! their decoder's per-MB / per-MV events against ffmpeg's. The trace
//! event vocabulary is documented at
//! `docs/video/h263/h263-fixtures-and-traces.md`.
//!
//! Spec references for the test logic:
//! * ITU-T Rec. H.263 (01/2005) §5.1 — picture header (PSC, TR, PTYPE,
//!   PLUSPTYPE, source format).
//! * ITU-T Rec. H.263 §5.2 — GOB layer (GBSC, GN); Annex K §K.2 — slice
//!   layer (SSC, MBA, SQUANT, GFID).
//! * 3GPP TS 26.244 / ISO/IEC 14496-12 — 3GP/ISO BMFF for the
//!   containerless-elementary-vs-3gp .3gp branch (we walk the file by
//!   hand and look for the first PSC inside `mdat` rather than taking a
//!   dev-dep on `oxideav-mp4`, which would couple this crate's
//!   published version surface to a crate it does not need at runtime).

use std::fs;
use std::path::PathBuf;

use oxideav_core::packet::PacketFlags;
use oxideav_core::{CodecId, Decoder, Error, Frame, Packet, TimeBase};
use oxideav_h263::decoder::H263Decoder;

/// Locate `docs/video/h263/fixtures/<name>/`. Tests run with CWD set
/// to the crate root, so we walk two levels up to the workspace root
/// and then into `docs/`.
fn fixture_dir(name: &str) -> PathBuf {
    PathBuf::from("../../docs/video/h263/fixtures").join(name)
}

/// Aggregated diff stats for one frame's planes. `pct` is the fraction
/// of bytes that match exactly; `max` is the largest absolute
/// difference observed. PSNR is computed per-plane group (Y vs UV)
/// using the 8-bit reference (255^2 / MSE).
#[derive(Clone, Copy, Debug, Default)]
struct FrameDiff {
    y_total: usize,
    y_exact: usize,
    y_max: i32,
    y_sse: u64,
    uv_total: usize,
    uv_exact: usize,
    uv_max: i32,
    uv_sse: u64,
}

impl FrameDiff {
    fn pct(&self) -> f64 {
        let exact = self.y_exact + self.uv_exact;
        let total = self.y_total + self.uv_total;
        if total == 0 {
            0.0
        } else {
            exact as f64 / total as f64 * 100.0
        }
    }
    fn y_psnr(&self) -> f64 {
        psnr_db(self.y_total, self.y_sse)
    }
    fn uv_psnr(&self) -> f64 {
        psnr_db(self.uv_total, self.uv_sse)
    }
    fn merge(&mut self, other: &FrameDiff) {
        self.y_total += other.y_total;
        self.y_exact += other.y_exact;
        self.y_max = self.y_max.max(other.y_max);
        self.y_sse += other.y_sse;
        self.uv_total += other.uv_total;
        self.uv_exact += other.uv_exact;
        self.uv_max = self.uv_max.max(other.uv_max);
        self.uv_sse += other.uv_sse;
    }
}

fn psnr_db(total: usize, sse: u64) -> f64 {
    if total == 0 || sse == 0 {
        return f64::INFINITY;
    }
    let mse = sse as f64 / total as f64;
    10.0 * (255.0_f64 * 255.0_f64 / mse).log10()
}

/// Per-byte compare of two equal-length slices, returning
/// `(total, exact_matches, max_abs_diff, sum_squared_error)`.
fn cmp_bytes(a: &[u8], b: &[u8]) -> (usize, usize, i32, u64) {
    let n = a.len().min(b.len());
    let mut ex = 0usize;
    let mut max = 0i32;
    let mut sse: u64 = 0;
    for i in 0..n {
        let d = (a[i] as i32 - b[i] as i32).abs();
        if d == 0 {
            ex += 1;
        }
        if d > max {
            max = d;
        }
        sse += (d as u64) * (d as u64);
    }
    (n, ex, max, sse)
}

/// Diff three planes (Y, U, V) of a decoded frame against the
/// per-frame slice of `expected.yuv`.
fn diff_planes(our: (&[u8], &[u8], &[u8]), refp: (&[u8], &[u8], &[u8])) -> FrameDiff {
    let (yt, ye, ym, ys) = cmp_bytes(our.0, refp.0);
    let (ut, ue, um, us) = cmp_bytes(our.1, refp.1);
    let (vt, ve, vm, vs) = cmp_bytes(our.2, refp.2);
    FrameDiff {
        y_total: yt,
        y_exact: ye,
        y_max: ym,
        y_sse: ys,
        uv_total: ut + vt,
        uv_exact: ue + ve,
        uv_max: um.max(vm),
        uv_sse: us + vs,
    }
}

#[derive(Clone, Copy, Debug)]
enum Tier {
    /// Must decode bit-exactly. Test fails on any divergence.
    /// (Unused on day one — every fixture starts ReportOnly per task
    /// brief; promote individual entries here as follow-up rounds
    /// confirm bit-exact decode.)
    #[allow(dead_code)]
    BitExact,
    /// Decode is permitted to diverge from reference; we log the deltas
    /// but do not gate CI on it (the underlying bug is queued for
    /// follow-up).
    ReportOnly,
}

struct CorpusCase {
    name: &'static str,
    width: usize,
    height: usize,
    n_frames: usize,
    tier: Tier,
}

/// Repack a stride-packed `VideoFrame` into the row-major plane layout
/// libavcodec emits when dumping `-f rawvideo`: Y plane (W*H) || U
/// plane (Cw*Ch) || V plane (Cw*Ch). Strips any per-row padding so the
/// resulting buffer matches `expected.yuv` byte-for-byte.
fn videoframe_to_packed(
    vf: &oxideav_core::VideoFrame,
    w: usize,
    h: usize,
    cw: usize,
    ch: usize,
) -> Option<Vec<u8>> {
    if vf.planes.len() < 3 {
        return None;
    }
    fn pack(dst: &mut Vec<u8>, src: &[u8], stride: usize, cols: usize, rows: usize) -> bool {
        if stride < cols {
            return false;
        }
        if stride == cols {
            let n = cols * rows;
            if src.len() < n {
                return false;
            }
            dst.extend_from_slice(&src[..n]);
            return true;
        }
        for r in 0..rows {
            let start = r * stride;
            let end = start + cols;
            if end > src.len() {
                return false;
            }
            dst.extend_from_slice(&src[start..end]);
        }
        true
    }
    let mut out = Vec::with_capacity(w * h + 2 * cw * ch);
    if !pack(&mut out, &vf.planes[0].data, vf.planes[0].stride, w, h) {
        return None;
    }
    if !pack(&mut out, &vf.planes[1].data, vf.planes[1].stride, cw, ch) {
        return None;
    }
    if !pack(&mut out, &vf.planes[2].data, vf.planes[2].stride, cw, ch) {
        return None;
    }
    Some(out)
}

/// One frame's decode outcome.
type FrameResult = Result<FrameDiff, String>;

/// Drive a fresh `H263Decoder` over the provided H.263 elementary
/// stream byte slice and score each emitted frame against the matching
/// slice of `expected.yuv`.
///
/// `es_bytes` is the raw elementary stream (everything from the first
/// PSC onward). The H.263 stream is self-framing via the 22-bit Picture
/// Start Code, so feeding the entire stream as one packet is enough —
/// the decoder walks PSC / GBSC / SSC boundaries internally.
fn decode_es_to_results(case: &CorpusCase, es_bytes: Vec<u8>, yuv_ref: &[u8]) -> Vec<FrameResult> {
    let cw = case.width / 2;
    let ch = case.height / 2;
    let y_size = case.width * case.height;
    let uv_size = cw * ch;
    let frame_size = y_size + 2 * uv_size;

    if yuv_ref.len() != case.n_frames * frame_size {
        return vec![Err(format!(
            "expected.yuv size {} != {} (frames {} * frame_size {})",
            yuv_ref.len(),
            case.n_frames * frame_size,
            case.n_frames,
            frame_size
        ))];
    }

    let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
    let pkt = Packet {
        stream_index: 0,
        data: es_bytes,
        pts: Some(0),
        dts: Some(0),
        duration: None,
        time_base: TimeBase::new(1, 90_000),
        flags: PacketFlags {
            keyframe: true,
            ..PacketFlags::default()
        },
    };
    let mut results: Vec<FrameResult> = Vec::with_capacity(case.n_frames);
    if let Err(e) = dec.send_packet(&pkt) {
        results.push(Err(format!("send_packet: {e:?}")));
        return results;
    }
    if let Err(e) = dec.flush() {
        results.push(Err(format!("flush: {e:?}")));
        // Continue — some decoded frames may already be queued.
    }

    let mut visible_idx = 0usize;
    loop {
        match dec.receive_frame() {
            Ok(Frame::Video(vf)) => {
                if visible_idx >= case.n_frames {
                    visible_idx += 1;
                    continue;
                }
                let our = match videoframe_to_packed(&vf, case.width, case.height, cw, ch) {
                    Some(b) => b,
                    None => {
                        results.push(Err(format!(
                            "visible {visible_idx}: unexpected frame shape (planes={}, p0.stride={})",
                            vf.planes.len(),
                            vf.planes.first().map(|p| p.stride).unwrap_or(0)
                        )));
                        visible_idx += 1;
                        continue;
                    }
                };
                if our.len() != frame_size {
                    results.push(Err(format!(
                        "visible {visible_idx}: packed frame size {} != expected {} \
                         (W={} H={})",
                        our.len(),
                        frame_size,
                        case.width,
                        case.height
                    )));
                    visible_idx += 1;
                    continue;
                }
                let off = visible_idx * frame_size;
                let ref_y = &yuv_ref[off..off + y_size];
                let ref_u = &yuv_ref[off + y_size..off + y_size + uv_size];
                let ref_v = &yuv_ref[off + y_size + uv_size..off + frame_size];
                let our_y = &our[..y_size];
                let our_u = &our[y_size..y_size + uv_size];
                let our_v = &our[y_size + uv_size..frame_size];
                let d = diff_planes((our_y, our_u, our_v), (ref_y, ref_u, ref_v));
                results.push(Ok(d));
                visible_idx += 1;
            }
            Ok(_) => continue,
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => {
                results.push(Err(format!("visible {visible_idx}: receive_frame: {e:?}")));
                break;
            }
        }
    }

    if visible_idx < case.n_frames {
        results.push(Err(format!(
            "decoder produced {} visible frames, expected {} — short by {}",
            visible_idx,
            case.n_frames,
            case.n_frames - visible_idx
        )));
    } else if visible_idx > case.n_frames {
        eprintln!(
            "[note] {}: decoder produced {} visible frames, expected {} (extras dropped)",
            case.name, visible_idx, case.n_frames
        );
    }

    results
}

/// Read `input.h263` + `expected.yuv` for a fixture, then decode and
/// score. Returns `None` if either file is missing on disk (CI without
/// docs/ stays green).
fn decode_fixture(case: &CorpusCase) -> Option<Vec<FrameResult>> {
    let dir = fixture_dir(case.name);
    let h263_path = dir.join("input.h263");
    let yuv_path = dir.join("expected.yuv");
    let h263 = match fs::read(&h263_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skip {}: missing {} ({e})", case.name, h263_path.display());
            return None;
        }
    };
    let yuv_ref = match fs::read(&yuv_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skip {}: missing {} ({e})", case.name, yuv_path.display());
            return None;
        }
    };
    Some(decode_es_to_results(case, h263, &yuv_ref))
}

/// Pretty-print + tier-aware assertion.
fn evaluate_results(case: &CorpusCase, results: Vec<FrameResult>) {
    let mut agg = FrameDiff::default();
    let mut errors: Vec<String> = Vec::new();
    let mut decoded_frames = 0usize;
    for (i, r) in results.iter().enumerate() {
        match r {
            Ok(d) => {
                decoded_frames += 1;
                eprintln!(
                    "  frame {i}: Y {}/{} exact (max diff {}, PSNR {:.2} dB), \
                     UV {}/{} exact (max diff {}, PSNR {:.2} dB), pct={:.2}%",
                    d.y_exact,
                    d.y_total,
                    d.y_max,
                    d.y_psnr(),
                    d.uv_exact,
                    d.uv_total,
                    d.uv_max,
                    d.uv_psnr(),
                    d.pct()
                );
                agg.merge(d);
            }
            Err(e) => {
                eprintln!("  frame {i}: ERROR {e}");
                errors.push(format!("frame {i}: {e}"));
            }
        }
    }

    eprintln!(
        "[{:?}] {}: decoded {} of {} frames; aggregate Y PSNR {:.2} dB ({:.2}% match, max diff {}), \
         UV PSNR {:.2} dB ({}/{} exact, max diff {}), {} errors",
        case.tier,
        case.name,
        decoded_frames,
        case.n_frames,
        agg.y_psnr(),
        if agg.y_total == 0 {
            0.0
        } else {
            agg.y_exact as f64 / agg.y_total as f64 * 100.0
        },
        agg.y_max,
        agg.uv_psnr(),
        agg.uv_exact,
        agg.uv_total,
        agg.uv_max,
        errors.len(),
    );

    match case.tier {
        Tier::BitExact => {
            assert!(
                errors.is_empty(),
                "{}: {} frame errors prevented bit-exact comparison: {:?}",
                case.name,
                errors.len(),
                errors
            );
            let total = agg.y_total + agg.uv_total;
            let exact = agg.y_exact + agg.uv_exact;
            assert_eq!(
                exact,
                total,
                "{}: not bit-exact (Y max diff {}, UV max diff {}; {:.4}% match)",
                case.name,
                agg.y_max,
                agg.uv_max,
                agg.pct()
            );
        }
        Tier::ReportOnly => {
            // Don't fail. The eprintln! above is the human-readable
            // diagnostic; CI scrapes it to track per-fixture progress.
        }
    }
}

fn evaluate(case: &CorpusCase) {
    let results = match decode_fixture(case) {
        Some(r) => r,
        None => return,
    };
    evaluate_results(case, results);
}

/// Walk a 3GP / ISO BMFF byte slice by hand and pull out the first
/// H.263 elementary-stream payload (everything from the first PSC up
/// to the next PSC, or to end-of-file). We do this without depending
/// on `oxideav-mp4` to keep this dev-dep-free, mirroring the rationale
/// in `tests/mp4_3gp_iframe.rs`. The H.263 payload inside `mdat` is a
/// run of one or more samples each beginning with a 22-bit Picture
/// Start Code (`00 00 80..83`).
fn first_h263_es_from_3gp(data: &[u8]) -> Option<Vec<u8>> {
    for i in 0..data.len().saturating_sub(3) {
        if data[i] == 0x00 && data[i + 1] == 0x00 && (data[i + 2] & 0xFC) == 0x80 {
            let mut end = data.len();
            for j in (i + 3)..data.len().saturating_sub(3) {
                if data[j] == 0x00 && data[j + 1] == 0x00 && (data[j + 2] & 0xFC) == 0x80 {
                    end = j;
                    break;
                }
            }
            return Some(data[i..end].to_vec());
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Per-fixture tests
// ---------------------------------------------------------------------------
//
// Day-one round ships every fixture as ReportOnly per the task brief
// (#234 / #249 / #250 wired vp8 / h264 / av1 the same way). Each test
// exercises one bitstream feature dimension; the per-fixture eprintln
// records whether we decoded all frames, plus per-plane match-pct and
// PSNR. Promote individual entries to Tier::BitExact once a follow-up
// confirms the decode matches libavcodec byte-for-byte.

// --- Baseline H.263 (§5) — the simplest fixtures ---

#[test]
fn corpus_tiny_i_only_sqcif_baseline() {
    // Sub-QCIF (128 x 96), single I-picture, PQUANT=3, all annex bits
    // cleared. The smallest possible H.263 stream — exercises just the
    // PSC + picture header + intra MB layer + IDCT.
    evaluate(&CorpusCase {
        name: "tiny-i-only-sqcif-baseline",
        width: 128,
        height: 96,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_i_only_qcif_baseline() {
    // QCIF (176 x 144), single I-picture, PQUANT=8. The most common
    // H.263 fixture shape.
    evaluate(&CorpusCase {
        name: "i-only-qcif-baseline",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_i_only_cif_baseline() {
    // CIF (352 x 288), single I-picture, PQUANT=12. 22 x 18 = 396 MBs.
    evaluate(&CorpusCase {
        name: "i-only-cif-baseline",
        width: 352,
        height: 288,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_i_frame_then_p_frame_qcif() {
    // I + 2 P frames at QCIF. Exercises COD/MCBPC/median-MV path
    // (§5.3.5 / §5.3.7 / §6.1.1).
    evaluate(&CorpusCase {
        name: "i-frame-then-p-frame-qcif",
        width: 176,
        height: 144,
        n_frames: 3,
        tier: Tier::ReportOnly,
    });
}

// --- Quantizer extremes — same baseline path, different QP. ---

#[test]
fn corpus_qp_low() {
    // PQUANT = 2, near-lossless QCIF I-frame. Largest fixture in the
    // corpus (~10.9 KB) — stresses the AC TCOEF VLC density.
    evaluate(&CorpusCase {
        name: "qp-low",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_qp_high() {
    // PQUANT = 31, max-quant QCIF I-frame. Smallest fixture (~1.8 KB);
    // exercises the high-QP branch of dequant + the rare-but-legal
    // mostly-zero AC stream.
    evaluate(&CorpusCase {
        name: "qp-high",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

// --- H.263+ Annex-bearing fixtures (PLUSPTYPE picture header). ---

#[test]
fn corpus_unrestricted_mv_mode() {
    // Annex D — Unrestricted Motion Vectors (UMV+) on H.263+ PLUSPTYPE
    // PCF + slice-structured mode auto-enabled by ffmpeg threading.
    // 3 QCIF frames.
    // TODO(h263-corpus): decoder must parse PLUSPTYPE OPPTYPE bit UMV
    // + Custom PCF + slice-struct headers concurrently — track via the
    // per-fixture pct in CI.
    evaluate(&CorpusCase {
        name: "unrestricted-mv-mode",
        width: 176,
        height: 144,
        n_frames: 3,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_advanced_prediction_mode() {
    // Annex F — Advanced Prediction (4MV + OBMC). PTYPE bit 13 in
    // baseline (no PLUSPTYPE) header form. 3 QCIF frames.
    // TODO(h263-corpus): exercises the §F.3 OBMC blending path.
    evaluate(&CorpusCase {
        name: "advanced-prediction-mode",
        width: 176,
        height: 144,
        n_frames: 3,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_advanced_intra_coding() {
    // Annex I (AIC) + Annex T (ModQuant) on a single QCIF I-picture
    // inside a PLUSPTYPE header. Exercises §I.3 AC prediction +
    // §I.4 dequant + §T.3 DQUANT delta interpretation.
    evaluate(&CorpusCase {
        name: "advanced-intra-coding",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_alt_inter_vlc() {
    // Annex S (Alternative Inter VLC) layered on AIC + ModQuant +
    // slice-struct, 3 QCIF frames. The §S CBPY no-XOR branch fires on
    // any inter MB whose `cbpc & 3 == 3` — track via pct.
    evaluate(&CorpusCase {
        name: "alt-inter-vlc",
        width: 176,
        height: 144,
        n_frames: 3,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_deblocking_filter() {
    // Annex J (loop filter) on H.263+ PLUSPTYPE, 3 QCIF frames. Decoder
    // auto-enables the in-loop deblocker via the OPPTYPE DF bit.
    evaluate(&CorpusCase {
        name: "deblocking-filter",
        width: 176,
        height: 144,
        n_frames: 3,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_slice_structured_mode() {
    // Annex K (slice-struct) replaces GOB layer with §K.2 slice
    // headers (SSC + MBA + SQUANT + GFID), 3 QCIF frames.
    evaluate(&CorpusCase {
        name: "slice-structured-mode",
        width: 176,
        height: 144,
        n_frames: 3,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_h263p_modern() {
    // H.263+ baseline (PLUSPTYPE form, no annexes — only Custom PCF).
    // Minimum H.263+ (1998) compliance fixture, 3 QCIF frames.
    evaluate(&CorpusCase {
        name: "h263p-modern",
        width: 176,
        height: 144,
        n_frames: 3,
        tier: Tier::ReportOnly,
    });
}

// --- 3GP container (RTP/3GP framing variant of i-only-qcif-baseline). ---
//
// `containerless-elementary-vs-3gp` ships TWO bitstream files:
// the raw `input.h263` and a 3GP-wrapped `input.3gp` whose H.263
// payload is byte-identical. We test both — the decoder MUST produce
// the same `expected.yuv` from either.

#[test]
fn corpus_containerless_elementary_vs_3gp_raw() {
    // The raw .h263 half — same elementary stream as
    // i-only-qcif-baseline (sha-256 fd75…b2e5c1).
    evaluate(&CorpusCase {
        name: "containerless-elementary-vs-3gp",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_containerless_elementary_vs_3gp_mp4() {
    // The 3GP-wrapped half — extract the H.263 elementary stream from
    // `input.3gp` via the inline first-PSC-in-mdat scanner and decode
    // against the same expected.yuv.
    let case = CorpusCase {
        name: "containerless-elementary-vs-3gp",
        width: 176,
        height: 144,
        n_frames: 1,
        tier: Tier::ReportOnly,
    };
    let dir = fixture_dir(case.name);
    let mp4_path = dir.join("input.3gp");
    let yuv_path = dir.join("expected.yuv");
    let mp4 = match fs::read(&mp4_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!(
                "skip {} (3gp): missing {} ({e})",
                case.name,
                mp4_path.display()
            );
            return;
        }
    };
    let yuv_ref = match fs::read(&yuv_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!(
                "skip {} (3gp): missing {} ({e})",
                case.name,
                yuv_path.display()
            );
            return;
        }
    };
    let es = match first_h263_es_from_3gp(&mp4) {
        Some(b) => b,
        None => {
            eprintln!(
                "skip {} (3gp): no PSC found inside container payload",
                case.name
            );
            return;
        }
    };
    let results = decode_es_to_results(&case, es, &yuv_ref);
    evaluate_results(&case, results);
}
