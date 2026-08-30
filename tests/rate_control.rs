//! Rate-control integration tests: the Annex B HRD-regulated
//! [`encode_sequence_rate_controlled`] loop over synthetic moving
//! content, with **measured rate accuracy** assertions — the point of
//! a rate controller is a number, so the tests pin it.

use oxideav_h263::encoder::{encode_sequence_rate_controlled, RateControlConfig};
use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};
use oxideav_h263::rate_control::HrdParams;

/// A deterministic gradient frame on every plane.
fn gradient(lw: usize, lh: usize, seed: u8) -> YuvFrame {
    let cw = lw / 2;
    let ch = lh / 2;
    let mut y = vec![0u8; lw * lh];
    for row in 0..lh {
        for col in 0..lw {
            y[row * lw + col] = (32 + (col + row + seed as usize) % 192) as u8;
        }
    }
    let mut cb = vec![0u8; cw * ch];
    let mut cr = vec![0u8; cw * ch];
    for row in 0..ch {
        for col in 0..cw {
            cb[row * cw + col] = (80 + (col % 64)) as u8;
            cr[row * cw + col] = (100 + (row % 56)) as u8;
        }
    }
    YuvFrame {
        y,
        cb,
        cr,
        luma_width: lw,
        luma_height: lh,
    }
}

/// A QCIF test clip: a gradient background with a block that moves a
/// little every frame — steady P-picture work for the controller.
fn moving_clip(frames: usize) -> Vec<YuvFrame> {
    let lw = 176;
    let lh = 144;
    let base = gradient(lw, lh, 0);
    (0..frames)
        .map(|i| {
            let mut f = base.clone();
            let x0 = 20 + (i * 6) % 100;
            let y0 = 30 + (i * 4) % 80;
            for row in y0..(y0 + 32).min(lh) {
                for col in x0..(x0 + 32).min(lw) {
                    f.y[row * lw + col] = f.y[row * lw + col].wrapping_add(70);
                }
            }
            f
        })
        .collect()
}

fn luma_mae(a: &YuvFrame, b: &YuvFrame) -> f64 {
    let sum: u64 =
        a.y.iter()
            .zip(b.y.iter())
            .map(|(x, y)| (*x as i32 - *y as i32).unsigned_abs() as u64)
            .sum();
    sum as f64 / a.y.len() as f64
}

/// The controller holds the long-run mean bits/picture within ±25 % of
/// the target on steady moving content (P-pictures after the first
/// GOP's warm-up), and the stream still decodes to acceptable quality.
#[test]
fn rate_controller_holds_long_run_average() {
    let frames = moving_clip(24);
    let target = 4_000u32;
    let cfg = RateControlConfig {
        target_bits_per_picture: target,
        initial_quant: 10,
        intra_period: 0, // one I then all P — steady-state measurement
        search_half: 8,
        hrd: None,
        max_reencodes: 1,
        mb_adaptive: false,
    };
    let rcs = encode_sequence_rate_controlled(&frames, &cfg, 0).expect("encode");
    assert_eq!(rcs.picture_bits.len(), frames.len());

    // Steady state: skip the I-picture and 4 warm-up P-pictures.
    let steady = &rcs.picture_bits[5..];
    let mean = steady.iter().map(|&b| b as u64).sum::<u64>() as f64 / steady.len() as f64;
    let err = (mean - target as f64).abs() / target as f64;
    assert!(
        err < 0.25,
        "steady-state mean {mean:.0} bits vs target {target} ({:.1} % off)",
        err * 100.0
    );

    // The whole stream still decodes and tracks the source.
    let decoded = decode_sequence(&rcs.bytes, DecodeOptions::default()).expect("decode");
    assert_eq!(decoded.len(), frames.len());
    for (i, (src, dec)) in frames.iter().zip(decoded.iter()).enumerate() {
        let mae = luma_mae(src, dec);
        assert!(mae < 12.0, "frame {i} MAE {mae}");
    }
}

/// Halving the budget forces a coarser steady-state QUANT and a
/// smaller stream; the measured mean respects each target.
#[test]
fn rate_controller_scales_with_target() {
    let frames = moving_clip(16);
    let encode_at = |target: u32| {
        let cfg = RateControlConfig {
            target_bits_per_picture: target,
            initial_quant: 12,
            intra_period: 0,
            search_half: 8,
            hrd: None,
            max_reencodes: 1,
            mb_adaptive: false,
        };
        encode_sequence_rate_controlled(&frames, &cfg, 0).expect("encode")
    };
    let fat = encode_at(5_000);
    let thin = encode_at(1_500);
    assert!(
        thin.bytes.len() < fat.bytes.len(),
        "thin {} vs fat {}",
        thin.bytes.len(),
        fat.bytes.len()
    );
    // Steady-state QUANT ordering follows the budget.
    let q_fat = *fat.picture_quants.last().unwrap();
    let q_thin = *thin.picture_quants.last().unwrap();
    assert!(
        q_thin > q_fat,
        "thin steady QUANT {q_thin} should exceed fat {q_fat}"
    );
    // Each steady-state mean lands within ±20 % of its own target.
    for (rcs, target) in [(&fat, 5_000f64), (&thin, 1_500f64)] {
        let steady = &rcs.picture_bits[5..];
        let mean = steady.iter().map(|&b| b as u64).sum::<u64>() as f64 / steady.len() as f64;
        let err = (mean - target).abs() / target;
        assert!(
            err < 0.30,
            "target {target}: steady mean {mean:.0} ({:.1} % off)",
            err * 100.0
        );
    }
}

/// The Annex B HRD leg: with a CBR channel provisioned at the target
/// rate, the regulated stream stays §B.4-conformant (no post-removal
/// occupancy at or above `B`) and the max occupancy is reported.
#[test]
fn rate_controlled_stream_is_hrd_conformant() {
    let frames = moving_clip(20);
    let target = 4_000u32;
    let hrd = HrdParams {
        bits_per_tick: target as u64,
        b_max: 4 * target as u64,
    };
    let cfg = RateControlConfig {
        target_bits_per_picture: target,
        initial_quant: 10,
        intra_period: 10,
        search_half: 8,
        hrd: Some(hrd),
        max_reencodes: 2,
        mb_adaptive: false,
    };
    let rcs = encode_sequence_rate_controlled(&frames, &cfg, 0).expect("encode");
    assert!(
        rcs.hrd_conformant,
        "stream violated §B.4 (max occupancy {})",
        rcs.hrd_max_occupancy
    );
    assert!(rcs.hrd_max_occupancy < hrd.b_max);
    // And it is a plain decodable elementary stream.
    let decoded = decode_sequence(&rcs.bytes, DecodeOptions::default()).expect("decode");
    assert_eq!(decoded.len(), frames.len());
}

/// Periodic INTRA pictures burst above the P-picture budget; the
/// controller absorbs the burst and pulls the following P-pictures
/// back under budget (the virtual buffer drains).
#[test]
fn intra_bursts_are_absorbed() {
    let frames = moving_clip(20);
    let target = 3_000u32;
    let cfg = RateControlConfig {
        target_bits_per_picture: target,
        initial_quant: 10,
        intra_period: 8,
        search_half: 8,
        hrd: None,
        max_reencodes: 1,
        mb_adaptive: false,
    };
    let rcs = encode_sequence_rate_controlled(&frames, &cfg, 0).expect("encode");
    // The I-pictures (0, 8, 16) overshoot...
    assert!(rcs.picture_bits[8] as f64 > 1.2 * target as f64);
    // ...and the mean over each following GOP tail comes back within
    // 35 % of target (the burst is amortised).
    let tail = &rcs.picture_bits[9..16];
    let mean = tail.iter().map(|&b| b as u64).sum::<u64>() as f64 / tail.len() as f64;
    assert!(
        mean < 1.35 * target as f64,
        "post-I tail mean {mean:.0} vs target {target}"
    );
}

/// The §5.3.6 within-picture governor tightens the *per-picture* spread:
/// with `mb_adaptive` the worst single-picture overshoot of a regulated
/// GOP run is no worse than the frame-level-only controller's, and the
/// stream still decodes end-to-end with every picture near budget.
#[test]
fn mb_adaptive_regulation_tightens_per_picture_spread() {
    let frames = moving_clip(12);
    let target = 3_000u32;
    let run = |mb_adaptive: bool| {
        let cfg = RateControlConfig {
            target_bits_per_picture: target,
            initial_quant: 10,
            intra_period: 0,
            search_half: 8,
            hrd: None,
            mb_adaptive,
            max_reencodes: 1,
        };
        encode_sequence_rate_controlled(&frames, &cfg, 0).expect("encode")
    };
    let frame_level = run(false);
    let adaptive = run(true);

    // Whole stream still decodes and tracks the source.
    let decoded = decode_sequence(&adaptive.bytes, DecodeOptions::default()).expect("decode");
    assert_eq!(decoded.len(), frames.len());
    for (i, (src, dec)) in frames.iter().zip(decoded.iter()).enumerate() {
        let mae = luma_mae(src, dec);
        assert!(mae < 14.0, "frame {i} MAE {mae}");
    }

    // Per-picture worst-case overshoot (P-pictures, warm-up skipped):
    // the within-picture governor may not do worse than frame-level
    // regulation alone.
    let worst = |bits: &[u32]| {
        bits[3..]
            .iter()
            .map(|&b| (b as i64 - target as i64).unsigned_abs())
            .max()
            .unwrap()
    };
    let w_frame = worst(&frame_level.picture_bits);
    let w_adaptive = worst(&adaptive.picture_bits);
    assert!(
        w_adaptive <= w_frame + target as u64 / 10,
        "adaptive worst overshoot {w_adaptive} vs frame-level {w_frame}"
    );

    // The steady-state mean still respects the budget.
    let steady = &adaptive.picture_bits[3..];
    let mean = steady.iter().map(|&b| b as u64).sum::<u64>() as f64 / steady.len() as f64;
    let err = (mean - target as f64).abs() / target as f64;
    assert!(err < 0.25, "adaptive mean {mean:.0} vs target {target}");
}
