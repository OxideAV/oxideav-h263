//! Annex V Data-Partitioned Slice end-to-end tests: the partitioned
//! layout must round-trip closed-loop byte-exact through the DPS
//! decoder, agree pixel-for-pixel with the interleaved (plain
//! Annex K) coding of the same content, and enforce its marker /
//! redundancy structure.

use oxideav_h263::encoder::{encode_inter_picture_dps, encode_intra_picture_dps};
use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};

fn textured(seed: usize) -> YuvFrame {
    let mut f = YuvFrame::grey(176, 144);
    for y in 0..144usize {
        for x in 0..176usize {
            f.y[y * 176 + x] =
                ((x * 5 + y * 3 + seed * 9) & 0xFF) as u8 ^ (((x / 8) & 1) as u8 * 40);
        }
    }
    for y in 0..72usize {
        for x in 0..88usize {
            f.cb[y * 88 + x] = (100 + ((x + seed) % 32)) as u8;
            f.cr[y * 88 + x] = (150 - ((y + seed) % 32)) as u8;
        }
    }
    f
}

/// A shifted copy so P-pictures carry motion (and MVDs exercise the
/// single prediction thread).
fn shifted(base: &YuvFrame, dx: usize, dy: usize) -> YuvFrame {
    let mut f = YuvFrame::grey(176, 144);
    for y in 0..144usize {
        for x in 0..176usize {
            let sx = (x + dx).min(175);
            let sy = (y + dy).min(143);
            f.y[y * 176 + x] = base.y[sy * 176 + sx];
        }
    }
    for y in 0..72usize {
        for x in 0..88usize {
            let sx = (x + dx / 2).min(87);
            let sy = (y + dy / 2).min(71);
            f.cb[y * 88 + x] = base.cb[sy * 88 + sx];
            f.cr[y * 88 + x] = base.cr[sy * 88 + sx];
        }
    }
    f
}

fn frame_bytes(f: &YuvFrame) -> Vec<u8> {
    let mut v = f.y.clone();
    v.extend_from_slice(&f.cb);
    v.extend_from_slice(&f.cr);
    v
}

fn encode_dps_stream(rows_per_slice: usize) -> (Vec<u8>, Vec<YuvFrame>) {
    let f0 = textured(0);
    let f1 = shifted(&f0, 3, 2);
    let f2 = shifted(&f0, 6, 4);
    let mut stream = encode_intra_picture_dps(&f0, 7, 0, rows_per_slice).expect("I");
    let mut recons = decode_sequence(&stream, DecodeOptions::default()).expect("decode I");
    for (i, f) in [f1, f2].into_iter().enumerate() {
        let p = encode_inter_picture_dps(
            &f,
            recons.last().expect("recon"),
            7,
            (i + 1) as u8,
            8,
            rows_per_slice,
        )
        .expect("P");
        stream.extend_from_slice(&p);
        recons = decode_sequence(&stream, DecodeOptions::default()).expect("decode");
    }
    (stream, recons)
}

#[test]
fn dps_stream_round_trips_closed_loop() {
    for rows_per_slice in [1usize, 3, 9] {
        let (stream, recons) = encode_dps_stream(rows_per_slice);
        let decoded = decode_sequence(&stream, DecodeOptions::default()).expect("decode");
        assert_eq!(decoded.len(), 3, "rows_per_slice {rows_per_slice}");
        for (i, (d, r)) in decoded.iter().zip(recons.iter()).enumerate() {
            assert_eq!(
                frame_bytes(d),
                frame_bytes(r),
                "picture {i} at {rows_per_slice} rows/slice"
            );
        }
    }
}

#[test]
fn dps_intra_matches_plain_slice_coding_pixels() {
    // The partitioning rearranges the syntax but not the maths: the
    // same content coded through the interleaved Annex K INTRA slice
    // encoder must reconstruct byte-identically (same transform,
    // same quantiser, same slice segmentation).
    use oxideav_h263::encoder::encode_intra_picture_slices;
    let f0 = textured(4);
    let dps = encode_intra_picture_dps(&f0, 9, 0, 3).expect("DPS");
    let plain = encode_intra_picture_slices(&f0, 0, 3, |_| 9).expect("plain slices");
    let d1 = decode_sequence(&dps, DecodeOptions::default()).expect("decode DPS");
    let d2 = decode_sequence(&plain, DecodeOptions::default()).expect("decode plain");
    assert_eq!(frame_bytes(&d1[0]), frame_bytes(&d2[0]));
    // And the DPS stream is a different wire layout.
    assert_ne!(dps, plain);
}

#[test]
fn corrupt_markers_and_lmvv_are_rejected() {
    let (stream, _) = encode_dps_stream(9);
    let plain_ok = decode_sequence(&stream, DecodeOptions::default());
    assert!(plain_ok.is_ok());

    // Flipping any single bit in the second picture must not panic,
    // and the redundancy structure catches many corruptions as hard
    // errors rather than silent mis-decodes. (A flip inside
    // coefficient data can still decode to different pixels — DPS
    // localises, it does not detect everything.)
    let second = oxideav_h263::picture::next_picture_start_code(&stream, 1).expect("PSC 2");
    let mut corrupt_count = 0usize;
    for bit in 0..64usize {
        let mut mutated = stream.clone();
        mutated[second + 8 + bit / 8] ^= 0x80 >> (bit % 8);
        if decode_sequence(&mutated, DecodeOptions::default()).is_err() {
            corrupt_count += 1;
        }
    }
    assert!(
        corrupt_count > 0,
        "some corruptions must surface as structural errors"
    );
}
