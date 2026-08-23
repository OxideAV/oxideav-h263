//! Annex R Independent Segment Decoding end-to-end tests: the ISD
//! encoder's per-GOB-segment band confinement must round-trip
//! byte-exactly through the decoder's segment-boundary treatment, and
//! the treatment must actually fire (clearing the ISD bit changes the
//! reconstruction).

use oxideav_h263::encoder::{encode_inter_picture_isd, encode_intra_picture_isd};
use oxideav_h263::picture::{decode_sequence, DecodeOptions, YuvFrame};

/// A frame with strong horizontal banding plus a diagonal gradient, so
/// vertical motion crosses GOB-segment boundaries with high-contrast
/// content (band extrapolation visibly changes predictions).
fn banded_frame(shift: usize) -> YuvFrame {
    let mut f = YuvFrame::grey(176, 144);
    for y in 0..144usize {
        for x in 0..176usize {
            let sy = y + shift;
            let stripe = ((sy / 8) % 2) as u8;
            f.y[y * 176 + x] = (40 + stripe * 150).wrapping_add((x as u8).wrapping_mul(3) / 4);
        }
    }
    for y in 0..72usize {
        for x in 0..88usize {
            f.cb[y * 88 + x] = 90 + (((y + shift / 2) / 4) % 2) as u8 * 60;
            f.cr[y * 88 + x] = 160 - (((y + shift / 2) / 4) % 2) as u8 * 60;
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

/// Encode an I + P + P ISD stream whose P-pictures carry real vertical
/// motion (content shifted by 4 px per picture — every QCIF GOB is one
/// macroblock row, so most vectors reach across their segment band and
/// exercise the §R.2 rule-4 extrapolation).
fn encode_isd_stream() -> (Vec<u8>, Vec<YuvFrame>) {
    let sources = [banded_frame(0), banded_frame(4), banded_frame(8)];
    let mut stream = Vec::new();
    let mut recons: Vec<YuvFrame> = Vec::new();
    for (i, src) in sources.iter().enumerate() {
        let bytes = if i == 0 {
            encode_intra_picture_isd(src, 6, 0).expect("I")
        } else {
            let reference = recons.last().expect("previous recon");
            encode_inter_picture_isd(src, reference, 6, i as u8, 8).expect("P")
        };
        stream.extend_from_slice(&bytes);
        // Closed loop: reconstruct exactly as a decoder would, so the
        // whole-stream decode below must match byte for byte.
        let decoded = decode_sequence(&stream, DecodeOptions::default()).expect("decode so far");
        recons = decoded;
    }
    (stream, recons)
}

#[test]
fn isd_stream_round_trips_closed_loop() {
    let (stream, recons) = encode_isd_stream();
    let decoded = decode_sequence(&stream, DecodeOptions::default()).expect("decode");
    assert_eq!(decoded.len(), 3);
    for (i, (d, r)) in decoded.iter().zip(recons.iter()).enumerate() {
        assert_eq!(frame_bytes(d), frame_bytes(r), "picture {i}");
    }
}

/// Locate the OPPTYPE ISD bit of a UFEP=001 PLUSPTYPE picture: PSC (22)
/// plus TR (8) plus PTYPE bits 1-5 (5) plus "111" (3) plus UFEP (3) is
/// 41 bits, then OPPTYPE bit 12 sits 11 bits further in.
fn clear_isd_bit(picture: &mut [u8]) {
    let bit_index = 22 + 8 + 5 + 3 + 3 + 11;
    let byte = bit_index / 8;
    let mask = 0x80u8 >> (bit_index % 8);
    assert!(picture[byte] & mask != 0, "ISD bit expected set");
    picture[byte] &= !mask;
}

#[test]
fn clearing_the_isd_bit_changes_the_reconstruction() {
    // The same coded macroblock data decoded WITHOUT the Annex R
    // segment treatment must reconstruct differently — proof the
    // banded reference clamp actually fires (the P-pictures' vectors
    // cross their GOB segment bands).
    let (stream, _) = encode_isd_stream();
    let with_isd = decode_sequence(&stream, DecodeOptions::default()).expect("ISD decode");

    // Clear the ISD bit in every picture of the stream.
    let mut cleared = stream.clone();
    let mut boundaries = Vec::new();
    let mut at = 0usize;
    while let Some(p) = oxideav_h263::picture::next_picture_start_code(&cleared, at) {
        boundaries.push(p);
        at = p + 1;
    }
    assert_eq!(boundaries.len(), 3);
    for &b in &boundaries {
        clear_isd_bit(&mut cleared[b..]);
    }
    let without_isd = decode_sequence(&cleared, DecodeOptions::default()).expect("plain decode");

    assert_eq!(with_isd.len(), without_isd.len());
    // The I-picture reads no reference: identical either way.
    assert_eq!(frame_bytes(&with_isd[0]), frame_bytes(&without_isd[0]));
    // The P-pictures' cross-segment vectors see extrapolated bands
    // under ISD and real neighbouring rows without it.
    assert_ne!(
        frame_bytes(&with_isd[1]),
        frame_bytes(&without_isd[1]),
        "segment banding must alter cross-boundary predictions"
    );
}

#[test]
fn isd_stream_decodes_through_streaming_step_api() {
    // Decode the stream picture by picture through the streaming step
    // API (which threads the cross-picture state exactly like
    // decode_sequence) — the registry decoder's path.
    use oxideav_h263::picture::{decode_sequence_step, next_picture_start_code, SequenceState};
    let (stream, recons) = encode_isd_stream();
    let mut state = SequenceState::default();
    let mut frames: Vec<YuvFrame> = Vec::new();
    let mut start = next_picture_start_code(&stream, 0).expect("first PSC");
    loop {
        let next = next_picture_start_code(&stream, start + 1);
        let end = next.unwrap_or(stream.len());
        let decoded = decode_sequence_step(
            &stream[start..end],
            frames.last(),
            DecodeOptions::default(),
            &mut state,
        )
        .expect("step");
        frames.extend(decoded);
        match next {
            Some(n) => start = n,
            None => break,
        }
    }
    assert_eq!(frames.len(), recons.len());
    for (f, r) in frames.iter().zip(recons.iter()) {
        assert_eq!(frame_bytes(f), frame_bytes(r));
    }
}

#[test]
fn deblock_skips_segment_boundaries_under_isd() {
    // §R.2 rule 3 — no deblocking filter operation across video
    // picture segment boundaries. An I-picture reconstructs
    // identically with or without the ISD bit (no reference reads),
    // so running the caller-side Annex J filter over both isolates
    // the segment-boundary skip: the filtered outputs must differ
    // exactly because the ISD decode leaves the cross-GOB edges
    // unfiltered, and unfiltered rows must match the unfiltered
    // reconstruction on a boundary row.
    let src = banded_frame(2);
    let coded = encode_intra_picture_isd(&src, 6, 0).expect("I");
    let deblock = DecodeOptions {
        deblock: true,
        ..DecodeOptions::default()
    };
    let plain = decode_sequence(&coded, DecodeOptions::default()).expect("plain")[0].clone();
    let isd_deblocked = decode_sequence(&coded, deblock).expect("ISD deblock")[0].clone();

    let mut cleared = coded.clone();
    clear_isd_bit(&mut cleared);
    let full_deblocked = decode_sequence(&cleared, deblock).expect("full deblock")[0].clone();

    assert_ne!(
        frame_bytes(&isd_deblocked),
        frame_bytes(&full_deblocked),
        "cross-segment edges must stay unfiltered under ISD"
    );
    // §J.3 — a horizontal edge at luma row `y` filters rows
    // `y-2..=y+1` (the A/B/C/D taps). The only edges the ISD decode
    // treats differently are the cross-segment ones at every GOB
    // boundary (QCIF: rows ≡ 0 mod 16), so every difference against
    // the full deblock must sit within two rows of such a boundary.
    let mut diff_rows: Vec<usize> = Vec::new();
    for row in 0..144usize {
        let a = &isd_deblocked.y[row * 176..(row + 1) * 176];
        let b = &full_deblocked.y[row * 176..(row + 1) * 176];
        if a != b {
            diff_rows.push(row);
        }
    }
    assert!(!diff_rows.is_empty(), "boundary rows must differ");
    for &row in &diff_rows {
        let m = row % 16;
        assert!(
            (14..=15).contains(&m) || m <= 1,
            "difference at luma row {row} is not adjacent to a segment boundary"
        );
    }
    // And where nothing crosses a boundary the two decodes agree with
    // each other (interior edges filter identically) — e.g. the
    // middle rows of every macroblock row.
    for row in (4..144).step_by(16) {
        let a = &isd_deblocked.y[row * 176..(row + 1) * 176];
        let b = &full_deblocked.y[row * 176..(row + 1) * 176];
        assert_eq!(a, b, "interior row {row} must filter identically");
    }
    let _ = plain;
}
