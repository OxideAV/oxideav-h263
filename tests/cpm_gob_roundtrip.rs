//! §5.1.20 / §5.1.21 / §5.2.4 — Continuous Presence Multipoint on the
//! GOB path: CPM = "1" + PSBI in the picture header, GSBI in every GOB
//! header, both directions.

use oxideav_h263::encoder::{
    encode_inter_picture_gobs, encode_inter_picture_gobs_cpm, encode_intra_picture_gobs,
    encode_intra_picture_gobs_cpm, EOS_BYTES,
};
use oxideav_h263::picture::{
    decode_picture_no_gob0_header, decode_sequence, DecodeOptions, YuvFrame,
};
use oxideav_h263::Error;

fn textured(lw: usize, lh: usize, seed: usize) -> YuvFrame {
    let cw = lw / 2;
    let ch = lh / 2;
    let mut y = vec![0u8; lw * lh];
    for r in 0..lh {
        for c in 0..lw {
            let v = (r * 5 + c * 3 + seed * 11) % 256;
            let checker = if ((r / 8) + (c / 8)) % 2 == 0 { 30 } else { 0 };
            y[r * lw + c] = ((v + checker) % 256) as u8;
        }
    }
    let cb = (0..cw * ch).map(|i| (90 + (i % 50) + seed) as u8).collect();
    let cr = (0..cw * ch)
        .map(|i| (160 - (i % 40) + seed) as u8)
        .collect();
    YuvFrame {
        y,
        cb,
        cr,
        luma_width: lw,
        luma_height: lh,
    }
}

fn translated(frame: &YuvFrame, dx: usize, dy: usize) -> YuvFrame {
    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = lw / 2;
    let ch = lh / 2;
    let mut out = frame.clone();
    for r in 0..lh {
        for c in 0..lw {
            out.y[r * lw + c] = frame.y[((r + lh - dy) % lh) * lw + (c + lw - dx) % lw];
        }
    }
    for r in 0..ch {
        for c in 0..cw {
            let s = ((r + ch - dy / 2) % ch) * cw + (c + cw - dx / 2) % cw;
            out.cb[r * cw + c] = frame.cb[s];
            out.cr[r * cw + c] = frame.cr[s];
        }
    }
    out
}

/// Byte offset of the first GOB header (GN = 1) in a QCIF picture: the
/// byte-aligned GBSC + GN reads `00 00 84..87` (`1 00001 gg`), which no
/// PSC (`00 00 80..83`) can emulate.
fn first_gob_header_offset(stream: &[u8]) -> usize {
    (2..stream.len())
        .find(|&i| stream[i - 2] == 0 && stream[i - 1] == 0 && (0x84..=0x87).contains(&stream[i]))
        .expect("a GN = 1 GOB header")
}

#[test]
fn cpm_gob_stream_reconstructs_like_the_single_bitstream_form() {
    let base = textured(176, 144, 3);
    let next = translated(&base, 3, 1);
    let quant = |gn: usize| 5 + (gn % 3) as u8;
    for psbi in 0..4u8 {
        let i_cpm = encode_intra_picture_gobs_cpm(&base, 0, psbi, quant).unwrap();
        let i_plain = encode_intra_picture_gobs(&base, 0, quant).unwrap();
        // CPM adds 3 header bits + 2 per GOB header (8 of them at QCIF).
        assert!(i_cpm.len() > i_plain.len());
        let r_cpm = decode_picture_no_gob0_header(&i_cpm, None, DecodeOptions::default()).unwrap();
        let r_plain =
            decode_picture_no_gob0_header(&i_plain, None, DecodeOptions::default()).unwrap();
        assert_eq!(r_cpm, r_plain, "PSBI {psbi}: CPM framing is pixel-neutral");

        let p_cpm = encode_inter_picture_gobs_cpm(&next, &r_cpm, 6, 1, 5, psbi).unwrap();
        let p_plain = encode_inter_picture_gobs(&next, &r_plain, 6, 1, 5).unwrap();
        let mut stream = i_cpm.clone();
        stream.extend_from_slice(&p_cpm);
        stream.extend_from_slice(&EOS_BYTES);
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        let expected =
            decode_picture_no_gob0_header(&p_plain, Some(&r_plain), DecodeOptions::default())
                .unwrap();
        assert_eq!(decoded.len(), 2);
        assert_eq!(
            decoded[1], expected,
            "PSBI {psbi}: P-picture through decode_sequence"
        );
    }
}

/// A GOB whose GSBI names another sub-bitstream cannot belong to this
/// picture: the single-Sub-Bitstream decode refuses it.
#[test]
fn cpm_gob_with_foreign_gsbi_is_refused() {
    let base = textured(176, 144, 8);
    let mut stream = encode_intra_picture_gobs_cpm(&base, 0, 2, |_| 7).unwrap();
    assert!(decode_picture_no_gob0_header(&stream, None, DecodeOptions::default()).is_ok());
    let at = first_gob_header_offset(&stream);
    // `1 00001 gg` — flip the GSBI bits from 2 (`10`) to 1 (`01`).
    assert_eq!(stream[at], 0x84 | 0b10);
    stream[at] = 0x84 | 0b01;
    assert_eq!(
        decode_picture_no_gob0_header(&stream, None, DecodeOptions::default()).unwrap_err(),
        Error::NotImplemented
    );
}

#[test]
fn cpm_gob_encoders_reject_out_of_range_psbi() {
    let base = textured(128, 96, 1);
    assert_eq!(
        encode_intra_picture_gobs_cpm(&base, 0, 4, |_| 5).unwrap_err(),
        Error::BadSliceSsbiCode
    );
    assert_eq!(
        encode_inter_picture_gobs_cpm(&base, &base, 5, 1, 4, 4).unwrap_err(),
        Error::BadSliceSsbiCode
    );
}
