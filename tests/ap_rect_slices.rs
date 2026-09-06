//! Annex K Rectangular Slice / Arbitrary Slice Ordering + Annex F
//! Advanced Prediction on the encode side (§K.1 rules 1 / 3 per stripe).

use oxideav_h263::encoder::{
    encode_inter_picture_ap, encode_inter_picture_ap_slices_rect, encode_intra_picture_slices_rect,
    EOS_BYTES,
};
use oxideav_h263::picture::{decode_picture_layer, decode_sequence, DecodeOptions, YuvFrame};
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

/// Left half moves by `(dx_l, dy)`, right half by `(dx_r, dy)` —
/// non-uniform motion so the four-vector field is not trivially flat.
fn split_motion(frame: &YuvFrame, dx_l: usize, dx_r: usize, dy: usize) -> YuvFrame {
    let lw = frame.luma_width;
    let lh = frame.luma_height;
    let cw = lw / 2;
    let ch = lh / 2;
    let mut out = frame.clone();
    for r in 0..lh {
        for c in 0..lw {
            let dx = if c < lw / 2 { dx_l } else { dx_r };
            out.y[r * lw + c] = frame.y[((r + lh - dy) % lh) * lw + (c + lw - dx) % lw];
        }
    }
    for r in 0..ch {
        for c in 0..cw {
            let dx = if c < cw / 2 { dx_l / 2 } else { dx_r / 2 };
            let s = ((r + ch - dy / 2) % ch) * cw + (c + cw - dx) % cw;
            out.cb[r * cw + c] = frame.cb[s];
            out.cr[r * cw + c] = frame.cr[s];
        }
    }
    out
}

fn luma_mae(a: &YuvFrame, b: &YuvFrame) -> f64 {
    a.y.iter()
        .zip(b.y.iter())
        .map(|(&x, &y)| x.abs_diff(y) as f64)
        .sum::<f64>()
        / a.y.len() as f64
}

fn luma_psnr(a: &YuvFrame, b: &YuvFrame) -> f64 {
    let mse =
        a.y.iter()
            .zip(b.y.iter())
            .map(|(&x, &y)| (x as f64 - y as f64).powi(2))
            .sum::<f64>()
            / a.y.len() as f64;
    10.0 * (255.0f64 * 255.0 / mse.max(1e-9)).log10()
}

#[test]
fn ap_rect_stripes_round_trip_in_both_slice_orders() {
    let base = textured(176, 144, 4);
    let i_bytes = encode_intra_picture_slices_rect(&base, 6, 0, 4, false).unwrap();
    let r0 = decode_sequence(&i_bytes, DecodeOptions::default())
        .unwrap()
        .remove(0);
    let next = split_motion(&base, 4, 2, 1);
    let free = encode_inter_picture_ap(&next, &r0, 6, 1, 5).unwrap();
    let free_recon = {
        let mut stream = i_bytes.clone();
        stream.extend_from_slice(&free);
        decode_sequence(&stream, DecodeOptions::default())
            .unwrap()
            .remove(1)
    };
    for (width, aso) in [(4usize, false), (4, true), (3, false), (11, true)] {
        let p = encode_inter_picture_ap_slices_rect(&next, &r0, 6, 1, 5, width, aso).unwrap();
        let recon = decode_picture_layer(&p, Some(&r0), DecodeOptions::default()).unwrap();
        let (mae, psnr) = (luma_mae(&recon, &next), luma_psnr(&recon, &next));
        eprintln!(
            "stripes {width} wide, ASO {aso}: {} bytes (free-running AP {} bytes), luma MAE {mae:.3}, PSNR {psnr:.2} dB",
            p.len(),
            free.len()
        );
        assert!(
            mae < 3.0 && psnr > 35.0,
            "stripes {width} ASO {aso}: MAE {mae:.3} PSNR {psnr:.2}"
        );
        // A single full-width stripe is one segment: it must
        // reconstruct exactly like the free-running AP picture (same
        // vectors, same remotes) — only the slice framing differs.
        if width == 11 {
            assert_eq!(
                recon, free_recon,
                "one stripe == the free-running AP picture"
            );
        }
        let mut stream = i_bytes.clone();
        stream.extend_from_slice(&p);
        stream.extend_from_slice(&EOS_BYTES);
        let decoded = decode_sequence(&stream, DecodeOptions::default()).unwrap();
        assert_eq!(decoded.len(), 2);
        assert_eq!(decoded[1], recon);
    }
    assert_eq!(
        encode_inter_picture_ap_slices_rect(&next, &r0, 6, 1, 5, 12, false).unwrap_err(),
        Error::UnsupportedPictureGeometry
    );
}
