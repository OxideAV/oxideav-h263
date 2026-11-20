//! Annex N Reference Picture Selection, forward channel: the RPS
//! picture encoders driven back through `decode_picture_layer_rps`.

use oxideav_h263::annex_n::RpsReferenceStore;
use oxideav_h263::encoder::{
    RpsFields, encode_inter_picture_plus, encode_inter_picture_rps, encode_intra_picture_rps,
};
use oxideav_h263::picture::{
    DecodeOptions, YuvFrame, decode_picture_layer, decode_picture_layer_rps,
};
use oxideav_h263::{Error, ModeRule};

fn scene(lw: usize, lh: usize, t: usize) -> YuvFrame {
    let (cw, ch) = (lw / 2, lh / 2);
    let mut y = vec![0u8; lw * lh];
    for r in 0..lh {
        for c in 0..lw {
            let base = 40 + ((r + 2 * t) / 3 + (c + 3 * t) / 2) % 150;
            let sq = (30 + 6 * t..62 + 6 * t).contains(&c) && (40..72).contains(&r);
            y[r * lw + c] = if sq { 220 } else { base as u8 };
        }
    }
    YuvFrame {
        y,
        cb: (0..cw * ch).map(|i| (100 + (i % cw) / 4) as u8).collect(),
        cr: (0..cw * ch).map(|i| (140 - (i / cw) / 4) as u8).collect(),
        luma_width: lw,
        luma_height: lh,
    }
}

/// I(0) → P(1) → P(2 predicted from TR 0, skipping picture 1) → P(3
/// from the most recent anchor): the decoder selects each reference
/// from its store by TRP and reproduces exactly what the encoder
/// predicted from.
#[test]
fn rps_trp_selects_the_encoders_reference() {
    let opts = DecodeOptions::default();
    for umv in [false, true] {
        let mut store = RpsReferenceStore::new();
        let rps = RpsFields::new().with_need_nack(true);

        let i = encode_intra_picture_rps(&scene(176, 144, 0), 6, 0, rps).unwrap();
        let r0 = decode_picture_layer_rps(&i, &mut store, opts).unwrap();

        let p1 = encode_inter_picture_rps(&scene(176, 144, 1), &r0, 6, 1, 6, umv, rps).unwrap();
        let r1 = decode_picture_layer_rps(&p1, &mut store, opts).unwrap();
        assert_ne!(r1, r0);

        // Picture 2 predicts from TR 0, not from picture 1.
        let f2 = scene(176, 144, 2);
        let p2 = encode_inter_picture_rps(&f2, &r0, 6, 2, 8, umv, rps.with_trp(Some(0))).unwrap();
        let r2 = decode_picture_layer_rps(&p2, &mut store, opts).unwrap();
        if !umv {
            // Same macroblock layer as the plain PLUSPTYPE coding
            // against the same reference: identical reconstruction.
            let plain = encode_inter_picture_plus(&f2, &r0, 6, 2, 8).unwrap();
            assert_eq!(p2.len(), plain.len() + 2, "16 RPS header bits");
            assert_eq!(r2, decode_picture_layer(&plain, Some(&r0), opts).unwrap());
        }
        // A static picture against TR 0 skips everywhere and comes
        // back as picture 0 — only possible if TRP was honoured.
        let p_static =
            encode_inter_picture_rps(&r0, &r0, 6, 3, 2, umv, rps.with_trp(Some(0))).unwrap();
        let mut probe = store.clone();
        assert_eq!(
            decode_picture_layer_rps(&p_static, &mut probe, opts).unwrap(),
            r0
        );

        // TRPI = 0: the most recent anchor (picture 2).
        let p3 = encode_inter_picture_rps(&r2, &r2, 6, 3, 2, umv, rps).unwrap();
        assert_eq!(decode_picture_layer_rps(&p3, &mut store, opts).unwrap(), r2);

        // A TRP the store does not hold is the §N.5 forced-update case.
        let lost =
            encode_inter_picture_rps(&r2, &r2, 6, 4, 2, umv, rps.with_trp(Some(200))).unwrap();
        assert!(decode_picture_layer_rps(&lost, &mut store, opts).is_err());
    }
}

/// §5.1.14 — "TRPI shall be 0 whenever the picture header indicates an
/// I- or EI-picture".
#[test]
fn rps_trp_in_an_intra_picture_is_refused_with_the_rule() {
    let f = scene(176, 144, 0);
    let err = encode_intra_picture_rps(&f, 6, 0, RpsFields::new().with_trp(Some(3))).unwrap_err();
    assert_eq!(err, Error::ModeCombination(ModeRule::TrpInIntraPicture));
    assert_eq!(ModeRule::TrpInIntraPicture.clause(), "5.1.14");
}
