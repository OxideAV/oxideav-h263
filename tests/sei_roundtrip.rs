//! Annex L / Annex W supplemental-enhancement end-to-end tests: SEI
//! octets inserted into real encoded pictures must be recoverable and
//! must not perturb reconstruction (decoders "may simply discard any
//! PSUPP information bits", §L.1).

use oxideav_h263::encoder::{
    encode_inter_picture_motion, encode_intra_picture, encode_intra_picture_sac,
};
use oxideav_h263::picture::{
    decode_picture_no_gob0_header, decode_picture_sac, decode_sequence, extract_psupp,
    insert_psupp, DecodeOptions, YuvFrame,
};
use oxideav_h263::{parse_psupp, write_psupp, PictureMessage, PictureRect, SeiFunction};

fn gradient(seed: usize) -> YuvFrame {
    let mut f = YuvFrame::grey(176, 144);
    for y in 0..144usize {
        for x in 0..176usize {
            f.y[y * 176 + x] = ((x * 2 + y * 3 + seed * 7) & 0xFF) as u8;
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

fn sample_sei() -> Vec<SeiFunction> {
    vec![
        SeiFunction::FullPictureSnapshot(0x0102_0304),
        SeiFunction::PictureMessage(PictureMessage::for_picture_number(513)),
        SeiFunction::PartialPictureFreeze(PictureRect {
            x: 0,
            y: 0,
            width: 3,
            height: 2,
        }),
    ]
}

#[test]
fn intra_picture_sei_is_recoverable_and_pixel_neutral() {
    let frame = gradient(0);
    let plain = encode_intra_picture(&frame, 8, 0).expect("encode");
    assert_eq!(
        extract_psupp(&plain).expect("no-SEI extract"),
        Vec::<u8>::new()
    );

    let octets = write_psupp(&sample_sei()).expect("write_psupp");
    let with_sei = insert_psupp(&plain, &octets).expect("insert");
    assert_eq!(extract_psupp(&with_sei).expect("extract"), octets);
    assert_eq!(parse_psupp(&octets).expect("parse")[..3], sample_sei()[..]);

    let base = decode_picture_no_gob0_header(&plain, None, DecodeOptions::default())
        .expect("decode plain");
    let carried = decode_picture_no_gob0_header(&with_sei, None, DecodeOptions::default())
        .expect("decode with SEI");
    assert_eq!(
        frame_bytes(&base),
        frame_bytes(&carried),
        "SEI must be pixel-neutral"
    );
}

#[test]
fn inter_picture_sei_round_trips_through_decode_sequence() {
    let f0 = gradient(0);
    let f1 = gradient(1);
    let i_pic = encode_intra_picture(&f0, 8, 0).expect("I");
    let recon0 =
        decode_picture_no_gob0_header(&i_pic, None, DecodeOptions::default()).expect("recon0");
    let p_pic = encode_inter_picture_motion(&f1, &recon0, 8, 1, 8).expect("P");

    let octets = write_psupp(&[SeiFunction::VideoTimeSegmentStart(42)]).expect("octets");
    let i_sei = insert_psupp(&i_pic, &octets).expect("insert I");
    let p_sei = insert_psupp(&p_pic, &octets).expect("insert P");

    let mut plain_stream = i_pic.clone();
    plain_stream.extend_from_slice(&p_pic);
    let mut sei_stream = i_sei.clone();
    sei_stream.extend_from_slice(&p_sei);

    let plain = decode_sequence(&plain_stream, DecodeOptions::default()).expect("plain");
    let carried = decode_sequence(&sei_stream, DecodeOptions::default()).expect("SEI");
    assert_eq!(plain.len(), carried.len());
    for (a, b) in plain.iter().zip(carried.iter()) {
        assert_eq!(frame_bytes(a), frame_bytes(b));
    }
    // And each picture's SEI is recoverable from the stream slices.
    let second =
        oxideav_h263::picture::next_picture_start_code(&sei_stream, 1).expect("second PSC");
    assert_eq!(extract_psupp(&sei_stream[..second]).expect("I SEI"), octets);
    assert_eq!(extract_psupp(&sei_stream[second..]).expect("P SEI"), octets);
}

#[test]
fn sac_picture_tolerates_inserted_sei() {
    // SAC pictures are bit-sequential after the header, so the PEI
    // splice shifts them harmlessly.
    let frame = gradient(2);
    let plain = encode_intra_picture_sac(&frame, 10, 0).expect("SAC encode");
    let octets = write_psupp(&[SeiFunction::FixedPointIdct(0)]).expect("octets");
    let with_sei = insert_psupp(&plain, &octets).expect("insert");
    let base = decode_picture_sac(&plain, None, DecodeOptions::default()).expect("decode");
    let carried = decode_picture_sac(&with_sei, None, DecodeOptions::default()).expect("decode");
    assert_eq!(frame_bytes(&base), frame_bytes(&carried));
    assert_eq!(extract_psupp(&with_sei).expect("extract"), octets);
}

#[test]
fn insert_appends_after_existing_octets() {
    let frame = gradient(3);
    let plain = encode_intra_picture(&frame, 8, 0).expect("encode");
    let first = write_psupp(&[SeiFunction::VideoTimeSegmentStart(1)]).expect("octets 1");
    let second = write_psupp(&[SeiFunction::VideoTimeSegmentEnd(1)]).expect("octets 2");
    let once = insert_psupp(&plain, &first).expect("insert 1");
    let twice = insert_psupp(&once, &second).expect("insert 2");
    let mut combined = first.clone();
    combined.extend_from_slice(&second);
    assert_eq!(extract_psupp(&twice).expect("extract"), combined);
    let base = decode_picture_no_gob0_header(&plain, None, DecodeOptions::default()).unwrap();
    let carried = decode_picture_no_gob0_header(&twice, None, DecodeOptions::default()).unwrap();
    assert_eq!(frame_bytes(&base), frame_bytes(&carried));
}
