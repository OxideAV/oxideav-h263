#![no_main]
//! Picture-layer parsers (baseline + PLUSPTYPE) and the PSUPP
//! extract / insert pair on arbitrary bytes.

use libfuzzer_sys::fuzz_target;
use oxideav_core::bits::BitReader;
use oxideav_h263::picture::{extract_psupp, insert_psupp};
use oxideav_h263::plus_ptype::InheritedExtendedState;

fuzz_target!(|data: &[u8]| {
    let _ = oxideav_h263::parse_picture_header_from_bytes(data);
    let mut r = BitReader::new(data);
    let _ = oxideav_h263::parse_picture_layer(&mut r, InheritedExtendedState::default());
    if let Ok(octets) = extract_psupp(data) {
        if let Ok(rewritten) = insert_psupp(data, &[0x10]) {
            let mut expect = octets;
            expect.push(0x10);
            assert_eq!(extract_psupp(&rewritten).expect("reinserted"), expect);
        }
    }
});
