#![no_main]
//! Whole-elementary-stream decode: every byte string must either
//! decode or return an error — never panic, never run away.

use libfuzzer_sys::fuzz_target;
use oxideav_h263::picture::{decode_sequence, DecodeOptions};

fuzz_target!(|data: &[u8]| {
    // Bound the work per input: H.263 pictures are at most 16CIF, but
    // a hostile stream could chain many; the decoder itself is the
    // subject, so cap the input length rather than the picture count.
    if data.len() > 64 * 1024 {
        return;
    }
    let _ = decode_sequence(data, DecodeOptions::default());
    let _ = decode_sequence(
        data,
        DecodeOptions {
            deblock: true,
            obmc_skip_zero_right: true,
            ..DecodeOptions::default()
        },
    );
});
