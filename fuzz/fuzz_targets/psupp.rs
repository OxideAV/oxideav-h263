#![no_main]
//! Annex L / W PSUPP parse → write → parse must be total and
//! idempotent on accepted inputs.

use libfuzzer_sys::fuzz_target;
use oxideav_h263::{parse_psupp, write_psupp};

fuzz_target!(|data: &[u8]| {
    if let Ok(functions) = parse_psupp(data) {
        let rewritten = write_psupp(&functions).expect("accepted functions must serialise");
        let again = parse_psupp(&rewritten).expect("serialised functions must parse");
        // write_psupp may append a §L.3 Do Nothing; the parsed prefix
        // must be the original inventory.
        assert!(again.len() >= functions.len());
        assert_eq!(&again[..functions.len()], &functions[..]);
    }
});
