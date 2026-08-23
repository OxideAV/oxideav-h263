#![no_main]
//! The streaming registry decoder under arbitrary packetisation: the
//! first byte picks a chunk size, the rest is shredded into packets
//! and fed through send_packet / receive_frame / flush / reset.

use libfuzzer_sys::fuzz_target;
use oxideav_core::{CodecId, CodecParameters, Error, Packet, TimeBase};

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 || data.len() > 64 * 1024 {
        return;
    }
    let chunk = (data[0] as usize % 64) + 1;
    let params = CodecParameters::video(CodecId::new("h263"));
    let mut dec = match oxideav_h263::make_decoder(&params) {
        Ok(d) => d,
        Err(_) => return,
    };
    for (i, piece) in data[1..].chunks(chunk).enumerate() {
        let packet = Packet::new(0, TimeBase::MICROS, piece.to_vec()).with_pts(i as i64);
        if dec.send_packet(&packet).is_err() {
            // A decode error ends this stream; a reset must make the
            // decoder usable again.
            let _ = dec.reset();
            continue;
        }
        loop {
            match dec.receive_frame() {
                Ok(_) => {}
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(_) => break,
            }
        }
    }
    let _ = dec.flush();
    loop {
        match dec.receive_frame() {
            Ok(_) => {}
            Err(_) => break,
        }
    }
});
