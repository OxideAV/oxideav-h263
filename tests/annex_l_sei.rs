//! Integration tests for **Annex L — Supplemental Enhancement Information**.
#![allow(clippy::unusual_byte_groupings)]
//!
//! These exercise the parser side end-to-end: synthesise a baseline
//! sub-QCIF I-picture header with a PEI loop carrying PSUPP bytes, parse
//! it, and assert the surfaced [`Sei`] records match what we put in.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_h263::picture::{parse_picture_header, PictureCodingType, SourceFormat};
use oxideav_h263::sei::Sei;

/// Helper: build a minimal sub-QCIF I-picture header bit stream up to
/// (and not including) the PEI loop. Returns the writer so the caller
/// can append PEI/PSPARE bits + finalise.
fn write_subqcif_iframe_prefix(bw: &mut BitWriter) {
    // PSC (22): 0000 0000 0000 0000 1 00000
    bw.write_bits(0b00_0000_0000_0000_0000_1_00000, 22);
    // TR (8) = 0
    bw.write_bits(0, 8);
    // PTYPE: marker(1)=1, id(1)=0, split=0, cam=0, freeze=0
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    // src_format = 001 sub-QCIF
    bw.write_bits(0b001, 3);
    // coding type = I
    bw.write_bits(0, 1);
    // UMV/SAC/AP/PB all off
    bw.write_bits(0, 4);
    // PQUANT = 5
    bw.write_bits(5, 5);
    // CPM = 0
    bw.write_bits(0, 1);
}

#[test]
fn no_psupp_yields_empty_sei_list() {
    let mut bw = BitWriter::new();
    write_subqcif_iframe_prefix(&mut bw);
    bw.write_bits(0, 1); // PEI = 0
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert_eq!(hdr.source_format, SourceFormat::SubQcif);
    assert_eq!(hdr.coding_type, PictureCodingType::Intra);
    assert!(hdr.sei.is_empty());
}

#[test]
fn full_picture_freeze_request_propagates_through_pei_loop() {
    // PSUPP = [0x20] (FTYPE=2, DSIZE=0)
    let mut bw = BitWriter::new();
    write_subqcif_iframe_prefix(&mut bw);
    // PEI = 1, PSPARE = 0x20, then PEI = 0 to terminate.
    bw.write_bits(1, 1);
    bw.write_bits(0x20, 8);
    bw.write_bits(0, 1);
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert_eq!(hdr.sei, vec![Sei::FullPictureFreezeRequest]);
}

#[test]
fn snapshot_tag_with_id_propagates_through_pei_loop() {
    // PSUPP = [0x64, 0x00, 0x00, 0x00, 0x42] — Full-Picture-Snapshot
    // FTYPE=6 DSIZE=4 id=0x42
    let mut bw = BitWriter::new();
    write_subqcif_iframe_prefix(&mut bw);
    for &byte in &[0x64u8, 0x00, 0x00, 0x00, 0x42] {
        bw.write_bits(1, 1);
        bw.write_bits(byte as u32, 8);
    }
    bw.write_bits(0, 1); // PEI = 0 — terminate
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert_eq!(hdr.sei, vec![Sei::FullPictureSnapshotTag { id: 0x42 }]);
}

#[test]
fn unknown_ftype_is_carried_as_blob_record() {
    // PSUPP = [0xD0] — FTYPE=13 (reserved), DSIZE=0
    let mut bw = BitWriter::new();
    write_subqcif_iframe_prefix(&mut bw);
    bw.write_bits(1, 1);
    bw.write_bits(0xD0, 8);
    bw.write_bits(0, 1);
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert_eq!(
        hdr.sei,
        vec![Sei::Unknown {
            ftype: 13,
            payload: Vec::new()
        }]
    );
}

#[test]
fn multiple_psupp_records_in_one_picture_round_trip() {
    // Do-Nothing then Full-Picture-Freeze.
    let mut bw = BitWriter::new();
    write_subqcif_iframe_prefix(&mut bw);
    for &byte in &[0x10u8, 0x20] {
        bw.write_bits(1, 1);
        bw.write_bits(byte as u32, 8);
    }
    bw.write_bits(0, 1);
    let bytes = bw.finish();
    let mut br = BitReader::new(&bytes);
    let hdr = parse_picture_header(&mut br).unwrap();
    assert_eq!(hdr.sei, vec![Sei::DoNothing, Sei::FullPictureFreezeRequest]);
}
