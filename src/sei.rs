//! H.263 **Annex L** — Supplemental Enhancement Information (SEI / PSUPP).
//!
//! The PSUPP byte stream is carried inside the picture-header PEI loop
//! (§5.1.24 / §5.1.25): if `PEI == 1` then 8 bits of PSUPP data follow,
//! then another `PEI` bit, etc., until a `PEI == 0` terminates the loop.
//!
//! Each PSUPP block is structured per §L.2 as a 4-bit `FTYPE` (function
//! type) plus a 4-bit `DSIZE` (parameter size in bytes), optionally
//! followed by `DSIZE` bytes of parameter data. Multiple FTYPE/DSIZE
//! records can be concatenated within a single PSUPP byte stream.
//!
//! The defined FTYPE values are listed in Table L.1 / §L.3..§L.14:
//!
//! | FTYPE | Function                                               | DSIZE       |
//! |------:|--------------------------------------------------------|-------------|
//! | 0     | Reserved                                               | n/a         |
//! | 1     | Do Nothing (start-code-emulation prevention; §L.3)     | 0           |
//! | 2     | Full-Picture Freeze Request (§L.4)                     | 0           |
//! | 3     | Partial-Picture Freeze Request (§L.5)                  | 4           |
//! | 4     | Resizing Partial-Picture Freeze Request (§L.6)         | 8           |
//! | 5     | Partial-Picture Freeze-Release Request (§L.7)          | 4           |
//! | 6     | Full-Picture Snapshot Tag (§L.8)                       | 4           |
//! | 7     | Partial-Picture Snapshot Tag (§L.9)                    | 8           |
//! | 8     | Video Time Segment Start Tag (§L.10)                   | 4           |
//! | 9     | Video Time Segment End Tag (§L.11)                     | 4           |
//! | 10    | Progressive Refinement Segment Start Tag (§L.12)       | 4           |
//! | 11    | Progressive Refinement Segment End Tag (§L.13)         | 4           |
//! | 12    | Chroma Keying Information (§L.14)                      | 1..=9       |
//! | 13    | Reserved                                               | n/a         |
//! | 14    | Reserved                                               | n/a         |
//! | 15    | Extended Function Type (§L.15)                         | 0 (+ 1 ext) |
//!
//! **Decoder policy.** §5.1.25 mandates that decoders which do not
//! understand the extended capabilities "shall be designed to discard
//! PSUPP if PEI is set to 1". Round 25 implements parser-side recognition:
//! we parse every PSUPP record, validate its DSIZE against the spec
//! envelope, and surface the parsed records on the
//! [`PictureHeader::sei`](crate::picture::PictureHeader::sei) field for
//! callers that want to act on them. Records with reserved FTYPE values
//! are kept as `Sei::Unknown` blobs (per §L.2's forward-compatibility
//! note: a decoder that doesn't understand a FTYPE skips DSIZE bytes and
//! moves on).
//!
//! Action semantics (e.g. actually freezing the displayed picture for
//! [`Sei::FullPictureFreezeRequest`]) are out of scope of this codec
//! crate — they are downstream presentation concerns. We only perform
//! the syntax recognition + diagnostic surfacing here.

use oxideav_core::Result;

/// One Supplemental Enhancement Information record extracted from a
/// picture's PSUPP byte stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Sei {
    /// `FTYPE = 1` — §L.3 Do Nothing (start-code emulation prevention).
    DoNothing,
    /// `FTYPE = 2` — §L.4 Full-Picture Freeze Request.
    FullPictureFreezeRequest,
    /// `FTYPE = 3` — §L.5 Partial-Picture Freeze Request.
    /// Coordinates are in units of 8 pixels.
    PartialPictureFreezeRequest { x: u8, y: u8, width: u8, height: u8 },
    /// `FTYPE = 4` — §L.6 Resizing Partial-Picture Freeze Request.
    /// First quad locates the displayed-picture rectangle, second quad
    /// locates the decoded-picture rectangle. All in units of 8 pixels.
    ResizingPartialPictureFreezeRequest {
        displayed: (u8, u8, u8, u8),
        decoded: (u8, u8, u8, u8),
    },
    /// `FTYPE = 5` — §L.7 Partial-Picture Freeze-Release Request.
    PartialPictureFreezeReleaseRequest { x: u8, y: u8, width: u8, height: u8 },
    /// `FTYPE = 6` — §L.8 Full-Picture Snapshot Tag.
    FullPictureSnapshotTag { id: u32 },
    /// `FTYPE = 7` — §L.9 Partial-Picture Snapshot Tag.
    PartialPictureSnapshotTag {
        id: u32,
        x: u8,
        y: u8,
        width: u8,
        height: u8,
    },
    /// `FTYPE = 8` — §L.10 Video Time Segment Start Tag.
    VideoTimeSegmentStartTag { id: u32 },
    /// `FTYPE = 9` — §L.11 Video Time Segment End Tag.
    VideoTimeSegmentEndTag { id: u32 },
    /// `FTYPE = 10` — §L.12 Progressive Refinement Segment Start Tag.
    ProgressiveRefinementSegmentStartTag { id: u32 },
    /// `FTYPE = 11` — §L.13 Progressive Refinement Segment End Tag.
    ProgressiveRefinementSegmentEndTag { id: u32 },
    /// `FTYPE = 12` — §L.14 Chroma Keying Information.
    ChromaKeyingInformation { payload: Vec<u8> },
    /// `FTYPE = 15` — §L.15 Extended Function Type. The extra octet
    /// following the indicator carries the extended FTYPE in its high
    /// nibble and a DSIZE in its low nibble; we surface the raw parameter
    /// bytes for caller inspection.
    ExtendedFunctionType {
        ext_ftype: u8,
        ext_dsize: u8,
        payload: Vec<u8>,
    },
    /// Any unrecognised / reserved FTYPE — round-25 forward-compatibility
    /// fallback. The decoder skips the DSIZE bytes per §L.2 and surfaces
    /// the raw record for the caller.
    Unknown { ftype: u8, payload: Vec<u8> },
}

/// Parse the concatenated PSUPP byte stream (already de-interleaved from
/// the picture-header PEI bits) into a list of [`Sei`] records.
///
/// Returns the parsed records in stream order. A trailing partial record
/// (DSIZE bytes missing) is treated as a hard error — this signals a
/// malformed bitstream rather than the spec's "discard unknown" policy
/// (which only applies to *complete* records with unknown FTYPE).
pub fn parse_psupp_stream(bytes: &[u8]) -> Result<Vec<Sei>> {
    let mut out = Vec::new();
    let mut p = 0usize;
    while p < bytes.len() {
        let hdr = bytes[p];
        p += 1;
        let ftype = (hdr >> 4) & 0x0F;
        let dsize = (hdr & 0x0F) as usize;
        if p + dsize > bytes.len() {
            return Err(oxideav_core::Error::invalid(format!(
                "h263 Annex L PSUPP: FTYPE={ftype} DSIZE={dsize} extends past stream end \
                 ({} bytes remaining)",
                bytes.len() - p
            )));
        }
        let payload = &bytes[p..p + dsize];
        p += dsize;
        let rec = match ftype {
            1 => {
                expect_dsize(ftype, dsize, &[0])?;
                Sei::DoNothing
            }
            2 => {
                expect_dsize(ftype, dsize, &[0])?;
                Sei::FullPictureFreezeRequest
            }
            3 => {
                expect_dsize(ftype, dsize, &[4])?;
                Sei::PartialPictureFreezeRequest {
                    x: payload[0],
                    y: payload[1],
                    width: payload[2],
                    height: payload[3],
                }
            }
            4 => {
                expect_dsize(ftype, dsize, &[8])?;
                Sei::ResizingPartialPictureFreezeRequest {
                    displayed: (payload[0], payload[1], payload[2], payload[3]),
                    decoded: (payload[4], payload[5], payload[6], payload[7]),
                }
            }
            5 => {
                expect_dsize(ftype, dsize, &[4])?;
                Sei::PartialPictureFreezeReleaseRequest {
                    x: payload[0],
                    y: payload[1],
                    width: payload[2],
                    height: payload[3],
                }
            }
            6 => {
                expect_dsize(ftype, dsize, &[4])?;
                Sei::FullPictureSnapshotTag {
                    id: u32_be(payload),
                }
            }
            7 => {
                expect_dsize(ftype, dsize, &[8])?;
                Sei::PartialPictureSnapshotTag {
                    id: u32_be(&payload[0..4]),
                    x: payload[4],
                    y: payload[5],
                    width: payload[6],
                    height: payload[7],
                }
            }
            8 => {
                expect_dsize(ftype, dsize, &[4])?;
                Sei::VideoTimeSegmentStartTag {
                    id: u32_be(payload),
                }
            }
            9 => {
                expect_dsize(ftype, dsize, &[4])?;
                Sei::VideoTimeSegmentEndTag {
                    id: u32_be(payload),
                }
            }
            10 => {
                expect_dsize(ftype, dsize, &[4])?;
                Sei::ProgressiveRefinementSegmentStartTag {
                    id: u32_be(payload),
                }
            }
            11 => {
                expect_dsize(ftype, dsize, &[4])?;
                Sei::ProgressiveRefinementSegmentEndTag {
                    id: u32_be(payload),
                }
            }
            12 => {
                if !(1..=9).contains(&dsize) {
                    return Err(oxideav_core::Error::invalid(format!(
                        "h263 Annex L PSUPP: CKIF DSIZE must be 1..=9 (got {dsize})"
                    )));
                }
                Sei::ChromaKeyingInformation {
                    payload: payload.to_vec(),
                }
            }
            15 => {
                expect_dsize(ftype, dsize, &[0])?;
                // §L.15: an additional octet follows whose high nibble is
                // the extended FTYPE and low nibble is an extended DSIZE
                // counting subsequent bytes to skip.
                if p >= bytes.len() {
                    return Err(oxideav_core::Error::invalid(
                        "h263 Annex L PSUPP: extended-FTYPE indicator at end of stream",
                    ));
                }
                let ext = bytes[p];
                p += 1;
                let ext_ftype = (ext >> 4) & 0x0F;
                let ext_dsize = (ext & 0x0F) as usize;
                if p + ext_dsize > bytes.len() {
                    return Err(oxideav_core::Error::invalid(format!(
                        "h263 Annex L PSUPP: extended FTYPE={ext_ftype} DSIZE={ext_dsize} \
                         extends past stream end"
                    )));
                }
                let ext_payload = bytes[p..p + ext_dsize].to_vec();
                p += ext_dsize;
                Sei::ExtendedFunctionType {
                    ext_ftype,
                    ext_dsize: ext_dsize as u8,
                    payload: ext_payload,
                }
            }
            // §L.2: reserved / unknown FTYPE. Decoder discards DSIZE bytes
            // and continues. We surface the raw record for diagnostics.
            _ => Sei::Unknown {
                ftype,
                payload: payload.to_vec(),
            },
        };
        out.push(rec);
    }
    Ok(out)
}

fn expect_dsize(ftype: u8, got: usize, allowed: &[usize]) -> Result<()> {
    if !allowed.contains(&got) {
        return Err(oxideav_core::Error::invalid(format!(
            "h263 Annex L PSUPP FTYPE={ftype}: DSIZE={got} (expected one of {allowed:?})"
        )));
    }
    Ok(())
}

fn u32_be(bytes: &[u8]) -> u32 {
    debug_assert!(bytes.len() >= 4);
    ((bytes[0] as u32) << 24)
        | ((bytes[1] as u32) << 16)
        | ((bytes[2] as u32) << 8)
        | (bytes[3] as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_psupp_stream_yields_no_records() {
        let recs = parse_psupp_stream(&[]).unwrap();
        assert!(recs.is_empty());
    }

    #[test]
    fn do_nothing_round_trip() {
        // FTYPE=1, DSIZE=0 → 0x10
        let recs = parse_psupp_stream(&[0x10]).unwrap();
        assert_eq!(recs, vec![Sei::DoNothing]);
    }

    #[test]
    fn full_picture_freeze_request_round_trip() {
        // FTYPE=2, DSIZE=0 → 0x20
        let recs = parse_psupp_stream(&[0x20]).unwrap();
        assert_eq!(recs, vec![Sei::FullPictureFreezeRequest]);
    }

    #[test]
    fn partial_picture_freeze_request_with_rect() {
        // FTYPE=3, DSIZE=4, then x,y,w,h = 0,0,3,2 (24-pixel × 16-pixel
        // upper-left corner — the §L.5 example).
        let recs = parse_psupp_stream(&[0x34, 0, 0, 3, 2]).unwrap();
        assert_eq!(
            recs,
            vec![Sei::PartialPictureFreezeRequest {
                x: 0,
                y: 0,
                width: 3,
                height: 2,
            }]
        );
    }

    #[test]
    fn full_picture_snapshot_tag_carries_id() {
        // FTYPE=6, DSIZE=4, id = 0x12_34_56_78
        let recs = parse_psupp_stream(&[0x64, 0x12, 0x34, 0x56, 0x78]).unwrap();
        assert_eq!(recs, vec![Sei::FullPictureSnapshotTag { id: 0x1234_5678 }]);
    }

    #[test]
    fn multiple_records_in_one_stream() {
        // Do-Nothing, then Full-Picture-Freeze, then Snapshot-Tag.
        let recs = parse_psupp_stream(&[0x10, 0x20, 0x64, 0x00, 0x00, 0x00, 0x42]).unwrap();
        assert_eq!(
            recs,
            vec![
                Sei::DoNothing,
                Sei::FullPictureFreezeRequest,
                Sei::FullPictureSnapshotTag { id: 0x42 },
            ]
        );
    }

    #[test]
    fn unknown_ftype_is_kept_as_blob() {
        // FTYPE=13 (reserved) with DSIZE=2 → swallow 2 bytes silently.
        let recs = parse_psupp_stream(&[0xD2, 0xAA, 0xBB]).unwrap();
        assert_eq!(
            recs,
            vec![Sei::Unknown {
                ftype: 13,
                payload: vec![0xAA, 0xBB],
            }]
        );
    }

    #[test]
    fn truncated_payload_is_an_error() {
        // FTYPE=3 demands DSIZE=4 but only 2 bytes follow.
        let err = parse_psupp_stream(&[0x34, 0xAA, 0xBB]).unwrap_err();
        assert!(err.to_string().contains("Annex L PSUPP"), "{err}");
    }

    #[test]
    fn wrong_dsize_for_known_ftype_is_an_error() {
        // FTYPE=2 (Full-Picture-Freeze) requires DSIZE=0; supplying 0x21
        // (DSIZE=1) must be rejected.
        let err = parse_psupp_stream(&[0x21, 0x00]).unwrap_err();
        assert!(
            err.to_string().contains("FTYPE=2"),
            "should mention FTYPE: {err}"
        );
    }

    #[test]
    fn extended_function_type_carries_payload() {
        // FTYPE=15 (extended), DSIZE=0 → 0xF0
        // Then extended octet 0x32 → ext_ftype=3, ext_dsize=2
        // Then 2 bytes of payload.
        let recs = parse_psupp_stream(&[0xF0, 0x32, 0xCA, 0xFE]).unwrap();
        assert_eq!(
            recs,
            vec![Sei::ExtendedFunctionType {
                ext_ftype: 3,
                ext_dsize: 2,
                payload: vec![0xCA, 0xFE],
            }]
        );
    }

    #[test]
    fn ckif_with_dsize_one_is_accepted() {
        let recs = parse_psupp_stream(&[0xC1, 0x00]).unwrap();
        assert_eq!(
            recs,
            vec![Sei::ChromaKeyingInformation {
                payload: vec![0x00],
            }]
        );
    }
}
