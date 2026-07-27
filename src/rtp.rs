//! RTP payload format for H.263 / H.263+ video streams per RFC 4629
//! (the current `video/H263-1998` / `video/H263-2000` payload format;
//! RFC 2429 defines the same payload header layout).
//!
//! This module handles the **payload** level: the variable-length
//! H.263+ payload header (RFC 4629 §5) and the packetization /
//! depacketization of an H.263 elementary stream into payload-sized
//! chunks (RFC 4629 §6). The 12-byte RTP transport header (sequence
//! number, timestamp, SSRC — RFC 3550) is the transport stack's
//! concern and is not produced or consumed here; one returned payload
//! corresponds to exactly one RTP packet's payload field.
//!
//! Covered:
//!
//! * **§5.1 general payload header** — the 16-bit `RR | P | V | PLEN |
//!   PEBIT` field, with the §5.2 8-bit VRC extension (`TID | Trun |
//!   S`) when `V = 1`, and the `PLEN`-byte redundant picture header.
//! * **§6.1 picture segment packets (`P = 1`)** — packets beginning at
//!   a byte-aligned Picture / GOB / Slice start code (or EOS / EOSBS):
//!   the two leading zero bytes of the start code are stripped on
//!   packetization and re-synthesised on depacketization.
//! * **§6.2 Follow-on packets (`P = 0`)** — continuation chunks that
//!   begin at an arbitrary byte position inside a segment.
//!
//! The packetizer prefers cuts at byte-aligned start-code boundaries
//! (every H.263 start code opens with 16 zero bits, which the VLC
//! design guarantees cannot appear in coded macroblock data), falling
//! back to Follow-on packets when a single segment exceeds the payload
//! budget — exactly the §7 usage guidance.

use crate::{Error, Result};

/// Length in bytes of the fixed part of the §5.1 payload header
/// (`RR + P + V + PLEN + PEBIT`).
pub const PAYLOAD_HEADER_BYTES: usize = 2;

/// Length in bytes of the §5.2 VRC extension when present (`V = 1`).
pub const VRC_HEADER_BYTES: usize = 1;

/// Maximum value of the 6-bit PLEN field (§5.1).
pub const PLEN_MAX: u8 = 63;

/// RFC 4629 §5.2 — the 8-bit Video Redundancy Coding header extension
/// (`TID | Trun | S`), present when the payload header's `V` bit is
/// set. Carried opaquely: VRC thread scheduling is an encoder-policy
/// concern layered on Annex N Reference Picture Selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VrcHeader {
    /// Bits 1-3 — Thread ID (0..=7); thread 0 is the canonical thread.
    pub tid: u8,
    /// Bits 4-7 — per-thread packet counter, monotonically increasing
    /// modulo 16.
    pub trun: u8,
    /// Bit 8 — `true` iff the packet content belongs to a sync frame.
    pub sync: bool,
}

/// RFC 4629 §5.1 — a parsed H.263+ payload header, together with the
/// optional §5.2 VRC extension and the `PLEN`-byte redundant picture
/// header that may follow.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct H263PayloadHeader {
    /// `P` — the packet begins at a Picture / GOB / Slice / EOS / EOSBS
    /// start code whose two leading zero bytes were stripped (§6.1).
    pub p: bool,
    /// The §5.2 VRC extension (`Some` iff the `V` bit was set).
    pub vrc: Option<VrcHeader>,
    /// The redundant picture header attached when `PLEN > 0`
    /// (§6.1.2), exactly `PLEN` bytes, beginning with the `"100000"`
    /// tail of the PSC. Empty when `PLEN = 0`.
    pub extra_picture_header: Vec<u8>,
    /// `PEBIT` — number of least-significant bits to ignore in the
    /// last byte of [`Self::extra_picture_header`] (0 when `PLEN = 0`).
    pub pebit: u8,
}

/// Serialize a §5.1 payload header (plus the optional VRC extension
/// and redundant picture header) into `out`.
///
/// `RR` is emitted as zero per §5.1. Rejects an
/// `extra_picture_header` longer than [`PLEN_MAX`] bytes, a `pebit`
/// above 7, or a non-zero `pebit` with an empty header
/// ([`Error::RtpBadPayloadHeader`]).
pub fn write_payload_header(out: &mut Vec<u8>, header: &H263PayloadHeader) -> Result<()> {
    let plen = header.extra_picture_header.len();
    if plen > PLEN_MAX as usize {
        return Err(Error::RtpBadPayloadHeader);
    }
    if header.pebit > 7 || (plen == 0 && header.pebit != 0) {
        return Err(Error::RtpBadPayloadHeader);
    }
    let plen = plen as u16;
    // 16 bits: RR(5)=0 | P(1) | V(1) | PLEN(6) | PEBIT(3).
    let mut word: u16 = 0;
    if header.p {
        word |= 1 << 10;
    }
    if header.vrc.is_some() {
        word |= 1 << 9;
    }
    word |= plen << 3;
    word |= header.pebit as u16;
    out.push((word >> 8) as u8);
    out.push((word & 0xFF) as u8);
    if let Some(vrc) = header.vrc {
        if vrc.tid > 7 || vrc.trun > 15 {
            return Err(Error::RtpBadPayloadHeader);
        }
        out.push((vrc.tid << 5) | (vrc.trun << 1) | u8::from(vrc.sync));
    }
    out.extend_from_slice(&header.extra_picture_header);
    Ok(())
}

/// Parse the §5.1 payload header (plus VRC extension and redundant
/// picture header) from the front of `payload`, returning the parsed
/// header and the byte offset at which the H.263 bitstream data
/// begins.
///
/// Per §5.1 the `RR` bits "MUST be ignored by receivers", so any RR
/// value is accepted. A non-zero `PEBIT` with `PLEN = 0` violates the
/// §5.1 "shall" and is rejected, as is a payload shorter than its own
/// declared header fields.
pub fn parse_payload_header(payload: &[u8]) -> Result<(H263PayloadHeader, usize)> {
    if payload.len() < PAYLOAD_HEADER_BYTES {
        return Err(Error::RtpTruncatedPacket);
    }
    let word = u16::from_be_bytes([payload[0], payload[1]]);
    let p = word & (1 << 10) != 0;
    let v = word & (1 << 9) != 0;
    let plen = ((word >> 3) & 0x3F) as usize;
    let pebit = (word & 0b111) as u8;
    if plen == 0 && pebit != 0 {
        return Err(Error::RtpBadPayloadHeader);
    }
    let mut offset = PAYLOAD_HEADER_BYTES;
    let vrc = if v {
        let byte = *payload.get(offset).ok_or(Error::RtpTruncatedPacket)?;
        offset += VRC_HEADER_BYTES;
        Some(VrcHeader {
            tid: byte >> 5,
            trun: (byte >> 1) & 0x0F,
            sync: byte & 1 != 0,
        })
    } else {
        None
    };
    if payload.len() < offset + plen {
        return Err(Error::RtpTruncatedPacket);
    }
    let extra_picture_header = payload[offset..offset + plen].to_vec();
    offset += plen;
    Ok((
        H263PayloadHeader {
            p,
            vrc,
            extra_picture_header,
            pebit,
        },
        offset,
    ))
}

/// `true` iff `data[pos..]` opens with a byte-aligned H.263 start code
/// — 16 zero bits followed by a `1` bit. Picture (§5.1.1), GOB
/// (§5.2.2), Slice (§K.2.2), EOS (§5.1.27) and EOSBS start codes all
/// share this prefix, and the H.263 VLC design guarantees the pattern
/// cannot occur inside coded macroblock data.
fn is_start_code_at(data: &[u8], pos: usize) -> bool {
    data.len() >= pos + 3 && data[pos] == 0 && data[pos + 1] == 0 && data[pos + 2] & 0x80 != 0
}

/// Configuration for [`packetize_stream`].
#[derive(Debug, Clone, Copy)]
pub struct PacketizeConfig {
    /// Maximum size, in bytes, of one returned payload **including**
    /// its payload header (the RTP transport header is not counted).
    /// Must leave room for the 2-byte payload header plus at least one
    /// bitstream byte.
    pub max_payload: usize,
}

impl Default for PacketizeConfig {
    fn default() -> Self {
        // A common Ethernet-MTU-derived RTP payload budget: 1500 minus
        // IP/UDP/RTP headers, conservatively.
        PacketizeConfig { max_payload: 1440 }
    }
}

/// Packetize an H.263 elementary stream into RFC 4629 §6 payloads.
///
/// The stream must begin with a byte-aligned Picture Start Code (the
/// shape every encoder in this crate emits, §5.1.28 PSTUF-aligned).
/// Each returned `Vec<u8>` is one RTP packet's payload: the §5.1
/// payload header followed by bitstream bytes.
///
/// Cut policy (§7): a new packet always starts at every Picture Start
/// Code; within a picture the packetizer greedily fills each packet,
/// cutting at the **last byte-aligned start-code boundary** (GOB /
/// Slice / EOS) that fits the budget — those packets carry `P = 1`
/// with the two leading zero bytes stripped (§6.1). When no boundary
/// falls inside the budget (a segment larger than `max_payload`), the
/// cut is at an arbitrary byte position and the continuation is a
/// §6.2 Follow-on packet (`P = 0`).
///
/// # Errors
///
/// * [`Error::RtpPayloadTooSmall`] — `max_payload` cannot hold the
///   payload header plus one bitstream byte.
/// * [`Error::BadPictureStartCode`] — the stream does not begin with
///   a byte-aligned start code.
pub fn packetize_stream(stream: &[u8], cfg: PacketizeConfig) -> Result<Vec<Vec<u8>>> {
    if cfg.max_payload < PAYLOAD_HEADER_BYTES + 1 {
        return Err(Error::RtpPayloadTooSmall);
    }
    if stream.is_empty() {
        return Ok(Vec::new());
    }
    if !is_start_code_at(stream, 0) {
        return Err(Error::BadPictureStartCode);
    }

    // Collect every byte-aligned start-code position (each is a legal
    // P=1 packet start and a preferred cut point).
    let mut boundaries: Vec<usize> = Vec::new();
    let mut i = 0;
    while i + 3 <= stream.len() {
        if is_start_code_at(stream, i) {
            boundaries.push(i);
            // A start code consumes at least 17 bits; skip two bytes so
            // the trailing zeros of this code are not re-matched.
            i += 2;
        } else {
            i += 1;
        }
    }

    let budget = cfg.max_payload - PAYLOAD_HEADER_BYTES;
    let mut payloads = Vec::new();
    let mut pos = 0usize;
    // Whether the current position sits on a start-code boundary (the
    // first two zero bytes still present in `stream` at `pos`).
    let mut at_boundary = true;
    let mut boundary_idx = 0usize; // index into `boundaries` of `pos`

    while pos < stream.len() {
        // Data to emit for this packet: on a boundary the two zero
        // bytes are stripped and represented by P=1.
        let data_start = if at_boundary { pos + 2 } else { pos };
        let max_end = (data_start + budget).min(stream.len());

        // Preferred cut: the last start-code boundary in
        // `(data_start, max_end]`... a boundary *at* data_start would
        // yield an empty packet, so it must be strictly beyond.
        let mut cut = max_end;
        let mut next_at_boundary = false;
        if max_end < stream.len() {
            let candidate = boundaries[boundary_idx..]
                .iter()
                .copied()
                .take_while(|&b| b <= max_end)
                .filter(|&b| b > data_start)
                .last();
            if let Some(b) = candidate {
                cut = b;
                next_at_boundary = true;
            }
        } else {
            // Final packet: runs to the end of the stream.
            cut = stream.len();
        }

        let mut packet = Vec::with_capacity(PAYLOAD_HEADER_BYTES + (cut - data_start));
        write_payload_header(
            &mut packet,
            &H263PayloadHeader {
                p: at_boundary,
                vrc: None,
                extra_picture_header: Vec::new(),
                pebit: 0,
            },
        )?;
        packet.extend_from_slice(&stream[data_start..cut]);
        payloads.push(packet);

        pos = cut;
        at_boundary = next_at_boundary;
        while boundary_idx < boundaries.len() && boundaries[boundary_idx] < pos {
            boundary_idx += 1;
        }
        // When the cut was arbitrary but happens to land exactly on a
        // boundary, treat the continuation as a segment packet anyway
        // (strictly better resilience at zero cost).
        if !at_boundary && is_start_code_at(stream, pos) {
            at_boundary = true;
        }
    }

    Ok(payloads)
}

/// Reassemble an H.263 elementary stream from a sequence of RFC 4629
/// payloads (in transmission order, losslessly received).
///
/// The inverse of [`packetize_stream`]: each payload's header is
/// parsed, a `P = 1` packet re-synthesises the two stripped zero
/// bytes of its start code, and the bitstream bytes are concatenated.
/// A `PLEN > 0` redundant picture header is discarded — it duplicates
/// information present in the primary stream and exists for
/// loss-resilience only (§6.1.2); with all packets present the
/// primary picture header is authoritative.
///
/// # Errors
///
/// The parse errors of [`parse_payload_header`], plus
/// [`Error::RtpBadPayloadHeader`] if the **first** payload is not a
/// `P = 1` segment packet (a stream cannot begin with a Follow-on).
pub fn depacketize_payloads<I, B>(payloads: I) -> Result<Vec<u8>>
where
    I: IntoIterator<Item = B>,
    B: AsRef<[u8]>,
{
    let mut stream = Vec::new();
    let mut first = true;
    for payload in payloads {
        let payload = payload.as_ref();
        let (header, offset) = parse_payload_header(payload)?;
        if first && !header.p {
            return Err(Error::RtpBadPayloadHeader);
        }
        first = false;
        if header.p {
            // §6.1 — re-synthesise the two stripped zero bytes.
            stream.push(0);
            stream.push(0);
        }
        stream.extend_from_slice(&payload[offset..]);
    }
    Ok(stream)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_header_round_trips() {
        for p in [false, true] {
            for vrc in [
                None,
                Some(VrcHeader {
                    tid: 3,
                    trun: 9,
                    sync: true,
                }),
            ] {
                for (extra, pebit) in [
                    (Vec::new(), 0u8),
                    (vec![0x83, 0x00, 0x0F], 5),
                    (vec![0x80; 63], 7),
                ] {
                    let header = H263PayloadHeader {
                        p,
                        vrc,
                        extra_picture_header: extra,
                        pebit,
                    };
                    let mut bytes = Vec::new();
                    write_payload_header(&mut bytes, &header).unwrap();
                    // Trailing bitstream data must not confuse the parse.
                    bytes.extend_from_slice(&[0xAA, 0xBB]);
                    let (parsed, offset) = parse_payload_header(&bytes).unwrap();
                    assert_eq!(parsed, header);
                    assert_eq!(&bytes[offset..], &[0xAA, 0xBB]);
                }
            }
        }
    }

    #[test]
    fn payload_header_rejects_bad_fields() {
        let mut out = Vec::new();
        // PLEN > 63.
        assert!(matches!(
            write_payload_header(
                &mut out,
                &H263PayloadHeader {
                    p: true,
                    vrc: None,
                    extra_picture_header: vec![0u8; 64],
                    pebit: 0,
                },
            ),
            Err(Error::RtpBadPayloadHeader)
        ));
        // PEBIT without a header.
        assert!(matches!(
            write_payload_header(
                &mut out,
                &H263PayloadHeader {
                    p: true,
                    vrc: None,
                    extra_picture_header: Vec::new(),
                    pebit: 3,
                },
            ),
            Err(Error::RtpBadPayloadHeader)
        ));
        // Parse side: PEBIT non-zero with PLEN = 0 (word 0x0403 = P=1,
        // PLEN=0, PEBIT=3).
        assert!(matches!(
            parse_payload_header(&[0x04, 0x03, 0x12]),
            Err(Error::RtpBadPayloadHeader)
        ));
        // Truncated fixed part.
        assert!(matches!(
            parse_payload_header(&[0x04]),
            Err(Error::RtpTruncatedPacket)
        ));
        // Declared PLEN longer than the buffer.
        assert!(matches!(
            parse_payload_header(&[0x04, 0b0010_1000]),
            Err(Error::RtpTruncatedPacket)
        ));
    }

    #[test]
    fn rr_bits_are_ignored_on_parse() {
        // RR = 0b11111 with P=1: parse must succeed per §5.1.
        let (header, offset) = parse_payload_header(&[0xFC, 0x00, 0x55]).unwrap();
        assert!(header.p);
        assert_eq!(header.vrc, None);
        assert_eq!(offset, 2);
    }

    #[test]
    fn depacketize_rejects_leading_follow_on() {
        // P=0 first packet.
        let payload = [0x00u8, 0x00, 0x12, 0x34];
        assert!(matches!(
            depacketize_payloads([&payload[..]]),
            Err(Error::RtpBadPayloadHeader)
        ));
    }
}
