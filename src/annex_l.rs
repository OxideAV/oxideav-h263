//! Annex L / Annex W — supplemental enhancement information (SEI).
//!
//! Annex L defines the payload carried by the §5.1.24 PEI / §5.1.25
//! PSUPP picture-header extension loop: a sequence of *functions*,
//! each a four-bit function type FTYPE, a four-bit parameter size
//! DSIZE, and DSIZE octets of parameter data (§L.2, Table L.1).
//! Annex W assigns the two FTYPE values Table L.1 left reserved:
//! FTYPE 13 (Fixed-Point IDCT, §W.5) and FTYPE 14 (Picture Message,
//! §W.6 — a CONT/EBIT/MTYPE header octet plus message data,
//! Table W.2).
//!
//! This module stages the whole layer as pure transformations:
//!
//! * [`read_pei_psupp`] / [`write_pei_psupp`] — the §5.1.24/§5.1.25
//!   bit-level loop (one PEI bit before every PSUPP octet, terminated
//!   by a PEI of "0");
//! * [`parse_psupp`] / [`write_psupp`] — the §L.2 function layer over
//!   the collected octets, including the §L.3 start-code-emulation
//!   rule (a Do Nothing function is appended whenever the last five
//!   or more bits of the final octet are all zero);
//! * the typed function inventory [`SeiFunction`] covering §L.3–§L.15
//!   and §W.5/§W.6.
//!
//! Decoder response to most functions is display-side and outside the
//! reconstruction loop ("decoders which do not provide the enhanced
//! capabilities may simply discard any PSUPP information bits",
//! §L.1) — the pixel drivers in [`crate::picture`] therefore keep
//! skipping the loop, and callers who want the SEI use
//! [`crate::picture::extract_psupp`] + [`parse_psupp`].

use crate::{Error, Result};
use oxideav_core::bits::{BitReader, BitWriter};

/// §L.5 — a rectangle expressed in units of eight pixels: horizontal
/// and vertical location of the upper-left corner, then width and
/// height, one octet each (the format shared by the partial-picture
/// freeze / freeze-release / snapshot functions and both rectangles
/// of the resizing freeze).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PictureRect {
    /// Horizontal position of the upper-left corner, units of 8 px.
    pub x: u8,
    /// Vertical position of the upper-left corner, units of 8 px.
    pub y: u8,
    /// Rectangle width, units of 8 px.
    pub width: u8,
    /// Rectangle height, units of 8 px.
    pub height: u8,
}

impl PictureRect {
    fn parse(data: &[u8]) -> PictureRect {
        PictureRect {
            x: data[0],
            y: data[1],
            width: data[2],
            height: data[3],
        }
    }

    fn write(self, out: &mut Vec<u8>) {
        out.extend_from_slice(&[self.x, self.y, self.width, self.height]);
    }
}

/// §L.14 — Chroma Keying Information function payload.
///
/// The first octet is the representation order; with DSIZE > 1 a flag
/// octet follows (bits, most significant first: `AY`, `AB`, `AR`,
/// `A1`, `A2`, `RPB`, two reserved bits), then one octet per set key
/// flag (in `KY`, `KB`, `KR` order) and two octets — most significant
/// octet first — per set threshold flag (`T1` then `T2`). §L.14
/// constrains DSIZE to `1` or `2 + (set key flags) + 2 × (set
/// threshold flags)`; [`parse_psupp`] enforces that shape. Absent
/// parameters mean "reuse the previous keyed picture's values" (with
/// the §L.14 defaults `KY = 50`, `KB = 220`, `KR = 100`, `T1 = 48`,
/// `T2 = 75` before any are sent).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChromaKeyingInfo {
    /// Representation order of the current picture — lower orders form
    /// the background for higher orders.
    pub representation_order: u8,
    /// `KY` key value for luminance, when the `AY` flag is set.
    pub key_y: Option<u8>,
    /// `KB` key value for CB, when the `AB` flag is set.
    pub key_cb: Option<u8>,
    /// `KR` key value for CR, when the `AR` flag is set.
    pub key_cr: Option<u8>,
    /// `T1` transparency threshold, when the `A1` flag is set.
    pub threshold_t1: Option<u16>,
    /// `T2` opacity threshold, when the `A2` flag is set.
    pub threshold_t2: Option<u16>,
    /// RPB — hold the temporally previous reference picture as the
    /// opaque background for this and subsequent keyed pictures.
    pub reference_picture_background: bool,
    /// The two reserved flag bits (bits 7 and 8), preserved verbatim.
    pub reserved_flags: [bool; 2],
    /// Whether the flag octet was present at all (DSIZE > 1). A
    /// one-octet CKIF (`false`) reuses every previous parameter.
    pub has_flag_octet: bool,
}

/// §W.6.3 / Table W.2 — Picture Message MTYPE inventory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    /// 0 — arbitrary binary data.
    ArbitraryBinaryData,
    /// 1 — arbitrary UTF-8 text.
    ArbitraryText,
    /// 2 — copyright UTF-8 text.
    CopyrightText,
    /// 3 — caption UTF-8 text.
    CaptionText,
    /// 4 — video description UTF-8 text.
    VideoDescriptionText,
    /// 5 — Uniform Resource Identifier UTF-8 text.
    UriText,
    /// 6 — current picture header repetition.
    CurrentPictureHeaderRepetition,
    /// 7 — previous picture header repetition.
    PreviousPictureHeaderRepetition,
    /// 8 — next picture header repetition, reliable TR.
    NextPictureHeaderRepetitionReliableTr,
    /// 9 — next picture header repetition, unreliable TR.
    NextPictureHeaderRepetitionUnreliableTr,
    /// 10 — top interlaced field indication.
    TopInterlacedField,
    /// 11 — bottom interlaced field indication.
    BottomInterlacedField,
    /// 12 — picture number (10 bits in two data octets).
    PictureNumber,
    /// 13 — spare reference pictures.
    SpareReferencePictures,
    /// 14..=15 — reserved MTYPE values, preserved verbatim.
    Reserved(u8),
}

impl MessageType {
    /// The 4-bit Table W.2 code for this message type.
    pub fn code(self) -> u8 {
        match self {
            MessageType::ArbitraryBinaryData => 0,
            MessageType::ArbitraryText => 1,
            MessageType::CopyrightText => 2,
            MessageType::CaptionText => 3,
            MessageType::VideoDescriptionText => 4,
            MessageType::UriText => 5,
            MessageType::CurrentPictureHeaderRepetition => 6,
            MessageType::PreviousPictureHeaderRepetition => 7,
            MessageType::NextPictureHeaderRepetitionReliableTr => 8,
            MessageType::NextPictureHeaderRepetitionUnreliableTr => 9,
            MessageType::TopInterlacedField => 10,
            MessageType::BottomInterlacedField => 11,
            MessageType::PictureNumber => 12,
            MessageType::SpareReferencePictures => 13,
            MessageType::Reserved(v) => v,
        }
    }

    fn from_code(code: u8) -> MessageType {
        match code {
            0 => MessageType::ArbitraryBinaryData,
            1 => MessageType::ArbitraryText,
            2 => MessageType::CopyrightText,
            3 => MessageType::CaptionText,
            4 => MessageType::VideoDescriptionText,
            5 => MessageType::UriText,
            6 => MessageType::CurrentPictureHeaderRepetition,
            7 => MessageType::PreviousPictureHeaderRepetition,
            8 => MessageType::NextPictureHeaderRepetitionReliableTr,
            9 => MessageType::NextPictureHeaderRepetitionUnreliableTr,
            10 => MessageType::TopInterlacedField,
            11 => MessageType::BottomInterlacedField,
            12 => MessageType::PictureNumber,
            13 => MessageType::SpareReferencePictures,
            v => MessageType::Reserved(v & 0x0F),
        }
    }

    /// §W.6.2 — whether EBIT carries a text track number (text
    /// message types) rather than an ignore-bits count.
    pub fn is_text(self) -> bool {
        matches!(
            self,
            MessageType::ArbitraryText
                | MessageType::CopyrightText
                | MessageType::CaptionText
                | MessageType::VideoDescriptionText
                | MessageType::UriText
        )
    }
}

/// §W.6 — one Picture Message function: the CONT / EBIT / MTYPE
/// header octet (Figure W.1) plus the `DSIZE − 1` message data octets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PictureMessage {
    /// CONT — this message continues into the next picture message
    /// function (logical messages longer than 14 octets).
    pub continuation: bool,
    /// EBIT — for non-text messages, the count of least-significant
    /// bits to ignore in the last data octet; for text messages, the
    /// text track number (§W.6.2).
    pub ebit: u8,
    /// MTYPE — Table W.2 message type.
    pub message_type: MessageType,
    /// The message data octets (everything after the header octet).
    pub data: Vec<u8>,
}

impl PictureMessage {
    /// §W.6.3.12 — the 10-bit Picture Number carried by a
    /// `PictureNumber` message (two data octets, EBIT = 6: the six
    /// least-significant bits of the last octet are ignored bits, so
    /// the number is the top 10 of the 16 data bits).
    pub fn picture_number(&self) -> Option<u16> {
        if self.message_type != MessageType::PictureNumber || self.data.len() != 2 {
            return None;
        }
        Some(((self.data[0] as u16) << 2) | (self.data[1] as u16 >> 6))
    }

    /// Build a §W.6.3.12 Picture Number message for `number`
    /// (10 bits, wrapped modulo 1024).
    pub fn for_picture_number(number: u16) -> PictureMessage {
        let n = number & 0x3FF;
        PictureMessage {
            continuation: false,
            ebit: 6,
            message_type: MessageType::PictureNumber,
            data: vec![(n >> 2) as u8, ((n & 0x3) as u8) << 6],
        }
    }

    /// §W.6 constraint check, enforced on parse and write: DSIZE
    /// (= 1 + data length) fits the 4-bit field; the interlaced field
    /// indications require DSIZE = 1, CONT = 0, EBIT = 0 (§W.6.3.11);
    /// Picture Number requires DSIZE = 3, CONT = 0, EBIT = 6
    /// (§W.6.3.12); non-text messages with CONT = 1 or no data octets
    /// require EBIT = 0 (§W.6.2).
    fn validate(&self) -> Result<()> {
        if self.data.len() > 14 {
            // DSIZE is four bits: at most 15 octets including the
            // header octet.
            return Err(Error::BadPictureMessage);
        }
        if self.ebit > 7 {
            return Err(Error::BadPictureMessage);
        }
        match self.message_type {
            MessageType::TopInterlacedField | MessageType::BottomInterlacedField => {
                if !self.data.is_empty() || self.continuation || self.ebit != 0 {
                    return Err(Error::BadPictureMessage);
                }
            }
            MessageType::PictureNumber => {
                if self.data.len() != 2 || self.continuation || self.ebit != 6 {
                    return Err(Error::BadPictureMessage);
                }
            }
            // §W.6.2 — in non-text messages EBIT must be zero when
            // the message continues or carries no data octet.
            t if !t.is_text() && (self.continuation || self.data.is_empty()) && self.ebit != 0 => {
                return Err(Error::BadPictureMessage);
            }
            _ => {}
        }
        Ok(())
    }
}

/// One §L.2 PSUPP function — the typed inventory of Table L.1 plus
/// the Annex W assignments (Table W.1).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SeiFunction {
    /// FTYPE 1, §L.3 — no action; inserted to prevent start-code
    /// emulation (see [`write_psupp`]).
    DoNothing,
    /// FTYPE 2, §L.4 — keep the entire prior displayed picture
    /// unchanged until freeze release or timeout.
    FullPictureFreeze,
    /// FTYPE 3, §L.5 — keep a rectangular area of the prior displayed
    /// picture unchanged.
    PartialPictureFreeze(PictureRect),
    /// FTYPE 4, §L.6 — freeze a displayed-picture rectangle fed by a
    /// larger decoded-picture rectangle (resized to fit).
    ResizingPartialPictureFreeze {
        /// The affected rectangle of the displayed picture.
        displayed: PictureRect,
        /// The corresponding (larger) rectangle of the decoded picture.
        decoded: PictureRect,
    },
    /// FTYPE 5, §L.7 — release a frozen rectangle for updating.
    PartialPictureFreezeRelease(PictureRect),
    /// FTYPE 6, §L.8 — label the current picture as a snapshot, with a
    /// 32-bit identification number (most significant octet first).
    FullPictureSnapshot(u32),
    /// FTYPE 7, §L.9 — label a rectangle of the current picture as a
    /// snapshot.
    PartialPictureSnapshot {
        /// Snapshot identification number for external use.
        id: u32,
        /// The tagged rectangle of the decoded picture.
        rect: PictureRect,
    },
    /// FTYPE 8, §L.10 — begin a tagged video time segment.
    VideoTimeSegmentStart(u32),
    /// FTYPE 9, §L.11 — end a tagged video time segment.
    VideoTimeSegmentEnd(u32),
    /// FTYPE 10, §L.12 — begin a progressive refinement segment.
    ProgressiveRefinementStart(u32),
    /// FTYPE 11, §L.13 — end a progressive refinement segment.
    ProgressiveRefinementEnd(u32),
    /// FTYPE 12, §L.14 — chroma keying information.
    ChromaKeying(ChromaKeyingInfo),
    /// FTYPE 13, §W.5 — the bitstream was constructed with the
    /// indicated fixed-point IDCT approximation (0 = the §W.5.3
    /// reference IDCT; 1..=255 reserved).
    FixedPointIdct(u8),
    /// FTYPE 14, §W.6 — picture message.
    PictureMessage(PictureMessage),
    /// FTYPE 15, §L.15 — extended function: the following octet's top
    /// four bits are the extended function code, its bottom four bits
    /// a DSIZE for `data`.
    ExtendedFunction {
        /// The four-bit extended function code.
        function: u8,
        /// The extended function's parameter octets.
        data: Vec<u8>,
    },
    /// FTYPE 0 — reserved by Table L.1; parameter octets preserved
    /// verbatim so an unknown-but-legal stream round-trips.
    Reserved {
        /// The reserved FTYPE value (0).
        ftype: u8,
        /// The DSIZE parameter octets, preserved verbatim.
        data: Vec<u8>,
    },
}

/// Read `n` octets from `data` at `*pos`, advancing it.
fn take<'a>(data: &'a [u8], pos: &mut usize, n: usize) -> Result<&'a [u8]> {
    if data.len() - *pos < n {
        return Err(Error::TruncatedPsupp);
    }
    let out = &data[*pos..*pos + n];
    *pos += n;
    Ok(out)
}

fn read_u32_msb(data: &[u8]) -> u32 {
    ((data[0] as u32) << 24) | ((data[1] as u32) << 16) | ((data[2] as u32) << 8) | data[3] as u32
}

/// §L.14 — parse the Chroma Keying Information parameter octets.
fn parse_chroma_keying(params: &[u8]) -> Result<ChromaKeyingInfo> {
    // DSIZE 1..=9 (§L.14); the caller has already bounded DSIZE to
    // the four-bit range, so only the shape rules remain.
    if params.is_empty() || params.len() > 9 {
        return Err(Error::BadSupplementalDsize);
    }
    let mut info = ChromaKeyingInfo {
        representation_order: params[0],
        ..ChromaKeyingInfo::default()
    };
    if params.len() == 1 {
        return Ok(info);
    }
    info.has_flag_octet = true;
    let flags = params[1];
    let a_y = flags & 0x80 != 0;
    let a_b = flags & 0x40 != 0;
    let a_r = flags & 0x20 != 0;
    let a_1 = flags & 0x10 != 0;
    let a_2 = flags & 0x08 != 0;
    info.reference_picture_background = flags & 0x04 != 0;
    info.reserved_flags = [flags & 0x02 != 0, flags & 0x01 != 0];
    // §L.14 — DSIZE shall equal 2 + set key flags + 2 × set threshold
    // flags (the DSIZE = 1 form was handled above).
    let keys = [a_y, a_b, a_r].iter().filter(|f| **f).count();
    let thresholds = [a_1, a_2].iter().filter(|f| **f).count();
    if params.len() != 2 + keys + 2 * thresholds {
        return Err(Error::BadSupplementalDsize);
    }
    let mut pos = 2usize;
    let mut key = |present: bool| -> Option<u8> {
        if present {
            let v = params[pos];
            pos += 1;
            Some(v)
        } else {
            None
        }
    };
    info.key_y = key(a_y);
    info.key_cb = key(a_b);
    info.key_cr = key(a_r);
    let mut threshold = |present: bool| -> Option<u16> {
        if present {
            let v = ((params[pos] as u16) << 8) | params[pos + 1] as u16;
            pos += 2;
            Some(v)
        } else {
            None
        }
    };
    info.threshold_t1 = threshold(a_1);
    info.threshold_t2 = threshold(a_2);
    Ok(info)
}

/// §L.14 — write the Chroma Keying Information parameter octets.
fn write_chroma_keying(info: &ChromaKeyingInfo, out: &mut Vec<u8>) -> Result<()> {
    out.push(info.representation_order);
    if !info.has_flag_octet {
        // The DSIZE = 1 "reuse previous parameters" form must not
        // carry parameters.
        if info.key_y.is_some()
            || info.key_cb.is_some()
            || info.key_cr.is_some()
            || info.threshold_t1.is_some()
            || info.threshold_t2.is_some()
            || info.reference_picture_background
            || info.reserved_flags != [false, false]
        {
            return Err(Error::BadSupplementalDsize);
        }
        return Ok(());
    }
    let mut flags = 0u8;
    if info.key_y.is_some() {
        flags |= 0x80;
    }
    if info.key_cb.is_some() {
        flags |= 0x40;
    }
    if info.key_cr.is_some() {
        flags |= 0x20;
    }
    if info.threshold_t1.is_some() {
        flags |= 0x10;
    }
    if info.threshold_t2.is_some() {
        flags |= 0x08;
    }
    if info.reference_picture_background {
        flags |= 0x04;
    }
    if info.reserved_flags[0] {
        flags |= 0x02;
    }
    if info.reserved_flags[1] {
        flags |= 0x01;
    }
    out.push(flags);
    for key in [info.key_y, info.key_cb, info.key_cr].into_iter().flatten() {
        out.push(key);
    }
    for threshold in [info.threshold_t1, info.threshold_t2].into_iter().flatten() {
        out.push((threshold >> 8) as u8);
        out.push((threshold & 0xFF) as u8);
    }
    Ok(())
}

/// Parse a PSUPP octet string (as collected by [`read_pei_psupp`])
/// into its §L.2 function sequence.
///
/// Enforces the per-function DSIZE "shall" rules of §L.3–§L.15 and
/// §W.5/§W.6; a reserved FTYPE (0) and reserved picture-message MTYPEs
/// are preserved verbatim rather than rejected, per the §L.2 guidance
/// that unsupported functions are skippable by DSIZE.
///
/// # Errors
///
/// * [`Error::TruncatedPsupp`] — the octet string ends inside a
///   function's declared parameter data.
/// * [`Error::BadSupplementalDsize`] — a DSIZE outside its function's
///   mandated value (§L.3–§L.13: fixed sizes; §L.14: the flag-derived
///   size rule; §L.15 / §W.5: fixed sizes).
/// * [`Error::BadPictureMessage`] — a §W.6 constraint violation (see
///   [`PictureMessage`]).
pub fn parse_psupp(data: &[u8]) -> Result<Vec<SeiFunction>> {
    let mut out = Vec::new();
    let mut pos = 0usize;
    while pos < data.len() {
        let head = data[pos];
        pos += 1;
        let ftype = head >> 4;
        let dsize = (head & 0x0F) as usize;
        let expect = |want: usize| -> Result<()> {
            if dsize == want {
                Ok(())
            } else {
                Err(Error::BadSupplementalDsize)
            }
        };
        let func = match ftype {
            1 => {
                expect(0)?;
                SeiFunction::DoNothing
            }
            2 => {
                expect(0)?;
                SeiFunction::FullPictureFreeze
            }
            3 => {
                expect(4)?;
                SeiFunction::PartialPictureFreeze(PictureRect::parse(take(data, &mut pos, 4)?))
            }
            4 => {
                expect(8)?;
                let params = take(data, &mut pos, 8)?;
                SeiFunction::ResizingPartialPictureFreeze {
                    displayed: PictureRect::parse(&params[..4]),
                    decoded: PictureRect::parse(&params[4..]),
                }
            }
            5 => {
                expect(4)?;
                SeiFunction::PartialPictureFreezeRelease(PictureRect::parse(take(
                    data, &mut pos, 4,
                )?))
            }
            6 => {
                expect(4)?;
                SeiFunction::FullPictureSnapshot(read_u32_msb(take(data, &mut pos, 4)?))
            }
            7 => {
                expect(8)?;
                let params = take(data, &mut pos, 8)?;
                SeiFunction::PartialPictureSnapshot {
                    id: read_u32_msb(&params[..4]),
                    rect: PictureRect::parse(&params[4..]),
                }
            }
            8 => {
                expect(4)?;
                SeiFunction::VideoTimeSegmentStart(read_u32_msb(take(data, &mut pos, 4)?))
            }
            9 => {
                expect(4)?;
                SeiFunction::VideoTimeSegmentEnd(read_u32_msb(take(data, &mut pos, 4)?))
            }
            10 => {
                expect(4)?;
                SeiFunction::ProgressiveRefinementStart(read_u32_msb(take(data, &mut pos, 4)?))
            }
            11 => {
                expect(4)?;
                SeiFunction::ProgressiveRefinementEnd(read_u32_msb(take(data, &mut pos, 4)?))
            }
            12 => {
                let params = take(data, &mut pos, dsize)?;
                SeiFunction::ChromaKeying(parse_chroma_keying(params)?)
            }
            13 => {
                expect(1)?;
                SeiFunction::FixedPointIdct(take(data, &mut pos, 1)?[0])
            }
            14 => {
                if dsize == 0 {
                    return Err(Error::BadSupplementalDsize);
                }
                let params = take(data, &mut pos, dsize)?;
                let msg = PictureMessage {
                    continuation: params[0] & 0x80 != 0,
                    ebit: (params[0] >> 4) & 0x07,
                    message_type: MessageType::from_code(params[0] & 0x0F),
                    data: params[1..].to_vec(),
                };
                msg.validate()?;
                SeiFunction::PictureMessage(msg)
            }
            15 => {
                // §L.15 — DSIZE shall be zero; the next octet carries
                // the extended function code (top 4 bits) and its own
                // DSIZE (bottom 4 bits).
                expect(0)?;
                let ext = take(data, &mut pos, 1)?[0];
                let ext_dsize = (ext & 0x0F) as usize;
                let params = take(data, &mut pos, ext_dsize)?;
                SeiFunction::ExtendedFunction {
                    function: ext >> 4,
                    data: params.to_vec(),
                }
            }
            _ => {
                // FTYPE 0 — reserved; skip by DSIZE, preserving the
                // parameter octets.
                let params = take(data, &mut pos, dsize)?;
                SeiFunction::Reserved {
                    ftype,
                    data: params.to_vec(),
                }
            }
        };
        out.push(func);
    }
    Ok(out)
}

/// Serialise a §L.2 function sequence into the PSUPP octet string,
/// applying the §L.3 start-code-emulation rule: whenever the last
/// five or more bits of the final octet are all zero, a Do Nothing
/// function (FTYPE 1, DSIZE 0 — octet `0x10`) is appended.
///
/// # Errors
///
/// * [`Error::BadSupplementalDsize`] — a function whose parameter
///   shape cannot be represented (an [`SeiFunction::ExtendedFunction`]
///   or [`SeiFunction::Reserved`] with more than 15 data octets, an
///   extended function code above 15, or an inconsistent
///   [`ChromaKeyingInfo`]).
/// * [`Error::BadPictureMessage`] — a §W.6 constraint violation.
pub fn write_psupp(functions: &[SeiFunction]) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    let head = |out: &mut Vec<u8>, ftype: u8, dsize: usize| -> Result<()> {
        if dsize > 15 {
            return Err(Error::BadSupplementalDsize);
        }
        out.push((ftype << 4) | dsize as u8);
        Ok(())
    };
    for func in functions {
        match func {
            SeiFunction::DoNothing => head(&mut out, 1, 0)?,
            SeiFunction::FullPictureFreeze => head(&mut out, 2, 0)?,
            SeiFunction::PartialPictureFreeze(rect) => {
                head(&mut out, 3, 4)?;
                rect.write(&mut out);
            }
            SeiFunction::ResizingPartialPictureFreeze { displayed, decoded } => {
                head(&mut out, 4, 8)?;
                displayed.write(&mut out);
                decoded.write(&mut out);
            }
            SeiFunction::PartialPictureFreezeRelease(rect) => {
                head(&mut out, 5, 4)?;
                rect.write(&mut out);
            }
            SeiFunction::FullPictureSnapshot(id) => {
                head(&mut out, 6, 4)?;
                out.extend_from_slice(&id.to_be_bytes());
            }
            SeiFunction::PartialPictureSnapshot { id, rect } => {
                head(&mut out, 7, 8)?;
                out.extend_from_slice(&id.to_be_bytes());
                rect.write(&mut out);
            }
            SeiFunction::VideoTimeSegmentStart(id) => {
                head(&mut out, 8, 4)?;
                out.extend_from_slice(&id.to_be_bytes());
            }
            SeiFunction::VideoTimeSegmentEnd(id) => {
                head(&mut out, 9, 4)?;
                out.extend_from_slice(&id.to_be_bytes());
            }
            SeiFunction::ProgressiveRefinementStart(id) => {
                head(&mut out, 10, 4)?;
                out.extend_from_slice(&id.to_be_bytes());
            }
            SeiFunction::ProgressiveRefinementEnd(id) => {
                head(&mut out, 11, 4)?;
                out.extend_from_slice(&id.to_be_bytes());
            }
            SeiFunction::ChromaKeying(info) => {
                let mut params = Vec::new();
                write_chroma_keying(info, &mut params)?;
                head(&mut out, 12, params.len())?;
                out.extend_from_slice(&params);
            }
            SeiFunction::FixedPointIdct(idct) => {
                head(&mut out, 13, 1)?;
                out.push(*idct);
            }
            SeiFunction::PictureMessage(msg) => {
                msg.validate()?;
                head(&mut out, 14, 1 + msg.data.len())?;
                out.push(
                    (u8::from(msg.continuation) << 7)
                        | (msg.ebit << 4)
                        | (msg.message_type.code() & 0x0F),
                );
                out.extend_from_slice(&msg.data);
            }
            SeiFunction::ExtendedFunction { function, data } => {
                if *function > 15 || data.len() > 15 {
                    return Err(Error::BadSupplementalDsize);
                }
                head(&mut out, 15, 0)?;
                out.push((function << 4) | data.len() as u8);
                out.extend_from_slice(data);
            }
            SeiFunction::Reserved { ftype, data } => {
                if *ftype != 0 {
                    return Err(Error::BadSupplementalDsize);
                }
                head(&mut out, 0, data.len())?;
                out.extend_from_slice(data);
            }
        }
    }
    // §L.3 — whenever the last five or more bits of the final octet
    // are all zero and no further functions follow, a Do Nothing
    // function shall be inserted to prevent start-code emulation.
    if let Some(&last) = out.last() {
        if last & 0x1F == 0 {
            out.push(0x10);
        }
    }
    Ok(out)
}

/// Read the §5.1.24 / §5.1.25 PEI + PSUPP extension loop at the
/// reader's position, **collecting** the PSUPP octets (the counterpart
/// of the drivers' internal skip): PEI is one bit; while set, an
/// eight-bit PSUPP octet follows and then another PEI bit.
///
/// # Errors
///
/// [`Error::UnexpectedEof`] if the bitstream ends inside the loop.
pub fn read_pei_psupp(reader: &mut BitReader<'_>) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    loop {
        let pei = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        if !pei {
            return Ok(out);
        }
        out.push(reader.read_u32(8).map_err(|_| Error::UnexpectedEof)? as u8);
    }
}

/// Write the §5.1.24 / §5.1.25 PEI + PSUPP loop for the given octets:
/// a PEI of "1" before every octet and a terminating PEI of "0".
pub fn write_pei_psupp(writer: &mut BitWriter, octets: &[u8]) {
    for &octet in octets {
        writer.write_bit(true);
        writer.write_bits(octet as u32, 8);
    }
    writer.write_bit(false);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x: u8, y: u8, w: u8, h: u8) -> PictureRect {
        PictureRect {
            x,
            y,
            width: w,
            height: h,
        }
    }

    #[test]
    fn every_function_round_trips() {
        let funcs = vec![
            SeiFunction::FullPictureFreeze,
            SeiFunction::PartialPictureFreeze(rect(0, 0, 3, 2)),
            SeiFunction::ResizingPartialPictureFreeze {
                displayed: rect(1, 1, 2, 2),
                decoded: rect(4, 4, 8, 8),
            },
            SeiFunction::PartialPictureFreezeRelease(rect(0, 0, 3, 2)),
            SeiFunction::FullPictureSnapshot(0xDEAD_BEEF),
            SeiFunction::PartialPictureSnapshot {
                id: 7,
                rect: rect(2, 3, 4, 5),
            },
            SeiFunction::VideoTimeSegmentStart(1),
            SeiFunction::VideoTimeSegmentEnd(1),
            SeiFunction::ProgressiveRefinementStart(2),
            SeiFunction::ProgressiveRefinementEnd(2),
            SeiFunction::FixedPointIdct(0),
            SeiFunction::ExtendedFunction {
                function: 5,
                data: vec![0xAA, 0x55],
            },
            SeiFunction::Reserved {
                ftype: 0,
                data: vec![0x11],
            },
            SeiFunction::DoNothing,
        ];
        let octets = write_psupp(&funcs).expect("write");
        assert_eq!(parse_psupp(&octets).expect("parse"), funcs);
    }

    #[test]
    fn chroma_keying_shapes_round_trip() {
        // DSIZE = 1 reuse form.
        let reuse = SeiFunction::ChromaKeying(ChromaKeyingInfo {
            representation_order: 3,
            ..ChromaKeyingInfo::default()
        });
        // Full form: three keys + both thresholds + RPB → DSIZE 9.
        let full = SeiFunction::ChromaKeying(ChromaKeyingInfo {
            representation_order: 1,
            key_y: Some(50),
            key_cb: Some(220),
            key_cr: Some(100),
            threshold_t1: Some(48),
            threshold_t2: Some(75),
            reference_picture_background: true,
            reserved_flags: [false, false],
            has_flag_octet: true,
        });
        // Partial form: one key, one threshold → DSIZE 5.
        let partial = SeiFunction::ChromaKeying(ChromaKeyingInfo {
            representation_order: 2,
            key_cb: Some(200),
            threshold_t2: Some(300),
            has_flag_octet: true,
            ..ChromaKeyingInfo::default()
        });
        for func in [reuse, full, partial] {
            let octets = write_psupp(std::slice::from_ref(&func)).expect("write");
            let parsed = parse_psupp(&octets).expect("parse");
            // write_psupp may append a Do Nothing for emulation
            // prevention; the first function must round-trip.
            assert_eq!(parsed[0], func);
        }
    }

    #[test]
    fn chroma_keying_dsize_rule_enforced() {
        // Flag octet claims KY present but no key octet follows:
        // FTYPE 12, DSIZE 2, order, flags(AY).
        let octets = [0xC2, 0x00, 0x80];
        assert_eq!(
            parse_psupp(&octets).unwrap_err(),
            Error::BadSupplementalDsize
        );
    }

    #[test]
    fn do_nothing_appended_when_low_five_bits_zero() {
        // Full-picture freeze is octet 0x20 — its last five bits are
        // zero, so §L.3 requires a trailing Do Nothing.
        let octets = write_psupp(&[SeiFunction::FullPictureFreeze]).expect("write");
        assert_eq!(octets, vec![0x20, 0x10]);
        // A function ending in a non-zero low quintet needs none.
        let octets = write_psupp(&[SeiFunction::FixedPointIdct(0x33)]).expect("write");
        assert_eq!(octets, vec![0xD1, 0x33]);
    }

    #[test]
    fn picture_message_constraints() {
        let number = PictureMessage::for_picture_number(1023);
        assert_eq!(number.picture_number(), Some(1023));
        let octets = write_psupp(&[SeiFunction::PictureMessage(number.clone())]).expect("write");
        match &parse_psupp(&octets).expect("parse")[0] {
            SeiFunction::PictureMessage(m) => assert_eq!(m.picture_number(), Some(1023)),
            other => panic!("wrong function {other:?}"),
        }
        // Interlaced field indication must have no data / CONT / EBIT.
        let bad = PictureMessage {
            continuation: false,
            ebit: 1,
            message_type: MessageType::TopInterlacedField,
            data: Vec::new(),
        };
        assert_eq!(
            write_psupp(&[SeiFunction::PictureMessage(bad)]).unwrap_err(),
            Error::BadPictureMessage
        );
        // Text message with a UTF-8 payload and a track number.
        let text = PictureMessage {
            continuation: false,
            ebit: 2,
            message_type: MessageType::CaptionText,
            data: b"hi".to_vec(),
        };
        let octets = write_psupp(&[SeiFunction::PictureMessage(text.clone())]).expect("write");
        assert_eq!(
            parse_psupp(&octets).expect("parse")[0],
            SeiFunction::PictureMessage(text)
        );
    }

    #[test]
    fn truncated_parameters_rejected() {
        // Partial-picture freeze declares four octets, only two follow.
        let octets = [0x34, 0x00, 0x00];
        assert_eq!(parse_psupp(&octets).unwrap_err(), Error::TruncatedPsupp);
    }

    #[test]
    fn pei_loop_round_trips() {
        let octets = vec![0x20, 0x10, 0xAB];
        let mut w = BitWriter::new();
        write_pei_psupp(&mut w, &octets);
        w.write_bits(0b1010_1010, 8); // trailing payload
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_pei_psupp(&mut r).expect("read"), octets);
        assert_eq!(r.read_u32(8).expect("payload"), 0b1010_1010);
    }
}
