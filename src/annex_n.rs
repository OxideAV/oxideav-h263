//! Annex N — Reference Picture Selection mode (forward-channel decode).
//!
//! Annex N ("NEWPRED") lets the encoder pick, for each picture, GOB or
//! slice, which previously-decoded reference picture to predict from —
//! rather than always the most recent anchor. The encoder signals the
//! choice with the §5.1.14 / §N.4.1.3 **TRPI** flag and, when set, the
//! §5.1.15 / §N.4.1.4 **TRP** field carrying the Temporal Reference of
//! the picture to predict from.
//!
//! The decoder side (§N.5) maintains additional picture memories storing
//! correctly-decoded pictures together with their Temporal Reference,
//! and "uses the stored picture for which the TR is TRP as the reference
//! picture for inter-frame decoding instead of the last decoded picture,
//! if the TRP field exists in the forward-channel data". When TRP is not
//! present, "the most recent temporally previous anchor picture shall be
//! used for prediction, as when not in the Reference Picture Selection
//! mode" (§N.4.1.4).
//!
//! ## Scope of this module
//!
//! This module implements the **forward-channel** decoder behaviour: the
//! [`RpsReferenceStore`] picture memory keyed by 10-bit Temporal
//! Reference, and the §N.4.1.4 / §N.5 reference-selection rule. It also
//! stages the §N.4.2 **Back-Channel Message** (BCM) syntax itself —
//! [`parse_bcm`] / [`write_bcm`] over [`BackChannelMessage`] — the
//! ACK / NACK record a decoder returns to the encoder. BCM flows on a
//! separate logical channel (or the §N.4.2.8 videomux submode) and does
//! not affect the pixels a conformant forward-channel decoder produces;
//! the §5.1.16 BCI codeword inside a *forward* picture header is still
//! framed (and a present in-header BCM refused) by
//! [`crate::plus_ptype`], because the in-header BCM's GN/MBA width is a
//! property of the *other* video bitstream the message applies to
//! (§N.4.2.9 NOTE) — knowledge only the negotiating caller has, which
//! is exactly what [`BcmContext`] carries.
//!
//! The §N.4.1 GOB / slice-layer TRI / TR / TRPI / TRP fields (which let
//! the reference be re-selected mid-picture) are a further refinement;
//! this module's store + selection serve the picture-layer TRP, which is
//! the unit the single-picture decode API operates on.

use crate::picture::YuvFrame;
use crate::{Error, Result};
use oxideav_core::bits::{BitReader, BitWriter};

/// Length in bits of the §N.4.1.4 GOB/slice-layer TRP field (always 10
/// bits, unlike the picture-layer TR which is 8 or 10 bits per the
/// custom-PCF state).
pub const NEWPRED_TRP_BITS: u32 = 10;

/// §N.4.1 — the per-segment (GOB or slice) NEWPRED reference-selection
/// fields appended to the GOB / slice header when the Reference Picture
/// Selection mode (Annex N) is in use.
///
/// Figure N.2 / N.3 insert these fields **after** the standard
/// GBSC/GN/(GSBI)/GFID/GQUANT GOB header (or the SSC.../GFID slice
/// header) and **before** the macroblock data:
///
/// ```text
///   ... GFID GQUANT | TRI TR TRPI TRP | BCI [BCM] | Macroblock data
/// ```
///
/// Their semantics mirror the §5.1.14 / §5.1.15 picture-header fields,
/// but they re-select the prediction reference for the macroblocks of
/// *this* segment only — "TRP is valid until the next PSC, GSC, or SSC"
/// (§N.4.1.4). The §N.4.1.5 BCI codeword is parsed (and a present
/// §N.4.1.6 BCM refused — it is a decoder → encoder message with no
/// forward-channel pixel effect; a forward-channel BCI is always
/// `"01"`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GobNewpredFields {
    /// §N.4.1.2 — the segment's own Temporal Reference, present iff
    /// §N.4.1.1 TRI was `1`. When a custom picture clock frequency is in
    /// use it is the 10-bit ETR∥TR concatenation, else the 8-bit TR.
    pub tr: Option<u16>,
    /// §N.4.1.3 — TRPI: whether the following §N.4.1.4 TRP field is
    /// present. Must be `false` for an I- or EI-picture (the parser
    /// enforces this against the picture type).
    pub trpi: bool,
    /// §N.4.1.4 — the 10-bit Temporal Reference of the reference picture
    /// this segment predicts from, present iff `trpi`. When absent "the
    /// most recent temporally previous anchor picture shall be used".
    pub trp: Option<u16>,
    /// Total number of bits this NEWPRED field group consumed (TRI + any
    /// TR + TRPI + any TRP + the BCI codeword), so a caller composing a
    /// bit cursor can advance past it.
    pub field_bits: u32,
}

impl GobNewpredFields {
    /// Resolve the prediction reference's Temporal Reference for this
    /// segment. Returns `Some(trp)` when the segment re-selected a
    /// reference (§N.4.1.4 TRP present), else `None` (the caller falls
    /// back to "the most recent temporally previous anchor picture").
    pub fn segment_trp(&self) -> Option<u16> {
        if self.trpi {
            self.trp
        } else {
            None
        }
    }
}

/// §N.4.1 — parse the per-segment NEWPRED fields (TRI / TR / TRPI / TRP
/// / BCI) that follow a GOB or slice header when the Reference Picture
/// Selection mode is in use.
///
/// `custom_pcf` selects the §N.4.1.2 TR width (10 bits with a custom
/// picture clock frequency, else 8 bits). `is_intra_or_ei` enforces the
/// §N.4.1.3 rule that TRPI must be `0` for an I- or EI-picture.
///
/// On success the reader is positioned at the first bit of the
/// segment's macroblock data.
///
/// ### Errors
///
/// * [`Error::UnexpectedEof`] — the buffer ended inside the fields.
/// * [`Error::PlusPtypeReservedField`] — TRPI was `1` on an I/EI
///   picture (§N.4.1.3).
/// * [`Error::BadBackChannelMessage`] — the BCI codeword signalled a
///   present back-channel message (`"1"`) or carried the undefined
///   `"00"` shape (§N.4.1.5).
pub fn parse_gob_newpred_fields(
    reader: &mut BitReader<'_>,
    custom_pcf: bool,
    is_intra_or_ei: bool,
) -> Result<GobNewpredFields> {
    let mut field_bits = 0u32;

    // §N.4.1.1 — TRI (1 bit): is the TR field present?
    let tri = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    field_bits += 1;

    // §N.4.1.2 — TR (8 bits, or 10 with a custom PCF), present iff TRI.
    let tr = if tri {
        let width = if custom_pcf { 10 } else { 8 };
        let v = reader.read_u32(width).map_err(|_| Error::UnexpectedEof)? as u16;
        field_bits += width;
        Some(v)
    } else {
        None
    };

    // §N.4.1.3 — TRPI (1 bit).
    let trpi = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    field_bits += 1;
    if trpi && is_intra_or_ei {
        // §N.4.1.3 — "TRPI shall be equal to zero whenever the picture
        // is an I- or EI-picture."
        return Err(Error::PlusPtypeReservedField);
    }

    // §N.4.1.4 — TRP (10 bits), present iff TRPI.
    let trp = if trpi {
        let v = reader
            .read_u32(NEWPRED_TRP_BITS)
            .map_err(|_| Error::UnexpectedEof)? as u16;
        field_bits += NEWPRED_TRP_BITS;
        Some(v)
    } else {
        None
    };

    // §N.4.1.5 — BCI (1 or 2 bits). "1" signals a following §N.4.1.6
    // BCM; "01" signals its absence (the forward-channel default). The
    // BCM is a decoder → encoder message with no forward-channel pixel
    // effect, so a present one is refused rather than parsed.
    let first = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    field_bits += 1;
    if first {
        return Err(Error::BadBackChannelMessage);
    }
    let second = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    field_bits += 1;
    if !second {
        // "00" is not a defined BCI codeword (§N.4.1.5).
        return Err(Error::BadBackChannelMessage);
    }

    Ok(GobNewpredFields {
        tr,
        trpi,
        trp,
        field_bits,
    })
}

/// A single stored reference picture: its decoded samples plus the
/// 10-bit Temporal Reference under which it was coded.
#[derive(Debug, Clone, PartialEq, Eq)]
struct StoredPicture {
    /// §N.4.1.4 — the 10-bit Temporal Reference (ETR concatenated with
    /// TR for a custom picture clock frequency, else `TR` with the two
    /// MSBs zero) under which this picture was transmitted.
    tr: u16,
    /// The decoded picture samples.
    frame: YuvFrame,
}

/// §N.5 — the decoder's reference picture memory for the Reference
/// Picture Selection mode.
///
/// The store keeps the most recently inserted pictures up to a capacity
/// bound (the "additional number of picture memories" of §N.5, which is
/// negotiated externally; the default here keeps a small history). B-
/// pictures are never inserted (§N.5: "except for B-pictures, which are
/// not used as reference pictures"). Insertion is first-in, first-out:
/// once the capacity is reached the oldest stored picture is evicted.
#[derive(Debug, Clone)]
pub struct RpsReferenceStore {
    pictures: Vec<StoredPicture>,
    capacity: usize,
}

impl RpsReferenceStore {
    /// Default number of additional picture memories. §N.5 leaves the
    /// exact count to external negotiation; this is a conservative
    /// in-memory default sufficient for typical NEWPRED histories.
    pub const DEFAULT_CAPACITY: usize = 16;

    /// Create an empty store with [`Self::DEFAULT_CAPACITY`] memories.
    pub fn new() -> Self {
        RpsReferenceStore {
            pictures: Vec::new(),
            capacity: Self::DEFAULT_CAPACITY,
        }
    }

    /// Create an empty store holding up to `capacity` reference
    /// pictures. A `capacity` of zero is treated as one (a decoder must
    /// retain at least the most recent anchor).
    pub fn with_capacity(capacity: usize) -> Self {
        RpsReferenceStore {
            pictures: Vec::new(),
            capacity: capacity.max(1),
        }
    }

    /// Number of currently-stored reference pictures.
    pub fn len(&self) -> usize {
        self.pictures.len()
    }

    /// Whether the store holds no reference pictures.
    pub fn is_empty(&self) -> bool {
        self.pictures.is_empty()
    }

    /// §N.5 — insert a correctly-decoded anchor picture under its 10-bit
    /// Temporal Reference. A picture with a `tr` already present replaces
    /// the stored entry (a retransmission of the same temporal instant);
    /// otherwise it is appended and the oldest entry evicted if the
    /// capacity is exceeded (first-in, first-out).
    ///
    /// B-pictures must not be stored (§N.5); the caller is responsible
    /// for only inserting anchor (I / P / EI / EP) pictures.
    pub fn insert(&mut self, tr: u16, frame: YuvFrame) {
        if let Some(slot) = self.pictures.iter_mut().find(|p| p.tr == tr) {
            slot.frame = frame;
            return;
        }
        self.pictures.push(StoredPicture { tr, frame });
        while self.pictures.len() > self.capacity {
            self.pictures.remove(0);
        }
    }

    /// §N.4.1.4 / §N.5 — select the prediction reference for a picture
    /// whose header carried `trpi` / `trp`.
    ///
    /// * When `trpi == Some(true)` and `trp` is present, the stored
    ///   picture whose TR equals `trp` is returned. If no such picture
    ///   is in memory, `None` is returned — §N.5: "When the picture for
    ///   which the TR is TRP is not available at the decoder, the
    ///   decoder may send a forced INTRA update signal"; here the caller
    ///   surfaces the absence as a decode error.
    /// * Otherwise (TRP absent or `trpi != Some(true)`) "the most recent
    ///   temporally previous anchor picture shall be used for
    ///   prediction, as when not in the Reference Picture Selection
    ///   mode" — the most recently inserted stored picture.
    pub fn select_reference(&self, trpi: Option<bool>, trp: Option<u16>) -> Option<&YuvFrame> {
        match (trpi, trp) {
            (Some(true), Some(target_tr)) => self
                .pictures
                .iter()
                .find(|p| p.tr == target_tr)
                .map(|p| &p.frame),
            _ => self.most_recent(),
        }
    }

    /// The most recently inserted reference picture, or `None` when the
    /// store is empty.
    pub fn most_recent(&self) -> Option<&YuvFrame> {
        self.pictures.last().map(|p| &p.frame)
    }

    /// Whether a stored picture exists for the given Temporal Reference.
    pub fn contains_tr(&self, tr: u16) -> bool {
        self.pictures.iter().any(|p| p.tr == tr)
    }
}

impl Default for RpsReferenceStore {
    fn default() -> Self {
        Self::new()
    }
}

/// §N.4.1.4 — compose the 10-bit Temporal Reference from the picture
/// header's `TR` (8 bits) and, when a custom picture clock frequency was
/// in use, the `ETR` (2 MSBs). Without a custom PCF the two MSBs are
/// zero.
pub fn compose_tr(tr: u8, etr: Option<u8>) -> u16 {
    let lsb = u16::from(tr);
    match etr {
        Some(e) => (u16::from(e & 0b11) << 8) | lsb,
        None => lsb,
    }
}

/// §N.4.2.1 — the Back-channel message Type (BT) field's two defined
/// code points. The `"00"` / `"01"` patterns are reserved and rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BcmType {
    /// `"10"` — NACK: the corresponding forward-channel data decoded
    /// erroneously. Carries the §N.4.2.11 RTR field.
    Nack,
    /// `"11"` — ACK: the corresponding forward-channel data decoded
    /// correctly.
    Ack,
}

/// Externally-negotiated framing knowledge for a §N.4.2 Back-Channel
/// Message: whether the videomux submode is in use (the §N.4.2.8 /
/// §N.4.2.10 BEPB emulation-prevention bits are present only then) and
/// the §N.4.2.9 GN/MBA field width — the GN width (5 bits, §5.2.3)
/// when the *forward* bitstream the message applies to uses the GOB
/// layer, or the Table-K.2 MBA width when it is Slice Structured
/// (per the §N.4.2.9 NOTE, this is a property of the applied-to
/// bitstream, not of the channel transporting the BCM).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BcmContext {
    /// Videomux submode: BEPB1 / BEPB2 present.
    pub videomux: bool,
    /// Width in bits of the GN/MBA field (5 for the GOB layer;
    /// 5/6/7/9/11/12/13/14 per Table K.2 for Slice Structured).
    pub gn_mba_bits: u32,
}

/// One parsed §N.4.2 Back-Channel Message (Figure N.4), minus the
/// §N.4.2.12 BSTUF tail (external-framing concern left to the caller).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BackChannelMessage {
    /// §N.4.2.1 BT — ACK or NACK.
    pub message_type: BcmType,
    /// §N.4.2.2 URF — `true` when no reliable TR / GN/MBA was
    /// available to the reporting decoder.
    pub unreliable: bool,
    /// §N.4.2.3 TR — 10-bit temporal reference of the video picture
    /// segment the message refers to (ETR ∥ TR under a custom picture
    /// clock frequency; two zero MSBs otherwise).
    pub tr: u16,
    /// §N.4.2.4 / §N.4.2.5 — `Some(layer)` iff ELNUMI = "1" (the
    /// message refers to an Annex O enhancement layer).
    pub elnum: Option<u8>,
    /// §N.4.2.6 / §N.4.2.7 — `Some(sub_bitstream)` iff BCPM = "1"
    /// (CPM in use on the forward channel).
    pub bsbi: Option<u8>,
    /// §N.4.2.9 — GOB number (GOB layer) or macroblock address
    /// (Slice Structured) of the segment's first macroblock.
    pub gn_mba: u32,
    /// §N.4.2.11 RTR — requested temporal reference; present iff
    /// [`BcmType::Nack`].
    pub rtr: Option<u16>,
}

/// Parse one §N.4.2 Back-Channel Message at the reader's position.
///
/// The §N.4.2.12 BSTUF zero-stuffing (present only when the separate
/// logical channel mode ends an external frame here) is not consumed —
/// framing is the caller's concern.
///
/// # Errors
///
/// * [`Error::BadBackChannelMessage`] — a reserved BT code point
///   (`"00"` / `"01"`), a zero BEPB1 / BEPB2 in videomux mode, or a
///   NACK whose RTR is truncated.
/// * [`Error::UnexpectedEof`] — the buffer ended mid-message.
pub fn parse_bcm(reader: &mut BitReader<'_>, ctx: BcmContext) -> Result<BackChannelMessage> {
    // §N.4.2.1 — BT.
    let bt = reader.read_u32(2).map_err(|_| Error::UnexpectedEof)?;
    let message_type = match bt {
        0b10 => BcmType::Nack,
        0b11 => BcmType::Ack,
        _ => return Err(Error::BadBackChannelMessage),
    };
    // §N.4.2.2 — URF.
    let unreliable = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    // §N.4.2.3 — TR (10 bits).
    let tr = reader.read_u32(10).map_err(|_| Error::UnexpectedEof)? as u16;
    // §N.4.2.4 / §N.4.2.5 — ELNUMI + ELNUM.
    let elnumi = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let elnum = if elnumi {
        Some(reader.read_u32(4).map_err(|_| Error::UnexpectedEof)? as u8)
    } else {
        None
    };
    // §N.4.2.6 / §N.4.2.7 — BCPM + BSBI.
    let bcpm = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    let bsbi = if bcpm {
        Some(reader.read_u32(2).map_err(|_| Error::UnexpectedEof)? as u8)
    } else {
        None
    };
    // §N.4.2.8 — BEPB1 (videomux only, always "1").
    if ctx.videomux && !reader.read_bit().map_err(|_| Error::UnexpectedEof)? {
        return Err(Error::BadBackChannelMessage);
    }
    // §N.4.2.9 — GN / MBA.
    let gn_mba = reader
        .read_u32(ctx.gn_mba_bits)
        .map_err(|_| Error::UnexpectedEof)?;
    // §N.4.2.10 — BEPB2 (videomux only, always "1").
    if ctx.videomux && !reader.read_bit().map_err(|_| Error::UnexpectedEof)? {
        return Err(Error::BadBackChannelMessage);
    }
    // §N.4.2.11 — RTR, present iff NACK.
    let rtr = if matches!(message_type, BcmType::Nack) {
        Some(reader.read_u32(10).map_err(|_| Error::UnexpectedEof)? as u16)
    } else {
        None
    };
    Ok(BackChannelMessage {
        message_type,
        unreliable,
        tr,
        elnum,
        bsbi,
        gn_mba,
        rtr,
    })
}

/// Write one §N.4.2 Back-Channel Message — the exact inverse of
/// [`parse_bcm`] (BSTUF excluded; the caller appends any
/// external-frame zero-stuffing).
///
/// # Errors
///
/// [`Error::BadBackChannelMessage`] — field/value mismatches: an
/// out-of-range ELNUM / BSBI / TR / RTR / GN-MBA for the context's
/// widths, or an RTR present on an ACK (RTR is a NACK-only field).
pub fn write_bcm(w: &mut BitWriter, ctx: BcmContext, msg: &BackChannelMessage) -> Result<()> {
    if msg.tr > 0x3FF {
        return Err(Error::BadBackChannelMessage);
    }
    if msg.elnum.is_some_and(|e| e > 0xF) || msg.bsbi.is_some_and(|b| b > 0b11) {
        return Err(Error::BadBackChannelMessage);
    }
    if ctx.gn_mba_bits < 32 && msg.gn_mba >= (1u32 << ctx.gn_mba_bits) {
        return Err(Error::BadBackChannelMessage);
    }
    match msg.message_type {
        BcmType::Nack => {
            let rtr = msg.rtr.ok_or(Error::BadBackChannelMessage)?;
            if rtr > 0x3FF {
                return Err(Error::BadBackChannelMessage);
            }
        }
        BcmType::Ack => {
            if msg.rtr.is_some() {
                return Err(Error::BadBackChannelMessage);
            }
        }
    }

    // §N.4.2.1 — BT.
    w.write_bits(
        match msg.message_type {
            BcmType::Nack => 0b10,
            BcmType::Ack => 0b11,
        },
        2,
    );
    // §N.4.2.2 — URF.
    w.write_bit(msg.unreliable);
    // §N.4.2.3 — TR.
    w.write_bits(msg.tr as u32, 10);
    // §N.4.2.4 / §N.4.2.5 — ELNUMI + ELNUM.
    match msg.elnum {
        Some(layer) => {
            w.write_bit(true);
            w.write_bits(layer as u32, 4);
        }
        None => w.write_bit(false),
    }
    // §N.4.2.6 / §N.4.2.7 — BCPM + BSBI.
    match msg.bsbi {
        Some(sub) => {
            w.write_bit(true);
            w.write_bits(sub as u32, 2);
        }
        None => w.write_bit(false),
    }
    // §N.4.2.8 — BEPB1.
    if ctx.videomux {
        w.write_bit(true);
    }
    // §N.4.2.9 — GN / MBA.
    w.write_bits(msg.gn_mba, ctx.gn_mba_bits);
    // §N.4.2.10 — BEPB2.
    if ctx.videomux {
        w.write_bit(true);
    }
    // §N.4.2.11 — RTR.
    if let Some(rtr) = msg.rtr {
        w.write_bits(rtr as u32, 10);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// §N.4.1 — write a NEWPRED field group (TRI/TR/TRPI/TRP + BCI = "01")
    /// onto a fresh writer and return the bytes plus the count of bits
    /// written, so the parser's `field_bits` can be cross-checked.
    fn write_newpred(
        tr: Option<u16>,
        trp: Option<u16>,
        custom_pcf: bool,
        bci: &[bool],
    ) -> (Vec<u8>, u32) {
        let mut w = BitWriter::new();
        let mut bits = 0u32;
        w.write_bit(tr.is_some()); // TRI
        bits += 1;
        if let Some(t) = tr {
            let width = if custom_pcf { 10 } else { 8 };
            w.write_u32(u32::from(t), width);
            bits += width;
        }
        w.write_bit(trp.is_some()); // TRPI
        bits += 1;
        if let Some(p) = trp {
            w.write_u32(u32::from(p), NEWPRED_TRP_BITS);
            bits += NEWPRED_TRP_BITS;
        }
        for &b in bci {
            w.write_bit(b);
            bits += 1;
        }
        // Trailing sentinel bits so the reader never runs dry on a
        // legal field group.
        w.write_u32(0, 8);
        (w.finish(), bits)
    }

    #[test]
    fn newpred_no_tr_no_trp_consumes_three_bits() {
        // TRI = 0, TRPI = 0, BCI = "01" → 1 + 1 + 2 = 4 bits.
        let (bytes, bits) = write_newpred(None, None, false, &[false, true]);
        let mut r = BitReader::new(&bytes);
        let f = parse_gob_newpred_fields(&mut r, false, false).unwrap();
        assert_eq!(f.tr, None);
        assert!(!f.trpi);
        assert_eq!(f.trp, None);
        assert_eq!(f.field_bits, bits);
        assert_eq!(f.field_bits, 4);
        assert_eq!(f.segment_trp(), None);
        assert_eq!(r.bit_position(), u64::from(bits));
    }

    #[test]
    fn newpred_tr_present_8bit_without_custom_pcf() {
        let (bytes, bits) = write_newpred(Some(42), None, false, &[false, true]);
        let mut r = BitReader::new(&bytes);
        let f = parse_gob_newpred_fields(&mut r, false, false).unwrap();
        assert_eq!(f.tr, Some(42));
        assert_eq!(f.field_bits, bits); // 1 + 8 + 1 + 2 = 12
        assert_eq!(f.field_bits, 12);
    }

    #[test]
    fn newpred_tr_present_10bit_with_custom_pcf() {
        // A 10-bit TR value only representable with the custom-PCF width.
        let (bytes, bits) = write_newpred(Some(0x2A5), None, true, &[false, true]);
        let mut r = BitReader::new(&bytes);
        let f = parse_gob_newpred_fields(&mut r, true, false).unwrap();
        assert_eq!(f.tr, Some(0x2A5));
        assert_eq!(f.field_bits, bits); // 1 + 10 + 1 + 2 = 14
        assert_eq!(f.field_bits, 14);
    }

    #[test]
    fn newpred_trp_present_selects_segment_reference() {
        // TRI = 0, TRPI = 1, TRP = 17, BCI = "01".
        let (bytes, bits) = write_newpred(None, Some(17), false, &[false, true]);
        let mut r = BitReader::new(&bytes);
        let f = parse_gob_newpred_fields(&mut r, false, false).unwrap();
        assert!(f.trpi);
        assert_eq!(f.trp, Some(17));
        assert_eq!(f.segment_trp(), Some(17));
        assert_eq!(f.field_bits, bits); // 1 + 1 + 10 + 2 = 14
        assert_eq!(f.field_bits, 14);
    }

    #[test]
    fn newpred_trpi_on_intra_is_rejected() {
        // §N.4.1.3 — TRPI must be 0 for an I/EI picture.
        let (bytes, _) = write_newpred(None, Some(5), false, &[false, true]);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_gob_newpred_fields(&mut r, false, true),
            Err(Error::PlusPtypeReservedField)
        );
    }

    #[test]
    fn newpred_bci_one_refuses_back_channel_message() {
        // BCI = "1" signals a present BCM (§N.4.1.6) — refused.
        let (bytes, _) = write_newpred(None, None, false, &[true]);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_gob_newpred_fields(&mut r, false, false),
            Err(Error::BadBackChannelMessage)
        );
    }

    #[test]
    fn newpred_bci_double_zero_is_rejected() {
        // BCI = "00" is not a defined codeword (§N.4.1.5).
        let (bytes, _) = write_newpred(None, None, false, &[false, false]);
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_gob_newpred_fields(&mut r, false, false),
            Err(Error::BadBackChannelMessage)
        );
    }

    #[test]
    fn newpred_truncated_buffer_returns_eof() {
        // Only the TRI bit present, then the buffer ends.
        let bytes = [0b1000_0000u8]; // TRI = 1, then nothing usable for an 8-bit TR
        let mut r = BitReader::new(&bytes[..1]);
        // TRI=1 wants an 8-bit TR but only 7 bits remain → EOF.
        assert_eq!(
            parse_gob_newpred_fields(&mut r, false, false),
            Err(Error::UnexpectedEof)
        );
    }

    fn flat(value: u8) -> YuvFrame {
        let mut f = YuvFrame::grey(16, 16);
        f.y.iter_mut().for_each(|p| *p = value);
        f
    }

    #[test]
    fn compose_tr_without_etr() {
        assert_eq!(compose_tr(200, None), 200);
    }

    #[test]
    fn compose_tr_with_etr() {
        // ETR = 0b10 (only 2 LSBs used), TR = 5 => 0b10_00000101 = 517.
        assert_eq!(compose_tr(5, Some(0b10)), 0b10_0000_0101);
    }

    #[test]
    fn trp_selects_stored_picture_by_tr() {
        let mut store = RpsReferenceStore::new();
        store.insert(10, flat(10));
        store.insert(20, flat(20));
        store.insert(30, flat(30));

        // TRPI = 1, TRP = 20 selects the TR=20 picture, not the latest.
        let r = store
            .select_reference(Some(true), Some(20))
            .expect("TR=20 picture must be present");
        assert_eq!(r.y[0], 20);
    }

    #[test]
    fn trp_absent_selects_most_recent() {
        let mut store = RpsReferenceStore::new();
        store.insert(10, flat(10));
        store.insert(20, flat(20));
        // TRPI = 0 => most recent anchor (TR=20).
        let r = store
            .select_reference(Some(false), None)
            .expect("most-recent picture must be present");
        assert_eq!(r.y[0], 20);
    }

    #[test]
    fn missing_trp_yields_none() {
        let mut store = RpsReferenceStore::new();
        store.insert(10, flat(10));
        // TRP = 99 not in the store => None (forced-INTRA-update case).
        assert!(store.select_reference(Some(true), Some(99)).is_none());
    }

    #[test]
    fn fifo_eviction_at_capacity() {
        let mut store = RpsReferenceStore::with_capacity(2);
        store.insert(1, flat(1));
        store.insert(2, flat(2));
        store.insert(3, flat(3)); // evicts TR=1
        assert_eq!(store.len(), 2);
        assert!(!store.contains_tr(1));
        assert!(store.contains_tr(2));
        assert!(store.contains_tr(3));
    }

    #[test]
    fn reinsert_same_tr_replaces() {
        let mut store = RpsReferenceStore::new();
        store.insert(5, flat(50));
        store.insert(5, flat(60));
        assert_eq!(store.len(), 1);
        assert_eq!(store.most_recent().unwrap().y[0], 60);
    }

    #[test]
    fn empty_store_selects_none() {
        let store = RpsReferenceStore::new();
        assert!(store.select_reference(Some(false), None).is_none());
        assert!(store.most_recent().is_none());
    }
    fn bcm_ctx(videomux: bool, bits: u32) -> BcmContext {
        BcmContext {
            videomux,
            gn_mba_bits: bits,
        }
    }

    /// Every field-presence combination of the §N.4.2 BCM round-trips
    /// bit-exactly through `write_bcm` / `parse_bcm`.
    #[test]
    fn bcm_round_trips_field_matrix() {
        let messages = [
            BackChannelMessage {
                message_type: BcmType::Ack,
                unreliable: false,
                tr: 5,
                elnum: None,
                bsbi: None,
                gn_mba: 3,
                rtr: None,
            },
            BackChannelMessage {
                message_type: BcmType::Nack,
                unreliable: true,
                tr: 0x3FF,
                elnum: Some(0xF),
                bsbi: Some(0b11),
                gn_mba: 17,
                rtr: Some(0x2AB),
            },
            BackChannelMessage {
                message_type: BcmType::Nack,
                unreliable: false,
                tr: 0,
                elnum: None,
                bsbi: Some(1),
                gn_mba: 0,
                rtr: Some(0),
            },
        ];
        for videomux in [false, true] {
            for bits in [5u32, 7, 9] {
                let ctx = bcm_ctx(videomux, bits);
                for msg in &messages {
                    let mut w = BitWriter::new();
                    write_bcm(&mut w, ctx, msg).expect("write");
                    w.write_bits(0b1010, 4); // sentinel
                    let bytes = w.finish();
                    let mut r = BitReader::new(&bytes);
                    let got = parse_bcm(&mut r, ctx).expect("parse");
                    assert_eq!(&got, msg, "videomux={videomux} bits={bits}");
                    assert_eq!(r.read_u32(4).unwrap(), 0b1010, "position");
                }
            }
        }
    }

    /// Reserved BT code points and zero emulation-prevention bits are
    /// refused; an ACK with an RTR (or a NACK without one) is refused
    /// on the write side.
    #[test]
    fn bcm_rejects_malformed_shapes() {
        // BT = "00" (reserved).
        let mut w = BitWriter::new();
        w.write_bits(0b00, 2);
        w.write_bits(0, 20);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_bcm(&mut r, bcm_ctx(false, 5)).unwrap_err(),
            Error::BadBackChannelMessage
        );

        // Videomux BEPB1 = 0: BT=11 URF=0 TR(10)=0 ELNUMI=0 BCPM=0
        // then BEPB1=0.
        let mut w = BitWriter::new();
        w.write_bits(0b11, 2);
        w.write_bit(false);
        w.write_bits(0, 10);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false); // BEPB1 = 0
        w.write_bits(0, 8);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            parse_bcm(&mut r, bcm_ctx(true, 5)).unwrap_err(),
            Error::BadBackChannelMessage
        );

        // ACK carrying an RTR.
        let mut w = BitWriter::new();
        let bad_ack = BackChannelMessage {
            message_type: BcmType::Ack,
            unreliable: false,
            tr: 1,
            elnum: None,
            bsbi: None,
            gn_mba: 1,
            rtr: Some(2),
        };
        assert_eq!(
            write_bcm(&mut w, bcm_ctx(false, 5), &bad_ack).unwrap_err(),
            Error::BadBackChannelMessage
        );
        // NACK without an RTR.
        let bad_nack = BackChannelMessage {
            message_type: BcmType::Nack,
            rtr: None,
            ..bad_ack
        };
        assert_eq!(
            write_bcm(&mut w, bcm_ctx(false, 5), &bad_nack).unwrap_err(),
            Error::BadBackChannelMessage
        );
        // GN out of range for a 5-bit field.
        let bad_gn = BackChannelMessage {
            message_type: BcmType::Ack,
            gn_mba: 32,
            rtr: None,
            ..bad_ack
        };
        assert_eq!(
            write_bcm(&mut w, bcm_ctx(false, 5), &bad_gn).unwrap_err(),
            Error::BadBackChannelMessage
        );
    }
}
