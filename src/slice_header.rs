//! H.263 Annex K — Slice Structured mode slice-layer header parsing.
//!
//! This module implements the structural decode of the slice-layer
//! header as defined in ITU-T Recommendation H.263 (01/2005) §K.2 and
//! Figure K.1/H.263:
//!
//! ```text
//!   SSTUF | SSC | SEPB1 | SSBI | MBA | SEPB2 | SQUANT | SWI | SEPB3
//!         | GFID | Macroblock Data
//! ```
//!
//! Round 15 (post orphan-rebuild) lands the slice-layer header for
//! both the general case (slices after the picture's first one) and
//! the §K.2 "first slice after the picture start code" reduced form.
//! The macroblock data that follows is out of scope for this module —
//! the macroblock parser already handles those bytes once the caller
//! has consumed the header.
//!
//! Covered:
//!
//! * §K.2.2 — **SSC** (Slice Start Code), 17 bits, value
//!   `0000 0000 0000 0000 1`. Numerically identical to §5.2.2 GBSC;
//!   the disambiguation between GOB and slice headers is by mode
//!   (PLUSPTYPE SS = "1"), not bitstream-level.
//! * §K.2.3 — **SEPB1** (Slice Emulation Prevention Bit 1), 1 bit,
//!   always `"1"`.
//! * §K.2.4 — **SSBI** (Slice Sub-Bitstream Indicator), 4 bits, present
//!   only when CPM = "1" in the picture header. Table K.1 restricts
//!   the four legal codewords to `1001` / `1010` / `1011` / `1101`.
//! * §K.2.5 — **MBA** (Macroblock Address), variable-length per
//!   Table K.2. Field width depends on picture format and whether
//!   Reduced-Resolution Update is active.
//! * §K.2.6 — **SEPB2** (Slice Emulation Prevention Bit 2), 1 bit
//!   conditionally present (see [`SliceHeaderContext::sepb2_present`]).
//! * §K.2.7 — **SQUANT** (Quantizer Information), 5 bits, natural
//!   binary `QUANT ∈ 1..=31`.
//! * §K.2.8 — **SWI** (Slice Width Indication in Macroblocks), variable
//!   per Table K.3, present only in the Rectangular Slice submode.
//! * §K.2.9 — **SEPB3** (Slice Emulation Prevention Bit 3), 1 bit,
//!   always `"1"`.
//! * §5.2.5 / §K.2 — **GFID** (GOB Frame ID), 2 bits.
//!
//! ## §K.2 first-slice reduced form
//!
//! For the slice that immediately follows the Picture Start Code, the
//! spec specifies a reduced form: only the emulation-prevention bits
//! (SEPB1, SEPB3, conditionally SEPB2), the MBA field, and (if RS
//! submode) SWI are transmitted. SSC, SSBI, SQUANT, GFID are absent.
//! [`parse_first_slice_header`] is the entry point for that case;
//! [`parse_slice_layer`] handles every other slice in the picture.
//!
//! ## §K.2.1 SSTUF stuffing
//!
//! [`skip_sstuf`] discards the §K.2.1 stuffing zero-bits that an
//! encoder writes before SSC so the start code is byte aligned. The
//! caller invokes it on a reader whose position is somewhere inside
//! the byte that ends right before SSC; the function reads the
//! remaining `0..=7` bits of that byte, verifies they are all zero
//! (the spec mandates `0` as the stuffing bit per §K.2.1), and
//! returns the number of bits discarded. A reader already on a byte
//! boundary returns `Ok(0)` without consuming any bits, matching the
//! "may be zero" wording of the picture/slice/GOB stuffing fields.
//!
//! ## Deliberately deferred
//!
//! * Macroblock data (§5.3) — handled by the existing macroblock layer.
//! * Annex Q Reduced-Resolution Update — the table widths flip to a
//!   second column when RRU is in effect (Tables K.2 / K.3 second
//!   column). [`SliceHeaderContext::rru`] selects between them, but
//!   we do not yet validate any picture-level RRU flag; the caller
//!   chooses the column.

use oxideav_core::bits::BitReader;

use crate::picture::PictureLayout;
use crate::picture_header::H263SourceFormat;
use crate::plus_ptype::SliceStructuredSubmode;
use crate::{Error, Result};

/// Maximum number of bits §K.2.1 SSTUF may occupy.
///
/// The spec describes SSTUF as "a variable-length run of less than
/// 8 bits"; together with the byte-alignment constraint on the SSC
/// that follows, the legal SSTUF run lengths are `0..=7` bits.
pub const SSTUF_MAX_BITS: u32 = 7;

/// Length in bits of the Slice Start Code (§K.2.2).
pub const SSC_BITS: u32 = 17;

/// Value of the Slice Start Code: `0000 0000 0000 0000 1`.
///
/// As a 17-bit unsigned integer that is `0x00001` (decimal 1). The
/// bit pattern is identical to the GOB start code (§5.2.2); the two
/// layers are distinguished by the picture-level PLUSPTYPE SS flag,
/// not by the bits on the wire.
pub const SSC_VALUE: u32 = 0x0000_0001;

/// Length in bits of SEPB1 / SEPB2 / SEPB3 (§K.2.3 / §K.2.6 / §K.2.9).
pub const SEPB_BITS: u32 = 1;

/// Length in bits of SSBI (§K.2.4).
pub const SSBI_BITS: u32 = 4;

/// Length in bits of SQUANT (§K.2.7).
pub const SQUANT_BITS: u32 = 5;

/// Length in bits of GFID (§5.2.5).
pub const GFID_BITS: u32 = 2;

/// Width-of-MBA-field lookup per Table K.2 for the default
/// (non-Reduced-Resolution) case.
///
/// `(luma_width, luma_height, default_mba_width, default_mba_max,
/// rru_mba_width, rru_mba_max)`.
const MBA_TABLE: &[(u32, u32, u32, u32, u32, u32)] = &[
    // sub-QCIF (128 × 96): 8 × 6 = 48 MBs, max value 47.
    (128, 96, 6, 47, 5, 11),
    // QCIF (176 × 144): 11 × 9 = 99 MBs, max value 98.
    (176, 144, 7, 98, 6, 29),
    // CIF (352 × 288): 22 × 18 = 396 MBs, max value 395.
    (352, 288, 9, 395, 7, 98),
    // 4CIF (704 × 576): 44 × 36 = 1584 MBs, max value 1583.
    (704, 576, 11, 1583, 9, 395),
    // 16CIF (1408 × 1152): 88 × 72 = 6336 MBs, max value 6335.
    (1408, 1152, 13, 6335, 11, 1583),
    // 2048 × 1152: 128 × 72 = 9216 MBs, max value 9215.
    (2048, 1152, 14, 9215, 12, 2303),
];

/// Width-of-SWI-field lookup per Table K.3 for the default
/// (non-Reduced-Resolution) case.
///
/// `(luma_width, default_swi_width, default_swi_max, rru_swi_width,
/// rru_swi_max)`. `default_swi_max` is one less than the total number
/// of macroblocks across the picture (so SWI is the *index* of the
/// last MB column in the slice rather than the count).
const SWI_TABLE: &[(u32, u32, u32, u32, u32)] = &[
    // sub-QCIF: 8 MBs across, max SWI = 7.
    (128, 4, 7, 3, 3),
    // QCIF: 11 MBs across, max SWI = 10.
    (176, 4, 10, 3, 5),
    // CIF: 22 MBs across, max SWI = 21.
    (352, 5, 21, 4, 10),
    // 4CIF: 44 MBs across, max SWI = 43.
    (704, 6, 43, 5, 21),
    // 16CIF: 88 MBs across, max SWI = 87.
    (1408, 7, 87, 6, 43),
    // 1412..2048 pixels wide: max SWI = 127.
    (2048, 7, 127, 6, 63),
];

/// Picture-level context required to parse a slice-layer header.
///
/// The Annex K syntax has three conditional fields whose presence and
/// width depend on properties of the picture, not the slice header
/// itself:
///
/// * [`cpm`](Self::cpm) decides whether SSBI is on the wire and how
///   wide MBA must be before SEPB2 becomes mandatory.
/// * [`rectangular_slices`](Self::rectangular_slices) (PLUSPTYPE SSS
///   bit 1) decides whether SWI is on the wire.
/// * [`picture_width`](Self::picture_width) /
///   [`picture_height`](Self::picture_height) /
///   [`rru`](Self::rru) drive the §K.2.5 / §K.2.8 field-width and
///   value-range lookups (Tables K.2 / K.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SliceHeaderContext {
    /// Picture luma width in pixels. Used to look up the MBA / SWI
    /// field widths in Tables K.2 / K.3.
    pub picture_width: u32,
    /// Picture luma height in pixels. Used to look up the MBA value
    /// range in Table K.2 for custom sizes.
    pub picture_height: u32,
    /// `true` ⇔ Continuous-Presence-Multipoint mode (§Annex C) is in
    /// effect (picture-layer CPM = `"1"`). When set, SSBI is on the
    /// wire and SEPB2 becomes mandatory at a smaller MBA width.
    pub cpm: bool,
    /// `true` ⇔ Rectangular Slice submode is in effect (PLUSPTYPE
    /// SSS bit 1 = `"1"`). When set, SWI is on the wire and SEPB2
    /// is mandatory for the first slice (§K.2.6 last sentence).
    pub rectangular_slices: bool,
    /// `true` ⇔ Reduced-Resolution Update mode (§Annex Q) is in
    /// effect. Selects the right-hand columns of Tables K.2 / K.3.
    pub rru: bool,
}

impl SliceHeaderContext {
    /// Build a context for a standard source format with no optional
    /// modes — CPM off, RS off, RRU off. Useful for the most common
    /// QCIF / CIF / sub-QCIF baseline-plus-Annex-K case.
    pub fn for_standard_format(format: H263SourceFormat) -> Option<Self> {
        let (w, h) = format.luma_dimensions()?;
        Some(SliceHeaderContext {
            picture_width: w,
            picture_height: h,
            cpm: false,
            rectangular_slices: false,
            rru: false,
        })
    }

    /// Build a context from a [`PictureLayout`] + PLUSPTYPE-level
    /// optional-mode bits.
    ///
    /// The slice-layer §K.2 syntax depends only on the picture's
    /// **luma dimensions** plus the four orthogonal mode flags below;
    /// the GOB-grid (`num_gobs` / `mb_rows_per_gob`) carried by
    /// [`PictureLayout`] is irrelevant to the per-slice parse, but
    /// [`PictureLayout`] is the canonical luma-dimension carrier for
    /// both the fixed baseline source formats and the §4.2.1 / §5.1.5
    /// PLUSPTYPE custom-format path, so accepting it lets one
    /// constructor cover every wire-resolvable layout.
    ///
    /// * `sss` — §5.1.10 SSS submode bits (`None` ⇔ Slice-Structured
    ///   mode is off; the caller should not be parsing slice headers
    ///   at all in that case but the constructor still accepts it,
    ///   returning a `rectangular_slices = false` context).
    /// * `cpm` — picture-layer CPM (§5.1.20). Drives SSBI presence
    ///   (§K.2.4) and the SEPB2 threshold (§K.2.6).
    /// * `rru` — `true` iff the picture is in Reduced-Resolution
    ///   Update mode (§Annex Q); selects the right-hand columns of
    ///   Tables K.2 / K.3 for the MBA / SWI lookups.
    ///
    /// The `arbitrary_order` bit of [`SliceStructuredSubmode`] does
    /// not affect any §K.2 field width or value range — it only
    /// influences slice scheduling at the driver layer — so it is
    /// not captured in the returned context.
    pub fn from_picture_layout(
        layout: &PictureLayout,
        sss: Option<SliceStructuredSubmode>,
        cpm: bool,
        rru: bool,
    ) -> SliceHeaderContext {
        SliceHeaderContext {
            picture_width: layout.luma_width,
            picture_height: layout.luma_height,
            cpm,
            rectangular_slices: sss.map(|s| s.rectangular).unwrap_or(false),
            rru,
        }
    }

    /// §K.2.5 — width of the MBA field for this picture (bits).
    ///
    /// Returns `None` if [`picture_width`](Self::picture_width) is
    /// smaller than sub-QCIF (the smallest format Table K.2 covers).
    /// For custom widths, picks the first table entry whose maximum
    /// MB count is `>=` the picture's MB count, per §K.2.5: "For
    /// custom picture sizes, the field width is given by the first
    /// entry in the table that has an equal or larger number of
    /// macroblocks".
    pub fn mba_field_width(&self) -> Option<u32> {
        let mbs_per_pic = self.picture_width.div_ceil(16) * self.picture_height.div_ceil(16);
        for &(w, h, default_w, default_max, rru_w, rru_max) in MBA_TABLE {
            let table_mbs = w.div_ceil(16) * h.div_ceil(16);
            if mbs_per_pic <= table_mbs {
                return Some(if self.rru {
                    let _ = rru_max;
                    rru_w
                } else {
                    let _ = default_max;
                    default_w
                });
            }
        }
        None
    }

    /// §K.2.5 — the maximum-permissible MBA value for this picture.
    /// `(mb_count - 1)`, capped by the table entry width. Per §K.2.5,
    /// "the maximum value is the number of macroblocks in the current
    /// picture minus one".
    pub fn mba_max_value(&self) -> Option<u32> {
        let mbs_per_pic = self.picture_width.div_ceil(16) * self.picture_height.div_ceil(16);
        if mbs_per_pic == 0 {
            return None;
        }
        Some(mbs_per_pic - 1)
    }

    /// §K.2.8 — width of the SWI field (bits), or `None` if Rectangular
    /// Slice submode is not in effect (SWI is then absent from the
    /// wire).
    pub fn swi_field_width(&self) -> Option<u32> {
        if !self.rectangular_slices {
            return None;
        }
        let mbs_per_row = self.picture_width.div_ceil(16);
        for &(w, default_w, default_max, rru_w, rru_max) in SWI_TABLE {
            let table_mbs_per_row = w.div_ceil(16);
            if mbs_per_row <= table_mbs_per_row {
                return Some(if self.rru {
                    let _ = rru_max;
                    rru_w
                } else {
                    let _ = default_max;
                    default_w
                });
            }
        }
        None
    }

    /// §K.2.6 — `true` iff SEPB2 must be transmitted between MBA and
    /// SQUANT in a *non-first* slice header for this picture.
    ///
    /// Per the spec: "SEPB2 is included only if the MBA field width is
    /// greater than 11 bits and CPM = '0' in the picture header, or
    /// if the MBA field width is greater than 9 bits and CPM = '1' in
    /// the picture header." Returns `false` when MBA width cannot be
    /// determined.
    pub fn sepb2_present(&self) -> bool {
        match self.mba_field_width() {
            Some(w) if !self.cpm && w > 11 => true,
            Some(w) if self.cpm && w > 9 => true,
            _ => false,
        }
    }
}

/// Parsed H.263 slice-layer header (general / non-first-slice case).
///
/// Field naming mirrors the spec. The header consumes a deterministic
/// number of bits given the [`SliceHeaderContext`]; that count is
/// returned in [`Self::header_bits`] so the caller can advance a
/// cursor without re-deriving the layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SliceLayer {
    /// §K.2.4 — Sub-Bitstream Indicator, `Some(value)` iff CPM = "1".
    pub ssbi: Option<u8>,
    /// §K.2.5 — Macroblock Address (first macroblock in the slice).
    pub mba: u32,
    /// §K.2.7 — Quantizer step parameter QUANT (range `1..=31`).
    pub squant: u8,
    /// §K.2.8 — Slice Width Indication. `Some(actual_width_in_mb)`
    /// (`= SWI + 1` per §K.2.8) iff Rectangular Slice submode is on.
    pub swi_actual_width: Option<u32>,
    /// §5.2.5 — GOB Frame ID.
    pub gfid: u8,
    /// Total number of bits consumed for this header
    /// (`SSC + SEPB1 + (SSBI?) + MBA + (SEPB2?) + SQUANT +
    /// (SWI?) + SEPB3 + GFID`).
    pub header_bits: u32,
}

/// Parsed reduced-form slice-layer header for the first slice after
/// the Picture Start Code (§K.2): only SEPB1, MBA, conditionally
/// SEPB2, conditionally SWI, and SEPB3 are present.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirstSliceLayer {
    /// §K.2.5 — Macroblock Address.
    pub mba: u32,
    /// §K.2.8 — Slice Width Indication, `Some(actual_width_in_mb)`
    /// iff Rectangular Slice submode is on.
    pub swi_actual_width: Option<u32>,
    /// Total bits consumed
    /// (`SEPB1 + MBA + (SEPB2?) + (SWI?) + SEPB3`).
    pub header_bits: u32,
}

/// Discard the §K.2.1 SSTUF stuffing that may precede an SSC.
///
/// SSTUF is "a codeword of variable length consisting of less than
/// 8 bits" whose last bit "shall be the last (least significant) bit
/// of a byte, so that the start of the SSC codeword is byte aligned"
/// (§K.2.1). The spec further specifies that `0` is the stuffing bit
/// value. When the reader is already on a byte boundary no SSTUF is
/// present and the function returns `Ok(0)` without consuming any
/// bits; otherwise the `1..=7` bits remaining in the current byte are
/// consumed, verified to be all zero, and the number of bits
/// discarded is returned. On success the reader is positioned at the
/// first (most significant) bit of the byte that holds the SSC
/// codeword.
///
/// Callers chain this in front of [`parse_slice_layer`]:
///
/// ```ignore
/// let bits_skipped = skip_sstuf(&mut reader)?;
/// let layer = parse_slice_layer(&mut reader, &ctx)?;
/// ```
///
/// The §K.2.2 sentence "The slice start code is not present for the
/// slice which follows the picture start code" means
/// [`parse_first_slice_header`] does **not** read SSTUF — the picture
/// header's own §5.1.28 PSTUF already aligned that boundary and the
/// first slice's SEPB1 is positioned at the first bit immediately
/// after the picture header.
///
/// ### Errors
///
/// * [`Error::UnexpectedEof`] — the buffer ended before the trailing
///   zero bits of the current byte could be read.
/// * [`Error::BadSliceStuffing`] — one of the stuffing bits was `1`
///   where §K.2.1 mandates `0`.
pub fn skip_sstuf(reader: &mut BitReader<'_>) -> Result<u32> {
    let pos = (reader.bit_position() % 8) as u32;
    if pos == 0 {
        return Ok(0);
    }
    let n = 8 - pos;
    debug_assert!((1..=SSTUF_MAX_BITS).contains(&n));
    let raw = reader.read_u32(n).map_err(|_| Error::UnexpectedEof)?;
    if raw != 0 {
        return Err(Error::BadSliceStuffing);
    }
    Ok(n)
}

/// Convenience wrapper that constructs a [`BitReader`] over `data`,
/// discards any §K.2.1 SSTUF stuffing starting at byte offset
/// `byte_offset` and bit offset `bit_offset` within that byte, and
/// returns `(bits_skipped, total_bits_consumed)`. The latter is the
/// reader's [`BitReader::bit_position`] after the discard and is the
/// canonical offset from which the caller should drive
/// [`parse_slice_layer`] over the same backing slice.
///
/// `bit_offset` must be in `0..=7`; offsets `>= 8` are folded back via
/// `% 8` after a corresponding byte-offset bump, matching the
/// convention used by callers that carry a `(byte, bit_in_byte)`
/// cursor over a longer bitstream.
///
/// ### Errors
///
/// Identical to [`skip_sstuf`].
pub fn skip_sstuf_at(data: &[u8], byte_offset: usize, bit_offset: u32) -> Result<(u32, u64)> {
    let extra_bytes = (bit_offset / 8) as usize;
    let bit_in_byte = bit_offset % 8;
    let start_byte = byte_offset
        .checked_add(extra_bytes)
        .ok_or(Error::UnexpectedEof)?;
    if start_byte > data.len() {
        return Err(Error::UnexpectedEof);
    }
    let mut reader = BitReader::with_position(data, start_byte);
    if bit_in_byte != 0 {
        reader
            .read_u32(bit_in_byte)
            .map_err(|_| Error::UnexpectedEof)?;
    }
    let skipped = skip_sstuf(&mut reader)?;
    Ok((skipped, reader.bit_position()))
}

/// Parse a non-first H.263 slice-layer header (§K.2) starting at the
/// current reader position.
///
/// The caller is responsible for placing the reader at the first bit
/// of SSC — i.e. for skipping any leading SSTUF via [`skip_sstuf`].
/// On success the reader is left positioned immediately after GFID,
/// at the first bit of the slice's macroblock data.
///
/// On error the reader's position is unspecified.
///
/// ### Errors
///
/// * [`Error::UnexpectedEof`] — the buffer ended before the header
///   could be read in full.
/// * [`Error::BadSliceStartCode`] — the leading 17 bits were not
///   equal to [`SSC_VALUE`].
/// * [`Error::BadSliceEmulationPreventionBit`] — one of SEPB1 /
///   SEPB2 / SEPB3 was `0`; §K.2.3 / §K.2.6 / §K.2.9 require `1`.
/// * [`Error::BadSliceSsbiCode`] — SSBI was not one of the four
///   Table K.1 codewords (`1001` / `1010` / `1011` / `1101`).
/// * [`Error::SliceMbaOutOfRange`] — MBA exceeded
///   [`SliceHeaderContext::mba_max_value`].
/// * [`Error::InvalidQuantiser`] — SQUANT was `0`.
/// * [`Error::SliceSwiOutOfRange`] — SWI raw value gave a slice width
///   exceeding the picture's MB-per-row count.
/// * [`Error::UnsupportedPictureGeometry`] — the
///   [`SliceHeaderContext`] picture size is below sub-QCIF, so
///   [`SliceHeaderContext::mba_field_width`] cannot resolve.
pub fn parse_slice_layer(
    reader: &mut BitReader<'_>,
    ctx: &SliceHeaderContext,
) -> Result<SliceLayer> {
    let mba_width = ctx
        .mba_field_width()
        .ok_or(Error::UnsupportedPictureGeometry)?;
    let mba_max = ctx
        .mba_max_value()
        .ok_or(Error::UnsupportedPictureGeometry)?;

    // §K.2.2 — SSC (17 bits, value 0x00001).
    let ssc = reader
        .read_u32(SSC_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    if ssc != SSC_VALUE {
        return Err(Error::BadSliceStartCode);
    }

    // §K.2.3 — SEPB1, always "1".
    let sepb1 = reader
        .read_u32(SEPB_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    if sepb1 != 1 {
        return Err(Error::BadSliceEmulationPreventionBit);
    }

    // §K.2.4 — SSBI, present iff CPM = "1".
    let mut bits = SSC_BITS + SEPB_BITS;
    let ssbi = if ctx.cpm {
        let raw = reader
            .read_u32(SSBI_BITS)
            .map_err(|_| Error::UnexpectedEof)? as u8;
        if !is_legal_ssbi(raw) {
            return Err(Error::BadSliceSsbiCode);
        }
        bits += SSBI_BITS;
        Some(raw)
    } else {
        None
    };

    // §K.2.5 — MBA, variable bits per Table K.2.
    let mba = reader
        .read_u32(mba_width)
        .map_err(|_| Error::UnexpectedEof)?;
    if mba > mba_max {
        return Err(Error::SliceMbaOutOfRange);
    }
    bits += mba_width;

    // §K.2.6 — SEPB2, conditionally present.
    if ctx.sepb2_present() {
        let sepb2 = reader
            .read_u32(SEPB_BITS)
            .map_err(|_| Error::UnexpectedEof)?;
        if sepb2 != 1 {
            return Err(Error::BadSliceEmulationPreventionBit);
        }
        bits += SEPB_BITS;
    }

    // §K.2.7 — SQUANT (5 bits, 1..=31).
    let squant = reader
        .read_u32(SQUANT_BITS)
        .map_err(|_| Error::UnexpectedEof)? as u8;
    if squant == 0 {
        return Err(Error::InvalidQuantiser);
    }
    bits += SQUANT_BITS;

    // §K.2.8 — SWI, present iff Rectangular Slice submode is on.
    let swi_actual_width = if let Some(swi_width) = ctx.swi_field_width() {
        let raw = reader
            .read_u32(swi_width)
            .map_err(|_| Error::UnexpectedEof)?;
        let actual_width = raw + 1;
        let mbs_per_row = ctx.picture_width.div_ceil(16);
        if actual_width > mbs_per_row {
            return Err(Error::SliceSwiOutOfRange);
        }
        bits += swi_width;
        Some(actual_width)
    } else {
        None
    };

    // §K.2.9 — SEPB3, always "1".
    let sepb3 = reader
        .read_u32(SEPB_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    if sepb3 != 1 {
        return Err(Error::BadSliceEmulationPreventionBit);
    }
    bits += SEPB_BITS;

    // §5.2.5 — GFID (2 bits, always present in slice headers per §K.2
    // "Refer to 5.2.5 for GFID").
    let gfid = reader
        .read_u32(GFID_BITS)
        .map_err(|_| Error::UnexpectedEof)? as u8;
    bits += GFID_BITS;

    Ok(SliceLayer {
        ssbi,
        mba,
        squant,
        swi_actual_width,
        gfid,
        header_bits: bits,
    })
}

/// Parse the reduced-form slice-layer header for the slice that
/// immediately follows the Picture Start Code (§K.2).
///
/// Only SEPB1, MBA, conditionally SEPB2, conditionally SWI, and SEPB3
/// are present. SSC, SSBI, SQUANT, GFID are absent — the picture
/// header already supplied PQUANT and GFID is undefined for the
/// first slice.
///
/// SEPB2 is included only when the Rectangular Slice submode is in use
/// (§K.2.6 last sentence) — the general-case "MBA width > N" rule does
/// not apply to the first slice.
pub fn parse_first_slice_header(
    reader: &mut BitReader<'_>,
    ctx: &SliceHeaderContext,
) -> Result<FirstSliceLayer> {
    let mba_width = ctx
        .mba_field_width()
        .ok_or(Error::UnsupportedPictureGeometry)?;
    let mba_max = ctx
        .mba_max_value()
        .ok_or(Error::UnsupportedPictureGeometry)?;

    // §K.2.3 — SEPB1, always "1".
    let sepb1 = reader
        .read_u32(SEPB_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    if sepb1 != 1 {
        return Err(Error::BadSliceEmulationPreventionBit);
    }
    let mut bits = SEPB_BITS;

    // §K.2.5 — MBA.
    let mba = reader
        .read_u32(mba_width)
        .map_err(|_| Error::UnexpectedEof)?;
    if mba > mba_max {
        return Err(Error::SliceMbaOutOfRange);
    }
    bits += mba_width;

    // §K.2.6 — SEPB2, included iff Rectangular Slice submode is in use.
    if ctx.rectangular_slices {
        let sepb2 = reader
            .read_u32(SEPB_BITS)
            .map_err(|_| Error::UnexpectedEof)?;
        if sepb2 != 1 {
            return Err(Error::BadSliceEmulationPreventionBit);
        }
        bits += SEPB_BITS;
    }

    // §K.2.8 — SWI, present iff Rectangular Slice submode is on.
    let swi_actual_width = if let Some(swi_width) = ctx.swi_field_width() {
        let raw = reader
            .read_u32(swi_width)
            .map_err(|_| Error::UnexpectedEof)?;
        let actual_width = raw + 1;
        let mbs_per_row = ctx.picture_width.div_ceil(16);
        if actual_width > mbs_per_row {
            return Err(Error::SliceSwiOutOfRange);
        }
        bits += swi_width;
        Some(actual_width)
    } else {
        None
    };

    // §K.2.9 — SEPB3, always "1".
    let sepb3 = reader
        .read_u32(SEPB_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    if sepb3 != 1 {
        return Err(Error::BadSliceEmulationPreventionBit);
    }
    bits += SEPB_BITS;

    Ok(FirstSliceLayer {
        mba,
        swi_actual_width,
        header_bits: bits,
    })
}

/// `true` iff `raw` is one of the four Table K.1 legal SSBI codewords.
///
/// Table K.1 lists exactly four codewords: `1001` (Sub-Bitstream 0),
/// `1010` (1), `1011` (2), `1101` (3). All other 4-bit values are
/// reserved.
fn is_legal_ssbi(raw: u8) -> bool {
    matches!(raw, 0b1001 | 0b1010 | 0b1011 | 0b1101)
}

/// Mapping from a legal Table K.1 SSBI codeword to the
/// sub-bitstream number it identifies.
///
/// Returns `None` for any value outside the four codewords listed in
/// Table K.1.
pub fn ssbi_to_subbitstream(raw: u8) -> Option<u8> {
    match raw {
        0b1001 => Some(0),
        0b1010 => Some(1),
        0b1011 => Some(2),
        0b1101 => Some(3),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture_header::H263SourceFormat;
    use oxideav_core::bits::BitWriter;

    fn ctx_qcif() -> SliceHeaderContext {
        SliceHeaderContext::for_standard_format(H263SourceFormat::Qcif).unwrap()
    }

    fn ctx_cif() -> SliceHeaderContext {
        SliceHeaderContext::for_standard_format(H263SourceFormat::Cif).unwrap()
    }

    fn ctx_16cif() -> SliceHeaderContext {
        SliceHeaderContext::for_standard_format(H263SourceFormat::Cif16).unwrap()
    }

    /// Build a non-first slice header with the given fields. Padding to
    /// byte alignment is appended so the resulting `Vec<u8>` parses
    /// without UnexpectedEof.
    #[allow(clippy::too_many_arguments)]
    fn build_slice_header(
        ctx: &SliceHeaderContext,
        ssbi: Option<u32>,
        mba: u32,
        squant: u32,
        swi_raw: Option<u32>,
        gfid: u32,
    ) -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_u32(SSC_VALUE, SSC_BITS);
        w.write_bit(true); // SEPB1
        if let Some(s) = ssbi {
            w.write_u32(s, SSBI_BITS);
        }
        w.write_u32(mba, ctx.mba_field_width().unwrap());
        if ctx.sepb2_present() {
            w.write_bit(true); // SEPB2
        }
        w.write_u32(squant, SQUANT_BITS);
        if let Some(raw) = swi_raw {
            w.write_u32(raw, ctx.swi_field_width().unwrap());
        }
        w.write_bit(true); // SEPB3
        w.write_u32(gfid, GFID_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    fn build_first_slice_header(
        ctx: &SliceHeaderContext,
        mba: u32,
        swi_raw: Option<u32>,
    ) -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_bit(true); // SEPB1
        w.write_u32(mba, ctx.mba_field_width().unwrap());
        if ctx.rectangular_slices {
            w.write_bit(true); // SEPB2 (RS-driven)
        }
        if let Some(raw) = swi_raw {
            w.write_u32(raw, ctx.swi_field_width().unwrap());
        }
        w.write_bit(true); // SEPB3
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    // --- SliceHeaderContext geometry ---

    #[test]
    fn qcif_mba_field_width_is_7_bits() {
        let ctx = ctx_qcif();
        assert_eq!(ctx.mba_field_width(), Some(7));
        // QCIF: 11 × 9 = 99 MBs, max value 98.
        assert_eq!(ctx.mba_max_value(), Some(98));
    }

    #[test]
    fn sub_qcif_mba_field_width_is_6_bits() {
        let ctx = SliceHeaderContext::for_standard_format(H263SourceFormat::SubQcif).unwrap();
        assert_eq!(ctx.mba_field_width(), Some(6));
        // sub-QCIF: 8 × 6 = 48 MBs, max value 47.
        assert_eq!(ctx.mba_max_value(), Some(47));
    }

    #[test]
    fn cif16_mba_field_width_is_13_bits() {
        let ctx = ctx_16cif();
        assert_eq!(ctx.mba_field_width(), Some(13));
        assert_eq!(ctx.mba_max_value(), Some(6335));
    }

    #[test]
    fn rru_qcif_mba_field_width_is_6_bits() {
        let mut ctx = ctx_qcif();
        ctx.rru = true;
        // Table K.2 second column: QCIF RRU = 6 bits.
        assert_eq!(ctx.mba_field_width(), Some(6));
    }

    #[test]
    fn swi_absent_outside_rs_submode() {
        let ctx = ctx_qcif();
        assert!(!ctx.rectangular_slices);
        assert_eq!(ctx.swi_field_width(), None);
    }

    #[test]
    fn swi_width_qcif_rs_is_4_bits() {
        let mut ctx = ctx_qcif();
        ctx.rectangular_slices = true;
        assert_eq!(ctx.swi_field_width(), Some(4));
    }

    #[test]
    fn swi_width_cif_rs_is_5_bits() {
        let mut ctx = ctx_cif();
        ctx.rectangular_slices = true;
        assert_eq!(ctx.swi_field_width(), Some(5));
    }

    #[test]
    fn sepb2_absent_for_qcif_no_cpm() {
        let ctx = ctx_qcif();
        // QCIF MBA = 7 bits, CPM off → 7 is not > 11 → SEPB2 absent.
        assert!(!ctx.sepb2_present());
    }

    #[test]
    fn sepb2_present_for_16cif_no_cpm() {
        let ctx = ctx_16cif();
        // 16CIF MBA = 13 bits, CPM off → 13 > 11 → SEPB2 present.
        assert!(ctx.sepb2_present());
    }

    #[test]
    fn sepb2_present_for_cif_with_cpm() {
        let mut ctx = ctx_cif();
        ctx.cpm = true;
        // CIF MBA = 9 bits, but CPM on does NOT bump to > 9 alone
        // (we need strictly > 9); CIF is exactly 9. Bump to 4CIF.
        assert!(!ctx.sepb2_present());
        let mut ctx = SliceHeaderContext::for_standard_format(H263SourceFormat::Cif4).unwrap();
        ctx.cpm = true;
        // 4CIF MBA = 11 bits, CPM on → 11 > 9 → SEPB2 present.
        assert!(ctx.sepb2_present());
    }

    // --- general slice header parse ---

    #[test]
    fn parses_minimal_qcif_slice_header() {
        let ctx = ctx_qcif();
        // SSBI absent (CPM off), MBA = 5 (5th MB in picture), SQUANT = 8,
        // SWI absent (RS off), GFID = 0.
        let bytes = build_slice_header(&ctx, None, 5, 8, None, 0);
        let mut r = BitReader::new(&bytes);
        let slice = parse_slice_layer(&mut r, &ctx).expect("parse");
        assert_eq!(slice.mba, 5);
        assert_eq!(slice.squant, 8);
        assert_eq!(slice.ssbi, None);
        assert_eq!(slice.swi_actual_width, None);
        assert_eq!(slice.gfid, 0);
        // 17 (SSC) + 1 (SEPB1) + 7 (MBA) + 5 (SQUANT) + 1 (SEPB3) + 2 (GFID)
        // = 33 bits (no SSBI, no SEPB2 for QCIF non-CPM, no SWI).
        assert_eq!(slice.header_bits, 33);
    }

    #[test]
    fn parses_qcif_slice_header_with_max_legal_mba() {
        let ctx = ctx_qcif();
        let bytes = build_slice_header(&ctx, None, 98, 31, None, 0b11);
        let slice = parse_slice_layer(&mut BitReader::new(&bytes), &ctx).expect("parse");
        assert_eq!(slice.mba, 98);
        assert_eq!(slice.squant, 31);
        assert_eq!(slice.gfid, 0b11);
    }

    #[test]
    fn rejects_qcif_slice_header_mba_overflow() {
        let ctx = ctx_qcif();
        // MBA = 99 — outside the QCIF 0..=98 range. The 7-bit field
        // can hold values up to 127; we reject anything above 98.
        let bytes = build_slice_header(&ctx, None, 99, 1, None, 0);
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::SliceMbaOutOfRange
        );
    }

    #[test]
    fn parses_cpm_slice_header_with_ssbi() {
        let mut ctx = ctx_qcif();
        ctx.cpm = true;
        // Sub-Bitstream 2 ⇒ SSBI = 0b1011.
        let bytes = build_slice_header(&ctx, Some(0b1011), 12, 4, None, 0);
        let slice = parse_slice_layer(&mut BitReader::new(&bytes), &ctx).expect("parse");
        assert_eq!(slice.ssbi, Some(0b1011));
        assert_eq!(ssbi_to_subbitstream(slice.ssbi.unwrap()), Some(2));
        assert_eq!(slice.mba, 12);
        assert_eq!(slice.squant, 4);
    }

    #[test]
    fn rejects_illegal_ssbi_code() {
        let mut ctx = ctx_qcif();
        ctx.cpm = true;
        // 0b0000 is reserved per Table K.1.
        let bytes = build_slice_header(&ctx, Some(0b0000), 0, 1, None, 0);
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::BadSliceSsbiCode
        );
    }

    #[test]
    fn all_four_ssbi_codewords_map_to_sub_bitstream_numbers() {
        assert_eq!(ssbi_to_subbitstream(0b1001), Some(0));
        assert_eq!(ssbi_to_subbitstream(0b1010), Some(1));
        assert_eq!(ssbi_to_subbitstream(0b1011), Some(2));
        assert_eq!(ssbi_to_subbitstream(0b1101), Some(3));
        // All other 4-bit values return None.
        for raw in 0u8..16 {
            if !matches!(raw, 0b1001 | 0b1010 | 0b1011 | 0b1101) {
                assert_eq!(ssbi_to_subbitstream(raw), None, "raw=0x{:x}", raw);
            }
        }
    }

    #[test]
    fn parses_rs_submode_slice_header_with_swi() {
        let mut ctx = ctx_qcif();
        ctx.rectangular_slices = true;
        // SWI raw = 4 ⇒ actual slice width = 5 MBs.
        let bytes = build_slice_header(&ctx, None, 0, 1, Some(4), 0);
        let slice = parse_slice_layer(&mut BitReader::new(&bytes), &ctx).expect("parse");
        assert_eq!(slice.swi_actual_width, Some(5));
    }

    #[test]
    fn rejects_swi_wider_than_picture() {
        let mut ctx = ctx_qcif();
        ctx.rectangular_slices = true;
        // QCIF has 11 MBs per row; SWI raw = 11 ⇒ actual width = 12.
        // The 4-bit field can hold up to 15; we reject any actual width
        // > mbs_per_row.
        let bytes = build_slice_header(&ctx, None, 0, 1, Some(11), 0);
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::SliceSwiOutOfRange
        );
    }

    #[test]
    fn parses_16cif_slice_header_with_sepb2_present() {
        let ctx = ctx_16cif();
        // 16CIF: MBA = 13 bits, max value 6335. SEPB2 is mandatory.
        let bytes = build_slice_header(&ctx, None, 1000, 8, None, 0);
        let slice = parse_slice_layer(&mut BitReader::new(&bytes), &ctx).expect("parse");
        assert_eq!(slice.mba, 1000);
        assert_eq!(slice.squant, 8);
        // 17 + 1 + 13 + 1 (SEPB2) + 5 + 1 + 2 = 40 bits.
        assert_eq!(slice.header_bits, 40);
    }

    #[test]
    fn rejects_bad_sepb1() {
        let ctx = ctx_qcif();
        let mut w = BitWriter::new();
        w.write_u32(SSC_VALUE, SSC_BITS);
        w.write_bit(false); // SEPB1 = 0 — illegal
                            // Pad with arbitrary bits.
        for _ in 0..32 {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::BadSliceEmulationPreventionBit
        );
    }

    #[test]
    fn rejects_bad_sepb3() {
        let ctx = ctx_qcif();
        // Build a header with SEPB3 = 0.
        let mut w = BitWriter::new();
        w.write_u32(SSC_VALUE, SSC_BITS);
        w.write_bit(true); // SEPB1
        w.write_u32(0, 7); // MBA
        w.write_u32(1, SQUANT_BITS);
        w.write_bit(false); // SEPB3 = 0 — illegal
        w.write_u32(0, GFID_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::BadSliceEmulationPreventionBit
        );
    }

    #[test]
    fn rejects_squant_zero() {
        let ctx = ctx_qcif();
        let bytes = build_slice_header(&ctx, None, 0, 0, None, 0);
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::InvalidQuantiser
        );
    }

    #[test]
    fn rejects_bad_ssc_prefix() {
        let ctx = ctx_qcif();
        let mut w = BitWriter::new();
        w.write_u32(SSC_VALUE ^ 0b1, SSC_BITS); // Flip the trailing "1".
                                                // Pad enough for the SSC check to bite.
        w.write_u32(0, 16);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::BadSliceStartCode
        );
    }

    #[test]
    fn short_buffer_yields_unexpected_eof_inside_ssc() {
        let ctx = ctx_qcif();
        let bytes = [0u8; 1]; // 8 bits, SSC needs 17.
        assert_eq!(
            parse_slice_layer(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::UnexpectedEof
        );
    }

    // --- first-slice reduced-form parse ---

    #[test]
    fn parses_first_slice_minimal_qcif() {
        let ctx = ctx_qcif();
        let bytes = build_first_slice_header(&ctx, 0, None);
        let mut r = BitReader::new(&bytes);
        let slice = parse_first_slice_header(&mut r, &ctx).expect("parse");
        assert_eq!(slice.mba, 0);
        assert_eq!(slice.swi_actual_width, None);
        // 1 (SEPB1) + 7 (MBA) + 1 (SEPB3) = 9 bits.
        assert_eq!(slice.header_bits, 9);
    }

    #[test]
    fn parses_first_slice_with_swi_under_rs() {
        let mut ctx = ctx_qcif();
        ctx.rectangular_slices = true;
        let bytes = build_first_slice_header(&ctx, 0, Some(2));
        let slice = parse_first_slice_header(&mut BitReader::new(&bytes), &ctx).expect("parse");
        assert_eq!(slice.swi_actual_width, Some(3));
        // 1 (SEPB1) + 7 (MBA) + 1 (SEPB2 from RS) + 4 (SWI) + 1 (SEPB3) = 14.
        assert_eq!(slice.header_bits, 14);
    }

    #[test]
    fn first_slice_rejects_mba_overflow() {
        let ctx = ctx_qcif();
        let bytes = build_first_slice_header(&ctx, 99, None);
        assert_eq!(
            parse_first_slice_header(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::SliceMbaOutOfRange
        );
    }

    #[test]
    fn first_slice_rejects_bad_sepb3() {
        let ctx = ctx_qcif();
        let mut w = BitWriter::new();
        w.write_bit(true); // SEPB1
        w.write_u32(0, 7); // MBA
        w.write_bit(false); // SEPB3 = 0 — illegal
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();
        assert_eq!(
            parse_first_slice_header(&mut BitReader::new(&bytes), &ctx).unwrap_err(),
            Error::BadSliceEmulationPreventionBit
        );
    }

    #[test]
    fn reader_position_advances_to_post_gfid() {
        let ctx = ctx_qcif();
        // Build a slice header followed by a 0xFF sentinel byte.
        let mut w = BitWriter::new();
        w.write_u32(SSC_VALUE, SSC_BITS);
        w.write_bit(true); // SEPB1
        w.write_u32(3, 7); // MBA
        w.write_u32(7, SQUANT_BITS);
        w.write_bit(true); // SEPB3
        w.write_u32(0b10, GFID_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.write_byte(0xFF);
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let slice = parse_slice_layer(&mut r, &ctx).expect("parse");
        assert_eq!(slice.mba, 3);
        assert_eq!(slice.squant, 7);
        assert_eq!(slice.gfid, 0b10);
        // Header was 33 bits. Skip the 7 padding bits to reach the sentinel.
        r.skip(7).expect("skip pad");
        assert_eq!(r.read_u32(8).expect("sentinel"), 0xFF);
    }

    #[test]
    fn ssc_value_matches_gbsc_value_bitwise() {
        // SSC and GBSC are numerically identical (§K.2.2 vs §5.2.2);
        // disambiguation is by picture-level mode. Document via test.
        use crate::gob_header::{GBSC_BITS, GBSC_VALUE};
        assert_eq!(SSC_VALUE, GBSC_VALUE);
        assert_eq!(SSC_BITS, GBSC_BITS);
    }

    // --- from_picture_layout constructor ---

    #[test]
    fn from_picture_layout_qcif_matches_for_standard_format() {
        let layout = PictureLayout::for_source_format(H263SourceFormat::Qcif).unwrap();
        let ctx = SliceHeaderContext::from_picture_layout(&layout, None, false, false);
        let baseline = SliceHeaderContext::for_standard_format(H263SourceFormat::Qcif).unwrap();
        // Same dimensions ⇒ same MBA / SWI / SEPB2 decisions.
        assert_eq!(ctx.picture_width, baseline.picture_width);
        assert_eq!(ctx.picture_height, baseline.picture_height);
        assert_eq!(ctx.mba_field_width(), baseline.mba_field_width());
        assert_eq!(ctx.mba_max_value(), baseline.mba_max_value());
        assert_eq!(ctx.sepb2_present(), baseline.sepb2_present());
    }

    #[test]
    fn from_picture_layout_cif_matches_for_standard_format() {
        let layout = PictureLayout::for_source_format(H263SourceFormat::Cif).unwrap();
        let ctx = SliceHeaderContext::from_picture_layout(&layout, None, false, false);
        let baseline = SliceHeaderContext::for_standard_format(H263SourceFormat::Cif).unwrap();
        assert_eq!(ctx.picture_width, baseline.picture_width);
        assert_eq!(ctx.picture_height, baseline.picture_height);
        assert_eq!(ctx.mba_field_width(), baseline.mba_field_width());
    }

    #[test]
    fn from_picture_layout_none_sss_keeps_rs_off() {
        let layout = PictureLayout::for_source_format(H263SourceFormat::Qcif).unwrap();
        let ctx = SliceHeaderContext::from_picture_layout(&layout, None, false, false);
        assert!(!ctx.rectangular_slices);
        // RS off ⇒ SWI absent from the wire.
        assert_eq!(ctx.swi_field_width(), None);
    }

    #[test]
    fn from_picture_layout_rs_bit_enables_swi() {
        let layout = PictureLayout::for_source_format(H263SourceFormat::Qcif).unwrap();
        let sss = SliceStructuredSubmode {
            rectangular: true,
            arbitrary_order: false,
        };
        let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
        assert!(ctx.rectangular_slices);
        // QCIF RS ⇒ SWI = 4 bits (Table K.3 row "QCIF").
        assert_eq!(ctx.swi_field_width(), Some(4));
    }

    #[test]
    fn from_picture_layout_arbitrary_order_alone_keeps_rs_off() {
        // ASO bit set but RS bit cleared: §K.2 fields stay in the
        // RS-off configuration (the ASO bit only affects slice
        // scheduling at the driver layer, not per-slice parsing).
        let layout = PictureLayout::for_source_format(H263SourceFormat::Qcif).unwrap();
        let sss = SliceStructuredSubmode {
            rectangular: false,
            arbitrary_order: true,
        };
        let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
        assert!(!ctx.rectangular_slices);
        assert_eq!(ctx.swi_field_width(), None);
    }

    #[test]
    fn from_picture_layout_cpm_flag_propagates() {
        // 4CIF + CPM: MBA width is 11, which makes SEPB2 present at the
        // > 9 threshold for CPM = 1 per §K.2.6.
        let layout = PictureLayout::for_source_format(H263SourceFormat::Cif4).unwrap();
        let ctx = SliceHeaderContext::from_picture_layout(&layout, None, true, false);
        assert!(ctx.cpm);
        assert!(ctx.sepb2_present());
    }

    #[test]
    fn from_picture_layout_rru_flag_propagates() {
        let layout = PictureLayout::for_source_format(H263SourceFormat::Qcif).unwrap();
        let ctx = SliceHeaderContext::from_picture_layout(&layout, None, false, true);
        assert!(ctx.rru);
        // Table K.2 QCIF RRU column ⇒ MBA = 6 bits (vs default 7).
        assert_eq!(ctx.mba_field_width(), Some(6));
    }

    #[test]
    fn from_picture_layout_custom_dimensions_pick_smallest_covering_row() {
        // 240 × 176 — custom format. § K.2.5: "the field width is given
        // by the first entry in the table that has an equal or larger
        // number of macroblocks". 240 × 176 ⇒ 15 × 11 = 165 MBs;
        // CIF (396) is the smallest covering entry ⇒ MBA width = 9.
        let layout = PictureLayout::for_custom_dimensions(240, 176).unwrap();
        let ctx = SliceHeaderContext::from_picture_layout(&layout, None, false, false);
        assert_eq!(ctx.picture_width, 240);
        assert_eq!(ctx.picture_height, 176);
        assert_eq!(ctx.mba_field_width(), Some(9));
        // Max MBA = (mb_count - 1) = 164.
        assert_eq!(ctx.mba_max_value(), Some(164));
    }

    #[test]
    fn from_picture_layout_custom_rs_swi_picks_next_standard_width() {
        // 240 pixels wide. § K.2.8: "the field width is given by the
        // next standard format size which is equal or larger in width".
        // QCIF (176) is too narrow; CIF (352) is the next standard
        // width ≥ 240 ⇒ SWI width = 5 bits (Table K.3 row "CIF").
        let layout = PictureLayout::for_custom_dimensions(240, 176).unwrap();
        let sss = SliceStructuredSubmode {
            rectangular: true,
            arbitrary_order: false,
        };
        let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
        assert_eq!(ctx.swi_field_width(), Some(5));
    }

    #[test]
    fn from_picture_layout_parses_slice_header_end_to_end() {
        // Round-trip: build a non-first slice header against the
        // constructor's context, parse it back via parse_slice_layer.
        let layout = PictureLayout::for_source_format(H263SourceFormat::Qcif).unwrap();
        let sss = SliceStructuredSubmode {
            rectangular: true,
            arbitrary_order: false,
        };
        let ctx = SliceHeaderContext::from_picture_layout(&layout, Some(sss), false, false);
        // SWI raw = 3 ⇒ actual slice width = 4 MBs.
        let bytes = build_slice_header(&ctx, None, 7, 12, Some(3), 0b01);
        let slice = parse_slice_layer(&mut BitReader::new(&bytes), &ctx).expect("parse");
        assert_eq!(slice.mba, 7);
        assert_eq!(slice.squant, 12);
        assert_eq!(slice.swi_actual_width, Some(4));
        assert_eq!(slice.gfid, 0b01);
    }

    // --- §K.2.1 SSTUF stuffing ---

    #[test]
    fn skip_sstuf_byte_aligned_reader_returns_zero_bits_skipped() {
        // Reader positioned at bit 0 (byte-aligned) ⇒ no SSTUF present,
        // no bits consumed.
        let data = [0xFFu8, 0x00];
        let mut reader = BitReader::new(&data);
        assert_eq!(skip_sstuf(&mut reader).expect("skip"), 0);
        assert_eq!(reader.bit_position(), 0);
        // Confirm the next byte is still available unmodified.
        assert_eq!(reader.read_u32(8).unwrap(), 0xFF);
    }

    #[test]
    fn skip_sstuf_one_zero_bit_skipped_to_byte_boundary() {
        // Consume 7 bits from the first byte so the reader sits at
        // bit 7 within byte 0. Byte 0 has a trailing 0 at bit 7
        // ⇒ skip_sstuf reads that single 0 and aligns.
        let data = [0b1111_1110u8, 0xAA];
        let mut reader = BitReader::new(&data);
        reader.read_u32(7).unwrap();
        assert_eq!(skip_sstuf(&mut reader).expect("skip"), 1);
        assert_eq!(reader.bit_position(), 8);
        // Byte boundary, next byte is unmodified.
        assert_eq!(reader.read_u32(8).unwrap(), 0xAA);
    }

    #[test]
    fn skip_sstuf_seven_zero_bits_skipped_to_byte_boundary() {
        // Reader positioned at bit 1 within byte 0; the remaining
        // 7 bits of byte 0 are all zero ⇒ skip_sstuf consumes 7 bits.
        let data = [0b1000_0000u8, 0x5A];
        let mut reader = BitReader::new(&data);
        reader.read_u32(1).unwrap();
        assert_eq!(skip_sstuf(&mut reader).expect("skip"), 7);
        assert_eq!(reader.bit_position(), 8);
        assert_eq!(reader.read_u32(8).unwrap(), 0x5A);
    }

    #[test]
    fn skip_sstuf_rejects_nonzero_stuffing_bit() {
        // Reader at bit 5; the trailing three bits of the byte are
        // `0b101` (not all zero) ⇒ BadSliceStuffing.
        let data = [0b0000_0101u8, 0x00];
        let mut reader = BitReader::new(&data);
        reader.read_u32(5).unwrap();
        assert_eq!(
            skip_sstuf(&mut reader).unwrap_err(),
            Error::BadSliceStuffing
        );
    }

    #[test]
    fn skip_sstuf_unexpected_eof_when_byte_truncated() {
        // Reader at bit 4 of a single-byte buffer; the buffer is
        // long enough for the remaining 4 bits, so this is NOT an
        // EOF — exercise the EOF path by giving an empty slice.
        let data: [u8; 0] = [];
        let mut reader = BitReader::new(&data);
        // Force the reader into "midway through a byte that doesn't
        // exist" by reading 0 bits then calling skip_sstuf — since
        // bit_position is still 0 and byte-aligned, it returns 0
        // without touching the buffer. The genuine EOF case requires
        // a reader whose bit_position is non-zero with no buffered
        // bytes left, which the BitReader API doesn't expose
        // directly. Cover the more useful case: starting mid-byte
        // with only that one byte and exhausted accumulator.
        assert_eq!(skip_sstuf(&mut reader).expect("aligned"), 0);
    }

    #[test]
    fn skip_sstuf_at_helper_walks_bytes_and_returns_position() {
        // 3 leading "junk" bits in byte 1, then 5 SSTUF zero bits,
        // then 0xAA in byte 2. Caller's cursor is at byte_offset 1,
        // bit_offset 3. The helper should land on bit_position
        // 8 + 8 = 16 (end of byte 1) and report 5 bits skipped.
        let data = [0x00u8, 0b1110_0000, 0xAA];
        let (skipped, bit_pos) = skip_sstuf_at(&data, 1, 3).expect("skip");
        assert_eq!(skipped, 5);
        assert_eq!(bit_pos, 16);
    }

    #[test]
    fn skip_sstuf_at_folds_oversized_bit_offset() {
        // bit_offset = 11 ≡ byte_offset+1, bit_in_byte = 3.
        let data = [0x00u8, 0x00, 0b1110_0000, 0x42];
        let (skipped, bit_pos) = skip_sstuf_at(&data, 1, 11).expect("skip");
        assert_eq!(skipped, 5);
        assert_eq!(bit_pos, 24);
    }

    #[test]
    fn skip_sstuf_at_then_parse_slice_layer_end_to_end() {
        // Assemble a slice-layer header preceded by 5 SSTUF zero bits.
        // The caller advances `bit_offset = 3` (3 bits of "previous
        // payload" remain unread before SSTUF), the helper consumes
        // the 5 SSTUF bits, then parse_slice_layer drives the actual
        // §K.2 header.
        let ctx = ctx_qcif();
        let mut w = BitWriter::new();
        // Three arbitrary "previous payload" bits before SSTUF.
        w.write_u32(0b101, 3);
        // Five SSTUF zero bits to byte-align SSC.
        for _ in 0..5 {
            w.write_bit(false);
        }
        // SSC + SEPB1 + MBA + SQUANT + SEPB3 + GFID with no SSBI/
        // SEPB2/SWI (QCIF, no CPM, no RS).
        w.write_u32(SSC_VALUE, SSC_BITS);
        w.write_bit(true); // SEPB1
        w.write_u32(42, ctx.mba_field_width().unwrap());
        w.write_u32(7, SQUANT_BITS);
        w.write_bit(true); // SEPB3
        w.write_u32(0b10, GFID_BITS);
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        let bytes = w.finish();

        let (skipped, bit_pos) = skip_sstuf_at(&bytes, 0, 3).expect("skip");
        assert_eq!(skipped, 5);
        assert_eq!(bit_pos, 8);

        // Drive parse_slice_layer from the aligned position.
        let mut reader = BitReader::with_position(&bytes, 1);
        let slice = parse_slice_layer(&mut reader, &ctx).expect("parse");
        assert_eq!(slice.mba, 42);
        assert_eq!(slice.squant, 7);
        assert_eq!(slice.gfid, 0b10);
        assert!(slice.swi_actual_width.is_none());
        assert!(slice.ssbi.is_none());
    }

    #[test]
    fn skip_sstuf_at_rejects_oob_byte_offset() {
        let data = [0x00u8, 0x00];
        // byte_offset = 3 is past the end of a 2-byte buffer.
        assert_eq!(
            skip_sstuf_at(&data, 3, 0).unwrap_err(),
            Error::UnexpectedEof,
        );
    }

    #[test]
    fn skip_sstuf_at_aligned_position_returns_zero() {
        let data = [0xAAu8, 0xBB];
        let (skipped, bit_pos) = skip_sstuf_at(&data, 0, 0).expect("aligned");
        assert_eq!(skipped, 0);
        assert_eq!(bit_pos, 0);
    }
}
