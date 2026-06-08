//! H.263 PB-frame B-block layer field parsers (§5.3.3 / §5.3.4).
//!
//! The PB-frame mode (Annex G; signalled by PTYPE bit 13 in §5.1.3
//! or by the PLUSPTYPE picture-type code in §5.1.4.3) inserts two
//! additional macroblock-layer fields between MCBPC and CBPY:
//!
//! ```text
//!   COD   MCBPC   MODB   CBPB   CBPY   DQUANT   MVD   MVD2-4   MVDB   Block Data
//!                  ^^^^   ^^^^                                  ^^^^
//!                  §5.3.3 §5.3.4                                §5.3.9
//! ```
//!
//! This module provides the two pure-parser primitives the future
//! macroblock-layer driver needs to consume them:
//!
//! * [`parse_modb`] — §5.3.3 / Table 11 variable-length codeword that
//!   tells the decoder whether the B-block coefficient pattern (CBPB)
//!   and/or the B-block motion-vector difference (MVDB) are on the
//!   wire for the current macroblock. Returns a [`ModbPresence`] tag
//!   covering the three legal codewords; an unrecognised prefix is a
//!   bitstream error.
//! * [`parse_cbpb`] — §5.3.4 6-bit fixed-length field; bit `5` (MSB)
//!   carries B-block number 1, bit `0` (LSB) carries B-block number
//!   6. Returns the raw 6-bit pattern in the low bits of a `u8`.
//!   [`cbpb_block_present`] is the per-block accessor that consults
//!   the pattern with the Figure-5 / §5.3.4 "utmost left bit ↔ block
//!   number 1" convention.
//!
//! ## Composition with the macroblock driver
//!
//! These are pure-parser primitives. They do not consult any
//! enclosing-picture context (PTYPE / MCBPC MB-type / Annex M vs
//! Annex G mode-selector). The macroblock driver is responsible for
//! gating their invocation per §5.3.3 ("MODB is present for MB-type
//! 0-4 if PTYPE indicates 'PB-frame'") and §5.3.4 ("CBPB is only
//! present in PB-frames mode if indicated by MODB").
//!
//! The Annex M (Improved PB-frames) MODB table is **different** from
//! Table 11 — Annex M defines a 6-entry table (Table M.1) where the
//! Annex G [`parse_modb`] primitive covers only the 3-entry form.
//! [`parse_modb_annex_m`] is the sibling primitive for the Annex M
//! 6-entry form; it returns a [`ModbAnnexM`] tag combining the
//! `(CBPB, MVDB)` presence pair with the per-row coding mode
//! ([`BpbCodingMode`]) Table M.1 attaches to each row. The Annex M
//! "BPB" terminology (B-Part of an Improved PB-frame, per §M.1) is
//! used in the tag and its accessors; per §M.1 "B-picture, B-macroblock
//! and B-block will not be used in this annex".
//!
//! Per §5.3.9, MVDB is "a variable length codeword for the horizontal
//! component followed by a variable length codeword for the vertical
//! component of each vector. Variable length codes are given in
//! Table 14." Table 14 is the same MVD VLC the baseline §5.3.7 parser
//! already decodes via [`crate::macroblock::H263Macroblock::mvd`]'s
//! component decoder; no new VLC table lands for MVDB itself. The
//! macroblock driver wires the existing `decode_mvd_component` into
//! MVDB when MODB indicates MVDB presence.
//!
//! ## §G.4 — Calculation of vectors for the B-picture
//!
//! Once the §5.3.9 MVDB has been parsed (or determined absent from the
//! §5.3.3 / §M.4 MODB tag), §G.4 derives the per-luminance-block
//! forward and backward motion vectors `(MVF, MVB)` of the B-picture
//! from three inputs: the P-picture's vector `MV` for the 8×8 luma
//! block, the §5.3.9 delta `MVD` (zero if MVDB absent), and the
//! temporal-reference scalars `TRB` (§5.1.22) and `TRD` (the §5.1.2 TR
//! increment from the last picture header). The spec specifies the
//! pair via:
//!
//! ```text
//!   MVF = (TRB × MV) / TRD + MVD
//!   MVB = ((TRB - TRD) × MV) / TRD      if MVD == 0
//!   MVB = MVF - MV                      if MVD != 0
//! ```
//!
//! with "/" meaning division by truncation (Rust's signed-integer `/`
//! operator). Both `MVF` and `MVB` are returned in half-pel units;
//! [`pb_b_vectors`] computes one component pair (horizontal or
//! vertical), and [`pb_b_vector`] composes the two components into a
//! [`MotionVector`] pair for the full 8×8 luma block. §G.4 also
//! prescribes the chroma B-vector derivation: sum the four luma MVF
//! (resp. MVB) components and divide by 8, then snap toward the
//! nearest half-pel position per Table F.1 — the existing
//! [`crate::motion::chroma_mv_component_4mv`] / `chroma_mv_4mv`
//! primitive performs exactly that sum-of-4-luma-half-pel /
//! Table-F.1-snap transform, so [`pb_b_chroma_vector`] composes it
//! over the four luma B-vectors directly without duplicating the
//! Table F.1 logic.
//!
//! ## §G.5 — Bidirectional-prediction mask for a B-block
//!
//! §G.5 prescribes that, once the §G.4 vector pair `(MVF, MVB)` is
//! known, the per-pixel prediction of a B-block in a PB-frame splits
//! into two regions:
//!
//! * pixels where `MVB` points **inside** the just-reconstructed
//!   P-macroblock (PREC) are predicted bidirectionally — the average
//!   of the forward prediction (MVF, relative to the previous decoded
//!   picture) and the backward prediction (MVB, relative to PREC);
//!   the average is by truncation;
//! * all other pixels of the B-block are predicted by forward
//!   prediction only (MVF, relative to the previous decoded picture).
//!
//! §G.5 specifies the per-pixel split with two C-language loop nests
//! (one for luminance, one for chrominance), reproduced verbatim here
//! for cross-reference:
//!
//! ```text
//!   /* luminance: per 8 × 8 luma block (nh, nv) in the macroblock */
//!   for (nh = 0; nh <= 1; nh++) {
//!     for (nv = 0; nv <= 1; nv++) {
//!       for (i = nh*8 + max(0, (-mh(nh,nv)+1)/2 - nh*8);
//!            i <= nh*8 + min(7, 15 - (mh(nh,nv)+1)/2 - nh*8); i++) {
//!         for (j = nv*8 + max(0, (-mv(nh,nv)+1)/2 - nv*8);
//!              j <= nv*8 + min(7, 15 - (mv(nh,nv)+1)/2 - nv*8); j++) {
//!           predict pixel (i,j) bidirectionally
//!         }
//!       }
//!     }
//!   }
//!
//!   /* chrominance: one 8 × 8 chroma block per macroblock */
//!   for (i = max(0, (-mhc+1)/2); i <= min(7, 7 - (mhc+1)/2); i++) {
//!     for (j = max(0, (-mvc+1)/2); j <= min(7, 7 - (mvc+1)/2); j++) {
//!       predict pixel (i,j) bidirectionally;
//!     }
//!   }
//! ```
//!
//! Reading the two nests algebraically: per axis, the §G.5
//! bidirectional pixels of a luma block at (nh, nv) are those `i`
//! satisfying
//!
//! * `i ≥ nh*8` and `i ≥ (-mh+1)/2` (lower bound)
//! * `i ≤ nh*8 + 7` and `i ≤ 15 - (mh+1)/2` (upper bound)
//!
//! i.e. `i ∈ [max(nh*8, (-mh+1)/2), min(nh*8+7, 15 - (mh+1)/2)]` in
//! macroblock-local pixel coordinates 0..=15; the range is empty
//! when the lower bound exceeds the upper. For chrominance the
//! same shape applies inside a single 8 × 8 chroma block:
//! `i ∈ [max(0, (-mhc+1)/2), min(7, 7 - (mhc+1)/2)]` in 0..=7.
//!
//! "/" is integer division by truncation (matching Rust's signed
//! `/`). Per §G.5 each axis is independent — a pixel is
//! bidirectional if and only if **both** axes' ranges include its
//! coordinate. The mask thus factorises as the Cartesian product of
//! a horizontal and a vertical 1-D inclusive range, which is what the
//! primitives below return.
//!
//! [`pb_b_bidir_extent_component`] is the per-axis pure function: it
//! takes one half-pel `MVB` component and the inclusive `[lo, hi]`
//! block extent (`[0, 7]` for chroma or the nh=0 / nv=0 luma
//! sub-block, `[8, 15]` for nh=1 / nv=1) and returns the §G.5
//! inclusive range or `None` if empty. [`pb_b_bidir_luma_block_extent`]
//! composes it for one of the four 8 × 8 luma sub-blocks at
//! `(nh, nv)`, returning the 2-D rectangle of bidirectional pixels.
//! [`pb_b_bidir_chroma_extent`] is the chroma counterpart over the
//! single 0..=7 block. Pixels outside the returned rectangle are
//! forward-only per §G.5's "all other pixels" clause.
//!
//! The blend itself — the per-pixel arithmetic §G.5 prescribes for
//! pixels inside the rectangle — is [`pb_b_bidir_pixel`]: average
//! of the two prediction samples by integer division. The driver
//! convenience [`pb_b_blend_block`] applies the average across one
//! 8 × 8 block given the rectangle from an extent primitive and the
//! two prediction arrays, falling back to the forward sample for
//! pixels outside the rectangle.

use oxideav_core::bits::BitReader;

use crate::macroblock::{decode_mvd_component, Mvd};
use crate::motion::{chroma_mv_component_4mv, MotionVector};
use crate::{Error, Result};

/// Field width of the §5.3.4 CBPB Coded Block Pattern for B-blocks.
/// Six bits, one per B-block in the macroblock (four luma + two
/// chroma per §5.3.3 / Figure 5).
pub const CBPB_BITS: u32 = 6;

/// Per-B-block presence signalled by a §5.3.3 / Table 11 MODB
/// codeword. Names follow the §5.3.3 column headers ("CBPB", "MVDB")
/// with the "X" cells in Table 11 collapsed onto the tag variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModbPresence {
    /// Table 11 row 0 — neither CBPB nor MVDB on the wire for this
    /// macroblock's B-blocks. Code `0`, 1 bit.
    None,
    /// Table 11 row 1 — MVDB on the wire, CBPB absent. Code `10`,
    /// 2 bits.
    MvdbOnly,
    /// Table 11 row 2 — both CBPB and MVDB on the wire. Code `11`,
    /// 2 bits.
    CbpbAndMvdb,
}

impl ModbPresence {
    /// `true` iff Table 11 marks CBPB as present ("X" in the CBPB
    /// column). Only [`ModbPresence::CbpbAndMvdb`] does.
    pub fn has_cbpb(self) -> bool {
        matches!(self, ModbPresence::CbpbAndMvdb)
    }

    /// `true` iff Table 11 marks MVDB as present ("X" in the MVDB
    /// column). [`ModbPresence::MvdbOnly`] and
    /// [`ModbPresence::CbpbAndMvdb`].
    pub fn has_mvdb(self) -> bool {
        matches!(self, ModbPresence::MvdbOnly | ModbPresence::CbpbAndMvdb)
    }

    /// Length in bits of the §5.3.3 / Table 11 codeword that produced
    /// this tag. Useful for tests and for any caller that needs the
    /// post-parse bit cursor without re-running the bitreader.
    pub fn code_bits(self) -> u32 {
        match self {
            ModbPresence::None => 1,
            ModbPresence::MvdbOnly | ModbPresence::CbpbAndMvdb => 2,
        }
    }
}

/// Decode a §5.3.3 / Table 11 MODB variable-length codeword.
///
/// The reader is left positioned at the first bit following the
/// MODB code on success. On `Err(Error::UnexpectedEof)` the reader's
/// position is unspecified (1-bit MODB always succeeds on a non-empty
/// reader; only the 2-bit branch can run off the end mid-code).
///
/// Table 11 layout:
///
/// | Index | CBPB | MVDB | Code |
/// |-------|------|------|------|
/// | 0     |      |      | `0`  |
/// | 1     |      | X    | `10` |
/// | 2     | X    | X    | `11` |
///
/// The leading bit `0` immediately resolves [`ModbPresence::None`];
/// the leading bit `1` requires one more bit to disambiguate
/// [`ModbPresence::MvdbOnly`] (`10`) from [`ModbPresence::CbpbAndMvdb`]
/// (`11`). There is no unknown / forbidden prefix shape — every
/// 1- or 2-bit prefix starting from the reader is a legal Table 11
/// codeword. The only error this function returns is
/// [`Error::UnexpectedEof`] when the stream ends after the leading
/// `1` without a second bit.
pub fn parse_modb(reader: &mut BitReader<'_>) -> Result<ModbPresence> {
    let lead = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if !lead {
        return Ok(ModbPresence::None);
    }
    let second = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if second {
        Ok(ModbPresence::CbpbAndMvdb)
    } else {
        Ok(ModbPresence::MvdbOnly)
    }
}

/// §M.2 Improved PB-frames BPB-macroblock coding modes. Each Table M.1
/// row carries one of these three values in its "Coding mode" column;
/// the §M.2 sub-sections give the per-mode prediction recipe the
/// decoder applies once the mode is known.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BpbCodingMode {
    /// §M.2.1 — prediction uses the reference pictures before and
    /// after the BPB-picture. Equivalent to the Annex G prediction
    /// when MVD = 0. Table M.1 rows 0 and 1.
    Bidirectional,
    /// §M.2.2 — the BPB-macroblock has a single 16×16 forward MVDB
    /// vector pointing into the previous reference picture; no
    /// backward reference is used. Table M.1 rows 2 and 3.
    Forward,
    /// §M.2.3 — prediction is identical to PREC (defined in §G.5);
    /// no MVDB on the wire. Table M.1 rows 4 and 5.
    Backward,
}

/// Per-Table M.1 row tag returned by [`parse_modb_annex_m`]. The tag
/// collapses the table's three columns — `CBPB` presence, `MVDB`
/// presence, and the §M.2 coding mode — onto a single value matched
/// 1:1 with one of Table M.1's six rows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModbAnnexM {
    /// Table M.1 row 0 — CBPB absent, MVDB absent, §M.2.1
    /// bidirectional. Code `0`, 1 bit.
    BidirNoCbpbNoMvdb,
    /// Table M.1 row 1 — CBPB present, MVDB absent, §M.2.1
    /// bidirectional. Code `10`, 2 bits.
    BidirCbpbNoMvdb,
    /// Table M.1 row 2 — CBPB absent, MVDB present, §M.2.2 forward.
    /// Code `110`, 3 bits.
    ForwardNoCbpbMvdb,
    /// Table M.1 row 3 — CBPB present, MVDB present, §M.2.2 forward.
    /// Code `1110`, 4 bits.
    ForwardCbpbMvdb,
    /// Table M.1 row 4 — CBPB absent, MVDB absent, §M.2.3 backward.
    /// Code `11110`, 5 bits.
    BackwardNoCbpbNoMvdb,
    /// Table M.1 row 5 — CBPB present, MVDB absent, §M.2.3 backward.
    /// Code `11111`, 5 bits.
    BackwardCbpbNoMvdb,
}

impl ModbAnnexM {
    /// `true` iff Table M.1 marks CBPB as present (`x` in the CBPB
    /// column). Rows 1, 3 and 5.
    pub fn has_cbpb(self) -> bool {
        matches!(
            self,
            ModbAnnexM::BidirCbpbNoMvdb
                | ModbAnnexM::ForwardCbpbMvdb
                | ModbAnnexM::BackwardCbpbNoMvdb
        )
    }

    /// `true` iff Table M.1 marks MVDB as present (`x` in the MVDB
    /// column). Rows 2 and 3 only — §M.2.2 forward prediction is the
    /// only mode that carries MVDB on the wire under Annex M.
    pub fn has_mvdb(self) -> bool {
        matches!(
            self,
            ModbAnnexM::ForwardNoCbpbMvdb | ModbAnnexM::ForwardCbpbMvdb
        )
    }

    /// §M.2 coding mode signalled by this row's Table M.1 entry.
    pub fn coding_mode(self) -> BpbCodingMode {
        match self {
            ModbAnnexM::BidirNoCbpbNoMvdb | ModbAnnexM::BidirCbpbNoMvdb => {
                BpbCodingMode::Bidirectional
            }
            ModbAnnexM::ForwardNoCbpbMvdb | ModbAnnexM::ForwardCbpbMvdb => BpbCodingMode::Forward,
            ModbAnnexM::BackwardNoCbpbNoMvdb | ModbAnnexM::BackwardCbpbNoMvdb => {
                BpbCodingMode::Backward
            }
        }
    }

    /// Length in bits of the Table M.1 codeword that produced this
    /// tag. Useful for tests and for any caller that needs the
    /// post-parse bit cursor without re-running the bitreader.
    pub fn code_bits(self) -> u32 {
        match self {
            ModbAnnexM::BidirNoCbpbNoMvdb => 1,
            ModbAnnexM::BidirCbpbNoMvdb => 2,
            ModbAnnexM::ForwardNoCbpbMvdb => 3,
            ModbAnnexM::ForwardCbpbMvdb => 4,
            ModbAnnexM::BackwardNoCbpbNoMvdb | ModbAnnexM::BackwardCbpbNoMvdb => 5,
        }
    }
}

/// Decode an §M.4 / Table M.1 Improved PB-frames MODB variable-length
/// codeword.
///
/// This is the Annex M sibling of [`parse_modb`]. Annex M replaces
/// Table 11 with the 6-entry Table M.1 when the picture-header
/// PLUSPTYPE indicates "Improved PB-frame" (versus Annex G's plain
/// "PB-frame" carried by the legacy PTYPE bit 13). The decoder picks
/// the parser per-picture based on the picture-coding type; the
/// macroblock-layer driver dispatches between [`parse_modb`] and
/// [`parse_modb_annex_m`] accordingly.
///
/// The reader is left positioned at the first bit following the MODB
/// code on success. On `Err(Error::UnexpectedEof)` the reader's
/// position is unspecified (every legal Table M.1 codeword starts
/// with a run of 1..=4 leading `1` bits terminated by a `0`, except
/// for the 5-bit rows which use the run length 4 plus a tail bit;
/// any of those reads can run off the buffer end).
///
/// Table M.1 layout:
///
/// | Index | CBPB | MVDB | Bits | Code    | Coding mode    |
/// |-------|------|------|------|---------|----------------|
/// | 0     |      |      | 1    | `0`     | Bidirectional  |
/// | 1     | x    |      | 2    | `10`    | Bidirectional  |
/// | 2     |      | x    | 3    | `110`   | Forward        |
/// | 3     | x    | x    | 4    | `1110`  | Forward        |
/// | 4     |      |      | 5    | `11110` | Backward       |
/// | 5     | x    |      | 5    | `11111` | Backward       |
///
/// The decode is a count of leading `1` bits up to 4: 0 → row 0;
/// 1 → row 1; 2 → row 2; 3 → row 3; 4 → consult one more bit
/// (`0` → row 4, `1` → row 5). Every legal 1..=5 bit prefix is a
/// Table M.1 codeword; there is no unknown / forbidden prefix shape
/// for any leading bit pattern that terminates the run of `1`s within
/// four reads. The only error this function returns is
/// [`Error::UnexpectedEof`].
pub fn parse_modb_annex_m(reader: &mut BitReader<'_>) -> Result<ModbAnnexM> {
    // Count the leading `1` bits up to 4. A terminating `0` within
    // that run resolves rows 0..=3. A full run of four `1` bits
    // requires one more bit to disambiguate rows 4 vs 5.
    let mut ones = 0u32;
    while ones < 4 {
        let bit = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
        if !bit {
            return Ok(match ones {
                0 => ModbAnnexM::BidirNoCbpbNoMvdb,
                1 => ModbAnnexM::BidirCbpbNoMvdb,
                2 => ModbAnnexM::ForwardNoCbpbMvdb,
                3 => ModbAnnexM::ForwardCbpbMvdb,
                _ => unreachable!("ones is < 4 in this branch"),
            });
        }
        ones += 1;
    }
    // Four leading `1` bits consumed; tail bit selects rows 4 / 5.
    let tail = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    Ok(if tail {
        ModbAnnexM::BackwardCbpbNoMvdb
    } else {
        ModbAnnexM::BackwardNoCbpbNoMvdb
    })
}

/// Decode a §5.3.4 CBPB 6-bit fixed-length codeword.
///
/// Returns the six bits as a `u8` in the low six bits, with bit `5`
/// (the MSB of the field) carrying B-block number 1's CBPBN per the
/// §5.3.4 "the utmost left bit of CBPB corresponding with block
/// number 1" convention. Use [`cbpb_block_present`] to query an
/// individual block's CBPBN bit by 1-based block number without
/// re-deriving the mapping.
///
/// On `Err(Error::UnexpectedEof)` the reader's position is
/// unspecified (the field is fixed-length so any failure indicates
/// fewer than 6 bits remained).
pub fn parse_cbpb(reader: &mut BitReader<'_>) -> Result<u8> {
    let raw = reader
        .read_u32(CBPB_BITS)
        .map_err(|_| Error::UnexpectedEof)?;
    Ok(raw as u8)
}

/// `true` iff the §5.3.4 CBPB pattern marks B-block `block_number`
/// (1-based per Figure 5: 1..=4 luma in Y raster order, 5 = Cb,
/// 6 = Cr) as carrying at least one non-zero coefficient.
///
/// Returns `false` for any `block_number` outside `1..=6` (defensive;
/// the spec's block-numbering domain is exactly six).
pub fn cbpb_block_present(cbpb: u8, block_number: u32) -> bool {
    if !(1..=6).contains(&block_number) {
        return false;
    }
    // Block 1 → bit 5 (MSB of the 6-bit field), block 2 → bit 4, ...,
    // block 6 → bit 0.
    let bit_index = 6 - block_number;
    ((cbpb >> bit_index) & 1) != 0
}

/// Decode the §5.3.9 MVDB (Motion Vector Data for B-macroblock) pair.
///
/// Per §5.3.9: "MVDB is only present in PB-frames or Improved PB-frames
/// mode if indicated by MODB, and consists of a variable length codeword
/// for the horizontal component followed by a variable length codeword
/// for the vertical component of each vector. Variable length codes are
/// given in Table 14." Table 14 is the same MVD VLC the baseline
/// §5.3.7 MVD parser already decodes; this function composes that
/// primitive twice (horizontal first, then vertical) and packs the
/// result into the existing [`Mvd`] type.
///
/// The returned components are in **half-pel units** (the spec's
/// "Vector Differences" column scaled by 2 to keep the type integral),
/// matching the convention [`Mvd`] uses for §5.3.7 MVD and the
/// Annex F §F.2 MVD2-4 fields.
///
/// The caller (the macroblock-layer driver) is responsible for gating
/// the invocation per §5.3.9 — i.e. only calling this primitive after
/// MODB indicates MVDB presence ([`ModbPresence::has_mvdb`] or the
/// Annex M [`ModbAnnexM::has_mvdb`] accessor returns `true`). This
/// primitive itself does not consult MODB.
///
/// On `Err(Error::UnexpectedEof)` the reader's position is unspecified;
/// truncation may occur mid-horizontal-component or mid-vertical. On
/// `Err(Error::BadMvdCode)` an unknown Table 14 prefix was encountered
/// (the read MVD VLC machinery scans the full 13-bit codeword domain
/// before giving up).
pub fn parse_mvdb(reader: &mut BitReader<'_>) -> Result<Mvd> {
    let dx_half = decode_mvd_component(reader)?;
    let dy_half = decode_mvd_component(reader)?;
    Ok(Mvd { dx_half, dy_half })
}

/// §G.4 forward / backward motion-vector pair for one B-picture
/// 8×8 luminance block, **per component** (called once for the
/// horizontal component and once for the vertical).
///
/// Inputs:
///
/// * `p_mv` — the P-picture vector component for the corresponding
///   luminance block, in half-pel units (§6.1.1 / §F.2 convention).
///   "If only one vector per macroblock is transmitted, MV has the
///   same value for each of the four 8 × 8 luminance blocks" — the
///   caller is responsible for selecting the correct per-block value
///   (the macroblock-layer driver dispatches per [`crate::motion::LumaBlockIndex`]
///   when Annex F §F.2 INTER4V is active and replicates the single
///   MV across all four blocks otherwise).
/// * `mvd` — the §5.3.9 MVDB delta component. `None` covers the
///   "MVDB is not present, MVD is set to zero" branch (§G.4 first
///   half: MODB row 0 of Table 11 or MODB rows 0 / 1 / 3 / 4 / 5 of
///   the Annex M Table M.1 with `has_mvdb() == false`). `Some(d)`
///   carries the delta from a successful [`parse_mvdb`] (or the
///   replicated single-MVDB pair §G.4 paragraph 4 prescribes: "If
///   MVDB is present, the same MVD given by MVDB is used for each of
///   the four luminance B-blocks").
/// * `trb` — §5.1.22 Temporal Reference for B-pictures in PB-frames,
///   3 or 5 bits wide on the wire (`trb` in `[1, 7]` for CIF clock
///   frequency, `[1, 31]` for a custom picture clock frequency per
///   §5.1.22 last sentence).
/// * `trd` — the §G.4 P-to-P temporal-reference increment. Per §G.4
///   first paragraph: "If TRD is negative, then TRD = TRD + d where
///   d = 256 for CIF picture frequency and 1024 for any custom
///   picture clock frequency"; this wrap is the caller's
///   responsibility — `trd` here must be positive (the spec's
///   formulas are undefined for non-positive TRD, and a zero TRD
///   would yield a division-by-zero).
///
/// Returns `(mvf, mvb)` in half-pel units. The wrap of `MVF` into the
/// §6.1.1 permitted range `[-32, 31]` (or `[-63, 63]` under UMV) is
/// **not** applied here — §G.4 paragraph 4 says "Advantage is taken
/// of the fact that the range of values for MVF is constrained";
/// that constraint is enforced by the encoder choosing the MVDB pair
/// that lands within range, and decoders simply compute the formula
/// as-is. Callers that need the §6.1.1 wrapped form can post-process
/// via [`crate::motion::reconstruct_mv_component`] /
/// [`crate::motion::reconstruct_mv_component_umv`] but §G.4 itself
/// does not require it.
///
/// # Panics
///
/// Panics if `trd == 0`. §G.4 does not define behaviour for a zero
/// temporal-reference increment (the formula's denominator would be
/// zero); the panic surfaces this caller-error rather than silently
/// returning a garbage half-pel pair.
pub fn pb_b_vectors(p_mv: i32, mvd: Option<i32>, trb: i32, trd: i32) -> (i32, i32) {
    assert!(trd != 0, "§G.4 requires non-zero TRD");
    // Rust signed `/` is truncation toward zero, matching §G.4's
    // "/ means division by truncation".
    let mvf = (trb * p_mv) / trd + mvd.unwrap_or(0);
    let mvb = if mvd.is_some_and(|d| d != 0) {
        // §G.4: "MVB = MVF - MV   if MVD is unequal to 0".
        mvf - p_mv
    } else {
        // §G.4: "MVB = ((TRB - TRD) × MV) / TRD   if MVD is equal to 0".
        // This branch also covers the §G.4 "If MVDB is not present,
        // MVD is set to zero" case (caller passes `None`).
        ((trb - trd) * p_mv) / trd
    };
    (mvf, mvb)
}

/// §G.4 forward / backward vector pair for one B-picture 8×8
/// luminance block as a [`MotionVector`] pair. Two-component
/// composition of [`pb_b_vectors`] — `pb_b_vectors(p_mv.dx_half, …)`
/// for the horizontal axis and `pb_b_vectors(p_mv.dy_half, …)` for
/// the vertical. Both `MVF` and `MVB` come back in half-pel units.
///
/// Per §G.4 paragraph 4 ("If MVDB is present, the same MVD given by
/// MVDB is used for each of the four luminance B-blocks within the
/// macroblock"), the same `mvd` value is intended to be reused for
/// each of the four 8×8 luma block invocations under both the
/// one-MV-per-MB and the §F.2 four-MV-per-MB cases. The caller (the
/// macroblock-layer driver) selects the per-block `p_mv` and applies
/// the same `mvd` across all four invocations.
///
/// See [`pb_b_vectors`] for the per-component formulas and for the
/// `trd == 0` panic condition (which propagates from the underlying
/// per-component routine).
pub fn pb_b_vector(
    p_mv: MotionVector,
    mvd: Option<Mvd>,
    trb: i32,
    trd: i32,
) -> (MotionVector, MotionVector) {
    let (dx_mvd, dy_mvd) = match mvd {
        Some(m) => (Some(m.dx_half as i32), Some(m.dy_half as i32)),
        None => (None, None),
    };
    let (mvf_dx, mvb_dx) = pb_b_vectors(p_mv.dx_half, dx_mvd, trb, trd);
    let (mvf_dy, mvb_dy) = pb_b_vectors(p_mv.dy_half, dy_mvd, trb, trd);
    (
        MotionVector {
            dx_half: mvf_dx,
            dy_half: mvf_dy,
        },
        MotionVector {
            dx_half: mvb_dx,
            dy_half: mvb_dy,
        },
    )
}

/// §G.4 final two paragraphs — chroma B-vector for a macroblock,
/// derived from the four luminance B-vectors. Per §G.4:
///
/// > For chrominance blocks, MVF is derived by calculating the sum of
/// > the four corresponding luminance MVF vectors and dividing this
/// > sum by 8; the resulting sixteenth pixel resolution vector
/// > components are modified towards the nearest half-pixel position
/// > as indicated in Table F.1. MVB for chrominance is derived by
/// > calculating the sum of the four corresponding luminance MVB
/// > vectors and dividing this sum by 8; …
///
/// The "sum of four luma half-pel components → Table F.1 nearest
/// half-pel" transform is exactly the §F.2 chroma vector for a
/// four-MV macroblock, so this function delegates per-component to
/// [`crate::motion::chroma_mv_component_4mv`] (which sums in half-pel
/// units, recovers the sixteenth-pel fraction via `mag % 16`, and
/// snaps via the in-tree Table F.1 transcription). Returns the chroma
/// MVF / MVB pair, both in [`MotionVector`] half-pel units, applied
/// uniformly to both Cb and Cr blocks of the macroblock per §G.4's
/// invocation ("for chrominance blocks").
///
/// The two `&[MotionVector; 4]` slices are the §G.4 "four
/// corresponding luminance MVF vectors" and "four corresponding
/// luminance MVB vectors" respectively, in the [`crate::motion::LumaBlockIndex`]
/// ordering (block 0 top-left, block 1 top-right, block 2
/// bottom-left, block 3 bottom-right) the §F.2 four-MV neighbourhood
/// uses.
pub fn pb_b_chroma_vector(
    luma_mvf: &[MotionVector; 4],
    luma_mvb: &[MotionVector; 4],
) -> (MotionVector, MotionVector) {
    let sum_x_mvf = luma_mvf.iter().map(|mv| mv.dx_half).sum::<i32>();
    let sum_y_mvf = luma_mvf.iter().map(|mv| mv.dy_half).sum::<i32>();
    let sum_x_mvb = luma_mvb.iter().map(|mv| mv.dx_half).sum::<i32>();
    let sum_y_mvb = luma_mvb.iter().map(|mv| mv.dy_half).sum::<i32>();
    let mvf = MotionVector {
        dx_half: chroma_mv_component_4mv(sum_x_mvf),
        dy_half: chroma_mv_component_4mv(sum_y_mvf),
    };
    let mvb = MotionVector {
        dx_half: chroma_mv_component_4mv(sum_x_mvb),
        dy_half: chroma_mv_component_4mv(sum_y_mvb),
    };
    (mvf, mvb)
}

/// §G.5 per-axis bidirectional-prediction extent for a B-block, in
/// macroblock-local (luminance) or block-local (chrominance) pixel
/// coordinates.
///
/// Returns the inclusive `[lo, hi]` range of pixel positions along
/// one axis for which the §G.5 backward vector points **inside**
/// PREC (i.e. for which §G.5 prescribes bidirectional prediction),
/// or `None` if the range is empty (no bidirectional pixels along
/// this axis for this block, so the whole block is forward-only by
/// the §G.5 axis-product rule).
///
/// Inputs:
///
/// * `mvb_component` — one component of the §G.4 backward vector
///   `MVB` for this 8 × 8 block, in half-pel units (§6.1.1
///   convention); the same component is passed for both luma and
///   chroma blocks.
/// * `block_lo` / `block_hi` — the inclusive pixel-coordinate
///   bounds of the 8 × 8 block along this axis, in the relevant
///   coordinate space:
///   - luma sub-block `(nh, nv)` of a macroblock: `block_lo = n*8`,
///     `block_hi = n*8 + 7` where `n` is `nh` (for the horizontal
///     axis) or `nv` (for the vertical axis); coordinates run
///     0..=15 inside the macroblock;
///   - chroma block of a macroblock: `block_lo = 0`, `block_hi = 7`;
///     coordinates run 0..=7 inside the chroma block.
///
/// The returned range is `i ∈ [max(block_lo, (-mvb+1)/2),
/// min(block_hi, REF_MAX - (mvb+1)/2)]` where `REF_MAX` is `15` for
/// luma blocks (§G.5's `15` is the macroblock-wide pixel bound, the
/// same for both nh=0 and nh=1 luma sub-blocks since both belong to
/// the same 16-pixel macroblock) and `7` for the 8 × 8 chroma block
/// (the 8-pixel chroma block is the whole PREC chroma plane).
///
/// This asymmetry between luma and chroma comes straight from §G.5:
/// for luma the §G.5 `15` is the macroblock-wide upper pixel bound
/// (the four 8 × 8 luma blocks together span 0..=15, so PREC has a
/// "right edge" at 15 for both nh=0 and nh=1 blocks); for chroma
/// the upper pixel bound is `7` because the 8 × 8 chroma block is
/// the whole PREC chroma plane.
///
/// To spare the caller the §G.5 reading lift, [`pb_b_bidir_luma_block_extent`]
/// and [`pb_b_bidir_chroma_extent`] wrap this primitive with the
/// correct block bounds for the two cases.
///
/// "/" is signed integer division by truncation toward zero
/// (matching the §G.5 C expression `(-mh+1)/2`).
///
/// # Panics
///
/// Panics if `block_lo > block_hi`. §G.5 invokes the primitive only
/// for non-empty 8 × 8 blocks.
pub fn pb_b_bidir_extent_component(
    mvb_component: i32,
    block_lo: i32,
    block_hi: i32,
    ref_max: i32,
) -> Option<(i32, i32)> {
    assert!(block_lo <= block_hi, "§G.5 block extent must be non-empty");
    // §G.5: lower bound is `nh*8 + max(0, (-mh+1)/2 - nh*8)`, which
    // equals `max(nh*8, (-mh+1)/2)`. Upper bound is `nh*8 + min(7,
    // 15 - (mh+1)/2 - nh*8)`, which equals `min(nh*8 + 7, 15 -
    // (mh+1)/2)`. Generalising `15` to `ref_max` covers chroma's
    // 0..=7 form (where `7` plays the role of `15`).
    //
    // The `(-mh+1)/2` and `(mh+1)/2` C expressions evaluate with
    // truncation toward zero, the same as Rust's signed `/`.
    let lo = block_lo.max((-mvb_component + 1) / 2);
    let hi = block_hi.min(ref_max - (mvb_component + 1) / 2);
    if lo <= hi {
        Some((lo, hi))
    } else {
        None
    }
}

/// §G.5 bidirectional-prediction rectangle for one 8 × 8 luma
/// sub-block at `(nh, nv)` of a B-block's macroblock.
///
/// Returns `Some(((i_lo, i_hi), (j_lo, j_hi)))` — the inclusive 2-D
/// pixel-coordinate rectangle in macroblock-local coordinates
/// `(0..=15, 0..=15)` for which §G.5 prescribes bidirectional
/// prediction; the row count is `i_hi - i_lo + 1` and the column
/// count is `j_hi - j_lo + 1`. Pixels of the 8 × 8 block outside
/// this rectangle are forward-predicted only per §G.5's "all other
/// pixels" clause.
///
/// Returns `None` if the rectangle is empty along either axis —
/// i.e. the §G.5 backward vector `MVB` points fully outside PREC
/// for this sub-block. In that case the whole 8 × 8 sub-block is
/// forward-predicted only.
///
/// `mvb` is the §G.4 backward vector for the 8 × 8 luma sub-block
/// (each of the four sub-blocks has its own pair of §G.4 vectors
/// even in single-MV-per-MB mode, per §G.4 paragraph 4 — the four
/// luma blocks share one P-MV but the §G.4 formula is applied
/// per-block); per §G.5's `mh(nh,nv)` / `mv(nh,nv)` notation the
/// `(nh, nv)` argument selects which sub-block is being processed.
///
/// # Panics
///
/// Panics if `nh > 1` or `nv > 1`. §G.5 only enumerates the four
/// `(0, 0)`, `(0, 1)`, `(1, 0)`, `(1, 1)` sub-blocks.
pub fn pb_b_bidir_luma_block_extent(
    mvb: MotionVector,
    nh: u8,
    nv: u8,
) -> Option<((i32, i32), (i32, i32))> {
    assert!(nh <= 1, "§G.5 luma sub-block nh must be 0 or 1");
    assert!(nv <= 1, "§G.5 luma sub-block nv must be 0 or 1");
    // §G.5 luma: `REF_MAX = 15` (right-edge bound of the 16-pixel
    // macroblock; same for both nh=0 and nh=1 since both luma
    // sub-blocks belong to the same macroblock and the §G.5
    // bidirectional region is bounded by the macroblock as a whole,
    // not by the 8-pixel sub-block).
    let nh_i = nh as i32;
    let nv_i = nv as i32;
    let i_range = pb_b_bidir_extent_component(mvb.dx_half, nh_i * 8, nh_i * 8 + 7, 15)?;
    let j_range = pb_b_bidir_extent_component(mvb.dy_half, nv_i * 8, nv_i * 8 + 7, 15)?;
    Some((i_range, j_range))
}

/// §G.5 bidirectional-prediction rectangle for the 8 × 8 chroma
/// block of a B-block's macroblock.
///
/// Returns `Some(((i_lo, i_hi), (j_lo, j_hi)))` — the inclusive 2-D
/// pixel-coordinate rectangle in chroma-block-local coordinates
/// `(0..=7, 0..=7)` for which §G.5 prescribes bidirectional
/// prediction. Pixels of the 8 × 8 chroma block outside this
/// rectangle are forward-predicted only per §G.5's "all other
/// pixels" clause.
///
/// Returns `None` if the §G.5 chroma backward vector points fully
/// outside the 8 × 8 PREC chroma block along either axis.
///
/// `mvc` is the §G.4 chroma backward vector for the macroblock
/// (one chroma vector per macroblock, applied uniformly to both Cb
/// and Cr blocks per §G.4 final paragraph); the same `mvc` is
/// invoked once for Cb and once for Cr with this primitive.
pub fn pb_b_bidir_chroma_extent(mvc: MotionVector) -> Option<((i32, i32), (i32, i32))> {
    // §G.5 chroma: block extent is [0, 7] and REF_MAX is 7 (the
    // 8-pixel chroma block is the whole PREC chroma plane).
    let i_range = pb_b_bidir_extent_component(mvc.dx_half, 0, 7, 7)?;
    let j_range = pb_b_bidir_extent_component(mvc.dy_half, 0, 7, 7)?;
    Some((i_range, j_range))
}

/// §G.5 per-pixel bidirectional-prediction sample average.
///
/// Per §G.5: "Bidirectional prediction \[…\] is obtained as the
/// average of the forward prediction using MVF relative to the
/// previous decoded picture, and the backward prediction using MVB
/// relative to PREC. The average is calculated by dividing the sum
/// of the two predictions by two (division by truncation)."
///
/// `forward` is the forward-prediction sample (motion-compensated
/// from the previous decoded picture by MVF) and `backward` is the
/// backward-prediction sample (motion-compensated from PREC by MVB);
/// both are clipped `u8` luma- or chroma-plane samples per §6.3.2.
/// The return value is the §G.5 bidirectional prediction sample.
///
/// "Division by truncation" is signed integer division toward zero;
/// since both operands are non-negative the sum (`u16`-wide so no
/// `u8` overflow) divides cleanly by two — `(forward + backward) >>
/// 1` is the same value but `/ 2` matches §G.5's literal wording.
/// The cast back to `u8` cannot overflow: `(255 + 255) / 2 = 255`.
///
/// This is the per-pixel primitive; the caller iterates it over the
/// rectangle returned by [`pb_b_bidir_luma_block_extent`] (luma) or
/// [`pb_b_bidir_chroma_extent`] (chroma), using the forward sample
/// for every pixel outside that rectangle per §G.5's "all other
/// pixels" clause.
#[inline]
pub fn pb_b_bidir_pixel(forward: u8, backward: u8) -> u8 {
    let sum = forward as u16 + backward as u16;
    (sum / 2) as u8
}

/// §G.5 bidirectional-prediction blend over one 8 × 8 B-block.
///
/// Given the per-axis bidirectional rectangle returned by
/// [`pb_b_bidir_luma_block_extent`] (luma) or
/// [`pb_b_bidir_chroma_extent`] (chroma), composes the §G.5 split:
/// pixels inside the rectangle get the [`pb_b_bidir_pixel`] average
/// of the forward and backward sample at that position; pixels
/// outside it get the forward sample only.
///
/// The `i`-axis is the horizontal axis ("rows of pixels at a given
/// `j`") and the `j`-axis is the vertical axis, matching §G.5's
/// `(i, j)` loop ordering. The eight `forward` rows / eight `backward`
/// rows are addressed by `[j][i]` in block-local coordinates:
/// `j ∈ [block_j_origin, block_j_origin+7]` indexes the row,
/// `i ∈ [block_i_origin, block_i_origin+7]` indexes the column,
/// where `(block_i_origin, block_j_origin)` is the block's origin in
/// the coordinate system the §G.5 rectangle uses
/// (macroblock-local `(0..=15)` for luma; block-local `(0..=7)` for
/// chroma). Passing `None` for `bidir_extent` means §G.5's
/// bidirectional region is empty (the whole block is forward-only),
/// matching the `None` return from the extent primitives.
///
/// Returns an 8 × 8 array of §G.5 prediction samples in the same
/// row-major `[j][i]` order as the inputs.
///
/// # Panics
///
/// Panics if `bidir_extent` ranges are not contained in the 8 × 8
/// block addressed by `(block_i_origin, block_j_origin)`. §G.5
/// invokes the primitive only with rectangles produced by the
/// extent primitives, which respect the block bounds.
pub fn pb_b_blend_block(
    forward: &[[u8; 8]; 8],
    backward: &[[u8; 8]; 8],
    bidir_extent: Option<((i32, i32), (i32, i32))>,
    block_i_origin: i32,
    block_j_origin: i32,
) -> [[u8; 8]; 8] {
    let mut out = *forward;
    let Some(((i_lo, i_hi), (j_lo, j_hi))) = bidir_extent else {
        // §G.5 "all other pixels" — the whole block is forward-only.
        return out;
    };
    // Spec invariant: the §G.5 rectangle stays inside the 8 × 8
    // block (the extent primitives clamp to that block).
    assert!(
        i_lo >= block_i_origin && i_hi <= block_i_origin + 7,
        "§G.5 i-range must lie inside the 8 × 8 block"
    );
    assert!(
        j_lo >= block_j_origin && j_hi <= block_j_origin + 7,
        "§G.5 j-range must lie inside the 8 × 8 block"
    );
    for j in j_lo..=j_hi {
        let jb = (j - block_j_origin) as usize;
        for i in i_lo..=i_hi {
            let ib = (i - block_i_origin) as usize;
            out[jb][ib] = pb_b_bidir_pixel(forward[jb][ib], backward[jb][ib]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn finish_aligned(mut w: BitWriter) -> Vec<u8> {
        while !w.is_byte_aligned() {
            w.write_bit(false);
        }
        w.finish()
    }

    /// Table 11 row 0 (code `0`, 1 bit) decodes to
    /// [`ModbPresence::None`].
    #[test]
    fn modb_code_0_is_none() {
        let mut w = BitWriter::new();
        w.write_bit(false);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("parse");
        assert_eq!(modb, ModbPresence::None);
        assert!(!modb.has_cbpb());
        assert!(!modb.has_mvdb());
        assert_eq!(modb.code_bits(), 1);
        // Reader sits at bit position 1 (the bit after the 1-bit
        // codeword).
        assert_eq!(r.bit_position(), 1);
    }

    /// Table 11 row 1 (code `10`, 2 bits) decodes to
    /// [`ModbPresence::MvdbOnly`].
    #[test]
    fn modb_code_10_is_mvdb_only() {
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("parse");
        assert_eq!(modb, ModbPresence::MvdbOnly);
        assert!(!modb.has_cbpb());
        assert!(modb.has_mvdb());
        assert_eq!(modb.code_bits(), 2);
        assert_eq!(r.bit_position(), 2);
    }

    /// Table 11 row 2 (code `11`, 2 bits) decodes to
    /// [`ModbPresence::CbpbAndMvdb`].
    #[test]
    fn modb_code_11_is_cbpb_and_mvdb() {
        let mut w = BitWriter::new();
        w.write_u32(0b11, 2);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("parse");
        assert_eq!(modb, ModbPresence::CbpbAndMvdb);
        assert!(modb.has_cbpb());
        assert!(modb.has_mvdb());
        assert_eq!(modb.code_bits(), 2);
        assert_eq!(r.bit_position(), 2);
    }

    /// MODB starting with a `1` followed by EOF must yield
    /// [`Error::UnexpectedEof`]. We construct a 1-byte buffer whose
    /// last bit is the leading `1` of MODB and burn the seven
    /// preceding bits, so the leading `1` is read but no more bits
    /// remain in the buffer.
    #[test]
    fn modb_truncated_after_lead_one_returns_eof() {
        let data = [0b0000_0001u8];
        let mut r = BitReader::new(&data);
        r.read_u32(7).expect("seven padding bits");
        let err = parse_modb(&mut r).expect_err("second bit missing");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// MODB on an empty buffer yields [`Error::UnexpectedEof`]
    /// immediately (no bits at all to read).
    #[test]
    fn modb_empty_buffer_returns_eof() {
        let data: [u8; 0] = [];
        let mut r = BitReader::new(&data);
        let err = parse_modb(&mut r).expect_err("empty");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// CBPB all-zero pattern decodes to `0` and leaves the reader
    /// 6 bits in.
    #[test]
    fn cbpb_all_zero_pattern() {
        let mut w = BitWriter::new();
        w.write_u32(0b00_0000, 6);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let cbpb = parse_cbpb(&mut r).expect("parse");
        assert_eq!(cbpb, 0);
        assert_eq!(r.bit_position(), 6);
        // No B-block has a non-zero coefficient.
        for n in 1..=6 {
            assert!(!cbpb_block_present(cbpb, n));
        }
    }

    /// CBPB all-ones pattern decodes to `0x3F` and marks every
    /// B-block as carrying a non-zero coefficient.
    #[test]
    fn cbpb_all_one_pattern() {
        let mut w = BitWriter::new();
        w.write_u32(0b11_1111, 6);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let cbpb = parse_cbpb(&mut r).expect("parse");
        assert_eq!(cbpb, 0b11_1111);
        for n in 1..=6 {
            assert!(cbpb_block_present(cbpb, n));
        }
    }

    /// CBPB single-bit-set patterns isolate each B-block in turn:
    /// the §5.3.4 "utmost left bit ↔ block 1" convention means
    /// block 1 corresponds to bit 5, block 2 to bit 4, …, block 6
    /// to bit 0.
    #[test]
    fn cbpb_single_bit_per_block_isolates_correct_block() {
        for target_block in 1..=6u32 {
            // bit_index = 6 - block_number per §5.3.4.
            let bit_index = 6 - target_block;
            let pattern = 1u32 << bit_index;
            let mut w = BitWriter::new();
            w.write_u32(pattern, 6);
            let data = finish_aligned(w);
            let mut r = BitReader::new(&data);
            let cbpb = parse_cbpb(&mut r).expect("parse");
            assert_eq!(
                cbpb, pattern as u8,
                "block {} pattern round-trip",
                target_block
            );
            for query_block in 1..=6 {
                let expected = query_block == target_block;
                assert_eq!(
                    cbpb_block_present(cbpb, query_block),
                    expected,
                    "block {} present query for target block {} ",
                    query_block,
                    target_block
                );
            }
        }
    }

    /// CBPB block-1 vs block-6 specifically: the §5.3.4 "utmost left
    /// bit corresponds with block number 1" convention pins block 1
    /// to bit 5 (`0b10_0000`) and block 6 to bit 0 (`0b00_0001`).
    /// This test is a deliberate static check of those two endpoints
    /// so a future refactor of [`cbpb_block_present`] cannot silently
    /// invert the mapping.
    #[test]
    fn cbpb_block_1_is_msb_block_6_is_lsb() {
        assert!(cbpb_block_present(0b10_0000, 1));
        assert!(!cbpb_block_present(0b10_0000, 2));
        assert!(!cbpb_block_present(0b00_0001, 1));
        assert!(cbpb_block_present(0b00_0001, 6));
    }

    /// CBPB on a truncated buffer (fewer than 6 bits remaining)
    /// returns [`Error::UnexpectedEof`].
    #[test]
    fn cbpb_truncated_returns_eof() {
        // Two-byte buffer with cursor 11 bits in leaves only 5 bits
        // before EOF — too few for CBPB.
        let data = [0u8; 2];
        let mut r = BitReader::new(&data);
        r.read_u32(11).expect("burn 11 bits");
        let err = parse_cbpb(&mut r).expect_err("five bits left");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// CBPB on an empty buffer returns [`Error::UnexpectedEof`].
    #[test]
    fn cbpb_empty_buffer_returns_eof() {
        let data: [u8; 0] = [];
        let mut r = BitReader::new(&data);
        let err = parse_cbpb(&mut r).expect_err("empty");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// Out-of-range block numbers return `false` (defensive).
    #[test]
    fn cbpb_block_present_out_of_range_is_false() {
        for n in [0u32, 7, 8, 100, u32::MAX] {
            assert!(!cbpb_block_present(0b11_1111, n));
        }
    }

    /// `code_bits()` agrees with the bit advance observed by the
    /// reader for all three Table 11 entries (sanity check that the
    /// reader's bit_position and the tag-reported width stay
    /// consistent).
    #[test]
    fn modb_code_bits_matches_reader_advance() {
        for (raw_code, code_bits, expected) in [
            (0b0u32, 1u32, ModbPresence::None),
            (0b10, 2, ModbPresence::MvdbOnly),
            (0b11, 2, ModbPresence::CbpbAndMvdb),
        ] {
            let mut w = BitWriter::new();
            w.write_u32(raw_code, code_bits);
            let data = finish_aligned(w);
            let mut r = BitReader::new(&data);
            let modb = parse_modb(&mut r).expect("parse");
            assert_eq!(modb, expected);
            assert_eq!(modb.code_bits(), code_bits);
            assert_eq!(r.bit_position() as u32, code_bits);
        }
    }

    /// Table M.1 row 0 (`0`, 1 bit) → bidirectional, no CBPB, no MVDB.
    #[test]
    fn modb_annex_m_row_0_bidir_no_cbpb_no_mvdb() {
        let mut w = BitWriter::new();
        w.write_bit(false);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let tag = parse_modb_annex_m(&mut r).expect("parse");
        assert_eq!(tag, ModbAnnexM::BidirNoCbpbNoMvdb);
        assert!(!tag.has_cbpb());
        assert!(!tag.has_mvdb());
        assert_eq!(tag.coding_mode(), BpbCodingMode::Bidirectional);
        assert_eq!(tag.code_bits(), 1);
        assert_eq!(r.bit_position(), 1);
    }

    /// Table M.1 row 1 (`10`, 2 bits) → bidirectional, CBPB only.
    #[test]
    fn modb_annex_m_row_1_bidir_cbpb_only() {
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let tag = parse_modb_annex_m(&mut r).expect("parse");
        assert_eq!(tag, ModbAnnexM::BidirCbpbNoMvdb);
        assert!(tag.has_cbpb());
        assert!(!tag.has_mvdb());
        assert_eq!(tag.coding_mode(), BpbCodingMode::Bidirectional);
        assert_eq!(tag.code_bits(), 2);
        assert_eq!(r.bit_position(), 2);
    }

    /// Table M.1 row 2 (`110`, 3 bits) → forward, MVDB only.
    #[test]
    fn modb_annex_m_row_2_forward_mvdb_only() {
        let mut w = BitWriter::new();
        w.write_u32(0b110, 3);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let tag = parse_modb_annex_m(&mut r).expect("parse");
        assert_eq!(tag, ModbAnnexM::ForwardNoCbpbMvdb);
        assert!(!tag.has_cbpb());
        assert!(tag.has_mvdb());
        assert_eq!(tag.coding_mode(), BpbCodingMode::Forward);
        assert_eq!(tag.code_bits(), 3);
        assert_eq!(r.bit_position(), 3);
    }

    /// Table M.1 row 3 (`1110`, 4 bits) → forward with CBPB + MVDB.
    #[test]
    fn modb_annex_m_row_3_forward_cbpb_and_mvdb() {
        let mut w = BitWriter::new();
        w.write_u32(0b1110, 4);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let tag = parse_modb_annex_m(&mut r).expect("parse");
        assert_eq!(tag, ModbAnnexM::ForwardCbpbMvdb);
        assert!(tag.has_cbpb());
        assert!(tag.has_mvdb());
        assert_eq!(tag.coding_mode(), BpbCodingMode::Forward);
        assert_eq!(tag.code_bits(), 4);
        assert_eq!(r.bit_position(), 4);
    }

    /// Table M.1 row 4 (`11110`, 5 bits) → backward, no CBPB, no MVDB.
    #[test]
    fn modb_annex_m_row_4_backward_no_cbpb_no_mvdb() {
        let mut w = BitWriter::new();
        w.write_u32(0b11110, 5);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let tag = parse_modb_annex_m(&mut r).expect("parse");
        assert_eq!(tag, ModbAnnexM::BackwardNoCbpbNoMvdb);
        assert!(!tag.has_cbpb());
        assert!(!tag.has_mvdb());
        assert_eq!(tag.coding_mode(), BpbCodingMode::Backward);
        assert_eq!(tag.code_bits(), 5);
        assert_eq!(r.bit_position(), 5);
    }

    /// Table M.1 row 5 (`11111`, 5 bits) → backward with CBPB.
    /// §M.2.3 backward prediction does not carry MVDB on the wire, so
    /// even this row leaves MVDB off — only CBPB is present.
    #[test]
    fn modb_annex_m_row_5_backward_cbpb_only() {
        let mut w = BitWriter::new();
        w.write_u32(0b11111, 5);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let tag = parse_modb_annex_m(&mut r).expect("parse");
        assert_eq!(tag, ModbAnnexM::BackwardCbpbNoMvdb);
        assert!(tag.has_cbpb());
        assert!(!tag.has_mvdb());
        assert_eq!(tag.coding_mode(), BpbCodingMode::Backward);
        assert_eq!(tag.code_bits(), 5);
        assert_eq!(r.bit_position(), 5);
    }

    /// Sweep every Table M.1 row and assert the parser's bit advance
    /// agrees with the tag's self-reported width, the parsed tag, and
    /// the §M.2 coding-mode column. Six rows, single oracle.
    #[test]
    fn modb_annex_m_table_m1_round_trip_all_rows() {
        let rows: [(u32, u32, ModbAnnexM, BpbCodingMode, bool, bool); 6] = [
            (
                0b0,
                1,
                ModbAnnexM::BidirNoCbpbNoMvdb,
                BpbCodingMode::Bidirectional,
                false,
                false,
            ),
            (
                0b10,
                2,
                ModbAnnexM::BidirCbpbNoMvdb,
                BpbCodingMode::Bidirectional,
                true,
                false,
            ),
            (
                0b110,
                3,
                ModbAnnexM::ForwardNoCbpbMvdb,
                BpbCodingMode::Forward,
                false,
                true,
            ),
            (
                0b1110,
                4,
                ModbAnnexM::ForwardCbpbMvdb,
                BpbCodingMode::Forward,
                true,
                true,
            ),
            (
                0b11110,
                5,
                ModbAnnexM::BackwardNoCbpbNoMvdb,
                BpbCodingMode::Backward,
                false,
                false,
            ),
            (
                0b11111,
                5,
                ModbAnnexM::BackwardCbpbNoMvdb,
                BpbCodingMode::Backward,
                true,
                false,
            ),
        ];
        for (code, bits, expected_tag, mode, has_cbpb, has_mvdb) in rows {
            let mut w = BitWriter::new();
            w.write_u32(code, bits);
            let data = finish_aligned(w);
            let mut r = BitReader::new(&data);
            let tag = parse_modb_annex_m(&mut r).expect("parse");
            assert_eq!(tag, expected_tag, "code {:b}", code);
            assert_eq!(tag.code_bits(), bits, "code {:b} width", code);
            assert_eq!(r.bit_position() as u32, bits, "reader advance {:b}", code);
            assert_eq!(tag.coding_mode(), mode, "coding mode {:b}", code);
            assert_eq!(tag.has_cbpb(), has_cbpb, "has_cbpb {:b}", code);
            assert_eq!(tag.has_mvdb(), has_mvdb, "has_mvdb {:b}", code);
        }
    }

    /// Annex M MODB on an empty buffer yields UnexpectedEof immediately.
    #[test]
    fn modb_annex_m_empty_buffer_returns_eof() {
        let data: [u8; 0] = [];
        let mut r = BitReader::new(&data);
        let err = parse_modb_annex_m(&mut r).expect_err("empty");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// Annex M MODB starting with three `1` bits followed by EOF
    /// truncates while waiting for the run-terminator bit. Construct
    /// a 1-byte buffer whose last three bits are the leading `111`
    /// of MODB and burn the five preceding bits, so we read `1` /
    /// `1` / `1` then run off the end.
    #[test]
    fn modb_annex_m_truncated_in_run_returns_eof() {
        let data = [0b0000_0111u8];
        let mut r = BitReader::new(&data);
        r.read_u32(5).expect("burn five padding bits");
        let err = parse_modb_annex_m(&mut r).expect_err("run truncated");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// Annex M MODB with four `1` bits but no tail bit truncates on
    /// the row 4 / row 5 disambiguator. Two-byte buffer with eleven
    /// bits burned leaves five bits before EOF; we then write four
    /// `1` bits as the lead, read all four, and run off the end on
    /// the tail bit read. To set this up cleanly we instead build a
    /// fresh `1111`-then-EOF buffer at byte alignment by burning a
    /// known prefix.
    #[test]
    fn modb_annex_m_truncated_at_tail_returns_eof() {
        // Construct a single byte whose last four bits are `1111` and
        // burn the first four bits so the reader sits on the run of
        // four leading `1` bits with nothing after.
        let data = [0b0000_1111u8];
        let mut r = BitReader::new(&data);
        r.read_u32(4).expect("burn four padding bits");
        let err = parse_modb_annex_m(&mut r).expect_err("tail bit missing");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// End-to-end Annex M chain: MODB row 3 (`1110`, forward
    /// CBPB+MVDB) immediately followed by a CBPB pattern that lights
    /// only blocks 2 and 4. Reader sits at bit 10 (4 + 6) after both
    /// parses; CBPB queries return the expected per-block presence.
    #[test]
    fn modb_annex_m_then_cbpb_chain_advances_by_10_bits() {
        let mut w = BitWriter::new();
        w.write_u32(0b1110, 4); // row 3
        w.write_u32(0b01_0100, 6); // CBPB: block 2 (bit 4) + block 4 (bit 2)
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb_annex_m(&mut r).expect("modb");
        assert_eq!(modb, ModbAnnexM::ForwardCbpbMvdb);
        assert!(modb.has_cbpb());
        assert!(modb.has_mvdb());
        let cbpb = parse_cbpb(&mut r).expect("cbpb");
        assert_eq!(cbpb, 0b01_0100);
        assert!(!cbpb_block_present(cbpb, 1));
        assert!(cbpb_block_present(cbpb, 2));
        assert!(!cbpb_block_present(cbpb, 3));
        assert!(cbpb_block_present(cbpb, 4));
        assert!(!cbpb_block_present(cbpb, 5));
        assert!(!cbpb_block_present(cbpb, 6));
        assert_eq!(r.bit_position(), 10);
    }

    /// Cross-check: the Annex M parser is independent of the Annex G
    /// parser. Feeding the Annex G "code = 11" (Table 11 row 2) to
    /// the Annex M parser interprets the same bits as a partial
    /// Annex M codeword — specifically the first two bits of row 3
    /// (`1110`), so the Annex M parser will continue reading. Drive
    /// the bytes deliberately so we can pin the divergence between
    /// the two tables.
    #[test]
    fn modb_annex_m_does_not_share_codewords_with_annex_g() {
        // Annex G code `11` = CbpbAndMvdb (2 bits). Annex M code `1110`
        // is row 3 (4 bits) — extending the Annex G `11` with `10`
        // yields the Annex M row 3 codeword. Feed `11` then `10` to
        // the Annex M parser and assert it consumes all four bits.
        let mut w = BitWriter::new();
        w.write_u32(0b1110, 4);
        let data = finish_aligned(w);
        let mut r_m = BitReader::new(&data);
        let tag_m = parse_modb_annex_m(&mut r_m).expect("annex m");
        assert_eq!(tag_m, ModbAnnexM::ForwardCbpbMvdb);
        assert_eq!(r_m.bit_position(), 4);

        // Same bytes through the Annex G parser stop at bit 2.
        let mut r_g = BitReader::new(&data);
        let tag_g = parse_modb(&mut r_g).expect("annex g");
        assert_eq!(tag_g, ModbPresence::CbpbAndMvdb);
        assert_eq!(r_g.bit_position(), 2);
    }

    /// §5.3.9 MVDB single-bit per component round-trip. The §5.3.7
    /// MVD VLC has the `1` codeword mapped to half-pel value `0`, so
    /// the MVDB pair (`1`, `1`) decodes to `(0, 0)` and consumes
    /// exactly two bits.
    #[test]
    fn mvdb_zero_zero_pair_consumes_two_bits() {
        let mut w = BitWriter::new();
        w.write_bit(true); // dx
        w.write_bit(true); // dy
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mvdb = parse_mvdb(&mut r).expect("parse");
        assert_eq!(mvdb.dx_half, 0);
        assert_eq!(mvdb.dy_half, 0);
        assert_eq!(r.bit_position(), 2);
    }

    /// §5.3.9 MVDB asymmetric pair: dx = +1 (3-bit code `010`),
    /// dy = -1 (3-bit code `011`). Six bits consumed total.
    #[test]
    fn mvdb_plus_one_minus_one_round_trip() {
        let mut w = BitWriter::new();
        w.write_u32(0b010, 3); // dx = +1 per Table 14 idx 33
        w.write_u32(0b011, 3); // dy = -1 per Table 14 idx 31
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mvdb = parse_mvdb(&mut r).expect("parse");
        assert_eq!(mvdb.dx_half, 1);
        assert_eq!(mvdb.dy_half, -1);
        assert_eq!(r.bit_position(), 6);
    }

    /// §5.3.9 MVDB symmetric non-zero pair: dx = dy = -2 (4-bit code
    /// `0011`). Eight bits consumed total.
    #[test]
    fn mvdb_minus_two_minus_two_pair() {
        let mut w = BitWriter::new();
        w.write_u32(0b0011, 4); // dx = -2 per Table 14 idx 30
        w.write_u32(0b0011, 4); // dy = -2
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let mvdb = parse_mvdb(&mut r).expect("parse");
        assert_eq!(mvdb.dx_half, -2);
        assert_eq!(mvdb.dy_half, -2);
        assert_eq!(r.bit_position(), 8);
    }

    /// §5.3.9 MVDB on an empty buffer yields [`Error::UnexpectedEof`]
    /// from the horizontal-component read.
    #[test]
    fn mvdb_empty_buffer_returns_eof() {
        let data: [u8; 0] = [];
        let mut r = BitReader::new(&data);
        let err = parse_mvdb(&mut r).expect_err("empty");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// §5.3.9 MVDB with horizontal present but vertical truncated:
    /// burn seven bits then feed the single `1` bit (dx = 0); the
    /// vertical-component read runs off the end. The error is
    /// [`Error::UnexpectedEof`].
    #[test]
    fn mvdb_truncated_between_components_returns_eof() {
        let data = [0b0000_0001u8];
        let mut r = BitReader::new(&data);
        r.read_u32(7).expect("burn seven padding bits");
        // r is now at bit 7; the next read sees the `1` (dx = 0),
        // then dy_half tries to read from EOF.
        let err = parse_mvdb(&mut r).expect_err("dy missing");
        assert_eq!(err, Error::UnexpectedEof);
    }

    /// §5.3.9 MVDB end-to-end chain composed with the §5.3.3 MODB
    /// row 1 (`10`, MVDB only) primitive: the macroblock-layer
    /// driver's PB-mode wire sequence for MB-type 0..=4 when only
    /// MVDB is present is `MODB(2 bits) + MVDB(dx + dy)`. Feed
    /// MODB `10` plus dx = +1 (`010`) and dy = 0 (`1`) — the reader
    /// consumes exactly 2 + 3 + 1 = 6 bits and reports the expected
    /// `(dx, dy)` pair.
    #[test]
    fn mvdb_after_modb_mvdb_only_chain_advances_by_six_bits() {
        let mut w = BitWriter::new();
        w.write_u32(0b10, 2); // MODB row 1: MVDB only
        w.write_u32(0b010, 3); // dx = +1
        w.write_bit(true); // dy = 0
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("modb");
        assert_eq!(modb, ModbPresence::MvdbOnly);
        assert!(modb.has_mvdb());
        assert!(!modb.has_cbpb());
        let mvdb = parse_mvdb(&mut r).expect("mvdb");
        assert_eq!(mvdb.dx_half, 1);
        assert_eq!(mvdb.dy_half, 0);
        assert_eq!(r.bit_position(), 6);
    }

    /// §5.3.9 MVDB end-to-end chain with the §5.3.3 MODB row 2 / §5.3.4
    /// CBPB. Both CBPB and MVDB are present: the wire layout is
    /// `MODB(2 bits) + CBPB(6 bits) + MVDB(dx + dy)`. Pin the cursor
    /// position after each parser.
    #[test]
    fn mvdb_after_modb_cbpb_and_mvdb_chain() {
        let mut w = BitWriter::new();
        w.write_u32(0b11, 2); // MODB row 2: CBPB + MVDB
        w.write_u32(0b10_1010, 6); // CBPB pattern: blocks 1, 3, 5
        w.write_bit(true); // dx = 0
        w.write_u32(0b010, 3); // dy = +1
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("modb");
        assert_eq!(modb, ModbPresence::CbpbAndMvdb);
        assert!(modb.has_cbpb());
        assert!(modb.has_mvdb());
        assert_eq!(r.bit_position(), 2);
        let cbpb = parse_cbpb(&mut r).expect("cbpb");
        assert_eq!(cbpb, 0b10_1010);
        assert_eq!(r.bit_position(), 8);
        let mvdb = parse_mvdb(&mut r).expect("mvdb");
        assert_eq!(mvdb.dx_half, 0);
        assert_eq!(mvdb.dy_half, 1);
        assert_eq!(r.bit_position(), 12);
    }

    /// §5.3.9 MVDB end-to-end through the Annex M MODB primitive: row
    /// 2 (`110`, forward, MVDB only) followed by a non-zero MVDB pair.
    /// Demonstrates that the §5.3.9 primitive composes identically with
    /// both the §5.3.3 / Table 11 and §M.4 / Table M.1 MODB tags — the
    /// MVDB wire format itself does not change between Annex G and
    /// Annex M (§M.2.2 / §5.3.9 explicitly reuse the §5.3.7 / Table 14
    /// VLC).
    #[test]
    fn mvdb_after_modb_annex_m_forward_chain() {
        let mut w = BitWriter::new();
        w.write_u32(0b110, 3); // Annex M row 2: forward, MVDB only
        w.write_u32(0b010, 3); // dx = +1
        w.write_u32(0b011, 3); // dy = -1
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb_annex_m(&mut r).expect("modb m");
        assert_eq!(modb, ModbAnnexM::ForwardNoCbpbMvdb);
        assert!(modb.has_mvdb());
        assert_eq!(modb.coding_mode(), BpbCodingMode::Forward);
        assert_eq!(r.bit_position(), 3);
        let mvdb = parse_mvdb(&mut r).expect("mvdb");
        assert_eq!(mvdb.dx_half, 1);
        assert_eq!(mvdb.dy_half, -1);
        assert_eq!(r.bit_position(), 9);
    }

    /// §5.3.9 MVDB on a malformed Table 14 codeword returns
    /// [`Error::BadMvdCode`]. Construct a horizontal prefix that
    /// never resolves: thirteen zero bits with no leading `1`.
    #[test]
    fn mvdb_unknown_codeword_returns_bad_mvd_code() {
        let data = [0u8; 2];
        let mut r = BitReader::new(&data);
        let err = parse_mvdb(&mut r).expect_err("all-zero prefix");
        assert_eq!(err, Error::BadMvdCode);
    }

    /// End-to-end: MODB row 2 (`11`) immediately followed by a CBPB
    /// pattern of `0b10_1010` (blocks 1, 3, 5 carry coefficients)
    /// chained into one reader. After both parses, the reader sits
    /// at bit 8 (2 + 6) — the first bit of whatever follows CBPB
    /// in the macroblock layer (CBPY for the eventual PB-mode
    /// driver).
    #[test]
    fn modb_cbpb_chain_advances_reader_by_8_bits() {
        let mut w = BitWriter::new();
        w.write_u32(0b11, 2); // MODB row 2
        w.write_u32(0b10_1010, 6); // CBPB pattern: blocks 1, 3, 5 set
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb(&mut r).expect("modb");
        assert_eq!(modb, ModbPresence::CbpbAndMvdb);
        assert!(modb.has_cbpb());
        let cbpb = parse_cbpb(&mut r).expect("cbpb");
        assert_eq!(cbpb, 0b10_1010);
        assert!(cbpb_block_present(cbpb, 1));
        assert!(!cbpb_block_present(cbpb, 2));
        assert!(cbpb_block_present(cbpb, 3));
        assert!(!cbpb_block_present(cbpb, 4));
        assert!(cbpb_block_present(cbpb, 5));
        assert!(!cbpb_block_present(cbpb, 6));
        assert_eq!(r.bit_position(), 8);
    }

    // ---- §G.4 — Calculation of vectors for the B-picture ---------------

    /// §G.4 baseline — when MV == 0 and MVDB is absent, both MVF and
    /// MVB are zero regardless of (TRB, TRD). The formula reduces to
    /// `(TRB × 0) / TRD == 0` for MVF and `((TRB - TRD) × 0) / TRD == 0`
    /// for MVB.
    #[test]
    fn pb_b_vectors_zero_p_mv_no_mvdb_is_zero_pair() {
        let (mvf, mvb) = pb_b_vectors(0, None, 2, 4);
        assert_eq!(mvf, 0);
        assert_eq!(mvb, 0);
    }

    /// §G.4 middle-of-interval (TRB exactly half of TRD): MVF =
    /// (TRB × MV) / TRD = (1 × 8) / 2 = 4; MVB = ((TRB - TRD) × MV) /
    /// TRD = (-1 × 8) / 2 = -4. The B-picture sits midway between the
    /// previous P-picture and the next P-picture, so MVF and MVB are
    /// equal in magnitude with opposite signs.
    #[test]
    fn pb_b_vectors_mid_interval_symmetric_split() {
        let (mvf, mvb) = pb_b_vectors(8, None, 1, 2);
        assert_eq!(mvf, 4);
        assert_eq!(mvb, -4);
    }

    /// §G.4 close-to-next-P (TRB near TRD): MVF leans toward the full
    /// MV, MVB toward zero. TRB = 3, TRD = 4 → MVF = (3 × 16) / 4 =
    /// 12; MVB = ((3 - 4) × 16) / 4 = -4. The B-picture is closer to
    /// the next P-picture than to the previous one, so the forward
    /// vector is larger than the backward.
    #[test]
    fn pb_b_vectors_three_quarters_split() {
        let (mvf, mvb) = pb_b_vectors(16, None, 3, 4);
        assert_eq!(mvf, 12);
        assert_eq!(mvb, -4);
    }

    /// §G.4 close-to-previous-P (TRB near 0): MVF leans toward zero,
    /// MVB toward -MV. TRB = 1, TRD = 4 → MVF = (1 × 16) / 4 = 4;
    /// MVB = ((1 - 4) × 16) / 4 = -12.
    #[test]
    fn pb_b_vectors_one_quarter_split() {
        let (mvf, mvb) = pb_b_vectors(16, None, 1, 4);
        assert_eq!(mvf, 4);
        assert_eq!(mvb, -12);
    }

    /// §G.4 with non-zero MVDB: the MVB branch flips from the
    /// "((TRB - TRD) × MV) / TRD" formula (used when MVD == 0) to
    /// "MVF - MV" (used when MVD ≠ 0). TRB = 1, TRD = 2, MV = 8,
    /// MVD = 2 → MVF = (1 × 8) / 2 + 2 = 6; MVB = MVF - MV =
    /// 6 - 8 = -2.
    #[test]
    fn pb_b_vectors_nonzero_mvd_uses_mvf_minus_mv_branch() {
        let (mvf, mvb) = pb_b_vectors(8, Some(2), 1, 2);
        assert_eq!(mvf, 6);
        assert_eq!(mvb, -2);
    }

    /// §G.4 with negative MVD: same MVB-branch flip, sign carries
    /// through. MV = 10, MVD = -3, TRB = 1, TRD = 2 → MVF =
    /// (1 × 10) / 2 + (-3) = 5 - 3 = 2; MVB = 2 - 10 = -8.
    #[test]
    fn pb_b_vectors_negative_mvd() {
        let (mvf, mvb) = pb_b_vectors(10, Some(-3), 1, 2);
        assert_eq!(mvf, 2);
        assert_eq!(mvb, -8);
    }

    /// §G.4 explicit "MVD == 0" branch: `Some(0)` matches "MVDB is
    /// present but its value is zero" and therefore takes the
    /// `((TRB - TRD) × MV) / TRD` path, **not** the `MVF - MV` path.
    /// The result must match the `None` (MVDB-absent) case exactly.
    #[test]
    fn pb_b_vectors_explicit_zero_mvd_matches_absent_mvdb() {
        let with_zero = pb_b_vectors(8, Some(0), 1, 2);
        let without = pb_b_vectors(8, None, 1, 2);
        assert_eq!(with_zero, without);
        assert_eq!(with_zero, (4, -4));
    }

    /// §G.4 division-by-truncation behaviour matches Rust's signed
    /// `/`. Negative MV with non-exact TRB/TRD division: MV = -5,
    /// TRB = 1, TRD = 2 → (1 × -5) / 2 = -2 (truncation toward zero,
    /// **not** floor). MVB = ((1 - 2) × -5) / 2 = 5 / 2 = 2.
    #[test]
    fn pb_b_vectors_division_truncates_toward_zero() {
        let (mvf, mvb) = pb_b_vectors(-5, None, 1, 2);
        assert_eq!(mvf, -2); // would be -3 if floor; -2 is truncation
        assert_eq!(mvb, 2);
    }

    /// §G.4 panics on TRD == 0 (the spec's formula is undefined for a
    /// zero temporal-reference increment; a division by zero would
    /// otherwise occur).
    #[test]
    #[should_panic(expected = "non-zero TRD")]
    fn pb_b_vectors_panics_on_zero_trd() {
        let _ = pb_b_vectors(8, None, 1, 0);
    }

    /// [`pb_b_vector`] composes [`pb_b_vectors`] on each axis with
    /// the same (TRB, TRD) and the per-axis Mvd component split. MV
    /// = (8, 16), MVD = (+2, -3), TRB = 1, TRD = 2 → MVF.x = 6, MVF.y
    /// = 5, MVB.x = -2, MVB.y = -11 (MVF - MV per axis since MVD ≠
    /// 0 on both axes).
    #[test]
    fn pb_b_vector_composes_per_axis() {
        let p_mv = MotionVector::new(8, 16);
        let mvd = Mvd {
            dx_half: 2,
            dy_half: -3,
        };
        let (mvf, mvb) = pb_b_vector(p_mv, Some(mvd), 1, 2);
        assert_eq!(mvf, MotionVector::new(6, 5));
        assert_eq!(mvb, MotionVector::new(-2, -11));
    }

    /// [`pb_b_vector`] with `None` MVDB takes the §G.4 MVB =
    /// ((TRB - TRD) × MV) / TRD path on each axis. MV = (8, -16),
    /// TRB = 1, TRD = 2 → MVF = (4, -8); MVB = (-4, 8).
    #[test]
    fn pb_b_vector_no_mvdb_takes_zero_branch_on_both_axes() {
        let p_mv = MotionVector::new(8, -16);
        let (mvf, mvb) = pb_b_vector(p_mv, None, 1, 2);
        assert_eq!(mvf, MotionVector::new(4, -8));
        assert_eq!(mvb, MotionVector::new(-4, 8));
    }

    /// [`pb_b_vector`] with a zero Mvd struct (Some(Mvd{0,0})) must
    /// behave identically to `None` MVDB on both axes (the per-axis
    /// "MVD == 0" check selects the no-MVD branch independently).
    #[test]
    fn pb_b_vector_some_zero_mvd_matches_none() {
        let p_mv = MotionVector::new(10, 14);
        let with_zero = pb_b_vector(
            p_mv,
            Some(Mvd {
                dx_half: 0,
                dy_half: 0,
            }),
            1,
            2,
        );
        let without = pb_b_vector(p_mv, None, 1, 2);
        assert_eq!(with_zero, without);
    }

    /// End-to-end: parse §M.4 MODB row 2 (`110`, forward + MVDB no
    /// CBPB), then §5.3.9 MVDB `(+1, -1)`, then call [`pb_b_vector`]
    /// with the parsed Mvd applied to a (16, 0) P-MV at TRB = 1,
    /// TRD = 2. Verifies the round-trip parse → calculate chain
    /// the macroblock-layer driver will exercise.
    #[test]
    fn pb_b_vector_chained_after_modb_annex_m_and_mvdb_parse() {
        let mut w = BitWriter::new();
        w.write_u32(0b110, 3); // §M.4 Table M.1 row 2 (forward + MVDB)
                               // MVDB dx = +1 — Table 14 entry "+1 in half-pel" sits at
                               // VLC `010` per the established §5.3.7 transcription used in
                               // earlier MVDB tests of this module.
        w.write_u32(0b010, 3);
        // MVDB dy = -1 — Table 14 entry "-1 in half-pel" is VLC `011`.
        w.write_u32(0b011, 3);
        let data = finish_aligned(w);
        let mut r = BitReader::new(&data);
        let modb = parse_modb_annex_m(&mut r).expect("modb");
        assert!(modb.has_mvdb());
        let mvdb = parse_mvdb(&mut r).expect("mvdb");
        assert_eq!(mvdb.dx_half, 1);
        assert_eq!(mvdb.dy_half, -1);
        let p_mv = MotionVector::new(16, 0);
        let (mvf, mvb) = pb_b_vector(p_mv, Some(mvdb), 1, 2);
        // dx axis: MVF = (1 × 16) / 2 + 1 = 9; MVB = 9 - 16 = -7.
        // dy axis: MVF = (1 × 0) / 2 + (-1) = -1; MVB = -1 - 0 = -1.
        assert_eq!(mvf, MotionVector::new(9, -1));
        assert_eq!(mvb, MotionVector::new(-7, -1));
    }

    /// [`pb_b_chroma_vector`] §G.4 paragraph 5 / 6 — four uniform
    /// luma MVF / MVB vectors collapse to the §F.2 chroma-of-4-MV
    /// transform: sum / 16 + sixteenth-pel snap. Four MVF of (8, 0)
    /// sum to dx_sum = 32 → mag = 32, full_chroma_pixels = 2,
    /// sixteenth = 0 → chroma dx = 4. Four MVB of (-4, 0) sum to
    /// dx_sum = -16 → chroma dx = -2.
    #[test]
    fn pb_b_chroma_vector_uniform_luma_collapses_via_table_f1() {
        let luma_mvf = [MotionVector::new(8, 0); 4];
        let luma_mvb = [MotionVector::new(-4, 0); 4];
        let (chroma_mvf, chroma_mvb) = pb_b_chroma_vector(&luma_mvf, &luma_mvb);
        assert_eq!(chroma_mvf, MotionVector::new(4, 0));
        assert_eq!(chroma_mvb, MotionVector::new(-2, 0));
    }

    /// [`pb_b_chroma_vector`] zero-input identity — all-zero luma
    /// vectors yield all-zero chroma vectors on both planes.
    #[test]
    fn pb_b_chroma_vector_all_zero_is_zero() {
        let zero = [MotionVector::default(); 4];
        let (chroma_mvf, chroma_mvb) = pb_b_chroma_vector(&zero, &zero);
        assert_eq!(chroma_mvf, MotionVector::default());
        assert_eq!(chroma_mvb, MotionVector::default());
    }

    /// [`pb_b_chroma_vector`] mixed-magnitude luma exercises the
    /// Table F.1 sixteenth-pel snap. Four luma MVFs with dx sum 6
    /// (4+0+0+2): mag=6, full_chroma_pixels=0, sixteenth=6 → the
    /// Table F.1 row-6 snap (encoded in `chroma_mv_component_4mv`)
    /// drives the fractional component. Validate by computing the
    /// expected value through the public §F.2 primitive on the
    /// same sum.
    #[test]
    fn pb_b_chroma_vector_matches_chroma_mv_component_4mv() {
        let luma_mvf = [
            MotionVector::new(4, 1),
            MotionVector::new(0, 3),
            MotionVector::new(0, -2),
            MotionVector::new(2, 0),
        ];
        let luma_mvb = [
            MotionVector::new(-1, -1),
            MotionVector::new(-2, 0),
            MotionVector::new(1, 2),
            MotionVector::new(0, -3),
        ];
        let (chroma_mvf, chroma_mvb) = pb_b_chroma_vector(&luma_mvf, &luma_mvb);
        let expected_mvf_dx = chroma_mv_component_4mv(6);
        let expected_mvf_dy = chroma_mv_component_4mv(2);
        let expected_mvb_dx = chroma_mv_component_4mv(-2);
        let expected_mvb_dy = chroma_mv_component_4mv(-2);
        assert_eq!(chroma_mvf.dx_half, expected_mvf_dx);
        assert_eq!(chroma_mvf.dy_half, expected_mvf_dy);
        assert_eq!(chroma_mvb.dx_half, expected_mvb_dx);
        assert_eq!(chroma_mvb.dy_half, expected_mvb_dy);
    }

    // ---- §G.5 bidirectional-prediction mask tests --------------------

    /// §G.5 zero-MVB on a luma sub-block (nh=0): both axes' inclusive
    /// ranges become `[max(0, (0+1)/2), min(7, 15-(0+1)/2)] = [0, 7]`,
    /// so the whole 8 × 8 sub-block is bidirectionally predicted.
    #[test]
    fn pb_b_bidir_extent_component_zero_mvb_luma_nh0_is_full_block() {
        let r = pb_b_bidir_extent_component(0, 0, 7, 15);
        assert_eq!(r, Some((0, 7)));
    }

    /// §G.5 with the same zero MVB on the nh=1 luma sub-block: range
    /// is `[max(8, 0), min(15, 15-0)] = [8, 15]`, the full nh=1 block.
    #[test]
    fn pb_b_bidir_extent_component_zero_mvb_luma_nh1_is_full_block() {
        let r = pb_b_bidir_extent_component(0, 8, 15, 15);
        assert_eq!(r, Some((8, 15)));
    }

    /// §G.5 right-pointing MVB on nh=0 (mh=+2 half-pel = 1 luma pixel):
    /// lo = max(0, (-2+1)/2) = max(0, 0) = 0;
    /// hi = min(7, 15 - (2+1)/2) = min(7, 14) = 7. Full block — a
    /// 1-pixel right shift inside the macroblock still keeps the
    /// nh=0 sub-block entirely inside PREC.
    #[test]
    fn pb_b_bidir_extent_component_small_positive_mvb_keeps_full_block() {
        let r = pb_b_bidir_extent_component(2, 0, 7, 15);
        assert_eq!(r, Some((0, 7)));
    }

    /// §G.5 large left-pointing MVB on the nh=0 luma sub-block
    /// (mh=-4 half-pel = 2 luma pixels): lo = max(0, (4+1)/2) =
    /// max(0, 2) = 2; hi = min(7, 15 - (-4+1)/2) = min(7, 15 - (-1)) =
    /// min(7, 16) = 7. Range `[2, 7]` — the leftmost two columns of
    /// the nh=0 sub-block are forward-only since MVB points outside
    /// PREC to their left.
    #[test]
    fn pb_b_bidir_extent_component_left_mvb_shrinks_nh0_range() {
        let r = pb_b_bidir_extent_component(-4, 0, 7, 15);
        assert_eq!(r, Some((2, 7)));
    }

    /// §G.5 large right-pointing MVB on the nh=1 luma sub-block
    /// (mh=+8 half-pel = 4 luma pixels): lo = max(8, (-8+1)/2) =
    /// max(8, -3) = 8; hi = min(15, 15 - (8+1)/2) = min(15, 11) = 11.
    /// Range `[8, 11]` — the rightmost 4 columns of the nh=1
    /// sub-block are forward-only because MVB points outside PREC.
    #[test]
    fn pb_b_bidir_extent_component_right_mvb_shrinks_nh1_range() {
        let r = pb_b_bidir_extent_component(8, 8, 15, 15);
        assert_eq!(r, Some((8, 11)));
    }

    /// §G.5 with MVB so large the bidirectional rectangle is empty:
    /// nh=1 sub-block with mh=+16 half-pel (8 luma pixels right) →
    /// lo = max(8, -7) = 8; hi = min(15, 15 - 8) = 7. lo > hi, so the
    /// whole sub-block falls outside PREC for this axis — forward-only.
    #[test]
    fn pb_b_bidir_extent_component_large_positive_mvb_empties_nh1() {
        let r = pb_b_bidir_extent_component(16, 8, 15, 15);
        assert_eq!(r, None);
    }

    /// §G.5 division is by truncation toward zero (Rust signed `/`),
    /// matching the C expression `(-mh+1)/2` in the spec. mh=+3 (odd):
    /// `(-3+1)/2 = -2/2 = -1`; `(3+1)/2 = 2`. lo=max(0,-1)=0, hi=
    /// min(7, 15-2)=7. Range `[0, 7]` — full block. The point of the
    /// test is that we don't accidentally use floor for the negative
    /// numerator: `(-3+1)/2 = -1` truncation toward zero (not -1 or
    /// 0 by floor, but matches "trunc" here anyway). Sanity-check by
    /// using an mh with odd numerator that distinguishes the two
    /// modes: mh=-3 → `(3+1)/2 = 2`, lo=max(0,2)=2; `(-3+1)/2 = -1`,
    /// hi=min(7, 16)=7. Range `[2, 7]`.
    #[test]
    fn pb_b_bidir_extent_component_division_truncates_toward_zero() {
        // mh=-3: lo = max(0, 2) = 2; hi = min(7, 15 - (-1)) = 7.
        let r = pb_b_bidir_extent_component(-3, 0, 7, 15);
        assert_eq!(r, Some((2, 7)));
    }

    /// §G.5 luma block extent: zero MVB on each of the four
    /// `(nh, nv)` sub-blocks gives the full 8 × 8 rectangle for that
    /// sub-block.
    #[test]
    fn pb_b_bidir_luma_block_extent_zero_mvb_full_block_all_four_subblocks() {
        let mvb = MotionVector::default();
        assert_eq!(
            pb_b_bidir_luma_block_extent(mvb, 0, 0),
            Some(((0, 7), (0, 7)))
        );
        assert_eq!(
            pb_b_bidir_luma_block_extent(mvb, 1, 0),
            Some(((8, 15), (0, 7)))
        );
        assert_eq!(
            pb_b_bidir_luma_block_extent(mvb, 0, 1),
            Some(((0, 7), (8, 15)))
        );
        assert_eq!(
            pb_b_bidir_luma_block_extent(mvb, 1, 1),
            Some(((8, 15), (8, 15)))
        );
    }

    /// §G.5 luma block extent: MVB = (-4, -4) on the nh=0,nv=0
    /// sub-block shrinks each axis to `[2, 7]`, so the bidirectional
    /// rectangle is `[2..=7] × [2..=7]` (a 6 × 6 region inside the
    /// upper-left 8 × 8 sub-block).
    #[test]
    fn pb_b_bidir_luma_block_extent_left_up_mvb_on_nh0_nv0() {
        let mvb = MotionVector::new(-4, -4);
        let extent = pb_b_bidir_luma_block_extent(mvb, 0, 0).expect("non-empty rectangle");
        assert_eq!(extent, ((2, 7), (2, 7)));
    }

    /// §G.5 luma block extent: MVB with one axis empty short-circuits
    /// the rectangle to `None`. nh=1,nv=0 sub-block with MVB = (+16, 0)
    /// has empty horizontal range; the §G.5 axis-product makes the
    /// whole sub-block forward-only regardless of the vertical range.
    #[test]
    fn pb_b_bidir_luma_block_extent_empty_axis_yields_none() {
        let mvb = MotionVector::new(16, 0);
        assert_eq!(pb_b_bidir_luma_block_extent(mvb, 1, 0), None);
    }

    /// §G.5 luma block extent: MVB = (0, +16) on nh=0,nv=1 — vertical
    /// range empties (`max(8, -7) = 8`, `min(15, 15-8) = 7`); §G.5
    /// makes the whole nh=0,nv=1 sub-block forward-only.
    #[test]
    fn pb_b_bidir_luma_block_extent_empty_vertical_axis_yields_none() {
        let mvb = MotionVector::new(0, 16);
        assert_eq!(pb_b_bidir_luma_block_extent(mvb, 0, 1), None);
    }

    /// §G.5 chroma extent: zero MVC gives the full 8 × 8 chroma
    /// block: `[0..=7] × [0..=7]`.
    #[test]
    fn pb_b_bidir_chroma_extent_zero_mvc_is_full_chroma_block() {
        let mvc = MotionVector::default();
        assert_eq!(pb_b_bidir_chroma_extent(mvc), Some(((0, 7), (0, 7))));
    }

    /// §G.5 chroma extent with right + down MVC: mhc=+4 → lo =
    /// max(0, -1) = 0; hi = min(7, 7-2) = 5. mvc=+4 same on vertical
    /// axis. Bidirectional rectangle is `[0..=5] × [0..=5]`.
    #[test]
    fn pb_b_bidir_chroma_extent_right_down_mvc_shrinks_to_top_left() {
        let mvc = MotionVector::new(4, 4);
        assert_eq!(pb_b_bidir_chroma_extent(mvc), Some(((0, 5), (0, 5))));
    }

    /// §G.5 chroma extent with left + up MVC: mhc=-4 → lo =
    /// max(0, 2) = 2; hi = min(7, 7-(-1)) = min(7, 8) = 7. mvc=-4
    /// same on vertical axis. Bidirectional rectangle is
    /// `[2..=7] × [2..=7]`.
    #[test]
    fn pb_b_bidir_chroma_extent_left_up_mvc_shrinks_to_bottom_right() {
        let mvc = MotionVector::new(-4, -4);
        assert_eq!(pb_b_bidir_chroma_extent(mvc), Some(((2, 7), (2, 7))));
    }

    /// §G.5 chroma extent with MVC so large the block falls outside
    /// PREC: mhc=+16 → lo = max(0, -7) = 0; hi = min(7, 7-8) = -1.
    /// Empty range → whole chroma block is forward-only.
    #[test]
    fn pb_b_bidir_chroma_extent_large_mvc_yields_none() {
        let mvc = MotionVector::new(16, 0);
        assert_eq!(pb_b_bidir_chroma_extent(mvc), None);
    }

    /// §G.5 chroma extent reuses the per-component primitive with
    /// `ref_max = 7` and block bounds `[0, 7]`; verify by checking
    /// the two-axis composition matches independent per-axis calls.
    #[test]
    fn pb_b_bidir_chroma_extent_factorises_per_axis() {
        let mvc = MotionVector::new(-2, 6);
        let extent = pb_b_bidir_chroma_extent(mvc).expect("non-empty");
        let i = pb_b_bidir_extent_component(-2, 0, 7, 7).expect("i non-empty");
        let j = pb_b_bidir_extent_component(6, 0, 7, 7).expect("j non-empty");
        assert_eq!(extent, (i, j));
    }

    /// End-to-end §G.4 → §G.5: compute the §G.4 (MVF, MVB) pair for
    /// a P-MV (8, 0) at TRB=1, TRD=2 with MVDB `(+2, -2)`, then
    /// derive the §G.5 bidirectional rectangle for the nh=0,nv=0
    /// luma sub-block from the resulting MVB. Demonstrates the
    /// composed §G.4 + §G.5 pipeline the macroblock driver will run
    /// per luma sub-block.
    #[test]
    fn pb_b_bidir_chained_after_g4() {
        let p_mv = MotionVector::new(8, 0);
        let mvd = Some(Mvd {
            dx_half: 2,
            dy_half: -2,
        });
        let (mvf, mvb) = pb_b_vector(p_mv, mvd, 1, 2);
        // §G.4: MVF.x = (1×8)/2 + 2 = 6; MVF.y = (1×0)/2 + (-2) = -2.
        // MVB on x: since MVD.x=2≠0, MVB.x = MVF.x - MV.x = 6-8 = -2.
        // MVB on y: since MVD.y=-2≠0, MVB.y = MVF.y - MV.y = -2-0 = -2.
        assert_eq!(mvf, MotionVector::new(6, -2));
        assert_eq!(mvb, MotionVector::new(-2, -2));
        // §G.5: nh=0,nv=0 sub-block with MVB = (-2, -2).
        // x: lo=max(0, (2+1)/2)=max(0,1)=1; hi=min(7, 15-((-2+1)/2))=
        // min(7, 15-0)=7. Range [1,7].
        // y: same → [1,7].
        let extent = pb_b_bidir_luma_block_extent(mvb, 0, 0).expect("non-empty");
        assert_eq!(extent, ((1, 7), (1, 7)));
    }

    /// §G.5 luma `pb_b_bidir_luma_block_extent` panics on `nh > 1`
    /// (§G.5 only enumerates the four sub-blocks).
    #[test]
    #[should_panic(expected = "§G.5 luma sub-block nh must be 0 or 1")]
    fn pb_b_bidir_luma_block_extent_panics_on_invalid_nh() {
        let _ = pb_b_bidir_luma_block_extent(MotionVector::default(), 2, 0);
    }

    /// §G.5 luma `pb_b_bidir_luma_block_extent` panics on `nv > 1`.
    #[test]
    #[should_panic(expected = "§G.5 luma sub-block nv must be 0 or 1")]
    fn pb_b_bidir_luma_block_extent_panics_on_invalid_nv() {
        let _ = pb_b_bidir_luma_block_extent(MotionVector::default(), 0, 2);
    }

    /// §G.5 per-pixel average of identical samples is the sample
    /// itself: `(x + x) / 2 = x`.
    #[test]
    fn pb_b_bidir_pixel_identical_inputs_unchanged() {
        for x in 0u8..=255 {
            assert_eq!(pb_b_bidir_pixel(x, x), x);
        }
    }

    /// §G.5 per-pixel "average by division truncation" — sum divided
    /// by two. For `(0, 1)` the §G.5 spec gives `(0+1)/2 = 0`
    /// (truncation toward zero); for `(1, 0)` symmetrically the
    /// same. For `(1, 2)` it's `(1+2)/2 = 1` (not the
    /// round-half-up `2`).
    #[test]
    fn pb_b_bidir_pixel_truncates_toward_zero() {
        assert_eq!(pb_b_bidir_pixel(0, 1), 0);
        assert_eq!(pb_b_bidir_pixel(1, 0), 0);
        assert_eq!(pb_b_bidir_pixel(1, 2), 1);
        assert_eq!(pb_b_bidir_pixel(2, 1), 1);
        assert_eq!(pb_b_bidir_pixel(3, 4), 3);
    }

    /// §G.5 per-pixel average of maximum-range samples does not
    /// overflow `u8`: `(255 + 255) / 2 = 255`.
    #[test]
    fn pb_b_bidir_pixel_max_inputs_does_not_overflow() {
        assert_eq!(pb_b_bidir_pixel(255, 255), 255);
        assert_eq!(pb_b_bidir_pixel(255, 254), 254);
        assert_eq!(pb_b_bidir_pixel(254, 255), 254);
    }

    /// §G.5 per-pixel commutativity follows from the integer-sum
    /// formula; verify across a broad sample of pairs that
    /// `pb_b_bidir_pixel(a, b) == pb_b_bidir_pixel(b, a)`.
    #[test]
    fn pb_b_bidir_pixel_commutes() {
        for a in (0u8..=255).step_by(17) {
            for b in (0u8..=255).step_by(13) {
                assert_eq!(pb_b_bidir_pixel(a, b), pb_b_bidir_pixel(b, a));
            }
        }
    }

    /// §G.5 block-blend with `None` extent (whole block forward-only
    /// per §G.5's "all other pixels" clause when the rectangle is
    /// empty): output equals the forward array verbatim and the
    /// backward array is ignored.
    #[test]
    fn pb_b_blend_block_none_extent_returns_forward() {
        let mut fwd = [[0u8; 8]; 8];
        let mut bwd = [[0u8; 8]; 8];
        for j in 0..8 {
            for i in 0..8 {
                fwd[j][i] = (j * 8 + i) as u8;
                bwd[j][i] = 200u8.wrapping_sub((j * 8 + i) as u8);
            }
        }
        let out = pb_b_blend_block(&fwd, &bwd, None, 0, 0);
        assert_eq!(out, fwd);
    }

    /// §G.5 block-blend with the full 8 × 8 chroma rectangle
    /// `[0..=7] × [0..=7]`: every output pixel is the average of
    /// the corresponding forward and backward sample.
    #[test]
    fn pb_b_blend_block_full_chroma_extent_averages_every_pixel() {
        let mut fwd = [[0u8; 8]; 8];
        let mut bwd = [[0u8; 8]; 8];
        for j in 0..8 {
            for i in 0..8 {
                fwd[j][i] = 100;
                bwd[j][i] = 50;
            }
        }
        let out = pb_b_blend_block(&fwd, &bwd, Some(((0, 7), (0, 7))), 0, 0);
        for (j, row) in out.iter().enumerate() {
            for (i, &px) in row.iter().enumerate() {
                assert_eq!(px, 75, "j={}, i={}", j, i);
            }
        }
    }

    /// §G.5 block-blend with a sub-rectangle: pixels inside the
    /// rectangle are averaged, pixels outside it are taken from
    /// `forward` unchanged.
    #[test]
    fn pb_b_blend_block_partial_extent_averages_inside_only() {
        let fwd = [[10u8; 8]; 8];
        let bwd = [[200u8; 8]; 8];
        // Rectangle [2..=5] × [3..=6] in chroma-local 0..=7
        // coordinates.
        let out = pb_b_blend_block(&fwd, &bwd, Some(((2, 5), (3, 6))), 0, 0);
        for (j, row) in out.iter().enumerate() {
            for (i, &px) in row.iter().enumerate() {
                let inside_i = (2..=5).contains(&(i as i32));
                let inside_j = (3..=6).contains(&(j as i32));
                let expected = if inside_i && inside_j {
                    (10 + 200) / 2
                } else {
                    10
                };
                assert_eq!(px, expected, "j={}, i={}", j, i);
            }
        }
    }

    /// §G.5 block-blend for a luma sub-block at `(nh=1, nv=1)` —
    /// macroblock-local coordinates `[8..=15]` along each axis,
    /// origin `(8, 8)`. The §G.5 rectangle and the input arrays use
    /// macroblock-local coordinates for indexing the rectangle and
    /// block-local 0..=7 indexing for the arrays.
    #[test]
    fn pb_b_blend_block_nh1_nv1_luma_origin_offset() {
        let fwd = [[20u8; 8]; 8];
        let bwd = [[60u8; 8]; 8];
        // §G.5 rectangle for nh=1,nv=1 spanning the full 8 × 8 luma
        // sub-block: i ∈ [8, 15], j ∈ [8, 15].
        let out = pb_b_blend_block(&fwd, &bwd, Some(((8, 15), (8, 15))), 8, 8);
        for (j, row) in out.iter().enumerate() {
            for (i, &px) in row.iter().enumerate() {
                assert_eq!(px, 40, "j={}, i={}", j, i);
            }
        }
    }

    /// §G.5 block-blend rejects a rectangle that escapes the 8 × 8
    /// block on the high `i` side.
    #[test]
    #[should_panic(expected = "§G.5 i-range must lie inside the 8 × 8 block")]
    fn pb_b_blend_block_panics_on_i_overflow() {
        let fwd = [[0u8; 8]; 8];
        let bwd = [[0u8; 8]; 8];
        let _ = pb_b_blend_block(&fwd, &bwd, Some(((0, 8), (0, 7))), 0, 0);
    }

    /// §G.5 block-blend rejects a rectangle whose origin escapes
    /// the 8 × 8 block on the low `j` side.
    #[test]
    #[should_panic(expected = "§G.5 j-range must lie inside the 8 × 8 block")]
    fn pb_b_blend_block_panics_on_j_underflow() {
        let fwd = [[0u8; 8]; 8];
        let bwd = [[0u8; 8]; 8];
        let _ = pb_b_blend_block(&fwd, &bwd, Some(((0, 7), (-1, 6))), 0, 0);
    }

    /// End-to-end §G.4 → §G.5 mask → §G.5 blend: compose all three
    /// primitives for one luma sub-block. Uses a synthetic but
    /// spec-consistent §G.4 MVB and a constant-fill forward /
    /// backward prediction so the per-pixel result is predictable
    /// from the rectangle.
    #[test]
    fn pb_b_blend_chained_g4_extent_blend() {
        let p_mv = MotionVector::new(8, 0);
        let mvd = Some(Mvd {
            dx_half: 2,
            dy_half: -2,
        });
        let (_mvf, mvb) = pb_b_vector(p_mv, mvd, 1, 2);
        let extent = pb_b_bidir_luma_block_extent(mvb, 0, 0);
        assert_eq!(extent, Some(((1, 7), (1, 7))));
        let fwd = [[80u8; 8]; 8];
        let bwd = [[240u8; 8]; 8];
        let out = pb_b_blend_block(&fwd, &bwd, extent, 0, 0);
        // Inside [1..=7] × [1..=7] (luma origin 0,0): (80+240)/2 =
        // 160. Outside: 80.
        for (j, row) in out.iter().enumerate() {
            for (i, &px) in row.iter().enumerate() {
                let inside = (1..=7).contains(&(i as i32)) && (1..=7).contains(&(j as i32));
                let expected = if inside { 160 } else { 80 };
                assert_eq!(px, expected, "j={}, i={}", j, i);
            }
        }
    }
}
