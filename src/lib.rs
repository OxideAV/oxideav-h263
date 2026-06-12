//! # oxideav-h263
//!
//! Pure-Rust ITU-T H.263 baseline video codec for the
//! [oxideav](https://github.com/OxideAV/oxideav) framework.
//!
//! **Status:** orphan-rebuild round-2 (post 2026-05-18 audit).
//!
//! The decoder is being re-built clean-room against
//! [ITU-T Recommendation H.263 (01/2005)][spec]. Coverage so far:
//!
//! * **Round 1** — picture-header parser per §5.1: the Picture Start
//!   Code (PSC), Temporal Reference (TR), and the variable-length
//!   Type Information (PTYPE) field with its source-format and
//!   picture-coding sub-fields.
//! * **Round 2** — GOB-layer header per §5.2: Group of Blocks Start
//!   Code (GBSC), Group Number (GN), GOB Frame ID (GFID), and
//!   Quantizer Information (GQUANT), for the CPM = "0" branch (no
//!   GSBI on the wire).
//! * **Round 3** — Macroblock-layer header per §5.3: COD (§5.3.1),
//!   MCBPC (§5.3.2, Tables 7 / 8), CBPY (§5.3.5, Table 12),
//!   DQUANT baseline form (§5.3.6, Table 13), and primary +
//!   secondary MVD (§5.3.7 / §5.3.8, Table 14). The decode is
//!   purely structural — no IDCT or pixel reconstruction.
//! * **Round 4** — Block-layer parsing per §5.4: INTRADC (§5.4.1,
//!   Table 15 — 8-bit FLC with two forbidden codes and the
//!   `0xFF`-means-1024 special case) and TCOEF (§5.4.2, Table 16
//!   — 102 regular VLC code-points + one 7-bit ESCAPE prefix
//!   followed by a fixed-length 1+6+8 event). Decoded coefficients
//!   land in a 64-entry zigzag-scan-order array per block; the
//!   §6.2.3 / Figure 14 zigzag→8×8 position table is exposed for
//!   callers that need it.
//! * **Round 5** — Coefficient reconstruction §6.1 / §6.2.1
//!   inverse-quant (H.261-style modulo-2-oddifier), §6.2.2 clip,
//!   §6.2.3 zigzag scatter, §6.2.4 separable 8×8 IDCT (direct
//!   orthonormal kernel in `f64`, meeting Annex A.7), and §6.3.2
//!   intra-block sample clip to `[0, 255]`. Composed end-to-end
//!   into [`reconstruct_intra_block`] which takes a parsed
//!   [`H263Block`] + QUANT and emits an 8×8 `u8` sample block.
//! * **Round 6** — P-frame motion compensation: §6.1.1 differential
//!   motion-vector reconstruction (predictor + Table-14 difference,
//!   wrapped into the permitted `[-32, 31]` half-pel window) and
//!   median predictor, Table 18 chroma-vector derivation, §6.1.2
//!   half-pel bilinear interpolation (Figure 13) with §D.1 edge
//!   replication, and §6.3.1 INTER summation + §6.3.2 clip. Composed
//!   end-to-end into [`reconstruct_inter_block_with_prediction`]. See
//!   the [`motion`] module.
//! * **Round 7** — Annex J Deblocking Filter mode: the §J.3
//!   four-tap edge filter (`d = (A−4B+4C−D)/8`,
//!   `d1 = UpDownRamp(d, STRENGTH)`,
//!   `d2 = clipd1((A−D)/4, d1/2)`), the full Table J.2 STRENGTH
//!   lookup for QUANT in `1..=31`, the §J.3 picture-edge skip rule,
//!   and the §J.3 horizontal-before-vertical edge ordering driver
//!   [`deblock_plane`]. See the [`deblock`] module.
//! * **Round 8** — Annex I Advanced INTRA Coding, scan + mode layer:
//!   the §I.2 INTRA_MODE field VLC (Table I.1 →
//!   [`aic::IntraMode`]), the two §I.3 alternate DCT scans
//!   (Figure I.2: [`aic::ALT_HORIZONTAL_TO_BLOCK_POS`] /
//!   [`aic::ALT_VERTICAL_TO_BLOCK_POS`]), and the §I.3 scan-selection
//!   rule [`aic::scan_for_intra_mode`]. See the [`aic`] module. The
//!   Table I.2 separate INTRA-coefficient VLC, the modified inverse
//!   quantization, and the DC/AC prediction reconstruction are
//!   deferred (they need the macroblock-grid driver for neighbour
//!   blocks).
//! * **Round 9** — full-picture decode driver: [`decode_picture`]
//!   walks all GOBs / macroblocks of a picture (§4.2.1) and composes
//!   the per-layer parsers and per-block reconstruction primitives
//!   into a decoded planar 4:2:0 [`YuvFrame`]. The baseline single-MV
//!   path covers INTRA / INTRA+Q / INTER / INTER+Q / skipped
//!   macroblocks, the §6.1.1 / Figure-12 candidate-predictor selection
//!   border rules, and an optional Annex J §J.3 deblocking pass. See
//!   the [`picture`] module.
//! * **Round 10** — Annex D §D.2 Unrestricted Motion Vector mode
//!   (PLUSPTYPE absent): [`reconstruct_mv_component_umv`] /
//!   [`reconstruct_mv_umv`] extend the per-component range from the
//!   default `[-32, 31]` to `[-63, 63]` half-pel, with the §D.2
//!   predictor-dependent selection of the Table-14 difference pair.
//!   The driver applies it whenever the PTYPE bit-10 UMV flag is set;
//!   §D.1 edge replication (already always-on) supplies the
//!   out-of-picture samples. The PLUSPTYPE / UUI ranges of Tables
//!   D.1 / D.2 and the Table-D.3 reversible VLC stay gated on the
//!   not-yet-decoded extended-PTYPE header.
//! * **Round 11** — Annex F §F.2 four-motion-vector candidate-predictor
//!   redefinition (Figure F.1) and Table F.1 sixteenth-pixel chroma
//!   derivation as pure transformations: [`LumaBlockIndex`] /
//!   [`Mb4Mv`] / [`Mb4MvNeighbourhood`] + [`select_4mv_candidates`]
//!   return the three §6.1.1 median-predictor candidates `(MV1, MV2,
//!   MV3)` for any of the four 8×8 luminance blocks in a current
//!   macroblock, given the four-MV grids of its left / above /
//!   above-right / right neighbours. [`chroma_mv_4mv`] /
//!   [`chroma_mv_component_4mv`] reduce the sum of the four luma
//!   vectors to one chroma vector with the Table F.1 sixteenth →
//!   half-pixel snap (asymmetric `{0,1,2}→0`, `{3..=13}→1`,
//!   `{14,15}→2` mapping).
//! * **Round 12** — Annex F §F.3 overlapped block motion compensation
//!   (OBMC) for the 8×8 luminance prediction, as the pure function
//!   [`obmc_predict_block`] over the Figures F.2 / F.3 / F.4 weight
//!   matrices [`H0`] / [`H1`] / [`H2`]. Each pixel is
//!   `(q·H0 + r·H1 + s·H2 + 4) / 8` with `q` the current block's MV
//!   and `r` / `s` the per-pixel "top-or-bottom" / "left-or-right"
//!   remote vectors, each wrapped in [`RemoteMv`] so the caller can
//!   encode the §F.3 substitution rules (not-coded → zero; INTRA /
//!   outside picture / bottom-of-MB → current vector) without folding
//!   the resolved vector here. The macroblock-driver wiring that walks
//!   the live neighbour grid and dispatches `obmc_predict_block` per
//!   8×8 luminance block of an INTER4V macroblock remains out of scope.
//! * **Round 14** — Annex I §I.3 / Table I.2 separate INTRA-coefficient
//!   VLC, as the pure event-level primitive [`decode_intra_tcoef_event`]
//!   in the new [`intra_tcoef`] module. The 102 regular codewords reuse
//!   Table 16's bit patterns at every index (per §I.3) but reassign the
//!   `(RUN, |LEVEL|)` columns (with `LAST` preserved); the 7-bit
//!   ESCAPE prefix and its 1+6+8 fixed-length tail are decoded
//!   identically to §5.4.2 with the baseline forbidden LEVEL codes
//!   (`0x00` / `0x80`) applied. Wiring the I.2 VLC into a full
//!   INTRA-block parser with the §I.3 absorbed-INTRADC semantics
//!   (line 4214) and the DC/AC prediction reconstruction is deferred
//!   pending the macroblock-grid driver.
//! * **Round 15** — Annex K Slice Structured mode slice-layer header,
//!   as the [`slice_header`] module. [`parse_slice_layer`] decodes the
//!   §K.2 / Figure K.1 layout (SSC + SEPB1 + optional SSBI + MBA +
//!   optional SEPB2 + SQUANT + optional SWI + SEPB3 + GFID) for slices
//!   other than the first; [`parse_first_slice_header`] decodes the
//!   §K.2 reduced form (SEPB1 + MBA + optional SEPB2 + optional SWI +
//!   SEPB3) for the slice that immediately follows the picture start
//!   code. Field widths are looked up against Tables K.2 (MBA) and K.3
//!   (SWI) per the [`SliceHeaderContext`] picture-geometry / CPM /
//!   RS-submode / RRU inputs, with the §K.1 legal-codeword set for
//!   SSBI (Table K.1) enforced. Wiring the slice header into the
//!   `decode_picture` driver (so a slice-structured bitstream actually
//!   reconstructs a frame) is the next round's work.
//! * **Round 16** — Annex F §F.2 / §F.3 INTER4V four-motion-vector +
//!   Overlapped Block Motion Compensation driver wiring. The
//!   [`decode_picture`] driver now reconstructs INTER4V / INTER4V+Q
//!   macroblocks end-to-end whenever the picture header's Advanced
//!   Prediction flag is set: each of the four luma MVs is built from
//!   `select_4mv_candidates` + `predict_mv_median` over a live
//!   four-MV neighbour grid, the §F.3 OBMC weighted average is
//!   dispatched per luma block via [`obmc_predict_block`] with the
//!   four remote MVs classified into [`RemoteMv`] tags per the §F.3
//!   substitution rules (not-coded → zero; INTRA / off-picture →
//!   current; baseline → coded vector; §F.3 last sentence forces B3 /
//!   B4's bottom remote to `Current`), and the chroma vector comes
//!   from [`chroma_mv_4mv`] (sum of the four luma vectors / 8 with
//!   the Table F.1 sixteenth → half snap). Chroma blocks use standard
//!   half-pel motion compensation (no OBMC for chroma per §F.2).
//! * **Round 19** — Annex I §I.3 INTRA DC/AC prediction reconstruction,
//!   as the pure function [`reconstruct_intra_block_aic`] in the new
//!   [`aic_predict`] module. Given a current INTRA block's dequantized
//!   residual `RecC(u,v)` array, the §I.2 `INTRA_MODE`, and an optional
//!   pair of already-reconstructed neighbour blocks (`RecA'` immediately
//!   above, `RecB'` immediately to the left), returns the final
//!   `RecC'(u,v)` array post-`clipAC` for AC slots and post-
//!   `oddifyclipDC` for the DC slot. The three §I.3 page-79 INTRA_MODE
//!   rules are encoded directly: Mode 0 averages the two DC neighbours
//!   with truncation toward zero; Mode 1 / Mode 2 predict DC plus the
//!   first row / column from `RecA'(u, 0)` / `RecB'(0, v)`. The §I.3
//!   page-78 "same video picture segment" availability test lives in
//!   the macroblock-grid driver and is surfaced via the [`Neighbour`]
//!   tag; the predictor itself takes the availability decision as
//!   input. The fallback DC predictor `1024` is exposed as
//!   [`AIC_FALLBACK_DC_PREDICTOR`]. This closes the round-17 / round-18
//!   "DC/AC prediction deferred" gap as a pure-function primitive; the
//!   macroblock-grid driver that walks the picture, accumulates `RecA'`
//!   / `RecB'` arrays, and dispatches this primitive plus the inverse
//!   DCT remains the next round.
//!
//! * **Round 20** — Annex I §I.3 end-to-end INTRA-block reconstruction
//!   pipeline as two pure functions in [`aic_predict`]:
//!   [`aic_intra_reconstruct_coefficients`] composes §I.3 modified
//!   inverse quantisation, the [`aic::scan_for_intra_mode`] /
//!   Figure I.2 scan-permutation scatter, and the §I.3 DC/AC prediction
//!   reconstruction into a single `H263Block` zigzag-LEVEL →
//!   `RecC'(u,v)` transformation; [`aic_intra_reconstruct_samples`]
//!   runs the §6.2.4 IDCT + §6.3.2 sample clip on the resulting
//!   block-position coefficient array. Together they cover the four
//!   composition steps `block_aic.rs` flagged as the §I.3 downstream
//!   pipeline (modified-inverse-quant + scan-scatter + DC/AC
//!   prediction + IDCT) as pure functions, leaving only the
//!   macroblock-grid driver's job of walking the picture, accumulating
//!   neighbour `RecA'` / `RecB'` arrays, and dispatching this pipeline
//!   per INTRA block.
//!
//! * **Round 27** — §5.3.3 / §5.3.4 PB-frame B-block field parsers as
//!   the new [`pb_layer`] module. [`parse_modb`] decodes the Table 11
//!   3-entry MODB variable-length codeword into a [`ModbPresence`] tag
//!   (`None` / `MvdbOnly` / `CbpbAndMvdb`); [`parse_cbpb`] decodes the
//!   6-bit fixed-length CBPB coded-block-pattern; [`cbpb_block_present`]
//!   queries an individual B-block's CBPBN bit by the §5.3.4 / Figure 5
//!   "utmost left bit ↔ block number 1" mapping. These are the wire
//!   primitives the future macroblock-driver wiring for PB-frame mode
//!   (Annex G) will compose with the existing MVD-component decoder
//!   (which §5.3.9 reuses verbatim for MVDB). The Annex-M (Improved
//!   PB-frames) MODB Table M.1 7-entry form remains a separate
//!   primitive a future round will add.
//!
//! PB-frame / Annex-T / extended-PTYPE-gated decode paths are still
//! out of scope, and although the §I.3 AIC prediction reconstruction
//! pipeline now exists end-to-end in [`aic_predict`], the
//! macroblock-grid driver that walks the picture and dispatches it is
//! not yet wired into [`decode_picture`]; the driver returns
//! [`Error::NotImplemented`] for those paths. The
//! `oxideav_core::Decoder` registration is still a no-op —
//! [`decode_picture`] is the frame-yielding surface pending the full
//! streaming `Decoder` impl.
//!
//! [spec]: https://www.itu.int/rec/T-REC-H.263

#![warn(missing_debug_implementations)]

use oxideav_core::bits::BitReader;
use oxideav_core::RuntimeContext;

pub mod aic;
pub mod aic_dequant;
pub mod aic_predict;
pub mod block;
pub mod block_aic;
pub mod deblock;
pub mod dequant;
pub mod gob_header;
pub mod idct;
pub mod intra_tcoef;
pub mod macroblock;
pub mod motion;
pub mod pb_layer;
pub mod picture;
pub mod picture_header;
pub mod plus_ptype;
pub mod slice_header;

pub use aic::{
    decode_intra_mode, scan_for_intra_mode, IntraMode, ALT_HORIZONTAL_TO_BLOCK_POS,
    ALT_VERTICAL_TO_BLOCK_POS,
};
pub use aic_dequant::{
    aic_dequant_coefficient, clip_ac, oddify_clip_dc, AIC_AC_REC_MAX, AIC_AC_REC_MIN,
    AIC_DC_REC_MAX, AIC_DC_REC_MIN,
};
pub use aic_predict::{
    aic_intra_reconstruct_coefficients, aic_intra_reconstruct_samples, reconstruct_intra_block_aic,
    Neighbour, AIC_FALLBACK_DC_PREDICTOR,
};
pub use block::{parse_block, BlockContext, H263Block, COEFFS_PER_BLOCK, ZIGZAG_TO_BLOCK_POS};
pub use block_aic::parse_intra_block_aic;
pub use deblock::{
    apply_edge_samples, clipd1, deblock_plane, filter_edge_samples, strength_for_quant,
    up_down_ramp, EdgeCondition, STRENGTH_RRU_INFINITE,
};
pub use dequant::{dequantise_ac, scatter_into_block, AC_REC_MAX, AC_REC_MIN};
pub use gob_header::{
    parse_gob_layer, parse_gob_layer_from_bytes, GobLayer, GBSC_BITS, GBSC_VALUE, GFID_BITS,
    GN_BITS, GOB_HEADER_BITS_NO_CPM, GQUANT_BITS,
};
pub use idct::{idct_8x8, reconstruct_intra_samples, BLOCK_DIM, IDCT_OUT_MAX, IDCT_OUT_MIN};
pub use intra_tcoef::{decode_intra_tcoef_event, IntraTcoefEvent, INTRA_TCOEF_REGULAR_ENTRIES};
pub use macroblock::{parse_macroblock, H263Macroblock, MbContext, MbType, Mvd};
pub use motion::{
    chroma_mv, chroma_mv_4mv, chroma_mv_component, chroma_mv_component_4mv, median3,
    motion_compensate_block, obmc_predict_block, predict_mv_median, reconstruct_inter_block,
    reconstruct_mv, reconstruct_mv_component, reconstruct_mv_component_umv, reconstruct_mv_umv,
    select_4mv_candidates, LumaBlockIndex, Mb4Mv, Mb4MvNeighbourhood, MotionVector, RefPlane,
    RemoteMv, H0, H1, H2, MV_HALF_MAX, MV_HALF_MIN, MV_HALF_SPAN, MV_UMV_HALF_MAX, MV_UMV_HALF_MIN,
    OBMC_WEIGHT_SUM, RCONTROL_DEFAULT,
};
pub use pb_layer::{
    cbpb_block_present, parse_cbpb, parse_modb, parse_modb_annex_m, parse_mvdb,
    pb_b_bidir_chroma_extent, pb_b_bidir_extent_component, pb_b_bidir_luma_block_extent,
    pb_b_bidir_pixel, pb_b_blend_block, pb_b_chroma_vector, pb_b_vector, pb_b_vectors, pb_bquant,
    BpbCodingMode, ModbAnnexM, ModbPresence, CBPB_BITS,
};
pub use picture::{
    decode_pb_picture, decode_picture, decode_picture_layer, decode_picture_layer_with_inherited,
    DecodeOptions, DecodePictureOutcome, PbFramePair, PictureLayout, YuvFrame,
};
pub use picture_header::{
    parse_picture_header, parse_picture_layer, H263ExtendedPicture, H263PictureCodingType,
    H263PictureHeader, H263PictureLayer, H263SourceFormat, PSC_BITS, PSC_VALUE,
};
pub use plus_ptype::{
    parse_plus_ptype, CustomPcf, CustomPictureFormat, ExtendedPar, InheritedExtendedState, Mpptype,
    Opptype, PlusPictureType, PlusPtypeHeader, PlusSourceFormat, SliceStructuredSubmode, Uui,
    CPCFC_BITS, CPFMT_BITS, EPAR_BITS, ETR_BITS, MPPTYPE_BITS, OPPTYPE_BITS, SSS_BITS, UFEP_BITS,
    UFEP_FULL, UFEP_MANDATORY_ONLY,
};
pub use slice_header::{
    parse_first_slice_header, parse_slice_layer, skip_sstuf, skip_sstuf_at, ssbi_to_subbitstream,
    FirstSliceLayer, SliceHeaderContext, SliceLayer, SEPB_BITS, SQUANT_BITS, SSBI_BITS, SSC_BITS,
    SSC_VALUE, SSTUF_MAX_BITS,
};

/// Crate-local error type. The orphan-rebuild scaffold returns
/// [`Error::NotImplemented`] for any decode path that is not yet wired
/// up; the picture-header parser returns the variants below directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The crate is partially scaffolded; the path the caller invoked
    /// is not yet implemented.
    NotImplemented,
    /// Bitstream ended before the parser could read the requested
    /// number of bits. Returned by [`parse_picture_header`] when the
    /// input buffer is shorter than the picture-layer header demands.
    UnexpectedEof,
    /// The 22-bit Picture Start Code (PSC, value `0x000020`) was not
    /// present at the current bitstream position. See §5.1.1.
    BadPictureStartCode,
    /// PTYPE bit 1 (always "1") or bit 2 (always "0") did not have
    /// their fixed values. See §5.1.3.
    BadPtypeFixedBits,
    /// PTYPE source-format field (bits 6-8) had the forbidden value
    /// `"000"`. See §5.1.3.
    ForbiddenSourceFormat,
    /// The extended-PTYPE path (PTYPE source-format `"111"`) is not
    /// yet decoded — round 1 only covers the non-extended PTYPE
    /// header. See §5.1.4.
    ExtendedPtypeNotSupported,
    /// A reserved-valued or fixed-bit field inside the extended-PTYPE
    /// (PLUSPTYPE) header did not hold its spec-mandated value: a
    /// reserved UFEP code (§5.1.4.1), a missing OPPTYPE / MPPTYPE
    /// start-code-emulation guard or a set reserved bit (§5.1.4.2 /
    /// §5.1.4.3), a reserved picture-type code, or a forbidden CPFMT /
    /// EPAR / CPCFC field value (§5.1.5–§5.1.7).
    PlusPtypeReservedField,
    /// The extended-PTYPE (PLUSPTYPE) header signalled an optional mode
    /// or picture type whose remaining picture-header fields carry
    /// variable-length or externally-negotiated sub-bitstreams that are
    /// not staged for byte-level parsing here — Reference Picture
    /// Selection (§5.1.13–§5.1.17, Annex N), Reference Picture
    /// Resampling (§5.1.18, Annex P), or a scalability-layer picture
    /// type (B / EI / EP, Annex O). See §5.1.4.
    PlusPtypeUnsupported,
    /// The 17-bit Group of Blocks Start Code (GBSC, value
    /// `0000 0000 0000 0000 1`) was not present at the current
    /// bitstream position. See §5.2.2.
    BadGroupStartCode,
    /// The Group Number (GN) was outside the legal GOB-layer range.
    /// Values `0`, `30`, `31` are rejected — `0` is reserved to PSC,
    /// `30` is the EOSBS marker, `31` is the EOS marker. See §5.2.3.
    InvalidGroupNumber,
    /// GQUANT was `0`; §5.2.6 limits the natural-binary QUANT field
    /// to `1..=31`. Also returned by the macroblock parser when the
    /// caller passes an out-of-range QUANT through [`MbContext`].
    InvalidQuantiser,
    /// The MCBPC codeword (§5.3.2) was not present in Table 7 or
    /// Table 8 — either the leading-zero run had no corresponding
    /// bucket, or the suffix bits did not match any defined code.
    BadMcbpcCode,
    /// The CBPY codeword (§5.3.5) was not present in Table 12.
    BadCbpyCode,
    /// A motion-vector difference component (§5.3.7, Table 14) was
    /// not present in the spec table — the prefix consumed 13 bits
    /// without matching any of the 64 entries.
    BadMvdCode,
    /// The INTRADC 8-bit FLC (§5.4.1, Table 15) was one of the
    /// two forbidden codes (`0x00` or `0x80`).
    BadIntradcCode,
    /// A TCOEF VLC prefix (§5.4.2, Table 16) was not present in
    /// the spec table — 13 bits consumed without matching any of
    /// the 102 regular code-points or the ESCAPE prefix.
    BadTcoefCode,
    /// The ESCAPE-mode LEVEL field (§5.4.2) was one of the two
    /// forbidden codes (`0x00` or `0x80`) in baseline mode (the
    /// `0x80` code is reserved as EXTENDED-ESCAPE under Annex T,
    /// which round 4 does not implement).
    BadTcoefEscapeLevel,
    /// A TCOEF event's RUN advanced the zigzag scan position past
    /// the 64-coefficient block boundary. Returned by [`block::parse_block`]
    /// (§5.4.2 implicit constraint: RUN must keep the scan position
    /// inside the block).
    BadTcoefRunOverflow,
    /// The 17-bit Slice Start Code (SSC, value
    /// `0000 0000 0000 0000 1`) was not present at the current
    /// bitstream position. See §K.2.2.
    BadSliceStartCode,
    /// One of the §K.2 slice emulation-prevention bits (SEPB1 / SEPB2
    /// / SEPB3) was `0` where §K.2.3 / §K.2.6 / §K.2.9 require `1`.
    BadSliceEmulationPreventionBit,
    /// The 4-bit SSBI codeword was not one of the four
    /// Table K.1/H.263 entries (`1001` / `1010` / `1011` / `1101`).
    /// See §K.2.4.
    BadSliceSsbiCode,
    /// The MBA field of a slice header exceeded the picture's
    /// maximum macroblock number (Table K.2 last column). See §K.2.5.
    SliceMbaOutOfRange,
    /// The SWI field of a slice header gave an actual slice width
    /// (`SWI + 1`) greater than the picture's macroblock-per-row
    /// count. See §K.2.8.
    SliceSwiOutOfRange,
    /// A slice-header parse was attempted against a
    /// [`SliceHeaderContext`] whose picture geometry is smaller than
    /// sub-QCIF — Table K.2 / K.3 cannot resolve a field width.
    UnsupportedPictureGeometry,
    /// One of the §K.2.1 SSTUF stuffing bits before an SSC was `1`
    /// where the spec mandates `0`. See [`slice_header::skip_sstuf`].
    BadSliceStuffing,
    /// A PB-frame's temporal references were unusable: the §5.1.22
    /// TRB field was `0` (the codeword is "the number of
    /// non-transmitted pictures plus one", so its minimum legal
    /// value is 1), or the §G.4 TRD increment between the current
    /// picture's TR and the caller-supplied previous TR was zero
    /// (the §G.4 vector scaling divides by TRD, which is undefined
    /// when the two pictures are co-timed).
    BadPbTemporalReference,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotImplemented => write!(
                f,
                "oxideav-h263: not yet implemented in this orphan-rebuild round"
            ),
            Error::UnexpectedEof => {
                write!(f, "oxideav-h263: bitstream ended inside picture header")
            }
            Error::BadPictureStartCode => write!(
                f,
                "oxideav-h263: picture start code (PSC) not found at expected position"
            ),
            Error::BadPtypeFixedBits => write!(
                f,
                "oxideav-h263: PTYPE fixed bits (bit1=1, bit2=0) violated"
            ),
            Error::ForbiddenSourceFormat => write!(
                f,
                "oxideav-h263: PTYPE source format had forbidden value 000"
            ),
            Error::ExtendedPtypeNotSupported => write!(
                f,
                "oxideav-h263: extended PTYPE (PLUSPTYPE) path not yet supported"
            ),
            Error::PlusPtypeReservedField => write!(
                f,
                "oxideav-h263: a reserved/fixed-bit field in the PLUSPTYPE header had an illegal value"
            ),
            Error::PlusPtypeUnsupported => write!(
                f,
                "oxideav-h263: PLUSPTYPE signalled an optional mode (RPS/RPR/scalability layer) whose header fields are not yet parsed"
            ),
            Error::BadGroupStartCode => write!(
                f,
                "oxideav-h263: group-of-blocks start code (GBSC) not found at expected position"
            ),
            Error::InvalidGroupNumber => write!(
                f,
                "oxideav-h263: group number (GN) outside the legal GOB-layer range (1..=29)"
            ),
            Error::InvalidQuantiser => {
                write!(f, "oxideav-h263: GQUANT was 0 (legal range is 1..=31)")
            }
            Error::BadMcbpcCode => write!(
                f,
                "oxideav-h263: MCBPC variable-length code not found in Table 7/8"
            ),
            Error::BadCbpyCode => write!(
                f,
                "oxideav-h263: CBPY variable-length code not found in Table 12"
            ),
            Error::BadMvdCode => write!(
                f,
                "oxideav-h263: MVD component variable-length code not found in Table 14"
            ),
            Error::BadIntradcCode => write!(
                f,
                "oxideav-h263: INTRADC code 0x00 or 0x80 is forbidden per Table 15"
            ),
            Error::BadTcoefCode => write!(
                f,
                "oxideav-h263: TCOEF variable-length code not found in Table 16"
            ),
            Error::BadTcoefEscapeLevel => write!(
                f,
                "oxideav-h263: ESCAPE-mode TCOEF LEVEL was a forbidden code (0x00 or 0x80)"
            ),
            Error::BadTcoefRunOverflow => write!(
                f,
                "oxideav-h263: TCOEF RUN advanced the zigzag scan past coefficient 63"
            ),
            Error::BadSliceStartCode => write!(
                f,
                "oxideav-h263: slice start code (SSC) not found at expected position"
            ),
            Error::BadSliceEmulationPreventionBit => write!(
                f,
                "oxideav-h263: slice emulation prevention bit (SEPB1/SEPB2/SEPB3) was 0 (must be 1)"
            ),
            Error::BadSliceSsbiCode => write!(
                f,
                "oxideav-h263: SSBI was not one of the four Table K.1 codewords"
            ),
            Error::SliceMbaOutOfRange => write!(
                f,
                "oxideav-h263: slice MBA field exceeded the picture's macroblock count - 1"
            ),
            Error::SliceSwiOutOfRange => write!(
                f,
                "oxideav-h263: slice SWI field gave an actual slice width above the picture's MB-per-row count"
            ),
            Error::UnsupportedPictureGeometry => write!(
                f,
                "oxideav-h263: picture geometry too small for the Annex K slice-header tables"
            ),
            Error::BadSliceStuffing => write!(
                f,
                "oxideav-h263: SSTUF stuffing bit was 1 (must be 0 per §K.2.1)"
            ),
            Error::BadPbTemporalReference => write!(
                f,
                "oxideav-h263: PB-frame temporal reference unusable (TRB = 0 or TRD = 0)"
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Convenience alias for results returned from the parser surface.
pub type Result<T> = core::result::Result<T, Error>;

/// No-op codec registration — the round-1 scaffold parses headers but
/// has no decoder to register into the runtime context. The full
/// `Decoder` impl will land in a later round once macroblock decode
/// exists.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("h263", register);

/// Free function alias mapping a byte slice to
/// [`parse_picture_header`] — provided so callers do not need to
/// allocate a [`BitReader`] themselves.
pub fn parse_picture_header_from_bytes(data: &[u8]) -> Result<H263PictureHeader> {
    let mut reader = BitReader::new(data);
    parse_picture_header(&mut reader)
}

/// End-to-end intra-block reconstruction: takes a parsed
/// [`H263Block`] (in zigzag scan order, with INTRADC already applied
/// to slot 0 per Table 15) and the macroblock's QUANT, and produces
/// an 8×8 `u8` sample block ready to copy into the picture buffer.
///
/// This composes:
///
/// 1. §6.1 / §6.2.1 inverse-quant of AC coefficients (DC preserved
///    for INTRA),
/// 2. §6.2.2 clip of AC reconstruction levels to `[-2048, 2047]`,
/// 3. §6.2.3 / Figure 14 zigzag → 8×8 scatter,
/// 4. §6.2.4 inverse DCT in `f64` (Annex A.7-conformant by
///    construction),
/// 5. §6.3.2 clip to the 8-bit picture range `[0, 255]`.
///
/// `quant` is the QUANT from §5.2.6 / §5.3.6 (range `1..=31`); the
/// function clamps out-of-range values defensively.
pub fn reconstruct_intra_block(block: &H263Block, quant: u8) -> [u8; COEFFS_PER_BLOCK] {
    let mut scan = block.clone();
    dequant::dequantise_ac(&mut scan, quant, /* is_intra = */ true);
    let scattered = dequant::scatter_into_block(&scan.coefficients);
    idct::reconstruct_intra_samples(&scattered)
}

/// End-to-end INTER-block reconstruction: takes a parsed
/// [`H263Block`] (zigzag scan order; INTER has no separate INTRADC, so
/// slot 0 is an ordinary AC coefficient), the macroblock's QUANT, and
/// an 8×8 motion-compensated `prediction` block (from
/// [`motion::motion_compensate_block`]), and produces the
/// reconstructed 8×8 `u8` sample block.
///
/// This composes:
///
/// 1. §6.1 / §6.2.1 inverse-quant of *all* coefficients (INTER has no
///    DC bypass — slot 0 is processed under the standard formula),
/// 2. §6.2.2 clip of reconstruction levels to `[-2048, 2047]`,
/// 3. §6.2.3 / Figure 14 zigzag → 8×8 scatter,
/// 4. §6.2.4 inverse DCT in `f64` (Annex A.7-conformant by
///    construction),
/// 5. §6.3.1 summation of the IDCT residual with the
///    motion-compensated prediction,
/// 6. §6.3.2 clip to the 8-bit picture range `[0, 255]`.
///
/// `quant` is the QUANT from §5.2.6 / §5.3.6 (range `1..=31`); the
/// function clamps out-of-range values defensively.
pub fn reconstruct_inter_block_with_prediction(
    block: &H263Block,
    quant: u8,
    prediction: &[u8; COEFFS_PER_BLOCK],
) -> [u8; COEFFS_PER_BLOCK] {
    let mut scan = block.clone();
    dequant::dequantise_ac(&mut scan, quant, /* is_intra = */ false);
    let scattered = dequant::scatter_into_block(&scan.coefficients);
    let residual = idct::idct_8x8(&scattered);
    motion::reconstruct_inter_block(prediction, &residual)
}

#[cfg(test)]
mod lib_tests {
    use super::*;

    /// End-to-end DC-only INTRA block: INTRADC code `0x10` → Table 15
    /// reconstruction level `0x10 * 8 = 128`. With no AC coefficients,
    /// scatter places 128 at block (0, 0); the IDCT distributes
    /// 128/8 = 16 to every pixel; §6.3.2 clip is a no-op.
    #[test]
    fn intra_dc_only_block_reconstruct_is_uniform_field() {
        let mut block = H263Block::empty();
        block.coefficients[0] = 128; // INTRADC reconstruction level
        block.had_intradc = true;

        // QUANT is irrelevant for an INTRA block with no AC, since
        // the DC slot bypasses the formula — pick a representative
        // mid-range value.
        let samples = reconstruct_intra_block(&block, 8);
        assert!(samples.iter().all(|&p| p == 16));
    }

    /// DC = 800 → pixel = 100 (within `[0, 255]`); same with QUANT = 1.
    #[test]
    fn intra_dc_800_reconstructs_to_pixel_100() {
        let mut block = H263Block::empty();
        block.coefficients[0] = 800;
        block.had_intradc = true;
        let samples = reconstruct_intra_block(&block, 1);
        assert!(samples.iter().all(|&p| p == 100));
    }

    /// Hand-derived block: INTRADC = 1024 (the `0xFF` Table 15 special
    /// case) plus one AC at zigzag slot 1 with LEVEL = 1, QUANT = 1
    /// (odd). After dequant slot 1 becomes 1 * 3 = 3. Scatter puts
    /// scan-slot 0 at block position 0 (DC), scan-slot 1 at block
    /// position 1 — i.e. F(u=1, v=0) = 3 — per Figure 14. The IDCT
    /// gives
    /// `f(x, y) = (1024/8) + (3/4)·(1/√2)·cos(π(2x+1)/16)`
    /// ≈ `128 + 0.530·cos(π(2x+1)/16)`.
    /// Cosine ranges roughly `[-0.981, +0.981]`, so the AC term is
    /// at most ≈ ±0.520 — every pixel rounds within ±1 of 128 after
    /// §6.3.2 clip. The pattern is independent of `y`.
    #[test]
    fn intra_dc_plus_small_ac_at_qp1() {
        let mut block = H263Block::empty();
        block.coefficients[0] = 1024;
        block.coefficients[1] = 1;
        block.had_intradc = true;
        let samples = reconstruct_intra_block(&block, 1);
        // Every pixel is within ±1 of 128, and the pattern is
        // y-invariant: row 0 must equal row 1 must equal ... must
        // equal row 7.
        for y in 1..BLOCK_DIM {
            for x in 0..BLOCK_DIM {
                assert_eq!(
                    samples[y * BLOCK_DIM + x],
                    samples[x],
                    "(x={}, y={}) differs from row 0",
                    x,
                    y
                );
            }
        }
        for (x, &p) in samples[..BLOCK_DIM].iter().enumerate() {
            let delta = (p as i32 - 128).abs();
            assert!(delta <= 1, "x={} pixel={} delta={}", x, p, delta);
        }
        // The horizontal-cosine modulation must give exactly one
        // sign reversal across the row (cos(π(2x+1)/16) crosses zero
        // between x=3 and x=4): pixels 0-3 should be ≥ 128, pixels
        // 4-7 should be ≤ 128.
        for (x, &p) in samples[..4].iter().enumerate() {
            assert!(p >= 128, "x={} = {}", x, p);
        }
        for (offset, &p) in samples[4..BLOCK_DIM].iter().enumerate() {
            let x = 4 + offset;
            assert!(p <= 128, "x={} = {}", x, p);
        }
    }

    /// QUANT = 2 (even) AC reconstruction: LEVEL = 1 → |REC| = 5.
    /// Scattered into F(1, 0), the IDCT produces a horizontal cosine
    /// modulation with amplitude (5/4)·(1/√2) ≈ 0.884 around the DC
    /// level. Pick DC = 1024 (pixel 128) and assert each pixel is
    /// within ±1 of 128.
    #[test]
    fn intra_even_quant_ac_reconstruction() {
        let mut block = H263Block::empty();
        block.coefficients[0] = 1024;
        block.coefficients[1] = 1;
        block.had_intradc = true;
        let samples = reconstruct_intra_block(&block, 2);
        for (i, &p) in samples.iter().enumerate() {
            let delta = (p as i32 - 128).abs();
            assert!(delta <= 1, "pixel {} = {} (delta {})", i, p, delta);
        }
    }

    /// Saturation: an INTRADC = 2032 alone (DC pixel = 254) plus a
    /// strong AC term can push some pixels above the §6.2.4 [-256,
    /// +255] window — the §6.3.2 clip pins display values to 255.
    /// Pick a high QUANT and a max LEVEL in slot 1 (zigzag F(1,0));
    /// the cosine modulation has amplitude (|REC|/4)·(1/√2). With
    /// |REC| = 31·255 = 7905 clipped to 2047 (§6.2.2), the amplitude
    /// is 2047/(4·√2) ≈ 361.7, well above the 1-pixel-of-room from
    /// the DC level. So pixels on the positive lobe must hit the
    /// §6.3.2 ceiling of 255, and pixels on the negative lobe must
    /// hit the §6.3.2 floor of 0.
    #[test]
    fn intra_reconstruction_clipped_at_both_picture_extremes() {
        let mut block = H263Block::empty();
        block.coefficients[0] = 2032; // INTRADC code 0xFE → 2032
        block.coefficients[1] = 127; // pre-dequant LEVEL on scan slot 1
        block.had_intradc = true;
        let samples = reconstruct_intra_block(&block, 31);
        // Some pixel must hit 255 (positive lobe of cosine).
        assert!(
            samples.contains(&255),
            "expected ≥1 pixel clipped at 255: {:?}",
            samples
        );
        // Some pixel must hit 0 (negative lobe of cosine).
        assert!(
            samples.contains(&0),
            "expected ≥1 pixel clipped at 0: {:?}",
            samples
        );
    }

    /// Empty INTER block (no AC, no DC, since INTER has no INTRADC)
    /// reconstructs to all zeros under the §6.2.4 + §6.3.2 path.
    /// This is the §A.8 "all zeros in, all zeros out" invariant.
    #[test]
    fn empty_block_reconstructs_to_zero_field() {
        let block = H263Block::empty();
        // We invoke the lower-level functions directly because
        // `reconstruct_intra_block` adds the §6.3.2 [0, 255] clip;
        // we want to exercise that an all-zero coefficient array is
        // already at 0 pre-clip.
        let scattered = dequant::scatter_into_block(&block.coefficients);
        let pixels = idct::idct_8x8(&scattered);
        assert!(pixels.iter().all(|&p| p == 0));
    }
}
