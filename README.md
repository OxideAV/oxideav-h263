# oxideav-h263

[![CI](https://github.com/OxideAV/oxideav-h263/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-h263/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-h263.svg)](https://crates.io/crates/oxideav-h263) [![docs.rs](https://docs.rs/oxideav-h263/badge.svg)](https://docs.rs/oxideav-h263) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A pure-Rust ITU-T H.263 baseline video **codec** for the
[oxideav](https://github.com/OxideAV/oxideav) framework, built
clean-room against [ITU-T Recommendation H.263 (01/2005)][spec].

## Status

Full baseline **decoder**, plus a growing **encoder**. The decoder
implements the H.263 baseline picture / GOB / macroblock / block layers
and reconstructs INTRA and INTER pictures end-to-end, plus a wide set of
optional Annexes (D, F, I, J, K, M, N, O, P, Q, S, T).

The encoder (round 376 onward) produces baseline **INTRA (I-)** and
**INTER (P-)** pictures plus a growing set of optional modes:
`encoder::encode_intra_picture` / `encode_inter_picture` /
`encode_inter_picture_motion` (SAD search + half-pel refinement with
§6.1.1 predictor replay), `encode_inter_picture_umv` (**Annex D**
extended-range motion with the exact §D.2 pair-selection inverse),
`encode_inter_picture_ap` (**Annex F** INTER4V four vectors per
macroblock with §F.3 OBMC-exact prediction, two-pass),
`encode_pb_picture` (**Annex G** PB-frames: P-part + §G.4/§G.5
bidirectionally-predicted B-part with BQUANT residuals),
`encode_intra_picture_dquant` (§5.3.6 per-macroblock DQUANT),
`encode_intra_picture_gobs` / `encode_inter_picture_gobs` (§5.2 GOB
headers with per-GOB GQUANT + segmented MV prediction),
`encode_intra_picture_aic` / `encode_intra_picture_aic_auto` /
`encode_intra_picture_aic_mq` / `encode_intra_sequence_aic`
(**Annex I** Advanced INTRA Coding: §I.2 per-macroblock INTRA_MODE
with a rate-driven mode decision, §I.3 coefficient-domain DC/AC
prediction from the encoder's own reconstructed neighbours, the
Table I.2 separate INTRA VLC, and the optional **Annex T** §T.3
chroma `QUANT_C` + §T.4 EXTENDED-ESCAPE range), and the closed-loop
GOP driver `encode_sequence` (I + P GOPs predicting from the
encoder's own decoded reconstruction — no drift; optional §5.1.27
EOS). Everything is built bottom-up from reusable layers —
`encoder_vlc`, `fdct`, `encoder_block` (§5.4), `encoder_aic` (§I.3
block plan + Table I.2 emit), `encoder_mb` (§5.3), `encoder_motion`
(§6.1.1/§F.2/§D.2 estimation + predictor replay) and `encoder`
(§5.1 / §5.2 picture layer) — each round-trip-verified against the
decoder, including a mixed-mode I + P + UMV-P + AP-P + PB elementary
stream decoded end-to-end by `decode_sequence`. The AIC closed loop
reconstructs every block through the exact decoder primitive so
encoder and decoder never drift (flat AIC pictures are byte-exact).
Annex K slice-structured encoding is the next milestone.

`register()` is currently a no-op pending a frame-yielding
`oxideav_core::Decoder` adapter — callers drive the decoder through the
free `decode_picture` entry point and the encoder through
`encode_intra_picture`.

Any path that is not yet wired returns `Error::NotImplemented` rather
than silently guessing.

## What works

The high-level `decode_picture` driver walks every GOB of a picture
top-to-bottom and every macroblock left-to-right, reconstructing a
planar 4:2:0 `YuvFrame`:

* **Picture layer (§5.1)** — PSC, Temporal Reference, non-extended
  PTYPE (split-screen / document-camera / freeze-release indicators,
  source format sub-QCIF..16CIF, INTRA / INTER coding type, optional
  Annex D/E/F/G mode flags), and the extended PLUSPTYPE header
  (§5.1.4 onward: UFEP / OPPTYPE / MPPTYPE + CPM / PSBI / CPFMT /
  EPAR / CPCFC / ETR / UUI / SSS), plus the §5.1.11–§5.1.16
  scalability / reference-picture-selection fixed-length fields
  ELNUM / RLNUM / RPSMF / TRPI / TRP and the BCI codeword (the
  variable-length §5.1.17 BCM and §5.1.18 RPRP payloads are refused).
* **GOB layer (§5.2)** — GBSC, Group Number, GOB Frame ID, GQUANT
  (CPM = "0" branch), plus the §5.2.2 first-GOB (group-number-0)
  header elision: the `decode_picture_no_gob0_header` entry point reads
  the §5.1.19 PQUANT / §5.1.20 CPM picture-header fields, the §5.1.24
  PEI / §5.1.25 PSUPP extension loop, decodes the header-less GOB 0 at
  QUANT = PQUANT, and treats every later GOB's header as **optional**
  (§5.2): `gob_header_present` probes for a GBSC (after any §5.2.1
  GSTUF byte-alignment); a present header primes a fresh QUANT and
  video-picture segment, an absent one continues the previous segment
  at the carried-over QUANT — so streams that omit empty GOB headers
  (the reference encoder for the standard formats) decode correctly
  (CPM = "1" refused).
* **Elementary-stream demux** — `decode_sequence` splits a multi-picture
  stream on byte-aligned Picture Start Codes (§5.1.1 / §5.1.28),
  decoding each picture and threading the reconstructed frame forward as
  the INTER reference for the next. It dispatches per picture on the
  §5.1.3 source-format selector: a baseline-PTYPE picture takes the
  §5.2.2 GOB-0-elided path, an extended-PTYPE (PLUSPTYPE / H.263+)
  picture routes through the full PLUSPTYPE driver (H.263+ Annex modes,
  slice-structured layout, reference resampling) while the §5.1.4.4 /
  §5.1.4.5 inherited extended-mode snapshot threads forward (a baseline
  picture resets it). The extended GOB and Annex N RPS paths read the
  §5.1.19 PQUANT that follows the PLUSPTYPE / CPFMT / RPRP block and
  elide the group-number-0 GOB header (§5.2.2), so the H.263+ wire a
  real encoder emits decodes end-to-end. Custom Picture Clock Frequency
  (§5.1.7 / §5.1.8) is accepted (CPCFC / ETR are framed but timing-only,
  with no effect on reconstructed pixels).
* **Macroblock layer (§5.3)** — COD, MCBPC (Tables 7 / 8), CBPY
  (Table 12), DQUANT, and MVD / MVD2-4 (Table 14).
* **Block layer (§5.4)** — INTRADC 8-bit FLC (Table 15) and TCOEF
  VLC (Table 16, 102 regular code-points + ESCAPE), accumulated in
  zigzag scan order.
* **INTRA reconstruction (§6.1 / §6.2 / §6.3.2)** — H.261-style
  inverse quantisation (modulo-2 oddifier), `[-2048, +2047]` AC clip,
  zigzag → 8×8 scatter (Figure 14), inverse DCT (f64 kernel meeting
  the Annex A.7 accuracy budget), sample clip to `[0, 255]`.
* **INTER reconstruction (§6.1.1 / §6.1.2 / §6.3.1)** — differential
  motion-vector reconstruction with the Figure-12 median predictor and
  candidate border-decision rules, Table 18 chroma vector derivation,
  half-pixel bilinear interpolation (Figure 13) with §D.1 edge
  replication, and residual summation + clip. Skipped macroblocks
  (COD = 1) copy the reference with a zero MV.
* **Annex D §D.2** — Unrestricted Motion Vector mode (PLUSPTYPE-absent
  extended `[-63, 63]` half-pel range with predictor-dependent
  difference-pair selection).
* **Annex F §F.2 / §F.3** — Advanced Prediction: four-motion-vector
  candidate-predictor redefinition (Figure F.1, threading each
  just-reconstructed vector back in as an intra-macroblock candidate),
  Table F.1 sixteenth-pixel chroma derivation, and overlapped block
  motion compensation (OBMC) over the Figures F.2 / F.3 / F.4 weight
  matrices with the `Zero` / `Current` / `Vector` remote-MV
  substitution rules. OBMC covers **every** coded INTER macroblock of
  an AP picture (a one-vector macroblock is "four vectors with the
  same value"), and the B2/B4 right-half remotes read the **actual**
  vector of the macroblock to the right — the luminance reconstruction
  is deferred one macroblock so the later-parsed right neighbour is
  known. INTER4V / INTER4V+Q macroblocks reconstruct end-to-end when
  Advanced Prediction is signalled.
* **Annex I §I.2 / §I.3** — Advanced INTRA Coding: the INTRA_MODE VLC
  (Table I.1), the two alternate DCT scans (Figure I.2) and scan
  selection, the separate INTRA-coefficient VLC (Table I.2), the
  no-dead-zone modified inverse quantisation, the `clipAC` /
  `oddifyclipDC` clips, and the DC/AC prediction reconstruction.
* **Annex J §J.3** — in-loop deblocking edge filter (four-tap formula
  + full Table J.2 STRENGTH lookup + horizontal-before-vertical
  ordering + picture-edge skip), opt-in via `DecodeOptions::deblock`.
  Deblocking-Filter mode also turns on the §5.3.8 / Table J.1 **four
  motion vectors per macroblock** element *without* the §F.3 OBMC
  element: when `deblock` is set, an INTER4V / INTER4V+Q macroblock
  carries MVD2-4 (gated by the new `MbContext::deblocking_filter`) and
  each 8×8 luma block is predicted with plain half-pel motion
  compensation by its own §F.2 four-vector-derived vector (the
  median-predictor / §D.1 edge replication / Table F.1 chroma derivation
  are shared with the Advanced-Prediction path; only the OBMC blend is
  skipped).
* **Annex K §K.2** — Slice Structured mode: the slice-layer header
  parse (SSC + SEPB1/2/3 + optional SSBI + MBA + SQUANT + optional
  SWI + GFID, plus the first-slice reduced form) and the free-running
  (non-Rectangular-Slice) end-to-end decode driver, including the
  §5.1.24 PEI / §5.1.25 PSUPP picture-header tail consumed before the
  first reduced slice header. The `slice-structured-mode` QCIF I+P+P
  conformance fixture decodes byte-exact within the Annex A.7 tolerance.
* **Annex S §S.2 / §S.3** — Alternative INTER VLC mode: each INTER
  coefficient block is interpreted with the baseline INTER VLC (Table
  16) first and re-interpreted with the Annex I INTRA VLC (Table I.2)
  only when the INTER reading would address coefficients past slot 63 of
  the block (§S.2.2 step 3, keyed on the run-overflow signal — both
  tables share one codeword inventory so the re-decode consumes the same
  bits); and, when both chrominance blocks of an INTER macroblock carry
  coefficients (`CBPC5 = CBPC6 = 1`), the CBPY codeword is the Table 12
  INTRA pattern (no INTER complement, §S.3). Wired into both the baseline
  single-MV INTER path and the Annex K Slice-Structured driver (via the
  shared per-macroblock reconstruction), auto-activated from the
  PLUSPTYPE OPPTYPE bit 13; refused only when combined with Advanced
  Prediction / INTER4V or PB-frames. The `alt-inter-vlc` (AIV + AIC + MQ
  + slice-structured) QCIF I+P+P conformance fixture decodes byte-exact.
* **Annex T** — Modified Quantization mode: the §T.2 variable-length
  DQUANT parser, the §T.3 chrominance `QUANT_C` step, and the §T.4
  EXTENDED-ESCAPE / EXTENDED-LEVEL extended coefficient range, driving
  an MQ-active picture reconstruction end-to-end for the baseline INTRA
  / INTER path, the Annex I Advanced INTRA Coding path, **and** the
  Annex K Slice-Structured driver (the §T.3 `QUANT_C` chroma dequant and
  the §T.5-rule-2 EXTENDED-ESCAPE extension to the Table I.2 VLC thread
  through the shared per-macroblock reconstruction on all three). The
  `advanced-intra-coding` (AIC + MQ + slice-structured) conformance
  fixture decodes byte-exact.
* **Annex Q §Q.6** — Reduced-Resolution Update mode prediction-error
  up-sampling: the 8×8 reduced-resolution reconstructed prediction-error
  block is up-sampled to a 16×16 block with the block-closed §Q.6.1
  interior filter (Figure Q.8 9/3/3/1 bilinear weights) and the §Q.6.2
  boundary filter (Figure Q.9 corner copy + 3:1 edge interpolation), all
  with §Q.6 division-by-truncation semantics. Exposed as the pure
  `upsample_prediction_error` primitive.
* **Annex Q §Q.7** — Reduced-Resolution Update block boundary filter run
  along the edges of the 16×16 reconstructed blocks: the §Q.7.1 default
  two-tap kernel (`A1 = (3A+B+2)/4`, `B1 = (A+3B+2)/4` with truncating
  division) and the §Q.7.2 Deblocking-Filter-mode variant (the §J.3
  four-tap filter with `STRENGTH = +∞`, which collapses `UpDownRamp` to
  the identity so `d1 = (A−4B+4C−D)/8`). Both honour the §J.3 edge
  ordering (horizontal-before-vertical), the coded-MB filter-on
  condition, and the picture-edge skip, with slice/ISD-segment skips
  surfaced through a per-edge condition closure. Exposed as the
  `rru_filter_plane` plane-level driver (plus the `rru_default_tap`
  kernel); the surrounding 32×32-macroblock RRU decode pipeline
  (pseudo-MV §Q.4, enlarged OBMC §Q.5, reference extension §Q.3) is not
  yet wired.
* **Annex N §N.4.1 / §N.5** — Reference Picture Selection mode (forward
  channel) end-to-end, including **per-GOB** re-selection: the
  `RpsReferenceStore` picture memory keys decoded anchors by their 10-bit
  Temporal Reference (ETR ∥ TR), and `decode_picture_layer_rps` selects
  the §N.4.1.4 prediction reference by the picture-layer TRP (the stored
  picture whose TR equals TRP, or the most recent anchor when TRP is
  absent). For INTER-pictures it further parses the §N.4.1 GOB-layer
  NEWPRED fields (`annex_n::parse_gob_newpred_fields`: TRI / TR / TRPI /
  TRP + BCI, Figure N.2) after each GOB header and re-selects that GOB's
  reference from the store "instead of the last decoded picture" (§N.5) —
  so a single picture can predict different GOBs from different stored
  references (a header-less GOB keeps the previous segment's reference,
  TRP being valid "until the next PSC, GSC or SSC"). The same per-segment
  re-selection threads through the Annex K Slice-Structured driver: each
  subsequent slice's NEWPRED fields (Figure N.3) re-select that slice's
  reference, while the first reduced-header slice keeps the picture-layer
  reference (parallel to GOB 0). A per-segment TRP not in the store
  surfaces the §N.5 forced-INTRA-update case as
  `Error::NotImplemented`. The §N.4.2 back-channel BCM is out of scope
  (decoder → encoder; no forward-channel pixel effect; a present
  GOB-layer BCI of `"1"` is refused with `Error::BadBackChannelMessage`).
* **Annex P §P.2 / §P.3** — Reference Picture Resampling: the §P.3 /
  §P.4.2 integer bilinear warp engine (`resample_yuv`) resamples the
  reference picture before motion compensation, driven from the §P.3
  corner displacements, the virtual-frame `H' / V'` powers of two, the
  §P.2.3 fill mode (clip / black / gray / color) and the §P.3 `RCRPR`
  rounding control. Both invocation paths reach pixels through
  `decode_picture_layer`: the §P.1 **implicit** resolution-change case
  (a size-mismatched reference, zero warp / clip / 1/16-pixel) and the
  §P.2 **explicit** RPRP field (WDA + eight Table-D.3 warping parameters
  + fill mode, parsed for INTER / B / Improved-PB pictures). The
  EP-picture explicit-RPR lower-layer-refinement case is not yet staged.
* **Annex G §G.1–§G.5** — PB-frames decode end-to-end, both through the
  per-layer `decode_pb_picture` driver (test-convention wire layout) and —
  new this round — through the headline `decode_sequence` streaming entry
  point: a baseline-PTYPE INTER picture that signals PB-frames mode
  (PTYPE bit 13) is routed to `decode_pb_picture_no_gob0_header`, which
  reads the spec-conformant baseline picture-header tail a real encoder
  emits (§5.1.19 PQUANT, §5.1.20 CPM, §5.1.22 TRB, §5.1.23 DBQUANT,
  §5.1.24/§5.1.25 PEI/PSUPP), elides the group-number-0 GOB header (§5.2.2)
  and tolerates the §5.2 optional later GOB headers. The decoded (B, P)
  pair is spliced into the frame sequence in display order (B before P);
  only the P-part advances the prediction reference and the §G.4 TR the
  next PB-frame scales against. Per macroblock the §5.3 / Figure 10 / Table
  10 PB-frame layer (COD, MCBPC, MODB, CBPB, CBPY, DQUANT, MVD, MVDB) drives
  the six P-blocks then the six §G.4 / §G.5 bidirectionally-predicted
  B-blocks with the Table-6 BQUANT dequant. SAC / Advanced Prediction / AIC
  combinations are refused (§G.1 bars the PLUSPTYPE-gated modes).
* **Annex M §M.1–§M.4** — Improved PB-frames decode end-to-end, both
  through the per-layer `decode_improved_pb_picture` driver and — new this
  round — through `decode_sequence`: an extended-PTYPE picture whose
  §5.1.4.3 MPPTYPE picture-type is `"010"` is detected before dispatch and
  routed to `decode_improved_pb_picture_with_inherited`, which threads the
  §5.1.4.4 inherited-state and returns the decoded (P, BPB) pair plus the
  next-picture OPPTYPE snapshot. The pair is spliced into the sequence in
  display order (BPB before P); only the P-part advances the reference and
  §G.4 TR. Per macroblock the §M.4 / Table M.1 MODB form drives the §M.2
  coding modes (bidirectional / forward with the §M.2.2 left-neighbour MVDB
  predictor / backward). Annex K + Improved-PB, Advanced Prediction, UMV
  and AIC combinations are refused (unstaged §M sub-cases).
* **Annex O §O.4 / §O.5** — temporal-scalability **B-pictures** decode
  end-to-end (`decode_b_picture` / `decode_b_picture_layer`): the Table
  O.1 MBTYPE layer drives Forward / Backward / Bi-dir / INTRA
  reconstruction against the two temporally surrounding anchors, with
  *separate* §O.5.1 forward / backward predictor grids (same-type-only
  median), and §O.5.2 **direct mode** (the COD-skipped row and the
  explicit Direct / Direct+Q rows) deriving the forward / backward
  vectors from the co-located subsequent-anchor vector by the §G.4
  scaling with `MVD = 0` (a co-located INTRA macroblock contributing the
  §O.5.2 zero vector). The EI- and EP-picture enhancement layers also
  reconstruct end-to-end (`decode_ei_picture` / `decode_ep_picture` —
  upward, forward, bidirectional and INTRA prediction). The §N.4.2
  back-channel and the B-picture INTER4V / Advanced-Prediction submodes
  are out of scope.
* **Annex O §O.6** — spatial-scalability reference up-sampling: the
  Figure O.8 / O.9 2-D and Figure O.10 / O.11 1-D (horizontal / vertical)
  interpolation filters (`upsample_plane_2d` /
  `upsample_plane_1d_horizontal` / `upsample_plane_1d_vertical`),
  threaded into the EI / EP upward-prediction path so a factor-of-two
  smaller reference layer (§O.1.3) is up-sampled to the enhancement
  geometry before prediction (a non-factor-of-two mismatch is still
  refused with `BadScalabilityReferenceGeometry`).

## Usage

The high-level entry point decodes a whole picture in one call:

```rust,ignore
use oxideav_h263::{decode_picture, DecodeOptions, YuvFrame};

// Decode an INTRA (I) picture — no reference frame needed.
let frame: YuvFrame = decode_picture(&bytes, None, DecodeOptions::default())?;
assert_eq!((frame.luma_width, frame.luma_height), (176, 144));

// Decode the next INTER (P) picture against the previous frame, with
// the Annex J deblocking filter enabled.
let next = decode_picture(
    &p_bytes,
    Some(&frame),
    DecodeOptions { deblock: true, ..DecodeOptions::default() },
)?;
```

For a complete baseline elementary stream (one or more pictures, as
produced by a real encoder), `decode_sequence` splits on Picture Start
Codes and threads the INTER reference automatically:

```rust,ignore
use oxideav_h263::{decode_sequence, DecodeOptions};

// `stream` is a raw .h263 elementary stream (I + P + P + ...).
let frames = decode_sequence(&stream, DecodeOptions::default())?;
```

The lower-level per-layer parsers and per-block reconstruction
primitives the driver composes remain public for callers that need
finer control:

```rust,ignore
use oxideav_core::bits::BitReader;
use oxideav_h263::{
    parse_block, parse_gob_layer, parse_macroblock, parse_picture_header,
    reconstruct_intra_block, BlockContext, H263SourceFormat, MbContext,
};

let mut r = BitReader::new(&bytes);
let pic = parse_picture_header(&mut r)?;
assert_eq!(pic.source_format.luma_dimensions(), Some((176, 144)));

// `r` is now at the first bit of the GOB layer (after any GSTUF).
let gob = parse_gob_layer(&mut r)?;

// One macroblock per spec §5.3, threading the picture's coding
// type and the GOB's QUANT through MbContext.
let mb = parse_macroblock(
    &mut r,
    MbContext {
        picture_coding_type: pic.coding_type,
        advanced_prediction: pic.advanced_prediction,
        quantiser_before: gob.quantiser,
    },
)?;

// One block of the macroblock per §5.4, with the caller deriving the
// INTRADC / coefficient presence from the MB type + CBP bits.
let block = parse_block(
    &mut r,
    BlockContext {
        has_intradc: mb.mb_type.unwrap().is_intra(),
        has_coefficients: false,
    },
)?;

// §6.1 / §6.2 / §6.3.2 intra-block reconstruction.
let samples_8x8 = reconstruct_intra_block(&block, gob.quantiser);
```

## Not yet implemented

* GOB-0-header elision in the *legacy* `decode_picture` /
  `decode_picture_layer` / PB / Annex-K-slice drivers (those still expect
  every GOB, including the topmost, to carry a header on the wire). The
  spec-conformant §5.2.2 elision is available through the dedicated
  `decode_picture_no_gob0_header` baseline entry point.
* Multi-picture sequence demuxing for the legacy per-layer baseline
  drivers (`decode_picture` / PB / Annex-K-slice) — those stay
  caller-side. The `decode_sequence` driver handles both baseline-PTYPE
  and extended-PTYPE (PLUSPTYPE / H.263+) INTRA / INTER picture streams.
* Annex N (Reference Picture Selection) **back-channel**: the
  §5.1.17 / §N.4.2 Back-Channel Message (videomux BCM ACK/NACK) is not
  staged — it flows decoder → encoder on a separate logical channel and
  does not affect forward-channel pixels (a present BCM is refused). The
  §N.4.1 per-segment TRP re-selection now decodes to pixels on both the
  GOB-layer and the Annex K slice-layer paths (see the supported list).
* Slice-boundary / Independent-Segment-Decoding deblock skip rules.
* Annex G PB-frames and Annex M Improved PB-frames now both decode
  end-to-end through `decode_sequence` (see the supported list). Still
  unstaged within those modes: Advanced Prediction / INTER4V B-blocks, UMV
  over-boundary forward vectors (Annex M), AIC, SAC, and the Annex K
  Slice-Structured + Improved-PB combination.
* Annex K Rectangular Slice submode, Annex K with Advanced Prediction /
  CPM, and Arbitrary Slice Ordering.
* Annex O CPM-multiplexed / Advanced-Prediction / SAC / Annex-K-slice
  enhancement-layer pictures (refused on the EI / EP / B paths); the
  Annex P explicit-warp resampling engine (see the supported list) is
  not yet threaded into the EP / spatial-scalability path (only the §O.6
  factor-of-two upsample is wired there).
* Annex Q Reduced-Resolution Update mode end-to-end (the §Q.6
  prediction-error up-sampling primitive and the §Q.7 block boundary
  filter are implemented as pure primitives; the 32×32 macroblock layer,
  §Q.4 pseudo-MV reconstruction, §Q.5 enlarged OBMC and §Q.3 reference
  extension are not yet wired, so the §Q.7 driver is not yet invoked from
  an end-to-end RRU reconstruction).
* GSBI (CPM = "1"); the EOSBS end marker (the §5.1.27 EOS is emitted
  by the encoder and transparently skipped by `decode_sequence`).
* Encoder: Annex K slice-structured encoding; UMV + AP combined mode;
  PB-frames with non-zero MVDB / Annex M Improved-PB; INTRA-refresh
  inside the AP and PB paths; AIC INTRA macroblocks inside a P-picture
  (only whole AIC I-pictures encode so far); on-wire PLUSPTYPE
  signalling of the §I / §T modes (they currently ride a baseline PTYPE
  and require `DecodeOptions { aic / modified_quant }`); rate control
  beyond the per-MB DQUANT / per-GOB GQUANT / per-MB INTRA_MODE
  primitives.
* `oxideav_core::Decoder` registration; `register()` is a no-op
  pending a frame-yielding decoder adapter.

## Testing

The crate carries an extensive unit-test suite over synthetic buffers
built with the spec's bit layout (round-tripped via
`oxideav_core::bits::BitWriter`), including full-table round-trips for
Tables 7 / 8 / 12 / 14 / 16, the inverse-quantisation invariants, IDCT
accuracy against the Annex A error budget, motion / OBMC / deblock /
AIC / PLUSPTYPE / slice-header coverage, the full Table I.2 INTRA VLC
encode round-trip, the §I.3 block-plan closed loop, and end-to-end
picture-decode tests. `tests/encode_roundtrip.rs` additionally drives
the public encode API back through the decoder, including the Annex I
AIC I-picture / auto-mode / AIC+MQ / AIC-sequence encoders (flat AIC
pictures byte-exact, AC-bearing content within the round-trip
tolerance).

`tests/fixture_decode.rs` adds end-to-end **conformance** tests against
real H.263 elementary streams (the reference encoder) vendored under
`tests/fixtures/`: sub-QCIF / QCIF / CIF I-only, a QCIF I+P+P sequence,
the QP=2 / QP=31 quantiser-boundary keyframes, and an H.263+ (PLUSPTYPE)
QCIF I+P+P stream (`h263p-modern`) that exercises the `decode_sequence`
extended-PTYPE dispatch + custom-PCF framing + GOB-0 elision.
Because §6.2 leaves the inverse-transform arithmetic undefined
and Annex A.7 only bounds the per-pixel peak error at 1, AC-bearing
output is asserted within that ±1 tolerance; the flat sub-QCIF keyframe
(no AC) is checked byte-exact plus a SHA-256 of its reference plane.

Run with `cargo test -p oxideav-h263`.

## License

MIT — see [LICENSE](./LICENSE).

[spec]: https://www.itu.int/rec/T-REC-H.263
