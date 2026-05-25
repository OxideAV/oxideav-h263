# oxideav-h263

A pure-Rust ITU-T H.263 baseline video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild round 11 — Annex F §F.2 Advanced Prediction mode
four-motion-vector candidate-predictor redefinition (Figure F.1) and
Table F.1 sixteenth-pixel chrominance vector derivation, as pure
transformations (`LumaBlockIndex` / `Mb4MvNeighbourhood` /
`select_4mv_candidates` / `chroma_mv_4mv`), on top of round 10's Annex
D §D.2 Unrestricted Motion Vector mode (extended `[-63, 63]` half-pel
per-component range with predictor-dependent difference-pair selection,
PLUSPTYPE-absent case) + round 9's full-picture decode driver (baseline
single-MV path: INTRA / INTER / skipped macroblocks, §6.1.1 Figure-12
MV prediction, optional Annex J deblocking) + round 8's picture + GOB +
macroblock headers + block data + intra-block reconstruction + P-frame
motion compensation and INTER-block reconstruction + Annex J deblocking
filter + Annex I Advanced INTRA Coding scan/mode layer.** The
prior implementation was retired on 2026-05-18 under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav/blob/master/docs/IMPLEMENTOR_ROUND.md):
the encoder VLC tables were declared as mirrors of a sibling crate's
tables whose own provenance has been retired. The transitive
contamination of the table values could not be defended; master
history was fully erased per the Hat-3 cold-enforcement procedure.

The crate is being re-built clean-room against ITU-T Recommendation
H.263 (01/2005). The current master implements §5.1 (picture layer),
§5.2 (GOB layer up through GQUANT), §5.3 (macroblock header through
MVD2-4), §5.4 (block-layer INTRADC + TCOEF), §6.1 / §6.2 / §6.3.2
(intra-block reconstruction = inverse-quant + zigzag scatter + IDCT +
sample clip), §6.1.1 / §6.1.2 / §6.3.1 (P-frame motion-vector
reconstruction, half-pel bilinear interpolation, and INTER-block
prediction + residual summation), Annex J §J.3 (in-loop block-edge
deblocking filter with the full Table J.2 STRENGTH lookup), and the
Annex I §I.2 / §I.3 Advanced INTRA Coding scan-and-mode layer
(INTRA_MODE VLC + the two alternate DCT scans + scan selection) for
the non-PB-frame baseline:

* §5.1.1 — Picture Start Code (PSC), 22 bits, value `0x000020`.
* §5.1.2 — Temporal Reference (TR), 8 bits at the standard CIF
  picture clock frequency.
* §5.1.3 — Type Information (PTYPE) in its non-extended form (13 bits):
  split-screen / document-camera / freeze-release indicators,
  source-format field (`001` sub-QCIF .. `101` 16CIF, plus the
  reserved `110` and the `111` extended-PTYPE escape), picture coding
  type (INTRA / INTER), and Annex D/E/F/G optional-mode flags.
* §5.2.2 — Group of Blocks Start Code (GBSC), 17 bits, value
  `0000 0000 0000 0000 1`.
* §5.2.3 — Group Number (GN), 5 bits; the parser accepts the union
  of the standard and custom picture-format ranges (`1..=29`) and
  rejects `0` (PSC overlap), `30` (EOSBS marker), `31` (EOS marker).
* §5.2.5 — GOB Frame ID (GFID), 2 bits; consumed and exposed for
  future inter-GOB continuity enforcement.
* §5.2.6 — Quantizer Information (GQUANT), 5 bits; QUANT range
  `1..=31`.
* §5.3.1 — Coded macroblock indication (COD), 1 bit, INTER
  pictures only.
* §5.3.2 — Macroblock type & CBPC (MCBPC), variable length; full
  Table 7 (I-pictures, 9 codes) and Table 8 (P-pictures, 25
  codes, including type-5 INTER4V+Q points reserved for
  PLUSPTYPE + Annex F/J).
* §5.3.5 — Coded Block Pattern for luminance (CBPY), variable
  length; full Table 12 (16 patterns), `CBPY(INTRA)` orientation.
* §5.3.6 — Quantizer Information (DQUANT), 2 bits in the
  baseline form, with QUANT clipped to `1..=31` after the
  differential.
* §5.3.7 / §5.3.8 — Motion Vector Data (MVD + MVD2-4), variable
  length; full Table 14 (64 codes). Components returned in
  half-pel units as signed `i8` in `[-32, +31]`.
* §5.4.1 — DC coefficient for INTRA blocks (INTRADC), 8-bit FLC
  per Table 15: codes `0x00` and `0x80` forbidden, `0xFF` is the
  special slot for reconstruction level 1024, all others linear
  `code * 8`.
* §5.4.2 — Transform Coefficient (TCOEF), variable length; full
  Table 16 (102 regular VLC code-points with trailing sign + the
  `0000 011` ESCAPE prefix followed by a fixed-length 1 + 6 + 8 =
  15-bit event with two forbidden LEVEL codes in baseline).
  Coefficients are accumulated into a 64-entry array in **zigzag
  scan position order**; the §6.2.3 / Figure 14 zigzag → 8×8
  block-position permutation is exposed as the
  `ZIGZAG_TO_BLOCK_POS` constant.
* §6.1 / §6.2.1 — Inverse quantisation of AC coefficients with the
  H.261-style modulo-2-oddifier rule: `|REC| = QUANT · (2 · |LEVEL|
  + 1)` for odd QUANT, minus 1 for even QUANT; INTRA's DC slot
  bypasses the formula (the Table 15 reconstruction level lands
  there at parse time).
* §6.2.2 — AC reconstruction-level clip to `[-2048, +2047]`.
* §6.2.3 — Zigzag → 8×8 scatter (Figure 14).
* §6.2.4 — Inverse DCT computed in `f64` against a 64-entry
  `cos(π·(2n+1)·k/16)` table, rounded to nearest integer and
  clipped to `[-256, +255]`. The spec's "arithmetic procedures …
  are not defined, but should meet the error tolerance specified
  in Annex A" — the `f64` kernel matches the Annex A.7 "at least
  64-bit floating point" reference exactly, so the accuracy
  budget is satisfied by construction.
* §6.3.2 — Intra-block sample clip to `[0, 255]`. End-to-end
  composer `reconstruct_intra_block(block, quant)` takes a parsed
  `H263Block` and produces an 8×8 `u8` sample block ready for the
  picture buffer.
* §6.1.1 — Differential motion-vector reconstruction. Each Table 14
  MVD code carries a *pair* of difference values; only one yields a
  component in the permitted range `[-16, 15.5]` (= `[-32, 31]`
  half-pel, a 64-wide window). `reconstruct_mv_component` forms
  `predictor + difference` and wraps it into the window;
  `reconstruct_mv` applies it to an `Mvd` per component. The
  predictor is the per-component median of the three Figure-12
  candidates (`predict_mv_median` / `median3`). Table 18 derives the
  chrominance vector (`chroma_mv` / `chroma_mv_component`): luma
  component halved, quarter-pel fraction snapped to the nearest half.
* §6.1.2 — Half-pixel bilinear interpolation (Figure 13) with
  `RCONTROL` (implied `0` in baseline): `a = A`,
  `b = (A+B+1−RCONTROL)/2`, `c = (A+C+1−RCONTROL)/2`,
  `d = (A+B+C+D+2−RCONTROL)/4`, truncating division. Reference-plane
  access (`RefPlane`) uses §D.1 edge replication. `motion_compensate_block`
  fetches an 8×8 motion-compensated prediction at a given block
  origin + motion vector.
* §6.3.1 / §6.3.2 — INTER-block reconstruction. `reconstruct_inter_block`
  sums the motion-compensated prediction with the IDCT residual and
  clips to `[0, 255]`. End-to-end composer
  `reconstruct_inter_block_with_prediction(block, quant, prediction)`
  runs dequant (no INTRA DC bypass) → §6.2.2 clip → zigzag scatter →
  IDCT → §6.3.1 summation → §6.3.2 clip.
* Annex J §J.3 — in-loop deblocking edge filter. Four-tap formula
  on `(A, B, C, D)` straddling each 8×8 block edge:
  `d = (A − 4B + 4C − D) / 8`,
  `d1 = UpDownRamp(d, STRENGTH)`,
  `d2 = clipd1((A − D) / 4, d1 / 2)`,
  `B1 = clip(B + d1)`, `C1 = clip(C − d1)`,
  `A1 = A − d2`, `D1 = D + d2`, with `UpDownRamp` per Figure J.2.
  Full Table J.2 (QUANT → STRENGTH) transcribed for QUANT `1..=31`.
  `deblock_plane` driver runs all horizontal edges before all
  vertical edges per the §J.3 ordering rule, skips picture-edge
  boundaries per the §J.3 picture-edge rule, and exposes a per-edge
  `EdgeCondition` callback so the macroblock-loop driver can express
  the §J.3 "block1 coded OR block2 coded" application condition and
  the §K/§R slice-boundary skip rules.
* Annex I §I.2 / §I.3 — Advanced INTRA Coding scan-and-mode layer
  (the `aic` module). The §I.2 INTRA_MODE field VLC (Table I.1):
  `0` → DC-Only, `10` → Vertical DC&AC, `11` → Horizontal DC&AC,
  decoded into `IntraMode` by `decode_intra_mode`. The two §I.3
  alternate DCT scans (Figure I.2) as scan-position → block-position
  permutations in the Figure-14 convention:
  `ALT_HORIZONTAL_TO_BLOCK_POS` (Figure I.2-a, horizontal
  frequencies first) and `ALT_VERTICAL_TO_BLOCK_POS` (Figure I.2-b,
  the ITU-T H.262 alternate scan). The §I.3 scan-selection rule
  `scan_for_intra_mode`: mode 0 keeps the Figure-14 zigzag, mode 1
  selects the alternate-horizontal scan, mode 2 the
  alternate-vertical scan. The Table I.2 separate INTRA-coefficient
  VLC, the modified inverse quantization, and the DC/AC prediction
  reconstruction (which need the neighbour blocks the macroblock-grid
  driver supplies) are deferred.
* §4.2.1 / §5 / §6 — full-picture decode driver (`picture` module).
  `decode_picture` walks all GOBs of a picture top-to-bottom (using the
  per-format GOB count and macroblock-rows-per-GOB from the source
  format) and all macroblocks of each GOB left-to-right, deriving each
  of the six blocks' `BlockContext` from the MB type + CBPY (luma) /
  CBPC (chroma) bits and dispatching `reconstruct_intra_block` /
  `reconstruct_inter_block_with_prediction`. For INTER macroblocks it
  derives the §6.1.1 / Figure-12 median predictor — implementing the
  candidate border-decision rules (INTRA / not-coded → zero, left/top/
  GOB-top/right borders) against a live macroblock grid — reconstructs
  the luma MV with the Table-14 MVD, motion-compensates the luma blocks
  and the Table-18 chroma blocks, and sums residuals. Skipped
  macroblocks (COD = 1) copy the reference with a zero MV. An optional
  Annex J §J.3 deblocking pass (via `DecodeOptions::deblock`) runs
  `deblock_plane` over all three planes with a per-edge `EdgeCondition`
  derived from the grid's coded/not-coded state and each macroblock's
  QUANT. The result is a planar 4:2:0 `YuvFrame`. The baseline subset
  covers INTRA / INTRA+Q / INTER / INTER+Q / skipped macroblocks for
  the standardized source formats; INTER4V (four MVs, Annex F),
  PB-frames, extended PTYPE, Annex T DQUANT, CPM = 1, slice mode and
  custom formats return `Error::NotImplemented`.
* Annex D §D.2 — Unrestricted Motion Vector mode (PLUSPTYPE absent).
  `reconstruct_mv_component_umv` / `reconstruct_mv_umv` extend the
  per-component MV range from the default `[-32, 31]` to `[-63, 63]`
  half-pel (spec `[-31.5, 31.5]`), applying the §D.2
  predictor-dependent difference-pair selection: a predictor in
  `[-31, 32]` half-pel uses the first Table-14 column directly with no
  wrap, while a predictor outside that range picks the pair member
  giving a component in `[-63, 63]` with the predictor's sign (zero
  allowed either way). The decode driver switches to this path when the
  PTYPE bit-10 UMV flag is set; the always-on §D.1 edge replication
  supplies the out-of-picture samples. The PLUSPTYPE / UUI ranges of
  Tables D.1 / D.2 and the Table-D.3 reversible VLC stay gated on the
  not-yet-decoded extended-PTYPE header.
* Annex F §F.2 — Advanced Prediction mode four-motion-vector
  candidate-predictor redefinition (Figure F.1) and Table F.1
  sixteenth-pixel chrominance vector derivation, as pure transformations
  in the `motion` module. `LumaBlockIndex` (B1 / B2 / B3 / B4) names
  the four 8×8 luminance blocks of a macroblock in Figure 5 order;
  `Mb4Mv` is the per-MB MV array; `Mb4MvNeighbourhood { current, left,
  above, above_right, right }` holds the §F.2-relevant neighbours with
  `Option` wrappers so the caller can encode the §6.1.1 default-to-zero
  decisions for INTRA / not-coded / border macroblocks.
  `select_4mv_candidates(block, &neighbourhood)` returns `(MV1, MV2,
  MV3)` per Figure F.1's "8×8 block at the physically same relative
  position around MV" rule: B1 → (B2 of MB-left, B3 of MB-above, B4 of
  MB-above); B2 → (B1 of current, B4 of MB-above, B3 of
  MB-above-right); B3 → (B4 of MB-left, B1 of current, B2 of current);
  B4 → (B3 of current, B2 of current, B1 of MB-right). The output
  feeds directly into `predict_mv_median` for the §6.1.1 per-component
  median. `chroma_mv_4mv(luma)` / `chroma_mv_component_4mv(sum)`
  perform §F.2's "sum of the four luminance vectors divided by 8"
  chroma derivation with the Table F.1 sixteenth → half-pixel snap
  (`{0,1,2}→0`, `{3..=13}→1`, `{14,15}→2`). The §F.3 overlapped block
  motion compensation (the weighted three-prediction H0/H1/H2 average)
  and the driver wiring that walks the live neighbour grid are out of
  scope for this round; the decode driver still returns
  `Error::NotImplemented` for INTER4V macroblocks.

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
    DecodeOptions { deblock: true },
)?;
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
assert_eq!(gob.header_bits, 29);                       // 17 + 5 + 2 + 5

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

// One block of the macroblock per §5.4, with the caller deriving
// the INTRADC / coefficient presence from the MB type + CBP bits.
let block = parse_block(
    &mut r,
    BlockContext {
        has_intradc: mb.mb_type.unwrap().is_intra(),
        has_coefficients: false, // for this luma block's CBPY bit
    },
)?;

// §6.1 / §6.2 / §6.3.2 intra-block reconstruction: dequantise,
// scatter zigzag → 8×8, inverse DCT, clip to [0, 255].
let samples_8x8 = reconstruct_intra_block(&block, gob.quantiser);
```

### What is NOT yet implemented

* INTER4V / INTER4V+Q macroblocks (MB types 2 / 5) — driver wiring
  for the four-vector path is still pending. Round 11 landed the
  §F.2 candidate-predictor redefinition (Figure F.1) and the Table
  F.1 sixteenth-pixel chroma derivation as pure primitives
  (`select_4mv_candidates`, `chroma_mv_4mv`), but the macroblock-loop
  driver does not yet walk the neighbour grid to feed them; the
  driver returns `Error::NotImplemented` when it meets an INTER4V
  macroblock. The §F.3 overlapped block motion compensation (the
  weighted three-prediction H0/H1/H2 average) is also out of scope.
* GOB-0-header-elision: the driver requires every GOB (including the
  topmost) to carry a GBSC/GN/GFID/GQUANT header on the wire, because
  the picture-layer PQUANT (the QUANT GOB 0 would inherit when its
  header is omitted) lives in the not-yet-decoded optional-field
  block. A bitstream that omits the GOB-0 header would mis-frame.
* Multi-picture sequence demuxing: `decode_picture` decodes one
  picture given an explicit reference frame; chaining pictures (PSC
  scanning, reference-frame management across a stream) is the
  caller's responsibility.
* Annex N (Reference Picture Selection mode) and slice-boundary /
  Independent-Segment-Decoding skip rules for the deblocking
  filter (the filter primitive itself is in `deblock`; the rules
  that tell it which edges to skip live in the macroblock driver).
* PB-frame MODB / CBPB / MVDB (§5.3.3 / §5.3.4 / §5.3.9, Annex G);
  the parser refuses no fields directly but the caller's picture
  context must keep `pb_frames = false`.
* Annex T variable-length DQUANT (Modified Quantization mode);
  the baseline 2-bit form is the only one decoded, and the
  Annex-T EXTENDED-ESCAPE LEVEL prefix (`1000 0000`) is not
  accepted in TCOEF.
* Annex I (Advanced INTRA Coding) — the remaining parts beyond the
  round-8 scan-and-mode layer: the Table I.2 separate
  INTRA-coefficient VLC (the dedicated |LEVEL|/RUN interpretation
  that replaces Table 16 for INTRA), the INTRADC-as-AC-coded-value
  path, the §I.3 modified inverse quantization
  (`RecC = 2·QUANT·LEVEL`, no dead-zone) with variable-step INTRADC,
  and the DC/AC prediction reconstruction + `oddifyclipDC` /
  `clipAC` clipping (which need the macroblock-grid driver's
  neighbour blocks). The §I.2 INTRA_MODE VLC, the two §I.3 alternate
  scans, and the §I.3 scan-selection rule landed in round 8.
  Round-4 §5.4.1 is still the baseline 8-bit FLC INTRADC form.
* Annex D — only the §D.2 PLUSPTYPE-absent extended range landed
  (round 10). The PLUSPTYPE / UUI-dependent ranges of Tables D.1 / D.2
  and the Table-D.3 reversible-VLC encoding of the difference (used
  whenever PLUSPTYPE is present) remain gated on the not-yet-decoded
  extended-PTYPE header.
* Annex O B/EI/EP picture macroblocks.
* GSTUF stuffing (§5.2.1) — the caller skips it before invoking the
  GOB parser; the parser does not auto-detect leading zeros.
* GSBI (§5.2.4, CPM = "1" case) — picture-layer CPM is not yet
  exposed, so the GOB parser only handles the CPM = "0" branch.
* Slice-structured mode (Annex K), end-of-sequence markers
  (§5.1.27, EOS/EOSBS as PSC-prefixed codes).
* The Annex-O optional fields after PTYPE: PQUANT, CPM/PSBI, TRB,
  DBQUANT, PEI/PSUPP.
* Extended PTYPE / PLUSPTYPE (§5.1.4) and every annex it gates
  (Annexes I, J, K, M, N, O, P, Q, R, S, T) — the parser surfaces a
  dedicated `ExtendedPtypeNotSupported` error rather than guessing.
* Encoder. Round 3 is decode-only.
* `oxideav_core::Decoder` registration; the `register()` function is
  still a no-op pending a frame-yielding decoder.

### Round 11 coverage estimate

* H.263 spec text covered: §4.2.1 (GOB / MB scan layout, per-format
  GOB & MB-row counts) + §5.1.1–§5.1.3 + §5.2.2 + §5.2.3 +
  §5.2.5 + §5.2.6 + §5.3.1 + §5.3.2 + §5.3.5 + §5.3.6 + §5.3.7 +
  §5.3.8 + §5.4.1 + §5.4.2 + §6.1.1 (MV reconstruct + median
  predictor + Figure-12 candidate border-decision rules + Table 18
  chroma) + §6.1.2 (half-pel interpolation, Figure 13) + §6.2.1 +
  §6.2.2 + §6.2.3 + §6.2.4 + §6.3.1 (INTER summation) + §6.3.2
  (sample clip) + §D.1 edge replication + Figure 14 zigzag table +
  Annex J §J.3 (four-tap edge filter + Table J.2 STRENGTH lookup +
  horizontal-before-vertical ordering + picture-edge skip + driver
  edge-condition wiring) + Annex I §I.2 INTRA_MODE VLC (Table I.1) +
  §I.3 alternate DCT scans (Figure I.2-a / I.2-b) + §I.3
  scan-selection rule + Annex D §D.2 (PLUSPTYPE-absent extended
  `[-63, 63]` half-pel range + predictor-dependent difference-pair
  selection) + Annex F §F.2 (four-vector candidate-predictor
  redefinition per Figure F.1 + Table F.1 sixteenth-pixel chroma
  derivation), now composed into a full-picture decode driver
  (`decode_picture` → `YuvFrame`) for the single-MV path and exposed
  as pure primitives for the §F.2 four-MV path. Roughly 20 pages of
  the ~144-page recommendation.
* Tests: 196 unit tests on synthetic buffers built with the spec's
  bit layout (round-trip via `oxideav_core::bits::BitWriter`),
  including full-table round-trips for Tables 7 (9 codes), 8
  (21 + 4 codes), 12 (16 codes), 14 (64 codes), 15 spot-check,
  and 16 (102 regular code-points across both sign polarities,
  plus the ESCAPE event with both signs and both forbidden LEVEL
  codes); 12 dequant tests including the §6.2.1 "REC is always
  odd" invariant across 31 QUANT × 20 LEVEL combinations and the
  §6.2.2 clip at both extremes; 8 IDCT tests including the §A.8
  zero-in/zero-out invariant, the single-AC-coefficient basis-
  pattern ±1 error budget, and IDCT diagonal symmetry; 6 end-to-end
  intra-block reconstruction tests; 30 motion tests covering MV
  reconstruction (in-range / both-side wrap / exhaustive
  in-range sweep), median predictor, Table 18 chroma derivation,
  §6.1.2 half-pel interpolation (integer / horizontal with RCONTROL
  0 and 1 / vertical / diagonal / edge replication), block-level
  motion compensation (zero / integer / half-pel shift),
  §6.3.1 + §6.3.2 INTER summation with clip, and the Annex D §D.2 UMV
  reconstruction (first-column no-wrap / below- and above-range
  sign-and-bound selection / extended-range invariant across the whole
  UMV space / full-vector application / agreement with the default
  rule where the default sum does not wrap); plus 21 deblock tests
  covering the full Table J.2 STRENGTH lookup, `UpDownRamp` shape
  (zero-input / identity-inside-window / descending-segment /
  above-2S-zero / RRU-infinite identity), `clipd1` symmetry, the
  four-tap filter (flat-input identity / in-window attenuation
  hand-derived against the spec / strong-edge preservation /
  clip-overflow on B1 and C1 / 1296-input never-panic sweep),
  and the `deblock_plane` driver (flat no-op / all-skip no-op /
  near-edge-only modification / horizontal-stripes-only-horizontal-
  pass / orientation symmetry / bad-dimension panics); plus 15 Annex I
  `aic` tests covering the INTRA_MODE VLC (each of the three Table I.1
  codes / exact-bit-consumption for the 1-bit and 2-bit forms / EOF
  mid-field / EOF on empty buffer / index round-trip), the two
  alternate scans (both are permutations of 0..=63 / DC-first in every
  scan / the alternate-vertical scan is the transpose of the
  alternate-horizontal scan / the scans differ off-DC / Figure-I.2
  spot-checks for both grids), and the §I.3 scan-selection rule; plus
  a composition test that chains four parsers (picture → GOB → MB →
  block) from a single `BitReader`; plus 20 `picture`-driver tests
  covering the per-format GOB / MB layout constants (QCIF / CIF / 4CIF),
  `YuvFrame` construction, Figure-5 luma-block origins, 8×8 blitting,
  the §6.1.1 / Figure-12 candidate-predictor selection (top-left
  all-border zero / left-neighbour at top row / INTRA-neighbour zero
  candidate / interior median / right-edge MV3-zero), and end-to-end
  full-picture decodes (QCIF INTRA DC-only uniform field at two DC
  levels / INTRA+deblock no-op on a flat field / CBPY-driven per-block
  AC presence / INTER all-skipped exact reference copy / INTER
  horizontal +1-pixel MV shift with §D.1 edge replication / Annex D
  §D.2 UMV vector kept in the extended range past the default wrap /
  missing reference + extended-PTYPE refusals); plus 17 Annex F §F.2
  tests covering `LumaBlockIndex` round-trip, the Figure F.1
  candidate-predictor selection per block (B1 isolated all-zero / B1
  left-only and above-only partial neighbourhoods / B2 / B3
  full-neighbourhood with distinctive vectors / B4 right-edge MV3-zero
  and B4 with MB-right present), the one-vector-per-MB equivalence (a
  uniform 4-MV array reduces to the Figure-12 single-MV candidates),
  the end-to-end median predictor on a uniform field, the Table F.1
  16-entry sixteenth → half-pixel transcription, the all-zero chroma
  MV, the four-uniform-luma equivalence with the §6.1.1 single-MV
  chroma rule across nine integer-pixel offsets, the
  positive/negative sixteenth-snap, the full-pixel integer chroma
  result, the Table F.1 asymmetry round-trip at the low (2/3) and
  high (13/14) boundaries with negative mirror, and the bounded
  chroma magnitude sweep across `[-200, +200]` sums.

## License

MIT — see [LICENSE](./LICENSE).
