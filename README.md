# oxideav-h263

A pure-Rust ITU-T H.263 baseline video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild round 7 — picture + GOB + macroblock headers +
block data + intra-block reconstruction + P-frame motion compensation
and INTER-block reconstruction + Annex J deblocking filter.** The
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
prediction + residual summation), and Annex J §J.3 (in-loop block-edge
deblocking filter with the full Table J.2 STRENGTH lookup) for the
non-PB-frame baseline:

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

The function surface is intentionally minimal:

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

* The §6.1.1 / Figure-12 border decision rules that *select* the
  three MV-prediction candidates (zero-out for INTRA / not-coded /
  outside-picture neighbours). Round 6's `predict_mv_median` takes
  the three candidates as given; deriving them needs the macroblock
  grid + COD state from the (not-yet-wired) driver loop.
* The per-macroblock driver loop that walks all six blocks (4 luma
  + 2 chroma), deriving each block's `BlockContext` from the
  macroblock's MB type and CBPY / CBPC bits, allocates the picture
  planes, selects the MV-prediction candidates, dispatches
  `reconstruct_intra_block` / `reconstruct_inter_block_with_prediction`
  per block, and invokes `deblock::deblock_plane` against the
  reconstructed luma and chroma planes (with per-edge `EdgeCondition`
  derived from the macroblock grid's COD / MB-type / segment-id
  state), is not yet wired — callers compose the per-block primitives
  themselves.
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
* Annex I (Advanced INTRA Coding) — alternate scans and the
  INTRADC-as-AC-coded-value path. Round-4 §5.4.1 is the baseline
  8-bit FLC INTRADC form.
* Annex D Table D.3 alternative MVD codes — round 3 uses Table 14
  unconditionally.
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

### Round 7 coverage estimate

* H.263 spec text covered: §5.1.1–§5.1.3 + §5.2.2 + §5.2.3 +
  §5.2.5 + §5.2.6 + §5.3.1 + §5.3.2 + §5.3.5 + §5.3.6 + §5.3.7 +
  §5.3.8 + §5.4.1 + §5.4.2 + §6.1.1 (MV reconstruct + median
  predictor + Table 18 chroma) + §6.1.2 (half-pel interpolation,
  Figure 13) + §6.2.1 + §6.2.2 + §6.2.3 + §6.2.4 + §6.3.1 (INTER
  summation) + §6.3.2 (sample clip) + §D.1 edge replication +
  Figure 14 zigzag table + Annex J §J.3 (four-tap edge filter
  + Table J.2 STRENGTH lookup + horizontal-before-vertical
  ordering + picture-edge skip). Roughly 16 pages of the
  ~144-page recommendation.
* Tests: 138 unit tests on synthetic buffers built with the spec's
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
  intra-block reconstruction tests; 24 motion tests covering MV
  reconstruction (in-range / both-side wrap / exhaustive
  in-range sweep), median predictor, Table 18 chroma derivation,
  §6.1.2 half-pel interpolation (integer / horizontal with RCONTROL
  0 and 1 / vertical / diagonal / edge replication), block-level
  motion compensation (zero / integer / half-pel shift), and
  §6.3.1 + §6.3.2 INTER summation with clip; plus 21 deblock tests
  covering the full Table J.2 STRENGTH lookup, `UpDownRamp` shape
  (zero-input / identity-inside-window / descending-segment /
  above-2S-zero / RRU-infinite identity), `clipd1` symmetry, the
  four-tap filter (flat-input identity / in-window attenuation
  hand-derived against the spec / strong-edge preservation /
  clip-overflow on B1 and C1 / 1296-input never-panic sweep),
  and the `deblock_plane` driver (flat no-op / all-skip no-op /
  near-edge-only modification / horizontal-stripes-only-horizontal-
  pass / orientation symmetry / bad-dimension panics); plus a
  composition test that chains four parsers (picture → GOB → MB →
  block) from a single `BitReader`.

## License

MIT — see [LICENSE](./LICENSE).
