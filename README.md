# oxideav-h263

A pure-Rust ITU-T H.263 baseline video decoder for the
[oxideav](https://github.com/OxideAV/oxideav) framework, built
clean-room against [ITU-T Recommendation H.263 (01/2005)][spec].

## Status

Decode-only. The crate implements the H.263 baseline picture / GOB /
macroblock / block layers and reconstructs INTRA and INTER pictures
end-to-end, plus a growing set of optional Annexes (D, F, I, K, T).
There is no encoder, and `register()` is currently a no-op pending a
frame-yielding `oxideav_core::Decoder` adapter — callers drive the
decoder through the free `decode_picture` entry point.

Any decode path that is not yet wired returns `Error::NotImplemented`
rather than silently guessing.

## What works

The high-level `decode_picture` driver walks every GOB of a picture
top-to-bottom and every macroblock left-to-right, reconstructing a
planar 4:2:0 `YuvFrame`:

* **Picture layer (§5.1)** — PSC, Temporal Reference, non-extended
  PTYPE (split-screen / document-camera / freeze-release indicators,
  source format sub-QCIF..16CIF, INTRA / INTER coding type, optional
  Annex D/E/F/G mode flags), and the extended PLUSPTYPE header
  (§5.1.4 onward: UFEP / OPPTYPE / MPPTYPE + CPM / PSBI / CPFMT /
  EPAR / CPCFC / ETR / UUI / SSS).
* **GOB layer (§5.2)** — GBSC, Group Number, GOB Frame ID, GQUANT
  (CPM = "0" branch).
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
  candidate-predictor redefinition (Figure F.1), Table F.1
  sixteenth-pixel chroma derivation, and overlapped block motion
  compensation (OBMC) over the Figures F.2 / F.3 / F.4 weight
  matrices with the `Zero` / `Current` / `Vector` remote-MV
  substitution rules. INTER4V / INTER4V+Q macroblocks reconstruct
  end-to-end when Advanced Prediction is signalled.
* **Annex I §I.2 / §I.3** — Advanced INTRA Coding: the INTRA_MODE VLC
  (Table I.1), the two alternate DCT scans (Figure I.2) and scan
  selection, the separate INTRA-coefficient VLC (Table I.2), the
  no-dead-zone modified inverse quantisation, the `clipAC` /
  `oddifyclipDC` clips, and the DC/AC prediction reconstruction.
* **Annex J §J.3** — in-loop deblocking edge filter (four-tap formula
  + full Table J.2 STRENGTH lookup + horizontal-before-vertical
  ordering + picture-edge skip), opt-in via `DecodeOptions::deblock`.
* **Annex K §K.2** — Slice Structured mode: the slice-layer header
  parse (SSC + SEPB1/2/3 + optional SSBI + MBA + SQUANT + optional
  SWI + GFID, plus the first-slice reduced form) and the free-running
  (non-Rectangular-Slice) end-to-end decode driver.
* **Annex S §S.2 / §S.3** — Alternative INTER VLC mode: each INTER
  coefficient block is interpreted with the baseline INTER VLC (Table
  16) first and re-interpreted with the Annex I INTRA VLC (Table I.2)
  only when the INTER reading would address coefficients past slot 63 of
  the block (§S.2.2 step 3, keyed on the run-overflow signal — both
  tables share one codeword inventory so the re-decode consumes the same
  bits); and, when both chrominance blocks of an INTER macroblock carry
  coefficients (`CBPC5 = CBPC6 = 1`), the CBPY codeword is the Table 12
  INTRA pattern (no INTER complement, §S.3). Wired into the baseline
  single-MV INTER path and auto-activated from the PLUSPTYPE OPPTYPE
  bit 13; refused when combined with Advanced Prediction / INTER4V,
  PB-frames, Slice-Structured or Modified Quantization.
* **Annex T** — Modified Quantization mode: the §T.2 variable-length
  DQUANT parser, the §T.3 chrominance `QUANT_C` step, and the §T.4
  EXTENDED-ESCAPE / EXTENDED-LEVEL extended coefficient range, driving
  an MQ-active picture reconstruction end-to-end for the baseline INTRA
  / INTER path **and** the Annex I Advanced INTRA Coding path (the
  §T.3 `QUANT_C` chroma dequant and the §T.5-rule-2 EXTENDED-ESCAPE
  extension to the Table I.2 VLC both thread through the AIC INTRA
  reconstruction).
* **Annex Q §Q.6** — Reduced-Resolution Update mode prediction-error
  up-sampling: the 8×8 reduced-resolution reconstructed prediction-error
  block is up-sampled to a 16×16 block with the block-closed §Q.6.1
  interior filter (Figure Q.8 9/3/3/1 bilinear weights) and the §Q.6.2
  boundary filter (Figure Q.9 corner copy + 3:1 edge interpolation), all
  with §Q.6 division-by-truncation semantics. Exposed as the pure
  `upsample_prediction_error` primitive; the surrounding 32×32-macroblock
  RRU decode pipeline (pseudo-MV §Q.4, enlarged OBMC §Q.5, reference
  extension §Q.3, block boundary filter §Q.7) is not yet wired.

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

* INTER4V macroblocks outside Advanced Prediction mode (the PLUSPTYPE
  Deblocking-Filter mode INTER4V+Q row).
* GOB-0-header elision (the driver requires every GOB to carry a header
  on the wire).
* Multi-picture sequence demuxing (PSC scanning / reference management
  across a stream stays caller-side).
* Annex N (Reference Picture Selection) and slice-boundary /
  Independent-Segment-Decoding deblock skip rules.
* PB-frames and Improved PB-frames (Annex G / M) end-to-end
  reconstruction.
* Annex K Rectangular Slice submode, Annex K with Advanced Prediction /
  CPM, and Arbitrary Slice Ordering.
* Annex O B/EI/EP scalability picture macroblocks; Annexes N / O / P
  PLUSPTYPE sub-bitstreams.
* Annex Q Reduced-Resolution Update mode end-to-end (only the §Q.6
  prediction-error up-sampling primitive is implemented; the 32×32
  macroblock layer, §Q.4 pseudo-MV reconstruction, §Q.5 enlarged OBMC,
  §Q.3 reference extension and §Q.7 block boundary filter are not yet
  wired).
* GSTUF stuffing auto-detection and GSBI (CPM = "1").
* End-of-sequence markers (EOS / EOSBS).
* Encoder. The crate is decode-only.
* `oxideav_core::Decoder` registration; `register()` is a no-op
  pending a frame-yielding decoder adapter.

## Testing

The crate carries an extensive unit-test suite over synthetic buffers
built with the spec's bit layout (round-tripped via
`oxideav_core::bits::BitWriter`), including full-table round-trips for
Tables 7 / 8 / 12 / 14 / 16, the inverse-quantisation invariants, IDCT
accuracy against the Annex A error budget, motion / OBMC / deblock /
AIC / PLUSPTYPE / slice-header coverage, and end-to-end picture-decode
tests. Run with `cargo test -p oxideav-h263`.

## License

MIT — see [LICENSE](./LICENSE).

[spec]: https://www.itu.int/rec/T-REC-H.263
