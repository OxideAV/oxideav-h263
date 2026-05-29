# oxideav-h263

A pure-Rust ITU-T H.263 baseline video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild round 19 — Annex I §I.3 INTRA DC/AC prediction
reconstruction (`aic_predict` module). One pure function lands the
§I.3 page-79 three-mode reconstruction step: given a current INTRA
block's dequantized residual array `RecC(u,v)`, the `INTRA_MODE`
decoded from §I.2 (Table I.1), and an optional pair of
already-reconstructed neighbour blocks (`RecA'` immediately above,
`RecB'` immediately to the left), returns the final `RecC'(u,v)`
array post-`clipAC` for AC slots and post-`oddifyclipDC` for the DC
slot. The driver supplies neighbour availability per the §I.3 page-78
"same video picture segment" rule via a `Neighbour::None` /
`Neighbour::Available` tag; the predictor primitive itself does not
encode that test.

* `reconstruct_intra_block_aic(rec_c_residual, mode, block_a, block_b)
  -> [i32; 64]` — applies the per-mode rule from §I.3 page 79:
  * **Mode 0** ([`IntraMode::DcOnly`]): AC slots = `clipAC(RecC(u,v))`,
    DC = `oddifyclipDC(RecC(0,0) + predictor)` with predictor =
    `(RecA'(0,0) + RecB'(0,0)) / 2` (truncation toward zero) when both
    A and B are available, single neighbour's DC if only one is, and
    `1024` if neither is.
  * **Mode 1** ([`IntraMode::VerticalDcAc`]): when A is available,
    DC + first-row AC slots `(u, 0)` for `u = 1..=7` are predicted
    from `RecA'(u, 0)`; rows `v = 1..=7` pass through as
    `clipAC(RecC(u, v))`. When A is unavailable, DC falls back to
    `+1024` and no AC slot is predicted.
  * **Mode 2** ([`IntraMode::HorizontalDcAc`]): symmetric to Mode 1,
    using block B and the first column `(0, v)` for `v = 1..=7`.
* `Neighbour<'a>` enum + `AIC_FALLBACK_DC_PREDICTOR = 1024` constant
  expose the §I.3 availability + fallback-predictor knobs to callers.
* All coefficient arrays here are in **block-position** layout
  (`index = v * 8 + u`) — the caller is expected to have already
  scattered the zigzag-scan-order output of
  [`block_aic::parse_intra_block_aic`] through the AIC-selected scan
  ([`aic::scan_for_intra_mode`]) and dequantized via
  [`aic_dequant::aic_dequant_coefficient`] before invoking this.

This closes the round-17 / round-18 "DC/AC prediction reconstruction
deferred to the macroblock-grid driver" gap as far as a pure-function
primitive is concerned: every §I.3 page-79 branch is now expressible
as a single call. The driver-side work that remains is the picture-
walking pass that computes per-block availability bits, accumulates
`RecA'` / `RecB'` arrays from prior reconstructions, and dispatches
this primitive plus the inverse DCT — that driver is the next round.

23 new unit tests cover: Mode 0 with no neighbours / only A / only B
/ both (averaging with truncation toward zero, including the
negative-sum truncation case); Mode 0 AC slots passing through
`clipAC` of the bare residual with neighbour AC values ignored; Mode 1
with A available (DC + first-row prediction wired, rows `v >= 1`
left as bare residuals) and with A unavailable (DC falls back to
`+1024`); Mode 2 symmetric for the first column; AC upper / lower
`clipAC` saturation; DC `oddifyclipDC` parity bump and clip-to-
`[0, 2047]` range (including a negative-sum case that clips to 0);
all-zero-residual / no-neighbour invariant across all three modes
(DC = 1025, AC = 0); observational identity of `Neighbour::None`
regardless of reason; `is_available` accessor; Mode 1 / Mode 2 zero-
residual predictor-passthrough; cross-mode invariant that every AC
output respects `[AIC_AC_REC_MIN, AIC_AC_REC_MAX]` and every DC
output respects `[AIC_DC_REC_MIN, AIC_DC_REC_MAX]`; fallback-DC
predictor consistency across modes; and `AIC_FALLBACK_DC_PREDICTOR
== 1024` constant guard.

On top of round 18's Annex I §I.3 absorbed-INTRADC INTRA-block
parser (`block_aic` module). One pure function lands the §I.3 (lines
4213-4217) bitstream-layout change for INTRA blocks when Advanced
INTRA Coding is in use: the §5.4.1 Table-15 8-bit FLC INTRADC prefix
is gone, and the per-block decode is purely a sequence of Table I.2
`(LAST, RUN, LEVEL)` events ([`intra_tcoef::decode_intra_tcoef_event`])
starting at scan position 0 — the DC slot is just slot 0 of the
coefficient buffer and is filled by whichever event's cumulative-RUN
lands on it (or stays zero when no event does — the §I.3 "zero
INTRADC will not be coded as a LEVEL, but will simply increase the
run for the following AC coefficients" semantics).

* `parse_intra_block_aic(reader, has_coefficients) -> Result<H263Block>`
  — single-block AIC INTRA parser. `has_coefficients` is the relevant
  CBP bit (CBPY for luma 0..=3, CBPC for chroma 4 / 5) per the §I.3
  redefinition: in AIC mode the CBP bit being 0 is the sole signal
  that the DC is also zero, since INTRADC is no longer special-cased.
  Returns an [`H263Block`] whose `coefficients[..]` holds the parsed
  `LEVEL` integers in zigzag-scan-position order and whose
  `had_intradc` is always `false` (no FLC was consumed regardless of
  whether slot 0 carries a non-zero LEVEL). The same scan-overflow,
  truncated-input, and forbidden-ESCAPE-LEVEL guards as
  `block::parse_block` apply via the underlying event decoder.

This is the §I.3 line 4214 "next round" promised by round 14
(`intra_tcoef`) — wiring the Table I.2 VLC into a full INTRA-block
decoder. Composing it with the §I.3 modified inverse-quantization
(round 17 `aic_dequant_coefficient`), the prediction reconstruction
(deferred, needs the macroblock-grid driver's neighbour blocks), the
§I.3 `clipAC` / `oddifyclipDC` post-prediction step (round 17), and
the §I.3 scan selected by [`aic::scan_for_intra_mode`] gives the full
§I.3 INTRA-block decode pipeline; the only remaining gap is the
macroblock-grid driver that supplies the live neighbour blocks for
the DC/AC prediction.

15 new unit tests cover: the no-coefficients path returns an empty
block without consuming bits; a single LAST=1 RUN=0 event places its
LEVEL at the DC slot (the §I.3 absorbed-INTRADC); the §I.3
zero-DC-via-RUN invariant for RUN ∈ {1, 3, 7}; a DC-bearing event
followed by an AC event lands LEVELs at slots 0 and 3; events at
boundary slot 63 (terminating well-formed, non-terminating overflow);
cumulative scan-position overflow when two events sum past slot 63;
truncated-input → UnexpectedEof; forbidden ESCAPE LEVEL `0x00` and
`0x80` reject with BadTcoefEscapeLevel while `0x81` (-127) and `0x7F`
(+127) decode correctly; `had_intradc` stays `false` even with a
non-zero DC; and a 8-event distribution-integration test placing
LEVELs at slots 0/2/7/18/19/40/46/63 simultaneously.

On top of round 17's Annex I §I.3 modified inverse-quantization
primitives (`aic_dequant` module): the dead-zone-free reconstruction
residual formula and the §I.3 DC / AC clip helpers that the AIC
coefficient pipeline composes after the round-14 Table I.2 VLC
event-decode:

* `aic_dequant_coefficient(level, quant) -> i32` — §I.3 formula
  `RecC(u,v) = 2 · QUANT · LEVEL(u,v)` applied to a single coefficient
  slot (DC or AC alike), strictly linear in both inputs and strictly
  even-valued, contrasting with the §6.2.1 H.261-style odd-fier
  baseline.
* `clip_ac(x)` — §I.3 `clipAC` range pin to `[-2048, +2047]` applied
  to every AC slot post-prediction-sum.
* `oddify_clip_dc(x)` — §I.3 `oddifyclipDC` combined parity-bump +
  `clipDC` range pin to `[0, +2047]` applied to the DC slot
  post-prediction-sum, protecting against the IDCT-mismatch resonance
  the spec calls out at the (0,0) / (0,4) / (4,0) / (4,4) basis-pattern
  cross-points.

The companion §I.3 INTRA prediction reconstruction itself (the three
INTRA_MODE-dependent rules that add `RecA'(u,v)` / `RecB'(u,v)` contributions
before the final clip) is still deferred to the macroblock-grid driver
that supplies the live neighbour blocks; round 17's primitives are the
building blocks the driver will compose.

On top of round 16's Annex F §F.2 / §F.3 INTER4V four-motion-
vector + Overlapped Block Motion Compensation driver wiring. The
`decode_picture` driver reconstructs INTER4V / INTER4V+Q
macroblocks end-to-end whenever the picture header's Advanced
Prediction flag is set: the per-macroblock grid carries a full
`[MotionVector; 4]` per MB (`LumaBlockIndex` / Figure-5 order);
each of the four luma MVs is reconstructed from
`select_4mv_candidates` + `predict_mv_median` with the §6.1.1
rule-3 / rule-4 border rewrites + Annex D §D.2 UMV extension
applied per block; Annex F §F.3 OBMC is dispatched per luma block
via `obmc_predict_block` with the four remote MVs classified into
`RemoteMv` tags per the §F.3 substitution rules (not-coded → zero,
INTRA / off-picture → current, baseline → coded vector; B3 / B4's
bottom remote unconditionally `Current` per §F.3 last sentence);
the chroma vector comes from `chroma_mv_4mv` (sum of four luma
vectors / 8 with the Table F.1 sixteenth → half snap), and both
chroma blocks use standard half-pel MC (no chroma OBMC per §F.2).
On top of round 15's Annex K Slice Structured mode slice-layer
header (`slice_header` module): `parse_slice_layer` and
`parse_first_slice_header` decode the §K.2 / Figure K.1 syntax
(SSC + SEPB1 + optional SSBI + MBA + optional SEPB2 + SQUANT +
optional SWI + SEPB3 + GFID) for general slices and the §K.2
reduced form for the first slice after the picture start code,
with Tables K.1 / K.2 / K.3 driving SSBI legality and the
MBA / SWI field widths per the `SliceHeaderContext`
picture-geometry / CPM / RS-submode / RRU inputs. On top of round
14's Annex I §I.3 / Table I.2 separate INTRA-coefficient VLC
(`decode_intra_tcoef_event` — 102 regular codewords reusing
Table 16 bit patterns under reassigned `(RUN, |LEVEL|)` columns,
plus the §5.4.2 ESCAPE event), round 13's extended-PTYPE (PLUSPTYPE)
picture-header parse (UFEP / OPPTYPE / MPPTYPE + CPM / PSBI /
CPFMT / EPAR / CPCFC / ETR / UUI / SSS), round 12's §F.3 OBMC
weighted three-prediction average (`obmc_predict_block` over
`H0` / `H1` / `H2` with the `RemoteMv` substitution rules), round
11's §F.2 four-motion-vector candidate-predictor redefinition +
Table F.1 sixteenth-pixel chroma derivation, round 10's Annex D
§D.2 Unrestricted Motion Vector mode (extended `[-63, 63]` half-pel
per-component range with predictor-dependent difference-pair
selection, PLUSPTYPE-absent case), round 9's full-picture decode
driver (baseline single-MV path: INTRA / INTER / skipped
macroblocks, §6.1.1 Figure-12 MV prediction, optional Annex J
deblocking), and rounds 1-8's picture + GOB + macroblock headers +
block data + intra-block reconstruction + P-frame motion
compensation and INTER-block reconstruction + Annex J deblocking
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
  (`{0,1,2}→0`, `{3..=13}→1`, `{14,15}→2`).
* Annex I §I.3 — separate INTRA-coefficient VLC (Table I.2), as the
  pure primitive `decode_intra_tcoef_event` in the `intra_tcoef`
  module. Per §I.3 line 4033 the 102 regular codewords are bit-for-bit
  identical to Table 16 at every index, but `(RUN, |LEVEL|)` are
  reassigned (e.g. idx 1 = `1111s` decodes to RUN=1/|L|=1 under I.2
  vs RUN=0/|L|=2 under Table 16); `LAST` is preserved between the
  two tables so indices 0..=57 stay `LAST=0` and 58..=101 stay
  `LAST=1`. The 7-bit ESCAPE prefix `0000 011` and its
  1 + 6 + 8 = 15-bit fixed-length tail are decoded identically to
  §5.4.2, with the baseline forbidden LEVEL codes (`0x00` / `0x80`)
  rejected. The §I.3 modified inverse quantization
  (`RecC = 2 · QUANT · LEVEL`, no dead-zone), the variable-step
  INTRADC reconstruction, and the DC/AC prediction reconstruction
  with `oddifyclipDC` / `clipAC` are deferred (they need the
  macroblock-grid driver's neighbour blocks), as is the §I.3
  line-4214 "INTRADC absorbed into the coefficient stream" reframing
  of MCBPC / CBPY.
* Annex F §F.3 — overlapped block motion compensation (OBMC) for the
  8×8 luminance prediction, as the pure function `obmc_predict_block`
  over the Figures F.2 / F.3 / F.4 weight matrices `H0` / `H1` / `H2`.
  Each output pixel `(i, j)` of the 8×8 block is
  `(q · H0[j][i] + r · H1[j][i] + s · H2[j][i] + 4) / 8`, with `q`
  the §6.1.2 / Figure-13 half-pel bilinear sample for the current
  block's MV and `r` / `s` the corresponding samples for the
  per-pixel "top-or-bottom" / "left-or-right" remote vectors:
  `j < 4` picks `r_top`, `j >= 4` picks `r_bot`; `i < 4` picks
  `s_left`, `i >= 4` picks `s_right`. The per-pixel sum `H0+H1+H2`
  is exactly 8 by construction (exposed as `OBMC_WEIGHT_SUM`), so
  the `(... + 4) / 8` rounding step divides cleanly. Each remote
  vector is supplied via a `RemoteMv` enum so the caller can encode
  the §F.3 substitution rules without folding the resolved vector
  here: `RemoteMv::Zero` for the "not coded → zero" rule;
  `RemoteMv::Current` for the union of the "INTRA / outside picture
  / current block at bottom of MB → use current vector" rules;
  `RemoteMv::Vector(mv)` for the baseline coded-neighbour case. The
  reference-plane fetches use `RefPlane::at`'s always-on §D.1 edge
  replication. The macroblock-loop driver wiring that walks the
  live four-MV neighbour grid for an INTER4V macroblock and
  dispatches `obmc_predict_block` four times (once per
  `LumaBlockIndex` with the correct `RemoteMv` classification) is
  out of scope for this round; the decode driver still returns
  `Error::NotImplemented` for INTER4V macroblocks.
* Annex K §K.2 — Slice Structured mode slice-layer header parse
  (`slice_header` module). `parse_slice_layer` decodes the §K.2 /
  Figure K.1 layout for slices other than the picture's first
  (SSC + SEPB1 + optional SSBI + MBA + optional SEPB2 + SQUANT +
  optional SWI + SEPB3 + GFID); `parse_first_slice_header` decodes
  the §K.2 reduced form for the slice that immediately follows the
  picture start code (SEPB1 + MBA + optional SEPB2 + optional SWI +
  SEPB3). `SliceHeaderContext` carries the picture geometry plus the
  CPM / Rectangular-Slice / RRU flags that drive conditional-field
  presence: SSBI is on the wire iff CPM is set, SWI iff Rectangular
  Slice submode is in effect (PLUSPTYPE SSS bit 1), and SEPB2 follows
  the §K.2.6 rule (MBA-width > 11 / > 9 thresholds against CPM, or
  RS-on for the first slice). MBA and SWI field widths come from
  Tables K.2 and K.3 with both the default and the Annex Q
  Reduced-Resolution Update columns transcribed (six rows each
  covering sub-QCIF / QCIF / CIF / 4CIF / 16CIF / 2048-wide).
  SSBI rejects every 4-bit value outside the Table K.1 set
  (`1001` / `1010` / `1011` / `1101`), exposed via
  `ssbi_to_subbitstream`. The §K.2.2 SSC value `0x00001` is
  numerically identical to the §5.2.2 GBSC value; the disambiguation
  is by picture-level mode (PLUSPTYPE SS=1), not bitstream-level.
  Wiring the slice header into the `decode_picture` driver (so a
  slice-structured bitstream actually reconstructs a frame) is the
  next round's work; `decode_picture` still walks GOB headers only.

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

* INTER4V macroblocks **outside** the Advanced Prediction mode — the
  only other place INTER4V appears is the PLUSPTYPE Deblocking-Filter
  mode (§5.3.2 Table 9 row 5: INTER4V+Q in DF mode without AP). The
  macroblock parser only pulls MVD2-4 when AP is on, and the
  `decode_inter4v_macroblock` path refuses with
  `Error::NotImplemented` when AP is off. The INTER4V driver itself
  is otherwise complete: round 16 landed the per-block 4-MV
  reconstruction (Figure F.1 candidates + Table 14 MVD + Annex D §D.2
  UMV extension when enabled), the Annex F §F.3 OBMC luma prediction
  per block (`obmc_predict_block` per `LumaBlockIndex`, fed with
  per-pixel-classified `RemoteMv` tags), the §F.2 / Table F.1
  sixteenth-pixel chroma vector (`chroma_mv_4mv`), and §6.3.1
  residual summation + §6.3.2 clip composed via
  `reconstruct_inter_block_with_prediction`.
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
  round-8 scan-and-mode layer, the round-14 Table I.2 separate
  INTRA-coefficient VLC, and the round-17 §I.3 modified inverse-
  quantization primitives: the INTRADC-as-AC-coded-value path (§I.3
  line 4214: INTRADC absorbed into the per-block coefficient stream
  for MCBPC / CBPY purposes), and the DC/AC prediction reconstruction
  itself (the three INTRA_MODE-dependent rules that compose
  `aic_dequant_coefficient` + `RecA'` / `RecB'` predictor sums +
  `clip_ac` / `oddify_clip_dc` — all of which need the macroblock-grid
  driver's neighbour blocks). The §I.2 INTRA_MODE VLC, the two §I.3
  alternate scans, and the §I.3 scan-selection rule landed in round 8;
  the Table I.2 event-level VLC primitive landed in round 14; the §I.3
  no-dead-zone residual formula `RecC(u,v) = 2·QUANT·LEVEL(u,v)` plus
  the `oddifyclipDC` / `clipAC` clipping helpers landed in round 17
  (`aic_dequant` module). Round-4 §5.4.1 is still the baseline 8-bit
  FLC INTRADC form (the AIC reframing of INTRADC awaits the
  macroblock-grid driver).
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
* Slice-structured mode (Annex K) — the §K.2 slice-layer header parse
  landed in round 15 (`slice_header` module: `parse_slice_layer` /
  `parse_first_slice_header`), but the `decode_picture` driver does
  not yet dispatch on it (it still walks GOB headers only). End-of-
  sequence markers (§5.1.27, EOS/EOSBS as PSC-prefixed codes) are
  still not parsed.
* The Annex-O optional fields after PTYPE: PQUANT, CPM/PSBI, TRB,
  DBQUANT, PEI/PSUPP.
* Extended PTYPE / PLUSPTYPE — the §5.1.4 onward picture-header *parse*
  landed in round 13 (`plus_ptype` module + `parse_picture_layer`):
  UFEP / OPPTYPE / MPPTYPE plus the deterministic-width CPM, PSBI,
  CPFMT, EPAR, CPCFC, ETR, UUI, and SSS fields (§5.1.4.1–§5.1.10 /
  §5.1.20 / §5.1.21). Still NOT done: the §5.1.11–§5.1.18 scalability /
  reference-picture-selection / reference-picture-resampling
  sub-bitstreams (Annexes N, O, P) — `parse_plus_ptype` returns
  `PlusPtypeUnsupported` for them rather than mis-framing — and wiring
  `parse_picture_layer` into the `decode_picture` driver (so the
  PLUSPTYPE-gated mode flags actually drive a decode, and custom source
  formats produce a frame). The legacy baseline-only
  `parse_picture_header` still returns `ExtendedPtypeNotSupported` for
  source-format `"111"`.
* Encoder. Round 3 is decode-only.
* `oxideav_core::Decoder` registration; the `register()` function is
  still a no-op pending a frame-yielding decoder.

### Round 17 coverage estimate

* H.263 spec text covered: §4.2.1 (GOB / MB scan layout, per-format
  GOB & MB-row counts) + §5.1.1–§5.1.3 + §5.1.4.1–§5.1.10 (extended
  PTYPE: UFEP / OPPTYPE / MPPTYPE + CPM / PSBI / CPFMT / EPAR / CPCFC /
  ETR / UUI / SSS picture-header parse) + §5.2.2 + §5.2.3 +
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
  scan-selection rule + Annex I §I.3 Table I.2 separate
  INTRA-coefficient VLC (102 regular + ESCAPE entries; reused
  Table-16 bit patterns reinterpreted under the I.2 (RUN, |LEVEL|)
  reassignment) + Annex I §I.3 modified inverse-quantization residual
  `RecC(u,v) = 2 · QUANT · LEVEL(u,v)` (no dead-zone) + Annex I §I.3
  `clipAC` AC reconstruction clip `[-2048, +2047]` + Annex I §I.3
  `oddifyclipDC` DC oddification and clip `[0, +2047]`
  (`aic_dequant` module) + Annex D §D.2 (PLUSPTYPE-absent extended
  `[-63, 63]` half-pel range + predictor-dependent difference-pair
  selection) + Annex F §F.2 (four-vector candidate-predictor
  redefinition per Figure F.1 + Table F.1 sixteenth-pixel chroma
  derivation, **now wired into the `decode_picture` driver** —
  INTER4V / INTER4V+Q macroblocks reconstruct end-to-end when
  Advanced Prediction is on) + Annex F §F.3 (overlapped block motion
  compensation weighted three-prediction average with the Figures
  F.2 / F.3 / F.4 weight matrices and the `Zero` / `Current` /
  `Vector` remote-MV substitution rules, **now dispatched per luma
  block by the INTER4V driver path** with per-pixel classification
  of the four remote MVs and the §F.3 last-sentence "bottom-of-MB →
  current" rule applied to B3 / B4) + Annex K §K.2 Slice Structured
  mode slice-layer header parse (SSC + SEPB1/2/3 + optional SSBI per
  Table K.1 + MBA per Table K.2 + SQUANT + optional SWI per Table K.3
  + GFID, plus the §K.2 first-slice reduced form), now composed into
  a full-picture decode driver (`decode_picture` → `YuvFrame`) for
  the single-MV path **and the Annex F four-MV + OBMC path**, plus
  the extended-PTYPE (PLUSPTYPE) picture-header parse (`plus_ptype`
  module + `parse_picture_layer` dispatch on PTYPE source-format
  `"111"`) and the §K.2 slice-layer header still exposed as a pure
  primitive. Roughly 25 pages of the ~144-page recommendation.
* Tests: 308 unit tests on synthetic buffers built with the spec's
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
  chroma magnitude sweep across `[-200, +200]` sums; plus 14 Annex F
  §F.3 OBMC tests covering the per-pixel `H0+H1+H2 == 8` invariant
  across every position, Figure F.2 spot-checks on `H0` (four
  corners = 4 / central 4×4 = 6 / first-row non-corner = 5),
  Figure F.3 spot-checks on `H1` (rows 0/7 all 2 / rows 1/6 cols
  2..=5 = 2 with col 0/7 edges = 1 / interior rows 2..=5 all 1),
  Figure F.4 spot-checks on `H2` (top/bottom row `[2,1,1,1,1,1,1,2]`
  / interior rows `[2,2,1,1,1,1,2,2]`), the `H1` vs `H2`
  corner-shape contrast (corners both 2 but the "+1 lane" runs along
  rows for H1 and columns for H2), `obmc_predict_block` flat-
  reference identity, all-current vector collapse to a single
  `motion_compensate_block` call, zero-vector reference copy on a
  column ramp, `RemoteMv::Zero` vs `RemoteMv::Vector(default())`
  equivalence, top-vs-bottom split observable on a row-ramp
  reference with hand-derived `(j=0, i=4) = 56` and `(j=7, i=4) =
  128`, left-vs-right split observable on a column-ramp reference
  with hand-derived `(j=2, i=0) = 28` and `(j=2, i=7) = 64`,
  `RemoteMv::resolve` per-variant rule, picture-edge replication
  (flat reference with origin past the right edge keeps every
  prediction pixel flat), and an in-range non-degenerate sweep on
  a mixed reference; plus 16 extended-PTYPE tests covering the
  `UFEP = "001"` full path (QCIF P minimal / CIF I with AP/AIC/DF/AIV/MQ
  on), CPM pulling PSBI, the custom-format chain (CPFMT → extended-PAR
  EPAR with `(PWI+1)*4 = 352` / `PHI*4 = 288`), the custom-PCF chain
  (CPCFC → ETR), the UUI `"1"` / `"01"` limited / unlimited forms, the
  `UFEP = "000"` path with and without an inherited custom-PCF state
  gating ETR, the reserved-UFEP / missing-SCE-guard / reserved
  picture-type / forbidden-PAR-code rejections, the RPS / RPR /
  B-picture `PlusPtypeUnsupported` refusals, a short-buffer EOF, and the
  `parse_picture_layer` Baseline-vs-Extended dispatch (with the legacy
  `parse_picture_header` still rejecting `"111"`); plus 19 Annex I §I.3
  Table I.2 INTRA-coefficient VLC tests covering the table-shape
  invariants (102 regular + 1 ESCAPE row; all 102
  `(LAST, RUN, |LEVEL|)` triples pairwise distinct; LAST column
  matches Table 16's index-58 boundary, proving no code/bits cross-up
  with Table 16), the full 102-entry round-trip across both sign
  polarities, spec spot-checks at indices 0 / 1 / 12 / 22 / 28 / 58 /
  101 (with the idx-1 and idx-22 ones designed to catch a
  silent-aliasing back to Table 16's interpretation), the ESCAPE
  positive-LEVEL round-trip, ESCAPE negative-LEVEL via two's
  complement, both baseline-forbidden ESCAPE LEVEL codes (`0x00` /
  `0x80`), 13-bits-all-zero → `BadTcoefCode`, empty-buffer →
  `UnexpectedEof`, exact bit-consumption of the 3-bit `10s` entry,
  and exact bit-consumption of the 22-bit ESCAPE event; plus 30
  Annex K §K.2 slice-header tests covering `SliceHeaderContext`
  geometry (Table K.2 MBA field widths for sub-QCIF / QCIF / 16CIF,
  Table K.3 SWI field widths for QCIF / CIF under RS submode, the
  Annex Q RRU column for QCIF), SEPB2-presence rule across CPM and
  picture-size combinations (QCIF-no-CPM absent, 16CIF-no-CPM
  present, CIF-with-CPM-still-absent, 4CIF-with-CPM-present), the
  minimal-QCIF non-first parse, max-legal-MBA, MBA-overflow rejection,
  CPM-on parse with a Table-K.1 SSBI codeword (and the
  `ssbi_to_subbitstream` mapping for all four codewords plus every
  non-codeword 4-bit value), illegal-SSBI rejection, RS-submode parse
  with SWI, SWI-wider-than-picture rejection, 16CIF parse exercising
  the mandatory SEPB2, bad-SEPB1 / bad-SEPB3 / SQUANT=0 / bad-SSC /
  short-buffer rejections, the §K.2 first-slice reduced-form parse
  (minimal QCIF, with-SWI under RS, MBA-overflow, bad-SEPB3), reader-
  position-after-parse advance, and the SSC-equals-GBSC numerical
  identity; plus 11 Annex F §F.2 / §F.3 INTER4V driver tests covering
  end-to-end INTER4V zero-MV reproducing the reference verbatim (the
  §F.3 `q = r = s = ref(x,y)` weighted-average identity), exact byte
  equivalence between INTER4V-zero and single-MV-zero on the same
  picture (the §F.2 last-paragraph "one vector = four equal vectors"
  rule), INTER4V on a flat-grey reference reproducing flat grey,
  INTER4V refusal without Advanced Prediction, INTER4V after an INTRA
  left-neighbour macroblock (the §F.3 INTRA-substitution path), the
  §F.3 `RemoteMv` classification for B1 at the top-left picture corner
  (top/left → `Current`; bottom/right resolve inside the current MB),
  the §F.3 last-sentence "B3 / B4 bottom remote always `Current`"
  rule, the §F.3 `not-coded` neighbour → `Zero` rule, the §F.3
  `INTRA` neighbour → `Current` rule, and the
  `build_4mv_neighbourhood` collapse of an INTRA / not-coded
  neighbour to `None` versus a coded neighbour exposing its per-block
  MV array; plus 19 Annex I §I.3 `aic_dequant` tests covering the
  reconstruction formula (QUANT=LEVEL=1 spot, sign-symmetry, zero-LEVEL
  invariant across every QUANT, strict even-valued output across
  31 × 255 QUANT × LEVEL pairs contrasting with the §6.2.1 odd-valued
  baseline, linearity in LEVEL and QUANT, the QUANT=31 / LEVEL=±127
  extreme `±7874` residual, the AIC-residual-strictly-smaller-than-
  §6.2.1-baseline invariant across the same 31 × 127 grid, and QUANT
  out-of-range clamping); `clip_ac` identity / upper-saturation /
  lower-saturation; `oddify_clip_dc` parity preservation on odd inputs,
  +1 bumping of even inputs, upper-saturation (even 2048 → 2047 via
  bump then clip; odd 5001 → 2047 via direct clip), lower-saturation
  (even -2 / -1000 / odd -1 / -999 all → 0), the in-range
  oddness-or-boundary invariant across the full -100..=3000 sweep, the
  full -3000..=3000 spec-pseudocode-equivalence cross-check, and a
  `clip_dc` basic round-trip.

## License

MIT — see [LICENSE](./LICENSE).
