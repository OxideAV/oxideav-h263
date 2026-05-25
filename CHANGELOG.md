# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Extended-PTYPE (PLUSPTYPE) picture-header parsing per §5.1.4 onward
  (round 13), in the new `plus_ptype` module, dispatched from the
  picture layer when PTYPE bits 6-8 are `"111"`:
  - `parse_plus_ptype(reader, inherited)` decodes the Figure-8 "optional
    PLUSPTYPE-related fields" block in order — `UFEP` (§5.1.4.1),
    `OPPTYPE` (§5.1.4.2, present iff `UFEP = "001"`), `MPPTYPE`
    (§5.1.4.3, always present), then the deterministic-width fields
    `CPM` / `PSBI` (§5.1.20 / §5.1.21 / §5.1.4.7), `CPFMT` (§5.1.5),
    `EPAR` (§5.1.6), `CPCFC` (§5.1.7), `ETR` (§5.1.8), `UUI` (§5.1.9),
    and `SSS` (§5.1.10), each gated by its spec presence rule.
  - `Opptype` exposes the 18-bit optional part: source format (with the
    `"110"` custom code), custom-PCF flag, and the Annex
    D/E/F/I/J/K/N/R/S/T mode bits, validating the bit-15
    start-code-emulation guard and the three reserved zero bits.
  - `Mpptype` exposes the 9-bit mandatory part: picture-type code
    (I / P / Improved-PB / B / EI / EP), RPR / RRU mode flags, rounding
    type, with the bit-9 guard and reserved-bit checks.
  - `CustomPictureFormat` / `ExtendedPar` / `CustomPcf` decode the
    custom-format chain with `luma_width` = `(PWI + 1) * 4` and
    `luma_height` = `PHI * 4` (§5.1.5), the extended-PAR `"1111"`
    follow-on (§5.1.6), and the custom-PCF divisor / conversion code
    (§5.1.7), rejecting the spec's forbidden field values.
  - `Uui` decodes the 1-or-2-bit Unlimited-UMV indicator (§5.1.9);
    `InheritedExtendedState` carries the prior `UFEP = "001"` custom-PCF
    state that a `UFEP = "000"` header needs to know whether ETR is
    present (§5.1.4.4 / §5.1.8).
  - `H263PictureLayer` + `parse_picture_layer(reader, inherited)`: a
    unified picture-header entry that returns `Baseline` for non-`"111"`
    source formats and `Extended` (TR + split/doc-cam/freeze prefix +
    `PlusPtypeHeader`) for the extended path. The legacy
    `parse_picture_header` keeps its baseline-only contract and still
    returns `ExtendedPtypeNotSupported` for `"111"`.
  - New `Error::PlusPtypeReservedField` (illegal reserved/fixed bits)
    and `Error::PlusPtypeUnsupported` (Reference Picture Selection
    §5.1.13–§5.1.17, Reference Picture Resampling §5.1.18, or a
    scalability-layer B/EI/EP picture type whose remaining
    variable-length / externally-negotiated header fields are not yet
    staged — refused rather than mis-framed).
  - Out of scope (deferred): the §5.1.11–§5.1.18 scalability / RPS / RPR
    sub-bitstreams (Annexes N, O, P) and wiring `parse_picture_layer`
    into the `decode_picture` driver for custom source formats.
- Annex F §F.3 Overlapped Block Motion Compensation (OBMC) for the
  8×8 luminance prediction (round 12), as a pure function in the
  [`motion`] module:
  - `H0`, `H1`, `H2`: the three Figures F.2 / F.3 / F.4 8×8 weighting
    matrices (`u8`), stored row-major so `H0[j][i] == H0(i, j)` per
    the §F.3 "(i, j) denotes the column and row" indexing note. The
    per-pixel sum `H0[j][i] + H1[j][i] + H2[j][i]` is exactly 8 by
    spec construction, exposed as `OBMC_WEIGHT_SUM`; this guarantees
    the `(... + 4) / 8` rounding step divides cleanly.
  - `RemoteMv { Zero, Current, Vector(MotionVector) }`: the §F.3
    last-paragraph substitution rules for the four remote vectors.
    `Zero` is the "not coded" rule; `Current` is the union of the
    "INTRA / outside picture / current block at bottom of MB → use
    current vector" rules; `Vector(mv)` is the baseline coded-neighbour
    case. `RemoteMv::resolve(current)` returns the half-pel
    `MotionVector` the §F.3 weighted average should sample with.
  - `obmc_predict_block(plane, block_x, block_y, q_mv, r_top, r_bot,
    s_left, s_right, rcontrol)`: produces one 8×8 OBMC prediction
    block. For every output pixel `(i, j)`:
    `P(i, j) = (q · H0[j][i] + r · H1[j][i] + s · H2[j][i] + 4) / 8`
    where `q` / `r` / `s` are §6.1.2 / Figure-13 half-pel bilinear
    samples for the current MV / the top-or-bottom remote MV
    (`j < 4` picks `r_top`, `j >= 4` picks `r_bot`) / the
    left-or-right remote MV (`i < 4` picks `s_left`, `i >= 4` picks
    `s_right`). Reference access uses `RefPlane::at`'s §D.1 edge
    replication. The final pixel is clipped to `[0, 255]` per §6.3.2.
  - Out of scope (deferred to a later round): the macroblock-loop
    driver wiring that, for each INTER4V macroblock, walks the live
    four-MV neighbour grid (left / right / above / below at 8×8
    granularity) and dispatches `obmc_predict_block` four times
    (one per `LumaBlockIndex`) with the correct `RemoteMv`
    classification per the §F.3 last-paragraph rules. The decode
    driver still returns `Error::NotImplemented` for INTER4V.
- Public re-exports from the crate root: `obmc_predict_block`,
  `RemoteMv`, `H0`, `H1`, `H2`, `OBMC_WEIGHT_SUM`.
- 12 unit tests in `motion::tests`: per-pixel weight sum (every
  position sums to 8); `H0` Figure F.2 spot-checks (four corners = 4,
  central 4×4 = 6, first-row non-corner = 5); `H1` Figure F.3
  spot-checks (rows 0/7 all 2, row 1/6 cols 2..=5 are 2 with col 0/7
  edges = 1, interior rows 2..=5 are 1 everywhere); `H2` Figure F.4
  spot-checks (top/bottom row = `[2,1,1,1,1,1,1,2]`, interior rows
  = `[2,2,1,1,1,1,2,2]`); `H1` vs `H2` corner-shape contrast
  (corners both 2 but the "+1 lane" runs along rows for H1 and along
  columns for H2 — not strict transposes); flat-reference identity
  (a uniform plane stays uniform regardless of vectors); all-current
  collapse (q == r == s ⇒ `(q·8 + 4)/8 == q`, matching
  `motion_compensate_block`); zero-vector reference copy on a column
  ramp; `RemoteMv::Zero` vs `RemoteMv::Vector(default())` equivalence;
  top-vs-bottom split observable on a row-ramp reference with hand-
  derived `(j=0, i=4) = 56` and `(j=7, i=4) = 128` per Figures F.2 /
  F.3; left-vs-right split observable on a column-ramp reference with
  hand-derived `(j=2, i=0) = 28` and `(j=2, i=7) = 64` per Figures
  F.2 / F.4; `RemoteMv::resolve` per-variant rule; picture-edge
  replication (a flat reference with the block origin past the right
  edge keeps every prediction pixel at the flat reference value);
  in-range non-degenerate sweep on a mixed reference (the prediction
  has at least two distinct pixel values, confirming the weighted
  sum actually executed).

- Annex F §F.2 Advanced Prediction mode — four-motion-vector
  candidate-predictor redefinition (Figure F.1) and Table F.1
  sixteenth-pixel chrominance vector derivation (round 11), as pure
  transformations in the [`motion`] module:
  - `LumaBlockIndex` (B1 / B2 / B3 / B4) names the four 8×8 luminance
    blocks in a macroblock in Figure 5 / §4.2.5 order. `Mb4Mv =
    [MotionVector; 4]` is the per-macroblock motion-vector array
    aligned with that index. `Mb4MvNeighbourhood { current, left,
    above, above_right, right }` holds the four-vector grids of the
    current macroblock and its four §F.2-relevant neighbours, with
    each external neighbour wrapped in `Option` so the caller can
    encode the §6.1.1 default-to-zero rules for INTRA / not-coded /
    border macroblocks.
  - `select_4mv_candidates(block, &neighbourhood)` implements Figure
    F.1's "the 8×8 block at the physically same relative position
    around MV" mapping. Per the figure, MV1 / MV2 / MV3 for each
    block are: B1 → (B2 of MB-left, B3 of MB-above, B4 of MB-above);
    B2 → (B1 of current, B4 of MB-above, B3 of MB-above-right);
    B3 → (B4 of MB-left, B1 of current, B2 of current); B4 → (B3
    of current, B2 of current, B1 of MB-right). Output feeds directly
    into `predict_mv_median` for the §6.1.1 per-component median.
  - `chroma_mv_component_4mv(luma_sum_half)` and `chroma_mv_4mv(luma)`
    perform §F.2's "sum of the four luminance vectors divided by 8"
    chroma derivation with the Table F.1 sixteenth → half-pixel snap
    (asymmetric `{0,1,2}→0`, `{3..=13}→1`, `{14,15}→2` mapping). The
    arithmetic exploits the identity that our luma half-pel sum is
    *directly* the chroma sixteenth-pel position (since one luma
    half-pel equals four chroma sixteenth-pels), so the computation
    is `(|sum| / 16) * 2 + TABLE_F1[|sum| % 16]` with sign restored.
  - Out of scope for this round (§F.3 overlapped block motion
    compensation — the weighted three-prediction H0/H1/H2 average —
    and the macroblock-loop driver wiring that walks the live
    neighbour grid and feeds it into `select_4mv_candidates`). The
    driver still returns `Error::NotImplemented` when it meets an
    INTER4V macroblock.
- Public re-exports from the crate root: `LumaBlockIndex`, `Mb4Mv`,
  `Mb4MvNeighbourhood`, `select_4mv_candidates`,
  `chroma_mv_component_4mv`, `chroma_mv_4mv`.
- 17 unit tests in `motion::tests`: `LumaBlockIndex` round-trip
  (`ALL.len() == 4`, `from_index` / `index` bijection); per-block
  candidate selection (B1 isolated all-zero, B1 left-only and
  above-only partial neighbourhoods, B2 / B3 full-neighbourhood with
  distinctive vectors, B4 right-edge MV3-zero and B4 with MB-right
  present); one-vector-per-MB equivalence (when every neighbour MB
  carries a uniform 4-MV array, §F.2's candidate selection reduces
  to the Figure-12 single-vector candidates); end-to-end median
  predictor from `select_4mv_candidates` output on a uniform field;
  Table F.1 16-entry transcription (every sixteenth-pel position
  `0..=15` snaps to the documented half-pel position); all-zero
  chroma MV; four-uniform-luma equivalence with the §6.1.1 single-MV
  chroma rule across nine integer-pixel offsets; positive /
  negative sixteenth-snap (one half-pel step out for sum ±4);
  full-pixel integer chroma derivation (sum = ±32 / ±64 →
  exact chroma half-pel multiples); Table F.1 asymmetry round-trip
  (low boundary at sixteenth = 2 / 3 and high boundary at 13 / 14,
  with negative mirror); bounded chroma magnitude sweep across
  `[-200, +200]` sums (`|chroma| ≤ |sum|·2/16 + 2`).

- Annex D §D.2 Unrestricted Motion Vector mode (round 10), PLUSPTYPE
  *absent* case, in the [`motion`] module:
  - `reconstruct_mv_component_umv(predictor, difference)` and
    `reconstruct_mv_umv(predictor, mvd)` extend the per-component
    motion-vector range from the default `[-32, 31]` to `[-63, 63]`
    half-pel (spec `[-31.5, 31.5]`) and apply the §D.2
    predictor-dependent selection of the Table-14 difference pair:
    a predictor inside `[-31, 32]` half-pel uses the first column
    directly (no wrap); a predictor outside that range picks the pair
    member yielding a component inside `[-63, 63]` with the same sign as
    the predictor (zero allowed for either sign).
  - New range constants `MV_UMV_HALF_MIN` (-63) / `MV_UMV_HALF_MAX`
    (+63), re-exported from the crate root alongside the two functions.
  - `decode_picture` applies the §D.2 reconstruction whenever the
    PTYPE bit-10 UMV flag is set; the always-on §D.1 edge replication
    supplies the out-of-picture samples. The PLUSPTYPE / UUI ranges of
    Tables D.1 / D.2 and the Table-D.3 reversible VLC stay gated on the
    not-yet-decoded extended-PTYPE header.
  - 6 unit tests in `motion::tests` (first-column no-wrap; below/above
    range sign-and-bound selection; extended-range invariant across the
    whole UMV space; full-vector application; agreement with the default
    reconstruction where the default sum does not wrap) and 1 driver
    test in `picture::tests` (a UMV vector that would have wrapped under
    the default rule is kept in the extended range).
- Full-picture decode driver (round 9), in the new [`picture`] module:
  - `decode_picture(data, reference, options) -> Result<YuvFrame>`
    walks all GOBs of a picture top-to-bottom (§4.2.1, using the
    per-source-format GOB count and macroblock-rows-per-GOB) and all
    macroblocks of each GOB left-to-right, threading QUANT through the
    GOB header and any per-macroblock DQUANT, and produces a decoded
    planar 4:2:0 [`YuvFrame`].
  - Per-macroblock dispatch: derives each of the six blocks'
    `BlockContext` from the MB type + CBPY (luma, with INTER
    complement) / CBPC (chroma) bits and runs `reconstruct_intra_block`
    (INTRA / INTRA+Q) or `reconstruct_inter_block_with_prediction`
    (INTER / INTER+Q). Skipped macroblocks (COD = 1) copy the
    reference with a zero motion vector (§5.3.1).
  - §6.1.1 / Figure-12 motion-vector prediction: implements the
    candidate border-decision rules (rule 1 INTRA / not-coded → zero;
    rule 2 left-border MV1 zero; rule 3 top / GOB-top MV2,MV3 ← MV1;
    rule 4 right-border MV3 zero) against a live macroblock grid, then
    reconstructs the luma MV with the Table-14 MVD, motion-compensates
    the four luma blocks and the Table-18-derived chroma blocks, and
    sums the IDCT residuals.
  - Optional Annex J §J.3 deblocking via `DecodeOptions::deblock`:
    runs `deblock_plane` over all three planes with a per-edge
    `EdgeCondition` derived from the grid's coded/not-coded state and
    each macroblock's QUANT (Table J.2 STRENGTH).
  - New source-format layout helpers on `H263SourceFormat`:
    `num_gobs`, `mb_rows_per_gob`, `mbs_per_row`, `total_macroblocks`.
  - Out of scope (return `Error::NotImplemented`): INTER4V / INTER4V+Q
    (Annex F four-vector prediction), PB-frames, extended PTYPE,
    Annex T DQUANT, CPM = 1, slice-structured mode, custom source
    formats, and an INTER picture with no reference.
- Public `picture` module re-exported from the crate root:
  `decode_picture`, `DecodeOptions`, `YuvFrame`.
- 19 unit tests in `picture::tests`: per-format GOB/MB layout constants
  (QCIF / CIF / 4CIF), `YuvFrame::grey` dimensions, Figure-5 luma-block
  origins, 8×8 block blitting, the five §6.1.1 / Figure-12
  candidate-predictor cases, and seven end-to-end decodes (QCIF INTRA
  DC-only uniform field at two DC levels; INTRA + deblock no-op on a
  flat field; CBPY-driven per-block AC presence; INTER all-skipped
  exact reference copy; INTER +1-pixel horizontal MV shift with §D.1
  edge replication; missing-reference and extended-PTYPE refusals).

- Annex I Advanced INTRA Coding — scan + prediction-mode layer
  (round 8), in the new [`aic`] module:
  - §I.2 INTRA_MODE field VLC (Table I.1) via
    [`aic::decode_intra_mode`] → [`aic::IntraMode`]: `0` → DC-Only,
    `10` → Vertical DC&AC, `11` → Horizontal DC&AC. One mode is
    transmitted per INTRA macroblock when Advanced INTRA Coding is
    in use. EOF mid-field surfaces `Error::UnexpectedEof` rather
    than guessing a mode.
  - §I.3 alternate DCT scans (Figure I.2) as scan-position →
    block-position permutations in the Figure-14 convention:
    [`aic::ALT_HORIZONTAL_TO_BLOCK_POS`] (Figure I.2-a,
    Alternate-Horizontal, horizontal frequencies first) and
    [`aic::ALT_VERTICAL_TO_BLOCK_POS`] (Figure I.2-b,
    Alternate-Vertical, identical to the ITU-T H.262 alternate
    scan). The two scans are transposes of each other.
  - §I.3 scan-selection rule [`aic::scan_for_intra_mode`]: prediction
    mode 0 keeps the Figure-14 zigzag scan, mode 1 selects the
    Alternate-Horizontal scan, mode 2 the Alternate-Vertical scan.
  - The remainder of Annex I — the Table I.2 separate
    INTRA-coefficient VLC, the §I.3 modified inverse quantization
    (`RecC = 2·QUANT·LEVEL`, no dead-zone) with variable-step
    INTRADC, and the DC/AC prediction reconstruction + `oddifyclipDC`
    / `clipAC` clipping — is deferred (it needs the macroblock-grid
    driver to supply the neighbouring reconstructed blocks).
- Public `aic` module re-exported from the crate root:
  `decode_intra_mode`, `scan_for_intra_mode`, `IntraMode`,
  `ALT_HORIZONTAL_TO_BLOCK_POS`, `ALT_VERTICAL_TO_BLOCK_POS`.
- 15 unit tests in `aic::tests`: the three Table I.1 INTRA_MODE
  codes; exact-bit-consumption for the 1-bit (`0`) and 2-bit
  (`10` / `11`) forms; EOF mid-field and EOF on an empty buffer;
  `IntraMode::index` round-trip; both alternate scans are
  permutations of 0..=63; DC is first in every scan; the
  Alternate-Vertical scan is the transpose of the
  Alternate-Horizontal scan; the scans differ off-DC; Figure-I.2
  spot-checks for both grids; and the §I.3 scan-selection rule.

- Annex J Deblocking Filter mode (round 7):
  - §J.3 four-tap edge filter in [`deblock::filter_edge_samples`] /
    [`deblock::apply_edge_samples`]. For the per-edge sample set
    `(A, B, C, D)` with A, B in `block1` and C, D in `block2`,
    computes `d = (A − 4B + 4C − D) / 8`,
    `d1 = UpDownRamp(d, STRENGTH)`,
    `d2 = clipd1((A − D) / 4, d1 / 2)`,
    `B1 = clip(B + d1)`, `C1 = clip(C − d1)`,
    `A1 = A − d2`, `D1 = D + d2`, with division truncating toward
    zero per §J.3 ("/ denotes division by truncation toward zero").
    `B1` and `C1` are clipped to `[0, 255]` per §6.3.2; `A1` and
    `D1` are also clipped in the `apply_edge_samples` convenience
    (defensive — §J.3's commentary asserts in-range by design).
  - §J.3 `UpDownRamp(x, STRENGTH)` function (Figure J.2):
    `SIGN(x) · max(0, |x| − max(0, 2·(|x| − STRENGTH)))`. The
    [`deblock::up_down_ramp`] implementation handles zero input,
    the identity-inside-strength-window region (`|x| ≤ S`), the
    descending-slope region (`S < |x| ≤ 2S`), and the
    above-2·STRENGTH zero region.
  - §J.3 `clipd1(x, lim) = clamp(x, −|lim|, +|lim|)` per spec.
  - Table J.2/H.263 transcription: full QUANT → STRENGTH lookup
    for QUANT `1..=31`, exposed as [`deblock::strength_for_quant`].
  - §J.3 plane-level driver [`deblock::deblock_plane`]: walks every
    8×8 horizontal edge first (per the §J.3 ordering rule
    "horizontal-before-vertical"), then every vertical edge.
    Built-in picture-edge skip per the §J.3 rule "no filtering
    across a picture edge". Per-edge application condition (the
    §J.3 "block1 coded OR block2 coded" rule and §K/§R slice
    skips) expressed by a caller-supplied closure that returns an
    [`deblock::EdgeCondition`] (`Filter { strength }` or `Skip`).
  - §Q.7.2 Reduced-Resolution Update mode escape: the
    [`deblock::STRENGTH_RRU_INFINITE`] constant degenerates
    `UpDownRamp` to the identity transform per §Q.7.2 ("the
    parameter STRENGTH is given the value of positive infinity").
- Public `deblock` module re-exported from the crate root:
  `apply_edge_samples`, `clipd1`, `deblock_plane`,
  `filter_edge_samples`, `strength_for_quant`, `up_down_ramp`,
  `EdgeCondition`, `STRENGTH_RRU_INFINITE`.
- 21 unit tests in `deblock::tests`: full Table J.2 transcription
  cross-check across all 31 QUANT values, QUANT clamping for
  out-of-range input, `UpDownRamp` zero-input, identity inside
  strength window (every `|x| ≤ S` for `S ∈ 1..=12`), descending
  segment spot-checks at S=7 (|x|=8 → 6, |x|=10 → 4, |x|=13 → 1,
  |x|=14 → 0), above-2S zero region, RRU-infinite identity sweep,
  `clipd1` symmetry, the four-tap filter on flat input
  (identity), a hand-derived in-window jump (A=B=100, C=D=120,
  STRENGTH=5 → A1=101, B1=103, C1=117, D1=119), a strong-edge
  preservation case (A=B=10, C=D=250, STRENGTH=5 → unchanged),
  a B1/C1 clip-overflow check, a 1296-input never-panic sweep,
  and six plane-driver tests covering flat picture (no-op),
  all-skip (no-op), near-edge-only modification, horizontal-stripes
  only-horizontal-pass-active, orientation symmetry across the
  H/V axes, and bad-dimension panic assertions.

- P-frame motion compensation + INTER-block reconstruction (round 6):
  - §6.1.1 differential motion-vector reconstruction. Each Table 14
    MVD code is a *pair* of difference values; only one yields a
    component in the permitted range `[-16, 15.5]` (= `[-32, 31]` in
    half-pel units, a 64-wide window). `reconstruct_mv_component`
    forms `predictor + difference` and wraps it into that window;
    `reconstruct_mv` applies it per-component to an [`Mvd`].
  - §6.1.1 median predictor: `median3` (three-candidate median) and
    `predict_mv_median` (per-component median of MV1/MV2/MV3 from
    Figure 12). The Figure-12 border decision rules that *select*
    the three candidates are the macroblock-loop driver's job (not
    yet wired) — these functions take the candidates as given.
  - Table 18 chrominance-vector derivation: `chroma_mv_component`
    halves the luma component and snaps the quarter-pel fraction to
    the nearest half-pel position (0→0, ¼/½/¾→½, 1→1).
  - §6.1.2 / Figure 13 half-pixel bilinear interpolation with
    `RCONTROL` (implied `0` in baseline): `a = A`,
    `b = (A+B+1−RCONTROL)/2`, `c = (A+C+1−RCONTROL)/2`,
    `d = (A+B+C+D+2−RCONTROL)/4` (truncating division). Reference
    access uses §D.1 edge replication (out-of-bounds → nearest edge
    pixel).
  - §6.1 block-level motion compensation: `motion_compensate_block`
    fetches an 8×8 motion-compensated prediction from a `RefPlane`
    view of a reference-picture plane at a given block origin + MV.
  - §6.3.1 summation + §6.3.2 clip: `reconstruct_inter_block` sums
    the motion-compensated prediction with the IDCT residual and
    clips to `[0, 255]`.
  - End-to-end composer `reconstruct_inter_block_with_prediction`
    (lib root): dequant (no INTRA DC bypass) → §6.2.2 clip → zigzag
    scatter → IDCT → §6.3.1 summation → §6.3.2 clip.
- Public `motion` module with `MotionVector`, `RefPlane`,
  `reconstruct_mv`, `reconstruct_mv_component`, `predict_mv_median`,
  `median3`, `chroma_mv`, `chroma_mv_component`,
  `motion_compensate_block`, `reconstruct_inter_block`, the
  `MV_HALF_MIN` / `MV_HALF_MAX` / `MV_HALF_SPAN` / `RCONTROL_DEFAULT`
  constants.
- 24 unit tests in `motion::tests`: MV-component zero / in-range /
  high-wrap / low-wrap / always-in-range exhaustive sweep, full-MV
  reconstruct, median3 + per-component median, Table 18 chroma
  (zero-fraction + fraction-snaps-to-half + negatives), half-pel
  integer / horizontal (RCONTROL 0 and 1) / vertical / diagonal /
  edge-replication, block MC (zero vector, integer shift, half-pel
  shift), INTER summation (zero residual, additive residual, §6.3.2
  clip at both extremes, flat-plane end-to-end).

- Inverse quantisation, zigzag scatter, and 8×8 inverse DCT (round 5):
  - §6.1 / §6.2.1 H.261-style modulo-2-oddifier inverse-quant rule
    applied to AC coefficients (DC slot preserved for INTRA — already
    holds the Table 15 reconstruction level from round 4):
    `|REC| = QUANT · (2 · |LEVEL| + 1)` for odd QUANT,
    `|REC| = QUANT · (2 · |LEVEL| + 1) − 1` for even QUANT,
    then `REC = sign(LEVEL) · |REC|`.
  - §6.2.2 reconstruction-level clip to `[-2048, +2047]` applied to
    AC slots in-place.
  - §6.2.3 / Figure 14 zigzag → 8×8 scatter via
    [`scatter_into_block`].
  - §6.2.4 inverse DCT computed directly in `f64` against a 64-entry
    `cos(π·(2n+1)·k/16)` table. Output rounded to nearest integer
    and clipped to `[-256, +255]` per §6.2.4. Annex A.7 accuracy
    budget satisfied by construction (the kernel matches the
    "at least 64-bit floating point" reference exactly).
  - §6.3.2 intra-block 8-bit sample clip to `[0, 255]`.
  - End-to-end composer [`reconstruct_intra_block`] takes a parsed
    [`H263Block`] + QUANT and emits an 8×8 `u8` sample block.
- Public `dequant` module with `dequantise_ac`, `scatter_into_block`,
  `AC_REC_MIN`, `AC_REC_MAX`.
- Public `idct` module with `idct_8x8`, `reconstruct_intra_samples`,
  `BLOCK_DIM`, `IDCT_OUT_MIN`, `IDCT_OUT_MAX`.
- 30 unit tests across `dequant::tests`, `idct::tests`, and
  `lib_tests`: zero-level stays zero across all 31 QUANT values,
  odd/even QUANT spot-checks at small and large LEVELs, the
  "REC is always odd" §6.2.1 invariant, INTRA DC preservation,
  INTER slot-0 processing, §6.2.2 clip at both extremes, scatter
  permutation bijectivity, §A.8 zero-in/zero-out invariant,
  DC-only IDCT spot-checks at positive and negative DC, the
  single-AC-coefficient horizontal-basis ±1 error budget, IDCT
  diagonal symmetry, IDCT saturation safety, intra-DC uniform-field
  reconstruct at three DC values, INTRADC = 800 → pixel 100,
  DC + small AC pattern with sign-reversal across the row,
  intra-reconstruction clipped at both 0 and 255 by §6.3.2.

### Block-layer parser (round 4, kept for context)

- Block-layer parser (§5.4, non-PB-frame, non-Annex-T, non-Annex-I
  baseline subset):
  - §5.4.1 INTRADC — 8-bit FLC with two forbidden codes (`0x00`,
    `0x80`) per Table 15, plus the `0xFF`-means-1024 special case;
    all other codes map linearly to `code * 8`.
  - §5.4.2 TCOEF — full Table 16 transcribed: 102 regular VLC
    code-points (each followed by a single sign bit) plus the
    `0000 011` ESCAPE prefix followed by a fixed-length 22-bit
    event (`LAST (1) || RUN (6) || LEVEL (8, two's complement,
    forbidden codes `0x00` / `0x80` in baseline)`). The VLC
    dispatcher reads up to 13 prefix bits and matches against the
    table by `(prefix-length, prefix-value)`; ESCAPE LEVEL is
    interpreted as `i8` two's complement.
  - Coefficient accumulation into a 64-entry `coefficients` array
    in **zigzag scan position order** (slot 0 = DC). The
    §6.2.3 / Figure 14 zigzag → 8×8 block-position permutation is
    exposed as the `ZIGZAG_TO_BLOCK_POS` constant for callers that
    need to scatter into pixel layout; the parser itself stays in
    scan order.
  - Per-block `BlockContext` (`has_intradc`, `has_coefficients`)
    threaded by the caller from MB type + CBPY/CBPC bits.
- Public `block` module with `H263Block`, `BlockContext`,
  `parse_block`, `COEFFS_PER_BLOCK`, `ZIGZAG_TO_BLOCK_POS`.
- 21 unit tests in `block::tests`: empty INTER skip, INTRA-DC-only,
  INTRADC Table 15 spot-check across both special-case codes and
  the linear range, single-event INTER, INTRA + AC, two-event with
  RUN gap, full ESCAPE round-trip (positive + negative LEVEL),
  both ESCAPE LEVEL forbidden codes, RUN-overflow rejection, the
  full 102-entry Table 16 round-trip across both sign polarities,
  Table-16 row-count invariant, zigzag-table endpoints +
  permutation invariant, truncated-INTRADC EOF, invalid-prefix
  rejection, and a `picture → GOB → MB → block` composition test.
- New error variants `BadIntradcCode`, `BadTcoefCode`,
  `BadTcoefEscapeLevel`, `BadTcoefRunOverflow`.

### Not yet wired (after round 6)

- Macroblock-loop assembly: the per-MB driver that walks all six
  blocks (deriving each block's `BlockContext` from MB type +
  CBPY/CBPC bits), selects the three §6.1.1 / Figure-12 MV-prediction
  candidates per the border decision rules, allocates per-frame
  picture planes, and dispatches `reconstruct_intra_block` /
  `reconstruct_inter_block_with_prediction` per block, is not yet
  wired. Round 6 provides the per-block INTER primitives; the driver
  that calls them across a whole picture is the next step.
- §6.1.1 Figure-12 border decision rules (candidate-predictor
  selection at GOB / picture edges). Round 6's `predict_mv_median`
  takes the three candidates as given; the rules that derive them
  (zero-out for INTRA / not-coded / outside-picture neighbours) need
  the macroblock grid + COD state from the driver loop.

### Not yet wired (after round 5, superseded by round 6 above for §6.1)

- P-frame motion compensation (§6.1, including §6.1.2 half-pel
  bilinear interpolation) — round 5 reconstructs INTRA blocks only.
  INTER block reconstruction needs the motion-compensated
  prediction added before §6.3.2 clipping.
- §6.3.2 deblocking filter for the §G.2 Improved PB-frames mode
  (not the §6.3.2 final-clip step, which is implemented).
- Annex I (Advanced INTRA Coding) — alternate scans + INTRADC as
  a regular AC-coded value.
- Annex T (Modified Quantization) — EXTENDED-ESCAPE LEVEL prefix
  (relaxes the `0x80` forbidden code).
- PB-frames B-block decode (§5.4 last paragraph + Annex G).
- Annex J deblocking filter.

- Macroblock-layer header parser (§5.3, non-PB-frame baseline subset):
  - §5.3.1 COD (1 bit, INTER pictures only); §5.3.2 MCBPC
    decoded against both Table 7 (I-pictures, 9 codes) and
    Table 8 (P-pictures, 25 codes including type-5
    INTER4V+Q points reserved for PLUSPTYPE + Annex F/J);
    §5.3.5 CBPY decoded against Table 12 (all 16 patterns);
    §5.3.6 DQUANT baseline two-bit form with QUANT clipped to
    `1..=31` per spec; §5.3.7 / §5.3.8 MVD + MVD2-4 decoded
    against Table 14 (all 64 codes), returned in half-pel
    units as signed `i8` in `[-32, +31]`.
  - VLC dispatch uses `BitReader::read_unary` for the
    leading-zero prefix and per-bucket suffix bits — every
    bit-pattern matches the spec's MSB-first printed code
    1-for-1.
- Public `H263Macroblock`, `MbType`, `MbContext`, `Mvd`
  types and `parse_macroblock` free function in the new
  `macroblock` module.
- 20 unit tests against synthetic buffers (full Table 7,
  full Table 8 indices 0..=20, full type-5 sub-codes, full
  Table 12, full Table 14 + extremes + non-zero round-trip),
  the COD-skip path, DQUANT clamping at both ends, the
  MCBPC stuffing path, and a composition test that chains
  picture / GOB / macroblock parsers through a single
  `BitReader`.
- New error variants `BadMcbpcCode`, `BadCbpyCode`,
  `BadMvdCode`.

### Not yet wired (after round 3, superseded by round 4 above
###  for §5.4)

- PB-frame MODB / CBPB / MVDB (§5.3.3 / §5.3.4 / §5.3.9,
  Annex G); the parser refuses no fields but the caller's
  picture context must keep `pb_frames = false`.
- Annex T variable-length DQUANT (Modified Quantization
  mode); the baseline 2-bit form is the only one decoded.
- Annex D Table D.3 alternative MVD codes — round 3 uses
  Table 14 unconditionally.
- Annex O B/EI/EP picture macroblocks.

- GOB-layer header parser (§5.2, CPM = "0" branch):
  - GBSC detection (17 bits, value `0000 0000 0000 0000 1`).
  - GN (5 bits), accepted in `1..=29` (covers both standard and
    custom picture-format ranges); `0` / `30` / `31` rejected as PSC
    overlap / EOSBS / EOS markers.
  - GFID (2 bits) consumed and exposed for future inter-GOB
    continuity checks.
  - GQUANT (5 bits) decoded as QUANT in `1..=31`; `0` rejected.
- Public `GobLayer`, `parse_gob_layer`,
  `parse_gob_layer_from_bytes`, and the `GBSC_VALUE` / `GBSC_BITS` /
  `GN_BITS` / `GFID_BITS` / `GQUANT_BITS` /
  `GOB_HEADER_BITS_NO_CPM` constants.
- 14 unit tests against synthetic buffers built per §5.2 bit layout,
  including one that chains the round-1 picture header with a
  round-2 GOB header through a single `BitReader` and asserts the
  reader is left at the first byte of macroblock data.
- New error variants: `BadGroupStartCode`, `InvalidGroupNumber`,
  `InvalidQuantiser`.
- Picture-layer header parser (§5.1, non-extended PTYPE):
  - PSC detection (22 bits, value `0x000020`).
  - Temporal Reference (8 bits at standard CIF PCF).
  - PTYPE bit-by-bit decode: split-screen / document-camera /
    freeze-release indicators, source-format field, picture coding
    type, and Annex D/E/F/G optional-mode flags.
- Public `H263PictureHeader`, `H263SourceFormat`,
  `H263PictureCodingType` types with `luma_dimensions()` per §4.1.
- 8 unit tests against synthetic buffers built per §5.1 bit layout.
- Distinct error variants (`BadPictureStartCode`,
  `BadPtypeFixedBits`, `ForbiddenSourceFormat`,
  `ExtendedPtypeNotSupported`, `UnexpectedEof`) so callers can
  surface bitstream problems specifically.

### Not yet wired

- Macroblock / motion-vector / DCT decode (§5.3, §5.4, Annex H).
- GSTUF stuffing (§5.2.1) — caller's responsibility to skip before
  invoking the GOB parser.
- GSBI (§5.2.4, CPM = "1" branch) — picture-layer CPM bit is not yet
  exposed by the round-1 parser, so the GOB parser only handles the
  CPM = "0" case.
- Slice-structured mode (Annex K).
- Annex-O optional fields after PTYPE (PQUANT, CPM/PSBI, TRB,
  DBQUANT, PEI/PSUPP).
- Extended PTYPE / PLUSPTYPE path (§5.1.4) and every annex it gates.
- `oxideav_core::Decoder` registration — `register()` is still a
  no-op pending a frame-yielding decoder.

### Erased

- Prior master history was force-erased on **2026-05-18** under
  Hat-3 cold enforcement of the workspace clean-room policy
  (`docs/IMPLEMENTOR_ROUND.md`).

### Reset

- Crate reduced to a minimal `oxideav_core::register!` stub. Every
  public API returns `Error::NotImplemented`. The crates.io version
  (`0.0.8`) is preserved on the new master to avoid breaking
  downstream version pins; the published versions on crates.io will
  be yanked by the maintainer.
- The `oxideav-mpeg4video` runtime dependency is dropped from the
  scaffold (the prior code reused mpeg4video's VLC tables; the
  rebuilt h263 will derive its own tables directly from the H.263
  spec).
