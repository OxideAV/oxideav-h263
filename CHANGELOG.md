# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- §5.3.3 / §5.3.4 PB-frame B-block field parsers (round 231):
  - `pb_layer` new public module.
  - `ModbPresence` new public enum (variants `None` / `MvdbOnly` /
    `CbpbAndMvdb`) collapses the §5.3.3 Table 11 CBPB-and-MVDB
    presence columns onto a tag. `has_cbpb()`, `has_mvdb()`, and
    `code_bits()` accessors.
  - `parse_modb(reader) -> Result<ModbPresence>` new public function
    decoding the Table 11 1-or-2-bit variable-length codeword.
    Leading bit `0` resolves `None` immediately; leading bit `1`
    consumes one more bit (`0` → `MvdbOnly`, `1` → `CbpbAndMvdb`).
    Only `Error::UnexpectedEof` is possible — every legal Table 11
    prefix shape is non-empty.
  - `parse_cbpb(reader) -> Result<u8>` new public function decoding
    the §5.3.4 6-bit fixed-length CBPB Coded Block Pattern. Returns
    the raw six bits in the low bits of a `u8`. Bit 5 (MSB of the
    field) carries B-block number 1's CBPBN per §5.3.4 / Figure 5
    "the utmost left bit of CBPB corresponding with block number 1".
  - `cbpb_block_present(cbpb, block_number) -> bool` new public
    function queries an individual B-block's CBPBN bit by 1-based
    block number; defensively returns `false` for `block_number`
    outside `1..=6`.
  - `CBPB_BITS = 6` new public constant.
  - 14 new tests (`cargo test -p oxideav-h263` reports 425 passed,
    previously 411): the three Table 11 codewords round-trip
    (`None` ← `0`, `MvdbOnly` ← `10`, `CbpbAndMvdb` ← `11`); MODB
    truncated after the leading `1` returns `UnexpectedEof`; MODB
    on an empty buffer returns `UnexpectedEof`; CBPB all-zero and
    all-one patterns round-trip; CBPB single-bit-per-block
    isolation across all six block positions; CBPB block-1-is-MSB
    / block-6-is-LSB endpoint pin test; CBPB truncated and empty
    buffer EOFs; out-of-range block-number queries return `false`;
    `code_bits()` agrees with the reader's bit advance; end-to-end
    chain test that an `11` MODB followed by a CBPB of `10_1010`
    advances the reader exactly 8 bits and isolates B-blocks 1, 3,
    5 as carrying coefficients.

- §K.2.1 SSTUF stuffing skipper (round 226):
  - `skip_sstuf(reader)` new public function in `slice_header.rs`.
    Reads the `0..=7` trailing zero bits of the byte the reader is
    inside, verifies all of them are `0` (the §K.2.1 stuffing-bit
    value), and returns the number of bits discarded. On a reader
    already on a byte boundary returns `Ok(0)` without consuming
    any bits. Leaves the reader positioned at the MSB of the byte
    that holds the SSC codeword.
  - `skip_sstuf_at(data, byte_offset, bit_offset) -> Result<(u32,
    u64)>` new public function. Byte-cursor wrapper: constructs a
    `BitReader` over `data` at `byte_offset + bit_offset/8`,
    advances `bit_offset % 8` bits, calls `skip_sstuf`, and
    returns `(bits_skipped, final_bit_position)`. Out-of-range
    `byte_offset` yields `Error::UnexpectedEof`. Useful for callers
    that walk a `(byte, bit)` cursor through a longer bitstream
    and need to recover SSC byte alignment without building a
    `BitReader` themselves.
  - `SSTUF_MAX_BITS = 7` new public constant.
  - `Error::BadSliceStuffing` new variant: one of the SSTUF bits
    was `1` where §K.2.1 mandates `0`.
  - 10 new tests (`cargo test -p oxideav-h263` reports 411
    passed, previously 401): byte-aligned reader returns 0, 1-bit
    SSTUF run, 7-bit SSTUF run, non-zero stuffing rejection,
    empty-buffer EOF, `skip_sstuf_at` byte-cursor walk,
    `skip_sstuf_at` with `bit_offset >= 8` (folds to next byte),
    `skip_sstuf_at` chained into `parse_slice_layer` end-to-end
    over a QCIF context, `skip_sstuf_at` OOB byte offset, and
    `skip_sstuf_at` aligned position returns `(0, 0)`.

- §K.2 `SliceHeaderContext` constructor from a `PictureLayout` +
  §5.1.10 SSS submode bits (round 220):
  - `SliceHeaderContext::from_picture_layout(layout, sss, cpm, rru)`
    new public constructor in `slice_header.rs`. Takes the canonical
    `PictureLayout` (both the baseline and §4.2.1 / §5.1.5
    custom-format luma-dimension carrier post-r214) plus the four
    orthogonal mode flags the §K.2 syntax depends on
    (`Option<SliceStructuredSubmode>`, CPM, RRU). The §K.2.5 /
    §K.2.8 field-width lookups inside `SliceHeaderContext` already
    pick the "first table entry that has an equal or larger number
    of macroblocks" / "next standard format size which is equal or
    larger in width" rule per §K.2.5 / §K.2.8 for custom picture
    sizes, so the constructor is a shape adapter — no new table
    data lands.
  - The `arbitrary_order` bit of `SliceStructuredSubmode` does not
    affect any §K.2 field width or value range — it only influences
    slice scheduling at the driver layer — so it is intentionally
    ignored by the constructor; only the `rectangular` bit
    propagates to `rectangular_slices`.

- §4.2.1 / §5.1.5 custom-source-format GOB-layout driver wiring
  (round 214):
  - `PictureLayout { luma_width, luma_height, num_gobs,
    mb_rows_per_gob }` new public struct in `picture.rs`. Captures
    the §4.2.1 GOB grid + dimensions the picture-decode walker uses;
    decouples the inner driver from `H263SourceFormat`.
  - `PictureLayout::for_source_format(H263SourceFormat)` resolves the
    five fixed baseline formats to their §4.2.1 grids.
  - `PictureLayout::for_custom_dimensions(luma_width, luma_height)`
    resolves a custom-source-format size to the §4.2.1 + Table-4
    `k`-parameter GOB grid (`k = 1` for ≤400 lines, `k = 2` for
    404..=800, `k = 4` for 804..=1152; `num_gobs = ceil(height /
    (k * 16))` with the §4.2.1 truncated-bottom-GOB rule when the
    height is not an integer multiple of `k * 16`). Returns `None`
    for sizes outside `[4, 2048] × [4, 1152]` and for spec-legal
    4-aligned sizes that are not 16-aligned (the per-MB raster
    requires macroblock-aligned dimensions).
  - `decode_picture_layer` and `decode_picture_layer_with_inherited`
    decode PLUSPTYPE pictures carrying source-format `"110"`
    (Custom) end-to-end: UFEP=001 sizes the GOB grid from the
    on-wire CPFMT; UFEP=000 sizes it from the inherited snapshot
    (see below).
  - `InheritedExtendedState` extended with a new
    `custom_dimensions: Option<(u32, u32)>` field captured from the
    last UFEP=001 picture's CPFMT (`Some` iff the prior UFEP=001
    carried `PlusSourceFormat::Custom`).
    `InheritedExtendedState::from_opptype_with_cpfmt(opptype, cpfmt)`
    new constructor populates the field;
    `InheritedExtendedState::from_opptype` continues to set it to
    `None` for the fixed-format inheritance path.
  - 7 new tests (`cargo test -p oxideav-h263` reports 391 passed,
    previously 385): a CPFMT-described 176×144 PLUSPTYPE INTRA
    picture decodes through `decode_picture_layer` to a frame
    sample-bit-identical to the same body decoded under the fixed
    QCIF source format; `PictureLayout::for_custom_dimensions`
    table-4 boundary tests for `k = 1`/`2`/`4` at 400 / 416 / 800 /
    816 / 1152 lines and the truncated-bottom-GOB case at 432 lines;
    out-of-range / non-16-aligned rejection; UFEP=001 +
    `PlusSourceFormat::Custom` captures the CPFMT-derived
    `(176, 144)` into the snapshot's `custom_dimensions`; UFEP=000 +
    inherited `PlusSourceFormat::Custom` + `custom_dimensions =
    Some((176, 144))` decodes the same body sample-bit-identically;
    and UFEP=000 + inherited Custom with `custom_dimensions = None`
    is refused.

- §5.1.4.4 / §5.1.4.5 PLUSPTYPE inherited-state stream driver
  (round 208):
  - `decode_picture_layer_with_inherited(data, reference, options,
    inherited)` new public function in `picture.rs`. Stream-aware
    counterpart to `decode_picture_layer`: accepts caller-supplied
    [`InheritedExtendedState`] and returns a `DecodePictureOutcome`
    carrying the decoded frame and the next-inherited snapshot the
    caller should thread into the following picture's decode.
  - `InheritedExtendedState` extended from the single-field
    `custom_pcf` snapshot into a full §5.1.4.4 mode + source-format
    capture: `source_format: Option<PlusSourceFormat>` (None when no
    prior UFEP=001 has been seen — UFEP=000 picture refused in that
    case), `umv`, `advanced_prediction`, `advanced_intra`, `deblocking`
    (refused-mode bits SAC / SS / IS / AIV / MQ / RPS are not retained
    because a follow-up UFEP=000 inheriting any of them would have
    already been refused at the prior UFEP=001 picture).
    `InheritedExtendedState::from_opptype` captures the snapshot from a
    parsed `Opptype`.
  - `plus_ptype_to_baseline_shim` extended to take `inherited`; on a
    UFEP=000 picture it falls back to the snapshot for source-format
    and OPPTYPE mode bits instead of refusing immediately.
  - §5.1.4.5 rule 1 — UMV (Annex D) and Advanced Prediction (Annex F)
    are inferred-off in I-pictures even when the inherited snapshot
    has them on. The override is applied to the synthetic baseline
    header the shim builds; the returned snapshot preserves the
    un-overridden stream state so a subsequent P-picture re-enables
    the modes without needing another UFEP=001.
  - §5.1.4.5 rule 3 — a baseline-PTYPE picture resets the outgoing
    snapshot to `InheritedExtendedState::default()` (all modes off,
    `source_format: None`).
  - `DecodePictureOutcome { frame, inherited }` new public struct
    re-exported from the crate root alongside the new entry point.
  - 7 new tests: UFEP=000 INTRA PLUSPTYPE picture with caller-supplied
    AIC-on snapshot reproduces the round-21 baseline-header AIC `+1`
    prediction footprint (130/132/132/134) and passes the snapshot
    through unchanged; UFEP=000 with no prior UFEP=001 (`source_format
    = None`) refused with `Error::NotImplemented`; UFEP=001 picture
    captures the OPPTYPE into `outcome.inherited`; baseline-PTYPE
    picture clears the snapshot to default; `decode_picture_layer`
    matches the new entry point's frame on a UFEP=001 PLUSPTYPE AIC
    picture; §5.1.4.5 rule-1 override (UFEP=000 INTRA inheriting
    UMV=on / AP=on decodes cleanly, snapshot preserved un-overridden);
    `InheritedExtendedState::from_opptype` captures only the driver-
    staged bits.
- `decode_picture_layer_with_inherited` / `DecodePictureOutcome`
  re-exported from the crate root alongside `decode_picture` /
  `decode_picture_layer` / `DecodeOptions` / `YuvFrame`.

- PLUSPTYPE → `DecodeOptions` auto-wiring driver entry point
  (round 202):
  - `decode_picture_layer(data, reference, options)` new public
    function in `picture.rs`. Dispatches `parse_picture_layer` between
    the baseline and extended-PTYPE paths and runs the shared
    `decode_after_picture_header` inner driver. On the `Extended` arm,
    `plus_ptype_to_baseline_shim` validates the picture against the
    driver's supported-layer-set (UFEP=001, one of the five standardised
    source formats, no custom-PCF, no CPM, no SAC, no SS, no IS, no
    AIV, no MQ, no RRU, INTRA/INTER picture type only, UMV either off
    or with `UUI = "1"`) and refuses anything else with
    `Error::NotImplemented` rather than mis-framing.
  - The shim OR-merges the wire-signalled `advanced_intra` and
    `deblocking` flags into the caller's `DecodeOptions` so an AIC- or
    DF-mode picture decodes correctly without the caller having to
    pre-set the option flags. UMV / Advanced Prediction wire flags
    drive the matching parser paths through the synthetic baseline
    header (no `DecodeOptions` plumbing needed).
  - The inner driver was refactored: the body of `decode_picture`
    after `parse_picture_header` became a shared
    `decode_after_picture_header(reader, header, reference, options)`
    helper. `decode_picture` now wraps `parse_picture_header` +
    `decode_after_picture_header`. The legacy entry point retains its
    `Error::ExtendedPtypeNotSupported` rejection of `"111"`
    source-format pictures.
  - 9 new tests: a synthetic QCIF PLUSPTYPE AIC INTRA picture decoded
    through `decode_picture_layer` with `DecodeOptions::default()`
    reproduces the round-21 baseline-header AIC `+1` prediction
    footprint (pixel 130 / 132 / 132 / 134); a PLUSPTYPE non-AIC INTRA
    picture decodes through the §5.3 / §6.1 baseline body and does
    NOT exhibit any AIC-predictor pixel value; a baseline-header
    passthrough test asserts `decode_picture_layer` produces an
    identical frame to `decode_picture` for the existing QCIF INTRA
    fixture; a caller-on-wire-on OR-merge test for AIC; an OPPTYPE-DF
    auto-wiring test (uniform AIC picture survives the deblocking
    filter unchanged); and four explicit `Error::NotImplemented`
    refusals for SAC, slice-structured, custom-format, and
    `UFEP="000"` PLUSPTYPE pictures.
- `decode_picture_layer` re-exported from the crate root alongside
  `decode_picture` / `DecodeOptions` / `YuvFrame`.

- Annex I §I.2 / §I.3 macroblock-grid driver wiring (round 196):
  - `DecodeOptions::aic` opt-in switches the picture driver to the
    Advanced INTRA Coding code path. When set, every INTRA macroblock
    is dispatched through a new `decode_intra_macroblock_aic` helper
    that:
    - Reads the §I.2 `INTRA_MODE` VLC between MCBPC and CBPY
      (added to `parse_macroblock` via the new
      `MbContext::aic_intra_mode` flag and surfaced on the new
      `H263Macroblock::intra_mode` field).
    - Parses every 8×8 block with `block_aic::parse_intra_block_aic`
      (absorbed INTRADC per §I.3 line 4214).
    - Assembles `Neighbour::Available(rec_a_prime)` from the 8×8 block
      immediately above and `Neighbour::Available(rec_b_prime)` from
      the block immediately to the left via a per-block
      `AicNeighbourGrid`, applying the §I.3 page-78 "same video picture
      segment" availability rule (segment id = GOB index in the
      baseline driver). Mismatched-segment or non-INTRA neighbours
      collapse to `Neighbour::None`.
    - Composes `aic_intra_reconstruct_coefficients` (modified
      inverse-quant + Figure-I.2 scan scatter + DC/AC prediction with
      `clipAC` / `oddifyclipDC`) followed by
      `aic_intra_reconstruct_samples` (IDCT + §6.3.2 sample clip).
    - Records each block's final `RecC'(u, v)` into the neighbour
      grid for downstream blocks; INTER / skipped macroblocks record
      their slots as non-INTRA so AIC INTRA neighbours of mixed-type
      macroblocks see the correct availability decision.
  - 11 new tests cover: `luma_block_grid_pos` Figure-5 mapping;
    `AicState` initial OUTSIDE state; `record_non_intra_macroblock`
    field-by-field; `aic_luma_neighbour_above` / `_left` border
    collapse; segment-id mismatch collapse; non-INTRA-neighbour
    collapse; intra-same-segment availability; an end-to-end QCIF AIC
    INTRA picture with zero-residual blocks (uniform 128 output via
    DC fallback `1024` → `oddifyclipDC` → `1025` → IDCT 128); an
    end-to-end QCIF AIC INTRA picture with `+1` DC LEVEL on every
    block (top-left luma block recovers pixel 130; block-1 picks up
    block-B predictor → 132; block-2 picks up block-A predictor →
    132; block-3 averages both → 134 — the §I.3 prediction is
    observable in the frame); GOB-boundary segment isolation test
    (the first MB of GOB 1 must NOT pick up GOB 0's block as a
    predictor and falls back to pixel 130).
- `MbContext::aic_intra_mode` (bool, default `false` for callers via
  literal construction; must be set to opt the parser into the §I.2
  INTRA_MODE field).
- `H263Macroblock::intra_mode` (`Option<IntraMode>`, populated only on
  the AIC path for INTRA-coded macroblocks).

## [0.0.8](https://github.com/OxideAV/oxideav-h263/releases/tag/v0.0.8) - 2026-05-30

### Other

- §I.3 end-to-end INTRA-block reconstruction pipeline (round 20)
- Annex I §I.3 INTRA DC/AC prediction reconstruction (round 19)
- Annex I §I.3 absorbed-INTRADC INTRA-block parser (round 18)
- Annex I §I.3 modified inverse-quant primitives (round 17)
- Annex F §F.2 / §F.3 INTER4V + OBMC driver wiring (round 16)
- Annex K §K.2 slice-layer header parse (round 15)
- Annex I §I.3 / Table I.2 INTRA-coefficient VLC (round 14)
- extended-PTYPE (PLUSPTYPE) picture-header parse (round 13)
- Annex F §F.3 OBMC weighted prediction (round 12)
- Annex F §F.2 four-MV candidate-predictor + Table F.1 chroma
- Annex D §D.2 Unrestricted Motion Vector mode (PLUSPTYPE absent)
- full-picture decode driver wiring layers into a YuvFrame
- Annex I Advanced INTRA Coding scan + prediction-mode layer
- Annex J §J.3 in-loop edge filter + Table J.2 STRENGTH
- inter path: §6.1.1 MV reconstruct + §6.1.2 half-pel interp + §6.3.1 INTER summation
- intra reconstruct: §6.1 + §6.2.1 dequant + §6.2.3 scatter + §6.2.4 IDCT + §6.3.2 clip
- §5.4 baseline decode — INTRADC (Table 15) + TCOEF VLC (Table 16)
- §5.3 baseline header — COD / MCBPC / CBPY / DQUANT / MVD
- round 2: GOB-layer header parser per ITU-T H.263 §5.2
- round 1: picture-header parser per ITU-T H.263 §5.1
- orphan rebuild: clean-room scaffold post 2026-05-18 audit

### Added

- Annex I §I.3 end-to-end INTRA-block reconstruction pipeline
  (round 20, two new pure functions in `aic_predict`):
  - `aic_intra_reconstruct_coefficients(zigzag_levels, mode, quant,
    block_a, block_b) -> [i32; 64]` — single pure function that takes the
    [`H263Block`] output of `block_aic::parse_intra_block_aic` (zigzag-
    scan-position-order `LEVEL` integers) plus the macroblock's `QUANT`,
    the `IntraMode` from `aic::decode_intra_mode`, and the §I.3
    `RecA'` / `RecB'` `Neighbour` tags, and returns the final block-
    position `RecC'(u,v)` array. Composes, in order, the round-17
    modified inverse-quantisation formula `RecC = 2·QUANT·LEVEL`
    (`aic_dequant_coefficient` per slot), the round-8 Figure-I.2 /
    scan-selection scatter (`scan_for_intra_mode(mode)` — zigzag for
    `DcOnly`, alternate-horizontal for `VerticalDcAc`,
    alternate-vertical for `HorizontalDcAc`), and the round-19 §I.3
    page-79 DC/AC prediction reconstruction with `clipAC` /
    `oddifyclipDC` (`reconstruct_intra_block_aic`).
  - `aic_intra_reconstruct_samples(rec_c_prime) -> [u8; 64]` — pure
    function that takes the §I.3 final coefficient array (output of
    `aic_intra_reconstruct_coefficients`) and runs the round-5 §6.2.4
    `idct_8x8` followed by the §6.3.2 sample clip to the 8-bit picture
    range `[0, 255]`. The narrowing `as i16` is lossless because
    `clipAC` keeps every AC slot in `[-2048, +2047]` and `clipDC`
    keeps the DC slot in `[0, +2047]`.
  - Together the two helpers cover the four §I.3 downstream pipeline
    steps `block_aic.rs` previously flagged as deferred (modified
    inverse quantisation, scan-scatter, DC/AC prediction, IDCT) as
    pure-function primitives. The split is deliberate: the
    macroblock-grid driver needs the coefficient array as the next
    block's `Neighbour::Available` payload (`RecA'` for the block
    below, `RecB'` for the block to the right), while only the
    `u8` sample array goes into the picture buffer.
  - 12 new unit tests cover: Mode 0 / no-neighbour DC-only uniform
    field; Mode 0 / single-neighbour DC propagation; Mode 1 alternate-
    horizontal scan dispatch (scan position 1 lands at the
    `ALT_HORIZONTAL_TO_BLOCK_POS[1]` slot); Mode 2 alternate-vertical
    scan dispatch; an explicit divergence check between the alternate-
    horizontal and alternate-vertical scans (guarding against a bug
    that would always use the zigzag); Mode 1 / block-A AC predictor
    propagation; Mode 2 / block-B AC predictor propagation; sample-clip
    saturation at the `AIC_DC_REC_MAX` upper bound; the §A.8 all-zeros-
    in / all-zeros-out invariant for the IDCT step; sample-clip
    saturation handling at the `AIC_AC_REC_MIN` lower bound (negative
    lobe of the AC basis pattern); a composition-contract test that
    locks the new helper to manual `aic_dequant_coefficient` + scatter
    + `reconstruct_intra_block_aic` for all three modes on a mixed
    DC+AC block; and a driver-shape feed-back test that uses the
    pipeline output of one block as the `Neighbour::Available`
    payload of a successor block.

- Annex I §I.3 INTRA DC/AC prediction reconstruction
  (round 19, new `aic_predict` module):
  - `reconstruct_intra_block_aic(rec_c_residual, mode, block_a, block_b)
    -> [i32; 64]` — single pure function that applies the three §I.3
    page-79 INTRA_MODE prediction rules to a current INTRA block's
    dequantized residual array and returns the final `RecC'(u,v)`
    array post-`clipAC` (AC slots) and `oddifyclipDC` (DC slot). All
    coefficient arrays are in block-position layout (`index = v * 8 + u`,
    `u` horizontal / `v` vertical), the convention used by the Figure 14
    / Figure I.2 scan-target tables already in `block` and `aic`.
  - `Neighbour<'a>` enum — `None` for "neighbour unavailable" (out of
    picture, INTER-coded, or in a different video picture segment per
    the §I.3 page-78 availability rule) and `Available(&[i32; 64])` for
    "neighbour is INTRA and in the same video picture segment, here's
    its final `RecA'` / `RecB'` array". The driver supplies the
    availability tag; the predictor module does not encode the
    same-segment test itself (which belongs in the driver).
  - `AIC_FALLBACK_DC_PREDICTOR = 1024` constant — the §I.3 fixed
    no-neighbour DC predictor (used by Mode 0 when neither A nor B is
    available, by Mode 1 when A is unavailable, and by Mode 2 when B is
    unavailable).
  - Per-mode reconstruction:
    - **Mode 0** (`IntraMode::DcOnly`): AC slots are bare-residual
      `clipAC`. DC is `oddifyclipDC( RecC(0,0) + predictor )` with
      predictor = `(RecA'(0,0) + RecB'(0,0)) / 2` (truncation toward
      zero per the §I.3 "/" division convention) when both A and B are
      available, a single neighbour's DC if only one is available, and
      `1024` if neither is.
    - **Mode 1** (`IntraMode::VerticalDcAc`): When A is available, DC
      gets `RecA'(0,0)` and AC slots `(u, 0)` for `u = 1..=7` get
      `RecA'(u, 0)`; rows `v = 1..=7` are bare-residual `clipAC`.
      When A is unavailable, DC falls back to `+1024` and no AC slot is
      predicted.
    - **Mode 2** (`IntraMode::HorizontalDcAc`): When B is available, DC
      gets `RecB'(0,0)` and AC slots `(0, v)` for `v = 1..=7` get
      `RecB'(0, v)`; columns `u = 1..=7` are bare-residual `clipAC`.
      When B is unavailable, DC falls back to `+1024` and no AC slot is
      predicted.
  - Composes with the round-14 Table I.2 event decoder
    (`intra_tcoef::decode_intra_tcoef_event`), the round-18 INTRA-block
    parser (`block_aic::parse_intra_block_aic`), the round-17 modified
    inverse-quantization primitives (`aic_dequant_coefficient`), the
    round-8 scan selection (`aic::scan_for_intra_mode`), and the
    `aic_dequant::clip_ac` / `oddify_clip_dc` clipping primitives to
    cover the full §I.3 INTRA-block coefficient pipeline from raw
    bitstream events to a final reconstructed coefficient array. The
    only remaining §I.3 gap is the macroblock-grid driver that walks
    the picture, computes the per-block "same video picture segment"
    availability bits, accumulates reconstructed `RecA'` / `RecB'`
    arrays, and dispatches this primitive plus the inverse DCT — that
    driver is the next round's work.
  - 23 new unit tests cover: Mode 0 with no neighbours / only A / only
    B / both (averaging with truncation toward zero, including the
    negative-sum truncation case); Mode 0 AC slots passing through
    `clipAC` of the bare residual with neighbour AC values ignored;
    Mode 1 with A available (DC + first-row prediction wired correctly,
    rows `v >= 1` left as bare residuals) and with A unavailable (DC
    falls back to `+1024`); Mode 2 symmetric to Mode 1 but for the
    first column; AC upper / lower `clipAC` saturation; DC
    `oddifyclipDC` parity bump and clip-to-`[0, 2047]` range (including
    a negative-sum case that clips to 0); all-zero-residual /
    no-neighbour invariant across all three modes (DC = 1025, AC = 0);
    observational identity of `Neighbour::None` regardless of why it
    is unavailable; `is_available` accessor; Mode 1 / Mode 2 zero-
    residual predictor-passthrough; cross-mode invariant that every AC
    output respects `[AIC_AC_REC_MIN, AIC_AC_REC_MAX]` and every DC
    output respects `[AIC_DC_REC_MIN, AIC_DC_REC_MAX]`; fallback-DC
    predictor consistency across modes; and `AIC_FALLBACK_DC_PREDICTOR
    == 1024` constant guard.

- Annex I §I.3 absorbed-INTRADC INTRA-block parser
  (round 18, new `block_aic` module):
  - `parse_intra_block_aic(reader, has_coefficients) -> Result<H263Block>`
    — wires the round-14 Table I.2 event decoder into a full INTRA-block
    parser using the §I.3 (lines 4213-4217) absorbed-INTRADC
    semantics: the §5.4.1 8-bit FLC INTRADC prefix is gone, and the
    per-block decode is purely a sequence of Table I.2
    `(LAST, RUN, LEVEL)` events starting at scan position 0. The DC
    slot is just slot 0 of the coefficient buffer and is filled by
    whichever event's cumulative-RUN lands on it (or stays zero when
    no event does — the §I.3 "a zero INTRADC will not be coded as a
    LEVEL, but will simply increase the run for the following AC
    coefficients" semantics).
  - The `has_coefficients` boolean is the relevant CBP bit (CBPY for
    luma 0..=3, CBPC for chroma 4 / 5) per the §I.3 redefinition: in
    AIC mode the CBP bit being 0 is the sole signal that the DC is
    also zero, since INTRADC is no longer special-cased. The returned
    `H263Block.had_intradc` is always `false` regardless of whether
    slot 0 carries a non-zero LEVEL after parsing — no FLC was
    consumed.
  - Composes with the round-14 `intra_tcoef::decode_intra_tcoef_event`
    (event-level VLC), the round-17 `aic_dequant_coefficient` /
    `clip_ac` / `oddify_clip_dc` (modified inverse-quant + clipping),
    the round-8 `aic::scan_for_intra_mode` (per-INTRA_MODE scan
    selection), and the deferred DC/AC prediction reconstruction step
    (needs the macroblock-grid driver's neighbour blocks) to cover the
    full §I.3 INTRA-block decode pipeline.
  - 15 new unit tests cover: no-coefficients path returns an empty
    block without consuming bits; single LAST=1 RUN=0 event places its
    LEVEL at the DC slot (the §I.3 absorbed-INTRADC); the §I.3
    zero-DC-via-RUN invariant for RUN ∈ {1, 3, 7}; a DC-bearing event
    followed by an AC event lands LEVELs at slots 0 and 3; events at
    boundary slot 63 (terminating well-formed, non-terminating
    overflow); cumulative scan-position overflow when two events sum
    past slot 63; truncated-input → UnexpectedEof; forbidden ESCAPE
    LEVEL `0x00` and `0x80` reject with BadTcoefEscapeLevel while
    `0x81` (-127) and `0x7F` (+127) decode correctly; `had_intradc`
    stays `false` even with a non-zero DC; and an 8-event
    distribution-integration test placing LEVELs at slots
    0/2/7/18/19/40/46/63 simultaneously.
  - `intra_tcoef` module doc updated to point at the new `block_aic`
    module as the round-18 fulfilment of its "wiring into a full
    INTRA-block decoder is the next round's job" promise.

- Annex I §I.3 modified inverse-quantization primitives
  (round 17, new `aic_dequant` module):
  - `aic_dequant_coefficient(level: i16, quant: u8) -> i32` — the §I.3
    "no dead-zone" residual formula `RecC(u,v) = 2 · QUANT · LEVEL(u,v)`,
    a pure linear-in-both-inputs function applied identically to every
    coefficient slot (DC and AC alike) before the §I.3 prediction
    contribution is added. Strictly even-valued by construction (the
    `2 ·` factor), contrasting with the round-1 §6.2.1 H.261-style
    odd-fier baseline (`|REC| = QUANT · (2|LEVEL|+1) [-1 for even Q]`).
  - `clip_ac(x: i32) -> i32` — the §I.3 `clipAC` range pin to
    `[-2048, +2047]` (constants `AIC_AC_REC_MIN` / `AIC_AC_REC_MAX`),
    applied per-AC-slot after the prediction-residual sum.
  - `oddify_clip_dc(x: i32) -> i32` — the §I.3 `oddifyclipDC(x)` step
    applied to the DC slot post-prediction-sum: `if x is even then
    clipDC(x + 1) else clipDC(x)`, with `clipDC` pinning the result to
    the non-negative range `[0, +2047]` (constants `AIC_DC_REC_MIN` /
    `AIC_DC_REC_MAX`). The +1 bump protects against the IDCT-mismatch
    resonance the spec calls out at the (0,0) / (0,4) / (4,0) / (4,4)
    basis-pattern cross-points (`8k + 4` DC values inverse-transform
    to a constant `k + 0.5` that rounds inconsistently between
    conforming IDCTs).
  - These primitives compose with the round-14 Table I.2 separate
    INTRA-coefficient VLC (`intra_tcoef::decode_intra_tcoef_event`) to
    cover the §I.3 coefficient pipeline from parsed `(RUN, LEVEL)` event
    to a reconstructed pre-prediction residual; the only remaining §I.3
    decode-time gap is the DC/AC prediction reconstruction itself (the
    three INTRA_MODE-dependent rules that add `RecA'(u,v)` / `RecB'(u,v)`
    contributions before the final `clip_ac` / `oddify_clip_dc` step),
    which needs the macroblock-grid driver's live neighbour blocks.
  - 19 new unit tests covering the §I.3 residual formula
    (simple / negative-LEVEL / zero-LEVEL invariant / strict
    even-valued output across 31×255 QUANT×LEVEL pairs /
    linearity-in-LEVEL / linearity-in-QUANT / max-magnitude `±7874`
    extreme / AIC-residual-strictly-smaller-than-§6.2.1-baseline
    invariant / QUANT clamp); `clip_ac` (identity inside range /
    upper saturation / lower saturation); `oddify_clip_dc` (odd inputs
    unchanged / even inputs bumped / upper saturation via post-bump
    clip / lower saturation via post-bump clip / in-range
    oddness-or-boundary invariant across -100..=3000 / full
    -3000..=3000 spec-pseudocode-equivalence cross-check); and a
    `clip_dc` basic round-trip.
  - `aic` module doc updated to point at the new `aic_dequant` module
    for the §I.3 residual formula and clipping helpers and to record
    that only the prediction-reconstruction step remains deferred.

- Annex F §F.2 / §F.3 INTER4V four-motion-vector + Overlapped Block
  Motion Compensation driver wiring (round 16) in the `picture`
  module. The full-picture decode driver `decode_picture` now
  reconstructs INTER4V / INTER4V+Q macroblocks end-to-end whenever the
  picture header's Advanced Prediction flag is set:
  - The per-macroblock grid carries a full `[MotionVector; 4]` per MB
    (one per 8×8 luminance block, in `LumaBlockIndex` / Figure-5
    order); single-MV INTER and skipped / INTRA macroblocks
    replicate / zero the same vector across all four slots per the
    §F.2 last paragraph ("one-vector macroblocks are defined as four
    vectors with the same value").
  - `decode_inter4v_macroblock` reconstructs each of the four luma
    MVs by feeding `select_4mv_candidates` + `predict_mv_median` with
    the live `Mb4MvNeighbourhood` built from the grid, applying the
    §6.1.1 rule-3 "above unavailable → MV2 = MV3 = MV1" rewrite and
    the rule-4 "right-edge → MV3 = 0" rewrite per block, plus the
    Annex D §D.2 UMV extended reconstruction when the picture header
    enables it.
  - The Annex F §F.3 OBMC weighted average is dispatched per luma
    block via `obmc_predict_block`, with the four remote MVs
    classified into `RemoteMv` tags per the §F.3 substitution rules:
    not-coded neighbour → `Zero`; INTRA / off-picture neighbour →
    `Current`; otherwise the surrounding 8×8 block's coded MV. The
    §F.3 last-sentence "bottom-of-MB → current" rule unconditionally
    forces the bottom remote to `Current` for B3 / B4.
  - The chroma vector is derived via `chroma_mv_4mv` (sum of the four
    luma vectors / 8 with the Table F.1 sixteenth → half snap); both
    chroma blocks use standard half-pel motion compensation (no
    chroma OBMC per §F.2). The §6.3.1 residual summation +
    §6.3.2 clip then composes per-block via
    `reconstruct_inter_block_with_prediction`, gated on the §5.3.5
    INTER orientation of CBPY (`cbpy ^ 0b1111`).
  - 11 new tests covering: end-to-end INTER4V zero-MV reproducing the
    reference verbatim (the OBMC `q = r = s = ref(x,y)` identity),
    INTER4V-vs-single-MV exact byte equivalence on the all-zero-MV
    case, INTER4V with a flat-grey reference reproducing flat grey
    (Annex F invariant on the §F.3 weighted average with H0+H1+H2=8),
    INTER4V refusal without Advanced Prediction (defensive guard for
    PLUSPTYPE Deblocking-Filter mode), INTER4V after an INTRA left
    neighbour (substitution-rule path), and direct unit tests for
    `classify_remote_mvs` (B1 at top-left corner →
    top/left = `Current`; B3 bottom remote always `Current`;
    not-coded neighbour → `Zero`; INTRA neighbour → `Current`) and
    `build_4mv_neighbourhood` (INTRA neighbour → `None`; coded
    neighbour → `Some([...])`).
  - Single-MV INTER, skipped, and INTRA macroblocks are unaffected
    (the existing driver tests continue to pass); the per-MB
    `Mb4Mv` slot is populated with `[mv; 4]` for single-MV INTER and
    `[zero; 4]` for INTRA / skipped, so the §F.2 last-paragraph
    "single-vector = four equal vectors" rule continues to hold for
    INTER4V macroblocks reading from single-MV neighbours.

- Annex K Slice Structured mode slice-layer header parse (round 15),
  in the new `slice_header` module:
  - `parse_slice_layer(reader, &SliceHeaderContext) -> Result<SliceLayer>`
    decodes the §K.2 / Figure K.1 syntax for slices other than the
    first in a picture: SSC (17 bits) + SEPB1 + optional SSBI (4 bits,
    Table K.1 codewords) + MBA (variable per Table K.2) + optional
    SEPB2 + SQUANT (5 bits) + optional SWI (variable per Table K.3) +
    SEPB3 + GFID (2 bits).
  - `parse_first_slice_header(reader, &SliceHeaderContext) ->
    Result<FirstSliceLayer>` decodes the §K.2 reduced form for the
    slice that immediately follows the picture start code: SEPB1 +
    MBA + optional SEPB2 + optional SWI + SEPB3 (SSC, SSBI, SQUANT,
    GFID are absent in this case).
  - `SliceHeaderContext { picture_width, picture_height, cpm,
    rectangular_slices, rru }` carries the picture-level inputs the
    parser needs: CPM gates SSBI / shrinks the SEPB2 MBA-width
    threshold; `rectangular_slices` (PLUSPTYPE SSS bit 1) gates SWI;
    RRU (Annex Q) selects the right-hand columns of Tables K.2 / K.3.
    `SliceHeaderContext::for_standard_format(H263SourceFormat)` is a
    convenience constructor for the common QCIF / CIF / sub-QCIF
    baseline-plus-Annex-K case.
  - `SliceLayer { ssbi, mba, squant, swi_actual_width, gfid,
    header_bits }` / `FirstSliceLayer { mba, swi_actual_width,
    header_bits }` carry the decoded fields. `swi_actual_width` is
    `SWI + 1` per §K.2.8.
  - `ssbi_to_subbitstream(raw) -> Option<u8>` maps the four legal
    Table K.1 codewords (`1001` / `1010` / `1011` / `1101`) to the
    sub-bitstream numbers `0..=3`; all other 4-bit values return
    `None`.
  - Six new `Error` variants: `BadSliceStartCode`,
    `BadSliceEmulationPreventionBit`, `BadSliceSsbiCode`,
    `SliceMbaOutOfRange`, `SliceSwiOutOfRange`,
    `UnsupportedPictureGeometry`.
  - Out of scope (deferred): wiring the slice header into the
    `decode_picture` driver (still walks GOB headers only); §K.2.1
    SSTUF byte-aligner stuffing (the caller skips it before invoking
    the parser, identical contract to the GOB parser for GSTUF);
    end-of-sequence markers (§5.1.27).
- Public re-exports from the crate root: `parse_first_slice_header`,
  `parse_slice_layer`, `ssbi_to_subbitstream`, `FirstSliceLayer`,
  `SliceHeaderContext`, `SliceLayer`, `SEPB_BITS`, `SQUANT_BITS`,
  `SSBI_BITS`, `SSC_BITS`, `SSC_VALUE`.
- 30 unit tests in `slice_header::tests` covering
  `SliceHeaderContext` geometry (Table K.2 MBA field widths for
  sub-QCIF / QCIF / 16CIF, Table K.3 SWI widths for QCIF / CIF, RRU
  column for QCIF), SEPB2-presence across CPM and picture-size
  combinations, minimal-QCIF non-first parse, max-legal MBA, MBA
  overflow rejection, CPM-on parse with Table K.1 SSBI (and the
  `ssbi_to_subbitstream` mapping for all four codewords plus every
  non-codeword), illegal-SSBI rejection, RS-submode parse with SWI,
  SWI-wider-than-picture rejection, 16CIF parse with mandatory SEPB2,
  bad-SEPB1 / bad-SEPB3 / SQUANT=0 / bad-SSC / short-buffer
  rejections, §K.2 first-slice reduced-form parse (minimal,
  with-SWI under RS, MBA overflow, bad-SEPB3), reader-position-after-
  parse advance, and the SSC-equals-GBSC numerical identity.

- Annex I §I.3 / Table I.2 separate INTRA-coefficient VLC (round 14),
  in the new `intra_tcoef` module:
  - `decode_intra_tcoef_event(reader) -> Result<IntraTcoefEvent>`
    decodes one Table-I.2 `(LAST, RUN, LEVEL)` event from a
    `BitReader`. The 102 regular codewords reuse Table 16's bit
    patterns at every index (per §I.3, line 4033 of the spec text)
    but reassign the `(RUN, |LEVEL|)` columns; `LAST` is preserved
    between the two tables at each index (indices 0..=57 are
    `LAST=0`, 58..=101 are `LAST=1`). The 7-bit ESCAPE prefix and
    its 1 + 6 + 8 fixed-length tail are decoded identically to
    §5.4.2, with the baseline forbidden LEVEL codes (`0x00` / `0x80`)
    applied — Annex T's EXTENDED-ESCAPE relaxation is out of scope.
  - `IntraTcoefEvent { last, run, level }` is the decoded triple
    (sign already folded into `level`; ESCAPE `LEVEL` interpreted
    as `i8` two's complement).
  - `INTRA_TCOEF_REGULAR_ENTRIES = 102` exposes the regular-entry
    count for callers that want to cross-check against Table 16.
  - Out of scope (deferred): driving a full INTRA-block parser
    around this primitive (the §I.3 modified inverse quantization
    `RecC = 2·QUANT·LEVEL` with variable-step INTRADC, the §I.3
    DC/AC prediction reconstruction with `oddifyclipDC` / `clipAC`,
    and the §I.3 line-4214 "INTRADC absorbed into the coefficient
    stream" reframing of MCBPC / CBPY all need the macroblock-grid
    driver to supply the neighbour blocks).
- Public re-exports from the crate root: `decode_intra_tcoef_event`,
  `IntraTcoefEvent`, `INTRA_TCOEF_REGULAR_ENTRIES`.
- 19 unit tests in `intra_tcoef::tests`:
  - Table-shape invariants (102 regular + 1 ESCAPE; all 102
    `(LAST, RUN, |LEVEL|)` tuples pairwise distinct; LAST column
    matches the spec's index-58 boundary that Table 16 also
    observes — proves we have not transposed any code/bits pair
    relative to Table 16).
  - Full 102-entry round-trip across both sign polarities (encode
    each row's `(bits - 1)` prefix + sign bit, decode, verify
    `(last, run, level)` matches).
  - Spec spot-checks at indices 0, 1, 12, 22, 28, 58, 101 — chosen
    to exercise both ends of the table, two LAST-equality boundaries
    (idx 0 / 58 / 101), and indices where the I.2 interpretation
    *diverges* from Table 16 (idx 1: RUN=1/|L|=1 vs Table 16's
    RUN=0/|L|=2; idx 22: |L|=5 vs Table 16's RUN=3/|L|=1).
  - ESCAPE positive-LEVEL round-trip (LAST=1, RUN=7, LEVEL=+50),
    negative-LEVEL via two's complement (LEVEL=-2 from `0xFE`),
    and both baseline-forbidden LEVEL codes (`0x00` / `0x80`).
  - Reader-failure paths: 13 zero bits → `BadTcoefCode`;
    empty buffer → `UnexpectedEof`; index-0 `10s` consumes exactly
    3 bits; ESCAPE consumes exactly 22 bits.
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
