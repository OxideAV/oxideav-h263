# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

### Not yet wired (after round 5)

- Macroblock-loop assembly: round 5 reconstructs *one* block given
  the caller's QUANT; the per-MB driver that walks all six blocks
  (and applies CBPY/CBPC bits + per-block context per-block) is not
  yet wired.
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
