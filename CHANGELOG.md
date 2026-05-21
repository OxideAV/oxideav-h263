# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

### Not yet wired (after round 3)

- Block-data decode (§5.4 / Annex H VLCs) — round 3 stops
  at the macroblock header.
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
