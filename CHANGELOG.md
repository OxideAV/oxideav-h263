# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
- GOB layer (§5.2), slice-structured mode (Annex K).
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
