# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.5](https://github.com/OxideAV/oxideav-h263/compare/v0.0.4...v0.0.5) - 2026-04-24

### Other

- Annex F — 4MV + OBMC encoder emission
- Annex D PLUSPTYPE — Table D.3 MVD VLC + UUI range + Annex K detect
- Annex E core — SAC arithmetic coder + every §E.8 cumul-freq model
- Annex F — Advanced Prediction mode (4MV + OBMC decode)
- Annex D — Unrestricted Motion Vectors (baseline PTYPE decode)

### Added

- Annex F (Advanced Prediction — 4MV + OBMC) **encoder emission**, opt-in
  via `H263Encoder::set_enable_annex_f`. When on, every P-picture header
  sets PTYPE bit 12 and the encoder runs a two-pass per-MB loop: pass 1
  compares the single-MV SAD against the four-block SAD sum and picks the
  one that wins by a material margin, also falling back to `skipped` /
  `intra-in-P` where cheaper; pass 2 computes the §F.3 OBMC-blended
  predictor against the full `MvGrid` and emits `Inter4MV` / `Inter` MCBPC
  + CBPY + up-to-4 MVDs + per-block residual TCOEF, with the chroma MV
  derived from the §F.2 Table F.1 sum-of-4 rule for 4MV MBs. The cached
  reference is produced by running the decoder's
  `apply_p_mb_reconstruction(advanced_prediction=true)` over the encoded
  state, so encoder ↔ decoder reconstruction stays bit-identical.
- Annex D (Unrestricted Motion Vector mode) decode path for baseline-PTYPE
  streams: PTYPE bit 10 (UMV) is now accepted; MV differentials are
  reconstructed via the §D.2 sign-of-predictor rule with the extended
  `[-31.5, +31.5]` pel range; picture-edge extrapolation (§D.1) replicates
  the nearest edge sample for out-of-picture references via the existing
  `interp::predict_block` clamp.

### Fixed

- PLUSPTYPE OPPTYPE bit layout corrected per §5.1.4.2: source format is
  now read from OPPTYPE bits 1-3 (not synthesised from `custom_src`), the
  trailing reserved-000 bits are validated, and the marker check pinpoints
  the correct bit. Streams with standard-format OPPTYPE + DF now parse
  without requiring a (non-existent) CPFMT block.
- PLUSPTYPE header now correctly reads and skips the variable-length UUI
  field when OPPTYPE signalled UMV (prior path never read the bits and
  would desync on the next field).

## [0.0.4](https://github.com/OxideAV/oxideav-h263/compare/v0.0.3...v0.0.4) - 2026-04-19

### Other

- bump oxideav-mpeg4video
- bump oxideav-core / oxideav-codec dep examples to "0.1"
- migrate register() to CodecInfo builder
- bump oxideav-core + oxideav-codec deps to "0.1"
- claim AVI FourCCs via oxideav-codec CodecTag registry
- migrate to oxideav_core::bits shared BitReader / BitWriter
- update mpeg1video reference to mpeg12video in bitwriter comment
