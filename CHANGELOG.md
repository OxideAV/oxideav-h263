# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Annex E (SAC) + Annex J (deblocking) **interaction** (round 16). Per
  §E.7 the MCBPC selector flips from `cumf_MCBPC_no4MVQ` to
  `cumf_MCBPC_4MVQ` whenever Annex F (4MV/OBMC) OR Annex J (deblocking
  filter) is active. The SAC P-picture encoder + decoder now plumb the
  Annex-J flag through (out-of-band on baseline PTYPE — there is no DF
  bit on the wire) and pick the 4MVQ model when DF is on. Encoder /
  decoder must agree via `set_enable_annex_j` on both sides; the
  out-of-band gate matches `H263Decoder::maybe_deblock`'s existing
  semantics. Validation in `tests/sac_annex_j_roundtrip.rs`: SAC+J
  roundtrip stays in sync, SAC+J ↔ VLC+J produce byte-identical decoded
  YUV (entropy stage only differs).
- Annex E (SAC) + Annex F (AP) **per-GOB resync** (round 16 / Part B).
  `encode_p_picture_sac_ap_with_recon_opts` now accepts an
  `emit_gob_headers` knob; when set, the SAC AP encoder calls
  `encoder_flush` (§E.7) at every GOB row boundary, drains a byte-aligned
  PSC_FIFO segment, writes the GOB header through the VLC channel, and
  opens a fresh `SacPPictureWriter` for the next segment. The decoder
  side (`decode_p_picture_sac_ap`) mirrors with a fresh
  `SacPPictureReader` per segment (§E.3 `decoder_reset`). The MV
  predictor and §F.3 OBMC reconstruction stay full-picture — §F.3 allows
  AP-mode predictors to reach across segments outside Slice Structured /
  ISD. Test coverage: `sac_ap_picture_with_gob_resync_cif` (CIF source,
  17 internal GOB boundaries per P-picture).
- Annex E (Syntax-based Arithmetic Coding) **I-picture MB-layer wiring**:
  the §E.7 VLC→SAC swap is now driven end-to-end on I-pictures. Encoder
  opt-in via `H263Encoder::set_enable_annex_e` sets PTYPE bit 11 and emits
  the I-picture body as a single SAC segment per picture
  (no in-body GOB headers — a follow-up will add `encoder_flush` /
  `decoder_reset` boundaries for sparse GOB resync). The decoder picks
  the SAC body driver automatically when the picture-header SAC bit is
  set. Every §E.8 model used by I-MB syntax is wired:
  cumf_MCBPC_intra, cumf_CBPY_intra, cumf_DQUANT, cumf_INTRADC,
  cumf_TCOEF1/2/3/r_intra (with cumf_SIGN), and the cumf_LAST_intra /
  cumf_RUN_intra / cumf_LEVEL_intra escape body. P-picture SAC is still
  rejected with a specific diagnostic — wiring cumf_COD + cumf_MCBPC +
  cumf_MVD into the COD / MV decode paths is the next round.
- `mb_sac` module hosting the I-picture SAC bridge:
  `SacIPictureWriter` / `SacIPictureReader` thin wrappers over the §E.2
  arithmetic coder + §E.5 PSC_FIFO, plus `decode_i_picture_sac` and
  `encode_i_picture_sac_body` MB-loop drivers. Self-roundtrip integration
  test (`tests/sac_iframe_roundtrip.rs`) confirms SAC-encoded → SAC-decoded
  output is byte-identical to the corresponding VLC pipeline.
- ffmpeg interop probe (`tests/sac_ffmpeg_interop.rs`, `#[ignore]`d):
  ffmpeg 8.1 explicitly rejects our SAC-encoded I-pictures with `H.263
  SAC not supported` — confirms our PSC + PTYPE header is well-formed
  (ffmpeg parses it correctly and identifies the SAC bit) but ffmpeg's
  baseline H.263 decoder never implemented the SAC body.

## [0.0.6](https://github.com/OxideAV/oxideav-h263/compare/v0.0.5...v0.0.6) - 2026-04-25

### Other

- drop oxideav-codec/oxideav-container shims, import from oxideav-core

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
