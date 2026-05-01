# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Annex G (PB-frames) — decoder + encoder (round 14).**
  Both sides now wire ITU-T Rec. H.263 (01/2005) §G.1 / §G.4 / §G.5
  for the legacy PB-frames mode (the H.263 baseline B-picture flavour
  that pairs each transmitted P with one bidirectionally-predicted B
  reconstructed from §G.4-scaled forward + backward MVs).
  * `picture::parse_picture_header` accepts PTYPE bit 13 (PBFR), reads
    the §5.1.22 / §5.1.23 TRB (3 bits) + DBQUANT (2 bits) tail after
    CPM/PSBI, and surfaces the parsed values on `PictureHeader.pb_frames`
    / `trb` / `dbquant`. The header-tail layout matches the standard
    (CIF) picture-clock-frequency variant (custom-PCF would extend TRB
    to 5 bits, which baseline PTYPE doesn't carry).
  * New [`pb`] module hosting the §G.4 / §G.5 math: `derive_b_block_mvs`
    (forward `MVF` + backward `MVB` from the P-MB's MV plus an optional
    `MVDB` delta, with the §G.4 sign-of-MVDB switch between
    `MVB = ((TRB-TRD)*MV)/TRD` and `MVB = MVF - MV`), `derive_b_mb_mvs`
    (per-MB four-luma + chroma derivation with §F.2 sum-of-4 + Table F.1
    rounding), `reconstruct_b_block` (§G.5 bidirectional MC with the
    spec's per-pixel "is `MVB` inside the freshly-reconstructed P-MB
    region?" test), `reconstruct_b_picture` (per-MB driver), and the
    `MODB` Table 11 VLC (encode + decode). The Table 6 BQUANT mapping
    (§5.1.23) lives here too as `bquant_from_quant`.
  * `mb::decode_p_mb_pb` reads the per-MB syntax in spec order (§5.3
    Figure 10): COD → MCBPC → **MODB** → optional CBPB (6 bits) →
    CBPY → DQUANT → MVD → MVDB. CBPB-driven B-residual bytes are
    parsed-and-discarded for round-14 scope (the round-trip with our
    own encoder always emits MODB = 0); MVDB uses a Table 14 pure
    differential (new `motion::decode_mvd_pure_differential` /
    `encode_mvd_pure_differential` helpers).
  * `decoder::decode_pb_picture` runs the per-MB PB decode then calls
    [`pb::reconstruct_b_picture`] with TRB from the picture header and
    `TRD = TRB + 1` (round-14 default). The decoder front-end emits two
    `VideoFrame`s per PB packet: B first, then P (display order). Only
    the P-half is stored as the MC reference for subsequent pictures
    (per §G.1 / §5.1.22).
  * `H263Encoder::set_enable_annex_g_pb` opts in to PB emission. The
    new `write_picture_header_pb` writer + `encode_pb_picture_with_recon`
    + `encode_p_mb_pb` + `encode_p_mb_pb_intra` / `encode_p_mb_pb_inter`
    write the picture header with the PB bit + TRB/DBQUANT and emit
    the per-MB stream with MODB at the spec-correct position. TRB /
    DBQUANT are tunable via `set_pb_trb` / `set_pb_dbquant` (defaults
    1 / `00`). Combinations with Annex D (UMV), Annex E (SAC), Annex F
    (Advanced Prediction), or Annex N (RPS) are rejected at
    `send_frame` for round 14.
  * Validation in `tests/annex_g_pb_frames.rs` (6 new tests + 1 ignored
    ffmpeg interop probe): PTYPE-bit + TRB/DBQUANT round-trip, 5-frame
    `[I, PB, PB, PB, PB]` self-roundtrip with PSNR ≥ 30 dB on the
    I-frame and every P-half (lands at **68.1 dB** for both the I and
    every P-half on the moving-square QCIF clip; B-halves at **28.8 dB**
    vs the midpoint-position source proxy, well above the ≥ 18 dB floor
    the bidirectional MC sets without B-residual), MODB Table 11 VLC
    bit-pattern check, MODB = 0 wire round-trip, B-picture dimension
    sanity, combination-guard rejection, ffmpeg cross-decode probe
    (informational — exit 0 with no error logs on ffmpeg 8.1, no
    interop assertion). 5 unit tests in `pb::tests` exercise the §G.4
    derivation at the midpoint + with MVDB delta, MODB round-trip,
    Table 6 BQUANT mapping, and Table F.1 chroma rounding.
- **Annex N (Reference Picture Selection) — decoder + encoder (round 13).**
  Both sides now wire ITU-T Rec. H.263 (01/2005) §5.1.13–§5.1.16 / Annex N
  for the picture-header path:
  * `picture::parse_picture_header` accepts PLUSPTYPE OPPTYPE bit 11
    (RPS) instead of returning `Unsupported`. The new fields RPSMF
    (3 bits, only when UFEP=001), TRPI (1 bit, mandatory when RPS in
    use), TRP (10 bits, only when TRPI=1), and BCI ("1" + BCM follows
    or "01" for no BCM) surface on the parsed `PictureHeader` as
    `rps_mode` / `rpsmf` / `trpi` / `trp` / `bci_present`. BCI = "1"
    is rejected with a specific `Unsupported` diagnostic citing
    §N.4.2 (the BCM body parse — BT/URF/TR/ELNUMI/ELNUM/BCPM/BSBI/
    BEPB1/2/GN/MBA/RTR/BSTUF — is out of round-13 scope).
  * `H263Decoder` gained an LRU picture-memory cache keyed by TR
    (default capacity 4, tunable via `set_rps_cache_capacity`).
    Every successfully decoded picture is pushed into the cache, and
    when a parsed P-picture has `rps_mode && trpi`, the decoder looks
    up `trp` in the cache and uses that picture as the
    motion-compensation reference. Cache misses degrade gracefully to
    "most recent anchor" (matches §N.5's fall-back). `IPicture` gained
    `#[derive(Clone)]` so the cache can hold owning copies.
  * `H263Encoder::set_enable_annex_n_rps` opts in to RPS emission.
    The new `write_plusptype_picture_header_rps` writer emits a
    PLUSPTYPE-form picture header (source-format `111`, UFEP=001,
    full OPPTYPE with RPS bit) with RPSMF=`100` (NEITHER — no
    back-channel signals), TRPI=0 (decoder uses most recent anchor),
    BCI=`01` (no BCM). The MB layer underneath is unchanged baseline
    1-MV inter — bit-identical to the non-RPS encoder for the same
    DCT/quant/MV pipeline. Combinations with Annex D / E / F return
    `Error::Unsupported` at `send_frame`.
  * Validation in `tests/annex_n_rps.rs` (7 new tests):
    PLUSPTYPE wire format check (PSC + TR + PTYPE prefix + UFEP + OPPTYPE
    bit 11 = 1), header parse round-trip of all RPS fields, self-roundtrip
    PSNR ≥ 30 dB on the moving-square QCIF clip, hand-rolled TRPI=1+TRP
    rewrite test (multi-reference cache lookup), combination-guard
    rejection for UMV/SAC/AP, ffmpeg cross-decode probe (ffmpeg 8.1
    logs "Reference Picture Selection not supported" and concealment
    decodes — frame 0 / I-picture lands at **51 dB**, P-pictures get
    concealed by ffmpeg's best-effort path), testsrc 5-frame QCIF
    PSNR (self ≥ 30 dB across the clip; ffmpeg I-picture floor 30 dB).
- **Annex D (Unrestricted Motion Vectors) — encoder emit (round 12).**
  `H263Encoder::set_enable_annex_d_umv(true)` activates the UMV path: every
  P-picture sets PTYPE bit 10 (UMV) in the header (`write_picture_header_full`
  is the new full-options writer), the motion estimator widens to
  `[-63, +63]` halfpel via `motion_estimate_mb_umv` (§D.1 out-of-picture
  references rely on `interp::predict_block`'s edge clamp; no stay-in-picture
  constraint), and MV components are emitted through the new
  `motion::encode_mv_component_umv` which selects the `(magnitude, sign)`
  pair whose §D.2 reconstruction yields the desired vector — fast-path for
  predictors in the baseline `[-31, +32]` band emits bytes byte-identical
  to the non-UMV encoder, general path enumerates and tie-breaks toward the
  non-wrapped candidate. Combinations with Annex E (SAC) or Annex F
  (Advanced Prediction) are rejected at `send_frame` for now (round 12 scope
  is the baseline 1-MV inter path). Validation in
  `tests/annex_d_umv_encoder.rs`: PTYPE-bit assertion, self round-trip
  PSNR ≥ 30 dB on a moving-square sequence, ffmpeg cross-decode parity
  check, plus the testsrc-style 5-frame QCIF clip both self-decoded and
  ffmpeg-cross-decoded at **51 dB**. Three unit tests in `motion::tests`
  exercise the encoder helper across the extended range and pin the
  baseline-equivalence guarantee.
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
