# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Annex L (Supplemental Enhancement Information) — decoder.** New
  `crate::sei` module exposes [`Sei`](crate::sei::Sei) (one variant per
  defined `FTYPE` in Table L.1 plus `Unknown` for reserved values) and
  [`parse_psupp_stream`](crate::sei::parse_psupp_stream) which walks
  the concatenated PSUPP byte sequence per §L.2 (4-bit `FTYPE` + 4-bit
  `DSIZE` + `DSIZE` parameter bytes). The picture-header parser now
  collects PSUPP bytes from the §5.1.24 / §5.1.25 PEI loop on both
  baseline and PLUSPTYPE inputs, parses them via the new module, and
  surfaces the result on `PictureHeader::sei`. Records with reserved /
  unknown FTYPE are kept as `Sei::Unknown` blobs (per §L.2's
  forward-compatibility note: discard `DSIZE` bytes and continue).
  Action semantics (e.g. actually freezing the displayed picture for
  `Sei::FullPictureFreezeRequest`) are out of scope of this codec
  crate — they are downstream presentation concerns. New integration
  tests in `tests/annex_l_sei.rs` plus a dozen unit tests in
  `crate::sei`.

- **Annex T (Modified Quantization) — header recognition + helpers +
  I-picture body driver.** New `crate::mq` module exposes:
  * [`decode_dquant_mq`](crate::mq::decode_dquant_mq) — §T.2 variable-
    length DQUANT field. First bit `1` → 2-bit small-step alteration
    (Table T.1 indexed by prior QUANT); first bit `0` → 6-bit arbitrary
    new QUANT. Returns the new QUANT in `1..=31`.
  * [`quant_c_for_quant`](crate::mq::quant_c_for_quant) — §T.3 Table T.2
    luma → chroma quant mapping (a smaller step size for chroma improves
    fidelity).
  * [`unrotate_extended_level`](crate::mq::unrotate_extended_level) /
    [`rotate_extended_level`](crate::mq::rotate_extended_level) — §T.4
    11-bit cyclic rotation that recovers a signed `LEVEL` outside the
    standard `[-127, +127]` range from the on-the-wire EXTENDED-LEVEL
    field. The rotation prevents start-code emulation.
  * `crate::block::decode_ac_mq` — INTER/INTRA AC decode honouring §T.4
    EXTENDED-ESCAPE (`0000011 ? ?????? 1000_0000` followed by 11 bits
    of cyclically-rotated EXTENDED-LEVEL) + §T.5 restrictions
    (`|level| > 127` only when `quant < 8`; reconstruction magnitude
    clipped to 4096).
  * `crate::mb::decode_intra_mb_mq` — I-picture MB body driver using
    §T.2 DQUANT, §T.3 chroma quant, and the §T.4 EXTENDED-ESCAPE-
    aware AC decoder.
  The picture-header parser surfaces `PictureHeader::modified_quantization`
  from PLUSPTYPE OPPTYPE bit 14. The I-picture decoder dispatches to
  `decode_intra_mb_mq` automatically when the flag is set. P-pictures
  with MQ are rejected at `decode_one_picture` with a specific
  `Unsupported` diagnostic (the §T.2 / §T.3 / §T.4 plumbing through
  `decode_p_mb` / `decode_p_mb_pb` is round-26 work). New integration
  tests in `tests/annex_t_modified_quantization.rs` plus 6 unit tests
  in `crate::mq`.

- **Annex S (Alternative INTER VLC) — header recognition + helper.**
  New `crate::block::decode_ac_aiv` implements the §S.2 try-INTER-then-
  fallback-to-INTRA AC decoder: a snapshot of the bit reader is taken,
  the inter table is tried first, and on RUN-overflow the snapshot is
  restored and the same bits are re-parsed through Table I.2 (the AIC
  INTRA TCOEF VLC). The picture-header parser surfaces
  `PictureHeader::alternative_inter_vlc` from PLUSPTYPE OPPTYPE bit 13.
  Per-MB plumbing (routing `decode_p_mb`'s residual decode through
  `decode_ac_aiv`, plus the §S.3 CBPY swap when `CBPC5 = CBPC6 = 1`)
  is round-26 work; for now the decoder rejects an AIV-flagged picture
  with a specific `Unsupported` diagnostic. New integration tests in
  `tests/annex_s_aiv.rs`.

- **Annex R (Independent Segment Decoding) — header recognition +
  §R.3.1 RS-submode constraint check.** The picture-header parser
  surfaces `PictureHeader::independent_segment_decoding` from PLUSPTYPE
  OPPTYPE bit 12. The decoder enforces §R.3.1 (Annex R + Annex K
  requires Annex K's Rectangular Slice submode), surfacing a specific
  `Invalid` diagnostic when the constraint is violated. The §R.2
  segment-isolation behaviour for MV prediction reuses the existing
  GOB-boundary mask in `predict_mv_with_gob_mask`; the §R.2.4 out-of-
  segment MV extrapolation is round-26 work, so for now the decoder
  rejects ISD + (UMV / AP / Annex J) with a specific `Unsupported`
  diagnostic. New integration tests in `tests/annex_r_isd.rs`.

- **Picture-header field surface widened.** `PictureHeader` gained four
  new fields (`independent_segment_decoding`, `alternative_inter_vlc`,
  `modified_quantization`, `sei: Vec<crate::sei::Sei>`); existing
  callers that built `PictureHeader` literally (none in this crate)
  must add the fields. The PLUSPTYPE OPPTYPE bits 12 / 13 / 14 (ISD /
  AIV / MQ) that previously returned `Error::Unsupported` now parse
  cleanly and surface on these fields.

  Total tests: 215 → 248 (+33 new tests covering Annex L PSUPP parser
  (11 unit + 5 integration), Annex T helpers + I-picture body (6 unit +
  5 integration), Annex R/S header recognition + guards (3+3
  integration each)).


## [0.0.7](https://github.com/OxideAV/oxideav-h263/compare/v0.0.6...v0.0.7) - 2026-05-03

### Other

- replace never-match regex with semver_check = false
- migrate to centralized OxideAV/.github reusable workflows
- Annex K (Slice Structured mode) — encoder + decoder
- Annex M (Improved PB-frames) — encoder + decoder
- Annex I (Advanced INTRA Coding) — encoder + decoder
- oxideav-core ^0.2 -> ^0.1 (0.2.0 was yanked)
- implement receive_arena_frame() for true zero-copy
- port to oxideav-core 0.1.8 DoS framework (DecoderLimits + ArenaPool)
- Annex G (PB-frames) — encoder + decoder
- Annex N (Reference Picture Selection) — picture header + multi-ref cache
- Annex D (UMV) encoder emission
- cargo fmt cleanups
- adopt slim VideoFrame shape
- adopt slim VideoFrame/AudioFrame shape
- Annex E SAC + Annex J (deblocking) interaction; SAC+AP per-GOB resync
- Annex E SAC + Annex F (4MV/OBMC) — combined emit + decode
- Annex E SAC — P-picture MB-layer VLC→SAC swap
- Annex E SAC — I-picture MB-layer VLC→SAC swap
- pin release-plz to patch-only bumps

### Added

- **Annex K — Slice Structured mode (encoder + decoder).** Replaces the
  GOB layer with the slice layer per §K.2 (Figure K.1):
  `SSTUF | SSC(17) | SEPB1 | (SSBI if CPM) | MBA(N) | (SEPB2 if needed) |
  SQUANT(5) | (SWI if RS) | SEPB3 | GFID(2)`, each slice acting as a
  resync point for bit-error / packet-loss recovery. The encoder opts in
  via `H263Encoder::set_enable_annex_k_slice(true)` and accepts a slice
  size in macroblocks via `set_slice_mb_size(n)` (default 22). Pictures
  are emitted with a PLUSPTYPE block carrying OPPTYPE bit 10 (SS) = 1
  and a 2-bit SSS submode field (round-23 always emits `00` — neither
  Rectangular Slice (RS) nor Arbitrary Slice Ordering (ASO) is
  generated). The decoder auto-detects SS via the parsed picture header
  and switches its body driver to a slice-based MB walker that
  *try-parses* candidate slice boundaries (snapshotted bit reader +
  SSC + SEPB1=1 + SEPB3=1 validation), guarding against the false
  positives long zero runs in skipped P-MBs would otherwise produce.
  MV prediction is reset at every slice boundary per §K.1 rule 1
  (matching the existing GOB-boundary behaviour). New helpers in
  `crate::slice`: `SliceHeader` / `SssMode` / `parse_slice_header_body`
  / `write_slice_header` / `mba_field_width` (Table K.2) /
  `swi_field_width` (Table K.3); new encoder entry-points
  `encode_i_picture_slice_with_recon` / `encode_p_picture_slice_with_recon`.
  New integration tests in `tests/annex_k_slice.rs`. Annex K +
  UMV/SAC/AP/RPS/PB/AIC return `Error::Unsupported` at `send_frame`
  for the round-23 scope.

- **Annex M — Improved PB-frames mode (encoder + decoder).** Extends the
  existing Annex G PB-frames path with per-MB selection across three
  BPB-block prediction shapes per §M.2:
  * **Bidirectional** — same predictor as Annex G with MVD = 0
    (forward from prior P, backward from new P, averaged inside the §G.5
    region).
  * **Forward** — single 16×16 forward MV from MVDB, predictor = prior P
    at the destination position offset by MVDB. The MVDB predictor follows
    §M.2.2 ("the left MB's forward MV, or 0 if absent"), VLC-coded via the
    same Table 14 + sign + sign-of-predictor cascade as the §5.3.7 P-MVD.
  * **Backward** — predictor = freshly-reconstructed P-MB pels (§M.2.3
    PREC), no MV data on the wire.
  Opt-in via `H263Encoder::set_enable_annex_m_impb(true)` (requires
  `set_enable_annex_g_pb(true)`); the matching decoder must also opt in
  via `H263Decoder::set_enable_annex_m_impb(true)` to read the Table M.1
  MODB codes correctly. The encoder runs a per-MB Lagrangian RDO over
  `SAD + lambda * R` with `lambda = QP * 4`, where the rate proxy counts
  the mode-discriminating bits of MODB + MVDB + CBPB; the cheapest of
  {bidir, forward, backward} wins. New helpers in `crate::pb`:
  `encode_modb_m` / `decode_modb_m` (Table M.1 VLC), `predict_b_block_forward`
  (§M.2.2), `predict_b_block_backward` (§M.2.3), `reconstruct_b_picture_m`
  (per-MB B-mode dispatch). New helper in `crate::motion`:
  `mvd_pure_differential_bits` (rate proxy for the RDO loop).
  New integration tests in `tests/annex_m_improved_pb.rs`. On the bundled
  mixed-motion fixture the Annex M output is ~52 % smaller than the
  matching Annex G output at the same QP — well above the ~5–10 %
  acceptance criterion. Annex M is signalled out-of-band per §M.1
  (ITU-T Rec. H.245 in the spec); ffmpeg cross-decode is informational
  only since ffmpeg has no in-band signal to switch to Table M.1.

- **Annex I (Advanced INTRA Coding) — encoder + decoder (round 24).**
  Opt-in via `H263Encoder::set_enable_annex_i_aic(true)`; auto-detected
  on input via the parsed PLUSPTYPE OPPTYPE bit 8 (`PictureHeader::aic_mode`).
  When AIC is in use, every INTRA macroblock writes:
  * an `INTRA_MODE` codeword (Table I.1, 1-or-2 bits) between MCBPC and CBPY,
  * every coefficient (DC + AC) coded via Table I.2 (different
    `(LAST, RUN, |LEVEL|)` mapping than the standard inter TCOEF, identical
    codeword shapes), starting at scan position 0 in the per-MB scan order
    (zig-zag for mode 0, alternate-horizontal for mode 1, alternate-vertical
    for mode 2),
  * dequantisation drops the dead-zone (`RecC = 2 * QUANT * LEVEL` for every
    coefficient — no INTRADC special-case),
  * §I.3 AC prediction (DC-only / vertical / horizontal) folds in the
    spatial-neighbour predictor with the `oddifyclipDC` IDCT-mismatch
    mitigator on the DC slot.
  The new `crate::aic` module owns the table data + AC-pred logic; the
  decoder dispatches to `crate::mb::decode_intra_mb_aic` when `aic_mode`
  is set; the encoder emits `encode_i_picture_aic_with_recon` (with a
  PLUSPTYPE picture header carrying OPPTYPE bit 8 = 1). FFmpeg
  cross-decodes the AIC stream cleanly (verified via the
  `tests/aic_ffmpeg_interop.rs` `--ignored` probe). AIC currently
  affects I-pictures only; intra-in-P MBs still use the baseline INTRADC
  path. AIC + other PLUSPTYPE optional modes (UMV / SAC / AP / RPS / PB)
  is rejected at `send_frame` for now. Bitrate delta on intra-rich
  content (talking-head QCIF): ~21 % smaller than the baseline non-AIC
  encoder for the same PQUANT; flat-DC content shrinks ~5×.

- **`receive_arena_frame()` — zero-copy decode path.**
  Overrides the new `oxideav_core::Decoder::receive_arena_frame()`
  method (added in oxideav-core 0.2.0) to return an arena-backed
  `oxideav_core::arena::sync::Frame` directly, skipping the per-plane
  memcpy that the legacy `receive_frame() -> Frame::Video(VideoFrame)`
  path requires for `Send`. The arena `ArenaPool` is now a `sync`
  variant whose `Frame = Arc<FrameInner>` is itself `Send + Sync`
  — callers can move the returned frame across thread boundaries
  for parallel render / encode / network sinks.
- New `tests/arena_frame.rs` exercising the zero-copy contract:
  encode an I-picture, decode via `receive_arena_frame`, verify
  (a) plane bytes match the legacy `receive_frame` output, (b) the
  arena pool stays exhausted while the returned frame is held
  (proves the planes really live inside the arena), and (c) the
  pool slot returns when the frame's last `Arc` clone drops.
- New `pic_to_arena_frame` public helper — the arena-building
  counterpart to the existing `pic_to_video_frame` heap helper.

### Changed

- **Bumped `oxideav-core` dep from `0.1` to `0.2`** to pick up the
  new `Decoder::receive_arena_frame` trait method (additive; default
  impl preserves backwards compatibility for every other
  `oxideav-h263` consumer).
- Internal queueing reorganised: decoded pictures are now queued as
  raw `IPicture`s rather than pre-built `VideoFrame`s. `receive_frame`
  builds a heap-backed `VideoFrame` on demand; `receive_arena_frame`
  leases an arena and builds an arena `Frame` on demand. This keeps
  the pool short-lived (one slot held only between drain and the
  caller dropping the `Arc<FrameInner>`) so a `send_packet` call
  that decodes many pictures before the consumer drains no longer
  exhausts the pool.

- **DoS-protection framework port (`oxideav-core` 0.1.8).** Wires the
  decoder front-end into the new `DecoderLimits` + `ArenaPool` stack:
  * `H263Decoder::with_limits(codec_id, DecoderLimits)` constructor and
    `DecoderLimits`-aware `make_decoder` factory — server callers can
    now hand a tightened `CodecParameters::with_limits(...)` and the
    decoder honours the caps.
  * Picture-header dimension check: every parsed `PictureHeader` is
    validated against `limits.max_pixels_per_frame` **before** any
    `IPicture::new` allocation, returning the new
    `Error::ResourceExhausted` variant on a malicious oversize header.
  * Per-decoder `ArenaPool` sized at construction from
    `limits.max_arenas_in_flight × min(limits.max_alloc_bytes_per_frame,
    DEFAULT_H263_ARENA_BYTES)` — the new constant clamps the per-arena
    cap at 4 MiB (enough for 16CIF YUV420p, far smaller than the
    `oxideav-core` default of 1 GiB so 8 idle arenas use ~32 MiB
    instead of 8 GiB).
  * New `pic_to_video_frame_arena` helper — every emitted frame stages
    its YUV plane copy through a leased arena before memcpy'ing into
    the existing `Frame::Video(VideoFrame)` (heap-owned `Vec<u8>`
    planes). The `Decoder::receive_frame` API is **unchanged**, so
    `oxideav-pipeline` consumers that move frames across `Send`
    boundaries continue to work without modification — the arena's
    contribution is to bound peak per-frame scratch RSS and to surface
    pool exhaustion as natural `ResourceExhausted` backpressure.
  * `H263Decoder::limits()` + `arena_pool()` accessors for tooling.
  * New `tests/dos_limits.rs` with five fuzz fixtures covering the
    pixel-cap rejection (with + without trip), pool-exhaustion lease,
    per-arena byte cap, and the `make_decoder` factory plumbing.
  * Encoder is unchanged — DoS protection only applies to the decoder
    per the task spec.

  Total tests: 142 → 147 (+5 dos-limits fixtures).

- **Annex G (PB-frames) — B-block residual emission (round 15).**
  Round 14 wired the PB-frames framing (MODB / CBPB / MVDB / DBQUANT
  picture-header tail and per-MB syntax) but kept MODB = `0` per MB on
  the encoder side, so every B-half was a pure §G.5 bidirectional MC
  predictor with zero residual — landing at **28.8 dB** PSNR vs the
  midpoint-position source proxy on the moving-square QCIF clip.
  Round 15 wires the encoder + decoder paths to actually carry a
  per-block B-residual at BQUANT (§5.1.23):
  * Encoder — `pb::predict_b_block` extracted from `reconstruct_b_block`
    (signed-i16 prediction so the encoder can do `source - prediction`
    without saturating). `encode_p_mb_pb_inter` rebuilt with a 2-stage
    flow: (1) quantise + reconstruct the P-half blocks into `recon`
    *before* writing any bits, (2) compute the §G.5 prediction for each
    of the 6 B-blocks against the freshly-reconstructed P-MB, subtract
    from the **input frame** pels (the streaming 1-input-per-PB-pair
    model uses the input as the B-source), forward-DCT, quantise at
    `bquant_from_quant(quant, dbquant)`, and pick CBPB bits from the
    per-block any-nonzero check. Then emit MCBPC, MODB (`11` when CBPB
    is non-zero / `0` when not), CBPB (6 bits, MSB = block 1), CBPY,
    MVD, MVDB = `(0, 0)` (pure differential, two `encode_mvd_pure_diff
    erential` codewords), the six P-block TCOEF runs, and the per-CBPB
    B-block TCOEF runs — exactly the §5.4 / §5.3 Figure 10 order. The
    intra-in-PB path (`encode_p_mb_pb_intra`) also gained the §G.2 MVD
    write that the round-14 path was missing (latent bug — the
    moving-square test never trips intra-in-PB but a third-party clip
    with intra-in-P MBs would have desynced the decoder).
  * Decoder — `mb::decode_p_mb_pb` now takes `dbquant` and decodes the
    per-CBPB B-block residual: `decode_ac` at BQUANT, `idct_signed` to
    get residual pels, surfaced on a new `PbMbInfo.b_residual` field.
    `decoder::decode_pb_picture` collects per-MB residuals into the
    picture-wide buffer it already passes into `pb::reconstruct_b_pictur
    e`, replacing the previous all-zero placeholder.
  * Tests — `pb_self_roundtrip_psnr` floor lifted from ≥ 18 dB to
    ≥ 40 dB on the B-half (lands at **55.4-57.1 dB** with residual
    emission at PQUANT = 5 / DBQUANT = 0 / BQUANT = 6); new
    `pb_b_residual_emission_psnr_jumps_with_finer_bquant` test checks
    the BQUANT relationship (DBQUANT = 0 → average B PSNR 56.2 dB,
    DBQUANT = 3 → 51.6 dB; the finer BQUANT consistently produces the
    cleaner reconstruction).
  * I-frame and P-half PSNR stay at **68.1 dB** (unchanged from round
    14 — the P-half encoder code path is byte-identical to before).
  * ffmpeg interop — informational probe still exits 0 with no error
    logs on ffmpeg 8.1; we don't assert pixel-level cross-decode parity
    because ffmpeg's PB-frames decoder is partial.

  Pending r16+: MVDB selection on the encoder side (currently always
  `(0, 0)` — picking a non-zero MVDB per MB requires a search), intra-
  in-P MVD selection (currently fixed at `(0, 0)`), Annex M Improved
  PB-frames, and the §G.2 chroma MVDB sign in 4MV mode (we're 1MV
  only).
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
