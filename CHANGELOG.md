# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Annex M **Improved PB-frames encoder** (`encoder_pb` module,
  re-exported from `encoder`): `encode_improved_pb_picture` /
  `encode_improved_pb_picture_stats` + `ImprovedPbConfig` emit the
  PLUSPTYPE `"010"` picture unit with a per-macroblock rate-biased
  §M.2 mode decision (bidirectional / forward with the §M.2.2
  left-neighbour predictor and MVDB search / backward = PREC), the
  Table M.1 MODB writer (`pb_layer::write_modb_annex_m`,
  `ModbAnnexM::from_parts` / `code`) and BQUANT residuals.
  `PlusModes::improved_pb` carries the §5.1.22 TRB / §5.1.23 DBQUANT
  header fields through `write_plus_picture_header`.
- **Advanced Prediction + PB-frames**, both annexes, both directions.
  Decoder: the B-part of an AP macroblock is parsed in bitstream order
  and reconstructed only after the deferred §F.3 OBMC flush (PREC is the
  *final* P-macroblock, §G.5); the §G.2 rule makes an INTRA neighbour's
  B-purpose vector its OBMC remote (`RemoteMv::Vector` instead of
  `Current`); the §6.1.1 rule-1 PB-frames exception now reaches the
  §F.2 four-vector / block-1 predictors too; INTRA macroblocks in an
  AIC PB picture carry their §G.2 vector. `decode_pb_picture` /
  `decode_pb_picture_no_gob0_header` / the Improved-PB drivers accept
  AP pictures. Encoder: `encode_pb_picture_ap` (Annex G + AP) and
  `ImprovedPbConfig::advanced_prediction` (Annex M + AP) — two-pass
  §F.2 vector field, OBMC PREC, per-block §G.4 scaling, MVDB search
  range-checked over all four vectors — plus
  `ImprovedPbConfig::intra_refresh` (INTRA macroblocks inside PB
  pictures, carrying the §G.2 / §M.2.1 MVD; Annex M keeps them
  bidirectional so the vector is always on the wire).
- **Unrestricted Motion Vectors + PB-frames**, both annexes, both
  directions. Annex G + Annex D (baseline header): the §D.2 Table 14
  pair rule applies to MVDB with the predictor `Pc = (TRB × MV)/TRD`,
  resolved **per luminance block** (`pb_layer::pb_b_effective_deltas`
  / `pb_b_predict_macroblock_deltas`) — decoder and the new
  `encode_pb_picture_umv` share the resolution, so a 22 px pan codes
  in a ninth of the default-range bytes. Annex M + UMV (PLUSPTYPE):
  the P-part vectors and the §M.2.2 forward vector are Table D.3
  single-valued differences under the UUI range, the forward fetch
  reaching over the picture boundary (§D.1 / §M.2.2);
  `ImprovedPbConfig::umv` on the encode side, the Improved-PB decode
  driver no longer refuses UMV.
- **Annex K slices + Improved PB-frames**, both directions: the slice
  driver takes a PB request (TRB / DBQUANT read between PQUANT and
  PEI, the B-part deferred after the OBMC flush under AP, the §M.2.2
  forward predictor restarting at every slice's / rectangle's left
  edge), the Improved-PB driver routes Slice-Structured pictures to it,
  and `ImprovedPbConfig::slice_rows` emits row-aligned free-running
  slices (per-slice §K.1 rule-1 predictors and rule-3 OBMC remotes)
  from the PB encoder core. The oracle agrees exactly on every P-part.
- **Annex K Rectangular Slice / Arbitrary Slice Ordering + Advanced
  Prediction on the encode side**: `encode_inter_picture_ap_slices_rect`
  emits full-height vertical stripes (SWI on the wire, right-to-left
  under ASO) whose macroblocks carry four §F.2 vectors predicted
  through the §F.3 OBMC blend, with both §K.1 confinement rules
  replayed per stripe — `Mv4Grid::with_segments` carries an arbitrary
  segment map (rule 1: a neighbour in another stripe is outside the
  slice; rule 3 folds the above / above-right candidates into MV1
  independently, as the decoder does) and the stripe's left / right
  edges substitute the current vector for the OBMC remotes. A single
  full-width stripe reconstructs byte-identically to the free-running
  AP picture. The oracle decoder refuses Rectangular Slices outright
  ("not supported"), so the check is own-decoder only.
- **Annex I INTRA macroblocks inside P-pictures and Improved
  PB-frames** (encode side): `encode_inter_picture_aic_plus` codes the
  INTRA-refresh macroblocks of a motion-searched P-picture per §I
  (INTRA_MODE decision, §I.3 DC/AC prediction from the encoder's own
  reconstructed INTRA neighbours — an INTER or skipped neighbour is no
  §I.3 predictor, as the decoder treats it — and the Table I.2 VLC);
  `ImprovedPbConfig::aic` does the same inside the PB core (INTRA_MODE
  between MCBPC and MODB per Figure I.1, PREC = the §I reconstruction,
  §I.3 segment ids following the Annex K slices). An all-INTRA AIC
  P-picture reconstructs byte-identically to the AIC I-picture
  encoder's; the oracle agrees exactly on both.
- **Annex Q + Annex J (§Q.7.2)**, both directions: an RRU picture with
  the Deblocking Filter mode on (OPPTYPE bit 9, or `DecodeOptions::deblock`)
  runs the §J.3 four-tap filter with `STRENGTH = +∞` on the 16 × 16
  block edges (`RruFilterMode::Deblocking`) in place of the §Q.7.1
  two-tap default; the Table J.1 four-vectors element makes MVD2-4
  parseable there, an INTER4V macroblock (four pseudo vectors per
  32 × 32 macroblock) staying refused. `encode_intra_picture_rru_deblock`
  / `encode_inter_picture_rru_deblock` signal the mode (single-vector
  macroblocks, identical coded data).
- **Annex P explicit RPR on EP-pictures** (§P.2.2 paragraph 2):
  `annex_p::parse_rprp_ep_refinement` reads the EP-picture RPRP form
  (WDA + one bit per warping parameter of every up-sampled dimension,
  no fill mode) and `RprParams::refine_for_layer` derives the effective
  parameters (`w' = 2·w + bit`, the lower layer's fill mode);
  `decode_ep_picture_rpr` / `decode_ep_picture_layer_rpr` thread the
  reference layer's parameters and warp the enhancement layer's forward
  reference (which may differ in size) before prediction. The header
  parser leaves the EP RPRP field to the driver instead of refusing it
  (an EI-picture with the RPR bit is still refused).
- **CPM on the GOB path**, both directions: `gob_header::parse_gob_layer_cpm`
  reads the §5.2.4 GSBI between GN and GFID (`GobLayer::gsbi`); the
  baseline / PB / Improved-PB / extended GOB drivers read §5.1.21 PSBI
  after a set CPM bit and validate every GOB header's GSBI against it
  (single-Sub-Bitstream decode; a foreign GSBI is refused), and the
  PLUSPTYPE shim no longer refuses CPM without Slice Structured mode.
  `encode_intra_picture_gobs_cpm` / `encode_inter_picture_gobs_cpm`
  emit CPM = "1" + PSBI and the matching GSBI on every GOB header
  (pixel-neutral against the CPM = "0" forms). The oracle decoder does
  not frame CPM (it misparses from the first GOB), so no black-box
  check exists for it.
- Black-box finding (pinned by `tests/ffmpeg_blackbox.rs`): the oracle
  decoder's P-part output for an AP + PB picture depends on the B-part's
  content (§G.3 / §G.5 forbid that), and it zeroes an INTRA
  macroblock's candidate predictor in PB-frames mode (§6.1.1 rule 1
  exempts it). The crate's decoder follows the text; the PB encoders
  send a zero INTRA vector so their plain (non-AP) streams decode
  identically on both. Under UMV + PLUSPTYPE the oracle reads a
  forward-mode MVDB with a table other than the Table D.3 this crate
  applies (§M.2.2 → §5.3.7 / §D.2) and rejects a Table 14 one outright;
  the Annex G + UMV MVDB (Table 14, §D.2 pair rule) agrees exactly.

### Fixed

- The Improved PB-frame decode driver consumed neither the §5.1.24
  PEI / PSUPP loop nor the §5.2.2 group-number-0 GOB-header elision:
  a spec-conformant Improved PB-frame (as any real encoder emits it,
  now including this crate's) failed with `BadGroupStartCode` through
  both `decode_improved_pb_picture` and `decode_sequence`. The driver
  now frames the header tail exactly like the extended INTER path.
- §M.2.1 — under Annex M an INTRA P-macroblock carries MVD only in
  the bidirectional mode; the macroblock parser read one for the
  forward / backward rows too.

## [0.0.10](https://github.com/OxideAV/oxideav-h263/compare/v0.0.9...v0.0.10) - 2026-08-30

### Other

- round 453 encoder refresh — Annex J both directions, MVDB, adaptive quantisation, oracle validation, rate/PSNR ladder
- rate/PSNR ladder pin over the closed-loop GOP encoder
- PB-frames gain the §5.3.9 MVDB delta-vector search
- encoder gains deblock / four_mv / picture_bits rate-control options
- within-picture adaptive quantisation over the §5.3.6 DQUANT primitives
- black-box reference-decoder cross-validation of every encoder family
- Deblocking Filter mode encoder arm — Table J.1 element set, one/four-vector decision, GOP-driver switch
- fix fuzz-found infinite loop on truncated arithmetic pictures
- Data-Partitioned Slice mode, decode + encode — RVLC header partition, single-thread MV partition, marker/LMVV redundancy
- §W.5.3 reference fixed-point IDCT 0 + companion FDCT, oracle-verified bit-exact
- four-target harness; fix fuzz-found quadratic eager re-decode in the registry decoder
- Independent Segment Decoding on the GOB segmentation — banded reference fetches, OBMC/deblock segment rules, ISD encoder pair
- supplemental enhancement information layer + picture-level PSUPP splice
- real oxideav_core codec entry — streaming decoder/encoder adapters, dual-API factories, tags + PSC payload magics
- DF-4MV Table D.3 cell + UFEP=000 UUI inheritance pinned
- AP + UMV+ combined encoder — Table D.3 four-vector emission
- RRU + UMV Table D.3 pseudo-vectors land; B/EI/EP refuse UMV
- UMV+ conformance fixture + interaction-matrix sweep + slice encoder
- UMV+PLUSPTYPE motion vectors switch to Table D.3, both directions
- section N.4.2 Back-Channel Message syntax, parse + write
- Reduced-Resolution Update end-to-end, decode + encode
- Figure-F.1 predictor fixes, skip-MB OBMC, K+AP and K+CPM compose; AP/DF fixtures vendored
- Annex F Advanced Prediction + Annex G PB-frames compose with SAC, both directions
- round 438 — Annex E SAC both directions, Annex K RS/ASO, HRD rate control, RFC 2190 Mode B/C
- RFC 2190 Mode B/C — macroblock-boundary fragmentation both directions
- Annex B HRD-regulated bit-budget loop over the GOP encoder
- Rectangular Slice + Arbitrary Slice Ordering submodes, decode + encode
- decode_sequence routing, motion-search SAC P encoder, §E.5 boundary-run fix
- Annex E pictures end-to-end — decode_picture_sac + SAC I/P encoders
- Annex E arithmetic coder core — §E.2/§E.3 coders, §E.5 stuffing, §E.8 models
- RFC 2190 legacy Mode A + hard picture-boundary rule for both packetizers
- round 432 — on-wire PLUSPTYPE + Annex K slice encoding + RFC 4629 RTP
- §6.1.2 redundant picture header attachment (PLEN>0 + PEBIT)
- RFC 4629 payload format — header, packetizer, depacketizer
- Annex K Slice-Structured INTER pictures (encode_inter_picture_slices)
- fix clippy doc-list lint in AIC+MQ slice docs
- AIC (+MQ) inside Annex K slices — per-segment §I.3 availability
- Annex K Slice-Structured INTRA pictures (encode_intra_picture_slices)
- §K.2 slice-header writers (first-slice reduced form + SSTUF/SSC full form)
- self-describing H.263+ pictures — on-wire AIC/MQ/UMV signalling
- §5.1.4 PLUSPTYPE picture-header writer (PlusModes + UFEP=001 emission)
- AIC intra sequence encoder + README AIC-encoder rollup
- AIC + Annex T Modified Quantization INTRA picture encoder
- per-macroblock AIC INTRA_MODE decision (rate-driven)
- Annex I §I.3 Advanced INTRA Coding picture encoder
- Annex I §I.3 block-level AIC encoder (plan + Table-I.2 emit)
- Annex I §I.3 / Table I.2 INTRA-coefficient VLC writer

### Added

- **Annex J Deblocking Filter mode — encoder arm** (round 453):
  `encode_intra_picture_deblock` / `encode_inter_picture_deblock`
  (`DeblockConfig`) emit self-describing H.263+ pictures with OPPTYPE
  bit 9, coding motion under the mode's Table J.1 element set — §F.2
  predictors for every macroblock, a per-macroblock one/four-vector
  (INTER4V) cost decision with plain (non-OBMC) block prediction, the
  Table 18 / Table F.1 chroma-vector split the decoder applies, the
  §5.3.2 INTRA-refresh decision and COD = 1 skipping — plus optional
  Annex D extended range (Table D.3 difference coding).
  `GopConfig::deblock` / `GopConfig::four_mv` route the closed-loop
  `encode_sequence` GOP driver through the mode, predicting from the
  §J.3-filtered reconstruction the crate decoder returns.

- **Within-picture adaptive quantisation** (round 453): the
  `encoder_rc` module's `encode_intra_picture_adaptive` /
  `encode_inter_picture_adaptive` (`AdaptiveQuantConfig`,
  `AdaptiveQuantPicture` with the per-macroblock QUANT trace) drive
  the §5.3.6 DQUANT primitives from a bit-budget governor: each coded
  macroblock's QUANT follows the pro-rata picture budget through the
  `+Q` macroblock types (±2 differential, `1..=31`), skipped
  macroblocks hold QUANT, and the pictures stay plain baseline
  streams. `RateControlConfig::mb_adaptive` routes the regulated GOP
  driver through them, so the frame-level controller and the
  within-picture governor compose.

- **Registry encoder options: `deblock`, `four_mv`, `picture_bits`**
  (round 453): the `oxideav_core` streaming encoder now drives the
  Annex J Deblocking Filter GOP loop (`deblock` / `four_mv`,
  byte-identical to `encode_sequence` with `GopConfig::deblock`) and
  the rate-controlled adaptive-quantisation loop (`picture_bits`
  engages the frame-level virtual-buffer controller plus the §5.3.6
  within-picture DQUANT governor; when the option is unset the target
  derives from `CodecParameters::bit_rate` + `frame_rate`). The
  baseline-only rate-controlled path refuses the `deblock` / `umv`
  combinations instead of silently ignoring them.

- **PB-frames with non-zero MVDB** (round 453): `encode_pb_picture`
  gains a §5.3.9 delta-vector search (`PbConfig::b_search_half`,
  half-pel window around the §G.4 scaled prediction): candidate
  deltas are admitted only when Table 14 codable **and** keeping each
  forward-vector component inside the §G.4 permitted range — the
  in-range value is the one any decoder selects from the Table 14
  codeword pair, its mate sitting exactly 64 half-pels outside — and
  the winning delta rides MODB `"10"` (MVDB only) or `"11"`
  (CBPB + MVDB), completing the Table 11 row set on the write side.
  Measured on non-linear motion the searched B-picture beats the
  `MVD = 0` encoder; `b_search_half = 0` restores the old behaviour.

- **Rate/PSNR ladder pin** (round 453): `tests/rate_psnr_ladder.rs`
  encodes QCIF and CIF moving clips through the closed-loop GOP
  encoder at QP 2..31 and asserts the ladder is monotone on both axes
  (rate falls, PSNR falls as QP coarsens) with loose endpoint
  anchors; the measured table is printed for the README.

- **Black-box reference-decoder cross-validation** (round 453):
  `tests/ffmpeg_blackbox.rs` hands crate-encoded elementary streams
  (baseline I at four quantisers and three sizes, I+P GOPs, UMV,
  Advanced Prediction, AIC+UMV H.263+, Annex J deblocking with
  one- and four-vector pictures) to an independent decoder binary and
  compares its planar output with the crate's own reconstruction:
  flat (no-AC) pictures byte-exact, AC-bearing picture `i` within the
  compounding per-IDCT divergence bound `±(i + 1)` (§6.2 leaves the
  inverse transform open; Annex A.7 bounds one pass at ±1) and a mean
  |diff| ceiling three orders of magnitude under a real mismatch.
  Skips with a notice when no oracle binary is on `PATH`.

### Fixed

- **Fuzz-found: infinite loop on truncated SAC pictures** (round
  450): a corrupt or truncated Annex E picture whose exhausted
  arithmetic source kept synthesising zero bits could decode §5.3.2
  MCBPC **stuffing** symbols forever — stuffing consumes no
  macroblock slot, so the SAC macroblock-stream loop never advanced
  (>15 s fuzzer timeouts; unbounded on real inputs). `SacDecoder` now
  tracks bits synthesised past the end of the buffer
  (`source_exhausted`, threshold 64 — far beyond the §E.6 flush
  lookahead) and the stuffing retry loop surfaces `UnexpectedEof`
  instead of spinning. Regression test truncates a real SAC picture
  at every eighth byte; a 180 s fuzz re-run with a per-input timeout
  is clean (76k runs).

- **Fuzz-found: quadratic eager re-decode in the streaming registry
  decoder** (round 450): a stream delivered in many small fragments
  made `H263StreamDecoder` re-attempt a decode of the growing
  unterminated tail after every packet — a delivery-shape DoS the new
  `registry_decoder` fuzz target surfaced as >10 s single inputs. The
  eager attempt now backs off geometrically (retry only once the tail
  has doubled since the last failed attempt; a following PSC still
  completes the picture immediately), taking the slow units from
  >10 s to <100 ms with one-packet-per-picture latency unchanged.

- **`register()` installs a real codec entry** (round 450; closes the
  cross-crate finding that the registry path was a no-op stub):
  registry resolution through an `oxideav_core::RuntimeContext` now
  constructs working codecs honouring `CodecParameters`. New `codec`
  module: `H263StreamDecoder` (streaming `Decoder` over the
  elementary-stream drivers — byte-aligned §5.1.1/§5.1.28 PSC
  re-framing so one-picture-per-packet and arbitrarily-split raw
  streams both decode, eager tail decode for zero-latency container
  delivery, PB / Improved-PB pairs in display order, per-packet PTS
  threading, `reset()` for seeks, `DecoderLimits` pixel cap),
  `H263StreamEncoder` (closed-loop per-frame form of
  `encode_sequence`; `quant` / `gop` / `search` / `umv` / `eos`
  codec-options knobs, keyframe-flagged packets), the direct
  `make_decoder` / `make_encoder` factories (dual-API convention),
  `H263` / `S263` FourCC tag claims and the `00 00 8x` PSC payload
  magics. `tests/registry.rs` resolves the codec through a fresh
  `RuntimeContext` and pins registry output byte-identical to
  `decode_sequence` / `encode_sequence` across whole-stream, 7-byte
  shred and one-picture-per-packet deliveries.

### Added

- **Annex V Data-Partitioned Slice mode, both directions**
  (round 450): OPPTYPE bit 17 accepted (`Opptype::data_partitioned_slices`)
  and routed to a dedicated slice driver — Table V.1 / V.2 RVLC
  COD + MCBPC inventories (prefix-free + reversal-closure pinned),
  §V.2.2 HM / §V.2.5 MVM markers, the §V.2.3.2 single
  motion-vector prediction thread with the §V.2.3.3 per-codeword
  emulation rule, §V.2.4 LMVV redundancy validation, and the §V.2.6
  coefficient layer. Encoders `encode_intra_picture_dps` /
  `encode_inter_picture_dps` (free-running row slices;
  `PlusModes::data_partitioned_slices`). Closed-loop byte-exact over
  1/3/9-row slicings; DPS INTRA pinned byte-identical to the
  interleaved Annex K coding; bit-corruption sweep surfaces
  structural errors. New error variants `BadDpsHeaderCode` /
  `BadDpsMarker` / `DpsPartitionMismatch`. Staged subset: I / P over
  sequential free-running slices (Rect / ASO, INTER4V, PB forms and
  other mode combinations refused).

- **Annex W §W.5.3 reference fixed-point IDCT 0 + companion FDCT**
  (round 450): `w_idct::idct_w0` / `w_idct::fdct_w0`, transcribed
  statement-for-statement from the C listing embedded in the staged
  Recommendation text (the ratified in-spec-listing rule), verified
  bit-identical to a local oracle build of that listing over 20k+
  random/edge blocks each; CI keeps pinned oracle vectors, Annex A
  ±1 agreement with the f64 reference kernel on realistic coefficient
  blocks, range-clip and round-trip tests. Enables bit-exact
  reconstruction of streams announcing `FixedPointIdct(0)` via the
  Annex L SEI layer (§W.5.2 forced-updating removal).

- **Fuzz harness** (round 450): `fuzz/` sub-crate with four
  `cargo fuzz` targets — `decode_sequence`, `registry_decoder`
  (fuzzer-chosen packetisation + reset-recovery), `psupp` (Annex L/W
  round-trip idempotence) and `picture_header`. All clean after the
  eager-decode fix above.

- **Annex R Independent Segment Decoding mode, GOB segmentation, both
  directions** (round 450): the PLUSPTYPE shim accepts the OPPTYPE
  bit-12 ISD flag (retained in `InheritedExtendedState` for UFEP=000
  followers) and the GOB driver treats each header-delimited video
  picture segment as a picture — byte-aligned GBSC pre-scan into a
  per-row segment-band map, `RefPlane::banded` reference fetches for
  single-MV / INTER4V / OBMC / chroma prediction (§R.2 rule 4 border
  extrapolation), §F.3 remote-vector substitution across segments
  (rule 2), §J.3 deblocking skip across segment boundaries (rule 3),
  Annex P resampling refused (rule 7). New encoders
  `encode_intra_picture_isd` / `encode_inter_picture_isd` (ISD + UMV
  PLUSPTYPE header, one GOB header per GOB, search + prediction against
  an edge-replicated per-segment reference view) plus
  `PlusModes::independent_segment_decoding`. `tests/isd_roundtrip.rs`
  pins closed-loop byte-exactness, a cleared-ISD-bit negative control,
  the streaming step path and the deblock boundary skip. ISD + SS /
  Improved-PB / RPS / RRU remain refused.

- **Annex L / Annex W supplemental enhancement information**
  (round 450): new `annex_l` module — §5.1.24/§5.1.25 PEI + PSUPP loop
  primitives, the §L.2 FTYPE/DSIZE function layer over the complete
  Table L.1 + Table W.1 inventory (freeze / snapshot / segment-tag
  functions, §L.14 Chroma Keying with the flag-derived DSIZE rule,
  §L.15 extended-function escape, §W.5 Fixed-Point IDCT, §W.6 Picture
  Message with Table W.2 MTYPEs and the §W.6.2/§W.6.3.11/§W.6.3.12
  CONT/EBIT/DSIZE constraints), the §L.3 start-code-emulation Do
  Nothing insertion on write, and picture-layer `extract_psupp` /
  `insert_psupp` (post-hoc SEI splicing into single-segment baseline
  pictures, bit-shifting the payload and re-padding PSTUF).
  `tests/sei_roundtrip.rs` pins recoverability and pixel-neutrality
  over the VLC I / P and SAC encoders. New error variants
  `TruncatedPsupp` / `BadSupplementalDsize` / `BadPictureMessage`.

- **Streaming sequence-decode API**: `picture::SequenceState` +
  `picture::decode_sequence_step` expose the per-picture step of
  `decode_sequence` (inherited §5.1.4.4 mode state, §G.4 reference-TR
  threading, display-order PB output — the last returned frame is the
  next prediction reference), and `picture::next_picture_start_code`
  exposes the byte-aligned PSC scanner. `decode_sequence` is now a
  thin loop over them (behaviour unchanged).

### Fixed

- **UMV + PLUSPTYPE motion vectors are Table D.3, both directions**
  (round 447; closes the round-443 self-found nonconformance): §5.3.7 —
  "if the Unrestricted Motion Vector mode is used and PLUSPTYPE is
  present, motion vectors are coded using Table D.3 instead of
  Table 14". Both sides previously used the Table 14 codes with the
  PLUSPTYPE-absent §D.2 pair-selection reconstruction on the UMV+
  path, so crate round-trips agreed while the wire disagreed with the
  Recommendation. Now: the macroblock parser reads every MVD /
  MVD2-4 / MVDB pair as two §D.3 reversible codewords when
  [`MbContext::umv_table_d3`] is set (with the §D.2 six-zero
  emulation-prevention bit after a `(+0.5, +0.5)` pair), the
  reconstruction is the single-valued `predictor + difference` (no
  wrap, no dependence on the predictor window), and each component is
  bounded by the §5.1.9 UUI selection — the Tables D.1 / D.2
  picture-size ranges under UUI = `"1"`, the Table D.3 codomain under
  UUI = `"01"` (previously refused, now decoded; §D.1.1 is enforced
  through the §D.1 edge replication). The last-sent UUI is inherited
  by UFEP=000 pictures (`InheritedExtendedState::uui`, parallel to
  the SSS rule). Encoder: `encode_inter_picture_umv_plus` now emits
  Table D.3 pairs (direct differences) under a Tables-D.1/D.2 +
  §D.1.1 candidate window (`estimate_motion_umv_plus`), so its wire
  deliberately differs from the baseline-PTYPE
  `encode_inter_picture_umv`. Validated in both directions: the
  staged H.263+ UMV conformance fixture (real-encoder QCIF I+P+P,
  slice-structured + custom-PCF) decodes with zero samples beyond the
  Annex A.7 ±1 bound, and a crate-encoded UMV+ I+P stream (25-pel
  motion) black-box-decodes through an external decoder binary with
  max per-sample diff 1. The PLUSPTYPE-absent Annex D form (Table 14
  + §D.2 pair selection) is unchanged.

- **§P.2.2 warping-parameter emulation-prevention condition** (round
  447): the RPRP parser inserted/required the emulation-prevention
  bit after a pair of *zero-value* (`"1"`) Table D.3 codewords; §P.2.2
  places it after a pair of **value-+1** (`"000"`) codewords — the
  only pair that emits six consecutive zero bits. Zero-value pairs now
  read no EPB, `(+1, +1)` pairs require it.

### Added

- **Table D.3 write primitive + §D.2 MVD pair codecs** (round 447):
  `annex_p::write_table_d3` (exact inverse of `read_table_d3`,
  full-codomain round-trip pinned against the §D.3 worked `-13`
  example), `encoder_vlc::write_mvd_pair_d3` /
  `macroblock::read_mvd_pair` (pair form with the §D.2
  emulation-prevention rule), `motion::reconstruct_mv_umv_plus` and
  the Tables D.1 / D.2 half-pel range helpers
  (`umv_plus_horizontal_range_half` / `umv_plus_vertical_range_half`).
  `Mvd` components widen from `i8` to `i16` to carry the §D.3
  `[-4095, 4095]` difference range (the Table 14 form is unchanged on
  the wire).

- **Conformance fixture: `unrestricted-mv-mode` vendored** (round
  447): the staged H.263+ Annex D reference stream (QCIF I+P+P — UMV+
  Table D.3 motion, Annex K slice-structured framing, custom PCF)
  decodes under `tests/fixture_decode.rs` with **zero** samples beyond
  the Annex A.7 ±1 bound, with a SHA-256 corruption guard, and joins
  the RFC 4629 packetize/depacketize round-trip battery.

- **UMV+ interaction matrix, decode side** (round 447): end-to-end
  driver tests pin Table D.3 MVDs composing with **Advanced
  Prediction** (a uniform INTER4V motion field beyond the Table 14
  window reconstructs as a pure translation through the §F.2
  predictors + §F.3 OBMC), **Modified Quantization** (§T.2
  variable-length DQUANT followed by a Table D.3 pair in the same
  macroblock header), the §D.2 emulation-prevention alignment across
  consecutive macroblocks, the UUI = "01" Unlimited acceptance and
  the Tables-D.1/D.2 range refusal. Slice-structured composition is
  pinned by the vendored fixture. Also pinned: the **Deblocking
  Filter four-vector** cell (Table J.1 DF-4MV INTER4V macroblocks
  read Table D.3 MVD2-4; a uniform extended-range field is
  §J.3-invariant on a linear ramp) and **UFEP=000 UUI inheritance**
  (a follow-up P-picture keeps the last-sent UUI in effect per the
  §5.1.4.4 pattern; a snapshot without one is refused).

- **`encode_inter_picture_umv_slices`** (round 447): UMV+ × Annex K on
  the encode side — H.263+ slice-structured P-pictures with Table D.3
  motion vector coding (the staged conformance stream's mode pairing),
  round-tripped through `decode_sequence` at 25-pel motion across
  slice heights 1 / 3 / 9.

- **`encode_inter_picture_ap_umv_plus`** (round 447; closes the
  encoder "UMV + AP combined mode" gap in its H.263+ form): the
  two-pass Annex F OBMC encoder on an extended-PTYPE header with
  OPPTYPE AP + UMV — per-8×8-block estimation over the
  Tables-D.1/D.2 range under the §D.1.1 border bound
  (`estimate_block_motion_umv_plus`), all four MVD pairs emitted as
  §5.3.7/§5.3.8/§D.2 Table D.3 codewords
  (`encode_inter4v_macroblock_umv_plus`). Round-tripped through
  `decode_sequence` at 25-pel motion (through the decoder's §F.2
  predictors + §F.3 OBMC) and black-box-decoded externally at max
  per-sample diff 1.

- **Annex Q × Annex D: RRU + UMV end-to-end** (round 447; removes a
  round-443 refusal): §Q.4 — with UMV also in use the pseudo motion
  vector is `pseudo-PC + difference` with the difference read from
  **Table D.3**, and the §D.2 range ("the specified range applies to
  the pseudo motion vectors") bounds the pseudo domain per the UUI
  selection — so the actual motion reach roughly doubles. Decoder:
  the RRU driver accepts the OPPTYPE UMV bit, parses UUI, and routes
  the pseudo reconstruction through the Table D.3 no-wrap path.
  Encoder: `encode_inter_picture_rru_umv` widens the pseudo-domain
  candidate window to the Tables-D.1/D.2 range and emits Table D.3
  pairs. Pinned: static RRU+UMV P lossless, a 40-pixel translation
  (pseudo ≈ 20 pel, outside the default `[-16, 15.5]` window)
  round-trips within the RRU low-pass budget and beats the
  default-window encoder on both size and error, and RRU+UMV I+P
  streams decode through `decode_sequence`.

### Fixed

- **B / EI / EP pictures refuse a UMV-signalled header** (round 447):
  §O.4.6 codes MVDFW / MVDBW "in Table 14, or in Table D.3 if the
  Unrestricted Motion Vector mode is used" — the scalability paths do
  not stage the Table D.3 form, and previously ignored the OPPTYPE
  UMV bit entirely, so a conformant UMV B-picture would have had its
  vector data misparsed as Table 14. The unstaged mode is now refused
  like AP / SAC / slice-structured on the same paths.

### Added

- **§N.4.2 Back-Channel Message syntax** (round 443):
  `annex_n::parse_bcm` / `write_bcm` stage the Figure-N.4 BCM record
  a decoder returns to the encoder — BT (ACK `"11"` / NACK `"10"`,
  reserved code points refused), URF, the 10-bit TR, ELNUMI + ELNUM,
  BCPM + BSBI, the videomux-only BEPB1/BEPB2 emulation-prevention
  bits, the GN/MBA field at a caller-supplied width, and the
  NACK-only 10-bit RTR — under a `BcmContext` carrying the
  externally-negotiated framing (videomux + GN/MBA width, properties
  of the bitstream the message applies to per the §N.4.2.9 NOTE).
  Field-matrix round-trips and malformed-shape refusals (reserved BT,
  zero BEPB, ACK-with-RTR / NACK-without-RTR, out-of-range GN) are
  pinned; the §N.4.2.12 BSTUF external-frame stuffing stays
  caller-side.

- **Annex Q Reduced-Resolution Update end-to-end, both directions**
  (round 443): the extended-PTYPE routing detects the §5.1.4.3
  MPPTYPE RRU bit and decodes through the new dedicated driver —
  §Q.1 geometry (`(H, V)` → reference `(HR, VR)` → coded `(HC, VC)`
  with the 32×32 macroblock grid), §Q.3 reference extension by edge
  replication, §Q.4 pseudo-motion-vector reconstruction
  (`rru_pseudo_component` / `rru_actual_component` — the §6.1.1
  predictor over actual vectors converted to the pseudo domain, the
  Table-14 MVD wrap applied in the baseline `[-16, 15.5]`-pel pseudo
  window, expansion to the half-integer-or-zero actual lattice),
  16×16 prediction blocks, §Q.2.2.2 texture decode + §Q.6
  up-sampling, §Q.2.2.3 summation/clip, the §Q.7.1 default boundary
  filter (either-side coded-MB condition, §J.3 ordering) and the
  §Q.2.3/§Q.2.4 crop. Encoder arm: `encode_intra_picture_rru`
  (16×16 → 8×8 down-sampling into the standard INTRA stage) and
  `encode_inter_picture_rru` (pseudo-domain SAD search so every
  candidate is codable, per-sub-block residual down-sampling), both
  self-describing via the new `PlusModes::rru` MPPTYPE bit.
  Round-trips: the pseudo↔actual lattice is pinned exhaustively, a
  static RRU P-picture is lossless, CIF (no extension) and QCIF
  (§Q.3 extension + crop) I-pictures and translated P-pictures
  reconstruct within the mode's low-pass budget, and RRU I + P
  elementary streams decode through `decode_sequence`. Unstaged
  combinations (UMV Table-D.3 pseudo-vectors, §Q.5 enlarged OBMC,
  §Q.7.2 DF filtering, Annex K, custom formats, B/EI/EP) are
  refused.

### Fixed

- **§Q.6 up-sampling rounding follows the Implementors' Guide**
  (round 443): the published Figure Q.8 / Q.9 divisions
  ("division by truncation") mis-round negative numerators; the
  Implementors' Guide for the Recommendation (2005-08) corrects them
  to the arithmetic shift `(N + D/2) >> K`. `upsample_prediction_error`
  now shifts, so a negative constant block up-samples to the same
  constant instead of biasing toward zero.

- **§F.2 / Figure F.1 candidate-predictor cells corrected** (round
  443): the four-vector candidate table read two cells wrong — B1's
  MV3 now reads the **above-right macroblock's B3** (previously the
  above macroblock's B4) and B4's MV2 / MV3 now read the current
  macroblock's **B1 / B2** (previously B2 / a right-neighbour cell
  that is never a candidate). The Figure-F.1 sub-figures pin both
  (the B1 MV3 cell sits past the macroblock's right edge; every B4
  candidate is intra-macroblock). The §6.1.1 rule-3 / rule-4 border
  rewrites follow the corrected cells (B1's MV3 now takes the
  right-edge zero and the above-right-unavailable → MV1 collapse),
  and `Mb4MvNeighbourhood` loses its never-consulted `right` field.
  Encoder (`Mv4Grid::predict_block`) and decoder moved together, so
  crate round-trips were unaffected — the vendored
  `advanced-prediction-mode` conformance fixture is what caught the
  deviation (frame 2 INTER4V macroblocks mispredicted their
  single-MV neighbours below).

- **Single-MV macroblocks in AP / DF-4MV pictures predict per
  Figure F.1** (round 443): §F.2 — "if only one vector per macroblock
  is present, MV1, MV2 and MV3 are defined as for the 8 × 8 block
  numbered 1". The single-MV path used the macroblock-level Figure-12
  predictor, which reads an INTER4V neighbour's B1 vector where
  Figure F.1 names its B3 / B2 cells. New `predict_mv_ap_single`
  (shared by the VLC, SAC and slice drivers) applies the block-1
  derivation whenever four-vector macroblocks are possible.

- **COD = 1 macroblocks in AP pictures are OBMC-blended** (round
  443): §5.3.1 NOTE — "in Advanced Prediction mode, overlapped block
  motion compensation is also performed if COD is set to '1'". The
  skipped-macroblock path plain-copied the co-located reference;
  it now defers a zero-vector `PendingApLuma` like every other AP
  macroblock, so the luminance is the §F.3 blend of the zero vector
  with the neighbours' remote vectors (chroma stays the co-located
  copy — the §F.2 sum-of-four vector of four zeros).

### Added

- **`DecodeOptions::obmc_skip_zero_right`** (round 443): opt-in
  ecosystem-compatibility deviation for AP pictures — a skipped
  macroblock's §F.3 right-half remote vectors are taken as zero
  instead of the right neighbour's actual vector, matching encoders
  whose COD decision is made under a one-pass model (their paired
  decoders reconstruct the same way). Spec default (`false`) keeps
  the §F.3 actual-vector reading; the divergence is bounded to the
  skipped-macroblock right halves and pinned by the fixture test.

- **Conformance fixtures: `advanced-prediction-mode` +
  `deblocking-filter` vendored** (round 443): the staged Annex F
  (baseline 4MV + OBMC QCIF I+P+P) and Annex J (H.263+ in-loop
  deblocking QCIF I+P+P) reference streams now decode under
  `tests/fixture_decode.rs` with SHA-256 corruption guards. The AP
  fixture is byte-exact within the Annex A.7 tolerance under
  `obmc_skip_zero_right` (and within 100 samples of it without); the
  DF fixture asserts a small bounded divergence (max ±3, ≤ 100
  samples) because the ±1 IDCT bound applies before the in-loop
  filter, which can amplify it and feed it forward through the
  prediction loop.

- **Annex K + Annex F: Advanced Prediction composes with the
  Slice-Structured mode** (round 443): the slice decode driver now
  accepts AP pictures — §K.1 rule 1 (motion-vector prediction "as if
  a GOB header were present") rides the per-segment grid checks now
  threaded through `reconstruct_inter4v_mvs` / `predict_mv_ap_single`,
  and §K.1 rule 3 / §F.3 ("remote motion vectors corresponding to
  blocks from other video picture segments are set to the motion
  vector of the current block") is applied by a segment-filtered
  deferred-OBMC flush (each slice's pending macroblock resolves
  within its own walk — under RS / ASO every out-of-slice remote
  substitutes Current, so one-macroblock deferral still suffices).
  The encoder arm `encode_inter_picture_ap_slices` emits free-running
  AP slices with the slice-confined estimation
  (`Mv4Grid::with_row_segments`) and OBMC remotes; a static picture
  is lossless, the single-slice form reconstructs byte-identically to
  the baseline-PTYPE AP encode, and an I + AP-slice-P stream decodes
  through `decode_sequence`.

- **Annex K + CPM: Continuous Presence Multipoint composes with the
  Slice-Structured mode** (round 443): the PLUSPTYPE shim accepts
  CPM = "1" when Annex K is signalled (GOB-path CPM stays refused —
  GSBI is not framed), threading the §5.1.21 PSBI to the slice
  driver, which parses the §K.2.4 SSBI on every non-first slice
  header and stages the **single-Sub-Bitstream** decode: every
  slice's SSBI must name the PSBI Sub-Bitstream (a true Annex C
  multiplex is refused). Encoder: `write_slice_layer_cpm` /
  `subbitstream_to_ssbi` emit the Table-K.1 codeword and
  `encode_intra_picture_slices_cpm` produces a self-describing CPM
  slice picture; reconstruction is byte-identical to the CPM-off
  slice encode across all four Sub-Bitstream numbers, and an SSBI /
  PSBI mismatch is refused.

- **Annex E SAC composed with Annex F Advanced Prediction** (round
  443): `parse_macroblock_sac` now decodes the Table-8 INTER4V /
  INTER4V+Q types with their §5.3.8 MVD2-4 pairs under the §E.7
  `cumf_MVD` model, and `decode_picture_sac` reconstructs an
  AP-signalled SAC picture through the exact VLC AP machinery — the
  shared `reconstruct_inter4v_mvs` §F.2/Figure-F.1 four-vector
  derivation (extracted from the VLC INTER4V path) and the §F.3
  deferred-OBMC luminance flush (`PendingApLuma`), with single-MV
  coded INTER macroblocks OBMC-blended too. The encoder arm
  `encode_inter_picture_ap_sac` is the arithmetic mirror of the
  two-pass VLC AP encoder (same estimation, same OBMC prediction,
  same transform stage, emitted via the new
  `sac::encode_inter4v_macroblock_sac`), so SAC-AP and VLC-AP
  pictures of the same source reconstruct **byte-identically**
  (pinned at QP 4/8/13 over sheared content; a static AP picture is
  lossless).

- **Annex E SAC composed with Annex G PB-frames** (round 443): the
  §E.8 `cumf_MODB_G` / `cumf_YCBPB` / `cumf_UVCBPB` models land with
  their §E.7 assignment — MODB (Table 11 indexing), the six per-block
  CBPB symbols (luma vs chroma model split), MVDB under `cumf_MVD`
  and the §G.2 PB-mode INTRA-macroblock vector. The new
  `decode_pb_picture_sac` decodes a baseline-PTYPE SAC + PB picture
  into the (B, P) pair — fixed-length header (PQUANT / CPM / TRB /
  DBQUANT / PEI, §E.6) then a single arithmetic segment whose
  per-macroblock B-parts run through the shared
  `reconstruct_pb_b_part` core (split out of the VLC B-part decoder)
  — and `decode_sequence` routes an SAC PB-frame there automatically.
  `encode_pb_picture_sac` mirrors the VLC PB encoder (same P-part
  estimation, §G.5 PREC reconstruction and §G.4 B-prediction), so SAC
  and VLC PB-frames reconstruct **identically in both parts** (pinned;
  a fully static SAC PB-frame is lossless on both parts; an SAC
  I + AP-P + PB elementary stream decodes in display order through
  `decode_sequence`). AP + PB together stays refused, mirroring the
  VLC driver's B-part-vs-deferred-OBMC ordering hazard.

- **Annex E SAC pictures end-to-end** (round 438): `decode_picture_sac`
  decodes a baseline-PTYPE picture whose §5.1.3 bit 11 (SAC) is set —
  fixed-length header layer (PQUANT / CPM / PEI-PSUPP, §E.6), then the
  §E.3 `decoder_reset` and a single-segment arithmetic macroblock walk
  (`sac::parse_macroblock_sac` — COD / MCBPC / CBPY / DQUANT / MVD
  under their §E.7 models — plus the §E.4 block layer), reconstructed
  through the exact baseline primitives (Figure-12 median predictor
  with the Annex D UMV range when PTYPE signals it, Table-18 chroma
  vectors, §6.1–§6.3 block reconstruction, optional Annex J post
  filter). The encoder arm lands alongside:
  `encode_intra_picture_sac` / `encode_inter_picture_sac` (PTYPE
  bit-11 header + `sac::encode_intra_macroblock_sac` /
  `encode_inter_macroblock_sac` / `encode_skipped_macroblock_sac` +
  §E.6 flush + PSTUF) share the VLC encoder's transform / quantiser
  stage, so SAC and VLC pictures of the same source reconstruct
  **byte-identically** — pinned by integration tests across
  sub-QCIF/QCIF/CIF at QP 2/4/13/31 (I) and QP 6 (P), together with
  flat-exactness, all-skip losslessness, mutual driver rejection,
  §5.1.4.6 barred-combination refusal (Annex S / T / AIC), and a
  no-PSC-emulation byte scan. Measured entropy-layer saving on the
  gradient QCIF corpus: SAC I-pictures are 6.4–25.5 % smaller than
  the VLC pictures (QP 31 → QP 2); an all-skip SAC P-picture is
  ≤ 32 bytes. §5.1.4.6 combinations (Annex S/T), PB-frames, Advanced
  Prediction and mid-picture GOB headers are refused pending later
  rounds.

- **RFC 2190 Mode B / Mode C macroblock-boundary fragmentation**
  (round 438): the legacy `video/H263` packetizer now fragments a
  Picture/GOB segment larger than the budget at **macroblock
  boundaries** instead of refusing. `picture::enumerate_mb_boundaries`
  walks a baseline (plain or PB-frames) picture without pixel
  reconstruction and returns the §5.2 resumption side channel per
  macroblock — bit offset, GOBN, in-GOB MBA, QUANT in force and the
  §6.1.1 median motion-vector predictor. The packetizer emits the
  first fragment of a start-code segment as Mode A (with EBIT) and
  every mid-picture fragment as **Mode B** (8-byte header:
  QUANT/GOBN/MBA + HMV1/VMV1 as 7-bit two's complement) or **Mode C**
  for PB pictures (12-byte header adding DBQ/TRB/TR);
  `depacketize_payloads_rfc2190` now reassembles **any mode mix at
  bit granularity** (SBIT/EBIT-aware, shared split bytes merged
  exactly once). Mode B/C header writers/parsers
  (`Rfc2190ModeB`/`Rfc2190ModeC`) are exact wire inverses incl.
  negative predictors. Tests: oversized baseline I+P streams
  fragment/round-trip byte-exactly at 192/256/512-byte budgets with
  the Mode B side channel cross-checked against
  `enumerate_mb_boundaries` ground truth; an oversized PB picture
  Mode-C-fragments and reassembles; header dispatch rejects
  cross-mode parses. (SAC and Advanced Prediction pictures are not
  fragmentable below GOB granularity and keep the refusal.)

- **Encoder rate control — Annex B HRD-regulated bit-budget loop**
  (round 438): the new `rate_control` module stages `HrdModel` (the
  Annex B Hypothetical Reference Decoder buffer simulation — §B.3
  empty start, §B.4 picture-clock examinations with instantaneous
  removal, the post-removal occupancy-below-`B = 4·R/PCF` §B.2/§B.4
  requirement) and `RateController` (a virtual-buffer QUANT governor:
  fullness integrates `actual − target` with symmetric caps, QUANT
  follows proportionally with a bounded ±4 step).
  `encode_sequence_rate_controlled` pairs them over the closed-loop
  I + P GOP encoder: per-picture QUANT from the controller, §B.4
  violations re-encoded at finer QUANT and > 4×-budget overshoots at
  coarser QUANT (bounded by `max_reencodes`), with per-picture
  bits / QUANTs and HRD conformance reported on
  `RateControlledStream`. Measured steady-state accuracy on the
  moving-square QCIF clip: mean bits/picture within **−9.1 % … +4.6 %**
  of target across 1.5 k–5 k bit/picture budgets, all §B.4-conformant
  at `B = 4T`; integration tests pin the long-run average (±25 %),
  budget scaling (thin stream smaller + coarser steady QUANT),
  HRD conformance and I-burst absorption.

### Fixed

- **Forward quantiser could emit unrepresentable LEVELs**
  (round 438): `fdct::quantise_ac_coefficient` now saturates the
  quantised magnitude at 127 — the §5.4.2 baseline ESCAPE LEVEL
  ceiling (8-bit two's complement, `0x00`/`0x80` forbidden). Sharp
  content at very fine quantisers (e.g. a 70-step edge at QUANT ≤ 2)
  previously produced |LEVEL| > 127 and the VLC/SAC emitters failed
  with `BadTcoefCode`; the clamp mirrors the Annex I planner's
  existing `MAX_LEVEL_BASELINE` policy (the MQ extended-range paths
  quantise through the Annex I planner and are unaffected).

### Added

- **Annex K Rectangular Slice + Arbitrary Slice Ordering submodes**
  (round 438): the Slice-Structured decode driver now stages both §K.1
  submodes. Under **RS** each slice header carries the §K.2.8 SWI
  field and the slice's macroblocks are walked in scanning order
  *within* the `SWI + 1`-wide rectangle anchored at MBA (a rectangle
  overhanging the right picture edge is refused with
  `SliceSwiOutOfRange`); under **ASO** the §K.1 strictly-increasing
  MBA rule is waived and slices land by address in any bitstream
  order — picture completion is coverage-driven (every macroblock
  decoded exactly once) rather than raster-driven, and the reduced
  first slice need not be the MBA-0 slice. The encoder arm:
  `encode_intra_picture_slices_rect` /
  `encode_inter_picture_slices_rect` tile the picture into
  full-height vertical stripes (each its own §6.1.1 segment, SWI on
  the wire, self-describing SSS submode bits), with `arbitrary_order`
  emitting the stripes right-to-left. Unit tests pin the two-stripe
  skipped-INTER reference copy, ASO out-of-order placement,
  out-of-order-without-ASO rejection and the rectangle-overhang
  refusal; integration tests pin rect-vs-free-running INTRA
  reconstruction identity across stripe widths 1/3/4/6/11, ASO-vs-
  sequential identity, static rect-P losslessness and rect-P parity
  with the plain zero-MV P-picture in both orders.

- **SAC streams + motion estimation** (round 438):
  `encode_inter_picture_motion_sac` is the arithmetic-coded mirror of
  the motion-search P encoder (SAD + half-pel estimation around the
  §6.1.1 median predictor replayed through `MvGrid`, intra-refresh
  decision, skip rule), and `decode_sequence` now routes a
  baseline-PTYPE picture whose PTYPE bit 11 is set to
  `decode_picture_sac` — so pure-SAC **and mixed VLC + SAC**
  elementary streams decode through the headline entry point with
  reference threading across the entropy-layer boundary. Integration
  tests pin motion-SAC-vs-motion-VLC byte-identical reconstruction,
  an SAC I + P(motion) + P(skip) stream and a VLC-I + SAC-P stream
  through `decode_sequence`.

### Fixed

- **§E.5 zero-run continuity across the header boundary**
  (round 438): the SAC stuffing / destuffing filters now seed their
  zero-run counter with the trailing zeros of the fixed-length
  picture-header string (`SacEncoder::with_zero_run` /
  `SacDecoder::with_zero_run` — §E.5 counts runs over the *whole*
  stream, headers included). Without the seed, a header tail of `k`
  zeros (PQUANT low bits + CPM = 0 + PEI = 0) followed by 14
  arithmetic zeros put `14 + k` consecutive zeros on the wire — for
  `k ≥ 2` a byte-aligned start-code emulation that truncated the
  picture inside `decode_sequence` (observed: a QP-6 P-picture
  emulating a PSC at byte 6). Regression-pinned across QP
  2/4/6/8/16/24 P-pictures plus the two elementary-stream tests.

### Added

- **Annex E Syntax-based Arithmetic Coding core** (round 438): the new
  `sac` module stages the §E.2 arithmetic encoder (`SacEncoder` —
  `encode_a_symbol` + the §E.6 `encoder_flush`), the §E.3 arithmetic
  decoder (`SacDecoder` — `decode_a_symbol` + `decoder_reset`), the
  §E.5 PSC_FIFO stuffing rule (a `"1"` stuffed after each 14-zero run
  on the encode side, the first `"1"` after each exactly-14-zero run
  deleted on the decode side) folded into both coders' bit paths, and
  all 23 §E.8 `cumul_freq` models needed by the baseline I/P syntax
  (COD, MCBPC I/P, CBPY, DQUANT, MVD, INTRADC, the TCOEF1/2/3/r INTER
  + INTRA families, SIGN, and the post-ESCAPE LAST/RUN/LEVEL models).
  Per-syntax-element codecs follow the §E.7 model assignment with the
  clause-5 table indexing (Table 7/8 MCBPC, Table 12 CBPY, Table 13
  DQUANT, Table 14 MVD with index 32 = 0, Table 15 INTRADC, Table 16
  TCOEF with ESCAPE = 102, Table 17 RUN/LEVEL), and
  `parse_block_sac` / `write_block_sac` stage the §E.4 Figure E.1
  block layer end-to-end. Unit tests pin model well-formedness
  (strictly-decreasing cumulative tables), mixed-model and
  600-symbol stress round-trips, most-probable-symbol compression,
  the no-15-zero-run stuffing invariant, stuffing-filter
  invertibility across 14/15/28/30-zero runs, and block-layer scan
  reproduction including ESCAPE-coded levels.

- **§5.1.4 PLUSPTYPE picture-header writer, encoder** (round 432):
  `encoder::write_plus_picture_header` + the `PlusModes` mode set emit
  the extended-PTYPE (H.263+) picture header — PSC / TR / PTYPE bits
  1-5 + the `"111"` escape, UFEP `"001"`, the 18-bit OPPTYPE (Annex
  D / F / I / J / K / S / T mode bits, bit-15 emulation guard), the
  9-bit MPPTYPE (I / P picture type, bit-9 guard), CPM = 0, the
  conditional §5.1.9 UUI (`"1"`, Limited) and §5.1.10 SSS fields,
  §5.1.19 PQUANT and §5.1.24 PEI = 0. Unit tests parse every emitted
  variant back through `parse_picture_layer` and pin the full field
  set, including all four SSS submode combinations and the
  PQUANT/PEI tail position.

- **On-wire H.263+ picture encoders** (round 432):
  `encode_intra_picture_plus`, `encode_intra_picture_aic_plus`,
  `encode_intra_picture_aic_mq_plus`, `encode_inter_picture_plus` and
  `encode_inter_picture_umv_plus` emit extended-PTYPE pictures whose
  Annex I / T / D modes are signalled in OPPTYPE (+ §5.1.9 UUI), so
  the streams are **self-describing**: they decode through
  `decode_picture_layer` / `decode_sequence` with
  `DecodeOptions::default()` — no caller-side option flags. The
  macroblock streams are bit-identical to the baseline-PTYPE forms;
  integration tests pin byte-exact reconstruction parity between the
  two header forms and drive a mixed H.263+ I + P + UMV-P elementary
  stream end-to-end.

- **§K.2 slice-header writers, encoder** (round 432):
  `slice_header::write_first_slice_header` (reduced form — SEPB1 +
  MBA + conditional SEPB2/SWI + SEPB3) and
  `slice_header::write_slice_layer` (SSTUF byte alignment + SSC +
  SEPB1 + MBA + conditional SEPB2 + SQUANT + conditional SWI + SEPB3
  + GFID) are the exact inverses of the Annex K parsers, with
  writer-side §K.2.5 MBA / §K.2.7 SQUANT / §K.2.8 SWI range
  validation. Round-trip tests cover all five standard formats, all
  eight SSTUF alignments, the 16CIF SEPB2-mandatory case and the
  Rectangular-Slice SWI form.

- **Annex K Slice-Structured INTRA encoder** (round 432):
  `encode_intra_picture_slices` emits an H.263+ I-picture whose body
  is §K.2 slices — the reduced-header first slice at PQUANT, then
  SSTUF + SSC slices every `mb_rows_per_slice` rows, each with its
  own §K.2.7 SQUANT (per-slice rate control decoupled from the GOB
  grid) — with the SS mode + free-running SSS signalled in PLUSPTYPE.
  Self-describing: decodes via `decode_picture_layer` /
  `decode_sequence` with default options. Tests pin byte-exact
  reconstruction parity with the single-segment encode at constant
  QUANT across five slice heights, verify per-slice SQUANT drives a
  measurable fine/coarse error split (q=2 band MAE < 1.5 vs q=28
  band), and decode a two-picture slice-structured stream.

- **Annex K + Annex I (+ Annex T) slice-structured AIC encoder**
  (round 432): `encode_intra_picture_slices_aic` /
  `encode_intra_picture_slices_aic_mq` encode the AIC (+ MQ) +
  Slice-Structured mode set of the `advanced-intra-coding`
  conformance fixture. The encoder-side `AicEncodeGrid` now records a
  per-macroblock video-picture-segment id and collapses
  cross-segment neighbours to unavailable, mirroring the decoder's
  §I.3 page-78 per-segment availability rule — so every slice's
  DC/AC prediction restarts exactly as the decoder expects. Tests:
  single-slice form reconstructs byte-exactly equal to the
  non-slice on-wire AIC encode, multi-slice AIC / AIC+MQ round-trips
  at MAE < 3.0 across four slice heights, and a flat AIC+MQ
  slice-structured picture is byte-exact.

- **Annex K Slice-Structured INTER encoder** (round 432):
  `encode_inter_picture_slices` encodes a motion-estimated H.263+
  P-picture as §K.2 slices, each slice its own §6.1.1 video picture
  segment — the encoder's predictor grid replays the decoder's
  slice-top `MV2 = MV3 = MV1` rule (row-aligned uniform slices make
  the segment-id check collapse to the same row test), so every MVD
  reconstructs exactly. The shared motion encoder now takes an
  `InterFraming` (single segment / GOB headers / slices). Tests:
  translated-content P-slices at MAE < 4.0 for three slice heights,
  byte-exact single-slice parity with the non-slice H.263+ P-form,
  and a slice-structured AIC-I + P + P GOP through `decode_sequence`.

- **RFC 4629 RTP payload format** (round 432): the new `rtp` module
  stages the H.263+ payload level — the §5.1 `RR|P|V|PLEN|PEBIT`
  payload header (writer + parser, RR ignored on receive per the
  spec), the §5.2 VRC extension, `packetize_stream` (P=1 segment
  packets at byte-aligned Picture/GOB/Slice/EOS start codes with the
  two zero bytes stripped, preferring the last in-budget start-code
  boundary as the cut, §6.2 P=0 Follow-on fallback for oversized
  segments) and `depacketize_payloads` (byte-exact reassembly,
  redundant PLEN headers discarded). Tests: header field round-trips
  incl. VRC / 63-byte PLEN / error paths, GOP + GOB + slice streams
  byte-exact across budgets 32..4096 with decoded-frame equality,
  GBSC/SSC cut preference (all-P=1 splits), and five vendored
  conformance fixtures byte-exact at three budgets each.

- **RFC 4629 §6.1.2 redundant picture headers** (round 432):
  `rtp::redundant_picture_header` extracts the current picture's
  header (the `"100000"` PSC tail through the PEI/PSUPP loop, exact
  PEBIT), refusing incomplete `UFEP = "000"` headers per §6.1.1;
  `PacketizeConfig::attach_picture_header` attaches it (`PLEN > 0`)
  to every GOB / slice segment packet for loss resilience, and
  `rtp::assemble_picture_header` reconstitutes a parseable
  byte-aligned header on the receive side. Tests re-parse the
  attached copy of every GOB packet and pin field-exact equality
  with the primary header (baseline + H.263+ slice streams), with
  reassembly still byte-exact.

- **RFC 2190 legacy `video/H263` Mode A** (round 432): the four-byte
  §5.1 Mode A payload header (write + parse; Mode B/C recognised and
  refused), `packetize_stream_rfc2190` (GOB/picture-boundary-only
  cuts with the start code carried in full, per-picture
  `SRC`/`I`/`U`/`S`/`A` PTYPE mirrors and PB-frame `DBQ`/`TRB`/`TR`
  fields extracted from each picture header, H.263+ and oversized
  Mode-B-territory segments refused) and
  `depacketize_payloads_rfc2190`. Both packetizers (RFC 4629 and
  2190) now enforce the **hard picture-boundary rule** — a packet
  never runs past the next PSC / EOS, so every coded frame begins
  its own segment packet (RFC 4629 §7 / §6.1.3) — with a test
  asserting PSC-count == picture-packet-count on every split.

### Fixed

- **§F.2 intra-macroblock candidate predictors, decoder** (round 384):
  the four-vector predictor derivation now threads each
  just-reconstructed vector back into the Figure-F.1 neighbourhood, so
  B2's MV1 reads B1's actual vector, B3's MV2/MV3 read B1's/B2's, and
  B4's MV1/MV2 read B3's/B2's — previously those intra-macroblock
  candidate cells stayed zero, mis-reconstructing every INTER4V
  macroblock whose first blocks carry non-zero vectors. Regression: an
  INTER4V MB with MVD1 = (+4,0) and MVD2-4 = 0 reconstructs all four
  vectors as (+4,0) via the propagated medians.

- **§F.3 OBMC conformance, decoder** (round 384) — two deviations
  in the Advanced-Prediction reconstruction corrected. (1) The OBMC
  right-half remote vectors of blocks B2 / B4 now use the **actual
  motion vector of the macroblock to the right**: since that
  macroblock is parsed later in the bitstream, the luminance
  reconstruction of every coded AP INTER macroblock is deferred one
  macroblock (`PendingApLuma` + `reconstruct_pending_ap_luma`, flushed
  once the right neighbour's grid entry is recorded or at the end of
  the macroblock row); previously the right remote silently resolved
  to a zero vector. (2) **Single-MV** coded INTER macroblocks in AP
  pictures are now OBMC-predicted too (§F.2: "defined as four vectors
  with the same value"); previously only INTER4V macroblocks were
  blended. Regression tests pin both against the pure
  `obmc_predict_block` oracle, and a full-range sweep shows the §F.2
  sum-of-four chroma derivation equals the Table-18 single-MV rule for
  equal vectors (so chroma is unchanged). Chrominance (no OBMC, §F.2)
  and Deblocking-Filter-only four-vector prediction (Table J.1: OBMC
  off) keep reconstructing immediately.

### Added

- **Annex G PB-frames encoder** (round 384):
  `encoder::encode_pb_picture` encodes a P-picture and a B-picture as
  one Annex G PB-frame unit (`PbConfig`: quant / TRB / DBQUANT /
  search window). The P-part is motion-estimated and coded like the
  baseline P-path; the encoder then reconstructs PREC (§G.5) exactly
  as the decoder will (dequantising its own coefficients through the
  shared reconstruction), forms the §G.4 TRB/TRD-scaled bidirectional
  B-prediction with `MVDB = 0` via the decoder's own
  `pb_b_predict_macroblock`, and codes surviving B-residuals at the
  Table-6 BQUANT (MODB `"11"` + CBPB; MODB `"0"` otherwise; fully
  static macroblocks skip). A static PB-frame is lossless on both
  parts; a translating (ref, B, P) triple round-trips within tolerance
  and an I + PB stream decodes through `decode_sequence` into display
  order [I, B, P]. The picture-header writer emits the §5.1.22 TRB +
  §5.1.23 DBQUANT fields when the PTYPE bit-13 PB flag is raised.

- **Closed-loop GOP sequence encoder + §5.1.27 EOS** (round 384):
  `encoder::encode_sequence` encodes a frame sequence with a classic
  I + P GOP structure (`GopConfig`: quantiser, intra period, search
  window, optional Annex D UMV P-pictures, optional EOS): every
  P-picture predicts from the **decoded reconstruction** of the
  previous picture (the encoder decodes its own output picture by
  picture), so the prediction reference is bit-identical to a
  conformant decoder's and long sequences accumulate no drift — a
  six-frame translating GOP shows the last P-frame no worse than the
  first. `GopConfig::eos` appends the byte-aligned §5.1.27 End Of
  Sequence codeword (`EOS_BYTES`), transparent to `decode_sequence`.

- **Annex F Advanced-Prediction (INTER4V + OBMC) encoder** (round
  384): `encoder::encode_inter_picture_ap` encodes a P-picture with the
  PTYPE bit-12 AP flag and **four motion vectors per macroblock**
  (§5.3.8 / §F.2), predicted through the §F.3 overlapped block motion
  compensation blend the decoder reconstructs with. Two passes:
  per-block estimation with §F.2 / Figure-F.1 predictor replay
  (`encoder_motion::Mv4Grid`, including the intra-macroblock candidate
  threading, + the 8×8 `estimate_block_motion` search), then — with
  the full motion field known — the exact decoder-side OBMC prediction
  per block (right-half remotes read the macroblock to the right) and
  residual coding. Chroma uses the §F.2 / Table-F.1 sum-of-four
  vector. `encoder_mb::encode_inter4v_macroblock` emits the Table-8
  INTER4V form with four MVD pairs. A static AP picture round-trips
  **losslessly**; divergent intra-macroblock motion (sheared halves)
  round-trips within tolerance at a fraction of the zero-motion bits.

- **§5.2 GOB-header emission (resync-friendly streams + per-GOB
  GQUANT)** (round 384): `encoder::encode_intra_picture_gobs` encodes
  an I-picture with a GOB header (GSTUF byte alignment + GBSC + GN +
  GFID + GQUANT) on every GOB after the first, driving a **per-GOB
  quantiser map** — unlike §5.3.6 DQUANT, GQUANT may jump anywhere in
  `1..=31` between GOBs (coarse rate control with resynchronisation
  points). `encoder::encode_inter_picture_gobs` does the same for
  motion-compensated P-pictures; `MvGrid::with_gob_headers` replays
  the decoder's §6.1.1 rule-3 per-GOB predictor segmentation (MV2/MV3
  collapse onto MV1 at every GOB top row) so each MVD still
  reconstructs exactly. A same-quant GOB-header I-picture reconstructs
  **identically** to the single-segment encode.

- **Annex D UMV INTER-picture encoder** (round 384):
  `encoder::encode_inter_picture_umv` encodes a P-picture in the
  Unrestricted Motion Vector mode (PLUSPTYPE absent): the PTYPE bit-10
  flag is raised and each macroblock's vector is estimated over the
  extended §D.2 `[-31.5, 31.5]`-pixel range (with §D.1 over-boundary
  referencing — encoder prediction and decoder reconstruction share
  the same edge-replicated sampling). A 20-pixel translation — beyond
  the default ±16-pixel range — round-trips with luma MAE < 8 and a
  smaller stream than the default-mode encoder needs for the same
  content; a static UMV P-picture stays all-skipped and lossless. The
  picture header writer gained a `PtypeFlags` carrier for the
  optional-mode PTYPE bits.

- **Annex D §D.2 UMV motion-vector coding, encoder side** (round 384):
  `encoder_motion::umv_mvd_component_for` / `umv_mvd_for` compute the
  Table-14 MVD the encoder must emit so the decoder's §D.2
  predictor-dependent pair selection reconstructs exactly the chosen
  extended-range vector — verified exhaustively against
  `reconstruct_mv_component_umv` over the full `[-63, 63]²`
  predictor × vector space (reachable pairs round-trip, unreachable
  ones are refused). `estimate_motion_umv` runs the SAD search over
  the extended `[-31.5, 31.5]`-pixel window with a per-candidate §D.2
  reachability filter, so every returned vector is codable; the
  default-mode `estimate_motion` is now a thin wrapper over the shared
  search core.

- **DQUANT (+Q) macroblock encoding** (round 381): the §5.3.6 Table 13
  baseline 2-bit DQUANT differential is now emitted by the encoder.
  `encoder_vlc::write_dquant` writes the `{-2,-1,+1,+2}` differential;
  `encode_intra_macroblock_dq` / `encode_inter_macroblock_dq` select the
  INTRA+Q / INTER+Q macroblock types and place the DQUANT field after
  CBPY (before MVD for INTER) per Figure 9. The new picture-layer entry
  `encode_intra_picture_dquant` drives a per-macroblock quantiser map
  through DQUANT (the foundation of rate control: it ramps QUANT in legal
  one-step increments across the raster), round-trip-verified through the
  decoder. New `Error::BadDquant` for out-of-range differentials.

- **INTRA/INTER mode decision in P-pictures (intra refresh)** (round 376):
  `encode_inter_picture_motion` now compares the motion-compensated INTER
  residual SAD against the macroblock's own intra AC energy (variance about
  the per-block mean) and codes the macroblock INTRA (Table 8 INTRA MCBPC,
  with COD) when intra is cheaper — the classic H.263 intra-refresh
  heuristic. A P-picture whose source is unrelated to the reference falls
  back to INTRA and reconstructs the source (not the reference) within the
  INTRA tolerance, exercising the INTRA-in-P bitstream path end-to-end.

- **Motion estimation + motion-compensated INTER encoder** (round 376):
  new `encoder_motion` module with `estimate_motion` (SAD over an integer
  window + half-pel refinement, biased toward the median predictor),
  `MvGrid` (replays the decoder's §6.1.1 median-predictor border rules for
  the single-segment stream so the emitted MVD reconstructs to exactly the
  chosen MV), and `mvd_for` (inverts the §6.1.1 wrap; round-trip verified
  through `reconstruct_mv` over the full predictor × MV grid).
  `encoder::encode_inter_picture_motion` wires these in: each macroblock's
  luma MV is estimated, the residual is taken against the
  motion-compensated prediction (bit-identical to the decoder's, chroma via
  Table 18), coded, and the MV grid updated. A translated frame
  reconstructs with luma MAE < 6 and produces a **smaller** stream than the
  zero-motion encoder on the same content.

- **Baseline INTER (P-) picture encoder, zero motion** (round 376):
  `encoder::encode_inter_picture` predicts from the previous
  reconstructed frame with `MVD = 0`. Because the encoder emits a single
  video-picture segment, the §6.1.1 median predictor over an all-zero-MV
  neighbourhood is zero, so the decoder reconstructs `MV = 0` and the
  prediction is the co-located reference block. The residual is forward
  DCT'd, dead-zone-quantised and coded per block; macroblocks whose
  residual quantises away are emitted as skipped (COD = 1). A static
  P-frame (source == reconstructed reference) reconstructs **bit-exactly**
  (all-skipped); a brightened frame round-trips with luma MAE < 10. True
  motion estimation is the next milestone.

- **Baseline INTRA-picture encoder** (round 376): `encoder::encode_intra_picture`
  turns a planar 4:2:0 `YuvFrame` into a complete byte-aligned baseline
  H.263 I-picture — §5.1 header (PSC, TR, PTYPE all-baseline, PQUANT,
  CPM=0, PEI=0) + §5.2.2 GOB-0-elided single-segment macroblock stream +
  §5.1.28 PSTUF byte alignment. The output decodes end-to-end through the
  crate's own `decode_picture_no_gob0_header`; a flat-grey QCIF frame
  reconstructs bit-exactly (128→DC 1024→128), and a gradient QCIF frame at
  QUANT=4 reconstructs with luma MAE < 8 and max error ≤ 40. sub-QCIF /
  QCIF / CIF dimensions verified; non-standard dimensions and
  out-of-range QUANT are rejected. Composes the `encoder_mb` (§5.3),
  `encoder_block` (§5.4), `fdct` and `encoder_vlc` layers.

- **Macroblock-layer encoder** (round 376): `encoder_mb` —
  `encode_intra_macroblock` / `encode_inter_macroblock` /
  `encode_skipped_macroblock` assemble the §5.3 header (COD, MCBPC, CBPY
  with INTER complement, MVD) + §5.4 six-block payload in the decoder's
  exact block order (Y1..Y4, Cb, Cr) and CBP-bit gating. Header
  round-trips through `parse_macroblock`.

- **Block-layer encoder** (round 376): `encoder_block` —
  `encode_intra_block` / `encode_inter_block` plus the §5.4.2 TCOEF
  run-length coder (`tcoef_events`, bit-exact inverse of the decoder's
  RUN/LEVEL loop) and the INTRADC + TCOEF emitters
  (`write_intra_block` / `write_inter_block_coeffs`).

- **Forward DCT + forward quantisation** (round 376): `fdct` — the
  orthonormal forward DCT that inverts the decoder's IDCT (±1 LSB on an
  unquantised round-trip), the conventional dead-zone forward quantiser
  `|L| = floor(|F| / (2·QUANT))`, and `quantise_intradc` (nearest legal
  Table 15 reconstruction level). Round-trips through §6.2.1 dequant to
  within ~2·QUANT.

- **Encoder VLC primitives** (round 376): new `encoder_vlc` module with
  the emit-direction inverse of every baseline decode table — `write_mcbpc_i`
  / `write_mcbpc_p` (§5.3.2 Tables 7/8), `write_cbpy` (§5.3.5 Table 12),
  `write_mvd_component` (§5.3.7 Table 14), `write_intradc` (§5.4.1 Table 15)
  and `write_tcoef` (§5.4.2 Table 16, dedicated VLC + ESCAPE form). Each is
  round-trip verified against the existing decoder over its full symbol
  range (all 24 P-picture MCBPC code-points, all 16 CBPY patterns, the full
  −32..+31 MVD range, every legal INTRADC reconstruction level, and
  representative TCOEF VLC / ESCAPE / multi-event blocks). The first
  encoder building block in this decode-only crate.

- **Multi-picture PB-frame reference-threading test** (round 371): an
  I + PB + P stream decodes to four frames [I, B, P, P2], asserting the
  PB-frame's P-part (not its display-only B-part) becomes the prediction
  reference for the following P-picture and `prev_tr` advances to the
  P-part's TR. Added an all-skipped baseline-INTER real-wire picture
  builder for the test.

- **Deblocking-Filter-mode four motion vectors per macroblock** (round
  371): per §5.3.8 / Table J.1, Annex J Deblocking Filter mode "includes
  the ability to use four motion vectors per macroblock" without the §F.3
  OBMC element. `MbContext` gained a `deblocking_filter` field and
  `MbType::has_mvd2_4` now takes both the AP and DF flags, so an INTER4V /
  INTER4V+Q macroblock carries MVD2-4 when **either** Advanced Prediction
  **or** Deblocking Filter mode is active. The picture / slice-structured
  drivers set `deblocking_filter: options.deblock`, and
  `decode_inter4v_macroblock` selects between the §F.3 OBMC prediction
  (AP on) and plain per-block half-pel motion compensation (DF-only); the
  §F.2 four-vector median-predictor derivation, §D.1 edge replication and
  Table F.1 chroma vector are shared. **API change:** `MbContext` literals
  must now set `deblocking_filter`. New test: a DF-mode INTER4V macroblock
  with all-zero MVDs reproduces the reference at the top-left macroblock
  (and confirms the parser reads four MVDs under DF mode without AP).

- **Annex M Improved PB-frames decode through `decode_sequence`** (round
  371): the extended-PTYPE streaming dispatch now detects an Improved-PB
  picture (§5.1.4.3 MPPTYPE picture-type "010") via `extended_is_improved_pb`
  before the single-frame `decode_picture_layer_with_inherited` driver
  (which refuses it) runs, and routes it to a new
  `decode_improved_pb_picture_with_inherited` that threads §5.1.4.4
  inherited-state and returns the decoded (P, BPB) pair plus the next
  picture's OPPTYPE snapshot. The pair is spliced into the output in
  display order (the BPB-picture before the P-picture); only the P-part
  advances the reference and the §G.4 TR. The existing
  `decode_improved_pb_picture` now delegates to the inherited-aware entry
  with the default snapshot (no behaviour change). New tests:
  `decode_sequence` of an I-frame + all-skipped Improved-PB frame yields
  three display-order frames [I, BPB, P]; the inherited-aware entry matches
  the plain entry and surfaces the UFEP=001 QCIF source-format snapshot.

- **Annex G PB-frames decode through `decode_sequence`** (round 371): the
  headline multi-picture streaming entry point now routes a baseline-PTYPE
  INTER picture that signals PB-frames mode (PTYPE bit 13) through a new
  real-elementary-stream PB driver and splices the decoded (B, P) pair into
  the output in display order — the B-picture *before* the P-picture, with
  only the P-part advancing the prediction reference and the §G.4 TR.
  `decode_pb_picture_no_gob0_header` consumes the spec-conformant baseline
  picture-header tail a real encoder emits for a PB-frame — §5.1.19 PQUANT,
  §5.1.20 CPM (the "1" Annex-C branch refused), §5.1.22 TRB, §5.1.23
  DBQUANT, §5.1.24/§5.1.25 PEI/PSUPP — then elides the group-number-0 GOB
  header (§5.2.2) and treats every later GOB header as optional (§5.2),
  exactly the framing the baseline non-PB streaming path uses (where the
  existing `decode_pb_picture` instead used the per-layer test convention:
  no PQUANT/CPM in the header and a mandatory header on every GOB). The TR
  of every decoded reference (baseline I/P, extended P, or the PB P-part)
  is tracked across the sequence loop so a following PB-frame scales its
  §G.4 B-vectors against the correct anchor. New tests: the real-wire PB
  driver reproduces the reference in both parts for an all-skipped frame,
  rejects a zero PQUANT, and `decode_sequence` of an I-frame + all-skipped
  PB-frame yields three display-order frames `[I, B, P]`.

- **Annex N §N.4.1 per-slice Reference Picture Selection through the
  Annex K Slice-Structured driver** (round 366): the slice driver now
  parses the §N.4.1 slice-layer NEWPRED fields (Figure N.3) after each
  subsequent slice header and re-selects that slice's prediction
  reference from the §N.5 store, exactly as the GOB path does. The first
  (reduced-header) slice after the Picture Start Code carries no NEWPRED
  fields and keeps the picture-layer §N.5 reference (parallel to GOB 0).
  The driver gained an `decode_slice_structured_after_header_inner` body
  carrying the optional per-segment context (the public wrapper passes
  `None`, no behaviour change for non-RPS slice callers).
  `decode_picture_layer_rps` builds the context for INTER-pictures on
  both the GOB and the Slice-Structured branch. New end-to-end test: a
  two-slice QCIF SS+RPS INTER picture where slice 0 predicts from the
  most recent anchor and slice 1's NEWPRED re-selects the older one —
  the decoded macroblocks of each slice copy their respective anchors.

- **Annex N §N.4.1 per-GOB Reference Picture Selection now reaches
  pixels end-to-end** (round 366): `decode_picture_layer_rps` threads the
  §N.4.1 GOB-layer NEWPRED fields through the GOB-walking driver for
  INTER-pictures. After each GOB header (Figure N.2) the driver parses
  TRI / TR / TRPI / TRP + BCI and, when the GOB's TRP re-selects a
  different stored reference, predicts that GOB's macroblocks from it
  "instead of the last decoded picture" (§N.5) — a header-less GOB keeps
  the previous segment's reference ("TRP is valid until the next PSC, GSC
  or SSC", §N.4.1.4) and GOB 0 (header elided, §5.2.2) stays on the
  picture-layer §N.5 selection. The §N.5 forced-INTRA-update case (a
  per-GOB TRP not in the store) surfaces as `Error::NotImplemented`. The
  driver gained an internal `RpsGobContext` and an
  `decode_after_picture_header_inner` body; every legacy / baseline / PB
  / extended caller passes `None` (no behaviour change). Two end-to-end
  tests: a QCIF INTER picture where GOB 5 re-selects the older anchor
  mid-picture (decoded frame carries anchor B in GOB 0's row and anchor A
  in GOB 5's row, proving the switch reached pixels), and the
  missing-per-GOB-TRP refusal. The existing picture-layer RPS test buffer
  was updated to emit the now-mandatory NEWPRED GOB fields.

- **Annex N §N.4.1 GOB/slice-layer NEWPRED field parser** (round 366):
  new `annex_n::parse_gob_newpred_fields` decodes the per-segment
  Reference Picture Selection fields appended to a GOB or slice header
  when Annex N is in use (Figure N.2 / N.3): §N.4.1.1 TRI + §N.4.1.2 TR
  (8 bits, or 10 with a custom picture clock frequency), §N.4.1.3 TRPI +
  §N.4.1.4 TRP (10 bits), and the §N.4.1.5 BCI codeword (`"01"` accepted;
  a present `"1"` back-channel message or the undefined `"00"` shape
  surfaces the new `Error::BadBackChannelMessage`). The §N.4.1.3 rule
  that TRPI must be `0` on an I/EI picture is enforced. New public
  `GobNewpredFields` struct (with `segment_trp()`) + `NEWPRED_TRP_BITS`
  constant. Nine unit tests cover the field-presence matrix, the
  custom-PCF 10-bit TR width, the TRPI-on-INTRA / BCI-`"1"` / BCI-`"00"`
  rejections, and a truncated-buffer EOF.

- **Annex T Modified Quantization + Annex S Alternative INTER VLC now
  thread through the Annex K Slice-Structured driver** (round 362): the
  slice macroblock walker passes `options.modified_quant` into its
  `MbContext` and routes the §S.2 / §S.3 Alternative INTER VLC through
  the shared `decode_one_macroblock`, so the §T.3 QUANT_C chroma step +
  §T.4 EXTENDED-ESCAPE range and the §S handling apply on slice-walked
  macroblocks. The PLUSPTYPE shim no longer refuses `MQ + slice` or
  `AIV + slice` (or `AIV + MQ`). Two new byte-exact conformance fixtures:
  `advanced-intra-coding` (AIC + MQ + slice, single QCIF keyframe) and
  `alt-inter-vlc` (AIV + AIC + MQ + slice, QCIF I+P+P), each with a
  reference-YUV SHA-256 corruption guard.

### Fixed

- **Annex K Slice-Structured mode now decodes real H.263+ streams**
  (round 362): `decode_slice_structured_after_header` was reading the
  §5.1.24 PEI bit that closes the picture header as the first slice's
  §K.2.3 SEPB1, failing with `BadSliceEmulationPreventionBit`. The driver
  now consumes the §5.1.24 PEI / §5.1.25 PSUPP loop after PQUANT and
  before the first reduced slice header. The vendored
  `slice-structured-mode` QCIF I+P+P conformance fixture now decodes
  byte-exact within the Annex A.7 ±1 IDCT tolerance. (The synthetic
  slice-structured test buffers were updated to write the PEI bit.)

### Added

- **H.263+ (PLUSPTYPE) elementary streams now decode through
  `decode_sequence`** (round 362): the multi-picture demuxer detects the
  §5.1.3 extended-PTYPE source-format selector (`"111"`) per picture and
  routes extended pictures through `decode_picture_layer_with_inherited`,
  threading the §5.1.4.4 / §5.1.4.5 inherited extended-mode state forward
  (a baseline picture resets it). Baseline pictures keep the §5.2.2
  GOB-0-elided path. Three new end-to-end conformance fixtures
  (`tests/fixtures/{qp-high,qp-low,h263p-modern}`) decode within the
  Annex A.7 ±1 IDCT tolerance, with their reference-YUV SHA-256s guarded
  against corruption.
- **§5.1.19 PQUANT + §5.2.2 GOB-0 header elision on the extended
  GOB / RPS decode paths** (round 362): `decode_picture_layer` (GOB
  branch) and `decode_picture_layer_rps` now read the picture-layer
  PQUANT that follows the PLUSPTYPE / CPFMT / RPRP block (Figure 6
  part 1) and the §5.1.24 PEI / §5.1.25 PSUPP loop, then decode the
  header-less group-number-0 GOB at QUANT = PQUANT — matching the wire a
  real H.263+ encoder emits (it omits the GOB-0 header). The synthetic
  PLUSPTYPE test buffers were updated to the spec-conformant layout
  (PQUANT after CPM, GOB 0 elided).
- **Custom Picture Clock Frequency (§5.1.7 / §5.1.8) no longer refused**
  on the PLUSPTYPE decode path (round 362): CPCFC + ETR are fully framed
  by the picture-layer parser and their semantics are timing-only — the
  wider temporal reference feeds only §G.4 PB-frame scaling and Annex N
  reference selection, neither reachable on the GOB / slice decode path —
  so a custom-PCF picture decodes to the same pixels as a standard-PCF
  one.

- Annex N **Reference Picture Selection (RPS)** forward-channel decode
  (round 347): new `annex_n` module + `decode_picture_layer_rps` entry
  decode an RPS ("NEWPRED") stream that predicts each picture from a
  chosen previously-decoded reference rather than always the most recent
  anchor.
  - `RpsReferenceStore` is the §N.5 decoder picture memory keyed by the
    10-bit Temporal Reference (ETR ∥ TR via `compose_tr`), with
    first-in-first-out eviction at capacity and same-TR replacement.
  - `select_reference` implements the §N.4.1.4 / §N.5 rule: when
    TRPI = 1 the stored picture whose TR equals TRP is the reference;
    when TRP is absent the most recent anchor is used "as when not in
    the Reference Picture Selection mode". A TRP referencing an absent
    picture surfaces as the §N.5 forced-INTRA-update case.
  - `decode_picture_layer_rps` parses the picture, selects the §N.5
    reference from the store, decodes against it (RPS now permitted
    through the shim via the new `allow_rps` flag), and stores the
    decoded anchor under its TR. An end-to-end test seeds two distinct
    INTRA anchors (TR=10 / TR=20) and shows a TRP=10 RPS INTER-picture
    reconstructs a copy of the *older* TR=10 anchor through pixels.
  - The §N.4.2 back-channel (BCM ACK/NACK) messages are explicitly out
    of scope — they flow decoder → encoder and do not affect
    forward-channel pixels; the §5.1.16 BCI codeword is still framed by
    `plus_ptype` (a present BCM refused).
- Annex P **Reference Picture Resampling (RPR)** resampling engine
  (round 347): new `annex_p` module implements the §P.3 / §P.4.2
  integer warp that resamples the previous decoded reference picture
  into a "warped" reference before motion compensation.
  - `resample_plane` / `resample_yuv` compute the §P.3 corner
    displacements from the eight warping parameters in 1/32-pixel
    luminance accuracy, derive the §P.3 virtual-frame `H' / V'` powers
    of two, bilinearly extrapolate the LT/RT/LB/RB virtual-point
    displacements (`//` round-half-away-from-zero), and run the §P.4.2
    raster loop with the four-tap bilinear `filter` and the §P.2.3 fill
    mode (clip / black / gray / color) plus §P.3 `RCRPR` rounding
    control.
  - `read_table_d3` decodes the §D.3 reversible Table-D.3 VLC
    (range -4095..=4095), verified against the §D.3 worked `-13`
    example; `parse_rprp` parses the §P.2 `RPRP` picture-header field
    (WDA, eight Table-D.3 warping parameters with §P.2.2 pair emulation-
    prevention bits, fill mode, optional §P.2.4 fill colour).
  - `RprParams::implicit` builds the §P.1 implicit parameter set (zero
    warping, clip fill, 1/16-pixel accuracy) for the predictively-
    encoded resolution-change case.
  - **End-to-end implicit RPR**: `decode_picture_layer` now resamples a
    size-mismatched reference before motion compensation when PLUSPTYPE
    is present, the picture is an INTER-picture, and the RPR mode bit is
    off (the §P.1 implicit resolution-change case) — previously such a
    size mismatch was refused with `NotImplemented`. The new
    `maybe_implicit_resample` helper warps the reference to the current
    picture's size using `RprParams::implicit` (RCRPR = the picture's
    RTYPE bit) and feeds the warped frame to the GOB / slice driver. An
    all-skipped QCIF INTER-picture decoded against a sub-QCIF reference
    reconstructs to the exact `resample_yuv` output.
  - **End-to-end explicit RPR**: the §5.1.18 RPRP picture-header field is
    now parsed (was refused with `PlusPtypeUnsupported`) for INTER / B /
    Improved-PB pictures into `PlusPtypeHeader::rprp`, and
    `decode_picture_layer` warps the reference with those explicit
    parameters before motion compensation via the new `explicit_resample`
    helper. The EP-picture explicit-RPR case (§P.2.2 lower-layer
    refinement) remains refused. An all-skipped QCIF INTER-picture
    signalling RPR-on with an all-zero-warp / clip RPRP field
    reconstructs to the exact `resample_yuv` output.
- Annex O §O.1.2 **EP-picture** ("Enhancement" P-picture) end-to-end
  reconstruction (round 343):
  - New `decode_ep_picture` + `decode_ep_picture_layer` decode the
    forward + upward predicted enhancement-layer picture against two
    references — `forward_ref` (the previous same-layer EI/EP picture)
    and `upward_ref` (the temporally-simultaneous reference-layer
    picture). All four Table-O.2 prediction types reconstruct: Forward
    (motion-compensated from the same-layer reference with an MVDFW
    vector reconstructed against the §O.5.1 forward-only median
    predictor), Upward (co-located reference-layer block, no vector),
    Bi-dir (per-pixel truncating average of the forward and upward
    predictions, §O.4), and INTRA (§6.2).
  - Figure-O.6 macroblock field order `COD MBTYPE CBPC CBPY DQUANT
    MVDFW Block` is honoured, with CBPY / DQUANT / MVDFW each gated on
    the resolved `ScalabilityMbHeader` presence flags; the §O.4.4 CBPY
    INTRA/INTER column selection is applied per prediction type.
  - Shared `BlockPredictor` trait (Upward / Forward / Bidir) and the
    `reconstruct_{intra,inter}_*_macroblock` helpers factor the
    enhancement-layer block reconstruction so the EI driver and the
    forthcoming B-picture driver share one code path.
  - 6 new `picture` tests cover Forward-skipped (copies the forward
    reference), Upward-no-texture (copies the upward reference),
    Forward-with-zero-MV, Bi-dir averaging, INTRA-DC, and the
    reference-geometry guard.

- §5.1.11–§5.1.16 PLUSPTYPE scalability / reference-picture-selection
  header fields (round 336):
  - `parse_plus_ptype` now frames the §5.1.11 ELNUM (4 bits, present for
    any layered B/EI/EP picture), §5.1.12 RLNUM (4 bits, UFEP=001 only),
    §5.1.13 RPSMF (3 bits, RPS + UFEP=001; reserved `000`–`011` rejected),
    §5.1.14 TRPI (1 bit, RPS; forced 0 on I/EI per §5.1.14), §5.1.15 TRP
    (10 bits, present iff TRPI=1), and §5.1.16 BCI (`"1"` / `"01"`)
    fields in their Figure-8 order, instead of refusing the whole header.
  - A `BCI == "1"` (videomux §5.1.17 BCM follows) and the §5.1.18 RPRP
    (Annex P) variable-length payloads remain `Error::PlusPtypeUnsupported`
    — those layouts are externally-negotiated and not staged.
  - New public `Rpsmf` enum and `PlusPtypeHeader::{elnum, rlnum, rpsmf,
    trpi, trp}` fields; `InheritedExtendedState` gains
    `reference_picture_selection` so a UFEP=000 follow-up frames its RPS
    fields.
  - `decode_picture_layer` keeps refusing B/EI/EP layered-MB decode and
    now also refuses an RPS-in-use picture (the §5.1.15 TRP multi-
    reference lookup is a stream-level concern the single-picture API
    does not manage).
  - 11 new `plus_ptype` tests cover the full-update and inherited-UFEP=000
    field-presence matrix plus the RPSMF-reserved / TRPI-on-INTRA /
    BCI-`"1"` / BCI-`"00"` rejections.
- §5.2.2 first-GOB (group-number-0) header elision (round 330):
  - New `decode_picture_no_gob0_header` baseline entry point honours the
    §5.2.2 rule that the first GOB of every picture (group number 0)
    carries **no** GOB header — "as group number 0 is used in the PSC".
  - It reads the §5.1.19 PQUANT (5-bit, range 1..=31) and §5.1.20 CPM
    fields that follow PTYPE in the non-extended picture header, decodes
    the header-less GOB 0 at QUANT = PQUANT, and parses a GBSC + GN +
    GFID + GQUANT header only for GOBs 1..N.
  - CPM = "1" (the §5.1.20 / Annex C multiplex with its trailing PSBI
    and per-GOB GSBI) is refused (`Error::NotImplemented`); PQUANT = 0 is
    rejected (`Error::InvalidQuantiser`).
  - The internal `decode_after_picture_header` driver gained an optional
    `gob0_pquant` argument: `Some(pquant)` elides the GOB-0 header,
    `None` preserves the legacy convention (every GOB carries a header on
    the wire) the existing `decode_picture` / `decode_picture_layer` / PB
    / Annex-K-slice fixtures are built around.
  - Five tests: a uniform-frame decode, a byte-for-byte match against the
    legacy every-GOB-header layout, PQUANT-driven GOB-0 dequant, and the
    PQUANT-0 / CPM-on rejections.
- Annex Q.7 Reduced-Resolution Update block boundary filter (round 326):
  - New `rru_filter` module implements the §Q.7 filter run along the
    edges of the 16×16 reconstructed blocks (not the 8×8 edges of the
    baseline §J.3 filter).
  - **§Q.7.1** — `RruFilterMode::Default` applies the two-tap kernel
    `A1 = (3·A + B + 2)/4`, `B1 = (A + 3·B + 2)/4` (truncating division)
    across the boundary, exposed standalone as `rru_default_tap`.
  - **§Q.7.2** — `RruFilterMode::Deblocking` reuses the §J.3 four-tap
    deblocking filter with `STRENGTH = +∞` (the published
    `deblock::STRENGTH_RRU_INFINITE` sentinel), which collapses
    `UpDownRamp(x, ∞)` to the identity so `d1 = (A−4B+4C−D)/8` and
    `d2 = clipd1((A−D)/4, d1/2)`, exactly as §Q.7.2 restates it.
  - `rru_filter_plane` is the plane-level driver: it honours the §Q.7
    /§J.3 edge ordering (every horizontal edge first, then every
    vertical edge), the coded-MB filter-on condition, and the
    picture-edge skip; slice (Annex K) and ISD-segment (Annex R) skips
    are surfaced through a per-edge `RruEdgeCondition` closure, mirroring
    `deblock::deblock_plane`.
  - Out of scope (reported, not guessed): the surrounding RRU
    reconstruction (32×32 macroblock layer, §Q.4 pseudo-MV, §Q.5
    enlarged OBMC, §Q.3 reference extension) is not yet wired, so the
    §Q.7 driver is a pure primitive not yet invoked end-to-end.
  - Tests: ten `rru_filter` unit tests (two-tap kernel formula, default
    filter on stepped/constant planes for vertical + horizontal
    boundaries, skip-condition suppression, picture-edge skip on a
    single-block plane, the §Q.7.2 vertical-edge worked example, and the
    two-pass ordering guard). `cargo test -p oxideav-h263`: 594 (was
    576 reported; +18 over the prior count).

- Annex S Alternative INTER VLC mode (round 318):
  - **§S.2** — `block::parse_inter_block_alt_inter_vlc` decodes each
    INTER coefficient block per §S.2.2: the codewords are interpreted
    with the baseline INTER VLC (Table 16) first, and only when the
    INTER interpretation would address coefficients past slot 63 of the
    block (the `BadTcoefRunOverflow` signal — step 3) the reader is
    rewound to the block's first bit and the same codewords are
    re-decoded with the Annex I Table I.2 (INTRA) interpretation. Both
    tables share one codeword inventory (§I.3 / §S.2), so the re-decode
    consumes exactly the same bits. Applied to every INTER block of the
    macroblock, luma and chroma.
  - **§S.3** — when both chrominance blocks of an INTER macroblock carry
    coefficients (`CBPC5 = CBPC6 = 1`), the CBPY codeword is the Table 12
    INTRA pattern (no INTER complement).
  - `DecodeOptions::alt_inter_vlc` gates the mode; the §5.1.4 PLUSPTYPE
    shim now auto-activates it from the OPPTYPE Alternative-INTER-VLC bit
    (bit 13) instead of refusing the picture, and ORs it into the
    caller's options like the AIC / DF / MQ wire bits.
  - Out of scope (reported, not guessed): AIV combined with Advanced
    Prediction / INTER4V (Annex F), PB / Improved PB-frames (Annex G / M),
    Slice-Structured (Annex K) or Modified Quantization (Annex T) — the
    shim refuses those combinations because the §S re-decode and §S.3
    CBPY orientation thread only through the baseline single-MV INTER
    reconstruction path.
  - Tests: three `block` unit tests (INTER-in-range, INTRA-reinterpret-on-
    overflow, baseline-INTER-overflows-on-same-stream) and one
    `decode_picture_layer` end-to-end test on a PLUSPTYPE AIV P-picture
    (previously refused). `cargo test -p oxideav-h263`: 576 (was 572).

- Annex T Modified Quantization mode combined with Annex I Advanced
  INTRA Coding (round 315):
  - The §5.1.4 PLUSPTYPE shim no longer refuses an OPPTYPE picture that
    signals both Modified Quantization (bit 14) and Advanced INTRA
    Coding (bit 8); the AIC+MQ combination decodes end-to-end through
    the §I.3 INTRA reconstruction path.
  - `decode_intra_macroblock_aic` threads the §T.3 chrominance
    `QUANT_C` step into the two AIC chroma blocks (the four luma blocks
    keep the luminance QUANT) and the §T.4 EXTENDED-ESCAPE flag into
    every AIC block parse.
  - `parse_intra_block_aic` and `decode_intra_tcoef_event` gained a
    `modified_quant` parameter. When set, the Table I.2 ESCAPE LEVEL
    `1000 0000` is the §T.4 EXTENDED-ESCAPE marker introducing an
    11-bit EXTENDED-LEVEL field, rather than a forbidden code — §T.5
    rule 2 extends the EXTENDED-ESCAPE mechanism to the Table I.2 VLC.
    The wire transform reuses `block::extended_level_from_wire`
    (now `pub(crate)`); with the flag clear `0x80` stays forbidden.
  - Out of scope (reported, not guessed): MQ combined with Advanced
    Prediction / INTER4V (Annex F), PB / Improved PB-frames
    (Annex G / M) or Slice-Structured (Annex K) — the shim still
    refuses those because their reconstruction paths do not yet thread
    the §T.3 / §T.4 dequant boundary.
  - Tests: the prior MQ+AIC shim-refusal test is replaced by a §T.3
    QUANT_C chroma comparison (MQ-on vs AIC-only over identical AIC
    macroblock bytes — luma bit-identical, both chroma planes differ)
    and a §T.4 EXTENDED-ESCAPE-in-chroma decode through the AIC path
    (+200 extended AC at QUANT_C 4). `cargo test -p oxideav-h263`: 572
    (was 571).

- Annex T Modified Quantization mode — MQ-active picture reconstruction
  end-to-end (round 310):
  - `DecodeOptions::modified_quant` new flag carries the MQ decision
    into the §4.2.1 GOB-walker driver.
    `plus_ptype_to_baseline_shim` no longer refuses the §5.1.4.2
    OPPTYPE Modified-Quantization bit (bit 14); it OR-merges the
    effective MQ bit into the returned options exactly as it already
    does for the AIC / DF bits, so a UFEP=001 PLUSPTYPE MQ picture
    auto-activates the mode through `decode_picture_layer` with the
    caller's default options.
  - `decode_one_macroblock` threads MQ into the macroblock + block
    layers: `MbContext::modified_quant` selects the §T.2
    variable-length DQUANT VLC
    ([`crate::annex_t::parse_modified_dquant`]); every
    `BlockContext::modified_quant` enables the §T.4 EXTENDED-ESCAPE
    interpretation of the §5.4.2 ESCAPE LEVEL `1000 0000`; and the two
    chrominance blocks are inverse-quantised with the §T.3 / Table T.2
    `QUANT_C` ([`crate::annex_t::quant_c_from_quant`]) while the four
    luma blocks keep the luminance QUANT.
  - Out of scope (reported, not guessed): MQ combined with Advanced
    INTRA Coding (Annex I), Advanced Prediction / INTER4V (Annex F), PB
    / Improved PB-frames (Annex G / M) or Slice-Structured (Annex K) —
    the shim returns `Error::NotImplemented` because those
    reconstruction paths do not yet thread the §T.3 / §T.4 dequant
    boundary. The §T.5 encoder-side usage restrictions are not enforced
    by the decoder.
  - 5 new tests: §T.3 MQ-on/MQ-off chroma comparison (identical INTRA
    body — luma bit-identical, both chroma planes differ); §T.3 QUANT_C
    reconstruction-level pin; §T.4 end-to-end EXTENDED-ESCAPE through
    the picture driver (+200 chroma coefficient at QUANT 4); §T.2
    arbitrary-selection DQUANT mid-picture decode; MQ + AIC shim
    refusal. `cargo test -p oxideav-h263`: 571 (was 566).

- Annex T Modified Quantization mode — §T.4 Modified coefficient range
  (EXTENDED-ESCAPE / EXTENDED-LEVEL) (round 308):
  - `BlockContext::modified_quant` flag threads the §T.4 interpretation
    into `parse_block`. When set, the §5.4.2 ESCAPE LEVEL field value
    `1000 0000` — forbidden in baseline — becomes the EXTENDED-ESCAPE
    marker, introducing an 11-bit EXTENDED-LEVEL field that represents
    AC coefficient magnitudes greater than 127. When clear the baseline
    rule applies (`1000 0000` forbidden); `0000 0000` stays forbidden in
    both modes.
  - New `extended_level_from_wire(wire) -> i16` decodes the §T.4 /
    Figure T.1 transform: the 11 wire bits are cyclically rotated left
    by 5 (the inverse of the encoder's rotate-right-by-5 that prevents
    start-code emulation) and the result is sign-extended as an 11-bit
    two's-complement LEVEL.
  - Out of scope (reported, not guessed): the §T.4 |REC| < 4096 and
    `QUANT < 8` §T.5 usage restrictions (encoder-side), and the full
    MQ-active picture reconstruction driver (the extended-PTYPE picture
    path still refuses the OPPTYPE MQ bit with `NotImplemented`; §T.4 is
    reachable via `parse_block` with `modified_quant: true`).
  - 9 new tests (`cargo test -p oxideav-h263`: 566, was 557).

- Annex T Modified Quantization mode — §T.2 Modified DQUANT + §T.3
  chrominance QUANT_C (round 302):
  - New `annex_t` module. `parse_modified_dquant(reader, prior_quant)
    -> Result<ModifiedDquant>` decodes the §T.2 variable-length DQUANT
    field: the §T.2.1 small-step form (first bit `1` + one more bit →
    Table T.1 lookup of the new QUANT from the prior QUANT, two bits
    total) and the §T.2.2 arbitrary-selection form (first bit `0` +
    five bits → a brand-new QUANT directly per §5.1.19, six bits
    total; a five-bit value of `0` is rejected). `ModifiedDquant`
    carries the new QUANT and the bit count consumed.
  - `quant_c_from_quant(quant) -> Result<u8>` implements the §T.3 /
    Table T.2 chrominance quantiser derivation (the smaller chroma
    step size used for inverse-quantising chrominance coefficients,
    and the Annex J chrominance deblocking filter, when MQ is in use).
  - `MbContext::modified_quant` flag: when set, the §5.3.6 DQUANT field
    in `parse_macroblock` is parsed via `parse_modified_dquant` instead
    of the baseline 2-bit Table 13 differential.
    `H263Macroblock::quantiser_after` becomes the §T.2 new QUANT and
    `dquant` carries the signed change (`new − prior`).
  - Out of scope (reported, not guessed): the §T.4 EXTENDED-ESCAPE /
    EXTENDED-LEVEL coefficient-range extension (it belongs in the
    §5.4.2 TCOEF VLC layer) and the §T.5 encoder usage restrictions.
  - 21 new tests (`cargo test -p oxideav-h263`: 557, was 536).

- Annex M Improved PB-frames mode end-to-end decode (round 295):
  - `decode_improved_pb_picture(data, reference, prev_tr, options) ->
    Result<PbFramePair>` new public entry point in `picture.rs`. It
    decodes a PLUSPTYPE picture whose §5.1.4.3 MPPTYPE picture-type is
    `"010"` (Improved PB-frame, §M.1) into the (P, BPB) pair. After the
    PLUSPTYPE header it consumes §5.1.19 PQUANT, §5.1.22 TRB (`0`
    rejected) and §5.1.23 DBQUANT, derives §G.4 TRD (the modulo-256
    negative wrap) from `prev_tr`, and drives the shared GOB /
    macroblock walker with an Annex M PB context.
  - §M.4 / Table M.1 MODB form wired into the macroblock parser: a new
    `MbContext::pb_annex_m` flag selects `parse_modb_annex_m` (the
    6-entry Table M.1) over the Annex G Table 11 form, surfacing the
    §M.2 coding mode plus CBPB / MVDB presence on the new
    `H263Macroblock::annex_m_modb`. CBPB / MVDB are gated by the
    Table M.1 accessors.
  - §M.2 three BPB-macroblock prediction modes: §M.2.1 bidirectional
    (the §G.4 / §G.5 composition with MVD = 0, §M.3); §M.2.2 forward (a
    single 16 × 16 MVDB vector plus the §M.2.2 left-neighbour forward-
    vector predictor — reset at each row's far-left edge — forward-only
    from the previous reference, no PREC); and §M.2.3 backward (the BPB
    prediction is PREC). The §6.3.1 BPB-residuals are added where CBPB
    lights them, dequantised with the Table 6 BQUANT, exactly as
    Annex G.
  - Refused (reported, not guessed): Improved-PB combined with Annex K
    Slice-Structured (§K.2 slice-boundary BPB exclusions), Advanced
    Prediction (the §F.2 four-vector BPB derivation), UMV (the §M.2.2
    over-boundary forward vector under the extended range), AIC, custom
    PCF, CPM and RRU — all `Error::NotImplemented`. The single-frame
    entry points (`decode_picture` / `decode_picture_layer`) refuse an
    Improved PB-frame (they cannot return the BPB-picture).
  - 9 new tests (all-skipped P/BPB reference reproduction; §M.2.3
    backward = PREC; §M.2.2 forward MVDB shift; §M.2.2 left-neighbour
    predictor chaining; entry-point gating; zero-TRB rejection; and
    three Table M.1 MODB parser tests for rows 0 / 3 / 4); suite now
    536 passing (was 527).
- Annex K Slice-Structured mode end-to-end decode, free-running
  (non-Rectangular-Slice) submode (round 291):
  - `decode_picture_layer` / `decode_picture_layer_with_inherited`
    now route a UFEP=001 PLUSPTYPE picture whose §5.1.4.4 OPPTYPE
    Slice-Structured bit (bit 10) is set to a new internal
    `decode_slice_structured_after_header` driver. It reads §5.1.19
    PQUANT, parses the §K.2 reduced first-slice header, and walks
    macroblocks in picture scanning order from the slice's MBA until
    the next §K.2.2 SSC or end of stream; subsequent slices re-anchor
    via `skip_sstuf` + `parse_slice_layer`.
  - §6.1.1 "outside the slice" motion-vector prediction: a per-
    macroblock `segment` id on the prediction grid makes a candidate
    neighbour in a different slice unavailable (MV1 zeroed, MV2/MV3
    copied from MV1). The baseline GOB driver is bit-identical.
  - §K.1 exact-tiling enforcement with the new `Error::BadSliceCoverage`
    (overlap, non-strictly-increasing MBA, or undecoded macroblock).
  - The §K.2.8 Rectangular Slice submode, Annex K + Advanced
    Prediction, CPM sub-bitstreams, RRU, and PB-frames are refused
    with `Error::NotImplemented` (reported, not guessed).
  - 7 new tests (QCIF single-/two-slice INTRA, all-skipped and
    two-slice INTER, MBA-order and coverage rejections, RS refusal);
    suite now 527 passing (was 521).
- Annex G PB-frame end-to-end decode driver (round 283):
  - `decode_pb_picture(data, reference, prev_tr, options) ->
    Result<PbFramePair>` new public entry point in `picture.rs`:
    decodes one baseline-PTYPE PB-frame (PTYPE bit 13) into the (P, B)
    picture pair. Parses §5.1.22 TRB (3 bits; `0` rejected) and
    §5.1.23 DBQUANT (2 bits) after PTYPE, derives §G.4 TRD from the
    caller-supplied `prev_tr` with the modulo-256 negative wrap, and
    drives the shared GOB/macroblock walker with a PB context. Per
    coded macroblock: P-blocks reconstruct as before; PREC (§G.5) is
    lifted from the P-frame planes; `pb_b_predict_macroblock` builds
    the §G.4 + §G.5 B-prediction from the reconstructed P-vectors
    (replicated single MV, the §G.2 INTRA B-purpose vector, or zeros
    for skipped MBs) and the parsed MVDB; the six B-blocks then add
    INTER-style TCOEF residuals (no INTRADC, §G.3) dequantised with
    the Table 6 BQUANT where CBPB lights them, in the §5.4 P-then-B
    wire order. PB + Advanced Prediction is refused
    (`Error::NotImplemented`) pending the §G.2 OBMC remote-vector
    exception; the single-frame entry points keep refusing PB
    pictures (they cannot return the B-picture).
  - `PbFramePair { p_frame, b_frame }` new public struct (display
    order B then P; only `p_frame` is a prediction reference).
  - `MbContext::pb_frames` new field +
    `H263Macroblock::{modb, cbpb, mvdb}` new fields: under the flag,
    `parse_macroblock` consumes the §5.3 Table 10 / Figure 10
    PB-frame layer — MODB (§5.3.3) after MCBPC for every coded
    non-stuffing macroblock, CBPB (§5.3.4) before CBPY when MODB
    indicates it, MVD also for INTRA types 3 / 4 (§5.3.7 "in
    PB-frames mode also for INTRA macroblocks"), MVDB (§5.3.9) after
    MVD2-4 when MODB indicates it.
  - §6.1.1 rule-1 PB exception: INTRA macroblocks' reconstructed
    vectors stay live candidate predictors in PB-frames mode ("if
    not in PB-frames mode with bidirectional prediction" qualifies
    the INTRA zeroing); the INTRA branch of the macroblock decoder
    reconstructs the §G.2 vector through the same §6.1.1 predictor +
    Table 14 path used for INTER vectors.
  - `pb_bquant(dbquant, quant)` new public function in `pb_layer.rs`:
    §5.1.23 / Table 6 `((5 + DBQUANT) × QUANT) / 4` by truncation,
    clipped to 31.
  - `Error::BadPbTemporalReference` new variant (TRB = 0 or TRD = 0).
  - 18 new tests: Table 6 rows / truncation / clip / two panic paths;
    PB macroblock wire layer at pinned bit budgets (MODB row 0 + MVD
    = 7 bits; MODB row 2 + CBPB + MVDB = 20 bits; INTRA + MODB row 1
    + MVD + MVDB = 19 bits with `mvd234` empty; skipped-MB and non-PB
    field absence); end-to-end: all-skipped PB picture reproduces a
    ramp reference sample-exactly in both frames; TRB = 0 / TRD = 0
    rejections; negative-TRD wrap (TR 1 after prev_tr 255 = forward
    step 2); entry-point gating both directions; MVDB-only
    B-prediction equal to the direct §G.4 + §G.5 composition;
    CBPB-lit B-residual hand-pinned (DBQUANT `11` × QUANT 8 → BQUANT
    16 → |REC| = 47 → +6/pixel → 106 on a 100 background); INTRA
    vector feeding both the B-part and the right neighbour's §6.1.1
    median (P-part decodes one full pel shifted). `cargo test -p
    oxideav-h263` reports 521 passed (was 503).

- §G.4 + §G.5 PB-frame B-macroblock prediction composition
  (round 279):
  - `pb_b_predict_macroblock(planes, mb_x, mb_y, p_mvs, mvd, trb,
    trd, rcontrol) -> PbBMacroblockPrediction` new public function in
    `pb_layer.rs`: the whole-macroblock §G.4 + §G.5 step the PB-mode
    picture driver invokes once the P-macroblock has been
    reconstructed and clipped into PREC (§6.3.2). Derives the four
    per-luma-block `(MVF, MVB)` pairs via `pb_b_vector` (the same MVD
    applied to all four blocks per §G.4), the chroma pair via
    `pb_b_chroma_vector`, the §G.5 bidirectional rectangles via
    `pb_b_bidir_luma_block_extent` / `pb_b_bidir_chroma_extent`, and
    predicts all six 8 × 8 B-blocks (four luma + Cb + Cr) via
    `pb_b_predict_block` — forward fetch from the previous decoded
    picture at the block's picture position, backward fetch from the
    macroblock-local PREC plane at `(nh*8, nv*8)` luma / `(0, 0)`
    chroma. Asserts PREC geometry (16 × 16 / 8 × 8 — §G.5 defines
    PREC as one macroblock and the §D.1 replication boundary must be
    PREC's own edge) and the §4.2.3 macroblock grid.
  - `PbBReferencePlanes` new public struct: the six reference planes
    the composition reads (`prev_y`/`prev_cb`/`prev_cr` full-picture
    forward references; `prec_y`/`prec_cb`/`prec_cr` macroblock-local
    PREC backward references).
  - `PbBMacroblockPrediction` new public struct: 16 × 16 luma + two
    8 × 8 chroma prediction arrays in §G.5 `[j][i]` layout; the
    §6.3.1 residual add for CBPB-lit B-blocks and the §6.3.2 clip
    stay with the caller.
  - 6 new tests: zero-MV uniform-plane full-bidir averages on all
    three channels; Figure-5 quadrant mapping; negative-MVB
    left-column forward-only split (luma MVB = −2 / chroma −1);
    §G.4 MVD ≠ 0 path (MVB = MVF − MV, shifted-ramp forward fetch +
    chroma-vector arithmetic pinned end-to-end); assembly consistency
    against direct per-block primitive composition; PREC-geometry
    rejection (`should_panic`). `cargo test -p oxideav-h263` reports
    503 passed (was 497).
- §G.5 PB-frame B-block motion-compensated prediction composition
  (round 272):
  - `pb_b_predict_block(prev_plane, prec_plane, fwd_x, fwd_y, bwd_x,
    bwd_y, mvf, mvb, bidir_extent, block_i_origin, block_j_origin,
    rcontrol) -> [[u8; 8]; 8]` new public function in `pb_layer.rs`:
    the §G.5 per-block prediction driver-primitive. Builds the
    forward prediction with `mvf` against `prev_plane` ("forward
    prediction using MVF relative to the previous decoded picture")
    and the backward prediction with `mvb` against `prec_plane`
    ("backward prediction using MVB relative to PREC") — both via
    §6.1.2 half-pel bilinear interpolation (`motion_compensate_block`)
    — then blends the two over the §G.5 `bidir_extent` rectangle via
    `pb_b_blend_block`. This closes the "fetch forward / backward 8 × 8
    predictions, blend per §G.5" step the macroblock-layer PB-mode
    driver needs per block, scoped as a pure function over two
    reference planes. The forward / backward fetches are independent
    (`prev_plane`/`prec_plane`, distinct origins), and a `None`
    extent short-circuits the whole block to forward-only.
  - `flat_to_ji` private helper: reshapes the flat row-major
    `[u8; 64]` from `motion_compensate_block` (`flat[py*8+px]`) into
    the `[[u8; 8]; 8]` `[j][i]` layout the §G.5 blend primitives
    consume (`nested[j][i] = flat[j*8+i]`; §G.5 `i` horizontal, `j`
    vertical).
  - 5 new tests: `None`-extent forward-only short-circuit; full-extent
    truncated-average over uniform planes (`(90+200)/2 = 145`);
    distinct-plane/origin partial-extent blend over a horizontal-ramp
    forward plane; non-zero integer MVF forward-fetch shift (+2 px
    with §D.1 right-edge replication); and `flat_to_ji` row-major
    order. `cargo test -p oxideav-h263` reports 497 passed (was 492).
- §G.5 PB-frame B-block per-pixel bidirectional-prediction blend
  (round 263):
  - `pb_b_bidir_pixel(forward, backward) -> u8` new public function
    in `pb_layer.rs`: the §G.5 per-pixel average primitive — "the
    average is calculated by dividing the sum of the two predictions
    by two (division by truncation)". Sum is computed `u16`-wide to
    avoid `u8` overflow; the `(255 + 255) / 2 = 255` boundary stays
    inside `u8` so the cast back never overflows. Commutative by
    construction (`(a+b)/2 = (b+a)/2`).
  - `pb_b_blend_block(forward, backward, bidir_extent,
    block_i_origin, block_j_origin) -> [[u8; 8]; 8]` new public
    function: composes `pb_b_bidir_pixel` over an 8 × 8 block given
    the §G.5 bidirectional rectangle from
    `pb_b_bidir_luma_block_extent` (luma) or
    `pb_b_bidir_chroma_extent` (chroma). Pixels inside the
    rectangle are averaged; pixels outside it are taken from
    `forward` per §G.5's "all other pixels" clause. `None` extent
    short-circuits to the forward array verbatim. The
    `(block_i_origin, block_j_origin)` parameters translate between
    the §G.5 macroblock-local 0..=15 (luma) / block-local 0..=7
    (chroma) coordinate spaces the extent primitives return and the
    block-local 0..=7 indexing of the input arrays. Panics if the
    rectangle escapes the 8 × 8 block addressed by the origins.
  - 11 new tests (`cargo test -p oxideav-h263` reports 492 passed,
    previously 481): per-pixel identity (`(x+x)/2 == x` over the
    full `u8` range), truncation-toward-zero pin
    (`pb_b_bidir_pixel_truncates_toward_zero` covering `(0,1)`,
    `(1,2)`, `(3,4)`), `u8` boundary
    (`pb_b_bidir_pixel_max_inputs_does_not_overflow`),
    commutativity across a wide sample
    (`pb_b_bidir_pixel_commutes`), block-blend `None`-extent
    forward fallback, full-chroma-extent uniform average,
    sub-rectangle partial blend with forward fallback outside,
    nh=1/nv=1 luma sub-block origin-offset coverage, block-bound
    panic paths on `i` overflow / `j` underflow, and an end-to-end
    §G.4 → §G.5 mask → §G.5 blend chain
    (`pb_b_blend_chained_g4_extent_blend` composing `pb_b_vector`,
    `pb_b_bidir_luma_block_extent`, then `pb_b_blend_block`).

- §G.5 PB-frame B-block bidirectional-prediction mask (round 258):
  - `pb_b_bidir_extent_component(mvb_component, block_lo, block_hi,
    ref_max) -> Option<(i32, i32)>` new public function in
    `pb_layer.rs`: per-axis inclusive `[lo, hi]` pixel-coordinate
    range for which the §G.5 backward vector `MVB` points inside
    PREC (bidirectional-prediction region). Returns `None` if the
    range is empty (whole axis falls outside PREC; per §G.5 the
    bidirectional rectangle factorises as the Cartesian product of
    horizontal and vertical 1-D ranges, so any empty axis makes
    the whole sub-block forward-only). `ref_max` parameter
    generalises the §G.5 luma `15` vs chroma `7` upper bound.
    "/" is Rust signed `/` (truncation toward zero, matching
    §G.5's C expression `(-mh+1)/2`).
  - `pb_b_bidir_luma_block_extent(mvb, nh, nv) -> Option<((i32,
    i32), (i32, i32))>` new public function: composition of
    `pb_b_bidir_extent_component` for one of the four 8×8 luma
    sub-blocks of a B-block's macroblock. Returns the inclusive
    2-D pixel rectangle in macroblock-local coordinates 0..=15,
    or `None` if §G.5's bidirectional region is empty. Panics on
    `nh > 1` or `nv > 1` (§G.5 only enumerates the four sub-blocks).
  - `pb_b_bidir_chroma_extent(mvc) -> Option<((i32, i32), (i32,
    i32))>` new public function: chroma counterpart over the
    single 0..=7 chroma block per macroblock, with `ref_max = 7`
    per §G.5 chroma form.
  - 19 new tests (`cargo test -p oxideav-h263` reports 481
    passed, previously 462): full-block coverage on each of the
    four `(nh, nv)` luma sub-blocks with zero MVB; per-component
    shrink coverage on nh=0 left-pointing MVB and nh=1
    right-pointing MVB; empty-axis short-circuit (MVB outside PREC
    for nh=0 / nh=1 / nv=0 / nv=1); chroma full / shrink-top-left /
    shrink-bottom-right / outside coverage and per-axis
    factorisation check; division-truncation-toward-zero pin;
    end-to-end §G.4 → §G.5 chain (`pb_b_bidir_chained_after_g4`
    composing `pb_b_vector` then `pb_b_bidir_luma_block_extent`);
    and panic paths for invalid `nh` / `nv`.

- §G.4 PB-frame B-picture motion-vector calculator (round 249):
  - `pb_b_vectors(p_mv, mvd, trb, trd) -> (i32, i32)` new public
    function in `pb_layer.rs`: per-component §G.4 formula
    `MVF = (TRB × MV) / TRD + MVD`; `MVB = ((TRB - TRD) × MV) /
    TRD` if MVD == 0 else `MVB = MVF - MV`. `mvd: Option<i32>`
    encodes "MVDB absent" (`None`) vs "MVDB present" (`Some(d)`);
    `Some(0)` falls through to the same MVD-zero branch as `None`.
    Rust's signed `/` matches §G.4 "/ means division by
    truncation". Panics on `trd == 0` (§G.4 undefined for a zero
    TR increment).
  - `pb_b_vector(p_mv, mvd, trb, trd) -> (MotionVector,
    MotionVector)` new public function: two-axis composition of
    `pb_b_vectors` on a `MotionVector` / `Option<Mvd>` pair,
    returning the (MVF, MVB) pair for one 8×8 luminance block of
    the B-picture. Per §G.4 paragraph 4 the same MVDB pair is
    reused across all four luma B-blocks; the caller selects the
    per-block `p_mv`.
  - `pb_b_chroma_vector(luma_mvf, luma_mvb) -> (MotionVector,
    MotionVector)` new public function: §G.4 paragraphs 5-6
    chroma vector derivation, summing the four luma MVF / MVB
    half-pel components and snapping via Table F.1
    sixteenth-pel positions through the existing §F.2
    `chroma_mv_component_4mv` primitive (since §G.4 and §F.2
    share the same "sum of 4 luma half-pel → snap" transform).
  - 16 new tests (`cargo test -p oxideav-h263` reports 462 passed,
    previously 446): per-component formula coverage
    (zero/symmetric-split/three-quarter/one-quarter splits at
    representative (TRB, TRD) ratios), MVD-branch coverage
    (positive / negative MVD with the MVF-minus-MV path; explicit
    `Some(0)` matching `None`), Rust signed `/` truncation
    behaviour pin (`pb_b_vectors_division_truncates_toward_zero`),
    `trd == 0` panic path, two-axis composition
    (`pb_b_vector_composes_per_axis` /
    `_no_mvdb_takes_zero_branch_on_both_axes` /
    `_some_zero_mvd_matches_none`), end-to-end MODB +
    MVDB + §G.4 calculation chain
    (`pb_b_vector_chained_after_modb_annex_m_and_mvdb_parse`),
    and chroma collapse
    (`pb_b_chroma_vector_uniform_luma_collapses_via_table_f1` /
    `_all_zero_is_zero` /
    `_matches_chroma_mv_component_4mv`).

- §5.3.9 MVDB (Motion Vector Data for B-macroblock) parser
  (round 248):
  - `parse_mvdb(reader) -> Result<Mvd>` new public function in
    `pb_layer.rs` decoding the §5.3.9 dx/dy half-pel pair via the
    §5.3.7 / Table 14 MVD VLC (the same primitive the baseline
    macroblock parser already uses for §5.3.7 MVD / §5.3.8
    MVD2-4). Per §5.3.9 ("a variable length codeword for the
    horizontal component followed by a variable length codeword
    for the vertical component"), horizontal is read first; the
    returned `Mvd` carries `(dx_half, dy_half)` in half-pel units.
    Composes identically with the §5.3.3 `parse_modb` /
    `ModbPresence::has_mvdb` gate and the §M.4 `parse_modb_annex_m`
    / `ModbAnnexM::has_mvdb` gate; the wire format does not change
    between Annex G and Annex M (§M.2.2 / §5.3.9 share Table 14).
  - The shared `decode_mvd_component` primitive in `macroblock.rs`
    is now `pub(crate)` so `pb_layer.rs` can call it without
    duplicating the 64-row Table 14 transcription.
  - 9 new tests (`cargo test -p oxideav-h263` reports 446 passed,
    previously 437): zero-pair round-trip
    (`mvdb_zero_zero_pair_consumes_two_bits`), asymmetric
    `(+1, -1)` pair (`mvdb_plus_one_minus_one_round_trip`),
    symmetric `(-2, -2)` pair (`mvdb_minus_two_minus_two_pair`),
    EOF paths on an empty buffer and mid-pair truncation
    (`mvdb_empty_buffer_returns_eof` /
    `mvdb_truncated_between_components_returns_eof`), three
    end-to-end chains composing MVDB after MODB (MVDB-only
    `parse_modb` + `parse_mvdb` at 6 bits; CBPB+MVDB
    `parse_modb` + `parse_cbpb` + `parse_mvdb` at 12 bits;
    Annex M forward `parse_modb_annex_m` + `parse_mvdb` at 9
    bits), and a malformed-codeword path
    (`mvdb_unknown_codeword_returns_bad_mvd_code`) covering
    `Error::BadMvdCode` on a thirteen-zero prefix.

- §M.4 Improved PB-frames Table M.1 MODB parser (round 237):
  - `BpbCodingMode` new public enum (`Bidirectional` / `Forward` /
    `Backward`) carries the §M.2 BPB-macroblock prediction mode
    each Table M.1 row attaches via its "Coding mode" column.
  - `ModbAnnexM` new public enum collapsing the six Table M.1 rows
    onto a single tag combining `(CBPB, MVDB)` presence with the
    `BpbCodingMode` value. `has_cbpb()`, `has_mvdb()`,
    `coding_mode()`, and `code_bits()` accessors.
  - `parse_modb_annex_m(reader) -> Result<ModbAnnexM>` new public
    function decoding the §M.4 / Table M.1 variable-length codeword
    used by Improved PB-frames (PLUSPTYPE picture-coding code
    `"010"` per §5.1.4.3). Decode counts leading `1` bits up to 4
    (0 / 1 / 2 / 3 → rows 0..=3); a full run of four `1` bits
    consults one more bit (`0` → row 4 / `1` → row 5). All six
    Table M.1 codewords (`0` / `10` / `110` / `1110` / `11110` /
    `11111`) round-trip; only `Error::UnexpectedEof` is possible.
  - 12 new tests (`cargo test -p oxideav-h263` reports 437 passed,
    previously 425): the six Table M.1 codewords round-trip with
    independent assertions on the `CBPB` / `MVDB` presence and
    `coding_mode()` column (`modb_annex_m_row_0..=row_5`); a sweep
    over all six rows pinning `code_bits()` and the reader-advance
    invariant (`modb_annex_m_table_m1_round_trip_all_rows`); EOF
    paths on an empty buffer, truncated mid-run, and truncated
    at the row-4-vs-5 disambiguator
    (`modb_annex_m_empty_buffer_returns_eof` /
    `modb_annex_m_truncated_in_run_returns_eof` /
    `modb_annex_m_truncated_at_tail_returns_eof`); an end-to-end
    `parse_modb_annex_m` + `parse_cbpb` chain advancing the reader
    by 4 + 6 = 10 bits with per-block CBPB queries
    (`modb_annex_m_then_cbpb_chain_advances_by_10_bits`); and a
    cross-check pinning that the Annex M parser is independent of
    the Annex G `parse_modb` parser — feeding the four bits `1110`
    consumes all four through Annex M but only the first two
    through Annex G
    (`modb_annex_m_does_not_share_codewords_with_annex_g`).
  - Updated `pb_layer` module-level doc string from "Annex M is a
    separate primitive a future round will add" to naming
    `parse_modb_annex_m` as that primitive plus the §M.1 "BPB"
    naming rationale.

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
