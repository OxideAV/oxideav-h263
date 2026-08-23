# oxideav-h263

[![CI](https://github.com/OxideAV/oxideav-h263/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-h263/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-h263.svg)](https://crates.io/crates/oxideav-h263) [![docs.rs](https://docs.rs/oxideav-h263/badge.svg)](https://docs.rs/oxideav-h263) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A pure-Rust ITU-T H.263 baseline video **codec** for the
[oxideav](https://github.com/OxideAV/oxideav) framework, built
clean-room against [ITU-T Recommendation H.263 (01/2005)][spec].

## Status

Full baseline **decoder**, plus a growing **encoder**. The decoder
implements the H.263 baseline picture / GOB / macroblock / block layers
and reconstructs INTRA and INTER pictures end-to-end, plus a wide set of
optional Annexes (D, E, F, I, J, K, L, M, N, O, P, Q, R, S, T, V, W).

The encoder (round 376 onward) produces baseline **INTRA (I-)** and
**INTER (P-)** pictures plus a growing set of optional modes:
`encoder::encode_intra_picture` / `encode_inter_picture` /
`encode_inter_picture_motion` (SAD search + half-pel refinement with
§6.1.1 predictor replay), `encode_inter_picture_umv` (**Annex D**
extended-range motion with the exact §D.2 pair-selection inverse;
the PLUSPTYPE forms `encode_inter_picture_umv_plus` /
`encode_inter_picture_umv_slices` emit the §D.2 **Table D.3**
reversible MVD codes under the §5.1.9 UUI Tables-D.1/D.2 range —
round 447),
`encode_inter_picture_ap` (**Annex F** INTER4V four vectors per
macroblock with §F.3 OBMC-exact prediction, two-pass;
`encode_inter_picture_ap_umv_plus` pairs it with UMV+ Table D.3
extended-range block vectors on an H.263+ header — round 447),
`encode_pb_picture` (**Annex G** PB-frames: P-part + §G.4/§G.5
bidirectionally-predicted B-part with BQUANT residuals),
`encode_intra_picture_dquant` (§5.3.6 per-macroblock DQUANT),
`encode_intra_picture_gobs` / `encode_inter_picture_gobs` (§5.2 GOB
headers with per-GOB GQUANT + segmented MV prediction),
`encode_intra_picture_aic` / `encode_intra_picture_aic_auto` /
`encode_intra_picture_aic_mq` / `encode_intra_sequence_aic`
(**Annex I** Advanced INTRA Coding: §I.2 per-macroblock INTRA_MODE
with a rate-driven mode decision, §I.3 coefficient-domain DC/AC
prediction from the encoder's own reconstructed neighbours, the
Table I.2 separate INTRA VLC, and the optional **Annex T** §T.3
chroma `QUANT_C` + §T.4 EXTENDED-ESCAPE range), and the closed-loop
GOP driver `encode_sequence` (I + P GOPs predicting from the
encoder's own decoded reconstruction — no drift; optional §5.1.27
EOS).

The encoder also emits **on-wire H.263+ (PLUSPTYPE)** pictures:
`write_plus_picture_header` (§5.1.4 UFEP `"001"` + OPPTYPE +
MPPTYPE + UUI / SSS emission) underpins the self-describing
`encode_intra_picture_plus` / `encode_inter_picture_plus` /
`encode_inter_picture_umv_plus` / `encode_intra_picture_aic_plus` /
`encode_intra_picture_aic_mq_plus` entry points, whose Annex
I / T / D modes are signalled in OPPTYPE and decode with
`DecodeOptions::default()`. **Annex K Slice Structured encoding**
is landed on both picture types: `encode_intra_picture_slices`
(per-slice §K.2.7 SQUANT rate control), `encode_inter_picture_slices`
(motion + per-slice §6.1.1 predictor segments) and
`encode_intra_picture_slices_aic` / `_aic_mq` (per-slice §I.3
AIC availability — the conformance fixtures' AIC + MQ + SS mode set,
now producible as well as decodable), all via the §K.2 slice-header
writers `write_first_slice_header` / `write_slice_layer`. The §K.1
**Rectangular Slice** and **Arbitrary Slice Ordering** submodes are
staged on both sides (round 438): the decoder walks each slice's
macroblocks in scanning order within its SWI-wide rectangle and —
under ASO — accepts slices in any bitstream order (coverage-driven
completion, per-segment predictor rules already order-independent);
`encode_intra_picture_slices_rect` / `encode_inter_picture_slices_rect`
emit full-height vertical stripes (SWI on the wire, optional
right-to-left ASO emission). Round 443 adds
`encode_inter_picture_ap_slices` (Annex K + Annex F: free-running
slices whose macroblocks carry four §F.2 vectors predicted through
the slice-confined §F.3 OBMC blend) and
`encode_intra_picture_slices_cpm` (Annex K + CPM: §5.1.21 PSBI +
per-slice §K.2.4 SSBI for one Sub-Bitstream).

**Annex E Syntax-based Arithmetic Coding** is implemented in both
directions (round 438): the `sac` module stages the §E.2/§E.3
arithmetic coders, the §E.5 PSC_FIFO stuffing rule (with the zero-run
counter spanning the header/arithmetic boundary) and the §E.8
`cumul_freq` models with §E.7 clause-5-table indexing.
`decode_picture_sac` decodes a baseline-PTYPE SAC picture (I / P, UMV
legal per §5.1.4.6; Annex S/T barred), and `decode_sequence` routes
SAC pictures automatically from PTYPE bit 11 — pure-SAC and mixed
VLC + SAC elementary streams both work. The SAC mode-combination tail
is closed on both directions (round 443): **SAC + Annex F Advanced
Prediction** (INTER4V four vectors under the §E.7 `cumf_MVD` model +
the §F.3 deferred-OBMC luminance reconstruction) and **SAC + Annex G
PB-frames** (`decode_pb_picture_sac` — MODB / CBPB / MVDB under their
§E.7 models, the §G.2 INTRA-macroblock vector, and the §G.4/§G.5
B-part through the shared reconstruction core). The encoder arm —
`encode_intra_picture_sac`, `encode_inter_picture_sac` (zero-MV),
`encode_inter_picture_motion_sac` (SAD + half-pel search, intra
refresh), `encode_inter_picture_ap_sac` (two-pass §F.2/§F.3 OBMC
INTER4V) and `encode_pb_picture_sac` (P + B pair) — shares the VLC
encoder's transform/quantiser stage, so SAC and VLC pictures of the
same source reconstruct **byte-identically** (pinned for the I / P /
AP / PB shapes); measured entropy-layer saving on the gradient QCIF
corpus is 6.4–25.5 % on I-pictures (QP 31 → QP 2).

The encoder gained **Annex Q Reduced-Resolution Update** entry points
(round 443): `encode_intra_picture_rru` (each 16×16 region
down-sampled to the reduced 8×8 block, standard INTRA stage over the
32×32 macroblock grid) and `encode_inter_picture_rru` (pseudo-domain
motion search so every candidate expands to a legal §Q.4
half-integer-or-zero vector, per-16×16-sub-block residuals
down-sampled to 8×8; `encode_inter_picture_rru_umv` widens the
pseudo window to the Tables-D.1/D.2 UMV range with Table D.3
difference coding — round 447) — both self-describing (MPPTYPE RRU bit) and
round-tripped through the RRU decode driver (static P lossless,
translated content within tolerance, I + P streams through
`decode_sequence`).

The encoder also has **rate control** (round 438): the
`rate_control` module pairs the Annex B Hypothetical Reference
Decoder buffer simulation (`HrdModel` — §B.3/§B.4 examinations, the
post-removal occupancy-below-`B = 4·R/PCF` requirement) with a
virtual-buffer QUANT governor (`RateController`), and
`encode_sequence_rate_controlled` drives the closed-loop I + P GOP
encoder against a bits-per-picture budget with §B.4-violation /
overshoot re-encodes. Measured steady-state accuracy on the
moving-square QCIF clip: mean bits/picture within −9.1 % … +4.6 % of
target across 1.5 k–5 k budgets, all §B.4-conformant at `B = 4T`.

Everything is built bottom-up from reusable layers —
`encoder_vlc`, `fdct`, `encoder_block` (§5.4), `encoder_aic` (§I.3
block plan + Table I.2 emit), `encoder_mb` (§5.3), `encoder_motion`
(§6.1.1/§F.2/§D.2 estimation + predictor replay) and `encoder`
(§5.1 / §5.2 / §5.1.4 / §K.2 picture layer) — each
round-trip-verified against the decoder, including a mixed-mode
I + P + UMV-P + AP-P + PB elementary stream decoded end-to-end by
`decode_sequence`. The AIC closed loop reconstructs every block
through the exact decoder primitive so encoder and decoder never
drift (flat AIC pictures are byte-exact, single-slice forms are
byte-exact against their single-segment counterparts).

The **`rtp` module** stages the RFC 4629 RTP payload format: the
§5.1 `RR|P|V|PLEN|PEBIT` payload header + §5.2 VRC extension,
`packetize_stream` (P=1 segment packets at byte-aligned
PSC/GBSC/SSC/EOS with the two zero bytes stripped, budget-aware
last-boundary cuts honouring the §7 every-PSC-starts-a-packet rule,
§6.2 Follow-on fallback, optional §6.1.2 redundant picture-header
attachment with exact PEBIT) and `depacketize_payloads` (byte-exact
reassembly) — validated over crate-encoded GOP/GOB/slice streams and
the vendored conformance fixtures across payload budgets from 32 to
4096 bytes. The RFC 2190 legacy `video/H263` format covers all three
header modes (round 438): **Mode A** (GOB/picture-boundary packets,
full start codes, per-picture PTYPE-mirror fields, PB-frame
DBQ/TRB/TR) plus **Mode B / Mode C** macroblock-boundary
fragmentation — `enumerate_mb_boundaries` walks a picture without
pixel reconstruction to build the §5.2 resumption side channel
(GOBN / MBA / QUANT / §6.1.1 MV predictors), the packetizer fragments
over-budget segments at MB boundaries with SBIT/EBIT bit-granular
cuts, and the depacketizer reassembles any mode mix at bit
granularity.

The crate is wired into the **`oxideav_core` registry** (round 450):
`register()` installs a real codec entry — the streaming
`H263StreamDecoder` (packetised `Decoder` over the elementary-stream
drivers: byte-aligned PSC re-framing, so one-picture-per-packet and
arbitrarily-split raw streams both decode; PB pairs yield two display-
order frames; `reset()` clears the cross-picture state for seeks) and
the closed-loop `H263StreamEncoder` (`Encoder`; per-frame form of
`encode_sequence` with `quant` / `gop` / `search` / `umv` / `eos`
option knobs), plus the direct `make_decoder` / `make_encoder`
factories, the `H263` / `S263` FourCC tag claims and the `00 00 8x`
Picture-Start-Code payload magics for raw-stream identification.
Callers can equally drive the decoder through the free
`decode_picture` / `decode_sequence` entry points (the streaming
per-picture form is `decode_sequence_step` + `SequenceState`) and the
encoder through `encode_intra_picture` and friends.

Any path that is not yet wired returns `Error::NotImplemented` rather
than silently guessing.

## What works

The high-level `decode_picture` driver walks every GOB of a picture
top-to-bottom and every macroblock left-to-right, reconstructing a
planar 4:2:0 `YuvFrame`:

* **Picture layer (§5.1)** — PSC, Temporal Reference, non-extended
  PTYPE (split-screen / document-camera / freeze-release indicators,
  source format sub-QCIF..16CIF, INTRA / INTER coding type, optional
  Annex D/E/F/G mode flags), and the extended PLUSPTYPE header
  (§5.1.4 onward: UFEP / OPPTYPE / MPPTYPE + CPM / PSBI / CPFMT /
  EPAR / CPCFC / ETR / UUI / SSS), plus the §5.1.11–§5.1.16
  scalability / reference-picture-selection fixed-length fields
  ELNUM / RLNUM / RPSMF / TRPI / TRP and the BCI codeword (the
  variable-length §5.1.17 BCM and §5.1.18 RPRP payloads are refused).
* **GOB layer (§5.2)** — GBSC, Group Number, GOB Frame ID, GQUANT
  (CPM = "0" branch), plus the §5.2.2 first-GOB (group-number-0)
  header elision: the `decode_picture_no_gob0_header` entry point reads
  the §5.1.19 PQUANT / §5.1.20 CPM picture-header fields, the §5.1.24
  PEI / §5.1.25 PSUPP extension loop, decodes the header-less GOB 0 at
  QUANT = PQUANT, and treats every later GOB's header as **optional**
  (§5.2): `gob_header_present` probes for a GBSC (after any §5.2.1
  GSTUF byte-alignment); a present header primes a fresh QUANT and
  video-picture segment, an absent one continues the previous segment
  at the carried-over QUANT — so streams that omit empty GOB headers
  (the reference encoder for the standard formats) decode correctly
  (CPM = "1" refused).
* **Elementary-stream demux** — `decode_sequence` splits a multi-picture
  stream on byte-aligned Picture Start Codes (§5.1.1 / §5.1.28),
  decoding each picture and threading the reconstructed frame forward as
  the INTER reference for the next. It dispatches per picture on the
  §5.1.3 source-format selector: a baseline-PTYPE picture takes the
  §5.2.2 GOB-0-elided path, an extended-PTYPE (PLUSPTYPE / H.263+)
  picture routes through the full PLUSPTYPE driver (H.263+ Annex modes,
  slice-structured layout, reference resampling) while the §5.1.4.4 /
  §5.1.4.5 inherited extended-mode snapshot threads forward (a baseline
  picture resets it). The extended GOB and Annex N RPS paths read the
  §5.1.19 PQUANT that follows the PLUSPTYPE / CPFMT / RPRP block and
  elide the group-number-0 GOB header (§5.2.2), so the H.263+ wire a
  real encoder emits decodes end-to-end. Custom Picture Clock Frequency
  (§5.1.7 / §5.1.8) is accepted (CPCFC / ETR are framed but timing-only,
  with no effect on reconstructed pixels).
* **Macroblock layer (§5.3)** — COD, MCBPC (Tables 7 / 8), CBPY
  (Table 12), DQUANT, and MVD / MVD2-4 (Table 14).
* **Block layer (§5.4)** — INTRADC 8-bit FLC (Table 15) and TCOEF
  VLC (Table 16, 102 regular code-points + ESCAPE), accumulated in
  zigzag scan order.
* **INTRA reconstruction (§6.1 / §6.2 / §6.3.2)** — H.261-style
  inverse quantisation (modulo-2 oddifier), `[-2048, +2047]` AC clip,
  zigzag → 8×8 scatter (Figure 14), inverse DCT (f64 kernel meeting
  the Annex A.7 accuracy budget), sample clip to `[0, 255]`.
* **INTER reconstruction (§6.1.1 / §6.1.2 / §6.3.1)** — differential
  motion-vector reconstruction with the Figure-12 median predictor and
  candidate border-decision rules, Table 18 chroma vector derivation,
  half-pixel bilinear interpolation (Figure 13) with §D.1 edge
  replication, and residual summation + clip. Skipped macroblocks
  (COD = 1) copy the reference with a zero MV.
* **Annex D §D.2** — Unrestricted Motion Vector mode, both header
  forms: PLUSPTYPE-absent (extended `[-63, 63]` half-pel range with
  predictor-dependent Table-14 difference-pair selection) and
  PLUSPTYPE-present ("UMV+", round 447 — **Table D.3** reversible MVD
  codes for MVD / MVD2-4 with the §D.2 six-zero emulation-prevention
  bit, single-valued `predictor + difference` reconstruction, the
  §5.1.9 UUI range selection: Tables D.1 / D.2 under UUI = "1",
  unlimited-but-for-§D.1.1 under UUI = "01", last-sent UUI inherited
  by UFEP=000 pictures). Composes with Advanced Prediction (Table D.3
  MVD2-4 through the §F.2/§F.3 path), Modified Quantization and the
  Annex K slice drivers.
* **Annex E** — Syntax-based Arithmetic Coding mode end-to-end:
  the §E.2 / §E.3 arithmetic coders, the §E.5 stuffing rule and the
  §E.8 models under §E.7 indexing, decoded through
  `decode_picture_sac` (baseline-PTYPE I / P, single video picture
  segment, UMV supported) and auto-routed by `decode_sequence` from
  PTYPE bit 11. Advanced Prediction composes (INTER4V MVD2-4 under
  `cumf_MVD`, §F.3 deferred-OBMC luminance, single-MV OBMC too) and
  PB-frames compose (`decode_pb_picture_sac` — MODB / per-block CBPB
  / MVDB models, §G.2 INTRA vectors, §G.4 / §G.5 B-parts through the
  shared reconstruction core, auto-routed by `decode_sequence`).
  §5.1.4.6 bars Annex S / T combinations; AP + PB together and
  mid-picture GOB headers stay refused. Reconstruction is
  byte-identical to the VLC path at equal quantised coefficients.
* **Annex F §F.2 / §F.3** — Advanced Prediction: four-motion-vector
  candidate-predictor redefinition (Figure F.1 — B1's MV3 reads the
  above-right macroblock's B3, B4's candidates are entirely
  intra-macroblock, each just-reconstructed vector threads back in as
  an intra-macroblock candidate, and a **single-MV** macroblock's
  predictor uses the block-1 derivation so an INTER4V neighbour
  contributes the exact 8×8 cell Figure F.1 names), Table F.1
  sixteenth-pixel chroma derivation, and overlapped block motion
  compensation (OBMC) over the Figures F.2 / F.3 / F.4 weight
  matrices with the `Zero` / `Current` / `Vector` remote-MV
  substitution rules. OBMC covers **every** INTER macroblock of an AP
  picture — including COD = 1 skipped macroblocks (§5.3.1 NOTE) and
  one-vector macroblocks ("four vectors with the same value") — and
  the B2/B4 right-half remotes read the **actual** vector of the
  macroblock to the right (deferred one macroblock). The vendored
  `advanced-prediction-mode` conformance fixture decodes within the
  Annex A.7 tolerance; `DecodeOptions::obmc_skip_zero_right` is an
  opt-in ecosystem-compatibility deviation (zero right-half remotes
  for skipped macroblocks) the fixture's producing encoder family
  requires — the spec-default differs only there.
* **Annex I §I.2 / §I.3** — Advanced INTRA Coding: the INTRA_MODE VLC
  (Table I.1), the two alternate DCT scans (Figure I.2) and scan
  selection, the separate INTRA-coefficient VLC (Table I.2), the
  no-dead-zone modified inverse quantisation, the `clipAC` /
  `oddifyclipDC` clips, and the DC/AC prediction reconstruction.
* **Annex J §J.3** — in-loop deblocking edge filter (four-tap formula
  + full Table J.2 STRENGTH lookup + horizontal-before-vertical
  ordering + picture-edge skip), opt-in via `DecodeOptions::deblock`
  and auto-enabled from the PLUSPTYPE OPPTYPE bit (the vendored
  `deblocking-filter` conformance fixture decodes within a small
  bounded tolerance — the Annex A.7 ±1 IDCT bound applies before the
  in-loop filter, which can amplify it slightly).
  Deblocking-Filter mode also turns on the §5.3.8 / Table J.1 **four
  motion vectors per macroblock** element *without* the §F.3 OBMC
  element: when `deblock` is set, an INTER4V / INTER4V+Q macroblock
  carries MVD2-4 (gated by the new `MbContext::deblocking_filter`) and
  each 8×8 luma block is predicted with plain half-pel motion
  compensation by its own §F.2 four-vector-derived vector (the
  median-predictor / §D.1 edge replication / Table F.1 chroma derivation
  are shared with the Advanced-Prediction path; only the OBMC blend is
  skipped).
* **Annex K §K.2** — Slice Structured mode: the slice-layer header
  parse (SSC + SEPB1/2/3 + optional SSBI + MBA + SQUANT + optional
  SWI + GFID, plus the first-slice reduced form) and the end-to-end
  decode driver, including the §5.1.24 PEI / §5.1.25 PSUPP
  picture-header tail consumed before the first reduced slice header.
  Both §K.1 submodes decode (round 438): **Rectangular Slice** (each
  slice's macroblocks in scanning order within its `SWI + 1`-wide
  rectangle; overhanging rectangles refused) and **Arbitrary Slice
  Ordering** (slices land by MBA in any bitstream order; the
  strictly-increasing rule applies only with ASO off; completion is
  the §K.1 exactly-once coverage invariant). **Advanced Prediction
  composes** (round 443): §K.1 rule 1 confines the §6.1.1/§F.2
  candidate predictors and rule 3 the §F.3 OBMC remote vectors to the
  current slice (an out-of-slice remote substitutes the current
  vector), with the deferred-OBMC flush segment-filtered per slice.
  **CPM composes** (round 443): a CPM = "1" picture's §5.1.21 PSBI
  and per-slice §K.2.4 SSBI (Table K.1) parse, with the
  single-Sub-Bitstream decode validating every slice's SSBI against
  PSBI (a true Annex C multiplex is refused). The
  `slice-structured-mode` QCIF I+P+P conformance fixture decodes
  byte-exact within the Annex A.7 tolerance.
* **Annex S §S.2 / §S.3** — Alternative INTER VLC mode: each INTER
  coefficient block is interpreted with the baseline INTER VLC (Table
  16) first and re-interpreted with the Annex I INTRA VLC (Table I.2)
  only when the INTER reading would address coefficients past slot 63 of
  the block (§S.2.2 step 3, keyed on the run-overflow signal — both
  tables share one codeword inventory so the re-decode consumes the same
  bits); and, when both chrominance blocks of an INTER macroblock carry
  coefficients (`CBPC5 = CBPC6 = 1`), the CBPY codeword is the Table 12
  INTRA pattern (no INTER complement, §S.3). Wired into both the baseline
  single-MV INTER path and the Annex K Slice-Structured driver (via the
  shared per-macroblock reconstruction), auto-activated from the
  PLUSPTYPE OPPTYPE bit 13; refused only when combined with Advanced
  Prediction / INTER4V or PB-frames. The `alt-inter-vlc` (AIV + AIC + MQ
  + slice-structured) QCIF I+P+P conformance fixture decodes byte-exact.
* **Annex T** — Modified Quantization mode: the §T.2 variable-length
  DQUANT parser, the §T.3 chrominance `QUANT_C` step, and the §T.4
  EXTENDED-ESCAPE / EXTENDED-LEVEL extended coefficient range, driving
  an MQ-active picture reconstruction end-to-end for the baseline INTRA
  / INTER path, the Annex I Advanced INTRA Coding path, **and** the
  Annex K Slice-Structured driver (the §T.3 `QUANT_C` chroma dequant and
  the §T.5-rule-2 EXTENDED-ESCAPE extension to the Table I.2 VLC thread
  through the shared per-macroblock reconstruction on all three). The
  `advanced-intra-coding` (AIC + MQ + slice-structured) conformance
  fixture decodes byte-exact.
* **Annex Q** — Reduced-Resolution Update mode **end-to-end** (round
  443): an extended-PTYPE picture whose §5.1.4.3 MPPTYPE RRU bit is
  set routes to the dedicated driver — §Q.1 geometry (display /
  reference / coded sizes, the 32×32 macroblock grid), §Q.3 reference
  extension by edge replication, standard §5.3/§5.4 macroblock syntax
  with §Q.4 pseudo-motion-vector reconstruction (the §6.1.1 predictor
  over the actual vectors converted to the pseudo domain, the
  Table-14 MVD applied there, the result expanded to the
  half-integer-or-zero lattice, `[-31.5, 30.5]`-pel default range),
  four 16×16 luminance + two 16×16 chrominance prediction blocks,
  §Q.2.2.2 texture decode + §Q.6 up-sampling (the block-closed
  Figure-Q.8/Q.9 filters with the Implementors' Guide arithmetic-shift
  rounding correction), §Q.2.2.3 summation + clip, the §Q.7.1 default
  block boundary filter (coded-MB condition, §J.3 edge ordering) and
  the §Q.2.3/§Q.2.4 crop back to the reference size. The staged
  subset is the single-segment I/P stream shape at the five standard
  formats. **UMV composes** (round 447): §Q.4 — the pseudo vector is
  `pseudo-PC + difference` with the difference read from Table D.3,
  the UUI-selected Tables-D.1/D.2 range applying to the *pseudo*
  vectors (actual motion reach roughly doubled). AP / DF / AIC / SAC
  / AIV / MQ / Annex K / B-EI-EP combinations inside RRU are refused. The §Q.7.2 Deblocking-Filter
  variant and §Q.5 enlarged OBMC stay as primitives
  (`rru_filter_plane`, `STRENGTH_RRU_INFINITE`).
* **Annex N §N.4.1 / §N.5** — Reference Picture Selection mode (forward
  channel) end-to-end, including **per-GOB** re-selection: the
  `RpsReferenceStore` picture memory keys decoded anchors by their 10-bit
  Temporal Reference (ETR ∥ TR), and `decode_picture_layer_rps` selects
  the §N.4.1.4 prediction reference by the picture-layer TRP (the stored
  picture whose TR equals TRP, or the most recent anchor when TRP is
  absent). For INTER-pictures it further parses the §N.4.1 GOB-layer
  NEWPRED fields (`annex_n::parse_gob_newpred_fields`: TRI / TR / TRPI /
  TRP + BCI, Figure N.2) after each GOB header and re-selects that GOB's
  reference from the store "instead of the last decoded picture" (§N.5) —
  so a single picture can predict different GOBs from different stored
  references (a header-less GOB keeps the previous segment's reference,
  TRP being valid "until the next PSC, GSC or SSC"). The same per-segment
  re-selection threads through the Annex K Slice-Structured driver: each
  subsequent slice's NEWPRED fields (Figure N.3) re-select that slice's
  reference, while the first reduced-header slice keeps the picture-layer
  reference (parallel to GOB 0). A per-segment TRP not in the store
  surfaces the §N.5 forced-INTRA-update case as
  `Error::NotImplemented`. The §N.4.2 **Back-Channel Message syntax**
  is staged (round 443): `annex_n::parse_bcm` / `write_bcm` frame the
  Figure-N.4 ACK / NACK record (BT / URF / TR / ELNUMI-ELNUM /
  BCPM-BSBI / videomux BEPBs / GN-MBA / NACK-only RTR) under a
  caller-supplied `BcmContext` (videomux + GN/MBA width — per the
  §N.4.2.9 NOTE these are properties of the bitstream the message
  applies to, so an in-picture-header BCM stays refused at the
  `plus_ptype` layer where that knowledge is absent).
* **Annex P §P.2 / §P.3** — Reference Picture Resampling: the §P.3 /
  §P.4.2 integer bilinear warp engine (`resample_yuv`) resamples the
  reference picture before motion compensation, driven from the §P.3
  corner displacements, the virtual-frame `H' / V'` powers of two, the
  §P.2.3 fill mode (clip / black / gray / color) and the §P.3 `RCRPR`
  rounding control. Both invocation paths reach pixels through
  `decode_picture_layer`: the §P.1 **implicit** resolution-change case
  (a size-mismatched reference, zero warp / clip / 1/16-pixel) and the
  §P.2 **explicit** RPRP field (WDA + eight Table-D.3 warping parameters
  + fill mode, parsed for INTER / B / Improved-PB pictures). The
  EP-picture explicit-RPR lower-layer-refinement case is not yet staged.
* **Annex G §G.1–§G.5** — PB-frames decode end-to-end, both through the
  per-layer `decode_pb_picture` driver (test-convention wire layout) and —
  new this round — through the headline `decode_sequence` streaming entry
  point: a baseline-PTYPE INTER picture that signals PB-frames mode
  (PTYPE bit 13) is routed to `decode_pb_picture_no_gob0_header`, which
  reads the spec-conformant baseline picture-header tail a real encoder
  emits (§5.1.19 PQUANT, §5.1.20 CPM, §5.1.22 TRB, §5.1.23 DBQUANT,
  §5.1.24/§5.1.25 PEI/PSUPP), elides the group-number-0 GOB header (§5.2.2)
  and tolerates the §5.2 optional later GOB headers. The decoded (B, P)
  pair is spliced into the frame sequence in display order (B before P);
  only the P-part advances the prediction reference and the §G.4 TR the
  next PB-frame scales against. Per macroblock the §5.3 / Figure 10 / Table
  10 PB-frame layer (COD, MCBPC, MODB, CBPB, CBPY, DQUANT, MVD, MVDB) drives
  the six P-blocks then the six §G.4 / §G.5 bidirectionally-predicted
  B-blocks with the Table-6 BQUANT dequant. SAC / Advanced Prediction / AIC
  combinations are refused (§G.1 bars the PLUSPTYPE-gated modes).
* **Annex M §M.1–§M.4** — Improved PB-frames decode end-to-end, both
  through the per-layer `decode_improved_pb_picture` driver and — new this
  round — through `decode_sequence`: an extended-PTYPE picture whose
  §5.1.4.3 MPPTYPE picture-type is `"010"` is detected before dispatch and
  routed to `decode_improved_pb_picture_with_inherited`, which threads the
  §5.1.4.4 inherited-state and returns the decoded (P, BPB) pair plus the
  next-picture OPPTYPE snapshot. The pair is spliced into the sequence in
  display order (BPB before P); only the P-part advances the reference and
  §G.4 TR. Per macroblock the §M.4 / Table M.1 MODB form drives the §M.2
  coding modes (bidirectional / forward with the §M.2.2 left-neighbour MVDB
  predictor / backward). Annex K + Improved-PB, Advanced Prediction, UMV
  and AIC combinations are refused (unstaged §M sub-cases).
* **Annex O §O.4 / §O.5** — temporal-scalability **B-pictures** decode
  end-to-end (`decode_b_picture` / `decode_b_picture_layer`): the Table
  O.1 MBTYPE layer drives Forward / Backward / Bi-dir / INTRA
  reconstruction against the two temporally surrounding anchors, with
  *separate* §O.5.1 forward / backward predictor grids (same-type-only
  median), and §O.5.2 **direct mode** (the COD-skipped row and the
  explicit Direct / Direct+Q rows) deriving the forward / backward
  vectors from the co-located subsequent-anchor vector by the §G.4
  scaling with `MVD = 0` (a co-located INTRA macroblock contributing the
  §O.5.2 zero vector). The EI- and EP-picture enhancement layers also
  reconstruct end-to-end (`decode_ei_picture` / `decode_ep_picture` —
  upward, forward, bidirectional and INTRA prediction). The §N.4.2
  back-channel and the B-picture INTER4V / Advanced-Prediction submodes
  are out of scope, and a B / EI / EP picture signalling UMV is refused
  (§O.4.6 switches MVDFW / MVDBW to Table D.3 in that mode, which this
  path does not stage — round 447).
* **Annex V** — Data-Partitioned Slice mode, both directions
  (round 450): an Annex K slice picture whose OPPTYPE bit 17 is set
  decodes through the §V.2 partitioned layout — the Table V.1 / V.2
  reversible COD + MCBPC codes for every macroblock of the slice (the
  `annex_v` module carries the full RVLC inventories, verified
  prefix-free and bit-reversal-closed), the §V.2.2 Header Marker
  (peeked at codeword boundaries — it cannot occur naturally in HD),
  the §V.2.3 motion-vector partition (Table D.3 codewords over the
  single §V.2.3.2 prediction thread with first-predictor zero and the
  §V.2.3.3 per-codeword emulation rule that replaces §D.2's pair
  rule), the redundant §V.2.4 LMVV validated against the thread, the
  §V.2.5 Motion Vector Marker, and the §V.2.6 coefficient layer
  (CBPY / DQUANT / block data in slice order). The encoder pair
  `encode_intra_picture_dps` / `encode_inter_picture_dps` emits the
  same layout over free-running row slices; DPS INTRA reconstruction
  is pinned byte-identical to the interleaved Annex K coding of the
  same content. Staged subset: INTRA / INTER over free-running
  sequential slices — Rect / ASO submodes, INTER4V classes, PB /
  Improved-PB (Tables V.6 / V.7 are transcribed), Annex O tables and
  the UMV / AP / AIC / DF / AIV / MQ / CPM combinations are refused.
* **Annex W §W.5** — the **reference fixed-point IDCT 0** and its
  informative companion FDCT (round 450): the `w_idct` module is a
  statement-for-statement transcription of the §W.5.3 C listing
  (16-bit storage / 32-bit intermediates, the saturating
  Multiply/Rotate/Round primitives, the two-pass butterfly and the
  HalfSwap∘Transpose∘HalfSwap output permutation), verified
  **bit-identical** to an oracle build of the listing itself over
  20 006 IDCT + 20 005 FDCT random and edge blocks (a pinned subset
  rides in CI). A bitstream announcing `FixedPointIdct(0)` through the
  Annex L PSUPP layer can thus be reconstructed drift-free
  (`idct_w0` / `fdct_w0`); wiring the alternate kernel through the
  per-block reconstruction paths as a decode option is future work.
* **Annex R** — Independent Segment Decoding mode on the GOB
  segmentation (round 450): an extended-PTYPE picture whose OPPTYPE
  bit 12 is set (inherited by UFEP=000 followers) treats each video
  picture segment — the GOBs delimited by the non-empty GOB headers on
  the wire (§R.2) — as a picture of its own: the driver pre-scans the
  byte-aligned GBSCs into a per-row segment-band map before any
  macroblock decodes (a segment's bottom is the *next* header's top,
  which must be known while predicting inside it), motion-compensated
  fetches — single-MV, INTER4V, OBMC and chrominance — clamp into the
  segment's reference band (§R.2 rule 4 border extrapolation, the
  `RefPlane::banded` view), §F.3 OBMC remotes from other segments
  substitute the current vector (rule 2), the §J.3 deblocking filter
  skips every edge crossing a segment boundary (rule 3), and §P
  Reference Picture Resampling is refused (rule 7); the §6.1.1 / §I.3
  predictor confinement (rule 1) is inherent in the per-header
  segmentation. The encoder pair `encode_intra_picture_isd` /
  `encode_inter_picture_isd` (PLUSPTYPE ISD + UMV, one GOB header per
  GOB, motion search and prediction against an edge-replicated
  per-segment reference view) round-trips closed-loop byte-exact;
  clearing the ISD bit in the emitted stream changes the P-picture
  reconstruction, pinning that the treatment fires. ISD + Slice
  Structured (§R.3.1 Rectangular Slice band confinement), ISD +
  Improved-PB / RPS / RRU stay refused.
* **Annex L / Annex W** — supplemental enhancement information
  (round 450): the `annex_l` module stages the §5.1.24/§5.1.25 PEI +
  PSUPP loop primitives (`read_pei_psupp` / `write_pei_psupp`) and the
  §L.2 function layer (`parse_psupp` / `write_psupp`) over the full
  Table L.1 inventory — Do Nothing (with the §L.3 start-code-emulation
  insertion rule applied on write), full / partial / resizing freeze
  requests and freeze release, snapshot tags, video-time and
  progressive-refinement segment tags, the §L.14 Chroma Keying
  Information record (flag-octet-driven key / threshold presence with
  the DSIZE consistency rule enforced), the §L.15 extended function
  escape — plus the Annex W assignments: §W.5 Fixed-Point IDCT and
  the §W.6 Picture Message (CONT / EBIT / MTYPE header, all fourteen
  Table W.2 message types, the §W.6.3.11 interlaced-field and
  §W.6.3.12 picture-number constraint rules, 10-bit picture-number
  accessor / builder). At the picture layer, `extract_psupp` recovers
  a baseline picture's raw PSUPP octets and `insert_psupp` splices SEI
  into any single-segment baseline picture post-hoc (bit-shifting the
  payload and re-padding PSTUF) — validated pixel-neutral over the
  VLC I / P and SAC encoders through `decode_sequence`.
* **Annex O §O.6** — spatial-scalability reference up-sampling: the
  Figure O.8 / O.9 2-D and Figure O.10 / O.11 1-D (horizontal / vertical)
  interpolation filters (`upsample_plane_2d` /
  `upsample_plane_1d_horizontal` / `upsample_plane_1d_vertical`),
  threaded into the EI / EP upward-prediction path so a factor-of-two
  smaller reference layer (§O.1.3) is up-sampled to the enhancement
  geometry before prediction (a non-factor-of-two mismatch is still
  refused with `BadScalabilityReferenceGeometry`).

## Usage

The high-level entry point decodes a whole picture in one call:

```rust,ignore
use oxideav_h263::{decode_picture, DecodeOptions, YuvFrame};

// Decode an INTRA (I) picture — no reference frame needed.
let frame: YuvFrame = decode_picture(&bytes, None, DecodeOptions::default())?;
assert_eq!((frame.luma_width, frame.luma_height), (176, 144));

// Decode the next INTER (P) picture against the previous frame, with
// the Annex J deblocking filter enabled.
let next = decode_picture(
    &p_bytes,
    Some(&frame),
    DecodeOptions { deblock: true, ..DecodeOptions::default() },
)?;
```

For a complete baseline elementary stream (one or more pictures, as
produced by a real encoder), `decode_sequence` splits on Picture Start
Codes and threads the INTER reference automatically:

```rust,ignore
use oxideav_h263::{decode_sequence, DecodeOptions};

// `stream` is a raw .h263 elementary stream (I + P + P + ...).
let frames = decode_sequence(&stream, DecodeOptions::default())?;
```

The lower-level per-layer parsers and per-block reconstruction
primitives the driver composes remain public for callers that need
finer control:

```rust,ignore
use oxideav_core::bits::BitReader;
use oxideav_h263::{
    parse_block, parse_gob_layer, parse_macroblock, parse_picture_header,
    reconstruct_intra_block, BlockContext, H263SourceFormat, MbContext,
};

let mut r = BitReader::new(&bytes);
let pic = parse_picture_header(&mut r)?;
assert_eq!(pic.source_format.luma_dimensions(), Some((176, 144)));

// `r` is now at the first bit of the GOB layer (after any GSTUF).
let gob = parse_gob_layer(&mut r)?;

// One macroblock per spec §5.3, threading the picture's coding
// type and the GOB's QUANT through MbContext.
let mb = parse_macroblock(
    &mut r,
    MbContext {
        picture_coding_type: pic.coding_type,
        advanced_prediction: pic.advanced_prediction,
        quantiser_before: gob.quantiser,
    },
)?;

// One block of the macroblock per §5.4, with the caller deriving the
// INTRADC / coefficient presence from the MB type + CBP bits.
let block = parse_block(
    &mut r,
    BlockContext {
        has_intradc: mb.mb_type.unwrap().is_intra(),
        has_coefficients: false,
    },
)?;

// §6.1 / §6.2 / §6.3.2 intra-block reconstruction.
let samples_8x8 = reconstruct_intra_block(&block, gob.quantiser);
```

## Not yet implemented

* GOB-0-header elision in the *legacy* `decode_picture` /
  `decode_picture_layer` / PB / Annex-K-slice drivers (those still expect
  every GOB, including the topmost, to carry a header on the wire). The
  spec-conformant §5.2.2 elision is available through the dedicated
  `decode_picture_no_gob0_header` baseline entry point.
* Multi-picture sequence demuxing for the legacy per-layer baseline
  drivers (`decode_picture` / PB / Annex-K-slice) — those stay
  caller-side. The `decode_sequence` driver handles both baseline-PTYPE
  and extended-PTYPE (PLUSPTYPE / H.263+) INTRA / INTER picture streams.
* Annex N back-channel **transport**: the §N.4.2 BCM record itself
  parses and writes (see the supported list), but the §5.1.17
  in-picture-header (videomux) placement stays refused — the BCM's
  GN/MBA width belongs to the *other* bitstream the message applies
  to (§N.4.2.9 NOTE), which the picture-header parser cannot know.
  The §N.4.1 per-segment TRP re-selection decodes to pixels on both
  the GOB-layer and the Annex K slice-layer paths.
* Annex R Independent Segment Decoding on the Annex K path (§R.3.1
  Rectangular-Slice rectangle confinement) and combined with Improved
  PB-frames / Annex N RPS / Annex Q RRU (the GOB-segmented I / P shape
  decodes and encodes — see the supported list).
* Annex G PB-frames and Annex M Improved PB-frames now both decode
  end-to-end through `decode_sequence` (see the supported list). Still
  unstaged within those modes: Advanced Prediction / INTER4V B-blocks, UMV
  over-boundary forward vectors (Annex M), AIC, SAC, and the Annex K
  Slice-Structured + Improved-PB combination.
* Annex K + Advanced Prediction under the Rectangular Slice /
  Arbitrary Slice Ordering submodes on the encode side (the decoder
  composes them — §K.1 rules 1/3 confine the predictors and OBMC
  remotes per slice — but the encoders emit only free-running AP
  slices); CPM on the GOB path (the §5.2.4 GSBI field — the Annex K
  SSBI path decodes and encodes, see the supported list).
* Annex E SAC combined with mid-picture GOB headers (the §E.5
  start-code resynchronisation inside a picture), or with Advanced
  Prediction **and** PB-frames simultaneously (each composes with SAC
  on its own — see the supported list); §5.1.4.6 bars the Annex S /
  Annex T combinations outright.
* Annex O CPM-multiplexed / Advanced-Prediction / SAC / Annex-K-slice
  enhancement-layer pictures (refused on the EI / EP / B paths); the
  Annex P explicit-warp resampling engine (see the supported list) is
  not yet threaded into the EP / spatial-scalability path (only the §O.6
  factor-of-two upsample is wired there).
* Annex Q Reduced-Resolution Update combined with Advanced
  Prediction (§Q.5 enlarged OBMC), Deblocking Filter (§Q.7.2 filter
  variant), Annex K slices, custom source formats or mid-picture GOB
  headers — the single-segment standard-format I/P subset (including
  the §Q.4 UMV Table-D.3 pseudo-vector coding, both directions)
  decodes and encodes end-to-end (see the supported list).
* GSBI (CPM = "1"); the EOSBS end marker (the §5.1.27 EOS is emitted
  by the encoder and transparently skipped by `decode_sequence`).
* Encoder: arbitrary (non-stripe) rectangular slice shapes and
  non-row-aligned free-running slices (the slice encoders emit
  row-aligned slices; the rect encoders emit full-height vertical
  stripes); UMV + AP on the *baseline-PTYPE* header (the H.263+ form
  is landed — `encode_inter_picture_ap_umv_plus`); PB-frames with
  non-zero MVDB /
  Annex M Improved-PB; INTRA-refresh inside the AP and PB paths; AIC
  INTRA macroblocks inside a P-picture (only whole AIC I-pictures
  encode so far); within-picture adaptive quantisation for rate
  control (the Annex B HRD loop regulates per picture — the per-MB
  DQUANT / per-GOB GQUANT / per-slice SQUANT primitives are not yet
  driven by the controller).
* RTP: RFC 2190 Mode B/C fragmentation of SAC or Advanced-Prediction
  pictures (no macroblock-aligned bit boundaries / no 4MV predictor
  side channel); RTP transport-header (RFC 3550) concerns
  (sequencing, timestamps, marker bit) stay caller-side.
* Registry adapter subsets: the `H263StreamDecoder` covers every
  stream shape `decode_sequence` covers (scalability / CPM streams
  still take their dedicated drivers), and the `H263StreamEncoder`
  drives the baseline I + P GOP loop (the Annex-mode and
  rate-controlled encoders remain direct-call entry points).

## Testing

The crate carries an extensive unit-test suite over synthetic buffers
built with the spec's bit layout (round-tripped via
`oxideav_core::bits::BitWriter`), including full-table round-trips for
Tables 7 / 8 / 12 / 14 / 16, the inverse-quantisation invariants, IDCT
accuracy against the Annex A error budget, motion / OBMC / deblock /
AIC / PLUSPTYPE / slice-header coverage, the full Table I.2 INTRA VLC
encode round-trip, the §I.3 block-plan closed loop, and end-to-end
picture-decode tests. `tests/encode_roundtrip.rs` additionally drives
the public encode API back through the decoder, including the Annex I
AIC I-picture / auto-mode / AIC+MQ / AIC-sequence encoders (flat AIC
pictures byte-exact, AC-bearing content within the round-trip
tolerance), the self-describing H.263+ (`_plus`) entry points
(byte-exact reconstruction parity with the baseline-PTYPE forms) and
the Annex K slice encoders (single-slice forms byte-exact against
their single-segment counterparts; per-slice SQUANT / AIC-availability
behaviour pinned). `tests/sac_roundtrip.rs` pins the Annex E arm:
byte-identical SAC-vs-VLC reconstruction across sizes and quantisers
— including the Advanced-Prediction (INTER4V + OBMC) and PB-frame
shapes on both parts — SAC elementary streams (pure, mixed with VLC
pictures, and I + AP-P + PB) through `decode_sequence`, static AP /
PB losslessness, §5.1.4.6 barred-combination refusals and a
no-PSC-emulation byte scan. `tests/rate_control.rs` pins the measured
rate-accuracy numbers and Annex B §B.4 conformance of the regulated
GOP encoder. `tests/rtp_roundtrip.rs` round-trips crate-encoded
and vendored conformance streams through the RFC 4629 packetizer
across payload budgets, including redundant-picture-header re-parse
checks, plus the RFC 2190 legs: Mode A GOB packets, Mode B / Mode C
macroblock-boundary fragmentation with the side channel cross-checked
against `enumerate_mb_boundaries`, and bit-granular reassembly.

`tests/fixture_decode.rs` adds end-to-end **conformance** tests against
real H.263 elementary streams (the reference encoder) vendored under
`tests/fixtures/`: sub-QCIF / QCIF / CIF I-only, a QCIF I+P+P sequence,
the QP=2 / QP=31 quantiser-boundary keyframes, an H.263+ (PLUSPTYPE)
QCIF I+P+P stream (`h263p-modern`) that exercises the `decode_sequence`
extended-PTYPE dispatch + custom-PCF framing + GOB-0 elision, a
baseline Annex F stream (`advanced-prediction-mode` — 4MV + OBMC,
including OBMC of skipped macroblocks), an H.263+ Annex J stream
(`deblocking-filter`) and an H.263+ Annex D stream
(`unrestricted-mv-mode` — UMV+ Table D.3 motion + slice-structured +
custom PCF, round 447).
Because §6.2 leaves the inverse-transform arithmetic undefined
and Annex A.7 only bounds the per-pixel peak error at 1, AC-bearing
output is asserted within that ±1 tolerance; the flat sub-QCIF keyframe
(no AC) is checked byte-exact plus a SHA-256 of its reference plane.

Run with `cargo test -p oxideav-h263`.

The `fuzz/` sub-crate (its own workspace) carries four
`cargo fuzz` targets: `decode_sequence` (whole-stream decode, default
and deblock/OBMC-deviation options), `registry_decoder` (the streaming
`oxideav_core` decoder adapter under fuzzer-chosen packetisation, with
mid-stream error recovery via `reset()`), `psupp` (Annex L/W
parse → write → parse idempotence) and `picture_header` (baseline +
PLUSPTYPE picture-layer parsers plus the `extract_psupp` /
`insert_psupp` pair). Seed the corpora from `tests/fixtures/`.

## License

MIT — see [LICENSE](./LICENSE).

[spec]: https://www.itu.int/rec/T-REC-H.263
