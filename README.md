# oxideav-h263

Pure-Rust **ITU-T H.263 baseline** video codec — I-picture and P-picture
decode + encode (YUV 4:2:0, sub-QCIF / QCIF / CIF / 4CIF / 16CIF), with
H.263+ PLUSPTYPE header recognition and optional Annex J in-loop
deblocking. Zero C dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-codec = "0.1"
oxideav-h263 = "0.0"
```

The crate depends on `oxideav-mpeg4video` internally — strictly to share
its bitreader and the VLC / IDCT / zig-zag tables (the H.263 baseline
VLCs are identical to the MPEG-4 Part 2 Simple Profile VLCs). Pulling in
this crate does not activate any MPEG-4 decoding behaviour.

## Feature matrix

| Feature                                          | Decode | Encode |
|--------------------------------------------------|:------:|:------:|
| Baseline picture header (PSC / PTYPE / PQUANT)   | yes    | yes    |
| GOB layer (GBSC / GN / GFID / GQUANT)            | yes    | yes    |
| Source formats 1..=5 (sub-QCIF .. 16CIF)         | yes    | yes    |
| I-picture (MB + 8x8 DCT + TCOEF VLC)             | yes    | yes    |
| P-picture (COD / MCBPC / CBPY / MV + half-pel)   | yes    | yes    |
| Annex J — Deblocking filter (out-of-band opt-in) | yes    | yes    |
| PLUSPTYPE / OPPTYPE header parse (H.263+)        | yes    | no     |
| Annex J via PLUSPTYPE DF bit (auto-deblock)      | yes    | no     |
| Annex D (UMV)                                    | yes    | yes    |
| Annex E (SAC arithmetic coding) — I-pictures     | yes    | yes    |
| Annex E (SAC arithmetic coding) — P-pictures     | no     | no     |
| Annex F (Advanced Prediction: 4MV / OBMC)        | no     | no     |
| Annex G (PB-frames) — header + per-MB syntax     | yes    | yes    |
| Annex I (Advanced Intra Coding)                  | no     | no     |
| Annex K (Slice Structured Mode)                  | no     | no     |
| Annex N (RPS — picture header + multi-ref)       | yes    | yes    |
| Annex P (Reference Picture Resampling)           | no     | no     |
| Annex Q (Reduced-Resolution Update)              | no     | no     |
| Annex T (Modified Quantization)                  | no     | no     |
| Custom picture clock frequency (CPCFC)           | no     | no     |
| Custom picture size (non-standard dimensions)    | no     | no     |
| CPM (continuous-presence multipoint)             | no     | no     |

### PLUSPTYPE support scope

The decoder recognises H.263+ PLUSPTYPE / OPPTYPE (ITU-T Rec. H.263
01/2005, source-format code `111`) and walks UFEP, MPPTYPE, OPPTYPE,
CPFMT, CPCFC / ETR. In practice it accepts a PLUSPTYPE-framed picture
only when all of the following hold:

* UFEP = `001` (full OPPTYPE present) — cross-picture feature-flag
  inheritance (UFEP = `000`) is not yet tracked.
* No Annex D / E / F / I / K / P / Q / R / S / T bits are set.
  (Annex N — RPS — IS now accepted; see below.)
* No custom picture clock frequency.
* Any custom picture size in CPFMT happens to coincide with one of the
  standard source formats (sub-QCIF / QCIF / CIF / 4CIF / 16CIF).
* Picture-type code is I or P only (no B-pictures, EI / EP, or
  Improved-PB).

Anything outside that envelope returns `Error::Unsupported` with a
message naming the specific feature / annex so the caller can distinguish
"unparseable" from "deliberately out of scope". The `DF` bit (Annex J
deblocking) is honoured automatically when it is signalled.

### Annex J — Deblocking filter

The in-loop deblocking filter is implemented bit-exact in
[`deblock::deblock_picture`] and can be enabled in three ways:

1. On baseline streams without a PLUSPTYPE header there is no way for
   the bitstream to signal the DF bit. Both sides must be configured
   explicitly:

   ```rust
   use oxideav_h263::decoder::H263Decoder;
   use oxideav_h263::encoder::H263Encoder;
   use oxideav_core::CodecId;

   let mut enc = H263Encoder::from_params(&params)?;
   enc.set_enable_annex_j(true);

   let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
   dec.set_enable_annex_j(true);
   # Ok::<(), oxideav_core::Error>(())
   ```

2. On a PLUSPTYPE-framed stream that asserts `DF=1` in OPPTYPE, the
   decoder auto-enables deblocking for that picture regardless of the
   out-of-band flag.

3. Left off by default — baseline H.263 bit-for-bit compatibility with
   existing streams.

The encoder does not yet emit PLUSPTYPE for Annex J — encoded output
using Annex J requires the decoder to be told out-of-band. (Annex N
RPS, below, IS emitted via PLUSPTYPE.)

### Annex N — Reference Picture Selection

Round 13 wires Annex N (RPS) on both decoder and encoder sides:

* **Decoder** (`picture::parse_picture_header`) accepts a PLUSPTYPE
  picture header carrying OPPTYPE bit 11 (RPS) = 1, reads RPSMF /
  TRPI / TRP / BCI per §5.1.13–§5.1.16, and surfaces the parsed
  values on `PictureHeader.rps_mode` / `rpsmf` / `trpi` / `trp` /
  `bci_present`. The new fields default to off / `None` / 0 / false on
  baseline-PTYPE streams. Back-channel-message (BCM) parsing is out
  of scope — a stream that signals BCI = `1` (BCM follows) is
  rejected with a specific `Unsupported` diagnostic citing §N.4.2.
* **Decoder picture-memory cache** — every successfully decoded picture
  is pushed into a small bounded LRU keyed by its TR (default
  capacity 4, tunable via [`H263Decoder::set_rps_cache_capacity`]).
  When a P-picture's `trpi` is set the decoder looks up its `trp` in
  the cache and uses that picture as the motion-compensation
  reference; cache misses degrade gracefully to the most-recent
  anchor (matching §N.5's "use the most recent temporally previous
  anchor picture" fall-back).
* **Encoder** opt-in via [`H263Encoder::set_enable_annex_n_rps`].
  When on, every picture is emitted with a PLUSPTYPE-form picture
  header (source-format `111`, UFEP=001, full OPPTYPE, RPS bit set),
  RPSMF=`100` (no back-channel signals needed), TRPI=0 on every
  P-picture (decoder uses the most recent anchor), BCI=`01`. The MB
  layer underneath is unchanged baseline 1-MV inter — bit-identical
  to the non-RPS encoder for the same DCT/quant/MV pipeline.
* **Combination guards** — RPS + UMV / SAC / AP returns
  `Error::Unsupported` at `send_frame` (round-13 scope).
* **ffmpeg interop** — ffmpeg 8.1's H.263 decoder explicitly logs
  "Reference Picture Selection not supported" and falls through to
  best-effort decode-with-error-concealment. The I-picture decodes
  cleanly (~51 dB cross-decode on the testsrc clip — same as the
  non-RPS Annex-D UMV baseline); P-pictures get concealed by ffmpeg
  but our self-decoder reproduces the source at PSNR ≥ 30 dB.

### Annex G — PB-frames

Round 14 wired the Annex G framing on both sides; round 15 added the
B-block residual emission so the B-half is no longer a pure MC
predictor:

* **Picture header** — `picture::parse_picture_header` accepts PTYPE
  bit 13 (PBFR) and reads the §5.1.22 / §5.1.23 TRB (3 bits) +
  DBQUANT (2 bits) tail after CPM/PSBI. Both fields surface on
  `PictureHeader.pb_frames` / `trb` / `dbquant`.
* **Per-MB syntax** (`mb::decode_p_mb_pb`) — between MCBPC and CBPY
  (per §5.3 Figure 10) the decoder reads MODB (Table 11/H.263), then
  optional CBPB (6 bits) and MVDB (Table 14 differential applied as
  the §G.4 MVDB delta). MODB = `0` means "no CBPB, no MVDB"; `10`
  means "MVDB only"; `11` means "CBPB + MVDB". When CBPB is
  non-zero, the per-block B-residual (TCOEF dequantised at BQUANT =
  `bquant_from_quant(quant, dbquant)` — Table 6 / §5.1.23) is
  decoded after the six P-block coefficients, IDCT'd, and stored on
  `PbMbInfo.b_residual` for the §G.5 reconstruction.
* **§G.4 / §G.5 reconstruction** ([`pb`] module) — given the
  co-located P-MB's MV, MVDB delta, TRB, and TRD the helper derives
  forward `MVF` and backward `MVB` (per-block + chroma via §F.2 sum
  rounded by Table F.1). `predict_b_block` returns the §G.5
  bidirectional prediction (forward + backward average inside the
  region where `MVB` maps into the freshly-reconstructed P-MB,
  forward-only outside); `reconstruct_b_block` then adds the
  per-block residual and clips to `[0, 255]`.
* **Encoder** opt-in via [`H263Encoder::set_enable_annex_g_pb`]. When
  on, every P-picture sets PTYPE bit 13 and the MB layer emits the
  full MODB / CBPB / MVDB / B-residual stream. For each MB the
  encoder (1) quantises + reconstructs the P-half blocks into the
  `recon` picture, (2) computes the §G.5 prediction for each of the
  6 B-blocks against the freshly-reconstructed P-MB, (3) subtracts
  from the **input frame** pels (the streaming 1-input-per-PB-pair
  model uses the input as the B-source), forward-DCTs, quantises at
  BQUANT, and picks CBPB bits from per-block any-nonzero. MODB = `11`
  carries CBPB + MVDB (MVDB = `(0, 0)` pure differential); MODB = `0`
  is emitted only when no B-block has any residual. TRB / DBQUANT
  are tunable via `set_pb_trb` / `set_pb_dbquant`.
* **Combination guards** — PB + UMV / SAC / AP / RPS returns
  `Error::Unsupported` at `send_frame` (round-15 scope is still
  baseline 1-MV inter only). PB + Annex F is also rejected on the
  decoder side.
* **Decoded output** — every PB-frame produces TWO `VideoFrame`s in
  display order: B first, then P. The B-frame is *not* stored as the
  motion-compensation reference for subsequent pictures (per §G.1 /
  §5.1.22) — only the P-half goes into the reference cache.
* **Round-trip PSNR** — on a synthetic moving-square QCIF clip,
  encode 5 frames as `[I, PB, PB, PB, PB]` and decode → I-frame +
  every P-half lands at **68.1 dB** (essentially lossless thanks to
  the small uniform regions); B-halves at **55.4-57.1 dB** with
  round-15 residual emission at PQUANT = 5 / DBQUANT = 0 (BQUANT =
  6) — a +27 dB jump from round-14's 28.8 dB MC-only baseline. At
  DBQUANT = 3 (BQUANT = 10) the B-half drops to 51.6 dB, as expected
  from the coarser quant.
* **ffmpeg interop** — purely informational. ffmpeg 8.1's H.263
  decoder accepts our PB-frames stream without error logs but its
  PB-frames support is partial; cross-decode parity is not asserted.

## Quick use

### Decoder

```rust
use oxideav_codec::Decoder;
use oxideav_core::{CodecId, CodecParameters, Packet, TimeBase};
use oxideav_h263::decoder::H263Decoder;

let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
let pkt = Packet::new(0, TimeBase::new(1, 30), bitstream_bytes);
dec.send_packet(&pkt)?;
match dec.receive_frame() {
    Ok(oxideav_core::Frame::Video(vf)) => {
        // vf.format == PixelFormat::Yuv420P
        // vf.planes[0..2] = Y, Cb, Cr
    }
    Err(oxideav_core::Error::NeedMore) => { /* feed more packets */ }
    Err(e) => return Err(e.into()),
    _ => {}
}
# Ok::<(), oxideav_core::Error>(())
```

### Encoder

```rust
use oxideav_codec::Encoder;
use oxideav_core::{CodecId, CodecParameters, Frame, PixelFormat, Rational};
use oxideav_h263::encoder::make_encoder;

let mut params = CodecParameters::video(CodecId::new(oxideav_h263::CODEC_ID_STR));
params.width = Some(176);
params.height = Some(144);
params.pixel_format = Some(PixelFormat::Yuv420P);
params.frame_rate = Some(Rational::new(30, 1));
let mut enc = make_encoder(&params)?;
enc.send_frame(&Frame::Video(yuv420_frame))?;
let pkt = enc.receive_packet()?; // first frame is always an I-picture
# Ok::<(), oxideav_core::Error>(())
```

### Codec ID

- Codec: `"h263"`; accepted pixel format `Yuv420P`; only the five
  standard source-format dimensions.
- MP4 sample entries `s263` and `h263` map to this id; raw `.h263`
  elementary streams probe to it as well.

## License

MIT — see [LICENSE](LICENSE).
