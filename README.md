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
| Annex D (UMV)                                    | no     | no     |
| Annex E (SAC arithmetic coding)                  | no     | no     |
| Annex F (Advanced Prediction: 4MV / OBMC)        | no     | no     |
| Annex G (PB-frames) and all B-pictures           | no     | no     |
| Annex I (Advanced Intra Coding)                  | no     | no     |
| Annex K (Slice Structured Mode)                  | no     | no     |
| Annex N (Reference Picture Selection)            | no     | no     |
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
* No Annex D / E / F / I / K / N / P / Q / R / S / T bits are set.
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

The encoder does not yet emit PLUSPTYPE, so encoded output using
Annex J requires the decoder to be told out-of-band.

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
