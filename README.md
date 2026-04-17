# oxideav-h263

Pure-Rust ITU-T H.263 baseline video decoder + encoder for oxideav.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace) framework — a
100% pure Rust media transcoding and streaming stack. No C libraries, no FFI
wrappers, no `*-sys` crates.

## Feature matrix

| Feature                                     | Decode | Encode |
|---------------------------------------------|:------:|:------:|
| Baseline picture header (PSC/PTYPE/PQUANT)  | yes    | yes    |
| GOB layer (GBSC/GN/GFID/GQUANT)             | yes    | yes    |
| Source formats 1..=5 (sub-QCIF..16CIF)      | yes    | yes    |
| I-picture (MB + 8×8 DCT + TCOEF VLC)        | yes    | yes    |
| P-picture (COD/MCBPC/CBPY/MV + half-pel MC) | yes    | yes    |
| **Annex J — Deblocking filter** (*)         | yes    | yes    |
| Annex D (UMV)                               | no     | no     |
| Annex E (SAC arithmetic coding)             | no     | no     |
| Annex F (Advanced Prediction: 4MV / OBMC)   | no     | no     |
| Annex G (PB-frames)                         | no     | no     |
| Annex I (Advanced Intra Coding)             | no     | no     |
| Annex K (Slice Structured Mode)             | no     | no     |
| Annex N (RPS)                               | no     | no     |
| Annex P (Reference Picture Resampling)      | no     | no     |
| Annex T (Modified Quantization)             | no     | no     |
| PLUSPTYPE / H.263+ extended picture format  | no     | no     |
| CPM (continuous-presence multipoint)        | no     | no     |
| B-pictures of any flavour                   | no     | no     |

(*) Annex J is implemented as an **out-of-band** toggle rather than a
bitstream-signalled flag — this crate does not yet parse the PLUSPTYPE /
OPPTYPE header extension that would carry the DF bit. Both the encoder and
the decoder expose `set_enable_annex_j(bool)` methods (on
`H263Encoder` and `H263Decoder` respectively); both sides must opt in
for the reconstructions to stay in sync. Default is off, preserving
baseline H.263 bit-for-bit compatibility with existing streams.

Example:

```rust
use oxideav_h263::decoder::H263Decoder;
use oxideav_h263::encoder::H263Encoder;
use oxideav_core::CodecId;

let mut enc = H263Encoder::from_params(&params).unwrap();
enc.set_enable_annex_j(true);

let mut dec = H263Decoder::new(CodecId::new(oxideav_h263::CODEC_ID_STR));
dec.set_enable_annex_j(true);
```

## Usage

```toml
[dependencies]
oxideav-h263 = "0.0"
```

## License

MIT — see [LICENSE](LICENSE).
