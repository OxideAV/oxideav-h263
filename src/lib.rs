//! Pure-Rust ITU-T H.263 baseline video decoder + encoder (I + P pictures).
//!
//! Scope:
//! * H.263 picture header (PSC, TR, PTYPE, source format, PQUANT, CPM, PEI/
//!   PSPARE loop) — Annex C of ITU-T Rec. H.263 (02/98).
//! * GOB header parse + emit (GBSC, GN, GFID, GQUANT) — §5.2.
//! * **I-picture decode** — MB layer (MCBPC for I, CBPY, optional DQUANT),
//!   block layer (8-bit INTRADC + AC TCOEF VLC), H.263 dequantisation, 8×8
//!   IDCT, output 4:2:0 YUV.
//! * **I-picture encode** — forward 8×8 DCT, H.263 quant, MCBPC (intra) +
//!   CBPY (no XOR for intra) + INTRADC with the spec's 0x00/0x80/0xFF
//!   handling + AC TCOEF VLC encode with `last + run(6) + level(8)` escape.
//! * **P-picture decode** — COD/MCBPC inter/CBPY/MV per §5.3.5 + §5.3.7;
//!   half-pel bilinear motion compensation on a single previous reference;
//!   inter TCOEF texture with the usual H.263 escape.
//! * **P-picture encode** — 3-step diamond + half-pel refinement motion
//!   estimator on the previous reconstructed frame, COD flag, MCBPC inter +
//!   CBPY XOR + MVD VLC + inter AC encode.
//! * Source formats 1..=5: sub-QCIF, QCIF, CIF, 4CIF, 16CIF.
//! * **Annex J — Deblocking filter**: applied on both the encoder (before
//!   the reconstruction is cached as the motion-compensation reference) and
//!   the decoder (to the emitted frame + reference). Opt-in via
//!   [`encoder::H263Encoder::set_enable_annex_j`] /
//!   [`decoder::H263Decoder::set_enable_annex_j`], or auto-enabled on the
//!   decoder side when a PLUSPTYPE-carrying stream asserts the `DF` bit in
//!   its OPPTYPE. See [`deblock::deblock_picture`].
//! * **Annex D — Unrestricted Motion Vectors (decode path)**: both the
//!   baseline-PTYPE form (§D.2 sign-of-predictor reconstruction over Table 14
//!   VLC) and the PLUSPTYPE form (Table D.3 "regular-structure MVD VLC" + UUI
//!   range selection per Tables D.1/D.2 + §D.2 last-paragraph MVD-pair
//!   start-code-emulation stuffing bit) are accepted. The PLUSPTYPE path
//!   reconstructs each MV component as `predictor + differential` directly
//!   (no wrap) and enforces the UUI="1" range limit (or leaves it unlimited
//!   under UUI="01"). The picture-edge extrapolation required by §D.1
//!   (samples outside the reference picture replicate from the nearest valid
//!   edge) is handled by [`interp::predict_block`] via `x.clamp(0, w-1)`.
//! * **Annex F — Advanced Prediction (decode + encode path)**: the decoder
//!   accepts PTYPE bit 12 (AP) on baseline streams and OPPTYPE bit 7 inside
//!   a PLUSPTYPE block. When AP is active the P-picture decoder runs a
//!   two-pass MB traversal: pass 1 decodes the MCBPC (accepting Inter4MV /
//!   Inter4MVQ codes with the MVD + MVD2-4 sequence of §5.3.8), the
//!   per-block MV predictors from §F.2 Figure F.1, and the residual
//!   coefficients; pass 2 applies §F.3 overlapped-block motion
//!   compensation (H0 / H1 / H2 weight tables from Figures F.2 / F.3 /
//!   F.4) to form the final luma predictor before adding the residual.
//!   Chroma uses the MVDCHR of §F.2 (sum of 4 luma MVs, divided by 8,
//!   rounded via Table F.1). OBMC is applied to every inter MB in AP
//!   mode including skipped (`COD == 1`) MBs, per the §F.3 note.
//!   The encoder mirrors this layout: when
//!   [`encoder::H263Encoder::set_enable_annex_f`] is on, each P-picture
//!   sets PTYPE bit 12 and uses a 2-pass emit path (pass 1: decide
//!   skipped / intra / 1MV / 4MV per-MB via separate 16×16 + four 8×8
//!   SAD searches; pass 2: emit the bitstream with the OBMC-blended
//!   predictor as the residual target). The reconstruction is bit-
//!   identical to what the decoder's two-pass §F.3 path produces.
//! * **PLUSPTYPE parse** (H.263+, ITU-T Rec. H.263 01/2005 Annex U): the
//!   decoder recognises extended picture headers carrying source-format
//!   code `111`, reads UFEP / MPPTYPE / OPPTYPE + CPFMT, and either returns
//!   a normal `PictureHeader` (when the stream sticks to baseline features
//!   on a standard source size + optional DF) or an `Error::Unsupported`
//!   naming the specific annex the stream requires.
//! * Reuses VLC tables and IDCT/dequantisation from `oxideav-mpeg4video`
//!   (the MPEG-4 Part 2 VLCs are identical to the H.263 baseline ones).
//!
//! * **Annex E — Syntax-based Arithmetic Coding (core)**: the §E.2 / §E.3
//!   arithmetic encoder + decoder and every §E.8 cumulative-frequency model
//!   live in [`sac`]. The §E.5 PSC_FIFO emulation-prevention (14-zero
//!   stuffing) is handled by [`sac::PscFifoWriter`] / [`sac::PscFifoReader`].
//!   Round-trip unit tests walk every symbol of every model. End-to-end
//!   integration with the picture / MB / block decoder is the next step;
//!   until that wiring is in place, SAC-signalled streams are rejected at
//!   the picture-header layer.
//!
//! Out of scope (returns `Error::Unsupported`):
//! * PB-frames mode (§G) and every B-picture flavour.
//! * Annex E wiring (VLC→SAC swap at the MB layer); the arithmetic coder
//!   is implemented in [`sac`], but SAC-active streams are not yet driven.
//! * Annex G (PB-frames), Annex I (Advanced Intra Coding), Annex K (Slice
//!   Structured Mode — detected with a specific diagnostic since ffmpeg's
//!   `h263p -umv 1` bundles it with Annex D), Annex N (RPS), Annex P
//!   (Reference Picture Resampling), Annex T (Modified Quantization).
//! * H.263+ custom picture clock frequency / custom picture sizes that don't
//!   match one of the standard source formats (sub-QCIF/QCIF/CIF/4CIF/16CIF).
//! * CPM continuous-presence multipoint mode.
//!
//! No runtime dependencies beyond `oxideav-core`, `oxideav-codec`, and
//! `oxideav-mpeg4video` (whose VLC tables we share).

#![allow(clippy::needless_range_loop)]
#![allow(clippy::too_many_arguments)]

pub mod block;
pub mod dct;
pub mod deblock;
pub mod decoder;
pub mod enc_tables;
pub mod encoder;
pub mod gob;
pub mod interp;
pub mod mb;
pub mod motion;
pub mod picture;
pub mod sac;
pub mod start_code;

use oxideav_codec::{CodecInfo, CodecRegistry};
use oxideav_core::{CodecCapabilities, CodecId, CodecTag};

/// The canonical oxideav codec id for ITU-T H.263 baseline video.
///
/// MP4 sample entries `s263` and `h263` map to this id; raw `.h263`
/// elementary-stream files probe to it as well.
pub const CODEC_ID_STR: &str = "h263";

/// Register the H.263 decoder + I-picture encoder with a codec registry.
pub fn register(reg: &mut CodecRegistry) {
    let caps = CodecCapabilities::video("h263_sw")
        .with_lossy(true)
        .with_intra_only(false)
        .with_max_size(1408, 1152);
    // AVI FourCC claims — H.263 baseline + the vendor-prefixed variants
    // from ITU-T Annex X encoders (VivoActive, UB Video, Intel, etc.).
    // All unambiguous.
    reg.register(
        CodecInfo::new(CodecId::new(CODEC_ID_STR))
            .capabilities(caps)
            .decoder(decoder::make_decoder)
            .encoder(encoder::make_encoder)
            .tags([
                CodecTag::fourcc(b"H263"),
                CodecTag::fourcc(b"U263"),
                CodecTag::fourcc(b"M263"),
                CodecTag::fourcc(b"ILVR"),
                CodecTag::fourcc(b"VX1K"),
                CodecTag::fourcc(b"VIV1"),
                CodecTag::fourcc(b"X263"),
                CodecTag::fourcc(b"T263"),
                CodecTag::fourcc(b"S263"),
                CodecTag::fourcc(b"L263"),
            ]),
    );
}
