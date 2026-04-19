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
//! * **PLUSPTYPE parse** (H.263+, ITU-T Rec. H.263 01/2005 Annex U): the
//!   decoder recognises extended picture headers carrying source-format
//!   code `111`, reads UFEP / MPPTYPE / OPPTYPE + CPFMT, and either returns
//!   a normal `PictureHeader` (when the stream sticks to baseline features
//!   on a standard source size + optional DF) or an `Error::Unsupported`
//!   naming the specific annex the stream requires.
//! * Reuses VLC tables and IDCT/dequantisation from `oxideav-mpeg4video`
//!   (the MPEG-4 Part 2 VLCs are identical to the H.263 baseline ones).
//!
//! Out of scope (returns `Error::Unsupported`):
//! * PB-frames mode (§G) and every B-picture flavour.
//! * Annex D (Unrestricted MV), Annex E (SAC), Annex F (Advanced Prediction
//!   — 4MV/OBMC), Annex G (PB-frames), Annex I (Advanced Intra Coding),
//!   Annex K (Slice Structured Mode), Annex N (RPS), Annex P (Reference
//!   Picture Resampling), Annex T (Modified Quantization).
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
pub mod start_code;

use oxideav_codec::CodecRegistry;
use oxideav_core::{CodecCapabilities, CodecId};

/// The canonical oxideav codec id for ITU-T H.263 baseline video.
///
/// MP4 sample entries `s263` and `h263` map to this id; raw `.h263`
/// elementary-stream files probe to it as well.
pub const CODEC_ID_STR: &str = "h263";

/// Register the H.263 decoder + I-picture encoder with a codec registry.
pub fn register(reg: &mut CodecRegistry) {
    let dec_caps = CodecCapabilities::video("h263_sw")
        .with_lossy(true)
        .with_intra_only(false)
        .with_max_size(1408, 1152);
    reg.register_decoder_impl(CodecId::new(CODEC_ID_STR), dec_caps, decoder::make_decoder);
    let enc_caps = CodecCapabilities::video("h263_sw")
        .with_lossy(true)
        .with_intra_only(false)
        .with_max_size(1408, 1152);
    reg.register_encoder_impl(CodecId::new(CODEC_ID_STR), enc_caps, encoder::make_encoder);
}
