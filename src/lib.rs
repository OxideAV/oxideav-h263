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
//! * **Annex E — Syntax-based Arithmetic Coding (I- and P-pictures)**: the
//!   §E.2 / §E.3 arithmetic encoder + decoder + every §E.8 cumulative-
//!   frequency model live in [`sac`], and the MB-layer bridge that swaps
//!   the VLC code path for SAC symbols lives in [`mb_sac`]. Round 13
//!   landed the I-picture path (MCBPC_INTRA, CBPY_INTRA, INTRADC,
//!   TCOEF1/2/3/r_intra + SIGN + LAST_INTRA / RUN_INTRA / LEVEL_INTRA
//!   escape); round 14 wires the P-picture path (cumf_COD,
//!   cumf_MCBPC_no4MVQ, cumf_CBPY (raw, NOT XOR'd per §E.7), cumf_DQUANT,
//!   cumf_MVD, INTER cumf_TCOEF1/2/3/r + SIGN + LAST/RUN/LEVEL escape).
//!   Encoder-side opt-in via [`encoder::H263Encoder::set_enable_annex_e`]
//!   sets PTYPE bit 11 on every I- and P-picture; the decoder front-end
//!   picks the SAC body driver automatically when the picture-header SAC
//!   bit is set. The §E.5 PSC_FIFO emulation-prevention (14-zero
//!   stuffing) is handled by [`sac::PscFifoWriter`] /
//!   [`sac::PscFifoReader`]. GOB-boundary `encoder_flush` /
//!   `decoder_reset` boundaries (§E.7 / §E.3) are wired in
//!   [`mb_sac::encode_p_picture_sac_body`] and
//!   [`mb_sac::decode_p_picture_sac`] — opt-in via
//!   [`encoder::encode_p_picture_sac_with_recon_opts`] for streams that
//!   want mid-picture resync points; the default (mirroring the VLC
//!   P-encoder) is to emit a single SAC segment per picture for byte-
//!   exact reconstruction parity. Self-roundtrip and SAC-vs-VLC
//!   pixel-identical reconstruction tests cover both. Round 15 wires the
//!   SAC + Annex F (Advanced Prediction / 4MV / OBMC) combination: when
//!   both `set_enable_annex_e(true)` AND `set_enable_annex_f(true)` are
//!   on, the P-picture sets PTYPE bits 11 (SAC) AND 12 (AP), the MB
//!   layer routes through `cumf_MCBPC_4MVQ` (§E.8 — `Inter4MV` rows
//!   16..=19, `Inter4MV+Q` rows 21..=24), each `Inter4MV` MB emits four
//!   MVDs via the §F.2 Figure F.1 redefined predictor, and §F.3 OBMC
//!   blending is applied on the local reconstruction so the decoder's
//!   pass-2 OBMC produces the same picture. SAC↔VLC byte-identical
//!   reconstruction parity holds for the AP path as well as the
//!   non-AP path. Round 16 wires SAC + Annex J (deblocking filter) per
//!   §E.7 — when DF is on (out-of-band on baseline PTYPE via
//!   `set_enable_annex_j` on both encoder and decoder), the MCBPC
//!   selector flips to `cumf_MCBPC_4MVQ` even with 1-MV macroblocks,
//!   matching what AP already does. Round 16 also adds per-GOB resync
//!   to the SAC + AP path
//!   ([`encoder::encode_p_picture_sac_ap_with_recon_opts`]'s
//!   `emit_gob_headers` knob): every GOB row boundary fires the §E.5
//!   SAC-flush + GOB-header bridge + fresh segment, mirroring the
//!   non-AP SAC path's behaviour. The MV predictor and §F.3 OBMC
//!   reconstruction stay full-picture across resync points (§F.3 allows
//!   AP-mode predictors to reach across segments outside Slice
//!   Structured / ISD).
//!
//! * **Annex N — Reference Picture Selection** (round 13). Both decode +
//!   encode emit the picture-header path: PLUSPTYPE OPPTYPE bit 11 (RPS),
//!   RPSMF (5.1.13 — `100`/`101`/`110`/`111`), TRPI (5.1.14), TRP (5.1.15),
//!   and BCI (5.1.16 — `01` accepted; `1` + BCM is rejected as round-14
//!   follow-up). The decoder maintains an LRU picture-memory cache keyed
//!   by TR (default capacity 4, tunable via
//!   [`decoder::H263Decoder::set_rps_cache_capacity`]); when a P-picture's
//!   parsed `trpi` is set, [`decoder::H263Decoder`] looks up `trp` in the
//!   cache and uses that picture as the motion-compensation reference,
//!   degrading gracefully to "most recent anchor" on cache miss
//!   (mirrors §N.5 fall-back). The encoder opts in via
//!   [`encoder::H263Encoder::set_enable_annex_n_rps`]; round-13 emits
//!   RPSMF=`100` (no back-channel) + TRPI=0 (decoder uses most recent
//!   anchor) + BCI=`01`, with the MB layer unchanged baseline 1-MV inter.
//!
//! * **Annex G — PB-frames mode (rounds 14-15 — encoder + decoder)**: a PB-
//!   frame couples a P-picture with a B-picture as one transmission unit.
//!   The encoder side opts in via
//!   [`encoder::H263Encoder::set_enable_annex_g_pb`] (PTYPE bit 13 set;
//!   TRB / DBQUANT in the picture header tail per §5.1.22 / §5.1.23).
//!   Per-MB the encoder emits the standard P-block fields followed by
//!   `MODB` (Table 11 VLC), optional `CBPB` (6 bits), the §G.4
//!   forward + backward MV derivation (with optional `MVDB` delta — same
//!   VLC family as MVD), and the §G.5 bidirectional B-block reconstruction.
//!   Round 15 wires the B-block residual coding: per-MB the encoder
//!   computes the §G.5 prediction against the freshly-reconstructed
//!   P-MB, subtracts from the **input frame** pels to form a residual,
//!   forward-DCTs and quantises at `BQUANT = bquant_from_quant(quant,
//!   dbquant)` (§5.1.23 / Table 6), and picks CBPB bits from per-block
//!   any-nonzero. MODB = `11` then carries CBPB + MVDB (MVDB = `(0, 0)`
//!   pure differential); MODB = `0` is emitted when no B-block has any
//!   residual. The decoder side mirrors the header parse (TRB / DBQUANT),
//!   the per-MB MODB / CBPB / MVDB read, and the §G.4 forward+backward
//!   MV derivation, then dequantises + IDCTs the per-CBPB B-residual at
//!   BQUANT and folds it into the §G.5 reconstruction. Output is two
//!   `VideoFrame`s per PB picture (B first, then P — display order).
//!
//! Out of scope (returns `Error::Unsupported`):
//! * Annex M (Improved PB-frames). Annex G (legacy PB-frames) is wired in
//!   round 14; Improved PB carries a 3-code MODB VLC and per-MB B-mode
//!   selection that is still pending.
//! * Annex I (Advanced Intra Coding), Annex K (Slice
//!   Structured Mode — detected with a specific diagnostic since ffmpeg's
//!   `h263p -umv 1` bundles it with Annex D), Annex P
//!   (Reference Picture Resampling), Annex T (Modified Quantization).
//! * Annex N back-channel-message body (BCM) — §N.4.2 BT/URF/TR/ELNUMI/
//!   ELNUM/BCPM/BSBI/BEPB1/2/GN/MBA/RTR/BSTUF parsing is rejected with
//!   a specific `Unsupported` diagnostic when BCI = "1".
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
pub mod mb_sac;
pub mod motion;
pub mod pb;
pub mod picture;
pub mod sac;
pub mod start_code;

use oxideav_core::{CodecCapabilities, CodecId, CodecTag};
use oxideav_core::{CodecInfo, CodecRegistry};

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
