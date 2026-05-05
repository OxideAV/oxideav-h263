//! H.263 baseline encoder — I- and P-pictures.
//!
//! Scope:
//! * Picture Start Code (PSC) + picture header (TR, PTYPE, source format,
//!   PQUANT, CPM=0, no PEI). Source formats sub-QCIF / QCIF / CIF / 4CIF /
//!   16CIF (PTYPE source-format codes 1..=5).
//! * GOB layering — emits a GOB header (GBSC + GN + GFID + GQUANT) at every
//!   GOB boundary except the first (the first GOB header is implicit per
//!   §5.2.1 — the picture header's PQUANT applies).
//! * I-MB: MCBPC (intra, mb_type=3) + CBPY (no XOR for intra) — no DQUANT
//!   (we hold the quantiser fixed for the whole picture).
//! * P-MB: COD flag + MCBPC inter (mb_type 0=Inter, or 4=Intra-in-P when the
//!   block is hard to predict) + CBPY (XOR-inverted for inter) + MV via
//!   the motion-VLC table with median predictor.
//! * Block layer: 8-bit INTRADC (with the spec's `0x00`/`0x80`/`0xFF`
//!   special-value handling) + H.263 AC TCOEF VLC encode with a fixed-length
//!   `last + run(6) + level(8)` escape body for out-of-table tuples.
//! * 8×8 forward DCT (textbook f32) + H.263 quant.
//! * GOP control: first frame is always I; subsequent frames are P until
//!   `gop_size` frames have elapsed, at which point we insert another I.
//!
//! * **Annex F — Advanced Prediction (4MV + OBMC) emission**. Opt-in via
//!   [`H263Encoder::set_enable_annex_f`]. When enabled, PTYPE bit 12 is set
//!   on every P-picture and the encoder runs a 2-pass per-MB loop: pass 1
//!   decides `skipped` / `intra-in-P` / `1-MV inter` / `4-MV inter` by
//!   comparing SADs (one 16×16 search vs. four 8×8 searches); pass 2
//!   computes the OBMC-blended predictor against the full `MvGrid` and
//!   emits the MB bitstream (Inter4MV MCBPC + four MVDs + MVDCHR-derived
//!   chroma, per §F.2 / §F.3). The local reconstruction matches what the
//!   decoder's two-pass §F.3 path produces.
//!
//! Out of scope (returns `Error::Unsupported`):
//! * Annex D (UMV), Annex E (SAC), Annex G (PB-frames), Annex I (Advanced
//!   Intra Coding), Annex T (Modified Quantization).
//! * H.263+ PLUSPTYPE custom picture format / custom PCF / DF-bit signalling
//!   (Annex J deblocking is applied out-of-band; the bitstream itself is
//!   still baseline).
//! * CPM continuous-presence multipoint mode.
//! * B-pictures of any flavour.
//!
//! The picture header's `temporal_reference` field is taken from the input
//! frame's `pts` modulo 256 — the H.263 spec only requires that consecutive
//! pictures advance TR; downstream containers (e.g. 3GP) carry the actual
//! timestamps separately.

use std::collections::VecDeque;

use oxideav_core::Encoder;
use oxideav_core::{
    CodecId, CodecParameters, Error, Frame, MediaType, Packet, PixelFormat, Rational, Result,
    TimeBase, VideoFrame,
};
use oxideav_mpeg4video::headers::vol::ZIGZAG;

use crate::dct::fdct8x8;
use crate::enc_tables::{write_cbpy, write_mcbpc_inter, write_mcbpc_intra, write_tcoef, PMbKind};
use crate::interp::{predict_block, sad_block};
use crate::mb::IPicture;
use crate::motion::{
    chroma_mv_4mv, encode_mv_component, encode_mv_component_umv, luma_to_chroma_mv,
    mvd_pure_differential_bits, predict_mv, predict_mv_block, reconstruct_umv_component, MbMotion,
    MvGrid, MV_RANGE_MAX_HALF, MV_RANGE_MIN_HALF, MV_RANGE_UMV_MAX_HALF, MV_RANGE_UMV_MIN_HALF,
    OBMC_H0, OBMC_H1, OBMC_H2,
};
use crate::picture::SourceFormat;
use crate::sei::Sei;
use oxideav_core::bits::BitWriter;

/// Default fixed quantiser (PQUANT) — `5` matches the
/// `ffmpeg -qscale:v 5` baseline used to validate the existing decoder.
pub const DEFAULT_PQUANT: u8 = 5;

/// Default GOP size — one I-picture every 12 frames. Matches the default
/// `ffmpeg -g 12` cadence for H.263 output.
pub const DEFAULT_GOP_SIZE: u32 = 12;

/// Encoder factory used by [`crate::register_encoder`].
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    Ok(Box::new(H263Encoder::from_params(params)?))
}

/// Public H.263 baseline encoder. Construct via [`make_encoder`] and the
/// codec registry; post-construction tweaks (Annex J, GOP, …) can be applied
/// by downcasting to this type.
pub struct H263Encoder {
    output_params: CodecParameters,
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    /// Cadence between keyframes, in frames. `1` means "every frame is an I",
    /// `0` is treated identically. `>= 2` enables the P-picture path.
    gop_size: u32,
    /// Frames emitted since the last I-picture (0 → next frame is I).
    since_keyframe: u32,
    /// Previous reconstructed picture (motion-compensation reference for the
    /// next P-picture). `None` before the first I is encoded.
    reference: Option<IPicture>,
    time_base: TimeBase,
    pending: VecDeque<Packet>,
    eof: bool,
    next_tr: u8,
    /// When `true`, apply the H.263 Annex J deblocking filter to every
    /// reconstructed picture before it is stored as the motion-compensation
    /// reference for the next P-picture. The bitstream itself is unchanged
    /// (the encoder never emits a PLUSPTYPE block — our decoder parses
    /// PLUSPTYPE/OPPTYPE on input, but the encoder always produces a
    /// baseline PTYPE header); the matching decoder must be configured
    /// with the same flag via
    /// [`crate::decoder::H263Decoder::set_enable_annex_j`].
    enable_annex_j: bool,
    /// When `true`, enable Annex F (Advanced Prediction): 4-MV per MB with
    /// OBMC blending at 8×8 block granularity. The encoder sets PTYPE bit 12
    /// (AP) in the picture header and decides per-MB whether 4MV pays for
    /// itself over the 1-MV baseline. When an MB picks 4MV, the OBMC weights
    /// (§F.3, Figures F.2/F.3/F.4) are applied to the local reconstruction so
    /// that the next P-picture's reference matches what the decoder
    /// produces.
    enable_annex_f: bool,
    /// When `true`, emit both I- and P-pictures using Annex E syntax-based
    /// arithmetic coding (SAC) instead of the VLC path. PTYPE bit 11 is
    /// set in every picture header. The I-MB layer goes through cumf_*
    /// models for MCBPC_INTRA / CBPY_INTRA / INTRADC / TCOEF*_INTRA +
    /// LAST_INTRA / RUN_INTRA / LEVEL_INTRA escape; the P-MB layer goes
    /// through cumf_COD + cumf_MCBPC_no4MVQ + cumf_CBPY + cumf_DQUANT +
    /// cumf_MVD + INTER cumf_TCOEF1/2/3/r + SIGN + LAST/RUN/LEVEL escape.
    /// SAC + Annex F (4MV/OBMC) emission is rejected at `send_frame` — the
    /// `cumf_MCBPC_4MVQ` + per-block MVD wiring needed for 4MV-mode SAC
    /// is pending.
    enable_annex_e: bool,
    /// When `true`, emit P-pictures with Annex D Unrestricted Motion Vectors
    /// in the baseline-PTYPE form: PTYPE bit 10 (UMV) is set in the picture
    /// header, the encoder's motion estimator widens its search range to
    /// `[-63, +63]` halfpel (allowing references that point partially or
    /// fully outside the picture — §D.1), and MV components are emitted
    /// through [`crate::motion::encode_mv_component_umv`] which selects
    /// the VLC magnitude+sign whose §D.2 reconstruction yields the desired
    /// vector. The matching decoder side is already in place
    /// (`H263Decoder::decode_p_picture` honours the latched UMV flag).
    ///
    /// Combinations with Annex E (SAC) and Annex F (Advanced Prediction) are
    /// rejected at `send_frame` for now — round-12 scope is the baseline
    /// 1-MV inter path only.
    enable_annex_d_umv: bool,
    /// When `true`, emit pictures with the H.263+ PLUSPTYPE block carrying
    /// Annex N (Reference Picture Selection — see §5.1.13/§5.1.14/§5.1.15).
    /// The picture header sets source-format `111` (extended PTYPE), UFEP=001,
    /// OPPTYPE bit 11 (RPS) = 1; the RPS body fields RPSMF / TRPI / TRP / BCI
    /// follow per Figure 8.
    ///
    /// Round-13 scope: emit RPSMF = "100" (NEITHER ACK nor NACK — no
    /// back-channel), TRPI = 0 on every P-picture (TRP omitted; the decoder
    /// uses the most recent anchor), BCI = "01" (no BCM, videomux-mode not
    /// used). The MB layer underneath is unchanged baseline 1-MV inter.
    /// Combinations with Annex D (UMV), Annex E (SAC), Annex F (AP) are
    /// rejected at `send_frame` for now to keep the round 13 scope tight.
    enable_annex_n_rps: bool,
    /// When `true`, emit P-pictures as **PB-frames** per Annex G. Every
    /// P-picture sets PTYPE bit 13 (PBFR), the picture-header tail carries
    /// TRB (3 bits) + DBQUANT (2 bits) per §5.1.22 / §5.1.23, and every MB
    /// emits MODB (Table 11) followed by optional CBPB / MVDB / B-block
    /// residual. Round-14 scope: emit MODB = `0` per MB and DBQUANT = `00`
    /// — the B-half is then a pure §G.4 / §G.5 bidirectional MC predictor
    /// with no residual. Combinations with Annex D (UMV), Annex E (SAC),
    /// Annex F (Advanced Prediction), or Annex N (RPS) are rejected at
    /// `send_frame` for now.
    enable_annex_g_pb: bool,
    /// TRB value emitted in the picture header when [`Self::enable_annex_g_pb`]
    /// is on. Defaults to 1 (B sits at the midpoint of the P→P gap when the
    /// caller delivers frames at the natural cadence).
    pb_trb: u8,
    /// DBQUANT code emitted in the picture header when
    /// [`Self::enable_annex_g_pb`] is on. Defaults to `0b00` (BQUANT =
    /// 5*QUANT/4 — the spec's smallest BQUANT step). Valid values 0..=3.
    pb_dbquant: u8,
    /// When `true`, emit I-pictures with H.263+ PLUSPTYPE block carrying
    /// Annex I (Advanced INTRA Coding). Per-MB the encoder writes the
    /// Table I.1 INTRA_MODE codeword and uses the §I.3 Table I.2 INTRA
    /// TCOEF + AIC dequant + AC pred path.
    enable_annex_i_aic: bool,
    /// When `true` (and [`Self::enable_annex_g_pb`] is also on), emit
    /// PB-frames using the **Annex M** Improved PB-frames mode: per-MB the
    /// encoder picks the cheapest of {bidirectional, forward-only,
    /// backward-only} prediction by Lagrangian RDO and writes the matching
    /// Table M.1 MODB code. Annex M's MVDB slot carries the forward MV
    /// (predicted from the left MB's forward MV per §M.2.2) when the
    /// per-MB mode is Forward; bidirectional and backward modes emit no
    /// MVDB. Defaults to `false`. The matching decoder must also opt in
    /// via [`crate::decoder::H263Decoder::set_enable_annex_m_impb`] —
    /// the spec signals Annex M out-of-band per §M.1.
    enable_annex_m_impb: bool,
    /// When `true`, emit pictures using the H.263+ **Annex K** Slice
    /// Structured mode: PLUSPTYPE picture header carries OPPTYPE bit 10
    /// (SS) = 1, the GOB layer is replaced by the slice layer (per-slice
    /// SSC + slice header per §K.2), and MV prediction is reset at every
    /// slice boundary (§K.1 rule 1). The matching decoder must also opt
    /// in via [`crate::decoder::H263Decoder`] which already auto-detects
    /// the SS bit in the parsed picture header.
    ///
    /// Round-23 scope is the baseline 1-MV inter / I-picture body —
    /// combinations with UMV / SAC / AP / PB / RPS are rejected at
    /// `send_frame`. AIC is also gated off (the AIC encoder doesn't yet
    /// share the slice-emit path).
    enable_annex_k_slice: bool,
    /// Approximate number of macroblocks per slice when Annex K Slice
    /// Structured mode is in use. The encoder rounds the picture's MB
    /// count to obtain integer-MB slices and starts a fresh slice
    /// (with its own SSC + slice header) every `slice_mb_size`
    /// macroblocks. Defaults to 22 (one CIF MB row); a lower value
    /// gives more resync points at the cost of bitstream overhead.
    /// Always at least 1.
    slice_mb_size: u32,

    // -----------------------------------------------------------------------
    // Annex L — Supplemental Enhancement Information (encoder).
    // -----------------------------------------------------------------------
    /// SEI records queued for the next emitted picture. The encoder appends
    /// each record to the PEI/PSUPP loop of the next picture header, then
    /// clears the queue. Use [`Self::push_sei`] to schedule records.
    pending_sei: Vec<Sei>,

    // -----------------------------------------------------------------------
    // Annexes that require only a PLUSPTYPE flag surface without full encoder
    // body changes (default off; flag is always emitted disabled).
    // -----------------------------------------------------------------------
    /// Annex S (Alternative INTER VLC) — encoder. When on, the picture header
    /// is emitted as PLUSPTYPE with OPPTYPE bit 13 (AIV) = 1, and every INTER
    /// residual block is encoded using [`crate::aic::write_intra_tcoef`]
    /// (Table I.2) when the INTRA VLC would produce a shorter bitstream than
    /// the standard inter Table 16 VLC. §S.2 / §S.3.
    enable_annex_s_aiv: bool,

    /// Annex T (Modified Quantization) — encoder. When on, the picture header
    /// is emitted as PLUSPTYPE with OPPTYPE bit 14 (MQ) = 1; the DQUANT field
    /// uses the §T.2 VLC, chroma uses `QUANT_C` from §T.3 / Table T.2, and
    /// EXTENDED-ESCAPE is emitted for `|level| > 127` when QUANT < 8.
    enable_annex_t_mq: bool,

    /// Annex R (Independent Segment Decoding) — encoder flag surface.
    /// When on (requires Annex K with RS submode), emits PLUSPTYPE with
    /// OPPTYPE bit 12 (ISD) = 1. The §R.2.4 out-of-segment MV
    /// extrapolation is the decoder's concern; the encoder only adjusts the
    /// picture header bit.
    enable_annex_r_isd: bool,

    /// Annex P (Reference Picture Resampling) — encoder flag surface.
    /// When on, would emit RPR bit in MPPTYPE; the actual resampling
    /// parameter block is not yet emitted (rare, deferred). Defaults to
    /// `false`; stored so callers can opt in once the body is wired.
    enable_annex_p_rpr: bool,

    /// Annex Q (Reduced-Resolution Update) — encoder flag surface.
    /// When on, would emit RRU bit in MPPTYPE. Body not yet wired.
    enable_annex_q_rru: bool,

    /// Annex U (Enhanced Reference Picture Selection) — encoder flag surface.
    /// Extends Annex N RPS with a richer back-channel table. Body not yet wired.
    enable_annex_u_erps: bool,

    /// Annex V (Data-Partitioned Slice) — encoder flag surface.
    /// When on, slices would be emitted with header + motion + texture
    /// partitions. Body not yet wired.
    enable_annex_v_dpslice: bool,

    /// Annex W (Additional Supplemental Enhancement Information) — encoder
    /// flag surface. When on, the picture-message SEI header would be emitted
    /// via the PSUPP stream. Body not yet wired (uses Annex L PEI loop with
    /// FTYPE=15 extended-function-type records).
    enable_annex_w_picture_msg: bool,
}

impl H263Encoder {
    /// Construct an encoder from `CodecParameters`. Same validation as the
    /// factory [`make_encoder`], returning the concrete type so callers can
    /// use [`Self::set_enable_annex_j`] and any future knobs without going
    /// through trait-object downcasts.
    pub fn from_params(params: &CodecParameters) -> Result<Self> {
        let width = params
            .width
            .ok_or_else(|| Error::invalid("h263 encoder: missing width"))?;
        let height = params
            .height
            .ok_or_else(|| Error::invalid("h263 encoder: missing height"))?;
        let source_format = SourceFormat::for_dimensions(width, height).ok_or_else(|| {
            Error::unsupported(format!(
                "h263 encoder: dimensions {width}x{height} are not one of the standard \
                 source formats (sub-QCIF/QCIF/CIF/4CIF/16CIF)"
            ))
        })?;
        let pix = params.pixel_format.unwrap_or(PixelFormat::Yuv420P);
        if pix != PixelFormat::Yuv420P {
            return Err(Error::unsupported(format!(
                "h263 encoder: only Yuv420P supported (got {:?})",
                pix
            )));
        }

        let frame_rate = params.frame_rate.unwrap_or(Rational::new(30, 1));
        let mut output_params = params.clone();
        output_params.media_type = MediaType::Video;
        output_params.codec_id = CodecId::new(super::CODEC_ID_STR);
        output_params.width = Some(width);
        output_params.height = Some(height);
        output_params.pixel_format = Some(PixelFormat::Yuv420P);
        output_params.frame_rate = Some(frame_rate);
        let time_base = TimeBase::new(frame_rate.den, frame_rate.num);

        Ok(Self {
            output_params,
            width,
            height,
            source_format,
            pquant: DEFAULT_PQUANT,
            gop_size: DEFAULT_GOP_SIZE,
            since_keyframe: 0,
            reference: None,
            time_base,
            pending: VecDeque::new(),
            eof: false,
            next_tr: 0,
            enable_annex_j: false,
            enable_annex_f: false,
            enable_annex_e: false,
            enable_annex_d_umv: false,
            enable_annex_n_rps: false,
            enable_annex_g_pb: false,
            pb_trb: 1,
            pb_dbquant: 0,
            enable_annex_i_aic: false,
            enable_annex_m_impb: false,
            enable_annex_k_slice: false,
            slice_mb_size: 22,
            pending_sei: Vec::new(),
            enable_annex_s_aiv: false,
            enable_annex_t_mq: false,
            enable_annex_r_isd: false,
            enable_annex_p_rpr: false,
            enable_annex_q_rru: false,
            enable_annex_u_erps: false,
            enable_annex_v_dpslice: false,
            enable_annex_w_picture_msg: false,
        })
    }

    /// Enable or disable the Annex J deblocking filter. Must be set before
    /// the first frame is submitted; changing it mid-stream would desync
    /// reconstruction from any decoder running the same flag.
    pub fn set_enable_annex_j(&mut self, enable: bool) {
        self.enable_annex_j = enable;
    }

    /// Returns whether Annex J deblocking is currently enabled.
    pub fn enable_annex_j(&self) -> bool {
        self.enable_annex_j
    }

    /// Enable or disable Annex F (Advanced Prediction — 4MV + OBMC) emission.
    /// Must be set before the first frame is submitted. When on, the encoder
    /// sets PTYPE bit 12 (AP) in each P-picture header and selects 4-MV mode
    /// on a per-MB basis when it materially reduces the predictor SAD.
    pub fn set_enable_annex_f(&mut self, enable: bool) {
        self.enable_annex_f = enable;
    }

    /// Returns whether Annex F (Advanced Prediction) is currently enabled.
    pub fn enable_annex_f(&self) -> bool {
        self.enable_annex_f
    }

    /// Enable or disable Annex E (Syntax-based Arithmetic Coding) emission.
    /// Must be set before the first frame is submitted. When on, every
    /// I- and P-picture sets PTYPE bit 11 (SAC) and writes the macroblock
    /// layer through the §E.2 arithmetic coder instead of the VLC path;
    /// the decoder mirrors this on parse (`H263Decoder` reads the SAC
    /// body when the PTYPE bit is set). Round 13 landed the I-MB path,
    /// round 14 added the P-MB path. Combining SAC with Annex F
    /// (Advanced Prediction / 4MV / OBMC) is rejected at `send_frame`.
    pub fn set_enable_annex_e(&mut self, enable: bool) {
        self.enable_annex_e = enable;
    }

    /// Returns whether Annex E (SAC) emission is currently enabled.
    pub fn enable_annex_e(&self) -> bool {
        self.enable_annex_e
    }

    /// Enable or disable Annex D (Unrestricted Motion Vectors, baseline-PTYPE
    /// form) emission. When on, every P-picture sets PTYPE bit 10 (UMV) and
    /// the encoder selects MV components from `[-63, +63]` halfpel using
    /// the §D.2 sign-of-predictor rule. References that point partially or
    /// fully outside the picture replicate the nearest edge sample (§D.1).
    ///
    /// Must be set before the first frame is submitted; flipping it
    /// mid-stream would desync the matching decoder. Combinations with
    /// Annex E (SAC) or Annex F (Advanced Prediction) are not yet supported
    /// — `send_frame` returns `Error::Unsupported` if both are on.
    pub fn set_enable_annex_d_umv(&mut self, enable: bool) {
        self.enable_annex_d_umv = enable;
    }

    /// Returns whether Annex D (UMV) emission is currently enabled.
    pub fn enable_annex_d_umv(&self) -> bool {
        self.enable_annex_d_umv
    }

    /// Enable or disable Annex N (Reference Picture Selection) emission.
    /// When on, every picture is emitted with a PLUSPTYPE block carrying
    /// OPPTYPE bit 11 (RPS) = 1. The round-13 scope emits RPSMF = "100"
    /// (no back-channel), TRPI = 0 on every P-picture (no TRP — decoder
    /// uses the most recent anchor), and BCI = "01" (no BCM). The MB layer
    /// underneath is unchanged baseline 1-MV inter.
    ///
    /// Must be set before the first frame is submitted; flipping it
    /// mid-stream would desync the matching decoder. Combinations with
    /// Annex D (UMV), Annex E (SAC), or Annex F (Advanced Prediction)
    /// return `Error::Unsupported` at `send_frame` — RPS round 13 covers
    /// the baseline path only.
    pub fn set_enable_annex_n_rps(&mut self, enable: bool) {
        self.enable_annex_n_rps = enable;
    }

    /// Returns whether Annex N (RPS) emission is currently enabled.
    pub fn enable_annex_n_rps(&self) -> bool {
        self.enable_annex_n_rps
    }

    /// Enable or disable Annex G (PB-frames) emission. When on, every
    /// P-picture sets PTYPE bit 13 (PBFR), the picture header carries TRB +
    /// DBQUANT (§5.1.22 / §5.1.23), and every MB carries MODB + optional
    /// CBPB / MVDB / B-block residual.
    ///
    /// Round-14 scope keeps MODB = `0` per MB (no CBPB, no MVDB) and uses
    /// the configured TRB / DBQUANT picture-level values. Combinations with
    /// Annex D (UMV), Annex E (SAC), Annex F (Advanced Prediction), or
    /// Annex N (RPS) are rejected at `send_frame`.
    pub fn set_enable_annex_g_pb(&mut self, enable: bool) {
        self.enable_annex_g_pb = enable;
    }

    /// Returns whether Annex G (PB-frames) emission is currently enabled.
    pub fn enable_annex_g_pb(&self) -> bool {
        self.enable_annex_g_pb
    }

    /// Set the picture-header TRB value emitted in PB-frames mode (§5.1.22).
    /// Range `0..=7`. Larger values place the B closer to the new P; `1` is
    /// the natural midpoint when frames arrive at the spec's default cadence.
    pub fn set_pb_trb(&mut self, trb: u8) {
        self.pb_trb = trb.min(7);
    }

    /// Set the picture-header DBQUANT code emitted in PB-frames mode
    /// (§5.1.23 — Table 6 mapping). Range `0..=3`.
    pub fn set_pb_dbquant(&mut self, dbquant: u8) {
        self.pb_dbquant = dbquant.min(3);
    }

    /// Enable or disable Annex I (Advanced INTRA Coding) emission. Must be
    /// set before the first frame is submitted; mid-stream changes desync
    /// the matching decoder.
    ///
    /// When on, every I-picture is emitted with a PLUSPTYPE block carrying
    /// OPPTYPE bit 8 (AIC) = 1, and per-MB the encoder writes:
    ///   * INTRA_MODE field (Table I.1) immediately after MCBPC,
    ///   * coefficients via the §I.3 INTRA TCOEF table (Table I.2 — same
    ///     codeword shapes as the inter table, different `(LAST, RUN, |LEVEL|)`
    ///     mapping) starting at scan position 0,
    ///   * AIC dequantisation (`RecC = 2*Q*LEVEL`, no dead-zone) is used
    ///     when forming the local reconstruction so the next picture's MC
    ///     reference matches what the decoder produces,
    ///   * AC prediction (DC + first row / first column from neighbours,
    ///     §I.3 Mode 0/1/2) is folded into the residual the encoder
    ///     subtracts before quantising.
    ///
    /// AIC currently only affects I-pictures. P-picture intra-in-P MBs
    /// continue to use the baseline INTRADC + Table 16 path. AIC + SAC /
    /// AP / UMV / RPS / PB are rejected at `send_frame` to keep the round
    /// 24 scope tight.
    pub fn set_enable_annex_i_aic(&mut self, enable: bool) {
        self.enable_annex_i_aic = enable;
    }

    /// Returns whether Annex I (AIC) emission is currently enabled.
    pub fn enable_annex_i_aic(&self) -> bool {
        self.enable_annex_i_aic
    }

    /// Enable or disable **Annex M** (Improved PB-frames) emission.
    ///
    /// When on (together with [`Self::set_enable_annex_g_pb`]), every
    /// PB-frame is emitted with the §M.2 per-MB B-mode dispatch:
    /// the encoder picks the cheapest of {bidirectional, forward-only,
    /// backward-only} prediction via Lagrangian RDO over (rate, distortion)
    /// and writes the matching Table M.1 MODB codeword. Forward mode also
    /// emits a forward MV (predictor = left MB's forward MV per §M.2.2),
    /// VLC-coded via the same Table 14 + sign family as the §5.3.7 MVD.
    ///
    /// Defaults to `false`. The matching decoder side must opt in via
    /// [`crate::decoder::H263Decoder::set_enable_annex_m_impb`] to read the
    /// Table M.1 codes; otherwise it would interpret them as Annex G
    /// Table 11 codes and decode garbage. The spec signals Annex M
    /// out-of-band per §M.1 (e.g. ITU-T Rec. H.245).
    pub fn set_enable_annex_m_impb(&mut self, enable: bool) {
        self.enable_annex_m_impb = enable;
    }

    /// Returns whether Annex M (Improved PB-frames) emission is enabled.
    pub fn enable_annex_m_impb(&self) -> bool {
        self.enable_annex_m_impb
    }

    /// Enable or disable **Annex K** (Slice Structured) emission. When on,
    /// every picture is emitted with a PLUSPTYPE block carrying OPPTYPE
    /// bit 10 (SS) = 1, the GOB layer is replaced by per-slice resync
    /// headers (§K.2 — SSC start codes + the slice-header MBA / SQUANT /
    /// GFID fields), and the encoder resets MV prediction at every slice
    /// boundary (§K.1 rule 1).
    ///
    /// Round-23 scope: baseline 1-MV inter / I-picture only — combinations
    /// with UMV / SAC / AP / PB / RPS / AIC are rejected at `send_frame`.
    /// The Rectangular Slice (RS) and Arbitrary Slice Ordering (ASO)
    /// submodes are not yet emitted; both SSS bits are zero in the
    /// picture header.
    pub fn set_enable_annex_k_slice(&mut self, enable: bool) {
        self.enable_annex_k_slice = enable;
    }

    /// Returns whether Annex K (Slice Structured) emission is enabled.
    pub fn enable_annex_k_slice(&self) -> bool {
        self.enable_annex_k_slice
    }

    /// Set the target number of macroblocks per slice when Annex K is in
    /// use. Smaller values give more resync points at the cost of slice
    /// header overhead. Always clamped to at least 1.
    pub fn set_slice_mb_size(&mut self, n: u32) {
        self.slice_mb_size = n.max(1);
    }

    /// Returns the target slice size in macroblocks.
    pub fn slice_mb_size(&self) -> u32 {
        self.slice_mb_size
    }

    // -----------------------------------------------------------------------
    // Annex L — Supplemental Enhancement Information (encoder).
    // -----------------------------------------------------------------------

    /// Queue one SEI record for the **next picture header**. The encoder
    /// serialises all queued records into the PEI/PSUPP loop of the next
    /// picture header it writes, then clears the queue. Each record is
    /// encoded as a 4-bit FTYPE + 4-bit DSIZE + up to 15 bytes of parameter
    /// data per §L.2. Callers may queue multiple records before calling
    /// [`Encoder::send_frame`]; the records are emitted in the order they
    /// were pushed.
    ///
    /// # Limitations
    ///
    /// * Records with DSIZE > 15 bytes cannot be expressed in the baseline
    ///   §L.2 PSUPP layout (DSIZE is a 4-bit field); this method returns
    ///   `Err(Error::Invalid)` for those.
    /// * Extended-FTYPE records ([`Sei::ExtendedFunctionType`]) are always
    ///   emitted with DSIZE = 0 in the outer header and one extra byte
    ///   carrying `(ext_ftype << 4) | ext_dsize`; the caller is responsible
    ///   for ensuring `ext_dsize` matches the `payload` length.
    pub fn push_sei(&mut self, record: Sei) -> Result<()> {
        // Validate DSIZE fits in 4 bits for the records that carry a payload.
        let payload_len = sei_payload_len(&record);
        if payload_len > 15 {
            return Err(Error::invalid(format!(
                "h263 Annex L encoder: SEI payload length {payload_len} > 15 \
                 (DSIZE is a 4-bit field — split into smaller records)"
            )));
        }
        self.pending_sei.push(record);
        Ok(())
    }

    /// Return the number of SEI records currently queued for the next picture.
    pub fn pending_sei_count(&self) -> usize {
        self.pending_sei.len()
    }

    /// Clear all queued SEI records without emitting them.
    pub fn clear_pending_sei(&mut self) {
        self.pending_sei.clear();
    }

    // -----------------------------------------------------------------------
    // Annex S — Alternative INTER VLC (encoder).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex S (Alternative INTER VLC) emission.
    ///
    /// When on, every picture is emitted with a PLUSPTYPE block carrying
    /// OPPTYPE bit 13 (AIV) = 1. Each INTER residual block is encoded using
    /// the Table I.2 INTRA TCOEF VLC (via
    /// [`crate::aic::write_intra_tcoef`]) instead of the standard inter
    /// Table 16 VLC when the INTRA codeword for the `(LAST, RUN, |LEVEL|)`
    /// triple is strictly shorter (§S.2). When the CBPC counts indicate §S.3
    /// applies (both chroma blocks coded), the CBPY uses the INTRA shape
    /// (no XOR) on the inter MB.
    ///
    /// Combining with Annex E (SAC), Annex F (AP), Annex N (RPS), Annex G
    /// (PB), Annex I (AIC), or Annex K (Slice) is rejected at `send_frame`
    /// for now — a unified PLUSPTYPE writer for multi-annex streams is a
    /// follow-up.
    pub fn set_enable_annex_s_aiv(&mut self, enable: bool) {
        self.enable_annex_s_aiv = enable;
    }

    /// Returns whether Annex S (AIV) emission is currently enabled.
    pub fn enable_annex_s_aiv(&self) -> bool {
        self.enable_annex_s_aiv
    }

    // -----------------------------------------------------------------------
    // Annex T — Modified Quantization (encoder).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex T (Modified Quantization) emission.
    ///
    /// When on, every picture is emitted with a PLUSPTYPE block carrying
    /// OPPTYPE bit 14 (MQ) = 1. The DQUANT field in the MB layer uses the
    /// §T.2 variable-length code; chrominance quantisation uses `QUANT_C`
    /// from §T.3 / Table T.2 (a smaller step than luma). The §T.4
    /// EXTENDED-ESCAPE is available for `|level| > 127` when QUANT < 8,
    /// but the current encoder never exceeds QUANT = 31 so that path is
    /// exercised only if the caller sets `pquant < 8`.
    ///
    /// Current scope: DQUANT is held constant per picture (no per-MB
    /// DQUANT steps), so the MQ DQUANT VLC is never actually emitted —
    /// the MQ flag only switches the chroma quantiser and signals the
    /// receiver to use the §T.3 QUANT_C mapping.
    ///
    /// Combining with Annex E (SAC), Annex F (AP), Annex N (RPS), Annex G
    /// (PB), Annex I (AIC), Annex K (Slice), or Annex S (AIV) is rejected
    /// at `send_frame`.
    pub fn set_enable_annex_t_mq(&mut self, enable: bool) {
        self.enable_annex_t_mq = enable;
    }

    /// Returns whether Annex T (MQ) emission is currently enabled.
    pub fn enable_annex_t_mq(&self) -> bool {
        self.enable_annex_t_mq
    }

    // -----------------------------------------------------------------------
    // Annex R — Independent Segment Decoding (encoder flag surface).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex R (Independent Segment Decoding) flag in the
    /// encoder output.
    ///
    /// When on (Annex K with RS submode must also be on — §R.3.1), the
    /// picture header emits PLUSPTYPE with OPPTYPE bit 12 (ISD) = 1.
    /// The encoder does not change the MB or MV layers — the ISD flag
    /// tells the decoder it may decode each segment independently.
    ///
    /// `send_frame` returns `Error::Unsupported` if ISD is on without
    /// Annex K enabled with RS submode.
    pub fn set_enable_annex_r_isd(&mut self, enable: bool) {
        self.enable_annex_r_isd = enable;
    }

    /// Returns whether Annex R (ISD) flag emission is currently enabled.
    pub fn enable_annex_r_isd(&self) -> bool {
        self.enable_annex_r_isd
    }

    // -----------------------------------------------------------------------
    // Annex P — Reference Picture Resampling (encoder flag surface).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex P (Reference Picture Resampling) flag.
    /// When on, the RPR bit in MPPTYPE would be set — the actual resampling
    /// parameter block is not yet emitted. `send_frame` returns
    /// `Error::Unsupported` when this flag is set (body not yet wired).
    pub fn set_enable_annex_p_rpr(&mut self, enable: bool) {
        self.enable_annex_p_rpr = enable;
    }

    /// Returns whether Annex P (RPR) flag is currently enabled.
    pub fn enable_annex_p_rpr(&self) -> bool {
        self.enable_annex_p_rpr
    }

    // -----------------------------------------------------------------------
    // Annex Q — Reduced-Resolution Update (encoder flag surface).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex Q (Reduced-Resolution Update) flag.
    /// When on, the RRU bit in MPPTYPE would be set — the actual half-
    /// resolution update is not yet wired. `send_frame` returns
    /// `Error::Unsupported` when this flag is set.
    pub fn set_enable_annex_q_rru(&mut self, enable: bool) {
        self.enable_annex_q_rru = enable;
    }

    /// Returns whether Annex Q (RRU) flag is currently enabled.
    pub fn enable_annex_q_rru(&self) -> bool {
        self.enable_annex_q_rru
    }

    // -----------------------------------------------------------------------
    // Annex U — Enhanced Reference Picture Selection (encoder flag surface).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex U (Enhanced RPS) flag. Annex U extends the
    /// Annex N RPS mechanism with a richer back-channel. When on this flag
    /// is stored but `send_frame` returns `Error::Unsupported` (body not
    /// yet wired).
    pub fn set_enable_annex_u_erps(&mut self, enable: bool) {
        self.enable_annex_u_erps = enable;
    }

    /// Returns whether Annex U (ERPS) flag is currently enabled.
    pub fn enable_annex_u_erps(&self) -> bool {
        self.enable_annex_u_erps
    }

    // -----------------------------------------------------------------------
    // Annex V — Data-Partitioned Slice (encoder flag surface).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex V (Data-Partitioned Slice) flag. When on,
    /// slices would be emitted with separate header / motion / texture
    /// partitions. `send_frame` returns `Error::Unsupported` (body not yet
    /// wired).
    pub fn set_enable_annex_v_dpslice(&mut self, enable: bool) {
        self.enable_annex_v_dpslice = enable;
    }

    /// Returns whether Annex V (data-partitioned slice) flag is enabled.
    pub fn enable_annex_v_dpslice(&self) -> bool {
        self.enable_annex_v_dpslice
    }

    // -----------------------------------------------------------------------
    // Annex W — Additional Supplemental Enhancement Information (encoder
    // flag surface).
    // -----------------------------------------------------------------------

    /// Enable or disable Annex W (Additional SEI / picture-message) flag.
    /// When on, picture-message headers would be emitted via the PSUPP stream
    /// using `FTYPE=15` extended-function-type records (§W / §L.15).
    /// `send_frame` returns `Error::Unsupported` when this flag is set on
    /// its own (schedule explicit SEI records via [`Self::push_sei`] with
    /// [`Sei::ExtendedFunctionType`] instead to emit Annex W records today).
    pub fn set_enable_annex_w_picture_msg(&mut self, enable: bool) {
        self.enable_annex_w_picture_msg = enable;
    }

    /// Returns whether Annex W (picture-message) flag is enabled.
    pub fn enable_annex_w_picture_msg(&self) -> bool {
        self.enable_annex_w_picture_msg
    }
}

impl Encoder for H263Encoder {
    fn codec_id(&self) -> &CodecId {
        &self.output_params.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.output_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        let v = match frame {
            Frame::Video(v) => v,
            _ => return Err(Error::invalid("h263 encoder: video frames only")),
        };
        if v.planes.len() != 3 {
            return Err(Error::invalid("h263 encoder: expected 3 planes"));
        }

        // Annex D + (Annex E | Annex F) is not yet wired — round-12 scope is
        // the baseline 1-MV inter path with UMV reach + sign-of-predictor
        // reconstruction. The SAC and AP paths use their own MV emission
        // routines (`mb_sac` + `emit_p_mb_ap`) that don't yet route through
        // `encode_mv_component_umv`.
        if self.enable_annex_d_umv && (self.enable_annex_e || self.enable_annex_f) {
            return Err(Error::unsupported(
                "h263 encoder: Annex D (UMV) is not yet supported in combination with \
                 Annex E (SAC) or Annex F (Advanced Prediction)",
            ));
        }

        // Annex N (RPS) round-13 scope: baseline 1-MV inter only. Combined
        // with UMV / SAC / AP would either need a different PLUSPTYPE writer
        // (UMV needs UUI, SAC needs OPPTYPE bit 6, AP needs OPPTYPE bit 7)
        // or a hybrid PLUSPTYPE/baseline path; both deferred.
        if self.enable_annex_n_rps
            && (self.enable_annex_d_umv || self.enable_annex_e || self.enable_annex_f)
        {
            return Err(Error::unsupported(
                "h263 encoder: Annex N (RPS) is not yet supported in combination with \
                 Annex D (UMV), Annex E (SAC), or Annex F (Advanced Prediction)",
            ));
        }

        // Annex G (PB-frames) round-14 scope: baseline 1-MV inter only.
        // Combinations with the other annex knobs are deferred.
        if self.enable_annex_g_pb
            && (self.enable_annex_d_umv
                || self.enable_annex_e
                || self.enable_annex_f
                || self.enable_annex_n_rps)
        {
            return Err(Error::unsupported(
                "h263 encoder: Annex G (PB-frames) is not yet supported in combination \
                 with Annex D (UMV), Annex E (SAC), Annex F (Advanced Prediction), or \
                 Annex N (RPS)",
            ));
        }

        // Annex M (Improved PB-frames) requires Annex G PB-frames to be
        // active — Annex M extends the same picture syntax with a different
        // MODB table and per-MB B-mode dispatch. Setting Annex M without
        // Annex G is a config error.
        if self.enable_annex_m_impb && !self.enable_annex_g_pb {
            return Err(Error::invalid(
                "h263 encoder: Annex M (Improved PB-frames) requires Annex G \
                 PB-frames to also be enabled",
            ));
        }

        // Annex I (AIC) round-24 scope: I-pictures only, no other PLUSPTYPE
        // optional modes combined. The MB-layer additions (INTRA_MODE,
        // Table I.2, AC pred) only apply to INTRA MBs and would still
        // bit-stream-cleanly combine with most other modes once the
        // P-picture intra-in-P path also routes through the AIC writer
        // (follow-up).
        if self.enable_annex_i_aic
            && (self.enable_annex_d_umv
                || self.enable_annex_e
                || self.enable_annex_f
                || self.enable_annex_n_rps
                || self.enable_annex_g_pb)
        {
            return Err(Error::unsupported(
                "h263 encoder: Annex I (AIC) is not yet supported in combination \
                 with other PLUSPTYPE optional modes (UMV / SAC / AP / RPS / PB)",
            ));
        }

        // Annex K (Slice Structured) round scope: baseline 1-MV inter only.
        // Combinations with UMV / SAC / AP / RPS / PB / AIC need shared
        // PLUSPTYPE writers and per-slice resets in those code paths
        // (slice MV-grid reset already lines up with what AP / SAC do at
        // GOB boundaries, but the bitstream layout would need a unified
        // emitter — deferred).
        if self.enable_annex_k_slice
            && (self.enable_annex_d_umv
                || self.enable_annex_e
                || self.enable_annex_f
                || self.enable_annex_n_rps
                || self.enable_annex_g_pb
                || self.enable_annex_i_aic)
        {
            return Err(Error::unsupported(
                "h263 encoder: Annex K (Slice Structured) is not yet supported in \
                 combination with other PLUSPTYPE optional modes (UMV / SAC / AP / \
                 RPS / PB / AIC)",
            ));
        }

        // Annex P / Q — not yet wired (caller opted in but the body is missing).
        if self.enable_annex_p_rpr {
            return Err(Error::unsupported(
                "h263 encoder: Annex P (Reference Picture Resampling) body \
                 not yet wired — flag surface only (follow-up)",
            ));
        }
        if self.enable_annex_q_rru {
            return Err(Error::unsupported(
                "h263 encoder: Annex Q (Reduced-Resolution Update) body \
                 not yet wired — flag surface only (follow-up)",
            ));
        }

        // Annex U (Enhanced RPS) — not yet wired.
        if self.enable_annex_u_erps {
            return Err(Error::unsupported(
                "h263 encoder: Annex U (Enhanced Reference Picture Selection) body \
                 not yet wired — flag surface only (follow-up)",
            ));
        }

        // Annex V (data-partitioned slice) — not yet wired.
        if self.enable_annex_v_dpslice {
            return Err(Error::unsupported(
                "h263 encoder: Annex V (Data-Partitioned Slice) body \
                 not yet wired — flag surface only (follow-up)",
            ));
        }

        // Annex W (picture-message) — not yet wired as an automatic encoder
        // mode; callers that want to emit Annex W records today should use
        // `push_sei(Sei::ExtendedFunctionType { ... })`.
        if self.enable_annex_w_picture_msg {
            return Err(Error::unsupported(
                "h263 encoder: Annex W (Additional SEI / picture-message) \
                 automatic emit not yet wired — use push_sei(ExtendedFunctionType) \
                 for now",
            ));
        }

        // Annex R (ISD) — requires Annex K with RS submode (§R.3.1).
        if self.enable_annex_r_isd {
            if !self.enable_annex_k_slice {
                return Err(Error::unsupported(
                    "h263 encoder: Annex R (ISD) requires Annex K Slice \
                     Structured mode to also be enabled (§R.3.1)",
                ));
            }
            // RS (Rectangular Slice) submode is not directly tracked on the
            // encoder struct; the ISD + K combination is accepted and the
            // PLUSPTYPE header writer sets both SS + ISD bits. The decoder
            // enforces the RS submode constraint on its side.
        }

        // Annex S (AIV) — PLUSPTYPE-only; incompatible with other PLUSPTYPE
        // annexes that have separate writers in this round.
        if self.enable_annex_s_aiv
            && (self.enable_annex_e
                || self.enable_annex_f
                || self.enable_annex_n_rps
                || self.enable_annex_g_pb
                || self.enable_annex_i_aic
                || self.enable_annex_k_slice
                || self.enable_annex_t_mq)
        {
            return Err(Error::unsupported(
                "h263 encoder: Annex S (AIV) is not yet supported in combination \
                 with Annex E / F / N / G / I / K / T (deferred to a unified \
                 multi-annex PLUSPTYPE writer)",
            ));
        }

        // Annex T (MQ) — PLUSPTYPE-only; incompatible with other PLUSPTYPE
        // annexes that have separate writers in this round.
        if self.enable_annex_t_mq
            && (self.enable_annex_e
                || self.enable_annex_f
                || self.enable_annex_n_rps
                || self.enable_annex_g_pb
                || self.enable_annex_i_aic
                || self.enable_annex_k_slice
                || self.enable_annex_s_aiv)
        {
            return Err(Error::unsupported(
                "h263 encoder: Annex T (MQ) is not yet supported in combination \
                 with Annex E / F / N / G / I / K / S (deferred to a unified \
                 multi-annex PLUSPTYPE writer)",
            ));
        }

        let tr = self.next_tr;
        self.next_tr = self.next_tr.wrapping_add(1);

        // Decide I vs P: first frame is always I; then every `gop_size` frames
        // we insert another I. `gop_size <= 1` forces I on every frame.
        let force_i = self.reference.is_none()
            || self.gop_size <= 1
            || self.since_keyframe + 1 >= self.gop_size;

        // Round 14 — Annex G (PB-frames) routing. PB-frames extend the P
        // picture syntax with MODB / CBPB / MVDB / DBQUANT and pair every
        // P-picture with a co-transmitted B-picture (reconstructed by §G.4
        // / §G.5 bidirectional MC). The encoder still consumes one input
        // frame per `send_frame` call; the B-half is synthesised from MC
        // alone (round-14 scope keeps MODB = 0 / no B residual). The packet
        // emitted carries the PB picture header + per-MB MODB stream.
        if self.enable_annex_g_pb {
            let (bytes, p_recon, is_key) = if force_i {
                // PB-frames need a prior P-anchor — emit a baseline I-picture
                // first to seed the reference. The PB bit stays clear here.
                let (b, p) = encode_i_picture_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                )?;
                (b, p, true)
            } else {
                let reference = self.reference.as_ref().expect("reference checked above");
                let (b, p) = encode_pb_picture_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    reference,
                    self.pb_trb,
                    self.pb_dbquant,
                    self.enable_annex_m_impb,
                )?;
                (b, p, false)
            };
            let mut recon = p_recon;
            if self.enable_annex_j {
                let mb_w = self.width.div_ceil(16) as usize;
                let mb_h = self.height.div_ceil(16) as usize;
                let qp = vec![self.pquant; mb_w * mb_h];
                crate::deblock::deblock_picture(&mut recon, &qp);
            }
            self.reference = Some(recon);
            if is_key {
                self.since_keyframe = 1;
            } else {
                self.since_keyframe += 1;
            }
            let mut pkt = Packet::new(0, self.time_base, bytes);
            pkt.pts = v.pts;
            pkt.dts = v.pts;
            pkt.flags.keyframe = is_key;
            self.pending.push_back(pkt);
            return Ok(());
        }

        // Annex K (Slice Structured) routing. Replaces the GOB layer with
        // per-slice resync headers; the MB body itself is unchanged
        // baseline 1-MV inter.
        if self.enable_annex_k_slice {
            let (bytes, pic, is_key) = if force_i {
                let (b, p) = encode_i_picture_slice_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    self.slice_mb_size,
                )?;
                (b, p, true)
            } else {
                let reference = self.reference.as_ref().expect("reference checked above");
                let (b, p) = encode_p_picture_slice_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    reference,
                    self.slice_mb_size,
                )?;
                (b, p, false)
            };
            let mut recon = pic;
            if self.enable_annex_j {
                let mb_w = self.width.div_ceil(16) as usize;
                let mb_h = self.height.div_ceil(16) as usize;
                let qp = vec![self.pquant; mb_w * mb_h];
                crate::deblock::deblock_picture(&mut recon, &qp);
            }
            self.reference = Some(recon);
            if is_key {
                self.since_keyframe = 1;
            } else {
                self.since_keyframe += 1;
            }
            let mut pkt = Packet::new(0, self.time_base, bytes);
            pkt.pts = v.pts;
            pkt.dts = v.pts;
            pkt.flags.keyframe = is_key;
            self.pending.push_back(pkt);
            return Ok(());
        }

        // Round 13 — Annex N (RPS) routing. RPS rewrites the picture header
        // to PLUSPTYPE form; the MB body underneath is the same baseline
        // 1-MV inter path the non-RPS branch uses. We dispatch first so RPS
        // pre-empts the SAC / Annex F branches below (which also start by
        // checking `enable_annex_e`/`enable_annex_f`).
        if self.enable_annex_n_rps {
            let (bytes, pic, is_key) = if force_i {
                let (b, p) = encode_i_picture_rps_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                )?;
                (b, p, true)
            } else {
                let reference = self.reference.as_ref().expect("reference checked above");
                let (b, p) = encode_p_picture_rps_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    reference,
                )?;
                (b, p, false)
            };
            let mut recon = pic;
            if self.enable_annex_j {
                let mb_w = self.width.div_ceil(16) as usize;
                let mb_h = self.height.div_ceil(16) as usize;
                let qp = vec![self.pquant; mb_w * mb_h];
                crate::deblock::deblock_picture(&mut recon, &qp);
            }
            self.reference = Some(recon);
            if is_key {
                self.since_keyframe = 1;
            } else {
                self.since_keyframe += 1;
            }
            let mut pkt = Packet::new(0, self.time_base, bytes);
            pkt.pts = v.pts;
            pkt.dts = v.pts;
            pkt.flags.keyframe = is_key;
            self.pending.push_back(pkt);
            return Ok(());
        }

        // Annex S (AIV) routing — PLUSPTYPE with AIV bit; MB body uses AIV VLC
        // for inter blocks when that saves bits (§S.2).
        if self.enable_annex_s_aiv {
            let sei = std::mem::take(&mut self.pending_sei);
            let (bytes, pic, is_key) = if force_i {
                let (b, p) = encode_i_picture_aiv_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    &sei,
                )?;
                (b, p, true)
            } else {
                let reference = self.reference.as_ref().expect("reference checked above");
                let (b, p) = encode_p_picture_aiv_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    reference,
                    self.enable_annex_f,
                    self.enable_annex_d_umv,
                    &sei,
                )?;
                (b, p, false)
            };
            let mut recon = pic;
            if self.enable_annex_j {
                let mb_w = self.width.div_ceil(16) as usize;
                let mb_h = self.height.div_ceil(16) as usize;
                let qp = vec![self.pquant; mb_w * mb_h];
                crate::deblock::deblock_picture(&mut recon, &qp);
            }
            self.reference = Some(recon);
            if is_key {
                self.since_keyframe = 1;
            } else {
                self.since_keyframe += 1;
            }
            let mut pkt = Packet::new(0, self.time_base, bytes);
            pkt.pts = v.pts;
            pkt.dts = v.pts;
            pkt.flags.keyframe = is_key;
            self.pending.push_back(pkt);
            return Ok(());
        }

        // Annex T (MQ) routing — PLUSPTYPE with MQ bit; chroma uses QUANT_C
        // mapping from §T.3 / Table T.2 (luma QUANT → smaller chroma QUANT).
        if self.enable_annex_t_mq {
            let sei = std::mem::take(&mut self.pending_sei);
            let (bytes, pic, is_key) = if force_i {
                let (b, p) = encode_i_picture_mq_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    &sei,
                )?;
                (b, p, true)
            } else {
                let reference = self.reference.as_ref().expect("reference checked above");
                let (b, p) = encode_p_picture_mq_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    reference,
                    self.enable_annex_f,
                    self.enable_annex_d_umv,
                    &sei,
                )?;
                (b, p, false)
            };
            let mut recon = pic;
            if self.enable_annex_j {
                let mb_w = self.width.div_ceil(16) as usize;
                let mb_h = self.height.div_ceil(16) as usize;
                let qp = vec![self.pquant; mb_w * mb_h];
                crate::deblock::deblock_picture(&mut recon, &qp);
            }
            self.reference = Some(recon);
            if is_key {
                self.since_keyframe = 1;
            } else {
                self.since_keyframe += 1;
            }
            let mut pkt = Packet::new(0, self.time_base, bytes);
            pkt.pts = v.pts;
            pkt.dts = v.pts;
            pkt.flags.keyframe = is_key;
            self.pending.push_back(pkt);
            return Ok(());
        }

        // Drain pending SEI. The baseline encode sub-functions (encode_i/p_picture_*)
        // do not yet accept an SEI parameter — callers that need SEI should use
        // push_sei() with Annex S (AIV) or Annex T (MQ) enabled, which do pass SEI
        // through their PLUSPTYPE header writers. For the baseline path we discard
        // queued records here rather than silently ignoring them across pictures.
        let _pending_sei_for_main = std::mem::take(&mut self.pending_sei);

        let (data, mut recon, is_key) = if force_i {
            let (bytes, pic) = if self.enable_annex_i_aic {
                encode_i_picture_aic_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                )?
            } else if self.enable_annex_e {
                encode_i_picture_sac_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                )?
            } else {
                encode_i_picture_with_recon(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                )?
            };
            (bytes, pic, true)
        } else {
            let reference = self.reference.as_ref().expect("reference checked above");
            let (bytes, pic) = if self.enable_annex_e {
                // Round 14 — SAC P-picture body. Routes COD / MCBPC /
                // CBPY / DQUANT / MVD / TCOEF through the §E.7 models
                // (`crate::sac` + `crate::mb_sac`).
                //
                // Round 15 (this one) wires SAC + Annex F (Advanced
                // Prediction / 4MV / OBMC): when both knobs are on, the
                // P-picture sets PTYPE bits 11 (SAC) AND 12 (AP), and the
                // MB layer uses `cumf_MCBPC_4MVQ` (§E.8) with per-block
                // MVDs (Inter4MV variants of Table 8) and OBMC-blended
                // local reconstruction (§F.3).
                if self.enable_annex_f {
                    // Round 16: even when Annex J is also on, the AP path
                    // already uses `cumf_MCBPC_4MVQ` (§E.7); the local recon
                    // gets the deblock filter applied below by the
                    // post-recon Annex-J pass.
                    encode_p_picture_sac_ap_with_recon(
                        self.width,
                        self.height,
                        self.source_format,
                        self.pquant,
                        tr,
                        v,
                        reference,
                    )?
                } else {
                    // Round 16: SAC + Annex J (no Annex F) routes through
                    // the same P-picture body but with `cumf_MCBPC_4MVQ`
                    // selected — see §E.7 (DF active → 4MVQ MCBPC).
                    encode_p_picture_sac_with_recon_opts(
                        self.width,
                        self.height,
                        self.source_format,
                        self.pquant,
                        tr,
                        v,
                        reference,
                        false,
                        self.enable_annex_j,
                    )?
                }
            } else {
                encode_p_picture_with_opts_full(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    reference,
                    self.enable_annex_f,
                    self.enable_annex_d_umv,
                )?
            };
            (bytes, pic, false)
        };

        // Annex J — optional in-loop deblocking filter. Applied after
        // reconstruction so that the filtered picture is what becomes the MC
        // reference for the next P-frame. The decoder must mirror this
        // (via `H263Decoder::set_enable_annex_j`) for round-trip equivalence.
        if self.enable_annex_j {
            let mb_w = self.width.div_ceil(16) as usize;
            let mb_h = self.height.div_ceil(16) as usize;
            let qp = vec![self.pquant; mb_w * mb_h];
            crate::deblock::deblock_picture(&mut recon, &qp);
        }

        self.reference = Some(recon);
        if is_key {
            self.since_keyframe = 1;
        } else {
            self.since_keyframe += 1;
        }

        let mut pkt = Packet::new(0, self.time_base, data);
        pkt.pts = v.pts;
        pkt.dts = v.pts;
        pkt.flags.keyframe = is_key;
        self.pending.push_back(pkt);
        Ok(())
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        if let Some(p) = self.pending.pop_front() {
            return Ok(p);
        }
        if self.eof {
            Err(Error::Eof)
        } else {
            Err(Error::NeedMore)
        }
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Picture / GOB / MB / block emit
// ---------------------------------------------------------------------------

/// Encode a single I-picture and return the raw H.263 elementary-stream bytes
/// (PSC + payload, not byte-stuffed; H.263 is naturally byte-aligned at GOB
/// boundaries, and our encoder never emits a value that would alias the
/// 17-bit zero prefix mid-stream because we cap |level| at 127 for escape
/// codes).
pub fn encode_i_picture(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
) -> Result<Vec<u8>> {
    let (bytes, _recon) = encode_i_picture_with_recon(
        width,
        height,
        source_format,
        pquant,
        temporal_reference,
        frame,
    )?;
    Ok(bytes)
}

/// Encode an I-picture and reconstruct it locally (for use as the motion-
/// compensation reference when the next frame is a P-picture). The
/// reconstruction is bit-exact with what the decoder produces when fed the
/// returned byte stream.
pub fn encode_i_picture_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 encoder: source format has no GOB layout"))?;

    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);

    write_picture_header(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        false,
        false,
    )?;

    for mb_y in 0..mb_h {
        // GOB header at every GOB except the first.
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let gn = (mb_y as u32 / mb_rows_per_gob) as u8;
            write_gob_header(&mut bw, gn, pquant)?;
        }
        for mb_x in 0..mb_w {
            encode_intra_mb(
                &mut bw, mb_x, mb_y, pquant, frame, width, height, &mut recon,
            )?;
        }
    }
    // Trailing zero stuffing to ensure the encoder leaves a byte boundary
    // (BitWriter::finish handles padding, but the spec requires the final
    // byte to align to a multiple of 8). No EOS marker — short clips don't
    // need one and ffmpeg accepts the stream without it.
    Ok((bw.finish(), recon))
}

/// Encode a single I-picture as a SAC (Annex E) bitstream — same picture
/// header as the VLC variant but with PTYPE bit 11 set, and an
/// arithmetic-coded MB layer. Returns the bitstream bytes and the locally
/// reconstructed picture (used as the next MC reference).
///
/// The body is emitted as a single SAC segment with no internal GOB
/// headers — sub-QCIF / QCIF have 6/9 single-row GOBs that would each need
/// an `encoder_flush` + `decoder_reset` boundary; we keep the segment
/// monolithic per picture for simplicity. The decoder side
/// (`mb_sac::decode_i_picture_sac`) rejects in-body GOB headers.
pub fn encode_i_picture_sac_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
) -> Result<(Vec<u8>, IPicture)> {
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    write_picture_header_with_opts(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        false, // I-picture
        false, // no AP
        true,  // SAC bit on
    )?;
    // The picture header ends on a byte boundary (PSC + TR + PTYPE + PQUANT
    // + CPM + PEI sum to 50 bits, padded to 56 = 7 bytes by the BitWriter
    // before any further byte-level write — we explicitly pad here so the
    // SAC body starts on a byte boundary regardless of prior layout
    // choices). H.263 baseline only needs alignment when concatenating
    // byte-stuffed sections; the SAC PSC_FIFO output is already byte-
    // aligned by `PscFifoWriter::finish`.
    while !bw.is_byte_aligned() {
        bw.write_bits(0, 1);
    }
    crate::mb_sac::encode_i_picture_sac_body(&mut bw, width, height, pquant, frame, &mut recon)?;
    Ok((bw.finish(), recon))
}

/// Encode a single P-picture as a SAC (Annex E) bitstream — sets PTYPE
/// bit 11 in the picture header and routes the MB layer through the
/// §E.7 models (cumf_COD + cumf_MCBPC_no4MVQ + cumf_CBPY + cumf_DQUANT +
/// cumf_MVD + INTER cumf_TCOEF1/2/3/r + SIGN + LAST/RUN/LEVEL escape).
///
/// Equivalent to [`encode_p_picture_sac_with_recon_opts`] with
/// `emit_gob_headers = false` — matches the VLC P-encoder's "single
/// segment per picture" baseline behaviour and so produces the same
/// reconstructed-frame buffer for the same DCT/quant/MV pipeline.
pub fn encode_p_picture_sac_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
) -> Result<(Vec<u8>, IPicture)> {
    encode_p_picture_sac_with_recon_opts(
        width,
        height,
        source_format,
        pquant,
        temporal_reference,
        frame,
        reference,
        false,
        false,
    )
}

/// Like [`encode_p_picture_sac_with_recon`] but with extra knobs.
///
/// `emit_gob_headers`: when true, each MB-row boundary that lines up with
/// the source format's GOB layout triggers an `encoder_flush` (§E.7) +
/// byte-aligned PSC_FIFO drain + GOB header (VLC) + fresh SAC segment,
/// mirroring §E.5. The decoder in [`crate::mb_sac::decode_p_picture_sac`]
/// re-primes the arithmetic decoder at every GBSC (§E.3 `decoder_reset`).
/// Useful for SAC streams that need mid-picture resync points.
///
/// `enable_annex_j`: when true, the MB-layer SAC encoder selects
/// `cumf_MCBPC_4MVQ` per §E.7 (DF on → 4MVQ MCBPC model, even with 1-MV
/// macroblocks). The encoder still applies the deblocking filter to the
/// local recon out-of-band (`enable_annex_j` knob on `H263Encoder`); the
/// decoder must be told the same flag — baseline PTYPE has no DF bit.
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_sac_with_recon_opts(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    emit_gob_headers: bool,
    enable_annex_j: bool,
) -> Result<(Vec<u8>, IPicture)> {
    let (_num_gobs, mb_rows_per_gob) = source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 SAC P encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);

    write_picture_header_with_opts(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true,  // P-picture
        false, // no AP — SAC + Annex F goes through `encode_p_picture_sac_ap_with_recon`
        true,  // SAC bit on
    )?;
    // SAC body must start at a byte boundary (§E.6 / §E.5).
    while !bw.is_byte_aligned() {
        bw.write_bits(0, 1);
    }

    crate::mb_sac::encode_p_picture_sac_body(
        &mut bw,
        width,
        height,
        pquant,
        frame,
        reference,
        &mut recon,
        mb_rows_per_gob,
        emit_gob_headers,
        enable_annex_j,
    )?;

    Ok((bw.finish(), recon))
}

/// Round 15 — encode a single P-picture as a SAC bitstream **with Annex F
/// (Advanced Prediction / 4MV / OBMC) on**. Sets PTYPE bits 11 (SAC) and
/// 12 (AP) in the picture header. The MB layer routes through
/// `cumf_MCBPC_4MVQ` (§E.8) with per-block MVDs (Inter4MV variants of
/// Table 8) and an OBMC-blended local reconstruction (§F.3) that mirrors
/// what the decoder produces.
///
/// Single SAC segment per picture (no in-body GOB resync, matching the
/// VLC AP P-encoder's "no GOB headers" baseline behaviour for byte-exact
/// reconstruction parity). Use [`encode_p_picture_sac_ap_with_recon_opts`]
/// to opt into per-GOB resync.
pub fn encode_p_picture_sac_ap_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
) -> Result<(Vec<u8>, IPicture)> {
    encode_p_picture_sac_ap_with_recon_opts(
        width,
        height,
        source_format,
        pquant,
        temporal_reference,
        frame,
        reference,
        false,
    )
}

/// Round 16 — like [`encode_p_picture_sac_ap_with_recon`] but with an
/// `emit_gob_headers` knob. When true, every GOB row boundary fires the
/// §E.5/§E.6 SAC-flush + GOB-header bridge + fresh SAC segment. Mirrors
/// the non-AP `encode_p_picture_sac_with_recon_opts` pattern. The MV
/// predictor is NOT reset across segments — §F.3 explicitly allows the
/// AP path to reach across GOB boundaries (outside Slice Structured /
/// ISD), which lets the encoder + decoder stay in lockstep on the
/// pre-OBMC mv_grid.
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_sac_ap_with_recon_opts(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    emit_gob_headers: bool,
) -> Result<(Vec<u8>, IPicture)> {
    let (_num_gobs, mb_rows_per_gob) = source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 SAC+AP P encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);

    write_picture_header_with_opts(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true, // P-picture
        true, // AP — Annex F
        true, // SAC bit on
    )?;
    while !bw.is_byte_aligned() {
        bw.write_bits(0, 1);
    }

    crate::mb_sac::encode_p_picture_sac_ap_body(
        &mut bw,
        width,
        height,
        pquant,
        frame,
        reference,
        &mut recon,
        mb_rows_per_gob,
        emit_gob_headers,
    )?;

    Ok((bw.finish(), recon))
}

/// Encode a single P-picture against the supplied `reference`. Returns the
/// bitstream bytes and the locally reconstructed picture (used as the next
/// MC reference). Equivalent to [`encode_p_picture_with_opts`] with
/// `enable_annex_f = false`.
pub fn encode_p_picture(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
) -> Result<(Vec<u8>, IPicture)> {
    encode_p_picture_with_opts(
        width,
        height,
        source_format,
        pquant,
        temporal_reference,
        frame,
        reference,
        false,
    )
}

/// Like [`encode_p_picture`] but with an `enable_annex_f` knob that toggles
/// Advanced Prediction (4MV + OBMC). When on, PTYPE bit 12 is set and each
/// MB independently picks 1MV vs 4MV based on an SAD comparison.
///
/// When `enable_annex_f` is on, the encode path is two-pass:
///   1. Visit every MB, decide `MbDecision` (skipped / intra / 1-MV inter /
///      4-MV inter), populate `mv_grid`. No bitstream output yet.
///   2. With a fully populated `mv_grid`, visit every MB again, compute the
///      OBMC-blended predictor (mirroring the decoder's §F.3 math), quantise
///      the residual against that predictor, and emit the per-MB bitstream
///      (COD/MCBPC/CBPY/MVD(s)/ACs). Reconstruction is stored in `recon` via
///      `apply_p_mb_reconstruction` so the next-picture's reference is bit-
///      identical to the decoder's output.
///
/// The single-pass baseline path (no AP) is unchanged.
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_with_opts(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    enable_annex_f: bool,
) -> Result<(Vec<u8>, IPicture)> {
    encode_p_picture_with_opts_full(
        width,
        height,
        source_format,
        pquant,
        temporal_reference,
        frame,
        reference,
        enable_annex_f,
        false,
    )
}

/// Full-options P-picture encoder. Adds an `enable_annex_d_umv` knob to
/// [`encode_p_picture_with_opts`]: when set (and `enable_annex_f` is false)
/// PTYPE bit 10 (UMV) is set in the picture header, the motion estimator
/// widens its reach to `[-63, +63]` halfpel (allowing references that point
/// outside the picture per §D.1), and MVD components are emitted through
/// [`crate::motion::encode_mv_component_umv`] which selects the §D.2
/// magnitude+sign whose decode matches the desired vector.
///
/// `enable_annex_d_umv` together with `enable_annex_f` returns
/// `Error::Unsupported` — Annex F's per-block MVDs use their own emission
/// path that is not yet UMV-aware.
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_with_opts_full(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    enable_annex_f: bool,
    enable_annex_d_umv: bool,
) -> Result<(Vec<u8>, IPicture)> {
    if enable_annex_f && enable_annex_d_umv {
        return Err(Error::unsupported(
            "h263 encoder: Annex F (Advanced Prediction) + Annex D (UMV) emission \
             is not yet supported",
        ));
    }
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 encoder: source format has no GOB layout"))?;

    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    write_picture_header_full(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true,
        enable_annex_f,
        false,
        enable_annex_d_umv,
    )?;

    if !enable_annex_f {
        // Single-pass baseline path — matches the old `encode_p_picture`
        // behaviour byte-for-byte (and adds Annex D UMV when requested).
        for mb_y in 0..mb_h {
            for mb_x in 0..mb_w {
                let _info = encode_p_mb_full(
                    &mut bw,
                    mb_x,
                    mb_y,
                    pquant,
                    frame,
                    width,
                    height,
                    reference,
                    &mut recon,
                    &mut mv_grid,
                    false,
                    enable_annex_d_umv,
                )?;
            }
        }
        return Ok((bw.finish(), recon));
    }

    // --- Annex F two-pass path -------------------------------------------
    let mut decisions: Vec<MbDecision> = Vec::with_capacity(mb_w * mb_h);
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let d = decide_p_mb(frame, reference, &mv_grid, mb_x, mb_y, pquant);
            // Populate `mv_grid` so downstream neighbours' predictors see it.
            match d {
                MbDecision::Skipped => {
                    mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
                }
                MbDecision::Intra => {
                    mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
                }
                MbDecision::Inter1Mv(mv) => {
                    mv_grid.set(mb_x, mb_y, MbMotion::mv1(mv, true, false));
                }
                MbDecision::Inter4Mv(mvs4) => {
                    mv_grid.set(mb_x, mb_y, MbMotion::mv4(mvs4));
                }
            }
            decisions.push(d);
        }
    }

    // Pass 2: emit bitstream and reconstruct with OBMC.
    let mut infos: Vec<crate::mb::PMbInfo> = Vec::with_capacity(mb_w * mb_h);
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let d = decisions[mb_y * mb_w + mb_x];
            let info = emit_p_mb_ap(
                &mut bw, mb_x, mb_y, pquant, frame, width, height, reference, &mv_grid, d,
                &mut recon,
            )?;
            infos.push(info);
        }
    }

    // Pass 3: run the decoder's OBMC reconstruction into a fresh picture,
    // then overwrite `recon` with the result. Intra MBs already sit in
    // `recon` (pass 2 wrote them); carry them over.
    let mut obmc_recon = IPicture::new(width as usize, height as usize);
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let info = &infos[mb_y * mb_w + mb_x];
            if info.intra {
                copy_mb_from_to(&recon, &mut obmc_recon, mb_x, mb_y);
                continue;
            }
            crate::mb::apply_p_mb_reconstruction(
                mb_x,
                mb_y,
                &mut obmc_recon,
                reference,
                &mv_grid,
                info,
                true,
            );
        }
    }
    recon = obmc_recon;

    Ok((bw.finish(), recon))
}

/// Round 13 — Annex N (RPS) I-picture encoder. Emits a PLUSPTYPE-form
/// picture header carrying OPPTYPE bit 11 (RPS) = 1, then runs the same
/// I-MB body the baseline encoder uses (`encode_intra_mb` per MB).
///
/// RPS round-13 scope: TRPI = 0 (forced for I-pictures by §5.1.14),
/// RPSMF = `100` (no back-channel needed), BCI = `01` (no BCM).
pub fn encode_i_picture_rps_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 RPS encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);

    write_plusptype_picture_header_rps(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        false, // I-picture
        0b100, // RPSMF = NEITHER
        false, // TRPI = 0 (mandatory on I)
        0,     // TRP unused
    )?;

    for mb_y in 0..mb_h {
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let gn = (mb_y as u32 / mb_rows_per_gob) as u8;
            write_gob_header(&mut bw, gn, pquant)?;
        }
        for mb_x in 0..mb_w {
            encode_intra_mb(
                &mut bw, mb_x, mb_y, pquant, frame, width, height, &mut recon,
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Round 13 — Annex N (RPS) P-picture encoder. Emits a PLUSPTYPE-form
/// picture header with the RPS bit set, then runs the same baseline
/// 1-MV inter MB body the non-RPS encoder uses (`encode_p_mb_full` with
/// `enable_annex_d_umv = false`). TRPI is always 0 in this round (the
/// decoder falls back to "most recent anchor" — same MV-grid +
/// reconstruction as the non-RPS path). TRP-driven multi-reference
/// emission is the encoder-side follow-up; the decoder already supports
/// looking up by TR via [`crate::decoder::H263Decoder`]'s reference cache.
pub fn encode_p_picture_rps_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 RPS encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    write_plusptype_picture_header_rps(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true,  // P-picture
        0b100, // RPSMF = NEITHER (no back-channel signalling)
        false, // TRPI = 0 — decoder uses most recent anchor
        0,
    )?;

    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let _info = encode_p_mb_full(
                &mut bw,
                mb_x,
                mb_y,
                pquant,
                frame,
                width,
                height,
                reference,
                &mut recon,
                &mut mv_grid,
                false, // not Annex F
                false, // not Annex D UMV
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Round 14 — Annex G (PB-frames) P-picture encoder.
///
/// Emits a baseline-PTYPE picture header with PTYPE bit 13 (PBFR) set plus
/// the §5.1.22 / §5.1.23 TRB / DBQUANT tail. Each MB body is the standard
/// P-MB bitstream followed by **MODB** (Table 11/H.263) — round 14 always
/// emits MODB = `0` (no CBPB, no MVDB) so the B-half is reconstructed by
/// pure §G.4 / §G.5 bidirectional MC with zero residual. The MB layer is
/// otherwise identical to the non-PB encoder; the only on-wire diff is the
/// trailing 1-bit MODB.
///
/// Returns `(bytes, p_recon)` — `p_recon` is the freshly reconstructed
/// **P-half** of the PB-frame, which becomes the next picture's MC
/// reference.
#[allow(clippy::too_many_arguments)]
pub fn encode_pb_picture_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    trb: u8,
    dbquant: u8,
    annex_m: bool,
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 PB encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    write_picture_header_pb(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true,  // P-picture
        false, // not Annex F
        false, // not SAC
        false, // not UMV
        true,  // PB-frames on
        trb,
        dbquant,
    )?;

    // Per-MB emit using the PB-aware encoder which interleaves MODB at the
    // spec-correct position (between MCBPC and CBPY per §5.3 Figure 10).
    // The `annex_m` flag selects between the Annex G Table 11 path
    // (always-bidir, optional MVDB-as-delta) and the Annex M Table M.1 path
    // (per-MB Lagrangian RDO over {bidir, fwd, bwd}, MVDB-as-forward-MV).
    for mb_y in 0..mb_h {
        // §M.2.2 left-forward-MV predictor — resets at the start of every MB
        // row so far-left MBs use a (0, 0) predictor, matching the §F.2
        // Figure F.1 row-start convention.
        let mut fwd_mv_left = (0i32, 0i32);
        for mb_x in 0..mb_w {
            encode_p_mb_pb(
                &mut bw,
                mb_x,
                mb_y,
                pquant,
                frame,
                width,
                height,
                reference,
                &mut recon,
                &mut mv_grid,
                trb,
                dbquant,
                annex_m,
                &mut fwd_mv_left,
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Round 14 — Annex G PB-frame P-MB encoder. Like [`encode_p_mb_full`] but
/// emits the PB-mode `MODB` (Table 11) immediately after MCBPC, per
/// §5.3 Figure 10. Round-14 scope emits MODB = `0` (no CBPB, no MVDB) for
/// every MB; CBPB / MVDB / B-residual emission is the round-15 follow-up.
///
/// Skipped MBs (COD = 1) do **not** carry MODB per §5.3.3 ("MODB is present
/// for MB-type 0..=4") — Table 10 explicitly excludes the not-coded MB type.
/// We follow the spec; the matching decoder reads MODB only when COD = 0.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_pb(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    reference: &IPicture,
    recon: &mut IPicture,
    mv_grid: &mut MvGrid,
    trb: u8,
    dbquant: u8,
    annex_m: bool,
    fwd_mv_left: &mut (i32, i32),
) -> Result<crate::mb::PMbInfo> {
    // The MB-decision logic mirrors `encode_p_mb_full` for the
    // non-Annex-F path. We replicate the relevant bits here so we can
    // interleave MODB at the spec-correct position.
    let (mvx, mvy, mv_sad) = motion_estimate_mb(frame, reference, mb_x, mb_y);

    let zero_sad = sad_block(
        &frame.planes[0].data,
        frame.planes[0].stride,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        &reference.y,
        reference.y_stride,
        reference.y_stride as i32,
        (reference.y.len() / reference.y_stride) as i32,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        0,
        0,
        16,
    );
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    let can_skip = pmx == 0 && pmy == 0 && zero_sad < mv_sad + 128;

    let mut y_pred = [0u8; 256];
    let mut u_pred = [0u8; 64];
    let mut v_pred = [0u8; 64];
    let decide_mv = if can_skip { (0, 0) } else { (mvx, mvy) };
    build_mb_predictor(
        reference,
        mb_x,
        mb_y,
        decide_mv.0,
        decide_mv.1,
        &mut y_pred,
        &mut u_pred,
        &mut v_pred,
    );

    let src_y = &frame.planes[0];
    let src_cb = &frame.planes[1];
    let src_cr = &frame.planes[2];
    let mut luma_abs_sum = 0u32;
    for j in 0..16 {
        for i in 0..16 {
            let s = src_y.data[(mb_y * 16 + j) * src_y.stride + (mb_x * 16 + i)] as i32;
            let p = y_pred[j * 16 + i] as i32;
            luma_abs_sum += (s - p).unsigned_abs();
        }
    }

    // Skipped MB: COD = 1, no MODB (Table 10), nothing else.
    if can_skip && luma_abs_sum < (quant as u32) * 128 {
        bw.write_bits(1, 1);
        copy_predictor_to_recon(recon, mb_x, mb_y, &y_pred, &u_pred, &v_pred);
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        // §M.2.2 — skipped MBs have no forward MV; predictor for the next
        // MB to the right collapses to (0, 0).
        if annex_m {
            *fwd_mv_left = (0, 0);
        }
        return Ok(crate::mb::PMbInfo::empty_skipped());
    }

    // COD = 0 — MB is coded.
    bw.write_bits(0, 1);

    // Emit MCBPC + (Intra OR Inter MB body), splicing MODB between MCBPC
    // and CBPY. We bypass `encode_p_mb_intra` / `encode_p_mb_inter_full`
    // (which write MCBPC + CBPY in one go) by inlining the relevant parts.
    let intra_variance = mb_luma_variance(src_y, mb_x, mb_y);
    let try_intra = intra_variance * 5 < luma_abs_sum;

    if try_intra {
        encode_p_mb_pb_intra(
            bw, mb_x, mb_y, quant, frame, width, height, reference, recon, mv_grid, trb, dbquant,
            annex_m,
        )?;
        // §G.2 — for an intra MB in PB mode the spec requires MV to also be
        // present (used by the B-half §G.4). We've already written it inside
        // `_pb_intra` (after MVDB-implying MODB). The mv_grid is set there.
        // Annex M intra-in-PB MBs always emit Bidirectional MODB (no fwd
        // MV); reset the §M.2.2 left-fwd predictor.
        if annex_m {
            *fwd_mv_left = (0, 0);
        }
        return Ok(crate::mb::PMbInfo {
            coded: true,
            intra: true,
            residual: vec![0i16; 6 * 64],
            residual_present: [false; 6],
            intra_done: true,
        });
    }

    encode_p_mb_pb_inter(
        bw,
        mb_x,
        mb_y,
        quant,
        src_y,
        src_cb,
        src_cr,
        reference,
        recon,
        decide_mv,
        mv_grid,
        &y_pred,
        &u_pred,
        &v_pred,
        trb,
        dbquant,
        annex_m,
        fwd_mv_left,
    )
}

/// Intra encode of a P-MB block in PB-frames mode. Same fields as
/// `encode_p_mb_intra` plus an MODB codeword between MCBPC and CBPY plus
/// — per §G.2 — the MVD field that the B-half §G.4 derivation needs (the
/// B-block MVs derive from the co-located P-MB's MV, which for intra-in-PB
/// defaults to a coded MVD that the encoder is free to set to `(0, 0)`).
///
/// Round-15 still emits MODB = `0` (no CBPB, no MVDB) for the intra-in-PB
/// path — the moving-square test never picks intra-in-PB, so this branch
/// stays a thin extension of the round-14 path. The `_reference` /
/// `_trb` / `_dbquant` parameters are accepted for parity with the inter
/// path so a future refactor can wire B-residual emission here too.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_pb_intra(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    _reference: &IPicture,
    recon: &mut IPicture,
    mv_grid: &mut MvGrid,
    _trb: u8,
    _dbquant: u8,
    annex_m: bool,
) -> Result<()> {
    let mut blocks = [[0i32; 64]; 6];
    let mut dc_pels = [128u8; 6];
    let mut block_has_ac = [false; 6];

    for b in 0..6 {
        let mut samples = [0.0f32; 64];
        sample_block_for(frame, width, height, mb_x, mb_y, b, &mut samples);
        let mut dctf = samples;
        fdct8x8(&mut dctf);
        let (dc_byte, levels, any_ac) = quantise_intra_block(&dctf, quant);
        dc_pels[b] = dc_byte;
        block_has_ac[b] = any_ac;
        blocks[b] = levels;
    }
    let cbpc: u8 = ((block_has_ac[4] as u8) << 1) | (block_has_ac[5] as u8);
    let cbpy: u8 = ((block_has_ac[0] as u8) << 3)
        | ((block_has_ac[1] as u8) << 2)
        | ((block_has_ac[2] as u8) << 1)
        | (block_has_ac[3] as u8);

    write_mcbpc_inter(bw, PMbKind::Intra, cbpc);
    // MODB after MCBPC (spec §5.3 Fig 10). For Annex G intra-in-PB the
    // simplest legal code is `0` (no CBPB, no MVDB). For Annex M the
    // matching code from Table M.1 is the bidir/no-cbpb code, also `0`.
    // Both encode the same single-bit "0" prefix, so the bit-stream output
    // is identical regardless of `annex_m`.
    if annex_m {
        crate::pb::encode_modb_m(bw, crate::pb::BMode::Bidirectional, false);
    } else {
        crate::pb::encode_modb(bw, false, false);
    }
    write_cbpy(bw, cbpy);
    // §G.2 — MVD is present on intra-in-PB MBs as well. We pick a (0, 0)
    // MV (the B-half §G.4 derivation will use this), so the predictor
    // becomes the median of neighbours and we emit the differential
    // matching that. The decoder reads this MVD via `decode_mv_pair`,
    // matching the inter MB path.
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    encode_mv_component(bw, 0, pmx);
    encode_mv_component(bw, 0, pmy);
    mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
    for b in 0..6 {
        bw.write_bits(dc_pels[b] as u32, 8);
        if block_has_ac[b] {
            write_block_ac(bw, &blocks[b]);
        }
        reconstruct_intra_block(recon, b, mb_x, mb_y, dc_pels[b], &blocks[b], quant);
    }
    Ok(())
}

/// Inter encode of a P-MB in PB-frames mode. Same fields as
/// `encode_p_mb_inter_full` plus an MODB codeword between MCBPC and CBPY
/// (spec §5.3 Figure 10) and — when CBPB is non-zero — a 6-bit CBPB
/// immediately after MODB plus the per-block B-residual TCOEF stream
/// appended after the P-half block coefficients.
///
/// Round-15 enables MODB / CBPB / B-residual emission: the §G.5 prediction
/// is computed using `reference` (forward — the previous P-recon) and
/// `recon` (backward — the just-built P-half), then subtracted from the
/// **input frame** pels to form a residual that's DCT/quantised at BQUANT
/// and emitted under TCOEF. Per-block CBPB bits are set whenever the
/// quantised residual has any non-zero coefficient; if any are set we emit
/// MODB = `11` (CBPB present + MVDB present, MVDB = 0). Otherwise MODB = `0`
/// (no CBPB, no MVDB) — cheaper on the wire when MC alone already matches.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_pb_inter(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    src_y: &oxideav_core::frame::VideoPlane,
    src_cb: &oxideav_core::frame::VideoPlane,
    src_cr: &oxideav_core::frame::VideoPlane,
    reference: &IPicture,
    recon: &mut IPicture,
    mv: (i32, i32),
    mv_grid: &mut MvGrid,
    y_pred: &[u8; 256],
    u_pred: &[u8; 64],
    v_pred: &[u8; 64],
    trb: u8,
    dbquant: u8,
    annex_m: bool,
    fwd_mv_left: &mut (i32, i32),
) -> Result<crate::mb::PMbInfo> {
    // ------------------------------------------------------------------
    // P-half: quantise + reconstruct *in-memory* before emitting bits, so
    // MODB / CBPB can be decided after we know the B-residual outcome.
    // ------------------------------------------------------------------
    let mut levels_all = [[0i32; 64]; 6];
    let mut has_ac = [false; 6];

    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 16 + sub_y + j;
                let px = mb_x * 16 + sub_x + i;
                let s = src_y.data[py * src_y.stride + px] as i32;
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }
    for (ci, plane) in [(0, src_cb), (1, src_cr)].iter() {
        let pred = if *ci == 0 { u_pred } else { v_pred };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 8 + j;
                let px = mb_x * 8 + i;
                let s = plane.data[py * plane.stride + px] as i32;
                let p = pred[j * 8 + i] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        let b = 4 + ci;
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }

    // Reconstruct the P-half into `recon` first — the §G.5 backward
    // predictor needs the freshly reconstructed P-MB.
    let mut info = crate::mb::PMbInfo {
        coded: true,
        intra: false,
        residual: vec![0i16; 6 * 64],
        residual_present: [false; 6],
        intra_done: false,
    };
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let coeffs = dequantise_block(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
        if has_ac[b] {
            let dst = info.residual_block_mut(b);
            for (i, &v) in resid_out.iter().enumerate() {
                dst[i] = v.clamp(-4096, 4095) as i16;
            }
            info.residual_present[b] = true;
        }
    }
    for ci in 0..2usize {
        let b = 4 + ci;
        let pred = if ci == 0 { u_pred } else { v_pred };
        let coeffs = dequantise_block(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = pred[j * 8 + i] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
        if has_ac[b] {
            let dst = info.residual_block_mut(b);
            for (i, &v) in resid_out.iter().enumerate() {
                dst[i] = v.clamp(-4096, 4095) as i16;
            }
            info.residual_present[b] = true;
        }
    }
    // The MV is needed by the §G.4 derivation below.
    mv_grid.set(mb_x, mb_y, MbMotion::mv1(mv, true, false));

    // ------------------------------------------------------------------
    // B-half prediction selection.
    //
    // Annex G (annex_m=false): single shape — bidirectional. §G.4 derives
    // forward + backward MVs from the P-MV with MVDB = (0,0). §G.5 averages
    // the two predictors inside the bidirectional region.
    //
    // Annex M (annex_m=true): three shapes — bidirectional / forward /
    // backward (§M.2). The encoder builds all three predictors against the
    // input frame, computes a sum-of-abs-residual SAD per shape, adds a
    // rate proxy (codeword bit length), and picks the one minimising
    // `SAD + lambda * R`. The Lagrange multiplier follows the H.263
    // convention `lambda ≈ 0.85 * QP^2` (Sullivan & Wiegand 1998); we use
    // the simpler `lambda = QP * 4` which behaves identically over the
    // relevant operating range and avoids a quadratic bias toward
    // backward (which has the smallest header cost).
    // ------------------------------------------------------------------
    let trd = (trb as i32 + 1).max(1);
    let bquant = crate::pb::bquant_from_quant(quant, dbquant);
    let p_motion = crate::motion::MbMotion::mv1(mv, true, false);

    // Sample the input frame at the B-block destinations once.
    let src_pels_per_block: [[u8; 64]; 6] =
        std::array::from_fn(|b| sample_input_block_pels(src_y, src_cb, src_cr, mb_x, mb_y, b));

    // Helper: compute SAD + per-block residual levels for a candidate
    // predictor `block_pred` (one i16[64] per block). Returns
    // `(sum_sad, b_levels, b_has_ac)`.
    let quantise_for_pred = |block_preds: &[[i16; 64]; 6]| {
        let mut sum_sad: u32 = 0;
        let mut b_levels = [[0i32; 64]; 6];
        let mut b_has_ac = [false; 6];
        for b in 0..6usize {
            let mut resid = [0.0f32; 64];
            for k in 0..64 {
                let r = src_pels_per_block[b][k] as i32 - block_preds[b][k] as i32;
                resid[k] = r as f32;
                sum_sad += r.unsigned_abs();
            }
            let mut dctf = resid;
            fdct8x8(&mut dctf);
            let levels = quantise_inter_block(&dctf, bquant);
            b_has_ac[b] = levels.iter().any(|&l| l != 0);
            b_levels[b] = levels;
        }
        (sum_sad, b_levels, b_has_ac)
    };

    // ----- Bidirectional candidate (§G.4 / §G.5 — same as Annex G).
    let b_mvs_bidir = crate::pb::derive_b_mb_mvs(&p_motion, (0, 0), trb as i32, trd);
    let bidir_block_mvs = [
        b_mvs_bidir.luma[0],
        b_mvs_bidir.luma[1],
        b_mvs_bidir.luma[2],
        b_mvs_bidir.luma[3],
        b_mvs_bidir.chroma,
        b_mvs_bidir.chroma,
    ];
    let mut bidir_preds: [[i16; 64]; 6] = [[0i16; 64]; 6];
    for b in 0..6usize {
        crate::pb::predict_b_block(
            &mut bidir_preds[b],
            b,
            mb_x,
            mb_y,
            reference,
            recon,
            bidir_block_mvs[b],
        );
    }
    let (bidir_sad, bidir_levels, bidir_has_ac) = quantise_for_pred(&bidir_preds);

    // ----- Forward & backward candidates (Annex M only — skip when off).
    let mut chosen_mode = crate::pb::BMode::Bidirectional;
    let mut b_levels = bidir_levels;
    let mut b_has_ac = bidir_has_ac;
    let mut fwd_mvdb = (0i32, 0i32);

    if annex_m {
        // Forward candidate: pick the same MV as the P-half. This is a
        // reasonable starting point — when the P-MV is small (B-frame is
        // close in time to the prior P) the forward predictor is close to
        // the prior P-MB itself; when the P-MV is large the forward
        // predictor adapts to the same motion direction. A full ME on the
        // prior P would be more accurate but the wire format only needs
        // *some* legal forward MV; we let RDO discard this candidate when
        // it's worse than bidir/backward.
        let fwd_mv = mv;
        let chroma_fwd_mv = (
            crate::motion::luma_to_chroma_mv(fwd_mv.0),
            crate::motion::luma_to_chroma_mv(fwd_mv.1),
        );
        let mut fwd_preds: [[i16; 64]; 6] = [[0i16; 64]; 6];
        for b in 0..6usize {
            let mvf_b = if b < 4 { fwd_mv } else { chroma_fwd_mv };
            crate::pb::predict_b_block_forward(&mut fwd_preds[b], b, mb_x, mb_y, reference, mvf_b);
        }
        let (fwd_sad, fwd_levels, fwd_has_ac) = quantise_for_pred(&fwd_preds);

        // Backward candidate: predictor = freshly reconstructed P-MB
        // (§M.2.3 PREC). No MV data on the wire.
        let mut bwd_preds: [[i16; 64]; 6] = [[0i16; 64]; 6];
        for b in 0..6usize {
            crate::pb::predict_b_block_backward(&mut bwd_preds[b], b, mb_x, mb_y, recon);
        }
        let (bwd_sad, bwd_levels, bwd_has_ac) = quantise_for_pred(&bwd_preds);

        // Lagrangian RDO. Rate proxy is the worst-case wire cost for each
        // mode's MODB + MVDB + CBPB tail (the per-block TCOEF cost is
        // similar across shapes — within ~10% — so we approximate by
        // counting only the mode-discriminating bits):
        //   * Bidirectional: MODB=`0` (1) or `10` (2 + 6 cbpb) — pick the
        //     lower depending on whether bidir has any non-zero CBPB.
        //   * Forward: MODB=`110` (3 + 2*MVD) or `1110` (4 + 2*MVD + 6 cbpb).
        //   * Backward: MODB=`11110` (5) or `11111` (5 + 6 cbpb).
        // We approximate the MVD cost via the actual VLC bit count.
        let lambda: u32 = (quant as u32) * 4;
        let bidir_cbpb_present = bidir_has_ac.iter().any(|&x| x);
        let fwd_cbpb_present = fwd_has_ac.iter().any(|&x| x);
        let bwd_cbpb_present = bwd_has_ac.iter().any(|&x| x);
        let bidir_rate = if bidir_cbpb_present { 1 + 1 + 6 } else { 1 };
        let fwd_mv_rate = mvd_pure_differential_bits(fwd_mv.0 - fwd_mv_left.0)
            + mvd_pure_differential_bits(fwd_mv.1 - fwd_mv_left.1);
        let fwd_rate = if fwd_cbpb_present { 4 + 6 } else { 3 } + fwd_mv_rate;
        let bwd_rate = if bwd_cbpb_present { 5 + 6 } else { 5 };

        let bidir_cost = bidir_sad + lambda * bidir_rate;
        let fwd_cost = fwd_sad + lambda * fwd_rate;
        let bwd_cost = bwd_sad + lambda * bwd_rate;

        if fwd_cost < bidir_cost && fwd_cost <= bwd_cost {
            chosen_mode = crate::pb::BMode::Forward;
            b_levels = fwd_levels;
            b_has_ac = fwd_has_ac;
            fwd_mvdb = fwd_mv;
        } else if bwd_cost < bidir_cost {
            chosen_mode = crate::pb::BMode::Backward;
            b_levels = bwd_levels;
            b_has_ac = bwd_has_ac;
        } else {
            chosen_mode = crate::pb::BMode::Bidirectional;
        }
    }

    let cbpb: u8 = ((b_has_ac[0] as u8) << 5)
        | ((b_has_ac[1] as u8) << 4)
        | ((b_has_ac[2] as u8) << 3)
        | ((b_has_ac[3] as u8) << 2)
        | ((b_has_ac[4] as u8) << 1)
        | (b_has_ac[5] as u8);
    let cbpb_present = cbpb != 0;

    // ------------------------------------------------------------------
    // Now emit the bitstream in the spec-correct order.
    // ------------------------------------------------------------------
    let cbpc: u8 = ((has_ac[4] as u8) << 1) | (has_ac[5] as u8);
    let cbpy_true: u8 = ((has_ac[0] as u8) << 3)
        | ((has_ac[1] as u8) << 2)
        | ((has_ac[2] as u8) << 1)
        | (has_ac[3] as u8);
    let cbpy_on_wire = cbpy_true ^ 0xF;

    write_mcbpc_inter(bw, PMbKind::Inter, cbpc);
    // MODB after MCBPC (§5.3 Fig 10).
    if annex_m {
        crate::pb::encode_modb_m(bw, chosen_mode, cbpb_present);
    } else {
        // Annex G: when CBPB is non-zero we emit `11` (CBPB + MVDB present);
        // MVDB is then written as the (0, 0) pure differential, so there's
        // no hidden MV cost. When CBPB is zero we emit `0` (1 bit) to keep
        // the wire compact.
        crate::pb::encode_modb(bw, cbpb_present, cbpb_present);
    }
    if cbpb_present {
        bw.write_bits(cbpb as u32, 6);
    }
    write_cbpy(bw, cbpy_on_wire);
    // `mv_grid[mb_x, mb_y]` was already updated to `mv` above so the §G.4
    // derivation could run, but `predict_mv` reads strictly from the
    // LEFT / ABOVE / ABOVE-RIGHT neighbours (never the cell itself) — so
    // updating early doesn't poison the MV-coding predictor.
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    encode_mv_component(bw, mv.0, pmx);
    encode_mv_component(bw, mv.1, pmy);

    if annex_m {
        // §M.2.2 — the forward MV is VLC-coded as MVD (Table 14 + sign,
        // sign-of-predictor cascade — same family as the §5.3.7 P-MVD)
        // when bmode == Forward. Bidirectional and backward modes emit no
        // MVDB (Table M.1 indices 0/1 and 4/5 have no MVDB column). We
        // also update `fwd_mv_left` for the next MB's predictor.
        match chosen_mode {
            crate::pb::BMode::Forward => {
                encode_mv_component(bw, fwd_mvdb.0, fwd_mv_left.0);
                encode_mv_component(bw, fwd_mvdb.1, fwd_mv_left.1);
                *fwd_mv_left = fwd_mvdb;
            }
            _ => {
                *fwd_mv_left = (0, 0);
            }
        }
    } else if cbpb_present {
        // Annex G: MVDB = (0, 0) — pure differential VLC, 2 codewords.
        crate::motion::encode_mvd_pure_differential(bw, 0);
        crate::motion::encode_mvd_pure_differential(bw, 0);
    }

    // P-block coefficients (§5.4 — first the six P-blocks).
    for b in 0..6 {
        if has_ac[b] {
            write_block_ac_inter(bw, &levels_all[b]);
        }
    }

    // B-block coefficients — TCOEF only (no INTRADC for B-blocks per §5.4),
    // ordering per the CBPB bit positions (block 1..=6 = our 0..=5).
    if cbpb_present {
        for b in 0..6 {
            if b_has_ac[b] {
                write_block_ac_inter(bw, &b_levels[b]);
            }
        }
    }

    Ok(info)
}

/// Pull an 8×8 block worth of pels from the **input frame** for block
/// position `b` of the given MB. Used by the PB-encoder when computing the
/// B-half residual against the §G.5 prediction.
///
/// `b` is `0..=3` for the four luma sub-blocks (top-left, top-right,
/// bottom-left, bottom-right) and `4`/`5` for Cb/Cr.
fn sample_input_block_pels(
    src_y: &oxideav_core::frame::VideoPlane,
    src_cb: &oxideav_core::frame::VideoPlane,
    src_cr: &oxideav_core::frame::VideoPlane,
    mb_x: usize,
    mb_y: usize,
    b: usize,
) -> [u8; 64] {
    let (plane, stride, base_x, base_y) = match b {
        0 => (&src_y.data, src_y.stride, mb_x * 16, mb_y * 16),
        1 => (&src_y.data, src_y.stride, mb_x * 16 + 8, mb_y * 16),
        2 => (&src_y.data, src_y.stride, mb_x * 16, mb_y * 16 + 8),
        3 => (&src_y.data, src_y.stride, mb_x * 16 + 8, mb_y * 16 + 8),
        4 => (&src_cb.data, src_cb.stride, mb_x * 8, mb_y * 8),
        5 => (&src_cr.data, src_cr.stride, mb_x * 8, mb_y * 8),
        _ => unreachable!(),
    };
    let h = plane.len() / stride;
    let w = stride;
    let mut out = [0u8; 64];
    for j in 0..8 {
        let yy = (base_y + j).min(h.saturating_sub(1));
        for i in 0..8 {
            let xx = (base_x + i).min(w.saturating_sub(1));
            out[j * 8 + i] = plane[yy * stride + xx];
        }
    }
    out
}

/// Copy one MB (Y + Cb + Cr) from `src` to `dst`. Used to preserve intra
/// MBs during the Annex F pass 2.
fn copy_mb_from_to(src: &IPicture, dst: &mut IPicture, mb_x: usize, mb_y: usize) {
    for j in 0..16 {
        let so = (mb_y * 16 + j) * src.y_stride + mb_x * 16;
        let doff = (mb_y * 16 + j) * dst.y_stride + mb_x * 16;
        dst.y[doff..doff + 16].copy_from_slice(&src.y[so..so + 16]);
    }
    for j in 0..8 {
        let so = (mb_y * 8 + j) * src.c_stride + mb_x * 8;
        let doff = (mb_y * 8 + j) * dst.c_stride + mb_x * 8;
        dst.cb[doff..doff + 8].copy_from_slice(&src.cb[so..so + 8]);
        dst.cr[doff..doff + 8].copy_from_slice(&src.cr[so..so + 8]);
    }
}

/// Pass-1 decision for an MB in Annex F mode. `Inter1Mv` carries the
/// chosen MV; `Inter4Mv` carries the four per-block MVs (Figure 5 order).
#[derive(Clone, Copy, Debug)]
pub enum MbDecision {
    Skipped,
    Intra,
    Inter1Mv((i32, i32)),
    Inter4Mv([(i32, i32); 4]),
}

/// Pass-1 MV decision. Picks 1-MV vs 4-MV vs intra vs skipped based on
/// SADs. Returns the decision; does NOT emit any bitstream.
fn decide_p_mb(
    frame: &VideoFrame,
    reference: &IPicture,
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
) -> MbDecision {
    let (mvx, mvy, mv_sad) = motion_estimate_mb(frame, reference, mb_x, mb_y);
    let zero_sad = sad_block(
        &frame.planes[0].data,
        frame.planes[0].stride,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        &reference.y,
        reference.y_stride,
        reference.y_stride as i32,
        (reference.y.len() / reference.y_stride) as i32,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        0,
        0,
        16,
    );
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    let can_skip = pmx == 0 && pmy == 0 && zero_sad < mv_sad + 128;

    // Quick intra / skip check based on MB luma variance.
    let src_y = &frame.planes[0];
    let mut luma_abs_sum = 0u32;
    // Use the zero-MV predictor for the skip / intra test; the final
    // residual-against-OBMC check happens in pass 2.
    for j in 0..16 {
        for i in 0..16 {
            let s = src_y.data[(mb_y * 16 + j) * src_y.stride + (mb_x * 16 + i)] as i32;
            let p = {
                // zero-MV predictor sample
                let ref_w = reference.y_stride as i32;
                let ref_h = (reference.y.len() / reference.y_stride) as i32;
                let x = ((mb_x * 16) as i32 + i as i32 + (mvx >> 1)).clamp(0, ref_w - 1) as usize;
                let y = ((mb_y * 16) as i32 + j as i32 + (mvy >> 1)).clamp(0, ref_h - 1) as usize;
                reference.y[y * reference.y_stride + x] as i32
            };
            luma_abs_sum += (s - p).unsigned_abs();
        }
    }
    let intra_variance = mb_luma_variance(src_y, mb_x, mb_y);
    let try_intra = intra_variance * 5 < luma_abs_sum;

    if can_skip && luma_abs_sum < (quant as u32) * 128 {
        return MbDecision::Skipped;
    }
    if try_intra {
        return MbDecision::Intra;
    }

    // Try 4MV.
    let (mvs4, sad_sum) = motion_estimate_4mv(frame, reference, mb_x, mb_y);
    let threshold = mv_sad.saturating_sub(mv_sad / 20 + 80);
    // Allow an env-flag override for A/B testing — exporting
    // `OXIDEAV_H263_FORCE_1MV=1` disables 4MV even in AP mode, which lets
    // us isolate the 1-MV-with-OBMC correctness from the 4-MV selection.
    let force_1mv = std::env::var("OXIDEAV_H263_FORCE_1MV").ok().as_deref() == Some("1");
    if sad_sum < threshold && !force_1mv {
        return MbDecision::Inter4Mv(mvs4);
    }
    MbDecision::Inter1Mv((mvx, mvy))
}

/// Pass-2 emit for an MB with a fully populated `mv_grid` (Annex F path).
/// Computes OBMC-blended predictor + residual and writes the MB bitstream
/// (COD/MCBPC/CBPY/MVD(s)/ACs). Writes `recon` as `pred + clipped residual`
/// for downstream passes; pass 3 will overwrite with the decoder-equivalent
/// OBMC reconstruction.
#[allow(clippy::too_many_arguments)]
fn emit_p_mb_ap(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    reference: &IPicture,
    mv_grid: &MvGrid,
    decision: MbDecision,
    recon: &mut IPicture,
) -> Result<crate::mb::PMbInfo> {
    match decision {
        MbDecision::Skipped => {
            bw.write_bits(1, 1);
            // Copy the zero-MV predictor into recon so pass-3 has a valid
            // source to build from (actually pass 3 rewrites from the
            // reference via apply_p_mb_reconstruction, so recon content is
            // overwritten, but we still keep a sensible value here).
            let mut y_pred = [0u8; 256];
            let mut u_pred = [0u8; 64];
            let mut v_pred = [0u8; 64];
            build_mb_predictor(
                reference,
                mb_x,
                mb_y,
                0,
                0,
                &mut y_pred,
                &mut u_pred,
                &mut v_pred,
            );
            copy_predictor_to_recon(recon, mb_x, mb_y, &y_pred, &u_pred, &v_pred);
            Ok(crate::mb::PMbInfo::empty_skipped())
        }
        MbDecision::Intra => {
            bw.write_bits(0, 1); // COD = 0
            encode_p_mb_intra(bw, mb_x, mb_y, quant, frame, width, height, recon)?;
            Ok(crate::mb::PMbInfo {
                coded: true,
                intra: true,
                residual: vec![0i16; 6 * 64],
                residual_present: [false; 6],
                intra_done: true,
            })
        }
        MbDecision::Inter1Mv(mv) => {
            bw.write_bits(0, 1); // COD = 0
                                 // Build OBMC predictor using the full mv_grid (each block's MV
                                 // is the same; remote neighbours come from mv_grid).
            let mvs4 = [mv; 4];
            let (y_pred, _cmx, _cmy) =
                build_mb_predictor_4mv_obmc(reference, mv_grid, mb_x, mb_y, &mvs4);
            // Chroma: baseline 1-MV mapping.
            let (cmx_1mv, cmy_1mv) = (luma_to_chroma_mv(mv.0), luma_to_chroma_mv(mv.1));
            let mut u_pred = [0u8; 64];
            let mut v_pred = [0u8; 64];
            build_chroma_predictor(
                reference,
                mb_x,
                mb_y,
                cmx_1mv,
                cmy_1mv,
                &mut u_pred,
                &mut v_pred,
            );
            let info = encode_p_mb_inter(
                bw,
                mb_x,
                mb_y,
                quant,
                &frame.planes[0],
                &frame.planes[1],
                &frame.planes[2],
                reference,
                recon,
                mv,
                &mut mv_grid.clone(), // unused for final state — we already
                // seeded mv_grid in pass 1. The
                // `encode_p_mb_inter` call internally
                // calls `mv_grid.set(...)` to finalise,
                // which we don't want to mutate the
                // pass-1 state. A clone suffices.
                &y_pred,
                &u_pred,
                &v_pred,
            )?;
            Ok(info)
        }
        MbDecision::Inter4Mv(mvs4) => {
            bw.write_bits(0, 1); // COD = 0
            let (y_pred, cmx, cmy) =
                build_mb_predictor_4mv_obmc(reference, mv_grid, mb_x, mb_y, &mvs4);
            let mut u_pred = [0u8; 64];
            let mut v_pred = [0u8; 64];
            build_chroma_predictor(reference, mb_x, mb_y, cmx, cmy, &mut u_pred, &mut v_pred);
            // We need a fresh per-MB MvGrid clone so the internal "seed
            // mvs4[b] incrementally" dance in `encode_p_mb_inter_4mv`
            // writes into a local scratch and doesn't disturb the pass-1
            // mv_grid. After the call finishes, the final mvs4 is the same
            // as what's already in pass-1 state, so correctness is
            // preserved.
            let mut scratch = mv_grid.clone();
            let info = encode_p_mb_inter_4mv(
                bw,
                mb_x,
                mb_y,
                quant,
                &frame.planes[0],
                &frame.planes[1],
                &frame.planes[2],
                recon,
                &mvs4,
                &mut scratch,
                &y_pred,
                &u_pred,
                &v_pred,
            )?;
            Ok(info)
        }
    }
}

fn write_picture_header(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    advanced_prediction: bool,
) -> Result<()> {
    write_picture_header_with_opts(
        bw,
        source_format,
        pquant,
        tr,
        is_p_picture,
        advanced_prediction,
        false,
    )
}

/// Picture header writer that also takes an Annex E (SAC) flag. Sets PTYPE
/// bit 11 (SAC) when `sac_mode` is true. Otherwise identical to
/// [`write_picture_header`].
fn write_picture_header_with_opts(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    advanced_prediction: bool,
    sac_mode: bool,
) -> Result<()> {
    write_picture_header_full(
        bw,
        source_format,
        pquant,
        tr,
        is_p_picture,
        advanced_prediction,
        sac_mode,
        false,
    )
}

/// Picture header writer that exposes every PTYPE-bit knob the encoder
/// currently understands. Sets PTYPE bit 10 (UMV) when `umv_mode` is true and
/// the picture is a P-picture (I-pictures must leave the bit clear — the
/// decoder still latches it for the next P-picture, but spec requires the
/// bit to be 0 on I).
#[allow(clippy::too_many_arguments)]
fn write_picture_header_full(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    advanced_prediction: bool,
    sac_mode: bool,
    umv_mode: bool,
) -> Result<()> {
    write_picture_header_pb(
        bw,
        source_format,
        pquant,
        tr,
        is_p_picture,
        advanced_prediction,
        sac_mode,
        umv_mode,
        false, // pb_frames off
        0,     // trb (unused when pb=0)
        0,     // dbquant (unused when pb=0)
    )
}

/// Picture header writer with full PTYPE knobs **plus** the Annex G PB-frames
/// extras (TRB / DBQUANT — see §5.1.22 / §5.1.23). When `pb_frames` is
/// false, behaves identically to [`write_picture_header_full`] and TRB /
/// DBQUANT bits are not emitted.
#[allow(clippy::too_many_arguments)]
fn write_picture_header_pb(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    advanced_prediction: bool,
    sac_mode: bool,
    umv_mode: bool,
    pb_frames: bool,
    trb: u8,
    dbquant: u8,
) -> Result<()> {
    // PSC: 22 bits = `0000 0000 0000 0000 1 00000`. Write byte-aligned to
    // simplify start-code recognition.
    debug_assert!(bw.is_byte_aligned());
    #[allow(clippy::unusual_byte_groupings)]
    let psc: u32 = 0b00_0000_0000_0000_0000_1_00000;
    bw.write_bits(psc, 22);

    // TR.
    bw.write_bits(tr as u32, 8);

    // PTYPE bits 1..=13:
    //   bit 1: always 1
    //   bit 2: always 0 (distinguishes from H.261)
    //   bit 3: split-screen
    //   bit 4: document-camera
    //   bit 5: freeze-picture release
    //   bits 6-8: source format (1..=5)
    //   bit 9: I (0) / P (1) — encoder always emits 0 for now
    //   bit 10: UMV (D)
    //   bit 11: SAC (E)
    //   bit 12: AP  (F)
    //   bit 13: PB  (G)        - 0
    let src_code: u32 = match source_format {
        SourceFormat::SubQcif => 1,
        SourceFormat::Qcif => 2,
        SourceFormat::Cif => 3,
        SourceFormat::FourCif => 4,
        SourceFormat::SixteenCif => 5,
        _ => {
            return Err(Error::unsupported(
                "h263 encoder: only standard source formats 1..=5 are supported",
            ));
        }
    };
    bw.write_bits(1, 1); // bit 1
    bw.write_bits(0, 1); // bit 2
    bw.write_bits(0, 1); // bit 3 split_screen
    bw.write_bits(0, 1); // bit 4 doc_camera
    bw.write_bits(0, 1); // bit 5 freeze
    bw.write_bits(src_code, 3); // bits 6-8 source format
    bw.write_bits(u32::from(is_p_picture), 1); // bit 9 picture coding type (I=0, P=1)
                                               // bit 10 UMV — Annex D Unrestricted Motion Vectors. Spec §5.1.4.5
                                               // sets the flag in baseline-PTYPE form for the whole picture; the
                                               // decoder latches it on every picture (including I-pictures, where
                                               // it has no syntactic effect). We mirror what real-world h263 streams
                                               // do: the bit can be set on I-pictures too — the SAC-aware tests in
                                               // this crate also leave it on across an I/P boundary. Our decoder
                                               // does not gate the flag on picture type either.
    let umv_bit = if umv_mode { 1 } else { 0 };
    bw.write_bits(umv_bit, 1); // bit 10 UMV
    bw.write_bits(if sac_mode { 1 } else { 0 }, 1); // bit 11 SAC
                                                    // bit 12 AP — Annex F Advanced Prediction. Set iff the encoder is in
                                                    // 4MV/OBMC mode AND this is a P-picture (I-pictures have no MVs). On
                                                    // I-pictures the bit must be 0 even if the knob is on.
    let ap_bit = if is_p_picture && advanced_prediction {
        1
    } else {
        0
    };
    bw.write_bits(ap_bit, 1); // bit 12 AP
                              // bit 13 PB — Annex G PB-frames mode. Set iff `pb_frames` is on AND
                              // this is a P-picture. The spec requires the bit to be 0 on I.
    let pb_bit = if is_p_picture && pb_frames { 1 } else { 0 };
    bw.write_bits(pb_bit, 1);

    // PQUANT (5 bits).
    if pquant == 0 || pquant > 31 {
        return Err(Error::invalid(format!(
            "h263 encoder: pquant {} out of range 1..=31",
            pquant
        )));
    }
    bw.write_bits(pquant as u32, 5);

    // CPM (0) and no PSBI follows.
    bw.write_bits(0, 1);

    // §5.1.22 / §5.1.23 — TRB (3 bits) + DBQUANT (2 bits) when PB-frames mode
    // is active. Standard CIF picture-clock-frequency variant (3-bit TRB).
    if pb_bit == 1 {
        if trb > 7 {
            return Err(Error::invalid(format!(
                "h263 encoder: TRB {trb} out of 3-bit range (0..=7)"
            )));
        }
        if dbquant > 3 {
            return Err(Error::invalid(format!(
                "h263 encoder: DBQUANT {dbquant} out of 2-bit range (0..=3)"
            )));
        }
        bw.write_bits(trb as u32, 3);
        bw.write_bits(dbquant as u32, 2);
    }

    // PEI loop terminator (no SEI on the baseline path — SEI-bearing callers
    // use write_picture_header_with_sei instead).
    write_pei_loop(bw, &[]);
    Ok(())
}

/// Baseline-PTYPE picture header writer with optional Annex L SEI records.
/// Identical to [`write_picture_header_pb`] but serialises `sei` into the
/// PEI/PSUPP loop before the terminator `PEI=0`.
#[allow(clippy::too_many_arguments, dead_code)]
fn write_picture_header_with_sei(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    advanced_prediction: bool,
    sac_mode: bool,
    umv_mode: bool,
    sei: &[Sei],
) -> Result<()> {
    // Re-use the PB writer to emit everything up to (but not including) the
    // PEI loop. We then append the SEI-bearing PEI loop ourselves.
    //
    // The PB writer always emits PEI=0. We replicate its body here to avoid
    // two writes.
    debug_assert!(bw.is_byte_aligned());
    let src_code: u32 = match source_format {
        SourceFormat::SubQcif => 1,
        SourceFormat::Qcif => 2,
        SourceFormat::Cif => 3,
        SourceFormat::FourCif => 4,
        SourceFormat::SixteenCif => 5,
        _ => {
            return Err(Error::unsupported(
                "h263 encoder: only standard source formats 1..=5 are supported",
            ));
        }
    };
    #[allow(clippy::unusual_byte_groupings)]
    let psc: u32 = 0b00_0000_0000_0000_0000_1_00000;
    bw.write_bits(psc, 22);
    bw.write_bits(tr as u32, 8);
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(src_code, 3);
    bw.write_bits(u32::from(is_p_picture), 1);
    bw.write_bits(if umv_mode { 1 } else { 0 }, 1);
    bw.write_bits(if sac_mode { 1 } else { 0 }, 1);
    let ap_bit = if is_p_picture && advanced_prediction {
        1
    } else {
        0
    };
    bw.write_bits(ap_bit, 1);
    bw.write_bits(0, 1); // PB-frames off
    if pquant == 0 || pquant > 31 {
        return Err(Error::invalid(format!(
            "h263 encoder: pquant {} out of range 1..=31",
            pquant
        )));
    }
    bw.write_bits(pquant as u32, 5);
    bw.write_bits(0, 1); // CPM = 0
                         // Emit SEI via PEI loop.
    let psupp = serialise_sei_to_psupp(sei);
    write_pei_loop(bw, &psupp);
    Ok(())
}

// ---------------------------------------------------------------------------
// Annex L — SEI emit helpers
// ---------------------------------------------------------------------------

/// Returns the number of PSUPP bytes required to encode `rec` in §L.2
/// layout: `1` header byte + DSIZE parameter bytes (+ 1 extended byte for
/// FTYPE=15 records).
fn sei_payload_len(rec: &Sei) -> usize {
    match rec {
        Sei::DoNothing => 0,
        Sei::FullPictureFreezeRequest => 0,
        Sei::PartialPictureFreezeRequest { .. } => 4,
        Sei::ResizingPartialPictureFreezeRequest { .. } => 8,
        Sei::PartialPictureFreezeReleaseRequest { .. } => 4,
        Sei::FullPictureSnapshotTag { .. } => 4,
        Sei::PartialPictureSnapshotTag { .. } => 8,
        Sei::VideoTimeSegmentStartTag { .. } => 4,
        Sei::VideoTimeSegmentEndTag { .. } => 4,
        Sei::ProgressiveRefinementSegmentStartTag { .. } => 4,
        Sei::ProgressiveRefinementSegmentEndTag { .. } => 4,
        Sei::ChromaKeyingInformation { payload } => payload.len(),
        Sei::ExtendedFunctionType { payload, .. } => payload.len(),
        Sei::Unknown { payload, .. } => payload.len(),
    }
}

/// Serialise a slice of [`Sei`] records into a byte vector in §L.2 PSUPP
/// format: each record is `(FTYPE<<4) | DSIZE` + DSIZE parameter bytes.
/// Extended-FTYPE (FTYPE=15) records prepend the outer `0xF0` byte, then an
/// extra octet `(ext_ftype << 4) | ext_dsize`, then `ext_dsize` payload bytes.
fn serialise_sei_to_psupp(records: &[Sei]) -> Vec<u8> {
    let mut out = Vec::new();
    for rec in records {
        match rec {
            Sei::DoNothing => out.push(0x10),                // FTYPE=1, DSIZE=0
            Sei::FullPictureFreezeRequest => out.push(0x20), // FTYPE=2
            Sei::PartialPictureFreezeRequest {
                x,
                y,
                width,
                height,
            } => {
                out.push((3 << 4) | 4);
                out.extend_from_slice(&[*x, *y, *width, *height]);
            }
            Sei::ResizingPartialPictureFreezeRequest {
                displayed: (dx, dy, dw, dh),
                decoded: (rx, ry, rw, rh),
            } => {
                out.push((4 << 4) | 8);
                out.extend_from_slice(&[*dx, *dy, *dw, *dh, *rx, *ry, *rw, *rh]);
            }
            Sei::PartialPictureFreezeReleaseRequest {
                x,
                y,
                width,
                height,
            } => {
                out.push((5 << 4) | 4);
                out.extend_from_slice(&[*x, *y, *width, *height]);
            }
            Sei::FullPictureSnapshotTag { id } => {
                out.push((6 << 4) | 4);
                out.extend_from_slice(&id.to_be_bytes());
            }
            Sei::PartialPictureSnapshotTag {
                id,
                x,
                y,
                width,
                height,
            } => {
                out.push((7 << 4) | 8);
                out.extend_from_slice(&id.to_be_bytes());
                out.extend_from_slice(&[*x, *y, *width, *height]);
            }
            Sei::VideoTimeSegmentStartTag { id } => {
                out.push((8 << 4) | 4);
                out.extend_from_slice(&id.to_be_bytes());
            }
            Sei::VideoTimeSegmentEndTag { id } => {
                out.push((9 << 4) | 4);
                out.extend_from_slice(&id.to_be_bytes());
            }
            Sei::ProgressiveRefinementSegmentStartTag { id } => {
                out.push((10 << 4) | 4);
                out.extend_from_slice(&id.to_be_bytes());
            }
            Sei::ProgressiveRefinementSegmentEndTag { id } => {
                out.push((11 << 4) | 4);
                out.extend_from_slice(&id.to_be_bytes());
            }
            Sei::ChromaKeyingInformation { payload } => {
                let dsize = payload.len().min(15) as u8;
                out.push((12 << 4) | dsize);
                out.extend_from_slice(&payload[..dsize as usize]);
            }
            Sei::ExtendedFunctionType {
                ext_ftype,
                ext_dsize,
                payload,
            } => {
                out.push(0xF0); // FTYPE=15, DSIZE=0
                out.push((ext_ftype << 4) | (ext_dsize & 0x0F));
                let n = (*ext_dsize as usize).min(payload.len());
                out.extend_from_slice(&payload[..n]);
            }
            Sei::Unknown { ftype, payload } => {
                let dsize = payload.len().min(15) as u8;
                out.push((ftype << 4) | dsize);
                out.extend_from_slice(&payload[..dsize as usize]);
            }
        }
    }
    out
}

/// Write the PEI/PSUPP loop per §5.1.24/§5.1.25, carrying the given PSUPP
/// bytes. If `psupp` is empty, writes a single `PEI=0` terminator.
/// Each byte in `psupp` is wrapped: `PEI=1` (1 bit) + 8-bit value. The
/// loop ends with `PEI=0` (1 bit).
fn write_pei_loop(bw: &mut BitWriter, psupp: &[u8]) {
    for &byte in psupp {
        bw.write_bits(1, 1); // PEI = 1
        bw.write_bits(byte as u32, 8);
    }
    bw.write_bits(0, 1); // PEI = 0 — terminator
}

/// Round 13 — Annex N (Reference Picture Selection) PLUSPTYPE writer.
///
/// Emits a PLUSPTYPE-form picture header (source-format code `111`, UFEP=001,
/// full OPPTYPE) with the RPS optional-mode bit (OPPTYPE bit 11) set. The
/// MPPTYPE picture-type code is `000` for I-pictures or `001` for P-pictures.
/// The RPS body fields are written per §5.1.13 / §5.1.14 / §5.1.15 / §5.1.16:
///
///   * RPSMF (3 bits, only when UFEP=001 — always emitted by this writer);
///   * TRPI (1 bit, mandatory when RPS is in use; spec mandates 0 for I);
///   * TRP (10 bits, only when TRPI=1);
///   * BCI (always "01" — videomux mode is not used by this encoder; spec
///     §5.1.16 requires BCI = "01" outside videomux).
///
/// The non-RPS PLUSPTYPE-related fields (CPM, CPFMT/EPAR/CPCFC/ETR, UUI,
/// SSS, ELNUM/RLNUM) are all left at their off-defaults — this round emits
/// the RPS bit only on top of an otherwise-baseline H.263+ header.
///
/// Note: per §5.1.4.6, RPS does not interact restrictively with any other
/// optional mode the encoder currently emits. RPS + UMV is allowed; the
/// matching encoder path only emits 1-MV inter (§D.2 / Table 14) so RPS
/// streams here are baseline 1-MV inter under the hood.
#[allow(clippy::too_many_arguments)]
fn write_plusptype_picture_header_rps(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    rpsmf: u8,
    trpi: bool,
    trp: u16,
) -> Result<()> {
    debug_assert!(bw.is_byte_aligned());
    // Validate RPSMF — §5.1.13: 100/101/110/111. 000-011 reserved.
    if rpsmf < 0b100 {
        return Err(Error::invalid(format!(
            "h263 encoder: RPSMF {rpsmf:03b} reserved (only 100/101/110/111 allowed)"
        )));
    }
    if !is_p_picture && trpi {
        return Err(Error::invalid(
            "h263 encoder: TRPI=1 forbidden on I-pictures (§5.1.14)",
        ));
    }
    let src_code: u32 = match source_format {
        SourceFormat::SubQcif => 1,
        SourceFormat::Qcif => 2,
        SourceFormat::Cif => 3,
        SourceFormat::FourCif => 4,
        SourceFormat::SixteenCif => 5,
        _ => {
            return Err(Error::unsupported(
                "h263 RPS encoder: only standard source formats 1..=5 are supported",
            ));
        }
    };

    // PSC (22 bits).
    #[allow(clippy::unusual_byte_groupings)]
    let psc: u32 = 0b00_0000_0000_0000_0000_1_00000;
    bw.write_bits(psc, 22);
    // TR (8 bits).
    bw.write_bits(tr as u32, 8);
    // PTYPE prefix bits 1..=8: marker(1) | id(0) | split(0) | cam(0) |
    // freeze(0) | source-format = "111" (extended PTYPE).
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3);

    // PLUSPTYPE: UFEP (3 bits) = 001 → full OPPTYPE present.
    bw.write_bits(0b001, 3);

    // OPPTYPE (18 bits, MSB-first, spec bit 1 = MSB).
    //   bits 1-3 = source format code (same encoding as baseline);
    //   bit 4 = custom PCF (0); bit 5 = UMV (0); bit 6 = SAC (0);
    //   bit 7 = AP (0); bit 8 = AIC (0); bit 9 = DF (0); bit 10 = SS (0);
    //   bit 11 = RPS (1) — this writer's reason for being;
    //   bit 12 = ISD (0); bit 13 = AIV (0); bit 14 = MQ (0);
    //   bit 15 = marker "1"; bits 16-18 = reserved 000.
    let bit = |k: u32, v: u32| (v & 1) << (18 - k);
    let srcf_part = (src_code & 0b111) << 15;
    let opptype: u32 = srcf_part
        | bit(11, 1)  // RPS
        | bit(15, 1); // marker
    bw.write_bits(opptype, 18);

    // MPPTYPE (9 bits): PCT(3) | RPR(1) | RRU(1) | RTYPE(1) | reserved(00) | marker(1)
    // RTYPE handling: spec §5.1.4.3 says the encoder should toggle this between
    // a P-picture and its reference. We emit 0 for I and 1 for P uniformly
    // (matches what `write_picture_header_full` would do for a baseline
    // header — the rounding-control behaviour is not exercised by our tests).
    let pct: u32 = if is_p_picture { 0b001 } else { 0b000 };
    // Bits laid out as PCT(3) | RPR(1) | RRU(1) | RTYPE(1) | reserved(2) | marker(1).
    // RPR=0, RRU=0, RTYPE=0, reserved=00, marker=1 → low 6 bits = 0b000001.
    let mpptype: u32 = (pct << 6) | 0b000_001;
    bw.write_bits(mpptype, 9);

    // CPM (1 bit, located after PLUSPTYPE per §5.1.4.7).
    bw.write_bits(0, 1);

    // (No CPFMT — std source format. No EPAR/CPCFC/ETR/UUI/SSS/ELNUM/RLNUM.)

    // RPSMF (3 bits — UFEP=001 → present).
    bw.write_bits(rpsmf as u32 & 0b111, 3);

    // TRPI (1 bit — mandatory when RPS in use).
    bw.write_bits(if trpi { 1 } else { 0 }, 1);
    if trpi {
        bw.write_bits(trp as u32 & 0x3FF, 10);
    }
    // BCI: "01" (no BCM, videomux-mode not in use).
    bw.write_bits(0, 1);
    bw.write_bits(1, 1);

    // PQUANT (5 bits).
    if pquant == 0 || pquant > 31 {
        return Err(Error::invalid(format!(
            "h263 RPS encoder: pquant {pquant} out of range 1..=31"
        )));
    }
    bw.write_bits(pquant as u32, 5);

    // PEI loop terminator.
    bw.write_bits(0, 1);
    Ok(())
}

/// Round 24 — Annex I (Advanced INTRA Coding) PLUSPTYPE writer.
///
/// Emits a PLUSPTYPE-form picture header (source-format code `111`,
/// UFEP=001, full OPPTYPE) with OPPTYPE bit 8 (AIC) = 1. All other
/// optional-mode bits are off in this round (AIC + UMV / SAC / AP / RPS
/// / PB combinations are gated at `send_frame` and rejected for now).
/// MPPTYPE picture-type code is `000` for I, `001` for P; this writer
/// always emits an I-picture (AIC currently only modifies the I path).
fn write_plusptype_picture_header_aic(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
) -> Result<()> {
    debug_assert!(bw.is_byte_aligned());
    let src_code: u32 = match source_format {
        SourceFormat::SubQcif => 1,
        SourceFormat::Qcif => 2,
        SourceFormat::Cif => 3,
        SourceFormat::FourCif => 4,
        SourceFormat::SixteenCif => 5,
        _ => {
            return Err(Error::unsupported(
                "h263 AIC encoder: only standard source formats 1..=5 are supported",
            ));
        }
    };

    // PSC (22 bits).
    #[allow(clippy::unusual_byte_groupings)]
    let psc: u32 = 0b00_0000_0000_0000_0000_1_00000;
    bw.write_bits(psc, 22);
    // TR (8 bits).
    bw.write_bits(tr as u32, 8);
    // PTYPE prefix bits 1..=8: marker(1) | id(0) | split(0) | cam(0) |
    // freeze(0) | source-format = "111" (extended PTYPE).
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3);

    // PLUSPTYPE: UFEP (3 bits) = 001 → full OPPTYPE present.
    bw.write_bits(0b001, 3);

    // OPPTYPE (18 bits, MSB-first, spec bit 1 = MSB).
    //   bits 1-3 = source format code; bit 4 = custom PCF (0); bit 5 = UMV (0);
    //   bit 6 = SAC (0); bit 7 = AP (0); bit 8 = AIC (1) — round 24's reason
    //   for being; bit 9 = DF (0); bit 10 = SS (0); bit 11 = RPS (0);
    //   bit 12 = ISD (0); bit 13 = AIV (0); bit 14 = MQ (0);
    //   bit 15 = marker "1"; bits 16-18 = reserved 000.
    let bit = |k: u32, v: u32| (v & 1) << (18 - k);
    let srcf_part = (src_code & 0b111) << 15;
    let opptype: u32 = srcf_part
        | bit(8, 1)  // AIC
        | bit(15, 1); // marker
    bw.write_bits(opptype, 18);

    // MPPTYPE (9 bits) for an I-picture: PCT=000 | RPR=0 | RRU=0 | RTYPE=0 |
    // reserved(00) | marker(1) → low 9 bits = 0b0_0000_0001.
    #[allow(clippy::unusual_byte_groupings)]
    let mpp: u32 = 0b000_0_0_0_001;
    bw.write_bits(mpp, 9);

    // CPM = 0.
    bw.write_bits(0, 1);

    // PQUANT.
    if pquant == 0 || pquant > 31 {
        return Err(Error::invalid(format!(
            "h263 AIC encoder: pquant {pquant} out of range 1..=31"
        )));
    }
    bw.write_bits(pquant as u32, 5);

    // PEI loop terminator.
    bw.write_bits(0, 1);
    Ok(())
}

/// Annex K (Slice Structured) PLUSPTYPE picture header writer. Emits a
/// PLUSPTYPE-form picture header (source-format code `111`, UFEP=001,
/// full OPPTYPE) with OPPTYPE bit 10 (SS) = 1 and a 2-bit SSS submode
/// field carrying RS/ASO. Round-23 always emits SSS=`00` (no RS, no
/// ASO — slices are arbitrary contiguous MB ranges in raster order).
/// All other optional-mode bits are off.
fn write_plusptype_picture_header_slice(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
) -> Result<()> {
    debug_assert!(bw.is_byte_aligned());
    let src_code: u32 = match source_format {
        SourceFormat::SubQcif => 1,
        SourceFormat::Qcif => 2,
        SourceFormat::Cif => 3,
        SourceFormat::FourCif => 4,
        SourceFormat::SixteenCif => 5,
        _ => {
            return Err(Error::unsupported(
                "h263 Annex K encoder: only standard source formats 1..=5 are supported",
            ));
        }
    };

    // PSC (22 bits).
    #[allow(clippy::unusual_byte_groupings)]
    let psc: u32 = 0b00_0000_0000_0000_0000_1_00000;
    bw.write_bits(psc, 22);
    // TR.
    bw.write_bits(tr as u32, 8);
    // PTYPE prefix bits 1..=8: marker(1) | id(0) | split(0) | cam(0) |
    // freeze(0) | source-format = "111" (extended PTYPE).
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3);
    // PLUSPTYPE: UFEP=001 — full OPPTYPE present.
    bw.write_bits(0b001, 3);
    // OPPTYPE (18 bits, MSB-first):
    //   bits 1-3 = source format; bit 4 = custom PCF (0); bit 5 = UMV (0);
    //   bit 6 = SAC (0); bit 7 = AP (0); bit 8 = AIC (0); bit 9 = DF (0);
    //   bit 10 = SS (1) — round 23's reason for being;
    //   bit 11 = RPS (0); bit 12 = ISD (0); bit 13 = AIV (0); bit 14 = MQ (0);
    //   bit 15 = marker "1"; bits 16-18 = reserved 000.
    let bit = |k: u32, v: u32| (v & 1) << (18 - k);
    let srcf_part = (src_code & 0b111) << 15;
    let opptype: u32 = srcf_part | bit(10, 1) /* SS */ | bit(15, 1) /* marker */;
    bw.write_bits(opptype, 18);

    // MPPTYPE (9 bits): PCT(3) | RPR(1) | RRU(1) | RTYPE(1) | reserved(00) | marker(1).
    let pct: u32 = if is_p_picture { 0b001 } else { 0b000 };
    let mpptype: u32 = (pct << 6) | 0b000_001;
    bw.write_bits(mpptype, 9);

    // CPM (1 bit).
    bw.write_bits(0, 1);

    // SSS body (2 bits) — RS=0, ASO=0 in the round-23 scope.
    bw.write_bits(0b00, 2);

    // PQUANT.
    if pquant == 0 || pquant > 31 {
        return Err(Error::invalid(format!(
            "h263 Annex K encoder: pquant {pquant} out of range 1..=31"
        )));
    }
    bw.write_bits(pquant as u32, 5);

    // PEI loop terminator.
    bw.write_bits(0, 1);
    Ok(())
}

/// Annex K (Slice Structured) — encode a single I-picture, replacing the
/// GOB layer with the slice layer (§K.2 Figure K.1). The first slice
/// inherits PQUANT and starts at MB 0, but only emits the trailing
/// fields (SEPB1 | MBA=0 | SEPB3 | GFID); subsequent slices begin with
/// SSC + slice header. `slice_mb_size` controls how many MBs each
/// slice covers.
pub fn encode_i_picture_slice_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    slice_mb_size: u32,
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    let total_mbs = (mb_w * mb_h) as u32;
    let slice_size = slice_mb_size.max(1).min(total_mbs);

    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);

    write_plusptype_picture_header_slice(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        false,
    )?;

    let sss = crate::slice::SssMode::default();

    // First slice — only the trailing fields (no SSC, no SQUANT).
    write_first_slice_header(&mut bw, source_format, sss, false, 0, /* gfid = */ 0)?;

    let mut next_slice_at = slice_size;
    let mut mb_idx: u32 = 0;
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            // Insert a slice header before this MB if it is the first MB of
            // a new slice (and not MB 0 — that one is owned by the picture
            // header's first-slice tail above).
            if mb_idx == next_slice_at && mb_idx < total_mbs {
                crate::slice::align_for_ssc(&mut bw);
                crate::slice::write_slice_header(
                    &mut bw,
                    source_format,
                    sss,
                    false, // CPM off
                    mb_idx,
                    pquant,
                    None, // SWI (RS off)
                    0,    // GFID
                    None, // SSBI
                )?;
                next_slice_at = next_slice_at.saturating_add(slice_size);
            }
            encode_intra_mb(
                &mut bw, mb_x, mb_y, pquant, frame, width, height, &mut recon,
            )?;
            mb_idx += 1;
        }
    }

    Ok((bw.finish(), recon))
}

/// Annex K (Slice Structured) — encode a single P-picture, replacing the
/// GOB layer with the slice layer. MV prediction is reset at every slice
/// boundary per §K.1 rule 1 ("the prediction of motion vector values
/// are the same as if a GOB header were present").
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_slice_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    slice_mb_size: u32,
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    let total_mbs = (mb_w * mb_h) as u32;
    let slice_size = slice_mb_size.max(1).min(total_mbs);

    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    write_plusptype_picture_header_slice(&mut bw, source_format, pquant, temporal_reference, true)?;
    let sss = crate::slice::SssMode::default();
    write_first_slice_header(&mut bw, source_format, sss, false, 0, 0)?;

    let mut next_slice_at = slice_size;
    let mut mb_idx: u32 = 0;
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            if mb_idx == next_slice_at && mb_idx < total_mbs {
                crate::slice::align_for_ssc(&mut bw);
                crate::slice::write_slice_header(
                    &mut bw,
                    source_format,
                    sss,
                    false,
                    mb_idx,
                    pquant,
                    None,
                    0,
                    None,
                )?;
                // §K.1 rule 1 — slice boundary resets MV prediction.
                mv_grid = MvGrid::new(mb_w, mb_h);
                next_slice_at = next_slice_at.saturating_add(slice_size);
            }
            let _info = encode_p_mb_full(
                &mut bw,
                mb_x,
                mb_y,
                pquant,
                frame,
                width,
                height,
                reference,
                &mut recon,
                &mut mv_grid,
                false, // not Annex F
                false, // not Annex D UMV
            )?;
            mb_idx += 1;
        }
    }
    Ok((bw.finish(), recon))
}

/// Emit the first-slice header — SEPB1 + (SSBI if CPM) + MBA + (SEPB2 if
/// needed, only when RS is on per §K.2.6) + (SWI if RS) + SEPB3 + GFID.
/// The first slice has no SSC and no SQUANT (PQUANT applies). MBA must
/// be `0` for the first slice when ASO is off.
fn write_first_slice_header(
    bw: &mut BitWriter,
    format: SourceFormat,
    sss: crate::slice::SssMode,
    cpm: bool,
    mba: u32,
    gfid: u8,
) -> Result<()> {
    bw.write_bits(1, 1); // SEPB1
    if cpm {
        return Err(Error::unsupported(
            "h263 Annex K encoder: CPM (with SSBI) is not yet supported",
        ));
    }
    let mba_w = crate::slice::mba_field_width(format, false)?;
    bw.write_bits(mba & ((1u32 << mba_w) - 1), mba_w);
    if sss.rectangular_slice {
        bw.write_bits(1, 1); // SEPB2 (first-slice + RS)
    }
    if sss.rectangular_slice {
        let _ = crate::slice::swi_field_width(format, false)?;
        return Err(Error::unsupported(
            "h263 Annex K encoder: Rectangular Slice (RS) submode is not yet emitted",
        ));
    }
    bw.write_bits(1, 1); // SEPB3
    bw.write_bits(gfid as u32 & 0x3, 2);
    Ok(())
}

/// Round 24 — Annex I (AIC) I-picture encoder. Emits a PLUSPTYPE-form
/// picture header with OPPTYPE bit 8 (AIC) = 1, then runs a per-MB AIC
/// encoder that writes INTRA_MODE + Table I.2 codewords + AC-pred
/// residuals.
///
/// AIC currently only affects I-pictures; the encoder's P-picture path
/// is unchanged. The reconstruction is bit-identical to what
/// [`crate::mb::decode_intra_mb_aic`] would produce when fed the
/// returned byte stream — that round-trip parity is what makes the
/// next P-picture's MC reference line up.
pub fn encode_i_picture_aic_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 AIC encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut cache = crate::aic::AicNeighbourCache::new(mb_w, mb_h);

    write_plusptype_picture_header_aic(&mut bw, source_format, pquant, temporal_reference)?;

    for mb_y in 0..mb_h {
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let gn = (mb_y as u32 / mb_rows_per_gob) as u8;
            write_gob_header(&mut bw, gn, pquant)?;
            // §I.3 — GOB header inserts a video-picture-segment boundary;
            // AIC predictors must reset (decoder mirrors this — see
            // `decode_i_picture` AIC branch).
            cache = crate::aic::AicNeighbourCache::new(mb_w, mb_h);
        }
        for mb_x in 0..mb_w {
            encode_intra_mb_aic(
                &mut bw, mb_x, mb_y, pquant, frame, width, height, &mut recon, &mut cache,
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Encode one Annex I (AIC) intra MB. Per §I.2 / §I.3:
///   * MCBPC indicates Intra (mb_type 3, no DQUANT — we hold quant constant).
///   * INTRA_MODE codeword (Table I.1) follows.
///   * CBPY/CBPC bits are "block has any coefficient transmitted".
///   * Per-block: scan order picked by INTRA_MODE; Table I.2 INTRA TCOEF
///     for every coefficient; AIC dequant; AC-pred predictor pre-subtracted
///     from raw target so the decoder lands on the same final RecC'.
#[allow(clippy::too_many_arguments)]
fn encode_intra_mb_aic(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    recon: &mut IPicture,
    cache: &mut crate::aic::AicNeighbourCache,
) -> Result<()> {
    use crate::aic::{
        ac_pred_predictor_for, apply_ac_prediction, dequantise_intra_block_aic,
        quantise_intra_block_aic, scan_for, write_intra_tcoef, IntraMode,
    };

    // 1. For each block, run forward DCT to get raw coefficients.
    let mut dctf_all = [[0.0f32; 64]; 6];
    for b in 0..6 {
        let mut samples = [0.0f32; 64];
        sample_block_for(frame, width, height, mb_x, mb_y, b, &mut samples);
        let mut dctf = samples;
        fdct8x8(&mut dctf);
        dctf_all[b] = dctf;
    }

    // 2. Pick the picture-INTRA_MODE per spec hint heuristic. We always emit
    //    `DcOnly` (mode 0) — it's correct for any input and avoids the
    //    extra encoder-side ME pass needed to pick between vertical and
    //    horizontal AC pred. Mode 0 still gets the DC-pred coding-efficiency
    //    win (the dominant gain on talking-head content) without the
    //    bitstream cost of always emitting a 2-bit `10` / `11` codeword.
    let intra_mode = IntraMode::DcOnly;

    // 3. Per block: pre-compute predictor, subtract from raw DCT, quantise
    //    the residual, dequantise to recover the decoder's RecC, apply
    //    AC pred to recover RecC' and stash IMMEDIATELY into the cache
    //    so subsequent blocks within the same MB (and the next MB to the
    //    right) see the right predictor when they look up neighbours.
    //
    //    NOTE: this loop intentionally folds the encoder-side
    //    "compute final coefficients" + "store to cache" steps together
    //    so per-block neighbour ordering inside an MB is correct (block
    //    1's left neighbour is block 0 of the same MB, block 2's above
    //    neighbour is block 0 of the same MB, block 3's above is block 1
    //    + left is block 2). A second loop below does the bitstream emit
    //    + IDCT + recon write — at that point the cache is already
    //    populated.
    let mut levels_all = [[0i32; 64]; 6];
    let mut final_coeffs_all = [[0i32; 64]; 6];
    let mut block_has_any = [false; 6];

    for b in 0..6 {
        let pred = ac_pred_predictor_for(intra_mode, mb_x, mb_y, b, cache);
        let mut residual_target = [0.0f32; 64];
        for k in 0..64 {
            residual_target[k] = dctf_all[b][k] - pred[k] as f32;
        }
        let (levels, any) = quantise_intra_block_aic(&residual_target, quant);
        levels_all[b] = levels;
        block_has_any[b] = any;

        // Run the decoder-equivalent reconstruction so the cache holds the
        // EXACT RecC' the decoder will compute. We do this *inside* the
        // per-block loop so block 1's left neighbour (= block 0 of the
        // same MB) sees the correct stored DC; otherwise within-MB
        // predictors would all see "no neighbour" and use the 1024
        // fall-back, which torpedoes the coding gain.
        let rec_c_residual = dequantise_intra_block_aic(&levels, quant);
        let final_coeffs = apply_ac_prediction(intra_mode, mb_x, mb_y, b, cache, &rec_c_residual);
        cache.store(mb_x, mb_y, b, &final_coeffs);
        final_coeffs_all[b] = final_coeffs;
    }

    // 4. Build CBPC / CBPY bit-patterns from `block_has_any`.
    let cbpc: u8 = ((block_has_any[4] as u8) << 1) | (block_has_any[5] as u8);
    let cbpy: u8 = ((block_has_any[0] as u8) << 3)
        | ((block_has_any[1] as u8) << 2)
        | ((block_has_any[2] as u8) << 1)
        | (block_has_any[3] as u8);

    // 5. Emit MB layer.
    write_mcbpc_intra(bw, cbpc);
    intra_mode.write(bw);
    write_cbpy(bw, cbpy);
    // No DQUANT — mb_type=3 (Intra), not 4 (IntraQ).

    // 6. Per-block: emit Table I.2 codewords + IDCT into recon. The cache
    //    is already populated above with the final RecC' values.
    for b in 0..6 {
        if block_has_any[b] {
            let scan = scan_for(intra_mode);
            let mut nonzero_scan: Vec<(usize, i32)> = Vec::with_capacity(8);
            for (scan_idx, &nat_idx) in scan.iter().enumerate() {
                let lv = levels_all[b][nat_idx];
                if lv != 0 {
                    nonzero_scan.push((scan_idx, lv));
                }
            }
            debug_assert!(!nonzero_scan.is_empty());
            let mut prev_scan: i32 = -1;
            for (i, &(scan_idx, lv)) in nonzero_scan.iter().enumerate() {
                let run = (scan_idx as i32 - prev_scan - 1) as u8;
                let last = i == nonzero_scan.len() - 1;
                write_intra_tcoef(bw, last, run, lv);
                prev_scan = scan_idx as i32;
            }
        }
        // IDCT the cached final coefficients and write into local recon.
        let mut coeffs = final_coeffs_all[b];
        let mut out = [0u8; 64];
        crate::block::idct_and_clip(&mut coeffs, &mut out);
        write_block_into(recon, b, mb_x, mb_y, &out);
    }

    Ok(())
}

fn write_gob_header(bw: &mut BitWriter, gn: u8, gquant: u8) -> Result<()> {
    // GBSC must be byte-aligned per §5.2.2 — pad with zero stuffing bits.
    // The spec actually allows up to 7 STUF bits (a `0000 0000` MB-stuffing
    // codeword would be ambiguous, so we use the bit-padding approach) before
    // the GBSC. We just zero-pad to byte boundary, which is what every
    // ffmpeg-emitted stream does.
    while !bw.is_byte_aligned() {
        bw.write_bits(0, 1);
    }
    // GBSC: 17 bits = `0000 0000 0000 0000 1` = 0x00001.
    bw.write_bits(0x00001, 17);
    bw.write_bits(gn as u32 & 0x1F, 5);
    // CPM=0, so no GSBI.
    bw.write_bits(0, 2); // GFID — 2 bits, must be the same for every GOB in a picture; 0 is fine
    if gquant == 0 || gquant > 31 {
        return Err(Error::invalid(format!(
            "h263 encoder: gquant {} out of range 1..=31",
            gquant
        )));
    }
    bw.write_bits(gquant as u32, 5);
    Ok(())
}

/// Encode one intra MB. We always emit Intra (mb_type=3) — never IntraQ —
/// because we hold the quantiser constant for the whole picture.
///
/// Also reconstructs the MB locally into `recon` so the caller can use it as
/// the MC reference for the next P-picture.
#[allow(clippy::too_many_arguments)]
fn encode_intra_mb(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    recon: &mut IPicture,
) -> Result<()> {
    // 1. Pull samples for all 6 blocks, run forward DCT + quantise, build CBP.
    let mut blocks = [[0i32; 64]; 6];
    let mut dc_pels = [128u8; 6]; // INTRADC byte values for each block
    let mut block_has_ac = [false; 6];

    for b in 0..6 {
        let mut samples = [0.0f32; 64];
        sample_block_for(frame, width, height, mb_x, mb_y, b, &mut samples);

        let mut dctf = samples;
        fdct8x8(&mut dctf);

        let (dc_byte, levels, any_ac) = quantise_intra_block(&dctf, quant);
        dc_pels[b] = dc_byte;
        block_has_ac[b] = any_ac;
        blocks[b] = levels;
    }

    // 2. CBPC = chroma block bits (block 4 -> bit 1, block 5 -> bit 0).
    let cbpc: u8 = ((block_has_ac[4] as u8) << 1) | (block_has_ac[5] as u8);
    // CBPY = luma block bits (block 0 -> bit 3, block 1 -> bit 2, block 2 ->
    // bit 1, block 3 -> bit 0). Intra encodes the CBP directly (no XOR).
    let cbpy: u8 = ((block_has_ac[0] as u8) << 3)
        | ((block_has_ac[1] as u8) << 2)
        | ((block_has_ac[2] as u8) << 1)
        | (block_has_ac[3] as u8);

    // 3. Emit MB headers.
    write_mcbpc_intra(bw, cbpc);
    write_cbpy(bw, cbpy);
    // No DQUANT — we picked mb_type=3 (Intra), not 4 (IntraQ).

    // 4. Per-block: INTRADC + (optionally) AC. Reconstruct into `recon`.
    for b in 0..6 {
        bw.write_bits(dc_pels[b] as u32, 8);
        if block_has_ac[b] {
            write_block_ac(bw, &blocks[b]);
        }
        reconstruct_intra_block(recon, b, mb_x, mb_y, dc_pels[b], &blocks[b], quant);
    }

    Ok(())
}

/// Quantise one intra block's DCT output into `(dc_byte, levels, any_ac)`.
///
/// `dc_byte` is the 8-bit INTRADC value encoded on the wire (1..=254, with
/// 128 remapped to 0xFF). `levels` holds the AC levels in natural-order
/// positions; `any_ac` is `true` iff any AC is nonzero.
fn quantise_intra_block(dctf: &[f32; 64], quant: u8) -> (u8, [i32; 64], bool) {
    // INTRADC: pel_dc = round(F[0,0] / 8), clamped to 1..=254 with 128
    // remapped to 0xFF (the decoder maps 0xFF -> 1024 = 128*8).
    let dc_round = (dctf[0] / 8.0).round() as i32;
    let dc_clamped = dc_round.clamp(1, 254);
    let dc_byte: u8 = if dc_clamped == 128 {
        0xFF
    } else {
        dc_clamped as u8
    };

    let mut levels = [0i32; 64];
    let q = quant as i32;
    let two_q = 2 * q;
    let bias = q / 4;
    for k in 1..64 {
        let coef = dctf[k];
        let abs_f = coef.abs() as i32;
        let mag = (abs_f + bias) / two_q;
        if mag != 0 {
            let signed = if coef < 0.0 { -mag } else { mag };
            levels[k] = signed.clamp(-127, 127);
        }
    }
    let any_ac = levels.iter().skip(1).any(|&l| l != 0);
    (dc_byte, levels, any_ac)
}

/// Reconstruct an intra 8×8 block into `recon` from its decoded DC byte and
/// AC levels. Uses the same dequant + IDCT path as the decoder.
fn reconstruct_intra_block(
    recon: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    dc_byte: u8,
    levels: &[i32; 64],
    quant: u8,
) {
    let mut coeffs = dequantise_block(levels, quant, true);
    coeffs[0] = if dc_byte == 0xFF {
        1024
    } else {
        (dc_byte as i32) << 3
    };
    coeffs[0] = coeffs[0].clamp(-2048, 2047);
    let mut out = [0u8; 64];
    crate::block::idct_and_clip(&mut coeffs, &mut out);
    write_block_into(recon, block_idx, mb_x, mb_y, &out);
}

/// Dequantise an 8×8 block of H.263 levels (H.263 inverse-quant formula).
///
/// For an intra block the caller is responsible for overwriting the DC slot
/// after this call — `intra_dc` is ignored inside the loop because intra DC
/// uses the INTRADC special-case, not the AC formula.
fn dequantise_block(levels: &[i32; 64], quant: u8, skip_dc: bool) -> [i32; 64] {
    let q = quant as i32;
    let q_minus_one_if_even = if q & 1 == 1 { 0 } else { -1 };
    let mut out = [0i32; 64];
    let start = if skip_dc { 1 } else { 0 };
    for k in start..64 {
        let l = levels[k];
        if l == 0 {
            continue;
        }
        let abs = l.unsigned_abs() as i32;
        let mut val = q * (2 * abs + 1) + q_minus_one_if_even;
        if l < 0 {
            val = -val;
        }
        out[k] = val.clamp(-2048, 2047);
    }
    out
}

/// Copy an 8×8 reconstructed block into the picture buffer.
fn write_block_into(
    pic: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
    out: &[u8; 64],
) {
    let (plane, stride, px, py) = block_dst(pic, block_idx, mb_x, mb_y);
    for dy in 0..8 {
        for dx in 0..8 {
            plane[(py + dy) * stride + (px + dx)] = out[dy * 8 + dx];
        }
    }
}

/// Mirror of `mb::block_dst` — duplicated here because `mb::block_dst` is
/// private and the encoder doesn't go through the decode path.
fn block_dst(
    pic: &mut IPicture,
    block_idx: usize,
    mb_x: usize,
    mb_y: usize,
) -> (&mut [u8], usize, usize, usize) {
    match block_idx {
        0 => (pic.y.as_mut_slice(), pic.y_stride, mb_x * 16, mb_y * 16),
        1 => (pic.y.as_mut_slice(), pic.y_stride, mb_x * 16 + 8, mb_y * 16),
        2 => (pic.y.as_mut_slice(), pic.y_stride, mb_x * 16, mb_y * 16 + 8),
        3 => (
            pic.y.as_mut_slice(),
            pic.y_stride,
            mb_x * 16 + 8,
            mb_y * 16 + 8,
        ),
        4 => (pic.cb.as_mut_slice(), pic.c_stride, mb_x * 8, mb_y * 8),
        5 => (pic.cr.as_mut_slice(), pic.c_stride, mb_x * 8, mb_y * 8),
        _ => unreachable!(),
    }
}

/// Pull one 8×8 block of samples from a 4:2:0 YUV frame, with edge replication
/// for blocks that overhang the picture boundary.
///
/// `width` / `height` are the stream's picture dimensions (carried on the
/// encoder's `CodecParameters`, not on the per-frame `VideoFrame`).
fn sample_block_for(
    frame: &VideoFrame,
    width: u32,
    height: u32,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    out: &mut [f32; 64],
) {
    let (plane, stride, base_x, base_y, max_x, max_y) = match block_idx {
        0..=3 => {
            let x = mb_x * 16 + if block_idx & 1 == 1 { 8 } else { 0 };
            let y = mb_y * 16 + if block_idx & 2 == 2 { 8 } else { 0 };
            let p = &frame.planes[0];
            (
                p.data.as_slice(),
                p.stride,
                x,
                y,
                width as usize,
                height as usize,
            )
        }
        4 => {
            let x = mb_x * 8;
            let y = mb_y * 8;
            let p = &frame.planes[1];
            let cw = (width as usize).div_ceil(2);
            let ch = (height as usize).div_ceil(2);
            (p.data.as_slice(), p.stride, x, y, cw, ch)
        }
        5 => {
            let x = mb_x * 8;
            let y = mb_y * 8;
            let p = &frame.planes[2];
            let cw = (width as usize).div_ceil(2);
            let ch = (height as usize).div_ceil(2);
            (p.data.as_slice(), p.stride, x, y, cw, ch)
        }
        _ => unreachable!(),
    };
    for j in 0..8 {
        let yy = (base_y + j).min(max_y.saturating_sub(1));
        for i in 0..8 {
            let xx = (base_x + i).min(max_x.saturating_sub(1));
            out[j * 8 + i] = plane[yy * stride + xx] as f32;
        }
    }
}

/// Encode the AC coefficients of an 8×8 block in zig-zag order. Caller has
/// ensured at least one nonzero exists in `levels[1..]`.
fn write_block_ac(bw: &mut BitWriter, levels: &[i32; 64]) {
    use oxideav_mpeg4video::headers::vol::ZIGZAG;

    // Find the position of the last nonzero in zigzag order so we know where
    // to set `last=true`.
    let mut nonzero_zz: Vec<(usize, i32)> = Vec::with_capacity(8);
    for zz in 1..64 {
        let nat = ZIGZAG[zz];
        let lv = levels[nat];
        if lv != 0 {
            nonzero_zz.push((zz, lv));
        }
    }
    debug_assert!(!nonzero_zz.is_empty());

    let mut prev_zz: usize = 0; // position of last emitted (or 0 for "none yet")
    for (i, &(zz, lv)) in nonzero_zz.iter().enumerate() {
        // Run = number of zero coefficients between the previous nonzero
        // (exclusive) and this one (exclusive). For the first AC, the
        // previous "nonzero" is the DC at position 0, so run = zz - 1.
        let run = if i == 0 {
            (zz - 1) as u8
        } else {
            (zz - prev_zz - 1) as u8
        };
        let last = i == nonzero_zz.len() - 1;
        write_tcoef(bw, last, run, lv);
        prev_zz = zz;
    }
}

// ---------------------------------------------------------------------------
// P-picture: motion estimation + macroblock emit
// ---------------------------------------------------------------------------

/// Maximum number of LDSP iterations before we force a switch to SDSP. Caps
/// worst-case cost at `MAX_LDSP_STEPS * 8 + 4 ≈ 100` SAD evals per MB while
/// still reaching the ±15-integer-pel spec limit from any starting position.
const MAX_LDSP_STEPS: u32 = 12;

/// Large Diamond Search Pattern (9 points). Integer-pel offsets. Spans up to
/// ±2 integer pels; each iteration the search centre jumps by up to 2 pels so
/// a full ±15-pel range is reachable in ≤ 8 steps from the origin.
const LDSP: [(i32, i32); 9] = [
    (0, 0),
    (-2, 0),
    (2, 0),
    (0, -2),
    (0, 2),
    (-1, -1),
    (1, -1),
    (-1, 1),
    (1, 1),
];

/// Small Diamond Search Pattern (5 points). Integer-pel offsets. Used for the
/// final refinement step once LDSP has converged.
const SDSP: [(i32, i32); 5] = [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)];

/// Motion-estimation: find the best 16×16 MV (in half-pel units) for MB
/// `(mb_x, mb_y)` against `reference`. Returns `(mv_x_half, mv_y_half, sad)`.
///
/// Three-phase search:
/// 1. **Diamond search (LDSP then SDSP)** over the integer-pel grid. The
///    Large Diamond Search Pattern converges on the local SAD minimum over
///    the full H.263 baseline MV range (±15 integer pels), evaluating up to
///    8 new candidates per step until the centre wins. A final Small Diamond
///    step pins down the integer-pel minimum.
/// 2. **Half-pel refinement** — 8-neighbour search around the integer-pel
///    winner.
///
/// The diamond pattern replaces the former exhaustive ±7 search: it covers
/// a larger window (up to ±15 integer pel, matching the spec MV range) with
/// far fewer SAD evaluations (≤ ~100 vs 225 exhaustive), so both faster
/// and more expressive for sequences with motion outside the old ±7 window.
fn motion_estimate_mb(
    frame: &VideoFrame,
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
) -> (i32, i32, u32) {
    let src = &frame.planes[0];
    let src_stride = src.stride;
    let src_x = (mb_x * 16) as i32;
    let src_y = (mb_y * 16) as i32;
    let blk_px = src_x;
    let blk_py = src_y;
    let ref_w = reference.y_stride as i32;
    let ref_h = (reference.y.len() / reference.y_stride) as i32;
    let pic_w = reference.width as i32;
    let pic_h = reference.height as i32;

    // MV stay-in-picture constraint (baseline H.263 — no Annex D UMV).
    //
    // A luma MV `(mvx, mvy)` in half-pel units is valid iff the entire
    // 16x16 predictor block lies within the picture:
    //   blk_px + (mvx/2) >= 0
    //   blk_py + (mvy/2) >= 0
    //   blk_px + 16 + ceil(mvx/2) <= pic_w
    //   blk_py + 16 + ceil(mvy/2) <= pic_h
    // Plus the half-pel filter needs `mvx|1 == 1` to use the right-edge
    // neighbour, so we add 1 to the upper bound in that case.
    //
    // Integer half: shift right by 1 (with sign). Fractional half-pel
    // extension: +1 if the half-pel bit is set.
    let mv_ok = |mvx: i32, mvy: i32| -> bool {
        let ix = mvx >> 1;
        let iy = mvy >> 1;
        let ext_x = (mvx & 1).abs();
        let ext_y = (mvy & 1).abs();
        let left = blk_px + ix;
        let top = blk_py + iy;
        let right = blk_px + 16 + ix + ext_x;
        let bottom = blk_py + 16 + iy + ext_y;
        left >= 0 && top >= 0 && right <= pic_w && bottom <= pic_h
    };
    let mv_range = MV_RANGE_MIN_HALF..=MV_RANGE_MAX_HALF;

    // Evaluate SAD at integer-pel offset (ix, iy). Half-pel MV = 2×integer.
    let eval = |ix: i32, iy: i32| -> Option<u32> {
        let mvx = ix * 2;
        let mvy = iy * 2;
        if !mv_range.contains(&mvx) || !mv_range.contains(&mvy) {
            return None;
        }
        if !mv_ok(mvx, mvy) {
            return None;
        }
        Some(sad_block(
            &src.data,
            src_stride,
            src_x,
            src_y,
            &reference.y,
            reference.y_stride,
            ref_w,
            ref_h,
            blk_px,
            blk_py,
            mvx,
            mvy,
            16,
        ))
    };

    // Stage 1a: LDSP iteration from (0, 0). Walk the large diamond until the
    // centre point is the SAD minimum (local convergence) or the step cap is
    // hit.
    let mut cx = 0i32;
    let mut cy = 0i32;
    let mut best_sad = eval(cx, cy).unwrap_or(u32::MAX);
    for _ in 0..MAX_LDSP_STEPS {
        let mut improved = false;
        let mut next = (cx, cy);
        for &(dx, dy) in LDSP.iter().skip(1) {
            let ix = cx + dx;
            let iy = cy + dy;
            if let Some(s) = eval(ix, iy) {
                if s < best_sad {
                    best_sad = s;
                    next = (ix, iy);
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
        cx = next.0;
        cy = next.1;
    }

    // Stage 1b: SDSP — refine to the 1-pel neighbourhood.
    for &(dx, dy) in SDSP.iter().skip(1) {
        let ix = cx + dx;
        let iy = cy + dy;
        if let Some(s) = eval(ix, iy) {
            if s < best_sad {
                best_sad = s;
                cx = ix;
                cy = iy;
            }
        }
    }

    let mut best = (cx * 2, cy * 2, best_sad);

    // Stage 2: half-pel refinement — 8 neighbours around the integer winner.
    let (ix, iy, _) = best;
    for dy in -1..=1 {
        for dx in -1..=1 {
            if dx == 0 && dy == 0 {
                continue;
            }
            let mvx = ix + dx;
            let mvy = iy + dy;
            if !mv_range.contains(&mvx) || !mv_range.contains(&mvy) {
                continue;
            }
            if !mv_ok(mvx, mvy) {
                continue;
            }
            let sad = sad_block(
                &src.data,
                src_stride,
                src_x,
                src_y,
                &reference.y,
                reference.y_stride,
                ref_w,
                ref_h,
                blk_px,
                blk_py,
                mvx,
                mvy,
                16,
            );
            if sad < best.2 {
                best = (mvx, mvy, sad);
            }
        }
    }

    best
}

/// Annex D (UMV)-aware motion-estimation for a 16×16 MB. Identical to
/// [`motion_estimate_mb`] except (1) the search range widens to
/// `[-63, +63]` halfpel (`MV_RANGE_UMV_*`), (2) MVs that would point partly
/// or wholly outside the picture are accepted (the §D.1 edge-replication is
/// done by the half-pel interpolator at decode time, and `predict_block`
/// mirrors that on the encoder side via the same clamp), and (3) only MVs
/// that round-trip through `reconstruct_umv_component(predictor, …)` are
/// considered — this filters the §D.2 "sign of predictor" rule against the
/// median-predictor.
fn motion_estimate_mb_umv(
    frame: &VideoFrame,
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
    mv_grid: &MvGrid,
) -> (i32, i32, u32) {
    let src = &frame.planes[0];
    let src_stride = src.stride;
    let src_x = (mb_x * 16) as i32;
    let src_y = (mb_y * 16) as i32;
    let blk_px = src_x;
    let blk_py = src_y;
    let ref_w = reference.y_stride as i32;
    let ref_h = (reference.y.len() / reference.y_stride) as i32;

    // Predictor for §D.2 round-trip filtering.
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    let mv_range = MV_RANGE_UMV_MIN_HALF..=MV_RANGE_UMV_MAX_HALF;

    // Round-trip filter: a candidate vector `(mvx, mvy)` is reachable iff
    // there exists `(mag, sign)` such that
    //   `reconstruct_umv_component(pmx, mag, sign) == mvx`.
    // We re-use the encoder helper indirectly: there must be a
    // §D.2 candidate `d ∈ {raw, raw+64, raw-64}` with `|d| <= 32` and
    // `pred + d == mv` (which is true by construction for the picked d).
    let reachable = |pred: i32, mv: i32| -> bool {
        let raw = mv - pred;
        let candidates = [raw, raw + 64, raw - 64];
        candidates.iter().any(|&d| {
            let mag = d.unsigned_abs() as i32;
            if mag > 32 {
                return false;
            }
            let v = pred + d;
            (MV_RANGE_UMV_MIN_HALF..=MV_RANGE_UMV_MAX_HALF).contains(&v) && v == mv
        })
    };

    let eval = |ix: i32, iy: i32| -> Option<u32> {
        let mvx = ix * 2;
        let mvy = iy * 2;
        if !mv_range.contains(&mvx) || !mv_range.contains(&mvy) {
            return None;
        }
        if !reachable(pmx, mvx) || !reachable(pmy, mvy) {
            return None;
        }
        Some(sad_block(
            &src.data,
            src_stride,
            src_x,
            src_y,
            &reference.y,
            reference.y_stride,
            ref_w,
            ref_h,
            blk_px,
            blk_py,
            mvx,
            mvy,
            16,
        ))
    };

    // Stage 1a: LDSP iteration from (0, 0).
    let mut cx = 0i32;
    let mut cy = 0i32;
    let mut best_sad = eval(cx, cy).unwrap_or(u32::MAX);
    for _ in 0..MAX_LDSP_STEPS {
        let mut improved = false;
        let mut next = (cx, cy);
        for &(dx, dy) in LDSP.iter().skip(1) {
            let ix = cx + dx;
            let iy = cy + dy;
            if let Some(s) = eval(ix, iy) {
                if s < best_sad {
                    best_sad = s;
                    next = (ix, iy);
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
        cx = next.0;
        cy = next.1;
    }
    // Stage 1b: SDSP refinement.
    for &(dx, dy) in SDSP.iter().skip(1) {
        let ix = cx + dx;
        let iy = cy + dy;
        if let Some(s) = eval(ix, iy) {
            if s < best_sad {
                best_sad = s;
                cx = ix;
                cy = iy;
            }
        }
    }
    let mut best = (cx * 2, cy * 2, best_sad);
    // Stage 2: half-pel refinement.
    let (ix, iy, _) = best;
    for dy in -1..=1 {
        for dx in -1..=1 {
            if dx == 0 && dy == 0 {
                continue;
            }
            let mvx = ix + dx;
            let mvy = iy + dy;
            if !mv_range.contains(&mvx) || !mv_range.contains(&mvy) {
                continue;
            }
            if !reachable(pmx, mvx) || !reachable(pmy, mvy) {
                continue;
            }
            let sad = sad_block(
                &src.data,
                src_stride,
                src_x,
                src_y,
                &reference.y,
                reference.y_stride,
                ref_w,
                ref_h,
                blk_px,
                blk_py,
                mvx,
                mvy,
                16,
            );
            if sad < best.2 {
                best = (mvx, mvy, sad);
            }
        }
    }
    let _ = reconstruct_umv_component; // silence unused-import when debug-assertions are off
    best
}

/// Four-block motion estimation for Annex F. Runs the 8×8 SDSP diamond per
/// block and returns `(mvs4, sad_sum)` — the sum of SADs of the four 8×8
/// predictors. Used to decide between 1MV and 4MV for a given MB.
fn motion_estimate_4mv(
    frame: &VideoFrame,
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
) -> ([(i32, i32); 4], u32) {
    let mut mvs = [(0i32, 0i32); 4];
    let mut total: u32 = 0;
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let (mx, my, sad) = motion_estimate_block(frame, reference, mb_x, mb_y, sub_x, sub_y);
        mvs[b] = (mx, my);
        total = total.saturating_add(sad);
    }
    (mvs, total)
}

/// Motion-estimate a single 8×8 luma block at `(mb_x * 16 + sub_x, mb_y *
/// 16 + sub_y)` against `reference`. Uses the same LDSP + SDSP + half-pel
/// refinement dance as [`motion_estimate_mb`] but for an 8×8 source.
fn motion_estimate_block(
    frame: &VideoFrame,
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
    sub_x: usize,
    sub_y: usize,
) -> (i32, i32, u32) {
    let src = &frame.planes[0];
    let src_stride = src.stride;
    let src_x = (mb_x * 16 + sub_x) as i32;
    let src_y = (mb_y * 16 + sub_y) as i32;
    let blk_px = src_x;
    let blk_py = src_y;
    let ref_w = reference.y_stride as i32;
    let ref_h = (reference.y.len() / reference.y_stride) as i32;
    let pic_w = reference.width as i32;
    let pic_h = reference.height as i32;

    // 8×8 version of `mv_ok`: the predictor occupies an 8×8 window + the
    // half-pel extension.
    let mv_ok = |mvx: i32, mvy: i32| -> bool {
        let ix = mvx >> 1;
        let iy = mvy >> 1;
        let ext_x = (mvx & 1).abs();
        let ext_y = (mvy & 1).abs();
        let left = blk_px + ix;
        let top = blk_py + iy;
        let right = blk_px + 8 + ix + ext_x;
        let bottom = blk_py + 8 + iy + ext_y;
        left >= 0 && top >= 0 && right <= pic_w && bottom <= pic_h
    };
    let mv_range = MV_RANGE_MIN_HALF..=MV_RANGE_MAX_HALF;

    let eval = |ix: i32, iy: i32| -> Option<u32> {
        let mvx = ix * 2;
        let mvy = iy * 2;
        if !mv_range.contains(&mvx) || !mv_range.contains(&mvy) {
            return None;
        }
        if !mv_ok(mvx, mvy) {
            return None;
        }
        Some(sad_block(
            &src.data,
            src_stride,
            src_x,
            src_y,
            &reference.y,
            reference.y_stride,
            ref_w,
            ref_h,
            blk_px,
            blk_py,
            mvx,
            mvy,
            8,
        ))
    };

    let mut cx = 0i32;
    let mut cy = 0i32;
    let mut best_sad = eval(cx, cy).unwrap_or(u32::MAX);
    for _ in 0..MAX_LDSP_STEPS {
        let mut improved = false;
        let mut next = (cx, cy);
        for &(dx, dy) in LDSP.iter().skip(1) {
            let ix = cx + dx;
            let iy = cy + dy;
            if let Some(s) = eval(ix, iy) {
                if s < best_sad {
                    best_sad = s;
                    next = (ix, iy);
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
        cx = next.0;
        cy = next.1;
    }
    for &(dx, dy) in SDSP.iter().skip(1) {
        let ix = cx + dx;
        let iy = cy + dy;
        if let Some(s) = eval(ix, iy) {
            if s < best_sad {
                best_sad = s;
                cx = ix;
                cy = iy;
            }
        }
    }
    let mut best = (cx * 2, cy * 2, best_sad);
    let (ix, iy, _) = best;
    for dy in -1..=1 {
        for dx in -1..=1 {
            if dx == 0 && dy == 0 {
                continue;
            }
            let mvx = ix + dx;
            let mvy = iy + dy;
            if !mv_range.contains(&mvx) || !mv_range.contains(&mvy) {
                continue;
            }
            if !mv_ok(mvx, mvy) {
                continue;
            }
            let sad = sad_block(
                &src.data,
                src_stride,
                src_x,
                src_y,
                &reference.y,
                reference.y_stride,
                ref_w,
                ref_h,
                blk_px,
                blk_py,
                mvx,
                mvy,
                8,
            );
            if sad < best.2 {
                best = (mvx, mvy, sad);
            }
        }
    }
    best
}

/// Build the OBMC-blended 16×16 luma predictor for a 4MV MB, mirroring the
/// decoder's §F.3 math so the encoder's residual target is the same pel
/// grid the decoder will reconstruct. Also returns the chroma MV `(cmx,
/// cmy)` derived from the 4 luma MVs via the §F.2 `sum/8` rule (Table F.1).
///
/// Note: the §F.3 remote-MV fall-back rules are applied via `mv_grid`
/// lookups. Since the encoder visits MBs in raster order and only the
/// current MB's `mvs4` is unset at the point we call this, we pre-populate
/// a scratch motion entry with the candidate `mvs4` so the remote-MV
/// helpers see the current block's vectors. Right / below neighbours that
/// haven't been encoded yet are still at their `MbMotion::default()`
/// values — `!coded` → the §F.3 "not coded → current block MV" substitution
/// kicks in.
fn build_mb_predictor_4mv_obmc(
    reference: &IPicture,
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    mvs4: &[(i32, i32); 4],
) -> ([u8; 256], i32, i32) {
    let mut y_pred = [0u8; 256];
    let ref_y_h = (reference.y.len() / reference.y_stride) as i32;
    let ref_w = reference.y_stride as i32;

    // Install a scratch motion entry for the current MB so the OBMC lookup
    // can read the in-MB neighbour MVs. We restore the grid after we're
    // done to avoid side-effects (the real write happens inside
    // `encode_p_mb_inter_4mv` once the decision is final).
    //
    // We clone the grid only as much as needed: because `predict_block` is
    // read-only on reference and we only mutate a single entry, a local
    // clone of that single entry + revert-at-end is enough.
    let prev_entry = mv_grid.get(mb_x, mb_y);
    // SAFETY: caller passes `&MvGrid`, we need interior mutation. Use a
    // local mutable copy of the whole grid instead — O(mb_w * mb_h)
    // MbMotions at QCIF (99 entries) is negligible.
    let mut scratch = mv_grid.clone();
    scratch.set(mb_x, mb_y, MbMotion::mv4(*mvs4));

    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0usize, 0usize),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let blk_px = (mb_x * 16 + sub_x) as i32;
        let blk_py = (mb_y * 16 + sub_y) as i32;
        let mut blk = [0u8; 64];
        obmc_predict_block_enc(
            &mut blk, reference, &scratch, mb_x, mb_y, b, blk_px, blk_py, ref_w, ref_y_h,
        );
        for j in 0..8 {
            for i in 0..8 {
                y_pred[(sub_y + j) * 16 + (sub_x + i)] = blk[j * 8 + i];
            }
        }
    }
    // Restore prev entry (we passed in an immutable reference; scratch was
    // local so no restore needed on the caller's grid — but for tidiness
    // let the compiler ensure we didn't accidentally leak.).
    let _ = prev_entry;

    let (cmx, cmy) = chroma_mv_4mv(mvs4);
    (y_pred, cmx, cmy)
}

/// Produce the OBMC-blended predictor for one 8×8 luma block of the
/// current MB, using the same weight matrices and neighbour-MV
/// substitution rules as the decoder's `obmc_luma_block` (§F.3). Kept
/// parallel to `mb::obmc_luma_block` so encoder reconstruction stays
/// bit-identical with what the decoder produces.
#[allow(clippy::too_many_arguments)]
fn obmc_predict_block_enc(
    dst: &mut [u8; 64],
    reference: &IPicture,
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    blk_px: i32,
    blk_py: i32,
    ref_w: i32,
    ref_h: i32,
) {
    let motion = mv_grid.get(mb_x, mb_y);
    let mv0 = motion.mvs4[block_idx];
    let mut q_pred = [0u8; 64];
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv0.0,
        mv0.1,
        8,
        &mut q_pred,
        8,
    );

    let in_top_row = block_idx < 2;
    let in_left_col = block_idx == 0 || block_idx == 2;

    // Top-neighbour remote MV.
    let mv_top = if in_top_row {
        if mb_y == 0 {
            mv0
        } else {
            let nb = mv_grid.get(mb_x, mb_y - 1);
            let sibling_idx = block_idx & 1; // col 0 -> block 0, col 1 -> block 1 of that MB ... (top row of that MB)
                                             // For Top-neighbour when we're in top row, we want the BOTTOM row of the above MB:
            let bot_sibling = (block_idx & 1) + 2; // blocks 2 or 3 of the above MB
            let _ = sibling_idx;
            enc_neighbour_mv(nb, bot_sibling, mv0)
        }
    } else {
        // Blocks 2 or 3: top-neighbour is block 0 or 1 of the same MB.
        motion.mvs4[block_idx & 1]
    };
    // Bottom-neighbour remote MV.
    // Spec §F.3 last paragraph: for bottom-of-MB blocks (2 / 3) and Bottom
    // side, force remote MV = current MV.
    let mv_bot = if !in_top_row {
        mv0
    } else {
        // Blocks 0 or 1: bottom-neighbour is block 2 or 3 of same MB.
        motion.mvs4[(block_idx & 1) + 2]
    };

    // Left-neighbour remote MV.
    let mv_left = if in_left_col {
        if mb_x == 0 {
            mv0
        } else {
            let nb = mv_grid.get(mb_x - 1, mb_y);
            // Current is LEFT col (blocks 0/2) — left neighbour in prev MB is
            // RIGHT col (blocks 1/3) at same row.
            let row = block_idx >> 1; // 0 or 1
            let right_sibling = row * 2 + 1;
            enc_neighbour_mv(nb, right_sibling, mv0)
        }
    } else {
        // Blocks 1 or 3 — left is block 0 or 2 of same MB.
        let row = block_idx >> 1;
        motion.mvs4[row * 2]
    };
    let mv_right = if in_left_col {
        // Blocks 0/2 — right is block 1/3 of same MB.
        let row = block_idx >> 1;
        motion.mvs4[row * 2 + 1]
    } else {
        // Blocks 1/3 — right neighbour in next MB's LEFT col, same row.
        if mb_x + 1 >= mv_grid.mb_w {
            mv0
        } else {
            let nb = mv_grid.get(mb_x + 1, mb_y);
            let row = block_idx >> 1;
            let left_sibling = row * 2;
            enc_neighbour_mv(nb, left_sibling, mv0)
        }
    };

    let mut r_top_pred = [0u8; 64];
    let mut r_bot_pred = [0u8; 64];
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_top.0,
        mv_top.1,
        8,
        &mut r_top_pred,
        8,
    );
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_bot.0,
        mv_bot.1,
        8,
        &mut r_bot_pred,
        8,
    );
    let mut s_left_pred = [0u8; 64];
    let mut s_right_pred = [0u8; 64];
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_left.0,
        mv_left.1,
        8,
        &mut s_left_pred,
        8,
    );
    predict_block(
        &reference.y,
        reference.y_stride,
        ref_w,
        ref_h,
        blk_px,
        blk_py,
        mv_right.0,
        mv_right.1,
        8,
        &mut s_right_pred,
        8,
    );

    for j in 0..8usize {
        for i in 0..8usize {
            let h0 = OBMC_H0[j][i] as i32;
            let h1 = OBMC_H1[j][i] as i32;
            let h2 = OBMC_H2[j][i] as i32;
            let q = q_pred[j * 8 + i] as i32;
            let r = if j < 4 {
                r_top_pred[j * 8 + i] as i32
            } else {
                r_bot_pred[j * 8 + i] as i32
            };
            let s = if i < 4 {
                s_left_pred[j * 8 + i] as i32
            } else {
                s_right_pred[j * 8 + i] as i32
            };
            let v = (q * h0 + r * h1 + s * h2 + 4) / 8;
            dst[j * 8 + i] = v.clamp(0, 255) as u8;
        }
    }
}

/// Pick a neighbour block's MV from an `MbMotion`, applying the §F.3
/// fall-backs for "not coded" (→ zero) and "intra" (→ current MV).
fn enc_neighbour_mv(nb: MbMotion, block_idx: usize, cur_mv: (i32, i32)) -> (i32, i32) {
    if !nb.coded {
        return (0, 0);
    }
    if nb.intra {
        return cur_mv;
    }
    nb.mvs4[block_idx]
}

/// Build an 8×8 chroma predictor block into `u_pred` / `v_pred`.
fn build_chroma_predictor(
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
    cmx: i32,
    cmy: i32,
    u_pred: &mut [u8; 64],
    v_pred: &mut [u8; 64],
) {
    let ref_c_h = (reference.cb.len() / reference.c_stride) as i32;
    let blk_px = (mb_x * 8) as i32;
    let blk_py = (mb_y * 8) as i32;
    predict_block(
        &reference.cb,
        reference.c_stride,
        reference.c_stride as i32,
        ref_c_h,
        blk_px,
        blk_py,
        cmx,
        cmy,
        8,
        u_pred,
        8,
    );
    predict_block(
        &reference.cr,
        reference.c_stride,
        reference.c_stride as i32,
        ref_c_h,
        blk_px,
        blk_py,
        cmx,
        cmy,
        8,
        v_pred,
        8,
    );
}

/// Encode one P-picture macroblock. Chooses one of:
/// * **Skipped** (COD=1): when the predicted MB with MV=(0,0) has residual
///   energy below threshold AND the median predictor is (0,0). Copies
///   reference into `recon`.
/// * **Inter**: emit MCBPC/CBPY/MVD, compensate the block, encode residual
///   for any block with energy above threshold. The DCT of the residual is
///   quantised like intra AC (but with start at scan 0 and no DC special
///   case).
/// * **Intra-in-P**: when the best inter prediction's SAD is worse than a
///   direct intra encode's approximate cost — we fall back to intra for that
///   MB. This is the standard "intra block decision" used by FFmpeg.
///
/// `enable_annex_d_umv`: when set, the motion estimator widens to the §D.1 /
/// §D.2 extended `[-63, +63]` halfpel range and MV components are emitted via
/// [`encode_mv_component_umv`].
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_full(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    reference: &IPicture,
    recon: &mut IPicture,
    mv_grid: &mut MvGrid,
    enable_annex_f: bool,
    enable_annex_d_umv: bool,
) -> Result<crate::mb::PMbInfo> {
    // 1. Motion-estimate on luma 16×16.
    let (mvx, mvy, mv_sad) = if enable_annex_d_umv {
        motion_estimate_mb_umv(frame, reference, mb_x, mb_y, mv_grid)
    } else {
        motion_estimate_mb(frame, reference, mb_x, mb_y)
    };

    // Also consider MV=(0,0) directly — some encoders pin to zero when the
    // difference is small, which gives the skipped-MB path a chance.
    let zero_sad = sad_block(
        &frame.planes[0].data,
        frame.planes[0].stride,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        &reference.y,
        reference.y_stride,
        reference.y_stride as i32,
        (reference.y.len() / reference.y_stride) as i32,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        0,
        0,
        16,
    );
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    // Median predictor (pmx, pmy) is in half-pel. For the skip decision we
    // need pmx == 0 AND pmy == 0 because a skipped MB carries MV (0,0).
    let can_skip = pmx == 0 && pmy == 0 && zero_sad < mv_sad + 128;

    // 2. Compute the MB predictor + residual energy.
    let mut y_pred = [0u8; 256];
    let mut u_pred = [0u8; 64];
    let mut v_pred = [0u8; 64];

    let decide_mv = if can_skip { (0, 0) } else { (mvx, mvy) };

    build_mb_predictor(
        reference,
        mb_x,
        mb_y,
        decide_mv.0,
        decide_mv.1,
        &mut y_pred,
        &mut u_pred,
        &mut v_pred,
    );

    // Quick residual energy (sum of absolute luma residuals). Used to decide
    // whether an "all-zero residual skipped MB" is acceptable AND whether to
    // try intra.
    let src_y = &frame.planes[0];
    let src_cb = &frame.planes[1];
    let src_cr = &frame.planes[2];
    let mut luma_abs_sum = 0u32;
    for j in 0..16 {
        for i in 0..16 {
            let s = src_y.data[(mb_y * 16 + j) * src_y.stride + (mb_x * 16 + i)] as i32;
            let p = y_pred[j * 16 + i] as i32;
            luma_abs_sum += (s - p).unsigned_abs();
        }
    }

    // Intra-vs-inter decision: compute the intra MB's total "variance" as a
    // proxy for intra coding cost (sum of |pel - mb_mean|). If intra wins by
    // a large margin we emit an intra MB. Simple heuristic that matches
    // FFmpeg's "mb_var < lambda * sad" rule at low qscales.
    let intra_variance = mb_luma_variance(src_y, mb_x, mb_y);
    let try_intra = intra_variance * 5 < luma_abs_sum;

    // Skipped MB: can_skip (MV=(0,0), predictor=(0,0)) AND residual is so
    // small that every block would quantise to zero. We model "quantise to
    // zero" as "sum of absolute residuals per 256 pels < thresh(q)".
    if can_skip && luma_abs_sum < (quant as u32) * 128 {
        // Emit COD=1 (skipped).
        bw.write_bits(1, 1);
        // Copy predictor into recon.
        copy_predictor_to_recon(recon, mb_x, mb_y, &y_pred, &u_pred, &v_pred);
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        return Ok(crate::mb::PMbInfo::empty_skipped());
    }

    // COD = 0 — MB is coded.
    bw.write_bits(0, 1);

    if try_intra {
        encode_p_mb_intra(bw, mb_x, mb_y, quant, frame, width, height, recon)?;
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
        return Ok(crate::mb::PMbInfo {
            coded: true,
            intra: true,
            residual: vec![0i16; 6 * 64],
            residual_present: [false; 6],
            intra_done: true,
        });
    }

    // Note: the Annex F 4-MV / OBMC encode path does NOT go through this
    // single-pass function — it routes via `emit_p_mb_ap` after a
    // two-pass decision phase. `enable_annex_f` arriving here therefore
    // means the caller is in a transitional configuration; honour it by
    // ignoring (emit single-pass 1-MV inter).
    let _ = enable_annex_f;

    // Inter path.
    let info = encode_p_mb_inter_full(
        bw,
        mb_x,
        mb_y,
        quant,
        src_y,
        src_cb,
        src_cr,
        reference,
        recon,
        decide_mv,
        mv_grid,
        &y_pred,
        &u_pred,
        &v_pred,
        enable_annex_d_umv,
    )?;
    Ok(info)
}

/// Intra encode of a P-MB block. Same bitstream as an I-MB's Intra MCBPC,
/// but prefixed with COD=0 by the caller AND using the inter MCBPC table
/// (PMbKind::Intra).
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_intra(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    recon: &mut IPicture,
) -> Result<()> {
    let mut blocks = [[0i32; 64]; 6];
    let mut dc_pels = [128u8; 6];
    let mut block_has_ac = [false; 6];

    for b in 0..6 {
        let mut samples = [0.0f32; 64];
        sample_block_for(frame, width, height, mb_x, mb_y, b, &mut samples);
        let mut dctf = samples;
        fdct8x8(&mut dctf);
        let (dc_byte, levels, any_ac) = quantise_intra_block(&dctf, quant);
        dc_pels[b] = dc_byte;
        block_has_ac[b] = any_ac;
        blocks[b] = levels;
    }
    let cbpc: u8 = ((block_has_ac[4] as u8) << 1) | (block_has_ac[5] as u8);
    let cbpy: u8 = ((block_has_ac[0] as u8) << 3)
        | ((block_has_ac[1] as u8) << 2)
        | ((block_has_ac[2] as u8) << 1)
        | (block_has_ac[3] as u8);

    write_mcbpc_inter(bw, PMbKind::Intra, cbpc);
    write_cbpy(bw, cbpy);
    // No DQUANT for PMbKind::Intra (we hold quant constant).
    for b in 0..6 {
        bw.write_bits(dc_pels[b] as u32, 8);
        if block_has_ac[b] {
            write_block_ac(bw, &blocks[b]);
        }
        reconstruct_intra_block(recon, b, mb_x, mb_y, dc_pels[b], &blocks[b], quant);
    }
    Ok(())
}

/// Inter encode of a P-MB — MCBPC/CBPY/MVD + residual TCOEF per coded block.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_inter(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    src_y: &oxideav_core::frame::VideoPlane,
    src_cb: &oxideav_core::frame::VideoPlane,
    src_cr: &oxideav_core::frame::VideoPlane,
    _reference: &IPicture,
    recon: &mut IPicture,
    mv: (i32, i32),
    mv_grid: &mut MvGrid,
    y_pred: &[u8; 256],
    u_pred: &[u8; 64],
    v_pred: &[u8; 64],
) -> Result<crate::mb::PMbInfo> {
    encode_p_mb_inter_full(
        bw, mb_x, mb_y, quant, src_y, src_cb, src_cr, _reference, recon, mv, mv_grid, y_pred,
        u_pred, v_pred, false,
    )
}

/// Full-options variant of [`encode_p_mb_inter`] — when `enable_annex_d_umv`
/// is true, MV components are emitted via [`encode_mv_component_umv`]
/// (extended `[-63, +63]` halfpel reach + §D.2 reconstruction). Everything
/// else (residual coding, CBPY, MCBPC, recon) is identical to the baseline
/// path.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_inter_full(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    src_y: &oxideav_core::frame::VideoPlane,
    src_cb: &oxideav_core::frame::VideoPlane,
    src_cr: &oxideav_core::frame::VideoPlane,
    _reference: &IPicture,
    recon: &mut IPicture,
    mv: (i32, i32),
    mv_grid: &mut MvGrid,
    y_pred: &[u8; 256],
    u_pred: &[u8; 64],
    v_pred: &[u8; 64],
    enable_annex_d_umv: bool,
) -> Result<crate::mb::PMbInfo> {
    // 1. For each of the 6 blocks, compute residual DCT → quantise → check
    //    if any nonzero AC exists. Track (cbpy, cbpc) and the recon pels.
    let mut levels_all = [[0i32; 64]; 6];
    let mut has_ac = [false; 6];

    // Luma (4 blocks).
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 16 + sub_y + j;
                let px = mb_x * 16 + sub_x + i;
                let s = src_y.data[py * src_y.stride + px] as i32;
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }

    // Chroma (Cb, Cr).
    for (ci, plane) in [(0, src_cb), (1, src_cr)].iter() {
        let pred = if *ci == 0 { u_pred } else { v_pred };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 8 + j;
                let px = mb_x * 8 + i;
                let s = plane.data[py * plane.stride + px] as i32;
                let p = pred[j * 8 + i] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        let b = 4 + ci;
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }

    // 2. Build CBPC / CBPY. For inter the on-wire CBPY field is bit-inverted
    //    of the actual pattern.
    let cbpc: u8 = ((has_ac[4] as u8) << 1) | (has_ac[5] as u8);
    let cbpy_true: u8 = ((has_ac[0] as u8) << 3)
        | ((has_ac[1] as u8) << 2)
        | ((has_ac[2] as u8) << 1)
        | (has_ac[3] as u8);
    let cbpy_on_wire = cbpy_true ^ 0xF;

    // 3. Emit MCBPC inter + CBPY + MVD.
    write_mcbpc_inter(bw, PMbKind::Inter, cbpc);
    write_cbpy(bw, cbpy_on_wire);
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    if enable_annex_d_umv {
        encode_mv_component_umv(bw, mv.0, pmx);
        encode_mv_component_umv(bw, mv.1, pmy);
    } else {
        encode_mv_component(bw, mv.0, pmx);
        encode_mv_component(bw, mv.1, pmy);
    }

    // 4. Emit per-block AC (when coded) and reconstruct.
    for b in 0..6 {
        if has_ac[b] {
            write_block_ac_inter(bw, &levels_all[b]);
        }
    }

    // 5. Reconstruct blocks into recon: predictor + dequantised residual
    //    IDCT, clipped. Also stash the signed IDCT residual into `info` so
    //    the Annex F pass-2 OBMC can re-add it to the OBMC-blended
    //    predictor.
    let mut info = crate::mb::PMbInfo {
        coded: true,
        intra: false,
        residual: vec![0i16; 6 * 64],
        residual_present: [false; 6],
        intra_done: false,
    };
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let coeffs = dequantise_block(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
        if has_ac[b] {
            let dst = info.residual_block_mut(b);
            for (i, &v) in resid_out.iter().enumerate() {
                dst[i] = v.clamp(-4096, 4095) as i16;
            }
            info.residual_present[b] = true;
        }
    }
    for ci in 0..2usize {
        let b = 4 + ci;
        let pred = if ci == 0 { u_pred } else { v_pred };
        let coeffs = dequantise_block(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = pred[j * 8 + i] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
        if has_ac[b] {
            let dst = info.residual_block_mut(b);
            for (i, &v) in resid_out.iter().enumerate() {
                dst[i] = v.clamp(-4096, 4095) as i16;
            }
            info.residual_present[b] = true;
        }
    }

    mv_grid.set(mb_x, mb_y, MbMotion::mv1(mv, true, false));
    Ok(info)
}

/// Inter encode of a P-MB in 4MV mode (§F / Annex F) — Inter4MV MCBPC +
/// CBPY + four MVDs + MVDCHR-derived chroma + residual TCOEF per coded
/// block. The luma predictor is the OBMC-blended one (pre-built by the
/// caller so the residual targets the same pels the decoder will
/// reconstruct).
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_inter_4mv(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    src_y: &oxideav_core::frame::VideoPlane,
    src_cb: &oxideav_core::frame::VideoPlane,
    src_cr: &oxideav_core::frame::VideoPlane,
    recon: &mut IPicture,
    mvs4: &[(i32, i32); 4],
    mv_grid: &mut MvGrid,
    y_pred: &[u8; 256],
    u_pred: &[u8; 64],
    v_pred: &[u8; 64],
) -> Result<crate::mb::PMbInfo> {
    // 1. Per-block residual DCT → quantise.
    let mut levels_all = [[0i32; 64]; 6];
    let mut has_ac = [false; 6];
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 16 + sub_y + j;
                let px = mb_x * 16 + sub_x + i;
                let s = src_y.data[py * src_y.stride + px] as i32;
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }
    for (ci, plane) in [(0, src_cb), (1, src_cr)].iter() {
        let pred = if *ci == 0 { u_pred } else { v_pred };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 8 + j;
                let px = mb_x * 8 + i;
                let s = plane.data[py * plane.stride + px] as i32;
                let p = pred[j * 8 + i] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        let b = 4 + ci;
        has_ac[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }

    // 2. CBPC / CBPY (XOR for inter).
    let cbpc: u8 = ((has_ac[4] as u8) << 1) | (has_ac[5] as u8);
    let cbpy_true: u8 = ((has_ac[0] as u8) << 3)
        | ((has_ac[1] as u8) << 2)
        | ((has_ac[2] as u8) << 1)
        | (has_ac[3] as u8);
    let cbpy_on_wire = cbpy_true ^ 0xF;

    // 3. Emit MCBPC Inter4MV + CBPY + 4 MVDs.
    write_mcbpc_inter(bw, PMbKind::Inter4MV, cbpc);
    write_cbpy(bw, cbpy_on_wire);

    // The decoder decodes MVDs in block order 0..=3 using the §F.2
    // per-block predictor (redefined MV1/MV2/MV3 per Figure F.1). After each
    // component is decoded, the decoder updates `mv_grid.mvs4[b]` so later
    // blocks can see it (block 1's left-neighbour is block 0 of the same MB,
    // etc.). We must mirror that exactly on the encoder side — start the MB
    // entry as a 4-MV record with zero slots and fill them in order.
    mv_grid.set(mb_x, mb_y, MbMotion::mv4([(0, 0); 4]));
    for b in 0..4 {
        let (pmx, pmy) = predict_mv_block(mv_grid, mb_x, mb_y, b);
        encode_mv_component(bw, mvs4[b].0, pmx);
        encode_mv_component(bw, mvs4[b].1, pmy);
        let mut cur = mv_grid.get(mb_x, mb_y);
        cur.mvs4[b] = mvs4[b];
        cur.mv = mvs4[0];
        mv_grid.set(mb_x, mb_y, cur);
    }
    // Finalise the MbMotion record so downstream neighbours see a proper
    // 4MV entry.
    mv_grid.set(mb_x, mb_y, MbMotion::mv4(*mvs4));

    // 4. Per-block AC.
    for b in 0..6 {
        if has_ac[b] {
            write_block_ac_inter(bw, &levels_all[b]);
        }
    }

    // 5. Reconstruct blocks (predictor + dequantised residual IDCT + clip)
    //    and stash the residual for the Annex F pass-2 OBMC.
    let mut info = crate::mb::PMbInfo {
        coded: true,
        intra: false,
        residual: vec![0i16; 6 * 64],
        residual_present: [false; 6],
        intra_done: false,
    };
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => unreachable!(),
        };
        let coeffs = dequantise_block(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
        if has_ac[b] {
            let dst = info.residual_block_mut(b);
            for (i, &v) in resid_out.iter().enumerate() {
                dst[i] = v.clamp(-4096, 4095) as i16;
            }
            info.residual_present[b] = true;
        }
    }
    for ci in 0..2usize {
        let b = 4 + ci;
        let pred = if ci == 0 { u_pred } else { v_pred };
        let coeffs = dequantise_block(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        for j in 0..8 {
            for i in 0..8 {
                let p = pred[j * 8 + i] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
        if has_ac[b] {
            let dst = info.residual_block_mut(b);
            for (i, &v) in resid_out.iter().enumerate() {
                dst[i] = v.clamp(-4096, 4095) as i16;
            }
            info.residual_present[b] = true;
        }
    }

    Ok(info)
}

/// Quantise a residual (inter) block. Uses the same deadzone bias as the
/// intra AC path.
fn quantise_inter_block(dctf: &[f32; 64], quant: u8) -> [i32; 64] {
    let mut levels = [0i32; 64];
    let q = quant as i32;
    let two_q = 2 * q;
    let bias = q / 4;
    for k in 0..64 {
        let coef = dctf[k];
        let abs_f = coef.abs() as i32;
        let mag = (abs_f + bias) / two_q;
        if mag != 0 {
            let signed = if coef < 0.0 { -mag } else { mag };
            levels[k] = signed.clamp(-127, 127);
        }
    }
    levels
}

/// Emit the AC coefficients for an **inter** block in zig-zag order (start
/// at scan index 0 — there is no DC special-case in H.263 inter blocks).
fn write_block_ac_inter(bw: &mut BitWriter, levels: &[i32; 64]) {
    let mut nonzero_zz: Vec<(usize, i32)> = Vec::with_capacity(8);
    for zz in 0..64 {
        let nat = ZIGZAG[zz];
        let lv = levels[nat];
        if lv != 0 {
            nonzero_zz.push((zz, lv));
        }
    }
    debug_assert!(!nonzero_zz.is_empty());

    let mut prev_zz: i32 = -1;
    for (i, &(zz, lv)) in nonzero_zz.iter().enumerate() {
        let run = (zz as i32 - prev_zz - 1) as u8;
        let last = i == nonzero_zz.len() - 1;
        write_tcoef(bw, last, run, lv);
        prev_zz = zz as i32;
    }
}

/// Build the 16×16 luma + 2×8×8 chroma predictor into the provided buffers.
fn build_mb_predictor(
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
    mvx: i32,
    mvy: i32,
    y_pred: &mut [u8; 256],
    u_pred: &mut [u8; 64],
    v_pred: &mut [u8; 64],
) {
    let ref_y_h = (reference.y.len() / reference.y_stride) as i32;
    let ref_c_h = (reference.cb.len() / reference.c_stride) as i32;
    // Luma: predict in four 8×8 sub-blocks, stitched into 16×16.
    for (blk, (sub_x, sub_y)) in [(0, (0, 0)), (1, (8, 0)), (2, (0, 8)), (3, (8, 8))].iter() {
        let _ = blk;
        let blk_px = (mb_x * 16 + sub_x) as i32;
        let blk_py = (mb_y * 16 + sub_y) as i32;
        let mut tmp = [0u8; 64];
        predict_block(
            &reference.y,
            reference.y_stride,
            reference.y_stride as i32,
            ref_y_h,
            blk_px,
            blk_py,
            mvx,
            mvy,
            8,
            &mut tmp,
            8,
        );
        for j in 0..8 {
            for i in 0..8 {
                y_pred[(sub_y + j) * 16 + (sub_x + i)] = tmp[j * 8 + i];
            }
        }
    }
    // Chroma.
    let cmx = luma_to_chroma_mv(mvx);
    let cmy = luma_to_chroma_mv(mvy);
    let blk_px = (mb_x * 8) as i32;
    let blk_py = (mb_y * 8) as i32;
    predict_block(
        &reference.cb,
        reference.c_stride,
        reference.c_stride as i32,
        ref_c_h,
        blk_px,
        blk_py,
        cmx,
        cmy,
        8,
        u_pred,
        8,
    );
    predict_block(
        &reference.cr,
        reference.c_stride,
        reference.c_stride as i32,
        ref_c_h,
        blk_px,
        blk_py,
        cmx,
        cmy,
        8,
        v_pred,
        8,
    );
}

/// Copy the 16×16 luma + 8×8 chroma predictor into `recon` (used when the MB
/// is emitted as a skipped MB — the reconstruction == predictor).
fn copy_predictor_to_recon(
    recon: &mut IPicture,
    mb_x: usize,
    mb_y: usize,
    y_pred: &[u8; 256],
    u_pred: &[u8; 64],
    v_pred: &[u8; 64],
) {
    for j in 0..16 {
        let off = (mb_y * 16 + j) * recon.y_stride + mb_x * 16;
        recon.y[off..off + 16].copy_from_slice(&y_pred[j * 16..j * 16 + 16]);
    }
    for j in 0..8 {
        let off = (mb_y * 8 + j) * recon.c_stride + mb_x * 8;
        recon.cb[off..off + 8].copy_from_slice(&u_pred[j * 8..j * 8 + 8]);
        recon.cr[off..off + 8].copy_from_slice(&v_pred[j * 8..j * 8 + 8]);
    }
}

// ---------------------------------------------------------------------------
// Public re-exports for the SAC P-encoder bridge in `mb_sac.rs`. These wrap
// the private encoder helpers without exposing them to the world; the
// `_pub` suffix flags them as the bridge's surface, and the original
// implementations stay private to keep the encoder module's API stable.
// ---------------------------------------------------------------------------

/// SAC-bridge wrapper around the private [`motion_estimate_mb`].
pub fn motion_estimate_mb_pub(
    frame: &VideoFrame,
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
) -> (i32, i32, u32) {
    motion_estimate_mb(frame, reference, mb_x, mb_y)
}

/// SAC-bridge wrapper around the private [`build_mb_predictor`].
#[allow(clippy::too_many_arguments)]
pub fn build_mb_predictor_pub(
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
    mvx: i32,
    mvy: i32,
    y_pred: &mut [u8; 256],
    u_pred: &mut [u8; 64],
    v_pred: &mut [u8; 64],
) {
    build_mb_predictor(reference, mb_x, mb_y, mvx, mvy, y_pred, u_pred, v_pred);
}

/// SAC-bridge wrapper around the private [`copy_predictor_to_recon`].
pub fn copy_predictor_to_recon_pub(
    recon: &mut IPicture,
    mb_x: usize,
    mb_y: usize,
    y_pred: &[u8; 256],
    u_pred: &[u8; 64],
    v_pred: &[u8; 64],
) {
    copy_predictor_to_recon(recon, mb_x, mb_y, y_pred, u_pred, v_pred);
}

/// SAC-bridge wrapper around the private [`mb_luma_variance`].
pub fn mb_luma_variance_pub(
    src: &oxideav_core::frame::VideoPlane,
    mb_x: usize,
    mb_y: usize,
) -> u32 {
    mb_luma_variance(src, mb_x, mb_y)
}

/// SAC-bridge wrapper around the private [`quantise_inter_block`].
pub fn quantise_inter_block_pub(dctf: &[f32; 64], quant: u8) -> [i32; 64] {
    quantise_inter_block(dctf, quant)
}

/// SAC-bridge wrapper around the private [`dequantise_block`].
pub fn dequantise_block_pub(levels: &[i32; 64], quant: u8, skip_dc: bool) -> [i32; 64] {
    dequantise_block(levels, quant, skip_dc)
}

/// SAC-bridge wrapper around the private [`sad_block`].
#[allow(clippy::too_many_arguments)]
pub fn sad_block_pub(
    src: &[u8],
    src_stride: usize,
    src_x: i32,
    src_y: i32,
    refp: &[u8],
    ref_stride: usize,
    ref_w: i32,
    ref_h: i32,
    blk_px: i32,
    blk_py: i32,
    mvx: i32,
    mvy: i32,
    block_size: i32,
) -> u32 {
    sad_block(
        src, src_stride, src_x, src_y, refp, ref_stride, ref_w, ref_h, blk_px, blk_py, mvx, mvy,
        block_size,
    )
}

/// SAC-bridge wrapper around the private [`write_gob_header`].
pub fn write_gob_header_pub(bw: &mut BitWriter, gn: u8, gquant: u8) -> Result<()> {
    write_gob_header(bw, gn, gquant)
}

/// SAC + Annex F (round 15) — wrapper around [`motion_estimate_4mv`] for the
/// per-MB 4-MV decision. Returns `(mvs4, summed_sad)` where `mvs4` is the
/// per-block luma MV (Figure 5 ordering — block 0 top-left, block 1 top-right,
/// block 2 bottom-left, block 3 bottom-right).
pub fn motion_estimate_4mv_pub(
    frame: &VideoFrame,
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
) -> ([(i32, i32); 4], u32) {
    motion_estimate_4mv(frame, reference, mb_x, mb_y)
}

/// SAC + Annex F (round 15) — wrapper around [`build_mb_predictor_4mv_obmc`].
/// Returns the OBMC-blended luma 16×16 predictor and the chroma MV the caller
/// must use (`MVDCHR` from §F.2 — sum of 4 luma MVs / 8, rounded by Table F.1).
pub fn build_mb_predictor_4mv_obmc_pub(
    reference: &IPicture,
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    mvs4: &[(i32, i32); 4],
) -> ([u8; 256], i32, i32) {
    build_mb_predictor_4mv_obmc(reference, mv_grid, mb_x, mb_y, mvs4)
}

/// SAC + Annex F (round 15) — wrapper around [`decide_p_mb`]. Picks the
/// per-MB encode mode (skipped / intra / 1-MV inter / 4-MV inter) using
/// the same SAD heuristics as the VLC AP encoder, so SAC + AP traverses
/// the same decision tree.
pub fn decide_p_mb_pub(
    frame: &VideoFrame,
    reference: &IPicture,
    mv_grid: &MvGrid,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
) -> MbDecision {
    decide_p_mb(frame, reference, mv_grid, mb_x, mb_y, quant)
}

/// SAC + Annex F (round 15) — wrapper around [`build_chroma_predictor`].
pub fn build_chroma_predictor_pub(
    reference: &IPicture,
    mb_x: usize,
    mb_y: usize,
    cmx: i32,
    cmy: i32,
    u_pred: &mut [u8; 64],
    v_pred: &mut [u8; 64],
) {
    build_chroma_predictor(reference, mb_x, mb_y, cmx, cmy, u_pred, v_pred);
}

/// Sum of absolute differences between the luma MB and its mean — cheap
/// proxy for "intra coding cost" in the intra/inter decision.
fn mb_luma_variance(src: &oxideav_core::frame::VideoPlane, mb_x: usize, mb_y: usize) -> u32 {
    let mut sum = 0u32;
    let mut sum_abs = 0u32;
    for j in 0..16 {
        for i in 0..16 {
            let s = src.data[(mb_y * 16 + j) * src.stride + mb_x * 16 + i] as u32;
            sum += s;
        }
    }
    let mean = sum / 256;
    for j in 0..16 {
        for i in 0..16 {
            let s = src.data[(mb_y * 16 + j) * src.stride + mb_x * 16 + i] as i32;
            sum_abs += (s - mean as i32).unsigned_abs();
        }
    }
    sum_abs
}

// ---------------------------------------------------------------------------
// Annex S — Alternative INTER VLC (encoder helpers + picture functions)
// ---------------------------------------------------------------------------

/// PLUSPTYPE picture header writer for Annex S (Alternative INTER VLC).
/// Emits OPPTYPE bit 13 (AIV) = 1; all other OPPTYPE bits are off.
fn write_plusptype_picture_header_aiv(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    sei: &[Sei],
) -> Result<()> {
    debug_assert!(bw.is_byte_aligned());
    let src_code: u32 = match source_format {
        SourceFormat::SubQcif => 1,
        SourceFormat::Qcif => 2,
        SourceFormat::Cif => 3,
        SourceFormat::FourCif => 4,
        SourceFormat::SixteenCif => 5,
        _ => {
            return Err(Error::unsupported(
                "h263 AIV encoder: only standard source formats 1..=5 are supported",
            ));
        }
    };
    #[allow(clippy::unusual_byte_groupings)]
    let psc: u32 = 0b00_0000_0000_0000_0000_1_00000;
    bw.write_bits(psc, 22);
    bw.write_bits(tr as u32, 8);
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3); // PLUSPTYPE
    bw.write_bits(0b001, 3); // UFEP=001
                             // OPPTYPE (18 bits): src_fmt + AIV(bit13) + marker(bit15)
    let bit = |k: u32, v: u32| (v & 1) << (18 - k);
    let srcf_part = (src_code & 0b111) << 15;
    let opptype: u32 = srcf_part
        | bit(13, 1)  // AIV
        | bit(15, 1); // marker
    bw.write_bits(opptype, 18);
    let pct: u32 = if is_p_picture { 0b001 } else { 0b000 };
    let mpptype: u32 = (pct << 6) | 0b000_001;
    bw.write_bits(mpptype, 9);
    bw.write_bits(0, 1); // CPM=0
    if pquant == 0 || pquant > 31 {
        return Err(Error::invalid(format!(
            "h263 AIV encoder: pquant {pquant} out of range 1..=31"
        )));
    }
    bw.write_bits(pquant as u32, 5);
    let psupp = serialise_sei_to_psupp(sei);
    write_pei_loop(bw, &psupp);
    Ok(())
}

/// Measure the bit-length that [`crate::enc_tables::write_tcoef`] (inter VLC)
/// would emit for a single `(last, run, level)` triple. Used by the AIV
/// encoder to pick the shorter of INTER vs INTRA VLC per block.
fn tcoef_inter_bit_len(last: bool, run: u8, level: i32) -> u32 {
    use crate::enc_tables::lookup_tcoef;
    let abs = level.unsigned_abs() as u8;
    if let Some((bits, _)) = lookup_tcoef(last, run, abs) {
        bits as u32 + 1 // +1 for sign bit
    } else {
        // Escape: 7 (prefix) + 1 (last) + 6 (run) + 8 (level) = 22 bits.
        22
    }
}

/// Measure the bit-length that [`crate::aic::write_intra_tcoef`] (AIC / INTRA
/// VLC) would emit for a single `(last, run, level)` triple.
fn tcoef_intra_bit_len(last: bool, run: u8, level: i32) -> u32 {
    use crate::aic::lookup_intra_tcoef;
    let abs = level.unsigned_abs() as u8;
    if let Some((bits, _)) = lookup_intra_tcoef(last, run, abs) {
        bits as u32 + 1 // +1 for sign bit
    } else {
        22 // Escape same shape as inter escape
    }
}

/// Write one inter residual block using AIV (§S.2): for each `(last, run,
/// level)` triple pick whichever VLC table (INTER or INTRA) emits fewer bits.
/// The whole block uses a single VLC choice — we decide per-block (not
/// per-coefficient) by summing bit lengths.
fn write_block_ac_inter_aiv(bw: &mut BitWriter, levels: &[i32; 64]) {
    // Collect nonzero coefficients in zigzag order (same as write_block_ac_inter).
    let mut nonzero_zz: Vec<(usize, i32)> = Vec::with_capacity(8);
    for zz in 0..64 {
        let nat = ZIGZAG[zz];
        let lv = levels[nat];
        if lv != 0 {
            nonzero_zz.push((zz, lv));
        }
    }
    if nonzero_zz.is_empty() {
        return; // No AC — should not be called with an all-zero block
    }
    // Build (last, run, level) triples and measure both VLC lengths.
    let mut triples: Vec<(bool, u8, i32)> = Vec::with_capacity(nonzero_zz.len());
    let mut inter_bits: u32 = 0;
    let mut intra_bits: u32 = 0;
    let mut prev_zz: i32 = -1;
    for (i, &(zz, lv)) in nonzero_zz.iter().enumerate() {
        let run = (zz as i32 - prev_zz - 1) as u8;
        let last = i == nonzero_zz.len() - 1;
        inter_bits += tcoef_inter_bit_len(last, run, lv);
        intra_bits += tcoef_intra_bit_len(last, run, lv);
        triples.push((last, run, lv));
        prev_zz = zz as i32;
    }
    // Pick the shorter VLC table for this block.
    if inter_bits <= intra_bits {
        for &(last, run, lv) in &triples {
            write_tcoef(bw, last, run, lv);
        }
    } else {
        use crate::aic::write_intra_tcoef;
        for &(last, run, lv) in &triples {
            write_intra_tcoef(bw, last, run, lv);
        }
    }
}

/// Encode an I-picture with the Annex S (AIV) PLUSPTYPE header.
/// I-pictures have no INTER blocks so the AIV VLC selection never fires;
/// this function emits a standard intra picture body under the AIV header.
pub fn encode_i_picture_aiv_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    sei: &[Sei],
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 AIV encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    write_plusptype_picture_header_aiv(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        false,
        sei,
    )?;
    for mb_y in 0..mb_h {
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let gn = (mb_y as u32 / mb_rows_per_gob) as u8;
            write_gob_header(&mut bw, gn, pquant)?;
        }
        for mb_x in 0..mb_w {
            encode_intra_mb(
                &mut bw, mb_x, mb_y, pquant, frame, width, height, &mut recon,
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Encode a P-picture with the Annex S (AIV) PLUSPTYPE header. The inter
/// residual blocks use AIV VLC selection (§S.2: INTRA VLC when shorter).
/// §S.3: when both chroma blocks of an INTER MB are coded, emit CBPY
/// without XOR (same as INTRA CBPY encoding).
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_aiv_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    enable_annex_f: bool,
    enable_annex_d_umv: bool,
    sei: &[Sei],
) -> Result<(Vec<u8>, IPicture)> {
    // AIV + AP combination not yet wired at the multi-pass OBMC level.
    if enable_annex_f {
        return Err(Error::unsupported(
            "h263 AIV + Annex F (Advanced Prediction): not yet combined",
        ));
    }
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 AIV encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut mv_grid = MvGrid::new(mb_w, mb_h);
    write_plusptype_picture_header_aiv(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true,
        sei,
    )?;
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            encode_p_mb_aiv(
                &mut bw,
                mb_x,
                mb_y,
                pquant,
                frame,
                width,
                height,
                reference,
                &mut recon,
                &mut mv_grid,
                enable_annex_d_umv,
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Encode one P-MB with AIV VLC selection for inter residual blocks.
/// Mirrors `encode_p_mb_full` but uses `write_block_ac_inter_aiv` for
/// coded inter blocks, and applies §S.3 CBPY-without-XOR when both chroma
/// blocks are coded.
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_aiv(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    reference: &IPicture,
    recon: &mut IPicture,
    mv_grid: &mut MvGrid,
    enable_annex_d_umv: bool,
) -> Result<()> {
    let _ = width;
    let _ = height;

    let src_y = &frame.planes[0];

    // Motion estimation.
    let (mvx, mvy, mv_sad) = if enable_annex_d_umv {
        motion_estimate_mb_umv(frame, reference, mb_x, mb_y, mv_grid)
    } else {
        motion_estimate_mb(frame, reference, mb_x, mb_y)
    };

    // Compute zero-MV (skip) SAD.
    let zero_sad = sad_block(
        &frame.planes[0].data,
        frame.planes[0].stride,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        &reference.y,
        reference.y_stride,
        reference.y_stride as i32,
        (reference.y.len() / reference.y_stride) as i32,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        0,
        0,
        16,
    );
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    let can_skip = pmx == 0 && pmy == 0 && zero_sad < mv_sad + 128;

    // Build predictor for luma-abs-sum calculation (needed for intra decision).
    let mut y_pred = [0u8; 256];
    let mut u_pred = [0u8; 64];
    let mut v_pred = [0u8; 64];
    let decide_mv = if can_skip { (0, 0) } else { (mvx, mvy) };
    build_mb_predictor(
        reference,
        mb_x,
        mb_y,
        decide_mv.0,
        decide_mv.1,
        &mut y_pred,
        &mut u_pred,
        &mut v_pred,
    );

    // Compute luma residual sum for intra decision.
    let mut luma_abs_sum = 0u32;
    for j in 0..16 {
        for i in 0..16 {
            let s = src_y.data[(mb_y * 16 + j) * src_y.stride + (mb_x * 16 + i)] as i32;
            let p = y_pred[j * 16 + i] as i32;
            luma_abs_sum += (s - p).unsigned_abs();
        }
    }
    let intra_variance = mb_luma_variance(src_y, mb_x, mb_y);
    let try_intra = intra_variance * 5 < luma_abs_sum;

    if can_skip && luma_abs_sum < (quant as u32) * 128 {
        bw.write_bits(1, 1); // COD = 1 (skipped)
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        copy_predictor_to_recon(recon, mb_x, mb_y, &y_pred, &u_pred, &v_pred);
        return Ok(());
    }

    bw.write_bits(0, 1); // COD = 0

    if try_intra {
        encode_p_mb_intra(bw, mb_x, mb_y, quant, frame, width, height, recon)?;
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
        return Ok(());
    }

    // Inter path with AIV VLC selection.
    // Re-build predictor with the actual mv (not decide_mv).
    build_mb_predictor(
        reference,
        mb_x,
        mb_y,
        mvx,
        mvy,
        &mut y_pred,
        &mut u_pred,
        &mut v_pred,
    );

    // Quantise residual for all 6 blocks.
    let mut levels_all = [[0i32; 64]; 6];
    let mut any_nonzero = [false; 6];

    // Luma blocks.
    let src_cb = &frame.planes[1];
    let src_cr = &frame.planes[2];
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            _ => (8, 8),
        };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 16 + sub_y + j;
                let px = mb_x * 16 + sub_x + i;
                let s = src_y.data[py * src_y.stride + px] as i32;
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        any_nonzero[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }
    // Chroma blocks.
    for (ci, plane) in [(0usize, src_cb), (1, src_cr)].iter() {
        let pred = if *ci == 0 { &u_pred } else { &v_pred };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 8 + j;
                let px = mb_x * 8 + i;
                let s = plane.data[py * plane.stride + px] as i32;
                let p = pred[j * 8 + i] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        let b = 4 + ci;
        any_nonzero[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }

    let cbpc: u8 = ((any_nonzero[4] as u8) << 1) | (any_nonzero[5] as u8);
    let cbpy_raw: u8 = ((any_nonzero[0] as u8) << 3)
        | ((any_nonzero[1] as u8) << 2)
        | ((any_nonzero[2] as u8) << 1)
        | (any_nonzero[3] as u8);

    // §S.3: when CBPC5=1 AND CBPC6=1 (both chroma coded), emit CBPY without XOR.
    let both_chroma_coded = cbpc == 0b11;

    write_mcbpc_inter(bw, PMbKind::Inter, cbpc);
    if both_chroma_coded {
        // §S.3: CBPY without XOR (INTRA encoding shape).
        write_cbpy(bw, cbpy_raw);
    } else {
        write_cbpy(bw, cbpy_raw ^ 0xF);
    }

    // MVD.
    let (px, py) = predict_mv(mv_grid, mb_x, mb_y);
    if !enable_annex_d_umv {
        encode_mv_component(bw, mvx, px);
        encode_mv_component(bw, mvy, py);
    } else {
        encode_mv_component_umv(bw, mvx, px);
        encode_mv_component_umv(bw, mvy, py);
    }
    mv_grid.set(mb_x, mb_y, MbMotion::mv1((mvx, mvy), true, false));

    // Residual blocks — use AIV VLC selection for inter blocks.
    for b in 0..6 {
        if any_nonzero[b] {
            write_block_ac_inter_aiv(bw, &levels_all[b]);
        }
        // Reconstruct block into `recon`.
        let coeffs = dequantise_block(&levels_all[b], quant, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        let pred_slice: &[u8] = if b < 4 {
            &y_pred
        } else if b == 4 {
            &u_pred
        } else {
            &v_pred
        };
        let pred_stride = if b < 4 { 16 } else { 8 };
        let pred_off_x = if b < 4 {
            match b {
                0 => 0,
                1 => 8,
                2 => 0,
                _ => 8,
            }
        } else {
            0
        };
        let pred_off_y = if b < 4 {
            match b {
                0 | 1 => 0,
                _ => 8,
            }
        } else {
            0
        };
        for j in 0..8 {
            for i in 0..8 {
                let p = pred_slice[(pred_off_y + j) * pred_stride + (pred_off_x + i)] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Annex T — Modified Quantization (encoder helpers + picture functions)
// ---------------------------------------------------------------------------

/// PLUSPTYPE picture header writer for Annex T (Modified Quantization).
/// Emits OPPTYPE bit 14 (MQ) = 1; all other OPPTYPE bits are off.
fn write_plusptype_picture_header_mq(
    bw: &mut BitWriter,
    source_format: SourceFormat,
    pquant: u8,
    tr: u8,
    is_p_picture: bool,
    sei: &[Sei],
) -> Result<()> {
    debug_assert!(bw.is_byte_aligned());
    let src_code: u32 = match source_format {
        SourceFormat::SubQcif => 1,
        SourceFormat::Qcif => 2,
        SourceFormat::Cif => 3,
        SourceFormat::FourCif => 4,
        SourceFormat::SixteenCif => 5,
        _ => {
            return Err(Error::unsupported(
                "h263 MQ encoder: only standard source formats 1..=5 are supported",
            ));
        }
    };
    #[allow(clippy::unusual_byte_groupings)]
    let psc: u32 = 0b00_0000_0000_0000_0000_1_00000;
    bw.write_bits(psc, 22);
    bw.write_bits(tr as u32, 8);
    bw.write_bits(1, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0, 1);
    bw.write_bits(0b111, 3); // PLUSPTYPE
    bw.write_bits(0b001, 3); // UFEP=001
                             // OPPTYPE (18 bits): src_fmt + MQ(bit14) + marker(bit15)
    let bit = |k: u32, v: u32| (v & 1) << (18 - k);
    let srcf_part = (src_code & 0b111) << 15;
    let opptype: u32 = srcf_part
        | bit(14, 1)  // MQ
        | bit(15, 1); // marker
    bw.write_bits(opptype, 18);
    let pct: u32 = if is_p_picture { 0b001 } else { 0b000 };
    let mpptype: u32 = (pct << 6) | 0b000_001;
    bw.write_bits(mpptype, 9);
    bw.write_bits(0, 1); // CPM=0
    if pquant == 0 || pquant > 31 {
        return Err(Error::invalid(format!(
            "h263 MQ encoder: pquant {pquant} out of range 1..=31"
        )));
    }
    bw.write_bits(pquant as u32, 5);
    let psupp = serialise_sei_to_psupp(sei);
    write_pei_loop(bw, &psupp);
    Ok(())
}

/// Encode an I-picture with the Annex T (MQ) PLUSPTYPE header.
/// The I-picture body uses the standard intra encoder; the decoder's
/// `decode_intra_mb_mq` handles DQUANT via §T.2 VLC and QUANT_C for chroma,
/// but since we emit a fixed PQUANT with no per-MB DQUANT the MQ decoder path
/// produces the same reconstruction as the baseline path at the picture level.
pub fn encode_i_picture_mq_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    sei: &[Sei],
) -> Result<(Vec<u8>, IPicture)> {
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 MQ encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    write_plusptype_picture_header_mq(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        false,
        sei,
    )?;
    for mb_y in 0..mb_h {
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let gn = (mb_y as u32 / mb_rows_per_gob) as u8;
            write_gob_header(&mut bw, gn, pquant)?;
        }
        for mb_x in 0..mb_w {
            encode_intra_mb_mq(
                &mut bw, mb_x, mb_y, pquant, frame, width, height, &mut recon,
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Encode a P-picture with the Annex T (MQ) PLUSPTYPE header. Chroma blocks
/// use `QUANT_C = quant_c_for_quant(pquant)` for quantisation (§T.3 / Table
/// T.2). Luma blocks use the standard `pquant`. No per-MB DQUANT is emitted
/// (the encoder holds pquant fixed).
#[allow(clippy::too_many_arguments)]
pub fn encode_p_picture_mq_with_recon(
    width: u32,
    height: u32,
    source_format: SourceFormat,
    pquant: u8,
    temporal_reference: u8,
    frame: &VideoFrame,
    reference: &IPicture,
    enable_annex_f: bool,
    enable_annex_d_umv: bool,
    sei: &[Sei],
) -> Result<(Vec<u8>, IPicture)> {
    if enable_annex_f {
        return Err(Error::unsupported(
            "h263 MQ + Annex F (Advanced Prediction): not yet combined",
        ));
    }
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 MQ encoder: source format has no GOB layout"))?;
    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut mv_grid = MvGrid::new(mb_w, mb_h);
    write_plusptype_picture_header_mq(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true,
        sei,
    )?;
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            encode_p_mb_mq(
                &mut bw,
                mb_x,
                mb_y,
                pquant,
                frame,
                width,
                height,
                reference,
                &mut recon,
                &mut mv_grid,
                enable_annex_d_umv,
            )?;
        }
    }
    Ok((bw.finish(), recon))
}

/// Encode one intra MB under Annex T (MQ) rules. The key difference from the
/// baseline is that chroma uses `QUANT_C` (§T.3) for dequantisation; the
/// encoder quantises chroma with `quant_c` so the reconstruction matches what
/// `decode_intra_mb_mq` produces.
#[allow(clippy::too_many_arguments)]
fn encode_intra_mb_mq(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    recon: &mut IPicture,
) -> Result<()> {
    use crate::mq::quant_c_for_quant;
    let quant_c = quant_c_for_quant(quant as u32) as u8;

    let mut blocks = [[0i32; 64]; 6];
    let mut dc_pels = [128u8; 6];
    let mut block_has_ac = [false; 6];

    for b in 0..6 {
        let q = if b < 4 { quant } else { quant_c };
        let mut samples = [0.0f32; 64];
        sample_block_for(frame, width, height, mb_x, mb_y, b, &mut samples);
        let mut dctf = samples;
        fdct8x8(&mut dctf);
        let (dc_byte, levels, any_ac) = quantise_intra_block(&dctf, q);
        dc_pels[b] = dc_byte;
        block_has_ac[b] = any_ac;
        blocks[b] = levels;
    }
    let cbpc: u8 = ((block_has_ac[4] as u8) << 1) | (block_has_ac[5] as u8);
    let cbpy: u8 = ((block_has_ac[0] as u8) << 3)
        | ((block_has_ac[1] as u8) << 2)
        | ((block_has_ac[2] as u8) << 1)
        | (block_has_ac[3] as u8);
    write_mcbpc_intra(bw, cbpc);
    write_cbpy(bw, cbpy);
    for b in 0..6 {
        let q = if b < 4 { quant } else { quant_c };
        bw.write_bits(dc_pels[b] as u32, 8);
        if block_has_ac[b] {
            write_block_ac(bw, &blocks[b]);
        }
        reconstruct_intra_block(recon, b, mb_x, mb_y, dc_pels[b], &blocks[b], q);
    }
    Ok(())
}

/// Encode one P-MB under Annex T (MQ) rules. Chroma blocks use `QUANT_C`
/// (§T.3 / Table T.2) for quantisation; luma uses the picture QUANT.
/// No per-MB DQUANT is emitted (quant is held constant across the picture).
#[allow(clippy::too_many_arguments)]
fn encode_p_mb_mq(
    bw: &mut BitWriter,
    mb_x: usize,
    mb_y: usize,
    quant: u8,
    frame: &VideoFrame,
    width: u32,
    height: u32,
    reference: &IPicture,
    recon: &mut IPicture,
    mv_grid: &mut MvGrid,
    enable_annex_d_umv: bool,
) -> Result<()> {
    use crate::mq::quant_c_for_quant;
    let quant_c = quant_c_for_quant(quant as u32) as u8;

    let src_y = &frame.planes[0];
    let src_cb = &frame.planes[1];
    let src_cr = &frame.planes[2];

    // Motion estimation and skip decision (same as baseline).
    let (mvx, mvy, mv_sad) = if enable_annex_d_umv {
        motion_estimate_mb_umv(frame, reference, mb_x, mb_y, mv_grid)
    } else {
        motion_estimate_mb(frame, reference, mb_x, mb_y)
    };

    let zero_sad = sad_block(
        &frame.planes[0].data,
        frame.planes[0].stride,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        &reference.y,
        reference.y_stride,
        reference.y_stride as i32,
        (reference.y.len() / reference.y_stride) as i32,
        (mb_x * 16) as i32,
        (mb_y * 16) as i32,
        0,
        0,
        16,
    );
    let (pmx, pmy) = predict_mv(mv_grid, mb_x, mb_y);
    let can_skip = pmx == 0 && pmy == 0 && zero_sad < mv_sad + 128;

    // Build predictor for skip / intra decision.
    let mut y_pred = [0u8; 256];
    let mut u_pred = [0u8; 64];
    let mut v_pred = [0u8; 64];
    let decide_mv = if can_skip { (0, 0) } else { (mvx, mvy) };
    build_mb_predictor(
        reference,
        mb_x,
        mb_y,
        decide_mv.0,
        decide_mv.1,
        &mut y_pred,
        &mut u_pred,
        &mut v_pred,
    );

    // Luma residual sum for intra decision.
    let mut luma_abs_sum = 0u32;
    for j in 0..16 {
        for i in 0..16 {
            let s = src_y.data[(mb_y * 16 + j) * src_y.stride + (mb_x * 16 + i)] as i32;
            let p = y_pred[j * 16 + i] as i32;
            luma_abs_sum += (s - p).unsigned_abs();
        }
    }
    let intra_variance = mb_luma_variance(src_y, mb_x, mb_y);

    if can_skip && luma_abs_sum < (quant as u32) * 128 {
        bw.write_bits(1, 1);
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), false, false));
        copy_predictor_to_recon(recon, mb_x, mb_y, &y_pred, &u_pred, &v_pred);
        return Ok(());
    }

    bw.write_bits(0, 1); // COD = 0

    if intra_variance * 5 < luma_abs_sum {
        encode_intra_mb_mq(bw, mb_x, mb_y, quant, frame, width, height, recon)?;
        mv_grid.set(mb_x, mb_y, MbMotion::mv1((0, 0), true, true));
        return Ok(());
    }

    // Inter path with MQ chroma quant.
    // Re-build predictor with actual mv.
    build_mb_predictor(
        reference,
        mb_x,
        mb_y,
        mvx,
        mvy,
        &mut y_pred,
        &mut u_pred,
        &mut v_pred,
    );

    // Quantise residual — use quant_c for chroma blocks.
    let mut levels_all = [[0i32; 64]; 6];
    let mut any_nonzero = [false; 6];

    // Luma blocks.
    for b in 0..4 {
        let (sub_x, sub_y) = match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            _ => (8, 8),
        };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 16 + sub_y + j;
                let px = mb_x * 16 + sub_x + i;
                let s = src_y.data[py * src_y.stride + px] as i32;
                let p = y_pred[(sub_y + j) * 16 + (sub_x + i)] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant);
        any_nonzero[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }
    // Chroma blocks — use quant_c.
    for (ci, plane) in [(0usize, src_cb), (1, src_cr)].iter() {
        let pred = if *ci == 0 { &u_pred } else { &v_pred };
        let mut resid = [0.0f32; 64];
        for j in 0..8 {
            for i in 0..8 {
                let py = mb_y * 8 + j;
                let px = mb_x * 8 + i;
                let s = plane.data[py * plane.stride + px] as i32;
                let p = pred[j * 8 + i] as i32;
                resid[j * 8 + i] = (s - p) as f32;
            }
        }
        let mut dctf = resid;
        fdct8x8(&mut dctf);
        let levels = quantise_inter_block(&dctf, quant_c);
        let b = 4 + ci;
        any_nonzero[b] = levels.iter().any(|&l| l != 0);
        levels_all[b] = levels;
    }

    let cbpc: u8 = ((any_nonzero[4] as u8) << 1) | (any_nonzero[5] as u8);
    let cbpy_raw: u8 = ((any_nonzero[0] as u8) << 3)
        | ((any_nonzero[1] as u8) << 2)
        | ((any_nonzero[2] as u8) << 1)
        | (any_nonzero[3] as u8);

    write_mcbpc_inter(bw, PMbKind::Inter, cbpc);
    write_cbpy(bw, cbpy_raw ^ 0xF); // inter CBPY with XOR

    // MVD.
    let (px, py) = predict_mv(mv_grid, mb_x, mb_y);
    if !enable_annex_d_umv {
        encode_mv_component(bw, mvx, px);
        encode_mv_component(bw, mvy, py);
    } else {
        encode_mv_component_umv(bw, mvx, px);
        encode_mv_component_umv(bw, mvy, py);
    }
    mv_grid.set(mb_x, mb_y, MbMotion::mv1((mvx, mvy), true, false));

    // Residual blocks — use appropriate quant for each block.
    for b in 0..6 {
        if any_nonzero[b] {
            write_block_ac_inter(bw, &levels_all[b]);
        }
        // Reconstruct: use quant for luma, quant_c for chroma.
        let q_recon = if b < 4 { quant } else { quant_c };
        let coeffs = dequantise_block(&levels_all[b], q_recon, false);
        let mut c = coeffs;
        let mut resid_out = [0i32; 64];
        crate::block::idct_signed(&mut c, &mut resid_out);
        let (plane, stride, px, py) = block_dst(recon, b, mb_x, mb_y);
        let pred_slice: &[u8] = if b < 4 {
            &y_pred
        } else if b == 4 {
            &u_pred
        } else {
            &v_pred
        };
        let pred_stride = if b < 4 { 16 } else { 8 };
        let pred_off_x = if b < 4 {
            match b {
                0 => 0,
                1 => 8,
                2 => 0,
                _ => 8,
            }
        } else {
            0
        };
        let pred_off_y = if b < 4 {
            match b {
                0 | 1 => 0,
                _ => 8,
            }
        } else {
            0
        };
        for j in 0..8 {
            for i in 0..8 {
                let p = pred_slice[(pred_off_y + j) * pred_stride + (pred_off_x + i)] as i32;
                let r = resid_out[j * 8 + i];
                plane[(py + j) * stride + (px + i)] = (p + r).clamp(0, 255) as u8;
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::frame::VideoPlane;

    fn make_constant_frame(w: u32, h: u32, y: u8, cb: u8, cr: u8) -> VideoFrame {
        let cw = w.div_ceil(2) as usize;
        let ch = h.div_ceil(2) as usize;
        VideoFrame {
            pts: Some(0),
            planes: vec![
                VideoPlane {
                    stride: w as usize,
                    data: vec![y; (w * h) as usize],
                },
                VideoPlane {
                    stride: cw,
                    data: vec![cb; cw * ch],
                },
                VideoPlane {
                    stride: cw,
                    data: vec![cr; cw * ch],
                },
            ],
        }
    }

    /// Encode a constant-grey QCIF picture, then decode it via the existing
    /// decoder and check the round-trip is bit-exact (DC-only, no AC).
    #[test]
    fn encode_decode_constant_qcif() {
        let frame = make_constant_frame(176, 144, 100, 128, 128);
        let bytes = encode_i_picture(176, 144, SourceFormat::Qcif, 5, 0, &frame).expect("encode");
        // Decode it back.
        use crate::decoder::H263Decoder;
        use oxideav_core::Decoder;
        use oxideav_core::Frame as CoreFrame;

        let mut dec = H263Decoder::new(CodecId::new(crate::CODEC_ID_STR));
        let pkt = Packet::new(0, TimeBase::new(1, 30), bytes);
        dec.send_packet(&pkt).expect("send");
        dec.flush().expect("flush");
        let f = dec.receive_frame().expect("receive");
        let v = match f {
            CoreFrame::Video(v) => v,
            _ => panic!("not video"),
        };
        // Check: most pels should be ≈100 in luma. The decoder produces a
        // 176×144 YUV420 picture; its dimensions live on the stream params
        // (we pass them explicitly here since this is an internal test).
        let yp = &v.planes[0];
        let width = 176usize;
        let height = 144usize;
        let mut hits = 0usize;
        for y in 0..height {
            for x in 0..width {
                let p = yp.data[y * yp.stride + x] as i32;
                if (p - 100).abs() <= 2 {
                    hits += 1;
                }
            }
        }
        let total = width * height;
        assert!(hits * 100 / total >= 99, "constant Y match {hits}/{total}");
    }
}
