//! PB-frame encoders: Annex M **Improved PB-frames** (§M.1 – §M.4) and
//! the Annex G / Annex M **Advanced Prediction** compositions.
//!
//! An Improved PB-frame is one picture unit carrying a P-picture and a
//! BPB-picture (the "B-part", §M.1), signalled by the PLUSPTYPE
//! MPPTYPE picture type `"010"` (§5.1.4.3). Its P-part is coded exactly
//! like [`crate::encoder::encode_pb_picture`]'s; the difference is the
//! B-part, where every BPB-macroblock chooses one of the three §M.2
//! prediction modes through the Table M.1 MODB codeword:
//!
//! * **bidirectional** (§M.2.1) — the §G.4 / §G.5 scaled-vector
//!   composition with `MVD = 0` (no MVDB on the wire);
//! * **forward** (§M.2.2) — a single 16 × 16 forward vector into the
//!   previous reference picture, coded as MVDB against the §M.2.2
//!   left-neighbour predictor;
//! * **backward** (§M.2.3) — the prediction is PREC (§G.5), no vector
//!   data.
//!
//! The mode decision is rate-biased SAD (the luminance SAD of the
//! B-source against each candidate prediction plus λ × the MODB / MVDB
//! bits); the residual of the chosen prediction is coded at the
//! Table-6 BQUANT where it survives quantisation (CBPB).
//!
//! With **Advanced Prediction** (Annex F) the P-part carries four
//! §F.2 vectors per macroblock predicted through the §F.3 OBMC blend
//! (two-pass: the whole vector field is estimated first so every
//! remote vector is known), PREC is that OBMC reconstruction, and the
//! B-part's §G.4 forward / backward vectors are scaled per 8 × 8 block.
//! In PB-frames mode an INTRA macroblock carries a motion vector for
//! its B-blocks (§G.2), which also serves as its neighbours' OBMC
//! remote vector ("the remote 'INTRA' motion vector is used") and
//! §6.1.1 candidate predictor — the optional INTRA refresh exercises
//! exactly those rules on the crate's decoder (the vector sent is zero,
//! see [`ImprovedPbConfig::intra_refresh`]).
//!
//! Every output decodes through the crate's PB drivers
//! ([`crate::picture::decode_pb_picture_no_gob0_header`] /
//! [`crate::picture::decode_improved_pb_picture_with_inherited`]) and —
//! inside an elementary stream — [`crate::picture::decode_sequence`],
//! which splices the decoded pair in display order (B before P).

use crate::block::{COEFFS_PER_BLOCK, H263Block};
use crate::encoder::{
    PbConfig, PlusModes, PtypeFlags, extract_macroblock, motion_compensated_block_rc, residual_of,
    source_format_for, write_picture_header, write_plus_picture_header,
};
use crate::encoder_block::{
    EncodedInterBlock, EncodedIntraBlock, encode_inter_block, encode_intra_block,
    write_inter_block_coeffs, write_intra_block,
};
use crate::encoder_motion::{Mv4Grid, MvGrid, estimate_block_motion, estimate_motion, mvd_for};
use crate::encoder_vlc::{write_cbpy, write_mcbpc_p, write_mvd_component};
use crate::macroblock::{MbType, Mvd};
use crate::motion::{
    LumaBlockIndex, Mb4Mv, MotionVector, RCONTROL_DEFAULT, RefPlane, RemoteMv, chroma_mv,
    chroma_mv_4mv, obmc_predict_block,
};
use crate::pb_layer::{
    BpbCodingMode, ModbAnnexM, PbBMacroblockPrediction, PbBReferencePlanes, pb_bquant,
    write_modb_annex_m,
};
use crate::picture::YuvFrame;
use crate::{Error, Result};
use oxideav_core::bits::BitWriter;

/// Configuration for [`encode_improved_pb_picture`].
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct ImprovedPbConfig {
    /// Quantiser for the P-blocks (`1..=31`); the BPB-blocks run at the
    /// §5.1.23 / Table-6 BQUANT derived from it and `dbquant`.
    pub quant: u8,
    /// §5.1.22 TRB (3-bit form, `1..=7`).
    pub trb: u8,
    /// §5.1.23 DBQUANT (`0..=3`).
    pub dbquant: u8,
    /// Motion-search window for the P-part (±whole pixels).
    pub search_half: i32,
    /// §M.2.2 forward-vector search window for the BPB-part (±whole
    /// pixels around the left-neighbour predictor). `0` disables the
    /// forward mode altogether (only bidirectional / backward compete).
    pub forward_search_half: i32,
    /// Allow the §M.2.3 backward mode (prediction = PREC) to compete.
    pub allow_backward: bool,
    /// Annex F Advanced Prediction on the P-part (OPPTYPE bit 7): four
    /// §F.2 vectors per macroblock through the §F.3 OBMC blend, with
    /// the B-part's §G.4 vectors scaled per 8 × 8 block.
    pub advanced_prediction: bool,
    /// Annex D Unrestricted Motion Vector mode on the PLUSPTYPE header
    /// (OPPTYPE bit 5, UUI = "1"): the P-part vectors, the INTRA
    /// macroblocks' §G.2 MVD and the §M.2.2 forward vectors are Table
    /// D.3 coded over the Tables-D.1/D.2 range, the forward fetch
    /// reaching over the picture boundary through §D.1.
    pub umv: bool,
    /// Annex K Slice Structured mode (OPPTYPE bit 10, free-running
    /// sequential slices): `slice_rows > 0` emits a §K.2 slice every
    /// `slice_rows` macroblock rows, each its own §6.1.1 / §F.3 video
    /// picture segment (predictors and OBMC remotes confined per §K.1
    /// rules 1 / 3, the §M.2.2 forward predictor restarting at every
    /// slice's left edge). `0` emits the single-segment GOB layout.
    pub slice_rows: usize,
    /// Annex I Advanced INTRA Coding (OPPTYPE bit 8): the INTRA-refresh
    /// macroblocks are coded per §I (INTRA_MODE decision, §I.3 DC/AC
    /// prediction from the encoder's own reconstructed INTRA neighbours
    /// — an INTER macroblock is no §I.3 predictor — and the Table I.2
    /// VLC); their PREC is the §I reconstruction.
    pub aic: bool,
    /// INTRA-code every `intra_refresh`-th macroblock (raster order,
    /// starting with the first); `0` disables the refresh. A PB-frame
    /// INTRA macroblock still carries the vector its B-blocks use
    /// (§G.2 / §M.2.1); this encoder always sends the **zero** vector
    /// there. Any vector is legal, but a zero one keeps the stream
    /// decodable by decoders that ignore the §6.1.1 rule-1 PB-frames
    /// exception (an INTRA candidate predictor is *not* zeroed in
    /// PB-frames mode) — the black-box oracle decoder does exactly
    /// that, so a non-zero INTRA vector would desynchronise every
    /// vector predicted from it there.
    pub intra_refresh: usize,
    /// MPPTYPE RTYPE bit (§5.1.4.3): the §6.1.2 `RCONTROL` of the
    /// P-part's half-pel prediction (the BPB-part always uses `0`).
    /// §5.1.4.3 recommends the value differ from the reference
    /// picture's so rounding errors do not accumulate; a sequence
    /// driver alternates it picture by picture.
    pub rounding_type: bool,
}

impl ImprovedPbConfig {
    /// Set [`ImprovedPbConfig::quant`].
    #[must_use]
    pub fn with_quant(mut self, v: u8) -> Self {
        self.quant = v;
        self
    }

    /// Set [`ImprovedPbConfig::trb`].
    #[must_use]
    pub fn with_trb(mut self, v: u8) -> Self {
        self.trb = v;
        self
    }

    /// Set [`ImprovedPbConfig::dbquant`].
    #[must_use]
    pub fn with_dbquant(mut self, v: u8) -> Self {
        self.dbquant = v;
        self
    }

    /// Set [`ImprovedPbConfig::search_half`].
    #[must_use]
    pub fn with_search_half(mut self, v: i32) -> Self {
        self.search_half = v;
        self
    }

    /// Set [`ImprovedPbConfig::forward_search_half`].
    #[must_use]
    pub fn with_forward_search_half(mut self, v: i32) -> Self {
        self.forward_search_half = v;
        self
    }

    /// Set [`ImprovedPbConfig::allow_backward`].
    #[must_use]
    pub fn with_allow_backward(mut self, v: bool) -> Self {
        self.allow_backward = v;
        self
    }

    /// Set [`ImprovedPbConfig::advanced_prediction`].
    #[must_use]
    pub fn with_advanced_prediction(mut self, v: bool) -> Self {
        self.advanced_prediction = v;
        self
    }

    /// Set [`ImprovedPbConfig::umv`].
    #[must_use]
    pub fn with_umv(mut self, v: bool) -> Self {
        self.umv = v;
        self
    }

    /// Set [`ImprovedPbConfig::slice_rows`].
    #[must_use]
    pub fn with_slice_rows(mut self, v: usize) -> Self {
        self.slice_rows = v;
        self
    }

    /// Set [`ImprovedPbConfig::aic`].
    #[must_use]
    pub fn with_aic(mut self, v: bool) -> Self {
        self.aic = v;
        self
    }

    /// Set [`ImprovedPbConfig::intra_refresh`].
    #[must_use]
    pub fn with_intra_refresh(mut self, v: usize) -> Self {
        self.intra_refresh = v;
        self
    }

    /// Set [`ImprovedPbConfig::rounding_type`].
    #[must_use]
    pub fn with_rounding_type(mut self, v: bool) -> Self {
        self.rounding_type = v;
        self
    }
}

impl Default for ImprovedPbConfig {
    fn default() -> Self {
        ImprovedPbConfig {
            quant: 8,
            trb: 1,
            dbquant: 0,
            search_half: 8,
            forward_search_half: 4,
            allow_backward: true,
            advanced_prediction: false,
            umv: false,
            slice_rows: 0,
            aic: false,
            intra_refresh: 0,
            rounding_type: false,
        }
    }
}

/// Per-picture mode census returned by the `_stats` encoder forms.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct ImprovedPbStats {
    /// Macroblocks emitted with COD = 1 (their B-part is the
    /// zero-vector bidirectional prediction).
    pub skipped: usize,
    /// Coded macroblocks whose B-part took the bidirectional mode
    /// (every coded Annex G macroblock counts here).
    pub bidirectional: usize,
    /// Coded macroblocks whose BPB-part took the §M.2.2 forward mode.
    pub forward: usize,
    /// Coded macroblocks whose BPB-part took the §M.2.3 backward mode.
    pub backward: usize,
    /// Macroblocks with at least one CBPB-lit B-block.
    pub b_residual: usize,
    /// Macroblocks whose P-part was INTRA coded.
    pub intra: usize,
    /// Macroblocks that carried a non-zero MVDB (Annex G delta search
    /// hits, or Annex M forward vectors).
    pub mvdb: usize,
}

impl ImprovedPbStats {
    /// Set [`ImprovedPbStats::skipped`].
    #[must_use]
    pub fn with_skipped(mut self, v: usize) -> Self {
        self.skipped = v;
        self
    }

    /// Set [`ImprovedPbStats::bidirectional`].
    #[must_use]
    pub fn with_bidirectional(mut self, v: usize) -> Self {
        self.bidirectional = v;
        self
    }

    /// Set [`ImprovedPbStats::forward`].
    #[must_use]
    pub fn with_forward(mut self, v: usize) -> Self {
        self.forward = v;
        self
    }

    /// Set [`ImprovedPbStats::backward`].
    #[must_use]
    pub fn with_backward(mut self, v: usize) -> Self {
        self.backward = v;
        self
    }

    /// Set [`ImprovedPbStats::b_residual`].
    #[must_use]
    pub fn with_b_residual(mut self, v: usize) -> Self {
        self.b_residual = v;
        self
    }

    /// Set [`ImprovedPbStats::intra`].
    #[must_use]
    pub fn with_intra(mut self, v: usize) -> Self {
        self.intra = v;
        self
    }

    /// Set [`ImprovedPbStats::mvdb`].
    #[must_use]
    pub fn with_mvdb(mut self, v: usize) -> Self {
        self.mvdb = v;
        self
    }
}

/// Which PB-frame flavour the shared core emits.
#[derive(Debug, Clone, Copy)]
enum PbFlavour {
    /// Annex G (baseline PTYPE bit 13, Table 11 MODB): the B-part is
    /// always bidirectional, optionally refined by a §5.3.9 MVDB delta
    /// searched over ±`b_search_half` half-pels.
    AnnexG { b_search_half: i32 },
    /// Annex M (PLUSPTYPE `"010"`, Table M.1 MODB): the three §M.2
    /// modes compete.
    AnnexM {
        forward_search_half: i32,
        allow_backward: bool,
    },
}

/// Which §5.3.7 / §D.2 motion-vector coding the core emits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UmvMode {
    /// Table 14 with the default §6.1.1 wrap.
    Off,
    /// Annex D on a baseline header (Annex G only): Table 14 with the
    /// §D.2 predictor-dependent pair rule for MVD / MVD2-4; MVDB is a
    /// first-column value applied once per macroblock (§G.4), chosen
    /// so a per-block `Pc = (TRB × MV)/TRD` resolution agrees.
    Wrap,
    /// Annex D on a PLUSPTYPE header (Annex M only): Table D.3
    /// single-valued differences under the UUI = "1" range.
    Plus,
}

/// The flavour-independent knobs of the shared core.
#[derive(Debug, Clone, Copy)]
struct PbCore {
    quant: u8,
    trb: u8,
    dbquant: u8,
    search_half: i32,
    advanced_prediction: bool,
    umv: UmvMode,
    /// Annex K row-aligned free-running slices every `slice_rows`
    /// macroblock rows (`0` = none; Annex M only).
    slice_rows: usize,
    /// Annex I on the INTRA-refresh macroblocks (Annex M only).
    aic: bool,
    intra_refresh: usize,
    /// §6.1.2 `RCONTROL` of the P-part (the PLUSPTYPE RTYPE bit;
    /// always `0` for Annex G, whose header has no RTYPE).
    rcontrol: i32,
}

/// Encode an Annex M **Improved PB-frame**: one picture unit carrying a
/// P-picture (`p_source`, predicted from `reference`) and a BPB-picture
/// (`b_source`, temporally between `reference` and `p_source`).
///
/// See the [module documentation](self) for the per-macroblock mode
/// decision. `tr_p` is the §5.1.2 Temporal Reference of the P-part;
/// `prev_tr` is the reference picture's TR (their difference mod 256 is
/// the §G.4 TRD, which must be non-zero and greater than
/// [`ImprovedPbConfig::trb`]).
///
/// # Errors
///
/// * [`Error::InvalidQuantiser`] — `cfg.quant` outside `1..=31`.
/// * [`Error::BadPbTemporalReference`] — TRB / DBQUANT out of range or
///   `TRB >= TRD`.
/// * [`Error::NotImplemented`] — the three frames disagree in geometry
///   or the size is not a standard §5.1.3 source format.
pub fn encode_improved_pb_picture(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &ImprovedPbConfig,
) -> Result<Vec<u8>> {
    encode_improved_pb_picture_stats(p_source, b_source, reference, tr_p, prev_tr, cfg)
        .map(|(bytes, _)| bytes)
}

/// As [`encode_improved_pb_picture`], additionally returning the
/// per-picture [`ImprovedPbStats`] mode census.
pub fn encode_improved_pb_picture_stats(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &ImprovedPbConfig,
) -> Result<(Vec<u8>, ImprovedPbStats)> {
    let (core, flavour) = annex_m_core(cfg);
    encode_pb_core(p_source, b_source, reference, tr_p, prev_tr, core, flavour)
        .map(|o| (o.bytes, o.stats))
}

/// Encode an Annex G **PB-frame with Advanced Prediction** (baseline
/// PTYPE bits 12 + 13): the P-part carries four §F.2 vectors per
/// macroblock through the §F.3 OBMC blend, PREC is that OBMC
/// reconstruction, and the B-part's §G.4 vectors (plus the searched
/// §5.3.9 MVDB delta, [`PbConfig::b_search_half`]) are scaled per 8 × 8
/// block. Same contract as [`crate::encoder::encode_pb_picture`]
/// otherwise; decodes through
/// [`crate::picture::decode_pb_picture_no_gob0_header`] /
/// `decode_sequence`.
pub fn encode_pb_picture_ap(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<Vec<u8>> {
    encode_pb_picture_ap_stats(p_source, b_source, reference, tr_p, prev_tr, cfg)
        .map(|(bytes, _)| bytes)
}

/// As [`encode_pb_picture_ap`], additionally returning the per-picture
/// [`ImprovedPbStats`] census (`forward` / `backward` stay zero under
/// Annex G).
pub fn encode_pb_picture_ap_stats(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<(Vec<u8>, ImprovedPbStats)> {
    encode_pb_core(
        p_source,
        b_source,
        reference,
        tr_p,
        prev_tr,
        annex_g_core(cfg, true, UmvMode::Off),
        PbFlavour::AnnexG {
            b_search_half: cfg.b_search_half,
        },
    )
    .map(|o| (o.bytes, o.stats))
}

/// Encode an Annex G **PB-frame in the Unrestricted Motion Vector
/// mode** (baseline PTYPE bits 10 + 13): the P-part vectors are
/// searched over the §D.2 extended `[-31.5, 31.5]` range with the
/// predictor-dependent Table 14 pair rule, and the §5.3.9 MVDB delta
/// ([`PbConfig::b_search_half`]) is one first-column Table 14 value
/// per macroblock (§G.4), restricted to deltas a per-block §D.2
/// resolution with `Pc = (TRB × MV)/TRD` reads identically. Same
/// contract as [`crate::encoder::encode_pb_picture`] otherwise; Annex
/// F composes through [`encode_pb_picture_umv_ap`].
pub fn encode_pb_picture_umv(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<Vec<u8>> {
    encode_pb_picture_umv_stats(p_source, b_source, reference, tr_p, prev_tr, cfg)
        .map(|(bytes, _)| bytes)
}

/// As [`encode_pb_picture_umv`], additionally returning the
/// per-picture [`ImprovedPbStats`] census.
pub fn encode_pb_picture_umv_stats(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<(Vec<u8>, ImprovedPbStats)> {
    encode_pb_core(
        p_source,
        b_source,
        reference,
        tr_p,
        prev_tr,
        annex_g_core(cfg, false, UmvMode::Wrap),
        PbFlavour::AnnexG {
            b_search_half: cfg.b_search_half,
        },
    )
    .map(|o| (o.bytes, o.stats))
}

/// Encode an Annex G **PB-frame in the Unrestricted Motion Vector mode
/// with Advanced Prediction** (baseline PTYPE bits 10 + 12 + 13, no
/// PLUSPTYPE): every P-macroblock carries four §F.2 vectors searched
/// over the §D.2 `[-31.5, 31.5]` range, each coded as a Table 14 MVD
/// through the §D.2 pair rule against its own block predictor, with
/// the §F.3 OBMC P-luma as PREC. The §5.3.9 MVDB delta is one
/// first-column Table 14 value per macroblock added to every 8 × 8
/// block's scaled vector (§G.4), restricted to deltas a per-block §D.2
/// resolution reads identically. Same contract as
/// [`crate::encoder::encode_pb_picture`] otherwise.
pub fn encode_pb_picture_umv_ap(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<Vec<u8>> {
    encode_pb_picture_umv_ap_stats(p_source, b_source, reference, tr_p, prev_tr, cfg)
        .map(|(bytes, _)| bytes)
}

/// As [`encode_pb_picture_umv_ap`], additionally returning the
/// per-picture [`ImprovedPbStats`] census.
pub fn encode_pb_picture_umv_ap_stats(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    cfg: &PbConfig,
) -> Result<(Vec<u8>, ImprovedPbStats)> {
    encode_pb_core(
        p_source,
        b_source,
        reference,
        tr_p,
        prev_tr,
        annex_g_core(cfg, true, UmvMode::Wrap),
        PbFlavour::AnnexG {
            b_search_half: cfg.b_search_half,
        },
    )
    .map(|o| (o.bytes, o.stats))
}

/// The shared-core knobs of an Annex G PB-frame encode.
fn annex_g_core(cfg: &PbConfig, advanced_prediction: bool, umv: UmvMode) -> PbCore {
    PbCore {
        quant: cfg.quant,
        trb: cfg.trb,
        dbquant: cfg.dbquant,
        search_half: cfg.search_half,
        advanced_prediction,
        umv,
        slice_rows: 0,
        aic: false,
        intra_refresh: 0,
        rcontrol: 0,
    }
}

/// The shared-core knobs of an Annex M Improved PB-frame encode.
fn annex_m_core(cfg: &ImprovedPbConfig) -> (PbCore, PbFlavour) {
    (
        PbCore {
            quant: cfg.quant,
            trb: cfg.trb,
            dbquant: cfg.dbquant,
            search_half: cfg.search_half,
            advanced_prediction: cfg.advanced_prediction,
            umv: if cfg.umv { UmvMode::Plus } else { UmvMode::Off },
            slice_rows: cfg.slice_rows,
            aic: cfg.aic,
            intra_refresh: cfg.intra_refresh,
            rcontrol: cfg.rounding_type as i32,
        },
        PbFlavour::AnnexM {
            forward_search_half: cfg.forward_search_half,
            allow_backward: cfg.allow_backward,
        },
    )
}

/// The P-part of one macroblock after prediction + residual coding:
/// what goes on the wire and the decoder-side reconstruction (PREC).
struct PPart {
    /// `Some` when the macroblock is an Annex I INTRA macroblock (its
    /// blocks, CBPC / CBPY and PREC all come from the plan).
    aic_plan: Option<crate::encoder::MbAicPlan>,
    luma_inter: [Option<EncodedInterBlock>; 4],
    luma_intra: [Option<EncodedIntraBlock>; 4],
    cb_inter: Option<EncodedInterBlock>,
    cr_inter: Option<EncodedInterBlock>,
    cb_intra: Option<EncodedIntraBlock>,
    cr_intra: Option<EncodedIntraBlock>,
    prec_y: [u8; 256],
    prec_cb: [u8; COEFFS_PER_BLOCK],
    prec_cr: [u8; COEFFS_PER_BLOCK],
    /// CBPC bits (`0b10` Cb, `0b01` Cr) and CBPY in INTRA orientation.
    cbpc: u8,
    cbpy: u8,
    any_coeffs: bool,
}

/// The B-part decision for one macroblock.
struct BPart {
    mode: BpbCodingMode,
    /// Annex G delta / Annex M forward vector difference on the wire.
    mvdb: Option<Mvd>,
    /// Annex M forward vector (the next macroblock's predictor).
    forward_mv: Option<MotionVector>,
    blocks: [EncodedInterBlock; 6],
    any: bool,
    /// The chosen B-prediction (the decoder-side reconstruction is this
    /// plus the dequantised residual of `blocks`).
    pred: PbBMacroblockPrediction,
}

/// What [`encode_pb_core`] produces: the picture unit plus the
/// encoder-side reconstructions of both parts (what a conforming
/// decoder must output).
struct PbCoreOutput {
    bytes: Vec<u8>,
    stats: ImprovedPbStats,
    #[cfg_attr(not(test), allow(dead_code))]
    p_recon: YuvFrame,
    #[cfg_attr(not(test), allow(dead_code))]
    b_recon: YuvFrame,
}

#[allow(clippy::too_many_arguments)]
fn encode_pb_core(
    p_source: &YuvFrame,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    tr_p: u8,
    prev_tr: u8,
    core: PbCore,
    flavour: PbFlavour,
) -> Result<PbCoreOutput> {
    if core.quant == 0 || core.quant > 31 {
        return Err(Error::InvalidQuantiser);
    }
    if core.trb == 0 || core.trb > 7 || core.dbquant > 3 {
        return Err(Error::BadPbTemporalReference);
    }
    let trd = i32::from(tr_p.wrapping_sub(prev_tr));
    if trd == 0 || i32::from(core.trb) >= trd {
        return Err(Error::BadPbTemporalReference);
    }
    if p_source.luma_width != reference.luma_width
        || p_source.luma_height != reference.luma_height
        || b_source.luma_width != reference.luma_width
        || b_source.luma_height != reference.luma_height
    {
        return Err(Error::NotImplemented);
    }
    let fmt = source_format_for(p_source.luma_width, p_source.luma_height)
        .ok_or(Error::NotImplemented)?;
    let layout =
        crate::picture::PictureLayout::for_source_format(fmt).ok_or(Error::NotImplemented)?;

    let quant = core.quant;
    let bquant = pb_bquant(core.dbquant, quant);
    let mut p_recon = reference.clone();
    let mut b_recon = reference.clone();
    let trb = i32::from(core.trb);
    let ap = core.advanced_prediction;
    let lambda = 2 * quant as u32;
    let mb_rows_total = p_source.luma_height / 16;
    if core.slice_rows > mb_rows_total
        || (core.slice_rows > 0 && matches!(flavour, PbFlavour::AnnexG { .. }))
    {
        // Annex K is PLUSPTYPE-only (§G.1 bars it from Annex G).
        return Err(Error::UnsupportedPictureGeometry);
    }
    // Annex K free-running sequential slices every `slice_rows` rows.
    let sss = (core.slice_rows > 0).then_some(crate::plus_ptype::SliceStructuredSubmode {
        rectangular: false,
        arbitrary_order: false,
    });
    let slice_ctx = sss.map(|sss| {
        crate::slice_header::SliceHeaderContext::from_picture_layout(
            &layout,
            Some(sss),
            false,
            false,
        )
    });
    if core.aic && matches!(flavour, PbFlavour::AnnexG { .. }) {
        // Annex I is PLUSPTYPE-only (§G.1 bars it from Annex G).
        return Err(Error::NotImplemented);
    }
    let mut aic_grid = crate::encoder::AicEncodeGrid::new(mb_cols_of(p_source), mb_rows_total);
    // Vector search + difference coding under the picture's UMV mode.
    let search_mb =
        |src: &YuvFrame, mb_col: usize, mb_row: usize, predictor: MotionVector| match core.umv {
            UmvMode::Off => estimate_motion(
                src,
                reference,
                mb_col,
                mb_row,
                predictor,
                core.search_half,
                lambda,
            ),
            UmvMode::Wrap => crate::encoder_motion::estimate_motion_umv(
                src,
                reference,
                mb_col,
                mb_row,
                predictor,
                core.search_half,
                lambda,
            ),
            UmvMode::Plus => crate::encoder_motion::estimate_motion_umv_plus(
                src,
                reference,
                mb_col,
                mb_row,
                predictor,
                core.search_half,
                lambda,
            ),
        };
    let diff_for = |mv: MotionVector, predictor: MotionVector| -> Mvd {
        match core.umv {
            UmvMode::Off => mvd_for(mv, predictor),
            // `estimate_motion_umv` only admits §D.2-reachable vectors.
            UmvMode::Wrap => crate::encoder_motion::umv_mvd_for(mv, predictor)
                .expect("UMV search admits only §D.2-reachable vectors"),
            UmvMode::Plus => Mvd {
                dx_half: (mv.dx_half - predictor.dx_half) as i16,
                dy_half: (mv.dy_half - predictor.dy_half) as i16,
            },
        }
    };
    let mut stats = ImprovedPbStats::default();

    let lw = p_source.luma_width;
    let lh = p_source.luma_height;
    let cw = p_source.chroma_width();
    let ch = p_source.chroma_height();
    let mb_cols = lw / 16;
    let mb_rows = lh / 16;
    let is_intra_mb =
        |idx: usize| -> bool { core.intra_refresh > 0 && idx.is_multiple_of(core.intra_refresh) };

    // ---- Header. ------------------------------------------------------
    let mut w = BitWriter::new();
    match flavour {
        PbFlavour::AnnexG { .. } => write_picture_header(
            &mut w,
            fmt,
            quant,
            tr_p,
            /* is_inter */ true,
            PtypeFlags {
                advanced_prediction: ap,
                umv: core.umv == UmvMode::Wrap,
                ..PtypeFlags::default()
            },
            Some((core.trb, core.dbquant)),
        ),
        PbFlavour::AnnexM { .. } => write_plus_picture_header(
            &mut w,
            fmt,
            quant,
            tr_p,
            /* is_inter */ true,
            PlusModes {
                advanced_prediction: ap,
                umv: core.umv == UmvMode::Plus,
                advanced_intra: core.aic,
                improved_pb: Some((core.trb, core.dbquant)),
                slice_structured: sss,
                rounding_type: core.rcontrol != 0,
                ..PlusModes::default()
            },
        )?,
    }
    // §K.2.2 — the reduced first-slice header (MBA 0) follows the
    // picture header.
    if let Some(ctx) = slice_ctx.as_ref() {
        crate::slice_header::write_first_slice_header(&mut w, ctx, 0, None)?;
    }
    let gfid = tr_p & 0b11;

    // ---- Pass 1 (Advanced Prediction): the whole §F.2 vector field,
    // so every §F.3 remote vector is known before any prediction. An
    // INTRA-refresh macroblock carries one 16 × 16 vector (§G.2) whose
    // predictor is the §F.2 block-1 form. -------------------------------
    let mut field: Vec<Mb4Mv> = Vec::new();
    let mut mvds_field: Vec<[Mvd; 4]> = Vec::new();
    if ap {
        let mut grid4 = if core.slice_rows > 0 {
            Mv4Grid::with_row_segments(mb_cols, mb_rows, core.slice_rows)
        } else {
            Mv4Grid::new(mb_cols, mb_rows)
        };
        for mb_row in 0..mb_rows {
            for mb_col in 0..mb_cols {
                let idx = mb_row * mb_cols + mb_col;
                let mut cur: Mb4Mv = [MotionVector::new(0, 0); 4];
                let mut mvds = [Mvd {
                    dx_half: 0,
                    dy_half: 0,
                }; 4];
                if is_intra_mb(idx) {
                    // The INTRA macroblock's B-purpose vector is zero
                    // (see `intra_refresh`); MVD still codes it against
                    // the §F.2 block-1 predictor.
                    let predictor = grid4.predict_block(mb_col, mb_row, LumaBlockIndex::B1, &cur);
                    mvds[0] = diff_for(MotionVector::new(0, 0), predictor);
                } else {
                    for &blk in &LumaBlockIndex::ALL {
                        let blk_i = blk.index();
                        let bx = mb_col * 16 + (blk_i % 2) * 8;
                        let by = mb_row * 16 + (blk_i / 2) * 8;
                        let predictor = grid4.predict_block(mb_col, mb_row, blk, &cur);
                        let mv = if core.umv == UmvMode::Wrap {
                            // §D.2 pair rule per block: only vectors
                            // Table-14-codable from this block's §F.2
                            // predictor.
                            crate::encoder_motion::estimate_block_motion_umv(
                                p_source,
                                reference,
                                bx,
                                by,
                                predictor,
                                core.search_half,
                                lambda,
                            )
                        } else if core.umv == UmvMode::Plus {
                            crate::encoder_motion::estimate_block_motion_umv_plus(
                                p_source,
                                reference,
                                bx,
                                by,
                                predictor,
                                core.search_half,
                                lambda,
                            )
                        } else {
                            estimate_block_motion(
                                p_source,
                                reference,
                                bx,
                                by,
                                predictor,
                                core.search_half,
                                lambda,
                            )
                        };
                        cur[blk_i] = mv;
                        mvds[blk_i] = diff_for(mv, predictor);
                    }
                }
                grid4.set(mb_col, mb_row, cur);
                field.push(cur);
                mvds_field.push(mvds);
            }
        }
    }

    // ---- Pass 2: prediction, residuals, B-part, emission. ---------------
    let y_ref = RefPlane::new(&reference.y, lw, lh);
    let prev_cb = RefPlane::new(&reference.cb, cw, ch);
    let prev_cr = RefPlane::new(&reference.cr, cw, ch);
    let mut grid = if core.slice_rows > 0 {
        MvGrid::with_gob_headers(mb_cols, mb_rows, core.slice_rows)
    } else {
        MvGrid::new(mb_cols, mb_rows)
    };

    for mb_row in 0..mb_rows {
        // §K.2 — SSTUF + SSC slice header at every slice start after
        // the first (reduced-header) slice.
        if let Some(ctx) = slice_ctx.as_ref()
            && mb_row > 0
            && mb_row % core.slice_rows == 0
        {
            let mba = (mb_row * mb_cols) as u32;
            crate::slice_header::write_slice_layer(&mut w, ctx, mba, quant, gfid, None)?;
        }
        // §M.2.2 — the forward-vector predictor restarts at the far-left
        // edge of every macroblock row (the decoder resets it per row,
        // and at every slice's left edge — the same rows here).
        let mut left_forward: Option<MotionVector> = None;
        // §K.1 rule 3 / §F.3 — a neighbour in another slice is outside
        // for OBMC purposes (remote = current vector).
        let above_in_slice = core.slice_rows == 0 || mb_row % core.slice_rows != 0;
        for mb_col in 0..mb_cols {
            let idx = mb_row * mb_cols + mb_col;
            let mb_x = mb_col * 16;
            let mb_y = mb_row * 16;
            let intra = is_intra_mb(idx);

            // ---- Vectors (and their MVDs) for this macroblock. --------
            let (mvs4, mvds): (Mb4Mv, [Mvd; 4]) = if ap {
                (field[idx], mvds_field[idx])
            } else {
                let predictor = grid.predict(mb_col, mb_row);
                let mv = if intra {
                    MotionVector::new(0, 0)
                } else {
                    search_mb(p_source, mb_col, mb_row, predictor)
                };
                let zero = Mvd {
                    dx_half: 0,
                    dy_half: 0,
                };
                ([mv; 4], [diff_for(mv, predictor), zero, zero, zero])
            };
            let single_mv = mvs4[0];
            let all_zero = mvs4.iter().all(|m| m.dx_half == 0 && m.dy_half == 0);

            // ---- P-part. ----------------------------------------------
            let src = extract_macroblock(p_source, mb_col, mb_row);
            // §I.3 segment id: the slice index under Annex K, else 0.
            let segment = mb_row.checked_div(core.slice_rows).unwrap_or(0) as u32;
            let p_part = if intra && core.aic {
                intra_p_part_aic(
                    &aic_grid,
                    &src,
                    crate::encoder::AicParams {
                        quant,
                        chroma_quant: quant,
                        modified_quant: false,
                        segment,
                    },
                    mb_col,
                    mb_row,
                )
            } else if intra {
                intra_p_part(&src, quant)
            } else if ap {
                let above = (mb_row > 0 && above_in_slice).then(|| field[idx - mb_cols]);
                let left = (mb_col > 0).then(|| field[idx - 1]);
                let right = (mb_col + 1 < mb_cols).then(|| field[idx + 1]);
                inter_p_part_ap(
                    &src,
                    &y_ref,
                    reference,
                    mb_col,
                    mb_row,
                    &mvs4,
                    above,
                    left,
                    right,
                    quant,
                    core.rcontrol,
                )
            } else {
                inter_p_part_single(
                    &src,
                    reference,
                    mb_col,
                    mb_row,
                    single_mv,
                    quant,
                    core.rcontrol,
                )
            };

            // ---- B-part. ----------------------------------------------
            let planes = PbBReferencePlanes {
                prev_y: y_ref,
                prev_cb,
                prev_cr,
                prec_y: RefPlane::new(&p_part.prec_y, 16, 16),
                prec_cb: RefPlane::new(&p_part.prec_cb, 8, 8),
                prec_cr: RefPlane::new(&p_part.prec_cr, 8, 8),
            };
            let b_src = extract_macroblock(b_source, mb_col, mb_row);
            // §M.2.1 — only the bidirectional row carries MVD for an
            // INTRA macroblock; a forward / backward INTRA macroblock
            // would transmit no vector at all, so its neighbours (which
            // already used the estimated vector as §6.1.1 candidate and
            // §G.2 OBMC remote) would desynchronise. Keep INTRA
            // macroblocks bidirectional under Annex M.
            let b_part = decide_b_part(
                &planes,
                &b_src,
                mb_x,
                mb_y,
                &mvs4,
                trb,
                trd,
                bquant,
                lambda,
                flavour,
                left_forward,
                b_source,
                reference,
                mb_col,
                mb_row,
                /* bidirectional_only */ intra,
                core.umv,
            );

            // ---- Encoder-side reconstruction (PREC and the B-part). ---
            blit_recon(
                &mut p_recon,
                mb_col,
                mb_row,
                &p_part.prec_y,
                &p_part.prec_cb,
                &p_part.prec_cr,
            );
            {
                let mut by = [0u8; 256];
                for blk in 0..4 {
                    let (ox, oy) = ((blk % 2) * 8, (blk / 2) * 8);
                    let mut pred = [0u8; COEFFS_PER_BLOCK];
                    for j in 0..8 {
                        pred[j * 8..j * 8 + 8]
                            .copy_from_slice(&b_part.pred.luma[oy + j][ox..ox + 8]);
                    }
                    blit_prec_luma(
                        &mut by,
                        blk,
                        &recon_inter_block(&b_part.blocks[blk], &pred, bquant),
                    );
                }
                let flat8 = |p: &[[u8; 8]; 8]| -> [u8; COEFFS_PER_BLOCK] {
                    let mut o = [0u8; COEFFS_PER_BLOCK];
                    for j in 0..8 {
                        o[j * 8..j * 8 + 8].copy_from_slice(&p[j]);
                    }
                    o
                };
                let bcb = recon_inter_block(&b_part.blocks[4], &flat8(&b_part.pred.cb), bquant);
                let bcr = recon_inter_block(&b_part.blocks[5], &flat8(&b_part.pred.cr), bquant);
                blit_recon(&mut b_recon, mb_col, mb_row, &by, &bcb, &bcr);
            }

            // ---- Skip / emit. -----------------------------------------
            // A skipped macroblock (COD = 1) carries no MODB: its B-part
            // is the zero-vector bidirectional prediction, which has no
            // forward vector, so the §M.2.2 predictor of the macroblock
            // to its right resets to zero. Under Advanced
            // Prediction the decoder still OBMC-blends a skipped
            // macroblock (§5.3.1 NOTE) with zero vectors — which is what
            // the field holds when `all_zero`.
            let b_needs_nothing = !b_part.any
                && b_part.mvdb.is_none()
                && matches!(b_part.mode, BpbCodingMode::Bidirectional);
            if !intra && !p_part.any_coeffs && all_zero && b_needs_nothing {
                crate::encoder_mb::encode_skipped_macroblock(&mut w);
                grid.set_zero_candidate(mb_col, mb_row);
                aic_grid.record_non_intra(mb_col, mb_row, segment);
                stats.skipped += 1;
                left_forward = None;
                continue;
            }

            // COD = 0; MCBPC (Table 8).
            w.write_bit(false);
            let mb_type = if intra {
                MbType::Intra
            } else if ap {
                MbType::Inter4V
            } else {
                MbType::Inter
            };
            write_mcbpc_p(&mut w, mb_type, p_part.cbpc)?;
            // §I.2 — INTRA_MODE between MCBPC and MODB (Figure I.1).
            if let Some(plan) = p_part.aic_plan.as_ref() {
                crate::aic::write_intra_mode(&mut w, plan.mode);
            }

            // MODB (+ CBPB).
            match flavour {
                PbFlavour::AnnexG { .. } => {
                    // §5.3.3 Table 11: "0" none, "10" MVDB, "11" CBPB + MVDB.
                    if b_part.any {
                        w.write_bits(0b11, 2);
                    } else if b_part.mvdb.is_some() {
                        w.write_bits(0b10, 2);
                    } else {
                        w.write_bit(false);
                    }
                }
                PbFlavour::AnnexM { .. } => {
                    write_modb_annex_m(&mut w, ModbAnnexM::from_parts(b_part.mode, b_part.any));
                }
            }
            if b_part.any {
                // §5.3.4 CBPB — block N lights bit (6 − N).
                let mut cbpb = 0u8;
                for (blk, e) in b_part.blocks.iter().enumerate() {
                    if e.has_coeffs {
                        cbpb |= 1 << (6 - (blk + 1));
                    }
                }
                w.write_bits(cbpb as u32, 6);
                stats.b_residual += 1;
            }

            // §5.3.5 CBPY (INTRA orientation for INTRA, complement for INTER).
            let cbpy_wire = if intra {
                p_part.cbpy
            } else {
                p_part.cbpy ^ 0b1111
            };
            write_cbpy(&mut w, cbpy_wire)?;

            // §5.3.7 MVD — every INTER macroblock; an INTRA macroblock
            // in PB-frames mode too (§G.2), except Annex M's forward /
            // backward rows (§M.2.1).
            let intra_mvd = match flavour {
                PbFlavour::AnnexG { .. } => true,
                PbFlavour::AnnexM { .. } => matches!(b_part.mode, BpbCodingMode::Bidirectional),
            };
            let d3 = core.umv == UmvMode::Plus;
            if !intra || intra_mvd {
                write_mv_pair(&mut w, mvds[0], d3)?;
            }
            // §5.3.8 MVD2-4 — INTER4V only ("never used for INTRA", §G.2).
            if ap && !intra {
                for d in mvds.iter().skip(1) {
                    write_mv_pair(&mut w, *d, d3)?;
                }
            }
            // §5.3.9 MVDB — "coded in the same way as MVD" (§M.2.2), so
            // Table D.3 under UMV+ too.
            if let Some(d) = b_part.mvdb {
                write_mv_pair(&mut w, d, d3)?;
                if d.dx_half != 0 || d.dy_half != 0 {
                    stats.mvdb += 1;
                }
            }
            match b_part.mode {
                BpbCodingMode::Forward => {
                    left_forward = b_part.forward_mv;
                    stats.forward += 1;
                }
                BpbCodingMode::Backward => {
                    left_forward = None;
                    stats.backward += 1;
                }
                // §M.2.2 — only a forward-predicted left neighbour
                // supplies the predictor; a bidirectional one resets it
                // to zero (no older forward vector is carried along).
                BpbCodingMode::Bidirectional => {
                    left_forward = None;
                    stats.bidirectional += 1;
                }
            }
            if intra {
                stats.intra += 1;
            }

            // §G.3 — six P-blocks, then six B-blocks.
            if let Some(plan) = p_part.aic_plan.as_ref() {
                plan.write_blocks(&mut w)?;
                crate::encoder::commit_macroblock_aic(&mut aic_grid, plan, mb_col, mb_row, segment);
            } else if intra {
                aic_grid.record_non_intra(mb_col, mb_row, segment);
                for e in p_part.luma_intra.iter().flatten() {
                    write_intra_block(&mut w, e.dc_level, &e.scan, e.has_ac)?;
                }
                let cb = p_part.cb_intra.as_ref().expect("intra chroma");
                let cr = p_part.cr_intra.as_ref().expect("intra chroma");
                write_intra_block(&mut w, cb.dc_level, &cb.scan, cb.has_ac)?;
                write_intra_block(&mut w, cr.dc_level, &cr.scan, cr.has_ac)?;
            } else {
                aic_grid.record_non_intra(mb_col, mb_row, segment);
                for e in p_part.luma_inter.iter().flatten() {
                    if e.has_coeffs {
                        write_inter_block_coeffs(&mut w, &e.scan)?;
                    }
                }
                for e in [&p_part.cb_inter, &p_part.cr_inter].into_iter().flatten() {
                    if e.has_coeffs {
                        write_inter_block_coeffs(&mut w, &e.scan)?;
                    }
                }
            }
            for e in b_part.blocks.iter() {
                if e.has_coeffs {
                    write_inter_block_coeffs(&mut w, &e.scan)?;
                }
            }

            // In PB-frames mode an INTRA macroblock's vector is a
            // §6.1.1 candidate for its neighbours (rule-1 exception).
            grid.set_inter(mb_col, mb_row, single_mv);
        }
    }

    w.align_to_byte_zero();
    Ok(PbCoreOutput {
        bytes: w.finish(),
        stats,
        p_recon,
        b_recon,
    })
}

/// One MVD pair: two Table 14 codewords, or the Table D.3 pair (with
/// its §D.2 emulation-prevention bit) under UMV with PLUSPTYPE.
fn write_mv_pair(w: &mut BitWriter, mvd: Mvd, table_d3: bool) -> Result<()> {
    if table_d3 {
        crate::encoder_vlc::write_mvd_pair_d3(w, mvd)
    } else {
        write_mvd_component(w, mvd.dx_half)?;
        write_mvd_component(w, mvd.dy_half)
    }
}

/// Copy one reconstructed macroblock (16 × 16 luma in raster order,
/// two 8 × 8 chroma blocks) into `frame`.
fn blit_recon(
    frame: &mut YuvFrame,
    mb_col: usize,
    mb_row: usize,
    y: &[u8; 256],
    cb: &[u8; COEFFS_PER_BLOCK],
    cr: &[u8; COEFFS_PER_BLOCK],
) {
    let lw = frame.luma_width;
    let cw = frame.chroma_width();
    for j in 0..16 {
        let at = (mb_row * 16 + j) * lw + mb_col * 16;
        frame.y[at..at + 16].copy_from_slice(&y[j * 16..j * 16 + 16]);
    }
    for j in 0..8 {
        let at = (mb_row * 8 + j) * cw + mb_col * 8;
        frame.cb[at..at + 8].copy_from_slice(&cb[j * 8..j * 8 + 8]);
        frame.cr[at..at + 8].copy_from_slice(&cr[j * 8..j * 8 + 8]);
    }
}

fn mb_cols_of(frame: &YuvFrame) -> usize {
    frame.luma_width / 16
}

fn to_i16(pred: &[u8; COEFFS_PER_BLOCK]) -> [i16; COEFFS_PER_BLOCK] {
    let mut out = [0i16; COEFFS_PER_BLOCK];
    for (d, &p) in out.iter_mut().zip(pred.iter()) {
        *d = p as i16;
    }
    out
}

/// The decoder-side reconstruction of one INTER block: prediction plus
/// the dequantised residual (§6.3.1) and the §6.3.2 clip, or the bare
/// prediction when no coefficient survived.
fn recon_inter_block(
    enc: &EncodedInterBlock,
    pred: &[u8; COEFFS_PER_BLOCK],
    quant: u8,
) -> [u8; COEFFS_PER_BLOCK] {
    if enc.has_coeffs {
        let block = H263Block {
            coefficients: enc.scan,
            tcoef_event_count: 0,
            had_intradc: false,
        };
        crate::reconstruct_inter_block_with_prediction(&block, quant, pred)
    } else {
        *pred
    }
}

/// The decoder-side reconstruction of one INTRA block (§6.2 / §6.3.2).
fn recon_intra_block(enc: &EncodedIntraBlock, quant: u8) -> [u8; COEFFS_PER_BLOCK] {
    let mut coefficients = enc.scan;
    coefficients[0] = enc.dc_level;
    let block = H263Block {
        coefficients,
        tcoef_event_count: 0,
        had_intradc: true,
    };
    crate::reconstruct_intra_block(&block, quant)
}

fn blit_prec_luma(prec_y: &mut [u8; 256], blk: usize, samples: &[u8; COEFFS_PER_BLOCK]) {
    let ox = (blk % 2) * 8;
    let oy = (blk / 2) * 8;
    for j in 0..8 {
        prec_y[(oy + j) * 16 + ox..(oy + j) * 16 + ox + 8]
            .copy_from_slice(&samples[j * 8..j * 8 + 8]);
    }
}

fn intra_p_part(src: &crate::encoder_mb::MacroblockSamples, quant: u8) -> PPart {
    let mut prec_y = [0u8; 256];
    let mut luma_intra: [Option<EncodedIntraBlock>; 4] = [None, None, None, None];
    let mut cbpy = 0u8;
    for (blk, slot) in luma_intra.iter_mut().enumerate() {
        let e = encode_intra_block(&src.luma[blk], quant);
        if e.has_ac {
            cbpy |= 1 << (3 - blk);
        }
        blit_prec_luma(&mut prec_y, blk, &recon_intra_block(&e, quant));
        *slot = Some(e);
    }
    let cb = encode_intra_block(&src.cb, quant);
    let cr = encode_intra_block(&src.cr, quant);
    let mut cbpc = 0u8;
    if cb.has_ac {
        cbpc |= 0b10;
    }
    if cr.has_ac {
        cbpc |= 0b01;
    }
    let prec_cb = recon_intra_block(&cb, quant);
    let prec_cr = recon_intra_block(&cr, quant);
    PPart {
        aic_plan: None,
        luma_inter: [None, None, None, None],
        luma_intra,
        cb_inter: None,
        cr_inter: None,
        cb_intra: Some(cb),
        cr_intra: Some(cr),
        prec_y,
        prec_cb,
        prec_cr,
        cbpc,
        cbpy,
        any_coeffs: true,
    }
}

/// §I — the INTRA-refresh macroblock coded with Advanced INTRA Coding:
/// planned against the encoder's reconstructed INTRA neighbours, PREC
/// being the §I reconstruction.
fn intra_p_part_aic(
    grid: &crate::encoder::AicEncodeGrid,
    src: &crate::encoder_mb::MacroblockSamples,
    params: crate::encoder::AicParams,
    mb_col: usize,
    mb_row: usize,
) -> PPart {
    let plan = crate::encoder::plan_choose_macroblock_aic(grid, src, params, None, mb_col, mb_row);
    let (prec_y, prec_cb, prec_cr) = plan.reconstruct_samples();
    let (cbpc, cbpy) = (plan.cbpc(), plan.cbpy());
    PPart {
        aic_plan: Some(plan),
        luma_inter: [None, None, None, None],
        luma_intra: [None, None, None, None],
        cb_inter: None,
        cr_inter: None,
        cb_intra: None,
        cr_intra: None,
        prec_y,
        prec_cb,
        prec_cr,
        cbpc,
        cbpy,
        any_coeffs: true,
    }
}

fn finish_inter_p_part(
    src: &crate::encoder_mb::MacroblockSamples,
    luma_pred: [[u8; COEFFS_PER_BLOCK]; 4],
    cb_pred: [u8; COEFFS_PER_BLOCK],
    cr_pred: [u8; COEFFS_PER_BLOCK],
    quant: u8,
) -> PPart {
    let mut prec_y = [0u8; 256];
    let mut luma_inter: [Option<EncodedInterBlock>; 4] = [None, None, None, None];
    let mut cbpy = 0u8;
    let mut any = false;
    for blk in 0..4 {
        let e = encode_inter_block(
            &residual_of(&src.luma[blk], &to_i16(&luma_pred[blk])),
            quant,
        );
        if e.has_coeffs {
            cbpy |= 1 << (3 - blk);
            any = true;
        }
        blit_prec_luma(
            &mut prec_y,
            blk,
            &recon_inter_block(&e, &luma_pred[blk], quant),
        );
        luma_inter[blk] = Some(e);
    }
    let cb = encode_inter_block(&residual_of(&src.cb, &to_i16(&cb_pred)), quant);
    let cr = encode_inter_block(&residual_of(&src.cr, &to_i16(&cr_pred)), quant);
    let mut cbpc = 0u8;
    if cb.has_coeffs {
        cbpc |= 0b10;
        any = true;
    }
    if cr.has_coeffs {
        cbpc |= 0b01;
        any = true;
    }
    let prec_cb = recon_inter_block(&cb, &cb_pred, quant);
    let prec_cr = recon_inter_block(&cr, &cr_pred, quant);
    PPart {
        aic_plan: None,
        luma_inter,
        luma_intra: [None, None, None, None],
        cb_inter: Some(cb),
        cr_inter: Some(cr),
        cb_intra: None,
        cr_intra: None,
        prec_y,
        prec_cb,
        prec_cr,
        cbpc,
        cbpy,
        any_coeffs: any,
    }
}

fn inter_p_part_single(
    src: &crate::encoder_mb::MacroblockSamples,
    reference: &YuvFrame,
    mb_col: usize,
    mb_row: usize,
    mv: MotionVector,
    quant: u8,
    rcontrol: i32,
) -> PPart {
    let lw = reference.luma_width;
    let lh = reference.luma_height;
    let cw = reference.chroma_width();
    let ch = reference.chroma_height();
    let mut luma_pred = [[0u8; COEFFS_PER_BLOCK]; 4];
    for (blk, pred) in luma_pred.iter_mut().enumerate() {
        let bx = mb_col * 16 + (blk % 2) * 8;
        let by = mb_row * 16 + (blk / 2) * 8;
        *pred = motion_compensated_block_rc(&reference.y, lw, lh, bx, by, mv, rcontrol);
    }
    let chroma_vec = chroma_mv(mv);
    let cb_pred = motion_compensated_block_rc(
        &reference.cb,
        cw,
        ch,
        mb_col * 8,
        mb_row * 8,
        chroma_vec,
        rcontrol,
    );
    let cr_pred = motion_compensated_block_rc(
        &reference.cr,
        cw,
        ch,
        mb_col * 8,
        mb_row * 8,
        chroma_vec,
        rcontrol,
    );
    finish_inter_p_part(src, luma_pred, cb_pred, cr_pred, quant)
}

/// §F.3 OBMC prediction of an INTER4V P-macroblock. Every macroblock of
/// the field carries vectors — INTER4V ones their four, INTRA-refresh
/// ones the §G.2 B-purpose vector (four copies), which per §G.2 is
/// exactly the remote the neighbours use ("the remote 'INTRA' motion
/// vector is used") — so a present neighbour always contributes its
/// actual cell vector and only an off-picture neighbour falls back to
/// the current vector.
#[allow(clippy::too_many_arguments)]
fn inter_p_part_ap(
    src: &crate::encoder_mb::MacroblockSamples,
    y_ref: &RefPlane<'_>,
    reference: &YuvFrame,
    mb_col: usize,
    mb_row: usize,
    cur: &Mb4Mv,
    above: Option<Mb4Mv>,
    left: Option<Mb4Mv>,
    right: Option<Mb4Mv>,
    quant: u8,
    rcontrol: i32,
) -> PPart {
    let remote = |nb: Option<Mb4Mv>, cell: LumaBlockIndex| -> RemoteMv {
        match nb {
            Some(m) => RemoteMv::Vector(m[cell.index()]),
            None => RemoteMv::Current,
        }
    };
    let tags = |blk: LumaBlockIndex| -> (RemoteMv, RemoteMv, RemoteMv, RemoteMv) {
        match blk {
            LumaBlockIndex::B1 => (
                remote(above, LumaBlockIndex::B3),
                RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                remote(left, LumaBlockIndex::B2),
                RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
            ),
            LumaBlockIndex::B2 => (
                remote(above, LumaBlockIndex::B4),
                RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
                RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                remote(right, LumaBlockIndex::B1),
            ),
            LumaBlockIndex::B3 => (
                RemoteMv::Vector(cur[LumaBlockIndex::B1.index()]),
                RemoteMv::Current,
                remote(left, LumaBlockIndex::B4),
                RemoteMv::Vector(cur[LumaBlockIndex::B4.index()]),
            ),
            LumaBlockIndex::B4 => (
                RemoteMv::Vector(cur[LumaBlockIndex::B2.index()]),
                RemoteMv::Current,
                RemoteMv::Vector(cur[LumaBlockIndex::B3.index()]),
                remote(right, LumaBlockIndex::B3),
            ),
        }
    };
    let mut luma_pred = [[0u8; COEFFS_PER_BLOCK]; 4];
    for &blk in &LumaBlockIndex::ALL {
        let blk_i = blk.index();
        let bx = mb_col * 16 + (blk_i % 2) * 8;
        let by = mb_row * 16 + (blk_i / 2) * 8;
        let (r_top, r_bot, s_left, s_right) = tags(blk);
        luma_pred[blk_i] = obmc_predict_block(
            y_ref, bx, by, cur[blk_i], r_top, r_bot, s_left, s_right, rcontrol,
        );
    }
    // §F.2 chroma: sum-of-four / Table F.1 vector, plain half-pel
    // motion compensation (no OBMC).
    let chroma_vec = chroma_mv_4mv(cur);
    let cw = reference.chroma_width();
    let ch = reference.chroma_height();
    let cb_pred = motion_compensated_block_rc(
        &reference.cb,
        cw,
        ch,
        mb_col * 8,
        mb_row * 8,
        chroma_vec,
        rcontrol,
    );
    let cr_pred = motion_compensated_block_rc(
        &reference.cr,
        cw,
        ch,
        mb_col * 8,
        mb_row * 8,
        chroma_vec,
        rcontrol,
    );
    finish_inter_p_part(src, luma_pred, cb_pred, cr_pred, quant)
}

/// Luminance SAD of the B-source macroblock against a candidate B
/// prediction.
fn b_sad(b_src: &crate::encoder_mb::MacroblockSamples, pred: &PbBMacroblockPrediction) -> u32 {
    let mut sad = 0u32;
    for blk in 0..4 {
        let ox = (blk % 2) * 8;
        let oy = (blk / 2) * 8;
        for j in 0..8 {
            for i in 0..8 {
                let sv = b_src.luma[blk][j * 8 + i] as i32;
                let pv = pred.luma[oy + j][ox + i] as i32;
                sad += (sv - pv).unsigned_abs();
            }
        }
    }
    sad
}

/// Code the B-residual of `pred` at BQUANT.
fn b_residual(
    b_src: &crate::encoder_mb::MacroblockSamples,
    pred: &PbBMacroblockPrediction,
    bquant: u8,
) -> ([EncodedInterBlock; 6], bool) {
    let mut out: Vec<EncodedInterBlock> = Vec::with_capacity(6);
    for blk in 0..4 {
        let ox = (blk % 2) * 8;
        let oy = (blk / 2) * 8;
        let mut pred_i16 = [0i16; COEFFS_PER_BLOCK];
        for j in 0..8 {
            for i in 0..8 {
                pred_i16[j * 8 + i] = pred.luma[oy + j][ox + i] as i16;
            }
        }
        out.push(encode_inter_block(
            &residual_of(&b_src.luma[blk], &pred_i16),
            bquant,
        ));
    }
    let mut cb_pred = [0i16; COEFFS_PER_BLOCK];
    let mut cr_pred = [0i16; COEFFS_PER_BLOCK];
    for j in 0..8 {
        for i in 0..8 {
            cb_pred[j * 8 + i] = pred.cb[j][i] as i16;
            cr_pred[j * 8 + i] = pred.cr[j][i] as i16;
        }
    }
    out.push(encode_inter_block(
        &residual_of(&b_src.cb, &cb_pred),
        bquant,
    ));
    out.push(encode_inter_block(
        &residual_of(&b_src.cr, &cr_pred),
        bquant,
    ));
    let any = out.iter().any(|e| e.has_coeffs);
    let blocks: [EncodedInterBlock; 6] = out.try_into().expect("six blocks");
    (blocks, any)
}

/// The per-macroblock B-part decision for either flavour.
#[allow(clippy::too_many_arguments)]
fn decide_b_part(
    planes: &PbBReferencePlanes<'_>,
    b_src: &crate::encoder_mb::MacroblockSamples,
    mb_x: usize,
    mb_y: usize,
    mvs4: &Mb4Mv,
    trb: i32,
    trd: i32,
    bquant: u8,
    lambda: u32,
    flavour: PbFlavour,
    left_forward: Option<MotionVector>,
    b_source: &YuvFrame,
    reference: &YuvFrame,
    mb_col: usize,
    mb_row: usize,
    bidirectional_only: bool,
    umv: UmvMode,
) -> BPart {
    // The decoder-side §G.4 composition: one MVDB delta per
    // macroblock, added to every luminance block's scaled vector.
    let predict_at = |delta: Option<Mvd>| {
        let deltas =
            crate::pb_layer::pb_b_effective_deltas(mvs4, delta, trb, trd, umv == UmvMode::Wrap);
        crate::pb_layer::pb_b_predict_macroblock_deltas(
            planes,
            mb_x,
            mb_y,
            mvs4,
            &deltas,
            trb,
            trd,
            RCONTROL_DEFAULT,
        )
    };
    let mut best_pred = predict_at(None);
    let mut best_cost = b_sad(b_src, &best_pred) + lambda;
    let mut mode = BpbCodingMode::Bidirectional;
    let mut mvdb: Option<Mvd> = None;
    let mut forward_mv: Option<MotionVector> = None;

    match flavour {
        PbFlavour::AnnexG { b_search_half } if b_search_half > 0 => {
            // §5.3.9 / §G.4 — MVDB refinement: search a small window of
            // delta vectors, keeping only deltas that are Table 14
            // codable and whose per-component MVF stays inside the §G.4
            // permitted range for every block (the in-range value is
            // what any decoder selects from the Table 14 pair, since
            // the pair mate sits exactly 64 half-pels away — outside a
            // range only 64 wide). The zero delta competes with a flat
            // bias so static content never pays the MODB / MVDB bits.
            // Outside UMV the in-range MVF is what any decoder selects
            // from the Table 14 pair (the mate sits 64 half-pels away,
            // outside a range only 64 wide). Under baseline UMV the
            // delta is the first-column value applied once per
            // macroblock; candidates are further restricted (below) to
            // those a per-block §D.2 pair resolution would also pick,
            // so both readings of the clause decode the stream alike.
            let mvf_in_range = |p_comp: i32, delta: i32| -> bool {
                umv == UmvMode::Wrap || (-32..=31).contains(&((trb * p_comp) / trd + delta))
            };
            let bw = b_search_half;
            for dy in -bw..=bw {
                for dx in -bw..=bw {
                    if (dx == 0 && dy == 0)
                        || !(-32..=31).contains(&dx)
                        || !(-32..=31).contains(&dy)
                        || !mvs4
                            .iter()
                            .all(|m| mvf_in_range(m.dx_half, dx) && mvf_in_range(m.dy_half, dy))
                    {
                        continue;
                    }
                    let delta = Mvd {
                        dx_half: dx as i16,
                        dy_half: dy as i16,
                    };
                    if umv == UmvMode::Wrap
                        && !crate::pb_layer::pb_mvdb_first_column_is_unambiguous(
                            mvs4, delta, trb, trd,
                        )
                    {
                        continue;
                    }
                    let cand = predict_at(Some(delta));
                    let cost = b_sad(b_src, &cand)
                        + lambda * (dx.unsigned_abs() + dy.unsigned_abs())
                        + 4 * lambda;
                    if cost < best_cost {
                        best_cost = cost;
                        mvdb = Some(delta);
                        best_pred = cand;
                    }
                }
            }
        }
        PbFlavour::AnnexG { .. } => {}
        PbFlavour::AnnexM { .. } if bidirectional_only => {}
        PbFlavour::AnnexM {
            forward_search_half,
            allow_backward,
        } => {
            // §M.2.2 — forward: one 16 × 16 vector into the previous
            // reference, coded against the left-neighbour predictor.
            if forward_search_half > 0 {
                let fwd_predictor = left_forward.unwrap_or_default();
                // §M.2.2 — "VLC coded in the same way as … (MVD)": Table
                // D.3 single-valued differences over the UUI range under
                // UMV+ (the vector may then reach over the picture
                // boundary, §D.1), the §6.1.1 wrap otherwise.
                let (fwd_mv, d) = if umv == UmvMode::Plus {
                    let mv = crate::encoder_motion::estimate_motion_umv_plus(
                        b_source,
                        reference,
                        mb_col,
                        mb_row,
                        fwd_predictor,
                        forward_search_half,
                        lambda,
                    );
                    (
                        mv,
                        Mvd {
                            dx_half: (mv.dx_half - fwd_predictor.dx_half) as i16,
                            dy_half: (mv.dy_half - fwd_predictor.dy_half) as i16,
                        },
                    )
                } else {
                    let mv = estimate_motion(
                        b_source,
                        reference,
                        mb_col,
                        mb_row,
                        fwd_predictor,
                        forward_search_half,
                        lambda,
                    );
                    (mv, mvd_for(mv, fwd_predictor))
                };
                let fwd_pred = forward_prediction(planes, mb_x, mb_y, fwd_mv);
                let bits =
                    3 + (d.dx_half.unsigned_abs() as u32) + (d.dy_half.unsigned_abs() as u32) + 2;
                let cost = b_sad(b_src, &fwd_pred) + lambda * bits;
                if cost < best_cost {
                    best_cost = cost;
                    mode = BpbCodingMode::Forward;
                    best_pred = fwd_pred;
                    mvdb = Some(d);
                    forward_mv = Some(fwd_mv);
                }
            }
            // §M.2.3 — backward: the prediction is PREC itself.
            if allow_backward {
                let bwd_pred = backward_prediction(planes);
                let cost = b_sad(b_src, &bwd_pred) + lambda * 5;
                if cost < best_cost {
                    mode = BpbCodingMode::Backward;
                    best_pred = bwd_pred;
                    mvdb = None;
                    forward_mv = None;
                }
            }
        }
    }

    let (blocks, any) = b_residual(b_src, &best_pred, bquant);
    // Annex G: a lit CBPB without a searched delta still needs MODB
    // "11" → a zero MVDB on the wire.
    if matches!(flavour, PbFlavour::AnnexG { .. }) && any && mvdb.is_none() {
        mvdb = Some(Mvd {
            dx_half: 0,
            dy_half: 0,
        });
    }
    BPart {
        mode,
        mvdb,
        forward_mv,
        blocks,
        any,
        pred: best_pred,
    }
}

/// §M.2.2 — the forward-only BPB prediction: the four luma blocks
/// fetched with one 16 × 16 vector from the previous reference and the
/// two chroma blocks with its Table-18 chroma vector.
fn forward_prediction(
    planes: &PbBReferencePlanes<'_>,
    mb_x: usize,
    mb_y: usize,
    forward_mv: MotionVector,
) -> PbBMacroblockPrediction {
    let mut luma = [[0u8; 16]; 16];
    for n in 0..4 {
        let nh = n & 1;
        let nv = n >> 1;
        let block = crate::motion::motion_compensate_block(
            &planes.prev_y,
            mb_x + nh * 8,
            mb_y + nv * 8,
            forward_mv,
            RCONTROL_DEFAULT,
        );
        for j in 0..8 {
            luma[nv * 8 + j][nh * 8..nh * 8 + 8].copy_from_slice(&block[j * 8..j * 8 + 8]);
        }
    }
    let chroma_vec = chroma_mv(forward_mv);
    let (cx, cy) = (mb_x / 2, mb_y / 2);
    let cb_flat = crate::motion::motion_compensate_block(
        &planes.prev_cb,
        cx,
        cy,
        chroma_vec,
        RCONTROL_DEFAULT,
    );
    let cr_flat = crate::motion::motion_compensate_block(
        &planes.prev_cr,
        cx,
        cy,
        chroma_vec,
        RCONTROL_DEFAULT,
    );
    let mut cb = [[0u8; 8]; 8];
    let mut cr = [[0u8; 8]; 8];
    for j in 0..8 {
        cb[j].copy_from_slice(&cb_flat[j * 8..j * 8 + 8]);
        cr[j].copy_from_slice(&cr_flat[j * 8..j * 8 + 8]);
    }
    PbBMacroblockPrediction { luma, cb, cr }
}

/// §M.2.3 — the backward BPB prediction is PREC.
fn backward_prediction(planes: &PbBReferencePlanes<'_>) -> PbBMacroblockPrediction {
    let mut luma = [[0u8; 16]; 16];
    for (j, row) in luma.iter_mut().enumerate() {
        row.copy_from_slice(&planes.prec_y.samples[j * 16..j * 16 + 16]);
    }
    let mut cb = [[0u8; 8]; 8];
    let mut cr = [[0u8; 8]; 8];
    for j in 0..8 {
        cb[j].copy_from_slice(&planes.prec_cb.samples[j * 8..j * 8 + 8]);
        cr[j].copy_from_slice(&planes.prec_cr.samples[j * 8..j * 8 + 8]);
    }
    PbBMacroblockPrediction { luma, cb, cr }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::picture::{
        DecodeOptions, decode_improved_pb_picture, decode_pb_picture_no_gob0_header,
        decode_sequence,
    };

    /// Smooth sinusoidal QCIF texture sampled at a sub-pixel offset
    /// `(ox, oy)` (in quarter pixels), plus three 16 × 16 random-texture
    /// squares whose positions jitter per `seed` — so half-pel vectors,
    /// and a mix of forward / bidirectional / backward BPB modes, are
    /// all natural choices.
    fn scene(ox: i32, oy: i32, seed: u32) -> YuvFrame {
        let (lw, lh) = (176usize, 144usize);
        let f = |x: f64, y: f64| -> f64 {
            110.0 + 50.0 * (x / 5.3).sin() * (y / 3.7).cos() + 30.0 * ((x + 2.0 * y) / 9.1).sin()
        };
        let mut y = vec![0u8; lw * lh];
        for r in 0..lh {
            for c in 0..lw {
                let v = f(c as f64 + ox as f64 / 4.0, r as f64 + oy as f64 / 4.0);
                y[r * lw + c] = v.round().clamp(0.0, 255.0) as u8;
            }
        }
        let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(12345);
        let mut rnd = || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        for &col in &[8usize, 80, 136] {
            let jx = (rnd() % 13) as usize;
            let jy = (rnd() % 13) as usize;
            let (x0, y0) = ((col + jx).min(lw - 17), (40 + jy * 3).min(lh - 17));
            for r in 0..16 {
                for c in 0..16 {
                    y[(y0 + r) * lw + x0 + c] = (rnd() % 200 + 28) as u8;
                }
            }
        }
        let cb = (0..88 * 72)
            .map(|i| (100 + (i % 88) / 3 + (i / 88) / 4) as u8)
            .collect();
        let cr = (0..88 * 72)
            .map(|i| (150 - (i % 88) / 4 + (i / 88) / 5) as u8)
            .collect();
        YuvFrame {
            y,
            cb,
            cr,
            luma_width: lw,
            luma_height: lh,
        }
    }

    fn intra_anchor() -> (Vec<u8>, YuvFrame) {
        let bytes = crate::encoder::encode_intra_picture(&scene(0, 0, 1), 8, 0).unwrap();
        let recon = decode_sequence(&bytes, DecodeOptions::default())
            .unwrap()
            .remove(0);
        (bytes, recon)
    }

    /// Annex M (PLUSPTYPE): for every UMV / AP / RTYPE combination the
    /// decoder reproduces the encoder's own P-part (PREC) and BPB-part
    /// reconstruction sample for sample — the §6.1.2 RTYPE rounding of
    /// the P-part, the zero RCONTROL of the BPB-part, the Table D.3
    /// MVDB with its per-pair emulation bit and the §M.2.2 predictor
    /// reset all agree between the two sides.
    #[test]
    fn improved_pb_encoder_recon_equals_decoder_output() {
        let (_, r0) = intra_anchor();
        let p = scene(6, 2, 3);
        let b = scene(3, 1, 2);
        for umv in [false, true] {
            for ap in [false, true] {
                for rtype in [false, true] {
                    let cfg = ImprovedPbConfig::default()
                        .with_quant(6)
                        .with_search_half(6)
                        .with_forward_search_half(4)
                        .with_allow_backward(true)
                        .with_advanced_prediction(ap)
                        .with_umv(umv)
                        .with_rounding_type(rtype);
                    let (core, flavour) = annex_m_core(&cfg);
                    let out = encode_pb_core(&p, &b, &r0, 2, 0, core, flavour).unwrap();
                    let tag = format!("umv {umv} ap {ap} rtype {rtype}: {:?}", out.stats);
                    assert!(
                        out.stats.forward > 0 && out.stats.bidirectional > 0,
                        "{tag}"
                    );
                    let pair =
                        decode_improved_pb_picture(&out.bytes, &r0, 0, DecodeOptions::default())
                            .unwrap();
                    assert_eq!(pair.p_frame, out.p_recon, "P-part, {tag}");
                    assert_eq!(pair.b_frame, out.b_recon, "BPB-part, {tag}");
                }
            }
        }
    }

    /// RTYPE is signalled on the wire and changes the P-part
    /// reconstruction (half-pel vectors are present), while a decoder
    /// that ignored it would drift from the encoder.
    #[test]
    fn improved_pb_rtype_is_signalled_and_load_bearing() {
        let (_, r0) = intra_anchor();
        let p = scene(6, 2, 3);
        let b = scene(3, 1, 2);
        let cfg = ImprovedPbConfig::default()
            .with_quant(6)
            .with_advanced_prediction(true);
        let (c0, f0) = annex_m_core(&cfg);
        let (c1, f1) = annex_m_core(&cfg.with_rounding_type(true));
        let o0 = encode_pb_core(&p, &b, &r0, 2, 0, c0, f0).unwrap();
        let o1 = encode_pb_core(&p, &b, &r0, 2, 0, c1, f1).unwrap();
        assert_ne!(o0.p_recon, o1.p_recon, "RTYPE alters the P-part");
        // MPPTYPE bit 6 sits 22 + 8 + 8 + 3 + 18 + 5 bits into the
        // picture (PSC, TR, PTYPE, UFEP, OPPTYPE, MPPTYPE bits 1-5).
        let bit = |bytes: &[u8], at: usize| (bytes[at / 8] >> (7 - at % 8)) & 1;
        assert_eq!(bit(&o0.bytes, 64), 0);
        assert_eq!(bit(&o1.bytes, 64), 1);
    }

    /// Annex G on the version-1 header, plain / AP / UMV / UMV + AP: the
    /// decoder reproduces both encoder-side reconstructions exactly,
    /// including the per-macroblock first-column MVDB of §G.4 applied to
    /// four per-block scaled vectors under UMV + AP.
    #[test]
    fn annex_g_encoder_recon_equals_decoder_output() {
        let (_, r0) = intra_anchor();
        let p = scene(10, 4, 3);
        let b = scene(5, 2, 2);
        let cfg = PbConfig::default()
            .with_quant(6)
            .with_trb(1)
            .with_search_half(8)
            .with_b_search_half(2);
        for (ap, umv) in [
            (false, UmvMode::Off),
            (true, UmvMode::Off),
            (false, UmvMode::Wrap),
            (true, UmvMode::Wrap),
        ] {
            let out = encode_pb_core(
                &p,
                &b,
                &r0,
                2,
                0,
                annex_g_core(&cfg, ap, umv),
                PbFlavour::AnnexG { b_search_half: 2 },
            )
            .unwrap();
            let tag = format!("ap {ap} umv {umv:?}: {:?}", out.stats);
            let pair =
                decode_pb_picture_no_gob0_header(&out.bytes, &r0, 0, DecodeOptions::default())
                    .unwrap();
            assert_eq!(pair.p_frame, out.p_recon, "P-part, {tag}");
            assert_eq!(pair.b_frame, out.b_recon, "B-part, {tag}");
        }
    }

    /// The first-column MVDB guard: a delta whose per-block §D.2 pair
    /// resolution would pick the second column for a large scaled
    /// predictor is refused, a small one passes.
    #[test]
    fn mvdb_first_column_guard() {
        let mv = MotionVector::new(60, 0);
        let mvs = [mv; 4];
        // TRB 6 / TRD 7: Pc = 51 (outside [-31, 32]); a delta of +15
        // would leave the range, so the pair resolves to 15 - 64.
        let big = Mvd {
            dx_half: 15,
            dy_half: 0,
        };
        assert!(!crate::pb_layer::pb_mvdb_first_column_is_unambiguous(
            &mvs, big, 6, 7
        ));
        let small = Mvd {
            dx_half: -2,
            dy_half: 1,
        };
        assert!(crate::pb_layer::pb_mvdb_first_column_is_unambiguous(
            &mvs, small, 6, 7
        ));
    }
}
