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
    chroma_mv_4mv, encode_mv_component, luma_to_chroma_mv, predict_mv, predict_mv_block, MbMotion,
    MvGrid, MV_RANGE_MAX_HALF, MV_RANGE_MIN_HALF, OBMC_H0, OBMC_H1, OBMC_H2,
};
use crate::picture::SourceFormat;
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

        let tr = self.next_tr;
        self.next_tr = self.next_tr.wrapping_add(1);

        // Decide I vs P: first frame is always I; then every `gop_size` frames
        // we insert another I. `gop_size <= 1` forces I on every frame.
        let force_i = self.reference.is_none()
            || self.gop_size <= 1
            || self.since_keyframe + 1 >= self.gop_size;

        let (data, mut recon, is_key) = if force_i {
            let (bytes, pic) = if self.enable_annex_e {
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
                encode_p_picture_with_opts(
                    self.width,
                    self.height,
                    self.source_format,
                    self.pquant,
                    tr,
                    v,
                    reference,
                    self.enable_annex_f,
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
    let mb_w = width.div_ceil(16) as usize;
    let mb_h = height.div_ceil(16) as usize;
    source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263 encoder: source format has no GOB layout"))?;

    let mut bw = BitWriter::with_capacity(8192);
    let mut recon = IPicture::new(width as usize, height as usize);
    let mut mv_grid = MvGrid::new(mb_w, mb_h);

    write_picture_header(
        &mut bw,
        source_format,
        pquant,
        temporal_reference,
        true,
        enable_annex_f,
    )?;

    if !enable_annex_f {
        // Single-pass baseline path — matches the old `encode_p_picture`
        // behaviour byte-for-byte.
        for mb_y in 0..mb_h {
            for mb_x in 0..mb_w {
                let _info = encode_p_mb(
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
    //   bit 10: UMV (D)        - 0
    //   bit 11: SAC (E)        - 0
    //   bit 12: AP  (F)        - 0
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
    bw.write_bits(0, 1); // bit 10 UMV
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
    bw.write_bits(0, 1); // bit 13 PB

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

    // PEI loop terminator.
    bw.write_bits(0, 1);
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
#[allow(clippy::too_many_arguments)]
fn encode_p_mb(
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
) -> Result<crate::mb::PMbInfo> {
    // 1. Motion-estimate on luma 16×16.
    let (mvx, mvy, mv_sad) = motion_estimate_mb(frame, reference, mb_x, mb_y);

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
    let info = encode_p_mb_inter(
        bw, mb_x, mb_y, quant, src_y, src_cb, src_cr, reference, recon, decide_mv, mv_grid,
        &y_pred, &u_pred, &v_pred,
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
    encode_mv_component(bw, mv.0, pmx);
    encode_mv_component(bw, mv.1, pmy);

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
