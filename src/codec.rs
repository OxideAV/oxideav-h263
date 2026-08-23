//! `oxideav_core` registry integration — the streaming
//! [`Decoder`] / [`Encoder`] adapters over the crate's picture
//! drivers, the direct [`make_decoder`] / [`make_encoder`] factories,
//! and the [`register`] entry point that installs them (plus the
//! codec's container-tag and payload-magic claims) into a
//! [`RuntimeContext`].
//!
//! ## Decoder
//!
//! [`H263StreamDecoder`] wraps the elementary-stream machinery of
//! [`crate::picture`] ([`decode_sequence_step`] +
//! [`next_picture_start_code`]) behind the packetised
//! [`Decoder`] contract:
//!
//! * input packets are treated as arbitrary byte slices of one H.263 /
//!   H.263+ elementary stream — the adapter re-frames them on
//!   byte-aligned Picture Start Codes (§5.1.1 / §5.1.28), so both
//!   one-picture-per-packet container output and arbitrarily-split
//!   raw streams decode identically;
//! * a picture is decoded as soon as it is known complete: either a
//!   following PSC terminates it, or an eager decode of the buffered
//!   tail succeeds (a picture carries a fixed macroblock count, so a
//!   decode that runs off the end of a truncated buffer surfaces
//!   [`crate::Error::UnexpectedEof`] and the adapter simply waits for
//!   more bytes);
//! * the §5.1.4.4 inherited extended-mode state, the §G.4 reference
//!   TR and the prediction reference thread across packets exactly as
//!   in [`crate::picture::decode_sequence`]; an Annex G / Annex M
//!   PB-frame yields two output frames in display order;
//! * `reset()` drops the buffer, the reference and the cross-picture
//!   state so decode resumes cleanly after a seek.
//!
//! ## Encoder
//!
//! [`H263StreamEncoder`] drives the closed-loop I + P GOP encoder
//! (the same per-picture loop as [`crate::encoder::encode_sequence`]):
//! every INTER picture predicts from the encoder's own decoded
//! reconstruction, so the packet stream it emits decodes drift-free.
//! One frame in, one packet out; the GOP shape, quantiser, motion
//! search range, Annex D UMV mode and the optional trailing §5.1.27
//! EOS marker are [`CodecParameters::options`] knobs (see
//! [`H263EncoderOptions`]).

use std::collections::VecDeque;

use oxideav_core::registry::{CodecInfo, Decoder, Encoder, RuntimeContext};
use oxideav_core::{
    CodecCapabilities, CodecId, CodecOptionsStruct, CodecParameters, CodecTag, Error as CoreError,
    Frame, OptionField, OptionKind, OptionValue, Packet, PixelFormat, Result as CoreResult,
    TimeBase, VideoFrame, VideoPlane,
};

use crate::encoder::{
    encode_inter_picture_motion, encode_inter_picture_umv, encode_intra_picture, EOS_BYTES,
};
use crate::picture::{
    decode_picture_no_gob0_header, decode_sequence_step, next_picture_start_code, DecodeOptions,
    SequenceState, YuvFrame,
};

/// The registry identifier this crate's codec registers under.
pub const CODEC_ID: &str = "h263";

/// Map a crate-level decode/encode error onto the framework error
/// vocabulary: [`crate::Error::NotImplemented`] means "legal stream,
/// unstaged feature" ([`CoreError::Unsupported`]); everything else is
/// a property of the input bytes ([`CoreError::InvalidData`]).
fn core_error(e: crate::Error) -> CoreError {
    match e {
        crate::Error::NotImplemented => CoreError::Unsupported(e.to_string()),
        other => CoreError::InvalidData(other.to_string()),
    }
}

// ───────────────────────── decoder options ─────────────────────────

/// Decoder tuning knobs recognised in [`CodecParameters::options`].
///
/// The three Annex modes the *baseline* picture header cannot signal
/// on the wire (Annex I / Annex T / Annex S) plus the Annex J
/// deblocking filter are exposed as opt-ins, mirroring
/// [`DecodeOptions`]; PLUSPTYPE streams signal all four in OPPTYPE and
/// need no options. `obmc_skip_zero_right` is the documented
/// ecosystem-compatibility deviation for Advanced-Prediction streams.
#[derive(Debug, Clone, Copy, Default)]
pub struct H263DecoderOptions {
    /// Annex J §J.3 in-loop deblocking filter (baseline streams only —
    /// PLUSPTYPE streams auto-enable it from OPPTYPE).
    pub deblock: bool,
    /// Annex I Advanced INTRA Coding on baseline-header pictures.
    pub aic: bool,
    /// Annex T Modified Quantization on baseline-header pictures.
    pub modified_quant: bool,
    /// Annex S Alternative INTER VLC on baseline-header pictures.
    pub alt_inter_vlc: bool,
    /// §F.3 right-half remote vectors of a not-coded macroblock read
    /// as zero instead of the right neighbour's actual vector (an
    /// ecosystem-compatibility deviation some encoder families
    /// require — see [`DecodeOptions::obmc_skip_zero_right`]).
    pub obmc_skip_zero_right: bool,
}

impl H263DecoderOptions {
    fn to_decode_options(self) -> DecodeOptions {
        DecodeOptions {
            deblock: self.deblock,
            aic: self.aic,
            modified_quant: self.modified_quant,
            alt_inter_vlc: self.alt_inter_vlc,
            obmc_skip_zero_right: self.obmc_skip_zero_right,
        }
    }
}

impl CodecOptionsStruct for H263DecoderOptions {
    const SCHEMA: &'static [OptionField] = &[
        OptionField {
            name: "deblock",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Annex J deblocking filter on baseline-header pictures",
        },
        OptionField {
            name: "aic",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Annex I Advanced INTRA Coding on baseline-header pictures",
        },
        OptionField {
            name: "modified_quant",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Annex T Modified Quantization on baseline-header pictures",
        },
        OptionField {
            name: "alt_inter_vlc",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Annex S Alternative INTER VLC on baseline-header pictures",
        },
        OptionField {
            name: "obmc_skip_zero_right",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "zero right-half OBMC remotes for skipped macroblocks (ecosystem deviation)",
        },
    ];

    fn apply(&mut self, key: &str, value: &OptionValue) -> CoreResult<()> {
        match key {
            "deblock" => self.deblock = value.as_bool()?,
            "aic" => self.aic = value.as_bool()?,
            "modified_quant" => self.modified_quant = value.as_bool()?,
            "alt_inter_vlc" => self.alt_inter_vlc = value.as_bool()?,
            "obmc_skip_zero_right" => self.obmc_skip_zero_right = value.as_bool()?,
            _ => unreachable!("guarded by SCHEMA"),
        }
        Ok(())
    }
}

// ───────────────────────── encoder options ─────────────────────────

/// Encoder tuning knobs recognised in [`CodecParameters::options`].
#[derive(Debug, Clone, Copy)]
pub struct H263EncoderOptions {
    /// Quantiser for every picture (`1..=31`).
    pub quant: u32,
    /// GOP length: an INTRA picture every `gop` frames (frame 0 is
    /// always INTRA); `0` means only the first frame is INTRA.
    pub gop: u32,
    /// Motion-search window for P-pictures (± whole pixels around the
    /// §6.1.1 predictor).
    pub search: u32,
    /// Encode P-pictures in Annex D Unrestricted Motion Vector mode.
    pub umv: bool,
    /// Append the §5.1.27 End Of Sequence marker (as one final packet)
    /// when the encoder is flushed.
    pub eos: bool,
}

impl Default for H263EncoderOptions {
    fn default() -> Self {
        H263EncoderOptions {
            quant: 8,
            gop: 12,
            search: 8,
            umv: false,
            eos: false,
        }
    }
}

impl CodecOptionsStruct for H263EncoderOptions {
    const SCHEMA: &'static [OptionField] = &[
        OptionField {
            name: "quant",
            kind: OptionKind::U32,
            default: OptionValue::U32(8),
            help: "picture quantiser, 1..=31",
        },
        OptionField {
            name: "gop",
            kind: OptionKind::U32,
            default: OptionValue::U32(12),
            help: "INTRA picture every N frames; 0 = first frame only",
        },
        OptionField {
            name: "search",
            kind: OptionKind::U32,
            default: OptionValue::U32(8),
            help: "P-picture motion search range in whole pixels",
        },
        OptionField {
            name: "umv",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Annex D Unrestricted Motion Vector mode for P-pictures",
        },
        OptionField {
            name: "eos",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "emit the End Of Sequence marker on flush",
        },
    ];

    fn apply(&mut self, key: &str, value: &OptionValue) -> CoreResult<()> {
        match key {
            "quant" => self.quant = value.as_u32()?,
            "gop" => self.gop = value.as_u32()?,
            "search" => self.search = value.as_u32()?,
            "umv" => self.umv = value.as_bool()?,
            "eos" => self.eos = value.as_bool()?,
            _ => unreachable!("guarded by SCHEMA"),
        }
        Ok(())
    }
}

// ───────────────────────── frame conversion ─────────────────────────

/// Convert a decoded [`YuvFrame`] into the framework's planar 4:2:0
/// [`VideoFrame`] (three tightly-packed planes, stride = plane width).
fn yuv_to_video_frame(f: YuvFrame, pts: Option<i64>) -> VideoFrame {
    let cw = f.chroma_width();
    VideoFrame {
        pts,
        planes: vec![
            VideoPlane {
                stride: f.luma_width,
                data: f.y,
            },
            VideoPlane {
                stride: cw,
                data: f.cb,
            },
            VideoPlane {
                stride: cw,
                data: f.cr,
            },
        ],
    }
}

/// Copy one plane out of a [`VideoFrame`], honouring its stride, into
/// a tightly-packed `width × height` buffer.
fn copy_plane(plane: &VideoPlane, width: usize, height: usize, name: &str) -> CoreResult<Vec<u8>> {
    if height == 0 {
        return Ok(Vec::new());
    }
    if plane.stride < width {
        return Err(CoreError::invalid(format!(
            "oxideav-h263: {name} plane stride {} shorter than width {width}",
            plane.stride
        )));
    }
    let need = plane
        .stride
        .checked_mul(height - 1)
        .and_then(|v| v.checked_add(width))
        .ok_or_else(|| CoreError::invalid("oxideav-h263: plane geometry overflow"))?;
    if plane.data.len() < need {
        return Err(CoreError::invalid(format!(
            "oxideav-h263: {name} plane holds {} bytes, {need} required for {width}x{height}",
            plane.data.len()
        )));
    }
    let mut out = Vec::with_capacity(width * height);
    for row in 0..height {
        let start = row * plane.stride;
        out.extend_from_slice(&plane.data[start..start + width]);
    }
    Ok(out)
}

/// Convert an input [`VideoFrame`] into the encoder's [`YuvFrame`],
/// validating the planar 4:2:0 shape against the configured geometry.
fn video_frame_to_yuv(v: &VideoFrame, width: usize, height: usize) -> CoreResult<YuvFrame> {
    let planes = v.image_planes();
    if planes.len() != 3 {
        return Err(CoreError::invalid(format!(
            "oxideav-h263: encoder expects 3 planar 4:2:0 planes, got {}",
            planes.len()
        )));
    }
    let cw = width / 2;
    let ch = height / 2;
    Ok(YuvFrame {
        y: copy_plane(&planes[0], width, height, "luma")?,
        cb: copy_plane(&planes[1], cw, ch, "Cb")?,
        cr: copy_plane(&planes[2], cw, ch, "Cr")?,
        luma_width: width,
        luma_height: height,
    })
}

// ───────────────────────── decoder adapter ─────────────────────────

/// Streaming H.263 / H.263+ elementary-stream decoder implementing
/// the framework [`Decoder`] contract — see the module docs for the
/// re-framing / eager-decode behaviour.
#[derive(Debug)]
pub struct H263StreamDecoder {
    id: CodecId,
    options: DecodeOptions,
    /// [`oxideav_core::DecoderLimits::max_pixels_per_frame`] cap the
    /// construction parameters carried; checked against every decoded
    /// picture's geometry (H.263 geometry is architecturally bounded
    /// at 2048 × 1152 by §5.1.5, so the default cap never fires).
    max_pixels: u64,
    /// Undecoded stream bytes (always beginning at, or before, the
    /// next picture's PSC).
    buf: Vec<u8>,
    /// Absolute stream offset of `buf[0]` — the coordinate system for
    /// `pts_marks`.
    stream_pos: u64,
    /// `(absolute offset of a packet's first byte, its PTS)` — the
    /// picture that *starts* at or after a mark inherits that packet's
    /// PTS (consumed once; further pictures from the same packet carry
    /// no PTS).
    pts_marks: VecDeque<(u64, Option<i64>)>,
    /// Decoded frames not yet collected via `receive_frame`.
    pending: VecDeque<Frame>,
    /// Prediction reference: the last decoded reference picture.
    reference: Option<YuvFrame>,
    /// §5.1.4.4 inherited-mode + §G.4 reference-TR stream state.
    state: SequenceState,
    /// End-of-stream flag set by `flush`.
    flushed: bool,
}

impl H263StreamDecoder {
    /// Construct from codec parameters (the [`make_decoder`] factory
    /// body): parses [`H263DecoderOptions`] strictly and captures the
    /// decode limits. Stream geometry is read from the self-describing
    /// bitstream; `params.width` / `params.height` are advisory and
    /// not enforced.
    pub fn from_params(params: &CodecParameters) -> CoreResult<Self> {
        let opts: H263DecoderOptions = oxideav_core::parse_options(&params.options)?;
        Ok(H263StreamDecoder {
            id: CodecId::new(CODEC_ID),
            options: opts.to_decode_options(),
            max_pixels: params.limits.max_pixels_per_frame,
            buf: Vec::new(),
            stream_pos: 0,
            pts_marks: VecDeque::new(),
            pending: VecDeque::new(),
            reference: None,
            state: SequenceState::default(),
            flushed: false,
        })
    }

    /// Drop `n` bytes from the front of the buffer, advancing the
    /// absolute stream position.
    fn discard(&mut self, n: usize) {
        self.buf.drain(..n);
        self.stream_pos += n as u64;
    }

    /// Take the PTS the picture starting at absolute offset `at`
    /// inherits: the most recent packet mark at or before `at`,
    /// consumed on first use.
    fn take_pts(&mut self, at: u64) -> Option<i64> {
        while self.pts_marks.len() >= 2 && self.pts_marks[1].0 <= at {
            self.pts_marks.pop_front();
        }
        match self.pts_marks.front_mut() {
            Some(mark) if mark.0 <= at => mark.1.take(),
            _ => None,
        }
    }

    /// Decode one complete picture slice, push its frames, advance the
    /// reference / limits bookkeeping.
    fn decode_picture_slice(&mut self, end: usize) -> CoreResult<()> {
        let pts = self.take_pts(self.stream_pos);
        let picture = &self.buf[..end];
        let frames = decode_sequence_step(picture, self.reference.as_ref(), self.options, {
            // Split borrow: state is disjoint from buf/reference.
            &mut self.state
        })
        .map_err(core_error)?;
        self.commit_frames(frames, pts)?;
        self.discard(end);
        Ok(())
    }

    /// Push decoded frames (display order) into the output queue; the
    /// last frame becomes the prediction reference.
    fn commit_frames(&mut self, frames: Vec<YuvFrame>, mut pts: Option<i64>) -> CoreResult<()> {
        let last = frames.len().saturating_sub(1);
        for (i, f) in frames.into_iter().enumerate() {
            let pixels = (f.luma_width as u64) * (f.luma_height as u64);
            if pixels > self.max_pixels {
                return Err(CoreError::resource_exhausted(format!(
                    "oxideav-h263: decoded picture {}x{} exceeds max_pixels_per_frame {}",
                    f.luma_width, f.luma_height, self.max_pixels
                )));
            }
            if i == last {
                self.reference = Some(f.clone());
            }
            self.pending
                .push_back(Frame::Video(yuv_to_video_frame(f, pts.take())));
        }
        Ok(())
    }

    /// Decode every picture currently known complete. With `at_eof`
    /// the buffered tail is decoded unconditionally (a truncated tail
    /// is then a hard error); otherwise an eager tail decode that runs
    /// out of bits simply waits for more input.
    fn drain(&mut self, at_eof: bool) -> CoreResult<()> {
        loop {
            // Align the buffer to the next byte-aligned PSC, dropping
            // pre-stream garbage but retaining a possible split PSC
            // prefix (a lone 0x00 or 0x00 0x00 tail).
            match next_picture_start_code(&self.buf, 0) {
                Some(0) => {}
                Some(p) => self.discard(p),
                None => {
                    let keep = match self.buf.as_slice() {
                        [.., 0x00, 0x00] => 2,
                        [.., 0x00] => 1,
                        _ => 0,
                    };
                    let drop = self.buf.len() - keep;
                    self.discard(drop);
                    return Ok(());
                }
            }
            match next_picture_start_code(&self.buf, 1) {
                Some(next) => {
                    // A following PSC terminates the picture (§5.1.28
                    // byte-alignment guarantee).
                    self.decode_picture_slice(next)?;
                }
                None if at_eof => {
                    // Final picture of the stream (any trailing EOS
                    // bytes ride along and are ignored).
                    let end = self.buf.len();
                    self.decode_picture_slice(end)?;
                    return Ok(());
                }
                None => {
                    // Unterminated tail: decode eagerly — a picture
                    // carries a fixed macroblock count, so a truncated
                    // tail surfaces UnexpectedEof and we wait instead.
                    let pts_at = self.stream_pos;
                    let mut speculative = self.state;
                    match decode_sequence_step(
                        &self.buf,
                        self.reference.as_ref(),
                        self.options,
                        &mut speculative,
                    ) {
                        Ok(frames) => {
                            self.state = speculative;
                            let pts = self.take_pts(pts_at);
                            self.commit_frames(frames, pts)?;
                            // Retain a possible split PSC prefix of the
                            // *next* picture (harmless if it was this
                            // picture's own trailing stuffing — the
                            // next alignment scan skips it).
                            let keep = match self.buf.as_slice() {
                                [.., 0x00, 0x00] => 2,
                                [.., 0x00] => 1,
                                _ => 0,
                            };
                            let drop = self.buf.len() - keep;
                            self.discard(drop);
                            return Ok(());
                        }
                        Err(crate::Error::UnexpectedEof) => return Ok(()),
                        Err(e) => return Err(core_error(e)),
                    }
                }
            }
        }
    }
}

impl Decoder for H263StreamDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.id
    }

    fn send_packet(&mut self, packet: &Packet) -> CoreResult<()> {
        if self.flushed {
            return Err(CoreError::invalid(
                "oxideav-h263: send_packet after flush (reset the decoder first)",
            ));
        }
        if !packet.data.is_empty() {
            self.pts_marks
                .push_back((self.stream_pos + self.buf.len() as u64, packet.pts));
            self.buf.extend_from_slice(&packet.data);
        }
        self.drain(false)
    }

    fn receive_frame(&mut self) -> CoreResult<Frame> {
        match self.pending.pop_front() {
            Some(f) => Ok(f),
            None if self.flushed => Err(CoreError::Eof),
            None => Err(CoreError::NeedMore),
        }
    }

    fn flush(&mut self) -> CoreResult<()> {
        if !self.flushed {
            self.flushed = true;
            // Decode whatever complete picture the tail still holds
            // (EOS-only / empty tails fall through the PSC scan).
            self.drain(true)?;
        }
        Ok(())
    }

    fn reset(&mut self) -> CoreResult<()> {
        self.buf.clear();
        self.stream_pos = 0;
        self.pts_marks.clear();
        self.pending.clear();
        self.reference = None;
        self.state = SequenceState::default();
        self.flushed = false;
        Ok(())
    }
}

// ───────────────────────── encoder adapter ─────────────────────────

/// Streaming closed-loop H.263 encoder implementing the framework
/// [`Encoder`] contract — the per-frame form of
/// [`crate::encoder::encode_sequence`].
#[derive(Debug)]
pub struct H263StreamEncoder {
    id: CodecId,
    out_params: CodecParameters,
    width: usize,
    height: usize,
    opts: H263EncoderOptions,
    /// The encoder's own decoded reconstruction of the last picture —
    /// the next P-picture's prediction reference (closed loop, no
    /// drift).
    recon: Option<YuvFrame>,
    /// Frames encoded so far; drives the §5.1.2 TR (mod 256) and the
    /// GOP cadence.
    frame_index: usize,
    pending: VecDeque<Packet>,
    flushed: bool,
}

impl H263StreamEncoder {
    /// Construct from codec parameters (the [`make_encoder`] factory
    /// body). Requires `width` / `height` naming one of the five §5.1.3
    /// standard source formats (sub-QCIF 128×96, QCIF 176×144, CIF
    /// 352×288, 4CIF 704×576, 16CIF 1408×1152) and, when set, a
    /// `pixel_format` of planar 4:2:0.
    pub fn from_params(params: &CodecParameters) -> CoreResult<Self> {
        let opts: H263EncoderOptions = oxideav_core::parse_options(&params.options)?;
        if !(1..=31).contains(&opts.quant) {
            return Err(CoreError::invalid(format!(
                "oxideav-h263: quant {} outside 1..=31",
                opts.quant
            )));
        }
        let (width, height) = match (params.width, params.height) {
            (Some(w), Some(h)) => (w as usize, h as usize),
            _ => {
                return Err(CoreError::invalid(
                    "oxideav-h263: encoder needs width and height in CodecParameters",
                ))
            }
        };
        const STANDARD_FORMATS: [(usize, usize); 5] =
            [(128, 96), (176, 144), (352, 288), (704, 576), (1408, 1152)];
        if !STANDARD_FORMATS.contains(&(width, height)) {
            return Err(CoreError::unsupported(format!(
                "oxideav-h263: {width}x{height} is not a §5.1.3 standard source format \
                 (sub-QCIF/QCIF/CIF/4CIF/16CIF)"
            )));
        }
        if let Some(pf) = params.pixel_format {
            if pf != PixelFormat::Yuv420P {
                return Err(CoreError::unsupported(format!(
                    "oxideav-h263: encoder input must be Yuv420P, got {pf:?}"
                )));
            }
        }
        let mut out_params = CodecParameters::video(CodecId::new(CODEC_ID));
        out_params.width = Some(width as u32);
        out_params.height = Some(height as u32);
        out_params.pixel_format = Some(PixelFormat::Yuv420P);
        out_params.frame_rate = params.frame_rate;
        out_params.tag = Some(CodecTag::fourcc(b"H263"));
        Ok(H263StreamEncoder {
            id: CodecId::new(CODEC_ID),
            out_params,
            width,
            height,
            opts,
            recon: None,
            frame_index: 0,
            pending: VecDeque::new(),
            flushed: false,
        })
    }
}

impl Encoder for H263StreamEncoder {
    fn codec_id(&self) -> &CodecId {
        &self.id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.out_params
    }

    fn send_frame(&mut self, frame: &Frame) -> CoreResult<()> {
        if self.flushed {
            return Err(CoreError::invalid("oxideav-h263: send_frame after flush"));
        }
        let video = match frame {
            Frame::Video(v) => v,
            _ => {
                return Err(CoreError::invalid(
                    "oxideav-h263: encoder accepts video frames only",
                ))
            }
        };
        let yuv = video_frame_to_yuv(video, self.width, self.height)?;
        let tr = (self.frame_index & 0xFF) as u8;
        let gop = self.opts.gop as usize;
        let force_intra = self.recon.is_none() || (gop != 0 && self.frame_index % gop == 0);
        let quant = self.opts.quant as u8;
        let search = self.opts.search as i32;
        let bytes = if force_intra {
            encode_intra_picture(&yuv, quant, tr)
        } else {
            let reference = self.recon.as_ref().expect("recon present for P-picture");
            if self.opts.umv {
                encode_inter_picture_umv(&yuv, reference, quant, tr, search)
            } else {
                encode_inter_picture_motion(&yuv, reference, quant, tr, search)
            }
        }
        .map_err(core_error)?;
        // Closed loop: the next picture predicts from the *decoded*
        // reconstruction of this one, exactly like the decoder will.
        let decoded = decode_picture_no_gob0_header(
            &bytes,
            if force_intra {
                None
            } else {
                self.recon.as_ref()
            },
            DecodeOptions::default(),
        )
        .map_err(core_error)?;
        self.recon = Some(decoded);
        self.frame_index += 1;
        let mut packet = Packet::new(0, TimeBase::MICROS, bytes).with_keyframe(force_intra);
        packet.pts = video.pts;
        packet.dts = video.pts;
        self.pending.push_back(packet);
        Ok(())
    }

    fn receive_packet(&mut self) -> CoreResult<Packet> {
        match self.pending.pop_front() {
            Some(p) => Ok(p),
            None if self.flushed => Err(CoreError::Eof),
            None => Err(CoreError::NeedMore),
        }
    }

    fn flush(&mut self) -> CoreResult<()> {
        if !self.flushed {
            self.flushed = true;
            if self.opts.eos {
                // §5.1.27 — the End Of Sequence codeword as one final
                // (non-picture) packet; decoders skip it transparently.
                self.pending
                    .push_back(Packet::new(0, TimeBase::MICROS, EOS_BYTES.to_vec()));
            }
        }
        Ok(())
    }
}

// ───────────────────────── factories + registration ─────────────────────────

/// Direct decoder factory (the id-keyed registry entry): build a fresh
/// [`H263StreamDecoder`] honouring `params` (decoder options + limits).
pub fn make_decoder(params: &CodecParameters) -> CoreResult<Box<dyn Decoder>> {
    Ok(Box::new(H263StreamDecoder::from_params(params)?))
}

/// Direct encoder factory: build a fresh closed-loop
/// [`H263StreamEncoder`] honouring `params` (geometry, pixel format,
/// encoder options).
pub fn make_encoder(params: &CodecParameters) -> CoreResult<Box<dyn Encoder>> {
    Ok(Box::new(H263StreamEncoder::from_params(params)?))
}

/// Build this codec's [`CodecInfo`] registration record: capabilities,
/// both factories, the option schemas, the container tags (`H263` —
/// AVI-family FourCC — and `S263`, the 3GP/MP4 sample-entry code), and
/// the byte-aligned §5.1.1 Picture-Start-Code payload magics
/// (`00 00 8x`, the four values of the TR high bits) for raw
/// elementary-stream identification.
fn codec_info() -> CodecInfo {
    let mut caps = CodecCapabilities::video("h263_sw");
    caps.lossy = true;
    caps.lossless = false;
    caps.intra_only = false;
    // §5.1.5 CPFMT bounds the coded geometry at 2048 × 1152.
    caps.max_width = Some(2048);
    caps.max_height = Some(1152);
    CodecInfo::new(CodecId::new(CODEC_ID))
        .capabilities(caps)
        .decoder(make_decoder)
        .encoder(make_encoder)
        .decoder_options::<H263DecoderOptions>()
        .encoder_options::<H263EncoderOptions>()
        .tags([CodecTag::fourcc(b"H263"), CodecTag::fourcc(b"S263")])
        .payload_magics([
            vec![0x00, 0x00, 0x80],
            vec![0x00, 0x00, 0x81],
            vec![0x00, 0x00, 0x82],
            vec![0x00, 0x00, 0x83],
        ])
}

/// Install this crate's codec — decoder + encoder factories, tag and
/// payload-magic claims — into the runtime context's codec registry.
pub fn register(ctx: &mut RuntimeContext) {
    ctx.codecs.register(codec_info());
}
