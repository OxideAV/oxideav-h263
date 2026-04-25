//! H.263 decoder front-end.
//!
//! Parses one coded picture from each compressed packet, produces one
//! `VideoFrame` per picture (I-picture or P-picture). The previous decoded
//! frame is retained as the motion-compensation reference for the next
//! P-picture; an I-picture clears it. Both baseline PTYPE headers and
//! H.263+ PLUSPTYPE headers are recognised — streams that assert any
//! still-unimplemented annex (Annex D/E/F/G/I/K/N/P/Q/R/S/T or PB-frames)
//! are rejected at the picture-header layer with a diagnostic naming the
//! specific annex; see `picture::parse_picture_header`.

use std::collections::VecDeque;

use oxideav_core::bits::BitReader;
use oxideav_core::frame::VideoPlane;
use oxideav_core::Decoder;
use oxideav_core::{
    CodecId, CodecParameters, Error, Frame, Packet, PixelFormat, Rational, Result, TimeBase,
    VideoFrame,
};

use crate::gob::parse_gob_header;
use crate::mb::{
    apply_p_mb_reconstruction, decode_intra_mb, decode_p_mb, decode_p_mb_pass1, IPicture, PMbInfo,
    UmvMode,
};
use crate::motion::MvGrid;
use crate::picture::{parse_picture_header, PictureCodingType, PictureHeader};
use crate::start_code::{find_next_start_code, StartCode, GN_EOS, GN_PICTURE};

/// Factory for the registry.
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(H263Decoder::new(params.codec_id.clone())))
}

pub struct H263Decoder {
    codec_id: CodecId,
    buffer: Vec<u8>,
    ready_frames: VecDeque<VideoFrame>,
    pending_pts: Option<i64>,
    pending_tb: TimeBase,
    eof: bool,
    /// Previous decoded picture, kept as the motion-compensation reference
    /// for the next P-picture. Cleared on I-pictures (before the I is
    /// decoded) and refreshed after every successful decode.
    reference: Option<IPicture>,
    /// When `true`, apply the Annex J deblocking filter to every decoded
    /// picture (both the output and the MC reference). Default `false`.
    /// When the picture header itself carries a PLUSPTYPE block with the DF
    /// bit set (H.263+), deblocking is also applied for that picture even
    /// if this flag is left off — see [`maybe_deblock`]. For baseline
    /// streams that don't carry a DF bit at all, callers must opt in via
    /// [`Self::set_enable_annex_j`] (and match whatever the encoder did).
    ///
    /// [`maybe_deblock`]: Self::maybe_deblock
    enable_annex_j: bool,
}

impl H263Decoder {
    pub fn new(codec_id: CodecId) -> Self {
        Self {
            codec_id,
            buffer: Vec::new(),
            ready_frames: VecDeque::new(),
            pending_pts: None,
            pending_tb: TimeBase::new(1, 90_000),
            eof: false,
            reference: None,
            enable_annex_j: false,
        }
    }

    /// Enable or disable the Annex J deblocking filter. Must be set before
    /// the first packet is submitted; mid-stream changes would desync the
    /// reconstruction from the encoder.
    pub fn set_enable_annex_j(&mut self, enable: bool) {
        self.enable_annex_j = enable;
    }

    /// Returns whether Annex J deblocking is currently enabled.
    pub fn enable_annex_j(&self) -> bool {
        self.enable_annex_j
    }

    /// Walk the buffer for picture start codes and process each picture
    /// in turn. Bytes past the last complete picture are retained for the
    /// next packet.
    fn process(&mut self) -> Result<()> {
        let data = std::mem::take(&mut self.buffer);
        let mut pos = 0usize;
        // Find first PSC.
        let first_psc = loop {
            match find_next_start_code(&data, pos) {
                Some(sc) if sc.gn == GN_PICTURE => break sc,
                Some(sc) => {
                    // GBSC without preceding PSC — malformed prologue; skip.
                    pos = sc.byte_pos + 3;
                }
                None => return Ok(()), // no start codes at all
            }
        };
        let mut cur = first_psc.byte_pos;
        loop {
            // Find the next PSC (or EOS) after cur.
            let mut scan = cur + 3;
            let next_psc = loop {
                match find_next_start_code(&data, scan) {
                    Some(sc) if sc.gn == GN_PICTURE || sc.gn == GN_EOS => break Some(sc),
                    Some(sc) => {
                        // GBSC inside this picture — keep walking.
                        scan = sc.byte_pos + 3;
                    }
                    None => break None,
                }
            };
            let end = next_psc.map(|s| s.byte_pos).unwrap_or(data.len());
            // If we don't have a known boundary AND we're not at EOF, retain
            // the remaining bytes for the next packet.
            if next_psc.is_none() && !self.eof {
                // Save unprocessed tail starting at `cur`.
                self.buffer.extend_from_slice(&data[cur..]);
                return Ok(());
            }
            let pic_bytes = &data[cur..end];
            self.decode_one_picture(pic_bytes)?;
            match next_psc {
                Some(sc) if sc.gn == GN_PICTURE => {
                    cur = sc.byte_pos;
                }
                _ => return Ok(()),
            }
        }
    }

    fn decode_one_picture(&mut self, bytes: &[u8]) -> Result<()> {
        let mut br = BitReader::new(bytes);
        let hdr = parse_picture_header(&mut br)?;
        match hdr.coding_type {
            PictureCodingType::Intra => {
                let mut pic = if hdr.sac_mode {
                    // Annex E SAC body — arithmetic-coded MB layer per §E.7.
                    crate::mb_sac::decode_i_picture_sac(&hdr, bytes)?
                } else {
                    decode_i_picture(&mut br, &hdr, bytes)?
                };
                self.maybe_deblock(&mut pic, &hdr);
                let frame = pic_to_video_frame(&pic, self.pending_pts, self.pending_tb);
                self.reference = Some(pic);
                self.ready_frames.push_back(frame);
                Ok(())
            }
            PictureCodingType::Predicted => {
                let reference = self.reference.as_ref().ok_or_else(|| {
                    Error::invalid(
                        "h263 P-picture: no reference frame available (stream must start with I)",
                    )
                })?;
                if reference.width != hdr.width as usize || reference.height != hdr.height as usize
                {
                    return Err(Error::invalid(
                        "h263 P-picture: dimension change without I-picture",
                    ));
                }
                let mut pic = if hdr.sac_mode {
                    // Round 14 — SAC P-picture body driver.
                    // Round 15 — when AP is also signalled, dispatch to the
                    // 4MVQ MCBPC + 2-pass OBMC variant.
                    // Round 16 — SAC + Annex J (no AP) flips the MCBPC
                    // model to `cumf_MCBPC_4MVQ` per §E.7. Baseline PTYPE
                    // has no DF bit on the wire; we honour the out-of-band
                    // `set_enable_annex_j` knob OR the PLUSPTYPE DF bit,
                    // matching `maybe_deblock`'s gate.
                    if hdr.advanced_prediction {
                        crate::mb_sac::decode_p_picture_sac_ap(&hdr, bytes, reference)?
                    } else {
                        let df_active = self.enable_annex_j || hdr.deblocking_filter;
                        crate::mb_sac::decode_p_picture_sac(&hdr, bytes, reference, df_active)?
                    }
                } else {
                    decode_p_picture(&mut br, &hdr, bytes, reference)?
                };
                self.maybe_deblock(&mut pic, &hdr);
                let frame = pic_to_video_frame(&pic, self.pending_pts, self.pending_tb);
                self.reference = Some(pic);
                self.ready_frames.push_back(frame);
                Ok(())
            }
        }
    }

    /// Apply the Annex J deblocking filter to `pic` iff the caller opted in
    /// via [`Self::set_enable_annex_j`] OR the picture header signalled the
    /// DF bit inside a PLUSPTYPE (H.263+) block. Uses the picture-header
    /// PQUANT as a uniform per-MB quantiser (matches the encoder, which
    /// holds the quantiser constant for the whole picture).
    fn maybe_deblock(&self, pic: &mut IPicture, hdr: &PictureHeader) {
        let enable = self.enable_annex_j || hdr.deblocking_filter;
        if !enable {
            return;
        }
        let mb_w = pic.mb_width;
        let mb_h = pic.mb_height;
        let qp = vec![hdr.pquant; mb_w * mb_h];
        crate::deblock::deblock_picture(pic, &qp);
    }
}

/// Decode an I-picture body. `bytes` is the full picture (including PSC) so
/// that GOB headers can be located by absolute byte offset within `br`.
pub fn decode_i_picture(
    br: &mut BitReader<'_>,
    hdr: &PictureHeader,
    bytes: &[u8],
) -> Result<IPicture> {
    let mb_w = hdr.width.div_ceil(16) as usize;
    let mb_h = hdr.height.div_ceil(16) as usize;
    let (num_gobs, mb_rows_per_gob) = hdr
        .source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263: source format has no GOB layout"))?;
    let _ = num_gobs;
    let mut pic = IPicture::new(hdr.width as usize, hdr.height as usize);
    let mut quant = hdr.pquant as u32;

    // Pre-compute byte offsets of every GBSC in the picture body so we can
    // realign the bitstream at GOB boundaries (encoders may emit GOB headers
    // sparsely).
    let gob_starts = collect_gob_offsets(bytes);

    for mb_y in 0..mb_h {
        // GOB header check: GOBs start at MB rows (mb_y % mb_rows_per_gob)==0
        // and mb_y > 0 (the first GOB has no header — picture header serves).
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let _ = try_consume_gob_header(br, &gob_starts, hdr, &mut quant)?;
        }
        for mb_x in 0..mb_w {
            quant = decode_intra_mb(br, mb_x, mb_y, quant, &mut pic).map_err(|e| {
                Error::invalid(format!(
                    "h263 I-picture MB ({mb_x},{mb_y}) (q={quant}): {e}"
                ))
            })?;
        }
    }
    Ok(pic)
}

/// Decode a P-picture body. `reference` is the previous reconstructed picture
/// (used for motion compensation). The output picture has the same MB-aligned
/// dimensions as `reference`.
pub fn decode_p_picture(
    br: &mut BitReader<'_>,
    hdr: &PictureHeader,
    bytes: &[u8],
    reference: &IPicture,
) -> Result<IPicture> {
    let mb_w = hdr.width.div_ceil(16) as usize;
    let mb_h = hdr.height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = hdr
        .source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263: source format has no GOB layout"))?;
    let mut pic = IPicture::new(hdr.width as usize, hdr.height as usize);
    let mut quant = hdr.pquant as u32;
    let gob_starts = collect_gob_offsets(bytes);

    let mut mv_grid = MvGrid::new(mb_w, mb_h);
    let umv_mode = UmvMode::from_header(hdr);

    if !hdr.advanced_prediction {
        // Fast single-pass path: MC is a strict function of the current MB's
        // MV and the reference, so we can write final pels as we decode.
        for mb_y in 0..mb_h {
            if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
                let consumed = try_consume_gob_header(br, &gob_starts, hdr, &mut quant)?;
                if consumed {
                    // GOB header present → MV-predictor reset (§5.3.7.2).
                    mv_grid = MvGrid::new(mb_w, mb_h);
                }
            }
            for mb_x in 0..mb_w {
                quant = decode_p_mb(
                    br,
                    mb_x,
                    mb_y,
                    quant,
                    &mut pic,
                    reference,
                    &mut mv_grid,
                    umv_mode,
                )
                .map_err(|e| {
                    Error::invalid(format!(
                        "h263 P-picture MB ({mb_x},{mb_y}) (q={quant}): {e}"
                    ))
                })?;
            }
        }
        return Ok(pic);
    }

    // Two-pass Annex F path: pass 1 populates `mv_grid` + per-MB residuals;
    // pass 2 runs OBMC and writes final pels. Unlike the fast path we do NOT
    // reset `mv_grid` at non-empty GOB boundaries — §F.3 says "remote motion
    // vectors from other video picture segments are used in the same way as
    // remote motion vectors inside the current GOB" (outside Slice
    // Structured / Independent Segment Decoding, neither of which this
    // crate implements). Instead we pass a `gob_top_row` flag into the MV
    // predictor, which collapses above-row neighbours to `(0,0)` for
    // predictor purposes only — OBMC still reads the real MVs.
    let mut residuals: Vec<PMbInfo> = Vec::with_capacity(mb_w * mb_h);
    let mut gob_top_rows = vec![false; mb_h];
    for mb_y in 0..mb_h {
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let consumed = try_consume_gob_header(br, &gob_starts, hdr, &mut quant)?;
            if consumed {
                gob_top_rows[mb_y] = true;
            }
        }
        for mb_x in 0..mb_w {
            let (new_q, info) = decode_p_mb_pass1(
                br,
                mb_x,
                mb_y,
                quant,
                &mut pic,
                &mut mv_grid,
                umv_mode,
                true, // advanced_prediction
                gob_top_rows[mb_y],
            )
            .map_err(|e| {
                Error::invalid(format!(
                    "h263 P-picture MB ({mb_x},{mb_y}) (q={quant}): {e}"
                ))
            })?;
            quant = new_q;
            residuals.push(info);
        }
    }

    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let info = &residuals[mb_y * mb_w + mb_x];
            apply_p_mb_reconstruction(mb_x, mb_y, &mut pic, reference, &mv_grid, info, true);
        }
    }

    Ok(pic)
}

/// Collect byte offsets of every GBSC marker in the picture body. Used by
/// `try_consume_gob_header` to decide whether to align.
fn collect_gob_offsets(bytes: &[u8]) -> Vec<StartCode> {
    let mut out = Vec::new();
    let mut pos = 0;
    while let Some(sc) = find_next_start_code(bytes, pos) {
        out.push(sc);
        pos = sc.byte_pos + 3;
    }
    out
}

/// If the bitstream is at (or near) a GBSC for the current GOB, consume the
/// GOB header and update QUANT. Otherwise leave the bit position alone.
///
/// Encoders are allowed to elide GOB headers when MB row boundaries don't
/// need a resync point — most short clips have no GOB headers at all. We
/// only realign when a registered GBSC sits within a few bytes of the current
/// bit position.
fn try_consume_gob_header(
    br: &mut BitReader<'_>,
    gobs: &[StartCode],
    hdr: &PictureHeader,
    quant: &mut u32,
) -> Result<bool> {
    let cur_bit = br.bit_position();
    let cur_byte = (cur_bit / 8) as usize;
    let target = gobs
        .iter()
        .find(|g| g.byte_pos >= cur_byte && g.gn != GN_PICTURE && g.gn != GN_EOS);
    let Some(target) = target else {
        return Ok(false);
    };
    let pad_bits = target.byte_pos as u64 * 8 - cur_bit;
    if pad_bits > 32 {
        // The next GBSC isn't near here — the encoder elided this GOB header.
        return Ok(false);
    }
    if pad_bits > 0 {
        br.skip(pad_bits as u32)?;
    }
    let gob = parse_gob_header(br, hdr.cpm)?;
    *quant = gob.gquant as u32;
    Ok(true)
}

/// Build a stride-packed YUV420P `VideoFrame` from an `IPicture`.
pub fn pic_to_video_frame(pic: &IPicture, pts: Option<i64>, tb: TimeBase) -> VideoFrame {
    let w = pic.width;
    let h = pic.height;
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    let mut y = vec![0u8; w * h];
    for row in 0..h {
        y[row * w..row * w + w].copy_from_slice(&pic.y[row * pic.y_stride..row * pic.y_stride + w]);
    }
    let mut cb = vec![0u8; cw * ch];
    let mut cr = vec![0u8; cw * ch];
    for row in 0..ch {
        cb[row * cw..row * cw + cw]
            .copy_from_slice(&pic.cb[row * pic.c_stride..row * pic.c_stride + cw]);
        cr[row * cw..row * cw + cw]
            .copy_from_slice(&pic.cr[row * pic.c_stride..row * pic.c_stride + cw]);
    }
    VideoFrame {
        format: PixelFormat::Yuv420P,
        width: w as u32,
        height: h as u32,
        pts,
        time_base: tb,
        planes: vec![
            VideoPlane { stride: w, data: y },
            VideoPlane {
                stride: cw,
                data: cb,
            },
            VideoPlane {
                stride: cw,
                data: cr,
            },
        ],
    }
}

impl Decoder for H263Decoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        self.pending_pts = packet.pts;
        self.pending_tb = packet.time_base;
        self.buffer.extend_from_slice(&packet.data);
        self.process()
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        if let Some(f) = self.ready_frames.pop_front() {
            return Ok(Frame::Video(f));
        }
        if self.eof {
            Err(Error::Eof)
        } else {
            Err(Error::NeedMore)
        }
    }

    fn reset(&mut self) -> Result<()> {
        // H.263 is self-contained per packet bar the motion-compensation
        // reference. Drop the NAL buffer, ready queue, and the last
        // decoded picture used as MV reference for the next P-picture.
        self.buffer.clear();
        self.ready_frames.clear();
        self.pending_pts = None;
        self.eof = false;
        self.reference = None;
        Ok(())
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        self.process()
    }
}

/// Build a `CodecParameters` from a parsed picture header.
pub fn codec_parameters_from_header(hdr: &PictureHeader) -> CodecParameters {
    let mut params = CodecParameters::video(CodecId::new(crate::CODEC_ID_STR));
    params.width = Some(hdr.width);
    params.height = Some(hdr.height);
    // H.263 doesn't carry frame-rate in the bitstream; assume 30 fps as a
    // placeholder (matches RFC 4629 / RTP defaults).
    params.frame_rate = Some(Rational::new(30, 1));
    params
}
