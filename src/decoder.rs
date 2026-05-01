//! H.263 decoder front-end.
//!
//! Parses one coded picture from each compressed packet, produces one
//! `VideoFrame` per picture (I-picture or P-picture). The previous decoded
//! frame is retained as the motion-compensation reference for the next
//! P-picture; an I-picture clears it. Both baseline PTYPE headers and
//! H.263+ PLUSPTYPE headers are recognised — streams that assert any
//! still-unimplemented annex (Annex G/I/K/P/Q/R/S/T or PB-frames) are
//! rejected at the picture-header layer with a diagnostic naming the
//! specific annex; see `picture::parse_picture_header`. Annex D/E/F are
//! handled in the MB body; Annex N is parsed in the picture header and
//! plumbed through the decoder's RPS picture-memory cache (round 13).

use std::collections::VecDeque;
use std::sync::Arc;

use oxideav_core::arena::sync::{ArenaPool, FrameHeader, FrameInner};
use oxideav_core::bits::BitReader;
use oxideav_core::format::PixelFormat;
use oxideav_core::frame::VideoPlane;
use oxideav_core::Decoder;
use oxideav_core::{
    CodecId, CodecParameters, DecoderLimits, Error, Frame, Packet, Rational, Result, TimeBase,
    VideoFrame,
};

use crate::gob::parse_gob_header;
use crate::mb::{
    apply_p_mb_reconstruction, decode_intra_mb, decode_p_mb, decode_p_mb_pass1, decode_p_mb_pb,
    IPicture, PMbInfo, UmvMode,
};
use crate::motion::{MbMotion, MvGrid};
use crate::picture::{parse_picture_header, PictureCodingType, PictureHeader};
use crate::start_code::{find_next_start_code, StartCode, GN_EOS, GN_PICTURE};

/// Factory for the registry. Honours [`CodecParameters::limits`] — the
/// returned decoder bounds peak memory by the supplied
/// [`DecoderLimits`] (per-frame pixel cap + arena-pool size +
/// per-arena byte cap).
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(H263Decoder::with_limits(
        params.codec_id.clone(),
        *params.limits(),
    )))
}

pub struct H263Decoder {
    codec_id: CodecId,
    buffer: Vec<u8>,
    /// Decoded pictures awaiting `receive_frame` / `receive_arena_frame`.
    /// Stored as raw `IPicture`s plus the pts the picture was tagged
    /// with. Each drain materialises one of two output forms on
    /// demand: `receive_frame` builds a heap-backed `VideoFrame` (no
    /// arena involved); `receive_arena_frame` leases an arena from
    /// the pool and builds an `arena::sync::Frame` whose plane bytes
    /// live inside the leased buffer. Deferring the arena lease to
    /// drain time keeps the pool short-lived (one slot held for at
    /// most the time between `receive_arena_frame` and the caller
    /// dropping the last `Arc` clone), so `send_packet` calls that
    /// decode multiple pictures before the consumer drains do not
    /// exhaust the pool.
    ready_frames: VecDeque<(IPicture, Option<i64>)>,
    pending_pts: Option<i64>,
    pending_tb: TimeBase,
    eof: bool,
    /// Previous decoded picture, kept as the motion-compensation reference
    /// for the next P-picture. Cleared on I-pictures (before the I is
    /// decoded) and refreshed after every successful decode.
    reference: Option<IPicture>,
    /// Round 13 — Annex N (Reference Picture Selection) multi-reference
    /// cache. Keyed by the picture's TR (temporal reference); populated on
    /// every successful decode regardless of whether the stream signalled
    /// RPS (so a future picture's TRP lookup always finds the latest
    /// candidates). Bounded LRU — older entries fall off after
    /// [`Self::rps_cache_capacity`].
    ///
    /// Only consulted when a P-picture's parsed `PictureHeader.trpi` is set
    /// AND `rps_mode` is set; otherwise the most-recent `reference` is used
    /// (`§N.5` "When the picture for which the TR is TRP is not available
    /// at the decoder, the decoder may send a forced INTRA update signal
    /// to the encoder by external means" — we instead fall back to the
    /// most recent reference and log via `Result::Err` upstream).
    rps_cache: VecDeque<(u16, IPicture)>,
    /// Maximum entries in [`Self::rps_cache`]. Default 4 — Annex N's
    /// "additional picture memory" is signalled out-of-band (H.245); we
    /// pick a small default that still demonstrates multi-reference
    /// selection. Tunable via [`Self::set_rps_cache_capacity`].
    rps_cache_capacity: usize,
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
    /// DoS-protection caps threaded from the stream's
    /// [`CodecParameters::limits`]. Header-parse code rejects pictures
    /// whose declared `width × height` exceeds
    /// [`DecoderLimits::max_pixels_per_frame`] before any allocation.
    limits: DecoderLimits,
    /// Bounded buffer pool used as scratch space for arena-backed
    /// frames returned via [`Decoder::receive_arena_frame`]. Sized at
    /// construction from `limits.max_arenas_in_flight ×
    /// min(max_alloc_bytes_per_frame, DEFAULT_H263_ARENA_BYTES)`.
    /// When every slot is checked out, [`ArenaPool::lease`] returns
    /// [`Error::ResourceExhausted`] — the decoder propagates that to
    /// its caller as natural backpressure.
    ///
    /// Round 2 (oxideav-core 0.2 wiring): the arena lease is deferred
    /// until `receive_arena_frame` is called rather than being taken
    /// at decode time. This keeps the pool slot held for at most the
    /// time between drain and the caller dropping the last `Arc`
    /// clone. The legacy `Decoder::receive_frame` path returns a
    /// heap-backed `VideoFrame` built straight from the queued
    /// `IPicture` and never touches the pool at all.
    arena_pool: Arc<ArenaPool>,
}

/// Cap the per-arena byte size at 2 MiB by default. The largest
/// standard H.263 source format is 16CIF (1408×1152 = ~1.62M luma
/// pels; ~2.43 MB once chroma is included as 4:2:0). We round up to
/// 4 MiB to leave room for stride padding (MB-aligned strides) plus
/// a small fudge factor — enough for every standard format including
/// 16CIF, but small enough that 8 in-flight arenas (the default) max
/// out at 32 MiB even when a malicious stream tries to wedge huge
/// CPFMT dimensions through the parser (the parser also rejects
/// non-standard sizes, but defense in depth).
pub const DEFAULT_H263_ARENA_BYTES: u64 = 4 * 1024 * 1024;

impl H263Decoder {
    pub fn new(codec_id: CodecId) -> Self {
        Self::with_limits(codec_id, DecoderLimits::default())
    }

    /// Construct a decoder with explicit DoS-protection caps. Use this
    /// when wiring a server / sandbox: tighter `limits.max_pixels_per_frame`
    /// rejects malformed pictures earlier; tighter
    /// `limits.max_arenas_in_flight` provides earlier backpressure.
    pub fn with_limits(codec_id: CodecId, limits: DecoderLimits) -> Self {
        // Per-arena byte cap: the smaller of the configured global
        // cap and our per-codec ceiling. The global default of 1 GiB
        // would happily back a single 16CIF picture, but keeping it
        // small means 8 in-flight arenas only consume ~32 MiB (vs.
        // 8 GiB of unused mappings).
        let cap = (limits
            .max_alloc_bytes_per_frame
            .min(DEFAULT_H263_ARENA_BYTES)) as usize;
        let pool = ArenaPool::with_alloc_count_cap(
            limits.max_arenas_in_flight as usize,
            cap,
            limits.max_alloc_count_per_frame,
        );
        Self {
            codec_id,
            buffer: Vec::new(),
            ready_frames: VecDeque::new(),
            pending_pts: None,
            pending_tb: TimeBase::new(1, 90_000),
            eof: false,
            reference: None,
            enable_annex_j: false,
            rps_cache: VecDeque::new(),
            rps_cache_capacity: 4,
            limits,
            arena_pool: pool,
        }
    }

    /// Borrow the in-flight DoS limits in effect for this decoder.
    pub fn limits(&self) -> &DecoderLimits {
        &self.limits
    }

    /// Borrow the arena pool. Tests and tooling can probe pool state
    /// (e.g. lease N+1 arenas to assert
    /// [`Error::ResourceExhausted`]).
    pub fn arena_pool(&self) -> &Arc<ArenaPool> {
        &self.arena_pool
    }

    /// Set the maximum number of decoded pictures retained for Annex N
    /// (Reference Picture Selection) TRP-based lookup. Default 4.
    pub fn set_rps_cache_capacity(&mut self, n: usize) {
        self.rps_cache_capacity = n.max(1);
        while self.rps_cache.len() > self.rps_cache_capacity {
            self.rps_cache.pop_front();
        }
    }

    /// Number of pictures currently retained for Annex N TRP lookup.
    pub fn rps_cache_len(&self) -> usize {
        self.rps_cache.len()
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
        // DoS check: reject declared dimensions whose pixel count exceeds
        // [`DecoderLimits::max_pixels_per_frame`] BEFORE any allocation.
        // H.263's standard source formats are all small (max 16CIF =
        // ~1.6M pels), but this guards future PLUSPTYPE custom-size
        // support and lets server callers tighten the cap further.
        let pixels = (hdr.width as u64).saturating_mul(hdr.height as u64);
        if pixels > self.limits.max_pixels_per_frame {
            return Err(Error::resource_exhausted(format!(
                "h263 picture {}x{} exceeds DecoderLimits::max_pixels_per_frame={}",
                hdr.width, hdr.height, self.limits.max_pixels_per_frame
            )));
        }
        match hdr.coding_type {
            PictureCodingType::Intra => {
                let mut pic = if hdr.sac_mode {
                    // Annex E SAC body — arithmetic-coded MB layer per §E.7.
                    crate::mb_sac::decode_i_picture_sac(&hdr, bytes)?
                } else {
                    decode_i_picture(&mut br, &hdr, bytes)?
                };
                self.maybe_deblock(&mut pic, &hdr);
                // Annex N — push into RPS cache before stamping into
                // `self.reference`. Even non-RPS streams populate the cache
                // so that a later RPS-enabled stream lookup can succeed if
                // the matching encoder happens to address an older TR.
                self.push_rps_cache(hdr.temporal_reference as u16, pic.clone());
                self.ready_frames.push_back((pic.clone(), self.pending_pts));
                self.reference = Some(pic);
                Ok(())
            }
            PictureCodingType::Predicted => {
                // Annex N (RPS) — when the picture header signalled TRPI=1
                // and the requested TR is in our cache, use that; otherwise
                // fall back to the most recent reference (matches §N.5
                // "the most recent temporally previous anchor picture shall
                // be used for prediction" when TRP is not present, and is
                // the documented fall-back when TRP is requested but
                // unavailable).
                let reference = self.pick_reference_for(&hdr).ok_or_else(|| {
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
                // Round 14 — Annex G PB-frames branch. PB-frames pair the
                // P-half (decoded just like a regular P-picture but with
                // MODB / CBPB / MVDB woven in per MB) with a B-half
                // reconstructed from §G.4 / §G.5 bidirectional MC. We emit
                // the B frame first (display order), then the P frame.
                if hdr.pb_frames {
                    let reference = reference.clone();
                    let (p_pic, b_pic) = decode_pb_picture(&mut br, &hdr, bytes, &reference)?;
                    let mut p_pic = p_pic;
                    self.maybe_deblock(&mut p_pic, &hdr);
                    // B-frame is **not** stored as MC reference (§G.1 / §5.1.22);
                    // only the P-half goes into the cache.
                    self.push_rps_cache(hdr.temporal_reference as u16, p_pic.clone());
                    self.ready_frames.push_back((b_pic, self.pending_pts));
                    self.ready_frames
                        .push_back((p_pic.clone(), self.pending_pts));
                    self.reference = Some(p_pic);
                    return Ok(());
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
                self.push_rps_cache(hdr.temporal_reference as u16, pic.clone());
                self.ready_frames.push_back((pic.clone(), self.pending_pts));
                self.reference = Some(pic);
                Ok(())
            }
        }
    }

    /// Push `pic` into the RPS cache under key `tr`. If `tr` already
    /// existed (rewrap or duplicate), the older copy is removed first so
    /// the LRU stays well-defined.
    fn push_rps_cache(&mut self, tr: u16, pic: IPicture) {
        // Remove any prior entry for the same TR (matches the §N.5
        // "first-in, first-out" intent — duplicate entries would just
        // waste cache space).
        if let Some(idx) = self.rps_cache.iter().position(|(t, _)| *t == tr) {
            self.rps_cache.remove(idx);
        }
        self.rps_cache.push_back((tr, pic));
        while self.rps_cache.len() > self.rps_cache_capacity {
            self.rps_cache.pop_front();
        }
    }

    /// Pick the reference picture for the next P-picture decode, honouring
    /// Annex N TRP if present. Returns `None` when neither a TRP-keyed
    /// cache hit nor a "most recent reference" candidate exists.
    fn pick_reference_for(&self, hdr: &PictureHeader) -> Option<&IPicture> {
        if hdr.rps_mode && hdr.trpi {
            // Try TRP-keyed lookup first. If miss, fall back to most-recent
            // (§N.5 last paragraph allows a forced INTRA update by external
            // means; we degrade gracefully here so well-formed streams
            // don't fail on transient cache misses).
            if let Some((_, pic)) = self.rps_cache.iter().rev().find(|(t, _)| *t == hdr.trp) {
                return Some(pic);
            }
        }
        self.reference.as_ref()
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

/// Decode the PB-frame body (Annex G). Returns `(p_recon, b_recon)` where
/// `p_recon` is the freshly reconstructed P-half and `b_recon` is the
/// §G.4 / §G.5 bidirectional B-half. The B-half is **display-order
/// preceding** the P-half (the spec interleaves them for compression but the
/// caller emits them as B then P).
pub fn decode_pb_picture(
    br: &mut BitReader<'_>,
    hdr: &PictureHeader,
    bytes: &[u8],
    reference: &IPicture,
) -> Result<(IPicture, IPicture)> {
    let mb_w = hdr.width.div_ceil(16) as usize;
    let mb_h = hdr.height.div_ceil(16) as usize;
    let (_num_gobs, mb_rows_per_gob) = hdr
        .source_format
        .gob_layout()
        .ok_or_else(|| Error::invalid("h263: source format has no GOB layout"))?;
    let mut p_pic = IPicture::new(hdr.width as usize, hdr.height as usize);
    let mut quant = hdr.pquant as u32;
    let gob_starts = collect_gob_offsets(bytes);

    let mut mv_grid = MvGrid::new(mb_w, mb_h);
    let umv_mode = UmvMode::from_header(hdr);

    // PB-frames don't combine with Annex F in our scope — single-pass.
    if hdr.advanced_prediction {
        return Err(Error::unsupported(
            "h263 PB-frames + Annex F (Advanced Prediction): not yet supported",
        ));
    }
    if hdr.sac_mode {
        return Err(Error::unsupported(
            "h263 PB-frames + Annex E (SAC): not yet supported",
        ));
    }

    // Per-MB MVDB deltas (zero when MODB indicated no MVDB). Used by §G.4.
    let mut mb_mvdbs: Vec<(i32, i32)> = vec![(0, 0); mb_w * mb_h];
    // Per-MB B-block residual buffer. Filled by `decode_p_mb_pb` when CBPB
    // is non-zero; left zero otherwise (the §G.5 reconstruction then uses
    // pure bidirectional MC for that block).
    let mut b_residuals = vec![0i16; mb_w * mb_h * 6 * 64];

    for mb_y in 0..mb_h {
        if mb_y > 0 && (mb_y as u32) % mb_rows_per_gob == 0 {
            let consumed = try_consume_gob_header(br, &gob_starts, hdr, &mut quant)?;
            if consumed {
                mv_grid = MvGrid::new(mb_w, mb_h);
            }
        }
        for mb_x in 0..mb_w {
            let (new_q, info) = decode_p_mb_pb(
                br,
                mb_x,
                mb_y,
                quant,
                &mut p_pic,
                reference,
                &mut mv_grid,
                umv_mode,
                hdr.dbquant,
            )
            .map_err(|e| {
                Error::invalid(format!(
                    "h263 PB-picture MB ({mb_x},{mb_y}) (q={quant}): {e}"
                ))
            })?;
            quant = new_q;
            let mb_idx = mb_y * mb_w + mb_x;
            mb_mvdbs[mb_idx] = info.mvdb;
            // Splice the per-MB B-residual into the picture-wide buffer.
            let base = mb_idx * 6 * 64;
            b_residuals[base..base + 6 * 64].copy_from_slice(&info.b_residual);
        }
    }

    // §G.4 / §G.5 — reconstruct the B-half. TRD = 1 by spec when the encoder
    // emits successive PB pictures; we use a constant TRD = 2 (matching the
    // common "B sits at the midpoint of the P→P gap"). TRB comes from the
    // picture header.
    let trb = hdr.trb.max(1) as i32;
    // TRD = TRB + 1 in the simplest case (one B between two Ps); the spec
    // §G.4 derives it from the picture-header TR delta but baseline H.263
    // doesn't carry that explicitly here. Round-14 picks TRD = TRB + 1 so
    // the B sits proportionally where the encoder placed it.
    let trd = (trb + 1).max(1);

    let mb_motions: Vec<MbMotion> = (0..(mb_w * mb_h)).map(|i| mv_grid.mvs[i]).collect();
    let b_pic = crate::pb::reconstruct_b_picture(
        hdr.width as usize,
        hdr.height as usize,
        reference,
        &p_pic,
        &mb_motions,
        &mb_mvdbs,
        &b_residuals,
        trb,
        trd,
    );

    Ok((p_pic, b_pic))
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

/// Build an arena-backed [`oxideav_core::arena::sync::Frame`] from an
/// `IPicture` by leasing one arena from `pool` and copying the three
/// stride-packed YUV 4:2:0 planes into it.
///
/// Returns [`Error::ResourceExhausted`] when the pool is full
/// (back-pressure: a slow downstream consumer must drop a previously-
/// emitted frame before we can lease again — at the
/// `Decoder::receive_frame` / `receive_arena_frame` API level this
/// surfaces as the same error variant the caller would already match
/// on).
///
/// The returned `Frame` keeps the arena alive via its single
/// `Arc<FrameInner>` owner; drop the `Frame` (and any clones) to
/// return the buffer to the pool. Both [`Decoder::receive_frame`]
/// (legacy heap-owned path) and [`Decoder::receive_arena_frame`]
/// (zero-copy path) consume the same arena `Frame` — the difference
/// is whether the caller sees the bytes through an extra memcpy or
/// straight out of the arena buffer.
pub fn pic_to_arena_frame(
    pic: &IPicture,
    pts: Option<i64>,
    pool: &Arc<ArenaPool>,
) -> Result<oxideav_core::arena::sync::Frame> {
    let w = pic.width;
    let h = pic.height;
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    // Lazy DoS check: the byte total for stride-packed planes (already
    // bounded by the pixel-cap check at the picture-header layer, but
    // re-checked here so the arena-cap error message names the actual
    // figures rather than letting the bump allocator phrase it).
    let y_bytes = w
        .checked_mul(h)
        .ok_or_else(|| Error::resource_exhausted("h263 luma size overflow".to_string()))?;
    let c_bytes = cw
        .checked_mul(ch)
        .ok_or_else(|| Error::resource_exhausted("h263 chroma size overflow".to_string()))?;
    let plane_bytes = y_bytes
        .checked_add(c_bytes)
        .and_then(|n| n.checked_add(c_bytes))
        .ok_or_else(|| Error::resource_exhausted("h263 frame size overflow".to_string()))?;
    if plane_bytes > pool.cap_per_arena() {
        return Err(Error::resource_exhausted(format!(
            "h263 frame {}x{} needs {} arena bytes (cap {})",
            w,
            h,
            plane_bytes,
            pool.cap_per_arena()
        )));
    }

    let arena = pool.lease()?;
    // Three small allocs (Y, Cb, Cr) so each plane gets its own
    // contiguous range inside the arena — that's what
    // `arena::sync::FrameInner::plane(i)` will hand back to a caller
    // walking the plane table. Allocation-count pressure is minimal
    // (3 vs. the configured cap of >= max_alloc_count_per_frame).
    let y_off = arena.used();
    let y_buf = arena.alloc::<u8>(y_bytes)?;
    for row in 0..h {
        y_buf[row * w..row * w + w]
            .copy_from_slice(&pic.y[row * pic.y_stride..row * pic.y_stride + w]);
    }
    let cb_off = arena.used();
    let cb_buf = arena.alloc::<u8>(c_bytes)?;
    for row in 0..ch {
        cb_buf[row * cw..row * cw + cw]
            .copy_from_slice(&pic.cb[row * pic.c_stride..row * pic.c_stride + cw]);
    }
    let cr_off = arena.used();
    let cr_buf = arena.alloc::<u8>(c_bytes)?;
    for row in 0..ch {
        cr_buf[row * cw..row * cw + cw]
            .copy_from_slice(&pic.cr[row * pic.c_stride..row * pic.c_stride + cw]);
    }
    let header = FrameHeader::new(w as u32, h as u32, PixelFormat::Yuv420P, pts);
    FrameInner::new(
        arena,
        &[(y_off, y_bytes), (cb_off, c_bytes), (cr_off, c_bytes)],
        header,
    )
}

/// Materialise a stride-packed YUV420P `VideoFrame` from an
/// arena-backed `Frame` by `to_vec()`'ing each plane. Used by
/// [`Decoder::receive_frame`] for callers that want the legacy
/// heap-owned `VideoFrame` API across `Send` boundaries.
fn arena_frame_to_video_frame(
    af: &oxideav_core::arena::sync::Frame,
    pts: Option<i64>,
) -> VideoFrame {
    let hdr = af.header();
    let w = hdr.width as usize;
    let cw = w.div_ceil(2);
    let y_plane = af.plane(0).expect("plane 0 (Y) must exist");
    let cb_plane = af.plane(1).expect("plane 1 (Cb) must exist");
    let cr_plane = af.plane(2).expect("plane 2 (Cr) must exist");
    VideoFrame {
        pts,
        planes: vec![
            VideoPlane {
                stride: w,
                data: y_plane.to_vec(),
            },
            VideoPlane {
                stride: cw,
                data: cb_plane.to_vec(),
            },
            VideoPlane {
                stride: cw,
                data: cr_plane.to_vec(),
            },
        ],
    }
}

/// Backwards-compat shim: build a heap-backed `VideoFrame` from an
/// `IPicture` while still going through the arena pool for the
/// per-plane scratch copy. Kept for callers that imported
/// `pic_to_video_frame_arena` directly (tests, encoder roundtrips).
/// Internally builds an arena `Frame` then materialises a `VideoFrame`
/// from it; the arena handle drops at end of scope so the buffer
/// returns to the pool immediately.
pub fn pic_to_video_frame_arena(
    pic: &IPicture,
    pts: Option<i64>,
    _tb: TimeBase,
    pool: &Arc<ArenaPool>,
) -> Result<VideoFrame> {
    let af = pic_to_arena_frame(pic, pts, pool)?;
    let video = arena_frame_to_video_frame(&af, pts);
    drop(af);
    Ok(video)
}

/// Build a stride-packed YUV420P `VideoFrame` from an `IPicture`.
///
/// Stream-level properties (pixel format, width, height, time base) live on
/// the stream's `CodecParameters`; the frame only carries pts + planes. The
/// `_tb` argument is retained for source-compat but ignored.
///
/// **Note:** the on-decoder path now goes through
/// [`pic_to_video_frame_arena`] (DoS-bounded). This function is kept
/// for backwards-compat with downstream callers that build frames
/// directly from an `IPicture` (e.g. encoder roundtrip tests).
pub fn pic_to_video_frame(pic: &IPicture, pts: Option<i64>, _tb: TimeBase) -> VideoFrame {
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
        pts,
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
        if let Some((pic, pts)) = self.ready_frames.pop_front() {
            // Build a heap-backed VideoFrame for the legacy enum.
            // Straight Picture → Vec<u8> per plane; the arena pool is
            // not touched.
            let video = pic_to_video_frame(&pic, pts, self.pending_tb);
            return Ok(Frame::Video(video));
        }
        if self.eof {
            Err(Error::Eof)
        } else {
            Err(Error::NeedMore)
        }
    }

    fn receive_arena_frame(&mut self) -> Result<oxideav_core::arena::sync::Frame> {
        if let Some((pic, pts)) = self.ready_frames.pop_front() {
            // Lease one arena and copy the YUV planes in. The arena
            // slot stays checked out until the caller drops the last
            // `Arc` clone of the returned frame; pool exhaustion
            // surfaces as `Error::ResourceExhausted` (natural
            // backpressure).
            return pic_to_arena_frame(&pic, pts, &self.arena_pool);
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
        self.rps_cache.clear();
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
