//! H.263 picture header parser — Annex C §5.1 of ITU-T Rec. H.263 (02/98)
//! with the H.263+ PLUSPTYPE / OPPTYPE extension from the 01/2005 revision.
//!
//! Baseline layout (no PLUSPTYPE):
//!
//! | Field       | Bits  | Notes                                            |
//! |-------------|-------|--------------------------------------------------|
//! | PSC         | 22    | `0000 0000 0000 0000 1 00000`                    |
//! | TR          | 8     | Temporal reference                               |
//! | PTYPE bit 1 | 1     | Always `1` (start-code emulation prevention)     |
//! | PTYPE bit 2 | 1     | Always `0` (distinguishes from H.261)            |
//! | PTYPE bit 3 | 1     | Split-screen indicator                           |
//! | PTYPE bit 4 | 1     | Document-camera indicator                        |
//! | PTYPE bit 5 | 1     | Freeze-picture release                           |
//! | Source fmt  | 3     | 1=sub-QCIF .. 5=16CIF; 7 = PLUSPTYPE follows     |
//! | PType       | 1     | 0 = I-picture, 1 = P-picture                     |
//! | Annex flags | 4     | UMV (D), SAC (E), AP (F), PB-frames (G)          |
//! | PQUANT      | 5     | Quantiser 1..=31                                 |
//! | CPM         | 1     | Continuous presence multipoint mode              |
//! | PSBI        | 2     | Present iff CPM == 1                             |
//! | TRB         | 3     | Present iff PB-frames mode                       |
//! | DBQUANT     | 2     | Present iff PB-frames mode                       |
//! | PEI/PSPARE  | n     | 1-bit PEI, then 8-bit PSPARE if PEI==1, repeat   |
//!
//! When `Source fmt == 7` the picture carries a PLUSPTYPE block instead of
//! the standard PTYPE tail. PLUSPTYPE is parsed by
//! [`parse_plusptype_tail`] below — we recognise the full syntax so that
//! streams built around H.263+ can be read up to the point where an
//! actually-unsupported annex is signalled (at which point we return a
//! specific `Error::Unsupported`). Custom Picture Format (CPFMT) is
//! accepted when the dimensions happen to match one of the standard
//! source formats; non-standard sizes are rejected for now as the MB grid
//! and motion-compensation paths below still assume the fixed formats.
//!
//! GOB data immediately follows the header (no further alignment required).

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

/// H.263 source-format codes (PTYPE bits 6-8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SourceFormat {
    /// Forbidden value `000`.
    Forbidden,
    /// `001` — sub-QCIF, 128 × 96 luma.
    SubQcif,
    /// `010` — QCIF, 176 × 144 luma.
    Qcif,
    /// `011` — CIF, 352 × 288 luma.
    Cif,
    /// `100` — 4CIF, 704 × 576 luma.
    FourCif,
    /// `101` — 16CIF, 1408 × 1152 luma.
    SixteenCif,
    /// `110` — reserved.
    Reserved,
    /// `111` — extended PTYPE (H.263+); not supported.
    Extended,
}

impl SourceFormat {
    pub fn from_code(c: u8) -> Self {
        match c & 0x7 {
            0 => SourceFormat::Forbidden,
            1 => SourceFormat::SubQcif,
            2 => SourceFormat::Qcif,
            3 => SourceFormat::Cif,
            4 => SourceFormat::FourCif,
            5 => SourceFormat::SixteenCif,
            6 => SourceFormat::Reserved,
            7 => SourceFormat::Extended,
            _ => unreachable!(),
        }
    }

    /// Picture dimensions `(width, height)` in luma samples. Returns `None`
    /// for forbidden / reserved / extended formats.
    pub fn dimensions(self) -> Option<(u32, u32)> {
        match self {
            SourceFormat::SubQcif => Some((128, 96)),
            SourceFormat::Qcif => Some((176, 144)),
            SourceFormat::Cif => Some((352, 288)),
            SourceFormat::FourCif => Some((704, 576)),
            SourceFormat::SixteenCif => Some((1408, 1152)),
            _ => None,
        }
    }

    /// Pick the source-format code that exactly matches `(w, h)`. Returns
    /// `None` for non-standard dimensions (H.263 baseline cannot signal
    /// arbitrary sizes — that requires PLUSPTYPE).
    pub fn for_dimensions(w: u32, h: u32) -> Option<Self> {
        match (w, h) {
            (128, 96) => Some(SourceFormat::SubQcif),
            (176, 144) => Some(SourceFormat::Qcif),
            (352, 288) => Some(SourceFormat::Cif),
            (704, 576) => Some(SourceFormat::FourCif),
            (1408, 1152) => Some(SourceFormat::SixteenCif),
            _ => None,
        }
    }

    /// Number of GOBs in a picture of this source format. Sub-QCIF, QCIF use
    /// 6 GOBs of one MB row each (and one GOB at half height for sub-QCIF).
    /// CIF / 4CIF / 16CIF have GOBs spanning multiple MB rows.
    ///
    /// Returns `(num_gobs, mb_rows_per_gob)`.
    pub fn gob_layout(self) -> Option<(u32, u32)> {
        match self {
            SourceFormat::SubQcif => Some((6, 1)),
            SourceFormat::Qcif => Some((9, 1)),
            SourceFormat::Cif => Some((18, 1)),
            SourceFormat::FourCif => Some((18, 2)),
            SourceFormat::SixteenCif => Some((18, 4)),
            _ => None,
        }
    }
}

/// Picture coding type (PTYPE bit 9).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PictureCodingType {
    Intra,
    Predicted,
}

/// Parsed H.263 picture header.
#[derive(Clone, Debug)]
pub struct PictureHeader {
    pub temporal_reference: u8,
    pub split_screen: bool,
    pub document_camera: bool,
    pub freeze_release: bool,
    pub source_format: SourceFormat,
    pub coding_type: PictureCodingType,
    pub umv_mode: bool,
    pub sac_mode: bool,
    pub advanced_prediction: bool,
    pub pb_frames: bool,
    pub pquant: u8,
    pub cpm: bool,
    pub psbi: u8,
    pub trb: u8,
    pub dbquant: u8,
    pub width: u32,
    pub height: u32,
    /// Set to `true` when the picture header carried a PLUSPTYPE block
    /// (source format code `111`). Baseline streams leave this at `false`.
    pub plusptype: bool,
    /// Annex J (Deblocking Filter) mode — only meaningful when [`Self::plusptype`]
    /// is set; baseline streams cannot signal DF inside the bitstream and
    /// leave this at `false`. Mirrors the `DF` bit of OPPTYPE.
    pub deblocking_filter: bool,
    /// UUI = "01" — unlimited range (§5.1.9). Only meaningful when UMV is on
    /// in a PLUSPTYPE header. `false` means "UUI = 1" (range limited per
    /// Tables D.1/D.2) or UMV off.
    pub uui_unlimited: bool,
}

/// Parse the picture header that follows the 22-bit PSC.
///
/// `br` must be positioned at the start of the byte that contains the PSC.
/// On entry the function consumes the 22-bit PSC and validates it.
pub fn parse_picture_header(br: &mut BitReader<'_>) -> Result<PictureHeader> {
    // PSC: 22 bits = 0x0000_8000 >> 10 in MSB form. Read as 22 bits and
    // compare against the constant.
    let psc = br.read_u32(22)?;
    // 0000 0000 0000 0000 1 00000 = bit 17 is 1, the remaining low 5 bits
    // of the 22-bit value are zero. As an integer this is 0x20.
    #[allow(clippy::unusual_byte_groupings)]
    const PSC_VALUE: u32 = 0b00_0000_0000_0000_0000_1_00000;
    if psc != PSC_VALUE {
        return Err(Error::invalid(format!(
            "h263 picture: bad PSC 0x{psc:06x} (want 0x{PSC_VALUE:06x})"
        )));
    }

    let tr = br.read_u32(8)? as u8;

    // PTYPE bits 1-13.
    let always_one = br.read_u1()?;
    if always_one != 1 {
        return Err(Error::invalid("h263 picture: PTYPE bit 1 must be 1"));
    }
    let always_zero = br.read_u1()?;
    if always_zero != 0 {
        return Err(Error::invalid("h263 picture: PTYPE bit 2 must be 0"));
    }
    let split_screen = br.read_u1()? == 1;
    let document_camera = br.read_u1()? == 1;
    let freeze_release = br.read_u1()? == 1;
    let src_code = br.read_u32(3)? as u8;
    let source_format = SourceFormat::from_code(src_code);

    if matches!(
        source_format,
        SourceFormat::Forbidden | SourceFormat::Reserved
    ) {
        return Err(Error::invalid("h263 picture: forbidden source format"));
    }

    if source_format == SourceFormat::Extended {
        return parse_plusptype_tail(br, tr, split_screen, document_camera, freeze_release);
    }

    let coding_bit = br.read_u1()?;
    let coding_type = if coding_bit == 0 {
        PictureCodingType::Intra
    } else {
        PictureCodingType::Predicted
    };
    let umv_mode = br.read_u1()? == 1;
    let sac_mode = br.read_u1()? == 1;
    let advanced_prediction = br.read_u1()? == 1;
    let pb_frames = br.read_u1()? == 1;

    if sac_mode {
        // Annex E arithmetic coder + every §E.8 probability model are
        // implemented in `crate::sac`, but the wiring that replaces VLC
        // decodes at the MB / block layer with SAC calls is not yet in
        // place. Reject the stream with a diagnostic that points at the
        // actual integration gap rather than leaving callers guessing.
        return Err(Error::unsupported(
            "h263 Annex E syntax-based arithmetic coding: MB-layer wiring \
             pending (arithmetic coder lives in crate::sac)",
        ));
    }
    // Annex F (Advanced Prediction — 4MV + OBMC) is accepted on baseline
    // PTYPE streams and handled by the P-MB decoder via `decode_p_mb`.
    // The PLUSPTYPE form is still rejected (Table D.3 intersection /
    // OPPTYPE AP bit still returns Unsupported in `parse_plusptype_tail`).
    if pb_frames {
        return Err(Error::unsupported(
            "h263 Annex G PB-frames mode: follow-up (B-pictures)",
        ));
    }

    let pquant = br.read_u32(5)? as u8;
    if pquant == 0 {
        return Err(Error::invalid("h263 picture: PQUANT == 0"));
    }

    let cpm = br.read_u1()? == 1;
    let psbi = if cpm { br.read_u32(2)? as u8 } else { 0 };
    if cpm {
        return Err(Error::unsupported(
            "h263 CPM continuous-presence multipoint: follow-up",
        ));
    }

    // PB-frames extras would go here if pb_frames were set — already rejected.
    let trb = 0u8;
    let dbquant = 0u8;

    // PEI / PSPARE loop.
    loop {
        let pei = br.read_u1()?;
        if pei == 0 {
            break;
        }
        let _pspare = br.read_u32(8)?;
    }

    let (width, height) = source_format
        .dimensions()
        .ok_or_else(|| Error::unsupported("h263 picture: source format has no fixed dimensions"))?;

    Ok(PictureHeader {
        temporal_reference: tr,
        split_screen,
        document_camera,
        freeze_release,
        source_format,
        coding_type,
        umv_mode,
        sac_mode,
        advanced_prediction,
        pb_frames,
        pquant,
        cpm,
        psbi,
        trb,
        dbquant,
        width,
        height,
        plusptype: false,
        deblocking_filter: false,
        uui_unlimited: false,
    })
}

/// Parse the PLUSPTYPE tail (H.263+, Annex U of the 01/2005 edition). Called
/// after the baseline PTYPE prefix reaches source-format code `111`.
///
/// Layout after source-format = 111:
///
/// | Field    | Bits | Notes                                             |
/// |----------|------|---------------------------------------------------|
/// | UFEP     | 3    | `000` = MPPTYPE only; `001` = OPPTYPE follows     |
/// | OPPTYPE  | 18   | Only if UFEP == `001`                             |
/// | MPPTYPE  | 9    | Picture-type code + RPR/RRU/RTYPE + `001` marker  |
/// | CPM      | 1    | Continuous presence multipoint                    |
/// | PSBI     | 2    | Only if CPM == 1                                  |
/// | CPFMT    | 23   | Only if OPPTYPE signalled custom format           |
/// | CPCFC    | 8    | Only if OPPTYPE signalled custom PCF              |
/// | ETR      | 8    | Only if OPPTYPE signalled custom PCF              |
/// | UUI      | 1..2 | Unlimited Unrestricted MV indicator (Annex D)     |
/// | SSS      | 2    | Slice Structured sub-mode (Annex K)               |
/// | ELNUM    | 4    | Scalability (Annex O)                             |
/// | RLNUM    | 4    | Scalability (Annex O)                             |
/// | RPSMF    | 3    | Reference Picture Selection (Annex N)             |
/// | TRPI     | 1    | RPS TRP indicator                                 |
/// | TRP      | 10   | RPS                                               |
/// | BCI      | 1..  | Backchannel message indicator                     |
/// | RPRP     | var  | Reference Picture Resampling (Annex P)            |
/// | PQUANT   | 5    |                                                   |
/// | MVD      | 1    | Only if RTYPE signalled B-pictures                |
/// | CPM      | 1    | (second copy when UFEP == `000`)                  |
/// | DBQUANT  | 2    | If B-pictures                                     |
/// | PEI loop | n    |                                                   |
///
/// This parser recognises the bitstream shape and either returns a baseline-
/// compatible [`PictureHeader`] when the stream happens to only use the
/// PLUSPTYPE-form-of-standard features (standard source format + DF bit for
/// Annex J), or returns `Error::Unsupported` with a specific diagnostic
/// naming the offending annex.
fn parse_plusptype_tail(
    br: &mut BitReader<'_>,
    tr: u8,
    split_screen: bool,
    document_camera: bool,
    freeze_release: bool,
) -> Result<PictureHeader> {
    let ufep = br.read_u32(3)?;

    // OPPTYPE (18 bits, MSB first per §5.1.4.2) — present only when UFEP ==
    // `001`. If absent, the decoder is expected to carry the optional-
    // feature flags from the last full PLUSPTYPE seen on the stream. We
    // don't track cross-picture state yet, so we only accept pictures that
    // either carry the full OPPTYPE (and use a subset we support) or
    // default everything to baseline.
    //
    // Field layout (spec bits 1..=18 mapped to helper bit positions where
    // bit `N` is the (18-N)-th LSB of the 18-bit value — spec bit 1 is the
    // MSB of OPPTYPE, read first from the stream):
    //   Bits 1-3: Source Format (3 bits, identical codes as baseline PTYPE,
    //             with 6 = custom format).
    //   Bit 4:    Custom PCF.
    //   Bit 5:    UMV (Annex D).
    //   Bit 6:    SAC (Annex E).
    //   Bit 7:    AP  (Annex F).
    //   Bit 8:    AIC (Annex I).
    //   Bit 9:    DF  (Annex J deblocking).
    //   Bit 10:   SS  (Annex K slice structured).
    //   Bit 11:   RPS (Annex N reference picture selection).
    //   Bit 12:   ISD (Annex R).
    //   Bit 13:   AIV (Annex S alternative inter VLC).
    //   Bit 14:   MQ  (Annex T modified quantization).
    //   Bit 15:   Marker, must be "1".
    //   Bits 16-18: Reserved, must be "000".
    let (opptype_src_format, custom_src_format, custom_pcf, umv_mode, df_mode, ap_mode) =
        if ufep == 0b001 {
            let opptype = br.read_u32(18)?;
            // MSB-first helpers: `bit(k)` returns the k-th spec bit
            // (1-indexed, where 1 is the MSB of the 18-bit field).
            let bit = |k: u32| -> u32 { (opptype >> (18 - k)) & 1 };
            let src_fmt_code = (opptype >> (18 - 3)) & 0b111;
            let custom_pcf = bit(4) != 0;
            let umv = bit(5) != 0;
            let sac = bit(6) != 0;
            let ap = bit(7) != 0;
            let aic = bit(8) != 0;
            let df = bit(9) != 0;
            let sss = bit(10) != 0;
            let rps = bit(11) != 0;
            let isd = bit(12) != 0;
            let aiv = bit(13) != 0;
            let mq = bit(14) != 0;
            let marker = bit(15);
            // Reserved bits 16..=18 must all be zero.
            let reserved = (opptype & 0b111) as u8;
            if marker != 1 {
                return Err(Error::invalid("h263 PLUSPTYPE: OPPTYPE marker bit != 1"));
            }
            if reserved != 0 {
                return Err(Error::invalid(format!(
                    "h263 PLUSPTYPE: OPPTYPE reserved bits != 000 (got {reserved:03b})"
                )));
            }
            let custom_src = src_fmt_code == 0b110;
            if src_fmt_code == 0b000 || src_fmt_code == 0b111 {
                return Err(Error::invalid(format!(
                    "h263 PLUSPTYPE: reserved OPPTYPE source format {src_fmt_code:03b}"
                )));
            }
            if sac {
                return Err(Error::unsupported(
                    "h263 Annex E syntax-based arithmetic coding (PLUSPTYPE): \
                 MB-layer wiring pending (arithmetic coder lives in crate::sac)",
                ));
            }
            // Annex F AP in PLUSPTYPE — accepted, same 4MV/OBMC path as the
            // baseline PTYPE form. `advanced_prediction` is reflected back to
            // the caller via the returned `PictureHeader`.
            if aic {
                return Err(Error::unsupported(
                    "h263 Annex I advanced intra coding (PLUSPTYPE): follow-up",
                ));
            }
            // Annex K (Slice Structured mode) is detected here with a specific
            // diagnostic. ffmpeg's `h263p -umv 1` bundles this with Annex D+UMV,
            // so decoders that want to reach the MVD VLC layer need to bail out
            // at the picture header rather than silently misread slice headers
            // as GOB headers.
            if sss {
                return Err(Error::unsupported(
                    "h263 Annex K slice structured mode: follow-up (bit 10 of OPPTYPE; \
                 slice headers replace GOB headers per §K.2)",
                ));
            }
            if rps {
                return Err(Error::unsupported(
                    "h263 Annex N reference picture selection: follow-up",
                ));
            }
            if isd {
                return Err(Error::unsupported(
                    "h263 Annex R independent segment decoding: follow-up",
                ));
            }
            if aiv {
                return Err(Error::unsupported(
                    "h263 Annex S alternative inter VLC: follow-up",
                ));
            }
            if mq {
                return Err(Error::unsupported(
                    "h263 Annex T modified quantization: follow-up",
                ));
            }
            // PLUSPTYPE + UMV: Table D.3 MVD VLC + UUI + direct `predictor +
            // differential` reconstruction (no sign-of-predictor cascade). Both
            // encode and decode paths are in `crate::motion`; wired into
            // `decode_p_mb` / `decode_p_mb_pass1` below via the `umv_plusptype`
            // flag on `PictureHeader`.
            (src_fmt_code as u8, custom_src, custom_pcf, umv, df, ap)
        } else if ufep == 0b000 {
            // Inherit previous-picture OPPTYPE state. Since we do not yet retain
            // OPPTYPE across pictures, we treat the inherited state as "baseline
            // with DF and custom format both off" — the same default a freshly
            // constructed decoder would pick. This is good enough for streams
            // whose very first PLUSPTYPE picture supplies a full OPPTYPE and
            // whose subsequent P-pictures keep the same options (the common
            // case); anything else will diverge downstream on the first feature
            // bit that actually matters. `opptype_src_format = 0` signals
            // "no OPPTYPE, dimensions must be inferable from later CPFMT (which
            // in turn will not be present for UFEP==000)".
            (0, false, false, false, false, false)
        } else {
            return Err(Error::invalid(format!(
                "h263 PLUSPTYPE: invalid UFEP {ufep:03b}"
            )));
        };

    // MPPTYPE (9 bits): PCT(3) | RPR(1) | RRU(1) | RTYPE(1) | `001` marker
    let mpptype = br.read_u32(9)?;
    let pct = (mpptype >> 6) & 0b111;
    let rpr = (mpptype >> 5) & 1 != 0;
    let rru = (mpptype >> 4) & 1 != 0;
    let rtype = (mpptype >> 3) & 1 != 0;
    let mpp_marker = mpptype & 0b111;
    if mpp_marker != 0b001 {
        return Err(Error::invalid(format!(
            "h263 PLUSPTYPE: MPPTYPE marker != 001 (got {mpp_marker:03b})"
        )));
    }
    let _ = rtype;
    if rpr {
        return Err(Error::unsupported(
            "h263 Annex P reference picture resampling: follow-up",
        ));
    }
    if rru {
        return Err(Error::unsupported(
            "h263 Annex Q reduced-resolution update: follow-up",
        ));
    }
    let coding_type = match pct {
        0b000 => PictureCodingType::Intra,
        0b001 => PictureCodingType::Predicted,
        0b010 => {
            return Err(Error::unsupported(
                "h263 Improved PB-frames picture (PCT=010): follow-up",
            ));
        }
        0b011 => {
            return Err(Error::unsupported("h263 B-picture (PCT=011): follow-up"));
        }
        0b100 => {
            return Err(Error::unsupported(
                "h263 EI-picture (PCT=100, Annex O scalability): follow-up",
            ));
        }
        0b101 => {
            return Err(Error::unsupported(
                "h263 EP-picture (PCT=101, Annex O scalability): follow-up",
            ));
        }
        _ => {
            return Err(Error::invalid(format!(
                "h263 PLUSPTYPE: reserved picture-type code {pct:03b}"
            )));
        }
    };

    let cpm = br.read_u1()? == 1;
    let psbi = if cpm { br.read_u32(2)? as u8 } else { 0 };
    if cpm {
        return Err(Error::unsupported(
            "h263 CPM continuous-presence multipoint (PLUSPTYPE): follow-up",
        ));
    }

    // CPFMT (23 bits): PAR code (4) | PWI (9) | `1` marker | PHI (9). Only
    // present when OPPTYPE signalled a custom source format (code 110). For
    // the standard codes 001..=101 the dimensions are implied.
    let (width, height) = if custom_src_format {
        let par_code = br.read_u32(4)?;
        let pwi = br.read_u32(9)?;
        let marker = br.read_u1()?;
        let phi = br.read_u32(9)?;
        if marker != 1 {
            return Err(Error::invalid("h263 PLUSPTYPE CPFMT: marker bit != 1"));
        }
        let _ = par_code;
        // Custom size is (PWI+1)*4 by (PHI)*4 per §5.1.5.
        let w = (pwi + 1) * 4;
        let h = phi * 4;
        if w == 0 || h == 0 {
            return Err(Error::invalid("h263 PLUSPTYPE CPFMT: zero-sized picture"));
        }
        // We currently only support the standard source formats for the
        // actual MB/GOB layout below. Accept custom dimensions only when
        // they happen to coincide with one.
        if SourceFormat::for_dimensions(w, h).is_none() {
            return Err(Error::unsupported(format!(
                "h263 PLUSPTYPE custom picture size {w}x{h}: only standard \
                 sub-QCIF/QCIF/CIF/4CIF/16CIF dimensions supported so far"
            )));
        }
        (w, h)
    } else if ufep == 0b001 {
        // OPPTYPE present → standard source-format code in bits 1-3.
        let (w, h) = SourceFormat::from_code(opptype_src_format)
            .dimensions()
            .ok_or_else(|| {
                Error::invalid(format!(
                    "h263 PLUSPTYPE OPPTYPE source format {opptype_src_format:03b} has no dimensions"
                ))
            })?;
        (w, h)
    } else {
        // When not signalled, size is inherited from the last full PLUSPTYPE
        // which in our stateless parse means "we don't know". Baseline
        // decoding is impossible without a size.
        return Err(Error::unsupported(
            "h263 PLUSPTYPE with UFEP=000: cross-picture OPPTYPE inheritance \
             (picture size) is not yet tracked",
        ));
    };

    if custom_pcf {
        let _cpcfc = br.read_u32(8)?;
        let _etr = br.read_u32(8)?;
        return Err(Error::unsupported(
            "h263 custom picture clock frequency (CPCFC/ETR): follow-up",
        ));
    }

    // UUI (Annex D) — variable-length, 1 or 2 bits, only present when OPPTYPE
    // had UMV set. "1" → range limited per Tables D.1/D.2. "01" → range
    // unlimited except by picture size (§5.1.9). Table D.3 is used for the
    // MVD VLC regardless of UUI setting (see `motion::decode_mvd_table_d3`).
    //
    // UUI is present only when `ufep == 001` per §5.1.9; inherited otherwise.
    // In the UFEP=000 branch above we reject with an Unsupported diagnostic,
    // so by the time we get here `ufep == 001` is guaranteed when `umv_mode`
    // is true.
    let uui_unlimited = if umv_mode {
        let first = br.read_u1()?;
        if first == 1 {
            false
        } else {
            let second = br.read_u1()?;
            if second != 1 {
                return Err(Error::invalid("h263 PLUSPTYPE: invalid UUI prefix"));
            }
            true
        }
    } else {
        false
    };

    // SSS (Annex K) — only if slice structured was signalled (rejected above).

    // PQUANT
    let pquant = br.read_u32(5)? as u8;
    if pquant == 0 {
        return Err(Error::invalid("h263 PLUSPTYPE: PQUANT == 0"));
    }

    // PEI / PSPARE loop.
    loop {
        let pei = br.read_u1()?;
        if pei == 0 {
            break;
        }
        let _pspare = br.read_u32(8)?;
    }

    let source_format = SourceFormat::for_dimensions(width, height).expect("validated above");

    Ok(PictureHeader {
        temporal_reference: tr,
        split_screen,
        document_camera,
        freeze_release,
        source_format,
        coding_type,
        umv_mode,
        sac_mode: false,
        advanced_prediction: ap_mode,
        pb_frames: false,
        pquant,
        cpm,
        psbi,
        trb: 0,
        dbquant: 0,
        width,
        height,
        plusptype: true,
        deblocking_filter: df_mode,
        uui_unlimited,
    })
}

#[cfg(test)]
#[allow(
    clippy::identity_op,
    clippy::erasing_op,
    clippy::double_parens,
    clippy::unusual_byte_groupings
)]
mod tests {
    use super::*;

    /// Build the byte sequence for a minimal sub-QCIF I-picture header with
    /// PQUANT=5, no CPM, no PEI, no annexes. Mirrors the bit layout produced
    /// by `ffmpeg -c:v h263 -qscale:v 5` for a 128×96 source.
    fn minimal_subqcif_iframe() -> Vec<u8> {
        // Bit stream (50 bits, padded with zeros to byte boundary):
        //   PSC(22)     = 0000 0000 0000 0000 1 00000
        //   TR(8)       = 00000000
        //   PTYPE(13)   = 1 0 0 0 0 001 0 0 0 0 0
        //                 (marker, id, split, cam, freeze, fmt=1, I, all annex 0)
        //   PQUANT(5)   = 00101 (=5)
        //   CPM(1)      = 0
        //   PEI(1)      = 0
        // Concatenated: 0000 0000 0000 0000 1000 0000 0000 0010 0000 0100 0000 0101 0010 0000
        //              = 00 00 80 02 04 05 20 (with 0x20 trailing)
        vec![0x00, 0x00, 0x80, 0x02, 0x04, 0x05, 0x20]
    }

    #[test]
    fn parses_subqcif_iframe() {
        let data = minimal_subqcif_iframe();
        let mut br = BitReader::new(&data);
        let p = parse_picture_header(&mut br).unwrap();
        assert_eq!(p.temporal_reference, 0);
        assert_eq!(p.source_format, SourceFormat::SubQcif);
        assert_eq!(p.coding_type, PictureCodingType::Intra);
        assert_eq!(p.pquant, 5);
        assert!(!p.cpm);
        assert_eq!(p.width, 128);
        assert_eq!(p.height, 96);
        assert!(!p.plusptype);
        assert!(!p.deblocking_filter);
    }

    /// Tiny MSB-first bit writer used by the PLUSPTYPE synthesis tests
    /// below. Mirrors [`crate::bitwriter::BitWriter`] but kept local so the
    /// picture-header unit tests don't take a dependency on the encoder
    /// module's public surface.
    struct BitBuf {
        bytes: Vec<u8>,
        acc: u64,
        n: u32,
    }
    impl BitBuf {
        fn new() -> Self {
            Self {
                bytes: Vec::new(),
                acc: 0,
                n: 0,
            }
        }
        fn put(&mut self, v: u32, bits: u32) {
            self.acc = (self.acc << bits) | (v as u64 & ((1u64 << bits) - 1));
            self.n += bits;
            while self.n >= 8 {
                self.n -= 8;
                self.bytes.push((self.acc >> self.n) as u8);
            }
        }
        fn finish(mut self) -> Vec<u8> {
            if self.n > 0 {
                self.bytes.push(((self.acc << (8 - self.n)) & 0xff) as u8);
            }
            self.bytes
        }
    }

    /// Build an OPPTYPE u32 from spec-numbered bit fields. Bit 1 is the MSB
    /// of the 18-bit field. The helper takes each optional-mode flag as a
    /// bool and packs the marker + reserved bits for the caller.
    #[allow(clippy::too_many_arguments)]
    fn pack_opptype(
        src_fmt: u32,
        custom_pcf: bool,
        umv: bool,
        sac: bool,
        ap: bool,
        aic: bool,
        df: bool,
        sss: bool,
        rps: bool,
        isd: bool,
        aiv: bool,
        mq: bool,
    ) -> u32 {
        let b = |k: u32, v: u32| (v & 1) << (18 - k);
        // Bits 1-3 (MSB): source-format code (placed at bits 17..=15).
        let srcf = (src_fmt & 0b111) << 15;
        srcf | b(4, custom_pcf as u32)
            | b(5, umv as u32)
            | b(6, sac as u32)
            | b(7, ap as u32)
            | b(8, aic as u32)
            | b(9, df as u32)
            | b(10, sss as u32)
            | b(11, rps as u32)
            | b(12, isd as u32)
            | b(13, aiv as u32)
            | b(14, mq as u32)
            | b(15, 1) // marker
                       // bits 16-18 = reserved 0.
    }

    #[test]
    fn plusptype_qcif_iframe_with_df_flag() {
        let mut w = BitBuf::new();
        // PSC (22) + TR=0 (8) = 30 bits
        w.put(0b00_0000_0000_0000_0000_1_00000, 22);
        w.put(0, 8);
        // PTYPE prefix: marker=1, id=0, split=0, cam=0, freeze=0, fmt=111
        w.put(1, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0b111, 3);
        // PLUSPTYPE tail starts here.
        // UFEP = 001
        w.put(0b001, 3);
        // OPPTYPE: src_fmt=010 (QCIF, standard), DF=1, rest off.
        let opptype = pack_opptype(
            0b010, false, false, false, false, false, true, false, false, false, false, false,
        );
        w.put(opptype, 18);
        // MPPTYPE (9): PCT=000 (I) | RPR=0 | RRU=0 | RTYPE=0 | marker=001
        w.put(0b000_0_0_0_001, 9);
        // CPM = 0
        w.put(0, 1);
        // No CPFMT (std src fmt). No UUI (UMV off).
        // PQUANT = 5
        w.put(5, 5);
        // PEI = 0
        w.put(0, 1);
        let data = w.finish();
        let mut br = BitReader::new(&data);
        let p = parse_picture_header(&mut br).unwrap();
        assert!(p.plusptype);
        assert_eq!(p.coding_type, PictureCodingType::Intra);
        assert_eq!(p.width, 176);
        assert_eq!(p.height, 144);
        assert_eq!(p.pquant, 5);
        assert!(p.deblocking_filter);
    }

    /// PLUSPTYPE + UMV is now accepted — Table D.3 VLC is implemented in
    /// `motion::decode_mvd_table_d3`. The header parse must complete and
    /// consume the UUI prefix (default `1` → limited range per Tables
    /// D.1/D.2). I-picture body is ignored by the test; we only validate
    /// the header fields.
    #[test]
    fn plusptype_umv_accepted_with_table_d3() {
        let mut w = BitBuf::new();
        w.put(0b00_0000_0000_0000_0000_1_00000, 22);
        w.put(0, 8);
        w.put(1, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0b111, 3);
        w.put(0b001, 3);
        // OPPTYPE: src_fmt=010 (QCIF), UMV=1.
        let opptype = pack_opptype(
            0b010, false, true, false, false, false, false, false, false, false, false, false,
        );
        w.put(opptype, 18);
        // MPPTYPE: PCT=000 (I) | RPR=0 | RRU=0 | RTYPE=0 | marker=001
        w.put(0b000_0_0_0_001, 9);
        // CPM = 0
        w.put(0, 1);
        // UUI = "1" → limited range.
        w.put(1, 1);
        // PQUANT = 5
        w.put(5, 5);
        // PEI = 0
        w.put(0, 1);
        let data = w.finish();
        let mut br = BitReader::new(&data);
        let p = parse_picture_header(&mut br).expect("PLUSPTYPE+UMV must parse");
        assert!(p.plusptype);
        assert!(p.umv_mode);
        assert!(!p.uui_unlimited);
        assert_eq!(p.source_format, SourceFormat::Qcif);
    }

    /// PLUSPTYPE + UMV with UUI = "01" → unlimited range flag latches.
    #[test]
    fn plusptype_umv_uui_unlimited_latches() {
        let mut w = BitBuf::new();
        w.put(0b00_0000_0000_0000_0000_1_00000, 22);
        w.put(0, 8);
        w.put(1, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0b111, 3);
        w.put(0b001, 3);
        let opptype = pack_opptype(
            0b010, false, true, false, false, false, false, false, false, false, false, false,
        );
        w.put(opptype, 18);
        w.put(0b000_0_0_0_001, 9); // MPPTYPE I
        w.put(0, 1); // CPM
                     // UUI = "01"
        w.put(0, 1);
        w.put(1, 1);
        w.put(5, 5); // PQUANT
        w.put(0, 1); // PEI
        let data = w.finish();
        let mut br = BitReader::new(&data);
        let p = parse_picture_header(&mut br).expect("PLUSPTYPE+UMV must parse");
        assert!(p.plusptype);
        assert!(p.umv_mode);
        assert!(p.uui_unlimited);
    }

    /// Annex K (slice structured) in PLUSPTYPE must still surface a specific
    /// diagnostic naming the annex — the slice layer syntax replaces GOB
    /// headers with SSC start codes, and we don't yet track that.
    #[test]
    fn plusptype_slice_structured_rejected() {
        let mut w = BitBuf::new();
        w.put(0b00_0000_0000_0000_0000_1_00000, 22);
        w.put(0, 8);
        w.put(1, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0b111, 3);
        w.put(0b001, 3);
        // OPPTYPE: src_fmt=010 (QCIF), SSS=1 (bit 10 of OPPTYPE).
        let opptype = pack_opptype(
            0b010, false, false, false, false, false, false, true, false, false, false, false,
        );
        w.put(opptype, 18);
        let data = w.finish();
        let mut br = BitReader::new(&data);
        let err = parse_picture_header(&mut br).unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("Annex K"), "got {msg}");
    }

    /// Baseline PTYPE (no PLUSPTYPE) with UMV enabled should now parse — the
    /// MV decoder handles §D.2 directly on top of Table 14.
    #[test]
    fn baseline_ptype_umv_accepted() {
        let mut w = BitBuf::new();
        w.put(0b00_0000_0000_0000_0000_1_00000, 22);
        w.put(0, 8);
        // PTYPE bits (13): 1, 0, split=0, cam=0, freeze=0, fmt=010 QCIF, P-pic,
        //                   UMV=1, SAC=0, AP=0, PB=0
        w.put(1, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.put(0b010, 3); // QCIF
        w.put(1, 1); // P-picture
        w.put(1, 1); // UMV ON
        w.put(0, 1); // SAC
        w.put(0, 1); // AP
        w.put(0, 1); // PB
        w.put(5, 5); // PQUANT=5
        w.put(0, 1); // CPM
        w.put(0, 1); // PEI=0
        let data = w.finish();
        let mut br = BitReader::new(&data);
        let p = parse_picture_header(&mut br).unwrap();
        assert!(p.umv_mode, "UMV flag should be latched");
        assert_eq!(p.source_format, SourceFormat::Qcif);
        assert_eq!(p.coding_type, PictureCodingType::Predicted);
        assert!(!p.plusptype);
    }
}
