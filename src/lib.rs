//! # oxideav-h263
//!
//! Pure-Rust ITU-T H.263 baseline video codec for the
//! [oxideav](https://github.com/OxideAV/oxideav) framework.
//!
//! **Status:** orphan-rebuild round-2 (post 2026-05-18 audit).
//!
//! The decoder is being re-built clean-room against
//! [ITU-T Recommendation H.263 (01/2005)][spec]. Coverage so far:
//!
//! * **Round 1** — picture-header parser per §5.1: the Picture Start
//!   Code (PSC), Temporal Reference (TR), and the variable-length
//!   Type Information (PTYPE) field with its source-format and
//!   picture-coding sub-fields.
//! * **Round 2** — GOB-layer header per §5.2: Group of Blocks Start
//!   Code (GBSC), Group Number (GN), GOB Frame ID (GFID), and
//!   Quantizer Information (GQUANT), for the CPM = "0" branch (no
//!   GSBI on the wire).
//!
//! No macroblock, motion-vector, or DCT decode is wired up yet;
//! every operational decode path still returns
//! [`Error::NotImplemented`].
//!
//! [spec]: https://www.itu.int/rec/T-REC-H.263

#![warn(missing_debug_implementations)]

use oxideav_core::bits::BitReader;
use oxideav_core::RuntimeContext;

pub mod gob_header;
pub mod picture_header;

pub use gob_header::{
    parse_gob_layer, parse_gob_layer_from_bytes, GobLayer, GBSC_BITS, GBSC_VALUE, GFID_BITS,
    GN_BITS, GOB_HEADER_BITS_NO_CPM, GQUANT_BITS,
};
pub use picture_header::{
    parse_picture_header, H263PictureCodingType, H263PictureHeader, H263SourceFormat, PSC_BITS,
    PSC_VALUE,
};

/// Crate-local error type. The orphan-rebuild scaffold returns
/// [`Error::NotImplemented`] for any decode path that is not yet wired
/// up; the picture-header parser returns the variants below directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The crate is partially scaffolded; the path the caller invoked
    /// is not yet implemented.
    NotImplemented,
    /// Bitstream ended before the parser could read the requested
    /// number of bits. Returned by [`parse_picture_header`] when the
    /// input buffer is shorter than the picture-layer header demands.
    UnexpectedEof,
    /// The 22-bit Picture Start Code (PSC, value `0x000020`) was not
    /// present at the current bitstream position. See §5.1.1.
    BadPictureStartCode,
    /// PTYPE bit 1 (always "1") or bit 2 (always "0") did not have
    /// their fixed values. See §5.1.3.
    BadPtypeFixedBits,
    /// PTYPE source-format field (bits 6-8) had the forbidden value
    /// `"000"`. See §5.1.3.
    ForbiddenSourceFormat,
    /// The extended-PTYPE path (PTYPE source-format `"111"`) is not
    /// yet decoded — round 1 only covers the non-extended PTYPE
    /// header. See §5.1.4.
    ExtendedPtypeNotSupported,
    /// The 17-bit Group of Blocks Start Code (GBSC, value
    /// `0000 0000 0000 0000 1`) was not present at the current
    /// bitstream position. See §5.2.2.
    BadGroupStartCode,
    /// The Group Number (GN) was outside the legal GOB-layer range.
    /// Values `0`, `30`, `31` are rejected — `0` is reserved to PSC,
    /// `30` is the EOSBS marker, `31` is the EOS marker. See §5.2.3.
    InvalidGroupNumber,
    /// GQUANT was `0`; §5.2.6 limits the natural-binary QUANT field
    /// to `1..=31`.
    InvalidQuantiser,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotImplemented => write!(
                f,
                "oxideav-h263: not yet implemented in this orphan-rebuild round"
            ),
            Error::UnexpectedEof => {
                write!(f, "oxideav-h263: bitstream ended inside picture header")
            }
            Error::BadPictureStartCode => write!(
                f,
                "oxideav-h263: picture start code (PSC) not found at expected position"
            ),
            Error::BadPtypeFixedBits => write!(
                f,
                "oxideav-h263: PTYPE fixed bits (bit1=1, bit2=0) violated"
            ),
            Error::ForbiddenSourceFormat => write!(
                f,
                "oxideav-h263: PTYPE source format had forbidden value 000"
            ),
            Error::ExtendedPtypeNotSupported => write!(
                f,
                "oxideav-h263: extended PTYPE (PLUSPTYPE) path not yet supported"
            ),
            Error::BadGroupStartCode => write!(
                f,
                "oxideav-h263: group-of-blocks start code (GBSC) not found at expected position"
            ),
            Error::InvalidGroupNumber => write!(
                f,
                "oxideav-h263: group number (GN) outside the legal GOB-layer range (1..=29)"
            ),
            Error::InvalidQuantiser => {
                write!(f, "oxideav-h263: GQUANT was 0 (legal range is 1..=31)")
            }
        }
    }
}

impl std::error::Error for Error {}

/// Convenience alias for results returned from the parser surface.
pub type Result<T> = core::result::Result<T, Error>;

/// No-op codec registration — the round-1 scaffold parses headers but
/// has no decoder to register into the runtime context. The full
/// `Decoder` impl will land in a later round once macroblock decode
/// exists.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("h263", register);

/// Free function alias mapping a byte slice to
/// [`parse_picture_header`] — provided so callers do not need to
/// allocate a [`BitReader`] themselves.
pub fn parse_picture_header_from_bytes(data: &[u8]) -> Result<H263PictureHeader> {
    let mut reader = BitReader::new(data);
    parse_picture_header(&mut reader)
}
