//! Annex O — Temporal, SNR, and Spatial Scalability macroblock layer.
//!
//! This module decodes the macroblock-layer syntax of the three
//! enhancement-layer picture types defined by ITU-T H.263 Annex O
//! (§O.4): the **B-picture** (temporal scalability), the **EI-picture**
//! (SNR / spatial scalability, "upward" predicted only), and the
//! **EP-picture** ("Enhancement" P-picture, forward + upward
//! predicted).
//!
//! The three picture types share the §5.4 block-layer machinery
//! ([`crate::block::parse_block`]); what differs is the per-macroblock
//! *type* field that precedes the block data:
//!
//! * **B- and EP-pictures** carry an `MBTYPE` VLC (§O.4.2, Tables O.1 /
//!   O.2) followed by an optional `CBPC` VLC (§O.4.3, Table O.4). The
//!   syntax is `COD MBTYPE CBPC CBPY DQUANT MVDFW MVDBW Block` per
//!   Figure O.6.
//! * **EI-pictures** combine type and chroma-pattern into a single
//!   `MCBPC` VLC (§O.4.2, Table O.3): `COD MCBPC CBPY DQUANT Block` per
//!   Figure O.7. EI-pictures use only upward prediction and never carry
//!   a motion vector.
//!
//! ## Scope of this module
//!
//! This file implements the *structural* macroblock-header decode: the
//! `COD` bit (§O.4.1), the MBTYPE / MCBPC VLCs, and the CBPC VLC. It
//! resolves each macroblock to a [`ScalabilityMbHeader`] describing the
//! prediction type, the presence of forward / backward motion-vector
//! data, the INTRA flag, the DQUANT flag, and the chroma / luma coded
//! block patterns. The picture driver in [`crate::picture`] consumes
//! that header to wire block decode + §O reconstruction.
//!
//! ## CBPY convention (§O.4.4)
//!
//! Per §O.4.4, the CBPY column to use depends on the macroblock's
//! prediction type, mirroring §5.3.5 / Table 12:
//!
//! * Upward-predicted MBs (EI and EP), bidirectional MBs in
//!   EP-pictures, and *all* INTRA MBs (EI / EP / B) use the **INTRA**
//!   CBPY definition (the natural-binary pattern).
//! * Every other type (forward / backward / bidirectional in
//!   B-pictures, forward in EP-pictures) uses the **INTER** CBPY
//!   definition (the bitwise complement of the natural-binary value).
//!
//! The shared [`crate::macroblock`] CBPY decoder returns the
//! natural-binary `CBPY(INTRA)` pattern; this module records which
//! convention applies so the driver complements when required.

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Which of the three Annex O enhancement-layer picture types a
/// macroblock belongs to. Selects the MBTYPE / MCBPC VLC table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScalabilityPictureType {
    /// B-picture — temporal scalability (Table O.1). Forward, backward,
    /// bidirectional, and direct prediction; the only type with a
    /// backward motion vector and with the direct mode.
    BPicture,
    /// EP-picture — "Enhancement" P-picture (Table O.2). Forward and
    /// upward prediction (no backward vector; upward uses no vector).
    EpPicture,
    /// EI-picture — upward-predicted enhancement picture (Table O.3).
    /// MBTYPE and CBPC are fused into a single MCBPC VLC. No motion
    /// vectors.
    EiPicture,
}

/// The §O.4 prediction type of a single macroblock, after the COD bit
/// and the MBTYPE / MCBPC VLC have been resolved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScalabilityPredType {
    /// B-picture direct (bidirectional) prediction (§O.5.2). When
    /// signalled by `COD = 1` it is "Direct (skipped)" — no MBTYPE and
    /// no data are sent.
    Direct,
    /// Forward prediction from a previous reference-layer picture
    /// (B-pictures) or a previous EI/EP picture in the same layer
    /// (EP-pictures). Carries a forward motion vector.
    Forward,
    /// Backward prediction from a temporally subsequent reference-layer
    /// picture (B-pictures only). Carries a backward motion vector.
    Backward,
    /// Upward prediction from the temporally simultaneous (possibly
    /// interpolated) reference-layer picture (EI / EP). No motion
    /// vector.
    Upward,
    /// Bidirectional prediction. For B-pictures this averages a forward
    /// and a backward reference; for EP-pictures it averages a forward
    /// (same-layer) and an upward (reference-layer) reference.
    Bidirectional,
    /// INTRA-coded macroblock (no prediction).
    Intra,
}

/// Fully-resolved Annex O macroblock header (post-COD, post-MBTYPE /
/// MCBPC, post-CBPC).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScalabilityMbHeader {
    /// `false` when `COD = 1` — the macroblock is "skipped" and its
    /// prediction type is the table's skipped-row default
    /// (Direct for B, Forward+zeroMV for EP, Upward+zeroMV for EI).
    /// When skipped, no MBTYPE / CBPC / CBPY / DQUANT / MVD / block
    /// data follows.
    pub coded: bool,
    /// §O.4.2 resolved prediction type.
    pub pred_type: ScalabilityPredType,
    /// Whether forward motion-vector data (MVDFW) is present.
    pub has_mvdfw: bool,
    /// Whether backward motion-vector data (MVDBW) is present (B-pictures
    /// bidirectional / backward only).
    pub has_mvdbw: bool,
    /// Whether the table row carries the CBPC + CBPY texture-pattern
    /// fields (and therefore block data may be present). INTRA rows and
    /// most non-"no texture" rows set this; "(no texture)" rows clear
    /// it.
    pub has_cbp: bool,
    /// Whether a DQUANT field follows (the "+ Q" rows).
    pub has_dquant: bool,
    /// The 2-bit chrominance coded-block pattern (CBPC), decoded from
    /// Table O.4 for B / EP, or carried in the MCBPC code for EI. Bit 1
    /// (`0b10`) is the Cb (block 5) flag, bit 0 (`0b01`) the Cr (block
    /// 6) flag, matching §5.3.2. Zero when `has_cbp` is false.
    pub cbpc: u8,
    /// Whether this macroblock uses the INTRA CBPY column (§O.4.4).
    /// `true` for upward-predicted, EP-bidirectional, and all INTRA
    /// macroblocks; `false` otherwise. The driver complements the
    /// natural-binary CBPY pattern when this is `false`.
    pub cbpy_uses_intra_column: bool,
}

impl ScalabilityMbHeader {
    /// `true` if the macroblock is INTRA-coded.
    pub fn is_intra(self) -> bool {
        matches!(self.pred_type, ScalabilityPredType::Intra)
    }
}

/// §O.4.2 / Table O.4 — CBPC VLC for B- and EP-pictures.
///
/// | pattern (56) | code  |
/// |--------------|-------|
/// | 00           | `0`   |
/// | 01           | `10`  |
/// | 10           | `111` |
/// | 11           | `110` |
///
/// Returns the 2-bit pattern with bit 1 = Cb (block 5), bit 0 = Cr
/// (block 6).
pub fn decode_cbpc(reader: &mut BitReader<'_>) -> Result<u8> {
    let b0 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if !b0 {
        // `0` -> 00.
        return Ok(0b00);
    }
    let b1 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if !b1 {
        // `10` -> 01.
        return Ok(0b01);
    }
    let b2 = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    // `110` -> 11, `111` -> 10.
    Ok(if b2 { 0b10 } else { 0b11 })
}

/// One row of an Annex O MBTYPE / MCBPC table: the prediction type plus
/// the presence flags the row implies.
#[derive(Debug, Clone, Copy)]
struct MbTypeRow {
    pred_type: ScalabilityPredType,
    has_mvdfw: bool,
    has_mvdbw: bool,
    has_cbp: bool,
    has_dquant: bool,
}

/// §O.4.2 Table O.1 — MBTYPE for B-pictures.
///
/// The VLC is decoded by walking the prefix tree. Codewords (from the
/// `MBTYPE` column of Table O.1):
///
/// | code        | type              | MVDFW | MVDBW | CBP | DQUANT |
/// |-------------|-------------------|-------|-------|-----|--------|
/// | `11`        | Direct            |       |       | X   |        |
/// | `0001`      | Direct + Q        |       |       | X   | X      |
/// | `100`       | Forward (no tex)  | X     |       |     |        |
/// | `101`       | Forward           | X     |       | X   |        |
/// | `00110`     | Forward + Q       | X     |       | X   | X      |
/// | `010`       | Backward (no tex) |       | X     |     |        |
/// | `011`       | Backward          |       | X     | X   |        |
/// | `00111`     | Backward + Q      |       | X     | X   | X      |
/// | `00100`     | Bi-dir (no tex)   | X     | X     |     |        |
/// | `00101`     | Bi-dir            | X     | X     | X   |        |
/// | `00001`     | Bi-dir + Q        | X     | X     | X   | X      |
/// | `000001`    | INTRA             |       |       | X   |        |
/// | `0000001`   | INTRA + Q         |       |       | X   | X      |
/// | `000000001` | Stuffing          |       |       |     |        |
fn decode_mbtype_b(reader: &mut BitReader<'_>) -> Result<MbTypeRow> {
    use ScalabilityPredType::*;
    let b = |r: &mut BitReader<'_>| r.read_bit().map_err(|_| Error::UnexpectedEof);

    // First bit.
    if b(reader)? {
        // 1...
        if b(reader)? {
            // 11 -> Direct.
            return Ok(MbTypeRow {
                pred_type: Direct,
                has_mvdfw: false,
                has_mvdbw: false,
                has_cbp: true,
                has_dquant: false,
            });
        }
        // 10...
        if b(reader)? {
            // 101 -> Forward.
            return Ok(MbTypeRow {
                pred_type: Forward,
                has_mvdfw: true,
                has_mvdbw: false,
                has_cbp: true,
                has_dquant: false,
            });
        }
        // 100 -> Forward (no texture).
        return Ok(MbTypeRow {
            pred_type: Forward,
            has_mvdfw: true,
            has_mvdbw: false,
            has_cbp: false,
            has_dquant: false,
        });
    }
    // 0...
    if b(reader)? {
        // 01...
        if b(reader)? {
            // 011 -> Backward.
            return Ok(MbTypeRow {
                pred_type: Backward,
                has_mvdfw: false,
                has_mvdbw: true,
                has_cbp: true,
                has_dquant: false,
            });
        }
        // 010 -> Backward (no texture).
        return Ok(MbTypeRow {
            pred_type: Backward,
            has_mvdfw: false,
            has_mvdbw: true,
            has_cbp: false,
            has_dquant: false,
        });
    }
    // 00...
    if b(reader)? {
        // 001...
        if b(reader)? {
            // 0011...
            if b(reader)? {
                // 00111 -> Backward + Q.
                Ok(MbTypeRow {
                    pred_type: Backward,
                    has_mvdfw: false,
                    has_mvdbw: true,
                    has_cbp: true,
                    has_dquant: true,
                })
            } else {
                // 00110 -> Forward + Q.
                Ok(MbTypeRow {
                    pred_type: Forward,
                    has_mvdfw: true,
                    has_mvdbw: false,
                    has_cbp: true,
                    has_dquant: true,
                })
            }
        } else {
            // 0010...
            if b(reader)? {
                // 00101 -> Bi-dir.
                Ok(MbTypeRow {
                    pred_type: Bidirectional,
                    has_mvdfw: true,
                    has_mvdbw: true,
                    has_cbp: true,
                    has_dquant: false,
                })
            } else {
                // 00100 -> Bi-dir (no texture).
                Ok(MbTypeRow {
                    pred_type: Bidirectional,
                    has_mvdfw: true,
                    has_mvdbw: true,
                    has_cbp: false,
                    has_dquant: false,
                })
            }
        }
    } else {
        // 000...
        if b(reader)? {
            // 0001 -> Direct + Q.
            return Ok(MbTypeRow {
                pred_type: Direct,
                has_mvdfw: false,
                has_mvdbw: false,
                has_cbp: true,
                has_dquant: true,
            });
        }
        // 0000...
        if b(reader)? {
            // 00001 -> Bi-dir + Q.
            return Ok(MbTypeRow {
                pred_type: Bidirectional,
                has_mvdfw: true,
                has_mvdbw: true,
                has_cbp: true,
                has_dquant: true,
            });
        }
        // 00000...
        if b(reader)? {
            // 000001 -> INTRA.
            return Ok(MbTypeRow {
                pred_type: Intra,
                has_mvdfw: false,
                has_mvdbw: false,
                has_cbp: true,
                has_dquant: false,
            });
        }
        // 000000...
        if b(reader)? {
            // 0000001 -> INTRA + Q.
            return Ok(MbTypeRow {
                pred_type: Intra,
                has_mvdfw: false,
                has_mvdbw: false,
                has_cbp: true,
                has_dquant: true,
            });
        }
        // 0000000... (7 leading zeros). The only remaining defined
        // codeword is "000000001" (Stuffing). Whatever the trailing
        // bits, no real macroblock type is encoded here, so this is a
        // control code: reject it as a non-macroblock.
        Err(Error::BadScalabilityMbType)
    }
}

/// §O.4.2 Table O.2 — MBTYPE for EP-pictures.
///
/// | code      | type             | MVDFW | CBP | DQUANT |
/// |-----------|------------------|-------|-----|--------|
/// | `1`       | Forward          | X     | X   |        |
/// | `001`     | Forward + Q      | X     | X   | X      |
/// | `010`     | Upward (no tex)  |       |     |        |
/// | `011`     | Upward           |       | X   |        |
/// | `00001`   | Upward + Q       |       | X   | X      |
/// | `00010`   | Bi-dir (no tex)  |       |     |        |
/// | `00011`   | Bi-dir           | X     | X   |        |
/// | `000001`  | Bi-dir + Q       | X     | X   | X      |
/// | `0000001` | INTRA            |       | X   |        |
/// | `00000001`| INTRA + Q        |       | X   | X      |
/// | `000000001`| Stuffing        |       |     |        |
///
/// EP-pictures never carry a backward vector; the "upward" reference is
/// the temporally-simultaneous reference-layer picture (no vector).
/// Bidirectional rows therefore set `has_mvdfw` per the table but never
/// `has_mvdbw`.
fn decode_mbtype_ep(reader: &mut BitReader<'_>) -> Result<MbTypeRow> {
    use ScalabilityPredType::*;
    let b = |r: &mut BitReader<'_>| r.read_bit().map_err(|_| Error::UnexpectedEof);

    if b(reader)? {
        // 1 -> Forward.
        return Ok(MbTypeRow {
            pred_type: Forward,
            has_mvdfw: true,
            has_mvdbw: false,
            has_cbp: true,
            has_dquant: false,
        });
    }
    // 0...
    if b(reader)? {
        // 01...
        if b(reader)? {
            // 011 -> Upward.
            Ok(MbTypeRow {
                pred_type: Upward,
                has_mvdfw: false,
                has_mvdbw: false,
                has_cbp: true,
                has_dquant: false,
            })
        } else {
            // 010 -> Upward (no texture).
            Ok(MbTypeRow {
                pred_type: Upward,
                has_mvdfw: false,
                has_mvdbw: false,
                has_cbp: false,
                has_dquant: false,
            })
        }
    } else {
        // 00...
        // Count run of zeros after the leading "00".
        if b(reader)? {
            // 001 -> Forward + Q.
            return Ok(MbTypeRow {
                pred_type: Forward,
                has_mvdfw: true,
                has_mvdbw: false,
                has_cbp: true,
                has_dquant: true,
            });
        }
        // 000...
        if b(reader)? {
            // 0001...
            if b(reader)? {
                // 00011 -> Bi-dir.
                Ok(MbTypeRow {
                    pred_type: Bidirectional,
                    has_mvdfw: true,
                    has_mvdbw: false,
                    has_cbp: true,
                    has_dquant: false,
                })
            } else {
                // 00010 -> Bi-dir (no texture).
                Ok(MbTypeRow {
                    pred_type: Bidirectional,
                    has_mvdfw: false,
                    has_mvdbw: false,
                    has_cbp: false,
                    has_dquant: false,
                })
            }
        } else {
            // 0000...
            if b(reader)? {
                // 00001 -> Upward + Q.
                return Ok(MbTypeRow {
                    pred_type: Upward,
                    has_mvdfw: false,
                    has_mvdbw: false,
                    has_cbp: true,
                    has_dquant: true,
                });
            }
            // 00000...
            if b(reader)? {
                // 000001 -> Bi-dir + Q.
                return Ok(MbTypeRow {
                    pred_type: Bidirectional,
                    has_mvdfw: true,
                    has_mvdbw: false,
                    has_cbp: true,
                    has_dquant: true,
                });
            }
            // 000000...
            if b(reader)? {
                // 0000001 -> INTRA.
                return Ok(MbTypeRow {
                    pred_type: Intra,
                    has_mvdfw: false,
                    has_mvdbw: false,
                    has_cbp: true,
                    has_dquant: false,
                });
            }
            // 0000000...
            if b(reader)? {
                // 00000001 -> INTRA + Q.
                return Ok(MbTypeRow {
                    pred_type: Intra,
                    has_mvdfw: false,
                    has_mvdbw: false,
                    has_cbp: true,
                    has_dquant: true,
                });
            }
            // 00000000...
            if b(reader)? {
                // 000000001 -> Stuffing.
                return Err(Error::BadScalabilityMbType);
            }
            Err(Error::BadScalabilityMbType)
        }
    }
}

/// The MCBPC decode result for an EI-picture: prediction type (Upward
/// or Intra), the DQUANT flag, and the 2-bit CBPC. Per Table O.3 the
/// EI MCBPC fuses the type, the chroma code-block pattern, and the "+ Q"
/// flag.
struct EiMcbpc {
    pred_type: ScalabilityPredType,
    has_dquant: bool,
    cbpc: u8,
}

/// §O.4.2 Table O.3 — MCBPC for EI-pictures.
///
/// | code        | type     | CBPC | DQUANT |
/// |-------------|----------|------|--------|
/// | `1`         | Upward   | 00   |        |
/// | `001`       | Upward   | 01   |        |
/// | `010`       | Upward   | 10   |        |
/// | `011`       | Upward   | 11   |        |
/// | `0001`      | Upward+Q | 00   | X      |
/// | `0000001`   | Upward+Q | 01   | X      |
/// | `0000010`   | Upward+Q | 10   | X      |
/// | `0000011`   | Upward+Q | 11   | X      |
/// | `00000001`  | INTRA    | 00   |        |
/// | `00001001`  | INTRA    | 01   |        |
/// | `00001010`  | INTRA    | 10   |        |
/// | `00001011`  | INTRA    | 11   |        |
/// | `00001100`  | INTRA+Q  | 00   | X      |
/// | `00001101`  | INTRA+Q  | 01   | X      |
/// | `00001110`  | INTRA+Q  | 10   | X      |
/// | `00001111`  | INTRA+Q  | 11   | X      |
/// | `000000001` | Stuffing |      |        |
///
/// The codewords are not a simple prefix grouping, so this decoder
/// reads up to nine bits and matches the bit string directly. The CBPC
/// here is the Table O.3 "Code block pattern (56)" — bit 1 = Cb (block
/// 5), bit 0 = Cr (block 6).
fn decode_mcbpc_ei(reader: &mut BitReader<'_>) -> Result<EiMcbpc> {
    use ScalabilityPredType::*;
    let b = |r: &mut BitReader<'_>| r.read_bit().map_err(|_| Error::UnexpectedEof);

    if b(reader)? {
        // 1 -> Upward, CBPC 00.
        return Ok(EiMcbpc {
            pred_type: Upward,
            has_dquant: false,
            cbpc: 0b00,
        });
    }
    // 0...
    if b(reader)? {
        // 01...
        if b(reader)? {
            // 011 -> Upward, CBPC 11.
            Ok(EiMcbpc {
                pred_type: Upward,
                has_dquant: false,
                cbpc: 0b11,
            })
        } else {
            // 010 -> Upward, CBPC 10.
            Ok(EiMcbpc {
                pred_type: Upward,
                has_dquant: false,
                cbpc: 0b10,
            })
        }
    } else {
        // 00...
        if b(reader)? {
            // 001 -> Upward, CBPC 01.
            return Ok(EiMcbpc {
                pred_type: Upward,
                has_dquant: false,
                cbpc: 0b01,
            });
        }
        // 000...
        if b(reader)? {
            // 0001 -> Upward + Q, CBPC 00.
            return Ok(EiMcbpc {
                pred_type: Upward,
                has_dquant: true,
                cbpc: 0b00,
            });
        }
        // 0000...
        if b(reader)? {
            // 00001... — eight-bit INTRA group. The trailing three bits
            // select the row (the `000` suffix is undefined).
            let lo = reader.read_u32(3).map_err(|_| Error::UnexpectedEof)?;
            return match lo {
                // 00001_001 -> INTRA, CBPC 01.
                0b001 => Ok(EiMcbpc {
                    pred_type: Intra,
                    has_dquant: false,
                    cbpc: 0b01,
                }),
                // 00001_010 -> INTRA, CBPC 10.
                0b010 => Ok(EiMcbpc {
                    pred_type: Intra,
                    has_dquant: false,
                    cbpc: 0b10,
                }),
                // 00001_011 -> INTRA, CBPC 11.
                0b011 => Ok(EiMcbpc {
                    pred_type: Intra,
                    has_dquant: false,
                    cbpc: 0b11,
                }),
                // 00001_100 -> INTRA + Q, CBPC 00.
                0b100 => Ok(EiMcbpc {
                    pred_type: Intra,
                    has_dquant: true,
                    cbpc: 0b00,
                }),
                // 00001_101 -> INTRA + Q, CBPC 01.
                0b101 => Ok(EiMcbpc {
                    pred_type: Intra,
                    has_dquant: true,
                    cbpc: 0b01,
                }),
                // 00001_110 -> INTRA + Q, CBPC 10.
                0b110 => Ok(EiMcbpc {
                    pred_type: Intra,
                    has_dquant: true,
                    cbpc: 0b10,
                }),
                // 00001_111 -> INTRA + Q, CBPC 11.
                0b111 => Ok(EiMcbpc {
                    pred_type: Intra,
                    has_dquant: true,
                    cbpc: 0b11,
                }),
                _ => Err(Error::BadScalabilityMbType),
            };
        }
        // 00000... (bits 0-4 are all zero). The remaining codewords:
        //   0000010 -> Upward+Q 10   (bit5=1, bit6=0)
        //   0000011 -> Upward+Q 11   (bit5=1, bit6=1)
        //   0000001 -> Upward+Q 01   (bit5=0, bit6=1)
        //   00000001 -> INTRA 00     (bit5=0, bit6=0, bit7=1)
        //   000000001 -> Stuffing    (bit5=0, bit6=0, bit7=0, bit8=1)
        if b(reader)? {
            // 000001...
            let b6 = b(reader)?;
            // 0000011 -> Upward+Q 11, 0000010 -> Upward+Q 10.
            return Ok(EiMcbpc {
                pred_type: Upward,
                has_dquant: true,
                cbpc: if b6 { 0b11 } else { 0b10 },
            });
        }
        // 000000...
        if b(reader)? {
            // 0000001 -> Upward+Q, CBPC 01.
            return Ok(EiMcbpc {
                pred_type: Upward,
                has_dquant: true,
                cbpc: 0b01,
            });
        }
        // 0000000...
        if b(reader)? {
            // 00000001 -> INTRA, CBPC 00.
            return Ok(EiMcbpc {
                pred_type: Intra,
                has_dquant: false,
                cbpc: 0b00,
            });
        }
        // 00000000... -> only "000000001" Stuffing remains; any code
        // that does not present the terminating 1 bit is illegal.
        if b(reader)? {
            // 000000001 -> Stuffing (treated as a control code, not a
            // real macroblock).
            return Err(Error::BadScalabilityMbType);
        }
        Err(Error::BadScalabilityMbType)
    }
}

/// Decode the Annex O macroblock header for a B- or EP-picture:
/// `COD MBTYPE [CBPC]` (the CBPY / DQUANT / MVD / block fields are read
/// by the picture driver using the returned presence flags).
///
/// `reader` is positioned at the COD bit. On success it is left after
/// the CBPC field (if present), i.e. before CBPY.
pub fn decode_mb_header_b_ep(
    reader: &mut BitReader<'_>,
    pic: ScalabilityPictureType,
) -> Result<ScalabilityMbHeader> {
    debug_assert!(matches!(
        pic,
        ScalabilityPictureType::BPicture | ScalabilityPictureType::EpPicture
    ));

    // §O.4.1 — COD bit.
    let cod = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if cod {
        // Skipped macroblock. Per §O.4.2:
        //  * B-picture: "Direct (skipped)" — direct prediction, no data.
        //  * EP-picture: "Forward (skipped)" — forward prediction with
        //    zero motion vector, no data.
        let pred_type = match pic {
            ScalabilityPictureType::BPicture => ScalabilityPredType::Direct,
            ScalabilityPictureType::EpPicture => ScalabilityPredType::Forward,
            ScalabilityPictureType::EiPicture => unreachable!(),
        };
        return Ok(ScalabilityMbHeader {
            coded: false,
            pred_type,
            has_mvdfw: false,
            has_mvdbw: false,
            has_cbp: false,
            has_dquant: false,
            cbpc: 0,
            // CBPY column is irrelevant for a skipped MB (no texture).
            cbpy_uses_intra_column: false,
        });
    }

    let row = match pic {
        ScalabilityPictureType::BPicture => decode_mbtype_b(reader)?,
        ScalabilityPictureType::EpPicture => decode_mbtype_ep(reader)?,
        ScalabilityPictureType::EiPicture => unreachable!(),
    };

    // §O.4.3 — CBPC follows MBTYPE only when the row carries texture.
    let cbpc = if row.has_cbp { decode_cbpc(reader)? } else { 0 };

    // §O.4.4 — CBPY column selection.
    let cbpy_uses_intra_column = cbpy_uses_intra_column(pic, row.pred_type);

    Ok(ScalabilityMbHeader {
        coded: true,
        pred_type: row.pred_type,
        has_mvdfw: row.has_mvdfw,
        has_mvdbw: row.has_mvdbw,
        has_cbp: row.has_cbp,
        has_dquant: row.has_dquant,
        cbpc,
        cbpy_uses_intra_column,
    })
}

/// Decode the Annex O macroblock header for an EI-picture:
/// `COD MCBPC` (CBPY / DQUANT / block fields follow, read by the
/// driver).
///
/// `reader` is positioned at the COD bit; left after MCBPC on success.
pub fn decode_mb_header_ei(reader: &mut BitReader<'_>) -> Result<ScalabilityMbHeader> {
    // §O.4.1 — COD bit.
    let cod = reader.read_bit().map_err(|_| Error::UnexpectedEof)?;
    if cod {
        // "Upward (skipped)" — upward prediction with zero motion
        // vector and no coefficients.
        return Ok(ScalabilityMbHeader {
            coded: false,
            pred_type: ScalabilityPredType::Upward,
            has_mvdfw: false,
            has_mvdbw: false,
            has_cbp: false,
            has_dquant: false,
            cbpc: 0,
            cbpy_uses_intra_column: true,
        });
    }

    let m = decode_mcbpc_ei(reader)?;
    let cbpy_uses_intra_column =
        cbpy_uses_intra_column(ScalabilityPictureType::EiPicture, m.pred_type);

    Ok(ScalabilityMbHeader {
        coded: true,
        pred_type: m.pred_type,
        has_mvdfw: false,
        has_mvdbw: false,
        has_cbp: true,
        has_dquant: m.has_dquant,
        cbpc: m.cbpc,
        cbpy_uses_intra_column,
    })
}

/// §O.4.4 — decide which CBPY column (INTRA natural-binary vs INTER
/// complement) applies to a macroblock of the given picture and
/// prediction type.
fn cbpy_uses_intra_column(pic: ScalabilityPictureType, pred_type: ScalabilityPredType) -> bool {
    use ScalabilityPredType::*;
    match pred_type {
        // All INTRA macroblocks use the INTRA CBPY column.
        Intra => true,
        // Upward-predicted MBs (EI and EP) use the INTRA column.
        Upward => true,
        // Bidirectional MBs in EP-pictures use the INTRA column;
        // bidirectional MBs in B-pictures use the INTER column.
        Bidirectional => matches!(pic, ScalabilityPictureType::EpPicture),
        // Forward / backward / direct use the INTER column.
        Forward | Backward | Direct => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Build a reader over a bit pattern written MSB-first.
    fn reader_from_bits(bits: &[u8]) -> Vec<u8> {
        let mut w = BitWriter::new();
        for &bit in bits {
            w.write_bit(bit != 0);
        }
        w.finish()
    }

    fn bits(s: &str) -> Vec<u8> {
        let v: Vec<u8> = s
            .bytes()
            .filter(|b| *b == b'0' || *b == b'1')
            .map(|b| b - b'0')
            .collect();
        reader_from_bits(&v)
    }

    #[test]
    fn cbpc_table_o4() {
        for (code, expect) in [("0", 0b00), ("10", 0b01), ("111", 0b10), ("110", 0b11)] {
            let buf = bits(code);
            let mut r = BitReader::new(&buf);
            assert_eq!(decode_cbpc(&mut r).unwrap(), expect, "code {code}");
        }
    }

    #[test]
    fn b_mbtype_direct() {
        let buf = bits("11");
        let mut r = BitReader::new(&buf);
        let row = decode_mbtype_b(&mut r).unwrap();
        assert_eq!(row.pred_type, ScalabilityPredType::Direct);
        assert!(row.has_cbp);
        assert!(!row.has_dquant);
        assert!(!row.has_mvdfw && !row.has_mvdbw);
    }

    #[test]
    fn b_mbtype_all_rows() {
        let cases = [
            ("11", ScalabilityPredType::Direct, false, false, true, false),
            (
                "0001",
                ScalabilityPredType::Direct,
                false,
                false,
                true,
                true,
            ),
            (
                "100",
                ScalabilityPredType::Forward,
                true,
                false,
                false,
                false,
            ),
            (
                "101",
                ScalabilityPredType::Forward,
                true,
                false,
                true,
                false,
            ),
            (
                "00110",
                ScalabilityPredType::Forward,
                true,
                false,
                true,
                true,
            ),
            (
                "010",
                ScalabilityPredType::Backward,
                false,
                true,
                false,
                false,
            ),
            (
                "011",
                ScalabilityPredType::Backward,
                false,
                true,
                true,
                false,
            ),
            (
                "00111",
                ScalabilityPredType::Backward,
                false,
                true,
                true,
                true,
            ),
            (
                "00100",
                ScalabilityPredType::Bidirectional,
                true,
                true,
                false,
                false,
            ),
            (
                "00101",
                ScalabilityPredType::Bidirectional,
                true,
                true,
                true,
                false,
            ),
            (
                "00001",
                ScalabilityPredType::Bidirectional,
                true,
                true,
                true,
                true,
            ),
            (
                "000001",
                ScalabilityPredType::Intra,
                false,
                false,
                true,
                false,
            ),
            (
                "0000001",
                ScalabilityPredType::Intra,
                false,
                false,
                true,
                true,
            ),
        ];
        for (code, pt, fw, bw, cbp, dq) in cases {
            let buf = bits(code);
            let mut r = BitReader::new(&buf);
            let row = decode_mbtype_b(&mut r).unwrap();
            assert_eq!(row.pred_type, pt, "code {code} type");
            assert_eq!(row.has_mvdfw, fw, "code {code} mvdfw");
            assert_eq!(row.has_mvdbw, bw, "code {code} mvdbw");
            assert_eq!(row.has_cbp, cbp, "code {code} cbp");
            assert_eq!(row.has_dquant, dq, "code {code} dquant");
        }
    }

    #[test]
    fn ep_mbtype_all_rows() {
        let cases = [
            ("1", ScalabilityPredType::Forward, true, true, false),
            ("001", ScalabilityPredType::Forward, true, true, true),
            ("010", ScalabilityPredType::Upward, false, false, false),
            ("011", ScalabilityPredType::Upward, false, true, false),
            ("00001", ScalabilityPredType::Upward, false, true, true),
            (
                "00010",
                ScalabilityPredType::Bidirectional,
                false,
                false,
                false,
            ),
            (
                "00011",
                ScalabilityPredType::Bidirectional,
                true,
                true,
                false,
            ),
            (
                "000001",
                ScalabilityPredType::Bidirectional,
                true,
                true,
                true,
            ),
            ("0000001", ScalabilityPredType::Intra, false, true, false),
            ("00000001", ScalabilityPredType::Intra, false, true, true),
        ];
        for (code, pt, fw, cbp, dq) in cases {
            let buf = bits(code);
            let mut r = BitReader::new(&buf);
            let row = decode_mbtype_ep(&mut r).unwrap();
            assert_eq!(row.pred_type, pt, "code {code} type");
            assert_eq!(row.has_mvdfw, fw, "code {code} mvdfw");
            assert!(!row.has_mvdbw, "code {code} ep never backward");
            assert_eq!(row.has_cbp, cbp, "code {code} cbp");
            assert_eq!(row.has_dquant, dq, "code {code} dquant");
        }
    }

    #[test]
    fn ei_mcbpc_upward_rows() {
        let cases = [
            ("1", ScalabilityPredType::Upward, 0b00, false),
            ("001", ScalabilityPredType::Upward, 0b01, false),
            ("010", ScalabilityPredType::Upward, 0b10, false),
            ("011", ScalabilityPredType::Upward, 0b11, false),
            ("0001", ScalabilityPredType::Upward, 0b00, true),
        ];
        for (code, pt, cbpc, dq) in cases {
            let buf = bits(code);
            let mut r = BitReader::new(&buf);
            let m = decode_mcbpc_ei(&mut r).unwrap();
            assert_eq!(m.pred_type, pt, "code {code}");
            assert_eq!(m.cbpc, cbpc, "code {code} cbpc");
            assert_eq!(m.has_dquant, dq, "code {code} dquant");
        }
    }

    #[test]
    fn ei_mcbpc_intra_rows() {
        let cases = [
            ("00000001", ScalabilityPredType::Intra, 0b00, false),
            ("00001001", ScalabilityPredType::Intra, 0b01, false),
            ("00001010", ScalabilityPredType::Intra, 0b10, false),
            ("00001011", ScalabilityPredType::Intra, 0b11, false),
            ("00001100", ScalabilityPredType::Intra, 0b00, true),
            ("00001101", ScalabilityPredType::Intra, 0b01, true),
            ("00001110", ScalabilityPredType::Intra, 0b10, true),
            ("00001111", ScalabilityPredType::Intra, 0b11, true),
        ];
        for (code, pt, cbpc, dq) in cases {
            let buf = bits(code);
            let mut r = BitReader::new(&buf);
            let m = decode_mcbpc_ei(&mut r).unwrap();
            assert_eq!(m.pred_type, pt, "code {code}");
            assert_eq!(m.cbpc, cbpc, "code {code} cbpc");
            assert_eq!(m.has_dquant, dq, "code {code} dquant");
        }
    }

    #[test]
    fn skipped_headers() {
        // B-picture COD=1 -> Direct skipped.
        let buf = bits("1");
        let mut r = BitReader::new(&buf);
        let h = decode_mb_header_b_ep(&mut r, ScalabilityPictureType::BPicture).unwrap();
        assert!(!h.coded);
        assert_eq!(h.pred_type, ScalabilityPredType::Direct);

        // EP-picture COD=1 -> Forward skipped.
        let buf = bits("1");
        let mut r = BitReader::new(&buf);
        let h = decode_mb_header_b_ep(&mut r, ScalabilityPictureType::EpPicture).unwrap();
        assert!(!h.coded);
        assert_eq!(h.pred_type, ScalabilityPredType::Forward);

        // EI-picture COD=1 -> Upward skipped.
        let buf = bits("1");
        let mut r = BitReader::new(&buf);
        let h = decode_mb_header_ei(&mut r).unwrap();
        assert!(!h.coded);
        assert_eq!(h.pred_type, ScalabilityPredType::Upward);
    }

    #[test]
    fn cbpy_column_selection() {
        use ScalabilityPictureType::*;
        // INTRA -> INTRA column everywhere.
        assert!(cbpy_uses_intra_column(BPicture, ScalabilityPredType::Intra));
        // Upward -> INTRA column.
        assert!(cbpy_uses_intra_column(
            EiPicture,
            ScalabilityPredType::Upward
        ));
        // EP bidirectional -> INTRA column.
        assert!(cbpy_uses_intra_column(
            EpPicture,
            ScalabilityPredType::Bidirectional
        ));
        // B bidirectional -> INTER column.
        assert!(!cbpy_uses_intra_column(
            BPicture,
            ScalabilityPredType::Bidirectional
        ));
        // Forward -> INTER column.
        assert!(!cbpy_uses_intra_column(
            BPicture,
            ScalabilityPredType::Forward
        ));
    }
}
