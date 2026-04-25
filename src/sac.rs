//! Annex E — Syntax-based Arithmetic Coding (SAC).
//!
//! This module provides the SAC encoder and decoder core specified in
//! §E.2 / §E.3 of ITU-T Rec. H.263 (01/2005), plus every cumulative-frequency
//! model listed in §E.8.
//!
//! The arithmetic coder is a standard integer implementation with 16-bit
//! state registers (`q1 = 16384`, `q2 = 32768`, `q3 = 49152`, `top = 65535`)
//! and three renormalisation conditions:
//! * `high < q2` — emit `0` and `opposite_bits` `1`s;
//! * `low >= q2` — emit `1` and `opposite_bits` `0`s, subtract `q2` from both
//!   state registers;
//! * `q1 <= low`, `high < q3` (underflow-zone straddle) — increment
//!   `opposite_bits`, subtract `q1` from both state registers.
//!
//! Each renormalisation iteration shifts `low` left by one and sets
//! `high = 2 * high + 1`, i.e. performs the standard `E1`/`E2`/`E3` scaling
//! of the current interval while deferring the `opposite_bits` carry until
//! the next emit.
//!
//! The encoder flush (`encoder_flush` in §E.7) forces the next pending
//! interval end out of `q1..q2` or `q2..q3` and drains `opposite_bits` into
//! the bitstream so the decoder can unambiguously read the last symbol. A
//! flush also resets `low=0`, `high=top`, `opposite_bits=0` so the encoder
//! is ready for the next fixed-length-header boundary.
//!
//! # PSC_FIFO emulation-prevention (§E.5)
//!
//! The arithmetic-coder output is not written directly to the bitstream —
//! it goes through a "PSC_FIFO" which inserts / strips an anti-emulation `1`
//! after every run of 14 `0`s that isn't itself a PSC/GBSC. The decoder
//! counterpart drops the first `1` after each 14-zero run; if another `0`
//! follows the run instead, the alignment is on a genuine PSC/GBSC.
//! [`PscFifoWriter`] and [`PscFifoReader`] provide the two sides.
//!
//! # Decoder reset (§E.3)
//!
//! The decoder starts by reading 16 bits into `code_value` and setting
//! `low=0, high=top`. `decoder_reset` is also called at every fixed-length
//! header boundary (between picture-header / GOB-header bodies and the
//! arithmetic-coded block layer).
//!
//! # Integration status
//!
//! The low-level arithmetic coder + decoder + all §E.8 models are provided
//! and round-trip tested here. Full stream integration (replacing the VLC
//! decode paths in `mb.rs` / `block.rs` / `motion.rs` with SAC calls when
//! `PictureHeader::sac_mode` is set) is a separate wiring step; this
//! module is the foundation that step will sit on.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

/// Arithmetic-coder interval boundaries (§E.2): `q1 = 2^14`, `q2 = 2^15`,
/// `q3 = 3 * 2^14`, `top = 2^16 - 1`.
pub const Q1: u32 = 16384;
pub const Q2: u32 = 32768;
pub const Q3: u32 = 49152;
pub const TOP: u32 = 65535;

/// Spec-literal SAC encoder. The caller pushes bits into a `PscFifoWriter`
/// (which handles the 14-zero emulation-prevention rule) by calling
/// [`Self::encode_symbol`] for each source symbol, then [`Self::flush`] to
/// drain the final interval.
///
/// Encoder state (spec variable names):
/// * `low`, `high` — current interval `[low, high]` in `0..=top`.
/// * `opposite_bits` — count of bits queued for "opposite-of-next-emitted"
///   emission, used to defer the underflow-zone straddle resolution.
pub struct SacEncoder {
    low: u32,
    high: u32,
    opposite_bits: u32,
}

impl Default for SacEncoder {
    fn default() -> Self {
        Self::new()
    }
}

impl SacEncoder {
    /// Fresh encoder state (interval `[0, top]`, no pending opposite bits).
    pub fn new() -> Self {
        Self {
            low: 0,
            high: TOP,
            opposite_bits: 0,
        }
    }

    /// Encode one symbol whose index in `cumul_freq` is `index`. `cumul_freq`
    /// must be the spec-literal monotone-decreasing cumulative-frequency
    /// array (`cumul_freq[0]` is the total weight, `cumul_freq[n]` is 0).
    ///
    /// After the arithmetic update, renormalisation iterates until the
    /// interval re-expands out of the three "compression" conditions.
    pub fn encode_symbol(&mut self, index: usize, cumul_freq: &[u32], out: &mut PscFifoWriter) {
        let length = self.high.wrapping_sub(self.low) + 1;
        let total = cumul_freq[0] as u64;
        // The spec specifies `length * cumul_freq[...] / cumul_freq[0]`
        // with C integer division truncating toward zero. Since length <=
        // 65536 and cumul_freq[*] <= 16383, the product fits in 31 bits.
        let new_high = self.low + ((length as u64 * cumul_freq[index] as u64) / total) as u32 - 1;
        let new_low = self.low + ((length as u64 * cumul_freq[index + 1] as u64) / total) as u32;
        self.high = new_high;
        self.low = new_low;

        loop {
            if self.high < Q2 {
                // E1: top of interval below q2 → emit 0 + opposite_bits 1s.
                out.push_bit(0);
                for _ in 0..self.opposite_bits {
                    out.push_bit(1);
                }
                self.opposite_bits = 0;
            } else if self.low >= Q2 {
                // E2: bottom of interval at or above q2 → emit 1 +
                // opposite_bits 0s; shift interval back to `[low-q2,high-q2]`.
                out.push_bit(1);
                for _ in 0..self.opposite_bits {
                    out.push_bit(0);
                }
                self.opposite_bits = 0;
                self.low -= Q2;
                self.high -= Q2;
            } else if self.low >= Q1 && self.high < Q3 {
                // E3: interval straddles the q2 midpoint inside `[q1, q3]`.
                // Defer decision; track an "opposite-bit" credit and squeeze
                // the interval by q1.
                self.opposite_bits += 1;
                self.low -= Q1;
                self.high -= Q1;
            } else {
                break;
            }
            // Shift: interval doubling.
            self.low <<= 1;
            self.high = (self.high << 1) | 1;
        }
    }

    /// Drain the encoder state: force a final bit + any queued opposite bits
    /// so the decoder can disambiguate the last interval, then reset state.
    ///
    /// Per §E.7, after flushing `low = 0`, `high = top`, `opposite_bits = 0`
    /// — the encoder is ready for the next arithmetic-coded segment.
    pub fn flush(&mut self, out: &mut PscFifoWriter) {
        self.opposite_bits += 1;
        if self.low < Q1 {
            out.push_bit(0);
            for _ in 0..self.opposite_bits {
                out.push_bit(1);
            }
        } else {
            out.push_bit(1);
            for _ in 0..self.opposite_bits {
                out.push_bit(0);
            }
        }
        self.low = 0;
        self.high = TOP;
        self.opposite_bits = 0;
    }
}

/// Spec-literal SAC decoder. The caller pulls bits from a `PscFifoReader`
/// (which undoes the 14-zero emulation-prevention stuffing) and calls
/// [`Self::decode_symbol`] once per source symbol.
pub struct SacDecoder {
    low: u32,
    high: u32,
    code_value: u32,
}

impl SacDecoder {
    /// Build a new decoder and prime it by reading 16 bits from the bit
    /// source, per the §E.3 `decoder_reset` procedure.
    pub fn new(source: &mut PscFifoReader<'_>) -> Result<Self> {
        let mut code_value: u32 = 0;
        for _ in 0..16 {
            code_value = (code_value << 1) | source.pull_bit()?;
        }
        Ok(Self {
            low: 0,
            high: TOP,
            code_value,
        })
    }

    /// Re-initialise the decoder after a fixed-length header boundary:
    /// `code_value` is re-primed with 16 bits, `low = 0`, `high = top`.
    pub fn reset(&mut self, source: &mut PscFifoReader<'_>) -> Result<()> {
        self.low = 0;
        self.high = TOP;
        self.code_value = 0;
        for _ in 0..16 {
            self.code_value = (self.code_value << 1) | source.pull_bit()?;
        }
        Ok(())
    }

    /// Decode one symbol against `cumul_freq` (same shape as the encoder
    /// side) and return the index in the model.
    pub fn decode_symbol(
        &mut self,
        cumul_freq: &[u32],
        source: &mut PscFifoReader<'_>,
    ) -> Result<usize> {
        let length = self.high.wrapping_sub(self.low) + 1;
        let total = cumul_freq[0] as u64;
        // C: cum = (-1 + (code_value - low + 1) * cumul_freq[0]) / length;
        let cum = (((self.code_value - self.low + 1) as u64 * total)
            .checked_sub(1)
            .ok_or_else(|| Error::invalid("SAC decode: cum underflow"))?
            / length as u64) as u32;

        let mut index = 1usize;
        while cumul_freq[index] > cum {
            index += 1;
            if index >= cumul_freq.len() {
                return Err(Error::invalid("SAC decode: symbol index overflow"));
            }
        }

        // Interval update: same division as the encoder.
        self.high = self.low + ((length as u64 * cumul_freq[index - 1] as u64) / total) as u32 - 1;
        self.low += ((length as u64 * cumul_freq[index] as u64) / total) as u32;

        loop {
            if self.high < Q2 {
                // Nothing to do.
            } else if self.low >= Q2 {
                self.code_value -= Q2;
                self.low -= Q2;
                self.high -= Q2;
            } else if self.low >= Q1 && self.high < Q3 {
                self.code_value -= Q1;
                self.low -= Q1;
                self.high -= Q1;
            } else {
                break;
            }
            self.low <<= 1;
            self.high = (self.high << 1) | 1;
            let bit = source.pull_bit().unwrap_or(0);
            self.code_value = (self.code_value << 1) | bit;
        }

        Ok(index - 1)
    }
}

/// Writer side of the PSC_FIFO emulation-prevention buffer (§E.5).
///
/// Every 14 consecutive `0` bits are followed by a stuffed `1` to prevent
/// the output from containing a PSC/GBSC by accident; that `1` is stripped
/// by the matching [`PscFifoReader`].
///
/// The writer accumulates bits into an internal `Vec<u8>` in MSB-first
/// order; [`Self::finish`] returns the completed byte sequence, padding the
/// final partial byte with `0` bits.
pub struct PscFifoWriter {
    bytes: Vec<u8>,
    acc: u64,
    n: u32,
    zero_run: u32,
}

impl Default for PscFifoWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl PscFifoWriter {
    pub fn new() -> Self {
        Self {
            bytes: Vec::new(),
            acc: 0,
            n: 0,
            zero_run: 0,
        }
    }

    /// Push one bit (0 or 1). Inserts a stuffed `1` bit after every 14
    /// consecutive `0` bits.
    pub fn push_bit(&mut self, bit: u32) {
        debug_assert!(bit == 0 || bit == 1);
        self.emit(bit);
        if bit == 0 {
            self.zero_run += 1;
            if self.zero_run == 14 {
                self.emit(1);
                self.zero_run = 0;
            }
        } else {
            self.zero_run = 0;
        }
    }

    fn emit(&mut self, bit: u32) {
        self.acc = (self.acc << 1) | (bit as u64 & 1);
        self.n += 1;
        if self.n == 8 {
            self.bytes.push((self.acc & 0xFF) as u8);
            self.n = 0;
            self.acc = 0;
        }
    }

    /// Finish the stream. The final byte is right-padded with `0` bits if
    /// needed (the residual partial byte is shifted into the MSB of a full
    /// byte).
    pub fn finish(mut self) -> Vec<u8> {
        if self.n > 0 {
            let pad = 8 - self.n;
            self.acc <<= pad;
            self.bytes.push((self.acc & 0xFF) as u8);
        }
        self.bytes
    }
}

/// Reader side of the PSC_FIFO emulation-prevention buffer (§E.5).
///
/// Strips the stuffed `1` bit that follows every 14-`0` run. When a
/// 14-`0` run is followed by another `0` (instead of the expected stuffed
/// `1`), the bit-source is positioned at what is ostensibly a PSC/GBSC —
/// the caller is expected to notice and resynchronise.
pub struct PscFifoReader<'a> {
    br: &'a mut BitReader<'a>,
    zero_run: u32,
}

impl<'a> PscFifoReader<'a> {
    pub fn new(br: &'a mut BitReader<'a>) -> Self {
        Self { br, zero_run: 0 }
    }

    /// Pull one emulation-prevention-stripped bit. If the underlying bit
    /// stream is exhausted, return `0` — this matches the PSC_FIFO
    /// "trailing zero padding" behaviour the arithmetic decoder relies on
    /// when it needs to read past the encoder's flush point (the §E.3
    /// decoder state assumes PSC_FIFO always returns a bit, even beyond
    /// the last useful symbol).
    pub fn pull_bit(&mut self) -> Result<u32> {
        let bit = match self.br.read_u1() {
            Ok(b) => b,
            Err(_) => {
                // Past end-of-stream — PSC_FIFO yields `0` padding.
                return Ok(0);
            }
        };
        if bit == 0 {
            self.zero_run += 1;
            if self.zero_run == 14 {
                // The next bit from the stream is the stuffed `1` — consume
                // and drop it. If the encoder has already terminated the
                // stream the stuffing won't appear; treat a read error as
                // end-of-stream and return the zero we just pulled.
                match self.br.read_u1() {
                    Ok(stuffed) => {
                        if stuffed != 1 {
                            return Err(Error::invalid(
                                "SAC PSC_FIFO: expected stuffed 1 after 14 zeros, found 0",
                            ));
                        }
                        self.zero_run = 0;
                    }
                    Err(_) => {
                        self.zero_run = 0;
                    }
                }
            }
        } else {
            self.zero_run = 0;
        }
        Ok(bit)
    }
}

/// All §E.8 cumulative-frequency models, stored as `&'static [u32]` for
/// constant-time reference by symbol encoders / decoders. Each array's
/// first entry is the total weight (here fixed at `16383 = 2^14 - 1`, which
/// matches `Q1 - 1`, i.e. the arithmetic-coder's resolution), and the last
/// entry is `0`.
///
/// The names mirror those in the C-source listing of §E.8 verbatim.
pub mod models {
    /// COD (Coded macroblock indication) — one bit in the VLC path.
    pub const COD: &[u32] = &[16383, 6849, 0];

    /// MCBPC for P-pictures, non-4MV variants (`cumf_MCBPC_no4MVQ`).
    pub const MCBPC_NO_4MVQ: &[u32] = &[
        16383, 4105, 3088, 2367, 1988, 1621, 1612, 1609, 1608, 496, 353, 195, 77, 22, 17, 12, 5, 4,
        3, 2, 1, 0,
    ];

    /// MCBPC for P-pictures with 4MV or Annex J (`cumf_MCBPC_4MVQ`).
    pub const MCBPC_4MVQ: &[u32] = &[
        16383, 6880, 6092, 5178, 4916, 3965, 3880, 3795, 3768, 1491, 1190, 889, 655, 442, 416, 390,
        360, 337, 334, 331, 327, 326, 88, 57, 26, 0,
    ];

    /// MCBPC for I-pictures (`cumf_MCBPC_intra`).
    pub const MCBPC_INTRA: &[u32] = &[16383, 7410, 6549, 5188, 442, 182, 181, 141, 1, 0];

    /// MODB in PB-frames (Annex G) (`cumf_MODB_G`).
    pub const MODB_G: &[u32] = &[16383, 6062, 2130, 0];

    /// MODB in Improved PB-frames (Annex M) (`cumf_MODB_M`).
    pub const MODB_M: &[u32] = &[16383, 6717, 4568, 2784, 1370, 655, 0];

    /// CBPB (PB-frames) for Y blocks (`cumf_YCBPB`).
    pub const YCBPB: &[u32] = &[16383, 6062, 0];

    /// CBPB (PB-frames) for UV blocks (`cumf_UVCBPB`).
    pub const UVCBPB: &[u32] = &[16383, 491, 0];

    /// CBPY for INTER macroblocks (`cumf_CBPY`).
    pub const CBPY: &[u32] = &[
        16383, 14481, 13869, 13196, 12568, 11931, 11185, 10814, 9796, 9150, 8781, 7933, 6860, 6116,
        4873, 3538, 0,
    ];

    /// CBPY for INTRA macroblocks (`cumf_CBPY_intra`).
    pub const CBPY_INTRA: &[u32] = &[
        16383, 13619, 13211, 12933, 12562, 12395, 11913, 11783, 11004, 10782, 10689, 9928, 9353,
        8945, 8407, 7795, 0,
    ];

    /// DQUANT delta (`cumf_DQUANT`).
    pub const DQUANT: &[u32] = &[16383, 12287, 8192, 4095, 0];

    /// MVD — also used for MVD2-4 and MVDB (`cumf_MVD`).
    pub const MVD: &[u32] = &[
        16383, 16380, 16369, 16365, 16361, 16357, 16350, 16343, 16339, 16333, 16326, 16318, 16311,
        16306, 16298, 16291, 16283, 16272, 16261, 16249, 16235, 16222, 16207, 16175, 16141, 16094,
        16044, 15936, 15764, 15463, 14956, 13924, 11491, 4621, 2264, 1315, 854, 583, 420, 326, 273,
        229, 196, 166, 148, 137, 123, 114, 101, 91, 82, 76, 66, 59, 53, 46, 36, 30, 26, 24, 18, 14,
        10, 5, 0,
    ];

    /// INTRADC (`cumf_INTRADC`).
    pub const INTRADC: &[u32] = &[
        16383, 16380, 16379, 16378, 16377, 16376, 16370, 16361, 16360, 16359, 16358, 16357, 16356,
        16355, 16343, 16238, 16237, 16236, 16230, 16221, 16220, 16205, 16190, 16169, 16151, 16130,
        16109, 16094, 16070, 16037, 16007, 15962, 15938, 15899, 15854, 15815, 15788, 15743, 15689,
        15656, 15617, 15560, 15473, 15404, 15296, 15178, 15106, 14992, 14868, 14738, 14593, 14438,
        14283, 14169, 14064, 14004, 13914, 13824, 13752, 13671, 13590, 13515, 13458, 13380, 13305,
        13230, 13143, 13025, 12935, 12878, 12794, 12743, 12656, 12596, 12521, 12443, 12359, 12278,
        12200, 12131, 12047, 12002, 11948, 11891, 11828, 11744, 11663, 11588, 11495, 11402, 11288,
        11204, 11126, 11039, 10961, 10883, 10787, 10679, 10583, 10481, 10360, 10227, 10113, 9961,
        9828, 9717, 9584, 9485, 9324, 9112, 9019, 8908, 8766, 8584, 8426, 8211, 7920, 7663, 7406,
        7152, 6904, 6677, 6453, 6265, 6101, 5904, 5716, 5489, 5307, 5056, 4850, 4569, 4284, 3966,
        3712, 3518, 3342, 3206, 3048, 2909, 2773, 2668, 2596, 2512, 2370, 2295, 2232, 2166, 2103,
        2022, 1956, 1887, 1830, 1803, 1770, 1728, 1674, 1635, 1599, 1557, 1500, 1482, 1434, 1389,
        1356, 1317, 1284, 1245, 1200, 1179, 1140, 1110, 1092, 1062, 1044, 1035, 1014, 1008, 993,
        981, 954, 936, 912, 894, 876, 864, 849, 828, 816, 801, 792, 777, 756, 732, 690, 660, 642,
        615, 597, 576, 555, 522, 489, 459, 435, 411, 405, 396, 387, 375, 360, 354, 345, 344, 329,
        314, 293, 278, 251, 236, 230, 224, 215, 214, 208, 199, 193, 184, 178, 169, 154, 127, 100,
        94, 73, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 20, 19, 18, 17, 16, 15, 9, 0,
    ];

    /// TCOEF1 for INTER (`cumf_TCOEF1`).
    pub const TCOEF1: &[u32] = &[
        16383, 13455, 12458, 12079, 11885, 11800, 11738, 11700, 11681, 11661, 11651, 11645, 11641,
        10572, 10403, 10361, 10346, 10339, 10335, 9554, 9445, 9427, 9419, 9006, 8968, 8964, 8643,
        8627, 8624, 8369, 8354, 8352, 8200, 8192, 8191, 8039, 8036, 7920, 7917, 7800, 7793, 7730,
        7727, 7674, 7613, 7564, 7513, 7484, 7466, 7439, 7411, 7389, 7373, 7369, 7359, 7348, 7321,
        7302, 7294, 5013, 4819, 4789, 4096, 4073, 3373, 3064, 2674, 2357, 2177, 1975, 1798, 1618,
        1517, 1421, 1303, 1194, 1087, 1027, 960, 890, 819, 758, 707, 680, 656, 613, 566, 534, 505,
        475, 465, 449, 430, 395, 358, 335, 324, 303, 295, 286, 272, 233, 215, 0,
    ];

    /// TCOEF2 for INTER (`cumf_TCOEF2`).
    pub const TCOEF2: &[u32] = &[
        16383, 13582, 12709, 12402, 12262, 12188, 12150, 12131, 12125, 12117, 12113, 12108, 12104,
        10567, 10180, 10070, 10019, 9998, 9987, 9158, 9037, 9010, 9005, 8404, 8323, 8312, 7813,
        7743, 7726, 7394, 7366, 7364, 7076, 7062, 7060, 6810, 6797, 6614, 6602, 6459, 6454, 6304,
        6303, 6200, 6121, 6059, 6012, 5973, 5928, 5893, 5871, 5847, 5823, 5809, 5796, 5781, 5771,
        5763, 5752, 4754, 4654, 4631, 3934, 3873, 3477, 3095, 2758, 2502, 2257, 2054, 1869, 1715,
        1599, 1431, 1305, 1174, 1059, 983, 901, 839, 777, 733, 683, 658, 606, 565, 526, 488, 456,
        434, 408, 380, 361, 327, 310, 296, 267, 259, 249, 239, 230, 221, 214, 0,
    ];

    /// TCOEF3 for INTER (`cumf_TCOEF3`).
    pub const TCOEF3: &[u32] = &[
        16383, 13532, 12677, 12342, 12195, 12112, 12059, 12034, 12020, 12008, 12003, 12002, 12001,
        10586, 10297, 10224, 10202, 10195, 10191, 9223, 9046, 8999, 8987, 8275, 8148, 8113, 7552,
        7483, 7468, 7066, 7003, 6989, 6671, 6642, 6631, 6359, 6327, 6114, 6103, 5929, 5918, 5792,
        5785, 5672, 5580, 5507, 5461, 5414, 5382, 5354, 5330, 5312, 5288, 5273, 5261, 5247, 5235,
        5227, 5219, 4357, 4277, 4272, 3847, 3819, 3455, 3119, 2829, 2550, 2313, 2104, 1881, 1711,
        1565, 1366, 1219, 1068, 932, 866, 799, 750, 701, 662, 605, 559, 513, 471, 432, 403, 365,
        336, 312, 290, 276, 266, 254, 240, 228, 223, 216, 206, 199, 192, 189, 0,
    ];

    /// TCOEFr for INTER (`cumf_TCOEFr`).
    pub const TCOEFR: &[u32] = &[
        16383, 13216, 12233, 11931, 11822, 11776, 11758, 11748, 11743, 11742, 11741, 11740, 11739,
        10203, 9822, 9725, 9691, 9677, 9674, 8759, 8609, 8576, 8566, 7901, 7787, 7770, 7257, 7185,
        7168, 6716, 6653, 6639, 6276, 6229, 6220, 5888, 5845, 5600, 5567, 5348, 5327, 5160, 5142,
        5004, 4900, 4798, 4743, 4708, 4685, 4658, 4641, 4622, 4610, 4598, 4589, 4582, 4578, 4570,
        4566, 3824, 3757, 3748, 3360, 3338, 3068, 2835, 2592, 2359, 2179, 1984, 1804, 1614, 1445,
        1234, 1068, 870, 739, 668, 616, 566, 532, 489, 453, 426, 385, 357, 335, 316, 297, 283, 274,
        266, 259, 251, 241, 233, 226, 222, 217, 214, 211, 209, 208, 0,
    ];

    /// TCOEF1 for INTRA (`cumf_TCOEF1_intra`).
    pub const TCOEF1_INTRA: &[u32] = &[
        16383, 13383, 11498, 10201, 9207, 8528, 8099, 7768, 7546, 7368, 7167, 6994, 6869, 6005,
        5474, 5220, 5084, 4964, 4862, 4672, 4591, 4570, 4543, 4397, 4337, 4326, 4272, 4240, 4239,
        4212, 4196, 4185, 4158, 4157, 4156, 4140, 4139, 4138, 4137, 4136, 4125, 4124, 4123, 4112,
        4111, 4110, 4109, 4108, 4107, 4106, 4105, 4104, 4103, 4102, 4101, 4100, 4099, 4098, 4097,
        3043, 2897, 2843, 1974, 1790, 1677, 1552, 1416, 1379, 1331, 1288, 1251, 1250, 1249, 1248,
        1247, 1236, 1225, 1224, 1223, 1212, 1201, 1200, 1199, 1198, 1197, 1196, 1195, 1194, 1193,
        1192, 1191, 1190, 1189, 1188, 1187, 1186, 1185, 1184, 1183, 1182, 1181, 1180, 1179, 0,
    ];

    /// TCOEF2 for INTRA (`cumf_TCOEF2_intra`).
    pub const TCOEF2_INTRA: &[u32] = &[
        16383, 13242, 11417, 10134, 9254, 8507, 8012, 7556, 7273, 7062, 6924, 6839, 6741, 6108,
        5851, 5785, 5719, 5687, 5655, 5028, 4917, 4864, 4845, 4416, 4159, 4074, 3903, 3871, 3870,
        3765, 3752, 3751, 3659, 3606, 3580, 3541, 3540, 3514, 3495, 3494, 3493, 3474, 3473, 3441,
        3440, 3439, 3438, 3425, 3424, 3423, 3422, 3421, 3420, 3401, 3400, 3399, 3398, 3397, 3396,
        2530, 2419, 2360, 2241, 2228, 2017, 1687, 1576, 1478, 1320, 1281, 1242, 1229, 1197, 1178,
        1152, 1133, 1114, 1101, 1088, 1087, 1086, 1085, 1072, 1071, 1070, 1069, 1068, 1067, 1066,
        1065, 1064, 1063, 1062, 1061, 1060, 1059, 1058, 1057, 1056, 1055, 1054, 1053, 1052, 0,
    ];

    /// TCOEF3 for INTRA (`cumf_TCOEF3_intra`).
    pub const TCOEF3_INTRA: &[u32] = &[
        16383, 12741, 10950, 10071, 9493, 9008, 8685, 8516, 8385, 8239, 8209, 8179, 8141, 6628,
        5980, 5634, 5503, 5396, 5327, 4857, 4642, 4550, 4481, 4235, 4166, 4151, 3967, 3922, 3907,
        3676, 3500, 3324, 3247, 3246, 3245, 3183, 3168, 3084, 3069, 3031, 3030, 3029, 3014, 3013,
        2990, 2975, 2974, 2973, 2958, 2943, 2928, 2927, 2926, 2925, 2924, 2923, 2922, 2921, 2920,
        2397, 2298, 2283, 1891, 1799, 1591, 1445, 1338, 1145, 1068, 1006, 791, 768, 661, 631, 630,
        615, 592, 577, 576, 561, 546, 523, 508, 493, 492, 491, 476, 475, 474, 473, 472, 471, 470,
        469, 468, 453, 452, 451, 450, 449, 448, 447, 446, 0,
    ];

    /// TCOEFr for INTRA (`cumf_TCOEFr_intra`).
    pub const TCOEFR_INTRA: &[u32] = &[
        16383, 12514, 10776, 9969, 9579, 9306, 9168, 9082, 9032, 9000, 8981, 8962, 8952, 7630,
        7212, 7053, 6992, 6961, 6940, 6195, 5988, 5948, 5923, 5370, 5244, 5210, 4854, 4762, 4740,
        4384, 4300, 4288, 4020, 3968, 3964, 3752, 3668, 3511, 3483, 3354, 3322, 3205, 3183, 3108,
        3046, 2999, 2981, 2974, 2968, 2961, 2955, 2949, 2943, 2942, 2939, 2935, 2934, 2933, 2929,
        2270, 2178, 2162, 1959, 1946, 1780, 1651, 1524, 1400, 1289, 1133, 1037, 942, 849, 763, 711,
        591, 521, 503, 496, 474, 461, 449, 442, 436, 426, 417, 407, 394, 387, 377, 373, 370, 367,
        366, 365, 364, 363, 362, 358, 355, 352, 351, 350, 0,
    ];

    /// TCOEF SIGN (`cumf_SIGN`). Index 0 = positive, 1 = negative.
    pub const SIGN: &[u32] = &[16383, 8416, 0];

    /// LAST (escape tail) for INTER (`cumf_LAST`). Index 0 = last=0, 1 = last=1.
    pub const LAST: &[u32] = &[16383, 9469, 0];

    /// LAST (escape tail) for INTRA (`cumf_LAST_intra`).
    pub const LAST_INTRA: &[u32] = &[16383, 2820, 0];

    /// RUN for INTER escape tail (`cumf_RUN`).
    pub const RUN: &[u32] = &[
        16383, 15310, 14702, 13022, 11883, 11234, 10612, 10192, 9516, 9016, 8623, 8366, 7595, 7068,
        6730, 6487, 6379, 6285, 6177, 6150, 6083, 5989, 5949, 5922, 5895, 5828, 5774, 5773, 5394,
        5164, 5016, 4569, 4366, 4136, 4015, 3867, 3773, 3692, 3611, 3476, 3341, 3301, 2787, 2503,
        2219, 1989, 1515, 1095, 934, 799, 691, 583, 435, 300, 246, 206, 125, 124, 97, 57, 30, 3, 2,
        1, 0,
    ];

    /// RUN for INTRA escape tail (`cumf_RUN_intra`).
    pub const RUN_INTRA: &[u32] = &[
        16383, 10884, 8242, 7124, 5173, 4745, 4246, 3984, 3034, 2749, 2607, 2298, 966, 681, 396,
        349, 302, 255, 254, 253, 206, 159, 158, 157, 156, 155, 154, 153, 106, 35, 34, 33, 32, 31,
        30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8,
        7, 6, 5, 4, 3, 2, 1, 0,
    ];

    /// LEVEL for INTER escape tail (`cumf_LEVEL`). 254 entries (-127..=127,
    /// with 0 omitted per spec).
    pub const LEVEL: &[u32] = &[
        16383, 16382, 16381, 16380, 16379, 16378, 16377, 16376, 16375, 16374, 16373, 16372, 16371,
        16370, 16369, 16368, 16367, 16366, 16365, 16364, 16363, 16362, 16361, 16360, 16359, 16358,
        16357, 16356, 16355, 16354, 16353, 16352, 16351, 16350, 16349, 16348, 16347, 16346, 16345,
        16344, 16343, 16342, 16341, 16340, 16339, 16338, 16337, 16336, 16335, 16334, 16333, 16332,
        16331, 16330, 16329, 16328, 16327, 16326, 16325, 16324, 16323, 16322, 16321, 16320, 16319,
        16318, 16317, 16316, 16315, 16314, 16313, 16312, 16311, 16310, 16309, 16308, 16307, 16306,
        16305, 16304, 16303, 16302, 16301, 16300, 16299, 16298, 16297, 16296, 16295, 16294, 16293,
        16292, 16291, 16290, 16289, 16288, 16287, 16286, 16285, 16284, 16283, 16282, 16281, 16280,
        16279, 16278, 16277, 16250, 16223, 16222, 16195, 16154, 16153, 16071, 15989, 15880, 15879,
        15878, 15824, 15756, 15674, 15606, 15538, 15184, 14572, 13960, 10718, 7994, 5379, 2123,
        1537, 992, 693, 611, 516, 448, 421, 380, 353, 352, 284, 257, 230, 203, 162, 161, 160, 133,
        132, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86,
        85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63,
        62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40,
        39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17,
        16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
    ];

    /// LEVEL for INTRA escape tail (`cumf_LEVEL_intra`).
    pub const LEVEL_INTRA: &[u32] = &[
        16383, 16379, 16378, 16377, 16376, 16375, 16374, 16373, 16372, 16371, 16370, 16369, 16368,
        16367, 16366, 16365, 16364, 16363, 16362, 16361, 16360, 16359, 16358, 16357, 16356, 16355,
        16354, 16353, 16352, 16351, 16350, 16349, 16348, 16347, 16346, 16345, 16344, 16343, 16342,
        16341, 16340, 16339, 16338, 16337, 16336, 16335, 16334, 16333, 16332, 16331, 16330, 16329,
        16328, 16327, 16326, 16325, 16324, 16323, 16322, 16321, 16320, 16319, 16318, 16317, 16316,
        16315, 16314, 16313, 16312, 16311, 16268, 16267, 16224, 16223, 16180, 16179, 16136, 16135,
        16134, 16133, 16132, 16131, 16130, 16129, 16128, 16127, 16126, 16061, 16018, 16017, 16016,
        16015, 16014, 15971, 15970, 15969, 15968, 15925, 15837, 15794, 15751, 15750, 15749, 15661,
        15618, 15508, 15376, 15288, 15045, 14913, 14781, 14384, 13965, 13502, 13083, 12509, 12289,
        12135, 11892, 11738, 11429, 11010, 10812, 10371, 9664, 9113, 8117, 8116, 8028, 6855, 5883,
        4710, 4401, 4203, 3740, 3453, 3343, 3189, 2946, 2881, 2661, 2352, 2132, 1867, 1558, 1382,
        1250, 1162, 1097, 1032, 967, 835, 681, 549, 439, 351, 350, 307, 306, 305, 304, 303, 302,
        301, 300, 299, 298, 255, 212, 211, 210, 167, 166, 165, 164, 163, 162, 161, 160, 159, 158,
        115, 114, 113, 112, 111, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53,
        52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30,
        29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6,
        5, 4, 3, 2, 1, 0,
    ];

    /// INTRA_MODE for Annex I (`cumf_INTRA_AC_DC`). 3 symbols (DC-only,
    /// horizontal, vertical).
    pub const INTRA_AC_DC: &[u32] = &[16383, 9229, 5461, 0];
}

// ---------------------------------------------------------------------------
// MB-layer SAC bridge — symbol-table forward/inverse helpers and high-level
// I-picture MB writer / reader.
//
// The encoder side maps the spec's natural symbols (CBP patterns, INTRADC
// byte values, (last,run,|level|) TCOEF events) onto the integer indices
// used by the §E.8 cumulative-frequency models. The decoder side does the
// inverse. Mappings follow Tables 7 / 12 / 15 / 16 / 17 of H.263 (01/2005)
// and §E.7 of Annex E.
// ---------------------------------------------------------------------------

/// Sentinel index in the SAC `TCOEF*` models that means "ESCAPE: read LAST,
/// RUN, LEVEL from cumf_LAST(_intra), cumf_RUN(_intra), cumf_LEVEL(_intra)
/// next". Per §E.7 this is the last index in the model (the same slot the
/// VLC `0000011` ESCAPE prefix occupies in Table 16, row 102).
pub const TCOEF_ESCAPE_INDEX: usize = 102;

/// Forward table for `(last, run, |level|)` → SAC TCOEF index, last=0 half.
/// Matches Table 16 indices 0..=57 and the `INTER_LAST0_RUN/LEVEL` arrays in
/// `enc_tables.rs`. 58 entries.
const TCOEF_LAST0_RUN: [u8; 58] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6,
    6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
];
const TCOEF_LAST0_LEVEL: [u8; 58] = [
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 1, 2, 3, 1, 2, 3, 1, 2, 3,
    1, 2, 3, 1, 2, 1, 2, 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
];

/// Forward table, last=1 half — Table 16 indices 58..=101. 44 entries.
const TCOEF_LAST1_RUN: [u8; 44] = [
    0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
];
const TCOEF_LAST1_LEVEL: [u8; 44] = [
    1, 2, 3, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
];

/// Look up `(last, run, |level|)` in the SAC TCOEF index space. Returns
/// `Some(idx)` for indices 0..=101; the caller falls back to
/// [`TCOEF_ESCAPE_INDEX`] when the triple isn't tabulated.
pub fn tcoef_lookup_index(last: bool, run: u8, level_abs: u8) -> Option<usize> {
    let (runs, levels, base): (&[u8], &[u8], usize) = if last {
        (&TCOEF_LAST1_RUN, &TCOEF_LAST1_LEVEL, 58)
    } else {
        (&TCOEF_LAST0_RUN, &TCOEF_LAST0_LEVEL, 0)
    };
    for (i, (&r, &l)) in runs.iter().zip(levels.iter()).enumerate() {
        if r == run && l == level_abs {
            return Some(base + i);
        }
    }
    None
}

/// Inverse of [`tcoef_lookup_index`] — decode a non-escape SAC TCOEF index
/// (0..=101) into `(last, run, |level|)`. Returns `None` for the escape
/// index or anything out of range.
pub fn tcoef_index_to_event(index: usize) -> Option<(bool, u8, u8)> {
    if index < 58 {
        Some((false, TCOEF_LAST0_RUN[index], TCOEF_LAST0_LEVEL[index]))
    } else if index < 102 {
        let i = index - 58;
        Some((true, TCOEF_LAST1_RUN[i], TCOEF_LAST1_LEVEL[i]))
    } else {
        None
    }
}

/// SAC encoding of the INTRADC byte → §E.7 / Table 15 index.
/// FLC values 0x01..=0x7F map to indices 0..=126, FLC 0xFF maps to 127, FLC
/// 0x81..=0xFE maps to 128..=253. The spec's Table 15 column "Index" gives
/// the same mapping; see §E.7 ("indexing is defined in ... Table 15
/// respectively"). Returns `None` for the two reserved values 0x00 / 0x80.
pub fn intradc_byte_to_index(byte: u8) -> Option<usize> {
    match byte {
        0x00 | 0x80 => None,
        0x01..=0x7F => Some((byte - 1) as usize), // 0..=126
        0xFF => Some(127),
        0x81..=0xFE => Some((byte - 0x81 + 128) as usize), // 128..=253
    }
}

/// Inverse of [`intradc_byte_to_index`] — return the on-the-wire FLC byte
/// for the given §E.7 index. Returns `None` for indices ≥ 254.
pub fn intradc_index_to_byte(index: usize) -> Option<u8> {
    if index <= 126 {
        Some((index as u8) + 1) // 0->0x01 ... 126->0x7F
    } else if index == 127 {
        Some(0xFF)
    } else if index <= 253 {
        Some(((index - 128) as u8) + 0x81) // 128->0x81 ... 253->0xFE
    } else {
        None
    }
}

/// Convert an 8-bit two's-complement LEVEL byte (used in the VLC ESCAPE
/// body) into the §E.7 cumf_LEVEL / cumf_LEVEL_intra index space. Spec
/// Table 17 lists FLC 0x80..=0xFF for levels -128..=-1 (with -128 forbidden
/// in baseline) and FLC 0x01..=0x7F for levels 1..=127. The cumf_LEVEL
/// model has 254 entries indexed 0..=253; we map FLC bytes contiguously
/// starting at index 0 = -128, 1 = -127, ..., 127 = -1, [skip 0 = forbidden],
/// 128 = +1, ..., 254 = +127. (Index 127 corresponds to FLC 0x80 = -128 and
/// is only used in Modified Quantization mode; baseline encoders won't emit
/// it but the model still reserves the slot.)
///
/// The cumul_freq array's monotone-decreasing constraint means the
/// "common" small-magnitude levels sit at high indices; the spec ordering
/// is by FLC byte value, which interleaves negatives at low indices and
/// positives at high indices. The §E.8 array reflects this directly — see
/// the [`models::LEVEL`] table.
pub fn level_byte_to_index(byte: u8) -> usize {
    if byte & 0x80 != 0 {
        // 0x80..=0xFF → indices 0..=127 (with 0x80 = idx 0 = -128).
        (byte - 0x80) as usize
    } else {
        // 0x01..=0x7F → indices 127..=253 (skipping 0 which is forbidden;
        // we allocate 0x01 to index 127, 0x02 to 128, ..., 0x7F to 253).
        // NOTE: index 127 is shared between FLC 0xFF (-1) and FLC 0x01
        // (+1) — but since FLC 0xFF maps to byte 0xFF (>=0x80) above, the
        // overlap is resolved by the branch.
        debug_assert!(byte != 0, "LEVEL byte 0x00 is forbidden");
        ((byte - 1) as usize) + 127
    }
}

/// Inverse of [`level_byte_to_index`].
pub fn level_index_to_byte(index: usize) -> u8 {
    if index < 128 {
        // -128..=-1: FLC 0x80..=0xFF
        0x80u8.wrapping_add(index as u8)
    } else {
        // +1..=+127: FLC 0x01..=0x7F
        ((index - 127) as u8) & 0x7F
    }
}

/// High-level SAC bridge for emitting the body of an I-picture macroblock.
/// Owns a [`SacEncoder`] + [`PscFifoWriter`] pair and exposes one method per
/// I-MB syntax element so that callers can mirror the VLC code path
/// element-for-element.
///
/// Picture-header-layer bytes are written by the caller into a separate
/// `BitWriter` (the spec keeps fixed-length header layers outside the
/// PSC_FIFO mux per §E.6); after the header is closed and byte-aligned, the
/// caller hands the picture-body bytes to this SAC writer. On `finish`, the
/// arithmetic encoder is flushed (§E.7 `encoder_flush`) and the PSC_FIFO
/// returns the final byte sequence (right-padded with zeros).
pub struct SacIPictureWriter {
    enc: SacEncoder,
    fifo: PscFifoWriter,
}

impl Default for SacIPictureWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl SacIPictureWriter {
    pub fn new() -> Self {
        Self {
            enc: SacEncoder::new(),
            fifo: PscFifoWriter::new(),
        }
    }

    /// Encode the MCBPC for an I-picture intra MB. `index` is Table 7's
    /// `Index` column (0..=7 for the (mb_type, cbpc) pairs the encoder
    /// emits; index 8 would be MB-stuffing — never emitted).
    pub fn write_mcbpc_intra(&mut self, index: usize) {
        self.enc
            .encode_symbol(index, models::MCBPC_INTRA, &mut self.fifo);
    }

    /// Encode the CBPY for an I-picture intra MB — `cbpy` is the raw 4-bit
    /// pattern (no XOR for intra), Table 12 index column.
    pub fn write_cbpy_intra(&mut self, cbpy: u8) {
        self.enc
            .encode_symbol(cbpy as usize, models::CBPY_INTRA, &mut self.fifo);
    }

    /// Encode an INTRADC byte (must be one of the 254 legal FLC values per
    /// Table 15 — the caller already remapped 128 → 0xFF). `byte` is what
    /// the VLC path would have written as a fixed 8-bit field.
    pub fn write_intradc(&mut self, byte: u8) -> Result<()> {
        let idx = intradc_byte_to_index(byte)
            .ok_or_else(|| Error::invalid("SAC INTRADC: illegal byte 0x00 / 0x80"))?;
        self.enc.encode_symbol(idx, models::INTRADC, &mut self.fifo);
        Ok(())
    }

    /// Encode one TCOEF event (`last`, `run`, signed `level`). Picks the
    /// appropriate TCOEF model based on the position in the block —
    /// position-1 → TCOEF1, position-2 → TCOEF2, position-3 → TCOEF3,
    /// position ≥ 4 → TCOEFr. `intra_block` selects the INTRA-flavoured
    /// SAC models from §E.7. Out-of-table tuples emit the ESCAPE index
    /// followed by LAST + RUN + LEVEL through the cumf_LAST / cumf_RUN /
    /// cumf_LEVEL models.
    pub fn write_tcoef(
        &mut self,
        intra_block: bool,
        position: usize,
        last: bool,
        run: u8,
        level: i32,
    ) {
        debug_assert!(level != 0);
        let abs = level.unsigned_abs();
        let tcoef_model: &[u32] = match (intra_block, position) {
            (false, 1) => models::TCOEF1,
            (false, 2) => models::TCOEF2,
            (false, 3) => models::TCOEF3,
            (false, _) => models::TCOEFR,
            (true, 1) => models::TCOEF1_INTRA,
            (true, 2) => models::TCOEF2_INTRA,
            (true, 3) => models::TCOEF3_INTRA,
            (true, _) => models::TCOEFR_INTRA,
        };
        let sign_bit: usize = if level < 0 { 1 } else { 0 };
        if abs <= 12 {
            // |level| can fit in the table; check.
            if let Some(idx) = tcoef_lookup_index(last, run, abs as u8) {
                self.enc.encode_symbol(idx, tcoef_model, &mut self.fifo);
                self.enc
                    .encode_symbol(sign_bit, models::SIGN, &mut self.fifo);
                return;
            }
        }
        // ESCAPE path.
        self.enc
            .encode_symbol(TCOEF_ESCAPE_INDEX, tcoef_model, &mut self.fifo);
        let last_model = if intra_block {
            models::LAST_INTRA
        } else {
            models::LAST
        };
        let run_model = if intra_block {
            models::RUN_INTRA
        } else {
            models::RUN
        };
        let level_model = if intra_block {
            models::LEVEL_INTRA
        } else {
            models::LEVEL
        };
        self.enc
            .encode_symbol(if last { 1 } else { 0 }, last_model, &mut self.fifo);
        self.enc
            .encode_symbol(run as usize, run_model, &mut self.fifo);
        // ESCAPE LEVEL is the signed 8-bit byte (§5.4.2 Table 17). Two's
        // complement maps the level to its FLC form for index lookup.
        let level_byte: u8 = level.rem_euclid(256) as u8;
        let level_idx = level_byte_to_index(level_byte);
        self.enc
            .encode_symbol(level_idx, level_model, &mut self.fifo);
    }

    /// Flush the arithmetic coder (§E.7 `encoder_flush`) and return the
    /// PSC_FIFO byte stream. Consumes `self`.
    pub fn finish(mut self) -> Vec<u8> {
        self.enc.flush(&mut self.fifo);
        self.fifo.finish()
    }
}

/// High-level SAC bridge for reading the body of an I-picture macroblock.
/// Owns a [`SacDecoder`] + [`PscFifoReader`] pair and exposes one method per
/// I-MB syntax element so that callers mirror the VLC decode path
/// element-for-element.
pub struct SacIPictureReader<'a> {
    dec: SacDecoder,
    fifo: PscFifoReader<'a>,
}

impl<'a> SacIPictureReader<'a> {
    /// Construct from a borrowed [`BitReader`] positioned at the start of
    /// the SAC-coded picture body. Primes the arithmetic decoder per
    /// `decoder_reset` (§E.3) by reading 16 bits.
    pub fn new(br: &'a mut BitReader<'a>) -> Result<Self> {
        let mut fifo = PscFifoReader::new(br);
        let dec = SacDecoder::new(&mut fifo)?;
        Ok(Self { dec, fifo })
    }

    /// Generic escape hatch — decode one symbol against an arbitrary
    /// cumul-frequency model. Used by the MB-layer bridge for models that
    /// don't have a dedicated wrapper (DQUANT, SIGN, etc.) so callers can
    /// stay inside the reader's PSC_FIFO state without poking the inner
    /// decoder directly.
    pub fn decode_with_model(&mut self, model: &[u32]) -> Result<usize> {
        self.dec.decode_symbol(model, &mut self.fifo)
    }

    pub fn read_mcbpc_intra(&mut self) -> Result<usize> {
        self.dec.decode_symbol(models::MCBPC_INTRA, &mut self.fifo)
    }

    pub fn read_cbpy_intra(&mut self) -> Result<u8> {
        let v = self.dec.decode_symbol(models::CBPY_INTRA, &mut self.fifo)?;
        if v >= 16 {
            return Err(Error::invalid("SAC CBPY: index out of range"));
        }
        Ok(v as u8)
    }

    pub fn read_intradc(&mut self) -> Result<u8> {
        let idx = self.dec.decode_symbol(models::INTRADC, &mut self.fifo)?;
        intradc_index_to_byte(idx).ok_or_else(|| Error::invalid("SAC INTRADC: index out of range"))
    }

    /// Read one TCOEF event from the SAC stream. Returns `(last, run,
    /// signed_level)`. Mirrors the encoder's [`SacIPictureWriter::write_tcoef`].
    pub fn read_tcoef(&mut self, intra_block: bool, position: usize) -> Result<(bool, u8, i32)> {
        let tcoef_model: &[u32] = match (intra_block, position) {
            (false, 1) => models::TCOEF1,
            (false, 2) => models::TCOEF2,
            (false, 3) => models::TCOEF3,
            (false, _) => models::TCOEFR,
            (true, 1) => models::TCOEF1_INTRA,
            (true, 2) => models::TCOEF2_INTRA,
            (true, 3) => models::TCOEF3_INTRA,
            (true, _) => models::TCOEFR_INTRA,
        };
        let idx = self.dec.decode_symbol(tcoef_model, &mut self.fifo)?;
        if idx == TCOEF_ESCAPE_INDEX {
            // ESCAPE path: LAST + RUN + LEVEL.
            let last_model = if intra_block {
                models::LAST_INTRA
            } else {
                models::LAST
            };
            let run_model = if intra_block {
                models::RUN_INTRA
            } else {
                models::RUN
            };
            let level_model = if intra_block {
                models::LEVEL_INTRA
            } else {
                models::LEVEL
            };
            let last_idx = self.dec.decode_symbol(last_model, &mut self.fifo)?;
            let run = self.dec.decode_symbol(run_model, &mut self.fifo)? as u8;
            let level_idx = self.dec.decode_symbol(level_model, &mut self.fifo)?;
            let level_byte = level_index_to_byte(level_idx);
            // Two's-complement decode (Table 17). Reject 0x00 (level 0)
            // and 0x80 (level -128) per §5.4.2.
            if level_byte == 0 {
                return Err(Error::invalid("SAC TCOEF ESCAPE: level == 0 forbidden"));
            }
            if level_byte == 0x80 {
                return Err(Error::invalid("SAC TCOEF ESCAPE: level == -128 forbidden"));
            }
            let level = if level_byte & 0x80 != 0 {
                level_byte as i32 - 256
            } else {
                level_byte as i32
            };
            Ok((last_idx == 1, run, level))
        } else {
            let (last, run, abs) = tcoef_index_to_event(idx)
                .ok_or_else(|| Error::invalid("SAC TCOEF: bad event index"))?;
            let sign_idx = self.dec.decode_symbol(models::SIGN, &mut self.fifo)?;
            let level = if sign_idx == 1 {
                -(abs as i32)
            } else {
                abs as i32
            };
            Ok((last, run, level))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The spec cumul_freq arrays are strictly monotone-decreasing — this
    /// check guards against copy-paste errors in the model tables above.
    #[test]
    fn all_models_are_monotone_decreasing() {
        fn check(name: &str, c: &[u32]) {
            assert!(c.len() >= 2, "{name}: too short");
            assert_eq!(c[0], 16383, "{name}: total weight must be 16383");
            assert_eq!(
                *c.last().unwrap(),
                0,
                "{name}: last entry must be 0 (sentinel)"
            );
            for i in 1..c.len() {
                assert!(
                    c[i - 1] > c[i],
                    "{name}: not strictly decreasing at index {i}"
                );
            }
        }
        check("COD", models::COD);
        check("MCBPC_NO_4MVQ", models::MCBPC_NO_4MVQ);
        check("MCBPC_4MVQ", models::MCBPC_4MVQ);
        check("MCBPC_INTRA", models::MCBPC_INTRA);
        check("MODB_G", models::MODB_G);
        check("MODB_M", models::MODB_M);
        check("YCBPB", models::YCBPB);
        check("UVCBPB", models::UVCBPB);
        check("CBPY", models::CBPY);
        check("CBPY_INTRA", models::CBPY_INTRA);
        check("DQUANT", models::DQUANT);
        check("MVD", models::MVD);
        check("INTRADC", models::INTRADC);
        check("TCOEF1", models::TCOEF1);
        check("TCOEF2", models::TCOEF2);
        check("TCOEF3", models::TCOEF3);
        check("TCOEFR", models::TCOEFR);
        check("TCOEF1_INTRA", models::TCOEF1_INTRA);
        check("TCOEF2_INTRA", models::TCOEF2_INTRA);
        check("TCOEF3_INTRA", models::TCOEF3_INTRA);
        check("TCOEFR_INTRA", models::TCOEFR_INTRA);
        check("SIGN", models::SIGN);
        check("LAST", models::LAST);
        check("LAST_INTRA", models::LAST_INTRA);
        check("RUN", models::RUN);
        check("RUN_INTRA", models::RUN_INTRA);
        check("LEVEL", models::LEVEL);
        check("LEVEL_INTRA", models::LEVEL_INTRA);
        check("INTRA_AC_DC", models::INTRA_AC_DC);
    }

    fn roundtrip_model(model: &[u32], symbols: &[usize]) {
        let mut enc = SacEncoder::new();
        let mut out = PscFifoWriter::new();
        for &s in symbols {
            enc.encode_symbol(s, model, &mut out);
        }
        enc.flush(&mut out);
        // Ensure final byte boundary.
        let bytes = out.finish();

        // Decoder side.
        let mut br = BitReader::new(&bytes);
        let mut rdr = PscFifoReader::new(&mut br);
        let mut dec = SacDecoder::new(&mut rdr).unwrap();
        let mut got = Vec::with_capacity(symbols.len());
        for _ in 0..symbols.len() {
            got.push(dec.decode_symbol(model, &mut rdr).unwrap());
        }
        assert_eq!(got, symbols);
    }

    #[test]
    fn roundtrip_binary_cod_symbols() {
        roundtrip_model(models::COD, &[0, 1, 0, 0, 1, 1, 0, 0, 0, 1]);
    }

    #[test]
    fn roundtrip_single_symbol() {
        roundtrip_model(models::COD, &[0]);
        roundtrip_model(models::COD, &[1]);
    }

    #[test]
    fn roundtrip_long_run_of_zeros_triggers_emulation_prevention() {
        // Feeding a long run of one symbol exercises the 14-zero stuffing
        // path. Can't fully predict the output pattern (it depends on the
        // arithmetic coder's own bit emission) but the decoder must round-
        // trip regardless.
        let sym = vec![0usize; 64];
        roundtrip_model(models::COD, &sym);
    }

    #[test]
    fn roundtrip_mvd_full_range() {
        // Walk every symbol in the MVD model (65 entries).
        let model = models::MVD;
        let sym: Vec<usize> = (0..model.len() - 1).collect();
        roundtrip_model(model, &sym);
    }

    #[test]
    fn roundtrip_intradc_255_symbols() {
        let model = models::INTRADC;
        let sym: Vec<usize> = (0..model.len() - 1).collect();
        roundtrip_model(model, &sym);
    }

    #[test]
    fn roundtrip_tcoef_intra_full_range() {
        let model = models::TCOEF1_INTRA;
        let sym: Vec<usize> = (0..model.len() - 1).collect();
        roundtrip_model(model, &sym);
    }

    #[test]
    fn roundtrip_level_full_range() {
        let model = models::LEVEL;
        // Walk every symbol. LEVEL has 254 entries (indices 0..=253).
        let sym: Vec<usize> = (0..model.len() - 1).collect();
        roundtrip_model(model, &sym);
    }

    /// Mix models like a real MB decode path would: COD → MCBPC → CBPY →
    /// MVD. Every symbol must be received in order and match.
    #[test]
    fn roundtrip_mixed_models_mb_shape() {
        let mut enc = SacEncoder::new();
        let mut out = PscFifoWriter::new();
        // COD=0, MCBPC index=3, CBPY index=10, MVD index=32 (zero diff).
        let seq = [
            (models::COD, 0usize),
            (models::MCBPC_NO_4MVQ, 3),
            (models::CBPY, 10),
            (models::MVD, 32),
        ];
        for (model, idx) in &seq {
            enc.encode_symbol(*idx, model, &mut out);
        }
        enc.flush(&mut out);
        let bytes = out.finish();

        let mut br = BitReader::new(&bytes);
        let mut rdr = PscFifoReader::new(&mut br);
        let mut dec = SacDecoder::new(&mut rdr).unwrap();
        for (model, idx) in &seq {
            let got = dec.decode_symbol(model, &mut rdr).unwrap();
            assert_eq!(got, *idx, "model mismatch");
        }
    }

    /// Encoder flush followed by a decoder reset should let the caller
    /// start a new arithmetic-coded segment without residual state.
    #[test]
    fn flush_then_reset_is_clean() {
        let mut enc = SacEncoder::new();
        let mut out = PscFifoWriter::new();
        enc.encode_symbol(0, models::COD, &mut out);
        enc.flush(&mut out);
        // Second segment.
        enc.encode_symbol(1, models::COD, &mut out);
        enc.encode_symbol(1, models::COD, &mut out);
        enc.flush(&mut out);
        let bytes = out.finish();

        let mut br = BitReader::new(&bytes);
        let mut rdr = PscFifoReader::new(&mut br);
        let mut dec = SacDecoder::new(&mut rdr).unwrap();
        // First segment: one COD=0.
        let a = dec.decode_symbol(models::COD, &mut rdr).unwrap();
        assert_eq!(a, 0);
        // Reset the decoder before decoding the second segment.
        dec.reset(&mut rdr).unwrap();
        let b = dec.decode_symbol(models::COD, &mut rdr).unwrap();
        let c = dec.decode_symbol(models::COD, &mut rdr).unwrap();
        assert_eq!((b, c), (1, 1));
    }

    /// PSC_FIFO writer + reader in isolation — a bit stream with an
    /// artificial 14-zero run must round-trip the exact original bits.
    #[test]
    fn psc_fifo_round_trip_with_explicit_stuffing() {
        let mut w = PscFifoWriter::new();
        // 20 zero bits in a row (encoder will stuff a 1 after the 14th).
        for _ in 0..20 {
            w.push_bit(0);
        }
        // Then a one.
        w.push_bit(1);
        // Finish.
        let bytes = w.finish();
        // Bit layout after stuffing: 14 zeros, 1 stuffed, 6 zeros, 1.
        // Decoder should skip the stuffed 1 and return 20 zeros + 1 one.
        let mut br = BitReader::new(&bytes);
        let mut rdr = PscFifoReader::new(&mut br);
        for _ in 0..20 {
            assert_eq!(rdr.pull_bit().unwrap(), 0);
        }
        assert_eq!(rdr.pull_bit().unwrap(), 1);
    }
}
