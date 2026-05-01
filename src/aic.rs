//! ITU-T H.263 Annex I — Advanced INTRA Coding (AIC).
//!
//! AIC is opt-in via the H.263+ PLUSPTYPE block (OPPTYPE bit 8). When in use
//! it changes the coding of INTRA macroblocks in three ways (§I.1):
//!
//! 1. **AC / DC prediction** from neighbouring INTRA blocks — see §I.3
//!    `Mode 0..2` and Figure I.3. The prediction direction is signalled
//!    per-MB via the 1-or-2-bit `INTRA_MODE` field (Table I.1).
//! 2. **Modified inverse quantisation** — every INTRA coefficient (DC and
//!    AC) is reconstructed without the H.263-baseline dead-zone, using
//!    `RecC(u,v) = 2 * QUANT * LEVEL(u,v)`. The INTRADC special-case
//!    (fixed step 8) is removed; DC uses the same formula.
//! 3. **Separate VLC table** for INTRA TCOEF — Table I.2. Codeword shapes
//!    are the same as the standard inter TCOEF (Table 16) but the
//!    `(LAST, RUN, |LEVEL|)` triples assigned to each codeword differ.
//!
//! When AIC is in use, INTRADC is no longer transmitted as a separate
//! 8-bit field. Instead every coefficient (DC at scan position 0 plus AC at
//! 1..=63) goes through the Table I.2 VLC, scanned in zig-zag (mode 0) /
//! alternate-vertical (mode 2) / alternate-horizontal (mode 1) order
//! depending on `INTRA_MODE`. CBPY/CBPC bits are repurposed to mean
//! "block has any coefficient transmitted" (§I.3 paragraph 4).
//!
//! This module is consumed by the encoder (when
//! [`crate::encoder::H263Encoder::set_enable_annex_i_aic`] is on) and by
//! the decoder (when the parsed PLUSPTYPE OPPTYPE bit 8 is set —
//! `PictureHeader::aic_mode`).

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_core::{Error, Result};

/// Per-MB INTRA prediction direction. Spec Table I.1 — 3 codewords:
///   0 (`0`)  — DC prediction only (no AC pred; zig-zag scan).
///   1 (`10`) — DC + AC from the block immediately above (alt-horizontal scan).
///   2 (`11`) — DC + AC from the block immediately to the left (alt-vertical scan).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntraMode {
    DcOnly = 0,
    Vertical = 1,
    Horizontal = 2,
}

impl IntraMode {
    /// Decode INTRA_MODE per Table I.1.
    pub fn read(br: &mut BitReader<'_>) -> Result<Self> {
        let b0 = br.read_u1()?;
        if b0 == 0 {
            return Ok(IntraMode::DcOnly);
        }
        let b1 = br.read_u1()?;
        if b1 == 0 {
            Ok(IntraMode::Vertical)
        } else {
            Ok(IntraMode::Horizontal)
        }
    }

    /// Emit INTRA_MODE per Table I.1.
    pub fn write(self, bw: &mut BitWriter) {
        match self {
            IntraMode::DcOnly => bw.write_bits(0, 1),
            IntraMode::Vertical => bw.write_bits(0b10, 2),
            IntraMode::Horizontal => bw.write_bits(0b11, 2),
        }
    }
}

/// Figure I.2-a — Alternate-Horizontal scan. Maps **scan index** (0..=63) to
/// the natural-order coefficient index.
///
/// Spec Figure I.2-a is laid out with scan position written into each cell;
/// here we invert the table so `ALT_HORIZONTAL_SCAN[k]` is the natural-order
/// `(row, col) -> row*8 + col` index of the coefficient that occupies scan
/// position `k`.
pub const ALT_HORIZONTAL_SCAN: [usize; 64] = {
    // Build by scanning Figure I.2-a's 8x8 grid for each scan index.
    // Spec table (1-indexed scan positions):
    //   col→0  1  2  3  4  5  6  7
    //   row 0: 1  2  3  4 11 12 13 14
    //   row 1: 5  6  9 10 18 17 16 15
    //   row 2: 7  8 20 19 27 28 29 30
    //   row 3: 21 22 25 26 31 32 33 34
    //   row 4: 23 24 35 36 43 44 45 46
    //   row 5: 37 38 41 42 47 48 49 50
    //   row 6: 39 40 51 52 57 58 59 60
    //   row 7: 53 54 55 56 61 62 63 64
    // Below: GRID[row][col] = spec scan position (1-based).
    const GRID: [[u8; 8]; 8] = [
        [1, 2, 3, 4, 11, 12, 13, 14],
        [5, 6, 9, 10, 18, 17, 16, 15],
        [7, 8, 20, 19, 27, 28, 29, 30],
        [21, 22, 25, 26, 31, 32, 33, 34],
        [23, 24, 35, 36, 43, 44, 45, 46],
        [37, 38, 41, 42, 47, 48, 49, 50],
        [39, 40, 51, 52, 57, 58, 59, 60],
        [53, 54, 55, 56, 61, 62, 63, 64],
    ];
    let mut out = [0usize; 64];
    let mut row = 0usize;
    while row < 8 {
        let mut col = 0usize;
        while col < 8 {
            let scan_pos = GRID[row][col] as usize - 1;
            out[scan_pos] = row * 8 + col;
            col += 1;
        }
        row += 1;
    }
    out
};

/// Figure I.2-b — Alternate-Vertical scan (same as MPEG-2 alternate scan).
pub const ALT_VERTICAL_SCAN: [usize; 64] = {
    // Spec table (1-indexed scan positions):
    //   row 0: 1  5  7 21 23 37 39 53
    //   row 1: 2  6  8 22 24 38 40 54
    //   row 2: 3  9 20 25 35 41 51 55
    //   row 3: 4 10 19 26 36 42 52 56
    //   row 4: 11 18 27 31 43 47 57 61
    //   row 5: 12 17 28 32 44 48 58 62
    //   row 6: 13 16 29 33 45 49 59 63
    //   row 7: 14 15 30 34 46 50 60 64
    const GRID: [[u8; 8]; 8] = [
        [1, 5, 7, 21, 23, 37, 39, 53],
        [2, 6, 8, 22, 24, 38, 40, 54],
        [3, 9, 20, 25, 35, 41, 51, 55],
        [4, 10, 19, 26, 36, 42, 52, 56],
        [11, 18, 27, 31, 43, 47, 57, 61],
        [12, 17, 28, 32, 44, 48, 58, 62],
        [13, 16, 29, 33, 45, 49, 59, 63],
        [14, 15, 30, 34, 46, 50, 60, 64],
    ];
    let mut out = [0usize; 64];
    let mut row = 0usize;
    while row < 8 {
        let mut col = 0usize;
        while col < 8 {
            let scan_pos = GRID[row][col] as usize - 1;
            out[scan_pos] = row * 8 + col;
            col += 1;
        }
        row += 1;
    }
    out
};

/// Pick the scan order for an AIC INTRA block based on the per-MB INTRA mode.
///
/// * Mode 0 (DC only) → standard zig-zag (same as Figure 14 in main spec).
/// * Mode 1 (vertical pred) → alternate-horizontal scan (Figure I.2-a).
/// * Mode 2 (horizontal pred) → alternate-vertical scan (Figure I.2-b).
pub fn scan_for(mode: IntraMode) -> &'static [usize; 64] {
    use oxideav_mpeg4video::headers::vol::ZIGZAG;
    match mode {
        IntraMode::DcOnly => &ZIGZAG,
        IntraMode::Vertical => &ALT_HORIZONTAL_SCAN,
        IntraMode::Horizontal => &ALT_VERTICAL_SCAN,
    }
}

// ---------------------------------------------------------------------------
// Table I.2 — INTRA TCOEF
//
// Per spec §I.3, codewords are the same shape as the inter TCOEF table
// (Table 16 / oxideav-mpeg4video::tables::tcoef::INTER_*) but they map to
// different `(LAST, RUN, |LEVEL|)` triples.
//
// We transcribe the table directly from the spec (lines 4039..=4156) below,
// keeping the same row layout as the inter table — `LAST=0` rows first
// (indices 0..=57), then `LAST=1` rows (indices 58..=101), then ESCAPE
// (index 102) which uses the same 7-bit `0000011` prefix as the inter VLC.
// ---------------------------------------------------------------------------

/// Encode-side: `(bits_without_sign, code)` for each LAST=0 entry of Table
/// I.2 (rows 0..=57). The encoder writes the magnitude codeword (`bits`
/// bits, MSB-first) then a 1-bit sign suffix `s`. Spec column "Bits"
/// counts the magnitude *plus* the sign — so `bits = spec_Bits - 1`.
#[rustfmt::skip]
const INTRA_LAST0_VLC: [(u8, u32); 58] = [
    ( 2, 0b10),
    ( 4, 0b1111),
    ( 6, 0b010101),
    ( 7, 0b0010111),
    ( 8, 0b00011111),
    ( 9, 0b000100101),
    ( 9, 0b000100100),
    (10, 0b0000100001),
    (10, 0b0000100000),
    (11, 0b00000000111),
    (11, 0b00000000110),
    (11, 0b00000100000),
    ( 3, 0b110),
    ( 6, 0b010100),
    ( 8, 0b00011110),
    (10, 0b0000001111),
    (11, 0b00000100001),
    (12, 0b000001010000),
    ( 4, 0b1110),
    ( 8, 0b00011101),
    (10, 0b0000001110),
    (12, 0b000001010001),
    ( 5, 0b01101),
    ( 9, 0b000100011),
    (10, 0b0000001101),
    ( 5, 0b01100),
    ( 9, 0b000100010),
    (12, 0b000001010010),
    ( 5, 0b01011),
    (10, 0b0000001100),
    (12, 0b000001010011),
    ( 6, 0b010011),
    (10, 0b0000001011),
    (12, 0b000001010100),
    ( 6, 0b010010),
    (10, 0b0000001010),
    ( 6, 0b010001),
    (10, 0b0000001001),
    ( 6, 0b010000),
    (10, 0b0000001000),
    ( 7, 0b0010110),
    (12, 0b000001010101),
    ( 7, 0b0010101),
    ( 7, 0b0010100),
    ( 8, 0b00011100),
    ( 8, 0b00011011),
    ( 9, 0b000100001),
    ( 9, 0b000100000),
    ( 9, 0b000011111),
    ( 9, 0b000011110),
    ( 9, 0b000011101),
    ( 9, 0b000011100),
    ( 9, 0b000011011),
    ( 9, 0b000011010),
    (11, 0b00000100010),
    (11, 0b00000100011),
    (12, 0b000001010110),
    (12, 0b000001010111),
];

const INTRA_LAST0_RUN: [u8; 58] = [
    // i=0..=11
    0, 1, 3, 5, 7, 8, 9, 10, 11, 4, 9, 13, // i=12..=17
    0, 1, 1, 1, 1, 1, // i=18..=21
    0, 3, 2, 3, // i=22..=24
    0, 4, 3, // i=25..=27
    0, 5, 5, // i=28..=30
    2, 6, 0, // i=31..=33
    4, 7, 0, // i=34..=36
    0, 8, 0, // i=37..=39
    2, 0, 12, // i=40..=41
    0, 0, // i=42..=45
    2, 1, 6, 0, // i=46..=52
    0, 0, 0, 0, 0, 0, 0, // i=53..=57
    0, 0, 0, 0, 0,
];

const INTRA_LAST0_LEVEL: [u8; 58] = [
    1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 2, 1, 2, 2, 4, 5, 6, 7, 3, 2, 3, 4, 5, 2, 3, 4, 2, 3, 1, 2, 25,
    1, 2, 24, 8, 2, 7, 4, 6, 1, 9, 23, 2, 3, 1, 10, 12, 11, 18, 17, 16, 15, 14, 13, 20, 19, 22, 21,
];

#[rustfmt::skip]
const INTRA_LAST1_VLC: [(u8, u32); 44] = [
    ( 4, 0b0111),
    ( 9, 0b000011001),
    (11, 0b00000000101),
    ( 6, 0b001111),
    (11, 0b00000000100),
    ( 6, 0b001110),
    ( 6, 0b001101),
    ( 6, 0b001100),
    ( 7, 0b0010011),
    ( 7, 0b0010010),
    ( 7, 0b0010001),
    ( 7, 0b0010000),
    ( 8, 0b00011010),
    ( 8, 0b00011001),
    ( 8, 0b00011000),
    ( 8, 0b00010111),
    ( 8, 0b00010110),
    ( 8, 0b00010101),
    ( 8, 0b00010100),
    ( 8, 0b00010011),
    ( 9, 0b000011000),
    ( 9, 0b000010111),
    ( 9, 0b000010110),
    ( 9, 0b000010101),
    ( 9, 0b000010100),
    ( 9, 0b000010011),
    ( 9, 0b000010010),
    ( 9, 0b000010001),
    (10, 0b0000000111),
    (10, 0b0000000110),
    (10, 0b0000000101),
    (10, 0b0000000100),
    (11, 0b00000100100),
    (11, 0b00000100101),
    (11, 0b00000100110),
    (11, 0b00000100111),
    (12, 0b000001011000),
    (12, 0b000001011001),
    (12, 0b000001011010),
    (12, 0b000001011011),
    (12, 0b000001011100),
    (12, 0b000001011101),
    (12, 0b000001011110),
    (12, 0b000001011111),
];

const INTRA_LAST1_RUN: [u8; 44] = [
    0, 14, 20, 1, 19, 2, 3, 0, 5, 6, 4, 0, 9, 10, 11, 12, 13, 8, 7, 0, 17, 18, 16, 15, 2, 1, 0, 0,
    4, 3, 1, 0, 2, 1, 0, 0, 21, 22, 23, 7, 6, 5, 3, 0,
];

const INTRA_LAST1_LEVEL: [u8; 44] = [
    1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 3, 1, 1, 1, 1, 1, 1, 1, 4, 1, 1, 1, 1, 2, 2, 6, 5, 2, 2, 3, 7,
    3, 4, 9, 8, 1, 1, 1, 2, 2, 2, 3, 10,
];

/// Look up `(bits, code)` for an INTRA TCOEF triple. Returns `None` for out-of-
/// table tuples — the caller falls back to the H.263 ESCAPE code (`0000011`,
/// 7 bits) followed by `last(1) + run(6) + level(8 signed)`. The escape body
/// is identical to the standard inter VLC's escape (§5.4.2.5).
pub fn lookup_intra_tcoef(last: bool, run: u8, level_abs: u8) -> Option<(u8, u32)> {
    let (vlc, runs, levels): (&[(u8, u32)], &[u8], &[u8]) = if last {
        (&INTRA_LAST1_VLC, &INTRA_LAST1_RUN, &INTRA_LAST1_LEVEL)
    } else {
        (&INTRA_LAST0_VLC, &INTRA_LAST0_RUN, &INTRA_LAST0_LEVEL)
    };
    for i in 0..vlc.len() {
        if runs[i] == run && levels[i] == level_abs {
            return Some(vlc[i]);
        }
    }
    None
}

/// Encode one `(last, run, level)` triple under Table I.2. Mirrors
/// [`crate::enc_tables::write_tcoef`] but uses the INTRA codebook.
pub fn write_intra_tcoef(bw: &mut BitWriter, last: bool, run: u8, level: i32) {
    debug_assert!(level != 0);
    debug_assert!(run < 64);
    let abs = level.unsigned_abs();
    let sign = if level < 0 { 1 } else { 0 };
    if abs <= 255 {
        if let Some((bits, code)) = lookup_intra_tcoef(last, run, abs as u8) {
            bw.write_bits(code, bits as u32);
            bw.write_bits(sign, 1);
            return;
        }
    }
    // Escape: 0000011 (7 bits) + last(1) + run(6) + level(8 signed). Same as
    // inter — §I.3 says only the codeword/triple mapping changes.
    bw.write_bits(0b0000011, 7);
    bw.write_bits(last as u32, 1);
    bw.write_bits(run as u32 & 0x3F, 6);
    let level_byte = level & 0xFF;
    debug_assert!(level_byte != 0 && level_byte != 0x80);
    bw.write_bits(level_byte as u32, 8);
}

/// Decoder-side INTRA TCOEF symbol. Same shape as the inter table's symbol —
/// either a `(last, run, |level|)` triple (sign bit follows on the wire) or
/// an `Escape` marker (the caller reads `last(1) + run(6) + level(8 signed)`
/// after the 7-bit prefix).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntraTcoefSym {
    RunLevel { last: bool, run: u8, level_abs: u8 },
    Escape,
}

/// Decode one Table I.2 codeword. Reads bits one at a time and matches them
/// against the table — small enough table that linear scan + a depth-bounded
/// bit accumulator is fine. We bail with `Error::invalid` on any prefix that
/// doesn't resolve within 13 bits (the table's max codeword length).
pub fn decode_intra_tcoef(br: &mut BitReader<'_>) -> Result<IntraTcoefSym> {
    // The escape prefix `0000011` must be detected separately — its 7-bit
    // prefix coincides with longer table codewords if checked in pure
    // longest-match-first order (specifically several 13-bit `LAST=0` entries
    // start with `00000`). We accumulate up to 13 bits and prefer the
    // longest exact table match; the 7-bit escape only fires when no longer
    // codeword matches.
    let mut acc: u32 = 0;
    let mut nbits: u8 = 0;
    let mut best_match: Option<IntraTcoefSym> = None;
    let mut best_len: u8 = 0;
    while nbits < 13 {
        acc = (acc << 1) | br.read_u1()?;
        nbits += 1;
        // Exact-match scan for this prefix length. Only one entry per
        // codeword length per (last, code) so the first hit is unique.
        for last_flag in [false, true] {
            let (vlc, runs, levels): (&[(u8, u32)], &[u8], &[u8]) = if last_flag {
                (&INTRA_LAST1_VLC, &INTRA_LAST1_RUN, &INTRA_LAST1_LEVEL)
            } else {
                (&INTRA_LAST0_VLC, &INTRA_LAST0_RUN, &INTRA_LAST0_LEVEL)
            };
            for i in 0..vlc.len() {
                let (bits, code) = vlc[i];
                if bits == nbits && code == acc {
                    best_match = Some(IntraTcoefSym::RunLevel {
                        last: last_flag,
                        run: runs[i],
                        level_abs: levels[i],
                    });
                    best_len = bits;
                }
            }
        }
        // Escape prefix: `0000011` is exactly 7 bits.
        if nbits == 7 && acc == 0b0000011 && best_match.is_none() {
            // No table entry shares the exact 7-bit code `0000011`; emit
            // ESCAPE without reading further bits. (Caller reads the
            // 1+6+8 escape body itself.)
            return Ok(IntraTcoefSym::Escape);
        }
        // Once we have a match we still need to keep scanning longer
        // codewords because some prefixes in this table are themselves
        // valid codewords (e.g. `10` matches both the 2-bit prefix of
        // `0010100` and the 3-bit codeword `10s` for index 0). Standard
        // VLC decoding picks the longest prefix that exactly matches, so
        // we *commit* the shorter match only when no longer match exists.
        // Because `read_u1` advances the cursor we cannot back up if a
        // longer probe fails — instead we pre-decide by scanning the
        // table once for any longer codewords that begin with the current
        // accumulator. If none do, the current shortest match is final.
        if let Some(sym) = best_match {
            if !any_longer_codeword_with_prefix(acc, nbits) {
                let _ = best_len;
                return Ok(sym);
            }
        }
    }
    Err(Error::invalid(format!(
        "h263 Annex I INTRA TCOEF: 13-bit prefix {acc:013b} did not match any codeword"
    )))
}

/// True iff there exists any Table I.2 codeword strictly longer than `nbits`
/// whose top `nbits` bits equal `acc`. Used by [`decode_intra_tcoef`] to
/// decide whether to commit the current match or wait for a longer one.
fn any_longer_codeword_with_prefix(acc: u32, nbits: u8) -> bool {
    for table in [&INTRA_LAST0_VLC[..], &INTRA_LAST1_VLC[..]] {
        for &(bits, code) in table {
            if bits > nbits {
                let shift = bits - nbits;
                if (code >> shift) == acc {
                    return true;
                }
            }
        }
    }
    false
}

// ---------------------------------------------------------------------------
// AIC quantisation
// ---------------------------------------------------------------------------

/// AIC INTRA dequantisation. Per §I.3:
///   `RecC(u,v) = 2 * QUANT * LEVEL(u,v)`, no dead-zone, applied to every
///   coefficient (DC and AC).
///
/// `levels` are signed quantised coefficients in **natural** order (the AC
/// scan already de-zig-zagged for the caller). The DC entry at index 0 uses
/// the same formula. Output is clipped to the spec range [-2048, 2047]
/// pending the AC-pred / oddify stage.
pub fn dequantise_intra_block_aic(levels: &[i32; 64], quant: u8) -> [i32; 64] {
    let q = quant as i32;
    let mut out = [0i32; 64];
    for k in 0..64 {
        let l = levels[k];
        if l == 0 {
            continue;
        }
        let v = 2 * q * l;
        out[k] = v.clamp(-2048, 2047);
    }
    out
}

/// Quantise an INTRA block under AIC. Mirrors the encoder's deadzone-bias
/// quantiser but uses `2 * QUANT` as the quantisation step for every
/// coefficient (DC and AC). Returns `(levels_in_natural_order, any_nonzero)`.
///
/// Caller is responsible for subtracting the AC-pred predictor from the DCT
/// coefficients **before** calling this function (so what we quantise is the
/// residual relative to the predictor).
///
/// Bias choice: since AIC has no dead-zone in the dequantiser
/// (`RecC = 2*Q*LEVEL`), the encoder uses round-to-nearest
/// `mag = (|coef| + Q) / (2*Q)` for AC coefficients to minimise bitrate
/// (smaller `|level|` → shorter Table I.2 codeword) without bias-induced
/// reconstruction drift. The DC slot at index 0 uses a slightly larger bias
/// so DC residuals near zero round to zero (the §I.3 AC-pred predictor
/// already absorbs most of the DC magnitude — what's left is high-frequency
/// signal).
pub fn quantise_intra_block_aic(dctf: &[f32; 64], quant: u8) -> ([i32; 64], bool) {
    let q = quant as i32;
    let two_q = 2 * q;
    let bias_ac = q / 2;
    let bias_dc = q / 2;
    let mut levels = [0i32; 64];
    let mut any = false;
    for k in 0..64 {
        let coef = dctf[k];
        let abs_f = coef.abs() as i32;
        let bias = if k == 0 { bias_dc } else { bias_ac };
        let mag = (abs_f + bias) / two_q;
        if mag != 0 {
            let signed = if coef < 0.0 { -mag } else { mag };
            levels[k] = signed.clamp(-127, 127);
            any = true;
        }
    }
    (levels, any)
}

// ---------------------------------------------------------------------------
// AC prediction (§I.3 — Mode 0/1/2)
// ---------------------------------------------------------------------------

/// Per-block AIC reconstruction state. We hold the **final** reconstructed
/// coefficient values (post-AC-pred, pre-IDCT) for block A (above) and block
/// B (left) of every spatial neighbour; this is the `RecA'` / `RecB'` state
/// needed by the §I.3 reconstruction equations on the next block.
///
/// Indexed as `[mb_y][mb_x][block_idx]` where `block_idx` is the standard
/// H.263 block numbering (0..=3 luma in raster order, 4 = Cb, 5 = Cr). Only
/// `coeffs[0..=7]` (the first row) and `coeffs[0], [8], [16], ..., [56]`
/// (the first column) plus the DC are consulted by the predictor; we store
/// the full 8x8 anyway because reading row/column slices on demand from a
/// single shared buffer keeps the bookkeeping simple.
#[derive(Clone)]
pub struct AicNeighbourCache {
    pub mb_w: usize,
    pub mb_h: usize,
    /// Per-block flat storage. `coeffs[idx_for(mb_x, mb_y, block_idx) ..
    /// idx_for(...) + 64]` is the 8x8 reconstructed coefficient block.
    coeffs: Vec<i16>,
    /// `is_intra[mb_y * mb_w * 6 + mb_x * 6 + block_idx]` — `true` iff that
    /// block was coded INTRA in the most recent picture. The §I.3 rules
    /// reference this when picking the predictor.
    is_intra: Vec<bool>,
}

impl AicNeighbourCache {
    pub fn new(mb_w: usize, mb_h: usize) -> Self {
        Self {
            mb_w,
            mb_h,
            coeffs: vec![0i16; mb_w * mb_h * 6 * 64],
            is_intra: vec![false; mb_w * mb_h * 6],
        }
    }

    fn idx(&self, mb_x: usize, mb_y: usize, block_idx: usize) -> usize {
        ((mb_y * self.mb_w) + mb_x) * 6 + block_idx
    }

    /// Stash the reconstructed-DCT-domain coefficients for block
    /// `(mb_x, mb_y, block_idx)`. `coeffs` are the **final** RecC' values
    /// (after AC-pred + oddify + clip) in natural order.
    pub fn store(&mut self, mb_x: usize, mb_y: usize, block_idx: usize, coeffs: &[i32; 64]) {
        let base = self.idx(mb_x, mb_y, block_idx) * 64;
        for k in 0..64 {
            self.coeffs[base + k] = coeffs[k].clamp(-32768, 32767) as i16;
        }
        let flag_idx = self.idx(mb_x, mb_y, block_idx);
        self.is_intra[flag_idx] = true;
    }

    /// Mark a block as non-INTRA (or "outside the same video segment") for
    /// the purposes of AC-pred lookups. Used by the encoder/decoder when
    /// the MB is coded inter (P-picture intra-in-P with AIC off — currently
    /// not supported) or when a GOB boundary crossed between this block
    /// and its neighbours.
    pub fn mark_non_intra(&mut self, mb_x: usize, mb_y: usize, block_idx: usize) {
        let flag_idx = self.idx(mb_x, mb_y, block_idx);
        self.is_intra[flag_idx] = false;
    }

    /// Borrow the reconstructed coefficient block (read-only).
    pub fn get(&self, mb_x: usize, mb_y: usize, block_idx: usize) -> Option<&[i16]> {
        if !self.is_intra[self.idx(mb_x, mb_y, block_idx)] {
            return None;
        }
        let base = self.idx(mb_x, mb_y, block_idx) * 64;
        Some(&self.coeffs[base..base + 64])
    }
}

/// Resolve the (mb, block) coordinate of the neighbour block ABOVE the
/// current block within its luma/chroma component, per §I.3 / Figure I.3.
///
/// Returns `None` when the neighbour falls outside the picture.
///
/// Block layout inside an MB (Figure 5 / §5.3.2):
///   * Luma: 0=TL, 1=TR, 2=BL, 3=BR.
///   * Cb=4, Cr=5 (single 8x8 each).
///
/// Above-neighbour mapping:
///   * Luma block 0 → block 2 of MB (mb_x, mb_y - 1) (or None at top edge).
///   * Luma block 1 → block 3 of MB (mb_x, mb_y - 1).
///   * Luma block 2 → block 0 of MB (mb_x, mb_y) (same MB).
///   * Luma block 3 → block 1 of MB (mb_x, mb_y).
///   * Cb (block 4) → Cb of MB (mb_x, mb_y - 1).
///   * Cr (block 5) → Cr of MB (mb_x, mb_y - 1).
pub fn above_neighbour(
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
) -> Option<(usize, usize, usize)> {
    match block_idx {
        0 => {
            if mb_y == 0 {
                None
            } else {
                Some((mb_x, mb_y - 1, 2))
            }
        }
        1 => {
            if mb_y == 0 {
                None
            } else {
                Some((mb_x, mb_y - 1, 3))
            }
        }
        2 => Some((mb_x, mb_y, 0)),
        3 => Some((mb_x, mb_y, 1)),
        4 | 5 => {
            if mb_y == 0 {
                None
            } else {
                Some((mb_x, mb_y - 1, block_idx))
            }
        }
        _ => None,
    }
}

/// Resolve the (mb, block) coordinate of the neighbour block immediately
/// to the LEFT of the current block.
///
/// Left-neighbour mapping:
///   * Luma block 0 → block 1 of MB (mb_x - 1, mb_y).
///   * Luma block 1 → block 0 of MB (mb_x, mb_y).
///   * Luma block 2 → block 3 of MB (mb_x - 1, mb_y).
///   * Luma block 3 → block 2 of MB (mb_x, mb_y).
///   * Cb (block 4) → Cb of MB (mb_x - 1, mb_y).
///   * Cr (block 5) → Cr of MB (mb_x - 1, mb_y).
pub fn left_neighbour(mb_x: usize, mb_y: usize, block_idx: usize) -> Option<(usize, usize, usize)> {
    match block_idx {
        0 => {
            if mb_x == 0 {
                None
            } else {
                Some((mb_x - 1, mb_y, 1))
            }
        }
        1 => Some((mb_x, mb_y, 0)),
        2 => {
            if mb_x == 0 {
                None
            } else {
                Some((mb_x - 1, mb_y, 3))
            }
        }
        3 => Some((mb_x, mb_y, 2)),
        4 | 5 => {
            if mb_x == 0 {
                None
            } else {
                Some((mb_x - 1, mb_y, block_idx))
            }
        }
        _ => None,
    }
}

/// Apply §I.3 AC prediction reconstruction in-place on `coeffs` (which
/// holds the dequantised residual values RecC) to produce the final
/// RecC' coefficients (pre-IDCT).
///
/// Returns the final RecC' block. The caller is expected to feed this into
/// `idct_signed` / `idct_and_clip` to produce pels, then store the same
/// RecC' into the [`AicNeighbourCache`] for future neighbours to read.
pub fn apply_ac_prediction(
    mode: IntraMode,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    cache: &AicNeighbourCache,
    rec_c: &[i32; 64],
) -> [i32; 64] {
    let mut out = [0i32; 64];
    // Copy through (RecC' = clipAC(RecC) for all u,v that aren't predicted).
    for k in 0..64 {
        out[k] = rec_c[k].clamp(-2048, 2047);
    }

    let above_block =
        above_neighbour(mb_x, mb_y, block_idx).and_then(|(ax, ay, bi)| cache.get(ax, ay, bi));
    let left_block =
        left_neighbour(mb_x, mb_y, block_idx).and_then(|(lx, ly, bi)| cache.get(lx, ly, bi));

    let temp_dc = match mode {
        IntraMode::DcOnly => match (above_block, left_block) {
            (Some(a), Some(b)) => rec_c[0] + ((a[0] as i32 + b[0] as i32) / 2),
            (Some(a), None) => rec_c[0] + a[0] as i32,
            (None, Some(b)) => rec_c[0] + b[0] as i32,
            (None, None) => rec_c[0] + 1024,
        },
        IntraMode::Vertical => {
            // Predict DC + first row from the block ABOVE.
            if let Some(a) = above_block {
                let dc = rec_c[0] + a[0] as i32;
                // First row (u = 1..=7, v = 0).
                for u in 1..8 {
                    let v = rec_c[u] + a[u] as i32;
                    out[u] = v.clamp(-2048, 2047);
                }
                // Remaining rows (v = 1..=7) untouched (already copied through).
                dc
            } else {
                rec_c[0] + 1024
            }
        }
        IntraMode::Horizontal => {
            // Predict DC + first column from the block to the LEFT.
            if let Some(b) = left_block {
                let dc = rec_c[0] + b[0] as i32;
                for v in 1..8 {
                    let nat = v * 8;
                    let val = rec_c[nat] + b[nat] as i32;
                    out[nat] = val.clamp(-2048, 2047);
                }
                dc
            } else {
                rec_c[0] + 1024
            }
        }
    };

    // §I.3 — oddifyclipDC: clip to [0, 2047], then if even add 1.
    out[0] = oddify_clip_dc(temp_dc);
    out
}

/// `oddifyclipDC(x)` — clip x to [0, 2047], then if the result is even, add 1.
/// (§I.3 — IDCT mismatch mitigation.)
pub fn oddify_clip_dc(x: i32) -> i32 {
    let clipped = x.clamp(0, 2047);
    if clipped & 1 == 0 {
        (clipped + 1).min(2047)
    } else {
        clipped
    }
}

/// Inverse of [`apply_ac_prediction`] — given the **final** reconstructed
/// coefficients `rec_c_prime` that the encoder wants to land at, and the
/// neighbour cache, return the **transmitted** RecC residual that should be
/// quantised + entropy-coded. Used by the encoder so the decoder's
/// AC-pred path lands on `rec_c_prime`.
///
/// Note: we cannot recover the encoder's intent perfectly from a single
/// `rec_c_prime` because of `oddify_clip_dc`; instead the encoder
/// computes its raw target (2*q*level), subtracts the predictor, quantises,
/// then stashes the **decoder-side** RecC' back into the neighbour cache.
/// This helper exists to compute the predictor that should be subtracted.
pub fn ac_pred_predictor_for(
    mode: IntraMode,
    mb_x: usize,
    mb_y: usize,
    block_idx: usize,
    cache: &AicNeighbourCache,
) -> [i32; 64] {
    let mut pred = [0i32; 64];
    let above_block =
        above_neighbour(mb_x, mb_y, block_idx).and_then(|(ax, ay, bi)| cache.get(ax, ay, bi));
    let left_block =
        left_neighbour(mb_x, mb_y, block_idx).and_then(|(lx, ly, bi)| cache.get(lx, ly, bi));

    match mode {
        IntraMode::DcOnly => {
            pred[0] = match (above_block, left_block) {
                (Some(a), Some(b)) => (a[0] as i32 + b[0] as i32) / 2,
                (Some(a), None) => a[0] as i32,
                (None, Some(b)) => b[0] as i32,
                (None, None) => 1024,
            };
        }
        IntraMode::Vertical => {
            if let Some(a) = above_block {
                pred[0] = a[0] as i32;
                for u in 1..8 {
                    pred[u] = a[u] as i32;
                }
            } else {
                pred[0] = 1024;
            }
        }
        IntraMode::Horizontal => {
            if let Some(b) = left_block {
                pred[0] = b[0] as i32;
                for v in 1..8 {
                    let nat = v * 8;
                    pred[nat] = b[nat] as i32;
                }
            } else {
                pred[0] = 1024;
            }
        }
    }
    pred
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intra_mode_round_trip() {
        for mode in [
            IntraMode::DcOnly,
            IntraMode::Vertical,
            IntraMode::Horizontal,
        ] {
            let mut bw = BitWriter::new();
            mode.write(&mut bw);
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            let m = IntraMode::read(&mut br).unwrap();
            assert_eq!(m, mode);
        }
    }

    #[test]
    fn alt_horizontal_scan_is_a_permutation() {
        let mut seen = [false; 64];
        for &k in ALT_HORIZONTAL_SCAN.iter() {
            assert!(!seen[k], "duplicate scan index {k}");
            seen[k] = true;
        }
    }

    #[test]
    fn alt_vertical_scan_is_a_permutation() {
        let mut seen = [false; 64];
        for &k in ALT_VERTICAL_SCAN.iter() {
            assert!(!seen[k], "duplicate scan index {k}");
            seen[k] = true;
        }
    }

    #[test]
    fn alt_horizontal_scan_starts_top_left() {
        // Spec Figure I.2-a: scan position 1 → (0,0); 2 → (0,1); 3 → (0,2);
        // 4 → (0,3); 5 → (1,0); 6 → (1,1); ...
        assert_eq!(ALT_HORIZONTAL_SCAN[0], 0); // (0,0)
        assert_eq!(ALT_HORIZONTAL_SCAN[1], 1); // (0,1)
        assert_eq!(ALT_HORIZONTAL_SCAN[2], 2); // (0,2)
        assert_eq!(ALT_HORIZONTAL_SCAN[3], 3); // (0,3)
        assert_eq!(ALT_HORIZONTAL_SCAN[4], 8); // (1,0)
    }

    #[test]
    fn alt_vertical_scan_starts_top_left() {
        // Spec Figure I.2-b: scan position 1 → (0,0); 2 → (1,0); 3 → (2,0); ...
        assert_eq!(ALT_VERTICAL_SCAN[0], 0); // (0,0)
        assert_eq!(ALT_VERTICAL_SCAN[1], 8); // (1,0)
        assert_eq!(ALT_VERTICAL_SCAN[2], 16); // (2,0)
    }

    /// Round-trip every Table I.2 entry through write_intra_tcoef +
    /// decode_intra_tcoef.
    #[test]
    fn intra_tcoef_full_round_trip() {
        for last_flag in [false, true] {
            let (vlc, runs, levels): (&[(u8, u32)], &[u8], &[u8]) = if last_flag {
                (&INTRA_LAST1_VLC, &INTRA_LAST1_RUN, &INTRA_LAST1_LEVEL)
            } else {
                (&INTRA_LAST0_VLC, &INTRA_LAST0_RUN, &INTRA_LAST0_LEVEL)
            };
            for i in 0..vlc.len() {
                for sign in [1, -1] {
                    let mut bw = BitWriter::new();
                    let level = sign * levels[i] as i32;
                    write_intra_tcoef(&mut bw, last_flag, runs[i], level);
                    let bytes = bw.finish();
                    let mut br = BitReader::new(&bytes);
                    let sym = decode_intra_tcoef(&mut br).unwrap();
                    if let IntraTcoefSym::RunLevel {
                        last,
                        run,
                        level_abs,
                    } = sym
                    {
                        let s = br.read_u1().unwrap();
                        let decoded = if s == 1 {
                            -(level_abs as i32)
                        } else {
                            level_abs as i32
                        };
                        assert_eq!(last, last_flag, "last mismatch at i={i}");
                        assert_eq!(run, runs[i], "run mismatch at i={i}");
                        assert_eq!(decoded, level, "level mismatch at i={i}");
                    } else {
                        panic!("expected RunLevel for table entry i={i}");
                    }
                }
            }
        }
    }

    #[test]
    fn intra_tcoef_escape_round_trip() {
        // Out-of-table tuple (last=false, run=0, level=100) → escape.
        let mut bw = BitWriter::new();
        write_intra_tcoef(&mut bw, false, 0, 100);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let sym = decode_intra_tcoef(&mut br).unwrap();
        assert!(matches!(sym, IntraTcoefSym::Escape));
        let last = br.read_u1().unwrap();
        let run = br.read_u32(6).unwrap();
        let lvl = br.read_u32(8).unwrap();
        assert_eq!(last, 0);
        assert_eq!(run, 0);
        assert_eq!(lvl, 100);
    }

    #[test]
    fn dc_only_no_neighbour_uses_1024() {
        let cache = AicNeighbourCache::new(2, 2);
        let mut rec_c = [0i32; 64];
        rec_c[0] = 0; // pure-zero residual
        let out = apply_ac_prediction(IntraMode::DcOnly, 0, 0, 0, &cache, &rec_c);
        // tempDC = 0 + 1024 = 1024 → oddify(1024) = 1025 (1024 is even).
        assert_eq!(out[0], 1025);
    }

    #[test]
    fn vertical_pred_first_row_is_added_back() {
        let mut cache = AicNeighbourCache::new(1, 2);
        // Block (0, 0, 2) is the bottom-left luma of the top MB — the "above"
        // neighbour of block 0 in MB (0, 1). Stash some reconstructed data.
        let mut above = [0i32; 64];
        above[0] = 800;
        for u in 1..8 {
            above[u] = (u as i32) * 10;
        }
        cache.store(0, 0, 2, &above);

        let mut rec_c = [0i32; 64];
        rec_c[0] = 0; // residual DC = 0
        for u in 1..8 {
            rec_c[u] = (u as i32) * (-3); // residual = -3, -6, -9, ...
        }
        let out = apply_ac_prediction(IntraMode::Vertical, 0, 1, 0, &cache, &rec_c);
        assert_eq!(out[0], oddify_clip_dc(800)); // 801 (already odd)
        for u in 1..8 {
            let want = (u as i32) * (-3) + (u as i32) * 10;
            assert_eq!(out[u], want, "row coefficient u={u}");
        }
        // Other coefficients should be unaffected (zero in, zero out).
        for v in 1..8 {
            for u in 0..8 {
                assert_eq!(out[v * 8 + u], 0, "out[{v},{u}] should be zero");
            }
        }
    }

    #[test]
    fn horizontal_pred_first_col_is_added_back() {
        let mut cache = AicNeighbourCache::new(2, 1);
        let mut left = [0i32; 64];
        left[0] = 800;
        for v in 1..8 {
            left[v * 8] = (v as i32) * 10;
        }
        cache.store(0, 0, 1, &left);

        let mut rec_c = [0i32; 64];
        rec_c[0] = 0;
        for v in 1..8 {
            rec_c[v * 8] = (v as i32) * (-3);
        }
        let out = apply_ac_prediction(IntraMode::Horizontal, 1, 0, 0, &cache, &rec_c);
        assert_eq!(out[0], oddify_clip_dc(800));
        for v in 1..8 {
            let want = (v as i32) * (-3) + (v as i32) * 10;
            assert_eq!(out[v * 8], want, "col coefficient v={v}");
        }
    }

    #[test]
    fn ac_pred_then_predictor_cancels_out() {
        // If the encoder computes pred from cache, subtracts pred from the
        // raw target rec_c_prime, then quantises 2*q*level and dequantises,
        // the decoder's `apply_ac_prediction` should land on rec_c_prime
        // (modulo `oddify_clip_dc` for DC).
        let mut cache = AicNeighbourCache::new(2, 1);
        let mut left = [0i32; 64];
        left[0] = 200;
        left[8] = 50;
        cache.store(0, 0, 1, &left);

        let mode = IntraMode::Horizontal;
        let pred = ac_pred_predictor_for(mode, 1, 0, 0, &cache);
        // Want final RecC' to be all zeros + DC = 51 (odd) — so transmitted
        // residual on DC should be 51 - 200 = -149; on (v=1,u=0) it should
        // be 0 - 50 = -50. Build a synthetic dequantised rec_c.
        let mut rec_c = [0i32; 64];
        rec_c[0] = 51 - pred[0]; // -149
        rec_c[8] = -pred[8]; // -50
        let out = apply_ac_prediction(mode, 1, 0, 0, &cache, &rec_c);
        assert_eq!(out[0], oddify_clip_dc(51));
        assert_eq!(out[8], 0);
    }
}
