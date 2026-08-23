//! Annex W §W.5.3 — Reference fixed-point IDCT 0 (and the informative
//! companion FDCT).
//!
//! §W.5 lets a bitstream announce — through the Annex L PSUPP
//! Fixed-Point IDCT function ([`crate::annex_l::SeiFunction::FixedPointIdct`]
//! with parameter `0`) — that it was constructed against one exact
//! integer IDCT: "the reference IDCT 0 is any implementation that,
//! for every input block, produces identical output values as the C
//! source program listed" in §W.5.3. A decoder that reconstructs with
//! [`idct_w0`] therefore matches such an encoder **bit-exactly**, and
//! §W.5.2 removes the clause-4.4 forced-updating requirement (INTRA
//! refresh every 132 codings exists only to bound IDCT-mismatch
//! drift, which bit-exact transforms do not accumulate).
//!
//! This module is a statement-for-statement transcription of the
//! §W.5.3 listing: 16-bit signed storage (`REGISTER`), 32-bit signed
//! intermediates (`LONG`, per the listing's own width comments), the
//! saturating `Multiply` / `Rotate` / `Round` primitives with their
//! exact `0x7FFF` rounding-bias behaviour, the two-pass butterfly
//! with its `pass`-dependent shifts, and the final
//! `HalfSwap`/`Transpose`/`HalfSwap` output permutation. The
//! informative FDCT listing (input 9-bit signed, output
//! `[-2048, 2047]`) is transcribed as [`fdct_w0`] so an encoder can
//! target reference-IDCT-0 decoders; §W.5.3's NOTE — the transform is
//! Annex A compliant — is pinned by tests against the crate's f64
//! reference kernel.
//!
//! Layout: both entry points take the 8×8 block in row-major natural
//! order (`block[8 * y + x]`); [`idct_w0`] takes dequantised
//! coefficients (12-bit signed input range) and leaves clipped spatial
//! values in `[-256, 255]`; [`fdct_w0`] takes 9-bit signed spatial
//! values (e.g. residuals) and leaves coefficients in
//! `[-2048, 2047]`.

/// `32768·cos(π/8)·1/√2` (§W.5.3 constant table).
const CPO8: i16 = 0x539f;
/// `32768·sin(π/8)·√2`.
const SPO8: i16 = 0x4546;
/// `32768·cos(π/16)`.
const CPO16: i16 = 0x7d8a;
/// `32768·sin(π/16)`.
const SPO16: i16 = 0x18f9;
/// `32768·cos(3π/16)`.
const C3PO16: i16 = 0x6a6e;
/// `32768·sin(3π/16)`.
const S3PO16: i16 = 0x471d;
/// `32768·1/√2`.
const OOR2: i16 = 0x5a82;

/// §W.5.3 `Transpose()` — transpose the 8×8 block in place.
fn transpose(block: &mut [i16; 64]) {
    for i in 0..8 {
        for j in 0..i {
            block.swap(8 * i + j, 8 * j + i);
        }
    }
}

/// §W.5.3 `HalfSwap()` — the one-dimensional row swap (rows 1↔4,
/// 3↔6, 5↔7).
fn half_swap(block: &mut [i16; 64]) {
    for i in 0..8 {
        block.swap(8 + i, 32 + i);
        block.swap(24 + i, 48 + i);
        block.swap(40 + i, 56 + i);
    }
}

/// §W.5.3 `Swap()` — the butterfly-order ↔ natural-order permutation
/// (an involution: `HalfSwap ∘ Transpose ∘ HalfSwap`).
fn swap(block: &mut [i16; 64]) {
    half_swap(block);
    transpose(block);
    half_swap(block);
}

/// §W.5.3 `Scale()` — arithmetic shift of every element (`sh > 0`
/// right, `sh < 0` left by `-sh`; the left shift wraps in 16 bits
/// exactly like the listing's `short` assignment).
fn scale(block: &mut [i16; 64], sh: i8) {
    if sh > 0 {
        for v in block.iter_mut() {
            *v >>= sh as u32;
        }
    } else {
        for v in block.iter_mut() {
            *v = ((*v as i32) << (-sh) as u32) as i16;
        }
    }
}

/// §W.5.3 `Round()` — final rounding: add the half-ulp bias unless it
/// would overflow the 16-bit ceiling (then pin to `0x7FFF`), shift,
/// and clamp into `[min, max]`.
fn round(block: &mut [i16; 64], sh: i8, min: i16, max: i16) {
    let sh = sh as u32;
    for v in block.iter_mut() {
        let mut t = *v as i32;
        if t < 0x0000_7FFF - (1 << (sh - 1)) {
            t += 1 << (sh - 1);
        } else {
            t = 0x0000_7FFF;
        }
        t >>= sh;
        *v = t.clamp(min as i32, max as i32) as i16;
    }
}

/// §W.5.3 `Multiply()` — multiply by a constant with shift, the
/// saturating `0x7FFF` rounding bias, and the high-half extraction.
fn multiply(a: i16, x: i16, sh: i8) -> i16 {
    let mut tmp = (a as i32).wrapping_mul(x as i32);
    if sh > 0 {
        tmp >>= sh as u32;
    } else {
        tmp = tmp.wrapping_shl((-sh) as u32);
    }
    if tmp < 0x7FFF_FFFF - 0x0000_7FFF {
        tmp += 0x0000_7FFF;
    } else {
        tmp = 0x7FFF_FFFF;
    }
    (tmp >> 16) as i16
}

/// §W.5.3 `Rotate()` — the planar rotation of two registers by the
/// constant pair `(a, b)` with per-factor shifts; `inv` selects the
/// inverse-transform rounding placement.
fn rotate(x: &mut i16, y: &mut i16, sha: i8, shb: i8, a: i16, b: i16, inv: bool) {
    let sh = |t: i32, s: i8| -> i32 {
        if s > 0 {
            t >> s as u32
        } else {
            t.wrapping_shl((-s) as u32)
        }
    };
    let mut tmplxa = sh((*x as i32).wrapping_mul(a as i32), sha);
    let tmplya = sh((*y as i32).wrapping_mul(a as i32), sha);
    let mut tmplxb = sh((*x as i32).wrapping_mul(b as i32), shb);
    let mut tmplyb = sh((*y as i32).wrapping_mul(b as i32), shb);
    let (tmpl1, tmpl2);
    if inv {
        tmplxa = tmplxa.wrapping_add(0x0000_7FFF);
        tmplxb = tmplxb.wrapping_add(0x0000_7FFF);
        tmpl1 = tmplxb.wrapping_sub(tmplya);
        tmpl2 = tmplxa.wrapping_add(tmplyb);
    } else {
        let tmplya = tmplya.wrapping_add(0x0000_7FFF);
        tmplyb = tmplyb.wrapping_add(0x0000_7FFF);
        tmpl1 = tmplxb.wrapping_add(tmplya);
        tmpl2 = (-tmplxa).wrapping_add(tmplyb);
    }
    *x = (tmpl1 >> 16) as i16;
    *y = (tmpl2 >> 16) as i16;
}

/// §W.5.3 `Butterfly()` — one-dimensional IDCT of an 8-element row
/// slice; `pass` is `0` for the first dimension and `1` for the
/// second (it offsets the rotation shifts and switches the DC pair to
/// the rounded half-sum form).
fn butterfly(block: &mut [i16; 64], base: usize, pass: i8) {
    // Rotations of the first phase. Split borrows via indices.
    let rot = |blk: &mut [i16; 64], i: usize, j: usize, sha: i8, shb: i8, a: i16, b: i16| {
        let (mut x, mut y) = (blk[base + i], blk[base + j]);
        rotate(&mut x, &mut y, sha, shb, a, b, true);
        blk[base + i] = x;
        blk[base + j] = y;
    };
    let shadow0 = [block[base], block[base + 4]];
    rot(block, 2, 6, pass - 2, pass - 1, CPO8, SPO8);
    rot(block, 1, 7, pass - 1, pass - 1, CPO16, SPO16);
    rot(block, 3, 5, pass - 1, pass - 1, C3PO16, S3PO16);

    if pass != 0 {
        let tmp = block[base + 4] as i32;
        let b0 = block[base] as i32;
        let a = b0 + tmp;
        let b = b0 - tmp;
        let borrow = if tmp < 0 { 1 } else { 0 };
        block[base] = ((a - borrow) >> 1) as i16;
        block[base + 4] = ((b - borrow) >> 1) as i16;
    } else {
        block[base] = shadow0[0].wrapping_add(shadow0[1]);
        block[base + 4] = shadow0[0].wrapping_sub(shadow0[1]);
    }

    let mut shadow = [0i16; 8];
    shadow.copy_from_slice(&block[base..base + 8]);

    // Second phase.
    block[base + 1] = shadow[1].wrapping_sub(shadow[3]);
    block[base + 3] = shadow[1].wrapping_add(shadow[3]);
    block[base + 7] = shadow[7].wrapping_sub(shadow[5]);
    block[base + 5] = shadow[7].wrapping_add(shadow[5]);
    block[base] = shadow[0].wrapping_add(shadow[6]);
    block[base + 6] = shadow[0].wrapping_sub(shadow[6]);
    block[base + 4] = shadow[4].wrapping_add(shadow[2]);
    block[base + 2] = shadow[4].wrapping_sub(shadow[2]);

    shadow.copy_from_slice(&block[base..base + 8]);

    // Third phase.
    block[base + 7] = shadow[7].wrapping_sub(shadow[3]);
    block[base + 3] = shadow[7].wrapping_add(shadow[3]);
    block[base + 1] = multiply(OOR2, shadow[1], -2);
    block[base + 5] = multiply(OOR2, shadow[5], -2);

    shadow.copy_from_slice(&block[base..base + 8]);

    // Fourth phase.
    block[base + 4] = shadow[4].wrapping_add(shadow[3]);
    block[base + 3] = shadow[4].wrapping_sub(shadow[3]);
    block[base + 2] = shadow[2].wrapping_add(shadow[7]);
    block[base + 7] = shadow[2].wrapping_sub(shadow[7]);
    block[base] = shadow[0].wrapping_add(shadow[5]);
    block[base + 5] = shadow[0].wrapping_sub(shadow[5]);
    block[base + 6] = shadow[6].wrapping_add(shadow[1]);
    block[base + 1] = shadow[6].wrapping_sub(shadow[1]);
}

/// §W.5.3 `IDCT()` — the reference fixed-point IDCT 0.
///
/// `block` holds the 64 dequantised coefficients in row-major natural
/// order (12-bit signed input range); on return it holds the spatial
/// output clipped to `[-256, 255]`. Any implementation producing these
/// exact values for every input is "reference IDCT 0" (§W.5.3).
pub fn idct_w0(block: &mut [i16; 64]) {
    scale(block, -4);
    for i in 0..8 {
        butterfly(block, 8 * i, 0);
    }
    transpose(block);
    for i in 0..8 {
        butterfly(block, 8 * i, 1);
    }
    round(block, 6, -256, 255);
    swap(block);
}

/// §W.5.3 `FButterfly()` — one-dimensional forward transform of an
/// 8-element row slice (informative companion listing).
fn fbutterfly(block: &mut [i16; 64], base: usize) {
    let mut shadow = [0i16; 8];
    shadow.copy_from_slice(&block[base..base + 8]);

    // First phase.
    for i in 0..4 {
        block[base + i] = shadow[i].wrapping_add(shadow[7 - i]);
        block[base + 7 - i] = shadow[i].wrapping_sub(shadow[7 - i]);
    }

    shadow.copy_from_slice(&block[base..base + 8]);

    // Second phase.
    block[base] = shadow[0].wrapping_add(shadow[3]);
    block[base + 3] = shadow[0].wrapping_sub(shadow[3]);
    block[base + 1] = shadow[1].wrapping_add(shadow[2]);
    block[base + 2] = shadow[1].wrapping_sub(shadow[2]);
    block[base + 4] = multiply(OOR2, shadow[4], -2);
    block[base + 7] = multiply(OOR2, shadow[7], -2);
    block[base + 6] = shadow[6].wrapping_sub(shadow[5]);
    block[base + 5] = shadow[6].wrapping_add(shadow[5]);

    shadow.copy_from_slice(&block[base..base + 8]);

    // Third phase.
    block[base] = shadow[0].wrapping_add(shadow[1]);
    block[base + 1] = shadow[0].wrapping_sub(shadow[1]);
    block[base + 6] = shadow[6].wrapping_sub(shadow[4]);
    block[base + 4] = shadow[6].wrapping_add(shadow[4]);
    block[base + 7] = shadow[7].wrapping_sub(shadow[5]);
    block[base + 5] = shadow[7].wrapping_add(shadow[5]);

    // Fourth phase.
    let rot = |blk: &mut [i16; 64], i: usize, j: usize, sha: i8, shb: i8, a: i16, b: i16| {
        let (mut x, mut y) = (blk[base + i], blk[base + j]);
        rotate(&mut x, &mut y, sha, shb, a, b, false);
        blk[base + i] = x;
        blk[base + j] = y;
    };
    rot(block, 2, 3, -2, -1, CPO8, SPO8);
    rot(block, 4, 5, -1, -1, CPO16, SPO16);
    rot(block, 6, 7, -1, -1, C3PO16, S3PO16);
}

/// §W.5.3 `FDCT()` — the informative fixed-point forward DCT paired
/// with [`idct_w0`].
///
/// `block` holds 64 spatial values in row-major natural order (9-bit
/// signed input range); on return it holds the coefficients clipped
/// to `[-2048, 2047]`.
pub fn fdct_w0(block: &mut [i16; 64]) {
    for i in 0..8 {
        fbutterfly(block, 8 * i);
    }
    transpose(block);
    for i in 0..8 {
        fbutterfly(block, 8 * i);
    }
    round(block, 3, -2048, 2047);
    swap(block);
}
