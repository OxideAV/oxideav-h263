//! H.263 P-frame motion compensation (§6.1) and INTER-block
//! reconstruction (§6.3.1 summation + §6.3.2 clip).
//!
//! Implements the default (non-Annex-D, non-Annex-F) prediction mode
//! of ITU-T Recommendation H.263 (01/2005) §6.1:
//!
//! * **§6.1.1 differential motion vectors** — the macroblock vector
//!   is obtained by adding the median predictor (taken from the three
//!   surrounding macroblocks per Figure 12) to the [`Mvd`] difference
//!   decoded from Table 14. Each VLC word for MVD represents a *pair*
//!   of difference values; only one of the pair yields a component in
//!   the permitted range `[-16, 15.5]` (= `[-32, 31]` in half-pel
//!   units, a span of 64). The reconstruction therefore wraps the
//!   `predictor + difference` sum into that 64-wide window.
//! * **Table 18 chrominance vector derivation** — the chroma vector
//!   is the macroblock luma vector divided by two, with the resulting
//!   quarter-pel position rounded towards the nearest half-pel
//!   position per Table 18.
//! * **§6.1.2 half-pixel interpolation** — half-pel sample values are
//!   found by bilinear interpolation per Figure 13 with `RCONTROL`
//!   (the rounding-type control bit, implied `0` in baseline H.263).
//! * **§6.3.1 summation** — INTER reconstruction = motion-compensated
//!   prediction + inverse-transformed residual, pixel by pixel.
//! * **§6.3.2 clipping** — the summed result is clipped to `[0, 255]`.
//!
//! The reference-picture access here uses the §D.1 edge-replication
//! rule: a pixel referenced outside the coded picture area is replaced
//! by the nearest edge pixel ("limiting the motion vector to the last
//! full-pixel position inside the coded picture area"). This is the
//! always-on boundary behaviour, and it is exactly the §D.1 rule the
//! Unrestricted Motion Vector mode relies on when a vector points
//! outside the picture.
//!
//! Annex D §D.2 — the **Unrestricted Motion Vector mode** with
//! PLUSPTYPE *absent* from the picture header — is wired through
//! [`reconstruct_mv_component_umv`] / [`reconstruct_mv_umv`]: the
//! per-component range is widened from the default `[-32, 31]` to
//! `[-63, 63]` half-pel, with the §D.2 predictor-dependent selection of
//! the Table-14 difference pair. The wider PLUSPTYPE / UUI ranges of
//! Tables D.1 / D.2 and the Table-D.3 reversible VLC remain gated on
//! the not-yet-decoded extended-PTYPE header.
//!
//! Annex F §F.2 — the **Advanced Prediction mode** four-motion-vector
//! candidate-predictor redefinition (Figure F.1) and the Table F.1
//! sixteenth-pixel chrominance-vector derivation — is provided as pure
//! transformations in [`LumaBlockIndex`], [`Mb4MvNeighbourhood`],
//! [`select_4mv_candidates`] and [`chroma_mv_4mv`] /
//! [`chroma_mv_component_4mv`]. They take a fully resolved neighbour
//! grid (the caller decides which neighbour MBs are present, INTRA, or
//! not coded — those map to `None`) and return the three §F.2 candidate
//! predictors for one of the four 8×8 luminance blocks in a
//! macroblock, ready to feed into [`predict_mv_median`]. The §F.3
//! overlapped block motion compensation (the weighted three-prediction
//! H0/H1/H2 average) and the macroblock-driver wiring that derives the
//! neighbour grid from the live picture state are out of scope for the
//! current round.

use crate::block::COEFFS_PER_BLOCK;
use crate::idct::BLOCK_DIM;
use crate::macroblock::Mvd;

/// §6.1.1 permitted motion-vector component range, lower bound, in
/// **half-pel units** (spec "Vector" value `-16`).
pub const MV_HALF_MIN: i32 = -32;
/// §6.1.1 permitted motion-vector component range, upper bound, in
/// **half-pel units** (spec "Vector" value `+15.5`).
pub const MV_HALF_MAX: i32 = 31;
/// Width of the permitted range (number of distinct half-pel
/// components), used as the wrap modulus in [`reconstruct_mv_component`].
pub const MV_HALF_SPAN: i32 = MV_HALF_MAX - MV_HALF_MIN + 1; // 64

/// A reconstructed motion vector, in **half-pel units** (a value of
/// `1` denotes half a pixel of displacement). Positive `dx`/`dy`
/// indicate prediction from pixels to the right of / below the pixels
/// being predicted (§6.1.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MotionVector {
    /// Horizontal component, half-pel units.
    pub dx_half: i32,
    /// Vertical component, half-pel units.
    pub dy_half: i32,
}

impl MotionVector {
    /// Construct from explicit half-pel components.
    pub const fn new(dx_half: i32, dy_half: i32) -> Self {
        Self { dx_half, dy_half }
    }
}

/// §6.1.1 reconstruction of one motion-vector component.
///
/// The macroblock vector component is `predictor + difference`, with
/// the caveat that "each VLC word for MVD represents a pair of
/// difference values; only one of the pair will yield a macroblock
/// vector component falling within the permitted range `[-16, 15.5]`".
/// In half-pel units that range is `[-32, 31]` — a window of
/// [`MV_HALF_SPAN`] (= 64) distinct values. The reconstruction wraps
/// the raw sum into that window:
///
/// ```text
///   v = predictor + difference
///   while v < -32 { v += 64 }
///   while v >  31 { v -= 64 }
/// ```
///
/// Both `predictor` and `difference` are in half-pel units; the
/// returned component is in `[-32, 31]`.
pub fn reconstruct_mv_component(predictor: i32, difference: i32) -> i32 {
    let mut v = predictor + difference;
    // The raw sum can be at most |predictor| + |difference| ≤
    // 31 + 32 = 63 from the window edge, so a single wrap on each
    // side always lands inside, but the loop form is robust and
    // matches the spec's "the value in range" selection exactly.
    while v < MV_HALF_MIN {
        v += MV_HALF_SPAN;
    }
    while v > MV_HALF_MAX {
        v -= MV_HALF_SPAN;
    }
    v
}

/// §6.1.1 reconstruction of a full motion vector from its predictor
/// and the decoded [`Mvd`] (Table 14, half-pel units).
pub fn reconstruct_mv(predictor: MotionVector, mvd: Mvd) -> MotionVector {
    MotionVector {
        dx_half: reconstruct_mv_component(predictor.dx_half, mvd.dx_half as i32),
        dy_half: reconstruct_mv_component(predictor.dy_half, mvd.dy_half as i32),
    }
}

// ---- Annex D §D.2 Unrestricted Motion Vector mode (non-PLUSPTYPE) ---

/// Annex D §D.2 extended motion-vector component range, lower bound, in
/// **half-pel units** (spec "Vector" value `-31.5`). Applies in the
/// Unrestricted Motion Vector mode when PLUSPTYPE is *absent* from the
/// picture header.
pub const MV_UMV_HALF_MIN: i32 = -63;
/// Annex D §D.2 extended motion-vector component range, upper bound, in
/// **half-pel units** (spec "Vector" value `+31.5`).
pub const MV_UMV_HALF_MAX: i32 = 63;

/// Annex D §D.2 reconstruction of one motion-vector component in the
/// **Unrestricted Motion Vector mode** with PLUSPTYPE *absent* from the
/// picture header.
///
/// In this mode the per-component range is extended from the default
/// `[-32, 31]` (spec `[-16, 15.5]`) to `[-63, 63]` half-pel
/// (spec `[-31.5, 31.5]`). `difference` is the Table-14 "Vector"
/// column value already decoded into `[-32, 31]` half-pel; that value
/// stands for a *pair* of differences `{difference, difference ± 64}`
/// (the two members differ by [`MV_HALF_SPAN`]). §D.2 selects which
/// member to add to the predictor:
///
/// * If the predictor `Pc` lies in `[-31, 32]` half-pel
///   (spec `[-15.5, 16]`), "only the first column of vector
///   differences applies" — the component is `Pc + difference`, with
///   no wrap. The result is guaranteed to land in `[Pc-32, Pc+31]`,
///   which is inside `[-63, 63]`.
/// * Otherwise (predictor outside `[-31, 32]`), the member of the pair
///   is chosen that yields a component inside `[-63, 63]` **with the
///   same sign as the predictor**, where zero counts as either sign.
///   Concretely (§D.2):
///   - `-63 ≤ Pc ≤ -32` ⇒ result in `[-63, 0]`,
///   - `33 ≤ Pc ≤ 63` ⇒ result in `[0, 63]`.
///
/// Both `predictor` and `difference` are half-pel; the returned
/// component is in `[-63, 63]`.
pub fn reconstruct_mv_component_umv(predictor: i32, difference: i32) -> i32 {
    // §D.2: predictor inside [-31, 32] -> the first column applies
    // directly (no pair selection, no wrap).
    if (-31..=32).contains(&predictor) {
        return predictor + difference;
    }

    // Predictor outside [-31, 32]: the Table-14 codeword denotes the
    // pair {difference, difference ± MV_HALF_SPAN}; pick the member
    // whose `predictor + member` lands inside [-63, 63] with the same
    // sign as the predictor (zero allowed for either sign).
    let alt = if difference >= 0 {
        difference - MV_HALF_SPAN
    } else {
        difference + MV_HALF_SPAN
    };
    let candidates = [predictor + difference, predictor + alt];
    let want_nonneg = predictor > 0;
    for &mvc in &candidates {
        if !(MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX).contains(&mvc) {
            continue;
        }
        // Same sign as the predictor, with zero permitted either way.
        let ok = if want_nonneg { mvc >= 0 } else { mvc <= 0 };
        if ok {
            return mvc;
        }
    }
    // Per §D.2 exactly one candidate satisfies the constraints; this
    // fallback (clamp into range) keeps the function total against
    // malformed predictor/difference combinations.
    (predictor + difference).clamp(MV_UMV_HALF_MIN, MV_UMV_HALF_MAX)
}

/// Annex D §D.2 reconstruction of a full motion vector in the
/// Unrestricted Motion Vector mode (PLUSPTYPE absent), applying
/// [`reconstruct_mv_component_umv`] per component.
pub fn reconstruct_mv_umv(predictor: MotionVector, mvd: Mvd) -> MotionVector {
    MotionVector {
        dx_half: reconstruct_mv_component_umv(predictor.dx_half, mvd.dx_half as i32),
        dy_half: reconstruct_mv_component_umv(predictor.dy_half, mvd.dy_half as i32),
    }
}

/// §6.1.1 median predictor for one component.
///
/// "For each component, the predictor is the median value of the three
/// candidate predictors for this component." (Figure 12 supplies the
/// three candidates MV1 / MV2 / MV3; the border decision rules that
/// select their values are the caller's responsibility.)
pub fn median3(a: i32, b: i32, c: i32) -> i32 {
    // median = a + b + c - max - min
    let max = a.max(b).max(c);
    let min = a.min(b).min(c);
    a + b + c - max - min
}

/// §6.1.1 median predictor for a full motion vector (per-component
/// median of the three candidate predictors).
pub fn predict_mv_median(mv1: MotionVector, mv2: MotionVector, mv3: MotionVector) -> MotionVector {
    MotionVector {
        dx_half: median3(mv1.dx_half, mv2.dx_half, mv3.dx_half),
        dy_half: median3(mv1.dy_half, mv2.dy_half, mv3.dy_half),
    }
}

/// Table 18 modification of one quarter-pixel chrominance vector
/// component towards the nearest half-pixel position.
///
/// The chrominance vector is the luminance macroblock vector divided
/// by two (§6.1.1): a luma half-pel component `l` corresponds to a
/// chroma quarter-pel value `l / 2` (quarter-pel units). Table 18
/// maps quarter-pixel positions to the nearest half-pixel position:
///
/// ```text
///   quarter-pel fractional part   0   1/4  1/2  3/4   1
///   resulting   fractional part   0   1/2  1/2  1/2   1
/// ```
///
/// In half-pel units, a luma component `l` (half-pel) gives a chroma
/// component whose magnitude is `round_to_table18(l)`. Concretely, the
/// integer part of `l/2` (in pixels) is preserved and any non-zero
/// fractional remainder collapses to a single half-pel step.
///
/// We compute this in half-pel chroma units (so the returned value is
/// directly usable as a half-pel displacement on the chroma plane).
/// `l` is the luma component in half-pel units; the result is the
/// chroma component in half-pel units.
pub fn chroma_mv_component(l: i32) -> i32 {
    // l is in luma half-pel units. l/2 (rounded toward zero in
    // quarter-pel terms) gives the chroma displacement in quarter-pel
    // units; Table 18 then snaps the quarter-pel fraction to the
    // nearest half. Work with sign separated so truncation behaves
    // symmetrically about zero (Table 18 is symmetric for ±).
    let sign = if l < 0 { -1 } else { 1 };
    let mag = l.unsigned_abs() as i32; // luma half-pel magnitude
                                       // quarter-pel magnitude = mag (because 1 luma half-pel == 1 chroma
                                       // quarter-pel after the /2). Express in chroma half-pel units:
                                       //   full pixels  = mag / 4   (each pixel is 4 quarter-pels)
                                       //   remainder    = mag % 4   (0..3 quarter-pels)
                                       // Table 18: remainder 0 -> +0 half-pel, remainder 1/2/3 -> +1 half-pel.
    let full_pixels = mag / 4;
    let rem = mag % 4;
    let frac_half = if rem == 0 { 0 } else { 1 };
    sign * (full_pixels * 2 + frac_half)
}

/// Table 18 chroma vector for a full luma motion vector.
pub fn chroma_mv(luma: MotionVector) -> MotionVector {
    MotionVector {
        dx_half: chroma_mv_component(luma.dx_half),
        dy_half: chroma_mv_component(luma.dy_half),
    }
}

/// A non-owning view of one reference-picture plane (luma or one
/// chroma channel) in row-major `u8` layout. Used as the source for
/// motion-compensated prediction.
#[derive(Debug, Clone, Copy)]
pub struct RefPlane<'a> {
    /// Row-major samples, `width * height` long.
    pub samples: &'a [u8],
    /// Plane width in pixels.
    pub width: usize,
    /// Plane height in pixels.
    pub height: usize,
}

impl<'a> RefPlane<'a> {
    /// Construct a plane view, asserting the sample buffer matches the
    /// declared dimensions.
    pub fn new(samples: &'a [u8], width: usize, height: usize) -> Self {
        debug_assert_eq!(samples.len(), width * height);
        Self {
            samples,
            width,
            height,
        }
    }

    /// Fetch a sample with §D.1 edge replication: coordinates outside
    /// the coded picture area are clamped to the nearest edge pixel.
    #[inline]
    fn at(&self, x: i32, y: i32) -> i32 {
        let cx = x.clamp(0, self.width as i32 - 1) as usize;
        let cy = y.clamp(0, self.height as i32 - 1) as usize;
        self.samples[cy * self.width + cx] as i32
    }
}

/// `RCONTROL` for §6.1.2 bilinear interpolation. In baseline H.263 it
/// is implied `0`; the rounding-type bit (Annex M / extended-PTYPE)
/// can set it to `1`. We accept it as a parameter so the caller can
/// pass the right value when extended PTYPE is decoded, but baseline
/// callers pass `0`.
pub const RCONTROL_DEFAULT: i32 = 0;

/// §6.1.2 / Figure 13 — fetch one motion-compensated sample at the
/// given half-pel position `(hx, hy)` (in half-pel units) from a
/// reference plane.
///
/// The integer pixel position is `(hx >> 1, hy >> 1)`; the low bit of
/// each component selects the sub-pixel phase. Per Figure 13 with
/// integer neighbours A/B/C/D (A = top-left integer pixel, B = A's
/// right neighbour, C = A's lower neighbour, D = the diagonal):
///
/// ```text
///   a = A                                  (no fractional offset)
///   b = (A + B + 1 - RCONTROL) / 2         (horizontal half-pel)
///   c = (A + C + 1 - RCONTROL) / 2         (vertical half-pel)
///   d = (A + B + C + D + 2 - RCONTROL) / 4 (diagonal half-pel)
/// ```
///
/// where `/` is truncating integer division. Edge replication (§D.1)
/// is applied when a neighbour falls outside the plane.
#[inline]
fn sample_half_pel(plane: &RefPlane<'_>, hx: i32, hy: i32, rcontrol: i32) -> i32 {
    let ix = hx >> 1; // floor toward -inf for negatives is fine: see note
    let iy = hy >> 1;
    let fx = hx & 1;
    let fy = hy & 1;
    let a = plane.at(ix, iy);
    match (fx, fy) {
        (0, 0) => a,
        (1, 0) => {
            let b = plane.at(ix + 1, iy);
            (a + b + 1 - rcontrol) / 2
        }
        (0, 1) => {
            let c = plane.at(ix, iy + 1);
            (a + c + 1 - rcontrol) / 2
        }
        _ => {
            let b = plane.at(ix + 1, iy);
            let c = plane.at(ix, iy + 1);
            let d = plane.at(ix + 1, iy + 1);
            (a + b + c + d + 2 - rcontrol) / 4
        }
    }
}

/// §6.1 + §6.1.2 — build the 8×8 motion-compensated prediction block
/// for a block whose top-left corner is at integer pixel position
/// `(block_x, block_y)` in the **current** picture, using the motion
/// vector `mv` (half-pel units) against the reference plane.
///
/// For each output pixel `(px, py)` in the 8×8 block, the half-pel
/// source position is
/// `((block_x + px) * 2 + mv.dx_half, (block_y + py) * 2 + mv.dy_half)`.
/// The sample is fetched via §6.1.2 bilinear interpolation with edge
/// replication.
pub fn motion_compensate_block(
    plane: &RefPlane<'_>,
    block_x: usize,
    block_y: usize,
    mv: MotionVector,
    rcontrol: i32,
) -> [u8; COEFFS_PER_BLOCK] {
    let mut out = [0u8; COEFFS_PER_BLOCK];
    for py in 0..BLOCK_DIM {
        for px in 0..BLOCK_DIM {
            let hx = ((block_x + px) as i32) * 2 + mv.dx_half;
            let hy = ((block_y + py) as i32) * 2 + mv.dy_half;
            let s = sample_half_pel(plane, hx, hy, rcontrol);
            // Reference samples are 8-bit, and bilinear averaging keeps
            // the result inside [0, 255]; clamp defensively.
            out[py * BLOCK_DIM + px] = s.clamp(0, 255) as u8;
        }
    }
    out
}

/// §6.3.1 summation + §6.3.2 clip for an INTER block.
///
/// Takes the motion-compensated 8×8 `prediction` (from
/// [`motion_compensate_block`]) and the 8×8 inverse-transformed
/// residual `residual` (the §6.2.4 IDCT output, signed, in
/// `[-256, 255]`), and forms the reconstructed sample block:
///
/// ```text
///   rec(x, y) = clip( prediction(x, y) + residual(x, y) )   §6.3.1 + §6.3.2
/// ```
///
/// The clip pins values below 0 to 0 and above 255 to 255 (§6.3.2).
pub fn reconstruct_inter_block(
    prediction: &[u8; COEFFS_PER_BLOCK],
    residual: &[i16; COEFFS_PER_BLOCK],
) -> [u8; COEFFS_PER_BLOCK] {
    let mut out = [0u8; COEFFS_PER_BLOCK];
    for ((dst, &p), &r) in out.iter_mut().zip(prediction.iter()).zip(residual.iter()) {
        let sum = p as i32 + r as i32;
        *dst = sum.clamp(0, 255) as u8;
    }
    out
}

// ---- Annex F §F.2 Four motion vectors per macroblock ----------------

/// Index of one 8×8 luminance block within a macroblock, in Figure 5
/// (§4.2.5) order.
///
/// The four luminance blocks of a macroblock are arranged in a 2×2
/// grid; this enum names the four positions so the Annex F §F.2
/// candidate-predictor redefinition (Figure F.1) can be expressed as a
/// pure function of "which block am I deriving predictors for".
///
/// ```text
///   | B1 (top-left)    | B2 (top-right)    |
///   | B3 (bottom-left) | B4 (bottom-right) |
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LumaBlockIndex {
    /// Top-left 8×8 luminance block in the macroblock (block 1 of
    /// Figure 5).
    B1,
    /// Top-right 8×8 luminance block (block 2 of Figure 5).
    B2,
    /// Bottom-left 8×8 luminance block (block 3 of Figure 5).
    B3,
    /// Bottom-right 8×8 luminance block (block 4 of Figure 5).
    B4,
}

impl LumaBlockIndex {
    /// All four luma blocks in Figure-5 order: `[B1, B2, B3, B4]`.
    pub const ALL: [LumaBlockIndex; 4] = [
        LumaBlockIndex::B1,
        LumaBlockIndex::B2,
        LumaBlockIndex::B3,
        LumaBlockIndex::B4,
    ];

    /// Per Figure 5: index `0..=3` mapping to [`Self::B1`] .. [`Self::B4`].
    pub fn from_index(i: usize) -> Option<Self> {
        Self::ALL.get(i).copied()
    }

    /// Per Figure 5: `B1 -> 0`, `B2 -> 1`, `B3 -> 2`, `B4 -> 3`.
    pub fn index(self) -> usize {
        match self {
            LumaBlockIndex::B1 => 0,
            LumaBlockIndex::B2 => 1,
            LumaBlockIndex::B3 => 2,
            LumaBlockIndex::B4 => 3,
        }
    }
}

/// The four 8×8 luminance motion vectors of a macroblock, in
/// [`LumaBlockIndex`] (Figure 5) order.
///
/// In the Advanced Prediction mode (§F.2) every macroblock carries four
/// vectors — the §F.2 last paragraph clarifies that even one-vector
/// macroblocks "are defined as four vectors with the same value", so
/// the caller can store every neighbouring macroblock as a 4-element
/// array unconditionally.
pub type Mb4Mv = [MotionVector; 4];

/// A view of the per-block motion vectors at the five macroblock
/// positions Figure F.1 references when deriving the [`MV1`, `MV2`,
/// `MV3`] candidates for any of the four luminance blocks in the
/// **current** macroblock.
///
/// Each `Option` represents an availability decision that must already
/// have been made by the caller per the §6.1.1 border-decision rules:
///
/// * `None` — the corresponding candidate predictor is treated as zero
///   (the §6.1.1 rule-1 "INTRA / not-coded → zero" decision, the
///   rule-2 "outside picture or slice at the left" decision, and the
///   rule-4 "outside picture at the right" decision all collapse the
///   neighbouring 4-MV array into `None`).
/// * `Some([mv0, mv1, mv2, mv3])` — the four 8×8 luminance vectors of
///   the neighbour macroblock are available, in [`LumaBlockIndex`] /
///   Figure-5 order.
///
/// The §6.1.1 rule-3 "outside picture/GOB at top → MV2 and MV3 become
/// MV1" is applied **after** [`select_4mv_candidates`] returns, by the
/// caller, because it depends on the resolved MV1 not on the raw
/// neighbour grid.
#[derive(Debug, Clone, Copy, Default)]
pub struct Mb4MvNeighbourhood {
    /// The current macroblock's four luma vectors (in Figure-5 order).
    /// Always present.
    pub current: Mb4Mv,
    /// The macroblock to the **left** of the current one. `None` if the
    /// current MB is at the left picture / slice border or the left
    /// neighbour was INTRA / not coded.
    pub left: Option<Mb4Mv>,
    /// The macroblock **above** the current one. `None` if at the top
    /// picture / GOB border or INTRA / not coded.
    pub above: Option<Mb4Mv>,
    /// The macroblock **above-right** of the current one. `None` if at
    /// the picture border, or INTRA / not coded.
    pub above_right: Option<Mb4Mv>,
    /// The macroblock to the **right** of the current one. `None` if
    /// at the right picture border or INTRA / not coded. Used only for
    /// the block-4 case (Figure F.1 lower-right sub-figure), where the
    /// "above-right" of block 4 reads block 1 of the right neighbour.
    pub right: Option<Mb4Mv>,
}

impl Mb4MvNeighbourhood {
    /// Construct a neighbourhood with only the current MB's vectors
    /// (every external neighbour set to `None`). Used at picture
    /// corners and in unit tests.
    pub fn isolated(current: Mb4Mv) -> Self {
        Self {
            current,
            left: None,
            above: None,
            above_right: None,
            right: None,
        }
    }
}

/// Annex F §F.2 / Figure F.1 — candidate-predictor selection for one of
/// the four luminance blocks in the current macroblock.
///
/// Returns the `(MV1, MV2, MV3)` candidates for the §6.1.1 median
/// predictor (which is then fed into [`predict_mv_median`]). The
/// neighbour-grid lookup follows Figure F.1's "the 8×8 block at the
/// physically same relative position around `MV`" convention:
///
/// | block | MV1 = left of block        | MV2 = above block            | MV3 = above-right of block         |
/// |-------|----------------------------|------------------------------|------------------------------------|
/// | B1    | `left.B2` else 0           | `above.B3` else 0            | `above.B4` else 0                  |
/// | B2    | `current.B1`               | `above.B4` else 0            | `above_right.B3` else 0            |
/// | B3    | `left.B4` else 0           | `current.B1`                 | `current.B2`                       |
/// | B4    | `current.B3`               | `current.B2`                 | `right.B1` else 0                  |
///
/// The "else 0" entries are the §6.1.1 default — when the requested
/// neighbour is not available the candidate is zero. The §6.1.1 rule-3
/// "if MB-above is unavailable, set MV2 and MV3 to MV1" rewrite is
/// **not** applied here: it's the caller's responsibility, because the
/// rule depends on the *resolved* MV1 and the caller knows from the
/// border state whether MB-above is present (rule 3) or whether
/// individual cells were INTRA (rule 1).
///
/// The caller passes the [`Mb4MvNeighbourhood`] with `None` for any
/// neighbour MB that is INTRA, not coded, or outside the picture /
/// slice / GOB; the function never looks "through" `None`.
pub fn select_4mv_candidates(
    block: LumaBlockIndex,
    n: &Mb4MvNeighbourhood,
) -> (MotionVector, MotionVector, MotionVector) {
    let zero = MotionVector::default();
    match block {
        // B1 (top-left): left of B1 is B2 of MB-left; above is B3 of
        // MB-above; above-right is B4 of MB-above (the upper-right
        // 8×8 of the row of blocks above current).
        LumaBlockIndex::B1 => {
            let mv1 = n
                .left
                .map(|m| m[LumaBlockIndex::B2.index()])
                .unwrap_or(zero);
            let mv2 = n
                .above
                .map(|m| m[LumaBlockIndex::B3.index()])
                .unwrap_or(zero);
            let mv3 = n
                .above
                .map(|m| m[LumaBlockIndex::B4.index()])
                .unwrap_or(zero);
            (mv1, mv2, mv3)
        }
        // B2 (top-right): left of B2 is B1 of current; above is B4 of
        // MB-above; above-right is B3 of MB-above-right.
        LumaBlockIndex::B2 => {
            let mv1 = n.current[LumaBlockIndex::B1.index()];
            let mv2 = n
                .above
                .map(|m| m[LumaBlockIndex::B4.index()])
                .unwrap_or(zero);
            let mv3 = n
                .above_right
                .map(|m| m[LumaBlockIndex::B3.index()])
                .unwrap_or(zero);
            (mv1, mv2, mv3)
        }
        // B3 (bottom-left): left of B3 is B4 of MB-left; above is B1
        // of current; above-right is B2 of current.
        LumaBlockIndex::B3 => {
            let mv1 = n
                .left
                .map(|m| m[LumaBlockIndex::B4.index()])
                .unwrap_or(zero);
            let mv2 = n.current[LumaBlockIndex::B1.index()];
            let mv3 = n.current[LumaBlockIndex::B2.index()];
            (mv1, mv2, mv3)
        }
        // B4 (bottom-right): left of B4 is B3 of current; above is B2
        // of current; above-right is B1 of MB-right.
        LumaBlockIndex::B4 => {
            let mv1 = n.current[LumaBlockIndex::B3.index()];
            let mv2 = n.current[LumaBlockIndex::B2.index()];
            let mv3 = n
                .right
                .map(|m| m[LumaBlockIndex::B1.index()])
                .unwrap_or(zero);
            (mv1, mv2, mv3)
        }
    }
}

/// Table F.1 — Annex F §F.2 modification of one **sixteenth-pixel**
/// chrominance vector component "towards the nearest half-pixel
/// position".
///
/// Index = sixteenth-pixel position `0..=15`; value = resulting
/// position expressed as a numerator over `2` (i.e., chroma half-pixel
/// units). 0 → 0/2, 3..=13 → 1/2, 14..=15 → 2/2. The table is **not**
/// symmetric around 8: positions 0, 1, 2 collapse to 0 (3 entries),
/// positions 14 and 15 collapse to 2 (2 entries), the middle 11
/// positions collapse to 1.
const TABLE_F1_SIXTEENTH_TO_HALF: [u8; 16] = [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2];

/// Annex F §F.2 / Table F.1 — modification of one chroma vector
/// component for the **four-motion-vector** case.
///
/// `luma_sum_half` is the **sum** of the four corresponding luma
/// motion-vector components (in luma half-pel units, [`MotionVector`]
/// convention). The returned chroma component is in **chroma half-pel
/// units** (the same convention as the rest of this module — directly
/// usable as the half-pel displacement on the chroma plane).
///
/// The derivation follows the spec literally: the sum of the four luma
/// components divided by 8 (chroma half-pel) lands on a position with
/// granularity `1/16` of a chroma pixel; Table F.1 then snaps the
/// residual sixteenth-pixel fraction to the nearest half-pixel
/// position (with the table's asymmetric mapping for the
/// `{0,1,2} → 0`, `{3..=13} → 1`, `{14,15} → 2` buckets).
///
/// In our `i32` half-pel arithmetic, the conversion identity is:
///
/// * `sum(luma_half_pel)` *is* the sixteenth-pel position directly —
///   each luma half-pel equals four chroma sixteenth-pels, and we sum
///   four of them then divide by 8, which is the same as dividing the
///   sum by 2 to get chroma half-pel, or equivalently by 16 to get
///   chroma pixels with the residual as the sixteenth-pel position.
/// * `|sum| / 16` is the integer **chroma-pixel** magnitude; multiply
///   by 2 to express it in chroma half-pel.
/// * `|sum| % 16` is the residual **sixteenth-pixel** position
///   (`0..=15`); Table F.1 maps it to a half-pixel fraction
///   (`0`, `1`, or `2` in chroma half-pel units).
/// * The sign is restored from `sign(sum)` (symmetric mirror — the
///   spec's Table F.1 only enumerates positive positions, and the
///   chroma vector is the same way symmetric around zero in baseline
///   bilinear interpolation).
pub fn chroma_mv_component_4mv(luma_sum_half: i32) -> i32 {
    let sign = if luma_sum_half < 0 { -1 } else { 1 };
    let mag = luma_sum_half.unsigned_abs() as i32;
    let full_chroma_pixels = mag / 16;
    let sixteenth = (mag % 16) as usize;
    let frac_half = TABLE_F1_SIXTEENTH_TO_HALF[sixteenth] as i32;
    sign * (full_chroma_pixels * 2 + frac_half)
}

/// Annex F §F.2 / Table F.1 — chroma vector for one macroblock's four
/// luma motion vectors. Returns the chroma displacement (in half-pel
/// units, [`MotionVector`] convention) applied to both Cb and Cr
/// blocks as the §F.2 last paragraph mandates ("the prediction for
/// chrominance is obtained by applying the motion vector MVDCHR to all
/// pixels in the two chrominance blocks").
pub fn chroma_mv_4mv(luma: &Mb4Mv) -> MotionVector {
    let sum_x = luma.iter().map(|mv| mv.dx_half).sum::<i32>();
    let sum_y = luma.iter().map(|mv| mv.dy_half).sum::<i32>();
    MotionVector {
        dx_half: chroma_mv_component_4mv(sum_x),
        dy_half: chroma_mv_component_4mv(sum_y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- §6.1.1 MV component reconstruction ----------------------

    /// Zero predictor + zero difference => zero vector.
    #[test]
    fn mv_component_zero() {
        assert_eq!(reconstruct_mv_component(0, 0), 0);
    }

    /// In-range sum stays as-is.
    #[test]
    fn mv_component_in_range_no_wrap() {
        assert_eq!(reconstruct_mv_component(4, 6), 10);
        assert_eq!(reconstruct_mv_component(-10, 5), -5);
        assert_eq!(reconstruct_mv_component(MV_HALF_MAX, 0), MV_HALF_MAX);
        assert_eq!(reconstruct_mv_component(MV_HALF_MIN, 0), MV_HALF_MIN);
    }

    /// Sum above +31 wraps down by 64 (the "other element of the
    /// pair" selection): predictor 31 + difference 5 = 36 -> 36 - 64
    /// = -28, which is the in-range value.
    #[test]
    fn mv_component_wraps_high() {
        assert_eq!(reconstruct_mv_component(31, 5), 36 - 64);
        // Boundary: 31 + 1 = 32 -> -32.
        assert_eq!(reconstruct_mv_component(31, 1), -32);
    }

    /// Sum below -32 wraps up by 64.
    #[test]
    fn mv_component_wraps_low() {
        assert_eq!(reconstruct_mv_component(-32, -5), -37 + 64);
        // Boundary: -32 + (-1) = -33 -> 31.
        assert_eq!(reconstruct_mv_component(-32, -1), 31);
    }

    /// Every (predictor, difference) pair lands inside [-32, 31].
    #[test]
    fn mv_component_always_in_range() {
        for p in MV_HALF_MIN..=MV_HALF_MAX {
            for d in MV_HALF_MIN..=MV_HALF_MAX {
                let v = reconstruct_mv_component(p, d);
                assert!(
                    (MV_HALF_MIN..=MV_HALF_MAX).contains(&v),
                    "p={p} d={d} -> {v} out of range"
                );
            }
        }
    }

    /// `reconstruct_mv` applies the per-component rule.
    #[test]
    fn reconstruct_full_mv() {
        let pred = MotionVector::new(31, -32);
        let mvd = Mvd {
            dx_half: 1,
            dy_half: -1,
        };
        let mv = reconstruct_mv(pred, mvd);
        assert_eq!(mv, MotionVector::new(-32, 31));
    }

    // ---- Annex D §D.2 UMV component reconstruction ---------------

    /// §D.2: predictor inside [-31, 32] uses the first column directly
    /// (no wrap), so the component can leave the default [-32, 31]
    /// window and reach the extended [-63, 63] range.
    #[test]
    fn umv_predictor_in_range_no_wrap() {
        // Default mode would wrap 32+31=63 down to -1; UMV keeps it.
        assert_eq!(reconstruct_mv_component_umv(32, 31), 63);
        // Symmetric low end: -31 + (-32) = -63 (in range, no wrap).
        assert_eq!(reconstruct_mv_component_umv(-31, -32), -63);
        // Ordinary interior sum is unchanged.
        assert_eq!(reconstruct_mv_component_umv(4, 6), 10);
        assert_eq!(reconstruct_mv_component_umv(0, 0), 0);
        // Boundary predictors of the "first column" range.
        assert_eq!(reconstruct_mv_component_umv(32, 0), 32);
        assert_eq!(reconstruct_mv_component_umv(-31, 0), -31);
    }

    /// §D.2: predictor below -31 forces a non-positive component with
    /// the same (negative) sign as the predictor. The pair member that
    /// satisfies `-63 ≤ MVc ≤ 0` is selected.
    #[test]
    fn umv_predictor_below_range_selects_nonpositive() {
        // Pc = -40. difference = 31 (Vector column). The pair is
        // {31, 31-64=-33}. Pc+31 = -9 (in [-63,0]) — selected.
        assert_eq!(reconstruct_mv_component_umv(-40, 31), -9);
        // difference = -32: pair {-32, -32+64=32}. Pc-32 = -72 (out),
        // Pc+32 = -8 (in [-63,0]) — selected.
        assert_eq!(reconstruct_mv_component_umv(-40, -32), -8);
        // Result always non-positive and in range for every difference.
        for d in -32..=31 {
            let v = reconstruct_mv_component_umv(-50, d);
            assert!(
                (MV_UMV_HALF_MIN..=0).contains(&v),
                "Pc=-50 d={d} -> {v} not in [-63, 0]"
            );
        }
    }

    /// §D.2: predictor above 32 forces a non-negative component with
    /// the same (positive) sign as the predictor.
    #[test]
    fn umv_predictor_above_range_selects_nonnegative() {
        // Pc = 40. difference = -32: pair {-32, 32}. Pc-32 = 8 (in
        // [0,63]) — selected; Pc+(-32)=8 actually, let the impl decide.
        let v = reconstruct_mv_component_umv(40, -32);
        assert!((0..=MV_UMV_HALF_MAX).contains(&v), "{v} not in [0,63]");
        // Result always non-negative and in range for every difference.
        for d in -32..=31 {
            let v = reconstruct_mv_component_umv(50, d);
            assert!(
                (0..=MV_UMV_HALF_MAX).contains(&v),
                "Pc=50 d={d} -> {v} not in [0, 63]"
            );
        }
    }

    /// Every (predictor, difference) pair in the UMV space yields a
    /// component inside the extended [-63, 63] window.
    #[test]
    fn umv_component_always_in_extended_range() {
        for p in MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX {
            for d in MV_HALF_MIN..=MV_HALF_MAX {
                let v = reconstruct_mv_component_umv(p, d);
                assert!(
                    (MV_UMV_HALF_MIN..=MV_UMV_HALF_MAX).contains(&v),
                    "p={p} d={d} -> {v} out of extended range"
                );
            }
        }
    }

    /// `reconstruct_mv_umv` applies the §D.2 rule per component.
    #[test]
    fn umv_full_vector() {
        // dx: predictor 32 (in range) + 31 -> 63 (no wrap).
        // dy: predictor -50 (below range) + 0 -> 0.
        let pred = MotionVector::new(32, -50);
        let mvd = Mvd {
            dx_half: 31,
            dy_half: 0,
        };
        let mv = reconstruct_mv_umv(pred, mvd);
        assert_eq!(mv.dx_half, 63);
        assert!((MV_UMV_HALF_MIN..=0).contains(&mv.dy_half));
    }

    /// In the predictor "first column" range, UMV and the default
    /// reconstruction agree whenever the default sum does not wrap.
    #[test]
    fn umv_matches_default_when_no_wrap() {
        for p in -31..=32 {
            for d in MV_HALF_MIN..=MV_HALF_MAX {
                let sum = p + d;
                if (MV_HALF_MIN..=MV_HALF_MAX).contains(&sum) {
                    assert_eq!(
                        reconstruct_mv_component_umv(p, d),
                        reconstruct_mv_component(p, d),
                        "p={p} d={d}"
                    );
                }
            }
        }
    }

    // ---- §6.1.1 median predictor ---------------------------------

    #[test]
    fn median3_basic() {
        assert_eq!(median3(1, 2, 3), 2);
        assert_eq!(median3(3, 1, 2), 2);
        assert_eq!(median3(-5, 0, 5), 0);
        assert_eq!(median3(7, 7, 1), 7);
        assert_eq!(median3(4, 4, 4), 4);
    }

    #[test]
    fn predict_median_per_component() {
        let m = predict_mv_median(
            MotionVector::new(1, -3),
            MotionVector::new(5, -1),
            MotionVector::new(3, -2),
        );
        assert_eq!(m, MotionVector::new(3, -2));
    }

    // ---- Table 18 chroma vector derivation -----------------------

    /// Quarter-pel position 0 (luma even half-pel that is a whole even
    /// pixel) -> chroma whole pixel, no fractional part.
    #[test]
    fn chroma_zero_fraction() {
        // luma 0 half-pel -> chroma 0
        assert_eq!(chroma_mv_component(0), 0);
        // luma 4 half-pel = 2 full luma pixels -> chroma 1 full pixel
        // = 2 chroma half-pel.
        assert_eq!(chroma_mv_component(4), 2);
        assert_eq!(chroma_mv_component(-4), -2);
        // luma 8 half-pel = 4 luma pixels -> chroma 2 pixels = 4 half-pel.
        assert_eq!(chroma_mv_component(8), 4);
    }

    /// Table 18: quarter-pel 1/4, 1/2, 3/4 all collapse to 1/2
    /// (one chroma half-pel step) plus the integer part.
    #[test]
    fn chroma_fraction_snaps_to_half() {
        // luma 1 half-pel: mag=1 -> full=0, rem=1 -> 0*2 + 1 = 1.
        assert_eq!(chroma_mv_component(1), 1);
        // luma 2 half-pel: mag=2 -> full=0, rem=2 -> 1.
        assert_eq!(chroma_mv_component(2), 1);
        // luma 3 half-pel: mag=3 -> full=0, rem=3 -> 1.
        assert_eq!(chroma_mv_component(3), 1);
        // luma 5 half-pel: mag=5 -> full=1, rem=1 -> 2+1 = 3.
        assert_eq!(chroma_mv_component(5), 3);
        // luma 6 half-pel: mag=6 -> full=1, rem=2 -> 3.
        assert_eq!(chroma_mv_component(6), 3);
        // luma 7 half-pel: mag=7 -> full=1, rem=3 -> 3.
        assert_eq!(chroma_mv_component(7), 3);
        // negatives mirror.
        assert_eq!(chroma_mv_component(-1), -1);
        assert_eq!(chroma_mv_component(-7), -3);
    }

    #[test]
    fn chroma_full_vector() {
        let c = chroma_mv(MotionVector::new(4, -3));
        assert_eq!(c, MotionVector::new(2, -1));
    }

    // ---- §6.1.2 half-pel interpolation ---------------------------

    fn plane_4x4() -> Vec<u8> {
        // A simple ascending plane:
        //   0  1  2  3
        //  10 11 12 13
        //  20 21 22 23
        //  30 31 32 33
        vec![
            0, 1, 2, 3, //
            10, 11, 12, 13, //
            20, 21, 22, 23, //
            30, 31, 32, 33,
        ]
    }

    /// Integer (full-pel) position returns the pixel itself.
    #[test]
    fn half_pel_integer_position() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Integer pixel (1, 2) -> half position (2, 4).
        assert_eq!(sample_half_pel(&p, 2, 4, 0), 21);
        assert_eq!(sample_half_pel(&p, 0, 0, 0), 0);
    }

    /// Horizontal half-pel `b = (A + B + 1) / 2` with RCONTROL=0.
    #[test]
    fn half_pel_horizontal() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Between pixel (0,0)=0 and (1,0)=1: half x at hx=1,hy=0.
        // b = (0 + 1 + 1 - 0) / 2 = 1.
        assert_eq!(sample_half_pel(&p, 1, 0, 0), 1);
        // Between (1,1)=11 and (2,1)=12: hx=3,hy=2.
        // b = (11 + 12 + 1) / 2 = 12.
        assert_eq!(sample_half_pel(&p, 3, 2, 0), 12);
    }

    /// RCONTROL=1 biases rounding downward: (0+1+1-1)/2 = 0.
    #[test]
    fn half_pel_horizontal_rcontrol1() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        assert_eq!(sample_half_pel(&p, 1, 0, 1), 0);
    }

    /// Vertical half-pel `c = (A + C + 1) / 2`.
    #[test]
    fn half_pel_vertical() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Between (0,0)=0 and (0,1)=10: hx=0,hy=1.
        // c = (0 + 10 + 1) / 2 = 5.
        assert_eq!(sample_half_pel(&p, 0, 1, 0), 5);
    }

    /// Diagonal half-pel `d = (A + B + C + D + 2) / 4`.
    #[test]
    fn half_pel_diagonal() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Around (0,0): A=0, B=1, C=10, D=11. hx=1,hy=1.
        // d = (0 + 1 + 10 + 11 + 2) / 4 = 24 / 4 = 6.
        assert_eq!(sample_half_pel(&p, 1, 1, 0), 6);
    }

    /// Edge replication: sampling outside the plane clamps to the
    /// nearest edge pixel (§D.1).
    #[test]
    fn half_pel_edge_replication() {
        let buf = plane_4x4();
        let p = RefPlane::new(&buf, 4, 4);
        // Negative integer position clamps to (0,0)=0.
        assert_eq!(sample_half_pel(&p, -10, -10, 0), 0);
        // Far past the right/bottom clamps to (3,3)=33.
        assert_eq!(sample_half_pel(&p, 100, 100, 0), 33);
        // Half position straddling the right edge: integer x=3 (last
        // col), B clamps to same column -> b = (3 + 3 + 1)/2 = 3.
        assert_eq!(sample_half_pel(&p, 7, 0, 0), 3);
    }

    // ---- §6.1 block-level motion compensation --------------------

    /// Zero vector against a flat reference returns that flat block.
    #[test]
    fn mc_block_zero_vector_flat() {
        let buf = vec![128u8; 16 * 16];
        let p = RefPlane::new(&buf, 16, 16);
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(0, 0), 0);
        assert!(pred.iter().all(|&v| v == 128));
    }

    /// Integer-pel vector shifts the source window; a full-pel right
    /// shift of 2 (dx_half = 4) at block (0,0) reads pixels starting
    /// at column 2.
    #[test]
    fn mc_block_integer_shift() {
        // 16x8 ramp where pixel value == column.
        let mut buf = vec![0u8; 16 * 8];
        for y in 0..8 {
            for x in 0..16 {
                buf[y * 16 + x] = x as u8;
            }
        }
        let p = RefPlane::new(&buf, 16, 8);
        // dx_half = 4 -> +2 px. Block (0,0) pixel (0,0) reads col 2.
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(4, 0), 0);
        // Row 0 of the prediction should read columns 2..10.
        for (x, &px) in pred.iter().take(BLOCK_DIM).enumerate() {
            assert_eq!(px, (x + 2) as u8, "col {x}");
        }
    }

    /// Half-pel right shift averages adjacent columns.
    #[test]
    fn mc_block_half_pel_shift() {
        // Reference: even columns = 0, odd columns = 100.
        let mut buf = vec![0u8; 16 * 8];
        for y in 0..8 {
            for x in 0..16 {
                buf[y * 16 + x] = if x % 2 == 0 { 0 } else { 100 };
            }
        }
        let p = RefPlane::new(&buf, 16, 8);
        // dx_half = 1 -> +0.5 px. Pixel (0,0): A=col0=0, B=col1=100.
        // b = (0 + 100 + 1) / 2 = 50.
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(1, 0), 0);
        assert_eq!(pred[0], 50);
        // Pixel (1,0): A=col1=100, B=col2=0 -> (100+0+1)/2 = 50.
        assert_eq!(pred[1], 50);
    }

    // ---- §6.3.1 + §6.3.2 INTER summation -------------------------

    /// Prediction + zero residual returns the prediction.
    #[test]
    fn inter_recon_zero_residual() {
        let pred = [100u8; COEFFS_PER_BLOCK];
        let res = [0i16; COEFFS_PER_BLOCK];
        let rec = reconstruct_inter_block(&pred, &res);
        assert!(rec.iter().all(|&v| v == 100));
    }

    /// Residual adds to prediction pixel-by-pixel.
    #[test]
    fn inter_recon_adds_residual() {
        let mut pred = [50u8; COEFFS_PER_BLOCK];
        let mut res = [0i16; COEFFS_PER_BLOCK];
        pred[0] = 50;
        res[0] = 30;
        res[1] = -20;
        let rec = reconstruct_inter_block(&pred, &res);
        assert_eq!(rec[0], 80);
        assert_eq!(rec[1], 30);
    }

    /// §6.3.2 clipping at both extremes.
    #[test]
    fn inter_recon_clips_to_picture_range() {
        let mut pred = [0u8; COEFFS_PER_BLOCK];
        let mut res = [0i16; COEFFS_PER_BLOCK];
        // pixel 0: prediction 250 + residual 50 = 300 -> 255.
        pred[0] = 250;
        res[0] = 50;
        // pixel 1: prediction 10 + residual -200 = -190 -> 0.
        pred[1] = 10;
        res[1] = -200;
        let rec = reconstruct_inter_block(&pred, &res);
        assert_eq!(rec[0], 255);
        assert_eq!(rec[1], 0);
    }

    /// End-to-end: a flat reference + a half-pel vector + a small DC
    /// residual reconstructs a uniformly shifted, biased block.
    #[test]
    fn inter_end_to_end_flat_plus_dc() {
        let buf = vec![100u8; 16 * 16];
        let p = RefPlane::new(&buf, 16, 16);
        // Half-pel vector on a flat plane is still 100 everywhere.
        let pred = motion_compensate_block(&p, 0, 0, MotionVector::new(1, 1), 0);
        assert!(pred.iter().all(|&v| v == 100));
        // Add a uniform +20 residual.
        let res = [20i16; COEFFS_PER_BLOCK];
        let rec = reconstruct_inter_block(&pred, &res);
        assert!(rec.iter().all(|&v| v == 120));
    }

    // ---- Annex F §F.2 / Figure F.1 four-MV candidate selection ----

    /// Helper: build a distinctive 4-MV array so candidate-selection
    /// tests can tell which block's vector was picked.
    fn distinctive_mb(tag: i32) -> Mb4Mv {
        [
            MotionVector::new(tag * 10 + 1, tag * 10 + 1),
            MotionVector::new(tag * 10 + 2, tag * 10 + 2),
            MotionVector::new(tag * 10 + 3, tag * 10 + 3),
            MotionVector::new(tag * 10 + 4, tag * 10 + 4),
        ]
    }

    /// `LumaBlockIndex::ALL` is exactly `[B1, B2, B3, B4]` and
    /// `index()` / `from_index()` round-trip.
    #[test]
    fn luma_block_index_round_trip() {
        assert_eq!(LumaBlockIndex::ALL.len(), 4);
        for (i, blk) in LumaBlockIndex::ALL.iter().enumerate() {
            assert_eq!(blk.index(), i);
            assert_eq!(LumaBlockIndex::from_index(i), Some(*blk));
        }
        assert_eq!(LumaBlockIndex::from_index(4), None);
    }

    /// B1 with every external neighbour `None` (picture top-left
    /// corner) gives `(0, 0, 0)` — every candidate is zero per the
    /// §6.1.1 default rule.
    #[test]
    fn select_4mv_b1_isolated_is_all_zero() {
        let n = Mb4MvNeighbourhood::isolated(distinctive_mb(0));
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B1, &n);
        assert_eq!(mv1, MotionVector::default());
        assert_eq!(mv2, MotionVector::default());
        assert_eq!(mv3, MotionVector::default());
    }

    /// B1 with MB-left present: MV1 = B2 of MB-left; MV2 / MV3 stay
    /// zero (no MB above).
    #[test]
    fn select_4mv_b1_left_only() {
        let n = Mb4MvNeighbourhood {
            current: distinctive_mb(1),
            left: Some(distinctive_mb(2)),
            above: None,
            above_right: None,
            right: None,
        };
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B1, &n);
        // B2 of MB-left has tag*10+2 = 22.
        assert_eq!(mv1, MotionVector::new(22, 22));
        assert_eq!(mv2, MotionVector::default());
        assert_eq!(mv3, MotionVector::default());
    }

    /// B1 with MB-above present: MV2 = B3 of MB-above, MV3 = B4 of
    /// MB-above; MV1 stays zero (no MB-left).
    #[test]
    fn select_4mv_b1_above_only() {
        let n = Mb4MvNeighbourhood {
            current: distinctive_mb(1),
            left: None,
            above: Some(distinctive_mb(3)),
            above_right: None,
            right: None,
        };
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B1, &n);
        assert_eq!(mv1, MotionVector::default());
        // B3 of MB-above (tag 3) = 33; B4 of MB-above = 34.
        assert_eq!(mv2, MotionVector::new(33, 33));
        assert_eq!(mv3, MotionVector::new(34, 34));
    }

    /// B2 with every neighbour present: MV1 = B1 of current, MV2 = B4
    /// of MB-above, MV3 = B3 of MB-above-right.
    #[test]
    fn select_4mv_b2_full_neighbourhood() {
        let n = Mb4MvNeighbourhood {
            current: distinctive_mb(1),
            left: Some(distinctive_mb(2)),
            above: Some(distinctive_mb(3)),
            above_right: Some(distinctive_mb(4)),
            right: Some(distinctive_mb(5)),
        };
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B2, &n);
        // B1 of current = 11; B4 of MB-above = 34; B3 of above-right = 43.
        assert_eq!(mv1, MotionVector::new(11, 11));
        assert_eq!(mv2, MotionVector::new(34, 34));
        assert_eq!(mv3, MotionVector::new(43, 43));
    }

    /// B3 lookups are entirely inside the current MB except for MV1
    /// which reads B4 of MB-left.
    #[test]
    fn select_4mv_b3_full_neighbourhood() {
        let n = Mb4MvNeighbourhood {
            current: distinctive_mb(1),
            left: Some(distinctive_mb(2)),
            above: Some(distinctive_mb(3)),
            above_right: Some(distinctive_mb(4)),
            right: Some(distinctive_mb(5)),
        };
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B3, &n);
        // B4 of MB-left = 24; B1 of current = 11; B2 of current = 12.
        assert_eq!(mv1, MotionVector::new(24, 24));
        assert_eq!(mv2, MotionVector::new(11, 11));
        assert_eq!(mv3, MotionVector::new(12, 12));
    }

    /// B4's MV3 reads MB-right's B1. With MB-right absent (right
    /// picture edge), MV3 falls back to zero per the §6.1.1 rule-4
    /// default; MV1 = current B3, MV2 = current B2 stay intra-MB.
    #[test]
    fn select_4mv_b4_right_edge_mv3_zero() {
        let n = Mb4MvNeighbourhood {
            current: distinctive_mb(1),
            left: Some(distinctive_mb(2)),
            above: Some(distinctive_mb(3)),
            above_right: Some(distinctive_mb(4)),
            right: None,
        };
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B4, &n);
        // B3 of current = 13; B2 of current = 12; MV3 = zero.
        assert_eq!(mv1, MotionVector::new(13, 13));
        assert_eq!(mv2, MotionVector::new(12, 12));
        assert_eq!(mv3, MotionVector::default());
    }

    /// B4 with MB-right present: MV3 = B1 of MB-right.
    #[test]
    fn select_4mv_b4_right_present() {
        let n = Mb4MvNeighbourhood {
            current: distinctive_mb(1),
            left: None,
            above: None,
            above_right: None,
            right: Some(distinctive_mb(5)),
        };
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B4, &n);
        // B3 of current = 13; B2 of current = 12; B1 of MB-right = 51.
        assert_eq!(mv1, MotionVector::new(13, 13));
        assert_eq!(mv2, MotionVector::new(12, 12));
        assert_eq!(mv3, MotionVector::new(51, 51));
    }

    /// §F.2 last paragraph: "if only one motion vector per macroblock
    /// is present, this is defined as four vectors with the same
    /// value." When a one-MV macroblock is treated as a uniform 4-MV
    /// MB, the §F.2 candidate-predictor selection collapses to the
    /// single-vector Figure-12 candidates: MV1 of B1 = the single MV
    /// of MB-left; MV2 of B1 = the single MV of MB-above; etc.
    #[test]
    fn one_mv_neighbour_uniform_4mv_equivalence() {
        let single_current = MotionVector::new(5, -3);
        let single_left = MotionVector::new(-2, 4);
        let single_above = MotionVector::new(7, 7);
        let single_above_right = MotionVector::new(-9, 0);
        let single_right = MotionVector::new(1, 1);
        let n = Mb4MvNeighbourhood {
            current: [single_current; 4],
            left: Some([single_left; 4]),
            above: Some([single_above; 4]),
            above_right: Some([single_above_right; 4]),
            right: Some([single_right; 4]),
        };
        // B1: MV1 should equal the single MV of MB-left, MV2/MV3 the
        // single MV of MB-above (Figure-12 single-MV layout).
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B1, &n);
        assert_eq!(mv1, single_left);
        assert_eq!(mv2, single_above);
        assert_eq!(mv3, single_above);
        // B2: MV1 = single of current, MV2 = single of above, MV3 =
        // single of above-right (= Figure-12 for the MB's single MV).
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B2, &n);
        assert_eq!(mv1, single_current);
        assert_eq!(mv2, single_above);
        assert_eq!(mv3, single_above_right);
        // B3: every candidate inside current MB / MB-left, so all
        // candidates equal the per-MB single vector.
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B3, &n);
        assert_eq!(mv1, single_left);
        assert_eq!(mv2, single_current);
        assert_eq!(mv3, single_current);
        // B4: same — MV3 picks up MB-right's single vector.
        let (mv1, mv2, mv3) = select_4mv_candidates(LumaBlockIndex::B4, &n);
        assert_eq!(mv1, single_current);
        assert_eq!(mv2, single_current);
        assert_eq!(mv3, single_right);
    }

    /// End-to-end: the §F.2 median predictor is just
    /// `predict_mv_median` applied to [`select_4mv_candidates`]'s
    /// output. Picking a uniform-everywhere neighbourhood gives the
    /// median of (5,5,5) = 5 for every block.
    #[test]
    fn predict_mv_median_from_4mv_candidates() {
        let v = MotionVector::new(5, -7);
        let n = Mb4MvNeighbourhood {
            current: [v; 4],
            left: Some([v; 4]),
            above: Some([v; 4]),
            above_right: Some([v; 4]),
            right: Some([v; 4]),
        };
        for blk in LumaBlockIndex::ALL {
            let (mv1, mv2, mv3) = select_4mv_candidates(blk, &n);
            assert_eq!(predict_mv_median(mv1, mv2, mv3), v, "block {blk:?}");
        }
    }

    // ---- Annex F §F.2 / Table F.1 chroma vector derivation --------

    /// Table F.1 transcription: every position `0..=15` snaps to the
    /// documented half-pel position.
    #[test]
    fn chroma_4mv_table_f1_transcription() {
        // Sum of four luma vectors == sixteenth-pel position directly
        // in our integer half-pel arithmetic (4 components of equal
        // half-pel each contribute exactly that many sixteenths to
        // the chroma vector, since chroma is half-resolution).
        let expected = [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2];
        for (sixteenth, &want) in expected.iter().enumerate() {
            let got = chroma_mv_component_4mv(sixteenth as i32);
            assert_eq!(got, want, "sixteenth={sixteenth}");
        }
    }

    /// All four luma MVs zero -> chroma MV zero.
    #[test]
    fn chroma_4mv_all_zero() {
        let mvs: Mb4Mv = [MotionVector::default(); 4];
        assert_eq!(chroma_mv_4mv(&mvs), MotionVector::default());
    }

    /// Four identical full-pel luma MVs collapse to the §6.1.1
    /// single-MV chroma rule: each component sums to 4·v, divided
    /// by 8 = v/2 (in luma half-pel terms ↔ chroma half-pel terms).
    /// For an integer luma full-pel offset (dx_half = 2k), this gives
    /// the same chroma half-pel as `chroma_mv_component` does for the
    /// single value `2k`.
    #[test]
    fn chroma_4mv_uniform_matches_single_mv_rule() {
        // Pick a few integer-pixel offsets (dx_half multiples of 2).
        for k in -8..=8 {
            let luma_half = 2 * k; // an even half-pel = full-pixel offset
            let mvs: Mb4Mv = [MotionVector::new(luma_half, 0); 4];
            let chroma = chroma_mv_4mv(&mvs);
            let want_single = super::chroma_mv_component(luma_half);
            assert_eq!(chroma.dx_half, want_single, "k={k}");
            assert_eq!(chroma.dy_half, 0);
        }
    }

    /// A four-vector sum of `+4` luma half-pel (each MV +1 half-pel)
    /// gives sixteenth position 4 → Table F.1 → half-position 1
    /// (one chroma half-pel step). Sign is preserved on the negative
    /// mirror.
    #[test]
    fn chroma_4mv_sixteenth_snap_positive_and_negative() {
        let mvs_pos: Mb4Mv = [
            MotionVector::new(1, 0),
            MotionVector::new(1, 0),
            MotionVector::new(1, 0),
            MotionVector::new(1, 0),
        ];
        assert_eq!(chroma_mv_4mv(&mvs_pos), MotionVector::new(1, 0));
        let mvs_neg: Mb4Mv = [
            MotionVector::new(-1, 0),
            MotionVector::new(-1, 0),
            MotionVector::new(-1, 0),
            MotionVector::new(-1, 0),
        ];
        assert_eq!(chroma_mv_4mv(&mvs_neg), MotionVector::new(-1, 0));
    }

    /// A four-vector sum that crosses a chroma-pixel boundary: each
    /// MV +8 half-pel = +4 luma pixels per block, sum = 32 luma
    /// half-pel = 2 chroma pixels = 4 chroma half-pel. Pure integer
    /// pixel result, no Table F.1 fractional contribution.
    #[test]
    fn chroma_4mv_full_pixel_integer_result() {
        let mvs: Mb4Mv = [MotionVector::new(8, 16); 4]; // +4 px / +8 px
        let chroma = chroma_mv_4mv(&mvs);
        // sum_x = 32 luma half-pel -> chroma x = 32/16 = 2 chroma
        // pixels = 4 chroma half-pel. sum_y = 64 -> 8 chroma half-pel.
        assert_eq!(chroma.dx_half, 4);
        assert_eq!(chroma.dy_half, 8);
    }

    /// Table F.1 asymmetry round-trip: position 2 → 0 (so chroma is
    /// a whole chroma pixel) but position 3 → 1 (so chroma snaps up
    /// to a chroma half-pel). The transition happens at the spec
    /// boundary, not at 8.
    #[test]
    fn chroma_4mv_table_f1_low_boundary_asymmetry() {
        // sixteenth = 2 -> half = 0.
        assert_eq!(chroma_mv_component_4mv(2), 0);
        // sixteenth = 3 -> half = 1.
        assert_eq!(chroma_mv_component_4mv(3), 1);
        // sixteenth = 13 -> half = 1.
        assert_eq!(chroma_mv_component_4mv(13), 1);
        // sixteenth = 14 -> half = 2 (= one chroma pixel).
        assert_eq!(chroma_mv_component_4mv(14), 2);
        // Negative mirror of each.
        assert_eq!(chroma_mv_component_4mv(-2), 0);
        assert_eq!(chroma_mv_component_4mv(-3), -1);
        assert_eq!(chroma_mv_component_4mv(-13), -1);
        assert_eq!(chroma_mv_component_4mv(-14), -2);
    }

    /// Chroma derivation is always in the half-pel chroma range
    /// implied by the luma-pixel UMV range: every (mv1..mv4) sum in
    /// the default `[-32, 31]` half-pel range produces a chroma
    /// component bounded by `|sum| / 16 * 2 + 2`.
    #[test]
    fn chroma_4mv_bounded_sweep() {
        // Sweep four-vector sums across the full single-MV span; the
        // chroma magnitude is always at most |sum|·2/16 + 2.
        for sum in -200..=200 {
            let got = chroma_mv_component_4mv(sum).unsigned_abs() as i32;
            let bound = sum.unsigned_abs() as i32 * 2 / 16 + 2;
            assert!(got <= bound, "sum={sum} got={got} bound={bound}");
        }
    }
}
