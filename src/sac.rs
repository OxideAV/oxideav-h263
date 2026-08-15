//! Annex E — **Syntax-based Arithmetic Coding (SAC) mode**.
//!
//! In SAC mode every variable-length coded symbol of the macroblock and
//! block layers is replaced by an arithmetic-coded symbol (§E.1); the
//! picture / GOB header layers stay fixed-length strings sent straight
//! through the bit FIFO (§E.6). This module stages the four spec
//! components:
//!
//! * the **arithmetic decoder** (§E.3: `decode_a_symbol` +
//!   `decoder_reset`) as [`SacDecoder`],
//! * the **arithmetic encoder** (§E.2: `encode_a_symbol`, §E.6:
//!   `encoder_flush`) as [`SacEncoder`],
//! * the **PSC_FIFO stuffing rule** (§E.5: a `"1"` is stuffed after
//!   each run of 14 `"0"`s that is not part of a PSC / GBSC, and the
//!   decoder deletes the first `"1"` after each exactly-14-zero run —
//!   a 15th zero instead signals a legal start code), folded into the
//!   bit source / sink of the two coders,
//! * the **§E.8 models** (`cumul_freq` arrays) with the §E.7 model
//!   assignment, exposed through per-syntax-element codec helpers
//!   ([`decode_cod`] / [`encode_cod`], MCBPC, CBPY, DQUANT, MVD,
//!   INTRADC and the [`parse_block_sac`] / [`write_block_sac`] TCOEF
//!   block layer with its TCOEF1/2/3/r position-dependent models,
//!   SIGN symbol and ESCAPE `LAST`/`RUN`/`LEVEL` models).
//!
//! Symbol indexing follows §E.7: "the indices as given in the VLC
//! tables of clause 5 are used for the indexing of the integers in the
//! models" — Table 7 / Table 8 for MCBPC, Table 12 for CBPY, Table 13
//! for DQUANT, Table 14 for MVD (index 32 = difference 0), Table 15
//! for INTRADC, Table 16 for the TCOEF symbols (ESCAPE = index 102)
//! and Table 17 for the post-ESCAPE RUN / LEVEL fixed codes.
//!
//! §5.1.4.6 interaction restrictions: SAC shall not be combined with
//! Alternative INTER VLC (Annex S) or Modified Quantization (Annex T),
//! and — when PLUSPTYPE is present — not with Unrestricted Motion
//! Vector mode either. The picture drivers enforce these.

use crate::block::{tcoef_table_index, tcoef_table_row, H263Block, TCOEF_ESCAPE_INDEX};
use crate::macroblock::MbType;
use crate::{Error, Result};
use oxideav_core::bits::{BitReader, BitWriter};

// §E.2 — the interval-register constants.
const Q1: i64 = 16384;
const Q2: i64 = 32768;
const Q3: i64 = 49152;
const TOP: i64 = 65535;

/// §E.5 — length of the zero run after which a stuffing `"1"` is
/// inserted (encoder) / deleted (decoder).
const STUFF_RUN: u32 = 14;

// ---------------------------------------------------------------------
// §E.8 models. Each array is a cumulative-frequency table: entry 0 is
// the total (16383), the probability of symbol `i` is
// `cumf[i] - cumf[i + 1]`, and the final entry is 0.
// ---------------------------------------------------------------------

/// Model for COD in P-pictures (index 0 = COD "0" / coded).
pub(crate) static CUMF_COD: [i64; 3] = [16383, 6849, 0];

/// Model for MCBPC in P-pictures, Table 8 indexing (index 20 =
/// stuffing; the type-5 rows 21..24 need PLUSPTYPE + AP/DF and use
/// the separate 4MVQ model, which is not staged).
pub(crate) static CUMF_MCBPC_NO4MVQ: [i64; 22] = [
    16383, 4105, 3088, 2367, 1988, 1621, 1612, 1609, 1608, 496, 353, 195, 77, 22, 17, 12, 5, 4, 3,
    2, 1, 0,
];

/// Model for MCBPC in I-pictures, Table 7 indexing (index 8 = stuffing).
pub(crate) static CUMF_MCBPC_INTRA: [i64; 10] = [16383, 7410, 6549, 5188, 442, 182, 181, 141, 1, 0];

/// Model for CBPY in INTER macroblocks, Table 12 indexing.
pub(crate) static CUMF_CBPY: [i64; 17] = [
    16383, 14481, 13869, 13196, 12568, 11931, 11185, 10814, 9796, 9150, 8781, 7933, 6860, 6116,
    4873, 3538, 0,
];

/// Model for CBPY in INTRA macroblocks, Table 12 indexing.
pub(crate) static CUMF_CBPY_INTRA: [i64; 17] = [
    16383, 13619, 13211, 12933, 12562, 12395, 11913, 11783, 11004, 10782, 10689, 9928, 9353, 8945,
    8407, 7795, 0,
];

/// Model for DQUANT, Table 13 indexing (0 → −1, 1 → −2, 2 → +1, 3 → +2).
pub(crate) static CUMF_DQUANT: [i64; 5] = [16383, 12287, 8192, 4095, 0];

/// Model for MODB under Annex G PB-frames, Table 11 indexing (index
/// 0 = neither CBPB nor MVDB, 1 = MVDB only, 2 = CBPB + MVDB).
pub(crate) static CUMF_MODB_G: [i64; 4] = [16383, 6062, 2130, 0];

/// Model for the luminance CBPBn bits (n = 1..=4), index 0 for
/// CBPBn = 0 and 1 for CBPBn = 1 (§E.7).
pub(crate) static CUMF_YCBPB: [i64; 3] = [16383, 6062, 0];

/// Model for the chrominance CBPBn bits (n = 5, 6), same indexing.
pub(crate) static CUMF_UVCBPB: [i64; 3] = [16383, 491, 0];

/// Model for MVD / MVD2-4 / MVDB, Table 14 indexing (index 32 =
/// vector difference 0; index `i` = difference `(i − 32) / 2` pixels).
pub(crate) static CUMF_MVD: [i64; 65] = [
    16383, 16380, 16369, 16365, 16361, 16357, 16350, 16343, 16339, 16333, 16326, 16318, 16311,
    16306, 16298, 16291, 16283, 16272, 16261, 16249, 16235, 16222, 16207, 16175, 16141, 16094,
    16044, 15936, 15764, 15463, 14956, 13924, 11491, 4621, 2264, 1315, 854, 583, 420, 326, 273,
    229, 196, 166, 148, 137, 123, 114, 101, 91, 82, 76, 66, 59, 53, 46, 36, 30, 26, 24, 18, 14, 10,
    5, 0,
];

/// Model for INTRADC, Table 15 indexing (index 0..=126 → FLC codes
/// 1..=127, index 127 → FLC 255 / level 1024, index 128..=253 → FLC
/// codes 129..=254).
pub(crate) static CUMF_INTRADC: [i64; 255] = [
    16383, 16380, 16379, 16378, 16377, 16376, 16370, 16361, 16360, 16359, 16358, 16357, 16356,
    16355, 16343, 16238, 16237, 16236, 16230, 16221, 16220, 16205, 16190, 16169, 16151, 16130,
    16109, 16094, 16070, 16037, 16007, 15962, 15938, 15899, 15854, 15815, 15788, 15743, 15689,
    15656, 15617, 15560, 15473, 15404, 15296, 15178, 15106, 14992, 14868, 14738, 14593, 14438,
    14283, 14169, 14064, 14004, 13914, 13824, 13752, 13671, 13590, 13515, 13458, 13380, 13305,
    13230, 13143, 13025, 12935, 12878, 12794, 12743, 12656, 12596, 12521, 12443, 12359, 12278,
    12200, 12131, 12047, 12002, 11948, 11891, 11828, 11744, 11663, 11588, 11495, 11402, 11288,
    11204, 11126, 11039, 10961, 10883, 10787, 10679, 10583, 10481, 10360, 10227, 10113, 9961, 9828,
    9717, 9584, 9485, 9324, 9112, 9019, 8908, 8766, 8584, 8426, 8211, 7920, 7663, 7406, 7152, 6904,
    6677, 6453, 6265, 6101, 5904, 5716, 5489, 5307, 5056, 4850, 4569, 4284, 3966, 3712, 3518, 3342,
    3206, 3048, 2909, 2773, 2668, 2596, 2512, 2370, 2295, 2232, 2166, 2103, 2022, 1956, 1887, 1830,
    1803, 1770, 1728, 1674, 1635, 1599, 1557, 1500, 1482, 1434, 1389, 1356, 1317, 1284, 1245, 1200,
    1179, 1140, 1110, 1092, 1062, 1044, 1035, 1014, 1008, 993, 981, 954, 936, 912, 894, 876, 864,
    849, 828, 816, 801, 792, 777, 756, 732, 690, 660, 642, 615, 597, 576, 555, 522, 489, 459, 435,
    411, 405, 396, 387, 375, 360, 354, 345, 344, 329, 314, 293, 278, 251, 236, 230, 224, 215, 214,
    208, 199, 193, 184, 178, 169, 154, 127, 100, 94, 73, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28,
    27, 26, 20, 19, 18, 17, 16, 15, 9, 0,
];

/// Model for the 1st TCOEF symbol of an INTER block, Table 16 indexing.
pub(crate) static CUMF_TCOEF1: [i64; 104] = [
    16383, 13455, 12458, 12079, 11885, 11800, 11738, 11700, 11681, 11661, 11651, 11645, 11641,
    10572, 10403, 10361, 10346, 10339, 10335, 9554, 9445, 9427, 9419, 9006, 8968, 8964, 8643, 8627,
    8624, 8369, 8354, 8352, 8200, 8192, 8191, 8039, 8036, 7920, 7917, 7800, 7793, 7730, 7727, 7674,
    7613, 7564, 7513, 7484, 7466, 7439, 7411, 7389, 7373, 7369, 7359, 7348, 7321, 7302, 7294, 5013,
    4819, 4789, 4096, 4073, 3373, 3064, 2674, 2357, 2177, 1975, 1798, 1618, 1517, 1421, 1303, 1194,
    1087, 1027, 960, 890, 819, 758, 707, 680, 656, 613, 566, 534, 505, 475, 465, 449, 430, 395,
    358, 335, 324, 303, 295, 286, 272, 233, 215, 0,
];

/// Model for the 2nd TCOEF symbol of an INTER block, Table 16 indexing.
pub(crate) static CUMF_TCOEF2: [i64; 104] = [
    16383, 13582, 12709, 12402, 12262, 12188, 12150, 12131, 12125, 12117, 12113, 12108, 12104,
    10567, 10180, 10070, 10019, 9998, 9987, 9158, 9037, 9010, 9005, 8404, 8323, 8312, 7813, 7743,
    7726, 7394, 7366, 7364, 7076, 7062, 7060, 6810, 6797, 6614, 6602, 6459, 6454, 6304, 6303, 6200,
    6121, 6059, 6012, 5973, 5928, 5893, 5871, 5847, 5823, 5809, 5796, 5781, 5771, 5763, 5752, 4754,
    4654, 4631, 3934, 3873, 3477, 3095, 2758, 2502, 2257, 2054, 1869, 1715, 1599, 1431, 1305, 1174,
    1059, 983, 901, 839, 777, 733, 683, 658, 606, 565, 526, 488, 456, 434, 408, 380, 361, 327, 310,
    296, 267, 259, 249, 239, 230, 221, 214, 0,
];

/// Model for the 3rd TCOEF symbol of an INTER block, Table 16 indexing.
pub(crate) static CUMF_TCOEF3: [i64; 104] = [
    16383, 13532, 12677, 12342, 12195, 12112, 12059, 12034, 12020, 12008, 12003, 12002, 12001,
    10586, 10297, 10224, 10202, 10195, 10191, 9223, 9046, 8999, 8987, 8275, 8148, 8113, 7552, 7483,
    7468, 7066, 7003, 6989, 6671, 6642, 6631, 6359, 6327, 6114, 6103, 5929, 5918, 5792, 5785, 5672,
    5580, 5507, 5461, 5414, 5382, 5354, 5330, 5312, 5288, 5273, 5261, 5247, 5235, 5227, 5219, 4357,
    4277, 4272, 3847, 3819, 3455, 3119, 2829, 2550, 2313, 2104, 1881, 1711, 1565, 1366, 1219, 1068,
    932, 866, 799, 750, 701, 662, 605, 559, 513, 471, 432, 403, 365, 336, 312, 290, 276, 266, 254,
    240, 228, 223, 216, 206, 199, 192, 189, 0,
];

/// Model for the 4th-and-later TCOEF symbols of an INTER block,
/// Table 16 indexing.
pub(crate) static CUMF_TCOEFR: [i64; 104] = [
    16383, 13216, 12233, 11931, 11822, 11776, 11758, 11748, 11743, 11742, 11741, 11740, 11739,
    10203, 9822, 9725, 9691, 9677, 9674, 8759, 8609, 8576, 8566, 7901, 7787, 7770, 7257, 7185,
    7168, 6716, 6653, 6639, 6276, 6229, 6220, 5888, 5845, 5600, 5567, 5348, 5327, 5160, 5142, 5004,
    4900, 4798, 4743, 4708, 4685, 4658, 4641, 4622, 4610, 4598, 4589, 4582, 4578, 4570, 4566, 3824,
    3757, 3748, 3360, 3338, 3068, 2835, 2592, 2359, 2179, 1984, 1804, 1614, 1445, 1234, 1068, 870,
    739, 668, 616, 566, 532, 489, 453, 426, 385, 357, 335, 316, 297, 283, 274, 266, 259, 251, 241,
    233, 226, 222, 217, 214, 211, 209, 208, 0,
];

/// Model for the 1st TCOEF symbol of an INTRA block, Table 16 indexing.
pub(crate) static CUMF_TCOEF1_INTRA: [i64; 104] = [
    16383, 13383, 11498, 10201, 9207, 8528, 8099, 7768, 7546, 7368, 7167, 6994, 6869, 6005, 5474,
    5220, 5084, 4964, 4862, 4672, 4591, 4570, 4543, 4397, 4337, 4326, 4272, 4240, 4239, 4212, 4196,
    4185, 4158, 4157, 4156, 4140, 4139, 4138, 4137, 4136, 4125, 4124, 4123, 4112, 4111, 4110, 4109,
    4108, 4107, 4106, 4105, 4104, 4103, 4102, 4101, 4100, 4099, 4098, 4097, 3043, 2897, 2843, 1974,
    1790, 1677, 1552, 1416, 1379, 1331, 1288, 1251, 1250, 1249, 1248, 1247, 1236, 1225, 1224, 1223,
    1212, 1201, 1200, 1199, 1198, 1197, 1196, 1195, 1194, 1193, 1192, 1191, 1190, 1189, 1188, 1187,
    1186, 1185, 1184, 1183, 1182, 1181, 1180, 1179, 0,
];

/// Model for the 2nd TCOEF symbol of an INTRA block, Table 16 indexing.
pub(crate) static CUMF_TCOEF2_INTRA: [i64; 104] = [
    16383, 13242, 11417, 10134, 9254, 8507, 8012, 7556, 7273, 7062, 6924, 6839, 6741, 6108, 5851,
    5785, 5719, 5687, 5655, 5028, 4917, 4864, 4845, 4416, 4159, 4074, 3903, 3871, 3870, 3765, 3752,
    3751, 3659, 3606, 3580, 3541, 3540, 3514, 3495, 3494, 3493, 3474, 3473, 3441, 3440, 3439, 3438,
    3425, 3424, 3423, 3422, 3421, 3420, 3401, 3400, 3399, 3398, 3397, 3396, 2530, 2419, 2360, 2241,
    2228, 2017, 1687, 1576, 1478, 1320, 1281, 1242, 1229, 1197, 1178, 1152, 1133, 1114, 1101, 1088,
    1087, 1086, 1085, 1072, 1071, 1070, 1069, 1068, 1067, 1066, 1065, 1064, 1063, 1062, 1061, 1060,
    1059, 1058, 1057, 1056, 1055, 1054, 1053, 1052, 0,
];

/// Model for the 3rd TCOEF symbol of an INTRA block, Table 16 indexing.
pub(crate) static CUMF_TCOEF3_INTRA: [i64; 104] = [
    16383, 12741, 10950, 10071, 9493, 9008, 8685, 8516, 8385, 8239, 8209, 8179, 8141, 6628, 5980,
    5634, 5503, 5396, 5327, 4857, 4642, 4550, 4481, 4235, 4166, 4151, 3967, 3922, 3907, 3676, 3500,
    3324, 3247, 3246, 3245, 3183, 3168, 3084, 3069, 3031, 3030, 3029, 3014, 3013, 2990, 2975, 2974,
    2973, 2958, 2943, 2928, 2927, 2926, 2925, 2924, 2923, 2922, 2921, 2920, 2397, 2298, 2283, 1891,
    1799, 1591, 1445, 1338, 1145, 1068, 1006, 791, 768, 661, 631, 630, 615, 592, 577, 576, 561,
    546, 523, 508, 493, 492, 491, 476, 475, 474, 473, 472, 471, 470, 469, 468, 453, 452, 451, 450,
    449, 448, 447, 446, 0,
];

/// Model for the 4th-and-later TCOEF symbols of an INTRA block,
/// Table 16 indexing.
pub(crate) static CUMF_TCOEFR_INTRA: [i64; 104] = [
    16383, 12514, 10776, 9969, 9579, 9306, 9168, 9082, 9032, 9000, 8981, 8962, 8952, 7630, 7212,
    7053, 6992, 6961, 6940, 6195, 5988, 5948, 5923, 5370, 5244, 5210, 4854, 4762, 4740, 4384, 4300,
    4288, 4020, 3968, 3964, 3752, 3668, 3511, 3483, 3354, 3322, 3205, 3183, 3108, 3046, 2999, 2981,
    2974, 2968, 2961, 2955, 2949, 2943, 2942, 2939, 2935, 2934, 2933, 2929, 2270, 2178, 2162, 1959,
    1946, 1780, 1651, 1524, 1400, 1289, 1133, 1037, 942, 849, 763, 711, 591, 521, 503, 496, 474,
    461, 449, 442, 436, 426, 417, 407, 394, 387, 377, 373, 370, 367, 366, 365, 364, 363, 362, 358,
    355, 352, 351, 350, 0,
];

/// Model for the SIGN symbol after a non-escaped TCOEF (index 0 =
/// positive, 1 = negative).
pub(crate) static CUMF_SIGN: [i64; 3] = [16383, 8416, 0];

/// Model for the post-ESCAPE LAST symbol in INTER blocks (index 0 =
/// LAST 0, 1 = LAST 1).
pub(crate) static CUMF_LAST: [i64; 3] = [16383, 9469, 0];

/// Model for the post-ESCAPE LAST symbol in INTRA blocks.
pub(crate) static CUMF_LAST_INTRA: [i64; 3] = [16383, 2820, 0];

/// Model for the post-ESCAPE RUN symbol in INTER blocks, Table 17
/// indexing (index = RUN).
pub(crate) static CUMF_RUN: [i64; 65] = [
    16383, 15310, 14702, 13022, 11883, 11234, 10612, 10192, 9516, 9016, 8623, 8366, 7595, 7068,
    6730, 6487, 6379, 6285, 6177, 6150, 6083, 5989, 5949, 5922, 5895, 5828, 5774, 5773, 5394, 5164,
    5016, 4569, 4366, 4136, 4015, 3867, 3773, 3692, 3611, 3476, 3341, 3301, 2787, 2503, 2219, 1989,
    1515, 1095, 934, 799, 691, 583, 435, 300, 246, 206, 125, 124, 97, 57, 30, 3, 2, 1, 0,
];

/// Model for the post-ESCAPE RUN symbol in INTRA blocks.
pub(crate) static CUMF_RUN_INTRA: [i64; 65] = [
    16383, 10884, 8242, 7124, 5173, 4745, 4246, 3984, 3034, 2749, 2607, 2298, 966, 681, 396, 349,
    302, 255, 254, 253, 206, 159, 158, 157, 156, 155, 154, 153, 106, 35, 34, 33, 32, 31, 30, 29,
    28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4,
    3, 2, 1, 0,
];

/// Model for the post-ESCAPE LEVEL symbol in INTER blocks, Table 17
/// indexing (index 0..=126 → LEVEL −127..−1, index 127..=253 →
/// LEVEL +1..+127).
pub(crate) static CUMF_LEVEL: [i64; 255] = [
    16383, 16382, 16381, 16380, 16379, 16378, 16377, 16376, 16375, 16374, 16373, 16372, 16371,
    16370, 16369, 16368, 16367, 16366, 16365, 16364, 16363, 16362, 16361, 16360, 16359, 16358,
    16357, 16356, 16355, 16354, 16353, 16352, 16351, 16350, 16349, 16348, 16347, 16346, 16345,
    16344, 16343, 16342, 16341, 16340, 16339, 16338, 16337, 16336, 16335, 16334, 16333, 16332,
    16331, 16330, 16329, 16328, 16327, 16326, 16325, 16324, 16323, 16322, 16321, 16320, 16319,
    16318, 16317, 16316, 16315, 16314, 16313, 16312, 16311, 16310, 16309, 16308, 16307, 16306,
    16305, 16304, 16303, 16302, 16301, 16300, 16299, 16298, 16297, 16296, 16295, 16294, 16293,
    16292, 16291, 16290, 16289, 16288, 16287, 16286, 16285, 16284, 16283, 16282, 16281, 16280,
    16279, 16278, 16277, 16250, 16223, 16222, 16195, 16154, 16153, 16071, 15989, 15880, 15879,
    15878, 15824, 15756, 15674, 15606, 15538, 15184, 14572, 13960, 10718, 7994, 5379, 2123, 1537,
    992, 693, 611, 516, 448, 421, 380, 353, 352, 284, 257, 230, 203, 162, 161, 160, 133, 132, 105,
    104, 103, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83,
    82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59,
    58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35,
    34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
    10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
];

/// Model for the post-ESCAPE LEVEL symbol in INTRA blocks.
pub(crate) static CUMF_LEVEL_INTRA: [i64; 255] = [
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
    4710, 4401, 4203, 3740, 3453, 3343, 3189, 2946, 2881, 2661, 2352, 2132, 1867, 1558, 1382, 1250,
    1162, 1097, 1032, 967, 835, 681, 549, 439, 351, 350, 307, 306, 305, 304, 303, 302, 301, 300,
    299, 298, 255, 212, 211, 210, 167, 166, 165, 164, 163, 162, 161, 160, 159, 158, 115, 114, 113,
    112, 111, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48,
    47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24,
    23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
];

// ---------------------------------------------------------------------
// §E.3 — arithmetic decoder.
// ---------------------------------------------------------------------

/// The §E.3 arithmetic decoder over a destuffed (§E.5) bit source.
///
/// Constructed at the first bit of a picture's arithmetic-coded
/// macroblock data (immediately after the fixed-length header string);
/// `new` performs the §E.3 `decoder_reset` (16-bit `code_value` load).
/// Bits past the end of the buffer read as `0` — after the encoder's
/// §E.6 flush, any continuation bits decode the already-encoded symbols
/// correctly, so the final symbols of a picture do not require bits
/// beyond the flush.
pub struct SacDecoder<'r, 'd> {
    reader: &'r mut BitReader<'d>,
    low: i64,
    high: i64,
    code_value: i64,
    /// §E.5 — length of the current run of `"0"`s seen by the
    /// destuffing filter.
    zero_run: u32,
}

impl core::fmt::Debug for SacDecoder<'_, '_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SacDecoder")
            .field("low", &self.low)
            .field("high", &self.high)
            .field("code_value", &self.code_value)
            .field("zero_run", &self.zero_run)
            .finish_non_exhaustive()
    }
}

impl<'r, 'd> SacDecoder<'r, 'd> {
    /// Initialise the decoder at the reader's current position —
    /// §E.3 `decoder_reset`: `code_value` is loaded with 16 bits.
    ///
    /// The §E.5 zero-run counter starts at 0; when the bits preceding
    /// the reader's position end in a run of zeros (a header tail —
    /// §E.5 counts runs over the whole FIFO stream, headers included),
    /// use [`SacDecoder::with_zero_run`] so the destuffing filter
    /// mirrors the encoder's stuffing decisions exactly.
    pub fn new(reader: &'r mut BitReader<'d>) -> Self {
        Self::with_zero_run(reader, 0)
    }

    /// As [`SacDecoder::new`], but seeding the §E.5 destuffing filter
    /// with the length of the zero run immediately preceding the
    /// reader's position (the trailing zeros of the fixed-length
    /// header string — e.g. a PQUANT low bit + CPM = 0 + PEI = 0
    /// tail). §E.5 stuffs after each 14-zero run of the *whole*
    /// stream, so a run that starts inside the header continues into
    /// the arithmetic data.
    pub fn with_zero_run(reader: &'r mut BitReader<'d>, zero_run: u32) -> Self {
        let mut dec = SacDecoder {
            reader,
            low: 0,
            high: TOP,
            code_value: 0,
            zero_run: zero_run.min(STUFF_RUN),
        };
        for _ in 0..16 {
            let bit = dec.next_bit();
            dec.code_value = 2 * dec.code_value + bit;
        }
        dec
    }

    /// Pull one destuffed bit from the source (§E.5): the first `"1"`
    /// after a run of exactly 14 `"0"`s is a stuffing bit and is
    /// deleted; a 15th `"0"` instead marks a legal start code (whose
    /// zeros pass through unmodified — by then the arithmetic data is
    /// complete and these are dead lookahead bits). End of buffer
    /// reads as `0`.
    fn next_bit(&mut self) -> i64 {
        loop {
            let bit = match self.reader.read_bit() {
                Ok(b) => b,
                // Past the §E.6 flush any bit value decodes the
                // already-coded symbols correctly.
                Err(_) => return 0,
            };
            if bit {
                if self.zero_run == STUFF_RUN {
                    // §E.5 — delete the stuffing "1".
                    self.zero_run = 0;
                    continue;
                }
                self.zero_run = 0;
                return 1;
            }
            self.zero_run += 1;
            return 0;
        }
    }

    /// §E.3 `decode_a_symbol` — decode one symbol under `cumf` and
    /// return its model index.
    pub fn decode_symbol(&mut self, cumf: &[i64]) -> usize {
        let length = self.high - self.low + 1;
        let cum = (-1 + (self.code_value - self.low + 1) * cumf[0]) / length;
        let mut index = 1usize;
        while index + 1 < cumf.len() && cumf[index] > cum {
            index += 1;
        }
        self.high = self.low - 1 + (length * cumf[index - 1]) / cumf[0];
        self.low += (length * cumf[index]) / cumf[0];
        loop {
            if self.high < Q2 {
                // No adjustment.
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
            self.low *= 2;
            self.high = 2 * self.high + 1;
            let bit = self.next_bit();
            self.code_value = 2 * self.code_value + bit;
        }
        index - 1
    }
}

// ---------------------------------------------------------------------
// §E.2 / §E.6 — arithmetic encoder.
// ---------------------------------------------------------------------

/// The §E.2 arithmetic encoder over a stuffing (§E.5) bit sink.
///
/// Constructed after the fixed-length picture-header string has been
/// written raw; symbols are encoded with [`SacEncoder::encode_symbol`]
/// and the interval registers are flushed with [`SacEncoder::flush`]
/// before any following fixed-length string (or the final PSTUF).
pub struct SacEncoder<'w> {
    writer: &'w mut BitWriter,
    low: i64,
    high: i64,
    opposite_bits: u64,
    /// §E.5 — length of the current emitted run of `"0"`s.
    zero_run: u32,
}

impl core::fmt::Debug for SacEncoder<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SacEncoder")
            .field("low", &self.low)
            .field("high", &self.high)
            .field("opposite_bits", &self.opposite_bits)
            .field("zero_run", &self.zero_run)
            .finish_non_exhaustive()
    }
}

impl<'w> SacEncoder<'w> {
    /// Initialise the encoder at the writer's current position with
    /// the §E.2 register defaults (`low = 0`, `high = top`,
    /// `opposite_bits = 0`).
    ///
    /// The §E.5 zero-run counter starts at 0; when the bits already in
    /// the writer end in a run of zeros (a header tail), use
    /// [`SacEncoder::with_zero_run`] so the stuffing filter counts the
    /// run over the whole stream as §E.5 requires.
    pub fn new(writer: &'w mut BitWriter) -> Self {
        Self::with_zero_run(writer, 0)
    }

    /// As [`SacEncoder::new`], but seeding the §E.5 stuffing filter
    /// with the length of the zero run the writer's existing output
    /// ends in (the trailing zeros of the fixed-length header string).
    /// Without the seed, a header tail of `k` zeros followed by 14
    /// arithmetic zeros would put `14 + k` consecutive zeros on the
    /// wire — a start-code emulation for `k ≥ 2` at the wrong
    /// alignment budget.
    pub fn with_zero_run(writer: &'w mut BitWriter, zero_run: u32) -> Self {
        let mut enc = SacEncoder {
            writer,
            low: 0,
            high: TOP,
            opposite_bits: 0,
            zero_run: zero_run.min(STUFF_RUN),
        };
        // §E.5 — a header tail that itself completes a 14-zero run is
        // stuffed immediately (the decoder's seeded filter deletes the
        // first "1" it reads). Unreachable for the crate's own header
        // shapes (tail ≤ 6 zeros) but kept for symmetry.
        if enc.zero_run == STUFF_RUN {
            enc.writer.write_bit(true);
            enc.zero_run = 0;
        }
        enc
    }

    /// Emit one bit through the §E.5 stuffing filter: after each run
    /// of 14 `"0"`s a `"1"` is stuffed.
    fn emit_bit(&mut self, bit: bool) {
        self.writer.write_bit(bit);
        if bit {
            self.zero_run = 0;
        } else {
            self.zero_run += 1;
            if self.zero_run == STUFF_RUN {
                // §E.5 — stuff a "1" after 14 consecutive zeros.
                self.writer.write_bit(true);
                self.zero_run = 0;
            }
        }
    }

    /// Emit `bit` followed by `opposite_bits` copies of its complement
    /// (the pending-bit expansion of §E.2).
    fn emit_with_opposites(&mut self, bit: bool) {
        self.emit_bit(bit);
        while self.opposite_bits > 0 {
            self.emit_bit(!bit);
            self.opposite_bits -= 1;
        }
    }

    /// §E.2 `encode_a_symbol` — encode the symbol at `index` under
    /// `cumf`.
    pub fn encode_symbol(&mut self, index: usize, cumf: &[i64]) {
        let length = self.high - self.low + 1;
        self.high = self.low - 1 + (length * cumf[index]) / cumf[0];
        self.low += (length * cumf[index + 1]) / cumf[0];
        loop {
            if self.high < Q2 {
                self.emit_with_opposites(false);
            } else if self.low >= Q2 {
                self.emit_with_opposites(true);
                self.low -= Q2;
                self.high -= Q2;
            } else if self.low >= Q1 && self.high < Q3 {
                self.opposite_bits += 1;
                self.low -= Q1;
                self.high -= Q1;
            } else {
                break;
            }
            self.low *= 2;
            self.high = 2 * self.high + 1;
        }
    }

    /// §E.6 `encoder_flush` — terminate the arithmetic interval so the
    /// decoder can resolve every encoded symbol, then reset the
    /// registers (`low = 0`, `high = top`). Must be called before any
    /// following fixed-length header string or the closing PSTUF.
    pub fn flush(&mut self) {
        self.opposite_bits += 1;
        let bit = self.low >= Q1;
        self.emit_with_opposites(bit);
        self.low = 0;
        self.high = TOP;
    }
}

// ---------------------------------------------------------------------
// §E.7 — per-syntax-element symbol codecs.
// ---------------------------------------------------------------------

/// Decode the P-picture COD flag (§5.3.1). Returns `true` when the
/// macroblock is **coded** (COD = "0", model index 0).
pub fn decode_cod(dec: &mut SacDecoder<'_, '_>) -> bool {
    dec.decode_symbol(&CUMF_COD) == 0
}

/// Encode the P-picture COD flag. `coded == true` emits COD = "0".
pub fn encode_cod(enc: &mut SacEncoder<'_>, coded: bool) {
    enc.encode_symbol(if coded { 0 } else { 1 }, &CUMF_COD);
}

/// Table 7 index → `(MbType, CBPC)` for I-picture MCBPC.
fn mcbpc_i_from_index(index: usize) -> Result<(MbType, u8)> {
    Ok(match index {
        0..=3 => (MbType::Intra, (index % 4) as u8),
        4..=7 => (MbType::IntraQ, (index % 4) as u8),
        8 => (MbType::Stuffing, 0),
        _ => return Err(Error::BadMcbpcCode),
    })
}

/// `(MbType, CBPC)` → Table 7 index for I-picture MCBPC.
fn mcbpc_i_to_index(mb_type: MbType, cbpc: u8) -> Result<usize> {
    Ok(match mb_type {
        MbType::Intra => cbpc as usize,
        MbType::IntraQ => 4 + cbpc as usize,
        MbType::Stuffing => 8,
        _ => return Err(Error::BadMcbpcCode),
    })
}

/// Table 8 index → `(MbType, CBPC)` for P-picture MCBPC (the
/// `cumf_MCBPC_no4MVQ` 21-symbol form; type 5 needs PLUSPTYPE).
fn mcbpc_p_from_index(index: usize) -> Result<(MbType, u8)> {
    let ty = match index {
        0..=3 => MbType::Inter,
        4..=7 => MbType::InterQ,
        8..=11 => MbType::Inter4V,
        12..=15 => MbType::Intra,
        16..=19 => MbType::IntraQ,
        20 => return Ok((MbType::Stuffing, 0)),
        _ => return Err(Error::BadMcbpcCode),
    };
    Ok((ty, (index % 4) as u8))
}

/// `(MbType, CBPC)` → Table 8 index for P-picture MCBPC.
fn mcbpc_p_to_index(mb_type: MbType, cbpc: u8) -> Result<usize> {
    let base = match mb_type {
        MbType::Inter => 0,
        MbType::InterQ => 4,
        MbType::Inter4V => 8,
        MbType::Intra => 12,
        MbType::IntraQ => 16,
        MbType::Stuffing => return Ok(20),
        MbType::Inter4VQ => return Err(Error::BadMcbpcCode),
    };
    Ok(base + cbpc as usize)
}

/// Decode an I-picture MCBPC symbol (Table 7 indexing,
/// `cumf_MCBPC_intra`).
pub fn decode_mcbpc_i_sac(dec: &mut SacDecoder<'_, '_>) -> Result<(MbType, u8)> {
    mcbpc_i_from_index(dec.decode_symbol(&CUMF_MCBPC_INTRA))
}

/// Encode an I-picture MCBPC symbol.
pub fn encode_mcbpc_i_sac(enc: &mut SacEncoder<'_>, mb_type: MbType, cbpc: u8) -> Result<()> {
    enc.encode_symbol(mcbpc_i_to_index(mb_type, cbpc)?, &CUMF_MCBPC_INTRA);
    Ok(())
}

/// Decode a P-picture MCBPC symbol (Table 8 indexing,
/// `cumf_MCBPC_no4MVQ`).
pub fn decode_mcbpc_p_sac(dec: &mut SacDecoder<'_, '_>) -> Result<(MbType, u8)> {
    mcbpc_p_from_index(dec.decode_symbol(&CUMF_MCBPC_NO4MVQ))
}

/// Encode a P-picture MCBPC symbol.
pub fn encode_mcbpc_p_sac(enc: &mut SacEncoder<'_>, mb_type: MbType, cbpc: u8) -> Result<()> {
    enc.encode_symbol(mcbpc_p_to_index(mb_type, cbpc)?, &CUMF_MCBPC_NO4MVQ);
    Ok(())
}

/// Decode a CBPY symbol. The returned 4-bit pattern is in the spec's
/// **CBPY(INTRA)** orientation (Table 12 index = INTRA pattern), the
/// same convention as [`crate::macroblock::H263Macroblock::cbpy`]
/// (INTER macroblocks complement it downstream).
pub fn decode_cbpy_sac(dec: &mut SacDecoder<'_, '_>, mb_is_intra: bool) -> u8 {
    let model: &[i64] = if mb_is_intra {
        &CUMF_CBPY_INTRA
    } else {
        &CUMF_CBPY
    };
    dec.decode_symbol(model) as u8
}

/// Encode a CBPY symbol (`cbpy` in CBPY(INTRA) orientation).
pub fn encode_cbpy_sac(enc: &mut SacEncoder<'_>, cbpy: u8, mb_is_intra: bool) -> Result<()> {
    if cbpy > 0b1111 {
        return Err(Error::BadCbpyCode);
    }
    let model: &[i64] = if mb_is_intra {
        &CUMF_CBPY_INTRA
    } else {
        &CUMF_CBPY
    };
    enc.encode_symbol(cbpy as usize, model);
    Ok(())
}

/// Decode a DQUANT symbol (Table 13 indexing).
pub fn decode_dquant_sac(dec: &mut SacDecoder<'_, '_>) -> i8 {
    match dec.decode_symbol(&CUMF_DQUANT) {
        0 => -1,
        1 => -2,
        2 => 1,
        _ => 2,
    }
}

/// Encode a DQUANT differential (`{-2, -1, +1, +2}`).
pub fn encode_dquant_sac(enc: &mut SacEncoder<'_>, diff: i8) -> Result<()> {
    let index = match diff {
        -1 => 0,
        -2 => 1,
        1 => 2,
        2 => 3,
        _ => return Err(Error::BadDquant),
    };
    enc.encode_symbol(index, &CUMF_DQUANT);
    Ok(())
}

/// Decode an Annex G MODB symbol (Table 11 indexing, `cumf_MODB_G`).
pub fn decode_modb_sac(dec: &mut SacDecoder<'_, '_>) -> crate::pb_layer::ModbPresence {
    use crate::pb_layer::ModbPresence;
    match dec.decode_symbol(&CUMF_MODB_G) {
        0 => ModbPresence::None,
        1 => ModbPresence::MvdbOnly,
        _ => ModbPresence::CbpbAndMvdb,
    }
}

/// Encode an Annex G MODB symbol (Table 11 indexing).
pub fn encode_modb_sac(enc: &mut SacEncoder<'_>, modb: crate::pb_layer::ModbPresence) {
    use crate::pb_layer::ModbPresence;
    let index = match modb {
        ModbPresence::None => 0,
        ModbPresence::MvdbOnly => 1,
        ModbPresence::CbpbAndMvdb => 2,
    };
    enc.encode_symbol(index, &CUMF_MODB_G);
}

/// Decode the six §5.3.4 CBPB bits — one symbol per block in Figure-5
/// order (blocks 1..=4 under `cumf_YCBPB`, blocks 5..=6 under
/// `cumf_UVCBPB`, §E.7). The returned pattern uses the
/// [`crate::pb_layer::cbpb_block_present`] convention (block 1 = MSB
/// of the 6-bit field).
pub fn decode_cbpb_sac(dec: &mut SacDecoder<'_, '_>) -> u8 {
    let mut cbpb = 0u8;
    for block_number in 1..=6u32 {
        let model: &[i64] = if block_number <= 4 {
            &CUMF_YCBPB
        } else {
            &CUMF_UVCBPB
        };
        if dec.decode_symbol(model) == 1 {
            cbpb |= 1 << (6 - block_number);
        }
    }
    cbpb
}

/// Encode the six §5.3.4 CBPB bits — the inverse of
/// [`decode_cbpb_sac`].
pub fn encode_cbpb_sac(enc: &mut SacEncoder<'_>, cbpb: u8) {
    for block_number in 1..=6u32 {
        let model: &[i64] = if block_number <= 4 {
            &CUMF_YCBPB
        } else {
            &CUMF_UVCBPB
        };
        let bit = (cbpb >> (6 - block_number)) & 1;
        enc.encode_symbol(bit as usize, model);
    }
}

/// Decode one MVD component (Table 14 indexing: index 32 = 0). The
/// returned value is in half-pel units, range `[-32, +31]`.
pub fn decode_mvd_component_sac(dec: &mut SacDecoder<'_, '_>) -> i8 {
    (dec.decode_symbol(&CUMF_MVD) as i32 - 32) as i8
}

/// Encode one MVD component (half-pel units, `[-32, +31]`).
pub fn encode_mvd_component_sac(enc: &mut SacEncoder<'_>, half: i8) -> Result<()> {
    if !(-32..=31).contains(&(half as i32)) {
        return Err(Error::BadMvdCode);
    }
    enc.encode_symbol((half as i32 + 32) as usize, &CUMF_MVD);
    Ok(())
}

/// Decode an INTRADC symbol (Table 15 indexing) into the DC
/// reconstruction level (a multiple of 8, or 1024).
pub fn decode_intradc_sac(dec: &mut SacDecoder<'_, '_>) -> i16 {
    let index = dec.decode_symbol(&CUMF_INTRADC);
    // Table 15: index 0..=126 → FLC 1..=127, index 127 → FLC 255
    // (level 1024), index 128..=253 → FLC 129..=254.
    if index == 127 {
        1024
    } else {
        (index as i16 + 1) * 8
    }
}

/// Encode an INTRADC reconstruction level (a multiple of 8 in
/// `[8, 2032]` with the 1024 special case; the two §5.4.1 forbidden
/// codes are unreachable by construction of the quantiser).
pub fn encode_intradc_sac(enc: &mut SacEncoder<'_>, level: i16) -> Result<()> {
    let index = if level == 1024 {
        127usize
    } else {
        if !(8..=2032).contains(&level) || level % 8 != 0 {
            return Err(Error::BadIntradcCode);
        }
        let code = (level / 8) as usize;
        if code == 128 {
            return Err(Error::BadIntradcCode);
        }
        code - 1
    };
    enc.encode_symbol(index, &CUMF_INTRADC);
    Ok(())
}

/// The TCOEF model for the `n`-th (0-based) coefficient event of a
/// block (§E.4 / §E.7: TCOEF1, TCOEF2, TCOEF3, then TCOEFr).
fn tcoef_model(event_ordinal: u32, mb_is_intra: bool) -> &'static [i64] {
    match (event_ordinal, mb_is_intra) {
        (0, false) => &CUMF_TCOEF1,
        (1, false) => &CUMF_TCOEF2,
        (2, false) => &CUMF_TCOEF3,
        (_, false) => &CUMF_TCOEFR,
        (0, true) => &CUMF_TCOEF1_INTRA,
        (1, true) => &CUMF_TCOEF2_INTRA,
        (2, true) => &CUMF_TCOEF3_INTRA,
        (_, true) => &CUMF_TCOEFR_INTRA,
    }
}

/// Decode one `(LAST, RUN, LEVEL)` TCOEF event: a Table 16 symbol
/// (`TCOEF1/2/3/r` model by `event_ordinal`) followed by a SIGN
/// symbol, or — for the ESCAPE index 102 — the `LAST` / `RUN` /
/// `LEVEL` model triple (§E.7, Table 17 indexing).
fn decode_tcoef_event_sac(
    dec: &mut SacDecoder<'_, '_>,
    event_ordinal: u32,
    mb_is_intra: bool,
) -> Result<(bool, u8, i16)> {
    let index = dec.decode_symbol(tcoef_model(event_ordinal, mb_is_intra));
    if index == TCOEF_ESCAPE_INDEX {
        let last = dec.decode_symbol(if mb_is_intra {
            &CUMF_LAST_INTRA
        } else {
            &CUMF_LAST
        }) == 1;
        let run = dec.decode_symbol(if mb_is_intra {
            &CUMF_RUN_INTRA
        } else {
            &CUMF_RUN
        }) as u8;
        let level_index = dec.decode_symbol(if mb_is_intra {
            &CUMF_LEVEL_INTRA
        } else {
            &CUMF_LEVEL
        });
        // Table 17: index 0..=126 → LEVEL −127..−1, 127..=253 → +1..+127.
        let level = if level_index <= 126 {
            level_index as i16 - 127
        } else {
            level_index as i16 - 126
        };
        return Ok((last, run, level));
    }
    let (last, run, abs_level) = tcoef_table_row(index).ok_or(Error::BadTcoefCode)?;
    let negative = dec.decode_symbol(&CUMF_SIGN) == 1;
    let level = if negative {
        -(abs_level as i16)
    } else {
        abs_level as i16
    };
    Ok((last, run, level))
}

/// Encode one `(LAST, RUN, LEVEL)` TCOEF event — the exact inverse of
/// [`decode_tcoef_event_sac`]. Events in Table 16 use the regular
/// symbol + SIGN; everything else goes through ESCAPE with the
/// Table 17 RUN / LEVEL models (LEVEL must be in `[-127, +127]`
/// excluding 0).
fn encode_tcoef_event_sac(
    enc: &mut SacEncoder<'_>,
    event_ordinal: u32,
    mb_is_intra: bool,
    last: bool,
    run: u8,
    level: i16,
) -> Result<()> {
    if level == 0 {
        return Err(Error::BadTcoefCode);
    }
    let model = tcoef_model(event_ordinal, mb_is_intra);
    let abs_level = level.unsigned_abs();
    if abs_level <= u8::MAX as u16 {
        if let Some(index) = tcoef_table_index(last, run, abs_level as u8) {
            enc.encode_symbol(index, model);
            enc.encode_symbol(if level < 0 { 1 } else { 0 }, &CUMF_SIGN);
            return Ok(());
        }
    }
    // ESCAPE: LAST / RUN / LEVEL models. §5.4.2 bounds LEVEL to
    // [-127, +127] \ {0} (no Annex T extension in SAC — §5.1.4.6 bars
    // the combination).
    if !(-127..=127).contains(&level) || run > 63 {
        return Err(Error::BadTcoefCode);
    }
    enc.encode_symbol(TCOEF_ESCAPE_INDEX, model);
    enc.encode_symbol(
        if last { 1 } else { 0 },
        if mb_is_intra {
            &CUMF_LAST_INTRA
        } else {
            &CUMF_LAST
        },
    );
    enc.encode_symbol(
        run as usize,
        if mb_is_intra {
            &CUMF_RUN_INTRA
        } else {
            &CUMF_RUN
        },
    );
    let level_index = if level < 0 {
        (level + 127) as usize
    } else {
        (level + 126) as usize
    };
    enc.encode_symbol(
        level_index,
        if mb_is_intra {
            &CUMF_LEVEL_INTRA
        } else {
            &CUMF_LEVEL
        },
    );
    Ok(())
}

/// Parse one 8×8 block of an SAC picture (§E.4 Figure E.1): the
/// INTRADC symbol (INTRA macroblocks) followed by the TCOEF1 / TCOEF2
/// / TCOEF3 / TCOEFr event stream when the block's CBP bit is lit.
/// The coefficient accumulation mirrors
/// [`crate::block::parse_block`] exactly (zigzag scan positions,
/// RUN-overflow checks), so the returned [`H263Block`] feeds the same
/// reconstruction primitives.
///
/// `has_intradc` / `has_coefficients` follow the
/// [`crate::block::BlockContext`] conventions; `mb_is_intra` selects
/// the INTRA or INTER model family (§E.7 — keyed on the macroblock
/// type, exactly like the Table 16 sharing in VLC mode).
pub fn parse_block_sac(
    dec: &mut SacDecoder<'_, '_>,
    has_intradc: bool,
    has_coefficients: bool,
    mb_is_intra: bool,
) -> Result<H263Block> {
    let mut block = H263Block::empty();

    if has_intradc {
        block.coefficients[0] = decode_intradc_sac(dec);
        block.had_intradc = true;
    }

    if has_coefficients {
        let mut scan_pos: usize = if has_intradc { 1 } else { 0 };
        loop {
            let (last, run, level) =
                decode_tcoef_event_sac(dec, block.tcoef_event_count, mb_is_intra)?;
            scan_pos = scan_pos
                .checked_add(run as usize)
                .ok_or(Error::BadTcoefRunOverflow)?;
            if scan_pos >= crate::block::COEFFS_PER_BLOCK {
                return Err(Error::BadTcoefRunOverflow);
            }
            block.coefficients[scan_pos] = level;
            block.tcoef_event_count += 1;
            if last {
                break;
            }
            scan_pos += 1;
            if scan_pos >= crate::block::COEFFS_PER_BLOCK {
                return Err(Error::BadTcoefRunOverflow);
            }
        }
    }

    Ok(block)
}

/// Write one 8×8 block of an SAC picture — the inverse of
/// [`parse_block_sac`]. `dc_level` is the Table 15 INTRADC
/// reconstruction level for INTRA blocks (`None` for INTER blocks);
/// `scan` carries the quantised coefficients in zigzag scan order and
/// `has_coefficients` the block's CBP bit (for INTRA blocks the DC
/// slot is carried by `dc_level` and slot 0 of `scan` is ignored).
pub fn write_block_sac(
    enc: &mut SacEncoder<'_>,
    dc_level: Option<i16>,
    scan: &[i16; crate::block::COEFFS_PER_BLOCK],
    has_coefficients: bool,
    mb_is_intra: bool,
) -> Result<()> {
    if let Some(dc) = dc_level {
        encode_intradc_sac(enc, dc)?;
    }
    if has_coefficients {
        let events = crate::encoder_block::tcoef_events(scan, dc_level.is_some());
        if events.is_empty() {
            return Err(Error::BadTcoefCode);
        }
        for (ordinal, ev) in events.iter().enumerate() {
            encode_tcoef_event_sac(enc, ordinal as u32, mb_is_intra, ev.last, ev.run, ev.level)?;
        }
    }
    Ok(())
}

/// Parse one macroblock header of an SAC picture (§5.3 syntax with
/// every VLC replaced by its §E.7 model): COD (P-pictures), MCBPC,
/// CBPY, optional DQUANT and the MVD pair. The returned
/// [`H263Macroblock`] uses the exact conventions of
/// [`crate::macroblock::parse_macroblock`] (CBPY in INTRA orientation,
/// `quantiser_after` post-DQUANT, `Stuffing` surfaced for the caller's
/// re-read loop).
///
/// Staged subset: the baseline I/P macroblock forms (plus the Annex D
/// UMV vector range — MVD symbols are Table 14 either way), the
/// Annex F / Annex J INTER4V form (MVD2-4 under `cumf_MVD` when the
/// context signals Advanced Prediction or Deblocking Filter mode) and
/// the Annex G PB-frame fields (MODB under `cumf_MODB_G`, the six
/// per-block CBPB symbols, the §G.2 INTRA-macroblock MVD and MVDB —
/// all per §E.7). Advanced INTRA Coding and Modified Quantization
/// (barred with SAC by §5.1.4.6) return [`Error::NotImplemented`], as
/// does an INTER4V type outside AP / DF mode and the Annex M MODB
/// form (Improved PB-frames are PLUSPTYPE-only).
pub fn parse_macroblock_sac(
    dec: &mut SacDecoder<'_, '_>,
    ctx: crate::macroblock::MbContext,
) -> Result<crate::macroblock::H263Macroblock> {
    use crate::macroblock::{H263Macroblock, MbContext, Mvd};
    use crate::picture_header::H263PictureCodingType;

    let MbContext {
        picture_coding_type,
        quantiser_before,
        ..
    } = ctx;
    if ctx.aic_intra_mode || ctx.modified_quant || ctx.pb_annex_m {
        return Err(Error::NotImplemented);
    }
    if quantiser_before == 0 || quantiser_before > 31 {
        return Err(Error::InvalidQuantiser);
    }

    let empty = H263Macroblock {
        coded: false,
        mb_type: None,
        cbpc: None,
        cbpy: None,
        dquant: None,
        quantiser_after: quantiser_before,
        mvd: None,
        mvd234: [None; 3],
        intra_mode: None,
        modb: None,
        annex_m_modb: None,
        cbpb: None,
        mvdb: None,
    };

    // §5.3.1 — COD, only in INTER pictures.
    let is_inter_picture = matches!(picture_coding_type, H263PictureCodingType::Inter);
    if is_inter_picture && !decode_cod(dec) {
        return Ok(empty);
    }

    // §5.3.2 — MCBPC.
    let (mb_type, cbpc) = if is_inter_picture {
        decode_mcbpc_p_sac(dec)?
    } else {
        decode_mcbpc_i_sac(dec)?
    };
    if matches!(mb_type, MbType::Stuffing) {
        return Ok(H263Macroblock {
            coded: true,
            mb_type: Some(MbType::Stuffing),
            ..empty
        });
    }
    if matches!(mb_type, MbType::Inter4V | MbType::Inter4VQ)
        && !(ctx.advanced_prediction || ctx.deblocking_filter)
    {
        // Table 8 only defines the four-vector types under Advanced
        // Prediction / Deblocking Filter mode (§5.3.8 / Table J.1).
        return Err(Error::NotImplemented);
    }

    // §5.3.3 — MODB (Table 11 under `cumf_MODB_G`), between MCBPC and
    // CBPY per Figure 10, for every coded non-stuffing macroblock in
    // PB-frames mode.
    let modb = if ctx.pb_frames {
        Some(decode_modb_sac(dec))
    } else {
        None
    };

    // §5.3.4 — CBPB, between MODB and CBPY, when MODB indicates it.
    let cbpb = if modb.is_some_and(|m| m.has_cbpb()) {
        Some(decode_cbpb_sac(dec))
    } else {
        None
    };

    // §5.3.5 — CBPY (model keyed on the macroblock type per §E.7).
    let cbpy = decode_cbpy_sac(dec, mb_type.is_intra());

    // §5.3.6 — DQUANT (baseline Table 13 differential; §5.1.4.6 bars
    // the Annex T variable-length form with SAC).
    let (dquant, quantiser_after) = if mb_type.has_dquant() {
        let diff = decode_dquant_sac(dec);
        let q = (quantiser_before as i16 + diff as i16).clamp(1, 31) as u8;
        (Some(diff), q)
    } else {
        (None, quantiser_before)
    };

    // §5.3.7 — MVD (horizontal then vertical) for INTER types; in
    // PB-frames mode also for INTRA macroblocks (§G.2 — the vector
    // predicts the B-blocks only).
    let mvd = if mb_type.has_mvd() || (ctx.pb_frames && mb_type.is_intra()) {
        let dx_half = decode_mvd_component_sac(dec);
        let dy_half = decode_mvd_component_sac(dec);
        Some(Mvd { dx_half, dy_half })
    } else {
        None
    };

    // §5.3.8 — MVD2-4 for the four-vector types (same `cumf_MVD`
    // model per §E.7).
    let mut mvd234 = [None; 3];
    if mb_type.has_mvd2_4(ctx.advanced_prediction, ctx.deblocking_filter) {
        for slot in mvd234.iter_mut() {
            let dx_half = decode_mvd_component_sac(dec);
            let dy_half = decode_mvd_component_sac(dec);
            *slot = Some(Mvd { dx_half, dy_half });
        }
    }

    // §5.3.9 — MVDB, last header field per Figure 10, when MODB
    // indicates it (model `cumf_MVD` per §E.7).
    let mvdb = if modb.is_some_and(|m| m.has_mvdb()) {
        let dx_half = decode_mvd_component_sac(dec);
        let dy_half = decode_mvd_component_sac(dec);
        Some(Mvd { dx_half, dy_half })
    } else {
        None
    };

    Ok(H263Macroblock {
        coded: true,
        mb_type: Some(mb_type),
        cbpc: Some(cbpc),
        cbpy: Some(cbpy),
        dquant,
        quantiser_after,
        mvd,
        mvd234,
        modb,
        cbpb,
        mvdb,
        ..empty
    })
}

/// Encode a baseline **INTRA** macroblock of an SAC picture — the
/// arithmetic-coded mirror of
/// [`crate::encoder_mb::encode_intra_macroblock`]: optional COD
/// (P-pictures), MCBPC, CBPY and the six §E.4 blocks (INTRADC symbol +
/// AC TCOEF events each). The six blocks are forward-transformed and
/// quantised at `quant` with the exact same
/// [`crate::encoder_block::encode_intra_block`] stage as the VLC
/// encoder, so SAC and VLC pictures of the same source reconstruct
/// identically.
pub fn encode_intra_macroblock_sac(
    enc: &mut SacEncoder<'_>,
    mb: &crate::encoder_mb::MacroblockSamples,
    quant: u8,
    write_cod: bool,
    picture_is_inter: bool,
) -> Result<()> {
    use crate::encoder_block::encode_intra_block;

    if write_cod {
        encode_cod(enc, true);
    }

    let y_enc = [
        encode_intra_block(&mb.luma[0], quant),
        encode_intra_block(&mb.luma[1], quant),
        encode_intra_block(&mb.luma[2], quant),
        encode_intra_block(&mb.luma[3], quant),
    ];
    let cb_enc = encode_intra_block(&mb.cb, quant);
    let cr_enc = encode_intra_block(&mb.cr, quant);

    let mut cbpc = 0u8;
    if cb_enc.has_ac {
        cbpc |= 0b10;
    }
    if cr_enc.has_ac {
        cbpc |= 0b01;
    }
    if picture_is_inter {
        encode_mcbpc_p_sac(enc, MbType::Intra, cbpc)?;
    } else {
        encode_mcbpc_i_sac(enc, MbType::Intra, cbpc)?;
    }

    let mut cbpy = 0u8;
    for (blk, e) in y_enc.iter().enumerate() {
        if e.has_ac {
            cbpy |= 1 << (3 - blk);
        }
    }
    encode_cbpy_sac(enc, cbpy, true)?;

    for e in &y_enc {
        write_block_sac(enc, Some(e.dc_level), &e.scan, e.has_ac, true)?;
    }
    write_block_sac(
        enc,
        Some(cb_enc.dc_level),
        &cb_enc.scan,
        cb_enc.has_ac,
        true,
    )?;
    write_block_sac(
        enc,
        Some(cr_enc.dc_level),
        &cr_enc.scan,
        cr_enc.has_ac,
        true,
    )?;

    Ok(())
}

/// Encode a baseline **INTER** macroblock of an SAC picture — the
/// arithmetic-coded mirror of
/// [`crate::encoder_mb::encode_inter_macroblock`]: COD = coded, MCBPC,
/// the complemented CBPY, the MVD pair and the coefficient payload.
pub fn encode_inter_macroblock_sac(
    enc: &mut SacEncoder<'_>,
    luma: &[crate::encoder_block::EncodedInterBlock; 4],
    cb: &crate::encoder_block::EncodedInterBlock,
    cr: &crate::encoder_block::EncodedInterBlock,
    mvd: crate::macroblock::Mvd,
) -> Result<()> {
    encode_cod(enc, true);

    let mut cbpc = 0u8;
    if cb.has_coeffs {
        cbpc |= 0b10;
    }
    if cr.has_coeffs {
        cbpc |= 0b01;
    }
    encode_mcbpc_p_sac(enc, MbType::Inter, cbpc)?;

    let mut cbpy_intra = 0u8;
    for (blk, e) in luma.iter().enumerate() {
        if e.has_coeffs {
            cbpy_intra |= 1 << (3 - blk);
        }
    }
    // §5.3.5 — INTER macroblocks code the complement pattern (the
    // Table 12 index the decoder complements back).
    encode_cbpy_sac(enc, cbpy_intra ^ 0b1111, false)?;

    encode_mvd_component_sac(enc, mvd.dx_half)?;
    encode_mvd_component_sac(enc, mvd.dy_half)?;

    for e in luma.iter() {
        if e.has_coeffs {
            write_block_sac(enc, None, &e.scan, true, false)?;
        }
    }
    if cb.has_coeffs {
        write_block_sac(enc, None, &cb.scan, true, false)?;
    }
    if cr.has_coeffs {
        write_block_sac(enc, None, &cr.scan, true, false)?;
    }

    Ok(())
}

/// Emit a "not coded" (skipped) P-picture macroblock: the COD = "1"
/// symbol (§5.3.1).
pub fn encode_skipped_macroblock_sac(enc: &mut SacEncoder<'_>) {
    encode_cod(enc, false);
}

/// Encode an Annex F / Annex J **INTER4V** macroblock of an SAC
/// picture — the arithmetic-coded mirror of
/// [`crate::encoder_mb::encode_inter4v_macroblock`]: COD = coded, the
/// Table-8 INTER4V MCBPC symbol, the complemented CBPY, the four MVD
/// pairs (§5.3.7 / §5.3.8, all under `cumf_MVD`) and the coefficient
/// payload.
pub fn encode_inter4v_macroblock_sac(
    enc: &mut SacEncoder<'_>,
    luma: &[crate::encoder_block::EncodedInterBlock; 4],
    cb: &crate::encoder_block::EncodedInterBlock,
    cr: &crate::encoder_block::EncodedInterBlock,
    mvds: &[crate::macroblock::Mvd; 4],
) -> Result<()> {
    encode_cod(enc, true);

    let mut cbpc = 0u8;
    if cb.has_coeffs {
        cbpc |= 0b10;
    }
    if cr.has_coeffs {
        cbpc |= 0b01;
    }
    encode_mcbpc_p_sac(enc, MbType::Inter4V, cbpc)?;

    let mut cbpy_intra = 0u8;
    for (blk, e) in luma.iter().enumerate() {
        if e.has_coeffs {
            cbpy_intra |= 1 << (3 - blk);
        }
    }
    encode_cbpy_sac(enc, cbpy_intra ^ 0b1111, false)?;

    for mvd in mvds {
        encode_mvd_component_sac(enc, mvd.dx_half)?;
        encode_mvd_component_sac(enc, mvd.dy_half)?;
    }

    for e in luma.iter() {
        if e.has_coeffs {
            write_block_sac(enc, None, &e.scan, true, false)?;
        }
    }
    if cb.has_coeffs {
        write_block_sac(enc, None, &cb.scan, true, false)?;
    }
    if cr.has_coeffs {
        write_block_sac(enc, None, &cr.scan, true, false)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every staged §E.8 model is a well-formed cumulative-frequency
    /// table: first entry 16383, last entry 0, strictly decreasing
    /// (every symbol has non-zero probability).
    #[test]
    fn models_are_well_formed() {
        let models: &[(&str, &[i64], usize)] = &[
            ("COD", &CUMF_COD, 3),
            ("MCBPC_no4MVQ", &CUMF_MCBPC_NO4MVQ, 22),
            ("MCBPC_intra", &CUMF_MCBPC_INTRA, 10),
            ("CBPY", &CUMF_CBPY, 17),
            ("CBPY_intra", &CUMF_CBPY_INTRA, 17),
            ("DQUANT", &CUMF_DQUANT, 5),
            ("MODB_G", &CUMF_MODB_G, 4),
            ("YCBPB", &CUMF_YCBPB, 3),
            ("UVCBPB", &CUMF_UVCBPB, 3),
            ("MVD", &CUMF_MVD, 65),
            ("INTRADC", &CUMF_INTRADC, 255),
            ("TCOEF1", &CUMF_TCOEF1, 104),
            ("TCOEF2", &CUMF_TCOEF2, 104),
            ("TCOEF3", &CUMF_TCOEF3, 104),
            ("TCOEFr", &CUMF_TCOEFR, 104),
            ("TCOEF1_intra", &CUMF_TCOEF1_INTRA, 104),
            ("TCOEF2_intra", &CUMF_TCOEF2_INTRA, 104),
            ("TCOEF3_intra", &CUMF_TCOEF3_INTRA, 104),
            ("TCOEFr_intra", &CUMF_TCOEFR_INTRA, 104),
            ("SIGN", &CUMF_SIGN, 3),
            ("LAST", &CUMF_LAST, 3),
            ("LAST_intra", &CUMF_LAST_INTRA, 3),
            ("RUN", &CUMF_RUN, 65),
            ("RUN_intra", &CUMF_RUN_INTRA, 65),
            ("LEVEL", &CUMF_LEVEL, 255),
            ("LEVEL_intra", &CUMF_LEVEL_INTRA, 255),
        ];
        for &(name, model, len) in models {
            assert_eq!(model.len(), len, "{name}: length");
            assert_eq!(model[0], 16383, "{name}: total");
            assert_eq!(*model.last().unwrap(), 0, "{name}: terminator");
            for w in model.windows(2) {
                assert!(w[0] > w[1], "{name}: not strictly decreasing at {w:?}");
            }
        }
    }

    /// Arithmetic coder round-trip: a mixed symbol sequence across
    /// several models encodes and decodes identically.
    #[test]
    fn coder_round_trips_mixed_symbols() {
        // (model, symbol index) pairs exercising small and large models.
        let script: Vec<(&[i64], usize)> = vec![
            (&CUMF_COD, 0),
            (&CUMF_MCBPC_INTRA, 3),
            (&CUMF_CBPY_INTRA, 15),
            (&CUMF_INTRADC, 119),
            (&CUMF_TCOEF1_INTRA, 0),
            (&CUMF_SIGN, 1),
            (&CUMF_TCOEF2_INTRA, 102),
            (&CUMF_LAST_INTRA, 1),
            (&CUMF_RUN_INTRA, 5),
            (&CUMF_LEVEL_INTRA, 250),
            (&CUMF_MVD, 32),
            (&CUMF_MVD, 0),
            (&CUMF_MVD, 63),
            (&CUMF_DQUANT, 2),
            (&CUMF_INTRADC, 0),
            (&CUMF_INTRADC, 253),
            (&CUMF_INTRADC, 127),
            (&CUMF_CBPY, 0),
            (&CUMF_TCOEFR, 45),
            (&CUMF_SIGN, 0),
        ];
        let mut w = BitWriter::new();
        {
            let mut enc = SacEncoder::new(&mut w);
            for &(model, index) in &script {
                enc.encode_symbol(index, model);
            }
            enc.flush();
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let mut dec = SacDecoder::new(&mut r);
        for (i, &(model, index)) in script.iter().enumerate() {
            assert_eq!(dec.decode_symbol(model), index, "symbol {i}");
        }
    }

    /// A long low-probability symbol run — chosen to drive the
    /// interval into long carry chains and zero runs — still round
    /// trips, exercising the §E.5 stuffing filter on both sides.
    #[test]
    fn coder_round_trips_stress_run() {
        let mut script: Vec<(&[i64], usize)> = Vec::new();
        for i in 0..600usize {
            script.push((&CUMF_INTRADC, (i * 37) % 254));
            script.push((&CUMF_SIGN, i % 2));
            if i % 3 == 0 {
                script.push((&CUMF_MVD, (i * 11) % 64));
            }
        }
        let mut w = BitWriter::new();
        {
            let mut enc = SacEncoder::new(&mut w);
            for &(model, index) in &script {
                enc.encode_symbol(index, model);
            }
            enc.flush();
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let mut dec = SacDecoder::new(&mut r);
        for (i, &(model, index)) in script.iter().enumerate() {
            assert_eq!(dec.decode_symbol(model), index, "symbol {i}");
        }
    }

    /// The most-probable-symbol path compresses: a run of highest-
    /// probability symbols takes well under one bit per symbol.
    #[test]
    fn coder_compresses_most_probable_symbols() {
        let n = 512usize;
        let mut w = BitWriter::new();
        {
            let mut enc = SacEncoder::new(&mut w);
            for _ in 0..n {
                // COD index 0 has probability (16383-6849)/16383 ≈ 0.58;
                // CBPY_intra index 0 has ≈ (16383-13619)/16383 ≈ 0.17 —
                // use SIGN index 0 (≈ 0.49) and LAST_intra index 1
                // (probability 2820/16383? no — index 1 spans 2820..0 ≈
                // 0.17). Stick to a genuinely high-probability symbol:
                // MVD index 32 spans 11491−4621 = 6870/16383 ≈ 0.42.
                enc.encode_symbol(32, &CUMF_MVD);
            }
            enc.flush();
        }
        let bits = w.bit_position();
        // 0.42 probability → ≈ 1.25 bits/symbol upper bound with margin.
        assert!(
            bits < (n as u64) * 2,
            "expected < {} bits, got {bits}",
            n * 2
        );
    }

    /// §E.5 stuffing: encoded output never contains 15 consecutive
    /// zero bits (a start-code prefix cannot be emulated).
    #[test]
    fn stuffing_prevents_start_code_emulation() {
        // Drive the encoder towards long zero runs by encoding the
        // symbol whose interval hugs the low end repeatedly.
        let mut w = BitWriter::new();
        {
            let mut enc = SacEncoder::new(&mut w);
            for i in 0..2000usize {
                // Cycle extreme indices to visit many interval shapes.
                enc.encode_symbol(if i % 5 == 0 { 253 } else { 0 }, &CUMF_INTRADC);
            }
            enc.flush();
        }
        let bytes = w.finish();
        let mut run = 0u32;
        let mut max_run = 0u32;
        for byte in &bytes {
            for bit in (0..8).rev() {
                if byte & (1 << bit) == 0 {
                    run += 1;
                    max_run = max_run.max(run);
                } else {
                    run = 0;
                }
            }
        }
        // The final PSTUF-less tail may carry trailing zeros from
        // BitWriter byte padding (< 8), so the in-stream bound is
        // 14 + 7.
        assert!(max_run <= 21, "zero run of {max_run} bits");
    }

    /// COD / MCBPC / CBPY / DQUANT / MVD / INTRADC symbol codecs are
    /// exact inverses.
    #[test]
    fn syntax_codecs_round_trip() {
        let mut w = BitWriter::new();
        {
            let mut enc = SacEncoder::new(&mut w);
            encode_cod(&mut enc, true);
            encode_cod(&mut enc, false);
            encode_mcbpc_i_sac(&mut enc, MbType::Intra, 0b10).unwrap();
            encode_mcbpc_i_sac(&mut enc, MbType::IntraQ, 0b01).unwrap();
            encode_mcbpc_i_sac(&mut enc, MbType::Stuffing, 0).unwrap();
            encode_mcbpc_p_sac(&mut enc, MbType::Inter, 0b11).unwrap();
            encode_mcbpc_p_sac(&mut enc, MbType::IntraQ, 0b00).unwrap();
            encode_mcbpc_p_sac(&mut enc, MbType::Stuffing, 0).unwrap();
            encode_cbpy_sac(&mut enc, 0b1010, true).unwrap();
            encode_cbpy_sac(&mut enc, 0b0101, false).unwrap();
            encode_dquant_sac(&mut enc, -2).unwrap();
            encode_dquant_sac(&mut enc, 1).unwrap();
            encode_mvd_component_sac(&mut enc, -32).unwrap();
            encode_mvd_component_sac(&mut enc, 0).unwrap();
            encode_mvd_component_sac(&mut enc, 31).unwrap();
            encode_intradc_sac(&mut enc, 8).unwrap();
            encode_intradc_sac(&mut enc, 1024).unwrap();
            encode_intradc_sac(&mut enc, 2032).unwrap();
            encode_intradc_sac(&mut enc, 1032).unwrap();
            enc.flush();
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let mut dec = SacDecoder::new(&mut r);
        assert!(decode_cod(&mut dec));
        assert!(!decode_cod(&mut dec));
        assert_eq!(decode_mcbpc_i_sac(&mut dec).unwrap(), (MbType::Intra, 0b10));
        assert_eq!(
            decode_mcbpc_i_sac(&mut dec).unwrap(),
            (MbType::IntraQ, 0b01)
        );
        assert_eq!(decode_mcbpc_i_sac(&mut dec).unwrap().0, MbType::Stuffing);
        assert_eq!(decode_mcbpc_p_sac(&mut dec).unwrap(), (MbType::Inter, 0b11));
        assert_eq!(
            decode_mcbpc_p_sac(&mut dec).unwrap(),
            (MbType::IntraQ, 0b00)
        );
        assert_eq!(decode_mcbpc_p_sac(&mut dec).unwrap().0, MbType::Stuffing);
        assert_eq!(decode_cbpy_sac(&mut dec, true), 0b1010);
        assert_eq!(decode_cbpy_sac(&mut dec, false), 0b0101);
        assert_eq!(decode_dquant_sac(&mut dec), -2);
        assert_eq!(decode_dquant_sac(&mut dec), 1);
        assert_eq!(decode_mvd_component_sac(&mut dec), -32);
        assert_eq!(decode_mvd_component_sac(&mut dec), 0);
        assert_eq!(decode_mvd_component_sac(&mut dec), 31);
        assert_eq!(decode_intradc_sac(&mut dec), 8);
        assert_eq!(decode_intradc_sac(&mut dec), 1024);
        assert_eq!(decode_intradc_sac(&mut dec), 2032);
        assert_eq!(decode_intradc_sac(&mut dec), 1032);
    }

    /// Forbidden INTRADC levels are refused by the encoder (the
    /// Table 15 mapping has no code for them).
    #[test]
    fn intradc_forbidden_levels_refused() {
        let mut w = BitWriter::new();
        let mut enc = SacEncoder::new(&mut w);
        assert_eq!(encode_intradc_sac(&mut enc, 0), Err(Error::BadIntradcCode));
        // 128 * 8 = 1024 is legal (index 127); the *code* 128 slot
        // (level 1024 via the natural mapping) is what Table 15
        // forbids — level 1028 is not a multiple of 8:
        assert_eq!(
            encode_intradc_sac(&mut enc, 1028),
            Err(Error::BadIntradcCode)
        );
        assert_eq!(
            encode_intradc_sac(&mut enc, 2040),
            Err(Error::BadIntradcCode)
        );
    }

    /// Block-layer round-trip: INTRA block with DC + AC events
    /// (including an ESCAPE event) and an INTER block reproduce the
    /// exact scan arrays.
    #[test]
    fn block_layer_round_trips() {
        use crate::block::COEFFS_PER_BLOCK;

        let mut intra_scan = [0i16; COEFFS_PER_BLOCK];
        intra_scan[1] = 3;
        intra_scan[4] = -1;
        intra_scan[20] = 90; // forces ESCAPE (|level| 90 not in Table 16)
        intra_scan[63] = 1;

        let mut inter_scan = [0i16; COEFFS_PER_BLOCK];
        inter_scan[0] = -5;
        inter_scan[7] = 2;
        inter_scan[8] = -120; // ESCAPE with negative level
        inter_scan[33] = 1;

        let mut w = BitWriter::new();
        {
            let mut enc = SacEncoder::new(&mut w);
            write_block_sac(&mut enc, Some(960), &intra_scan, true, true).unwrap();
            write_block_sac(&mut enc, None, &inter_scan, true, false).unwrap();
            // DC-only INTRA block (no AC).
            write_block_sac(&mut enc, Some(1024), &[0; COEFFS_PER_BLOCK], false, true).unwrap();
            enc.flush();
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let mut dec = SacDecoder::new(&mut r);

        let b1 = parse_block_sac(&mut dec, true, true, true).unwrap();
        assert_eq!(b1.coefficients[0], 960);
        assert_eq!(&b1.coefficients[1..], &intra_scan[1..]);

        let b2 = parse_block_sac(&mut dec, false, true, false).unwrap();
        assert_eq!(b2.coefficients, inter_scan);

        let b3 = parse_block_sac(&mut dec, true, false, true).unwrap();
        assert_eq!(b3.coefficients[0], 1024);
        assert_eq!(b3.tcoef_event_count, 0);
    }

    /// The §E.5 destuffing filter and the encoder stuffer are exact
    /// inverses at the raw-bit level: write bits through a stuffing
    /// SacEncoder-style sink, read them back destuffed.
    #[test]
    fn stuffing_filter_is_invertible() {
        // A pattern containing several 14-zero and longer runs.
        let mut pattern: Vec<bool> = Vec::new();
        for run in [3usize, 14, 15, 28, 1, 14, 5, 30] {
            pattern.extend(std::iter::repeat_n(false, run));
            pattern.push(true);
        }
        let mut w = BitWriter::new();
        {
            let mut enc = SacEncoder::new(&mut w);
            for &b in &pattern {
                enc.emit_bit(b);
            }
            // Pad with ones so byte-padding zeros can't confuse the read.
            for _ in 0..8 {
                enc.emit_bit(true);
            }
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let mut dec = SacDecoder {
            reader: &mut r,
            low: 0,
            high: TOP,
            code_value: 0,
            zero_run: 0,
        };
        for (i, &b) in pattern.iter().enumerate() {
            assert_eq!(dec.next_bit() == 1, b, "bit {i}");
        }
    }
}
