# oxideav-h263

A pure-Rust ITU-T H.263 baseline video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild round 3 — picture + GOB + macroblock headers.**
The prior implementation was retired on 2026-05-18 under the
workspace
[clean-room policy](https://github.com/OxideAV/oxideav/blob/master/docs/IMPLEMENTOR_ROUND.md):
the encoder VLC tables were declared as mirrors of a sibling crate's
tables whose own provenance has been retired. The transitive
contamination of the table values could not be defended; master
history was fully erased per the Hat-3 cold-enforcement procedure.

The crate is being re-built clean-room against ITU-T Recommendation
H.263 (01/2005). The current master implements §5.1 (picture layer),
§5.2 (GOB layer up through GQUANT), and §5.3 (macroblock header
through MVD2-4) for the non-PB-frame baseline:

* §5.1.1 — Picture Start Code (PSC), 22 bits, value `0x000020`.
* §5.1.2 — Temporal Reference (TR), 8 bits at the standard CIF
  picture clock frequency.
* §5.1.3 — Type Information (PTYPE) in its non-extended form (13 bits):
  split-screen / document-camera / freeze-release indicators,
  source-format field (`001` sub-QCIF .. `101` 16CIF, plus the
  reserved `110` and the `111` extended-PTYPE escape), picture coding
  type (INTRA / INTER), and Annex D/E/F/G optional-mode flags.
* §5.2.2 — Group of Blocks Start Code (GBSC), 17 bits, value
  `0000 0000 0000 0000 1`.
* §5.2.3 — Group Number (GN), 5 bits; the parser accepts the union
  of the standard and custom picture-format ranges (`1..=29`) and
  rejects `0` (PSC overlap), `30` (EOSBS marker), `31` (EOS marker).
* §5.2.5 — GOB Frame ID (GFID), 2 bits; consumed and exposed for
  future inter-GOB continuity enforcement.
* §5.2.6 — Quantizer Information (GQUANT), 5 bits; QUANT range
  `1..=31`.
* §5.3.1 — Coded macroblock indication (COD), 1 bit, INTER
  pictures only.
* §5.3.2 — Macroblock type & CBPC (MCBPC), variable length; full
  Table 7 (I-pictures, 9 codes) and Table 8 (P-pictures, 25
  codes, including type-5 INTER4V+Q points reserved for
  PLUSPTYPE + Annex F/J).
* §5.3.5 — Coded Block Pattern for luminance (CBPY), variable
  length; full Table 12 (16 patterns), `CBPY(INTRA)` orientation.
* §5.3.6 — Quantizer Information (DQUANT), 2 bits in the
  baseline form, with QUANT clipped to `1..=31` after the
  differential.
* §5.3.7 / §5.3.8 — Motion Vector Data (MVD + MVD2-4), variable
  length; full Table 14 (64 codes). Components returned in
  half-pel units as signed `i8` in `[-32, +31]`.

The function surface is intentionally minimal:

```rust,ignore
use oxideav_core::bits::BitReader;
use oxideav_h263::{
    parse_gob_layer, parse_macroblock, parse_picture_header,
    H263SourceFormat, MbContext,
};

let mut r = BitReader::new(&bytes);
let pic = parse_picture_header(&mut r)?;
assert_eq!(pic.source_format.luma_dimensions(), Some((176, 144)));

// `r` is now at the first bit of the GOB layer (after any GSTUF).
let gob = parse_gob_layer(&mut r)?;
assert_eq!(gob.header_bits, 29);                       // 17 + 5 + 2 + 5

// One macroblock per spec §5.3, threading the picture's coding
// type and the GOB's QUANT through MbContext.
let mb = parse_macroblock(
    &mut r,
    MbContext {
        picture_coding_type: pic.coding_type,
        advanced_prediction: pic.advanced_prediction,
        quantiser_before: gob.quantiser,
    },
)?;
```

### What is NOT yet implemented

* Block-data decode (§5.4 / Annex H VLCs) — round 3 stops at the
  macroblock header.
* PB-frame MODB / CBPB / MVDB (§5.3.3 / §5.3.4 / §5.3.9, Annex G);
  the parser refuses no fields directly but the caller's picture
  context must keep `pb_frames = false`.
* Annex T variable-length DQUANT (Modified Quantization mode);
  the baseline 2-bit form is the only one decoded.
* Annex D Table D.3 alternative MVD codes — round 3 uses Table 14
  unconditionally.
* Annex O B/EI/EP picture macroblocks.
* GSTUF stuffing (§5.2.1) — the caller skips it before invoking the
  GOB parser; the parser does not auto-detect leading zeros.
* GSBI (§5.2.4, CPM = "1" case) — picture-layer CPM is not yet
  exposed, so the GOB parser only handles the CPM = "0" branch.
* Slice-structured mode (Annex K), end-of-sequence markers
  (§5.1.27, EOS/EOSBS as PSC-prefixed codes).
* The Annex-O optional fields after PTYPE: PQUANT, CPM/PSBI, TRB,
  DBQUANT, PEI/PSUPP.
* Extended PTYPE / PLUSPTYPE (§5.1.4) and every annex it gates
  (Annexes I, J, K, M, N, O, P, Q, R, S, T) — the parser surfaces a
  dedicated `ExtendedPtypeNotSupported` error rather than guessing.
* Encoder. Round 3 is decode-only.
* `oxideav_core::Decoder` registration; the `register()` function is
  still a no-op pending a frame-yielding decoder.

### Round 3 coverage estimate

* H.263 spec text covered: §5.1.1–§5.1.3 + §5.2.2 + §5.2.3 +
  §5.2.5 + §5.2.6 + §5.3.1 + §5.3.2 + §5.3.5 + §5.3.6 + §5.3.7 +
  §5.3.8. Roughly 6 pages of the ~144-page recommendation.
* Tests: 42 unit tests on synthetic buffers built with the spec's
  bit layout (round-trip via `oxideav_core::bits::BitWriter`),
  including full-table round-trips for Tables 7 (9 codes), 8
  (21 + 4 codes), 12 (16 codes), and 14 (64 codes), plus one
  composition test that drives all three parsers from a single
  `BitReader`.

## License

MIT — see [LICENSE](./LICENSE).
