# oxideav-h263

A pure-Rust ITU-T H.263 baseline video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild round 2 — picture + GOB headers only.** The prior
implementation was retired on 2026-05-18 under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav/blob/master/docs/IMPLEMENTOR_ROUND.md):
the encoder VLC tables were declared as mirrors of a sibling crate's
tables whose own provenance has been retired. The transitive
contamination of the table values could not be defended; master
history was fully erased per the Hat-3 cold-enforcement procedure.

The crate is being re-built clean-room against ITU-T Recommendation
H.263 (01/2005). The current master implements §5.1 (picture layer)
and §5.2 (GOB layer up through GQUANT):

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

The function surface is intentionally minimal:

```rust,ignore
use oxideav_h263::{
    parse_gob_layer_from_bytes, parse_picture_header_from_bytes,
    H263SourceFormat,
};

let header = parse_picture_header_from_bytes(&bytes)?;
assert_eq!(header.source_format.luma_dimensions(), Some((176, 144)));

// `gob_bytes` starts at the first bit of GBSC after any GSTUF.
let gob = parse_gob_layer_from_bytes(&gob_bytes)?;
assert_eq!(gob.header_bits, 29); // 17 + 5 + 2 + 5
```

### What is NOT yet implemented

* Macroblock layer (§5.3), motion-vector decode (§5.3.7), DCT
  coefficient decode (§5.4 / Annex H VLCs).
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
* Encoder. Round 2 is decode-only.
* `oxideav_core::Decoder` registration; the `register()` function is
  still a no-op pending a frame-yielding decoder.

### Round 2 coverage estimate

* H.263 spec text covered: §5.1.1–§5.1.3 + §5.2.2 + §5.2.3 + §5.2.5 +
  §5.2.6. Roughly 2 pages of the ~144-page recommendation.
* Tests: 22 unit tests on synthetic buffers built with the spec's
  bit layout (round-trip via `oxideav_core::bits::BitWriter`), of
  which 1 is a composition test that drives the round-1 picture
  parser and the round-2 GOB parser from a single `BitReader`.

## License

MIT — see [LICENSE](./LICENSE).
