# oxideav-h263

A pure-Rust ITU-T H.263 baseline video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild round 1 — picture-header parser only.** The prior
implementation was retired on 2026-05-18 under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav/blob/master/docs/IMPLEMENTOR_ROUND.md):
the encoder VLC tables were declared as mirrors of a sibling crate's
tables whose own provenance has been retired. The transitive
contamination of the table values could not be defended; master
history was fully erased per the Hat-3 cold-enforcement procedure.

The crate is being re-built clean-room against ITU-T Recommendation
H.263 (01/2005). The current master implements only §5.1 of the
picture layer:

* §5.1.1 — Picture Start Code (PSC), 22 bits, value `0x000020`.
* §5.1.2 — Temporal Reference (TR), 8 bits at the standard CIF
  picture clock frequency.
* §5.1.3 — Type Information (PTYPE) in its non-extended form (13 bits):
  split-screen / document-camera / freeze-release indicators,
  source-format field (`001` sub-QCIF .. `101` 16CIF, plus the
  reserved `110` and the `111` extended-PTYPE escape), picture coding
  type (INTRA / INTER), and Annex D/E/F/G optional-mode flags.

The function surface is intentionally minimal:

```rust,ignore
use oxideav_h263::{parse_picture_header_from_bytes, H263SourceFormat};

let header = parse_picture_header_from_bytes(&bytes)?;
assert_eq!(header.source_format.luma_dimensions(), Some((176, 144)));
```

### What is NOT yet implemented

* Macroblock layer (§5.3), motion-vector decode (§5.3.7), DCT
  coefficient decode (§5.4 / Annex H VLCs).
* GOB layer (§5.2), slice-structured mode (Annex K), end-of-sequence
  (§5.1.27).
* The Annex-O optional fields after PTYPE: PQUANT, CPM/PSBI, TRB,
  DBQUANT, PEI/PSUPP.
* Extended PTYPE / PLUSPTYPE (§5.1.4) and every annex it gates
  (Annexes I, J, K, M, N, O, P, Q, R, S, T) — the parser surfaces a
  dedicated `ExtendedPtypeNotSupported` error rather than guessing.
* Encoder. Round 1 is decode-only.
* `oxideav_core::Decoder` registration; the `register()` function is
  still a no-op pending a frame-yielding decoder.

### Round 1 coverage estimate

* H.263 spec text covered: §5.1.1 + §5.1.2 + §5.1.3 (excluding the
  PLUSPTYPE branch in §5.1.4). Roughly 1 page of the ~144-page
  recommendation.
* Tests: 8 unit tests on synthetic buffers built with the spec's
  bit layout (round-trip via `oxideav_core::bits::BitWriter`).

## License

MIT — see [LICENSE](./LICENSE).
