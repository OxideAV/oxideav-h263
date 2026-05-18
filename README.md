# oxideav-h263

A pure-Rust ITU-T H.263 baseline video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Orphan-rebuild scaffold (2026-05-18).** The prior implementation was
retired under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav/blob/master/docs/IMPLEMENTOR_ROUND.md):
the encoder VLC tables were declared as mirrors of a sibling crate's
tables whose own provenance has been retired. The transitive
contamination of the table values could not be defended. Master
history was fully erased per the Hat-3 cold-enforcement procedure.

The implementation will be re-built against the published H.263
specification (ITU-T Recommendation H.263) in a future clean-room
round.

## License

MIT — see [LICENSE](./LICENSE).
