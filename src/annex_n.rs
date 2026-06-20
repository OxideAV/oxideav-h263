//! Annex N — Reference Picture Selection mode (forward-channel decode).
//!
//! Annex N ("NEWPRED") lets the encoder pick, for each picture, GOB or
//! slice, which previously-decoded reference picture to predict from —
//! rather than always the most recent anchor. The encoder signals the
//! choice with the §5.1.14 / §N.4.1.3 **TRPI** flag and, when set, the
//! §5.1.15 / §N.4.1.4 **TRP** field carrying the Temporal Reference of
//! the picture to predict from.
//!
//! The decoder side (§N.5) maintains additional picture memories storing
//! correctly-decoded pictures together with their Temporal Reference,
//! and "uses the stored picture for which the TR is TRP as the reference
//! picture for inter-frame decoding instead of the last decoded picture,
//! if the TRP field exists in the forward-channel data". When TRP is not
//! present, "the most recent temporally previous anchor picture shall be
//! used for prediction, as when not in the Reference Picture Selection
//! mode" (§N.4.1.4).
//!
//! ## Scope of this module
//!
//! This module implements the **forward-channel** decoder behaviour: the
//! [`RpsReferenceStore`] picture memory keyed by 10-bit Temporal
//! Reference, and the §N.4.1.4 / §N.5 reference-selection rule. It does
//! **not** implement the §N.3 / §N.4.2 back-channel (BCM) messages —
//! those carry ACK / NACK status from decoder to encoder over a separate
//! logical channel (or the videomux submode) and do not affect the
//! pixels a conformant forward-channel decoder produces. The §5.1.16 BCI
//! codeword is framed (and a present BCM refused) by
//! [`crate::plus_ptype`].
//!
//! The §N.4.1 GOB / slice-layer TRI / TR / TRPI / TRP fields (which let
//! the reference be re-selected mid-picture) are a further refinement;
//! this module's store + selection serve the picture-layer TRP, which is
//! the unit the single-picture decode API operates on.

use crate::picture::YuvFrame;

/// A single stored reference picture: its decoded samples plus the
/// 10-bit Temporal Reference under which it was coded.
#[derive(Debug, Clone, PartialEq, Eq)]
struct StoredPicture {
    /// §N.4.1.4 — the 10-bit Temporal Reference (ETR concatenated with
    /// TR for a custom picture clock frequency, else `TR` with the two
    /// MSBs zero) under which this picture was transmitted.
    tr: u16,
    /// The decoded picture samples.
    frame: YuvFrame,
}

/// §N.5 — the decoder's reference picture memory for the Reference
/// Picture Selection mode.
///
/// The store keeps the most recently inserted pictures up to a capacity
/// bound (the "additional number of picture memories" of §N.5, which is
/// negotiated externally; the default here keeps a small history). B-
/// pictures are never inserted (§N.5: "except for B-pictures, which are
/// not used as reference pictures"). Insertion is first-in, first-out:
/// once the capacity is reached the oldest stored picture is evicted.
#[derive(Debug, Clone)]
pub struct RpsReferenceStore {
    pictures: Vec<StoredPicture>,
    capacity: usize,
}

impl RpsReferenceStore {
    /// Default number of additional picture memories. §N.5 leaves the
    /// exact count to external negotiation; this is a conservative
    /// in-memory default sufficient for typical NEWPRED histories.
    pub const DEFAULT_CAPACITY: usize = 16;

    /// Create an empty store with [`Self::DEFAULT_CAPACITY`] memories.
    pub fn new() -> Self {
        RpsReferenceStore {
            pictures: Vec::new(),
            capacity: Self::DEFAULT_CAPACITY,
        }
    }

    /// Create an empty store holding up to `capacity` reference
    /// pictures. A `capacity` of zero is treated as one (a decoder must
    /// retain at least the most recent anchor).
    pub fn with_capacity(capacity: usize) -> Self {
        RpsReferenceStore {
            pictures: Vec::new(),
            capacity: capacity.max(1),
        }
    }

    /// Number of currently-stored reference pictures.
    pub fn len(&self) -> usize {
        self.pictures.len()
    }

    /// Whether the store holds no reference pictures.
    pub fn is_empty(&self) -> bool {
        self.pictures.is_empty()
    }

    /// §N.5 — insert a correctly-decoded anchor picture under its 10-bit
    /// Temporal Reference. A picture with a `tr` already present replaces
    /// the stored entry (a retransmission of the same temporal instant);
    /// otherwise it is appended and the oldest entry evicted if the
    /// capacity is exceeded (first-in, first-out).
    ///
    /// B-pictures must not be stored (§N.5); the caller is responsible
    /// for only inserting anchor (I / P / EI / EP) pictures.
    pub fn insert(&mut self, tr: u16, frame: YuvFrame) {
        if let Some(slot) = self.pictures.iter_mut().find(|p| p.tr == tr) {
            slot.frame = frame;
            return;
        }
        self.pictures.push(StoredPicture { tr, frame });
        while self.pictures.len() > self.capacity {
            self.pictures.remove(0);
        }
    }

    /// §N.4.1.4 / §N.5 — select the prediction reference for a picture
    /// whose header carried `trpi` / `trp`.
    ///
    /// * When `trpi == Some(true)` and `trp` is present, the stored
    ///   picture whose TR equals `trp` is returned. If no such picture
    ///   is in memory, `None` is returned — §N.5: "When the picture for
    ///   which the TR is TRP is not available at the decoder, the
    ///   decoder may send a forced INTRA update signal"; here the caller
    ///   surfaces the absence as a decode error.
    /// * Otherwise (TRP absent or `trpi != Some(true)`) "the most recent
    ///   temporally previous anchor picture shall be used for
    ///   prediction, as when not in the Reference Picture Selection
    ///   mode" — the most recently inserted stored picture.
    pub fn select_reference(&self, trpi: Option<bool>, trp: Option<u16>) -> Option<&YuvFrame> {
        match (trpi, trp) {
            (Some(true), Some(target_tr)) => self
                .pictures
                .iter()
                .find(|p| p.tr == target_tr)
                .map(|p| &p.frame),
            _ => self.most_recent(),
        }
    }

    /// The most recently inserted reference picture, or `None` when the
    /// store is empty.
    pub fn most_recent(&self) -> Option<&YuvFrame> {
        self.pictures.last().map(|p| &p.frame)
    }

    /// Whether a stored picture exists for the given Temporal Reference.
    pub fn contains_tr(&self, tr: u16) -> bool {
        self.pictures.iter().any(|p| p.tr == tr)
    }
}

impl Default for RpsReferenceStore {
    fn default() -> Self {
        Self::new()
    }
}

/// §N.4.1.4 — compose the 10-bit Temporal Reference from the picture
/// header's `TR` (8 bits) and, when a custom picture clock frequency was
/// in use, the `ETR` (2 MSBs). Without a custom PCF the two MSBs are
/// zero.
pub fn compose_tr(tr: u8, etr: Option<u8>) -> u16 {
    let lsb = u16::from(tr);
    match etr {
        Some(e) => (u16::from(e & 0b11) << 8) | lsb,
        None => lsb,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(value: u8) -> YuvFrame {
        let mut f = YuvFrame::grey(16, 16);
        f.y.iter_mut().for_each(|p| *p = value);
        f
    }

    #[test]
    fn compose_tr_without_etr() {
        assert_eq!(compose_tr(200, None), 200);
    }

    #[test]
    fn compose_tr_with_etr() {
        // ETR = 0b10 (only 2 LSBs used), TR = 5 => 0b10_00000101 = 517.
        assert_eq!(compose_tr(5, Some(0b10)), 0b10_0000_0101);
    }

    #[test]
    fn trp_selects_stored_picture_by_tr() {
        let mut store = RpsReferenceStore::new();
        store.insert(10, flat(10));
        store.insert(20, flat(20));
        store.insert(30, flat(30));

        // TRPI = 1, TRP = 20 selects the TR=20 picture, not the latest.
        let r = store
            .select_reference(Some(true), Some(20))
            .expect("TR=20 picture must be present");
        assert_eq!(r.y[0], 20);
    }

    #[test]
    fn trp_absent_selects_most_recent() {
        let mut store = RpsReferenceStore::new();
        store.insert(10, flat(10));
        store.insert(20, flat(20));
        // TRPI = 0 => most recent anchor (TR=20).
        let r = store
            .select_reference(Some(false), None)
            .expect("most-recent picture must be present");
        assert_eq!(r.y[0], 20);
    }

    #[test]
    fn missing_trp_yields_none() {
        let mut store = RpsReferenceStore::new();
        store.insert(10, flat(10));
        // TRP = 99 not in the store => None (forced-INTRA-update case).
        assert!(store.select_reference(Some(true), Some(99)).is_none());
    }

    #[test]
    fn fifo_eviction_at_capacity() {
        let mut store = RpsReferenceStore::with_capacity(2);
        store.insert(1, flat(1));
        store.insert(2, flat(2));
        store.insert(3, flat(3)); // evicts TR=1
        assert_eq!(store.len(), 2);
        assert!(!store.contains_tr(1));
        assert!(store.contains_tr(2));
        assert!(store.contains_tr(3));
    }

    #[test]
    fn reinsert_same_tr_replaces() {
        let mut store = RpsReferenceStore::new();
        store.insert(5, flat(50));
        store.insert(5, flat(60));
        assert_eq!(store.len(), 1);
        assert_eq!(store.most_recent().unwrap().y[0], 60);
    }

    #[test]
    fn empty_store_selects_none() {
        let store = RpsReferenceStore::new();
        assert!(store.select_reference(Some(false), None).is_none());
        assert!(store.most_recent().is_none());
    }
}
