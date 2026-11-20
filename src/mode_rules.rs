//! Optional-mode combination rules of ITU-T H.263 (01/2005).
//!
//! The Recommendation lets most annexes be combined freely, but a
//! handful of clauses forbid specific pairings or tie a mode to a
//! picture type or to the presence of PLUSPTYPE. [`ModeSet::check`]
//! applies every such rule to a requested mode set: an illegal request
//! is refused with [`Error::ModeCombination`](crate::Error::ModeCombination)
//! carrying the [`ModeRule`] that names the violated clause, and a
//! legal one returns the *effective* set after the §5.1.4.5 mode
//! inference (a mode that "does not apply" within the picture type has
//! an inferred state of "off" for that picture while its OPPTYPE flag
//! may stay set).
//!
//! The encoders call this before writing a picture header, so a stream
//! this crate emits never signals a forbidden combination.

use crate::plus_ptype::{PlusPictureType, SliceStructuredSubmode};

/// One mode-combination rule of the Recommendation. The variant names
/// the pairing; [`ModeRule::clause`] names the clause that states it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ModeRule {
    /// §5.1.4.4 / §G.1 — "the PB-frames mode of Annex G cannot be used
    /// with PLUSPTYPE present – the Improved PB-frames mode of Annex M
    /// should be used instead".
    PbFramesWithPlusPtype,
    /// §5.1.4.2 / §5.1.4.3 — the mode's flag exists only in PLUSPTYPE
    /// (Annexes I, J, K, M, N, O, P, Q, R, S, T, V and RTYPE), but the
    /// picture was requested on the baseline PTYPE header.
    NeedsPlusPtype,
    /// §5.1.3 — PB-frames mode (PTYPE bit 13) applies to INTER
    /// pictures only.
    PbFramesInIntraPicture,
    /// §5.1.4.5 item 4 — "the Reduced-Resolution Update mode bit shall
    /// not be set in an INTRA picture" (items 1 and 3: the mode does
    /// not apply within I- and EI-pictures).
    RruInIntraPicture,
    /// §5.1.4.5 item 4 — the Reference Picture Resampling bit shall
    /// not be set unless the picture type allows the mode (items 1 and
    /// 3: not within I- and EI-pictures).
    RprInIntraPicture,
    /// §5.1.4.3 — RTYPE "can be set to 1 only when bits 1-3 indicate a
    /// P-picture, Improved PB-frame, or EP-picture".
    RoundingTypeInNonPredictedPicture,
    /// §5.1.4.6 item 1 — SAC shall not be used with the Alternative
    /// INTER VLC mode.
    SacWithAlternativeInterVlc,
    /// §5.1.4.6 item 1 — SAC shall not be used with the Modified
    /// Quantization mode.
    SacWithModifiedQuantization,
    /// §5.1.4.6 item 2 — "if PLUSPTYPE is present, the Unrestricted
    /// Motion Vector mode shall not be used with the Syntax-based
    /// Arithmetic Coding mode".
    SacWithUmvUnderPlusPtype,
    /// §5.1.4.6 item 3 / §R.2 item 7 — Independent Segment Decoding
    /// shall not be used with Reference Picture Resampling.
    IsdWithReferencePictureResampling,
    /// §5.1.4.6 item 4 / §R.3.1 — Independent Segment Decoding shall
    /// not be used with the Slice Structured mode without the
    /// Rectangular Slice submode.
    IsdWithNonRectangularSlices,
    /// §V.3 — "the SS mode shall therefore be indicated as being in
    /// use whenever the DPS mode is in use".
    DpsWithoutSliceStructured,
    /// §V.3 — "the Syntax-Based Arithmetic Coding mode of Annex E
    /// shall not be used with this annex".
    DpsWithSac,
    /// §F.2 — four motion vectors per macroblock exist only "when in
    /// Advanced Prediction mode or Deblocking Filter mode".
    FourVectorsWithoutApOrDf,
}

impl ModeRule {
    /// The clause of the Recommendation that states the rule.
    pub fn clause(self) -> &'static str {
        match self {
            ModeRule::PbFramesWithPlusPtype => "5.1.4.4",
            ModeRule::NeedsPlusPtype => "5.1.4.2",
            ModeRule::PbFramesInIntraPicture => "5.1.3",
            ModeRule::RruInIntraPicture | ModeRule::RprInIntraPicture => "5.1.4.5 item 4",
            ModeRule::RoundingTypeInNonPredictedPicture => "5.1.4.3",
            ModeRule::SacWithAlternativeInterVlc | ModeRule::SacWithModifiedQuantization => {
                "5.1.4.6 item 1"
            }
            ModeRule::SacWithUmvUnderPlusPtype => "5.1.4.6 item 2",
            ModeRule::IsdWithReferencePictureResampling => "5.1.4.6 item 3",
            ModeRule::IsdWithNonRectangularSlices => "5.1.4.6 item 4",
            ModeRule::DpsWithoutSliceStructured | ModeRule::DpsWithSac => "V.3",
            ModeRule::FourVectorsWithoutApOrDf => "F.2",
        }
    }
}

impl std::fmt::Display for ModeRule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let what = match self {
            ModeRule::PbFramesWithPlusPtype => "PB-frames (Annex G) with PLUSPTYPE present",
            ModeRule::NeedsPlusPtype => "mode signalled only in PLUSPTYPE on a baseline header",
            ModeRule::PbFramesInIntraPicture => "PB-frames mode in an INTRA picture",
            ModeRule::RruInIntraPicture => "Reduced-Resolution Update in an I- or EI-picture",
            ModeRule::RprInIntraPicture => "Reference Picture Resampling in an I- or EI-picture",
            ModeRule::RoundingTypeInNonPredictedPicture => {
                "RTYPE set outside P / Improved PB / EP pictures"
            }
            ModeRule::SacWithAlternativeInterVlc => "SAC with Alternative INTER VLC",
            ModeRule::SacWithModifiedQuantization => "SAC with Modified Quantization",
            ModeRule::SacWithUmvUnderPlusPtype => "SAC with UMV under PLUSPTYPE",
            ModeRule::IsdWithReferencePictureResampling => {
                "Independent Segment Decoding with Reference Picture Resampling"
            }
            ModeRule::IsdWithNonRectangularSlices => {
                "Independent Segment Decoding with non-rectangular slices"
            }
            ModeRule::DpsWithoutSliceStructured => {
                "Data-Partitioned Slices without Slice Structured"
            }
            ModeRule::DpsWithSac => "Data-Partitioned Slices with SAC",
            ModeRule::FourVectorsWithoutApOrDf => {
                "four motion vectors without Advanced Prediction or Deblocking Filter"
            }
        };
        write!(f, "{what} (H.263 clause {})", self.clause())
    }
}

/// A requested set of optional modes for one picture.
///
/// `Default` is a baseline-PTYPE I-picture with every mode off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct ModeSet {
    /// Picture type (the MPPTYPE code; on a baseline header only
    /// `Intra` / `Inter` exist).
    pub picture_type: PlusPictureType,
    /// PLUSPTYPE present (PTYPE bits 6-8 = "111").
    pub plus_ptype: bool,
    /// Annex D Unrestricted Motion Vectors.
    pub umv: bool,
    /// Annex E Syntax-based Arithmetic Coding.
    pub sac: bool,
    /// Annex F Advanced Prediction.
    pub advanced_prediction: bool,
    /// Annex G PB-frames (baseline PTYPE bit 13).
    pub pb_frames: bool,
    /// Annex I Advanced INTRA Coding.
    pub advanced_intra: bool,
    /// Annex J Deblocking Filter.
    pub deblocking: bool,
    /// Annex K Slice Structured mode with its SSS submodes.
    pub slice_structured: Option<SliceStructuredSubmode>,
    /// Annex N Reference Picture Selection.
    pub reference_picture_selection: bool,
    /// Annex R Independent Segment Decoding.
    pub independent_segment_decoding: bool,
    /// Annex S Alternative INTER VLC.
    pub alt_inter_vlc: bool,
    /// Annex T Modified Quantization.
    pub modified_quant: bool,
    /// Annex V Data-Partitioned Slices.
    pub data_partitioned_slices: bool,
    /// Annex P Reference Picture Resampling (MPPTYPE bit 4).
    pub reference_picture_resampling: bool,
    /// Annex Q Reduced-Resolution Update (MPPTYPE bit 5).
    pub reduced_resolution_update: bool,
    /// RTYPE (MPPTYPE bit 6).
    pub rounding_type: bool,
    /// The encoder intends to emit INTER4V macroblocks.
    pub four_vectors: bool,
}

impl Default for ModeSet {
    fn default() -> Self {
        ModeSet {
            picture_type: PlusPictureType::Intra,
            plus_ptype: false,
            umv: false,
            sac: false,
            advanced_prediction: false,
            pb_frames: false,
            advanced_intra: false,
            deblocking: false,
            slice_structured: None,
            reference_picture_selection: false,
            independent_segment_decoding: false,
            alt_inter_vlc: false,
            modified_quant: false,
            data_partitioned_slices: false,
            reference_picture_resampling: false,
            reduced_resolution_update: false,
            rounding_type: false,
            four_vectors: false,
        }
    }
}

macro_rules! setter {
    ($(#[$m:meta])* $name:ident, $field:ident, $ty:ty) => {
        $(#[$m])*
        #[must_use]
        pub fn $name(mut self, v: $ty) -> Self {
            self.$field = v;
            self
        }
    };
}

impl ModeSet {
    /// A mode set for `picture_type` with every optional mode off.
    pub fn new(picture_type: PlusPictureType, plus_ptype: bool) -> Self {
        ModeSet {
            picture_type,
            plus_ptype,
            ..ModeSet::default()
        }
    }

    setter!(
        /// Set [`ModeSet::umv`].
        with_umv,
        umv,
        bool
    );
    setter!(
        /// Set [`ModeSet::sac`].
        with_sac,
        sac,
        bool
    );
    setter!(
        /// Set [`ModeSet::advanced_prediction`].
        with_advanced_prediction,
        advanced_prediction,
        bool
    );
    setter!(
        /// Set [`ModeSet::pb_frames`].
        with_pb_frames,
        pb_frames,
        bool
    );
    setter!(
        /// Set [`ModeSet::advanced_intra`].
        with_advanced_intra,
        advanced_intra,
        bool
    );
    setter!(
        /// Set [`ModeSet::deblocking`].
        with_deblocking,
        deblocking,
        bool
    );
    setter!(
        /// Set [`ModeSet::slice_structured`].
        with_slice_structured,
        slice_structured,
        Option<SliceStructuredSubmode>
    );
    setter!(
        /// Set [`ModeSet::reference_picture_selection`].
        with_reference_picture_selection,
        reference_picture_selection,
        bool
    );
    setter!(
        /// Set [`ModeSet::independent_segment_decoding`].
        with_independent_segment_decoding,
        independent_segment_decoding,
        bool
    );
    setter!(
        /// Set [`ModeSet::alt_inter_vlc`].
        with_alt_inter_vlc,
        alt_inter_vlc,
        bool
    );
    setter!(
        /// Set [`ModeSet::modified_quant`].
        with_modified_quant,
        modified_quant,
        bool
    );
    setter!(
        /// Set [`ModeSet::data_partitioned_slices`].
        with_data_partitioned_slices,
        data_partitioned_slices,
        bool
    );
    setter!(
        /// Set [`ModeSet::reference_picture_resampling`].
        with_reference_picture_resampling,
        reference_picture_resampling,
        bool
    );
    setter!(
        /// Set [`ModeSet::reduced_resolution_update`].
        with_reduced_resolution_update,
        reduced_resolution_update,
        bool
    );
    setter!(
        /// Set [`ModeSet::rounding_type`].
        with_rounding_type,
        rounding_type,
        bool
    );
    setter!(
        /// Set [`ModeSet::four_vectors`].
        with_four_vectors,
        four_vectors,
        bool
    );

    /// The first rule the set violates, if any.
    pub fn violation(&self) -> Option<ModeRule> {
        use PlusPictureType as T;
        let t = self.picture_type;
        if self.plus_ptype {
            if self.pb_frames {
                return Some(ModeRule::PbFramesWithPlusPtype);
            }
        } else {
            let plus_only = self.advanced_intra
                || self.deblocking
                || self.slice_structured.is_some()
                || self.reference_picture_selection
                || self.independent_segment_decoding
                || self.alt_inter_vlc
                || self.modified_quant
                || self.data_partitioned_slices
                || self.reference_picture_resampling
                || self.reduced_resolution_update
                || self.rounding_type
                || !matches!(t, T::Intra | T::Inter);
            if plus_only {
                return Some(ModeRule::NeedsPlusPtype);
            }
            if self.pb_frames && matches!(t, T::Intra) {
                return Some(ModeRule::PbFramesInIntraPicture);
            }
        }
        let intra_like = matches!(t, T::Intra | T::EiPicture);
        if self.reduced_resolution_update && intra_like {
            return Some(ModeRule::RruInIntraPicture);
        }
        if self.reference_picture_resampling && intra_like {
            return Some(ModeRule::RprInIntraPicture);
        }
        if self.rounding_type && !matches!(t, T::Inter | T::ImprovedPb | T::EpPicture) {
            return Some(ModeRule::RoundingTypeInNonPredictedPicture);
        }
        // §5.1.4.6 speaks of modes in *use*: apply it to the effective
        // (post-inference) state of the picture.
        let e = self.inferred();
        if e.sac && e.alt_inter_vlc {
            return Some(ModeRule::SacWithAlternativeInterVlc);
        }
        if e.sac && e.modified_quant {
            return Some(ModeRule::SacWithModifiedQuantization);
        }
        if e.plus_ptype && e.sac && e.umv {
            return Some(ModeRule::SacWithUmvUnderPlusPtype);
        }
        if e.independent_segment_decoding && e.reference_picture_resampling {
            return Some(ModeRule::IsdWithReferencePictureResampling);
        }
        if e.independent_segment_decoding && e.slice_structured.is_some_and(|s| !s.rectangular) {
            return Some(ModeRule::IsdWithNonRectangularSlices);
        }
        if e.data_partitioned_slices && e.slice_structured.is_none() {
            return Some(ModeRule::DpsWithoutSliceStructured);
        }
        if e.data_partitioned_slices && e.sac {
            return Some(ModeRule::DpsWithSac);
        }
        if e.four_vectors && !(e.advanced_prediction || e.deblocking) {
            return Some(ModeRule::FourVectorsWithoutApOrDf);
        }
        None
    }

    /// The set after the §5.1.4.5 mode inference: every mode that
    /// "does not apply" within the picture type is off (items 1–4).
    /// On a baseline header the PTYPE flags are taken as sent.
    pub fn inferred(&self) -> ModeSet {
        use PlusPictureType as T;
        let mut e = *self;
        match self.picture_type {
            T::Intra => {
                e.umv = false;
                e.advanced_prediction = false;
                e.alt_inter_vlc = false;
                e.four_vectors = false;
            }
            T::BPicture => {
                e.sac = false;
                e.deblocking = false;
                e.advanced_prediction = false;
            }
            T::EiPicture => {
                e.umv = false;
                e.sac = false;
                e.advanced_prediction = false;
                e.alt_inter_vlc = false;
            }
            T::EpPicture => {
                e.sac = false;
                e.advanced_prediction = false;
            }
            T::Inter | T::ImprovedPb => {}
        }
        e
    }

    /// Validate the set and return its effective (inferred) form.
    ///
    /// # Errors
    ///
    /// [`Error::ModeCombination`](crate::Error::ModeCombination) with
    /// the violated [`ModeRule`].
    pub fn check(&self) -> crate::Result<ModeSet> {
        match self.violation() {
            Some(rule) => Err(crate::Error::ModeCombination(rule)),
            None => Ok(self.inferred()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use PlusPictureType as T;

    fn rs(rectangular: bool) -> Option<SliceStructuredSubmode> {
        Some(SliceStructuredSubmode {
            rectangular,
            arbitrary_order: false,
        })
    }

    #[test]
    fn baseline_default_is_legal() {
        assert_eq!(ModeSet::default().violation(), None);
        let v1 = ModeSet::new(T::Inter, false)
            .with_umv(true)
            .with_sac(true)
            .with_advanced_prediction(true)
            .with_pb_frames(true);
        // §5.1.4.6 item 2 binds only when PLUSPTYPE is present.
        assert_eq!(v1.violation(), None);
    }

    #[test]
    fn every_rule_fires_and_names_its_clause() {
        let p = ModeSet::new(T::Inter, true);
        let cases: Vec<(ModeSet, ModeRule)> = vec![
            (p.with_pb_frames(true), ModeRule::PbFramesWithPlusPtype),
            (
                ModeSet::new(T::Inter, false).with_deblocking(true),
                ModeRule::NeedsPlusPtype,
            ),
            (ModeSet::new(T::ImprovedPb, false), ModeRule::NeedsPlusPtype),
            (
                ModeSet::new(T::Intra, false).with_pb_frames(true),
                ModeRule::PbFramesInIntraPicture,
            ),
            (
                ModeSet::new(T::Intra, true).with_reduced_resolution_update(true),
                ModeRule::RruInIntraPicture,
            ),
            (
                ModeSet::new(T::EiPicture, true).with_reference_picture_resampling(true),
                ModeRule::RprInIntraPicture,
            ),
            (
                ModeSet::new(T::BPicture, true).with_rounding_type(true),
                ModeRule::RoundingTypeInNonPredictedPicture,
            ),
            (
                p.with_sac(true).with_alt_inter_vlc(true),
                ModeRule::SacWithAlternativeInterVlc,
            ),
            (
                p.with_sac(true).with_modified_quant(true),
                ModeRule::SacWithModifiedQuantization,
            ),
            (
                p.with_sac(true).with_umv(true),
                ModeRule::SacWithUmvUnderPlusPtype,
            ),
            (
                p.with_independent_segment_decoding(true)
                    .with_reference_picture_resampling(true),
                ModeRule::IsdWithReferencePictureResampling,
            ),
            (
                p.with_independent_segment_decoding(true)
                    .with_slice_structured(rs(false)),
                ModeRule::IsdWithNonRectangularSlices,
            ),
            (
                p.with_data_partitioned_slices(true),
                ModeRule::DpsWithoutSliceStructured,
            ),
            (
                p.with_data_partitioned_slices(true)
                    .with_slice_structured(rs(false))
                    .with_sac(true),
                ModeRule::DpsWithSac,
            ),
            (
                p.with_four_vectors(true),
                ModeRule::FourVectorsWithoutApOrDf,
            ),
        ];
        for (set, rule) in cases {
            assert_eq!(set.violation(), Some(rule), "{set:?}");
            assert_eq!(set.check(), Err(crate::Error::ModeCombination(rule)));
            assert!(rule.to_string().contains(rule.clause()));
        }
    }

    #[test]
    fn inference_turns_inapplicable_modes_off_instead_of_refusing() {
        // An I-picture may carry UMV / AP / AIV flags (§5.1.4.5): they
        // are inferred off, so SAC + UMV does not conflict there.
        let i = ModeSet::new(T::Intra, true)
            .with_sac(true)
            .with_umv(true)
            .with_advanced_prediction(true)
            .with_alt_inter_vlc(true);
        let e = i.check().unwrap();
        assert!(e.sac && !e.umv && !e.advanced_prediction && !e.alt_inter_vlc);
        // SAC is inferred off in B / EI / EP pictures.
        for t in [T::BPicture, T::EiPicture, T::EpPicture] {
            let e = ModeSet::new(t, true)
                .with_sac(true)
                .with_modified_quant(true)
                .check()
                .unwrap();
            assert!(!e.sac && e.modified_quant);
        }
    }

    #[test]
    fn legal_pairs_of_the_matrix_pass() {
        let p = ModeSet::new(T::Inter, true);
        let legal = [
            p.with_reduced_resolution_update(true)
                .with_advanced_prediction(true)
                .with_four_vectors(true),
            p.with_reduced_resolution_update(true)
                .with_slice_structured(rs(false)),
            ModeSet::new(T::ImprovedPb, true).with_sac(true),
            p.with_independent_segment_decoding(true)
                .with_slice_structured(rs(true)),
            p.with_deblocking(true).with_four_vectors(true),
            p.with_umv(true)
                .with_advanced_prediction(true)
                .with_advanced_intra(true)
                .with_deblocking(true)
                .with_modified_quant(true)
                .with_alt_inter_vlc(true)
                .with_slice_structured(rs(false)),
        ];
        for set in legal {
            assert_eq!(set.violation(), None, "{set:?}");
        }
    }
}
