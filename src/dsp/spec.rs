use crate::dsp::biquad::BiquadCascade;
use crate::dsp::design;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterKind {
    /// RBJ cookbook lowpass — single biquad, Q is user-controlled.
    CookbookLowpass,
    CookbookHighpass,
    CookbookBandpass,
    CookbookNotch,
    /// Butterworth lowpass — order is user-controlled, Q ignored.
    ButterworthLowpass,
    /// Chebyshev I lowpass — order + ripple controlled.
    Chebyshev1Lowpass,
}

impl FilterKind {
    pub fn label(&self) -> &'static str {
        match self {
            Self::CookbookLowpass    => "Lowpass (1st-order biquad)",
            Self::CookbookHighpass   => "Highpass (1st-order biquad)",
            Self::CookbookBandpass   => "Bandpass",
            Self::CookbookNotch      => "Notch",
            Self::ButterworthLowpass => "Butterworth LP",
            Self::Chebyshev1Lowpass  => "Chebyshev I LP",
        }
    }

    pub fn uses_q(&self) -> bool {
        matches!(
            self,
            Self::CookbookLowpass
                | Self::CookbookHighpass
                | Self::CookbookBandpass
                | Self::CookbookNotch
        )
    }

    pub fn uses_order(&self) -> bool {
        matches!(self, Self::ButterworthLowpass | Self::Chebyshev1Lowpass)
    }

    pub fn uses_ripple(&self) -> bool {
        matches!(self, Self::Chebyshev1Lowpass)
    }
}

/// High-level user-facing description of a filter. The UI mutates this;
/// `compile()` turns it into runnable DSP.
#[derive(Debug, Clone, Copy)]
pub struct FilterSpec {
    pub enabled:    bool,
    pub kind:       FilterKind,
    pub cutoff_hz:  f64,
    pub q:          f64,
    pub order:      usize,
    pub ripple_db:  f64,
    pub zero_phase: bool,
}

impl Default for FilterSpec {
    fn default() -> Self {
        Self {
            enabled:   false,
            kind:      FilterKind::ButterworthLowpass,
            cutoff_hz: 10.0,
            q:         std::f64::consts::FRAC_1_SQRT_2,  // ≈ 0.7071
            order:     4,
            ripple_db: 1.0,
            zero_phase: false,
        }
    }
}

impl FilterSpec {
    /// Build a runnable cascade for the given sample rate. Returns `None` if
    /// the spec is invalid for that rate (e.g. cutoff above Nyquist).
    pub fn compile(&self, sample_rate_hz: f64) -> Option<BiquadCascade> {
        if sample_rate_hz <= 0.0 {
            return None;
        }
        let nyquist = sample_rate_hz / 2.0;
        // Clamp cutoff a hair below Nyquist so the prewarp's tan() doesn't blow up.
        let fc = self.cutoff_hz.clamp(1.0e-3, nyquist * 0.999);

        let cascade = match self.kind {
            FilterKind::CookbookLowpass => {
                BiquadCascade::from_sections(vec![design::lowpass(sample_rate_hz, fc, self.q)], 1.0)
            }
            FilterKind::CookbookHighpass => {
                BiquadCascade::from_sections(vec![design::highpass(sample_rate_hz, fc, self.q)], 1.0)
            }
            FilterKind::CookbookBandpass => {
                BiquadCascade::from_sections(vec![design::bandpass(sample_rate_hz, fc, self.q)], 1.0)
            }
            FilterKind::CookbookNotch => {
                BiquadCascade::from_sections(vec![design::notch(sample_rate_hz, fc, self.q)], 1.0)
            }
            FilterKind::ButterworthLowpass => {
                design::butterworth_lowpass(sample_rate_hz, fc, self.order.max(1))
            }
            FilterKind::Chebyshev1Lowpass => {
                design::chebyshev1_lowpass(sample_rate_hz, fc, self.order.max(1), self.ripple_db.max(0.01))
            }
        };
        Some(cascade)
    }
}