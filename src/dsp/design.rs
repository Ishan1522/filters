use std::f64::consts::PI;
use super::biquad::Biquad;

/// Lowpass with given cutoff and Q. Q = 1/√2 ≈ 0.7071 gives a Butterworth
/// (maximally flat) response — that's the sensible default.
pub fn lowpass(sample_rate: f64, cutoff_hz: f64, q: f64) -> Biquad {
    let w0 = 2.0 * PI * cutoff_hz / sample_rate;
    let (sw, cw) = w0.sin_cos();
    let alpha = sw / (2.0 * q);

    let b0 = (1.0 - cw) * 0.5;
    let b1 =  1.0 - cw;
    let b2 = (1.0 - cw) * 0.5;
    let a0 =  1.0 + alpha;
    let a1 = -2.0 * cw;
    let a2 =  1.0 - alpha;

    Biquad::new(b0/a0, b1/a0, b2/a0, a1/a0, a2/a0)
}

pub fn highpass(sample_rate: f64, cutoff_hz: f64, q: f64) -> Biquad {
    let w0 = 2.0 * PI * cutoff_hz / sample_rate;
    let (sw, cw) = w0.sin_cos();
    let alpha = sw / (2.0 * q);

    let b0 =  (1.0 + cw) * 0.5;
    let b1 = -(1.0 + cw);
    let b2 =  (1.0 + cw) * 0.5;
    let a0 =  1.0 + alpha;
    let a1 = -2.0 * cw;
    let a2 =  1.0 - alpha;

    Biquad::new(b0/a0, b1/a0, b2/a0, a1/a0, a2/a0)
}

/// Constant-0-dB-peak bandpass.
pub fn bandpass(sample_rate: f64, center_hz: f64, q: f64) -> Biquad {
    let w0 = 2.0 * PI * center_hz / sample_rate;
    let (sw, cw) = w0.sin_cos();
    let alpha = sw / (2.0 * q);

    let b0 =  alpha;
    let b1 =  0.0;
    let b2 = -alpha;
    let a0 =  1.0 + alpha;
    let a1 = -2.0 * cw;
    let a2 =  1.0 - alpha;

    Biquad::new(b0/a0, b1/a0, b2/a0, a1/a0, a2/a0)
}

/// Notch — kills a single frequency (e.g. 60 Hz hum). High Q = narrow notch.
pub fn notch(sample_rate: f64, center_hz: f64, q: f64) -> Biquad {
    let w0 = 2.0 * PI * center_hz / sample_rate;
    let (sw, cw) = w0.sin_cos();
    let alpha = sw / (2.0 * q);

    let b0 =  1.0;
    let b1 = -2.0 * cw;
    let b2 =  1.0;
    let a0 =  1.0 + alpha;
    let a1 = -2.0 * cw;
    let a2 =  1.0 - alpha;

    Biquad::new(b0/a0, b1/a0, b2/a0, a1/a0, a2/a0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsp::biquad::BiquadCascade;

    #[test]
    fn lowpass_passes_dc() {
        // At DC (ω=0), a lowpass should have magnitude ≈ 1.
        let bq = lowpass(1000.0, 100.0, std::f64::consts::FRAC_1_SQRT_2);
        let (mag, _) = bq.frequency_response(0.0);
        assert!((mag - 1.0).abs() < 1e-6, "DC mag = {}", mag);
    }

    #[test]
    fn highpass_blocks_dc() {
        let bq = highpass(1000.0, 100.0, std::f64::consts::FRAC_1_SQRT_2);
        let (mag, _) = bq.frequency_response(0.0);
        assert!(mag < 1e-6, "DC mag = {}", mag);
    }

    #[test]
    fn notch_kills_center() {
        let fs = 1000.0;
        let f0 = 60.0;
        let bq = notch(fs, f0, 30.0);
        let omega = 2.0 * std::f64::consts::PI * f0 / fs;
        let (mag, _) = bq.frequency_response(omega);
        assert!(mag < 1e-3, "notch mag at center = {}", mag);
    }

    #[test]
    fn cascade_step_response_is_finite() {
        // Sanity check: 4th-order LPF cascade processes a step without blowing up.
        let mut cascade = BiquadCascade::from_sections(
            vec![
                lowpass(1000.0, 50.0, 0.5412),  // Butterworth pole pair 1
                lowpass(1000.0, 50.0, 1.3066),  // Butterworth pole pair 2
            ],
            1.0,
        );
        let mut y = 0.0;
        for _ in 0..10_000 {
            y = cascade.process(1.0);
            assert!(y.is_finite());
        }
        assert!((y - 1.0).abs() < 1e-3, "settled value = {}", y);
    }
    #[test]
fn butterworth_minus_3db_at_cutoff() {
    // The defining property of Butterworth: |H(fc)| = 1/√2, regardless of order.
    let fs = 1000.0;
    let fc = 100.0;
    let target = std::f64::consts::FRAC_1_SQRT_2;
    for order in 1..=8 {
        let cascade = butterworth_lowpass(fs, fc, order);
        let omega = 2.0 * std::f64::consts::PI * fc / fs;
        let (mag, _) = cascade.frequency_response(omega);
        assert!(
            (mag - target).abs() < 1e-3,
            "order {}: |H(fc)| = {} (expected {})",
            order, mag, target,
        );
    }
}

#[test]
fn butterworth_dc_gain_is_unity() {
    for order in 1..=8 {
        let cascade = butterworth_lowpass(1000.0, 100.0, order);
        let (mag, _) = cascade.frequency_response(0.0);
        assert!((mag - 1.0).abs() < 1e-6, "order {}: DC = {}", order, mag);
    }
}

#[test]
fn chebyshev1_passband_ripple_bounds() {
    // Inside the passband, |H| should oscillate between 1 and 1/√(1+ε²).
    let fs = 1000.0;
    let fc = 100.0;
    let ripple_db = 1.0;
    let epsilon_sq = 10f64.powf(ripple_db / 10.0) - 1.0;
    let lower = 1.0 / (1.0 + epsilon_sq).sqrt();
    let order = 5;
    let cascade = chebyshev1_lowpass(fs, fc, order, ripple_db);

    // Sweep the passband and check magnitude stays in [lower, 1] (with tolerance).
    for i in 0..=100 {
        let f = fc * i as f64 / 100.0;
        let omega = 2.0 * std::f64::consts::PI * f / fs;
        let (mag, _) = cascade.frequency_response(omega);
        assert!(
            mag <= 1.0 + 1e-3 && mag >= lower - 1e-3,
            "f={}: |H|={} not in [{}, 1]",
            f, mag, lower,
        );
    }
}

#[test]
fn chebyshev1_at_cutoff_equals_ripple_floor() {
    // At fc exactly, Chebyshev I lands on the ripple floor by definition.
    let fs = 1000.0;
    let fc = 100.0;
    let ripple_db = 1.0;
    let epsilon_sq = 10f64.powf(ripple_db / 10.0) - 1.0;
    let expected = 1.0 / (1.0 + epsilon_sq).sqrt();
    for order in 2..=6 {
        let cascade = chebyshev1_lowpass(fs, fc, order, ripple_db);
        let omega = 2.0 * std::f64::consts::PI * fc / fs;
        let (mag, _) = cascade.frequency_response(omega);
        assert!(
            (mag - expected).abs() < 5e-3,
            "order {}: |H(fc)| = {} (expected {})",
            order, mag, expected,
        );
    }
}
}

use crate::dsp::biquad::BiquadCascade;

/// Bilinear-transform a 2nd-order analog lowpass prototype section.
///
/// `omega0_proto` is the section's natural frequency in *normalized prototype
/// units* (where the overall filter cutoff = 1 rad/s). For Butterworth this is
/// always 1; for Chebyshev I it varies per section.
fn lp_section_from_prototype(
    sample_rate: f64,
    cutoff_hz: f64,
    omega0_proto: f64,
    q: f64,
) -> Biquad {
    let k = (PI * cutoff_hz / sample_rate).tan();    // prewarp
    let w = omega0_proto * k;
    let w2 = w * w;
    let a0 = 1.0 + w / q + w2;
    Biquad::new(
        w2 / a0,
        2.0 * w2 / a0,
        w2 / a0,
        2.0 * (w2 - 1.0) / a0,
        (1.0 - w / q + w2) / a0,
    )
}

/// First-order lowpass section. Used for the leftover real pole when the
/// filter order is odd. `omega0_proto` is the prototype-units location of the
/// real pole on the negative real axis (1 for Butterworth, sinh(μ) for Cheby I).
fn first_order_lp_section(sample_rate: f64, cutoff_hz: f64, omega0_proto: f64) -> Biquad {
    let k = (PI * cutoff_hz / sample_rate).tan();
    let w = omega0_proto * k;
    let denom = 1.0 + w;
    // H(s) = w / (s + w)  →  bilinear  →  biquad with b2=a2=0
    Biquad::new(
        w / denom,        // b0
        w / denom,        // b1
        0.0,              // b2
        (w - 1.0) / denom,// a1   (note: derivation gives -(1-w)/(1+w))
        0.0,              // a2
    )
}

/// Butterworth lowpass of arbitrary order. Maximally flat passband, no ripple.
/// `order` ≥ 1. Higher order = steeper rolloff but more phase distortion.
pub fn butterworth_lowpass(sample_rate: f64, cutoff_hz: f64, order: usize) -> BiquadCascade {
    assert!(order >= 1, "order must be ≥ 1");
    assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0, "cutoff must be in (0, fs/2)");

    let mut sections = Vec::with_capacity(order.div_ceil(2));
    let n_pairs = order / 2;

    // Conjugate pole pairs, evenly spaced on the LHP unit circle.
    for k in 1..=n_pairs {
        let phi = (2 * k - 1) as f64 * PI / (2.0 * order as f64);
        let q = 1.0 / (2.0 * phi.sin());
        sections.push(lp_section_from_prototype(sample_rate, cutoff_hz, 1.0, q));
    }

    // Odd order: leftover real pole at s = -1.
    if order % 2 == 1 {
        sections.push(first_order_lp_section(sample_rate, cutoff_hz, 1.0));
    }

    BiquadCascade::from_sections(sections, 1.0)
}

/// Chebyshev Type I lowpass. Equiripple in passband, monotonic in stopband.
/// `ripple_db` is the peak-to-peak passband ripple — typical values 0.1 to 3 dB.
/// Smaller ripple = closer to Butterworth; larger ripple = sharper rolloff.
pub fn chebyshev1_lowpass(
    sample_rate: f64,
    cutoff_hz: f64,
    order: usize,
    ripple_db: f64,
) -> BiquadCascade {
    assert!(order >= 1, "order must be ≥ 1");
    assert!(ripple_db > 0.0, "ripple must be > 0");
    assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0, "cutoff must be in (0, fs/2)");

    let epsilon = (10f64.powf(ripple_db / 10.0) - 1.0).sqrt();
    let mu = (1.0 / epsilon).asinh() / order as f64;
    let sinh_mu = mu.sinh();
    let cosh_mu = mu.cosh();

    let mut sections = Vec::with_capacity(order.div_ceil(2));
    let n_pairs = order / 2;

    // Pole pairs on the ellipse.
    for k in 1..=n_pairs {
        let theta = (2 * k - 1) as f64 * PI / (2.0 * order as f64);
        let sigma = -sinh_mu * theta.sin();      // real part (negative)
        let omega =  cosh_mu * theta.cos();      // imaginary part
        let omega0 = (sigma * sigma + omega * omega).sqrt();
        let q = omega0 / (-2.0 * sigma);
        sections.push(lp_section_from_prototype(sample_rate, cutoff_hz, omega0, q));
    }

    // Odd order: real pole at s = -sinh(μ).
    if order % 2 == 1 {
        sections.push(first_order_lp_section(sample_rate, cutoff_hz, sinh_mu));
    }

    // Even-order Chebyshev I has DC gain 1/√(1+ε²) (ripple "starts low" at DC).
    // Odd-order has DC gain 1. Match scipy / MATLAB convention.
    let gain = if order % 2 == 0 {
        1.0 / (1.0 + epsilon * epsilon).sqrt()
    } else {
        1.0
    };

    BiquadCascade::from_sections(sections, gain)
}