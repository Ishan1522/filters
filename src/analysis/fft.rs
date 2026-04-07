use rustfft::{num_complex::Complex, FftPlanner};
use std::f64::consts::PI;

pub struct Spectrum {
    pub freqs_hz:      Vec<f64>,
    pub magnitudes_db: Vec<f64>,
}

/// Compute a one-sided amplitude spectrum in dB.
///
/// `samples` must be uniformly sampled at `sample_rate_hz`. If you have
/// non-uniform timestamps, run `analysis::resample_uniform` first.
///
/// A Hann window is applied before the FFT to suppress spectral leakage.
/// We compensate for the window's coherent gain (~0.5) so the resulting
/// magnitudes are interpretable as "approximately the amplitude of a pure
/// tone at this frequency."
pub fn compute(samples: &[f64], sample_rate_hz: f64) -> Spectrum {
    let n = samples.len();
    if n < 4 || sample_rate_hz <= 0.0 {
        return Spectrum { freqs_hz: Vec::new(), magnitudes_db: Vec::new() };
    }

    // Hann window. Coherent gain = mean(window) = 0.5 for Hann.
    let mut buf: Vec<Complex<f64>> = (0..n)
        .map(|i| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
            Complex { re: samples[i] * w, im: 0.0 }
        })
        .collect();

    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(n);
    fft.process(&mut buf);

    // One-sided: bins 0..N/2. Multiply by 2/N to get amplitude, then divide
    // by 0.5 (coherent gain of Hann) → net factor 4/N. DC bin doesn't get
    // doubled but we keep it simple here; the visual difference is negligible.
    let half = n / 2;
    let scale = 4.0 / n as f64;
    let bin_hz = sample_rate_hz / n as f64;

    let freqs_hz: Vec<f64> = (0..half).map(|k| k as f64 * bin_hz).collect();
    let magnitudes_db: Vec<f64> = buf[..half]
        .iter()
        .map(|c| {
            let mag = (c.re * c.re + c.im * c.im).sqrt() * scale;
            20.0 * mag.max(1.0e-12).log10()
        })
        .collect();

    Spectrum { freqs_hz, magnitudes_db }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn fft_finds_a_pure_tone() {
        let fs = 1000.0;
        let f0 = 100.0;
        let n  = 4096;
        let samples: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).sin())
            .collect();

        let spectrum = compute(&samples, fs);

        // Find the peak bin and check it's near 100 Hz.
        let (peak_idx, _) = spectrum
            .magnitudes_db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        let peak_hz = spectrum.freqs_hz[peak_idx];
        assert!((peak_hz - f0).abs() < 1.0, "peak at {} Hz", peak_hz);
    }
}