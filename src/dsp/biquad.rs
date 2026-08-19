/// A single biquad section, Direct Form II Transposed.
///
/// Transfer function (a0 normalized to 1):
///
///         b0 + b1·z⁻¹ + b2·z⁻²
///  H(z) = ──────────────────────
///          1 + a1·z⁻¹ + a2·z⁻²
///
/// DF-II Transposed difference equation:
///   y[n]  = b0·x[n] + z1
///   z1[n] = b1·x[n] − a1·y[n] + z2
///   z2[n] = b2·x[n] − a2·y[n]
#[derive(Debug, Clone, Copy)]
pub struct Biquad {
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
    // state — two unit delays
    z1: f64,
    z2: f64,
}

impl Biquad {
    pub fn new(b0: f64, b1: f64, b2: f64, a1: f64, a2: f64) -> Self {
        Self { b0, b1, b2, a1, a2, z1: 0.0, z2: 0.0 }
    }

    /// A passthrough biquad.
    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0, 0.0)
    }

    /// Clear the delay line. Call this when starting a new signal so the
    /// filter doesn't ring with leftover state from the previous one.
    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }

    /// Process a single sample. Inlined because this is the inner loop
    /// of literally everything.
    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        let y  = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }

    /// Process a slice. Equivalent to calling `process` in a loop, but
    /// keeps the state local so the optimizer can keep z1/z2 in registers.
    pub fn process_block(&mut self, input: &[f64], output: &mut [f64]) {
        debug_assert_eq!(input.len(), output.len());
        let (mut z1, mut z2) = (self.z1, self.z2);
        for (i, &x) in input.iter().enumerate() {
            let y = self.b0 * x + z1;
            z1 = self.b1 * x - self.a1 * y + z2;
            z2 = self.b2 * x - self.a2 * y;
            output[i] = y;
        }
        self.z1 = z1;
        self.z2 = z2;
    }

    /// Evaluate H(e^{jω}) for plotting magnitude/phase response.
    /// `omega` is normalized angular frequency in radians/sample (0..π).
    /// Returns (magnitude, phase_radians).
    pub fn frequency_response(&self, omega: f64) -> (f64, f64) {
        // z = e^{jω}  ⟹  z⁻¹ = cos(ω) − j·sin(ω),  z⁻² = cos(2ω) − j·sin(2ω)
        let (s1, c1) = omega.sin_cos();
        let (s2, c2) = (2.0 * omega).sin_cos();

        // numerator: b0 + b1·z⁻¹ + b2·z⁻²
        let num_re = self.b0 + self.b1 * c1 + self.b2 * c2;
        let num_im =          -self.b1 * s1 - self.b2 * s2;

        // denominator: 1 + a1·z⁻¹ + a2·z⁻²
        let den_re = 1.0 + self.a1 * c1 + self.a2 * c2;
        let den_im =      -self.a1 * s1 - self.a2 * s2;

        // complex division: (num_re + j·num_im) / (den_re + j·den_im)
        let den_mag2 = den_re * den_re + den_im * den_im;
        let h_re = (num_re * den_re + num_im * den_im) / den_mag2;
        let h_im = (num_im * den_re - num_re * den_im) / den_mag2;

        let mag   = (h_re * h_re + h_im * h_im).sqrt();
        let phase = h_im.atan2(h_re);
        (mag, phase)
    }

    /// The two poles of this biquad in the z-plane.
    /// Roots of: z² + a1·z + a2 = 0
    pub fn poles(&self) -> (Complex, Complex) {
        quadratic_roots(1.0, self.a1, self.a2)
    }

    /// The two zeros of this biquad in the z-plane.
    /// Roots of: b0·z² + b1·z + b2 = 0
    pub fn zeros(&self) -> (Complex, Complex) {
        quadratic_roots(self.b0, self.b1, self.b2)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Complex { pub re: f64, pub im: f64 }

/// Roots of the quadratic a·z² + b·z + c = 0 via the standard formula.
///
/// Returns a conjugate pair `(r1, r2)` where:
/// - real roots if discriminant ≥ 0
/// - complex conjugates if discriminant < 0
///
/// If `a ≈ 0` (degenerate), returns `(0+0i, 0+0i)`.
fn quadratic_roots(a: f64, b: f64, c: f64) -> (Complex, Complex) {
    if a.abs() < 1e-18 {
        // degenerate — only one "root", duplicated at origin
        return (Complex { re: 0.0, im: 0.0 }, Complex { re: 0.0, im: 0.0 });
    }
    let disc = b * b - 4.0 * a * c;
    if disc >= 0.0 {
        let s = disc.sqrt();
        (
            Complex { re: (-b + s) / (2.0 * a), im: 0.0 },
            Complex { re: (-b - s) / (2.0 * a), im: 0.0 },
        )
    } else {
        let s = (-disc).sqrt();
        (
            Complex { re: -b / (2.0 * a), im:  s / (2.0 * a) },
            Complex { re: -b / (2.0 * a), im: -s / (2.0 * a) },
        )
    }
}

/// A cascade of biquad sections plus a global gain. This is how every
/// high-order IIR filter you'll ever care about gets built.
#[derive(Debug, Clone, Default)]
pub struct BiquadCascade {
    pub sections: Vec<Biquad>,
    pub gain: f64,
}

impl BiquadCascade {
    pub fn new() -> Self {
        Self { sections: Vec::new(), gain: 1.0 }
    }

    pub fn from_sections(sections: Vec<Biquad>, gain: f64) -> Self {
        Self { sections, gain }
    }

    pub fn process(&mut self, x: f64) -> f64 {
        let mut s = x * self.gain;
        for bq in &mut self.sections {
            s = bq.process(s);
        }
        s
    }

    pub fn process_block(&mut self, input: &[f64], output: &mut [f64]) {
        // Apply gain into the output buffer first, then run each section in place.
        for (i, &x) in input.iter().enumerate() {
            output[i] = x * self.gain;
        }
        // We need a scratch buffer because process_block writes input→output;
        // for in-place we just alias.
        for bq in &mut self.sections {
            // Safety: in-place processing, same buffer as both src and dst.
            // process_block reads index i before writing it, so this is fine.
            let ptr = output.as_mut_ptr();
            let len = output.len();
            unsafe {
                let src = std::slice::from_raw_parts(ptr, len);
                let dst = std::slice::from_raw_parts_mut(ptr, len);
                bq.process_block(src, dst);
            }
        }
    }

    pub fn reset(&mut self) {
        for bq in &mut self.sections {
            bq.reset();
        }
    }

    /// Total magnitude response is the *product* of section magnitudes,
    /// total phase is the *sum* — because cascading is multiplication of H(z).
    pub fn frequency_response(&self, omega: f64) -> (f64, f64) {
        let mut mag = self.gain.abs();
        let mut phase = if self.gain < 0.0 { std::f64::consts::PI } else { 0.0 };
        for bq in &self.sections {
            let (m, p) = bq.frequency_response(omega);
            mag *= m;
            phase += p;
        }
        (mag, phase)
    }

/// Zero-phase filtering via forward-backward passes with reflection padding
/// to suppress edge transients. The result has zero phase distortion and
/// magnitude response |H(ω)|² (effectively double-order). OFFLINE ONLY.
pub fn process_block_filtfilt(&mut self, input: &[f64]) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }

    // Pad length. Two contributions:
    //   1. 3× the cumulative filter order (the classic scipy-style default),
    //   2. enough samples for the slowest pole's impulse response to decay to
    //      ~1e-14 — required so a *constant* input (worst case for edge
    //      artifacts, since reflection keeps it constant and only the
    //      zero-state transient remains) settles before the trim boundary.
    // Capped to input length so we never mirror more samples than we have.
    let order_estimate = self.sections.len() * 2;
    let mut worst_pole_radius = 0.0_f64;
    for bq in &self.sections {
        let (p1, p2) = bq.poles();
        let r1 = (p1.re * p1.re + p1.im * p1.im).sqrt();
        let r2 = (p2.re * p2.re + p2.im * p2.im).sqrt();
        worst_pole_radius = worst_pole_radius.max(r1).max(r2);
    }
    let decay_pad = if (0.0..1.0).contains(&worst_pole_radius) {
        // n where r^n ≈ 1e-14  →  n ≈ -14 / ln(r)
        (-14.0 / worst_pole_radius.ln()).ceil() as usize
    } else {
        0
    };
    let pad = (3 * order_estimate)
        .max(decay_pad)
        .min(input.len().saturating_sub(1));

    // Odd reflection: reflect about the endpoint value, not just mirror.
    // Mathematically: padded[i] = 2·x[0] − x[pad − i]  on the left,
    //                 padded[i] = 2·x[N-1] − x[N-2-(i-N-pad)]  on the right.
    // This preserves the *slope* across the boundary, not just the value —
    // a plain mirror would create a derivative discontinuity that the
    // filter would ring on. Odd reflection is what scipy's filtfilt does.
    let n = input.len();
    let mut padded = Vec::with_capacity(n + 2 * pad);

    let x0 = input[0];
    for i in 0..pad {
        padded.push(2.0 * x0 - input[pad - i]);
    }
    padded.extend_from_slice(input);
    let xn = input[n - 1];
    for i in 0..pad {
        padded.push(2.0 * xn - input[n - 2 - i]);
    }

    // Forward pass on padded signal.
    self.reset();
    let mut forward = vec![0.0; padded.len()];
    self.process_block(&padded, &mut forward);

    // Reverse, second forward pass, reverse back.
    forward.reverse();
    self.reset();
    let mut backward = vec![0.0; forward.len()];
    self.process_block(&forward, &mut backward);
    backward.reverse();

    // Trim the padding off both ends.
    backward[pad..pad + n].to_vec()
}
}
mod tests {
    use super::*;
    use crate::dsp::design;

    #[test]
    fn biquad_process_block_matches_process() {
        let coefs = Biquad::new(0.5, 0.25, 0.125, -0.3, 0.4);
        let input = [1.0, 0.5, -0.5, 0.25];

        let mut bq_block = coefs;
        let mut output_block = [0.0; 4];
        bq_block.process_block(&input, &mut output_block);

        let mut bq_seq = coefs;
        let mut output_seq = [0.0; 4];
        for (i, &x) in input.iter().enumerate() {
            output_seq[i] = bq_seq.process(x);
        }

        for i in 0..4 {
            assert!((output_block[i] - output_seq[i]).abs() < 1e-15,
                    "idx {}: block={}, seq={}", i, output_block[i], output_seq[i]);
        }
        assert!((bq_block.z1 - bq_seq.z1).abs() < 1e-15);
        assert!((bq_block.z2 - bq_seq.z2).abs() < 1e-15);
    }

    #[test]
fn filtfilt_has_zero_net_phase() {
    use crate::dsp::design;
    use std::f64::consts::PI;

    // A pure tone well below cutoff should come out aligned in time
    // (zero phase) and only mildly attenuated.
    let fs = 1000.0;
    let f0 = 20.0;
    let n  = 2048;
    let input: Vec<f64> = (0..n)
        .map(|i| (2.0 * PI * f0 * i as f64 / fs).sin())
        .collect();

    let mut cascade = design::butterworth_lowpass(fs, 100.0, 4);
    let output = cascade.process_block_filtfilt(&input);

    // Skip the edge transient region and compare alignment in the middle.
    let mid_start = n / 4;
    let mid_end   = 3 * n / 4;
    let mut max_phase_err = 0.0_f64;
    for i in mid_start..mid_end {
        // Difference between aligned samples should be small.
        max_phase_err = max_phase_err.max((output[i] - input[i]).abs());
    }
    assert!(max_phase_err < 0.05, "filtfilt phase error = {}", max_phase_err);
}

#[test]
fn filtfilt_edges_dont_ring() {
    use crate::dsp::design;

    // A constant signal (the worst case for edge artifacts) should pass
    // through filtfilt nearly unchanged everywhere, including the edges.
    //
    // Tolerance: with odd reflection a constant stays constant, so the only
    // residual is the zero-state transient, which decays as r^pad. For this
    // filter (4th-order Butterworth LP at 50 Hz / 1 kHz) the slowest pole is
    // |z| ≈ 0.94, and padding is capped at n−1 = 199 samples, giving a
    // theoretical floor of ~1e-6. The previous fixed 3·order pad (12
    // samples) produced ~0.25 of ringing here; we assert < 1e-4, two orders
    // of magnitude above the floor and thousands of times better than the
    // old behavior.
    let input = vec![5.0; 200];
    let mut cascade = design::butterworth_lowpass(1000.0, 50.0, 4);
    let output = cascade.process_block_filtfilt(&input);

    for (i, &y) in output.iter().enumerate() {
        assert!(
            (y - 5.0).abs() < 1e-4,
            "edge ringing at sample {}: y = {}",
            i, y,
        );
    }
}
}