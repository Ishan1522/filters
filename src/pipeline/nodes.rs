use crate::dsp::biquad::BiquadCascade;
use crate::dsp::spec::FilterSpec;
use crate::pipeline::node::Signal;

pub fn apply_filter(input: &Signal, spec: &FilterSpec, zero_phase: bool) -> Signal {
    if input.is_empty() {
        return Signal::empty();
    }
    let Some(mut cascade): Option<BiquadCascade> = spec.compile(input.sample_rate_hz) else {
        // Invalid spec for this fs — pass-through.
        return input.clone();
    };

    let output_samples = if zero_phase {
        cascade.process_block_filtfilt(&input.samples)
    } else {
        let mut out = vec![0.0; input.samples.len()];
        cascade.process_block(&input.samples, &mut out);
        out
    };

    Signal::new(output_samples, input.sample_rate_hz, input.t0_seconds)
}

pub fn sum_signals(inputs: &[&Signal]) -> Signal {
    // Empty / single-input cases.
    if inputs.is_empty() {
        return Signal::empty();
    }
    if inputs.len() == 1 {
        return inputs[0].clone();
    }

    // Use the first input as the reference for length, fs, t0. Inputs with
    // mismatched lengths get truncated to the shortest, mismatched fs is
    // a silent bug we should surface in the UI later (TODO: validation pass).
    let n = inputs.iter().map(|s| s.len()).min().unwrap_or(0);
    let fs = inputs[0].sample_rate_hz;
    let t0 = inputs[0].t0_seconds;

    let mut out = vec![0.0; n];
    for sig in inputs {
        for i in 0..n {
            out[i] += sig.samples[i];
        }
    }
    Signal::new(out, fs, t0)
}

pub fn differentiate(input: &Signal) -> Signal {
    if input.len() < 2 {
        return Signal::empty();
    }
    let fs = input.sample_rate_hz;
    let n = input.len();
    let mut out = vec![0.0; n];
    // Forward difference, with the first sample held to avoid a zero artifact.
    for i in 1..n {
        out[i] = (input.samples[i] - input.samples[i - 1]) * fs;
    }
    out[0] = out[1];
    Signal::new(out, fs, input.t0_seconds)
}

pub fn gain(input: &Signal, factor: f64) -> Signal {
    let scaled: Vec<f64> = input.samples.iter().map(|&v| v * factor).collect();
    Signal::new(scaled, input.sample_rate_hz, input.t0_seconds)
}