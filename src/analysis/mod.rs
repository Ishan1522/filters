pub mod fft;
pub mod pole_zero;   // existing
pub mod wavelet;     // existing

/// Estimate sample rate (Hz) from a sequence of microsecond timestamps
/// using the median of consecutive deltas. Median, not mean, because
/// rosbag recordings can have occasional jitter from logger backpressure.
pub fn estimate_sample_rate_hz(timestamps_us: &[u64]) -> Option<f64> {
    if timestamps_us.len() < 2 {
        return None;
    }
    let mut dts: Vec<u64> = timestamps_us
        .windows(2)
        .map(|w| w[1].saturating_sub(w[0]))
        .filter(|&d| d > 0)
        .collect();
    if dts.is_empty() {
        return None;
    }
    dts.sort_unstable();
    let median_us = dts[dts.len() / 2];
    Some(1_000_000.0 / median_us as f64)
}

/// Linearly resample a (timestamps_us, values) series onto a uniform grid
/// at `target_rate_hz`. Required for honest FFTs and for biquad filtering,
/// since biquads assume uniform sample spacing.
///
/// Returns (uniform_times_seconds, resampled_values).
pub fn resample_uniform(
    timestamps_us: &[u64],
    values: &[f64],
    target_rate_hz: f64,
) -> (Vec<f64>, Vec<f64>) {
    debug_assert_eq!(timestamps_us.len(), values.len());
    if timestamps_us.len() < 2 || target_rate_hz <= 0.0 {
        return (Vec::new(), Vec::new());
    }

    let t_start = timestamps_us[0] as f64 / 1.0e6;
    let t_end   = *timestamps_us.last().unwrap() as f64 / 1.0e6;
    if t_end <= t_start {
        return (Vec::new(), Vec::new());
    }

    let dt = 1.0 / target_rate_hz;
    let n = ((t_end - t_start) / dt).floor() as usize + 1;

    let mut times  = Vec::with_capacity(n);
    let mut output = Vec::with_capacity(n);

    let mut j = 0usize;
    for i in 0..n {
        let t = t_start + i as f64 * dt;
        // Advance j so that timestamps_us[j] <= t < timestamps_us[j+1].
        while j + 1 < timestamps_us.len()
            && (timestamps_us[j + 1] as f64 / 1.0e6) <= t
        {
            j += 1;
        }
        let v = if j + 1 >= timestamps_us.len() {
            values[j]
        } else {
            let t0 = timestamps_us[j] as f64 / 1.0e6;
            let t1 = timestamps_us[j + 1] as f64 / 1.0e6;
            let alpha = if t1 > t0 { ((t - t0) / (t1 - t0)).clamp(0.0, 1.0) } else { 0.0 };
            values[j] * (1.0 - alpha) + values[j + 1] * alpha
        };
        times.push(t);
        output.push(v);
    }

    (times, output)
}