//! Helpers shared by the offline (rosbag) and live (ROS 2 topic) data paths.

use crate::io::model::SampleValue;

/// Coerce a sample to `f64` for plotting / DSP.
///
/// Strings are not plottable and yield `None`. The coerced value is what gets
/// fed to the resampler, the FFT, and the filter cascade.
pub fn sample_to_f64(v: &SampleValue) -> Option<f64> {
    match v {
        SampleValue::Double(x)    => Some(*x),
        SampleValue::Float(x)     => Some(*x as f64),
        SampleValue::Int64(x)     => Some(*x as f64),
        SampleValue::Boolean(b)   => Some(if *b { 1.0 } else { 0.0 }),
        SampleValue::StringVal(_) => None,
    }
}

/// Whether a channel of the given `data_type` label can be plotted / filtered.
///
/// Accepts both the historical WPILOG-style labels (`"double"`, `"float"`,
/// `"int64"`, `"boolean"`) and the ROS 2 primitive names (`"float64"`,
/// `"float32"`, `"int64"`, `"bool"`, …).
pub fn is_plottable(dtype: &str) -> bool {
    matches!(
        dtype,
        // WPILOG-era labels (kept for any existing saved graphs/configs)
        "double" | "float" | "boolean"
        // ROS 2 primitive names
        | "float64" | "float32" | "int64" | "uint64" | "int32" | "uint32"
        | "int16" | "uint16" | "int8" | "uint8" | "bool" | "byte" | "char"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coercions_cover_all_numeric_kinds() {
        assert_eq!(sample_to_f64(&SampleValue::Double(1.5)), Some(1.5));
        assert_eq!(sample_to_f64(&SampleValue::Float(1.5)), Some(1.5));
        assert_eq!(sample_to_f64(&SampleValue::Int64(-3)), Some(-3.0));
        assert_eq!(sample_to_f64(&SampleValue::Boolean(true)), Some(1.0));
        assert_eq!(sample_to_f64(&SampleValue::Boolean(false)), Some(0.0));
        assert_eq!(sample_to_f64(&SampleValue::StringVal("x".into())), None);
    }

    #[test]
    fn plottable_labels() {
        assert!(is_plottable("double"));
        assert!(is_plottable("float64"));
        assert!(is_plottable("bool"));
        assert!(is_plottable("int64"));
        assert!(!is_plottable("string"));
        assert!(!is_plottable("sensor_msgs/msg/Imu"));
    }
}
