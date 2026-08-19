//! Protocol-neutral in-memory model for recorded / live signal data.
//!
//! This is the single internal representation the whole app works on. Both the
//! offline rosbag reader ([`crate::io::rosbag_loader`]) and the live ROS 2
//! topic client ([`crate::io::ros2_live`]) produce it. Nothing in here knows
//! about MCAP, CDR, rclrs, or any transport — it is deliberately boring so the
//! DSP + pipeline + GUI layers stay protocol-independent.

use std::collections::HashMap;

/// A single sample value, typed the way the source recorded it.
///
/// Numeric kinds are coercible to `f64` (see
/// [`crate::io::convert::sample_to_f64`]); strings are retained for
/// completeness but are never plotted.
///
/// The current ROS 2 sources (rosbag CDR decoder, live topic extractor)
/// produce `Double`; the other variants remain part of the model's
/// contract so protocol-neutral code and tests cover every recorded kind.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum SampleValue {
    Double(f64),
    Float(f32),
    Int64(i64),
    Boolean(bool),
    StringVal(String),
}

/// One timestamped sample of a channel.
#[derive(Debug, Clone)]
pub struct Sample {
    /// Microseconds since the UNIX epoch (matches the rosbag log_time axis).
    pub timestamp_us: u64,
    pub value: SampleValue,
}

/// A named, plottable signal.
///
/// For scalar ROS 2 messages (e.g. `std_msgs/msg/Float64`) one channel is
/// created per topic. For composite messages (e.g. `sensor_msgs/msg/Imu`) one
/// channel is created per selected field; `name` then carries a
/// "`topic · field`" suffix and `metadata` records the ROS message type and
/// field path.
#[derive(Debug, Clone)]
pub struct Channel {
    /// Stable id used as the key into [`LogFile::data`]. Assigned by the
    /// loader / live client; stable for the lifetime of the `LogFile`.
    pub entry_id: u32,
    pub name: String,
    /// Plottable primitive label, e.g. `"float64"` (see
    /// [`crate::io::convert::is_plottable`]).
    pub data_type: String,
    /// Human-readable origin, e.g. `"sensor_msgs/msg/Imu field=linear_acceleration.x"`.
    pub metadata: String,
}

/// A complete dataset: channel list plus the raw samples for each channel.
///
/// Successor of the old WPILOG `LogFile`; the shape is kept identical so the
/// analysis/pipeline/UI layers needed no protocol knowledge.
#[derive(Debug)]
pub struct LogFile {
    pub channels: Vec<Channel>,
    pub data: HashMap<u32, Vec<Sample>>,
}

impl LogFile {
    /// Total number of samples across all channels.
    pub fn sample_count(&self) -> usize {
        self.data.values().map(|v| v.len()).sum()
    }
}
