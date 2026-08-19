//! Offline rosbag2 reader (MCAP storage format).
//!
//! rosbag2 bags recorded with `ros2 bag record` use the **MCAP** storage
//! format by default since ROS 2 Jazzy (and it is available on Humble via the
//! `mcap_storage_plugin`). This module reads those files with the pure-Rust
//! [`mcap`] crate and decodes the CDR message payloads into the
//! protocol-neutral [`LogFile`] model that the rest of the app consumes.
//!
//! Supported message types (others are skipped and counted):
//! - `std_msgs/msg/Float64`, `Float32`, `Int64`, `Int32`, `Bool`
//! - `std_msgs/msg/Float64MultiArray` (one channel per element, capped)
//! - `sensor_msgs/msg/Imu`
//! - `sensor_msgs/msg/JointState` (position per joint)
//! - `nav_msgs/msg/Odometry`
//! - `geometry_msgs/msg/Twist`, `TwistStamped`, `Vector3`, `PointStamped`
//!
//! Timestamps are taken from the MCAP `log_time` (nanoseconds), converted to
//! microseconds to match the rest of the toolchain.

use std::collections::HashMap;
use std::path::Path;

use crate::io::cdr::CdrReader;
use crate::io::model::{Channel, LogFile, Sample, SampleValue};

/// Number of elements of a `Float64MultiArray` to expose as individual
/// channels at most. Protects against pathological array sizes.
const MAX_MULTIARRAY_CHANNELS: usize = 64;

#[derive(Debug)]
pub enum RosbagError {
    Io(std::io::Error),
    Mcap(String),
    /// A message used an encoding we don't understand (expected `"cdr"`).
    UnsupportedEncoding { topic: String, encoding: String },
}

impl std::fmt::Display for RosbagError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RosbagError::Io(e) => write!(f, "io error: {e}"),
            RosbagError::Mcap(e) => write!(f, "mcap error: {e}"),
            RosbagError::UnsupportedEncoding { topic, encoding } => {
                write!(f, "topic '{topic}' uses message encoding '{encoding}', expected 'cdr'")
            }
        }
    }
}

impl std::error::Error for RosbagError {}

/// Aggregate counters about what was loaded, for the status line in the UI /
/// CLI.
#[derive(Debug, Default, Clone)]
pub struct LoadStats {
    pub messages_total: u64,
    pub messages_decoded: u64,
    /// Messages skipped because their message type isn't in our decoder.
    pub messages_unsupported: u64,
    /// (topic, message type) pairs that were skipped, for the console log.
    pub unsupported_topics: Vec<(String, String)>,
}

/// Load a rosbag2 (MCAP) file into the shared [`LogFile`] model.
///
/// Channels are created in first-seen order. Composite messages expand into
/// one channel per extracted field (e.g. `/imu/data · linear_acceleration.z`).
pub fn load(path: &Path) -> Result<(LogFile, LoadStats), RosbagError> {
    let buf = std::fs::read(path).map_err(RosbagError::Io)?;
    let mut stats = LoadStats::default();
    let mut seen_unsupported: Vec<(String, String)> = Vec::new();

    let mut log = LogFile {
        channels: Vec::new(),
        data: HashMap::new(),
    };
    // (topic, field label) → entry_id
    let mut entry_ids: HashMap<(String, String), u32> = HashMap::new();

    let stream = mcap::read::MessageStream::new(&buf).map_err(|e| RosbagError::Mcap(e.to_string()))?;
    for msg in stream {
        let msg = msg.map_err(|e| RosbagError::Mcap(e.to_string()))?;
        stats.messages_total += 1;

        let topic = msg.channel.topic.clone();
        if msg.channel.message_encoding != "cdr" {
            return Err(RosbagError::UnsupportedEncoding {
                topic,
                encoding: msg.channel.message_encoding.clone(),
            });
        }

        let type_name = normalize_type(msg.channel.schema.as_ref().map(|s| s.name.as_str()));
        let decoded = decode_message(&type_name, &msg.data);
        let Some(fields) = decoded else {
            // Unknown / unsupported type — skip but remember it.
            stats.messages_unsupported += 1;
            let key = (topic, type_name.clone());
            if !seen_unsupported.contains(&key) {
                seen_unsupported.push(key);
            }
            continue;
        };

        stats.messages_decoded += 1;
        let timestamp_us = msg.log_time / 1000;
        for (label, value) in fields {
            let id = *entry_ids.entry((topic.clone(), label.clone())).or_insert_with(|| {
                let id = log.channels.len() as u32 + 1;
                let metadata = if label.is_empty() {
                    format!("{type_name}")
                } else {
                    format!("{type_name} field={label}")
                };
                log.channels.push(Channel {
                    entry_id: id,
                    name: if label.is_empty() {
                        topic.clone()
                    } else {
                        format!("{topic} · {label}")
                    },
                    data_type: "float64".to_string(),
                    metadata,
                });
                id
            });
            log.data.entry(id).or_default().push(Sample {
                timestamp_us,
                value: SampleValue::Double(value),
            });
        }
    }

    // Sort each channel by timestamp — MCAP writes are usually time-ordered,
    // but the estimator/resampler assume monotonic per-channel time.
    for samples in log.data.values_mut() {
        samples.sort_by_key(|s| s.timestamp_us);
    }

    stats.unsupported_topics = seen_unsupported;
    Ok((log, stats))
}

/// Map the schema name to a canonical message type.
///
/// rosbag2 writers store the schema name as `"ros2msg:<type>"` (some writers
/// omit the prefix, and some omit the `/msg/` path segment) — normalize all
/// forms to `pkg/msg/Type`.
fn normalize_type(schema_name: Option<&str>) -> String {
    let name = schema_name.unwrap_or("");
    let name = name.strip_prefix("ros2msg:").unwrap_or(name);
    if name.matches('/').count() == 1 {
        // "std_msgs/Float64" → "std_msgs/msg/Float64"
        if let Some((pkg, ty)) = name.split_once('/') {
            return format!("{pkg}/msg/{ty}");
        }
    }
    name.to_string()
}

/// Decode one CDR payload into (field label, value) pairs.
///
/// A `""` label means "use the topic name as the channel name" (scalar
/// messages); anything else becomes a `topic · label` channel.
fn decode_message(type_name: &str, data: &[u8]) -> Option<Vec<(String, f64)>> {
    let mut r = CdrReader::new(data);
    let fields: Vec<(String, f64)> = match type_name {
        "std_msgs/msg/Float64" => vec![("".to_string(), r.read_f64()?)],
        "std_msgs/msg/Float32" => vec![("".to_string(), r.read_f32()? as f64)],
        "std_msgs/msg/Int64" => vec![("".to_string(), r.read_i64()? as f64)],
        "std_msgs/msg/Int32" => vec![("".to_string(), r.read_i32()? as f64)],
        "std_msgs/msg/Bool" => vec![("".to_string(), r.read_bool()? as u8 as f64)],

        "std_msgs/msg/Float64MultiArray" => decode_multi_array(&mut r)?,

        "sensor_msgs/msg/Imu" => decode_imu(&mut r)?,
        "sensor_msgs/msg/JointState" => decode_joint_state(&mut r)?,

        "nav_msgs/msg/Odometry" => decode_odometry(&mut r)?,

        "geometry_msgs/msg/Twist" => decode_twist(&mut r)?,
        "geometry_msgs/msg/TwistStamped" => {
            skip_header(&mut r)?;
            decode_twist(&mut r)?
        }
        "geometry_msgs/msg/Vector3" => decode_vector3(&mut r)?,
        "geometry_msgs/msg/PointStamped" => {
            skip_header(&mut r)?;
            vec![
                ("point.x".to_string(), r.read_f64()?),
                ("point.y".to_string(), r.read_f64()?),
                ("point.z".to_string(), r.read_f64()?),
            ]
        }

        _ => return None,
    };
    Some(fields)
}

/// Skip a `std_msgs/msg/Header`: `Time stamp {int32 sec; uint32 nanosec}` +
/// `string frame_id`.
fn skip_header(r: &mut CdrReader) -> Option<()> {
    r.read_i32()?; // stamp.sec
    r.read_u32()?; // stamp.nanosec
    let _ = r.read_string()?; // frame_id
    Some(())
}

fn decode_vector3(r: &mut CdrReader) -> Option<Vec<(String, f64)>> {
    Some(vec![
        ("x".to_string(), r.read_f64()?),
        ("y".to_string(), r.read_f64()?),
        ("z".to_string(), r.read_f64()?),
    ])
}

fn decode_twist(r: &mut CdrReader) -> Option<Vec<(String, f64)>> {
    let (lx, ly, lz) = (r.read_f64()?, r.read_f64()?, r.read_f64()?);
    let (ax, ay, az) = (r.read_f64()?, r.read_f64()?, r.read_f64()?);
    Some(vec![
        ("linear.x".to_string(), lx),
        ("linear.y".to_string(), ly),
        ("linear.z".to_string(), lz),
        ("angular.x".to_string(), ax),
        ("angular.y".to_string(), ay),
        ("angular.z".to_string(), az),
    ])
}

fn decode_imu(r: &mut CdrReader) -> Option<Vec<(String, f64)>> {
    skip_header(r)?;
    // orientation (Quaternion: x,y,z,w)
    let q = [
        r.read_f64()?,
        r.read_f64()?,
        r.read_f64()?,
        r.read_f64()?,
    ];
    r.skip(9 * 8)?; // orientation_covariance[9]
    let av = [r.read_f64()?, r.read_f64()?, r.read_f64()?];
    r.skip(9 * 8)?; // angular_velocity_covariance[9]
    let la = [r.read_f64()?, r.read_f64()?, r.read_f64()?];
    // linear_acceleration_covariance[9] — not needed

    Some(vec![
        ("orientation.x".to_string(), q[0]),
        ("orientation.y".to_string(), q[1]),
        ("orientation.z".to_string(), q[2]),
        ("orientation.w".to_string(), q[3]),
        ("angular_velocity.x".to_string(), av[0]),
        ("angular_velocity.y".to_string(), av[1]),
        ("angular_velocity.z".to_string(), av[2]),
        ("linear_acceleration.x".to_string(), la[0]),
        ("linear_acceleration.y".to_string(), la[1]),
        ("linear_acceleration.z".to_string(), la[2]),
    ])
}

fn decode_odometry(r: &mut CdrReader) -> Option<Vec<(String, f64)>> {
    skip_header(r)?;
    let _child_frame = r.read_string()?;
    // pose.pose: Position (x,y,z) + Quaternion (x,y,z,w) + covariance[36]
    let p = [r.read_f64()?, r.read_f64()?, r.read_f64()?];
    let o = [
        r.read_f64()?,
        r.read_f64()?,
        r.read_f64()?,
        r.read_f64()?,
    ];
    r.skip(36 * 8)?;
    // twist.twist: linear (x,y,z) + angular (x,y,z) + covariance[36]
    let l = [r.read_f64()?, r.read_f64()?, r.read_f64()?];
    let a = [r.read_f64()?, r.read_f64()?, r.read_f64()?];

    Some(vec![
        ("pose.position.x".to_string(), p[0]),
        ("pose.position.y".to_string(), p[1]),
        ("pose.position.z".to_string(), p[2]),
        ("pose.orientation.x".to_string(), o[0]),
        ("pose.orientation.y".to_string(), o[1]),
        ("pose.orientation.z".to_string(), o[2]),
        ("pose.orientation.w".to_string(), o[3]),
        ("twist.linear.x".to_string(), l[0]),
        ("twist.linear.y".to_string(), l[1]),
        ("twist.linear.z".to_string(), l[2]),
        ("twist.angular.x".to_string(), a[0]),
        ("twist.angular.y".to_string(), a[1]),
        ("twist.angular.z".to_string(), a[2]),
    ])
}

fn decode_joint_state(r: &mut CdrReader) -> Option<Vec<(String, f64)>> {
    skip_header(r)?;
    let mut names = Vec::new();
    r.read_string_seq(&mut names)?;
    let mut positions = Vec::new();
    r.read_f64_seq(&mut positions)?;
    // velocity / effort follow — not extracted.
    Some(
        names
            .iter()
            .zip(positions.iter())
            .map(|(n, &p)| (format!("position[{n}]"), p))
            .collect(),
    )
}

fn decode_multi_array(r: &mut CdrReader) -> Option<Vec<(String, f64)>> {
    // MultiArrayLayout: uint32 dim_count, then per dim {string label; uint32
    // size; uint32 stride}, then uint32 data_offset.
    let dims = r.read_u32()?;
    for _ in 0..dims {
        let _ = r.read_string()?;
        r.read_u32()?; // size
        r.read_u32()?; // stride
    }
    r.read_u32()?; // data_offset
    let mut data = Vec::new();
    r.read_f64_seq(&mut data)?;

    let out: Vec<(String, f64)> = data
        .iter()
        .enumerate()
        .take(MAX_MULTIARRAY_CHANNELS)
        .map(|(i, &v)| (format!("data[{i}]"), v))
        .collect();
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::cdr::test_helpers::CdrWriter;
    use std::collections::BTreeMap;
    use std::io::Cursor;
    use std::sync::Arc;

    /// Unique temp-file suffix per invocation (cargo test runs tests in
    /// parallel threads — a shared filename would race).
    static NEXT_TMP_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

    /// Write a minimal in-memory MCAP file containing the given (topic, type,
    /// payload) messages, then load it back through [`load`]. Round-trips the
    /// full mcap-read + CDR-decode path.
    fn write_roundtrip_mcap(messages: &[(&str, &str, Vec<u8>)], log_times_ns: &[u64]) -> LogFile {
        let mut buf = Cursor::new(Vec::new());
        {
            let mut writer = mcap::Writer::new(&mut buf).expect("writer");
            let mut next_schema_id: u16 = 1;
            let mut next_channel_id: u16 = 1;
            for ((topic, type_name, payload), ts) in messages.iter().zip(log_times_ns.iter()) {
                let schema = mcap::Schema {
                    id: next_schema_id,
                    name: format!("ros2msg:{type_name}"),
                    encoding: "ros2msg".to_string(),
                    data: std::borrow::Cow::Owned(b"msg\n".to_vec()),
                };
                let channel = mcap::Channel {
                    id: next_channel_id,
                    topic: topic.to_string(),
                    schema: Some(Arc::new(schema)),
                    message_encoding: "cdr".to_string(),
                    metadata: BTreeMap::new(),
                };
                writer
                    .write(&mcap::Message {
                        channel: Arc::new(channel),
                        sequence: 0,
                        log_time: *ts,
                        publish_time: *ts,
                        data: std::borrow::Cow::Owned(payload.clone()),
                    })
                    .expect("write");
                next_schema_id += 1;
                next_channel_id += 1;
            }
            writer.finish().expect("finish");
        }
        let tmp_id = NEXT_TMP_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join("wpifilter_test_bags");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(format!("roundtrip_{}_{tmp_id}.mcap", std::process::id()));
        std::fs::write(&path, buf.into_inner()).unwrap();
        let (log, _stats) = load(&path).expect("load");
        let _ = std::fs::remove_file(&path);
        log
    }

    fn float64_payload(v: f64) -> Vec<u8> {
        v.to_le_bytes().to_vec()
    }

    #[test]
    fn scalar_topic_becomes_one_channel() {
        let mut messages = Vec::new();
        let mut ts = Vec::new();
        for i in 0..10u64 {
            messages.push(("/filtered/vel", "std_msgs/msg/Float64", float64_payload(i as f64 * 0.5)));
            ts.push((i + 1) * 1_000_000_000); // 1 Hz
        }
        let log = write_roundtrip_mcap(&messages, &ts);
        assert_eq!(log.channels.len(), 1);
        assert_eq!(log.channels[0].name, "/filtered/vel");
        assert_eq!(log.channels[0].data_type, "float64");
        let samples = &log.data[&log.channels[0].entry_id];
        assert_eq!(samples.len(), 10);
        assert_eq!(samples[0].timestamp_us, 1_000_000);
        match samples[5].value {
            SampleValue::Double(v) => assert!((v - 2.5).abs() < 1e-12),
            _ => panic!("expected double"),
        }
    }

    #[test]
    fn imu_topic_expands_to_channels_with_timestamps() {
        // Build a hand-encoded CDR Imu payload (absolute alignment), mirroring
        // what a real sensor_msgs/msg/Imu looks like on the wire.
        let mut w = CdrWriter::new();
        w.u32(1);
        w.u32(2);
        w.string("imu_link");
        for _ in 0..4 {
            w.f64(0.0);
        }
        for _ in 0..9 {
            w.f64(0.0);
        }
        w.f64(0.1);
        w.f64(0.2);
        w.f64(0.3);
        for _ in 0..9 {
            w.f64(0.0);
        }
        w.f64(9.81);
        w.f64(0.0);
        w.f64(-0.5);
        let payload = w.buf;

        let log = write_roundtrip_mcap(
            &[("/imu/data", "sensor_msgs/msg/Imu", payload)],
            &[1_234_567_890_000],
        );

        assert_eq!(log.channels.len(), 10);
        assert_eq!(log.channels[0].name, "/imu/data · orientation.x");
        assert_eq!(log.channels[7].name, "/imu/data · linear_acceleration.x");
        assert_eq!(log.channels[7].data_type, "float64");
        assert!(log.channels[7].metadata.contains("sensor_msgs/msg/Imu"));

        let samples = &log.data[&log.channels[7].entry_id];
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].timestamp_us, 1_234_567_890);
        match samples[0].value {
            SampleValue::Double(v) => assert!((v - 9.81).abs() < 1e-12),
            _ => panic!("expected double"),
        }
    }

    #[test]
    fn unsupported_type_is_skipped_not_fatal() {
        // std_msgs/msg/String is not plottable — the loader must skip it and
        // continue with the Float64 topic.
        let string_payload = {
            let mut w = CdrWriter::new();
            w.string("hello world");
            w.buf
        };
        let log = write_roundtrip_mcap(
            &[
                ("/notes", "std_msgs/msg/String", string_payload),
                ("/vel", "std_msgs/msg/Float64", float64_payload(3.0)),
            ],
            &[1_000_000_000, 2_000_000_000],
        );
        assert_eq!(log.channels.len(), 1);
        assert_eq!(log.channels[0].name, "/vel");
    }
}
