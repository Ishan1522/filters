//! Live ROS 2 topic subscription via [`rclrs`] — the official ros2-rust
//! client library — using **dynamic messages** (runtime type introspection).
//!
//! Dynamic subscriptions mean no generated message crates are needed: the
//! live client can subscribe to *any* message type the DDS graph publishes,
//! and this module extracts the scalar channel the user asked for via a
//! dotted field path (e.g. `""`, `"linear_acceleration.z"`, `"data[3]"`).
//!
//! This module is compiled only with the `ros2` cargo feature (`cargo build
//! --features ros2`), which requires a ROS 2 distro installed and sourced.
//! The default build ships without it and the live panel explains how to
//! enable it.
//!
//! Architecture mirrors the app's original live panel one-for-one:
//! - a background thread owns the rclrs executor and spins it in short
//!   time-sliced bursts so the UI can stop it responsively,
//! - subscriptions write into a shared [`LiveStore`] ring buffer,
//! - the UI reads a one-frame [`LogFile`] snapshot each frame and feeds it to
//!   the exact same signal/graph views used for offline rosbag analysis.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crossbeam_channel::{unbounded, Receiver, Sender};

use crate::io::live_store::LiveStore;

#[cfg(feature = "ros2")]
use rclrs::{
    Context, CreateBasicExecutor, DynamicMessage, InitOptions, IntoPrimitiveOptions, MessageInfo,
    MessageTypeName, QOS_PROFILE_SENSOR_DATA, SequenceValue, SimpleValue, SpinOptions, Value,
};

/// Status messages from the live thread to the UI.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LiveStatus {
    Initializing,
    Connected,
    Disconnected,
    Error(String),
}

/// One segment of a field path: either a named field or an array index.
#[derive(Debug, Clone, PartialEq, Eq)]
enum PathSegment {
    Field(String),
    Index(usize),
}

/// One configured live subscription.
///
/// `path` is a dotted field path into the message:
/// - `""`            → the message scalar itself (`Float64.data`, `Bool.data`, …)
/// - `"linear_acceleration.z"` → nested message field
/// - `"data[3]"`     → sequence element
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LiveTopicConfig {
    pub topic: String,
    /// Full ROS message type, e.g. `"sensor_msgs/msg/Imu"` (as reported by
    /// DDS discovery; auto-filled when picking from the discovered list).
    pub msg_type: String,
    pub path: String,
}

impl LiveTopicConfig {
    /// Channel label: `"/imu/data · linear_acceleration.z"`, or the bare
    /// topic name for scalar messages — identical to the offline rosbag
    /// naming so both paths feel the same.
    pub fn display_name(&self) -> String {
        if self.path.is_empty() {
            self.topic.clone()
        } else {
            format!("{} · {}", self.topic, self.path)
        }
    }

    fn parse_path(&self) -> Vec<PathSegment> {
        if self.path.trim().is_empty() {
            return Vec::new();
        }
        self.path
            .split('.')
            .map(|seg| {
                let seg = seg.trim();
                // Support `data[3]` → Index(3)
                if let Some(open) = seg.find('[') {
                    if let Some(close) = seg.rfind(']') {
                        if let Ok(idx) = seg[open + 1..close].trim().parse::<usize>() {
                            return PathSegment::Field(seg[..open].to_string());
                        }
                    }
                }
                PathSegment::Field(seg.to_string())
            })
            .collect()
    }
}

/// Handle to a running live client. The UI holds this; dropping it does NOT
/// stop the network thread — call [`Ros2LiveClient::disconnect`] explicitly.
pub struct Ros2LiveClient {
    pub status_rx: Receiver<LiveStatus>,
    pub store: Arc<LiveStore>,
    stop: Arc<AtomicBool>,
}

impl Ros2LiveClient {
    /// Start a live session subscribing to `configs` on the given ROS domain
    /// id (`None` = default domain, typically 0).
    pub fn connect(domain_id: Option<usize>, configs: Vec<LiveTopicConfig>) -> Self {
        let (status_tx, status_rx) = unbounded();
        let store = LiveStore::new();
        let store_clone = Arc::clone(&store);
        let stop = Arc::new(AtomicBool::new(false));
        let stop_clone = Arc::clone(&stop);

        thread::spawn(move || {
            run_network_thread(domain_id, configs, store_clone, status_tx, stop_clone);
        });

        Self {
            status_rx,
            store,
            stop,
        }
    }

    pub fn disconnect(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// One-shot topic discovery: list (topic, message type) pairs visible on
    /// the domain, for the topic-picker UI. Best-effort — DDS discovery needs
    /// a moment to hear about publishers, so retry if a topic is missing.
    pub fn discover_topics(domain_id: Option<usize>) -> Result<Vec<(String, String)>, String> {
        #[cfg(feature = "ros2")]
        {
            let context = Context::from_env(InitOptions::new().with_domain_id(domain_id))
                .map_err(|e| e.to_string())?;
            let executor = context.create_basic_executor();
            let node = executor
                .create_node("rosfilter_discovery")
                .map_err(|e| e.to_string())?;
            // Give DDS a moment to gather endpoint info before querying.
            thread::sleep(Duration::from_millis(200));
            let topics = node.get_topic_names_and_types().map_err(|e| e.to_string())?;
            let mut out: Vec<(String, String)> = topics
                .into_iter()
                .filter_map(|(topic, types)| {
                    types.first().map(|t| (topic, t.clone()))
                })
                .collect();
            out.sort();
            Ok(out)
        }
        #[cfg(not(feature = "ros2"))]
        {
            let _ = domain_id;
            Err("live ROS 2 support is not enabled (build with --features ros2)".to_string())
        }
    }
}

/// Timestamp in microseconds for the store: prefer the publisher's source
/// timestamp, fall back to receipt time / wall clock.
#[cfg(feature = "ros2")]
fn timestamp_us(info: &MessageInfo) -> u64 {
    let t = info
        .source_timestamp
        .or(info.received_timestamp)
        .unwrap_or_else(SystemTime::now);
    t.duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// The live thread. Owns the rclrs context/executor; spins in short bursts so
/// the stop flag is honored promptly.
#[cfg(feature = "ros2")]
fn run_network_thread(
    domain_id: Option<usize>,
    configs: Vec<LiveTopicConfig>,
    store: Arc<LiveStore>,
    status_tx: Sender<LiveStatus>,
    stop: Arc<AtomicBool>,
) {
    let _ = status_tx.send(LiveStatus::Initializing);

    let context = match Context::from_env(InitOptions::new().with_domain_id(domain_id)) {
        Ok(c) => c,
        Err(e) => {
            let _ = status_tx.send(LiveStatus::Error(format!("rclrs init: {e}")));
            return;
        }
    };
    let mut executor = context.create_basic_executor();
    let node = match executor.create_node("rosfilter_live") {
        Ok(n) => n,
        Err(e) => {
            let _ = status_tx.send(LiveStatus::Error(format!("create node: {e}")));
            return;
        }
    };

    let mut n_ok = 0usize;
    for cfg in &configs {
        match subscribe_one(&node, cfg, &store) {
            Ok(()) => n_ok += 1,
            Err(e) => {
                let _ = status_tx.send(LiveStatus::Error(format!(
                    "subscribe {} ({}): {e}",
                    cfg.topic, cfg.msg_type
                )));
            }
        }
    }
    if n_ok == 0 && !configs.is_empty() {
        return; // nothing could be subscribed; status already carries the error
    }

    let _ = status_tx.send(LiveStatus::Connected);
    while !stop.load(Ordering::Relaxed) {
        let errors = executor.spin(SpinOptions::default().timeout(Duration::from_millis(200)));
        if !errors.is_empty() {
            let _ = status_tx.send(LiveStatus::Error(format!(
                "executor: {}",
                errors
                    .iter()
                    .map(|e| e.to_string())
                    .collect::<Vec<_>>()
                    .join("; ")
            )));
            break;
        }
    }
    let _ = status_tx.send(LiveStatus::Disconnected);
}

/// Create one dynamic subscription for a config, wiring it into the store.
#[cfg(feature = "ros2")]
fn subscribe_one(
    node: &rclrs::Node,
    cfg: &LiveTopicConfig,
    store: &Arc<LiveStore>,
) -> Result<(), String> {
    let entry_id = store.announce(cfg.display_name(), cfg.msg_type.clone());
    let store = Arc::clone(store);
    let path = cfg.parse_path();

    let topic_type = MessageTypeName::try_from(cfg.msg_type.as_str())
        .map_err(|_| format!("invalid message type '{}'", cfg.msg_type))?;

    node.create_dynamic_subscription(
        topic_type,
        cfg.topic.as_str().qos(QOS_PROFILE_SENSOR_DATA),
        move |msg: DynamicMessage, info: MessageInfo| {
            if let Some(v) = extract_path(&msg, &path) {
                store.push(entry_id, timestamp_us(&info), v);
            }
        },
    )
    .map(|_| ())
    .map_err(|e| e.to_string())
}

/// Walk `path` into `msg` and coerce the terminal value to `f64`.
#[cfg(feature = "ros2")]
fn extract_path(msg: &DynamicMessage, path: &[PathSegment]) -> Option<f64> {
    if path.is_empty() {
        // Scalar message (e.g. Float64 / Bool): the value is the whole message.
        let (_, v) = msg.view().iter().next()?;
        return scalar_to_f64(&v);
    }

    let mut current: Option<Value<'_>> = None;
    let mut terminal: Option<f64> = None;

    for (i, seg) in path.iter().enumerate() {
        match seg {
            PathSegment::Field(name) => {
                let v = if i == 0 {
                    msg.get(name)
                } else {
                    let Value::Simple(SimpleValue::Message(view)) = current.as_ref()? else {
                        return None;
                    };
                    view.get(name)
                }?;
                current = Some(v);
            }
            PathSegment::Index(idx) => {
                let Value::Sequence(seq) = current.as_ref()? else {
                    return None;
                };
                terminal = sequence_element_f64(seq, *idx);
                break;
            }
        }
    }

    if let Some(t) = terminal {
        return Some(t);
    }
    scalar_to_f64(current.as_ref()?)
}

#[cfg(feature = "ros2")]
fn scalar_to_f64(v: &Value<'_>) -> Option<f64> {
    let Value::Simple(s) = v else {
        return None;
    };
    // NOTE: the immutable SimpleValue variants hold *references* to the
    // underlying scalars (`Double(&'msg f64)` etc.) — hence the double
    // deref below.
    match s {
        SimpleValue::Double(d) => Some(**d),
        SimpleValue::Float(f) => Some(**f as f64),
        SimpleValue::Int8(i) => Some(**i as f64),
        SimpleValue::Int16(i) => Some(**i as f64),
        SimpleValue::Int32(i) => Some(**i as f64),
        SimpleValue::Int64(i) => Some(**i as f64),
        SimpleValue::Uint8(u) => Some(**u as f64),
        SimpleValue::Uint16(u) => Some(**u as f64),
        SimpleValue::Uint32(u) => Some(**u as f64),
        SimpleValue::Uint64(u) => Some(**u as f64),
        SimpleValue::Boolean(b) => Some(if **b { 1.0 } else { 0.0 }),
        // LongDouble is a raw pointer with platform-dependent width
        // (80-bit x87 on x86) — not representable as f64, skip.
        _ => None,
    }
}

#[cfg(feature = "ros2")]
fn sequence_element_f64(seq: &SequenceValue<'_>, idx: usize) -> Option<f64> {
    match seq {
        SequenceValue::DoubleSequence(s) => s.get(idx).copied(),
        SequenceValue::FloatSequence(s) => s.get(idx).map(|&f| f as f64),
        SequenceValue::Int64Sequence(s) => s.get(idx).map(|&i| i as f64),
        SequenceValue::Int32Sequence(s) => s.get(idx).map(|&i| i as f64),
        SequenceValue::Uint64Sequence(s) => s.get(idx).map(|&u| u as f64),
        SequenceValue::Uint32Sequence(s) => s.get(idx).map(|&u| u as f64),
        SequenceValue::BooleanSequence(s) => s.get(idx).map(|&b| if b { 1.0 } else { 0.0 }),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_parsing() {
        let cfg = LiveTopicConfig {
            topic: "/imu".into(),
            msg_type: "sensor_msgs/msg/Imu".into(),
            path: "linear_acceleration.z".into(),
        };
        assert_eq!(
            cfg.parse_path(),
            vec![
                PathSegment::Field("linear_acceleration".into()),
                PathSegment::Field("z".into())
            ]
        );
        assert_eq!(cfg.display_name(), "/imu · linear_acceleration.z");

        let arr = LiveTopicConfig {
            topic: "/joints".into(),
            msg_type: "std_msgs/msg/Float64MultiArray".into(),
            path: "data[3]".into(),
        };
        assert_eq!(
            arr.parse_path(),
            vec![PathSegment::Field("data".into()), PathSegment::Index(3)]
        );
        assert_eq!(arr.display_name(), "/joints · data[3]");

        let scalar = LiveTopicConfig {
            topic: "/vel".into(),
            msg_type: "std_msgs/msg/Float64".into(),
            path: "".into(),
        };
        assert!(scalar.parse_path().is_empty());
        assert_eq!(scalar.display_name(), "/vel");
    }

    /// CI smoke test: subscribe to a live topic via rclrs and assert data
    /// actually flows into the ring buffer.
    ///
    /// Requires a sourced ROS 2 Jazzy env and a publisher on `/ci/vel`
    /// (the workflow starts `ci/publish_fixture.py` in the background):
    ///
    ///   ROS_LOCALHOST_ONLY=1 cargo test --features ros2 live_smoke -- --ignored
    #[cfg(feature = "ros2")]
    #[test]
    #[ignore = "requires a live ROS 2 node + publisher; run by CI"]
    fn live_smoke_receives_data() {
        use std::time::{Duration, Instant};

        let client = Ros2LiveClient::connect(
            Some(0),
            vec![LiveTopicConfig {
                topic: "/ci/vel".into(),
                msg_type: "std_msgs/msg/Float64".into(),
                path: String::new(),
            }],
        );

        // Drain status until connected (or a clear error).
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Ok(s) = client.status_rx.try_recv() {
                if matches!(s, LiveStatus::Error(e)) {
                    panic!("live client error: {e}");
                }
            }
            if client.store.stats().1 > 0 {
                break;
            }
            if Instant::now() > deadline {
                panic!("no samples received within 10s — is the publisher up?");
            }
            std::thread::sleep(Duration::from_millis(200));
        }

        let (n_topics, n_samples) = client.store.stats();
        eprintln!("live smoke: {n_topics} topics, {n_samples} samples received");
        assert!(n_samples > 0);
        client.disconnect();
        // Give the thread a moment to exit before the test process ends.
        std::thread::sleep(Duration::from_millis(500));
    }
}
