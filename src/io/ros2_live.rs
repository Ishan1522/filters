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

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crossbeam_channel::{unbounded, Receiver, Sender};

use crate::io::live_store::LiveStore;

#[cfg(feature = "ros2")]
use rclrs::{
    Context, CreateBasicExecutor, DynamicMessage, ExecutorCommands, InitOptions,
    IntoPrimitiveOptions, MessageInfo, MessageTypeName, QOS_PROFILE_SENSOR_DATA, SequenceValue,
    SimpleValue, SpinOptions, Value,
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
            .flat_map(|seg| {
                let seg = seg.trim();
                // Support `data[3]` → [Field("data"), Index(3)]
                if let Some(open) = seg.find('[') {
                    if let Some(close) = seg.rfind(']') {
                        if let Ok(idx) = seg[open + 1..close].trim().parse::<usize>() {
                            return vec![
                                PathSegment::Field(seg[..open].to_string()),
                                PathSegment::Index(idx),
                            ];
                        }
                    }
                }
                vec![PathSegment::Field(seg.to_string())]
            })
            .collect()
    }
}

/// Handle to a running live client. The UI holds this; dropping it does NOT
/// stop the network thread — call [`Ros2LiveClient::disconnect`] explicitly.
pub struct Ros2LiveClient {
    pub status_rx: Receiver<LiveStatus>,
    pub store: Arc<LiveStore>,
    /// Set by the network thread once the executor exists; lets the UI halt
    /// the (otherwise infinite) spin cleanly. `None` only in the tiny window
    /// before the thread starts.
    commands: Arc<Mutex<Option<Arc<ExecutorCommands>>>>,
}

impl Ros2LiveClient {
    /// Start a live session subscribing to `configs` on the given ROS domain
    /// id (`None` = default domain, typically 0).
    pub fn connect(domain_id: Option<usize>, configs: Vec<LiveTopicConfig>) -> Self {
        let (status_tx, status_rx) = unbounded();
        let store = LiveStore::new();
        let store_clone = Arc::clone(&store);
        let commands = Arc::new(Mutex::new(None));
        let commands_clone = Arc::clone(&commands);

        thread::spawn(move || {
            run_network_thread(domain_id, configs, store_clone, status_tx, commands_clone);
        });

        Self {
            status_rx,
            store,
            commands,
        }
    }

    pub fn disconnect(&self) {
        if let Some(c) = self.commands.lock().unwrap().as_ref() {
            c.halt_spinning();
        }
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
    commands_slot: Arc<Mutex<Option<Arc<ExecutorCommands>>>>,
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

    // Hand the commands handle to the UI thread so disconnect() can halt the
    // (otherwise infinite) spin below.
    *commands_slot.lock().unwrap() = Some(executor.commands().clone());

    let _ = status_tx.send(LiveStatus::Connected);
    // Infinite spin: rclrs's designed steady-state executor loop. Stopping is
    // done by the UI via ExecutorCommands::halt_spinning.
    let errors = executor.spin(SpinOptions::default());
    if let Some(e) = errors.first() {
        let _ = status_tx.send(LiveStatus::Error(format!("executor: {e}")));
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
    // Owned copies for the closure's diagnostic messages (the closure must
    // not capture the borrowed `cfg`).
    let topic_display = cfg.topic.clone();
    let path_display = cfg.path.clone();

    let topic_type = MessageTypeName::try_from(cfg.msg_type.as_str())
        .map_err(|_| format!("invalid message type '{}'", cfg.msg_type))?;

    node.create_dynamic_subscription(
        topic_type,
        cfg.topic.as_str().qos(QOS_PROFILE_SENSOR_DATA),
        move |msg: DynamicMessage, info: MessageInfo| {
            match extract_path(&msg, &path) {
                Some(v) => store.push(entry_id, timestamp_us(&info), v),
                // Keep a low-volume diagnostic: if messages arrive but the
                // field path can't be resolved, that is a config bug worth
                // surfacing (first occurrence + then every 500th).
                None => {
                    let n = RECEIVED_NO_EXTRACT.fetch_add(1, Ordering::Relaxed);
                    if n == 0 || n % 500 == 0 {
                        eprintln!(
                            "live: received msg on {} ({} samples so far) but \
                             field path '{:?}' did not yield a scalar",
                            topic_display,
                            store.stats().1,
                            path_display,
                        );
                    }
                }
            }
        },
    )
    .map(|_| ())
    .map_err(|e| e.to_string())
}

/// Total messages received but not extractable (across all subscriptions) —
/// diagnostics only.
#[cfg(feature = "ros2")]
static RECEIVED_NO_EXTRACT: AtomicU64 = AtomicU64::new(0);

/// Walk `path` into `msg` and coerce the terminal value to `f64`.
#[cfg(feature = "ros2")]
fn extract_path(msg: &DynamicMessage, path: &[PathSegment]) -> Option<f64> {
    if path.is_empty() {
        // Scalar message (e.g. Float64 / Bool): the value is the whole message.
        let view = msg.view();
        let (_, v) = view.iter().next()?;
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
        use rclrs::{Context, InitOptions};
        use std::time::{Duration, Instant};

        // ── Diagnostic: does an rclrs node in THIS process see the
        //    publisher at all? Graph queries work without spinning, so this
        //    isolates "rclrs DDS stack broken/isolated" from
        //    "dynamic subscription delivery broken".
        let diag_ctx = Context::from_env(InitOptions::new().with_domain_id(Some(0)))
            .expect("diag context");
        let diag_executor = diag_ctx.create_basic_executor();
        let diag_node = diag_executor
            .create_node("rosfilter_diag")
            .expect("diag node");
        std::thread::sleep(Duration::from_secs(3));
        match diag_node.get_topic_names_and_types() {
            Ok(topics) => {
                eprintln!("live smoke: rclrs discovery sees {} topic(s)", topics.len());
                for (t, types) in &topics {
                    eprintln!("live smoke:   {t} ({types:?})");
                }
            }
            Err(e) => eprintln!("live smoke: discovery query failed: {e}"),
        }

        let client = Ros2LiveClient::connect(
            Some(0),
            vec![LiveTopicConfig {
                topic: "/ci/vel".into(),
                msg_type: "std_msgs/msg/Float64".into(),
                path: String::new(),
            }],
        );

        // Drain status until connected (or a clear error).
        let started = Instant::now();
        let deadline = started + Duration::from_secs(10);
        let mut last_report = started;
        loop {
            while let Ok(s) = client.status_rx.try_recv() {
                if let LiveStatus::Error(e) = s {
                    panic!("live client error: {e}");
                }
            }
            let (n_topics, n_samples) = client.store.stats();
            if n_samples > 0 {
                break;
            }
            if Instant::now() - last_report >= Duration::from_secs(2) {
                eprintln!(
                    "live smoke: t+{:.0}s — topics={n_topics} samples={n_samples} \
                     received_not_extracted={}",
                    started.elapsed().as_secs_f32(),
                    RECEIVED_NO_EXTRACT.load(Ordering::Relaxed),
                );
                last_report = Instant::now();
            }
            if Instant::now() > deadline {
                let not_extracted = RECEIVED_NO_EXTRACT.load(Ordering::Relaxed);
                panic!(
                    "no samples received within 10s \
                     (topics={n_topics}, received_but_not_extracted={not_extracted}) — \
                     is the publisher up and does the field path match?"
                );
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

    // ──────────────────────────────────────────────────────────────────────
    // Typed-vs-dynamic A/B: rclrs 0.7's dynamic subscriptions appeared to
    // receive nothing while a plain rclpy subscriber on the same topic got
    // data. This test uses a hand-rolled `std_msgs/msg/Float64` message
    // (repr(C) double + the real C type-support pointer from the installed
    // distro) with a TYPED subscription, so CI can tell whether the fault is
    // in the dynamic-message path or in the client/env generally.
    // ──────────────────────────────────────────────────────────────────────

    /// Minimal `std_msgs/msg/Float64` (idiomatic == RMW-native: one double).
    #[cfg(feature = "ros2")]
    #[repr(C)]
    #[derive(Clone, Debug, Default)]
    struct CiFloat64 {
        data: f64,
    }

    #[cfg(feature = "ros2")]
    impl rclrs::MessageIDL for CiFloat64 {
        type RmwMsg = CiFloat64;

        fn into_rmw_message(
            msg: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg
        }

        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    /// Raw type-support pointers are not Send/Sync; the generated message
    /// crates hide this behind their static tables — here we mark the
    /// (process-lifetime, immutable) pointer as shareable.
    #[cfg(feature = "ros2")]
    struct TypeSupportPtr(*const std::ffi::c_void);
    #[cfg(feature = "ros2")]
    unsafe impl Send for TypeSupportPtr {}
    #[cfg(feature = "ros2")]
    unsafe impl Sync for TypeSupportPtr {}

    #[cfg(feature = "ros2")]
    impl rclrs::RmwMessageIDL for CiFloat64 {
        const TYPE_NAME: &'static str = "std_msgs/msg/Float64";

        fn get_type_support() -> *const std::ffi::c_void {
            use std::sync::OnceLock;
            static TS: OnceLock<TypeSupportPtr> = OnceLock::new();
            TS.get_or_init(|| {
                let prefix = std::env::var("AMENT_PREFIX_PATH")
                    .map(|p| p.split(':').next().unwrap_or("/opt/ros/jazzy").to_string())
                    .unwrap_or_else(|_| "/opt/ros/jazzy".to_string());
                let path = std::path::Path::new(&prefix)
                    .join("lib")
                    .join("libstd_msgs__rosidl_typesupport_c.so");
                // SAFETY: dlopen of a trusted system library; see rclrs's own
                // get_type_support_library for the same pattern.
                let lib = unsafe { libloading::Library::new(&path) }
                    .expect("libstd_msgs__rosidl_typesupport_c.so");
                // SAFETY: symbol has the expected type by the rosidl_typesupport_c
                // ABI; pointer kept valid by leaking the library below.
                let getter: libloading::Symbol<
                    unsafe extern "C" fn() -> *const std::ffi::c_void,
                > = unsafe {
                    lib.get(
                        b"rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Float64",
                    )
                    .expect("type support getter symbol")
                };
                let ptr = unsafe { getter() };
                // Keep the library loaded for the process lifetime.
                std::mem::forget(lib);
                TypeSupportPtr(ptr)
            })
            .0
        }
    }

    /// A/B control: a TYPED rclrs subscription on `/ci/vel`.
    #[cfg(feature = "ros2")]
    #[test]
    #[ignore = "requires a live ROS 2 node + publisher; run by CI"]
    fn live_typed_smoke_receives_data() {
        use rclrs::{Context, InitOptions};
        use std::sync::atomic::AtomicUsize;
        use std::time::Duration;

        let count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&count);

        let context =
            Context::from_env(InitOptions::new().with_domain_id(Some(0))).expect("context");
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("rosfilter_typed_diag").expect("node");
        let _sub = node
            .create_subscription::<CiFloat64, _>(
                "/ci/vel",
                move |_msg: CiFloat64, _info: MessageInfo| {
                    count_clone.fetch_add(1, Ordering::Relaxed);
                },
            )
            .expect("typed subscription");

        // Single long spin (up to 6s) — the runner dispatches continuously.
        let errors = executor.spin(SpinOptions::default().timeout(Duration::from_secs(6)));
        eprintln!(
            "typed spin errors: {:?} (timeout is expected at the end)",
            errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
        );
        let n = count.load(Ordering::Relaxed);
        eprintln!("typed: received {n} messages");
        assert!(n > 0, "typed subscription received nothing on /ci/vel");
    }
}
