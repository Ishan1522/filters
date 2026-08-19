//! Thread-safe ring-buffer store for live topics.
//!
//! The live ROS 2 client thread writes samples here; the UI thread reads a
//! one-frame [`LogFile`] snapshot via [`LiveStore::snapshot_to_log`] — the
//! same "snapshot to log" bridge the app always used, now fed by ROS 2 topic
//! subscriptions.
//!
//! `Mutex` over `RwLock` because the workload is write-heavy on the network
//! side (every message is a write) and the UI reads once per frame.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

use crate::io::model::{Channel, LogFile, Sample, SampleValue};

/// Maximum samples to keep per topic. At 50 Hz * 30 s = 1500; faster topics
/// evict older history, slower topics use less than capacity.
const MAX_SAMPLES_PER_TOPIC: usize = 4096;

/// One topic's metadata + sample history as a ring buffer.
#[derive(Debug, Clone)]
pub struct LiveTopic {
    pub name: String,
    pub type_str: String,
    /// Stable per-session id assigned on first sight. Never reused within a
    /// session, so selections in the UI stay valid.
    pub entry_id: u32,
    pub samples: VecDeque<(u64, f64)>,
}

impl LiveTopic {
    fn push(&mut self, ts: u64, v: f64) {
        if self.samples.len() == MAX_SAMPLES_PER_TOPIC {
            self.samples.pop_front();
        }
        self.samples.push_back((ts, v));
    }
}

#[derive(Default)]
pub struct LiveStore {
    inner: Mutex<LiveStoreInner>,
}

#[derive(Default)]
struct LiveStoreInner {
    topics: HashMap<u32, LiveTopic>,
    next_entry_id: u32,
}

impl LiveStore {
    pub fn new() -> Arc<Self> {
        let s = Self::default();
        s.inner.lock().unwrap().next_entry_id = 1;
        Arc::new(s)
    }

    /// Register a topic (called by the client thread when a subscription is
    /// created). Returns its stable entry_id.
    pub fn announce(&self, name: String, type_str: String) -> u32 {
        let mut inner = self.inner.lock().unwrap();
        let entry_id = inner.next_entry_id;
        inner.next_entry_id += 1;
        inner.topics.insert(
            entry_id,
            LiveTopic {
                name,
                type_str,
                entry_id,
                samples: VecDeque::new(),
            },
        );
        entry_id
    }

    /// Called by the client thread for every received message.
    pub fn push(&self, entry_id: u32, ts_us: u64, v: f64) {
        let mut inner = self.inner.lock().unwrap();
        if let Some(topic) = inner.topics.get_mut(&entry_id) {
            topic.push(ts_us, v);
        }
    }

    /// Build a one-frame [`LogFile`] snapshot for the UI.
    pub fn snapshot_to_log(&self) -> LogFile {
        let inner = self.inner.lock().unwrap();
        let mut channels = Vec::with_capacity(inner.topics.len());
        let mut data = HashMap::with_capacity(inner.topics.len());

        // Sort by entry_id so the channel list is stable across frames.
        let mut entries: Vec<&LiveTopic> = inner.topics.values().collect();
        entries.sort_by_key(|t| t.entry_id);

        for topic in entries {
            channels.push(Channel {
                entry_id: topic.entry_id,
                name: topic.name.clone(),
                data_type: "float64".to_string(),
                metadata: topic.type_str.clone(),
            });
            let samples: Vec<Sample> = topic
                .samples
                .iter()
                .map(|&(ts, v)| Sample {
                    timestamp_us: ts,
                    value: SampleValue::Double(v),
                })
                .collect();
            data.insert(topic.entry_id, samples);
        }

        LogFile { channels, data }
    }

    /// (topics tracked, total samples buffered) for the UI status line.
    pub fn stats(&self) -> (usize, usize) {
        let inner = self.inner.lock().unwrap();
        let n_topics = inner.topics.len();
        let n_samples = inner.topics.values().map(|t| t.samples.len()).sum();
        (n_topics, n_samples)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_buffer_evicts_oldest() {
        let store = LiveStore::new();
        let id = store.announce("/foo".into(), "std_msgs/msg/Float64".into());
        for i in 0..(MAX_SAMPLES_PER_TOPIC + 100) as u64 {
            store.push(id, i, i as f64);
        }
        let (_n_topics, n_samples) = store.stats();
        assert_eq!(n_samples, MAX_SAMPLES_PER_TOPIC);

        // Oldest 100 should be gone.
        let log = store.snapshot_to_log();
        let samples = &log.data[&id];
        assert_eq!(samples[0].timestamp_us, 100);
    }

    #[test]
    fn snapshot_is_stable_across_calls() {
        let store = LiveStore::new();
        let a = store.announce("/a".into(), "std_msgs/msg/Float64".into());
        let b = store.announce("/b".into(), "std_msgs/msg/Float64".into());
        store.push(a, 1, 1.0);
        store.push(b, 2, 2.0);
        let s1 = store.snapshot_to_log();
        let s2 = store.snapshot_to_log();
        let ids1: Vec<u32> = s1.channels.iter().map(|c| c.entry_id).collect();
        let ids2: Vec<u32> = s2.channels.iter().map(|c| c.entry_id).collect();
        assert_eq!(ids1, ids2);
        assert_eq!(ids1, vec![a, b]);
    }

    #[test]
    fn push_to_unknown_id_is_ignored() {
        let store = LiveStore::new();
        store.push(999, 0, 1.0);
        let (n_topics, n_samples) = store.stats();
        assert_eq!((n_topics, n_samples), (0, 0));
    }

    #[test]
    fn snapshot_values_are_double_coerced() {
        let store = LiveStore::new();
        let id = store.announce("/x".into(), "std_msgs/msg/Bool".into());
        store.push(id, 5, 1.0);
        let log = store.snapshot_to_log();
        let v = &log.data[&id][0].value;
        assert_eq!(crate::io::convert::sample_to_f64(v), Some(1.0));
    }
}
