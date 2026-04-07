use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

use crate::io::log_loader::{Channel, LogFile, Sample, SampleValue};
use crate::io::nt4_messages::Value;

/// Maximum samples to keep per topic. At 50 Hz robot loop * 30 seconds = 1500.
/// Faster topics get more history-time-equivalent dropped; slower topics use
/// less than capacity. We could make this configurable per topic later.
const MAX_SAMPLES_PER_TOPIC: usize = 4096;

/// One topic's metadata + sample history. The samples are stored as a
/// VecDeque so we can push to the back and pop from the front in O(1).
#[derive(Debug, Clone)]
pub struct LiveTopic {
    pub name:      String,
    pub type_str:  String,
    /// Stable per-session id we assign on first sight. We do *not* reuse the
    /// NT4 server's topic id, because the server can re-announce a topic
    /// with a different id after an unannounce, and signal_view holds
    /// HashSet<u32> of selected entry_ids that would silently break.
    pub entry_id:  u32,
    pub samples:   VecDeque<(u64, f64)>,
}

impl LiveTopic {
    fn push(&mut self, ts: u64, v: f64) {
        if self.samples.len() == MAX_SAMPLES_PER_TOPIC {
            self.samples.pop_front();
        }
        self.samples.push_back((ts, v));
    }
}

/// Thread-safe live data store. The network thread writes via `apply_*`
/// methods, the UI thread reads via `snapshot_to_log`. All access goes
/// through the Mutex.
///
/// We chose Mutex over RwLock because the workload is write-heavy on the
/// network side (every value frame is a write) and the UI side reads once
/// per frame — the contention pattern doesn't favor RwLock.
#[derive(Default)]
pub struct LiveStore {
    inner: Mutex<LiveStoreInner>,
}

#[derive(Default)]
struct LiveStoreInner {
    /// Map from server-assigned topic id to our stable entry_id.
    server_id_to_entry: HashMap<u32, u32>,
    /// Topics keyed by entry_id (the stable id we assign).
    topics:             HashMap<u32, LiveTopic>,
    /// Counter for assigning entry_ids. Starts at 1 to avoid collision
    /// with the "control record" entry_id 0 used by WPILOG.
    next_entry_id:      u32,
}

impl LiveStore {
    pub fn new() -> Arc<Self> {
        let s = Self::default();
        s.inner.lock().unwrap().next_entry_id = 1;
        Arc::new(s)
    }

    /// Called by the network thread when an `announce` message arrives.
    pub fn announce(&self, server_id: u32, name: String, type_str: String) {
        let mut inner = self.inner.lock().unwrap();
        // If we've seen this server_id before, this is a re-announce — drop
        // the old entry mapping and create a fresh one.
        if let Some(old_entry) = inner.server_id_to_entry.remove(&server_id) {
            inner.topics.remove(&old_entry);
        }
        let entry_id = inner.next_entry_id;
        inner.next_entry_id += 1;
        inner.server_id_to_entry.insert(server_id, entry_id);
        inner.topics.insert(entry_id, LiveTopic {
            name,
            type_str,
            entry_id,
            samples: VecDeque::new(),
        });
    }

    pub fn unannounce(&self, server_id: u32) {
        let mut inner = self.inner.lock().unwrap();
        if let Some(entry_id) = inner.server_id_to_entry.remove(&server_id) {
            // Keep the historical samples around — the user may still want
            // to look at them — but mark by removing the server mapping so
            // no new samples can land. We don't actually delete the topic.
            // If you want it gone, restart the live session.
            let _ = entry_id;
        }
    }

    /// Called by the network thread for every parsed value update.
    pub fn push_value(&self, server_id: u32, ts_us: u64, value: Value) {
        let v = match value {
            Value::Double(x)    => x,
            Value::Int64(x)     => x as f64,
            Value::Float(x)     => x as f64,
            Value::Boolean(b)   => if b { 1.0 } else { 0.0 },
            Value::StringVal(_) => return, // not plottable, drop silently
        };
        let mut inner = self.inner.lock().unwrap();
        let Some(&entry_id) = inner.server_id_to_entry.get(&server_id) else { return };
        if let Some(topic) = inner.topics.get_mut(&entry_id) {
            topic.push(ts_us, v);
        }
    }

    /// Build a one-frame `LogFile` snapshot for the UI to consume.
    /// This is the bridge that lets all the existing static-log code work
    /// unchanged on live data.
    pub fn snapshot_to_log(&self) -> LogFile {
        let inner = self.inner.lock().unwrap();
        let mut channels = Vec::with_capacity(inner.topics.len());
        let mut data     = HashMap::with_capacity(inner.topics.len());

        // Sort by entry_id so the channel list is stable across frames
        // (otherwise checkboxes would jump around as the HashMap iteration
        // order changed).
        let mut entries: Vec<&LiveTopic> = inner.topics.values().collect();
        entries.sort_by_key(|t| t.entry_id);

        for topic in entries {
            channels.push(Channel {
                entry_id:  topic.entry_id,
                name:      topic.name.clone(),
                data_type: nt4_type_to_wpilog(&topic.type_str).to_string(),
                metadata:  String::new(),
            });
            let samples: Vec<Sample> = topic
                .samples
                .iter()
                .map(|&(ts, v)| Sample {
                    timestamp_us: ts,
                    value:        SampleValue::Double(v), // we already coerced to f64 on push
                })
                .collect();
            data.insert(topic.entry_id, samples);
        }

        LogFile { channels, data }
    }

    /// For UI display: how many topics are we tracking, how many total samples?
    pub fn stats(&self) -> (usize, usize) {
        let inner = self.inner.lock().unwrap();
        let n_topics  = inner.topics.len();
        let n_samples = inner.topics.values().map(|t| t.samples.len()).sum();
        (n_topics, n_samples)
    }
}

fn nt4_type_to_wpilog(nt4: &str) -> &'static str {
    match nt4 {
        "double"  => "double",
        "int"     => "int64",
        "float"   => "float",
        "boolean" => "boolean",
        "string"  => "string",
        _         => "double", // we coerced to double on push, so this is correct
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_buffer_evicts_oldest() {
        let store = LiveStore::new();
        store.announce(10, "/foo".into(), "double".into());
        for i in 0..MAX_SAMPLES_PER_TOPIC + 100 {
            store.push_value(10, i as u64, Value::Double(i as f64));
        }
        let (_n_topics, n_samples) = store.stats();
        assert_eq!(n_samples, MAX_SAMPLES_PER_TOPIC);

        // Oldest 100 should be gone.
        let log = store.snapshot_to_log();
        let entry_id = log.channels[0].entry_id;
        let samples = &log.data[&entry_id];
        assert_eq!(samples[0].timestamp_us, 100);
    }

    #[test]
    fn snapshot_is_stable_across_calls() {
        let store = LiveStore::new();
        store.announce(1, "/a".into(), "double".into());
        store.announce(2, "/b".into(), "double".into());
        let s1 = store.snapshot_to_log();
        let s2 = store.snapshot_to_log();
        let ids1: Vec<u32> = s1.channels.iter().map(|c| c.entry_id).collect();
        let ids2: Vec<u32> = s2.channels.iter().map(|c| c.entry_id).collect();
        assert_eq!(ids1, ids2);
    }
}