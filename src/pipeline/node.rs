use std::sync::Arc;

/// The universal data type that flows on every edge.
/// Uniformly-sampled f64 values at a known sample rate.
/// Wrapped in Arc so fan-out doesn't copy.
#[derive(Debug, Clone)]
pub struct Signal {
    pub samples:        Arc<Vec<f64>>,
    pub sample_rate_hz: f64,
    /// Time of the first sample, in seconds. Preserved through processing
    /// so the UI can plot output signals against the original time axis.
    pub t0_seconds:     f64,
}

impl Signal {
    pub fn new(samples: Vec<f64>, sample_rate_hz: f64, t0_seconds: f64) -> Self {
        Self {
            samples: Arc::new(samples),
            sample_rate_hz,
            t0_seconds,
        }
    }

    pub fn empty() -> Self {
        Self::new(Vec::new(), 0.0, 0.0)
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

/// A port is a labeled input or output on a node. The label is for the UI;
/// internally we identify ports by their index in the node's port list.
#[derive(Debug, Clone)]
pub struct Port {
    pub name: String,
}

impl Port {
    pub fn new(name: impl Into<String>) -> Self {
        Self { name: name.into() }
    }
}

/// What a node *does*. This is the data-driven part of the graph: every
/// concrete node type is a variant of this enum, and the evaluator dispatches
/// on it. We could also use a trait object (Box<dyn NodeBehavior>), but enums
/// give us:
///   - free Clone/Debug for save/load
///   - exhaustive matching at the call site (compiler tells us if we forget
///     to handle a new node type)
///   - simpler serialization later
/// The downside is that adding a new node type means touching this enum and
/// the dispatch site, but for a tool with maybe 15 node types total, that's fine.
#[derive(Debug, Clone)]
pub enum NodeKind {
    /// Source node: pulls a channel (rosbag topic / live topic) from the
    /// loaded `LogFile` by channel id. Has 0 inputs, 1 output.
    TopicSource { channel_id: u32 },

    /// Sink: marks a signal as a "graph output" for the UI to plot.
    /// Has 1 input, 0 outputs (well, 0 outgoing graph edges — the value
    /// is read by the UI directly via the evaluator's cache).
    Output { label: String },

    /// Apply a compiled biquad cascade to the input signal.
    /// 1 input, 1 output. The FilterSpec is stored so the UI can edit it
    /// and the cascade is recompiled lazily during evaluation.
    Filter { spec: crate::dsp::spec::FilterSpec, zero_phase: bool },

    /// Sum N inputs sample-by-sample. Variable input count, 1 output.
    Sum,

    /// Discrete derivative: y[n] = (x[n] − x[n−1]) · fs.
    /// 1 input, 1 output.
    Differentiate,

    /// Constant-gain multiplier. 1 input, 1 output.
    Gain { factor: f64 },
}

#[derive(Debug, Clone)]
pub struct Node {
    pub kind:    NodeKind,
    pub inputs:  Vec<Port>,
    pub outputs: Vec<Port>,
    /// UI-facing display name. Defaults to something derived from `kind`.
    pub label:   String,
    /// Position in the graph editor canvas. Stored in the data layer
    /// (rather than UI state) so it can be saved/loaded with the graph.
    pub pos:     (f32, f32),
}

impl Node {
    pub fn new(kind: NodeKind, pos: (f32, f32)) -> Self {
        let (inputs, outputs, label) = ports_for_kind(&kind);
        Self { kind, inputs, outputs, label, pos }
    }
}

/// Maps a NodeKind to its port layout and default label. Centralized here
/// so the UI and the evaluator agree on what every node type looks like.
fn ports_for_kind(kind: &NodeKind) -> (Vec<Port>, Vec<Port>, String) {
    match kind {
        NodeKind::TopicSource { channel_id } => (
            vec![],
            vec![Port::new("out")],
            format!("topic channel #{channel_id}"),
        ),
        NodeKind::Output { label } => (
            vec![Port::new("in")],
            vec![],
            format!("output: {label}"),
        ),
        NodeKind::Filter { .. } => (
            vec![Port::new("in")],
            vec![Port::new("out")],
            "filter".to_string(),
        ),
        NodeKind::Sum => (
            // Sum starts with 2 inputs; UI can add more by mutating the node.
            vec![Port::new("a"), Port::new("b")],
            vec![Port::new("sum")],
            "sum".to_string(),
        ),
        NodeKind::Differentiate => (
            vec![Port::new("in")],
            vec![Port::new("d/dt")],
            "differentiate".to_string(),
        ),
        NodeKind::Gain { factor } => (
            vec![Port::new("in")],
            vec![Port::new("out")],
            format!("× {factor}"),
        ),
    }
}