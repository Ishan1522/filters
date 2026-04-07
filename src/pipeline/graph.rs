use std::collections::{HashMap, HashSet};

use slotmap::{new_key_type, SlotMap};

use crate::analysis;
use crate::io::log_loader::{LogFile, SampleValue};
use crate::pipeline::node::{Node, NodeKind, Signal};
use crate::pipeline::nodes;

new_key_type! { pub struct NodeId; }

/// An edge in the graph: connects (from_node, from_output_port_idx)
/// to (to_node, to_input_port_idx).
#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub from_node: NodeId,
    pub from_port: usize,
    pub to_node:   NodeId,
    pub to_port:   usize,
}

#[derive(Debug)]
pub enum EvalError {
    UnconnectedInput { node: NodeId, port: usize },
    Cycle { at: NodeId },
    MissingChannel { entry_id: u32 },
}

#[derive(Debug, Default, Clone)]
pub struct FilterGraph {
    pub nodes: SlotMap<NodeId, Node>,
    pub edges: Vec<Edge>,
}

impl FilterGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_node(&mut self, node: Node) -> NodeId {
        self.nodes.insert(node)
    }

    pub fn remove_node(&mut self, id: NodeId) {
        self.nodes.remove(id);
        self.edges.retain(|e| e.from_node != id && e.to_node != id);
    }

    /// Connect (from_node, from_port) → (to_node, to_port).
    /// Removes any existing edge feeding into the same input port (an input
    /// port can only be driven by one source — that's how filter graphs work).
    pub fn connect(&mut self, from_node: NodeId, from_port: usize, to_node: NodeId, to_port: usize) {
        self.edges.retain(|e| !(e.to_node == to_node && e.to_port == to_port));
        self.edges.push(Edge { from_node, from_port, to_node, to_port });
    }

    pub fn disconnect(&mut self, to_node: NodeId, to_port: usize) {
        self.edges.retain(|e| !(e.to_node == to_node && e.to_port == to_port));
    }

    /// All nodes that are graph outputs (sinks).
    pub fn outputs(&self) -> Vec<NodeId> {
        self.nodes
            .iter()
            .filter(|(_, n)| matches!(n.kind, NodeKind::Output { .. }))
            .map(|(id, _)| id)
            .collect()
    }

    /// Evaluate every output node in the graph against the given log.
    /// Returns a map from output NodeId to its computed Signal.
    pub fn evaluate(&self, log: &LogFile) -> Result<HashMap<NodeId, Signal>, EvalError> {
        let mut ctx = EvalContext {
            graph: self,
            log,
            cache: HashMap::new(),
            in_flight: HashSet::new(),
        };

        let mut results = HashMap::new();
        for output_id in self.outputs() {
            // An Output node has exactly one input port (port 0). Resolve
            // whatever feeds it and store that as the output's value.
            let signal = ctx.eval_input(output_id, 0)?;
            results.insert(output_id, signal);
        }
        Ok(results)
    }
}

/// Per-evaluation state. The cache is keyed by NodeId — every node has at
/// most one logical "result" because all our current node types have at most
/// one output. If we add multi-output nodes later (FFT → mag + phase), the
/// cache key becomes (NodeId, output_port_idx).
struct EvalContext<'a> {
    graph:     &'a FilterGraph,
    log:       &'a LogFile,
    cache:     HashMap<NodeId, Signal>,
    in_flight: HashSet<NodeId>,
}

impl<'a> EvalContext<'a> {
    /// Resolve the signal feeding `input_port` of `node`, then return its value.
    fn eval_input(&mut self, node: NodeId, input_port: usize) -> Result<Signal, EvalError> {
        let edge = self
            .graph
            .edges
            .iter()
            .find(|e| e.to_node == node && e.to_port == input_port)
            .copied();
        let Some(edge) = edge else {
            return Err(EvalError::UnconnectedInput { node, port: input_port });
        };
        self.eval_node(edge.from_node)
    }

    /// Evaluate a node and return its (single) output signal. Memoized.
    fn eval_node(&mut self, id: NodeId) -> Result<Signal, EvalError> {
        if let Some(cached) = self.cache.get(&id) {
            return Ok(cached.clone());
        }
        if !self.in_flight.insert(id) {
            return Err(EvalError::Cycle { at: id });
        }

        let node = self.graph.nodes.get(id).expect("node id from graph");
        let result = match &node.kind {
            NodeKind::LogChannel { entry_id } => self.load_log_channel(*entry_id)?,

            NodeKind::Output { .. } => {
                // Outputs just pass through their input. The graph evaluator
                // also reads outputs separately in `evaluate()`, but this
                // branch is here in case an Output is referenced as a source.
                self.eval_input(id, 0)?
            }

            NodeKind::Filter { spec, zero_phase } => {
                let input = self.eval_input(id, 0)?;
                nodes::apply_filter(&input, spec, *zero_phase)
            }

            NodeKind::Sum => {
                let mut signals = Vec::with_capacity(node.inputs.len());
                for port_idx in 0..node.inputs.len() {
                    signals.push(self.eval_input(id, port_idx)?);
                }
                let refs: Vec<&Signal> = signals.iter().collect();
                nodes::sum_signals(&refs)
            }

            NodeKind::Differentiate => {
                let input = self.eval_input(id, 0)?;
                nodes::differentiate(&input)
            }

            NodeKind::Gain { factor } => {
                let input = self.eval_input(id, 0)?;
                nodes::gain(&input, *factor)
            }
        };

        self.in_flight.remove(&id);
        self.cache.insert(id, result.clone());
        Ok(result)
    }

    /// Load a log channel and convert it into the universal Signal type:
    /// resampled onto a uniform grid at the channel's natural rate.
    fn load_log_channel(&self, entry_id: u32) -> Result<Signal, EvalError> {
        let samples = self
            .log
            .data
            .get(&entry_id)
            .ok_or(EvalError::MissingChannel { entry_id })?;

        let mut timestamps = Vec::with_capacity(samples.len());
        let mut values     = Vec::with_capacity(samples.len());
        for s in samples {
            if let Some(v) = sample_to_f64(&s.value) {
                timestamps.push(s.timestamp_us);
                values.push(v);
            }
        }
        if timestamps.is_empty() {
            return Ok(Signal::empty());
        }

        let fs = analysis::estimate_sample_rate_hz(&timestamps).unwrap_or(0.0);
        let t0 = timestamps[0] as f64 / 1.0e6;
        let (_, uniform) = analysis::resample_uniform(&timestamps, &values, fs);
        Ok(Signal::new(uniform, fs, t0))
    }
}

fn sample_to_f64(v: &SampleValue) -> Option<f64> {
    match v {
        SampleValue::Double(x)    => Some(*x),
        SampleValue::Float(x)     => Some(*x as f64),
        SampleValue::Int64(x)     => Some(*x as f64),
        SampleValue::Boolean(b)   => Some(if *b { 1.0 } else { 0.0 }),
        SampleValue::StringVal(_) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsp::spec::{FilterKind, FilterSpec};
    use crate::pipeline::node::Node;
    use std::sync::Arc;

    fn synthetic_log() -> LogFile {
        // Build a tiny in-memory log with one channel of a noisy sine wave.
        use crate::io::log_loader::{Channel, Sample, SampleValue};
        let fs = 1000.0_f64;
        let n  = 1000usize;
        let mut data = HashMap::new();
        let samples: Vec<Sample> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let v = (2.0 * std::f64::consts::PI * 5.0 * t).sin();
                Sample {
                    timestamp_us: (t * 1.0e6) as u64,
                    value: SampleValue::Double(v),
                }
            })
            .collect();
        data.insert(1, samples);
        LogFile {
            channels: vec![Channel {
                entry_id:  1,
                name:      "sine".to_string(),
                data_type: "double".to_string(),
                metadata:  String::new(),
            }],
            data,
        }
    }

    #[test]
    fn end_to_end_log_to_filter_to_output() {
        let log = synthetic_log();
        let mut g = FilterGraph::new();

        let src = g.add_node(Node::new(NodeKind::LogChannel { entry_id: 1 }, (0.0, 0.0)));
        let mut spec = FilterSpec::default();
        spec.kind = FilterKind::ButterworthLowpass;
        spec.cutoff_hz = 50.0;
        spec.order = 4;
        let filt = g.add_node(Node::new(
            NodeKind::Filter { spec, zero_phase: false },
            (200.0, 0.0),
        ));
        let out = g.add_node(Node::new(
            NodeKind::Output { label: "smoothed".into() },
            (400.0, 0.0),
        ));

        g.connect(src, 0, filt, 0);
        g.connect(filt, 0, out, 0);

        let results = g.evaluate(&log).expect("eval ok");
        let signal = results.get(&out).expect("output present");
        assert!(signal.len() > 0);
        // Should have nonzero variance (it's a 5Hz sine, well below 50Hz cutoff).
        let mean: f64 = signal.samples.iter().sum::<f64>() / signal.len() as f64;
        let var: f64 = signal.samples.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / signal.len() as f64;
        assert!(var > 0.1, "variance = {}", var);
    }

    #[test]
    fn cycles_are_caught() {
        let log = synthetic_log();
        let mut g = FilterGraph::new();

        // Two filters wired into each other — impossible to evaluate.
        let spec = FilterSpec::default();
        let a = g.add_node(Node::new(NodeKind::Filter { spec, zero_phase: false }, (0.0, 0.0)));
        let b = g.add_node(Node::new(NodeKind::Filter { spec, zero_phase: false }, (200.0, 0.0)));
        let out = g.add_node(Node::new(NodeKind::Output { label: "x".into() }, (400.0, 0.0)));

        g.connect(a, 0, b, 0);
        g.connect(b, 0, a, 0);
        g.connect(b, 0, out, 0);

        let result = g.evaluate(&log);
        assert!(matches!(result, Err(EvalError::Cycle { .. })));
    }
}