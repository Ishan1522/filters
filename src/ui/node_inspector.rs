use eframe::egui;

use crate::dsp::spec::{FilterKind, FilterSpec};
use crate::io::convert::is_plottable;
use crate::io::model::LogFile;
use crate::pipeline::node::{Node, NodeKind};
use crate::pipeline::{FilterGraph, NodeId};

pub struct NodeInspector;

impl NodeInspector {
    pub fn ui(
        ui:       &mut egui::Ui,
        graph:    &mut FilterGraph,
        selected: Option<NodeId>,
        log:      Option<&LogFile>,
    ) {
        ui.heading("Inspector");
        ui.separator();

        match selected {
            Some(id) => Self::edit_node(ui, graph, id),
            None     => Self::sources_panel(ui, graph, log),
        }
    }

    fn edit_node(ui: &mut egui::Ui, graph: &mut FilterGraph, id: NodeId) {
        let Some(node) = graph.nodes.get_mut(id) else {
            ui.label("(node no longer exists)");
            return;
        };

        ui.text_edit_singleline(&mut node.label);
        ui.separator();

        match &mut node.kind {
            NodeKind::TopicSource { channel_id } => {
                ui.label(format!("Topic channel #{channel_id}"));
                ui.small("Sources are read-only — delete and re-add to change channel.");
            }
            NodeKind::Output { label } => {
                ui.horizontal(|ui| {
                    ui.label("Output label:");
                    ui.text_edit_singleline(label);
                });
            }
            NodeKind::Filter { spec, zero_phase } => {
                Self::edit_filter_spec(ui, spec, zero_phase);
            }
            NodeKind::Sum => {
                ui.label(format!("Inputs: {}", node.inputs.len()));
                if ui.button("+ add input").clicked() {
                    let next = node.inputs.len();
                    node.inputs.push(crate::pipeline::node::Port::new(format!("in{next}")));
                }
                if node.inputs.len() > 2 && ui.button("− remove last").clicked() {
                    let last = node.inputs.len() - 1;
                    node.inputs.pop();
                    // Also drop any edges feeding the removed port.
                    graph.disconnect(id, last);
                }
            }
            NodeKind::Differentiate => {
                ui.small("y[n] = (x[n] − x[n−1]) · fs");
            }
            NodeKind::Gain { factor } => {
                ui.add(egui::Slider::new(factor, -10.0..=10.0).text("factor"));
            }
        }
    }

    fn edit_filter_spec(ui: &mut egui::Ui, spec: &mut FilterSpec, zero_phase: &mut bool) {
        ui.checkbox(&mut spec.enabled, "Enabled");

        egui::ComboBox::from_label("Type")
            .selected_text(spec.kind.label())
            .show_ui(ui, |ui| {
                for k in [
                    FilterKind::ButterworthLowpass,
                    FilterKind::Chebyshev1Lowpass,
                    FilterKind::CookbookLowpass,
                    FilterKind::CookbookHighpass,
                    FilterKind::CookbookBandpass,
                    FilterKind::CookbookNotch,
                ] {
                    ui.selectable_value(&mut spec.kind, k, k.label());
                }
            });

        ui.add(
            egui::Slider::new(&mut spec.cutoff_hz, 0.1..=500.0)
                .logarithmic(true)
                .text("Cutoff (Hz)"),
        );
        if spec.kind.uses_q() {
            ui.add(egui::Slider::new(&mut spec.q, 0.1..=20.0).logarithmic(true).text("Q"));
        }
        if spec.kind.uses_order() {
            ui.add(egui::Slider::new(&mut spec.order, 1..=10).text("Order"));
        }
        if spec.kind.uses_ripple() {
            ui.add(egui::Slider::new(&mut spec.ripple_db, 0.1..=6.0).text("Ripple (dB)"));
        }

        ui.checkbox(zero_phase, "Zero-phase (filtfilt)");
    }

    /// When nothing is selected, show a panel for adding TopicSource nodes
    /// from the loaded bag / live snapshot. This is the entry point that the
    /// right-click palette can't provide because it doesn't have LogFile access.
    fn sources_panel(ui: &mut egui::Ui, graph: &mut FilterGraph, log: Option<&LogFile>) {
        ui.label("(no node selected)");
        ui.separator();
        ui.label("Add source from bag:");

        let Some(log) = log else {
            ui.small("No bag loaded.");
            return;
        };

        egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
            for ch in &log.channels {
                if !is_plottable(&ch.data_type) {
                    continue;
                }
                let label = format!("+ {} ({})", ch.name, ch.data_type);
                if ui.button(label).clicked() {
                    let mut node = Node::new(
                        NodeKind::TopicSource { channel_id: ch.entry_id },
                        (40.0, 40.0 + (graph.nodes.len() as f32 * 30.0) % 400.0),
                    );
                    node.label = ch.name.clone();
                    graph.add_node(node);
                }
            }
        });
    }
}