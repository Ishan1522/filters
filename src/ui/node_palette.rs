use eframe::egui::{self, Pos2};

use crate::dsp::spec::FilterSpec;
use crate::pipeline::FilterGraph;
use crate::pipeline::node::{Node, NodeKind};

/// Renders the "Add node" popup contents and inserts the chosen node at `pos`.
/// The popup itself is owned by egui (via response.context_menu); we just
/// fill in the buttons.
pub fn show(ui: &mut egui::Ui, graph: &mut FilterGraph, pos: Pos2) {
    let mut close = false;

    if ui.button("Filter").clicked() {
        graph.add_node(Node::new(
            NodeKind::Filter { spec: FilterSpec::default(), zero_phase: false },
            (pos.x, pos.y),
        ));
        close = true;
    }
    if ui.button("Sum").clicked() {
        graph.add_node(Node::new(NodeKind::Sum, (pos.x, pos.y)));
        close = true;
    }
    if ui.button("Differentiate").clicked() {
        graph.add_node(Node::new(NodeKind::Differentiate, (pos.x, pos.y)));
        close = true;
    }
    if ui.button("Gain").clicked() {
        graph.add_node(Node::new(NodeKind::Gain { factor: 1.0 }, (pos.x, pos.y)));
        close = true;
    }

    ui.separator();
    ui.label("Source / sink");

    // Note: log channel sources are added from the inspector or a separate
    // panel — we don't show them here because we'd need access to the LogFile
    // to enumerate channel ids, and the palette is purely about node *types*.
    if ui.button("Output").clicked() {
        graph.add_node(Node::new(
            NodeKind::Output { label: "out".to_string() },
            (pos.x, pos.y),
        ));
        close = true;
    }

    if close {
        ui.close_menu();
    }
}