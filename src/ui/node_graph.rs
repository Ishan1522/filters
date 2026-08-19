use eframe::egui::{self, Color32, Pos2, Rect, Sense, Stroke, Vec2};

use crate::io::model::LogFile;
use crate::pipeline::{FilterGraph, NodeId};
use crate::pipeline::node::NodeKind;

use egui_plot::{Legend, Line, Plot, PlotPoints};

const NODE_WIDTH:    f32 = 160.0;
const NODE_HEADER_H: f32 = 22.0;
const PORT_RADIUS:   f32 = 6.0;
const PORT_SPACING:  f32 = 18.0;
const NODE_PADDING:  f32 = 8.0;

/// What the user is currently doing on the canvas. Mutually exclusive states —
/// at any moment we are in exactly one of these.
#[derive(Debug, Clone)]
enum Interaction {
    Idle,
    DraggingNode {
        node:        NodeId,
        grab_offset: Vec2,  // cursor pos − node pos at drag start
    },
    DraggingWire {
        from_node: NodeId,
        from_port: usize,
        cursor:    Pos2,    // current mouse position in graph space
    },
}

impl Default for Interaction {
    fn default() -> Self { Self::Idle }
}

/// Result of hit-testing the cursor position against the graph.
/// Order of variants matters: ports must be tested before bodies, and we
/// short-circuit on the first hit.
#[derive(Debug, Clone, Copy)]
enum HitTest {
    Empty,
    NodeBody(NodeId),
    OutputPort { node: NodeId, port: usize },
    InputPort  { node: NodeId, port: usize },
}

#[derive(Default)]
pub struct NodeGraphView {
    interaction: Interaction,
    /// Currently selected node (for the inspector panel). `None` = nothing selected.
    pub selected: Option<NodeId>,
}

impl NodeGraphView {
    pub fn ui(&mut self, ui: &mut egui::Ui, graph: &mut FilterGraph) {
        // Allocate the canvas. We use Sense::click_and_drag so we get both
        // click events (for selection / right-click menu) and drag events
        // (for moving nodes / drawing wires).
        let available = ui.available_rect_before_wrap();
        let (canvas_rect, response) = ui.allocate_exact_size(
            available.size(),
            Sense::click_and_drag(),
        );

        // Background fill so the canvas is visually distinct from the surrounding UI.
        ui.painter().rect_filled(
            canvas_rect,
            4.0,
            Color32::from_rgb(28, 28, 32),
        );
        Self::draw_grid(ui, canvas_rect);

        // Convert cursor (screen) to graph space. Today this is just an
        // offset; if we add pan/zoom later, this becomes a real transform.
        let cursor_screen = response.hover_pos();
        let cursor_graph  = cursor_screen.map(|p| Self::screen_to_graph(p, canvas_rect));

        // Hit-test the cursor against the graph contents *before* we mutate
        // anything. We need this for both event handling and hover highlighting.
        let hover = cursor_graph
            .map(|c| Self::hit_test(graph, c))
            .unwrap_or(HitTest::Empty);

        // ───── input handling ─────
        self.handle_input(ui, graph, &response, cursor_graph, hover);

        // ───── drawing ─────
        // Draw edges first (they should appear *under* the nodes). The
        // in-progress wire (if any) is drawn last so it floats on top.
        let painter = ui.painter_at(canvas_rect);
        Self::draw_edges(&painter, graph, canvas_rect);
        for (id, _) in graph.nodes.iter() {
            self.draw_node(&painter, graph, id, canvas_rect, hover);
        }
        if let Interaction::DraggingWire { from_node, from_port, cursor } = &self.interaction {
            let from_screen = Self::output_port_pos(graph, *from_node, *from_port)
                .map(|p| Self::graph_to_screen(p, canvas_rect));
            if let Some(from) = from_screen {
                let to = Self::graph_to_screen(*cursor, canvas_rect);
                Self::draw_wire(&painter, from, to, Color32::from_rgb(140, 200, 255));
            }
        }

        // ───── right-click context menu ─────
        // egui handles this via response.context_menu — it shows a popup
        // anchored to where the user right-clicked, and we populate it with
        // node-creation options. The menu closure runs only when open.
        response.context_menu(|ui| {
            ui.label("Add node");
            ui.separator();
            let add_pos = cursor_graph.unwrap_or(Pos2::new(100.0, 100.0));
            crate::ui::node_palette::show(ui, graph, add_pos);
        });
    }

    // ── input handling ────────────────────────────────────────────────

    fn handle_input(
        &mut self,
        ui:           &egui::Ui,
        graph:        &mut FilterGraph,
        response:     &egui::Response,
        cursor_graph: Option<Pos2>,
        hover:        HitTest,
    ) {
        // Pressed = the frame the mouse went down. Released = the frame it went up.
        // Dragged = held + moving. We use these to drive the state machine.
        let pressed  = response.drag_started_by(egui::PointerButton::Primary);
        let released = response.drag_stopped_by(egui::PointerButton::Primary);
        let clicked  = response.clicked_by(egui::PointerButton::Primary);

        // ── start of an interaction ──
        if pressed {
            if let Some(cursor) = cursor_graph {
                match hover {
                    HitTest::OutputPort { node, port } => {
                        self.interaction = Interaction::DraggingWire {
                            from_node: node,
                            from_port: port,
                            cursor,
                        };
                    }
                    HitTest::NodeBody(id) => {
                        if let Some(n) = graph.nodes.get(id) {
                            let grab_offset = cursor - Pos2::new(n.pos.0, n.pos.1);
                            self.interaction = Interaction::DraggingNode {
                                node: id,
                                grab_offset,
                            };
                            self.selected = Some(id);
                        }
                    }
                    // Clicking on an input port doesn't do anything yet
                    // (we could support "drag from input to output" too,
                    // but it's a UX detail — let's ship the basic flow).
                    HitTest::InputPort { .. } => {}
                    HitTest::Empty => {
                        self.selected = None;
                    }
                }
            }
        }

        // ── ongoing interaction ──
        match &mut self.interaction {
            Interaction::DraggingNode { node, grab_offset } => {
                if let Some(cursor) = cursor_graph {
                    if let Some(n) = graph.nodes.get_mut(*node) {
                        let new_pos = cursor - *grab_offset;
                        n.pos = (new_pos.x, new_pos.y);
                    }
                }
            }
            Interaction::DraggingWire { cursor, .. } => {
                if let Some(c) = cursor_graph {
                    *cursor = c;
                }
            }
            Interaction::Idle => {}
        }

        // ── end of an interaction ──
        if released {
            // Capture the wire info *before* resetting state, because we need
            // it to actually create the connection on a successful drop.
            let wire = if let Interaction::DraggingWire { from_node, from_port, .. } = &self.interaction {
                Some((*from_node, *from_port))
            } else {
                None
            };

            if let Some((from_node, from_port)) = wire {
                if let HitTest::InputPort { node: to_node, port: to_port } = hover {
                    // Don't allow connecting a node to itself.
                    if from_node != to_node {
                        graph.connect(from_node, from_port, to_node, to_port);
                    }
                }
                // Any other release target → wire is silently dropped.
            }

            self.interaction = Interaction::Idle;
        }

        // ── plain click on empty space (no drag): also clears selection ──
        // egui fires `clicked` on release-without-drag, which is a separate
        // path from `released` above (drags don't fire `clicked`).
        if clicked && matches!(hover, HitTest::Empty) {
            self.selected = None;
        }

        // ── delete key on selected node ──
        if let Some(id) = self.selected {
            if ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace)) {
                graph.remove_node(id);
                self.selected = None;
                self.interaction = Interaction::Idle;
            }
        }
    }

    // ── hit testing ───────────────────────────────────────────────────

    fn hit_test(graph: &FilterGraph, pos: Pos2) -> HitTest {
        // Walk nodes in reverse insertion order (last drawn = topmost).
        // For each node: test output ports, input ports, then body.
        for (id, node) in graph.nodes.iter().collect::<Vec<_>>().into_iter().rev() {
            // Output ports.
            for (i, _) in node.outputs.iter().enumerate() {
                let p = Self::output_port_pos_for(node, i);
                if (p - pos).length() <= PORT_RADIUS + 2.0 {
                    return HitTest::OutputPort { node: id, port: i };
                }
            }
            // Input ports.
            for (i, _) in node.inputs.iter().enumerate() {
                let p = Self::input_port_pos_for(node, i);
                if (p - pos).length() <= PORT_RADIUS + 2.0 {
                    return HitTest::InputPort { node: id, port: i };
                }
            }
            // Body.
            let r = Self::node_rect_for(node);
            if r.contains(pos) {
                return HitTest::NodeBody(id);
            }
        }
        HitTest::Empty
    }

    // ── geometry helpers ──────────────────────────────────────────────

    fn node_height(node: &crate::pipeline::node::Node) -> f32 {
        let rows = node.inputs.len().max(node.outputs.len()).max(1);
        NODE_HEADER_H + rows as f32 * PORT_SPACING + NODE_PADDING * 2.0
    }

    fn node_rect_for(node: &crate::pipeline::node::Node) -> Rect {
        let top_left = Pos2::new(node.pos.0, node.pos.1);
        Rect::from_min_size(top_left, Vec2::new(NODE_WIDTH, Self::node_height(node)))
    }

    fn input_port_pos_for(node: &crate::pipeline::node::Node, idx: usize) -> Pos2 {
        let rect = Self::node_rect_for(node);
        Pos2::new(
            rect.left(),
            rect.top() + NODE_HEADER_H + NODE_PADDING + idx as f32 * PORT_SPACING + PORT_SPACING * 0.5,
        )
    }

    fn output_port_pos_for(node: &crate::pipeline::node::Node, idx: usize) -> Pos2 {
        let rect = Self::node_rect_for(node);
        Pos2::new(
            rect.right(),
            rect.top() + NODE_HEADER_H + NODE_PADDING + idx as f32 * PORT_SPACING + PORT_SPACING * 0.5,
        )
    }

    fn output_port_pos(graph: &FilterGraph, id: NodeId, port: usize) -> Option<Pos2> {
        graph.nodes.get(id).map(|n| Self::output_port_pos_for(n, port))
    }

    fn input_port_pos(graph: &FilterGraph, id: NodeId, port: usize) -> Option<Pos2> {
        graph.nodes.get(id).map(|n| Self::input_port_pos_for(n, port))
    }

    fn screen_to_graph(p: Pos2, canvas: Rect) -> Pos2 {
        // Today: simple translation. With pan/zoom: apply inverse transform.
        p - canvas.min.to_vec2()
    }

    fn graph_to_screen(p: Pos2, canvas: Rect) -> Pos2 {
        p + canvas.min.to_vec2()
    }

    // ── drawing ───────────────────────────────────────────────────────

    fn draw_grid(ui: &egui::Ui, rect: Rect) {
        let painter = ui.painter_at(rect);
        let step = 24.0;
        let color = Color32::from_rgb(40, 40, 46);
        let stroke = Stroke::new(1.0, color);

        let mut x = rect.left();
        while x < rect.right() {
            painter.line_segment([Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())], stroke);
            x += step;
        }
        let mut y = rect.top();
        while y < rect.bottom() {
            painter.line_segment([Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)], stroke);
            y += step;
        }
    }

    fn draw_node(
        &self,
        painter: &egui::Painter,
        graph:   &FilterGraph,
        id:      NodeId,
        canvas:  Rect,
        hover:   HitTest,
    ) {
        let Some(node) = graph.nodes.get(id) else { return };

        // Translate the node's graph-space rect into screen space for drawing.
        let graph_rect = Self::node_rect_for(node);
        let screen_rect = Rect::from_min_size(
            Self::graph_to_screen(graph_rect.min, canvas),
            graph_rect.size(),
        );

        let selected = self.selected == Some(id);
        let body_color = match &node.kind {
            NodeKind::TopicSource { .. } => Color32::from_rgb(60, 80, 110),
            NodeKind::Output { .. }      => Color32::from_rgb(110, 70, 70),
            NodeKind::Filter { .. }      => Color32::from_rgb(70, 100, 70),
            NodeKind::Sum                => Color32::from_rgb(90, 80, 110),
            NodeKind::Differentiate      => Color32::from_rgb(100, 90, 60),
            NodeKind::Gain { .. }        => Color32::from_rgb(80, 80, 80),
        };

        // Body.
        painter.rect_filled(screen_rect, 4.0, body_color);
        let border = if selected {
            Stroke::new(2.0, Color32::from_rgb(255, 220, 100))
        } else {
            Stroke::new(1.0, Color32::from_rgb(20, 20, 24))
        };
        painter.rect_stroke(screen_rect, 4.0, border);
        // Header / label.
        let header_rect = Rect::from_min_size(
            screen_rect.min,
            Vec2::new(NODE_WIDTH, NODE_HEADER_H),
        );
        painter.rect_filled(
            header_rect,
            egui::Rounding { nw: 4.0, ne: 4.0, sw: 0.0, se: 0.0 },
            body_color.gamma_multiply(0.7),
        );
        painter.text(
            header_rect.center(),
            egui::Align2::CENTER_CENTER,
            &node.label,
            egui::FontId::proportional(12.0),
            Color32::WHITE,
        );

        // Input ports (left edge).
        for (i, port) in node.inputs.iter().enumerate() {
            let p_graph = Self::input_port_pos_for(node, i);
            let p_screen = Self::graph_to_screen(p_graph, canvas);
            let highlight = matches!(hover, HitTest::InputPort { node: n, port: pi } if n == id && pi == i);
            let color = if highlight { Color32::from_rgb(255, 220, 100) } else { Color32::from_rgb(180, 180, 200) };
            painter.circle_filled(p_screen, PORT_RADIUS, color);
            painter.text(
                p_screen + Vec2::new(PORT_RADIUS + 4.0, 0.0),
                egui::Align2::LEFT_CENTER,
                &port.name,
                egui::FontId::proportional(10.0),
                Color32::from_rgb(200, 200, 210),
            );
        }

        // Output ports (right edge).
        for (i, port) in node.outputs.iter().enumerate() {
            let p_graph = Self::output_port_pos_for(node, i);
            let p_screen = Self::graph_to_screen(p_graph, canvas);
            let highlight = matches!(hover, HitTest::OutputPort { node: n, port: pi } if n == id && pi == i);
            let color = if highlight { Color32::from_rgb(255, 220, 100) } else { Color32::from_rgb(180, 200, 180) };
            painter.circle_filled(p_screen, PORT_RADIUS, color);
            painter.text(
                p_screen - Vec2::new(PORT_RADIUS + 4.0, 0.0),
                egui::Align2::RIGHT_CENTER,
                &port.name,
                egui::FontId::proportional(10.0),
                Color32::from_rgb(200, 200, 210),
            );
        }
    }

    fn draw_edges(painter: &egui::Painter, graph: &FilterGraph, canvas: Rect) {
        for edge in &graph.edges {
            let from = Self::output_port_pos(graph, edge.from_node, edge.from_port);
            let to   = Self::input_port_pos(graph, edge.to_node, edge.to_port);
            if let (Some(f), Some(t)) = (from, to) {
                Self::draw_wire(
                    painter,
                    Self::graph_to_screen(f, canvas),
                    Self::graph_to_screen(t, canvas),
                    Color32::from_rgb(140, 160, 180),
                );
            }
        }
    }

    /// A bezier-style wire from output to input. Uses cubic control points
    /// offset horizontally so the wire approaches each port head-on.
    fn draw_wire(painter: &egui::Painter, from: Pos2, to: Pos2, color: Color32) {
        let dx = (to.x - from.x).abs().max(40.0) * 0.5;
        let c1 = Pos2::new(from.x + dx, from.y);
        let c2 = Pos2::new(to.x   - dx, to.y);

        // Sample the cubic with a polyline. egui has a CubicBezierShape but
        // a manual polyline is simpler and gives identical visual results.
        let mut points = Vec::with_capacity(33);
        for i in 0..=32 {
            let t = i as f32 / 32.0;
            let one_t = 1.0 - t;
            let p = from.to_vec2() * (one_t * one_t * one_t)
                  + c1.to_vec2()   * (3.0 * one_t * one_t * t)
                  + c2.to_vec2()   * (3.0 * one_t * t * t)
                  + to.to_vec2()   * (t * t * t);
            points.push(Pos2::new(p.x, p.y));
        }
        painter.add(egui::Shape::line(points, Stroke::new(2.0, color)));
    }
}

/// Plot every Output node's evaluated signal. Lives next to the canvas so
/// users see the live result of their pipeline as they edit.
pub fn plot_graph_outputs(ui: &mut egui::Ui, graph: &FilterGraph, log: &LogFile) {
    let results = match graph.evaluate(log) {
        Ok(r) => r,
        Err(e) => {
            ui.colored_label(
                Color32::from_rgb(255, 140, 140),
                format!("Graph error: {e:?}"),
            );
            return;
        }
    };

    if results.is_empty() {
        ui.label("Add an Output node to see results here.");
        return;
    }

    Plot::new("graph_output_plot")
        .legend(Legend::default())
        .x_axis_label("time (s)")
        .y_axis_label("value")
        .height(220.0)
        .show(ui, |plot_ui| {
            for (id, signal) in &results {
                if signal.is_empty() { continue; }
                let label = graph
                    .nodes
                    .get(*id)
                    .map(|n| n.label.clone())
                    .unwrap_or_else(|| "output".to_string());
                let dt = 1.0 / signal.sample_rate_hz.max(1.0);
                let pts: PlotPoints = signal
                    .samples
                    .iter()
                    .enumerate()
                    .map(|(i, &v)| [signal.t0_seconds + i as f64 * dt, v])
                    .collect();
                plot_ui.line(Line::new(pts).name(label));
            }
        });
}
