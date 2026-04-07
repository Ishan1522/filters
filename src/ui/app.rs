use eframe::egui;

use crate::dsp::spec::FilterSpec;
use crate::io::log_loader::LogFile;
use crate::pipeline::FilterGraph;
use crate::ui::export::ExportWindow;
use crate::ui::filter_panel::FilterPanel;
use crate::ui::live_panel::LivePanel;
use crate::ui::node_graph::{self, NodeGraphView};
use crate::ui::node_inspector::NodeInspector;
use crate::ui::signal_view::SignalView;

#[derive(Default, PartialEq, Eq)]
enum ViewMode {
    #[default]
    Signal,
    Graph,
    Live,
}

pub struct WpiFilterApp {
    log:           Option<LogFile>,
    view_mode:     ViewMode,

    signal_view:   SignalView,
    filter:        FilterSpec,
    export_window: ExportWindow,

    graph:         FilterGraph,
    graph_view:    NodeGraphView,

    live_panel:    LivePanel,
    /// One-frame snapshot of live data, rebuilt at the start of each Live
    /// frame and handed to signal_view as a borrowed `LogFile`.
    live_snapshot: Option<LogFile>,
}

impl WpiFilterApp {
    pub fn new(log: Option<LogFile>) -> Self {
        Self {
            log,
            view_mode:     ViewMode::Signal,
            signal_view:   SignalView::default(),
            filter:        FilterSpec::default(),
            export_window: ExportWindow::default(),
            graph:         FilterGraph::new(),
            graph_view:    NodeGraphView::default(),
            live_panel:    LivePanel::new(),
            live_snapshot: None,
        }
    }
}

impl Default for WpiFilterApp {
    fn default() -> Self { Self::new(None) }
}

impl eframe::App for WpiFilterApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Live mode needs continuous repaints; the network thread doesn't
        // wake egui on its own. Request_repaint at ~60Hz keeps the plot
        // scrolling smoothly.
        if matches!(self.view_mode, ViewMode::Live) {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }

        egui::TopBottomPanel::top("topbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("WPIFilter");
                ui.separator();
                ui.selectable_value(&mut self.view_mode, ViewMode::Signal, "Signal view");
                ui.selectable_value(&mut self.view_mode, ViewMode::Graph,  "Graph view");
                ui.selectable_value(&mut self.view_mode, ViewMode::Live,   "Live (NT4)");
                ui.separator();
                match self.view_mode {
                    ViewMode::Signal | ViewMode::Graph => {
                        match &self.log {
                            Some(log) => ui.label(format!(
                                "{} channels  /  {} entries",
                                log.channels.len(),
                                log.data.values().map(|v| v.len()).sum::<usize>(),
                            )),
                            None => ui.label("no log loaded"),
                        };
                    }
                    ViewMode::Live => {
                        if let Some(c) = &self.live_panel.client {
                            let (t, s) = c.store.stats();
                            ui.label(format!("{t} topics live / {s} samples buffered"));
                        } else {
                            ui.label("not connected");
                        }
                    }
                }
            });
        });

        match self.view_mode {
            ViewMode::Signal => self.signal_mode_ui(ctx),
            ViewMode::Graph  => self.graph_mode_ui(ctx),
            ViewMode::Live   => self.live_mode_ui(ctx),
        }
    }
}

impl WpiFilterApp {
    fn signal_mode_ui(&mut self, ctx: &egui::Context) {
        let fs = self.log.as_ref().and_then(|l| self.signal_view.estimated_sample_rate(l));

        egui::SidePanel::right("filter_panel")
            .default_width(280.0)
            .resizable(true)
            .show(ctx, |ui| {
                if FilterPanel::ui(ui, &mut self.filter, fs) {
                    self.export_window.open = true;
                }
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(log) = &self.log {
                self.signal_view.ui(ui, log, &mut self.filter);
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label("Pass a .wpilog path on the command line to get started.");
                });
            }
        });

        let preview_fs = fs.unwrap_or(1000.0);
        let cascade = if self.filter.enabled {
            self.filter.compile(preview_fs)
        } else {
            None
        };
        self.export_window.ui(ctx, cascade.as_ref());
    }

    fn graph_mode_ui(&mut self, ctx: &egui::Context) {
        egui::SidePanel::right("graph_inspector")
            .default_width(280.0)
            .resizable(true)
            .show(ctx, |ui| {
                NodeInspector::ui(
                    ui,
                    &mut self.graph,
                    self.graph_view.selected,
                    self.log.as_ref(),
                );
            });

        egui::TopBottomPanel::bottom("graph_output")
            .resizable(true)
            .default_height(240.0)
            .show(ctx, |ui| {
                ui.heading("Pipeline output");
                if let Some(log) = &self.log {
                    node_graph::plot_graph_outputs(ui, &self.graph, log);
                } else {
                    ui.label("Load a log to see pipeline output.");
                }
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.graph_view.ui(ui, &mut self.graph);
        });
    }

    fn live_mode_ui(&mut self, ctx: &egui::Context) {
        // Drain network thread status updates.
        self.live_panel.poll();

        // Build the one-frame snapshot. This is the magic line — everything
        // downstream gets a normal LogFile and doesn't know it came from
        // a live socket.
        self.live_snapshot = self
            .live_panel
            .client
            .as_ref()
            .map(|c| c.store.snapshot_to_log());

        // Right side: live connection panel + filter panel stacked.
        egui::SidePanel::right("live_right")
            .default_width(300.0)
            .resizable(true)
            .show(ctx, |ui| {
                self.live_panel.ui(ui);
                ui.separator();
                let fs = self.live_snapshot
                    .as_ref()
                    .and_then(|l| self.signal_view.estimated_sample_rate(l));
                let _ = FilterPanel::ui(ui, &mut self.filter, fs);
            });

        // Center: reuse signal_view on the snapshot. The view doesn't know
        // or care that the LogFile was built from a ring buffer 0.3 ms ago.
        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(snap) = &self.live_snapshot {
                if snap.channels.is_empty() {
                    ui.centered_and_justified(|ui| {
                        ui.label("Connect to a robot to see live data.");
                    });
                } else {
                    self.signal_view.ui(ui, snap, &mut self.filter);
                }
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label("Connect to a robot in the right panel.");
                });
            }
        });
    }
}