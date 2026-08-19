use eframe::egui;

use crate::dsp::spec::FilterSpec;
use crate::io::model::LogFile;
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

pub struct RosFilterApp {
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

impl RosFilterApp {
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

impl Default for RosFilterApp {
    fn default() -> Self { Self::new(None) }
}

impl eframe::App for RosFilterApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Live mode needs continuous repaints; the rclrs thread doesn't wake
        // egui on its own. Request_repaint at ~60Hz keeps the plots scrolling.
        if matches!(self.view_mode, ViewMode::Live) {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }

        egui::TopBottomPanel::top("topbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("rosfilter — ROS 2 filter workbench");
                ui.separator();
                ui.selectable_value(&mut self.view_mode, ViewMode::Signal, "Signal view");
                ui.selectable_value(&mut self.view_mode, ViewMode::Graph,  "Graph view");
                ui.selectable_value(&mut self.view_mode, ViewMode::Live,   "Live (ROS 2)");
                ui.separator();
                match self.view_mode {
                    ViewMode::Signal | ViewMode::Graph => {
                        match &self.log {
                            Some(log) => ui.label(format!(
                                "{} channels  /  {} samples",
                                log.channels.len(),
                                log.sample_count(),
                            )),
                            None => ui.label("no bag loaded"),
                        };
                    }
                    ViewMode::Live => {
                        #[cfg(feature = "ros2")]
                        {
                            if let Some(c) = &self.live_panel.client {
                                let (t, s) = c.store.stats();
                                ui.label(format!("{t} topics live / {s} samples buffered"));
                            } else {
                                ui.label("not connected");
                            }
                        }
                        #[cfg(not(feature = "ros2"))]
                        {
                            ui.label("live disabled (build with --features ros2)");
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

impl RosFilterApp {
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
                    ui.label("Pass a rosbag (.mcap) path on the command line to get started.");
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
                    ui.label("Load a rosbag to see pipeline output.");
                }
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.graph_view.ui(ui, &mut self.graph);
        });
    }

    fn live_mode_ui(&mut self, ctx: &egui::Context) {
        // Drain live thread status updates.
        self.live_panel.poll();

        // Build the one-frame snapshot. This is the magic line — everything
        // downstream gets a normal LogFile and doesn't know it came from a
        // live DDS subscription.
        #[cfg(feature = "ros2")]
        {
            self.live_snapshot = self
                .live_panel
                .client
                .as_ref()
                .map(|c| c.store.snapshot_to_log());
        }
        #[cfg(not(feature = "ros2"))]
        {
            self.live_snapshot = None;
        }

        // Right side: live connection panel + filter panel stacked.
        egui::SidePanel::right("live_right")
            .default_width(320.0)
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
        // or care that the LogFile was built from a ring buffer moments ago.
        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(snap) = &self.live_snapshot {
                if snap.channels.is_empty() {
                    ui.centered_and_justified(|ui| {
                        ui.label("Connect to ROS 2 topics to see live data.");
                    });
                } else {
                    self.signal_view.ui(ui, snap, &mut self.filter);
                }
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label("Connect to ROS 2 topics in the right panel.");
                });
            }
        });
    }
}
