use eframe::egui;

#[cfg(feature = "ros2")]
use crate::io::ros2_live::{LiveStatus, LiveTopicConfig, Ros2LiveClient};

/// Live ROS 2 topic panel. Lets the user pick topics (auto-discovered via
/// DDS, with an optional field path for composite messages), connect via
/// rclrs, and watch the ring-buffer snapshot in the same Signal view used
/// for offline bags.
///
/// With the `ros2` cargo feature disabled the panel renders in a read-only
/// "feature not enabled" state and the connect controls are hidden.
pub struct LivePanel {
    /// `#[cfg]`'d: the rclrs live client (feature `ros2` only).
    #[cfg(feature = "ros2")]
    pub client: Option<Ros2LiveClient>,
    #[cfg(feature = "ros2")]
    pub status: Option<LiveStatus>,
    #[cfg(feature = "ros2")]
    configs: Vec<LiveTopicConfig>,

    // Present only in the full (`ros2`) implementation.
    #[cfg(feature = "ros2")]
    domain_id: String,
    #[cfg(feature = "ros2")]
    topic_name: String,
    #[cfg(feature = "ros2")]
    msg_type: String,
    #[cfg(feature = "ros2")]
    field_path: String,
    #[cfg(feature = "ros2")]
    discovered: Option<Result<Vec<(String, String)>, String>>,
}

impl Default for LivePanel {
    fn default() -> Self {
        Self::new()
    }
}

/// Full implementation (with the `ros2` feature).
#[cfg(feature = "ros2")]
impl LivePanel {
    pub fn new() -> Self {
        Self {
            client: None,
            status: None,
            configs: Vec::new(),
            domain_id: "0".to_string(),
            topic_name: "/filtered/vel".to_string(),
            msg_type: "std_msgs/msg/Float64".to_string(),
            field_path: String::new(),
            discovered: None,
        }
    }

    fn domain_id(&self) -> Option<usize> {
        self.domain_id.trim().parse().ok()
    }

    /// Drain any pending status updates from the live thread. Called once
    /// per frame from the app.
    pub fn poll(&mut self) {
        if let Some(client) = &self.client {
            while let Ok(s) = client.status_rx.try_recv() {
                // A terminal state means the thread exited; drop the client
                // so the UI shows the disconnected state cleanly.
                let terminal = matches!(s, LiveStatus::Disconnected);
                self.status = Some(s);
                if terminal {
                    self.client = None;
                }
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("ROS topics (live)");
        ui.separator();

        // ── domain + discovery ──
        ui.horizontal(|ui| {
            ui.label("ROS_DOMAIN_ID:");
            ui.text_edit_singleline(&mut self.domain_id);
            if ui.button("Discover topics").clicked() {
                self.discovered = Some(Ros2LiveClient::discover_topics(self.domain_id()));
            }
        });
        ui.small("DDS discovery finds publishers on this domain automatically — no host/IP needed.");

        if let Some(discovered) = &self.discovered {
            match discovered {
                Ok(topics) if topics.is_empty() => {
                    ui.small("No topics discovered. Is something publishing on this domain?");
                }
                Ok(topics) => {
                    ui.separator();
                    ui.label("Discovered (click to add):");
                    egui::ScrollArea::vertical().max_height(120.0).show(ui, |ui| {
                        for (topic, type_str) in topics {
                            if ui
                                .selectable_label(false, format!("{topic}  ({type_str})"))
                                .clicked()
                            {
                                self.topic_name = topic.clone();
                                self.msg_type = type_str.clone();
                            }
                        }
                    });
                }
                Err(e) => {
                    ui.colored_label(egui::Color32::LIGHT_RED, format!("✗ {e}"));
                }
            }
        }
        ui.separator();

        // ── add-subscription form ──
        ui.label("Add subscription:");
        ui.horizontal(|ui| {
            ui.label("Topic:");
            ui.text_edit_singleline(&mut self.topic_name);
        });
        ui.horizontal(|ui| {
            ui.label("Type:");
            ui.text_edit_singleline(&mut self.msg_type);
        });
        ui.horizontal(|ui| {
            ui.label("Field:");
            ui.text_edit_singleline(&mut self.field_path);
            if ui.button("＋ Add").clicked() && !self.topic_name.trim().is_empty() {
                self.configs.push(LiveTopicConfig {
                    topic: self.topic_name.trim().to_string(),
                    msg_type: self.msg_type.trim().to_string(),
                    path: self.field_path.trim().to_string(),
                });
            }
        });
        ui.small(
            "Field path: empty for scalar messages (Float64/Bool/…), else dotted \
             — e.g. linear_acceleration.z, or data[3] for multi-arrays.",
        );

        // ── configured subscriptions ──
        if !self.configs.is_empty() {
            ui.separator();
            ui.label("Subscriptions:");
            let mut remove: Option<usize> = None;
            for (i, cfg) in self.configs.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(cfg.display_name());
                    ui.small(&cfg.msg_type);
                    if ui.button("✕").clicked() {
                        remove = Some(i);
                    }
                });
            }
            if let Some(i) = remove {
                self.configs.remove(i);
            }
        }

        ui.separator();

        // ── connect / disconnect ──
        let connected = matches!(self.status, Some(LiveStatus::Connected));
        let connecting = matches!(self.status, Some(LiveStatus::Initializing));
        ui.horizontal(|ui| {
            if !connected && !connecting {
                if ui.button("Connect").clicked() {
                    self.client = Some(Ros2LiveClient::connect(
                        self.domain_id(),
                        self.configs.clone(),
                    ));
                    self.status = Some(LiveStatus::Initializing);
                }
            } else {
                if ui.button("Disconnect").clicked() {
                    if let Some(c) = &self.client {
                        c.disconnect();
                    }
                }
            }
        });

        match &self.status {
            Some(LiveStatus::Initializing) => ui.label("⏳ initializing rclrs…"),
            Some(LiveStatus::Connected) => {
                ui.colored_label(egui::Color32::LIGHT_GREEN, "● connected")
            }
            Some(LiveStatus::Disconnected) => ui.label("○ disconnected"),
            Some(LiveStatus::Error(e)) => {
                ui.colored_label(egui::Color32::LIGHT_RED, format!("✗ {e}"))
            }
            None => ui.label("not started"),
        }

        if let Some(client) = &self.client {
            let (n_topics, n_samples) = client.store.stats();
            ui.separator();
            ui.label(format!("{n_topics} topics  /  {n_samples} samples buffered"));
        }
    }
}

/// Stub implementation (without the `ros2` feature): read-only panel that
/// points the user at the feature flag.
#[cfg(not(feature = "ros2"))]
impl LivePanel {
    pub fn new() -> Self {
        Self {}
    }

    pub fn poll(&mut self) {
        // Nothing to drain without a live client.
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("ROS topics (live)");
        ui.separator();
        ui.colored_label(
            egui::Color32::LIGHT_RED,
            "Live ROS 2 topics are not compiled in.",
        );
        ui.small(
            "Build with the `ros2` cargo feature and a sourced ROS 2 distro to \
             enable this panel:\n\n    cargo build --features ros2\n\n\
             (see README for setup.)",
        );
        ui.separator();
        ui.label("This build still supports offline rosbag analysis.");
    }
}
