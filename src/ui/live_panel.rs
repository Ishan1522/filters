use eframe::egui;

use crate::io::nt4_client::{NtClient, NtStatus};

#[derive(Default)]
pub struct LivePanel {
    pub host:    String,
    pub status:  Option<NtStatus>,
    pub client:  Option<NtClient>,
}

impl LivePanel {
    pub fn new() -> Self {
        Self {
            host:   "127.0.0.1".to_string(),
            status: None,
            client: None,
        }
    }

    /// Drain any pending status updates from the network thread. Called
    /// once per frame from the app.
    pub fn poll(&mut self) {
        if let Some(client) = &self.client {
            while let Ok(s) = client.status_rx.try_recv() {
                self.status = Some(s);
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("Live (NT4)");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Robot:");
            ui.text_edit_singleline(&mut self.host);
        });
        ui.small("e.g. 127.0.0.1, 10.TE.AM.2, or roborio-TEAM-frc.local");

        ui.separator();

        let connected = matches!(self.status, Some(NtStatus::Connected));
        let connecting = matches!(self.status, Some(NtStatus::Connecting));

        ui.horizontal(|ui| {
            if !connected && !connecting {
                if ui.button("Connect").clicked() {
                    self.client = Some(NtClient::connect(&self.host));
                    self.status = Some(NtStatus::Connecting);
                }
            } else {
                if ui.button("Disconnect").clicked() {
                    if let Some(c) = &self.client {
                        c.disconnect();
                    }
                    self.client = None;
                    self.status = Some(NtStatus::Disconnected);
                }
            }
        });

        ui.separator();
        match &self.status {
            Some(NtStatus::Connecting)   => { ui.label("⏳ connecting…"); }
            Some(NtStatus::Connected)    => { ui.colored_label(egui::Color32::LIGHT_GREEN, "● connected"); }
            Some(NtStatus::Disconnected) => { ui.label("○ disconnected"); }
            Some(NtStatus::Error(e))     => { ui.colored_label(egui::Color32::LIGHT_RED, format!("✗ {e}")); }
            None                         => { ui.label("not started"); }
        }

        if let Some(client) = &self.client {
            let (n_topics, n_samples) = client.store.stats();
            ui.separator();
            ui.label(format!("{n_topics} topics  /  {n_samples} samples buffered"));
        }
    }
}