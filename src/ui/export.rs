use eframe::egui;

use crate::dsp::biquad::BiquadCascade;
use crate::dsp::export::export_cascade;

#[derive(Default, PartialEq, Eq)]
enum ExportTab {
    #[default]
    Yaml,
    Rclcpp,
    Rclrs,
    Coefficients,
}

pub struct ExportWindow {
    pub open: bool,
    tab:      ExportTab,
    name:     String,
}

impl Default for ExportWindow {
    fn default() -> Self {
        Self {
            open: false,
            tab:  ExportTab::Yaml,
            name: "velocityFilter".to_string(),
        }
    }
}

impl ExportWindow {
    pub fn ui(&mut self, ctx: &egui::Context, cascade: Option<&BiquadCascade>) {
        if !self.open {
            return;
        }
        let mut still_open = self.open;
        egui::Window::new("Export filter — ROS 2")
            .open(&mut still_open)
            .default_width(680.0)
            .show(ctx, |ui| {
                let Some(cascade) = cascade else {
                    ui.label("Enable a filter to generate export code.");
                    return;
                };

                ui.horizontal(|ui| {
                    ui.label("Filter name:");
                    ui.text_edit_singleline(&mut self.name);
                });
                ui.small(
                    "Exports are causal single-pass — zero-phase (filtfilt) is offline-only.",
                );

                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.tab, ExportTab::Yaml, "filter_chain.yaml");
                    ui.selectable_value(&mut self.tab, ExportTab::Rclcpp, "rclcpp node (C++)");
                    ui.selectable_value(&mut self.tab, ExportTab::Rclrs, "rclrs node (Rust)");
                    ui.selectable_value(&mut self.tab, ExportTab::Coefficients, "Coefficients");
                });
                ui.separator();

                // Regenerate every frame — cheap, and means slider drags
                // update the displayed code instantly.
                let exported = export_cascade(cascade, &self.name);
                let body = match self.tab {
                    ExportTab::Yaml         => &exported.yaml,
                    ExportTab::Rclcpp       => &exported.rclcpp,
                    ExportTab::Rclrs        => &exported.rclrs,
                    ExportTab::Coefficients => &exported.coefficients,
                };

                egui::ScrollArea::vertical().max_height(420.0).show(ui, |ui| {
                    ui.add(
                        egui::Label::new(egui::RichText::new(body).monospace())
                            .selectable(true)
                            .wrap_mode(egui::TextWrapMode::Extend),
                    );
                });

                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("📋 Copy to clipboard").clicked() {
                        ctx.copy_text(body.clone());
                    }
                    ui.small("rclcpp/rclrs nodes subscribe to `input` and publish to `output` — remap at launch.");
                });
            });
        self.open = still_open;
    }
}
