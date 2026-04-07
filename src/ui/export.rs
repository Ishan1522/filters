use eframe::egui;

use crate::dsp::biquad::BiquadCascade;
use crate::dsp::export::export_cascade;

#[derive(Default, PartialEq, Eq)]
enum ExportTab {
    #[default]
    Java,
    Cpp,
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
            tab:  ExportTab::Java,
            name: "myFilter".to_string(),
        }
    }
}

impl ExportWindow {
    pub fn ui(&mut self, ctx: &egui::Context, cascade: Option<&BiquadCascade>) {
        if !self.open {
            return;
        }
        let mut still_open = self.open;
        egui::Window::new("Export filter")
            .open(&mut still_open)
            .default_width(620.0)
            .show(ctx, |ui| {
                let Some(cascade) = cascade else {
                    ui.label("Enable a filter to generate export code.");
                    return;
                };

                ui.horizontal(|ui| {
                    ui.label("Variable name:");
                    ui.text_edit_singleline(&mut self.name);
                });

                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.tab, ExportTab::Java, "Java");
                    ui.selectable_value(&mut self.tab, ExportTab::Cpp, "C++");
                    ui.selectable_value(&mut self.tab, ExportTab::Coefficients, "Coefficients");
                });
                ui.separator();

                // Regenerate every frame — cheap, and means slider drags
                // update the displayed code instantly.
                let exported = export_cascade(cascade, &self.name);
                let body = match self.tab {
                    ExportTab::Java         => &exported.java,
                    ExportTab::Cpp          => &exported.cpp,
                    ExportTab::Coefficients => &exported.coefficients,
                };

                egui::ScrollArea::vertical().max_height(380.0).show(ui, |ui| {
                    ui.add(
                        egui::Label::new(egui::RichText::new(body).monospace())
                            .selectable(true)
                            .wrap_mode(egui::TextWrapMode::Extend),
                    );
                });

                ui.separator();
                if ui.button("📋 Copy to clipboard").clicked() {
                    ctx.copy_text(body.clone());
                }
            });
        self.open = still_open;
    }
}