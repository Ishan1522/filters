pub struct WpiFilterApp;

impl Default for WpiFilterApp {
    fn default() -> Self { Self }
}

impl eframe::App for WpiFilterApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("wpifilter");
            ui.label("hello from egui 🦀");
        });
    }
}