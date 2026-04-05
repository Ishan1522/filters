mod analysis;
mod dsp;
mod io;
mod pipeline;
mod ui;

fn main() {
    let log = io::log_loader::load("pykit_20260219_093818.wpilog").expect("failed to parse");
    for ch in &log.channels {
        println!("{} ({})", ch.name, ch.data_type);
    }
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "wpifilter",
        options,
        Box::new(|_cc| Box::new(ui::app::WpiFilterApp::default())),
    ).unwrap();
}