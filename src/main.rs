mod analysis;
mod dsp;
mod io;
mod pipeline;
mod ui;

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "pykit_20260219_093818.wpilog".to_string());

    let log = io::log_loader::load(&path);
    if let Some(ref l) = log {
        println!("loaded {}: {} channels", path, l.channels.len());
        for ch in &l.channels {
            println!("  {} ({})", ch.name, ch.data_type);
        }
    } else {
        eprintln!("warning: failed to load log file '{}', starting with empty UI", path);
    }

    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "wpifilter",
        options,
        Box::new(|_cc| Ok(Box::new(ui::app::WpiFilterApp::new(log)))),
    )
    .unwrap();
}