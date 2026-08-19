mod analysis;
mod dsp;
mod io;
mod pipeline;
mod ui;

use std::path::PathBuf;

fn main() {
    // Optional positional arg: a rosbag2 (MCAP) file, e.g.
    //   wpifilter sim_recording.mcap
    let path: Option<PathBuf> = std::env::args_os().nth(1).map(PathBuf::from);

    let log = path.as_deref().and_then(load_bag);

    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "wpifilter — ROS 2 filter workbench",
        options,
        Box::new(|_cc| Ok(Box::new(ui::app::WpiFilterApp::new(log)))),
    )
    .unwrap();
}

fn load_bag(path: &std::path::Path) -> Option<io::model::LogFile> {
    match io::rosbag_loader::load(path) {
        Ok((log, stats)) => {
            println!(
                "loaded {}: {} channels, {} messages ({} decoded, {} unsupported type)",
                path.display(),
                log.channels.len(),
                stats.messages_total,
                stats.messages_decoded,
                stats.messages_unsupported,
            );
            for ch in &log.channels {
                println!("  {} ({})", ch.name, ch.data_type);
            }
            for (topic, ty) in &stats.unsupported_topics {
                println!("  (skipped unsupported topic {topic} of type {ty})");
            }
            Some(log)
        }
        Err(e) => {
            eprintln!("warning: failed to load bag '{}': {e}", path.display());
            eprintln!("         starting with an empty UI");
            None
        }
    }
}
