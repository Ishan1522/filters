mod analysis;
mod dsp;
mod headless;
mod io;
mod pipeline;
mod ui;

use std::path::PathBuf;

fn main() {
    let args: Vec<std::ffi::OsString> = std::env::args_os().skip(1).collect();

    // Headless / batch subcommand — the "executor" half of the two-sided
    // filter system (see `headless.rs`). Exits without touching the GUI.
    if args.first().is_some_and(|a| a.to_str() == Some("headless")) {
        std::process::exit(headless::run(&args[1..]));
    }

    // Root-level flags. GUI positional args (a bag path) are handled below.
    if let Some(flag) = args.first().and_then(|a| a.to_str()) {
        match flag {
            "--version" | "-V" => {
                println!("rosfilter {}", env!("CARGO_PKG_VERSION"));
                std::process::exit(0);
            }
            "--help" | "-h" => {
                print_root_help();
                std::process::exit(0);
            }
            _ if flag.starts_with('-') => {
                eprintln!("error: unknown flag `{flag}`");
                eprintln!();
                print_root_help();
                std::process::exit(2);
            }
            _ => {}
        }
    }

    // Optional positional arg: a rosbag2 (MCAP) file, e.g.
    //   rosfilter sim_recording.mcap
    let path: Option<PathBuf> = args.first().cloned().map(PathBuf::from);

    let log = path.as_deref().and_then(load_bag);

    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "rosfilter — ROS 2 filter workbench",
        options,
        Box::new(|_cc| Ok(Box::new(ui::app::RosFilterApp::new(log)))),
    )
    .unwrap();
}

fn print_root_help() {
    println!(
        "\
rosfilter {} — ROS 2 filter workbench

USAGE:
    rosfilter [bag.mcap]       open the GUI (optionally loading a rosbag2/MCAP recording)
    rosfilter headless ...     run a filter over a recorded signal, no GUI (batch/executor)
    rosfilter --version        print the version
    rosfilter --help           show this help

The GUI loads recordings (MCAP), connects to live ROS 2 topics (`--features ros2`),
designs filters, and exports them as ROS 2 nodes. See the README.

`rosfilter headless --help` documents the batch CLI (filter-spec JSON + signal in,
filtered JSONL out, same real DSP the GUI runs).",
        env!("CARGO_PKG_VERSION"),
    );
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
