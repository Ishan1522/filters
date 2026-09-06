//! Headless / batch CLI mode — run a real biquad filter over a recorded
//! signal with no GUI.
//!
//! Invoked as a subcommand from `main`:
//!
//! ```text
//! rosfilter headless --spec filter.json --input signal.jsonl --output out.jsonl
//! ```
//!
//! This is the "executor" half of the two-sided filter system: a browser
//! dashboard designs filters (scipy preview) and shells out to this binary to
//! run the *real* Rust biquad cascade on a signal and read back the
//! authoritative filtered output.
//!
//! # Reused runtime path (no second DSP implementation)
//!
//! The signal is resampled onto its natural-rate uniform grid with the same
//! helpers the node-graph evaluator uses — [`crate::analysis::estimate_sample_rate_hz`]
//! and [`crate::analysis::resample_uniform`], mirroring
//! `pipeline::graph::EvalContext::load_topic_channel` — then filtered through
//! [`crate::pipeline::nodes::apply_filter`], the exact function a `Filter`
//! node in the node graph dispatches to. That compiles the [`FilterSpec`]
//! and runs the causal cascade (`process_block`) or offline zero-phase
//! filtfilt (`process_block_filtfilt`). Headless output is therefore exactly
//! what the app would produce, and the causal default matches the exported
//! ROS 2 node's single-pass behavior.

use std::collections::HashMap;
use std::ffi::OsString;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::analysis;
use crate::dsp::spec::{FilterKind, FilterSpec};
use crate::io::convert::sample_to_f64;
use crate::io::model::{Channel, LogFile};
use crate::pipeline::node::Signal;
use crate::pipeline::nodes;

/// First 8 bytes of every MCAP file (same constant as `rosbag_loader`).
const MCAP_MAGIC: [u8; 8] = [0x89, b'M', b'C', b'A', b'P', b'0', b'\r', b'\n'];

// ─────────────────────────────────────────────────────────────────────────────
// CLI surface
// ─────────────────────────────────────────────────────────────────────────────

const USAGE: &str = "\
Run one filter over a recorded signal from the command line (no GUI).

USAGE:
    rosfilter headless [OPTIONS] --spec <SPEC_JSON> --input <SIGNAL> --output <OUT_JSONL>

OPTIONS:
    --spec <FILE>        Filter spec JSON — the schema the dashboard exports:
                         {name?, kind, cutoff_hz, q|order|ripple_db (null when
                         inapplicable), zero_phase}. `kind` is one of:
                         CookbookLowpass, CookbookHighpass, CookbookBandpass,
                         CookbookNotch, ButterworthLowpass, Chebyshev1Lowpass.
                         Unknown kinds / unknown fields are refused loudly.
    --input <FILE>       Signal source:
                           • trial JSONL — rows {\"t\": <s>, \"metric\":
                             \"joint_torques\", \"joint\": <name|idx>,
                             \"torque\": <Nm>}; pick a joint with --joint.
                           • bare JSONL — rows {\"t\": <s>, \"value\": <x>}.
                           • rosbag2 / MCAP recording (default build) — pick
                             a channel with --topic.
    --output <FILE>      Filtered samples as JSONL: {\"t\": <s>, \"value\": <y>}
                         one row per uniform-grid sample.
    --joint <NAME|IDX>   For trial JSONL: which joint's torque to filter
                         (name, or numeric index as decimal). Defaults to the
                         single joint when the file has exactly one.
    --topic <CHANNEL>    For MCAP input: the channel to filter — either a
                         scalar topic (e.g. /ci/vel) or a full field channel
                         as listed by --list-channels (e.g.
                         \"/imu/data · linear_acceleration.z\"). Alias: --channel.
    --zero-phase         Zero-phase filtfilt (offline-only) instead of the
                         default causal single pass — overrides the spec's
                         zero_phase field; without the flag the spec field is
                         honored (default false = causal).
    --list-channels      With an MCAP input: print the loadable channels and
                         exit (no spec/output needed).
    -h, --help           Show this help.

EXIT CODES:
    0   success
    1   runtime error (bad spec, bad/empty input, channel not found, …)
    2   usage error (missing/unknown arguments)

The filter is real DSP: the input is resampled to its natural-rate uniform
grid and run through the same biquad cascade the GUI/node graph uses, so the
output is exactly what the app would produce.";

struct HeadlessArgs {
    spec: Option<PathBuf>,
    input: PathBuf,
    output: Option<PathBuf>,
    /// `--joint` selector for trial JSONL.
    joint: Option<String>,
    /// `--topic` / `--channel` selector for MCAP input.
    channel: Option<String>,
    /// `Some(true)` if `--zero-phase` was passed (overrides the spec field).
    zero_phase: Option<bool>,
    list_channels: bool,
}

enum Options {
    Help,
    Run(HeadlessArgs),
}

/// Entry point for `rosfilter headless ...` — returns the process exit code.
pub fn run(args: &[OsString]) -> i32 {
    let argv: Vec<&str> = match args.iter().map(|a| a.to_str()).collect() {
        Some(v) => v,
        None => {
            eprintln!("error: arguments must be valid UTF-8");
            return 2;
        }
    };

    let opts = match parse_args(&argv) {
        Ok(Options::Help) => {
            println!("{USAGE}");
            return 0;
        }
        Ok(Options::Run(o)) => o,
        Err(msg) => {
            eprintln!("error: {msg}\n");
            eprintln!("{USAGE}");
            return 2;
        }
    };

    match execute(&opts) {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("error: {e}");
            1
        }
    }
}

fn parse_args(argv: &[&str]) -> Result<Options, String> {
    let mut spec = None;
    let mut input = None;
    let mut output = None;
    let mut joint = None;
    let mut channel = None;
    let mut zero_phase = false;
    let mut list_channels = false;

    let mut i = 0;
    while i < argv.len() {
        let flag = argv[i];
        let value = |i: &mut usize| -> Result<String, String> {
            *i += 1;
            argv.get(*i)
                .map(|s| s.to_string())
                .ok_or_else(|| format!("flag `{flag}` needs a value"))
        };
        match flag {
            "-h" | "--help" => return Ok(Options::Help),
            "--spec" => spec = Some(PathBuf::from(value(&mut i)?)),
            "--input" => input = Some(PathBuf::from(value(&mut i)?)),
            "--output" => output = Some(PathBuf::from(value(&mut i)?)),
            "--joint" => joint = Some(value(&mut i)?),
            "--topic" | "--channel" => channel = Some(value(&mut i)?),
            "--zero-phase" => zero_phase = true,
            "--list-channels" => list_channels = true,
            other => {
                return Err(format!(
                    "unknown argument `{other}` (headless subcommand; `-h` for usage)"
                ));
            }
        }
        i += 1;
    }

    let input = input.ok_or_else(|| "missing required flag `--input <SIGNAL>`".to_string())?;
    if !list_channels {
        if spec.is_none() {
            return Err("missing required flag `--spec <SPEC_JSON>`".to_string());
        }
        if output.is_none() {
            return Err("missing required flag `--output <OUT_JSONL>`".to_string());
        }
    }

    Ok(Options::Run(HeadlessArgs {
        spec,
        input,
        output,
        joint,
        channel,
        zero_phase: zero_phase.then_some(true),
        list_channels,
    }))
}

// ─────────────────────────────────────────────────────────────────────────────
// Execution
// ─────────────────────────────────────────────────────────────────────────────

fn execute(o: &HeadlessArgs) -> Result<(), String> {
    if o.list_channels {
        if !looks_like_mcap(&o.input) {
            return Err("--list-channels requires an MCAP (rosbag2) input".to_string());
        }
        return list_mcap_channels(&o.input);
    }

    let spec_path = o.spec.as_deref().expect("spec required by parse_args");
    let (mut spec, spec_name) = load_spec(spec_path)?;
    if let Some(true) = o.zero_phase {
        spec.zero_phase = true;
    }

    if looks_like_mcap(&o.input) {
        run_mcap(o, &spec, spec_name.as_deref())
    } else {
        run_trial_jsonl(o, &spec, spec_name.as_deref())
    }
}

/// Print the loadable channels of an MCAP recording and exit (like the GUI's
/// console listing). No spec/output needed.
fn list_mcap_channels(input: &Path) -> Result<(), String> {
    #[cfg(not(feature = "rosbag"))]
    {
        let _ = input;
        return Err(
            "this binary was built without the `rosbag` feature, so MCAP input is unavailable; \
             rebuild with default features, or feed trial JSONL"
                .to_string(),
        );
    }

    #[cfg(feature = "rosbag")]
    {
        let (log, stats) = crate::io::rosbag_loader::load(input)
            .map_err(|e| format!("failed to load bag '{}': {e}", input.display()))?;
        for ch in &log.channels {
            let meta = if ch.metadata.is_empty() {
                String::new()
            } else {
                format!("  ({})", ch.metadata)
            };
            println!("{}\t{}{}", ch.entry_id, ch.name, meta);
        }
        eprintln!(
            "headless: {}: {} channels, {} messages ({} decoded, {} unsupported)",
            input.display(),
            log.channels.len(),
            stats.messages_total,
            stats.messages_decoded,
            stats.messages_unsupported,
        );
        Ok(())
    }
}

/// Run the real filter path over one channel of a rosbag2/MCAP recording.
fn run_mcap(o: &HeadlessArgs, spec: &FilterSpec, spec_name: Option<&str>) -> Result<(), String> {
    #[cfg(not(feature = "rosbag"))]
    {
        let _ = (o, spec, spec_name);
        return Err(
            "this binary was built without the `rosbag` feature, so MCAP input is unavailable; \
             rebuild with default features, or feed trial JSONL"
                .to_string(),
        );
    }

    #[cfg(feature = "rosbag")]
    {
        let (log, stats) = crate::io::rosbag_loader::load(&o.input)
            .map_err(|e| format!("failed to load bag '{}': {e}", o.input.display()))?;
        eprintln!(
            "headless: loaded {}: {} channels, {} messages ({} decoded, {} unsupported)",
            o.input.display(),
            log.channels.len(),
            stats.messages_total,
            stats.messages_decoded,
            stats.messages_unsupported,
        );

        let selector = o.channel.as_deref().ok_or_else(|| {
            "MCAP input needs a channel selector: pass `--topic <channel>` \
             (see --list-channels for the available labels)"
                .to_string()
        })?;
        let channel = select_mcap_channel(&log, selector, &o.input.display().to_string())?;
        let (timestamps_us, values) = extract_channel(&log, channel)?;
        filter_and_write(o, spec, spec_name, &channel.name, &timestamps_us, &values)
    }
}

/// Run the real filter path over a dashboard trial JSONL (joint torques) or a
/// bare `{"t", "value"}` series.
fn run_trial_jsonl(
    o: &HeadlessArgs,
    spec: &FilterSpec,
    spec_name: Option<&str>,
) -> Result<(), String> {
    let (timestamps_us, values, label) = parse_input_jsonl(&o.input, o.joint.as_deref())?;
    filter_and_write(o, spec, spec_name, &label, &timestamps_us, &values)
}

/// Shared tail: resample to the natural uniform grid, run the real cascade,
/// write the filtered JSONL.
fn filter_and_write(
    o: &HeadlessArgs,
    spec: &FilterSpec,
    spec_name: Option<&str>,
    source_label: &str,
    timestamps_us: &[u64],
    values: &[f64],
) -> Result<(), String> {
    if timestamps_us.len() < 2 {
        return Err(format!(
            "'{source_label}': need at least 2 samples to estimate a sample rate, got {}",
            timestamps_us.len()
        ));
    }
    let fs = analysis::estimate_sample_rate_hz(timestamps_us).ok_or_else(|| {
        format!("'{source_label}': could not estimate a sample rate (timestamps not increasing?)")
    })?;

    // Same uniform-grid extraction the node graph uses (load_topic_channel).
    let (times_s, uniform) = analysis::resample_uniform(timestamps_us, values, fs);
    if uniform.is_empty() {
        return Err(format!(
            "'{source_label}': resampling produced no samples (input spans no time?)"
        ));
    }

    // Mirror the GUI: compile() clamps cutoff a hair below Nyquist. Surface
    // that so an executor doesn't silently filter at the wrong frequency.
    let nyquist = fs / 2.0;
    if spec.cutoff_hz >= nyquist * 0.999 {
        eprintln!(
            "warning: cutoff {:.3} Hz is at/above Nyquist ({:.1} Hz) for this signal — \
             the effective cutoff will be clamped, exactly as the GUI does",
            spec.cutoff_hz, nyquist,
        );
    }

    let signal = Signal::new(uniform, fs, times_s.first().copied().unwrap_or(0.0));
    // The real filter application: same function a Filter node runs.
    let output = nodes::apply_filter(&signal, spec, spec.zero_phase);
    if output.len() != signal.len() {
        return Err(format!(
            "internal error: filter changed the sample count ({} → {})",
            signal.len(),
            output.len()
        ));
    }

    write_jsonl(
        o.output.as_deref().expect("output required"),
        &times_s,
        output.samples.as_ref(),
    )?;

    let phase = if spec.zero_phase {
        "zero-phase (filtfilt)"
    } else {
        "causal single-pass"
    };
    let name = spec_name.map(|n| format!(" '{n}'")).unwrap_or_default();
    eprintln!(
        "headless: filtered {name} — '{}': {} uniform samples @ {:.1} Hz → {} ({}), \
         {} output rows written to {}",
        source_label,
        signal.len(),
        fs,
        spec.kind.label(),
        phase,
        output.len(),
        o.output.as_deref().expect("output required").display(),
    );
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Filter-spec JSON
// ─────────────────────────────────────────────────────────────────────────────

/// The dashboard-exported spec-file shape. Mirrors `dsp::spec::FilterSpec`,
/// minus `enabled` (the batch runner always runs) plus an optional display
/// `name`. `q`/`order`/`ripple_db` are null for kinds that don't use them.
/// Unknown fields and unknown `kind` values are hard errors.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SpecFile {
    name: Option<String>,
    kind: Option<FilterKind>,
    cutoff_hz: Option<f64>,
    q: Option<f64>,
    order: Option<usize>,
    ripple_db: Option<f64>,
    zero_phase: Option<bool>,
    /// Accepted for dashboard round-trips of FilterSpec; the runner always
    /// runs the filter it is given (this command is only invoked when the
    /// caller wants output), so the value is deliberately not read.
    #[allow(dead_code)]
    enabled: Option<bool>,
}

const VALID_KINDS: &str = "CookbookLowpass | CookbookHighpass | CookbookBandpass | \
     CookbookNotch | ButterworthLowpass | Chebyshev1Lowpass";

/// Read + validate a spec JSON file into a runnable [`FilterSpec`].
pub fn load_spec(path: &Path) -> Result<(FilterSpec, Option<String>), String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read spec '{}': {e}", path.display()))?;
    parse_spec_text(&text).map_err(|e| format!("invalid filter spec '{}': {e}", path.display()))
}

/// Validate a spec-JSON string into a runnable [`FilterSpec`] (plus its
/// optional display name). Pure so unit tests can exercise it without files.
fn parse_spec_text(text: &str) -> Result<(FilterSpec, Option<String>), String> {
    let file: SpecFile =
        serde_json::from_str(text).map_err(|e| format!("spec JSON parse error: {e}"))?;

    let kind = file
        .kind
        .ok_or_else(|| format!("missing required field `kind` — one of: {VALID_KINDS}"))?;
    let cutoff_hz = file.cutoff_hz.ok_or("missing required field `cutoff_hz`")?;
    if !(cutoff_hz.is_finite() && cutoff_hz > 0.0) {
        return Err(format!(
            "`cutoff_hz` must be a finite number > 0, got {cutoff_hz}"
        ));
    }
    if let Some(q) = file.q.filter(|&q| !(q.is_finite() && q > 0.0)) {
        return Err(format!("`q` must be a finite number > 0, got {q}"));
    }
    if let Some(order) = file.order {
        if order < 1 {
            return Err(format!("`order` must be >= 1, got {order}"));
        }
        if order > 64 {
            return Err(format!("`order` must be <= 64, got {order}"));
        }
    }
    if let Some(ripple) = file.ripple_db.filter(|&r| !(r.is_finite() && r > 0.0)) {
        return Err(format!(
            "`ripple_db` must be a finite number > 0, got {ripple}"
        ));
    }

    // Absent optional params fall back to the FilterSpec defaults (same values
    // the GUI starts from) — that is the "lenient" half of the contract.
    let d = FilterSpec::default();
    let spec = FilterSpec {
        enabled: true,
        kind,
        cutoff_hz,
        q: file.q.unwrap_or(d.q),
        order: file.order.unwrap_or(d.order),
        ripple_db: file.ripple_db.unwrap_or(d.ripple_db),
        zero_phase: file.zero_phase.unwrap_or(d.zero_phase),
    };
    Ok((spec, file.name))
}

// ─────────────────────────────────────────────────────────────────────────────
// Trial JSONL input
// ─────────────────────────────────────────────────────────────────────────────

/// One parsed row of the dashboard trial JSONL, keyed by joint.
struct TorqueRow {
    timestamp_us: u64,
    torque: f64,
}

/// Parse trial JSONL (torque rows and/or bare value rows) and return the
/// selected joint's series `(timestamps_us, values)` plus a display label.
fn parse_input_jsonl(
    path: &Path,
    selector: Option<&str>,
) -> Result<(Vec<u64>, Vec<f64>, String), String> {
    let file =
        File::open(path).map_err(|e| format!("cannot open input '{}': {e}", path.display()))?;

    let mut torque_rows: HashMap<String, Vec<TorqueRow>> = HashMap::new();
    let mut value_rows: Vec<(u64, f64)> = Vec::new();
    let mut schema: Option<&'static str> = None; // "torque" | "value"

    for (idx, line) in BufReader::new(file).lines().enumerate() {
        let lineno = idx + 1;
        let line = line.map_err(|e| format!("{}:{lineno}: {e}", path.display()))?;
        if line.trim().is_empty() {
            continue;
        }
        let row: serde_json::Value = serde_json::from_str(&line)
            .map_err(|e| format!("{}:{lineno}: bad JSON: {e}", path.display()))?;
        let obj = row.as_object().ok_or_else(|| {
            format!(
                "{}:{lineno}: expected a JSON object, got {row}",
                path.display()
            )
        })?;

        let t = read_field::<f64>(obj, "t", path, lineno)?;
        if !t.is_finite() || t < 0.0 {
            return Err(format!(
                "{}:{lineno}: `t` must be a finite number >= 0, got {t}",
                path.display()
            ));
        }
        let timestamp_us = (t * 1.0e6).round() as u64;

        let is_torque = obj.contains_key("torque");
        let is_value = obj.contains_key("value");
        if is_torque == is_value {
            return Err(format!(
                "{}:{lineno}: row must have exactly one of `torque` (joint_torques) or `value`, got: {}",
                path.display(),
                obj.keys().cloned().collect::<Vec<_>>().join(", ")
            ));
        }
        match schema {
            None => schema = Some(if is_torque { "torque" } else { "value" }),
            Some(s) if (s == "torque") != is_torque => {
                return Err(format!(
                    "{}:{lineno}: file mixes row schemas (first row was {s}, this one is {})",
                    path.display(),
                    if is_torque { "torque" } else { "value" }
                ));
            }
            Some(_) => {}
        }
        if is_torque {
            // metric is optional but must be the torque metric when present
            // (a mixed-metric recording's other rows are skipped).
            if obj
                .get("metric")
                .and_then(|m| m.as_str())
                .is_some_and(|metric| metric != "joint_torques")
            {
                continue;
            }
            let joint = obj.get("joint").ok_or_else(|| {
                format!(
                    "{}:{lineno}: joint_torques row is missing `joint`",
                    path.display()
                )
            })?;
            let key = joint_key(joint, path, lineno)?;
            let torque = read_field::<f64>(obj, "torque", path, lineno)?;
            if !torque.is_finite() {
                return Err(format!(
                    "{}:{lineno}: `torque` must be finite, got {torque}",
                    path.display()
                ));
            }
            torque_rows.entry(key).or_default().push(TorqueRow {
                timestamp_us,
                torque,
            });
        } else {
            let value = read_field::<f64>(obj, "value", path, lineno)?;
            if !value.is_finite() {
                return Err(format!(
                    "{}:{lineno}: `value` must be finite, got {value}",
                    path.display()
                ));
            }
            value_rows.push((timestamp_us, value));
        }
    }

    match schema {
        None => Err(format!(
            "'{}' has no data rows (expected JSONL of {{\"t\", \"metric\": \"joint_torques\", \
             \"joint\", \"torque\"}} or {{\"t\", \"value\"}})",
            path.display()
        )),
        Some("torque") => {
            if let Some(sel) = selector {
                let rows = torque_rows.remove(sel).ok_or_else(|| {
                    format!(
                        "'{}': no joint '{sel}' — file has joints: {}",
                        path.display(),
                        sorted_keys(&torque_rows).join(", ")
                    )
                })?;
                let (ts, vals) = rows_to_series(rows);
                Ok((ts, vals, sel.to_string()))
            } else if torque_rows.len() == 1 {
                let (key, rows) = torque_rows.into_iter().next().unwrap();
                let (ts, vals) = rows_to_series(rows);
                Ok((ts, vals, key))
            } else {
                Err(format!(
                    "'{}': {} distinct joints and no --joint — pass --joint <{}>",
                    path.display(),
                    torque_rows.len(),
                    sorted_keys(&torque_rows).join("|"),
                ))
            }
        }
        _ => {
            if selector.is_some() {
                return Err(format!(
                    "'{}': --joint was given but the file uses bare {{\"t\", \"value\"}} rows, \
                     not joint_torques rows",
                    path.display()
                ));
            }
            value_rows.sort_by_key(|r| r.0);
            let values = value_rows.iter().map(|r| r.1).collect();
            let timestamps = value_rows.iter().map(|r| r.0).collect();
            Ok((timestamps, values, "value".to_string()))
        }
    }
}

fn rows_to_series(rows: Vec<TorqueRow>) -> (Vec<u64>, Vec<f64>) {
    let mut rows = rows;
    rows.sort_by_key(|r| r.timestamp_us);
    let timestamps = rows.iter().map(|r| r.timestamp_us).collect();
    let values = rows.iter().map(|r| r.torque).collect();
    (timestamps, values)
}

fn sorted_keys(m: &HashMap<String, Vec<TorqueRow>>) -> Vec<String> {
    let mut keys: Vec<String> = m.keys().cloned().collect();
    keys.sort();
    keys
}

fn joint_key(joint: &serde_json::Value, path: &Path, lineno: usize) -> Result<String, String> {
    match joint {
        serde_json::Value::String(s) => Ok(s.clone()),
        serde_json::Value::Number(n) => n.as_u64().map(|u| u.to_string()).ok_or_else(|| {
            format!(
                "{}:{lineno}: numeric `joint` must be a non-negative integer, got {n}",
                path.display()
            )
        }),
        other => Err(format!(
            "{}:{lineno}: `joint` must be a name string or integer index, got {other}",
            path.display()
        )),
    }
}

fn read_field<T: serde::de::DeserializeOwned>(
    obj: &serde_json::Map<String, serde_json::Value>,
    field: &str,
    path: &Path,
    lineno: usize,
) -> Result<T, String> {
    obj.get(field)
        .ok_or_else(|| format!("{}:{lineno}: missing `{field}`", path.display()))
        .and_then(|v| {
            serde_json::from_value(v.clone()).map_err(|_| {
                format!(
                    "{}:{lineno}: `{field}` must be a number, got {v}",
                    path.display()
                )
            })
        })
}

// ─────────────────────────────────────────────────────────────────────────────
// MCAP input helpers
// ─────────────────────────────────────────────────────────────────────────────

fn looks_like_mcap(path: &Path) -> bool {
    if path.extension().is_some_and(|e| e == "mcap") {
        return true;
    }
    std::fs::File::open(path)
        .ok()
        .and_then(|mut f| {
            use std::io::Read;
            let mut head = [0u8; 8];
            f.read_exact(&mut head).ok().map(|_| head == MCAP_MAGIC)
        })
        .unwrap_or(false)
}

/// Pick the channel a `--topic`/`--channel` selector refers to. Matches the
/// exact channel name (the GUI's list format, e.g. `/ci/vel` or
/// `/imu/data · linear_acceleration.z`); a bare topic matches when it expands
/// to exactly one field channel.
fn select_mcap_channel<'a>(
    log: &'a LogFile,
    selector: &str,
    input_label: &str,
) -> Result<&'a Channel, String> {
    if let Some(ch) = log.channels.iter().find(|c| c.name == selector) {
        return Ok(ch);
    }

    let prefix = format!("{selector} · ");
    let field_matches: Vec<&Channel> = log
        .channels
        .iter()
        .filter(|c| c.name.starts_with(&prefix))
        .collect();
    if field_matches.len() == 1 {
        return Ok(field_matches[0]);
    }

    let mut available: Vec<&str> = log.channels.iter().map(|c| c.name.as_str()).collect();
    available.sort();
    Err(format!(
        "no channel '{selector}' in '{input_label}' — available channels:\n  {}",
        available.join("\n  "),
    ))
}

fn extract_channel(log: &LogFile, channel: &Channel) -> Result<(Vec<u64>, Vec<f64>), String> {
    let samples = log
        .data
        .get(&channel.entry_id)
        .ok_or_else(|| format!("internal error: no samples for channel '{}'", channel.name))?;
    let mut timestamps = Vec::with_capacity(samples.len());
    let mut values = Vec::with_capacity(samples.len());
    for s in samples {
        if let Some(v) = sample_to_f64(&s.value) {
            timestamps.push(s.timestamp_us);
            values.push(v);
        }
    }
    if timestamps.is_empty() {
        return Err(format!(
            "channel '{}' has no plottable (numeric) samples",
            channel.name
        ));
    }
    Ok((timestamps, values))
}

// ─────────────────────────────────────────────────────────────────────────────
// Output
// ─────────────────────────────────────────────────────────────────────────────

fn write_jsonl(out_path: &Path, times_s: &[f64], values: &[f64]) -> Result<(), String> {
    debug_assert_eq!(times_s.len(), values.len());
    let file = File::create(out_path)
        .map_err(|e| format!("cannot create output '{}': {e}", out_path.display()))?;
    let mut w = BufWriter::new(file);
    for (&t, &v) in times_s.iter().zip(values.iter()) {
        if !v.is_finite() {
            return Err(format!(
                "filter produced a non-finite value at t={t} — refusing to write output"
            ));
        }
        // f64 `{}` is shortest-round-trip, so this is exact JSON.
        writeln!(w, "{{\"t\": {t}, \"value\": {v}}}").map_err(|e| format!("write error: {e}"))?;
    }
    w.flush().map_err(|e| format!("write error: {e}"))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const BUTTER_JSON: &str = r#"{
        "name": "elbow LP",
        "kind": "ButterworthLowpass",
        "cutoff_hz": 50.0,
        "q": null,
        "order": 4,
        "ripple_db": null,
        "zero_phase": false
    }"#;

    #[test]
    fn spec_parses_with_null_inapplicable_params() {
        let (spec, name) = parse_spec_text(BUTTER_JSON).expect("valid butter spec");
        assert_eq!(spec.kind, FilterKind::ButterworthLowpass);
        assert_eq!(spec.cutoff_hz, 50.0);
        assert_eq!(spec.order, 4);
        assert!(!spec.zero_phase);
        assert!(spec.enabled);
        assert_eq!(name.as_deref(), Some("elbow LP"));
        // Nulls for inapplicable params fell back to defaults, not zero.
        assert!((spec.q - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
        assert_eq!(spec.ripple_db, 1.0);
    }

    #[test]
    fn spec_unknown_kind_is_refused_loudly() {
        let err = parse_spec_text(r#"{"kind": "FirLowpass", "cutoff_hz": 50.0}"#)
            .expect_err("unknown kind must fail");
        assert!(
            err.contains("unknown variant") && err.contains("ButterworthLowpass"),
            "serde error should list valid variants, got: {err}"
        );
    }

    #[test]
    fn spec_unknown_field_is_refused() {
        let err =
            parse_spec_text(r#"{"kind": "ButterworthLowpass", "cutoff_hz": 50.0, "cutff": 5.0}"#)
                .expect_err("unknown field must fail");
        assert!(
            err.contains("unknown field") && err.contains("cutff"),
            "serde error should name the unknown field, got: {err}"
        );
    }

    #[test]
    fn spec_missing_required_fields_is_refused() {
        let no_kind =
            parse_spec_text(r#"{"cutoff_hz": 50.0}"#).expect_err("missing kind must fail");
        assert!(
            no_kind.contains("missing required field `kind`"),
            "{no_kind}"
        );

        let no_cutoff =
            parse_spec_text(r#"{"kind": "CookbookNotch"}"#).expect_err("missing cutoff must fail");
        assert!(no_cutoff.contains("cutoff_hz"), "{no_cutoff}");
    }

    #[test]
    fn spec_bad_param_values_are_refused() {
        assert!(parse_spec_text(r#"{"kind":"CookbookLowpass","cutoff_hz":-1}"#).is_err());
        assert!(
            parse_spec_text(r#"{"kind":"ButterworthLowpass","cutoff_hz":10,"order":0}"#).is_err()
        );
        assert!(
            parse_spec_text(r#"{"kind":"Chebyshev1Lowpass","cutoff_hz":10,"ripple_db":0}"#)
                .is_err()
        );
        assert!(parse_spec_text(r#"{"kind":"CookbookNotch","cutoff_hz":10,"q":0}"#).is_err());
    }

    #[test]
    fn spec_every_kind_parses() {
        for (kind, extra) in [
            ("CookbookLowpass", r#""q": 0.7071"#),
            ("CookbookHighpass", r#""q": 0.7071"#),
            ("CookbookBandpass", r#""q": 1.0"#),
            ("CookbookNotch", r#""q": 30.0"#),
            ("ButterworthLowpass", r#""order": 2"#),
            ("Chebyshev1Lowpass", r#""order": 3, "ripple_db": 0.5"#),
        ] {
            let json = format!(r#"{{"kind": "{kind}", "cutoff_hz": 10.0, {extra}}}"#);
            let (spec, _) = parse_spec_text(&json).unwrap_or_else(|e| panic!("{kind}: {e}"));
            assert_eq!(spec.kind, parse_kind(kind));
        }
    }

    fn parse_kind(kind: &str) -> FilterKind {
        serde_json::from_str(&format!("\"{kind}\"")).unwrap()
    }

    #[test]
    fn torque_rows_with_missing_joint_selector_error_lists_joints() {
        let dir = std::env::temp_dir();
        let path = dir.join(format!(
            "rosfilter_headless_joints_{}.jsonl",
            std::process::id()
        ));
        std::fs::write(
            &path,
            "{\"t\": 0.0, \"metric\": \"joint_torques\", \"joint\": \"a\", \"torque\": 0.1}\n\
             {\"t\": 0.001, \"metric\": \"joint_torques\", \"joint\": \"b\", \"torque\": 0.2}\n",
        )
        .unwrap();
        let err = parse_input_jsonl(&path, None).expect_err("ambiguous joints must fail");
        assert!(
            err.contains("distinct joints") && err.contains("--joint"),
            "{err}"
        );
        // Selecting a joint that exists works.
        let (ts, vals, label) = parse_input_jsonl(&path, Some("b")).unwrap();
        assert_eq!(label, "b");
        assert_eq!(ts, vec![1000]);
        assert_eq!(vals, vec![0.2]);
        let _ = std::fs::remove_file(&path);
    }
}
