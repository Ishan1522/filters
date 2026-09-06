//! End-to-end tests for the headless CLI: spawn the real built binary
//! (`env!("CARGO_BIN_EXE_rosfilter")`) with a spec + signal and verify the
//! filtered output. These exercise the actual runtime path — spec JSON →
//! compile → biquad cascade (or filtfilt) → JSONL.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::sync::atomic::{AtomicU64, Ordering};

fn bin() -> &'static str {
    env!("CARGO_BIN_EXE_rosfilter")
}

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures/headless")
        .join(name)
}

fn headless(args: &[&str]) -> Output {
    Command::new(bin())
        .arg("headless")
        .args(args)
        .output()
        .expect("spawn rosfilter headless")
}

fn run(args: &[&str]) -> Output {
    Command::new(bin())
        .args(args)
        .output()
        .expect("spawn rosfilter")
}

/// Unique temp path per call (tests run in parallel threads).
fn tmp_path(suffix: &str) -> PathBuf {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let id = NEXT.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "rosfilter_hd_{}_{}_{suffix}",
        std::process::id(),
        id
    ))
}

fn write_file(path: &Path, contents: &str) {
    std::fs::write(path, contents).expect("write temp file");
}

/// Parse headless JSONL output rows into (t, value).
fn read_output(path: &Path) -> Vec<(f64, f64)> {
    let text = std::fs::read_to_string(path).expect("read output");
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let v: serde_json::Value = serde_json::from_str(l).expect("output row is JSON");
            (
                v["t"].as_f64().expect("t"),
                v["value"].as_f64().expect("value"),
            )
        })
        .collect()
}

fn rms(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    (values.iter().map(|v| v * v).sum::<f64>() / values.len() as f64).sqrt()
}

// ─────────────────────────────────────────────────────────────────────────────

/// The dashboard trial contract, end to end: torque JSONL → real binary →
/// causal filtered JSONL. In-band 5 Hz torque passes (~unity), out-of-band
/// 200 Hz torque is strongly attenuated, everything is finite, and the row
/// count matches the 1 kHz input grid.
#[test]
fn trial_jsonl_filters_joint_selection_and_frequency_response() {
    let out = tmp_path("trial_pb.jsonl");
    let status = headless(&[
        "--spec",
        fixture("spec_butterworth_lp_50.json").to_str().unwrap(),
        "--input",
        fixture("trial_joint_torques.jsonl").to_str().unwrap(),
        "--joint",
        "shoulder_pan",
        "--output",
        out.to_str().unwrap(),
    ]);
    assert!(
        status.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&status.stderr)
    );
    let rows = read_output(&out);
    assert_eq!(rows.len(), 1001, "1 kHz × 1 s uniform grid");
    assert!(rows.iter().all(|(_, v)| v.is_finite()), "all output finite");
    let mid: Vec<f64> = rows.iter().skip(100).map(|(_, v)| *v).collect(); // skip transient
    let mid_rms = rms(&mid);
    assert!(
        (0.65..=0.75).contains(&mid_rms),
        "5 Hz in passband should pass ~unchanged, rms = {mid_rms}"
    );
    // Output time axis matches the input grid.
    assert!(
        (rows[1].0 - 0.001).abs() < 1e-9,
        "second row t = {}",
        rows[1].0
    );

    // Same file, other joint: the 200 Hz tone is ~50 dB down after the LP.
    let out2 = tmp_path("trial_sb.jsonl");
    let status2 = headless(&[
        "--spec",
        fixture("spec_butterworth_lp_50.json").to_str().unwrap(),
        "--input",
        fixture("trial_joint_torques.jsonl").to_str().unwrap(),
        "--joint",
        "elbow_joint",
        "--output",
        out2.to_str().unwrap(),
    ]);
    assert!(status2.status.success());
    let rows2 = read_output(&out2);
    let sb = rms(&rows2.iter().skip(100).map(|(_, v)| *v).collect::<Vec<_>>());
    // Theory: bilinear-prewarped 4th-order Butterworth gives |H(200 Hz)| ≈
    // 2.3e-3 (prewarped ratio (tan(0.2π)/tan(0.05π))^4 ≈ 443), so RMS ≈
    // 0.707·2.3e-3 ≈ 1.6e-3. Assert well below unity with margin.
    assert!(
        sb < 0.02,
        "200 Hz in stopband should be attenuated, rms = {sb}"
    );
    let _ = (std::fs::remove_file(&out), std::fs::remove_file(&out2));
}

/// A constant (DC) input distinguishes causal from zero-phase: the causal
/// default ramps from 0 → 1, while --zero-phase (filtfilt) is 1 everywhere —
/// which also proves `--zero-phase` works through the real binary.
#[test]
fn causal_default_and_zero_phase_flag_on_a_step() {
    // 0.5 s of constant 1.0 at 1 kHz → 501 rows.
    let input = tmp_path("step.jsonl");
    let mut text = String::new();
    for i in 0..=500 {
        text.push_str(&format!(
            "{{\"t\": {}, \"value\": 1.0}}\n",
            i as f64 / 1000.0
        ));
    }
    write_file(&input, &text);

    // Default (spec zero_phase=false): causal single pass.
    let out = tmp_path("step_causal.jsonl");
    let status = headless(&[
        "--spec",
        fixture("spec_butterworth_lp_50.json").to_str().unwrap(),
        "--input",
        input.to_str().unwrap(),
        "--output",
        out.to_str().unwrap(),
    ]);
    assert!(
        status.status.success(),
        "{}",
        String::from_utf8_lossy(&status.stderr)
    );
    let rows = read_output(&out);
    assert_eq!(rows.len(), 501);
    assert!(
        rows[0].1 < 0.5,
        "causal step starts near 0, got {}",
        rows[0].1
    );
    assert!(
        (rows[500].1 - 1.0).abs() < 1e-3,
        "causal step settles to DC gain 1, got {}",
        rows[500].1
    );

    // --zero-phase flag overrides the spec's false → filtfilt: 1 everywhere.
    let out_zp = tmp_path("step_zp.jsonl");
    let status_zp = headless(&[
        "--spec",
        fixture("spec_butterworth_lp_50.json").to_str().unwrap(),
        "--input",
        input.to_str().unwrap(),
        "--output",
        out_zp.to_str().unwrap(),
        "--zero-phase",
    ]);
    assert!(
        status_zp.status.success(),
        "{}",
        String::from_utf8_lossy(&status_zp.stderr)
    );
    let rows_zp = read_output(&out_zp);
    for (i, (_, v)) in rows_zp.iter().enumerate() {
        assert!(
            (v - 1.0).abs() < 1e-2,
            "zero-phase constant stays constant, sample {i}: {v}"
        );
    }

    // Spec-JSON zero_phase=true is honored without the flag.
    let out_spec = tmp_path("step_spec_zp.jsonl");
    let status_spec = headless(&[
        "--spec",
        fixture("spec_butterworth_lp_50_zero_phase.json")
            .to_str()
            .unwrap(),
        "--input",
        input.to_str().unwrap(),
        "--output",
        out_spec.to_str().unwrap(),
    ]);
    assert!(status_spec.status.success());
    let rows_spec = read_output(&out_spec);
    assert!(
        (rows_spec[0].1 - 1.0).abs() < 1e-2,
        "spec zero_phase=true honored without flag, y0 = {}",
        rows_spec[0].1
    );

    for p in [input, out, out_zp, out_spec] {
        let _ = std::fs::remove_file(p);
    }
}

/// Numeric-index joints (`--joint 0`) and a cookbook notch spec both work.
#[test]
fn numeric_joint_selector_and_cookbook_notch() {
    // Notch at 200 Hz should flatten the 200 Hz sine that the LP let nothing
    // of through anyway — instead use it on the pure 200 Hz numeric file:
    // with the notch on the tone, output is tiny.
    let out = tmp_path("notch.jsonl");
    let status = headless(&[
        "--spec",
        fixture("spec_cookbook_notch_200.json").to_str().unwrap(),
        "--input",
        fixture("trial_joint_torques_numeric.jsonl")
            .to_str()
            .unwrap(),
        "--joint",
        "0",
        "--output",
        out.to_str().unwrap(),
    ]);
    assert!(
        status.status.success(),
        "{}",
        String::from_utf8_lossy(&status.stderr)
    );
    let rows = read_output(&out);
    assert_eq!(rows.len(), 1001);
    let sb = rms(&rows.iter().skip(100).map(|(_, v)| *v).collect::<Vec<_>>());
    assert!(sb < 0.05, "200 Hz notch kills the tone, rms = {sb}");
    let _ = std::fs::remove_file(out);
}

/// MCAP (rosbag2) input through the real loader path: `--list-channels`, then
/// filter a scalar Float64 topic.
#[test]
fn mcap_input_lists_channels_and_filters() {
    let mcap_path = tmp_path("sine.mcap");
    write_sine_mcap(&mcap_path, 1000, 200.0); // 200 Hz tone @ 1 kHz, 1 s

    let list = headless(&["--input", mcap_path.to_str().unwrap(), "--list-channels"]);
    assert!(
        list.status.success(),
        "{}",
        String::from_utf8_lossy(&list.stderr)
    );
    let stdout = String::from_utf8_lossy(&list.stdout);
    assert!(stdout.contains("/ci/sine"), "channel listed: {stdout}");

    let out = tmp_path("mcap_out.jsonl");
    let status = headless(&[
        "--spec",
        fixture("spec_butterworth_lp_50.json").to_str().unwrap(),
        "--input",
        mcap_path.to_str().unwrap(),
        "--topic",
        "/ci/sine",
        "--output",
        out.to_str().unwrap(),
    ]);
    assert!(
        status.status.success(),
        "{}",
        String::from_utf8_lossy(&status.stderr)
    );
    let rows = read_output(&out);
    assert_eq!(rows.len(), 1000, "1 kHz MCAP, 1 s → 1000 grid rows");
    assert!(rows.iter().all(|(_, v)| v.is_finite()));
    let sb = rms(&rows.iter().skip(100).map(|(_, v)| *v).collect::<Vec<_>>());
    assert!(sb < 0.02, "200 Hz attenuated after LP, rms = {sb}");
    let _ = (std::fs::remove_file(mcap_path), std::fs::remove_file(out));
}

/// Bad spec / bad usage produce loud non-zero exits, and version works.
#[test]
fn bad_specs_usage_and_version_exit_cleanly() {
    // Unknown kind → runtime error exit 1, message lists valid kinds.
    let bad_kind = tmp_path("bad_kind.json");
    write_file(&bad_kind, r#"{"kind": "FirLowpass", "cutoff_hz": 50.0}"#);
    let out = tmp_path("x.jsonl");
    let r1 = headless(&[
        "--spec",
        bad_kind.to_str().unwrap(),
        "--input",
        fixture("trial_joint_torques_numeric.jsonl")
            .to_str()
            .unwrap(),
        "--joint",
        "0",
        "--output",
        out.to_str().unwrap(),
    ]);
    assert_eq!(r1.status.code(), Some(1));
    let err = String::from_utf8_lossy(&r1.stderr);
    assert!(err.contains("unknown variant"), "stderr: {err}");

    // Unknown JSON field → refused.
    let bad_field = tmp_path("bad_field.json");
    write_file(
        &bad_field,
        r#"{"kind": "ButterworthLowpass", "cutoff_hz": 50.0, "cutff": 3.0}"#,
    );
    let r2 = headless(&[
        "--spec",
        bad_field.to_str().unwrap(),
        "--input",
        fixture("trial_joint_torques_numeric.jsonl")
            .to_str()
            .unwrap(),
        "--joint",
        "0",
        "--output",
        out.to_str().unwrap(),
    ]);
    assert_eq!(r2.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&r2.stderr).contains("unknown field"));

    // Missing required flag → usage error exit 2.
    let r3 = headless(&[
        "--spec",
        fixture("spec_butterworth_lp_50.json").to_str().unwrap(),
    ]);
    assert_eq!(r3.status.code(), Some(2));

    // Unknown flag on the subcommand → usage error exit 2.
    let r4 = headless(&["--bogus"]);
    assert_eq!(r4.status.code(), Some(2));

    // --version on the root binary.
    let r5 = run(&["--version"]);
    assert!(r5.status.success());
    assert_eq!(
        String::from_utf8_lossy(&r5.stdout).trim(),
        format!("rosfilter {}", env!("CARGO_PKG_VERSION"))
    );

    for p in [bad_kind, bad_field, out] {
        let _ = std::fs::remove_file(p);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tiny MCAP writer (mirrors the loader's own round-trip helper)
// ─────────────────────────────────────────────────────────────────────────────

fn write_sine_mcap(path: &Path, fs_hz: usize, tone_hz: f64) {
    use std::collections::BTreeMap;
    use std::io::Cursor;
    use std::sync::Arc;

    let mut buf = Cursor::new(Vec::new());
    {
        let mut writer = mcap::Writer::new(&mut buf).expect("mcap writer");
        let schema = mcap::Schema {
            id: 1,
            name: "ros2msg:std_msgs/msg/Float64".to_string(),
            encoding: "ros2msg".to_string(),
            data: std::borrow::Cow::Owned(b"msg\n".to_vec()),
        };
        let channel = mcap::Channel {
            id: 1,
            topic: "/ci/sine".to_string(),
            schema: Some(Arc::new(schema)),
            message_encoding: "cdr".to_string(),
            metadata: BTreeMap::new(),
        };
        let n = fs_hz;
        for i in 0..n {
            let t = i as f64 / fs_hz as f64;
            let v = (2.0 * std::f64::consts::PI * tone_hz * t).sin();
            writer
                .write(&mcap::Message {
                    channel: Arc::new(channel.clone()),
                    sequence: 0,
                    log_time: (i as u64) * 1_000_000, // ns → 1 ms steps
                    publish_time: (i as u64) * 1_000_000,
                    data: std::borrow::Cow::Owned(v.to_le_bytes().to_vec()),
                })
                .expect("write message");
        }
        writer.finish().expect("finish");
    }
    let mut file = std::fs::File::create(path).expect("create mcap");
    file.write_all(&buf.into_inner()).expect("write mcap bytes");
}
