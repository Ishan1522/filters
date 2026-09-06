# rosfilter — ROS 2 filter workbench

A desktop filter design + analysis workbench for **ROS 2** signal data — built
for both **simulation** (Gazebo rosbags) and **real robots** (live topics).

Load a rosbag recording (`ros2 bag record`), inspect time-domain / spectrum /
filter-response views, design biquad filters (Butterworth, Chebyshev I,
cookbook LP/HP/BP/notch), wire a node-graph pipeline, and **export the filter
as a ROS 2 node** you can drop straight into your robot or sim workspace.
Or connect live and watch the same views on real topics.

This is a full ROS 2 migration: it reads rosbag2 recordings in the **MCAP**
storage format (the default since ROS 2 Jazzy) and subscribes to live topics
via **rclrs** (the official ros2-rust client). No NetworkTables, no WPILOG.

---

## Feature flags

| Feature  | What it enables                                       | Default |
|----------|-------------------------------------------------------|---------|
| `rosbag` | Offline rosbag2 (MCAP) reading — pure Rust            | ✅ on   |
| `ros2`   | Live ROS 2 topic subscription via rclrs (needs ROS 2) | off     |

```bash
# Offline analysis only (no ROS 2 needed)
cargo build --release

# Live ROS 2 topics (requires a ROS 2 distro, see below)
cargo build --release --features ros2
```

The DSP core, pipeline, GUI, and rosbag reader compile with plain `cargo build`
— no ROS 2 install required. Only the live client needs a ROS 2 environment.

---

## Quick start (offline — simulation)

Record a bag in Gazebo / your sim, or from the robot:

```bash
ros2 bag record -a -o sim_recording
```

(`-a` records everything; since Jazzy this writes an `.mcap` file inside the
`sim_recording` directory — see the note below for Humble / pre-Jazzy bags.)

Then:

```bash
cargo run --release -- path/to/sim_recording/sim_recording_0.mcap
```

> **Pre-Jazzy bags (sqlite3 format)** — intentionally not supported. ros2 bags
> recorded before Jazzy used the sqlite3 storage plugin by default, and
> rosfilter reads **MCAP only**. Convert legacy bags (or record with MCAP):
>
> ```bash
> ros2 bag convert --input <old_bag> --storage mcap
> ros2 bag record -a --storage mcap -o sim_recording   # Jazzy/Humble with plugin
> ```
>
> (Humble users also need the plugin: `sudo apt install
> ros-humble-rosbag2-storage-mcap`.)

What you can do in the app:

- **Signal view** — click topics in the left panel; see time traces, spectrum
  (click the spectrum to set the filter cutoff), and filter response.
- **Graph view** — right-click the canvas to add Filter / Sum / Differentiate /
  Gain nodes; wire topic sources (added from the inspector panel) into Output
  sinks and watch the pipeline evaluate live.
- **Export** — enable a filter and hit **Export…** to generate:

  | Tab                  | What you get                                                          |
  |----------------------|-----------------------------------------------------------------------|
  | `filter_chain.yaml`  | YAML params (raw biquad coefficients) for the generated node          |
  | `rclcpp node (C++)`  | Standalone rclcpp filter node: subscribe → cascade → publish          |
  | `rclrs node (Rust)`  | Standalone rclrs filter node (same behavior)                          |
  | `Coefficients`       | Plain-text section coefficients                                       |

  Drop the generated node into a robot or sim package and run it — it
  reproduces exactly what the GUI previewed (causal single-pass; zero-phase
  filtfilt is offline-only).

### Supported rosbag message types

| Type | Extraction |
|------|------------|
| `std_msgs/msg/Float64`, `Float32`, `Int64`, `Int32`, `Bool` | the value |
| `std_msgs/msg/Float64MultiArray` | one signal per element (`data[i]`, capped at 64) |
| `sensor_msgs/msg/Imu` | orientation, angular velocity, linear acceleration components |
| `sensor_msgs/msg/JointState` | position per joint |
| `nav_msgs/msg/Odometry` | pose position/orientation, twist linear/angular |
| `geometry_msgs/msg/Twist`, `TwistStamped` | linear/angular velocity |
| `geometry_msgs/msg/Vector3`, `PointStamped` | x/y/z |

Other types are skipped (listed on the console) — extend the decoder in
`src/io/rosbag_loader.rs` (CDR reader in `src/io/cdr.rs`).

---

## Live ROS 2 topics (real robot or live sim)

Live mode uses **DDS discovery** — set your `ROS_DOMAIN_ID`, pick topics, and
connect. No host/IP box: the robot and this tool just need to be on the same
domain and network.

Live subscriptions are **dynamic** (runtime message introspection via rclrs),
so *any* message type works — scalars (`Float64`, `Bool`, …), IMU, odometry,
multi-arrays — with a dotted **field path** selecting the channel
(e.g. `linear_acceleration.z`, `data[3]`).

### 1. Install ROS 2 + the rclrs build prerequisites

**Canonical target: ROS 2 Jazzy Jalisco** (LTS until 2029). `rclrs` 0.7 also
builds against Humble and newer distros, but Jazzy is the pinned, tested
default. If a specific distro misbehaves with crates.io `rclrs = "0.7"`,
pin the git dependency instead: `rclrs = { git =
"https://github.com/ros2-rust/ros2_rust" }` (see the ros2-rust docs).

Install ROS 2 Jazzy per the official docs, then:

```bash
sudo apt install -y git libclang-dev python3-pip

# rclrs issue #557 workaround (see https://github.com/ros2-rust/ros2_rust/issues/557)
sudo apt install -y ros-$ROS_DISTRO-example-interfaces ros-$ROS_DISTRO-test-msgs
```

### 2. Build with the `ros2` feature

```bash
source /opt/ros/jazzy/setup.bash
cargo build --release --features ros2
```

`rclrs` comes from crates.io and links against your ROS 2 install — no colcon
workspace or generated message crates needed. (The repo also ships a
`package.xml` if you prefer to build it as a colcon package.)

### 3. Use it

```bash
ros2 run rosfilter rosfilter     # or: target/release/rosfilter
```

- Enter your `ROS_DOMAIN_ID` (default 0).
- **Discover topics** to list everything on the domain (topic + type), or
  type a topic name + type manually.
- Optionally set a **field path** (empty for scalar messages).
- **Connect** — samples stream into a ring buffer and appear in the same
  Signal view / graph pipeline as offline bags.

### Known live-mode limitations

- The live client subscribes with `SensorDataQoS` (best-effort) so it matches
  both best-effort and reliable publishers; message loss is possible under
  load, which is fine for visualization.
- Dynamic message decoding requires the `rosidl_dynamic_typesupport`
  libraries that ship with standard ROS 2 installs (they are pulled in by
  rclrs at runtime).
- Topic discovery is a one-shot DDS query — if a topic is missing, re-press
  **Discover** after the publisher has been up a moment.
- Changing the subscription list requires reconnecting (press **Disconnect**
  then **Connect**).

---

## Headless mode (batch / executor CLI)

`rosfilter headless` runs the **real** biquad cascade over a recorded signal
with no GUI — designed as the "executor" half of a two-sided filter system
(a browser dashboard designs filters with a scipy preview, then shells out to
this binary to get the authoritative Rust DSP output, closing the
design-vs-executor fidelity gate).

```bash
rosfilter headless --spec filter.json --input signal.jsonl --output filtered.jsonl
```

The output is exactly what the app produces: the input is resampled onto its
natural-rate uniform grid with the same helpers the node-graph evaluator uses,
then run through `pipeline::nodes::apply_filter` — the same function a Filter
node in the graph dispatches to. No second DSP implementation.

### Filter spec (`--spec`)

JSON mirroring `dsp::spec::FilterSpec`, as the dashboard exports it:

```json
{
  "name": "elbow torque LP 50 Hz",
  "kind": "ButterworthLowpass",
  "cutoff_hz": 50.0,
  "q": null,
  "order": 4,
  "ripple_db": null,
  "zero_phase": false
}
```

| Field | Meaning |
|-------|---------|
| `name` | optional display name (echoed on stderr) |
| `kind` | **required** — one of `CookbookLowpass`, `CookbookHighpass`, `CookbookBandpass`, `CookbookNotch`, `ButterworthLowpass`, `Chebyshev1Lowpass` (exact enum names) |
| `cutoff_hz` | **required** — finite, > 0 |
| `q` / `order` / `ripple_db` | optional; `null` when a kind doesn't use them (falls back to the GUI defaults: q ≈ 0.7071, order 4, ripple 1 dB). Used per kind exactly as the GUI does |
| `zero_phase` | optional bool, default `false` = causal single pass. `true` = zero-phase filtfilt (offline-only, like the GUI preview) |

Deserialization is lenient about nulls/missing optional params but **refuses
loudly** (exit 1, message on stderr) on unknown `kind` values and unknown
JSON fields.

### Signal input (`--input`)

| Format | Rows | Selector |
|--------|------|----------|
| Dashboard trial JSONL | `{"t": <s>, "metric": "joint_torques", "joint": <name\|idx>, "torque": <Nm>}` | `--joint <name\|idx>` (defaults to the file's only joint; ambiguous files list the joints) |
| Bare series JSONL | `{"t": <s>, "value": <x>}` | — |
| rosbag2 / MCAP (`rosbag` feature) | anything `rosbag_loader` decodes (Float64, Imu, JointState, …) | `--topic <channel>` — a scalar topic (`/ci/vel`) or a `"topic · field"` channel as listed by `--list-channels` (e.g. `/imu/data · linear_acceleration.z`) |

The sample rate is estimated from the timestamps (median inter-sample
interval) and the series is filtered on that natural uniform grid — the same
thing the GUI does before filtering, so headless output and GUI preview agree.
A `--topic`/`--joint` that matches nothing lists what *is* available.

### Output (`--output`)

Filtered samples, one per uniform-grid row:

```jsonl
{"t": 0.0, "value": -0.0003}
{"t": 0.001, "value": 0.0197}
```

`t` is seconds on the input's time axis; `value` is the causal (or, with
`--zero-phase` / `zero_phase: true`, the filtfilt) cascade output. Non-finite
filter output is treated as an error and nothing is written.

### Flags, exit codes

```
rosfilter headless --spec <json> --input <signal> --output <jsonl>
                   [--joint <name|idx>] [--topic <channel>|--channel <channel>]
                   [--zero-phase] [--list-channels]
```

- `--zero-phase` overrides the spec's `zero_phase` field (offline-only
  filtfilt); without it the spec field is honored (default = causal, matching
  the exported ROS 2 node).
- `--list-channels` prints an MCAP's loadable channels and exits (no spec or
  output needed).
- Exit codes: `0` ok · `1` runtime error (bad spec/input, missing channel,
  non-finite output) · `2` usage error. `--version` / `--help` exist on both
  the root binary and the subcommand.

Example fixtures live in [`fixtures/headless/`](fixtures/headless/) (spec
files + a two-joint 1 kHz trial), exercised end-to-end by
`tests/headless_cli.rs`, which spawns the real binary.

---

## Tests

```bash
cargo test          # DSP, pipeline, CDR, rosbag round-trip, live store
cargo test --features ros2   # also compiles the rclrs live client (needs ROS 2)
```

The rosbag reader has a self-contained round-trip test: it writes a tiny MCAP
in memory (hand-encoded CDR payloads) and reads it back through the full
loader path — no ROS 2 runtime needed.

---

## CI / Verification (GitHub Actions)

`.github/workflows/ros2.yml` provisions a **real ROS 2 Jazzy** environment
(Ubuntu 24.04, the canonical target) to verify the parts a plain dev box
can't — the rclrs live code and real `ros2 bag record` output:

| Job | Checks |
|-----|--------|
| **core** | `cargo build` + `cargo test` on default features (no ROS 2; includes the MCAP round-trip) |
| **ros2** | `cargo build --features ros2` compiles `ros2_live.rs` against **real rclrs 0.7 from crates.io**; `cargo test --features ros2`; a **real bag** is recorded (`ci/publish_fixture.py` publishes Float64/Imu/Twist topics, `ros2 bag record --storage mcap` captures them) and loaded through `rosbag_loader` asserting channels + decoded message counts; a **live smoke test** subscribes via rclrs and asserts the ring buffer actually receives published data |

The workflow explicitly verifies and reports the message-typesupport
libraries (`rosidl-dynamic-typesupport` package plus the introspection
typesupport `.so`s that rclrs dynamic messages dlopen at runtime). DDS is
confined to loopback (`ROS_LOCALHOST_ONLY=1`) so the in-CI
publisher↔subscriber pair discovers each other reliably.

Trigger it manually with **Actions → ros2 → Run workflow**, or it runs on
every PR / push to the migration branch.

---

## Repository layout

```
src/
├── io/
│   ├── model.rs          # protocol-neutral LogFile / Channel / Sample
│   ├── rosbag_loader.rs  # rosbag2 (MCAP) reader + message-type registry
│   ├── cdr.rs            # minimal CDR decoder for ROS 2 payloads
│   ├── live_store.rs     # thread-safe ring buffer (live topics)
│   └── ros2_live.rs      # [ros2] rclrs live subscription client (dynamic messages)
├── dsp/                  # biquad, filter design (RBJ/Butterworth/Chebyshev), ROS 2 export
├── headless.rs           # batch/executor CLI (`rosfilter headless`), no GUI
├── pipeline/             # node-graph dataflow engine (offline analysis)
├── analysis/             # FFT, resampling, sample-rate estimation
└── ui/                   # Signal / Graph / Live (ROS topics) views
fixtures/headless/        # filter-spec JSON + trial JSONL used by the headless tests
tests/headless_cli.rs     # end-to-end tests that spawn the real binary headless
```

`package.xml` declares the colcon `cargo` build type so the repo can also be
built as a ROS 2 workspace package (`pip install colcon-cargo
colcon-ros-cargo`, then `colcon build` inside a workspace with the repo in
`src/`).

---

## Roadmap / open questions

- **Distro support**: **Jazzy is the canonical, pinned target** (see Live ROS 2
  topics above). Other distros work via the `rclrs = { git = … }` fallback.
- **Pre-Jazzy `rosbag2` (sqlite3) bags**: **DECIDED — intentionally
  unsupported.** rosfilter reads MCAP only; convert legacy bags with
  `ros2 bag convert --input <old_bag> --storage mcap` (see the note in
  "Quick start").
- **More offline message types**: extend `decode_message` in
  `rosbag_loader.rs`. The live path already handles any type via dynamic
  messages.
