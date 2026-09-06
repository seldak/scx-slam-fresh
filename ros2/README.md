# ROS 2 integration

The optional ROS 2 Lyrical workspace provides an executor, synthetic callback
graph, and sensor-bag adapter. It is separate from the standalone build.

## Build and test

From the repository root:

```bash
make
make ros2
make test-ros2
```

The helper sources `/opt/ros/lyrical/setup.bash` by default. Set `ROS2_SETUP`
for another installation path. It rejects an active non-Lyrical overlay.
The executor links libfreshqos and needs the same libbpf development dependency
as the standalone build. Its client sources and ABI headers come from the
external `scx_fresh` checkout, selected by `SCX_FRESH_DIR` (default:
`../scx_fresh`). The ROS adapter remains here; it does not embed the scheduler.

Generated state stays in the ignored `.ros2-build`, `.ros2-install`, and
`.ros2-log` directories.

| Package | Purpose |
| --- | --- |
| `scx_slam_executor` | FreshnessExecutor and libfreshqos integration. |
| `scx_slam_msgs` | Stamped job message shared by stages. |
| `scx_slam_workload` | Smoke node, synthetic graph, and sensor-to-job adapter. |

## Execution model

```text
IMU source ----------------------> IMU propagation
Camera source -> vision FE -> state estimator -> mapping BE
```

Each stage has one executor instance with a dispatcher and one fixed callback
worker. Dispatchers handle DDS readiness and callback selection on the
housekeeping CPU. Workers and hogs use the experimental CPU and selected policy.

The dispatcher publishes a selected message's hint before waking its worker.
Expired messages may be rejected before publication or after handoff. Completed
hints remain through parking and are directly replaced at the next assignment.
The [executor contract](../docs/DESIGN_HINTS_API.md#executor-contract) specifies
message cleanup, ownership protection, and supported subscription types.

For an unprivileged smoke run:

```bash
source .ros2-install/setup.bash
ros2 run scx_slam_workload scx_slam_pipeline --duration 3
```

The pipeline accepts `--worker-cpu`, `--ext-policy`, `--pin`,
`--hint-mode`, `--hog`, and `--window-stats`. Its compute costs are synthetic,
including when sensor input comes from a bag.

## Prepare EuRoC MH_01_easy

The recorded evaluation uses the Machine Hall 01 easy sequence from the
[EuRoC MAV dataset](https://doi.org/10.3929/ethz-b-000690084).
Start with its ROS 1 `.bag`, not the ZIP of images and CSV files. The commands
below use the same [public mirror](https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01)
and pinned revision used for the evaluation. Allow about 5 GB for the download,
converted bag, and conversion tools.

### Download and verify

Run these commands as your normal user. They store the data separately from
the checkout and can resume an interrupted download:

```bash
euroc_dir="$HOME/datasets/euroc"
mkdir -p "$euroc_dir"
curl --fail --location --retry 3 --continue-at - \
  --output "$euroc_dir/MH_01_easy.bag" \
  'https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01/resolve/19434bff2188ded1943d3a01d5b5e6672afb117e/MH_01_easy.bag'
printf '%s  %s\n' \
  '57f440ccd68ec8dc8f9461269f5909656b86198bac3adfd677b1fcc7a1428fa9' \
  "$euroc_dir/MH_01_easy.bag" | sha256sum --check
```

Continue only when the checksum reports `OK`. The source file is 2,673,818,914
bytes. The mirror is a download source; ETH Zurich is the dataset publisher.

### Convert to ROS 2 MCAP

[Rosbags](https://ternaris.gitlab.io/rosbags/topics/convert.html) converts the
messages offline, without a ROS 1 installation or a running bridge. Install
the version used for the recorded conversion in a separate Python environment
(`python3-venv` is needed on Debian/Ubuntu):

```bash
python3 -m venv "$HOME/.venvs/euroc-tools"
"$HOME/.venvs/euroc-tools/bin/python" -m pip install 'rosbags==0.11.5'
"$HOME/.venvs/euroc-tools/bin/rosbags-convert" \
  --src "$euroc_dir/MH_01_easy.bag" \
  --dst "$euroc_dir/MH_01_easy_ros2_mcap" \
  --dst-storage mcap --dst-version 9 --dst-typestore ros2_lyrical \
  --include-topic /imu0 /cam0/image_raw
```

The destination must not already exist. This produces a ROS 2 bag directory
containing `metadata.yaml` and an MCAP file. Only IMU and the left camera are
included; this graph does not consume the right camera or ground truth.

### Check the converted input

From the repository root, using the same shell:

```bash
source /opt/ros/lyrical/setup.bash
ros2 bag info "$euroc_dir/MH_01_easy_ros2_mcap"
python3 scripts/ros2_bag_source_epoch.py \
  "$euroc_dir/MH_01_easy_ros2_mcap" /imu0 /cam0/image_raw
```

If MCAP storage support is missing, install `ros-lyrical-rosbag2-storage-mcap`.
Expected results are storage `mcap`, duration about 184.043 seconds,
36,820 `sensor_msgs/msg/Imu` messages on `/imu0`, and 3,682
`sensor_msgs/msg/Image` messages on `/cam0/image_raw`. The epoch script must print
`1403636579758555500`. This is the sensor header epoch, not the bag's storage
start time. The [evaluation report](../docs/evaluation/euroc.md) records the
converted MCAP checksum and exact measured window.

## Bag evaluation

After preparing the bag and building the workspace, run:

```bash
sudo env CPU=14 HOUSEKEEPING_CPU=1 DURATION=15 WARMUP=3 \
  REPETITIONS=3 HOG_THREADS=0 \
  scripts/run_ros2_bag_eval.sh "$HOME/datasets/euroc/MH_01_easy_ros2_mcap"
```

The harness runs CFS followed by hinted partial-switch SCX. Useful controls:

| Variable | Default | Meaning |
| --- | --- | --- |
| `CPU` / `HOUSEKEEPING_CPU` | `14` / `1` | Workers and hogs / DDS, dispatch, adapter, player, loader. |
| `DURATION` / `WARMUP` | `15` / `3` seconds | Measured source interval and preceding warmup. |
| `REPETITIONS` / `HOG_THREADS` | `3` / `0` | Runs per policy and contenders per run. |
| `IMU_TOPIC` | `/imu0` | Bag IMU topic. |
| `CAMERA_TOPIC` | `/cam0/image_raw` | Bag image topic. |
| `DEADLINE_GRACE_US` | unset | Historical age-demotion schedulers only (default 1000 there). Rejected by the harness with application-owned expiry. |
| `BE_SLICE_CAP_US` | `0` | Optional BE insertion cap in microseconds; zero disables it. |
| `HINTED_ONLY` | `0` | Set to one to skip CFS. |
| `SCX_VARIANT` | `hinted` | `hinted`, `imu-only`, or `fe-only`. |
| `BASELINE_DIR` | unset | Prior results for exact source-window comparison. |
| `OUTPUT_DIR` | generated | New result directory; existing output is not overwritten. |

Playback uses rate 1.0. The harness reads the source epoch and topic ordinals
offline, keeps QoS preflight paused, and resumes playback after endpoints are
ready. Ordinals make IDs and ID-dependent compute costs properties of the bag,
rather than startup timing.

The adapter samples monotonic release time when taking a sensor message.
Recorded header stamps identify the source window separately. Camera identity
and release time propagate through vision, estimation, and mapping.

An independent audit counts each camera input as an offer to all three stages.
`arrivals` counts subscription selections; `dropped_before_start` records
executor rejection; `dropped_upstream` resolves downstream offers whose input
was evicted earlier. A starved downstream stage cannot report zero offered work.
See [measurement rules](../docs/DESIGN_EVALUATION.md#measurement-rules).

The gate requires matched windows, no adapter drops, complete accounting, and
zero unfinished work. Every SCX variant must also complete all IMU jobs on time.
Downstream drops remain explicit outcomes. The result contains environment and
binary provenance, stage metrics, and adapter totals.

## Capped hint ablation

To reproduce full/A/B with two hogs and a 2 ms cap:

```bash
sudo env CPU=14 HOUSEKEEPING_CPU=1 DURATION=15 WARMUP=3 \
  REPETITIONS=3 DEADLINE_GRACE_US=1000 \
  scripts/run_ros2_bag_ablation.sh /absolute/path/to/ros2-bag
```

The driver fixes two hogs, the 2000 us cap, and SCX-only execution. It repeats
the full control along with A and B using identical binaries and source windows.

| Cell | IMU hint | Downstream hints |
| --- | --- | --- |
| Full | Dedicated IMU route | FE vision/estimator |
| A: `imu-only` | Dedicated IMU route | MISC/BE |
| B: `fe-only` | MISC/FE; no dedicated queue or wakeup preemption | FE vision/estimator |

These bag variants project published hints without changing admission profiles.
B preserves age protection with the existing ownership flag. It removes the
queue/preemption bundle, not preemption alone.

Hog iterations are counted in the original wall window; mapping CPU remains
in stage statistics. The driver retains strict-gate failures and continues
collecting cells, then exits nonzero if any failed. Infrastructure and adapter
failures stop the run. [Recorded results](../docs/evaluation/euroc.md) explain
why B fails the IMU gate despite complete accounting.

## CPU placement

Partial switch does not isolate the worker CPU from other scheduling classes.
The recorded bag runs used worker CPU 14, housekeeping CPU 1, and left SMT
sibling 15 unused. Their boot configuration was:

```text
isolcpus=managed_irq,14-15 nohz_full=14-15 rcu_nocbs=14-15 irqaffinity=0-13 systemd.cpu_affinity=0-13
```

These CPU numbers are machine-specific. The tested kernel rejects sched_ext
attachment with `isolcpus=domain`. The
[CPU-interference report](../docs/evaluation/ros-synthetic.md#zero-hog-partial-switch-interference-diagnosis)
records the evidence behind this setup.

## Periodic-source evaluation

For the earlier ROS graph with synthetic periodic publishers:

```bash
sudo env CPU=14 HOUSEKEEPING_CPU=1 DURATION=15 HOG_THREADS=2 \
  REPETITIONS=3 scripts/run_ros2_eval.sh
```

That runner uses `SCX_VARIANTS` (plural, comma-separated) and supports
`hinted,no-hints,imu-only,fe-only`. It is distinct from the bag runner:
legacy synthetic variants change stage profiles, whereas bag ablations project
only published hints. Do not combine their tables as one experiment.

The [periodic-source report](../docs/evaluation/ros-synthetic.md) records its
historical results. Standard-perf diagnostics use the configured worker and
housekeeping CPUs; their captured run intervals do not expose requested or
remaining sched_ext slices.

## Adapter plumbing

The adapter accepts `sensor_msgs/msg/Imu` and `sensor_msgs/msg/Image`,
publishing `StampedJob` on `/imu/jobs` and `/camera/jobs`. Input topic defaults
are `/imu` and `/camera/image_raw`; the bag harness supplies its topic overrides.

Inputs and outputs use reliable, volatile, keep-last QoS with depth 1000.
The harness creates matching playback overrides and inspects endpoints.
For manual plumbing checks, set `imu_input` and `camera_input` ROS parameters,
start the pipeline with `--input external --source-start-ns SOURCE_EPOCH_NS`,
and start playback only after both consumers are ready. Use the harness for
measured comparisons.
