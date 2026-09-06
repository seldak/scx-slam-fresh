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
The executor links libslamqos and needs the same libbpf development dependency
as the standalone build.

Generated state stays in the ignored `.ros2-build`, `.ros2-install`, and
`.ros2-log` directories.

| Package | Purpose |
| --- | --- |
| `scx_slam_executor` | FreshnessExecutor and libslamqos integration. |
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

## Bag evaluation

Keep datasets outside the repository. Build as the normal user, then run:

```bash
sudo env CPU=14 HOUSEKEEPING_CPU=1 DURATION=15 WARMUP=3 \
  REPETITIONS=3 HOG_THREADS=0 \
  scripts/run_ros2_bag_eval.sh /absolute/path/to/ros2-bag
```

The harness runs CFS followed by hinted partial-switch SCX. Useful controls:

| Variable | Default | Meaning |
| --- | --- | --- |
| `CPU` / `HOUSEKEEPING_CPU` | `14` / `1` | Workers and hogs / DDS, dispatch, adapter, player, loader. |
| `DURATION` / `WARMUP` | `15` / `3` seconds | Measured source interval and preceding warmup. |
| `REPETITIONS` / `HOG_THREADS` | `3` / `0` | Runs per policy and contenders per run. |
| `IMU_TOPIC` | `/imu0` | Bag IMU topic. |
| `CAMERA_TOPIC` | `/cam0/image_raw` | Bag image topic. |
| `DEADLINE_GRACE_US` | `1000` | BPF grace for unowned clients; executor expiry has no grace. |
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
