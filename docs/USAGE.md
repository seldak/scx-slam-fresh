# Standalone usage

The root [README](../README.md) contains the minimal build and run commands.
This page covers additional controls. ROS and bag commands are in the
[ROS guide](../ros2/README.md).

## Build modes

`make` builds the external `scx_fresh` checkout and the local standalone demo.
Set `SCX_FRESH_DIR` to select the checkout; the default is `../scx_fresh`.
The ignored local build directory contains copies of scheduler artifacts under
their legacy names so existing evaluation commands continue to work. ROS is
not required. `make test-scheduler-mode` and `make test-slice` run the external
scheduler's tests.

Check the embedded scheduler mode without attaching:

```bash
./build/scx_slam_fresh_user --print-ops-flags
make test-scheduler-mode
```

Default builds report `0x8` (partial switch). A full-switch build reports zero
and is rejected by the evaluation runners. For an explicit full-switch test
build, clean both the external scheduler build and the local build before
running `make SLAM_FULL_SWITCH=1`. This places all
eligible tasks under sched_ext and is outside the validated evaluation setup.
Return to clean default builds afterward. Make does not track command-line
flag changes. The local `BUILD_DIR` does not select a different external build.

The loader pins `task_hints` and the `events` ring buffer beneath the supplied
pin directory. In partial mode, workers must enter `SCHED_EXT`; the demo accepts
the numeric policy with `--ext-policy` because libc headers may lag the kernel.

## Demo controls

Run `./build/slam_pipeline_demo --help` for the complete CLI.

| Option | Purpose |
| --- | --- |
| `--lidar off\|light\|mid\|heavy` | Select the 10 Hz LiDAR workload; heavy intentionally accumulates backlog. |
| `--hog N` | Add CPU contenders. |
| `--duration S` | Sensor-release duration. |
| `--imu-work-us N` | IMU CPU work per 5 ms period; default 150 us. |
| `--drop-stale 1` | Reject expired dequeued jobs and evict expired queued backlog. |
| `--camera-burst-count N` | Delay and release N frames together, retaining timestamps. |
| `--camera-burst-at-ms N` | Select burst delivery offset. |
| `--vision-work-us N` | Fixed vision CPU work; zero keeps the default 3–5 ms pattern. |
| `--vision-budget-us N` | Vision execution budget. |
| `--vision-deadline-us N` | Vision relative deadline. |
| `--window-stats` | Report the fixed-window counters separately from post-cutoff drain. |

IMU compute zero disables work, not releases or hint publication.
At or above 5000 us per tick, IMU alone can fill the core. Its worker drains
all releases scheduled within the duration, so process exit may be later than
the measurement cutoff.

A short 30% nominal IMU-utilization smoke run:

```bash
./build/slam_pipeline_demo --no-hints --lidar off --hog 0 \
  --duration 2 --imu-work-us 1500
```

## Local checks

```bash
make
make test-demo test-scheduler-mode test-window test-slice test-e4
```

These checks do not replace loaded experiments. Build the optional ROS workspace
with `make ros2` and test it with `make test-ros2`.

## Evaluation runners

The [standalone report](evaluation/standalone.md) defines E0–E3: calibration,
overload and stale shedding, burst recovery, and budget enforcement.
Run the matrix with:

```bash
sudo env CPU=0 REPETITIONS=3 scripts/run_single_core_eval.sh
```

Select calibration alone with `EVAL_SCOPE=e0`, or the budget pair with
`EVAL_SCOPE=e3`. The [IMU sweep report](evaluation/imu-load.md) contains the
separate E4 command and its fixed-window definitions.

Historical probe options are outside the current bag evaluation. Consult the
runner's `--help` for the retained diagnostic switches.
