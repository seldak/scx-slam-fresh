# Standalone usage

The root [README](../README.md) contains the minimal build and run commands.
This page covers additional controls. ROS and bag commands are in the
[ROS guide](../ros2/README.md).

## Dependent threaded workload

`dependent_workload` is a separate synthetic scheduling workload. IMU and camera
processing feed a bounded-batch estimator; periodic control selects the latest
completed snapshot and a predetermined setpoint. Mapping consumes snapshots from
batches containing camera input. No estimation mathematics or physical plant is
implemented. The existing `slam_pipeline_demo` remains available for reproducing
earlier experiments.

```bash
make test-graph
build/dependent_workload --cpu 14 --housekeeping-cpu 1 --duration 2 --hogs 2
```

The fixed profile uses these experimental parameters, not measured algorithm costs:

| Worker | Activation | CPU work | Service class | Completion bound |
| --- | --- | --- | --- | --- |
| IMU processing | Every 5 ms | 0.15 ms | Deadline | 5 ms from release |
| Camera processing | Every 50 ms | 4 ms | Deadline | 33 ms from release |
| Estimator | Available measurements, up to 8 per batch | 0.3 ms plus 0.1 ms per IMU and 2 ms per camera input | Deadline | 33 ms from first selected input's source time |
| Control | Every 10 ms | 0.2 ms | Urgent | 10 ms from tick |
| Mapping | Completed batch containing camera input | 2 ms | Background | 100 ms from retained camera source time, measured only |
| Optional background workers | Dispatcher replenishes an idle worker during the source window | 1 ms per job | Background | 100 ms from assignment offer, measured only |

Background workers are finite CPU jobs with dispatcher handoffs, not continuously
runnable hogs. No job budgets are requested by this profile. Queue capacity is
32 per processing queue or measurement inbox, with reject-new overflow. Control
drops a tick if its previous job is still outstanding. A snapshot is eligible
when its retained IMU measurement is at most 20 ms old at selection; this is a
synthetic consumer rule, not a claim of estimator confidence or control safety.

Workers enroll and park before the source epoch is chosen. The dispatcher owns
selection and publishes a job hint before waking its worker. Completed hints
remain intact until replacement; after the workload drains, the dispatcher
retires them before waking workers for shutdown. Source offers stop at the fixed
window boundary; outstanding work drains afterward. CPU totals include that
drain, whose duration is reported separately.

`--pin DIRECTORY` enables hints and SCHED_EXT enrollment. `--trace FILE` buffers
per-job release, assignment, start, completion and dispatcher-observation times,
then writes CSV after shutdown. Lifecycle records include worker TIDs. Batching
can differ between runs and changes total estimator CPU work; compare consumed
measurements and CPU totals as well as callback counts.

For a self-contained loaded lifecycle/accounting check, with no other scheduler
attached:

```bash
sudo python3 scripts/test_dependent_loaded.py --cpu 14 --housekeeping-cpu 1
```

The runner uses a 2 ms Background slice cap and a 2 ms / 10 ms Background server,
captures loader and job logs, and cleans up its own pinned maps. It checks
enrollment, completed work and source conservation. Callback misses remain
reported; passing accounting is not a latency guarantee. Use an external timeout
when invoking the workload directly under a policy that may starve a worker.

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

Evaluation requires partial-switch mode (`0x8`). Scheduler attachment options
and build-mode details belong in the
[scheduler operation guide](https://github.com/seldak/scx_fresh/blob/main/docs/USAGE.md).
The local `BUILD_DIR` does not select a different external build.
The demo accepts the numeric worker policy with `--ext-policy` because libc
headers may lag the kernel.

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
