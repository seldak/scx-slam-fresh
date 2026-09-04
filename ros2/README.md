# Optional ROS 2 integration

This directory contains the optional ROS 2 Lyrical integration. It is not part
of the standalone `make` build and does not change the BPF scheduler, loader,
synthetic pipeline, or E0-E4 tests.

Build and test it explicitly from the repository root:

```bash
make ros2
make test-ros2
```

The helper sources `/opt/ros/lyrical/setup.bash` by default. Set `ROS2_SETUP`
to use another installation path. It rejects an active non-Lyrical overlay so
packages from two ROS distributions cannot be mixed accidentally.

The executor wrapper links the repository's existing `libslamqos`
implementation and therefore needs the same `libbpf-dev` system dependency as
the standalone build.

Generated Colcon state is isolated in `.ros2-build`, `.ros2-install`, and
`.ros2-log`, all of which are ignored by Git.

Packages:

- `scx_slam_executor` exports the existing `libslamqos` C implementation to
  Ament consumers and provides the first `FreshnessExecutor` implementation.
- `scx_slam_msgs` defines the stamped work item shared by the ROS pipeline.
- `scx_slam_workload` provides a downstream executor smoke node and a bounded
  ROS pub/sub pipeline plus the Phase 4 sensor-to-job adapter.

## FreshnessExecutor v1

The first executor is deliberately narrow: one ROS dispatcher and one callback
worker. The dispatcher waits for and selects an `rclcpp::AnyExecutable` only
while the worker is idle, publishes that callback group's profile for the
worker, and then wakes it. The worker clears its slot after the callback ends.
There is no work stealing or mid-callback migration.

This separation locates the experiment precisely:

- DDS/RMW readiness and callback selection stay on the dispatcher.
- Only callback compute runs on the configurable worker CPU and scheduling
  policy.
- `PinnedMapHintSink` sends profiles to the pinned sched_ext map; `NullHintSink`
  runs the identical executor without hints.

Ordinary callback groups use callback selection time as `release_ts_ns`.
Message-aware subscription groups instead let the dispatcher take a normal ROS
message from DDS, extract its `job_id` and monotonic source timestamp, publish
the exact hint, and hand that same message to the worker. DDS remains the only
pending-message queue. Serialized, dynamic, and intra-process delivery are not
part of this v1 path.

## Phase 2 ROS pipeline

The bounded workload uses real ROS topics and four separate executor instances:

```text
IMU source ──────────────> IMU propagation
Camera source -> vision FE -> state estimator -> mapping BE
```

Each stage has one fixed worker. Configuring every worker for the same CPU
creates kernel-visible contention between the stages, while each executor's
dispatcher performs DDS readiness and message selection outside callback
compute. Run the unloaded, no-hint version after building:

```bash
source .ros2-install/setup.bash
ros2 run scx_slam_workload scx_slam_pipeline --duration 3
```

The executable also accepts `--worker-cpu N`, `--ext-policy N`, `--pin DIR`,
`--hint-mode MODE`, `--hog N`, and `--window-stats`. The pin and policy options
opt into the attached sched_ext policy. Contenders use the same policy as
callback workers: they are ordinary CFS threads in the CFS case and unhinted
best-effort `SCHED_EXT` threads in the scx case.

This is still a synthetic compute workload. Phase 2 proves ROS transport,
message-derived hinting, fixed worker ownership, and stage-to-stage timestamp
propagation. It is not bag evidence or an estimator accuracy result.

## Phase 3 ROS scheduling evaluation

The matched evaluation keeps DDS, RMW, publishers, and executor dispatchers on
a housekeeping CPU. Only the four callback workers and the requested CPU
contenders are pinned to the experimental CPU. This isolates the sched_ext
experiment at callback compute while retaining real ROS topic transport and
readiness handling.

Build as the normal user, then run the privileged harness:

```bash
make
make ros2 test-ros2
sudo env CPU=0 HOUSEKEEPING_CPU=1 DURATION=15 HOG_THREADS=2 \
  REPETITIONS=3 scripts/run_ros2_eval.sh
```

The harness runs matched CFS and `scx_slam` cases. It refuses full-switch mode,
uses one attached loader for all scx repetitions, records binary hashes and the
environment, and writes `summary.tsv`. Each stage reports fixed-window offered,
completed, late, started-stale, unfinished, thread CPU, p99/max callback-start
age, and p99/max completion age. Work completed after the common monotonic
cutoff is excluded.

Phase 3 is an evaluation, not a pass/fail test. The harness deliberately does
not assert that scx must beat CFS. Results remain synthetic-work evidence until
bag replay and an actual estimator are introduced in later phases.

The default `SCX_VARIANTS=hinted` runs the complete metadata policy. Ablations
can be selected individually or together:

- `hinted`: dedicated IMU stage plus FE vision/estimator hints.
- `no-hints`: all workers enter `SCHED_EXT`, but `NullHintSink` publishes no
  metadata; this is the anonymous BE fallback.
- `imu-only`: only IMU keeps its dedicated hint; every other stage is
  `MISC/BE`.
- `fe-only`: vision and estimator remain FE; IMU retains its deadline and
  budget as ordinary FE but loses the dedicated IMU stage and wakeup-preempt
  path.

Run the complete split under the same attached scheduler and workload:

```bash
sudo env CPU=0 HOUSEKEEPING_CPU=1 DURATION=15 HOG_THREADS=2 \
  REPETITIONS=3 SCX_VARIANTS=hinted,no-hints,imu-only,fe-only \
  scripts/run_ros2_eval.sh
```

The clean loaded result and its scope are recorded in
[`DESIGN_EVALUATION.md`](../docs/DESIGN_EVALUATION.md#loaded-ros-2-callback-scheduling-snapshot).

The separate zero-hog maximum-tail diagnosis is closed and recorded in
[`DESIGN_EVALUATION.md`](../docs/DESIGN_EVALUATION.md#zero-hog-partial-switch-interference-diagnosis).
On an unshielded worker CPU, foreign fair-class browser/compositor threads
outranked partial-switch SCX and left the correctly woken IMU runnable for
44-48ms. The matched shielded capture used the same `f01e3f9` binary set and no
policy change. Its boot configuration was:

```text
isolcpus=managed_irq,14-15 nohz_full=14-15 rcu_nocbs=14-15 irqaffinity=0-13 systemd.cpu_affinity=0-13
```

The kernel rejects sched_ext with `isolcpus=domain`, so that flag must not be
added. Workers ran on CPU 14, CPU 1 retained housekeeping work, and SMT sibling
15 was left unused. Reproduce the standard-perf capture with:

```bash
sudo env CPU=14 HOUSEKEEPING_CPU=1 DURATION=15 REPETITIONS=3 \
  scripts/run_ros2_zero_hog_perf.sh
```

This fixed diagnostic runs only hinted SCX with zero hogs. It records
`sched_switch`, `sched_wakeup`, new wakeups, forks, and migrations on CPUs 0 and
1; `sched_stat_runtime` and custom BPF execution tracing are not enabled. The
perf recorder stays on the housekeeping CPU. Raw `perf.data`, decoded events,
workload metrics, and `perf sched timehist` are retained per repetition.
Standard scheduler tracepoints do not expose the incoming `scx.slice`, so the
report can show an observed successor run interval but must not call it the
task's requested or remaining slice. The shielded three-repetition result was
3000 completed IMU callbacks with zero late or started-stale callbacks in every
run and a maximum start age no greater than 2.532ms. Brief residual occupants
on CPU 14 mean the core is not claimed as absolutely reserved; they did not
recreate the 40ms-class failure.

The default build remains ROS-independent:

```bash
make
make test-demo test-e4 test-scheduler-mode test-window test-slice
```

## Phase 4 bag-backed input

`scx_slam_bag_adapter` accepts standard `sensor_msgs/msg/Imu` and
`sensor_msgs/msg/Image` topics from `ros2 bag play` and publishes the existing
`StampedJob` inputs on `/imu/jobs` and `/camera/jobs`. The scheduler-facing
`release_ts_ns` is sampled from `CLOCK_MONOTONIC` when the adapter takes the
sensor message. The original ROS header timestamp is preserved separately as
`source_ts_ns`; it is not used as a kernel timestamp.

The default input topics are `/imu` and `/camera/image_raw`. Override them with
ROS parameters to match a bag, for example:

```bash
ros2 run scx_slam_workload scx_slam_bag_adapter --ros-args \
  -p imu_input:=/imu0 -p camera_input:=/cam0/image_raw
```

Run the existing pipeline without its periodic synthetic publishers in a
second terminal:

```bash
ros2 run scx_slam_workload scx_slam_pipeline \
  --duration 30 --input external --window-stats
```

Then start playback in a third terminal:

```bash
ros2 bag play /path/to/bag
```

Start the adapter and pipeline before playback, and choose a pipeline duration
longer than the selected bag interval. External-mode `offered` fields count
callbacks taken by the measurement cutoff; messages still queued inside DDS
are not observable by this first adapter. Consequently this phase establishes
the bag-to-executor path but does not yet provide a matched scheduling result,
bag-level offered-count accounting, or estimator accuracy.
