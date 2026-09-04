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
  ROS pub/sub pipeline. Bag replay is not implemented yet.

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

The executable also accepts `--worker-cpu N`, `--ext-policy N`, and `--pin DIR`.
The last two opt into the attached sched_ext policy and pinned hint map. A later
evaluation harness will keep non-worker DDS threads off the experimental CPU and
run matched CFS/sched_ext cases.

This is still a synthetic compute workload. Phase 2 proves ROS transport,
message-derived hinting, fixed worker ownership, and stage-to-stage timestamp
propagation. It is not bag evidence or an estimator accuracy result.

The default build remains ROS-independent:

```bash
make
make test-demo test-e4 test-scheduler-mode test-window test-slice
```
