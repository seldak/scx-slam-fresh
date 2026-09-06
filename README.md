# scx-slam-fresh

Application integration and evaluation for the external `scx_fresh` Linux
`sched_ext` scheduler. Userspace selects a job and publishes its scheduling
metadata before waking the worker.

The repository includes a standalone synthetic workload and an optional ROS 2
executor and bag adapter. Neither runs a SLAM estimator; they exercise callback
scheduling with repeatable compute costs.

## How it works

- **IMU fast path:** wakeup preemption and a dedicated dispatch queue protect
  the 200 Hz propagation worker.
- **Front-end service:** vision and estimator workers use effective-deadline
  ordering. CPU-budget overruns demote eligible work to background service.
- **Executor ownership:** stale subscriptions can be rejected before callback
  entry. An accepted owner can finish its handoff and completion path without
  being stranded by age demotion.
- **Optional BE cap:** shorter background insertion slices reduce handoff
  waits under contention. The cap is disabled by default.

The hint map contains one selected job per worker, not an application queue.
See the [architecture](docs/DESIGN.md) and
[scheduler rules](docs/DESIGN_SCHED_ALGO.md).

## What the experiments show

On the tested EuRoC-driven synthetic graph with two EXT hogs, an opt-in 2 ms
BE cap kept all 300 camera-chain jobs timely while the IMU fast path preserved
3000 on-time callbacks per run. Removing downstream FE hints added roughly
5–8 ms to estimator p99 completion age. Removing the IMU queue/preemption
bundle caused 7370 of 9000 IMU callbacks to finish late across three runs.

Hog throughput was essentially unchanged between the capped hint variants.
Those counters do not establish the cap's overhead against the uncapped run.
These are measurements on one workload and machine, not latency guarantees
or estimator-accuracy results.

[Results and methodology](docs/DESIGN_EVALUATION.md) include the failed baseline,
the cap comparison, and the ablations.

## Build

Requires a Linux kernel with `CONFIG_SCHED_CLASS_EXT=y`, kernel BTF,
Clang/LLVM, bpftool, libbpf development headers, and a C/C++ build toolchain.
The recorded evaluations used Linux 7.0.0-30/31-generic. The implementation
targets sched_ext kernels starting at 6.12, with compatibility handling for
renamed kfuncs; kernel/API compatibility still needs checking on other builds.

Typical Debian/Ubuntu packages are `clang`, `llvm`, `bpftool`, `libbpf-dev`,
`build-essential`, `pkg-config`, `zlib1g-dev`, and `libelf-dev`.

The build requires the separate [scx_fresh](https://github.com/seldak/scx_fresh)
checkout. It defaults to the sibling
directory `../scx_fresh`; set `SCX_FRESH_DIR` for another location. No scheduler
sources are vendored here. The extraction was validated with scheduler revision
`0426580`. Access to the private scheduler repository is currently required.

```bash
git clone https://github.com/seldak/scx_fresh.git ../scx_fresh
git -C ../scx_fresh checkout 0426580
```

```bash
make
make test-scheduler-mode
```

The default object uses partial-switch mode (`ops_flags=0x8`): only
`SCHED_EXT` tasks enter this scheduler. Loading BPF requires root or equivalent
privileges. This is experimental code; use a test machine.

## Try it

A short workload smoke run needs no scheduler attachment:

```bash
./build/slam_pipeline_demo --no-hints --lidar off --hog 0 --duration 3
```

To run with the scheduler, start the loader:

```bash
sudo ./build/scx_slam_fresh_user --pin /sys/fs/bpf/scx_slam_fresh
```

In another terminal, run the demo with the system's `SCHED_EXT` policy number
(`7` on the tested kernel):

```bash
sudo ./build/slam_pipeline_demo \
  --pin /sys/fs/bpf/scx_slam_fresh --ext-policy 7 \
  --lidar off --hog 0 --duration 3
```

For ROS 2 Lyrical, build the optional workspace with `make ros2` and
`make test-ros2`. Follow the [ROS guide](ros2/README.md) for bag replay and
matched evaluations.

## Documentation

- [Usage](docs/USAGE.md): demo options, tests, and build modes.
- [Architecture](docs/DESIGN.md): components and work ownership.
- [Scheduler](docs/DESIGN_SCHED_ALGO.md): routing, budgets, and slices.
- [Hints API](docs/DESIGN_HINTS_API.md): publication and executor contract.
- [ROS 2](ros2/README.md): integration, replay, and harness commands.
- [Evaluation](docs/DESIGN_EVALUATION.md): findings, measurement rules, and reports.
- [Project status](docs/ROADMAP.md): completed work and open questions.

Upstream references:
[Linux sched_ext](https://docs.kernel.org/scheduler/sched-ext.html),
[reference schedulers](https://github.com/sched-ext/scx).

## License

This repository's application code and documentation are MIT-licensed. The
external scheduler and loader remain GPL-2.0-only, with an MIT client API.
See [licensing](LICENSING.md) for dependency and distribution notices.
