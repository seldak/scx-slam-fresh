# scx-slam-fresh

Application integration and evaluation for the external
[scx_fresh](https://github.com/seldak/scx_fresh) sched_ext scheduler.

This project asked whether application-selected scheduling hints improve
timely, useful processing under contention. **For the tested dependent graph,
we did not demonstrate an advantage over FIFO.** Under estimator overload,
budget demotion gave background work CPU but delayed fresh estimates.
The separate EDF task set matched Linux SCHED_DEADLINE and beat both tested
FIFO orders. PREEMPT_RT preserved that result; it did not resolve the overload
tradeoff. See the [closing findings](docs/evaluation/conclusion.md).

Feature development is paused. The code and experiments remain available.

## Workloads

The standalone dependent graph connects synthetic IMU and camera processing to
an estimator worker, a periodic control consumer and mapping. Compute costs are
repeatable; it does not implement a real estimator or SLAM.

The optional ROS 2 workload exercises executor ownership and bag-backed input
accounting. Its older graph has an independent IMU callback alongside a
camera-triggered chain. It does not perform sensor fusion. Its historical
results concern that graph, not the standalone dependent workload.

## Build and reproduce

Requires a sched_ext-capable Linux kernel with BTF, Clang/LLVM, bpftool,
libbpf development headers and a C/C++ toolchain. The scheduler defaults to a
sibling checkout; set `SCX_FRESH_DIR` to use another location.

```bash
git clone https://github.com/seldak/scx_fresh.git ../scx_fresh
make all build/edf_workload build/dependent_workload
make test-edf
```

Build scheduler and clients together: the hint interface is unversioned and
not stable. Loaded tests require root and explicit worker/housekeeping CPUs.
Follow the [closing report](docs/evaluation/conclusion.md) for comparison
commands and the [ROS guide](ros2/README.md) for optional bag replay.

## Documentation

- [Closing findings](docs/evaluation/conclusion.md): comparisons and limits.
- [Evaluation](docs/DESIGN_EVALUATION.md): historical reports and accounting.
- [Usage](docs/USAGE.md): workload and test commands.
- [Architecture](docs/DESIGN.md): application components.
- [Hint integration](docs/DESIGN_HINTS_API.md): executor ownership.
- [Scheduler](https://github.com/seldak/scx_fresh): policy, ABI and experimental RT patches.

## License

Application code and documentation are MIT-licensed. The external scheduler
and loader are GPL-2.0-only, with an MIT client API.
See [licensing](LICENSING.md) for dependency notices.
