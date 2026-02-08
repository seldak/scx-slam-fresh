# Roadmap (stretch goals)

These are great “resume bullets” because they show depth beyond the initial demo.

## Scheduling features
- Slack stealing: let BE consume slack when FE is comfortably ahead of deadline.
- Stage-aware CPU affinity hints (e.g., keep FE on fewer cores for cache locality).
- Optional “drop stale” policy: if stale, explicitly yield and skip compute.

## Better pipeline integration
- ROS2: integrate hint publishing into a custom executor.
- Add support for heterogeneous sensor rates (IMU + camera).

## Power / perf (still non-proprietary)
- Experiment with `scx_bpf_cpuperf_set()` (if available) to boost CPU perf during FE spikes.

## Observability
- Export events as JSON
- Add a Python notebook for plots (p50/p99, miss rate, wasted work)

## CI
- Build-only CI job (compile BPF and userspace, don’t attach)
