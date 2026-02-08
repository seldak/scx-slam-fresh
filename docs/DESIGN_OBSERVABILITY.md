# DESIGN: Observability

A scheduler project is only convincing if you can measure it.

This repo emits a small set of **structured ring buffer events** from BPF:
- deadline miss (detected while the task is running late)
- budget overrun (exec time exceeds the configured budget)
- stale demotion (job age exceeds freshness window)

The userspace loader prints events, and you can extend it to log JSON/CSV.

---

## Event types
- `SLAM_EVT_DEADLINE_MISS`
- `SLAM_EVT_BUDGET_OVERRUN`
- `SLAM_EVT_STALE_DEMOTION`

## Event schema
See `include/scx_slam_fresh_shared.h`:
- timestamp
- pid/tgid
- stage_id
- job_id
- deadline, exec, age
- class (FE/BE/STALE)

---

## Suggested evaluation artifacts
For a strong portfolio writeup:
1) capture event logs for baseline (CFS) vs scx_slam_fresh
2) plot:
   - deadline miss rate over time
   - p50/p95/p99 completion latency per stage
   - “wasted work”: time spent on stale jobs

3) correlate with:
   - `perf sched` timelines
   - ftrace sched_switch
   - CPU utilization and runqueue depth

---

## Extensions
- Export ringbuf events to ROS2 diagnostics
- Add a `bpftrace` script in `scripts/` for quick introspection
- Add a small Grafana dashboard (JSON) under `tools/`
