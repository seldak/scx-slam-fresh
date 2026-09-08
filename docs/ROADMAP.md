# Project status

This repository owns the workloads, ROS integration and evaluation. Scheduler
implementation lives in scx_fresh. The BE insertion cap and Background server
remain disabled by default; current work does not change scheduler policy.

## Completed

- Standalone calibration, overload, burst, budget, and IMU-load experiments.
- ROS callback execution with one worker per callback group.
- Application-owned expiry and executor stale-selection rejection.
- Deterministic bag identity, source windows, and explicit drop accounting.
- EuRoC uncapped baseline, opt-in 2 ms cap, and capped hint ablations.
- Optional Background-server integration, adapter match readiness and delivery tracing.

See the [evaluation reports](DESIGN_EVALUATION.md) for results and scope.

## Open questions

The current evidence does not separate FE class priority from EDF ordering
and budgets. It also does not quantify cap overhead against an uncapped run
with equivalent hog counters. These are limits of the results, not scheduled
policy changes.

Additional slice values, FIFO comparisons, other datasets, and a real estimator
integration are outside the current cleanup.

One server-enabled bag run produced a large delivery burst and missed IMU
deadlines. Subsequent diagnostic and uninstrumented checks passed, but the
original cause remains unresolved. Adapter readiness now checks local middleware
matches before playback; this is not evidence that the burst was fixed.

## Contract boundaries

Userspace remains responsible for selecting and evicting work. A BPF pending-job
queue would duplicate that ownership and is not planned. Mid-callback migration
would require job-scoped budget accounting before it could be supported.
