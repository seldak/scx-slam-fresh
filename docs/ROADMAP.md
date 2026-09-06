# Project status

The scheduler is frozen at the current partial-switch policy. The optional
BE insertion cap remains disabled by default.

## Completed

- Standalone calibration, overload, burst, budget, and IMU-load experiments.
- ROS callback execution with one worker per executor instance.
- Executor stale-selection rejection and protection of the completion tail.
- Deterministic bag identity, source windows, and explicit drop accounting.
- EuRoC uncapped baseline, opt-in 2 ms cap, and capped hint ablations.

See the [evaluation reports](DESIGN_EVALUATION.md) for results and scope.

## Open questions

The current evidence does not separate FE class priority from EDF ordering
and budgets. It also does not quantify cap overhead against an uncapped run
with equivalent hog counters. These are limits of the results, not scheduled
policy changes.

Additional slice values, FIFO comparisons, other datasets, and a real estimator
integration are outside the current work. No new scheduling mechanism is planned.

## Contract boundaries

Userspace remains responsible for selecting and evicting work. A BPF pending-job
queue would duplicate that ownership and is not planned. Mid-callback migration
would require job-scoped budget accounting before it could be supported.
