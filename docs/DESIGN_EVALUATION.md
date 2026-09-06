# Evaluation

The experiments measure CPU scheduling for synthetic sensor-processing jobs.
The ROS cases add real pub/sub transport, and the EuRoC case adds recorded
sensor timing and identity. Callback compute remains synthetic throughout.

## Current finding

On the tested EuRoC graph with two EXT hogs, the IMU queue/preemption bundle
keeps the 200 Hz worker on time. An opt-in 2 ms BE slice cap makes the shared
33 ms camera chain completable. Under that cap, downstream FE hints reduce
estimator p99 completion age by approximately 5–8 ms, with essentially unchanged
hog iteration count between hint variants.

The cap remains disabled by default. This finding does not separate FE class,
EDF ordering, and budget effects, or measure cap overhead against the older
uncapped run. No FIFO comparison or real-estimator accuracy result is included.

## Reports

| Workload | Question and evidence |
| --- | --- |
| [EuRoC bag replay](evaluation/euroc.md) | Executor recovery, failed uncapped baseline, 2 ms cap, and three-cell hint ablation. |
| [ROS periodic sources](evaluation/ros-synthetic.md) | Earlier callback scheduling matrix and CPU-interference diagnosis. |
| [Standalone demo](evaluation/standalone.md) | LiDAR calibration, overload, stale shedding, burst recovery, and budgets (E0–E3). |
| [IMU load sweep](evaluation/imu-load.md) | Lower-lane service as IMU compute increases under heavy LiDAR (E4). |
| [Historical notes](archive/investigations.md) | Why the full-switch result was withdrawn and the conclusions from the IMU diagnostics. |

These are different workloads and revisions. The older ROS result found no
downstream FE benefit; the later bag ablation found a tail-latency benefit.
The earlier FE-only case also predates executor recovery. Neither should be
read as a result for the other's configuration.

## Measurement rules

Scheduler release and deadline timestamps use `CLOCK_MONOTONIC`. Synthetic
compute consumes thread CPU time; it does not busy-wait for a wall duration.
Bag source timestamps identify dataset input and are kept separate from
monotonic scheduling timestamps.

| Experiment | Input and cutoff |
| --- | --- |
| Standalone E0–E3 | Process-lifetime totals; not directly comparable to fixed-window counts. |
| Standalone E4 | Fixed generator window; stage offers count delivered work, with drain reported separately. |
| ROS periodic sources | Fixed monotonic window with periodic synthetic offers. |
| EuRoC replay | Fixed source-time interval; wall cutoff anchored to the first measured IMU release. |

For bag replay, camera observations independently offer one opportunity to
vision, estimator, and mapping. An upstream drop therefore remains visible
downstream even though no subscription message reaches that stage.

```text
offered = completed + dropped_before_start + dropped_upstream + unfinished
unfinished = pending + in_flight
```

`arrivals` counts subscription selections. `executed` counts callback entries;
`completed` counts returns within the measurement. Late counts describe
completed jobs, not drops or outstanding work. Completion-age percentiles
exclude dropped and unfinished jobs, so report timely/offered beside them.
Ranges across per-run percentiles are not pooled percentiles.

CPU counters cover synthetic compute, not all executor or DDS overhead.
Hog iteration counts cover completions in the same wall window; they are a
throughput measure, not an exact CPU-time sample.

## Validity and gates

Bag cases require matching source identities and windows, zero adapter drops,
consistent accounting, and zero unfinished work. SCX cases additionally require
all IMU jobs completed with zero late. Downstream drops are explicit outcomes;
a passing accounting gate does not mean every downstream callback was timely.

The ablation runner retains rejected gate outcomes and returns nonzero if any
case fails. In the recorded FE-only cells, the failed condition was IMU
lateness, while accounting was complete. Failed cases remain in the report.

Each report records the relevant kernel, CPU placement, revision, and binary
hashes. Reproduction should use matched configurations and preserve every
repetition rather than selecting a passing run.

## Running evaluations

Use the [ROS guide](../ros2/README.md#bag-evaluation) for current bag commands,
and [standalone usage](USAGE.md) for demo builds and controls.
Historical experiment commands are kept with their reports. Datasets and raw
machine-local output stay outside Git.
