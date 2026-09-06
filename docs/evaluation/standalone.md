# Standalone workload evaluation

[Evaluation overview](../DESIGN_EVALUATION.md)

The standalone demo uses synthetic IMU, camera, and LiDAR jobs without ROS.
The experiment IDs identify calibration (E0), overload and stale shedding (E1),
burst recovery (E2), and budget enforcement (E3).

## Workload matrix

The demo is multi-rate: IMU at 200Hz, camera at 30Hz, and LiDAR at 10Hz. LiDAR modes provide increasing registration cost:

- `--lidar off`: no LiDAR stream
- `--lidar light`: sustainable registration load
- `--lidar mid`: borderline load
- `--lidar heavy`: intentionally unsustainable; creates a registration backlog

Additional controls are `--hog N`, `--drop-stale 1`, `--duration S`,
`--imu-work-us N` (default 150us per fixed 5ms period), and the
vision-stage `--vision-budget-us`, `--vision-work-us`, and
`--vision-deadline-us` knobs.

Sensor generators share one absolute start epoch, use absolute release times, and report generated counts. This keeps the offered input stream and measurement window constant across policies; scheduling delay appears as lateness or backlog rather than silently reducing the generated rate.

---

## Experiments

### E0: LiDAR calibration

- Run `light`, `mid`, and `heavy` under CFS with identical CPU affinity, hog
  count, and duration.
- Require light and mid to drain every registration job without stale work or
  pending backlog.
- Require strictly increasing registration costs, with mid consuming 50-100ms
  of each 100ms LiDAR period and heavy exceeding the period.
- Require heavy to process fewer jobs than generated and accumulate both stale
  and pending registration work.

The root benchmark runs and enforces this sweep before attaching sched_ext.
Run only this calibration with:
```bash
sudo env EVAL_SCOPE=e0 scripts/run_single_core_eval.sh
```

### E1: Overload and stale shedding

Use two matched sub-experiments because strict priority plus two hogs can fully
starve the back-end. A consumer that never runs cannot demonstrate
dequeue-time dropping.

- Deadline isolation: pin the demo to one CPU with heavy LiDAR and two CPU hogs;
  compare CFS with scx_slam_fresh.
- Stale shedding: keep scx_slam_fresh active and use zero CPU hogs; compare
  keeping stale work with `--drop-stale 1`.
- Check front-end deadline misses in the first pair, then stale drops and pending
  backlogs in the second pair.

When stale dropping is enabled, each producer prunes expired queued items before
enqueueing the next release. The in-flight item is not in the queue, and the
producer does not overwrite its active scheduler hint. The consumer still
performs a dequeue-time stale check as a second line of defense.

Run the reproducible matrix:
```bash
make
sudo scripts/run_single_core_eval.sh
```

The script refuses to replace an active sched_ext scheduler, uses a unique BPF pin directory, records the kernel and Git revision, and cleans up only its own loader and pins. The recorded validation used three repetitions.

Example with explicit controls:
```bash
sudo env CPU=0 DURATION=15 SWEEP_DURATION=8 HOG_THREADS=2 \
  STALE_HOG_THREADS=0 REPETITIONS=3 \
  scripts/run_single_core_eval.sh
```

### E2: Burst recovery

- Inject a deterministic delayed-delivery camera burst while preserving every
  frame's original sensor timestamp.
- Compare matched scx_slam_fresh runs with and without `--drop-stale 1`.
- Require stale dropping to process fewer obsolete burst frames, preserve the
  newest burst frame, and reduce its state-estimate completion age.

The root benchmark enforces all three conditions. Use `BURST_COUNT` and
`BURST_AT_MS` to change the default 12-frame burst delivered near 3000 ms.

### E3: Budget enforcement

- Compare matched scx_slam_fresh runs with fixed 12ms vision CPU and a 30ms
  relative deadline.
- Require the 16ms control budget to produce no overrun or demotion events.
- Require a 1ms budget to produce both overrun and confirmed BE-demotion
  events, while increasing vision deadline misses.
- Require the control run to drain every offered vision job so startup
  starvation cannot masquerade as a budget result.

Run E3 alone with:
```bash
sudo env EVAL_SCOPE=e3 scripts/run_single_core_eval.sh
```

## Validated partial-switch results

A three-repetition root run on 2026-09-04 used kernel
`7.0.0-30-generic`, CPU 0, 15-second cases, 8-second E0 sweeps, two
isolation/E3 hogs, and no stale-shedding hogs. The scheduler reported
`ops_flags=0x8` (`SCX_OPS_SWITCH_PARTIAL`) with `switch_all=0`. The clean
source revision was `749b57921a802eb720f8b1d1951b50a5044131bf`; scheduler
policy last changed in `0252acc`. The captured binaries were:

- demo: `9707cc74b5a19b11f007d3790aa7d1773b3c796c0c4b3cec3168947d5e5e8b00`
- loader: `daba71b9ce31716b8f837951f6a579c1221d7feb151c941eb8979e054de12c8c`
- BPF object: `054ec129d16803dc292821923b555c535683b807e3324150ef2a89cb97498a28`

The benchmark completed all 33 cases and its assertions passed. Process-lifetime
totals, rather than E4's fixed-window counters, produced these results.

| E0 mode | `reg_job_us` | registration completed | late | pending | registration `cpu_us` |
| --- | ---: | ---: | ---: | ---: | --- |
| light | 15804 | 80/80 | 0 | 0 | 1264447, 1264434, 1264439 |
| mid | 51846 | 80/80 | 0 | 0 | 4147789, 4147791, 4147795 |
| heavy | 280919 | 15/80 | 15 | 65 | 4213808, 4213807, 4213813 |

E1 deadline isolation reproduced across all three matched runs. Under CFS the
state estimator completed 454/455 jobs and missed 265, 263, and 269 deadlines
(58.4%, 57.9%, and 59.3%), with `cpu_us` 1363624, 1363657, and 1363631. Under
scx_slam_fresh it completed 455/455 with zero misses and zero pending in every
run, with `cpu_us` 1365634, 1365621, and 1365633.

E1 stale shedding also reproduced independently with zero hogs:

| Policy | registration processed | pending | queue-evicted | consumer-dropped | total dropped |
| --- | --- | --- | --- | --- | --- |
| keep | 28, 28, 28 | 122, 122, 122 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| drop | 28, 28, 28 | 3, 3, 3 | 99, 100, 100 | 20, 19, 19 | 119, 119, 119 |

E2 injected burst jobs 81 through 92. Keeping stale work processed all 12 burst
frames and completed state estimation for frame 92 at 109403, 109403, and
109404us of age. Dropping stale work processed two burst frames, still
preserved frame 92, and reduced its state-estimate age to 13759, 14040, and
13808us. Vision completion age for that newest frame fell from 101478, 101645,
and 101646us to 8059, 8353, and 8121us.

E3's 16ms budget completed 455/455 vision jobs with zero late, zero pending,
zero overruns, and zero demotions in every run. Its 1ms counterpart completed
19/455, missed four deadlines, left 436 pending, and emitted 19 overrun plus 19
confirmed demotion events in every run. Both used 12ms of vision CPU work and a
30ms deadline.

These results apply to the workload and configuration recorded here.

The earlier full-switch result was withdrawn; see the
[investigation archive](../archive/investigations.md#withdrawn-full-switch-result).


## Counter definitions

`processed` excludes expired items. `consumer_dropped_stale` counts rejection
after dequeue; `queue_evicted_stale` counts producer eviction under the queue
lock. Their sum is `dropped_stale`, and
`dequeued = processed + consumer_dropped_stale`. Pending counts expose backlog
at measurement end. Compute uses thread CPU time; release, deadline, and age
use monotonic wall time.
