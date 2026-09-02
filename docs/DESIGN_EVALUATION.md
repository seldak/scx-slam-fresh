# DESIGN: Evaluation plan

## Metrics
### Per stage
- completion latency (release_ts → stage output)
- deadline miss rate
- stale processing rate (age > stale_ns)
- dropped stale rate (`dropped_stale / stale_seen`)
- CPU time spent

`processed` excludes expired items. `consumer_dropped_stale` counts items rejected
after dequeue, while `queue_evicted_stale` counts expired backlog removed under the
queue lock by a producer. The reported `dropped_stale` is their sum, and
`dequeued = processed + consumer_dropped_stale`. `pending` exposes work left in
the queue when the measurement ends.

### System level
- total throughput (frames/s)
- control-relevant “freshness” of state estimate
- jitter (tail latency)

---

## Workload matrix

The demo is multi-rate: IMU at 200Hz, camera at 30Hz, and LiDAR at 10Hz. LiDAR modes provide increasing registration cost:

- `--lidar off`: no LiDAR stream
- `--lidar light`: sustainable registration load
- `--lidar mid`: borderline load
- `--lidar heavy`: intentionally unsustainable; creates a registration backlog

Additional controls are `--hog N`, `--drop-stale 1`, `--duration S`, and the
vision-stage `--vision-budget-us`, `--vision-work-us`, and
`--vision-deadline-us` knobs.

Sensor generators share one absolute start epoch, use absolute release times, and report generated counts. This keeps the offered input stream and measurement window constant across policies; scheduling delay appears as lateness or backlog rather than silently reducing the generated rate.

---

## Experiments

### ~~E0: LiDAR mode sweep~~ — completed

- Run `light`, `mid`, and `heavy` under CFS with identical CPU affinity, hog
  count, and duration.
- Require light and mid to drain every registration job without stale work or
  pending backlog.
- Require strictly increasing registration costs, with mid consuming 50-100ms
  of each 100ms LiDAR period and heavy exceeding the period.
- Require heavy to process fewer jobs than generated and accumulate both stale
  and pending registration work.

The root benchmark runs and enforces this sweep before attaching sched_ext.

### ~~E1: Heavy LiDAR overload~~ — completed

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

The script refuses to replace an active sched_ext scheduler, uses a unique BPF pin directory, records the kernel and Git revision, and cleans up only its own loader and pins. Use `REPETITIONS=3` for a less noisy comparison.

Example with explicit controls:
```bash
sudo env CPU=0 DURATION=15 SWEEP_DURATION=8 HOG_THREADS=2 \
  STALE_HOG_THREADS=0 REPETITIONS=3 \
  scripts/run_single_core_eval.sh
```

### ~~E2: Sensor burst / backlog~~ — completed

- Inject a deterministic delayed-delivery camera burst while preserving every
  frame's original sensor timestamp.
- Compare matched scx_slam_fresh runs with and without `--drop-stale 1`.
- Require stale dropping to process fewer obsolete burst frames, preserve the
  newest burst frame, and reduce its state-estimate completion age.

The root benchmark enforces all three conditions. Use `BURST_COUNT` and
`BURST_AT_MS` to change the default 12-frame burst delivered near 3000 ms.

#### Latest local verification snapshot

One root run on 2026-09-02 using kernel `7.0.0-30-generic`, CPU 0, 15-second
cases, and one repetition produced:

- E0 sweep: light and mid completed 80/80 registration jobs with no stale or
  pending work; heavy completed 29/80, observed 27 stale jobs, and left 51
  pending. Nominal registration costs were 15804us, 59583us, and 280919us.
- E1 isolation: state-estimator deadline misses were 47.7% under CFS and 0.0%
  under scx_slam_fresh.
- E1 stale shedding: LiDAR-registration pending backlog fell from 97 to 3;
  64 expired queued jobs were evicted.
- E2 burst recovery: both policies preserved newest burst frame 92. Stale
  dropping processed 2 rather than 12 burst frames and reduced its
  state-estimate completion age from 109322us to 25149us.
- E3 budget validation: with fixed 18ms vision work and a 20ms deadline, the
  correctly sized 24ms budget missed 5/455 deadlines; a 1ms budget produced
  455 overrun events, 455 confirmed BE-demotion events, and 126/455 misses.

These figures are a development verification snapshot, not a multi-run
performance claim. Reproduce them with the benchmark command above; the script
records the environment, revision, binary hashes, and per-case output in its
chosen results directory.

### ~~E3: Budget misconfiguration~~ — completed

- Compare matched scx_slam_fresh runs with fixed 18ms vision work and a 20ms
  relative deadline.
- Require the 24ms control budget to produce no overrun or demotion events.
- Require a 1ms budget to produce both overrun and confirmed BE-demotion
  events, while increasing vision deadline misses.
- Require the control run to drain every offered vision job so startup
  starvation cannot masquerade as a budget result.

Run E3 alone with:
```bash
sudo env EVAL_SCOPE=e3 scripts/run_single_core_eval.sh
```

### E4: IMU-lane isolation
- Sweep IMU compute cost while holding the heavy-LiDAR workload constant.
- Goal: quantify the range in which DSQ_IMU protects propagation without starving FE and BE lanes.
- Report IMU, vision, and estimator misses together; an IMU-only improvement is insufficient.

---

## Harness diagram
```mermaid
flowchart TD
  A["Run: CFS baseline"] --> B["Collect metrics\n(latency, events, perf)"]
  C["Run: scx_slam_fresh"] --> B
  B --> D["Compare:\nmiss rate, tail latency, stale work"]
  D --> E["Writeup:\nresults + discussion"]
```
