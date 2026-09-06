# IMU load sweep

[Evaluation overview](../DESIGN_EVALUATION.md)

This standalone experiment (E4) varies IMU compute cost with heavy LiDAR,
zero hogs, and a fixed measurement window. It measures which stages lose
service as IMU utilization rises. It predates the ROS executor recovery.

## Results

The heavy-LiDAR control completes only 27 of 150 offered registration jobs;
no tested point is a whole-pipeline operating range without backlog.
The sweep distinguishes four observations:

- **Critical stages healthy:** IMU, vision, and estimator have all offered jobs
  completed, zero late completions, and zero unfinished work.
- **Declining BE service:** registration completion rate falls relative to the
  same repetition's 3% control. Mapping is reported separately.
- **Estimator backlog:** 65% IMU utilization remains critical-healthy;
  at 70%, the deterministic peak misses and stale demotion compounds the delay.
- **IMU saturation:** at 95–100%, offered IMU compute approaches or fills the
  entire CPU window.

The validation used no tracing or diagnostic policy overrides:

```bash
sudo python3 scripts/run_e4_eval.py \
  --costs 150,500,750,1000,3000,3250,3500,4750,5000 \
  --repetitions 3 \
  --seed 17
```

This is 30 cases: each repetition has fresh 3% start/end controls around shuffled
10, 15, 20, 60, 65, 70, 95, and 100% points. The workload itself is not random:
vision and estimator work remain the deterministic `seq % 5` patterns.
Seed 17 changes only the interior case order, so it detects order/drift effects
while leaving the every-fifth-job peak unchanged. Diagnostic overrides were
disabled in the validation.

The seeded run completed on 2026-09-04 with kernel `7.0.0-30-generic`, CPU 0,
15-second fixed windows, partial-switch flags `0x8`, and source-matched binaries
at Git revision `0252accae58096d98edd3cfd974b23025fc129cf`. All six 3% controls were
identical: critical stages clean, LiDAR preprocess 150, registration 27, and
mapping 482; no closing control drifted.

| IMU work | Critical stages | LiDAR preprocess | Registration vs control | Mapping |
| --- | --- | ---: | ---: | ---: |
| 3% | healthy | 150 | 27 (1.000x) | 482 |
| 10% | healthy | 150 | 24 (0.889x) | 479 |
| 15% | healthy | 150 | 21 (0.778x) | 476 |
| 20% | healthy | 150 | 18 (0.667x) | 473 |
| 60% | healthy | 54, all late | 3 (0.111x) | 458 |
| 65% | healthy | 39, all late | 2 (0.074x) | 338, 119 unfinished |

Every repetition produced each row above exactly. “Healthy” means the frozen
critical-healthy definition: IMU 3000/0/0, vision 455/0/0, and estimator
455/0/0 for completed/late/unfinished. Registration declines monotonically;
mapping's short jobs conceal most of that loss until 65%, confirming that the
two BE columns cannot be combined.

The estimator cliff also repeated exactly. At 65%, estimator job 4 completed at
approximately 29.6ms with no lateness or unfinished work. At 70%, every run had
estimator job 4 late at approximately 94.6ms, then finished the window at
240 completed, 237 late, and 215 unfinished. Thus the default stale demotion
still turns the deterministic peak miss into backlog.

At 95%, IMU completed all 3000 jobs with one or two late and approximately
14.25 seconds of compute, while vision completed 42 and left 413 unfinished. At
100%, IMU completed 2989, left 11 unfinished, and consumed approximately 14.95
seconds of the window; vision completed two. Lower lanes receive only the
residue as offered IMU compute approaches and then fills the core.

The results apply to this heavy-LiDAR workload, kernel, and CPU pin. They do
not establish a new admission threshold or scheduler policy. The
[standalone report](standalone.md) covers the separate E0–E3 validation.

---

## Measurement and runner reference

### Runner defaults

```bash
make
make test-demo test-e4 test-scheduler-mode test-window
python3 scripts/run_e4_eval.py --dry-run
sudo python3 scripts/run_e4_eval.py
```

The separate runner leaves the E0–E3 workloads unchanged. Defaults are one CPU
(CPU 0), 15 seconds of sensor releases, heavy LiDAR, zero hogs, stale dropping
off, default vision compute/deadline/budget, and the existing partial-switch
scheduler policy. Only IMU compute cost changes. Zero hogs leaves a measurable
BE reference; this is not the two-hog E1 isolation workload.

Costs are 150, 500, 1000, 2000, 3000, 4000, 4500, 4750, 5000, 5500, and 6000us
(3–120% of the 5ms period). Each repetition starts and ends with a 150us control;
the interior points are shuffled reproducibly with seed 4. The end control
checks drift, not a second denominator. Use `--cpu`, `--duration`, `--costs`,
`--repetitions`, and `--seed` to override these settings. `--output` must name a
new directory; otherwise the runner creates a temporary results directory.

Artifacts include raw per-case output, loader events, commands, environment and
binary hashes, the runner/header sources, a tracked-source diff, `cases.json`,
`matrix.csv`, `stages.csv`, and `drain.csv`. Results are saved after each successful case.
Embedded full-switch flags are rejected before attachment. After attachment,
live guards report the loader exit code, state, enable sequence, and switch mode.
Timeout, attach loss/change, full-switch mode, hint-publication errors, and
inconsistent input/accounting mark the capture incomplete, never a pass. Cleanup
terminates only the processes and removes only the BPF pins created by this run.

### Fixed window and drain

E4 enables `--window-stats`. The observation interval is the generators' shared
`CLOCK_MONOTONIC` epoch through epoch + duration, **not process exit**. Counters
are reconstructed from timestamped events, not read by a potentially delayed
observer thread. Queue operations are timestamped under their queue lock;
completions and stale-at-start observations are timestamped by the worker.
An event at the cutoff belongs to drain. Underloaded runs stay alive at least
until the cutoff; overloaded IMU ticks still drain, preserving the behavior
being diagnosed.

`window_STAGE` / `stages.csv` report offered, completed, late completions,
stale-at-start/evicted work, dropped work, pending queue entries, in-flight work,
compute CPU, and rate (`window completions / window seconds`). Queued stages'
offered count is **actually delivered to that stage before the cutoff**, not
future upstream outputs. The implicit IMU queue offers every tick scheduled in
the window. Source sensor counts and source deliveries delayed past the cutoff
are retained separately. The identity is:

`offered = completed + dropped_stale + pending + in_flight`

The console, `matrix.csv`, and `stages.csv` also report
`unfinished = pending + in_flight` for each stage. This counts work already
delivered, not upstream work that has yet to arrive. The matrix retains raw
offered/completed/late counts, queue/in-flight counts, and CPU for every stage.
`vision_to_estimator_not_delivered_at_cutoff` is vision window completions
minus estimator window arrivals; it is separate from the estimator backlog.
The console shows vision output and estimator arrival rates alongside it.

Late counts apply to completed jobs, not outstanding jobs; read them alongside
pending and in-flight counts. A zero-completion miss fraction is undefined,
not 0%. IMU stale observations are now counted at job start as for other stages;
the IMU still never drops these jobs. Vision CPU and lateness are separate fields.

**CPU boundary precision:** compute loops bracket thread-CPU clock reads with
monotonic reads. A sample wholly before the cutoff updates a lower CPU bound;
the first sample provably after the cutoff bounds the missing CPU above. There
is no proportional interpolation across preemption. `cpu_ns` (and truncated
`cpu_us`) in the window is the lower bound; `cpu_uncertainty_ns` exposes its
remaining interval. This includes in-flight job compute. CPU refers to synthetic
compute, not all hint/queue bookkeeping. Instrumentation is opt-in and enabled
identically for both probe variants; all release/deadline/freshness clocks stay
monotonic. Inspect uncertainty before treating differences as meaningful.

`drain_STAGE` / `drain.csv` hold post-cutoff completions, lateness and CPU, never
classifier inputs. Drain CPU is total minus the window lower bound, hence an
upper bound with the same uncertainty. Window plus drain CPU nanoseconds and
completion counts reconcile with totals. `measurement` records exact intended
epochs and elapsed/drain wall time through worker shutdown; process elapsed
also includes startup overhead and is reported separately.

LiDAR-registration and mapping get **separate** fixed-window rate ratios to the
starting 3% control for the same repetition and preemption variant. There is no
combined BE ratio: cheap mapping completions must not conceal lost registration
service. A zero-progress control has an undefined ratio.

The [historical notes](../archive/investigations.md#imu-load-diagnostics)
summarize the diagnostic conclusions that preceded this validation.
