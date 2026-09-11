# Closing findings

This repository asked whether application-selected scheduling hints through
sched_ext improve timely, useful processing under contention compared with
existing Linux scheduling policies. **For the dependent workload tested, no
advantage over FIFO was demonstrated.** This does not establish that sched_ext
is generally ineffective.

The workloads are synthetic scheduling experiments, not SLAM implementations.
The dependent graph connects processed IMU and camera inputs to an estimator
worker and periodic control consumer. The older ROS graph is different: its
independent IMU callback does not feed the camera chain. Historical bag results
must not be presented as sensor-fusion results.

## Final RT comparison

The existing comparison runners were executed without changing
workload costs or retrying failed timings. CPU 14 ran workers, CPU 1 handled
housekeeping, and sibling CPU 15 stayed unused. The kernel was
`7.3.0-rc2-bpf-rt-investigation+`, with PREEMPT_RT and local timer patches.

The fixed EDF task set used 21 ms every 50 ms and 29.5 ms every 70 ms.
Each policy ran three times, with 24 jobs per run. SCX's Background server and
job budgets were disabled for this comparison.

| Policy | Total deadline misses, three runs |
| --- | ---: |
| FIFO, A higher priority | 6 |
| FIFO, B higher priority | 18 |
| Linux SCHED_DEADLINE | 0 |
| scx_fresh | 0 |

The saved 7.0.0-31-generic run with the same task costs had identical miss counts.
This demonstrates EDF-style ordering on the tested task set, not an RT improvement
or equivalence to Linux's CBS and admission guarantees.

The dependent graph ran ordinary scheduling, FIFO and hinted SCX, each with
three nominal and three estimator-burst repetitions (18 runs). SCX used a 2 ms
BE cap and a 2 ms / 10 ms Background server. The burst added 80 ms of estimator
CPU work; its total measured CPU work was approximately 82.3 ms.

| Burst outcome per run | FIFO | Hinted SCX |
| --- | ---: | ---: |
| Burst completion time | 87.16–87.17 ms | 202.93–240.67 ms |
| Stale control outputs | 9 | 21–25 |
| Dropped IMU inputs | 0 | 9–16 |
| Late control callbacks | 0 | 0 |

SCX let the hogs progress while the estimator was budget-demoted; FIFO completed
the estimate sooner. Timely control callbacks did not imply fresh inputs.
All lifecycle and accounting checks passed, and no new kernel warnings were
observed during these comparisons. The RT kernel did not eliminate the policy
tradeoff. Different kernel versions, debug configuration and scheduler changes
prevent attributing performance differences solely to PREEMPT_RT.

## Reproduction and scope

The [machine-readable summary](../../experiments/preempt-rt/summary.json)
retains run metadata, EDF miss counts and per-repetition burst metrics. The
scheduler revision in the metadata identifies the base commit; the runs also
included the uncommitted timer-coalescing fix documented in the scheduler repo.

From the application checkout, with a matching sibling scheduler checkout:

```bash
make all build/edf_workload build/dependent_workload test-edf
sudo python3 scripts/test_edf_loaded.py --cpu 14 --housekeeping-cpu 1
sudo python3 scripts/test_dependent_loaded.py --cpu 14 --housekeeping-cpu 1
```

The runners retain per-job measurements, configuration, revisions and scheduler
diffs under ignored `results/` directories. Reproduction generates new evidence;
it does not reproduce identical timings. Preserve original run artifacts when
publishing measurements. No datasets, perf recordings, keys or kernel build
outputs belong in Git.

The [scheduler RT notes](https://github.com/seldak/scx_fresh/tree/main/experiments/preempt-rt)
contain the experimental kernel patches and their validation limits. That work
is separate from the application conclusion and is not upstream-ready enablement.

Feature development is paused. No scx_layered, further policy matrix or real
estimator integration is needed to support this bounded conclusion.
