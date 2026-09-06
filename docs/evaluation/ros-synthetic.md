# ROS 2 synthetic callback evaluation

[Evaluation overview](../DESIGN_EVALUATION.md)

Historical results from revision `f01e3f9`. These runs use periodic synthetic
sources, not the later EuRoC replay or its executor recovery implementation.

## Loaded callback results

A clean three-repetition run used kernel `7.0.0-31-generic`, CPU 0 for callback
workers and two CPU hogs,
CPU 1 for DDS/RMW, publishers, and executor dispatch, 15-second fixed windows,
ROS 2 Lyrical, `ops_flags=0x8`, and `switch_all=0`. The source revision was
`f01e3f91c519325e5e06f0d42fd0a08bb872cc85`; scheduler policy last changed in
`0252acc`. The captured binaries were:

- ROS pipeline: `38a15888215e17aced3a0c8ffe99932bd0c61adb2ca0f35c77d63ec5293920e5`
- loader: `3dd5519925781964d451c1039017404169d8384deb835bce811e95ddca35243f`
- BPF object: `75719c3ac6959b41e80fdeb4975538aa28b9bb65eadfa42eedc6e858d42a1860`

The matched command selected CFS plus all four SCX metadata variants:

```bash
sudo env CPU=0 HOUSEKEEPING_CPU=1 DURATION=15 HOG_THREADS=2 \
  REPETITIONS=3 SCX_VARIANTS=hinted,no-hints,imu-only,fe-only \
  scripts/run_ros2_eval.sh
```

The IMU result isolates throughput and period survival. Values below are the
three repetitions; p99 columns give the range across those repetitions.

| Policy | IMU completed | late | unfinished | p99 start age | p99 completion age |
| --- | --- | --- | --- | ---: | ---: |
| CFS | 3000, 3000, 3000 | 1, 0, 1 | 0, 0, 0 | 2.897-2.901ms | 3.049-3.053ms |
| hinted | 3000, 3000, 3000 | 0, 0, 0 | 0, 0, 0 | 0.114-0.242ms | 0.266-0.398ms |
| no hints | 1876, 1878, 1878 | 1874, 1875, 1876 | 1124, 1122, 1122 | 5013.703-5014.427ms | 5013.856-5014.580ms |
| IMU only | 3000, 3000, 3000 | 0, 0, 0 | 0, 0, 0 | 0.136-0.149ms | 0.288-0.301ms |
| FE only | 3, 4, 4 | 1, 1, 1 | 2997, 2996, 2996 | 18.122-20.372ms | 18.274-20.525ms |

The other stages completed 450 jobs per repetition except FE-only mapping,
which completed 449. Start and completion age remain separate: the table shows
the range of each run's p99, not a pooled percentile.

| Policy | Vision start / completion | Estimator start / completion | Mapping start / completion |
| --- | ---: | ---: | ---: |
| CFS | 5.385-5.557 / 16.650-17.058ms | 19.927-21.241 / 28.602-29.913ms | 30.244-31.381 / 34.276-35.522ms |
| hinted | 3.595-3.673 / 8.756-8.833ms | 13.590-13.612 / 16.593-16.620ms | 18.585-18.614 / 20.587-20.617ms |
| no hints | 20.861-20.948 / 25.199-25.458ms | 25.386-25.653 / 28.307-28.465ms | 28.494-29.023 / 30.497-31.026ms |
| IMU only | 3.601-3.608 / 8.762-8.774ms | 13.598-13.600 / 16.600-16.604ms | 18.595-18.597 / 20.598-20.600ms |
| FE only | 18.878-19.124 / 23.386-23.626ms | 43.862-44.123 / 46.625-46.876ms | 54.422-54.661 / 56.426-56.668ms |

On this one-core, two-hog ROS pipeline, hinted `scx_slam` cuts callback-start
tails versus CFS without dropping work. The same workers and hogs all use
`SCHED_EXT` in the no-hint case, but anonymous BE service completes only about
1877/3000 IMU callbacks and leaves about 1123 unfinished. SCHED_EXT membership
alone did not provide timely IMU service.

Full hinting and IMU-only hinting are indistinguishable here, including vision
and estimator tails. Once the dedicated IMU path is protected, FE metadata for
vision and estimation adds no measurable benefit on this graph and load.
Ordinary-FE IMU is not a substitute: it becomes stale around job 4 or 5, moves
to the stale lane, and completes only 3-4 callbacks. This reproduces the
late/stale-demotion feedback observed in the standalone IMU sweep, on this
earlier executor implementation.

The dedicated IMU path combines `DSQ_IMU` routing, wakeup preemption, and
age-demotion exemption; their effects were not separated. This result applies
to synthetic callback compute on the recorded configuration. The zero-hog
diagnosis below addresses CPU interference separately.

### Zero-hog partial-switch interference diagnosis

This diagnostic used the same `f01e3f9` pipeline, loader, and BPF binaries with
hinted partial-switch SCX, no synthetic hogs, 15-second fixed windows, and
three repetitions.

With callback workers pinned to an unshielded CPU 0 and DDS/dispatch on CPU 1,
standard `sched_switch` and `sched_wakeup` traces showed that CPU 1 woke the IMU
worker after its expected 4.5-4.9ms sleep. The IMU then remained runnable for
44-48ms while foreign `SCHED_NORMAL` browser or compositor workers ran on CPU
0. In partial-switch mode those fair-class tasks remain outside the SCX DSQs
and have higher scheduling-class precedence than `SCHED_EXT`; neither the IMU
lane nor SCX wakeup preemption can preempt them. Pinning this workload's worker
threads did not prevent unrelated processes from using the same CPU.

The matched shielded capture moved the workers to CPU 14, kept CPU 1 for
DDS/dispatch, and left the SMT sibling CPU 15 unused. Linux rejects sched_ext
registration with `isolcpus=domain`, so scheduler-domain isolation was not
used. The effective boot configuration instead retained tick, RCU, and IRQ
isolation while systemd kept its service tree off CPUs 14-15:

```text
isolcpus=managed_irq,14-15 nohz_full=14-15 rcu_nocbs=14-15 irqaffinity=0-13 systemd.cpu_affinity=0-13
```

The standard-perf runner for this diagnosis is:

```bash
sudo env CPU=14 HOUSEKEEPING_CPU=1 DURATION=15 REPETITIONS=3 \
  scripts/run_ros2_zero_hog_perf.sh
```

It captures switches, wakeups, forks, and migrations on the configured worker
and housekeeping CPUs. These events show runnable waits and execution
intervals, not requested or remaining sched_ext slices.

After reboot, `/sys/devices/system/cpu/nohz_full` reported `14-15` and PID 1's
`Cpus_allowed_list` reported `0-13`. The worker explicitly selected CPU 14.

| Capture | IMU completed / late / started stale | p99 start age | maximum start age |
| --- | --- | ---: | ---: |
| Unshielded CPU 0 | 3000 / 9-14 / 8-12 | 1.213-1.258ms | 44.402-47.936ms |
| Shielded CPU 14 | 3000 / 0 / 0 | 0.633-0.777ms | 1.167-2.532ms |

Each range covers three repetitions. Short kernel and desktop occupants still
appeared on CPU 14, so this is not an absolute reserved-core claim. They did
not reproduce the 40ms-class runnable wait or cause an IMU deadline miss. The
zero-hog tail is therefore closed as unshielded fair-class interference; no
scheduler-policy change was made.

---
