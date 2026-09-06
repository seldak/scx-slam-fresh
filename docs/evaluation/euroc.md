# EuRoC bag-backed evaluation

[Evaluation overview](../DESIGN_EVALUATION.md) · [Run the harness](../../ros2/README.md#bag-evaluation)

On this EuRoC synthetic graph, two EXT hogs, cap=2ms: the IMU bundle keeps
200Hz on time; the cap makes the shared 33ms chain completable; downstream
FE hints cut estimator completion age by approximately 5–8ms at p99. Hog
iteration count does not materially move between those hint variants.

Replay supplies sensor timing and identity; callback compute is synthetic.
These results do not measure SLAM accuracy. The cap remains default-off.

The evidence is presented in order: recovery and the uncapped baseline, the
2ms intervention, then hint ablations under the cap.

## Uncapped baseline and recovery gate

The uncapped hog2 run on 2026-09-05 is the preserved baseline.
This run used clean revision `1e1bf87`, kernel
`7.0.0-31-generic`, CPU 14 for workers and two EXT hogs, CPU 1 for
housekeeping, partial switch (`ops_flags=0x8`), and 1ms deadline grace.
The SMT sibling CPU 15 remained unused. There was no FE wakeup preemption;
BE/unhinted insertion used the kernel default 20ms slice.

EuRoC MH_01_easy used source epoch `1403636579758555500`, 3s warmup and
a 15s measured source window. IMU IDs 601–3600 cover source timestamps
`1403636582758555500` through `1403636597753555500`; camera IDs 61–360
cover `1403636582763555500` through `1403636597713555500`. The downstream
stages retain the camera identities and original release timestamps.
The MCAP SHA256 is
`35ddb7974deabba3b1885eeff84bf9d6f49cdb20014b4aab6bd34d25239d9ae7`.

Three CFS repetitions completed, followed by one hinted repetition. The
harness rejected hinted-1 because mapping had one callback in flight at the
cutoff; hinted-2 and hinted-3 did not run. The failed case remains in the comparison; the gate requires
`unfinished == 0`.

| Metric | CFS repetitions 1, 2, 3 | Hinted repetition 1 |
| --- | --- | --- |
| IMU completed / late | 3000/630, 3000/657, 3000/597 | 3000/0 |
| IMU p99 start age | 16.280, 17.010, 15.731ms | 1.627ms |
| Vision completed / late | 300/0 each | 300/0 |
| Vision p99 start age | 5.349, 4.651, 5.167ms | 19.629ms |
| Estimator completed / late | 300/0 each | 194/75 |
| Estimator dropped before start | 0 each | 106 |
| Estimator timely / offered | 300/300 each | 119/300 (39.7%) |
| Mapping completed | 300 each | 193 |
| Mapping upstream drops / in flight | 0/0 each | 106/1 |

Source windows match and adapters reported zero drops. Mapping accounts for
all 300 source opportunities as 193 completed + 106 upstream drops + 1 in
flight. Estimator progressed through job 360. This is a cutoff failure under
load, not a recurrence of the permanent STALE ownership lock. Completion-age
percentiles exclude dropped and unfinished work.

The separate hog0 recovery gate on 2026-09-05 passed three hinted repetitions
using `875daa1` plus the bag-evaluation changes later committed as `1e1bf87`:
IMU 3000/0 late and
each downstream stage 300/0 late, with zero drops or unfinished work. That
validated recovery on the pinned input; it did not validate camera-chain
latency under hog2. The camera chain retains its shared 33ms source-relative
deadline. The uncapped policy protects IMU while losing camera timeliness.

Baseline binaries (SHA256):

- Pipeline: `31bea141b6a6e67f4702fafb17e82bd394593edbb4b05035e071fb929d3b3001`
- Adapter: `74fc0cffbed340627354dd8ba4f53a211867b2c5afb4c021bfb13f0939398181`
- Loader: `d22bb22f77056a90f1a17158b2c570fb4346f35b70f0ec094c1f2a59094ce210`
- BPF object: `341fab0a81e63aae509fa3faca5228e2de290f95dfcb85f63ae2e75d0002f68b`

## BE slice cap comparison

The capped hog2 capture on 2026-09-05 used
`BE_SLICE_CAP_US=2000 HINTED_ONLY=1 REPETITIONS=3 HOG_THREADS=2` and
the preserved uncapped hog2 baseline for source-window comparison.
It ran on `1e1bf87` plus the uncommitted cap experiment. The recorded loader
configuration confirms `be_slice_cap_us=2000`, `imu_preempt=wakeup`,
`deadline_grace_us=1000`, and `ops_flags=0x8`. The cap defaults to disabled.

Only BE-class/unhinted insertion slices are capped, including mapping and
the hogs. IMU and FE insertion slices (including budget-demoted FE), wakeup
preemption, source-relative deadlines, budgets, and the strict harness gate
remain unchanged. The pipeline and adapter binaries match the uncapped
baseline byte for byte. CFS was not rerun.

| Metric | Uncapped hinted baseline (one repetition) | 2ms cap (three repetitions) |
| --- | --- | --- |
| IMU completed / late | 3000/0 | 3000/0 each |
| IMU p99 start age | 1.627ms | 1.656–1.800ms |
| Vision completed / late | 300/0 | 300/0 each |
| Vision p99 start age | 19.629ms | 3.192–3.200ms |
| Estimator timely / offered | 119/300 (39.7%) | 300/300 (100%) each |
| Estimator completed / late / dropped | 194/75/106 | 300/0/0 each |
| Estimator p99 completion age | 35.742ms | 14.158–15.096ms |
| Mapping completed / upstream drops / unfinished | 193/106/1 | 300/0/0 each |
| Mapping p99 completion age | 64.613ms | 19.377–20.094ms |

Every capped case matched the baseline source identities and timestamps,
with zero adapter drops and no unfinished work. Ranges above describe the
three per-run p99 values, not a pooled percentile. The uncapped completion
percentiles are conditional on completion and exclude its drops/unfinished
work; timely/offered supplies the uncensored comparison.

With the shared camera deadline, timely
estimator completions reach the existing CFS baseline's 300/300 without
reintroducing IMU lateness. It supports long BE service intervals as a cause
of the multi-hop latency loss in this workload. It does not separately
measure DDS, dispatcher, and runnable-wait contributions or establish a
worst-case latency bound. No FE preemption or deadline rewrite was added.
The cap-enabled hog0 check on 2026-09-05 passed one hinted repetition at clean
revision `f8aea1a` before the documentation-only validation amendment.
It used the same 15s source window, `BE_SLICE_CAP_US=2000`, and identical
pipeline, adapter, loader, and BPF binaries to the capped hog2 capture.
IMU completed 3000/3000 and all downstream stages completed 300/300, with
zero late, dropped, or unfinished work and zero adapter drops. IMU p99
start age was 1.794ms; estimator p99 completion age was 10.343ms.
This confirms completion and timeliness without hogs for the same bag window.
Only the 2ms cap was evaluated; there was no FIFO comparison.

Capped scheduler binaries (SHA256):

- Loader: `57b7102c92472b801cca8d4cb7ab4b996a6900e51d9ed66ef918d02fb2ec9db0`
- BPF object: `7c1e55f9ee7b959bc261509db433e98aace56c758b125357e314f19b42e757b6`

## Hint ablation under the cap

The scheduler is frozen at the validated default-off BE-cap implementation.
The matrix contains three hinted cells on the same EuRoC window, with
two hogs and a 2ms cap throughout: full hints (control), `imu-only` (A), and
`fe-only` (B). No CFS, additional cap value, or new scheduler mechanism is
part of this matrix.

The 2026-09-06 run completed three repetitions per cell, using source epoch
`1403636579758555500`, a 15s measured interval after 3s warmup, and identical
binaries across all nine cases. Each case covered the same 3000 IMU and 300
camera source opportunities, with zero adapter drops and zero unfinished work.
Counts below sum three repetitions; p99 ranges are per-run percentiles, not
pooled percentiles. Hog iterations are the combined mean per 15s window.

| Metric | Full control | A: downstream MISC/BE | B: ordinary FE IMU |
| --- | ---: | ---: | ---: |
| IMU late / 9000 | 0 | 0 | 7370 |
| IMU p99 start age | 1.59–1.77ms | 1.60–1.64ms | 27.76–29.13ms |
| Estimator timely / 900 | 900 | 899; one dropped | 900 |
| Estimator p99 completion age | 14.54–14.99ms | 20.02–22.97ms | 12.83–13.12ms |
| Mapping completed / 900 | 900 | 899; one upstream drop | 900 |
| Combined hog iterations, mean | 11940 | 11942.3 | 11940.3 |
| Strict gates passed | 3/3 | 3/3 | 0/3 |

The cap comparison comes from the preceding uncapped/capped experiment;
all cells in this matrix use the cap.

A shows downstream FE is a latency refinement, not a requirement for nearly
all camera-chain completions here. It does not separate FE class priority,
EDF ordering, and budget effects. B fails solely on IMU lateness; accounting
is complete. Its lower camera-chain ages are consistent with removing IMU
preemption interruptions, but B removes the whole IMU queue/preemption bundle.
Mapping CPU time is approximately 0.601s per window, except A repetition 1
at 0.599s following the upstream drop. Full repetition 3 retains a completed
mapping tail with maximum start age 56.353ms and completion age 62.966ms;
this tail remains part of the recorded result.

### Ablation provenance and method

The instrumented pipeline SHA256 is
`5c72c3f203f82ea2e11675cd959e52206722d0cea0762eec5b959a21448ada4f`.
The adapter and capped scheduler hashes are unchanged from the preceding
experiment. The cap remains default-off; no scheduler policy changed for
this matrix.

Bag-mode admission always uses the original application stage, deadline,
stale window, and budget. A transport projection changes only the published
BPF hint after selection. A projects downstream hints to MISC/BE with no
deadline, stale window, or budget, retaining identity and executor ownership.
B projects IMU to MISC/FE, retaining its deadline and budget. The existing
executor-owned flag preserves its age-demotion exemption when the dedicated
IMU stage is removed; otherwise B would also retest the old STALE lock.
The real IMU admission profile remains unchanged and exempt from dropping.
B removes the dedicated queue/preemption bundle, not just one of its parts.

Each hog records completion timestamps for its unchanged 1000us CPU-work
iterations. `hog_window` counts recorded completions in the same half-open
wall interval used for the measured stage cutoff. An iteration crossing the
start counts if it finishes inside; one finishing at/after the end does not.
This is throughput, not an exact CPU-time measurement. Mapping retains its
existing source-window CPU accounting, including an in-flight compute sample.
All three cells, including the control, use the same newly instrumented
workload binary. The prior uncapped run has no hog iteration counters, so
this matrix alone cannot quantify cap overhead relative to that run.

The ablation driver preserves each case's strict gate status and continues
after a recorded performance/accounting rejection to collect the matrix.
Infrastructure and adapter-input failures stop the run. All cells must have
matching source windows, configuration, and binaries. Any rejected gate
leaves the matrix with a nonzero exit status.
