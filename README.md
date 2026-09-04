# scx-slam-fresh
**Freshness-aware SLAM pipeline scheduling with `sched_ext` (eBPF).**

Robotics pipelines (state estimation / VIO / SLAM) are not just “real-time” in the classical sense. They’re freshness-driven:

- A camera frame processed 50ms late often becomes useless (or worse: harmful) for control.
- A mapping backend can run opportunistically, but should never starve the front-end.
- When overload happens, the system should **degrade gracefully**: prioritize the most recent sensor data and stop wasting CPU on stale work.

This project implements an experimental **`sched_ext` BPF scheduler** that lets a robotics pipeline tell the kernel what matters:
deadlines, data age (freshness), and per-job compute budgets — then the scheduler enforces that policy in the CPU runqueue.

This is by no means production ready, rather a personal experiment.

---

## What you get
- A sched_ext scheduler: **`scx_slam_fresh`**
  - **EDF-like** ordering for front-end tasks using DSQ vtime ordering
  - **Budget enforcement** (job overrun demotion)
  - **Stale-work demotion** (freshness threshold)
  - **Best-effort fairness** for background tasks (CFS-like vtime)
- A tiny user-space loader/monitor: **`scx_slam_fresh_user`**
  - loads & attaches the BPF scheduler (struct_ops link)
  - pins maps under a directory
  - prints scheduling events from a ring buffer
- An instrumentation library: **`libslamqos`**
  - pipeline threads publish job hints (deadline, freshness window, budget)
- A runnable demo: **`slam_pipeline_demo`**
  - a synthetic VIO/SLAM-like pipeline (front-end + backend + a CPU hog)
  - generates repeatable overload so you can see the scheduler’s effect

---

## Kernel / feature requirements
- Upstream Linux **6.12+** with `sched_ext` support.
- Kernel config must include `CONFIG_SCHED_CLASS_EXT=y` (and optionally `CONFIG_SCHED_CLASS_EXT_DEBUG=y` for nicer diagnostics).
- Root (or the right privileges) to load BPF and attach a scheduler.

---

## Safety model
By default, this scheduler is built in **partial switch** mode:
only tasks running under the `SCHED_EXT` policy are scheduled by sched_ext; normal tasks keep running under CFS.

- This dramatically reduces accidental machine freeze risk while iterating.
- If anything goes wrong, the kernel can abort the BPF scheduler and revert tasks to CFS.


---

## Build
### Dependencies
You need typical BPF build tooling:
- clang/llvm
- bpftool
- libbpf development headers
- kernel BTF available at `/sys/kernel/btf/vmlinux`

On Debian/Ubuntu-ish systems this is usually something like:
- `clang`, `llvm`, `bpftool`, `libbpf-dev`, `build-essential`, `pkg-config`, `zlib1g-dev`, `libelf-dev`

### Compile everything
```bash
make
```

What `make` does:
1) generates `build/vmlinux.h` from BTF  
2) compiles `bpf/scx_slam_fresh.bpf.c` to `build/scx_slam_fresh.bpf.o`  
3) generates a libbpf skeleton header `build/scx_slam_fresh.skel.h`  
4) builds the user-space loader and the demo

Check the embedded scheduler mode without root or BPF attachment:
```bash
./build/scx_slam_fresh_user --print-ops-flags
make test-scheduler-mode
```

The default build must report `0x8` (`SCX_OPS_SWITCH_PARTIAL`), while an explicit
full-switch build reports `0x0`. The evaluation runners reject embedded
full-switch flags before attaching SCX. When changing `SLAM_FULL_SWITCH`, use a
clean build or a separate `BUILD_DIR`; Make does not track command-line flag changes.

### Optional ROS 2 workspace

ROS 2 is an opt-in integration and is not part of the default build. Cloning
the repository and running `make` or the standalone E0-E4 simulations does not
require ROS 2, Ament, or Colcon.

With ROS 2 Lyrical installed, build and test the integration explicitly:

```bash
make ros2
make test-ros2
```

This uses an isolated Colcon workspace under `.ros2-*`; it does not write into
the standalone `build/` directory. See [ros2/README.md](ros2/README.md) for the
package boundaries and current implementation status.

---

## Run
### 1) Start the scheduler (loader)
```bash
sudo ./build/scx_slam_fresh_user --pin /sys/fs/bpf/scx_slam_fresh
```

This pins (at least) the following maps:
- `task_hints` (written by `libslamqos`)
- `events` ring buffer

### 2) Run the demo pipeline

**Note:** The demo uses *wake-safe hinting*. A producer publishes the head item's hint only when waking a sleeping consumer; after `pop`, the consumer republishes the exact item it is processing. This gives sched_ext correct wake-up metadata without overwriting an in-flight job's hint when a backlog forms.

### Demo knobs
- `--lidar off|light|mid|heavy` enables a LiDAR stream (10Hz) with bursty compute
- `heavy` is intentionally unsustainable and creates a LiDAR-registration backlog
- `--hog N` adds CPU contention
- `--imu-work-us N` sets IMU compute CPU time per job (default `150` µs,
  or 3% of the fixed 5 ms period). `0` disables synthetic IMU compute, not
  releases or hint publication; values at or above `5000` permit overload tests.
  This does not change the IMU deadline, budget hint, or scheduler policy.
  The IMU worker finishes all releases scheduled within `--duration`, so
  overloaded runs can take longer than that wall-clock duration to exit.
- `--drop-stale 1` rejects expired dequeued jobs and lets producers evict
  expired queued backlog before enqueueing new work
- `--camera-burst-count N` delays `N` camera frames and releases them together
  with their original timestamps
- `--camera-burst-at-ms N` selects the approximate burst delivery offset
- `--vision-budget-us N` sets the vision front-end CPU budget
- `--vision-work-us N` selects a fixed vision compute cost; `0` keeps the
  default 3-5ms pattern
- `--vision-deadline-us N` sets the vision relative deadline
- `--duration S` controls run length
- `--window-stats` separates metrics at the common monotonic cutoff from
  post-window drain. E4 enables it automatically; ordinary runs keep legacy totals.

The results' `configuration` line records `imu_work_us`. For example, a
non-root smoke run with 30% nominal IMU utilization is:

```bash
./build/slam_pipeline_demo --no-hints --lidar off --hog 0 --duration 2 --imu-work-us 1500
```

Run the non-root CLI and IMU CPU-accounting regression tests with `make test-demo test-window`
(requires Python 3 and `taskset`). This is not the E4 isolation sweep.


#### Option A: partial mode
In partial mode, only `SCHED_EXT` tasks are controlled by this scheduler.
So the demo must move its pipeline threads into `SCHED_EXT`.

Because libc headers for `SCHED_EXT` may lag, the demo takes a numeric policy via CLI:
```bash
# Find the SCHED_EXT policy number from your system headers (or kernel uapi).
sudo ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --ext-policy <N>
```

#### Option B: full switch mode
This makes sched_ext schedule *everything* (not just SCHED_EXT tasks).

Build and run:
```bash
make clean
make SLAM_FULL_SWITCH=1
sudo ./build/scx_slam_fresh_user --pin /sys/fs/bpf/scx_slam_fresh
sudo ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh
```

---

## Evaluation (single-core overload)
The harness first sweeps LiDAR light, mid, and heavy under identical single-core
CFS conditions and asserts sustainable, borderline, and overloaded registration
regimes. Synthetic compute consumes the requested per-thread CPU time; release,
deadline, and freshness timestamps remain on `CLOCK_MONOTONIC`. The harness then
runs four matched single-core workloads. The deadline-isolation
pair uses LiDAR heavy (300k pts @10Hz), camera (30Hz), IMU (200Hz), and two CPU
hogs to compare CFS with scx_slam_fresh. The stale-shedding pair uses the same
sensor load under scx_slam_fresh with zero hogs, comparing stale retention
against `--drop-stale 1`. Zero hogs leaves enough back-end progress for stale
shedding to be observable. The burst-recovery pair delays 12 camera frames, then
verifies that stale dropping preserves the newest frame while reducing obsolete
work and newest-frame completion age. The budget pair compares correctly sized
and undersized vision budgets. Results depend on the kernel, hardware, and Git
revision; the benchmark script records all three.

Commands:
```bash
taskset -c 0 ./build/slam_pipeline_demo --no-hints --lidar heavy --hog 2 --duration 15
sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar heavy --hog 2 --duration 15 --ext-policy 7
sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar heavy --hog 0 --duration 15 --ext-policy 7
sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar heavy --hog 0 --duration 15 --ext-policy 7 --drop-stale 1
sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar off --hog 0 --duration 15 --ext-policy 7 --camera-burst-count 12 --camera-burst-at-ms 3000
sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar off --hog 0 --duration 15 --ext-policy 7 --camera-burst-count 12 --camera-burst-at-ms 3000 --drop-stale 1
```

Run the complete matrix, capture its environment, and save each run's output with:
```bash
sudo scripts/run_single_core_eval.sh
```

Run only the E0 LiDAR calibration sweep with:
```bash
sudo env EVAL_SCOPE=e0 scripts/run_single_core_eval.sh
```

Run only the focused E3 budget-misconfiguration pair with:
```bash
sudo env EVAL_SCOPE=e3 scripts/run_single_core_eval.sh
```

Reproduce the validated E4 IMU-cost grid with:
```bash
make test-e4
sudo python3 scripts/run_e4_eval.py \
  --costs 150,500,750,1000,3000,3250,3500,4750,5000 \
  --repetitions 3 \
  --seed 17
```

The grid uses heavy LiDAR, one pinned CPU, no hogs, stale dropping off, fixed
15-second windows, and bracketing 3% controls. On the validated kernel/pin,
critical stages remain healthy through 65%, registration service declines
monotonically, estimator backlog begins at 70%, and IMU saturation appears at
95–100%. Heavy-LiDAR registration is already unsustainable in the control, so
this is not a whole-pipeline safe band. See the
[validated E4 regimes](docs/DESIGN_EVALUATION.md#validated-observational-regimes)
for exact definitions, results, limitations, and the retained diagnostic history.
Opt-in preemption, execution, perf, grace, budget, and class probes remain in the
harness for reproduction; they are not current next steps and change no defaults.

E1–E3 were revalidated in three repetitions with the corrected partial-switch
build on kernel `7.0.0-30-generic`, CPU 0, and commit `749b579`. The run kept
state-estimator misses at 0% under scx_slam_fresh versus 57.9–59.3% under CFS,
reduced stale LiDAR-registration pending work from 122 to 3, reduced newest
burst-frame state-estimate age from about 109ms to 14ms, and reproduced the
vision budget-overrun/demotion control. See the
[replacement E0–E3 snapshot](docs/DESIGN_EVALUATION.md#replacement-verification-snapshot--partial-switch)
for exact per-repetition values, binary hashes, and scope. Compare:
- `imu_prop`, `vision_fe`, and `state_est` deadline-miss percentages
- per-stage `cpu_us`, which is accumulated with `CLOCK_THREAD_CPUTIME_ID`
- `lidar_reg` and `mapping_be` consumer-dropped and queue-evicted stale counts
- pending backlogs in the matched stale-keeping and stale-dropping runs
- work completed by stale back-end stages with and without `--drop-stale 1`
- burst frames processed, newest burst sequence, and newest-frame completion age
- vision budget-overrun and budget-demotion events under the E3 workload

Known limitation: the dedicated IMU lane has strict dispatch priority and can starve lower lanes if IMU utilization is misconfigured. The overload matrix is intended to quantify that tradeoff.

---

## Design docs
Start here:
- `docs/DESIGN.md`
- `docs/DESIGN_SCHED_ALGO.md`
- `docs/DESIGN_HINTS_API.md`
- `docs/DESIGN_EVALUATION.md`

---

## References
- Linux kernel documentation: `Documentation/scheduler/sched-ext.rst` / `docs.kernel.org/scheduler/sched-ext.html`
- Reference schedulers and tooling: https://github.com/sched-ext/scx
- eBPF docs (kfunc references): https://docs.ebpf.io/
