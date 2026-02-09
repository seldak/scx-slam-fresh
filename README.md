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

## Safety model (important)
By default, this scheduler is built in **partial switch** mode:
only tasks running under the `SCHED_EXT` policy are scheduled by sched_ext; normal tasks keep running under CFS.

- This dramatically reduces the “oops I froze my machine” risk while iterating.
- If anything goes wrong, the kernel can abort the BPF scheduler and revert tasks to CFS.

You can also build a **full switch** variant for benchmarking (see below).

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

**Note:** The demo uses *producer-driven hinting* (the thread that pushes a work item publishes the consumer's hint before waking it), so sched_ext sees correct metadata at wake-up.

### Demo knobs
- `--lidar off|light|mid|heavy` enables a LiDAR stream (10Hz) with bursty compute
- `--hog N` adds CPU contention
- `--drop-stale 1` skips compute for stale jobs (models backlog shedding)
- `--duration S` controls run length


#### Option A: partial mode (recommended)
In partial mode, only `SCHED_EXT` tasks are controlled by this scheduler.
So the demo must move its pipeline threads into `SCHED_EXT`.

Because libc headers for `SCHED_EXT` may lag, the demo takes a numeric policy via CLI:
```bash
# Find the SCHED_EXT policy number from your system headers (or kernel uapi).
sudo ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --ext-policy <N>
```

#### Option B: full switch mode (convenient for demos, riskier)
This makes sched_ext schedule *everything* (not just SCHED_EXT tasks).

Build and run:
```bash
make clean
make SLAM_FULL_SWITCH=1
sudo ./build/scx_slam_fresh_user --pin /sys/fs/bpf/scx_slam_fresh
sudo ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh
```

---

## Repo layout
```
.
├── bpf/                      # BPF scheduler (struct_ops)
├── include/                  # shared structs (BPF <-> user)
├── src/                      # loader + libslamqos
├── demo/                     # synthetic SLAM pipeline workload
├── docs/                     # design docs with mermaid diagrams
└── scripts/                  # helper scripts (vmlinux.h generation etc)
```

---

## Design docs
Start here:
- `docs/DESIGN.md`
- `docs/DESIGN_SCHED_ALGO.md`
- `docs/DESIGN_HINTS_API.md`
- `docs/DESIGN_OBSERVABILITY.md`
- `docs/DESIGN_EVALUATION.md`

---

## References
- Linux kernel documentation: `Documentation/scheduler/sched-ext.rst` / `docs.kernel.org/scheduler/sched-ext.html`
- Reference schedulers and tooling: https://github.com/sched-ext/scx
- eBPF docs (kfunc references): https://docs.ebpf.io/
