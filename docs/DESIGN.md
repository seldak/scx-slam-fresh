# DESIGN: scx-slam-fresh

## Problem framing
Classical real-time scheduling focuses on meeting deadlines for periodic tasks.
Robotics SLAM pipelines need something slightly different:

1) **Front-end tasks are freshness-critical.**  
   If you track features on an old frame, you can inject bad state.

2) **Back-end tasks are important but elastic.**  
   Loop closure and global optimization can lag.

3) Overload is inevitable on embedded SoCs.  
   The right behavior under overload is not “let everything slow down”, but:
   - keep the control-relevant state estimate current
   - shed/demote stale work
   - cap how much CPU one job can burn

`sched_ext` lets us encode this at the kernel scheduler boundary.

---

## Design goals
- Prioritize *fresh* sensor-derived work over stale work.
- Provide EDF-like behavior for front-end stages.
- Enforce per-job compute budgets with demotion (prevent pipeline collapse).
- Keep a best-effort fairness story for background tasks.
- Provide observability (events, counters) suitable for evaluation writeups.

## Non-goals
- Hard real-time guarantees equivalent to PREEMPT_RT + SCHED_FIFO.
- Vendor-specific or big.LITTLE-specific hacks.
- Per-packet NIC scheduling or GPU scheduling.

---

## Architecture

```mermaid
flowchart LR
  subgraph App["Robotics app (ROS2 / custom pipeline)"]
    FE1["Front-end thread(s)\n(IMU, tracking, VIO)"]
    BE1["Back-end thread(s)\n(mapping, loop closure)"]
    QOS["libslamqos\n(hint publisher)"]
    FE1 --> QOS
    BE1 --> QOS
  end

  subgraph Kernel["Linux kernel"]
    SCX["sched_ext BPF scheduler\n(scx_slam_fresh.bpf)"]
    DSQFE["DSQ_FE\n(vtime=effective deadline)"]
    DSQBE["DSQ_BE\n(vtime=vruntime)"]
    DSQST["DSQ_STALE\n(low priority)"]
    SCX --> DSQFE
    SCX --> DSQBE
    SCX --> DSQST
  end

  subgraph User["Userspace control/monitor"]
    Loader["scx_slam_fresh_user\n(load/attach, pin maps)"]
    Events["ringbuf\n(deadline miss, budget overrun, stale)"]
    Loader <--> Events
  end

  QOS -->|BPF map updates| SCX
  SCX -->|ringbuf events| Events
```

---

## Data model

### Per-task hint (written by userspace)
Each pipeline thread (or an executor thread pool) periodically publishes:

- **job_id**: monotonically increasing work item identifier
- **release_ts**: when data became ready
- **deadline_ts**: when the output stops being useful (absolute time)
- **stale_ns**: age threshold beyond which we demote the work
- **budget_ns**: compute budget for this job (demote on overrun)
- **slice_ns**: preferred time slice for responsiveness
- **class**: front-end vs back-end

### Per-task state (maintained by BPF)
- `last_start_ns`, `exec_ns_in_job`, `vruntime`
- per-job “already reported” fields to avoid spamming events

---

## DSQ strategy (why 3 queues)
We use 3 custom DSQs:

- DSQ_FE: vtime = effective deadline ⇒ EDF-like ordering  
- DSQ_BE: vtime = vruntime ⇒ CFS-like fairness among BE tasks  
- DSQ_STALE: for tasks past their freshness window (very low priority)

---

## Failure handling & safety
- We build with partial switch by default, so only `SCHED_EXT` tasks are controlled.
- If the scheduler stalls runnable tasks, or errors occur, the kernel can abort the BPF scheduler and revert tasks to CFS.

---

## Integration with real robots
The demo uses a synthetic pipeline, but the integration model maps cleanly to:
- ROS2 executors (per-callback-group threads)
- custom pipelines with lock-free queues
- sensor fusion / VIO with periodic IMU and camera updates

The only required integration is publishing hints (a small library call) at job start.
