# Userspace hints API

`libslamqos` writes scheduling metadata to the pinned `task_hints` BPF map.
Each key is a worker identity, packed as `(tgid << 32) | tid`. Each value
describes one selected job.

> Userspace selects work. sched_ext consumes metadata about the work already
> selected for a worker.

## Functions

In the external `scx_fresh` checkout, declarations are in `src/slamqos.h`; the shared layout is
`struct slam_task_hint` in `include/scx_slam_fresh_shared.h`.

| Function | Use |
| --- | --- |
| `slamqos_pid_tgid_self()` | Return the current worker's packed identity. |
| `slamqos_open(q, pin_dir)` | Open the pinned hint map into a caller-owned handle. |
| `slamqos_close(q)` | Close the map descriptor. |
| `slamqos_publish_hint(q, hint)` | Replace the current thread's hint. |
| `slamqos_publish_hint_for(q, worker, hint)` | Replace an explicitly identified worker's hint. |
| `slamqos_publish_job(...)` / `slamqos_publish_job_for(...)` | Build and publish a hint from individual fields. |
| `slamqos_clear_hint(q)` | Write a MISC/BE hint with job ID zero for the current thread. |

Use the full hint structure when setting flags such as
`SLAM_HINT_EXECUTOR_OWNED`. A clear is a map update, not an ownership or
cancellation operation; an executor must establish that clearing is safe.

## Fields

| Field | Meaning |
| --- | --- |
| `api_version` | Set to `SLAM_SCX_API_VERSION`. |
| `stage_id` | Application stage; the IMU stage selects the dedicated route. |
| `class_id` | FE or BE. |
| `flags` | Ownership flags; zero unless the client implements their contract. |
| `job_id` | Stable for one job; change it when selecting new work. |
| `release_ts_ns` | Monotonic release time. |
| `deadline_ts_ns` | Absolute monotonic deadline, or zero for none. |
| `stale_ns` | Maximum useful age, or zero to disable the stale threshold. |
| `budget_ns` | Execution budget, or zero to disable budget enforcement. |
| `slice_ns` | Requested insertion slice; zero selects the scheduler default. |
| `weight` | Reserved fairness parameter; current BPF runtime accounting does not apply it. |

For nonzero deadlines, use a value at or after release. Scheduler timestamps
use `CLOCK_MONOTONIC`, corresponding to BPF `bpf_ktime_get_ns()`. Recorded ROS
header stamps are not in this timebase and must not be sent as kernel deadlines.

## Executor contract

The single slot per worker makes publication ordering part of correctness.

### Assign, publish, wake

For a sleeping worker:

1. Select a job and bind it to that worker.
2. Publish its identity and scheduling fields into the worker's slot.
3. Wake the worker.

Publishing only after the worker starts running is too late to classify its
wakeup. A worker selecting another job without sleeping must publish after
selection and before compute.

### Ownership and eviction

A producer or eviction path must not overwrite an executing job's hint.
Only the worker may replace or clear it, unless the executor has independently
established completion.

Do not publish work evicted before assignment. If the selected head is removed
before a planned wake, publish the replacement first or do not wake the worker.
A pre-start thief publishes the stolen job onto its own slot before execution.

Once compute begins, the job stays with that worker until completion or
worker-controlled cancellation. Budget state is keyed by worker, so moving
a running job would reset accounting and could grant it a second FE budget.
Mid-job migration requires state keyed by at least stage and job identity.

### ROS message lifecycle

Each `FreshnessExecutor` instance has a dispatcher and one callback worker.
The dispatcher selects an `rclcpp::AnyExecutable` only when the worker is idle.
For message-aware subscriptions, it takes one normal message directly from DDS,
extracts its job metadata, and hands that same message to the worker.

With `reject_expired` enabled, expiry is checked twice: before hint publication
and wake, then after handoff before callback entry. Both use the raw deadline
and stale window without grace. A rejected message is returned, its callback
group is released, and the observer receives `DroppedBeforeStart`. This never
cancels an already-running callback.

Accepted non-IMU work in this recovery path carries
`SLAM_HINT_EXECUTOR_OWNED`. BPF age demotion cannot revoke its service before
it reaches the recheck, completion, or parking path. Normal class routing and
budget demotion still apply. Real IMU profiles remain unflagged and retain
the scheduler's IMU exemption.

The completed job's hint remains through parking. Once completion is
established, the dispatcher directly replaces it with the next selected job.
An intermediate MISC/BE clear could strand the worker's completion tail under
BE contention. The dispatcher clears the final slot at shutdown; callback
failure clears it on the worker. Retaining identity does not create a queued
job or authorize kernel-side selection.

Serialized, dynamic, and intra-process message delivery are outside the
message-aware path. The current worker does not steal or migrate callbacks.

### Standalone FIFO lifecycle

Each queue has one consumer. Producers publish the head item's hint only when
waking a sleeping consumer; after `pop()`, the consumer republishes the exact
selected item. Busy consumers' slots are not overwritten by later arrivals.
This is the standalone instance of the same ownership contract.

## Release time and bag identity

Ordinary callback groups use callback selection as release time. Message-aware
groups use the monotonic release carried by their message.

The bag adapter samples monotonic time when it takes a sensor message and
stores it in `release_ts_ns`. It preserves the recorded header stamp separately
as `source_ts_ns`; offline bag ordinals supply stable job IDs. Downstream
camera jobs retain both identity and release time, sharing the camera deadline.

Unannotated SCHED_EXT tasks receive BE fallback service. See the
[scheduler reference](DESIGN_SCHED_ALGO.md) for default slices, age rules,
budget enforcement, and the optional BE cap.
