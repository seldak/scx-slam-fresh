# Scheduler rules

This describes the current default policy. Userspace owns job selection;
the scheduler routes the thread named by its hint.

## Routing at enqueue

Rules are applied in this order:

| Condition | Destination |
| --- | --- |
| IMU stage, wakeup enqueue | CPU-local DSQ with `SCX_ENQ_PREEMPT` |
| IMU stage, other enqueue | `DSQ_IMU`, ordered by effective deadline |
| Non-IMU, unowned hint, stale or beyond deadline grace | `DSQ_STALE` |
| FE class, no budget overrun | `DSQ_FE`, ordered by effective deadline |
| BE class, unhinted task, or FE budget overrun | `DSQ_BE` |

In `dispatch`, at most one task moves to the local DSQ, in this order:

```text
IMU -> FE -> BE -> STALE
```

Moving one task avoids queuing a batch of lower-priority tasks ahead of a later
arrival. Only the default IMU wakeup path preempts. FE arrival does not shorten
a running BE slice.

## Age and effective deadlines

Age is `max(now - release_ts_ns, 0)`. A future release therefore does not
underflow. For an unowned non-IMU hint:

- A nonzero stale window and release timestamp enable stale demotion when
  age exceeds `stale_ns`.
- A nonzero deadline enables late demotion when elapsed time beyond the
  deadline exceeds `deadline_grace_ns`, which defaults to 1 ms.

IMU is exempt from both checks. `SLAM_HINT_EXECUTOR_OWNED` exempts a selected
non-IMU owner from age demotion so it can reach its recheck and completion
path. The executor performs its own expiry checks without grace. The flag
does not disable budget demotion or give BPF permission to select backlog.

Within IMU and FE queues, the effective deadline is the earliest available
value among `deadline_ts_ns` and `release_ts_ns + stale_ns`. The latter is
used only when both fields are nonzero. With neither bound, the key is the
current time. This is deadline ordering, not newest-message selection.

## Execution budgets

BPF stores execution accounting per worker. A new `job_id` resets the job's
execution total and overrun state at enqueue.
At `stopping`, elapsed running time is added to `exec_ns_in_job` and
`vruntime`.

When execution exceeds a nonzero budget, the scheduler marks the job overrun
and emits a budget-overrun event. A later enqueue sends overrun FE work to BE
and emits a budget-demotion event. The two events distinguish detection from
routing. IMU accounting still records overruns, but its special route takes
precedence over budget demotion.

Budget enforcement occurs at scheduling boundaries; it is not a timer that
cancels compute at the exact budget. Background ordering uses accumulated
runtime. The hint's `weight` field is currently not applied to that accumulator.

## Insertion slices

The hint's `slice_ns=0` means use `SCX_SLICE_DFL` (20 ms on the tested kernel).
It is translated to an explicit slice before insertion. Passing a literal
zero to the kernel would retain the residual slice, or grant 1 ns if exhausted.

The optional loader argument `--be-slice-cap-us N` caps the resolved slice
for missing hints and non-IMU hints whose original class is BE.
The bag harness exposes it as `BE_SLICE_CAP_US`.

| Setting | Effect |
| --- | --- |
| `0` (default) | No cap; existing requested/default slices apply. |
| Positive value | Eligible insertions use the smaller of the resolved slice and cap. |

Hogs and mapping are eligible. IMU and FE hints are not, including an FE job
routed to BE after a budget overrun. The cap changes neither the global
default slice nor queue order, deadlines, admission, or preemption.

## Hint lifetime and observability

A worker's hint may remain visible after callback compute completes.
That retained identity protects the userspace completion tail; it is replaced
only after the executor establishes completion. See the
[executor contract](DESIGN_HINTS_API.md#executor-contract).

Deadline events are best-effort observations at scheduler boundaries. They
are not the workload's completion-lateness counters. Use application metrics
to evaluate whether outputs completed on time.

## Kernel compatibility and diagnostics

The BPF source uses weak kfunc declarations and `bpf_ksym_exists()` for renamed
sched_ext helpers. The loader can report embedded switch flags before attachment;
evaluation harnesses require partial-switch mode.

Historical trace and preemption probes remain opt-in and are not part of
the default policy or the capped bag ablation.
