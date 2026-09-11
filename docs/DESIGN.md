# Application architecture

This repository owns workloads, work selection, ROS integration and evaluation.
The external scx_fresh repository owns the scheduler, loader and hint ABI.
The application publishes one selected job per worker using ABI version 1;
the client library stamps the version when writing the map.

## Workloads

These are distinct graphs, not interchangeable implementations of one pipeline.

| Workload | Structure | Purpose |
| --- | --- | --- |
| Dependent threaded workload | IMU and camera processing feed a bounded-batch estimator; periodic control selects a completed snapshot; mapping consumes camera-bearing snapshots. | Compare scheduling policies with dependencies, stale inputs and estimator overload. |
| ROS workload | Independent IMU callback alongside a camera-triggered vision → estimation → mapping chain. | Exercise executor ownership, middleware delivery and deterministic bag accounting. |
| Legacy standalone demo | Synthetic stage queues with configurable sensor releases, compute and backlog. | Reproduce the earlier calibration, shedding and budget experiments. |

All three consume synthetic CPU work. None implements SLAM, estimation
mathematics or a physical plant. Bag replay supplies real recorded input timing
and identity, not a real estimation algorithm.

## Dependent threaded workload

A dispatcher owns work selection. IMU and camera processing place measurements
in bounded inboxes; the estimator consumes at most eight measurements per job.
Periodic control selects the latest completed snapshot and a predetermined
setpoint. Mapping becomes ready after an estimator batch containing camera input.

In the hinted profile, control uses Urgent, IMU processing, camera processing
and estimation use Deadline, and mapping and background workers use Background.
The estimator-burst profile gives estimator jobs a CPU budget. These assignments
belong to the workload, not to sensor-specific scheduler routes.

Workers enroll and park before the source window begins. The dispatcher
publishes selected work before waking its worker and waits for completion before
replacing the hint. Work released within the source window drains afterward.
See [usage](USAGE.md#dependent-threaded-workload) for costs, timing bounds,
queue limits and comparison configurations.

## ROS workload

Each stage has an executor dispatcher and one callback worker. Dispatchers,
DDS and the adapter use the housekeeping CPU; callback workers and hogs share
the worker CPU. Message-aware subscriptions take messages directly from DDS
rather than duplicating its pending queue.

The workload assigns IMU to Urgent, vision and estimation to Deadline, and
mapping and hogs to Background. This differs from the dependent graph's
assignment, where control is Urgent.

The executor may reject expired messages before publication or after handoff,
before callback entry. Accepted work retains its hint through completion and
parking; the next assignment replaces it. Message cleanup and callback-group
release remain executor responsibilities. See the
[application hint contract](DESIGN_HINTS_API.md#executor-contract).

## Legacy demo

The legacy demo uses one consumer per FIFO stage queue. Producers may publish
the head hint when waking an idle consumer; the consumer republishes the exact
item it selects. A producer must not overwrite a busy consumer's hint.
This lifecycle differs from the dependent workload's dispatcher-owned selection.

## Time and scheduling boundaries

Scheduler timestamps use CLOCK_MONOTONIC. Bag timestamps identify source input
and are kept separate from scheduler deadlines. In the ROS camera chain,
downstream jobs retain the original camera identity and monotonic release time.
The dependent workload derives job bounds from its selected inputs and periodic
control releases.

The map transports metadata; it cannot select messages, cancel callbacks or
establish application completion. CPU affinity and partial-switch enrollment
do not exclude unrelated kernel or application work.

Routing, preemption, budget demotion and optional Background allocation are
defined in the [scheduler rules](https://github.com/seldak/scx_fresh/blob/main/docs/SCHEDULER.md).
The [evaluation guide](DESIGN_EVALUATION.md) defines accounting and links each
result to its workload. Timing results are not estimation-accuracy or control-safety claims.
