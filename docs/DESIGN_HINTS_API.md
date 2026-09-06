# Application hint integration

The scheduler owns the [hint ABI and publication contract](https://github.com/seldak/scx_fresh/blob/main/docs/HINTS_API.md).
This page describes how the standalone workload and ROS executor implement
that contract.

## Executor contract

Select work, publish its hint, then wake the worker. Never overwrite an
already-running callback's hint. The integration must establish completion
before replacing the slot; a map update is not callback cancellation.

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

`reject_expired` is local to the executor and applies to any service class.
It is not published as a kernel flag. The scheduler never demotes work because
its deadline or stale bound elapsed; normal class routing and budget demotion
still apply. The workload explicitly disables expiry rejection for IMU samples
and enables it for downstream bag subscriptions, preserving its admission policy.

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
