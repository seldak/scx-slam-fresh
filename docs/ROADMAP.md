# Roadmap (stretch goals)

These are great “resume bullets” because they show depth beyond the initial demo.

## Scheduling features
- Stage-aware CPU affinity hints (e.g., keep FE on fewer cores for cache locality).

Deferred pending E4 evidence:
- Slack stealing, only if E4 shows BE-only collapse while IMU and FE remain healthy.
- Bounded lower-lane service, only if E4 shows FE and BE collapse while IMU remains healthy.

Not planned:
- A BPF pending-hint ring. The userspace executor remains the source of truth
  for work selection; duplicating its queue in BPF would create divergent state.

## Better pipeline integration
- ~~Classify the loaded ROS snapshot's zero-hog maximum-tail anomaly.~~ Closed
  as unshielded fair-class interference by matched standard-perf captures; the
  shielded confirmation required no scheduler-policy change.
- Run the matched ROS 2 CFS/scx callback-compute evaluation and document the
  validated scope before changing scheduler policy.
- ROS2: validate a recorded sensor bag with the matched CFS/hinted harness and
  freeze bag-window offered-count rules from the captured job-id spans.
- Generalize the documented executor hint contract beyond the demo's 1:1 FIFO workers.

## Power / perf (still non-proprietary)
- Experiment with `scx_bpf_cpuperf_set()` (if available) to boost CPU perf during FE spikes.

## Observability
- Export events as JSON
- Add a Python notebook for plots (p50/p99, miss rate, wasted work)

## CI
- Build-only CI job (compile BPF and userspace, don’t attach)
