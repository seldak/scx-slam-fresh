# Historical notes

[Evaluation overview](../DESIGN_EVALUATION.md)

## Withdrawn full-switch result

The 2026-09-02 scheduler results were withdrawn after preflight exposed a build
bug: an `#ifdef` tested `SCX_OPS_SWITCH_PARTIAL`, an enum constant rather than a
macro. The resulting object silently used full-switch mode.

The default build now requires the flag directly. The loader reports embedded
flags without attaching, and evaluation runners reject full-switch objects.
The CFS-only calibration was unaffected.

The [replacement validation](../evaluation/standalone.md#validated-partial-switch-results)
used three repetitions at revision `749b579` with verified partial-switch flags.
The obsolete performance numbers are omitted.

## IMU load diagnostics

The standalone IMU sweep investigated a first estimator miss followed by
stale-demotion backlog. Three conclusions remain relevant:

| Question | Observation | Conclusion |
| --- | --- | --- |
| Does API slice zero request the kernel default? | Kernel zero preserves residual service, or grants 1 ns when exhausted. | Translate the hint API's zero to an explicit default slice. |
| Does earlier LiDAR-pre budget demotion prevent the first estimator miss? | A 6 ms budget moved LiDAR-pre to BE before its deadline, but estimator job 4 still missed. | Budget timing alone did not explain the first miss. |
| Does starting LiDAR-pre in BE prevent it? | Estimator job 4 still missed; later misses followed the every-fifth-job compute peak. | Initial LiDAR-pre class alone did not explain the first miss. |

At 70% nominal IMU utilization, 23.1 ms of IMU demand plus the peak 9 ms
vision/estimator pair leaves 0.9 ms in the shared 33 ms deadline for release
phase and overhead. The observed first miss was consistent with this narrow
margin; stale demotion then amplified the delay into backlog.

The [validated IMU sweep](../evaluation/imu-load.md) records the measured
regimes. These diagnostics preceded the ROS executor recovery and are not
results for the later bag graph.

Detailed probe commands, trace formats, and exploratory chronology remain in
Git history. They are not required to reproduce the current evaluation.
