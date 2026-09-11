# Project status

Feature development is paused. The [closing findings](evaluation/conclusion.md)
record what the experiments established and what they did not.

The repository retains synthetic standalone workloads, ROS executor integration,
bag replay and explicit accounting. It does not implement SLAM or a real fusion
estimator. Scheduler policy and experimental PREEMPT_RT patches live in
[scx_fresh](https://github.com/seldak/scx_fresh).

The dependent overload experiment did not demonstrate better freshness than
FIFO. The separate EDF task set matched Linux SCHED_DEADLINE; generic and
experimental RT runs had the same deadline-miss counts.

No further scheduler variants, datasets or estimator integration are scheduled.
Historical reports remain evidence for their stated configurations, not current
performance promises. Kernel timer work can be revisited independently.
