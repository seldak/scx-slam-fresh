# Optional ROS 2 integration

This directory contains the optional ROS 2 Lyrical integration. It is not part
of the standalone `make` build and does not change the BPF scheduler, loader,
synthetic pipeline, or E0-E4 tests.

Build and test it explicitly from the repository root:

```bash
make ros2
make test-ros2
```

The helper sources `/opt/ros/lyrical/setup.bash` by default. Set `ROS2_SETUP`
to use another installation path. It rejects an active non-Lyrical overlay so
packages from two ROS distributions cannot be mixed accidentally.

The executor wrapper links the repository's existing `libslamqos`
implementation and therefore needs the same `libbpf-dev` system dependency as
the standalone build.

Generated Colcon state is isolated in `.ros2-build`, `.ros2-install`, and
`.ros2-log`, all of which are ignored by Git.

Packages:

- `scx_slam_executor` exports the existing `libslamqos` C implementation to
  Ament consumers. The callback dispatcher/worker executor is not implemented
  yet.
- `scx_slam_workload` is a minimal downstream smoke node. The ROS workload and
  bag replay integration are not implemented yet.

The default build remains ROS-independent:

```bash
make
make test-demo test-e4 test-scheduler-mode test-window test-slice
```
