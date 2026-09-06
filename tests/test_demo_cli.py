# SPDX-License-Identifier: MIT
"""Non-root demo CLI and synthetic IMU CPU-accounting regression tests."""

import os
from pathlib import Path
import re
import subprocess
import unittest


DEMO = os.environ.get(
    "DEMO_BIN", str(Path(__file__).resolve().parents[1] / "build/slam_pipeline_demo")
)
MAX_WORK_US = (2**64 - 1) // 1000


class DemoCliTests(unittest.TestCase):
    def invoke(self, *args, pinned=False):
        command = [DEMO, *args]
        if pinned:
            # Match single-core contention without requiring a fixed CPU number.
            cpu = min(os.sched_getaffinity(0))
            command = ["taskset", "-c", str(cpu), *command]
        return subprocess.run(command, capture_output=True, text=True, timeout=20)

    def test_help_describes_imu_work(self):
        result = self.invoke("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--imu-work-us", result.stderr)
        self.assertIn("default 150", result.stderr)
        self.assertIn("5ms tick", result.stderr)
        self.assertIn("--lidar-pre-budget-us", result.stderr)
        self.assertIn("default 10000", result.stderr)
        self.assertIn("--lidar-pre-class", result.stderr)
        self.assertIn("default fe", result.stderr)

    def test_invalid_imu_work(self):
        for value in (
            "", "-1", "+1", " 150", "150 ", "1.5", "abc", "150us", "0x96",
            str(MAX_WORK_US + 1), str(2**64), "9" * 100,
        ):
            with self.subTest(value=value):
                # --help avoids starting workers even if parsing regresses.
                result = self.invoke("--imu-work-us", value, "--help")
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("error: IMU work", result.stderr)

    def test_missing_imu_work(self):
        result = self.invoke("--no-hints", "--imu-work-us")
        self.assertEqual(result.returncode, 1)
        self.assertIn("Usage:", result.stderr)

    def test_lidar_pre_budget_validation(self):
        for value in ("", "-1", "+1", " 7000", "7000 ", "7ms", str(MAX_WORK_US + 1)):
            with self.subTest(value=value):
                result = self.invoke("--lidar-pre-budget-us", value, "--help")
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("LiDAR preprocessing budget", result.stderr)
        for value in ("0", "7000", "10000", str(MAX_WORK_US)):
            with self.subTest(value=value):
                result = self.invoke("--lidar-pre-budget-us", value, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_lidar_pre_class_validation(self):
        for value in ("", "FE", "frontend", "0"):
            with self.subTest(value=value):
                result = self.invoke("--lidar-pre-class", value, "--help")
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("LiDAR preprocessing class", result.stderr)
        for value in ("fe", "be"):
            with self.subTest(value=value):
                result = self.invoke("--lidar-pre-class", value, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_valid_imu_work_bounds(self):
        for value in ("0", "000150", "5000", "6000", str(MAX_WORK_US)):
            with self.subTest(value=value):
                # Check the upper bound without executing a huge compute job.
                result = self.invoke("--imu-work-us", value, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)

    def check_workload(self, work_us, *, explicit=True, hogs=0):
        args = ["--no-hints", "--lidar", "off", "--hog", str(hogs),
                "--duration", "1"]
        if explicit:
            args += ["--imu-work-us", str(work_us)]
        result = self.invoke(*args, pinned=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        config = re.search(r"^configuration: (.*)$", result.stdout, re.MULTILINE)
        self.assertIsNotNone(config, result.stdout)
        fields = dict(re.findall(r"(\w+)=(\d+)", config.group(1)))
        self.assertEqual(int(fields["imu_work_us"]), work_us)
        self.assertEqual(int(fields["vision_budget_us"]), 12000)
        self.assertEqual(int(fields["vision_work_us"]), 0)
        self.assertEqual(int(fields["vision_deadline_us"]), 33000)
        self.assertEqual(int(fields["lidar_pre_budget_us"]), 10000)
        self.assertEqual(int(fields["lidar_pre_class_id"]), 1)
        imu = re.search(
            r"^imu_prop:\s+processed=(\d+) late=(\d+) \([^)]*\) cpu_us=(\d+)$",
            result.stdout, re.MULTILINE,
        )
        self.assertIsNotNone(imu, result.stdout)
        processed, late, cpu_us = map(int, imu.groups())
        self.assertEqual(processed, 200)  # Fixed 5ms period, including at overload.
        expected_cpu_us = processed * work_us
        self.assertGreaterEqual(cpu_us, expected_cpu_us)
        # Allow clock-call/loop overshoot, but catch a wrong scale or ignored knob.
        self.assertLess(cpu_us, expected_cpu_us + max(10000, expected_cpu_us // 5))
        if work_us > 5000:
            self.assertEqual(late, processed)  # Deadlines remain wall-clock based.
        print(f"IMU work={work_us}us hogs={hogs}: "
              f"processed={processed} late={late} cpu_us={cpu_us}", flush=True)

    def test_default_workload(self):
        self.check_workload(150, explicit=False)

    def test_explicit_control_workload(self):
        self.check_workload(150)

    def test_zero_compute(self):
        self.check_workload(0)

    def test_custom_work_under_contention(self):
        self.check_workload(1500, hogs=1)

    def test_above_period_workload(self):
        self.check_workload(6000)


if __name__ == "__main__":
    unittest.main(verbosity=2)
