# SPDX-License-Identifier: GPL-2.0
"""Non-root, single-core checks of the real demo's cutoff and drain counters."""

import importlib.util
import os
import re
from pathlib import Path
import subprocess
import unittest

REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("e4", REPO / "scripts/run_e4_eval.py")
e4 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(e4)
DEMO = os.environ.get("DEMO_BIN", str(REPO / "build/slam_pipeline_demo"))


class WindowTests(unittest.TestCase):
    def run_window(self, work):
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run(["taskset", "-c", str(cpu), DEMO, "--no-hints", "--lidar", "heavy",
                                 "--hog", "0", "--duration", "1", "--imu-work-us", str(work),
                                 "--window-stats"], capture_output=True, text=True, timeout=20, check=True)
        # CFS smoke tests are explicitly NOT SCX/E4 observations.
        return e4.parse_metrics(result.stdout, work, 1, expected_policy=0)

    def test_baseline_conservation(self):
        data = self.run_window(150)
        self.assertEqual(data["measurement"]["window_ns"], 1000000000)
        self.assertEqual(data["window_imu_prop"]["offered"], 200)
        self.assertLessEqual(data["window_imu_prop"]["cpu_ns"], data["imu_prop"]["cpu_us"] * 1000 + 999)

    def test_overload_splits_cpu_and_completions(self):
        data = self.run_window(6000)
        w, d = data["window_imu_prop"], data["drain_imu_prop"]
        self.assertLess(w["completed"], 200)
        self.assertGreater(d["completed"], 0)
        self.assertEqual(w["completed"] + d["completed"], 200)
        self.assertLessEqual(w["cpu_ns"], 1000000000)
        self.assertGreater(d["cpu_ns"], 0)
        self.assertGreater(data["measurement"]["drain_elapsed_ns"], 0)
        # The lower bound includes an unfinished job's in-window compute.
        if w["in_flight"]:
            self.assertGreaterEqual(w["cpu_ns"] + w["cpu_uncertainty_ns"], w["completed"] * 6000000)
        print(f"CFS-only cutoff smoke: IMU window={w['completed']} drain={d['completed']} "
              f"CPU uncertainty={w['cpu_uncertainty_ns']}ns", flush=True)

    def test_stale_drops_preserve_window_conservation(self):
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run(["taskset", "-c", str(cpu), DEMO, "--no-hints", "--lidar", "heavy",
                                 "--hog", "0", "--duration", "1", "--drop-stale", "1",
                                 "--camera-burst-count", "12", "--camera-burst-at-ms", "500", "--window-stats"],
                                capture_output=True, text=True, timeout=20, check=True)
        rows = []
        for line in result.stdout.splitlines():
            if line.startswith("window_"):
                row = {k: int(v) for k, v in re.findall(r"(\w+)=(\d+)", line)}
                self.assertEqual(row["offered"], sum(row[k] for k in ("completed", "dropped_stale", "pending", "in_flight")))
                self.assertLessEqual(row["in_flight"], 1)
                rows.append(row)
        self.assertEqual(len(rows), 6)
        self.assertGreater(sum(r["dropped_stale"] for r in rows), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
