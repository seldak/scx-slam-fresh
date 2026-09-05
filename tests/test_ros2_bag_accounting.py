#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Exercise the actual harness assertions without starting ROS or sched_ext."""
import pathlib
import re
import subprocess
import tempfile
import unittest


SCRIPT = (pathlib.Path(__file__).resolve().parents[1] /
          "scripts/run_ros2_bag_eval.sh").read_text()
ASSERT = re.search(r"^assert_case_accounting\(\) \{\n.*?^\}", SCRIPT,
                   re.M | re.S).group()


class BagAccounting(unittest.TestCase):
    def check_case(self, changes=None, adapter_drops=0):
        changes = changes or {}
        lines = []
        for stage in ("imu_prop", "vision_fe", "state_est", "mapping_be"):
            count = 3000 if stage == "imu_prop" else 300
            values = dict(offered=count, arrivals=count, executed=count,
                          completed=count, dropped_before_start=0,
                          dropped_upstream=0, unfinished=0, late=0,
                          first_job_id=601 if stage == "imu_prop" else 61,
                          last_job_id=3600 if stage == "imu_prop" else 360)
            values.update(changes.get(stage, {}))
            lines.append("window_" + stage + ": " + " ".join(
                f"{key}={value}" for key, value in values.items()))
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory)
            (path / "pipeline").write_text("\n".join(lines) + "\n")
            (path / "adapter").write_text(
                f"adapter_imu: dropped={adapter_drops}\n"
                "adapter_camera: dropped=0\n")
            result = subprocess.run(
                ["bash", "-c", ASSERT + '\nduration=15\n'
                 'assert_case_accounting "$1/pipeline" "$1/adapter" hinted-1',
                 "test", directory], capture_output=True, text=True)
            return result.returncode

    def test_complete_window(self):
        self.assertEqual(self.check_case(), 0)

    def test_explicit_estimator_drop_resolves_mapping_source_opportunity(self):
        self.assertEqual(self.check_case({
            "state_est": dict(executed=299, completed=299, dropped_before_start=1),
            "mapping_be": dict(arrivals=299, executed=299, completed=299,
                               dropped_upstream=1)}), 0)

    def test_starved_stage_cannot_hide_offered_input(self):
        self.assertNotEqual(self.check_case({"state_est": dict(
            offered=0, arrivals=0, executed=0, completed=0)}), 0)

    def test_unfinished_and_unaccounted_drop_fail(self):
        self.assertNotEqual(self.check_case({"state_est": dict(
            executed=299, completed=299, unfinished=1)}), 0)
        self.assertNotEqual(self.check_case({"state_est": dict(
            executed=299, completed=299)}), 0)

    def test_adapter_loss_and_imu_lateness_fail(self):
        self.assertNotEqual(self.check_case(adapter_drops=1), 0)
        self.assertNotEqual(self.check_case({"imu_prop": dict(late=1)}), 0)


if __name__ == "__main__":
    unittest.main()
