#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
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
MATCH = re.search(r"^assert_matched_source_windows\(\) \{\n.*?^\}", SCRIPT,
                 re.M | re.S).group()


class BagAccounting(unittest.TestCase):
    def test_expiry_options_match_loaded_policy(self):
        configure = re.search(r"^configure_expiry\(\) \{\n.*?^\}", SCRIPT,
                              re.M | re.S).group()
        command = configure + '''
deadline_grace_us=$2
configure_expiry "$1" || exit $?
printf '%s|%s|%s' "$expiry_policy" "$deadline_grace_us" "${deadline_grace_args[*]}"
'''
        for config, grace, expected in (
            ("expiry_policy=application", "", "application|not_applicable|"),
            ("deadline_grace_us=1000", "", "scheduler_age_demotion|1000|--deadline-grace-us 1000"),
            ("deadline_grace_us=1000", "0", "scheduler_age_demotion|0|--deadline-grace-us 0"),
        ):
            result = subprocess.run(["bash", "-c", command, "test", config, grace],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, expected)
        result = subprocess.run(["bash", "-c", command, "test", "expiry_policy=application", "1000"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("unset DEADLINE_GRACE_US", result.stderr)

    def test_ablation_validator_checks_all_cells_and_hog_windows(self):
        driver = (pathlib.Path(__file__).resolve().parents[1] /
                  "scripts/run_ros2_bag_ablation.sh").read_text()
        validator = driver.split("<<'PY'\n", 1)[1].split("\nPY\n", 1)[0]
        import csv
        import sys
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            stage_fields = ["mode", "repetition", "stage", "offered", "completed",
                            "dropped_before_start", "dropped_upstream", "unfinished",
                            "first_source_ts_ns", "last_source_ts_ns", "first_job_id", "last_job_id"]
            hog_fields = ["mode", "repetition", "hog", "iterations", "iteration_work_us",
                          "start_ns", "end_ns"]
            with (root / "summary.tsv").open("w") as stages, (root / "hog-summary.tsv").open("w") as hogs:
                sw = csv.writer(stages, delimiter="\t")
                hw = csv.writer(hogs, delimiter="\t")
                sw.writerow(stage_fields)
                hw.writerow(hog_fields)
                for mode in ("hinted", "imu-only", "fe-only"):
                    for stage in ("imu_prop", "vision_fe", "state_est", "mapping_be"):
                        count = 3000 if stage == "imu_prop" else 300
                        sw.writerow([mode, 1, stage, count, count, 0, 0, 0, 100, 200, 1, count])
                    for hog in (0, 1):
                        hw.writerow([mode, 1, hog, 5000, 1000, 100, 15000000100])
                    case = root / (mode + "-1")
                    case.mkdir()
                    (case / "environment.txt").write_text(
                        "source_start_ns=100\nduration=15\nwarmup=3\nhog_threads=2\n"
                        "cpu=14\nhousekeeping_cpu=1\nbe_slice_cap_us=2000\n"
                        "deadline_grace_us=1000\nops_flags=0x8\nbag_manifest_sha256=bag\n"
                        "ros_pipeline_sha256=p\nros_adapter_sha256=a\nloader_sha256=l\nbpf_object_sha256=b\n")
            def validate():
                return subprocess.run([sys.executable, "-", str(root), "1"], input=validator,
                                      capture_output=True, text=True)
            result = validate()
            self.assertEqual(result.returncode, 0, result.stderr)
            path = root / "fe-only-1/environment.txt"
            path.write_text(path.read_text().replace("ros_pipeline_sha256=p", "ros_pipeline_sha256=changed"))
            self.assertNotEqual(validate().returncode, 0)
            path.write_text(path.read_text().replace("ros_pipeline_sha256=changed", "ros_pipeline_sha256=p"))
            path = root / "hog-summary.tsv"
            path.write_text(path.read_text().replace("15000000100", "15000000101", 1))
            self.assertNotEqual(validate().returncode, 0)

    def test_hinted_only_windows_match_external_baseline_exactly(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            baseline = root / "baseline"
            output = root / "experiment"
            baseline.mkdir()
            output.mkdir()
            def summary(mode, first_stamp):
                fields = [mode, "1", "state_est", "300"] + ["0"] * 17
                fields[13:15] = [str(first_stamp), "1403636597713555500"]
                fields[19:21] = ["61", "360"]
                return "header\n" + "\t".join(fields) + "\n"
            stamp = 1403636582763555500
            (baseline / "summary.tsv").write_text(summary("cfs", stamp))
            for offset in (0, 1):
                (output / "summary.tsv").write_text(summary("hinted", stamp + offset))
                result = subprocess.run(
                    ["bash", "-c", MATCH + '\nbaseline_dir=$1\noutput_dir=$2\n'
                     'assert_matched_source_windows', "test", str(baseline), str(output)],
                    capture_output=True, text=True)
                self.assertEqual(result.returncode == 0, offset == 0, result.stderr)

    def check_case(self, changes=None, adapter_drops=0, case_name="hinted-1"):
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
                 'assert_case_accounting "$1/pipeline" "$1/adapter" "$2"',
                 "test", directory, case_name], capture_output=True, text=True)
            if result.returncode:
                self.assertIn("in " + case_name, result.stderr)
            return result.returncode

    def test_ablation_gate_reports_actual_case_and_keeps_imu_requirement(self):
        for name in ("hinted-1", "imu-only-2", "fe-only-3"):
            with self.subTest(name=name):
                self.assertEqual(self.check_case(case_name=name), 0)
                self.assertNotEqual(self.check_case(
                    {"imu_prop": dict(late=1)}, case_name=name), 0)
        self.assertEqual(self.check_case(
            {"imu_prop": dict(late=1)}, case_name="cfs-1"), 0)

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
