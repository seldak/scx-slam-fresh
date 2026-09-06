# SPDX-License-Identifier: MIT
"""E4 planning/parser tests; fixtures are synthetic, never benchmark evidence."""

import argparse
from copy import deepcopy
import csv
import importlib.util
import hashlib
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_e4_eval.py"
SPEC = importlib.util.spec_from_file_location("e4", SCRIPT)
e4 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(e4)


def fixture():
    lines = ["generated: imu=200 camera=31 lidar=10",
             "configuration: vision_budget_us=12000 vision_work_us=0 vision_deadline_us=33000 imu_work_us=150 lidar_pre_budget_us=10000 lidar_pre_class_id=1",
             "imu_prop: processed=200 late=0 (0.0%) cpu_us=30100",
             "focus_vision_fe: job=4 release_ns=1050000000 completion_ns=1070000000 age_ns=20000000 late=0",
             "focus_state_est: job=4 release_ns=1050000000 completion_ns=1080000000 age_ns=30000000 late=0"]
    for name, processed, pending in (("vision_fe", 31, 0), ("state_est", 31, 0),
                                      ("lidar_pre", 10, 0), ("lidar_reg", 2, 8),
                                      ("mapping_be", 20, 13)):
        lines.append(f"{name}: dequeued={processed} processed={processed} late=0 (0.0%) "
                     f"cpu_us=1000 pending={pending} stale_seen=0 dropped_stale=0")
        lines.append(f"window_{name}: offered={processed + pending} completed={processed} late=0 "
                     f"cpu_us=1000 cpu_ns=1000000 cpu_uncertainty_ns=0 pending={pending} in_flight=0 "
                     "stale_seen=0 dropped_stale=0")
        lines.append(f"drain_{name}: completed=0 late=0 cpu_us=0 cpu_ns=0 cpu_uncertainty_ns=0")
    lines += ["measurement: start_ns=1000000000 end_ns=2000000000 window_ns=1000000000 elapsed_ns=1100000000 drain_elapsed_ns=100000000",
              "imu_identity: pid_tgid=123 policy=7 stage_id=0",
              "window_imu_prop: offered=200 completed=190 late=0 cpu_us=28500 cpu_ns=28500000 cpu_uncertainty_ns=100 pending=9 in_flight=1 stale_seen=0 dropped_stale=0",
              "drain_imu_prop: completed=10 late=0 cpu_us=1600 cpu_ns=1600000 cpu_uncertainty_ns=100"]
    return "\n".join(lines)


class E4Tests(unittest.TestCase):
    def test_plan_is_reproducible_and_bracketed(self):
        costs = e4.costs_arg(e4.DEFAULT_COSTS)
        plan = list(e4.case_plan(costs, 2, 4))
        self.assertEqual(plan, list(e4.case_plan(costs, 2, 4)))
        for repetition in (1, 2):
            rows = [r for r in plan if r["repetition"] == repetition]
            self.assertEqual(rows[0]["role"], "control-start")
            self.assertEqual(rows[-1]["role"], "control-end")
            self.assertEqual(rows[0]["imu_work_us"], 150)
            self.assertEqual(rows[-1]["imu_work_us"], 150)
            self.assertEqual(sorted(r["imu_work_us"] for r in rows[1:-1]), costs[1:])

    def test_invalid_costs(self):
        for text in ("", "-1,150", "150,abc", "150,", "150,150", "1000", "150,10001"):
            with self.subTest(text=text), self.assertRaises(argparse.ArgumentTypeError):
                e4.costs_arg(text)

    def test_parse_and_offered_counts(self):
        data = e4.parse_metrics(fixture(), 150, 1)
        self.assertEqual(data["lidar_reg"]["offered"], 10)
        self.assertEqual(data["mapping_be"]["offered"], 33)
        self.assertEqual(data["state_est"]["offered"], 31)
        self.assertEqual(data["focus_vision_fe"]["age_ns"], 20_000_000)
        self.assertEqual(data["focus_state_est"]["age_ns"], 30_000_000)

    def test_focus_job_metrics_must_be_exact_and_consistent(self):
        for old, new in (("job=4", "job=5"), ("age_ns=20000000", "age_ns=20000001"),
                         ("age_ns=30000000 late=0", "age_ns=30000000 late=1")):
            with self.subTest(old=old), self.assertRaises(ValueError):
                e4.parse_metrics(fixture().replace(old, new, 1), 150, 1)

    def test_invalid_metrics(self):
        for old, new in (("imu=200", "imu=199"), ("imu_work_us=150", "imu_work_us=500"),
                         ("cpu_us=30100", "cpu_us=1"), ("late=0", "late=999"),
                         ("pending=8", "pending=9"), ("dropped_stale=0", "dropped_stale=1"),
                         ("vision_work_us=0", "vision_work_us=12000"),
                         ("mapping_be:", "unknown:")):
            with self.subTest(old=old), self.assertRaises(ValueError):
                e4.parse_metrics(fixture().replace(old, new), 150, 1)

    def test_duplicate_metrics(self):
        with self.assertRaises(ValueError):
            e4.parse_metrics(fixture() + "\ngenerated: imu=200 camera=31 lidar=10", 150, 1)

    def test_summary_is_not_a_regime_verdict(self):
        baseline = {"name": "control", "role": "control-start", "repetition": 1,
                    "imu_work_us": 150, "imu_preempt": "wakeup", "process_elapsed_s": 1.2,
                    "metrics": e4.parse_metrics(fixture(), 150, 1)}
        sweep = deepcopy(baseline)
        sweep.update(name="test", role="sweep", imu_work_us=1000)
        sweep["metrics"]["window_lidar_reg"]["completed"] = 1
        sweep["metrics"]["window_mapping_be"]["completed"] = 10
        sweep["metrics"]["window_vision_fe"]["completed"] = 0
        matrix, stages, drain = e4.summarize([baseline, sweep])
        self.assertNotIn("be_completed_ratio_to_control", matrix[1])
        self.assertEqual(matrix[1]["lidar_reg_rate_ratio_to_control"], 0.5)
        self.assertEqual(matrix[1]["mapping_be_rate_ratio_to_control"], 0.5)
        self.assertEqual(matrix[1]["regime"], "unclassified-exploratory")
        self.assertEqual(matrix[1]["observation"], "fixed-window")
        self.assertIsNone(matrix[1]["vision_fe_miss_pct"])
        self.assertEqual(len(stages), 12)
        self.assertEqual(len(drain), 12)
        self.assertIsNone(e4.ratio(0, 0))
        self.assertIsNone(e4.ratio(1, 0))

    def test_reject_legacy_and_invalid_window_metrics(self):
        for old, new in (("measurement:", "legacy:"), ("window_ns=1000000000", "window_ns=1100000000"),
                         ("policy=7", "policy=0"), ("pending=9", "pending=8"),
                         ("cpu_ns=28500000", "cpu_ns=990000000"),
                         ("drain_imu_prop: completed=10", "drain_imu_prop: completed=11")):
            with self.subTest(old=old), self.assertRaises(ValueError):
                e4.parse_metrics(fixture().replace(old, new), 150, 1)

    def test_drain_does_not_improve_window_rates(self):
        case = {"name": "control", "role": "control-start", "repetition": 1, "imu_preempt": "wakeup",
                "imu_work_us": 150, "process_elapsed_s": 1.2, "metrics": e4.parse_metrics(fixture(), 150, 1)}
        original = e4.summarize([case])[0][0]
        changed = deepcopy(case)
        changed["process_elapsed_s"] = 100
        changed["metrics"]["lidar_reg"]["processed"] = 1000
        changed["metrics"]["drain_lidar_reg"]["completed"] = 998
        changed["metrics"]["measurement"].update(elapsed_ns=100000000000, drain_elapsed_ns=99000000000)
        new = e4.summarize([changed])[0][0]
        for key in ("lidar_reg_rate_hz", "mapping_be_rate_hz", "imu_window_cpu_pct", "lidar_reg_rate_ratio_to_control"):
            self.assertEqual(original[key], new[key])

    def test_unfinished_is_not_missing_handoff(self):
        case = {"name": "control", "role": "control-start", "repetition": 1, "imu_preempt": "wakeup",
                "imu_work_us": 150, "process_elapsed_s": 1.2, "metrics": e4.parse_metrics(fixture(), 150, 1)}
        # All 31 vision outputs arrived; 23 estimator jobs are still unfinished.
        case["metrics"]["window_state_est"].update(completed=8, late=8, pending=22, in_flight=1)
        row = e4.summarize([case])[0][0]
        self.assertEqual(row["state_est_offered"], 31)
        self.assertEqual(row["state_est_unfinished"], 23)
        self.assertEqual(row["state_est_late"], 8)
        self.assertEqual(row["vision_to_estimator_not_delivered_at_cutoff"], 0)
        text = e4.format_case_summary(row)
        self.assertIn("state_est: offered=31 completed=8 late=8 unfinished=23 (pending=22 in_flight=1) cpu_us=1000", text)
        self.assertIn("output_hz=31.000 arrivals_hz=31.000 not_delivered_at_cutoff=0", text)
        # A completed vision job whose push crosses the cutoff is a different counter.
        case["metrics"]["window_state_est"].update(offered=30, pending=21)
        row = e4.summarize([case])[0][0]
        self.assertEqual(row["state_est_unfinished"], 22)
        self.assertEqual(row["vision_to_estimator_not_delivered_at_cutoff"], 1)

    def test_summary_csv_retains_counts_and_cpu_for_every_stage(self):
        case = {"name": "control", "role": "control-start", "repetition": 1, "imu_preempt": "wakeup",
                "imu_work_us": 150, "process_elapsed_s": 1.2, "metrics": e4.parse_metrics(fixture(), 150, 1)}
        with tempfile.TemporaryDirectory(prefix="scx-e4-report-test-") as directory:
            output = Path(directory)
            rows = e4.save_summaries(output, [case])
            with (output / "matrix.csv").open() as handle:
                row = next(csv.DictReader(handle))
            with (output / "stages.csv").open() as handle:
                stages = list(csv.DictReader(handle))
        for stage in e4.STAGES:
            m = case["metrics"][f"window_{stage}"]
            for key in ("offered", "completed", "late", "pending", "in_flight", "stale_seen", "dropped_stale", "cpu_us"):
                self.assertEqual(int(row[f"{stage}_{key}"]), m[key])
            unfinished = m["pending"] + m["in_flight"]
            self.assertEqual(int(row[f"{stage}_unfinished"]), unfinished)
            self.assertEqual(int(next(s for s in stages if s["stage"] == stage)["unfinished"]), unfinished)
        self.assertEqual(rows[0]["imu_prop_unfinished"], 10)
        text = e4.format_case_summary(rows[0])
        self.assertIn("lidar_pre: offered=10 completed=10 late=0", text)
        self.assertEqual(text.count("rate/control=1.000x"), 2)
        self.assertIn("DRAIN: 0.100s (excluded above)", text)
        rows[0]["lidar_reg_rate_ratio_to_control"] = None
        self.assertIn("rate/control=n/a", e4.format_case_summary(rows[0]))

    def test_repetitions_use_their_own_starting_control(self):
        control = {"name": "r1-control", "role": "control-start", "repetition": 1, "imu_preempt": "wakeup",
                   "imu_work_us": 150, "process_elapsed_s": 1.2, "metrics": e4.parse_metrics(fixture(), 150, 1)}
        second = deepcopy(control)
        second.update(name="r2-control", repetition=2)
        second["metrics"]["window_lidar_reg"]["completed"] = 4
        second["metrics"]["window_mapping_be"]["completed"] = 10
        sweep = deepcopy(second)
        sweep.update(name="r2-sweep", role="sweep", imu_work_us=3500)
        sweep["metrics"]["window_lidar_reg"]["completed"] = 1
        sweep["metrics"]["window_mapping_be"]["completed"] = 5
        end = deepcopy(second)
        end.update(name="r2-end", role="control-end")
        end["metrics"]["window_lidar_reg"]["completed"] = 2
        matrix = e4.summarize([control, second, sweep, end])[0]
        self.assertEqual(matrix[2]["control_case"], "r2-control")
        self.assertEqual(matrix[2]["lidar_reg_rate_ratio_to_control"], 0.25)
        self.assertEqual(matrix[2]["mapping_be_rate_ratio_to_control"], 0.5)
        self.assertEqual(matrix[3]["lidar_reg_rate_ratio_to_control"], 0.5)

    def test_refinement_cli_is_repeated_bracketed_and_untraced(self):
        import os
        result = subprocess.run([sys.executable, str(SCRIPT), "--costs", "150,3000,3250,3500,3750,4000",
                                 "--repetitions", "3", "--cpu", str(min(os.sched_getaffinity(0))), "--dry-run"],
                                capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 21)
        for repetition in (1, 2, 3):
            group = [line for line in lines if line.startswith(f"r{repetition}-")]
            self.assertIn("control-start-150", group[0])
            self.assertIn("control-end-150", group[-1])
            self.assertEqual(sorted(int(line.split()[0].rsplit("-", 1)[1]) for line in group[1:-1]),
                             [3000, 3250, 3500, 3750, 4000])
        for line in lines:
            self.assertIn("--duration 15", line)
            self.assertIn("--lidar heavy --hog 0 --drop-stale 0", line)
            self.assertIn("--imu-preempt wakeup, trace_imu=0, execution_cpu=-1", line)

    def test_perf_cli_is_a_narrow_default_off_diagnostic(self):
        import os
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run([sys.executable, str(SCRIPT), "--perf-sched", "--cpu", str(cpu), "--dry-run"],
                                capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 4)
        self.assertIn("control-start-150", lines[0])
        self.assertIn("control-end-150", lines[-1])
        self.assertEqual(sorted(int(line.split()[0].rsplit("-", 1)[1]) for line in lines[1:-1]), [3250, 3500])
        for line in lines:
            self.assertIn("perf record -a -k mono", line)
            self.assertIn("-e sched:sched_switch -e sched:sched_waking", line)
            self.assertNotIn("sched_stat_runtime", line)
            self.assertIn("--trace-estimator; perf_sched=1", line)
            self.assertNotIn("--trace-execution-cpu", line)
            self.assertNotIn("--trace-imu]", line)
        ordinary = subprocess.run([sys.executable, str(SCRIPT), "--costs", "150,3250,3500", "--cpu", str(cpu),
                                   "--dry-run"], capture_output=True, text=True, check=True)
        self.assertNotIn("perf record", ordinary.stdout)
        self.assertNotIn("--trace-estimator", ordinary.stdout)
        for conflict in ("--preempt-probe", "--execution-probe", "--binary-dir /tmp/archive"):
            bad = subprocess.run([sys.executable, str(SCRIPT), "--perf-sched", *conflict.split(), "--cpu", str(cpu),
                                  "--dry-run"], capture_output=True, text=True)
            self.assertNotEqual(bad.returncode, 0)

    def test_grace_probe_is_bracketed_randomized_and_uses_lean_perf(self):
        import os
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run([sys.executable, str(SCRIPT), "--grace-probe", "--cpu", str(cpu), "--dry-run"],
                                capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 4)
        self.assertIn("grace-1000-control-start-150", lines[0])
        self.assertIn("grace-1000-control-end-150", lines[-1])
        self.assertEqual({int(re.search(r"grace-(\d+)-sweep", line).group(1)) for line in lines[1:-1]},
                         {0, 1000})
        for line in lines:
            self.assertIn("perf record -a -k mono", line)
            self.assertNotIn("sched_stat_runtime", line)
            self.assertIn("--imu-preempt wakeup", line)
            self.assertIn("--trace-estimator; perf_sched=1", line)
        for conflict in ("--costs 150,3500", "--preempt-probe", "--execution-probe",
                         "--wakeup-only", "--binary-dir /tmp/archive"):
            bad = subprocess.run([sys.executable, str(SCRIPT), "--grace-probe", *conflict.split(),
                                  "--cpu", str(cpu), "--dry-run"], capture_output=True, text=True)
            self.assertNotEqual(bad.returncode, 0)

    def test_grace_plan_uses_one_binary_policy_and_default_controls(self):
        plan = list(e4.grace_case_plan(2, 4))
        self.assertEqual(len(plan), 8)
        for repetition in (1, 2):
            rows = [row for row in plan if row["repetition"] == repetition]
            self.assertEqual(rows[0]["deadline_grace_us"], 1000)
            self.assertEqual(rows[-1]["deadline_grace_us"], 1000)
            self.assertEqual({row["deadline_grace_us"] for row in rows[1:-1]}, {0, 1000})
            self.assertEqual({row["imu_preempt"] for row in rows}, {"wakeup"})

    def test_lidar_pre_budget_probe_is_bracketed_and_keeps_grace_fixed(self):
        import os
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run([sys.executable, str(SCRIPT), "--lidar-pre-budget-probe",
                                 "--cpu", str(cpu), "--dry-run"],
                                capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 4)
        self.assertIn("lpre-budget-10000-control-start-150", lines[0])
        self.assertIn("lpre-budget-10000-control-end-150", lines[-1])
        self.assertEqual({int(re.search(r"lpre-budget-(\d+)-sweep", line).group(1))
                         for line in lines[1:-1]}, {6000, 10000})
        for line in lines:
            self.assertIn("deadline_grace_us=1000", line)
            self.assertIn("perf record -a -k mono", line)
            self.assertNotIn("sched_stat_runtime", line)
            self.assertIn("--lidar-pre-budget-us", line)

    def test_lidar_pre_budget_plan_uses_default_controls(self):
        plan = list(e4.lidar_pre_budget_case_plan(2, 4))
        self.assertEqual(len(plan), 8)
        for repetition in (1, 2):
            rows = [row for row in plan if row["repetition"] == repetition]
            self.assertEqual(rows[0]["lidar_pre_budget_us"], 10000)
            self.assertEqual(rows[-1]["lidar_pre_budget_us"], 10000)
            self.assertEqual({row["lidar_pre_budget_us"] for row in rows[1:-1]}, {6000, 10000})
            self.assertEqual({row["deadline_grace_us"] for row in rows}, {1000})
            self.assertEqual({row["imu_preempt"] for row in rows}, {"wakeup"})

    def test_lidar_pre_class_probe_is_bracketed_and_keeps_budget_grace_fixed(self):
        import os
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run([sys.executable, str(SCRIPT), "--lidar-pre-class-probe",
                                 "--cpu", str(cpu), "--dry-run"],
                                capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 4)
        self.assertIn("lpre-class-fe-control-start-150", lines[0])
        self.assertIn("lpre-class-fe-control-end-150", lines[-1])
        self.assertEqual({re.search(r"lpre-class-(fe|be)-sweep", line).group(1)
                          for line in lines[1:-1]}, {"fe", "be"})
        for line in lines:
            self.assertIn("deadline_grace_us=1000", line)
            self.assertIn("lidar_pre_budget_us=10000", line)
            self.assertIn("perf record -a -k mono", line)
            self.assertNotIn("sched_stat_runtime", line)
            self.assertIn("--lidar-pre-class", line)

    def test_lidar_pre_class_plan_uses_fe_controls(self):
        plan = list(e4.lidar_pre_class_case_plan(2, 4))
        self.assertEqual(len(plan), 8)
        for repetition in (1, 2):
            rows = [row for row in plan if row["repetition"] == repetition]
            self.assertEqual(rows[0]["lidar_pre_class_id"], 1)
            self.assertEqual(rows[-1]["lidar_pre_class_id"], 1)
            self.assertEqual({row["lidar_pre_class_id"] for row in rows[1:-1]}, {0, 1})
            self.assertEqual({row["lidar_pre_budget_us"] for row in rows}, {10000})
            self.assertEqual({row["deadline_grace_us"] for row in rows}, {1000})
            self.assertEqual({row["imu_preempt"] for row in rows}, {"wakeup"})

    def test_probe_pairs_and_controls(self):
        plan = list(e4.case_plan([150, 2000, 3000], 1, 4, probe=True))
        self.assertEqual(len(plan), 8)
        for cost in (2000, 3000):
            self.assertEqual({c["imu_preempt"] for c in plan if c["imu_work_us"] == cost}, {"wakeup", "always"})
        for mode in ("wakeup", "always"):
            rows = [c for c in plan if c["imu_preempt"] == mode]
            self.assertEqual(rows[0]["role"], "control-start")
            self.assertEqual(rows[-1]["role"], "control-end")

    def test_wakeup_only_probe_retains_bracketing_controls(self):
        plan = list(e4.case_plan([150, 2000, 3000], 1, 4, probe=True, wakeup_only=True))
        self.assertEqual(len(plan), 4)
        self.assertEqual({c["imu_preempt"] for c in plan}, {"wakeup"})
        self.assertEqual(plan[0]["role"], "control-start")
        self.assertEqual(plan[-1]["role"], "control-end")
        self.assertEqual(sorted(c["imu_work_us"] for c in plan[1:-1]), [2000, 3000])

    def test_archived_binaries_use_explicit_paths_and_hash_checks(self):
        with tempfile.TemporaryDirectory(prefix="scx-e4-archive-test-") as directory:
            build = Path(directory)
            binary = build / "slam_pipeline_demo"
            binary.write_bytes(b"fixture, not an executable")
            hashes = {str(binary): hashlib.sha256(binary.read_bytes()).hexdigest()}
            e4.check_artifacts(hashes)
            args = argparse.Namespace(cpu=0, duration=15, binary_dir=build)
            command = e4.command_for({"imu_work_us": 150}, args, "/test/pins")
            self.assertEqual(command[3], str(binary))
            binary.write_bytes(b"changed fixture")
            with self.assertRaisesRegex(RuntimeError, "artifact changed"):
                e4.check_artifacts(hashes)

    def test_wakeup_only_cli_does_not_enable_always_preempt(self):
        import os
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run([sys.executable, str(SCRIPT), "--execution-probe", "--wakeup-only",
                                 "--costs", "150,2000,3000", "--cpu", str(cpu), "--dry-run"],
                                capture_output=True, text=True, check=True)
        self.assertEqual(len(result.stdout.splitlines()), 4)
        self.assertNotIn("--imu-preempt always", result.stdout)
        self.assertEqual(result.stdout.count("--imu-preempt wakeup"), 4)
        bad = subprocess.run([sys.executable, str(SCRIPT), "--wakeup-only", "--dry-run"],
                             capture_output=True, text=True)
        self.assertNotEqual(bad.returncode, 0)

    def test_probe_uses_same_variant_control(self):
        control = {"name": "control", "role": "control-start", "repetition": 1, "imu_preempt": "wakeup",
                   "imu_work_us": 150, "process_elapsed_s": 1.2, "metrics": e4.parse_metrics(fixture(), 150, 1)}
        always_control = deepcopy(control)
        always_control.update(name="always-control", imu_preempt="always")
        always_control["metrics"]["window_lidar_reg"]["completed"] = 10
        always_case = deepcopy(always_control)
        always_case.update(name="always-case", role="sweep", imu_work_us=3000)
        always_case["metrics"]["window_lidar_reg"]["completed"] = 5
        matrix, _, _ = e4.summarize([control, always_control, always_case])
        self.assertEqual(matrix[-1]["lidar_reg_rate_ratio_to_control"], 0.5)

    def test_enqueue_probe_parsing(self):
        metrics = e4.parse_metrics(fixture(), 150, 1)
        event = ("[imu_enqueue] ts_ns=1000000100 pid_tgid=123 job=1 stage=0 "
                 "release_ns=1000000000 deadline_ns=1000000050 enq_flags=0x0 dsq=0x1a01 "
                 "policy=7 cpu=0 wakeup=0 late=1 hint_present=1\n")
        summary = ("imu_trace_summary: enqueues=1 wakeup=0 nonwakeup=1 late_wakeup=0 late_nonwakeup=1 "
                   "local_preempt=0 dsq_imu=1 missing_hint=0 wrong_stage=0 wrong_policy=0 emitted=1 lost=0\n")
        events, counters = e4.parse_trace(event + summary, metrics, 0, "wakeup")
        self.assertEqual(events[0]["phase"], "window")
        self.assertEqual(events[0]["route"], "DSQ_IMU")
        self.assertEqual(counters["late_nonwakeup"], 1)
        with self.assertRaises(ValueError):
            e4.parse_trace(event + summary, metrics, 0, "always")
        for old, new in (("policy=7", "policy=0"), ("pid_tgid=123", "pid_tgid=456"), ("emitted=1", "emitted=2")):
            with self.subTest(old=old), self.assertRaises(ValueError):
                e4.parse_trace((event + summary).replace(old, new), metrics, 0, "wakeup")
        always = event.replace("dsq=0x1a01", "dsq=0x8000000000000002")
        events, _ = e4.parse_trace(always + summary, metrics, 0, "always")
        self.assertEqual(events[0]["route"], "local-preempt")
        # An event exactly at T belongs to drain, even if reported much later.
        at_cutoff = event.replace("ts_ns=1000000100", "ts_ns=2000000000")
        events, _ = e4.parse_trace(at_cutoff + summary, metrics, 0, "wakeup")
        self.assertEqual(events[0]["phase"], "drain")

    def test_changed_scheduler_rejected(self):
        loader = Mock()
        loader.poll.return_value = None
        state = {"state": "enabled", "enable_seq": "3", "switch_all": "0"}
        with patch.object(e4, "read_state", side_effect=state.__getitem__):
            e4.check_scheduler(loader, "3")
            for key, value in (("state", "disabled"), ("enable_seq", "4"), ("switch_all", "1")):
                old = state[key]
                state[key] = value
                with self.assertRaisesRegex(RuntimeError, f"{key}={value}"):
                    e4.check_scheduler(loader, "3")
                state[key] = old
            loader.poll.return_value = 1
            with self.assertRaisesRegex(RuntimeError, "loader_returncode=1"):
                e4.check_scheduler(loader, "3")

    def test_embedded_mode_preflight(self):
        with patch.object(e4.subprocess, "check_output", return_value="0x8\n"):
            self.assertEqual(e4.check_embedded_mode("loader"), "0x8")
        for value in ("0x0\n", "unknown\n"):
            with self.subTest(value=value), patch.object(e4.subprocess, "check_output", return_value=value):
                with self.assertRaises(RuntimeError):
                    e4.check_embedded_mode("loader")

    def test_cleanup_stops_only_its_process(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                                   start_new_session=True)
        try:
            e4.stop_process(process)
            self.assertIsNotNone(process.poll())
            e4.stop_process(process)  # Repeated cleanup is harmless.
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


if __name__ == "__main__":
    unittest.main(verbosity=2)
