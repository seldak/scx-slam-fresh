# SPDX-License-Identifier: MIT
"""Estimator lane/perf report tests; all inputs are synthetic."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/e4_perf.py"
SPEC = importlib.util.spec_from_file_location("e4_perf", SCRIPT)
e4_perf = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(e4_perf)


def metrics():
    key = (100 << 32) | 101
    return {"measurement": {"start_ns": 1_000_000_000, "end_ns": 4_000_000_000},
            "imu_identity": {"pid_tgid": key},
            "focus_vision_fe": {"job": 4, "release_ns": 1_050_000_000,
                                "completion_ns": 1_070_000_000, "age_ns": 20_000_000, "late": 0},
            "focus_state_est": {"job": 4, "release_ns": 1_050_000_000,
                                "completion_ns": 1_085_000_000, "age_ns": 35_000_000, "late": 1}}


def trace(lost=0, ts=1_100_000_000):
    key = (100 << 32) | 102
    enqueue = (f"[stage_enqueue] ts_ns={ts} pid_tgid={key} job=4 stage=2 release_ns=1066000000 "
               "deadline_ns=1099000000 enq_flags=0x1 dsq=0x5a1e slice_ns=20000000 vruntime=3 "
               "exec_ns=100 policy=7 cpu=0 overrun=0 state_present=1")
    job5 = (f"[stage_enqueue] ts_ns={ts + 50_000_000} pid_tgid={key} job=5 stage=2 release_ns=1099000000 "
            "deadline_ns=1132000000 enq_flags=0x1 dsq=0xfe01 slice_ns=20000000 vruntime=4 "
            "exec_ns=0 policy=7 cpu=0 overrun=0 state_present=1")
    job6 = (f"[stage_enqueue] ts_ns={ts + 100_000_000} pid_tgid={key} job=6 stage=2 release_ns=1132000000 "
            "deadline_ns=1165000000 enq_flags=0x1 dsq=0x5a1e slice_ns=20000000 vruntime=5 "
            "exec_ns=0 policy=7 cpu=0 overrun=0 state_present=1")
    event = (f"[evt] DEADLINE_MISS stage=2 job=4 pid_tgid=0x{key:x} age=34.000ms exec=0.100ms "
             f"ts_ns={ts} release_ns=1066000000 deadline_ns=1099000000")
    lpre_key = (100 << 32) | 103
    lpre_overrun = (f"[evt] BUDGET_OVERRUN stage=5 job=1 pid_tgid=0x{lpre_key:x} age=99.000ms "
                     f"exec=7.100ms ts_ns={ts - 1_000_000} release_ns={ts - 100_000_000} deadline_ns={ts}")
    lpre_demotion = (f"[evt] BUDGET_DEMOTION stage=5 job=1 pid_tgid=0x{lpre_key:x} age=99.100ms "
                      f"exec=7.100ms ts_ns={ts - 900_000} release_ns={ts - 100_000_000} deadline_ns={ts}")
    return (f"{enqueue}\n{job5}\n{job6}\n{event}\n{lpre_overrun}\n{lpre_demotion}\n"
            f"stage_trace_summary: enqueues=3 emitted=3 lost={lost}\n")


def case(name, cost, with_miss, grace=1000, budget=10000, class_id=1, ts=1_100_000_000):
    text = trace(ts=ts)
    if not with_miss:
        text = "\n".join(line for line in text.splitlines() if not line.startswith("[evt]")) + "\n"
    return {"name": name, "repetition": 1, "imu_work_us": cost, "deadline_grace_us": grace,
            "lidar_pre_budget_us": budget, "lidar_pre_class_id": class_id,
            "metrics": metrics(), "trace": text}


class PerfTests(unittest.TestCase):
    def test_estimator_lane_and_exact_event_time(self):
        parsed = e4_perf.parse_estimator(trace(), metrics(), 0)
        self.assertEqual(parsed["tid"], 102)
        self.assertEqual(parsed["enqueues"][0]["lane"], "DSQ_STALE")
        self.assertEqual(parsed["enqueues"][0]["phase"], "window")
        self.assertEqual(parsed["first_events"]["DEADLINE_MISS"]["relative_ns"], 100_000_000)
        self.assertIsNone(parsed["first_events"]["STALE_DEMOTION"])
        self.assertEqual([event["kind"] for event in parsed["lidar_pre_events"]],
                         ["BUDGET_OVERRUN", "BUDGET_DEMOTION"])
        self.assertLess(parsed["lidar_pre_events"][1]["deadline_delta_ns"], 0)

    def test_lane_stream_rejects_loss_identity_policy_and_unknown_lane(self):
        legacy = trace().replace("stage_trace_summary", "est_trace_summary").replace(
            "stage_enqueue", "est_enqueue")
        self.assertEqual(e4_perf.parse_estimator(legacy, metrics(), 0),
                         e4_perf.parse_estimator(trace(), metrics(), 0))
        for old, new in (("lost=0", "lost=1"), ("emitted=3", "emitted=2"),
                         ("policy=7", "policy=0"), ("cpu=0", "cpu=1"),
                         ("dsq=0x5a1e", "dsq=0x123"), ("stage=2", "stage=1")):
            with self.subTest(old=old), self.assertRaises(ValueError):
                e4_perf.parse_estimator(trace().replace(old, new), metrics(), 0)

    def test_report_bounds_and_perf_loss_detection(self):
        begin, end = e4_perf.report_bounds(metrics(), 150_000_000)
        self.assertEqual((begin, end), (1_050_000_000, 3_150_000_000))
        command = e4_perf.timehist_command("perf.data", begin, end, tid=102)
        self.assertIn("1.050000000,3.150000000", command)
        self.assertEqual(command[-2:], ["--tid", "102"])
        self.assertFalse(e4_perf.perf_loss_reported("lost 0 events"))
        self.assertTrue(e4_perf.perf_loss_reported("PERF_RECORD_LOST: lost 2 events"))

    def test_pair_uses_the_observed_70_percent_anchor(self):
        cases = [case("load-65", 3250, False), case("load-70", 3500, True),
                 case("load-70-zero-grace", 3500, True, grace=0, ts=1_200_000_000),
                 case("load-70-budget6", 3500, True, budget=6000, ts=1_250_000_000),
                 case("load-70-lpre-be", 3500, True, class_id=0, ts=1_300_000_000)]
        with tempfile.TemporaryDirectory(prefix="scx-e4-perf-test-") as directory:
            output = Path(directory)
            for item in cases:
                (output / f"{item['name']}.loader.txt").write_text(item.pop("trace"))
                (output / f"{item['name']}.record.stderr.txt").write_text("")
                (output / f"{item['name']}.perf.data").write_bytes(b"synthetic")

            def fake_report(command, path):
                path.write_text("test 1/1 [000] 1.1: sched:sched_switch: synthetic\n")
                path.with_suffix(path.suffix + ".stderr").write_text("")

            with patch.object(e4_perf, "run_report", side_effect=fake_report):
                e4_perf.report_perf_cases(output, cases, 0)
            for name in ("load-65", "load-70", "load-70-zero-grace", "load-70-budget6",
                         "load-70-lpre-be"):
                metadata = json.loads((output / f"{name}.estimator-focus.json").read_text())
                self.assertEqual(metadata["anchor_case"], "load-70")
                self.assertEqual(metadata["anchor_offset_ns"], 100_000_000)
                self.assertEqual(metadata["report_start_ns"], 1_000_000_000)
                self.assertEqual(metadata["report_end_ns"], 3_100_000_000)
                self.assertFalse(metadata["perf_loss_reported"])
                self.assertTrue(metadata["perf_switch_events_present"])
            load70 = json.loads((output / "load-70.estimator-focus.json").read_text())
            self.assertEqual(load70["enqueue_before_first_miss"]["lane"], "DSQ_STALE")
            self.assertEqual(load70["enqueue_after_first_miss"]["lane"], "DSQ_STALE")
            self.assertEqual(load70["focus_lane_counts"]["DSQ_STALE"], 2)
            self.assertEqual(load70["focus_jobs"]["vision_fe"]["age_ns"], 20_000_000)
            self.assertTrue(load70["estimator_job4"]["deadline_miss_observed"])
            self.assertEqual(load70["estimator_jobs_5_plus"]["first_lane_by_job"]["5"], "DSQ_FE")
            self.assertEqual(load70["estimator_jobs_5_plus"]["jobs_with_non_fe_lane"], [6])
            self.assertEqual(load70["lidar_pre_budget_path"]["first_budget_demotion"]["job"], 1)
            self.assertEqual(load70["lidar_pre_budget_path"]["demotion_destination"], "DSQ_BE")
            self.assertEqual(load70["lidar_pre_initial_class"]["destination"], "DSQ_FE")
            load70_be = json.loads((output / "load-70-lpre-be.estimator-focus.json").read_text())
            self.assertEqual(load70_be["anchor_case"], "load-70")
            self.assertEqual(load70_be["lidar_pre_initial_class"]["class"], "BE")
            self.assertEqual(load70_be["lidar_pre_initial_class"]["destination"], "DSQ_BE")


if __name__ == "__main__":
    unittest.main(verbosity=2)
