# SPDX-License-Identifier: MIT
"""Synthetic switch streams only; these tests are not scheduler evidence."""

from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import e4_execution as execution


IMU = (10 << 32) | 11
BE = (10 << 32) | 12
CFS = (20 << 32) | 20


def fixture():
    metrics = {"measurement": {"start_ns": 100, "end_ns": 200, "window_ns": 100},
               "imu_identity": {"pid_tgid": IMU}, "window_imu_prop": {"cpu_ns": 38}}

    def task(key):
        return {"key": key, "policy": 0 if key == CFS else 7, "stage": 0 if key == IMU else 6,
                "slice_ns": 5000, "last_dsq": 0x1A01 if key == IMU else 0xBE01,
                "insert_ns": 70, "requested_slice_ns": 0,
                "comm_hex": ("imu_prop" if key == IMU else "lidar_reg" if key == BE else "generator").encode().ljust(16, b"\0").hex()}

    def event(kind, ts, prev=IMU, nxt=BE, runnable=1, preempt=0, state=0, syscall="none", cpu=0):
        e = {"kind": kind, "ts_ns": ts, "urgent_key": IMU, "cpu": cpu, "runnable": runnable,
             "preempt": preempt, "prev_state": state, "callback_delta_ns": 0,
             "syscall_id": 230 if syscall == "clock_nanosleep" else (1 << 64) - 1, "syscall": syscall}
        e.update({f"task_{k}": v for k, v in task(prev).items()})
        e.update({f"next_{k}": v for k, v in task(nxt).items()})
        return e

    events = [event(3, 79), event(1, 80, CFS, IMU),
              event(4, 109, runnable=0),
              event(1, 110, runnable=0, state=1, syscall="clock_nanosleep"),
              event(2, 120, cpu=1),  # Remote wake splits blocked from waiting.
              event(1, 130, BE, CFS), event(3, 149), event(1, 150, CFS, IMU),
              event(4, 179), event(1, 180, preempt=1, state=1),
              event(3, 209), event(1, 210, BE, IMU),
              event(4, 239), event(1, 240)]
    return metrics, events


def log(events, lost=0):
    return "\n".join("[execution] " + " ".join(f"{k}={v}" for k, v in e.items()) for e in events) + (
        f"\nexecution_summary: emitted={len(events)} lost={lost} identity_conflicts=0\n")


def analyze(raw=None, lost=0):
    metrics, original = fixture()
    events, counters = execution.parse_execution(log(original if raw is None else raw, lost), metrics, 0)
    return execution.analyze_execution(events, counters, metrics)


class ExecutionTests(unittest.TestCase):
    def test_wakeup_splits_blocked_and_runnable_wait(self):
        summary, intervals, away, occupancy = analyze()
        self.assertTrue(summary["valid"], summary)
        self.assertEqual(summary["imu_switch_residency_ns"], 40)
        self.assertEqual(summary["imu_blocked_ns"], 10)
        self.assertEqual(summary["imu_runnable_wait_ns"], 50)
        self.assertEqual(sum(r["duration_ns"] for r in intervals), 100)
        blocked = [r for r in occupancy if r["imu_state"] == "blocked"]
        self.assertEqual(blocked[0]["blocked_syscall"], "clock_nanosleep")
        self.assertEqual(blocked[0]["window_ns"], 10)
        waiting = [r for r in occupancy if r["imu_state"] == "runnable_wait"]
        self.assertEqual(sum(r["window_ns"] for r in waiting if r["policy"] == 0), 20)
        self.assertEqual(sum(r["window_ns"] for r in waiting if r["policy"] == 7), 30)
        self.assertEqual(away[0]["next_residency_ns"], 20)
        self.assertEqual(away[0]["next_key"], BE)
        self.assertEqual(away[0]["imu_start_slice_ns"], 5000)
        self.assertEqual(summary["imu_callback_residency_ns"], 39)

    def test_drain_cannot_improve_window(self):
        _, events = fixture()
        for e in events:
            if e["ts_ns"] > 200:
                e["ts_ns"] += 1000
        original = analyze()[0]
        changed = analyze(events)[0]
        self.assertTrue(changed["valid"])
        for field in ("imu_switch_residency_ns", "imu_blocked_ns", "imu_runnable_wait_ns"):
            self.assertEqual(changed[field], original[field])

    def test_preemption_ignores_nonzero_raw_state(self):
        metrics, events = fixture()
        current = log(events)
        self.assertEqual(execution.parse_execution(current, metrics, 0),
                         execution.parse_execution(current.replace("urgent_key=", "imu_key="), metrics, 0))
        summary, _, _, _ = analyze()
        self.assertEqual(summary["imu_blocked_ns"], 10)
        metrics, events = fixture()
        events[9]["runnable"] = 0
        with self.assertRaisesRegex(ValueError, "raw state/preempt"):
            execution.parse_execution(log(events), metrics, 0)

    def test_loss_disables_gap_attribution(self):
        summary, intervals, away, occupancy = analyze(lost=1)
        self.assertFalse(summary["valid"])
        self.assertIn("ring loss", summary["reason"])
        self.assertEqual((intervals, away, occupancy), ([], [], []))

    def test_missing_switch_is_not_silently_filled(self):
        _, events = fixture()
        del events[5]  # Lose BE -> CFS transition, even with forged loss=0.
        summary, *_ = analyze(events)
        self.assertFalse(summary["valid"])
        self.assertIn("discontinuous", summary["reason"])

    def test_missing_wakeup_is_not_all_blocked(self):
        _, events = fixture()
        del events[4]
        # No event-loss counter would make this indistinguishable. Honest loss
        # accounting must suppress conclusions even if the switch chain holds.
        summary, *_ = analyze(events, lost=1)
        self.assertFalse(summary["valid"])

    def test_switch_and_callback_coverage_are_required(self):
        _, events = fixture()
        for remove in ((0, 1), (10, 11, 12, 13), (0, 2, 6, 8, 10, 12)):
            with self.subTest(remove=remove):
                kept = [e for i, e in enumerate(events) if i not in remove]
                summary, *_ = analyze(kept)
                self.assertFalse(summary["valid"])

    def test_identity_policy_and_migration_rejected(self):
        metrics, events = fixture()
        for index, key, value in ((2, "urgent_key", BE), (2, "task_policy", 0),
                                   (2, "task_stage", 6), (2, "cpu", 1), (3, "cpu", 1)):
            bad = deepcopy(events)
            bad[index][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                execution.parse_execution(log(bad), metrics, 0)

    def test_bad_count_and_encoding_rejected(self):
        metrics, events = fixture()
        text = log(events)
        for bad in (text.replace("emitted=14", "emitted=15"),
                    text.replace(events[0]["task_comm_hex"], "00"),
                    text + "execution_summary: emitted=14 lost=0 identity_conflicts=0\n"):
            with self.assertRaises(ValueError):
                execution.parse_execution(bad, metrics, 0)

    def test_wakeup_switch_race_is_explicitly_unknown(self):
        _, events = fixture()
        events[2]["ts_ns"] = 108
        events[4]["ts_ns"] = 109  # Wake between stopping and switch-out.
        summary, intervals, *_ = analyze(events)
        self.assertFalse(summary["valid"])
        self.assertTrue(any(r["blocked_syscall"] == "wake_switch_race" for r in intervals))

    def test_early_completion_is_not_sleep_starvation(self):
        _, events = fixture()
        events[9].update(runnable=0, preempt=0, prev_state=128, syscall="exit")
        summary, *_ = analyze(events)
        self.assertTrue(summary["valid"])
        self.assertEqual(summary["imu_exit_path_ns"], 20)
        self.assertEqual(summary["imu_blocked_ns"], 10)

    def test_old_wake_does_not_taint_a_later_sleep(self):
        _, events = fixture()
        old_wake = deepcopy(events[4])
        old_wake["ts_ns"] = 90  # A no-switch wake before the next stopping.
        events.append(old_wake)
        summary, *_ = analyze(events)
        self.assertTrue(summary["valid"])
        self.assertEqual(summary["imu_blocked_ns"], 10)

    def test_probe_cli_grid(self):
        script = Path(__file__).resolve().parents[1] / "scripts/run_e4_eval.py"
        import os
        cpu = min(os.sched_getaffinity(0))
        result = subprocess.run([sys.executable, str(script), "--execution-probe", "--cpu", str(cpu),
                                 "--dry-run"], capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 6)
        self.assertEqual(sum("sweep-3000" in line for line in lines), 2)
        self.assertEqual(sum("control-" in line for line in lines), 4)
        self.assertTrue(all("trace_imu=1" in line and f"execution_cpu={cpu}" in line for line in lines))
        self.assertNotIn("--imu-work-us 2000", result.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
