# SPDX-License-Identifier: GPL-2.0
"""Inspect the built loader's embedded object without BPF load/attach privileges."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


LOADER = os.environ.get(
    "LOADER_BIN", str(Path(__file__).resolve().parents[1] / "build/scx_slam_fresh_user")
)


class SchedulerModeTests(unittest.TestCase):
    def test_be_slice_cap_is_opt_in_and_validated(self):
        default = subprocess.check_output([LOADER, "--print-config"], text=True)
        self.assertIn("be_slice_cap_us=0", default)
        for cap in ("0", "2000", "5000"):
            config = subprocess.check_output(
                [LOADER, "--print-config", "--be-slice-cap-us", cap], text=True)
            self.assertIn(f"be_slice_cap_us={cap}", config)
            self.assertIn("deadline_grace_us=1000", config)
            self.assertIn("imu_preempt=wakeup", config)
        for cap in ("-1", "", "+1", "1x", "18446744073709552"):
            result = subprocess.run([LOADER, "--print-config", "--be-slice-cap-us", cap],
                                    capture_output=True)
            self.assertNotEqual(result.returncode, 0)

    def test_probe_is_opt_in(self):
        default = subprocess.check_output([LOADER, "--print-config"], text=True)
        self.assertIn("imu_preempt=wakeup trace_imu=0", default)
        self.assertIn("execution_cpu=-1", default)
        self.assertIn("trace_est=0", default)
        self.assertIn("deadline_grace_us=1000", default)
        always = subprocess.check_output([LOADER, "--print-config", "--imu-preempt", "always", "--trace-imu"], text=True)
        self.assertIn("imu_preempt=always trace_imu=1", always)
        execution = subprocess.check_output([LOADER, "--print-config", "--trace-execution-cpu", "0"], text=True)
        self.assertIn("imu_preempt=wakeup trace_imu=0 execution_cpu=0", execution)
        estimator = subprocess.check_output([LOADER, "--print-config", "--trace-estimator"], text=True)
        self.assertIn("imu_preempt=wakeup trace_imu=0 execution_cpu=-1 trace_est=1", estimator)
        zero_grace = subprocess.check_output(
            [LOADER, "--print-config", "--deadline-grace-us", "0"], text=True)
        self.assertIn("deadline_grace_us=0", zero_grace)
        for args in (("--imu-preempt", "invalid", "--print-config"), ("--imu-preempt",)):
            result = subprocess.run([LOADER, *args], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
        for cpu in ("-1", "", "abc", "9999999999999999999999", "1x"):
            result = subprocess.run([LOADER, "--print-config", "--trace-execution-cpu", cpu], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
        for grace in ("-1", "", "+1", "1x", "999999999999999999999999999999"):
            result = subprocess.run([LOADER, "--print-config", "--deadline-grace-us", grace],
                                    capture_output=True)
            self.assertNotEqual(result.returncode, 0)

    def test_embedded_flags(self):
        result = subprocess.run([LOADER, "--print-ops-flags"],
                                capture_output=True, text=True, check=True, timeout=10)
        flags = int(result.stdout.strip(), 16)
        expected = 0 if os.environ.get("EXPECTED_FULL_SWITCH", "0") == "1" else 8
        self.assertEqual(flags, expected)
        self.assertNotIn("setrlimit", result.stderr)
        print(f"Embedded scheduler flags: {flags:#x}", flush=True)

    def test_inspection_cannot_be_combined_with_attach(self):
        with tempfile.TemporaryDirectory(prefix="scx-mode-test-") as directory:
            pin_dir = Path(directory) / "pins"
            result = subprocess.run([LOADER, "--print-ops-flags", "--pin", str(pin_dir)],
                                    capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(pin_dir.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
