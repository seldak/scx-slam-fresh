# SPDX-License-Identifier: MIT
"""Run real threads and reconcile fixed-window offers, without latency gates."""
import os
import csv
import tempfile
from pathlib import Path
import subprocess
import unittest
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from test_dependent_loaded import validate, summarize


class ThreadedGraph(unittest.TestCase):
    def test_invalid_policy(self):
        for options in (['--policy', 'unknown'], ['--policy', 'fifo', '--pin', '/unused']):
            result = subprocess.run([str(ROOT / 'build/dependent_workload'), *options],
                                    capture_output=True, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('fifo', result.stderr)

    def test_accounting_with_and_without_contention(self):
        allowed = sorted(os.sched_getaffinity(0))
        if len(allowed) < 2:
            self.skipTest('requires two CPUs')
        worker, housekeeping = (14, 1) if {14, 1} <= set(allowed) else (allowed[1], allowed[0])
        for hogs in (0, 2):
            with self.subTest(hogs=hogs), tempfile.TemporaryDirectory() as directory:
                trace = Path(directory) / 'jobs.csv'
                result = subprocess.run([str(ROOT / 'build/dependent_workload'),
                    '--cpu', str(worker), '--housekeeping-cpu', str(housekeeping),
                    '--duration', '2', '--hogs', str(hogs), '--trace', str(trace)],
                    capture_output=True, text=True, check=True, timeout=15)
                workers, streams = {}, {}
                outcomes = None
                for line in result.stdout.splitlines():
                    if line.startswith(('worker=', 'stream=')):
                        fields = dict(item.split('=', 1) for item in line.split())
                        key = 'worker' if 'worker' in fields else 'stream'
                        name = fields.pop(key)
                        (workers if key == 'worker' else streams)[name] = {k: int(v) for k, v in fields.items()}
                    elif line.startswith('control '):
                        outcomes = {k: int(v) for k, v in (item.split('=') for item in line.split()[1:])}
                self.assertEqual(len(workers), 5 + hogs)
                for row in workers.values():
                    self.assertEqual(row['offered'], row['completed'] + row['dropped'])
                    self.assertEqual(row['pending'] + row['in_flight'], 0)
                    self.assertGreater(row['cpu_us'], 0)
                for name, source, offered in [('imu', 'imu_processing', 400), ('camera', 'camera_processing', 40)]:
                    self.assertEqual(workers[source]['offered'], offered)
                    row = streams[name]
                    self.assertEqual(row['offered'], workers[source]['completed'])
                    self.assertEqual(row['offered'], row['consumed'] + row['dropped'])
                    self.assertEqual(row['pending'] + row['in_flight'], 0)
                self.assertEqual(workers['control']['offered'], 200)
                self.assertEqual(sum(outcomes.values()), workers['control']['completed'])
                with trace.open() as file:
                    rows = list(csv.DictReader(file))
                self.assertEqual(len(rows), sum(w['completed'] for w in workers.values()))
                for name, summary in workers.items():
                    jobs = [r for r in rows if r['worker'] == name]
                    self.assertEqual(len({r['job'] for r in jobs}), len(jobs))
                    late = 0
                    for row in jobs:
                        times = [int(row[k]) for k in ('release_ns', 'assigned_ns', 'start_ns', 'end_ns', 'observed_ns')]
                        self.assertEqual(times, sorted(times))
                        late += int(row['end_ns']) > int(row['deadline_ns'])
                    self.assertEqual(late, summary['late'])
                if hogs == 2:
                    validate(result.stdout, trace, False)
                    summary = summarize(result.stdout, trace, 'ordinary', 1)
                    self.assertEqual(len(summary), 7)
                    for row in summary:
                        self.assertLessEqual(row['p99_start_us'], row['p99_completion_us'])
                        self.assertEqual(int(row['completed']), workers[row['worker']]['completed'])
                    with self.assertRaises(RuntimeError):
                        validate(result.stdout.replace('offered=400', 'offered=399'), trace, False)
                    with self.assertRaises(RuntimeError):
                        validate(result.stdout, trace, True)  # Unenrolled workers must fail.
                    with self.assertRaises(RuntimeError):
                        validate(result.stdout, trace, False, True)
                    with self.assertRaises(RuntimeError):
                        validate(result.stdout.replace('priority=0', 'priority=70'), trace, False)


if __name__ == '__main__':
    unittest.main()
