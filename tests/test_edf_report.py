# SPDX-License-Identifier: MIT
"""Check signed timing margins and rejection of incomplete evidence."""

import csv
import io
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from report_edf_results import MODES, summarize_directory, summarize_trace


def trace_text(late=False):
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(('task', 'job', 'release_ns', 'start_ns', 'end_ns',
                     'cpu_ns', 'deadline_ns'))
    for task, count, period, cost in (
        ('A', 14, 50_000_000, 21_000_000),
        ('B', 10, 70_000_000, 29_500_000),
    ):
        for job in range(1, count + 1):
            release = 1_000_000_000 + (job - 1) * period
            end = release + cost
            if late and task == 'B' and job == 3:
                end = release + period + 123
            writer.writerow((task, job, release, release, end, cost, release + period))
    return output.getvalue()


class ReportTests(unittest.TestCase):
    def test_signed_margin_and_worst_job(self):
        summary = summarize_trace(trace_text(late=True))
        self.assertEqual(summary['A']['minimum_margin_ns'], 29_000_000)
        self.assertEqual(summary['B']['minimum_margin_ns'], -123)
        self.assertEqual(summary['B']['minimum_margin_job'], 3)
        self.assertEqual(summary['B']['misses'], 1)
        self.assertEqual(summary['B']['cpu_ns'], 295_000_000)

    def test_complete_matrix_required(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with self.assertRaises(FileNotFoundError):
                summarize_directory(directory)

            for mode in MODES:
                for repetition in range(1, 4):
                    (directory / f'{mode}-{repetition}.csv').write_text(trace_text())

            self.assertEqual(len(summarize_directory(directory)), 12)
            (directory / 'scx-3.csv').write_text('task,job\n')
            with self.assertRaises(RuntimeError):
                summarize_directory(directory)


if __name__ == '__main__':
    unittest.main()
