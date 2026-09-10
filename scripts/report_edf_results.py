#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Report deadline margins from a complete independent EDF comparison."""

import argparse
import csv
import io
import json
from pathlib import Path

from test_edf_loaded import validate


MODES = ('fifo-a', 'fifo-b', 'deadline', 'scx')


def summarize_trace(text):
    """Positive completion margin means the job finished before its deadline."""
    misses = validate(text)
    rows = list(csv.DictReader(io.StringIO(text)))
    summary = {}

    for task in ('A', 'B'):
        jobs = [row for row in rows if row['task'] == task]
        margins = [int(row['deadline_ns']) - int(row['end_ns']) for row in jobs]
        start_delays = [int(row['start_ns']) - int(row['release_ns']) for row in jobs]
        response_times = [int(row['end_ns']) - int(row['release_ns']) for row in jobs]
        worst_index = min(range(len(jobs)), key=lambda index: margins[index])

        summary[task] = {
            'completed': len(jobs),
            'misses': misses[task],
            'minimum_margin_ns': margins[worst_index],
            'minimum_margin_job': int(jobs[worst_index]['job']),
            'maximum_start_delay_ns': max(start_delays),
            'maximum_response_ns': max(response_times),
            'cpu_ns': sum(int(row['cpu_ns']) for row in jobs),
        }

    return summary


def summarize_directory(directory):
    """Require all twelve cells; never summarize a partial matrix as complete."""
    runs = []
    for mode in MODES:
        for repetition in range(1, 4):
            trace = directory / f'{mode}-{repetition}.csv'
            runs.append({
                'mode': mode,
                'repetition': repetition,
                'tasks': summarize_trace(trace.read_text()),
            })
    return runs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--json', action='store_true', help='emit exact nanosecond metrics')
    args = parser.parse_args()

    try:
        runs = summarize_directory(args.directory)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.exit(1, f'error: {error}\n')

    if args.json:
        print(json.dumps(runs, indent=2))
        return

    print('Positive margin = early completion; negative margin = missed deadline.')
    print('These are observed extrema, not worst-case timing guarantees.')
    print('mode rep task completed misses min_margin_ms worst_job max_start_ms cpu_ms')
    for run in runs:
        for task, metrics in run['tasks'].items():
            print(
                f"{run['mode']} {run['repetition']} {task} "
                f"{metrics['completed']} {metrics['misses']} "
                f"{metrics['minimum_margin_ns'] / 1_000_000:.3f} "
                f"{metrics['minimum_margin_job']} "
                f"{metrics['maximum_start_delay_ns'] / 1_000_000:.3f} "
                f"{metrics['cpu_ns'] / 1_000_000:.3f}"
            )


if __name__ == '__main__':
    main()
