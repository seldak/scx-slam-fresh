#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the dependent workload with a fixed scheduler configuration and audit lifecycle."""
import argparse
import csv
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def validate(text, trace, hinted):
    workers, lifecycle, streams = {}, {}, {}
    window = None
    outcomes = None
    for line in text.splitlines():
        if line.startswith('lifecycle '):
            fields = dict(x.split('=', 1) for x in line.split()[1:])
            name = fields.pop('worker')
            lifecycle[name] = {k: int(v) for k, v in fields.items()}
        elif line.startswith(('worker=', 'stream=')):
            fields = dict(x.split('=', 1) for x in line.split())
            key = 'worker' if 'worker' in fields else 'stream'
            name = fields.pop(key)
            (workers if key == 'worker' else streams)[name] = {k: int(v) for k, v in fields.items()}
        elif line.startswith('measurement_window_ns='):
            window = {k: int(v) for k, v in (x.split('=') for x in line.split())}
        elif line.startswith('control '):
            outcomes = {k: int(v) for k, v in (x.split('=') for x in line.split()[1:])}
    expected = {'imu_processing', 'camera_processing', 'estimator', 'control', 'mapping', 'hog0', 'hog1'}
    if set(workers) != expected or set(lifecycle) != expected or not window:
        raise RuntimeError('incomplete worker/lifecycle output')
    with trace.open() as file:
        rows = list(csv.DictReader(file))
    if len(rows) != sum(w['completed'] for w in workers.values()):
        raise RuntimeError('trace completion count mismatch')
    for name, w in workers.items():
        if w['offered'] != w['completed'] + w['dropped'] or w['pending'] or w['in_flight']:
            raise RuntimeError(f'{name}: incomplete accounting')
        life = lifecycle[name]
        if hinted and not 0 < life['enrollment_begin_ns'] <= life['enrollment_end_ns'] < window['begin_ns']:
            raise RuntimeError(f'{name}: enrollment overlaps measured releases')
        jobs = [r for r in rows if r['worker'] == name]
        if len(jobs) != w['completed'] or len({r['job'] for r in jobs}) != len(jobs):
            raise RuntimeError(f'{name}: duplicate/missing trace jobs')
        late = 0
        for r in jobs:
            stamps = [int(r[k]) for k in ('release_ns', 'assigned_ns', 'start_ns', 'end_ns', 'observed_ns')]
            if stamps != sorted(stamps) or stamps[-1] > life['shutdown_begin_ns']:
                raise RuntimeError(f'{name}: invalid job lifecycle')
            late += int(r['end_ns']) > int(r['deadline_ns'])
        if late != w['late']:
            raise RuntimeError(f'{name}: deadline accounting mismatch')
    for source, worker, expected_count in [('imu', 'imu_processing', 400), ('camera', 'camera_processing', 40)]:
        s = streams[source]
        if workers[worker]['offered'] != expected_count or s['offered'] != workers[worker]['completed']:
            raise RuntimeError('source window mismatch')
        if s['offered'] != s['consumed'] + s['dropped'] or s['pending'] or s['in_flight']:
            raise RuntimeError('measurement accounting mismatch')
    if workers['control']['offered'] != 200 or sum(outcomes.values()) != workers['control']['completed']:
        raise RuntimeError('control accounting mismatch')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpu', type=int, required=True)
    parser.add_argument('--housekeeping-cpu', type=int, required=True)
    args = parser.parse_args()
    if os.geteuid() or args.cpu == args.housekeeping_cpu or min(args.cpu, args.housekeeping_cpu) < 0:
        parser.error('requires root and distinct nonnegative CPUs')
    if Path('/sys/kernel/sched_ext/state').read_text().strip() != 'disabled':
        parser.error('stop the existing scheduler first')
    output = Path(tempfile.mkdtemp(prefix='dependent-loaded-'))
    pin = Path('/sys/fs/bpf') / f'dependent-loaded-{os.getpid()}'
    print(f'Raw logs: {output}', flush=True)
    loader = None
    try:
        with (output / 'loader.log').open('w') as log:
            loader = subprocess.Popen(['taskset', '-c', str(args.housekeeping_cpu),
                str(ROOT / 'build/scx_slam_fresh_user'), '--pin', str(pin),
                '--be-slice-cap-us', '2000', '--background-server-us', '2000/10000'],
                stdout=log, stderr=subprocess.STDOUT)
            end = time.monotonic() + 10
            while not (pin / 'task_hints').exists():
                if loader.poll() is not None or time.monotonic() > end:
                    raise RuntimeError('scheduler attachment failed')
                time.sleep(.05)
            result = subprocess.run([str(ROOT / 'build/dependent_workload'),
                '--cpu', str(args.cpu), '--housekeeping-cpu', str(args.housekeeping_cpu),
                '--duration', '2', '--hogs', '2', '--pin', str(pin),
                '--trace', str(output / 'jobs.csv')], capture_output=True, text=True, timeout=15)
            (output / 'workload.log').write_text(result.stdout + result.stderr)
            print(result.stdout, end='', flush=True)
            if result.returncode or loader.poll() is not None:
                raise RuntimeError('workload/scheduler failed')
            validate(result.stdout, output / 'jobs.csv', True)
            print('Lifecycle and accounting gates passed; callback misses remain reported.')
    finally:
        if loader is not None:
            if loader.poll() is None:
                loader.send_signal(signal.SIGINT)
                try:
                    loader.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    loader.kill(); loader.wait()
        for name in ('task_hints', 'events'):
            (pin / name).unlink(missing_ok=True)
        if pin.exists():
            pin.rmdir()
        if 'SUDO_UID' in os.environ:
            for path in [output, *output.iterdir()]:
                os.chown(path, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))


if __name__ == '__main__':
    main()
