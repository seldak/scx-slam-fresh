#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the dependent workload with a fixed scheduler configuration and audit lifecycle."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def summarize(text, trace, variant, repetition):
    with trace.open() as file:
        jobs = list(csv.DictReader(file))
    result = []
    for line in text.splitlines():
        if not line.startswith('worker='):
            continue
        fields = dict(item.split('=', 1) for item in line.split())
        name = fields.pop('worker')
        row = dict(variant=variant, repetition=repetition, worker=name, **fields)
        selected = [j for j in jobs if j['worker'] == name]
        for metric, end in [('start', 'start_ns'), ('completion', 'end_ns')]:
            ages = sorted(int(j[end]) - int(j['release_ns']) for j in selected)
            row[f'p99_{metric}_us'] = ages[math.ceil(.99 * len(ages))-1] / 1000 if ages else ''
        result.append(row)
    return result


def validate(text, trace, hinted, fifo=False):
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
        priority = {'control': 70, 'imu_processing': 60, 'camera_processing': 50,
                    'estimator': 50}.get(name, 0) if fifo else 0
        policy = 7 if hinted else (1 if priority else 0)
        if life['policy'] != policy or life['priority'] != priority:
            raise RuntimeError(f'{name}: scheduler policy/priority mismatch')
        if (hinted or priority) and not 0 < life['enrollment_begin_ns'] <= life['enrollment_end_ns'] < window['begin_ns']:
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
    results = ROOT / 'results'
    results.mkdir(exist_ok=True)
    if 'SUDO_UID' in os.environ:
        os.chown(results, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))
    output = Path(tempfile.mkdtemp(prefix='dependent-loaded-', dir=results))
    pin = Path('/sys/fs/bpf') / f'dependent-loaded-{os.getpid()}'
    print(f'Raw logs: {output}', flush=True)
    summaries = []
    metadata = {'kernel': os.uname().release, 'cpu': args.cpu,
                'housekeeping_cpu': args.housekeeping_cpu, 'duration_s': 2,
                'hogs': 2, 'repetitions': 3, 'order': 'ordinary-1..3, fifo-1..3, hinted-1..3',
                'fifo_priorities': {'control': 70, 'imu_processing': 60,
                                    'camera_processing': 50, 'estimator': 50},
                'fifo_background_policy': 'SCHED_OTHER',
                'be_slice_cap_us': 2000, 'background_server_us': '2000/10000'}
    for setting in ('sched_rt_runtime_us', 'sched_rt_period_us'):
        metadata[setting] = int((Path('/proc/sys/kernel') / setting).read_text())
    for name in ('dependent_workload', 'scx_slam_fresh_user'):
        metadata[name + '_sha256'] = hashlib.sha256((ROOT / 'build' / name).read_bytes()).hexdigest()
    metadata['application_revision'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    metadata['scheduler_revision'] = (ROOT / 'build/scx_fresh.revision').read_text().strip()
    (output / 'scheduler.diff').write_bytes((ROOT / 'build/scx_fresh.diff').read_bytes())
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    (output / 'application.diff').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT))
    def workload(variant, repetition):
        prefix = f'{variant}-{repetition}'
        trace = output / f'{prefix}.csv'
        command = [str(ROOT / 'build/dependent_workload'), '--cpu', str(args.cpu),
                   '--housekeeping-cpu', str(args.housekeeping_cpu), '--duration', '2',
                   '--hogs', '2', '--trace', str(trace)]
        if variant == 'hinted':
            command += ['--pin', str(pin)]
        elif variant == 'fifo':
            command += ['--policy', 'fifo']
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        (output / f'{prefix}.log').write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f'{prefix}: workload failed')
        validate(result.stdout, trace, variant == 'hinted', variant == 'fifo')
        summaries.extend(summarize(result.stdout, trace, variant, repetition))
        print(f'{prefix}: lifecycle and accounting passed', flush=True)
        for line in result.stdout.splitlines():
            if line.startswith(('control ', 'control_age ')):
                print(line, flush=True)
    loader = None
    try:
        for repetition in range(1, 4):
            workload('ordinary', repetition)
        for repetition in range(1, 4):
            workload('fifo', repetition)
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
            for repetition in range(1, 4):
                workload('hinted', repetition)
                if loader.poll() is not None:
                    raise RuntimeError('scheduler failed')
        with (output / 'summary.csv').open('w') as file:
            writer = csv.DictWriter(file, fieldnames=list(summaries[0]))
            writer.writeheader(); writer.writerows(summaries)
        print('Nine runs complete; callback misses remain reported in summary.csv.')
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
