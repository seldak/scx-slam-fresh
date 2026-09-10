#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compare a fixed feasible task set under both FIFO orders, EDF and sched_ext."""
import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def validate(text):
    rows = list(csv.DictReader(io.StringIO(text)))
    if len(rows) != 24:
        raise RuntimeError('expected 24 completed jobs')
    epoch = None
    for task, count, period, cost in [('A', 14, 50_000_000, 21_000_000),
                                      ('B', 10, 70_000_000, 29_500_000)]:
        jobs = [r for r in rows if r['task'] == task]
        if [int(r['job']) for r in jobs] != list(range(1, count+1)):
            raise RuntimeError('missing/duplicate task jobs')
        for j, row in enumerate(jobs):
            release, start, end, cpu, deadline = [int(row[k]) for k in
                ('release_ns', 'start_ns', 'end_ns', 'cpu_ns', 'deadline_ns')]
            if epoch is None:
                epoch = release
            if release != epoch+j*period or deadline != release+period:
                raise RuntimeError('source window mismatch')
            if not release <= start <= end or not cost <= cpu <= end-start:
                raise RuntimeError('invalid execution accounting')
    return {task: sum(int(r['end_ns']) > int(r['deadline_ns']) for r in rows
                      if r['task'] == task) for task in ('A', 'B')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpu', type=int, required=True)
    parser.add_argument('--housekeeping-cpu', type=int, required=True)
    args = parser.parse_args()
    if os.geteuid() or args.cpu == args.housekeeping_cpu or min(args.cpu,args.housekeeping_cpu)<0:
        parser.error('requires root and distinct nonnegative CPUs')
    if Path('/sys/kernel/sched_ext/state').read_text().strip() != 'disabled':
        parser.error('another scheduler is attached')
    os.sched_setaffinity(0,{args.housekeeping_cpu})
    root = Path('/sys/fs/cgroup')
    if 'cpuset' not in (root/'cgroup.subtree_control').read_text().split():
        parser.error('cpuset controller must already be enabled at cgroup root')
    output_root = ROOT/'results'
    output_root.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='edf-',dir=output_root))
    print(f'Results: {output}',flush=True)
    group = root/f'scx-edf-{os.getpid()}'
    pin = Path('/sys/fs/bpf')/f'scx-edf-{os.getpid()}'
    loader = None
    results = []
    metadata = {'cpu':args.cpu,'housekeeping_cpu':args.housekeeping_cpu,
                'kernel':os.uname().release,'repetitions':3,
                'order':['fifo-a','fifo-b','deadline','scx'],
                'work_ns':[21_000_000,29_500_000],'period_ns':[50_000_000,70_000_000],
                'reservation_ns':[21_200_000,29_700_000],
                'background_server':False,'job_budget_ns':0,'inter_run_pause_s':0.2}
    for name in ('edf_workload','scx_slam_fresh_user'):
        metadata[name+'_sha256']=hashlib.sha256((ROOT/'build'/name).read_bytes()).hexdigest()
    for name in ('sched_rt_runtime_us','sched_rt_period_us'):
        metadata[name]=int((Path('/proc/sys/kernel')/name).read_text())
    metadata['kernel_servers']={}
    for kind in ('fair_server','ext_server'):
        directory=Path('/sys/kernel/debug/sched')/kind/f'cpu{args.cpu}'
        metadata['kernel_servers'][kind]={}
        for p in directory.glob('*'):
            if p.is_file():
                try:
                    metadata['kernel_servers'][kind][p.name]=p.read_text().strip()
                except OSError as error:
                    metadata['kernel_servers'][kind][p.name]=f'unavailable: {error.strerror}'
    metadata['application_revision']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    metadata['scheduler_revision']=(ROOT/'build/scx_fresh.revision').read_text().strip()
    (output/'application.diff').write_bytes(subprocess.check_output(['git','diff','HEAD'],cwd=ROOT))
    (output/'scheduler.diff').write_bytes((ROOT/'build/scx_fresh.diff').read_bytes())
    # Include new untracked tooling too; git diff alone cannot capture it.
    for name in ('demo/edf_workload.c','scripts/test_edf_loaded.py','tests/test_edf_workload.py'):
        (output/Path(name).name).write_bytes((ROOT/name).read_bytes())
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    try:
        group.mkdir()
        (group/'cpuset.mems').write_text((root/'cpuset.mems.effective').read_text())
        (group/'cpuset.cpus').write_text(str(args.cpu))
        (group/'cpuset.cpus.partition').write_text('root')
        if (group/'cpuset.cpus.partition').read_text().strip() != 'root' or \
           (group/'cpuset.cpus.effective').read_text().strip() != str(args.cpu):
            raise RuntimeError('single-CPU root domain could not be established')
        def enter_group():
            (group/'cgroup.procs').write_text(str(os.getpid()))
            os.sched_setaffinity(0,{args.cpu})
        for mode in metadata['order']:
            if mode == 'scx':
                log=(output/'loader.log').open('w')
                loader=subprocess.Popen([str(ROOT/'build/scx_slam_fresh_user'),'--pin',str(pin)],
                                        stdout=log,stderr=subprocess.STDOUT)
                log.close()
                limit=time.monotonic()+10
                while not (pin/'task_hints').exists():
                    if loader.poll() is not None or time.monotonic()>limit:
                        raise RuntimeError('SCX attachment failed; see loader log')
                    time.sleep(.05)
            for repetition in range(1,4):
                run=subprocess.run([str(ROOT/'build/edf_workload'),str(args.cpu),mode,str(pin)],
                                   preexec_fn=enter_group,capture_output=True,text=True,timeout=10)
                prefix=output/f'{mode}-{repetition}'
                prefix.with_suffix('.csv').write_text(run.stdout)
                prefix.with_suffix('.log').write_text(run.stderr)
                if run.returncode:
                    raise RuntimeError(f'{mode}: {run.stderr.strip()}')
                if loader is not None and loader.poll() is not None:
                    raise RuntimeError('scheduler exited during comparison')
                misses=validate(run.stdout)
                results.append(dict(mode=mode,repetition=repetition,misses=misses))
                (output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
                print(f'{mode}-{repetition}: misses={misses}',flush=True)
                # CBS bandwidth can remain charged until the inactive timer
                # retires an exiting task's reservation. Do not overlap cells.
                time.sleep(metadata['inter_run_pause_s'])
        print('All 12 runs accounted for; misses are reported without retries.')
    finally:
        if loader is not None and loader.poll() is None:
            loader.send_signal(signal.SIGINT)
            try:
                loader.wait(timeout=5)
            except subprocess.TimeoutExpired:
                loader.kill();loader.wait()
        for name in ('task_hints','events'):
            (pin/name).unlink(missing_ok=True)
        if pin.exists():
            pin.rmdir()
        if group.exists():
            group.rmdir()
        if 'SUDO_UID' in os.environ:
            for path in [output_root,output,*output.iterdir()]:
                os.chown(path,int(os.environ['SUDO_UID']),int(os.environ['SUDO_GID']))


if __name__ == '__main__':
    main()
