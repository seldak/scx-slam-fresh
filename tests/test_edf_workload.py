# SPDX-License-Identifier: MIT
"""Check the analytical contrast and trace conservation independently of timing noise."""
import csv
import io
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from test_edf_loaded import validate


def simulate(mode):
    pending=[]
    misses=0
    # Half-millisecond ticks represent the compute costs exactly.
    for t in range(1600):
        for task,period,work in [('A',100,42),('B',140,59)]:
            if t<1400 and t%period==0:
                pending.append(dict(task=task,deadline=t+period,remaining=work))
        if not pending:
            continue
        chosen=min(pending,key=lambda j:j['deadline'] if mode=='edf' else
                   (j['task'] != mode,j['deadline']))
        chosen['remaining']-=1
        if not chosen['remaining']:
            misses+=t+1>chosen['deadline']
            pending.remove(chosen)
    assert not pending
    return misses


class EDFTests(unittest.TestCase):
    def test_feasibility_and_both_priority_orders(self):
        self.assertEqual(simulate('edf'),0)
        self.assertGreater(simulate('A'),0)
        self.assertGreater(simulate('B'),0)

    def test_trace_validation(self):
        rows=[]
        for task,count,period,cost in [('A',14,50_000_000,21_000_000),('B',10,70_000_000,29_500_000)]:
            for j in range(count):
                release=1_000_000_000+j*period
                rows.append(dict(task=task,job=j+1,release_ns=release,start_ns=release,
                                 end_ns=release+cost,cpu_ns=cost,deadline_ns=release+period))
        def serialize():
            output=io.StringIO();writer=csv.DictWriter(output,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows);return output.getvalue()
        self.assertEqual(validate(serialize()),{'A':0,'B':0})
        rows[0]['release_ns']+=1
        with self.assertRaises(RuntimeError):validate(serialize())
        rows[0]['release_ns']-=1
        rows.pop()
        with self.assertRaises(RuntimeError):validate(serialize())

    def test_cli_rejects_unknown_policy(self):
        run=subprocess.run([str(ROOT/'build/edf_workload'),'14','invalid','unused'],capture_output=True)
        self.assertEqual(run.returncode,2)


if __name__=='__main__':unittest.main()
