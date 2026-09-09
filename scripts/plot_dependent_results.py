#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Plot per-run p99 values from test_dependent_loaded.py output."""
import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with args.summary.open() as file:
        rows = list(csv.DictReader(file))
    variants = [v for v in ('ordinary', 'fifo', 'hinted') if any(r['variant'] == v for r in rows)]
    labels = {'ordinary': 'Ordinary Linux', 'fifo': 'FIFO + OTHER', 'hinted': 'Hinted sched_ext'}
    colors = {'ordinary': '#606770', 'fifo': '#ad6500', 'hinted': '#087e8b'}
    fig, axes = plt.subplots(1, 2, figsize=(8, 4), layout='constrained')
    for ax, worker, metric, title in zip(
            axes, ('control', 'estimator'), ('start', 'completion'),
            ('Control start age', 'Estimator completion age')):
        for x, variant in enumerate(variants):
            values = [float(r[f'p99_{metric}_us']) / 1000 for r in rows
                      if r['worker'] == worker and r['variant'] == variant]
            if len(values) != 3:
                raise ValueError(f'{variant}/{worker}: expected three runs')
            ax.scatter([x - .06, x, x + .06], values, color=colors[variant], s=45)
        ax.set(xticks=list(range(len(variants))), xticklabels=[labels[v] for v in variants],
               ylabel='Per-run p99 (ms)', title=title, ylim=(0, None), xlim=(-.4, len(variants)-.6))
        ax.grid(axis='y', alpha=.2)
    fig.suptitle('Dependent synthetic workload · three 2-second runs per variant')
    fig.savefig(args.output, dpi=180)


if __name__ == '__main__':
    main()
