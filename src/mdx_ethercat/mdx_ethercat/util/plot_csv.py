"""Plot a recorded test CSV (e.g. from `pp_test -p record:=true`) in an interactive window.

Usage: ros2 run mdx_ethercat plot_csv [file.csv] [-t pp|csp] [-c position velocity ...]
Without a file the newest CSV under log/ in the current directory is used. One stacked subplot per
column with a shared time axis; use the toolbar to zoom, pan and save.
"""
import argparse
import csv
import glob
import os
import sys

import matplotlib.pyplot as plt

DEFAULT_COLUMNS = ['position', 'target_position', 'velocity', 'current', 'following_error',
                   'status_word']
# Columns drawn on the same subplot
GROUPS = {'target_position': 'position'}


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'{path}: no data rows')
    return {k: [float(r[k]) if r[k] not in ('', 'None') else float('nan') for r in rows]
            for k in rows[0]}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('file', nargs='?', help='CSV file (default: newest CSV under log/)')
    parser.add_argument('-t', '--test', help='only consider recordings of this test, e.g. pp or csp')
    parser.add_argument('-c', '--columns', nargs='+', help='columns to plot')
    args = parser.parse_args()

    path = args.file
    if not path:
        files = glob.glob(f'log/**/{args.test or ""}*.csv', recursive=True)
        if not files:
            sys.exit('no matching CSV given and none found under log/')
        path = max(files, key=os.path.getmtime)
    data = load(path)

    columns = args.columns or [c for c in DEFAULT_COLUMNS if c in data]
    unknown = [c for c in columns if c not in data or c == 'time_s']
    if unknown:
        sys.exit(f'unknown column(s) {unknown}; available: {[k for k in data if k != "time_s"]}')

    panels = {}
    for col in columns:
        panels.setdefault(GROUPS.get(col, col) if GROUPS.get(col, col) in columns else col,
                          []).append(col)

    fig, axes = plt.subplots(len(panels), 1, sharex=True, squeeze=False,
                             figsize=(11, 2.2 * len(panels) + 1))
    for ax, (name, cols) in zip(axes[:, 0], panels.items()):
        for col in cols:
            ax.plot(data['time_s'], data[col], label=col, linewidth=1)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        if len(cols) > 1:
            ax.legend(loc='upper left')
    axes[-1, 0].set_xlabel('time (s)')
    fig.suptitle(os.path.basename(path))
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
