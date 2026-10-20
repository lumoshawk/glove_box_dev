#!/usr/bin/env python3
"""Plot a recorded test CSV (e.g. from `pp_test -p record:=true`) in an interactive window.

Usage: ./src/eyou_ethercat/eyou_ethercat/util/plot_csv.py [file.csv] [-t pp|csp|pv] [-c position velocity ...] [--raw]
       ros2 run eyou_ethercat plot_csv [file.csv] [-t pp|csp|pv] [-c position velocity ...] [--raw]
Velocity columns plot in rpm and position columns in deg (--raw keeps drive counts and pulses/s).
`position_velocity` (black) is velocity derived from position over a 0.25 s window, added next to the
drive's jittery `velocity`.
Without a file the newest CSV under log/ in the current directory is used. One stacked subplot per
column with a shared time axis; use the toolbar to zoom, pan and save.
"""
import argparse
import csv
import glob
import os
import sys

import matplotlib.pyplot as plt

DEFAULT_COLUMNS = ['position', 'target_position', 'velocity', 'profile_velocity', 'target_velocity',
                   'velocity_offset', 'torque', 'following_error', 'status_word']
# Columns drawn on the same subplot
GROUPS = {'target_position': 'position', 'profile_velocity': 'velocity',
          'target_velocity': 'velocity', 'velocity_offset': 'velocity',
          'position_velocity': 'velocity'}
VELOCITY_COLUMNS = ('velocity', 'profile_velocity', 'target_velocity', 'velocity_offset',
                    'position_velocity')
ANGLE_COLUMNS = ('position', 'target_position', 'following_error')
RATE_WINDOW_S = 0.25  # span of the position-derived velocity


def position_rate(time, position, window=RATE_WINDOW_S):
    """Velocity from the position change over a sliding window; the drive's own 0x606C is jittery."""
    rate, j = [], 0
    for i, t in enumerate(time):
        while time[j] < t - window:
            j += 1
        rate.append((position[i] - position[j]) / (t - time[j]) if t > time[j] else float('nan'))
    return rate


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'{path}: no data rows')
    return {k: [float(r[k]) if r[k] not in ('', 'None') else float('nan') for r in rows]
            for k in rows[0]}


def deg_per_count(path):
    """Output-axis degrees per drive count for the motor named in the file (<test>_<motor>_<time>.csv)."""
    from eyou_ethercat import system
    motors = system.load_params()['motors']
    name = next((m for m in motors if f'_{m}_' in os.path.basename(path)), motors[0])
    profile = system.load_profile(name)
    mech = profile['mechanics']
    return 360 / (mech['pulses_per_rev'] * mech['gear_ratio']) * profile['calibration']['direction']


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('file', nargs='?', help='CSV file (default: newest CSV under log/)')
    parser.add_argument('-t', '--test', help='only consider recordings of this test, e.g. pp or csp')
    parser.add_argument('-c', '--columns', nargs='+', help='columns to plot')
    parser.add_argument('--raw', action='store_true',
                        help='plot drive units (counts, pulses/s) instead of deg and rpm')
    args = parser.parse_args()

    path = args.file
    if not path:
        files = glob.glob(f'log/**/{args.test or ""}*.csv', recursive=True)
        if not files:
            sys.exit('no matching CSV given and none found under log/')
        path = max(files, key=os.path.getmtime)
    data = load(path)
    data['position_velocity'] = position_rate(data['time_s'], data['position'])
    velocity_unit, angle_unit = 'pulses/s', 'counts'
    if not args.raw:
        scale = deg_per_count(path)
        for col in VELOCITY_COLUMNS:
            if col in data:
                data[col] = [v * scale / 6 for v in data[col]]  # deg/s -> rpm
        for col in ANGLE_COLUMNS:
            if col in data:
                data[col] = [v * scale for v in data[col]]
        velocity_unit, angle_unit = 'rpm', 'deg'

    columns = args.columns or [c for c in DEFAULT_COLUMNS if c in data]
    if 'velocity' in columns and 'position_velocity' not in columns:
        columns.insert(columns.index('velocity') + 1, 'position_velocity')
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
            style = {'linewidth': 2.2, 'color': 'k'} if col == 'position_velocity' else {'linewidth': 1}
            ax.plot(data['time_s'], data[col], label=col, **style)
        unit = velocity_unit if name in VELOCITY_COLUMNS else angle_unit if name in ANGLE_COLUMNS else ''
        ax.set_ylabel(f'{name} ({unit})' if unit else name)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelbottom=True)
        if len(cols) > 1:
            ax.legend(loc='upper left')
    axes[-1, 0].set_xlabel('time (s)')
    fig.suptitle(os.path.basename(path))
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
