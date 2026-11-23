#!/usr/bin/env python3
"""Make the report figures from docs/results.csv and the per-run CSV logs.

    python3 src/line_follower/scripts/make_figures.py

Writes PNG files to docs/figures/:
    error_vs_time.png       line error over one lap, one panel per base_speed (kp=1, kd=0 sweep)
    trajectory.png          robot path (from /odom) on top of the track, one panel per base_speed
    speed_vs_time.png       commanded linear speed over one lap, one panel per base_speed
    lap_time_vs_speed.png   bar chart: lap time per base_speed
    tuning.png              kp/kd tuning runs at the fastest speed: mean and max |error|
    tuning_error_vs_time.png  error over time for selected tuning runs (shows oscillation / failure)
"""
import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import track_geometry as tg  # noqa: E402

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
WS_DIR = os.path.abspath(os.path.join(PKG_DIR, '..', '..'))
RESULTS = os.path.join(PKG_DIR, 'docs', 'results.csv')
FIG_DIR = os.path.join(PKG_DIR, 'docs', 'figures')

# Reference palette (light mode), fixed order
SURFACE, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']
TRACK = '#9b9a95'

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
    'text.color': INK, 'axes.titlecolor': INK, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.8, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.axisbelow': True, 'font.size': 10, 'axes.titlesize': 11, 'figure.titlesize': 13, 'lines.linewidth': 1.6,
})


def load_results():
    rows = list(csv.DictReader(open(RESULTS)))
    for r in rows:
        for k in ('base_speed', 'kp', 'ki', 'kd'):
            r[k] = float(r[k])
        r['completed'] = r['completed'] == '1'
    return rows


def load_run(r):
    """Rows of a run CSV, cut at the end of lap 1 when the lap was completed."""
    if not r.get('csv_file'):
        return []
    rows = list(csv.DictReader(open(os.path.join(WS_DIR, r['csv_file']))))
    keys = ['time', 'error', 'linear_x', 'angular_z', 'x', 'y']
    if rows and all(x.get('gt_x') not in (None, '') for x in rows):
        keys += ['gt_x', 'gt_y']
    data = {k: [float(x[k]) for x in rows] for k in keys}
    if r['completed']:
        end = float(r['lap_time_s'])
        keep = [i for i, t in enumerate(data['time']) if t <= end]
        data = {k: [v[i] for i in keep] for k, v in data.items()}
    return data


def panel_title(r, tuning=False):
    status = f"lap {float(r['lap_time_s']):.1f} s" if r['completed'] else \
        f"not finished, line lost {r['line_lost_count']}x"
    return f"v = {r['base_speed']:.2f} m/s, kp = {r['kp']:g}, kd = {r['kd']:g}  ({status})"


def small_multiples(sweep, ykey, ylabel, title, fname, ylim=None, zero_line=False, tuning=False):
    n = len(sweep)
    cols = 2 if n > 1 else 1
    rows_n = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows_n, cols, figsize=(11, 3.2 * rows_n), sharey=True, squeeze=False)
    for ax, r in zip(axes.flat, sweep):
        d = load_run(r)
        if zero_line:
            ax.axhline(0, color=INK2, linewidth=0.8)
        if d:
            ax.plot(d['time'], d[ykey], color=SERIES[0])
        ax.set_title(panel_title(r, tuning), loc='left')
        ax.set_xlabel('simulation time (s)')
        ax.set_ylabel(ylabel)
        if ylim:
            ax.set_ylim(*ylim)
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    fig.suptitle(title, x=0.01, ha='left')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, fname), dpi=150)
    plt.close(fig)


def trajectory(sweep, title, fname):
    tx, ty = tg.track_outline()
    n = len(sweep)
    cols = 2 if n > 1 else 1
    rows_n = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows_n, cols, figsize=(11, 4.0 * rows_n), squeeze=False)
    for ax, r in zip(axes.flat, sweep):
        ax.plot(tx, ty, color=TRACK, linewidth=5, solid_capstyle='round', label='black line (track)')
        d = load_run(r)
        if d:
            if 'gt_x' in d:
                pts, label = list(zip(d['gt_x'], d['gt_y'])), 'robot centre (Gazebo ground truth)'
            else:
                pts = [tg.odom_to_world(x, y) for x, y in zip(d['x'], d['y'])]
                label = 'wheel axle (/odom)'
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=SERIES[0], linewidth=1.4, label=label)
        ax.plot(tg.START_X, tg.START_Y, 'o', color=INK, markersize=6, label='start')
        ax.annotate('start', (tg.START_X, tg.START_Y), xytext=(6, 6), textcoords='offset points',
                    color=INK2)
        ax.set_aspect('equal')
        ax.set_title(f"{panel_title(r)}\nmax deviation {float(r['max_deviation_m']) * 100:.1f} cm",
                     loc='left')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', ncol=3, frameon=False)
    fig.suptitle(title, x=0.01, ha='left')
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(FIG_DIR, fname), dpi=150)
    plt.close(fig)


def bar_with_labels(ax, labels, values, texts, colors=None):
    xs = range(len(labels))
    heights = [v if v is not None else 0 for v in values]
    ax.bar(xs, heights, width=0.5, color=colors or SERIES[0])
    top = max(heights) if any(heights) else 1
    for x, v, t in zip(xs, values, texts):
        ax.text(x, (v if v is not None else 0) + top * 0.02, t, ha='center', va='bottom', color=INK)
    ax.set_xticks(list(xs))
    ax.set_xticklabels(labels)
    ax.set_ylim(0, top * 1.18)
    ax.grid(axis='x', visible=False)


def gain_color(r):
    """Colour follows the controller gains in every chart: kp=1 blue, kp=3 orange, others aqua."""
    return {(1.0, 0.0): SERIES[0], (3.0, 0.0): SERIES[1]}.get((r['kp'], r['kd']), SERIES[2])


def lap_time_bar(runs):
    """One bar per run; colour = controller gains (blue kp=1, orange tuned)."""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    top = max(float(r['lap_time_s']) for r in runs if r['completed'])
    seen = set()
    for x, r in enumerate(runs):
        h = float(r['lap_time_s']) if r['completed'] else 0
        label = f"kp = {r['kp']:g}, kd = {r['kd']:g}"
        ax.bar(x, h, width=0.55, color=gain_color(r), label=None if label in seen else label)
        seen.add(label)
        ax.text(x, h + top * 0.015, f'{h:.1f} s' if r['completed'] else 'not finished',
                ha='center', va='bottom', color=INK)
    ax.set_xticks(range(len(runs)))
    ax.set_xticklabels([f"{r['base_speed']:.2f}\nkp={r['kp']:g}" for r in runs])
    ax.set_ylim(0, top * 1.15)
    ax.grid(axis='x', visible=False)
    ax.set_xlabel('base_speed (m/s) and controller gain')
    ax.set_ylabel('lap time (s, simulation time)')
    ax.set_title('Lap time per run', loc='left')
    ax.legend(frameon=False, loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, 'lap_time_vs_speed.png'), dpi=150)
    plt.close(fig)


def tuning_bar(tuning):
    if len(tuning) < 2:
        return
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    labels = [f"kp={r['kp']:g}\nkd={r['kd']:g}" for r in tuning]
    for ax, key, name in ((axes[0], 'mean_abs_error', 'mean |error|'),
                          (axes[1], 'max_abs_error', 'max |error|')):
        vals = [float(r[key]) if r['completed'] else None for r in tuning]
        texts = [f'{v:.3f}' if v is not None else 'not finished' for v in vals]
        bar_with_labels(ax, labels, vals, texts, colors=[gain_color(r) for r in tuning])
        ax.set_ylabel(f'{name} (unitless, -1..1)')
        ax.set_title(name, loc='left')
    fig.suptitle(f"Gain comparison at base_speed = {tuning[0]['base_speed']:.2f} m/s "
                 '(error 1.0 = line under the outermost sensor)', x=0.01, ha='left')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, 'gain_comparison.png'), dpi=150)
    plt.close(fig)


def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    # Remove only charts this script makes (old and current names); screenshots and diagrams stay
    generated = {'error_vs_time.png', 'speed_vs_time.png', 'trajectory.png', 'lap_time_vs_speed.png',
                 'gain_comparison.png', 'tuning.png', 'tuning_error_vs_time.png', 'error_vs_time_kp3.png',
                 'speed_vs_time_kp3.png', 'trajectory_kp3.png'}
    for f in os.listdir(FIG_DIR):
        if f in generated:
            os.remove(os.path.join(FIG_DIR, f))
    runs = sorted(load_results(), key=lambda r: (r['base_speed'], r['kp'], r['kd']))
    vmax = max(r['base_speed'] for r in runs)

    small_multiples(runs, 'error', 'error (-1..1)', 'Line position error vs time',
                    'error_vs_time.png', ylim=(-1.1, 1.1), zero_line=True)
    small_multiples(runs, 'linear_x', 'linear.x (m/s)', 'Forward speed (/cmd_vel command) vs time',
                    'speed_vs_time.png', ylim=(0, vmax * 1.1))
    trajectory(runs, 'Robot path (Gazebo ground truth) on the track', 'trajectory.png')
    lap_time_bar(runs)
    done = [r for r in runs if r['completed'] and r['kp'] == 1.0 and r['kd'] == 0]
    fastest = max(r['base_speed'] for r in done) if done else None
    same_speed = [r for r in runs if r['base_speed'] == fastest]
    tuning_bar(same_speed)
    print('Figures written to', FIG_DIR)
    for f in sorted(os.listdir(FIG_DIR)):
        print('  ', f)


if __name__ == '__main__':
    main()
