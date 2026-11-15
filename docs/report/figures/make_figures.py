#!/usr/bin/env python3
"""Charts for the report, from ../data/runs.json. Writes vector PDFs here.

Usage: python3 make_figures.py
"""
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = json.load(open(os.path.join(HERE, '..', 'data', 'runs.json')))

# Reference categorical palette, light mode, slots 1-3 (valid for all pairs)
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#d9d8d4'

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Liberation Serif', 'Times New Roman', 'DejaVu Serif'],
    'font.size': 8, 'axes.titlesize': 8, 'axes.labelsize': 8,
    'xtick.labelsize': 7, 'ytick.labelsize': 7, 'legend.fontsize': 7,
    'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'text.color': INK,
    'xtick.color': INK2, 'ytick.color': INK2,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.5,
    'axes.axisbelow': True,
    'pdf.fonttype': 42, 'ps.fonttype': 42,   # embed TrueType (IEEE PDF eXpress)
})
COL = 3.45   # IEEE column width in inches


def save(fig, name):
    fig.savefig(os.path.join(HERE, name), bbox_inches='tight', pad_inches=0.02)
    plt.close(fig)


def outcome(r):
    if r['gazebo_ok']:
        return 'success'
    return 'undetected' if r['mission_result'] == 'COMPLETE' else 'safe abort'


# ---------------------------------------------------------------- outcomes
def fig_outcomes():
    # The five pre-camera repeatability runs have no saved logs; their
    # outcomes are taken from the results recorded at the time.
    groups = [
        ('Pre-camera trials (P1–P5)', {'success': 4, 'safe abort': 1, 'undetected': 0}),
    ]
    order = [('Tracking runs (T1–T5)', lambda r: r['group'] == 'tracking'),
             ('Fresh clone, pre-camera (F1)', lambda r: r['group'] == 'fresh clone'),
             ('Camera, during development (C1–C8)',
              lambda r: r['group'] == 'camera' and int(r['id'][1:]) <= 8),
             ('Camera, final software (C9–C11)',
              lambda r: r['group'] == 'camera' and int(r['id'][1:]) >= 9)]
    for label, sel in order:
        c = {'success': 0, 'safe abort': 0, 'undetected': 0}
        for r in RUNS:
            if sel(r):
                c[outcome(r)] += 1
        groups.append((label, c))

    fig, ax = plt.subplots(figsize=(COL, 1.75))
    y = np.arange(len(groups))[::-1]
    keys = [('success', BLUE, 'Success (cube on drop pedestal)'),
            ('safe abort', AQUA, 'Stopped safely by a check'),
            ('undetected', ORANGE, 'Failure not detected')]
    left = np.zeros(len(groups))
    for k, color, lab in keys:
        vals = np.array([g[1][k] for g in groups], float)
        ax.barh(y, vals, left=left, color=color, height=0.62, label=lab,
                edgecolor='white', linewidth=1.0)
        for yi, l0, v in zip(y, left, vals):
            if v > 0:
                ax.text(l0 + v / 2, yi, f'{int(v)}', ha='center', va='center',
                        color='white', fontsize=7, fontweight='bold')
        left += vals
    ax.set_yticks(y)
    ax.set_yticklabels([g[0] for g in groups])
    ax.set_xlabel('Number of runs')
    ax.set_xlim(0, 8.5)
    ax.xaxis.set_major_locator(matplotlib.ticker.MultipleLocator(1))
    ax.grid(axis='y', visible=False)
    ax.legend(loc='upper center', bbox_to_anchor=(0.35, -0.32), ncol=2,
              frameon=False, handlelength=1.0, columnspacing=1.0)
    save(fig, 'outcomes.pdf')


# ---------------------------------------------------------------- parking
def fig_parking():
    arr = {'PICKUP': [], 'DROP': []}
    dock = {'PICKUP': [], 'DROP': []}
    yaw = []
    for r in RUNS:
        for a in r['arrival']:
            arr[a['pedestal']].append(a['lateral'] * 100)
        for d in r['docked']:
            dock[d['pedestal']].append(d['lateral'] * 100)
            yaw.append(d['yaw_deg'])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(COL, 1.9),
                                   gridspec_kw={'width_ratios': [2.1, 1]})
    rng = np.random.default_rng(1)
    pos = [0, 1, 2.6, 3.6]
    data = [arr['PICKUP'], dock['PICKUP'], arr['DROP'], dock['DROP']]
    colors = [ORANGE, BLUE, ORANGE, BLUE]
    for p, d, c in zip(pos, data, colors):
        x = p + rng.uniform(-0.18, 0.18, len(d))
        ax1.scatter(x, d, s=12, color=c, edgecolor='white', linewidth=0.5, zorder=3)
        ax1.hlines(np.mean(np.abs(d)) * np.sign(1), p - 0.3, p + 0.3, color=INK2,
                   linewidth=0, zorder=2)
    ax1.axhline(0, color=INK2, linewidth=0.6)
    ax1.set_xticks(pos)
    ax1.set_xticklabels(['after\nNav2', 'after\ndocking'] * 2)
    ax1.text(0.5, 1.02, 'Pickup', transform=ax1.get_xaxis_transform(), ha='center')
    ax1.text(3.1, 1.02, 'Drop', transform=ax1.get_xaxis_transform(), ha='center')
    ax1.set_ylabel('Sideways offset (cm)')
    ax1.set_xlim(-0.6, 4.2)
    ax1.grid(axis='x', visible=False)

    x = rng.uniform(-0.18, 0.18, len(yaw))
    ax2.scatter(x, yaw, s=12, color=BLUE, edgecolor='white', linewidth=0.5, zorder=3)
    ax2.axhline(0, color=INK2, linewidth=0.6)
    ax2.set_xticks([0])
    ax2.set_xticklabels(['after\ndocking'])
    ax2.set_xlim(-0.6, 0.6)
    ax2.set_ylabel('Heading error (°)')
    ax2.grid(axis='x', visible=False)
    fig.tight_layout(w_pad=1.2)
    save(fig, 'parking.pdf')


# ---------------------------------------------------------------- camera
def fig_camera():
    rows = [r for r in RUNS if r.get('camera_err_mm') is not None
            and not r['note'].startswith('aborted')]
    ids = [r['id'] for r in rows]
    err = [r['camera_err_mm'] for r in rows]
    fig, ax = plt.subplots(figsize=(COL, 1.45))
    x = np.arange(len(ids))
    ax.bar(x, err, width=0.55, color=BLUE)
    for xi, e in zip(x, err):
        ax.text(xi, e + 0.08, f'{e:.1f}', ha='center', va='bottom', fontsize=6.5,
                color=INK2)
    ax.axhline(np.mean(err), color=INK2, linewidth=0.8, linestyle=(0, (3, 2)))
    ax.text(-0.45, 4.45, f'dashed line: mean {np.mean(err):.1f} mm',
            ha='left', va='top', fontsize=6.5, color=INK2)
    ax.set_xticks(x)
    ax.set_xticklabels(ids)
    ax.set_ylabel('Position error (mm)')
    ax.set_ylim(0, 4.5)
    ax.grid(axis='x', visible=False)
    save(fig, 'camera_error.pdf')


# ---------------------------------------------------------------- phases
def fig_phases():
    names = [('drive_to_pickup', 'Drive to pickup'), ('dock_pickup', 'Dock (pickup)'),
             ('pick', 'Pick'), ('drive_to_drop', 'Drive to drop'),
             ('dock_drop', 'Dock (drop)'), ('place', 'Place')]
    ok = [r for r in RUNS if r['gazebo_ok']]
    fig, ax = plt.subplots(figsize=(COL, 1.8))
    rng = np.random.default_rng(2)
    y = np.arange(len(names))[::-1]
    for yi, (k, _) in zip(y, names):
        v = [r['phases_s'][k] for r in ok if r['phases_s'][k] is not None]
        ax.barh(yi, np.median(v), height=0.55, color=BLUE, alpha=0.35)
        ax.scatter(v, yi + rng.uniform(-0.15, 0.15, len(v)), s=9, color=BLUE,
                   edgecolor='white', linewidth=0.4, zorder=3)
        ax.text(208, yi, f'median {np.median(v):.0f} s', va='center',
                fontsize=6.5, color=INK2)
    ax.set_yticks(y)
    ax.set_yticklabels([n for _, n in names])
    ax.set_xlabel('Wall-clock time (s), simulation at about 0.3–0.5× real time')
    ax.set_xlim(0, 250)
    ax.set_xticks([0, 50, 100, 150, 200])
    ax.grid(axis='y', visible=False)
    save(fig, 'phases.pdf')


if __name__ == '__main__':
    fig_outcomes()
    fig_parking()
    fig_camera()
    fig_phases()
    print('figures written to', HERE)
