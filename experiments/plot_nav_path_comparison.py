#!/usr/bin/env python3
"""Plot Nav2 planned paths against the measured USV trajectory.

Usage:
  python3 experiments/plot_nav_path_comparison.py \
      --prefix report3/data/static_obstacle_20260830 \
      --scenario-events report3/data/static_obstacle_20260830_scenario.csv

The recorder writes one row per point for every /plan snapshot. The figure
shows all replanning snapshots faintly, the first/latest plan prominently, the
actual trajectory, and the obstacle position/events when available.
"""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def f(row, key, default=math.nan):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def path_length(points):
    return sum(math.hypot(x2 - x1, y2 - y1)
               for (x1, y1), (x2, y2) in zip(points, points[1:]))


def nearest_distance(x, y, points):
    if not points:
        return math.nan
    return min(math.hypot(x - px, y - py) for px, py in points)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prefix', required=True,
                    help='prefix used by record_nav_path.py')
    ap.add_argument('--scenario-events', default='',
                    help='optional dynamic_obstacle_scenario events CSV')
    ap.add_argument('--output', default='', help='PNG path; defaults to <prefix>_path_vs_trajectory.png')
    args = ap.parse_args()

    prefix = Path(args.prefix)
    trajectory_file = prefix.with_name(prefix.name + '_trajectory.csv')
    plans_file = prefix.with_name(prefix.name + '_plans.csv')
    obstacles_file = prefix.with_name(prefix.name + '_obstacles.csv')
    if not trajectory_file.exists() or not plans_file.exists():
        raise SystemExit(f'missing recorder files for prefix {prefix}')

    trajectory = read_csv(trajectory_file)
    plan_rows = read_csv(plans_file)
    plans = defaultdict(list)
    plan_times = {}
    for row in plan_rows:
        pid = int(row['plan_id'])
        plans[pid].append((f(row, 'x'), f(row, 'y')))
        plan_times[pid] = f(row, 't')
    plan_ids = sorted(plans)
    plan_ids = [pid for pid in plan_ids if len(plans[pid]) >= 2]

    xs = [f(r, 'pose_x') for r in trajectory]
    ys = [f(r, 'pose_y') for r in trajectory]
    ts = [f(r, 't') for r in trajectory]
    valid = [(t, x, y) for t, x, y in zip(ts, xs, ys)
             if math.isfinite(t) and math.isfinite(x) and math.isfinite(y)]
    if not valid:
        raise SystemExit('trajectory file has no finite poses')
    ts, xs, ys = zip(*valid)
    ts, xs, ys = list(ts), list(xs), list(ys)

    obstacle_xy = []
    if obstacles_file.exists():
        for row in read_csv(obstacles_file):
            x, y = f(row, 'x'), f(row, 'y')
            if math.isfinite(x) and math.isfinite(y):
                obstacle_xy.append((x, y))

    scenario_xy = []
    scenario_file = Path(args.scenario_events) if args.scenario_events else None
    if scenario_file and scenario_file.exists():
        for row in read_csv(scenario_file):
            # The transform_frozen event stores the boat-frame origin (0, 0),
            # not an obstacle. Only retain spawn/pose events as obstacle
            # locations, otherwise the clearance metric is contaminated by
            # the vessel's own starting point.
            if row.get('event') not in {'spawned', 'motion_start',
                                        'pose_sample', 'motion_complete'}:
                continue
            x, y = f(row, 'camera_x'), f(row, 'camera_y')
            if math.isfinite(x) and math.isfinite(y):
                scenario_xy.append((x, y))

    def first_last():
        return (plans[plan_ids[0]], plans[plan_ids[-1]]) if plan_ids else ([], [])

    first_plan, last_plan = first_last()
    aligned_errors = []
    for row, x, y in zip(trajectory, xs, ys):
        pid = int(row.get('plan_id') or 0)
        points = plans.get(pid, last_plan)
        d = nearest_distance(x, y, points)
        if math.isfinite(d):
            aligned_errors.append((f(row, 't'), d))
    all_planned_points = [p for pid in plan_ids for p in plans[pid]]
    obstacle_distance = []
    for x, y in zip(xs, ys):
        d = nearest_distance(x, y, scenario_xy)
        if math.isfinite(d):
            obstacle_distance.append(d)

    actual_distance = sum(math.hypot(x2 - x1, y2 - y1)
                          for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]))
    metrics = {
        'trajectory_samples': len(xs),
        'trajectory_duration_s': ts[-1] - ts[0],
        'plan_snapshot_count': len(plan_ids),
        'first_plan_points': len(first_plan),
        'last_plan_points': len(last_plan),
        'first_plan_length_m': path_length(first_plan),
        'last_plan_length_m': path_length(last_plan),
        'actual_traveled_distance_m': actual_distance,
        'mean_actual_to_time_aligned_plan_m': (
            sum(d for _, d in aligned_errors) / len(aligned_errors)
            if aligned_errors else math.nan),
        'p95_actual_to_time_aligned_plan_m': (
            sorted(d for _, d in aligned_errors)[int(0.95 * (len(aligned_errors) - 1))]
            if aligned_errors else math.nan),
        'max_actual_to_time_aligned_plan_m': (
            max(d for _, d in aligned_errors) if aligned_errors else math.nan),
        'min_actual_distance_to_scenario_obstacle_m': (
            min(obstacle_distance) if obstacle_distance else math.nan),
        'observed_obstacle_points': len(obstacle_xy),
        'scenario_obstacle_samples': len(scenario_xy),
    }

    out_png = Path(args.output) if args.output else prefix.with_name(prefix.name + '_path_vs_trajectory.png')
    out_metrics = out_png.with_suffix('.json')
    out_csv = out_png.with_suffix('.csv')
    with out_metrics.open('w') as fp:
        json.dump(metrics, fp, indent=2, allow_nan=True)
    with out_csv.open('w', newline='') as fp:
        w = csv.writer(fp)
        w.writerow(['metric', 'value'])
        w.writerows(metrics.items())

    fig, (ax, err_ax) = plt.subplots(1, 2, figsize=(15, 6), constrained_layout=True)
    # Show replanning history. Limit only visual clutter; all snapshots remain in CSV.
    visible_ids = plan_ids if len(plan_ids) <= 80 else plan_ids[::max(1, len(plan_ids) // 80)]
    for pid in visible_ids:
        p = plans[pid]
        ax.plot([q[0] for q in p], [q[1] for q in p], color='tab:blue', alpha=0.10, linewidth=0.8)
    if first_plan:
        ax.plot([q[0] for q in first_plan], [q[1] for q in first_plan],
                '--', color='tab:cyan', linewidth=1.5, label='first planned path')
    if last_plan and last_plan != first_plan:
        ax.plot([q[0] for q in last_plan], [q[1] for q in last_plan],
                '-', color='tab:purple', linewidth=2.0, label='latest planned path')

    ax.plot(xs, ys, color='tab:orange', linewidth=2.2, label='measured trajectory')
    ax.scatter(xs[0], ys[0], color='green', s=55, zorder=5, label='start')
    ax.scatter(xs[-1], ys[-1], color='black', marker='x', s=70, zorder=5, label='end')
    if scenario_xy:
        # Scenario events are authoritative obstacle centers in camera_init.
        sx, sy = zip(*scenario_xy)
        ax.scatter(sx, sy, color='red', marker='s', s=45, alpha=0.8,
                   label='scenario obstacle center')
    if obstacle_xy:
        # Downsample the observed cloud only for display.
        stride = max(1, len(obstacle_xy) // 3000)
        ox, oy = zip(*obstacle_xy[::stride])
        ax.scatter(ox, oy, color='gray', s=2, alpha=0.20,
                   label='observed obstacle cloud')
    ax.set_title('Obstacle-aware Nav2 path and USV trajectory')
    ax.set_xlabel('camera_init x [m]')
    ax.set_ylabel('camera_init y [m]')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='best', fontsize=8)

    if aligned_errors:
        et, ed = zip(*aligned_errors)
        err_ax.plot(et, ed, color='tab:orange', linewidth=1.5)
    err_ax.set_title('Trajectory-to-current-plan error')
    err_ax.set_xlabel('simulation time [s]')
    err_ax.set_ylabel('nearest path distance [m]')
    err_ax.grid(True, alpha=0.3)
    text = (f"plans: {metrics['plan_snapshot_count']}\n"
            f"first/latest length: {metrics['first_plan_length_m']:.2f} / {metrics['last_plan_length_m']:.2f} m\n"
            f"actual distance: {actual_distance:.2f} m\n"
            f"mean / P95 error: {metrics['mean_actual_to_time_aligned_plan_m']:.2f} / "
            f"{metrics['p95_actual_to_time_aligned_plan_m']:.2f} m")
    err_ax.text(0.98, 0.98, text, transform=err_ax.transAxes, va='top', ha='right',
                fontsize=8, bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    fig.suptitle(f'Path planning validation: {prefix.name}', fontsize=13)
    fig.savefig(out_png, dpi=160)
    print(f'figure: {out_png}')
    print(f'metrics: {out_metrics}')
    for k, v in metrics.items():
        print(f'{k}: {v}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
