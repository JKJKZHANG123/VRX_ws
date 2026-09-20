#!/usr/bin/env python3
"""Replay warning-corridor gate rules against a recorded trial.

The current gate releases thrust only when Nav2 commands a turn.  A Dubins
detour is arc-straight-arc, so on the straight segment -- when the boat is
already aligned on a path that clears the obstacle -- the gate sees "not
turning" and zeroes thrust.  In report3 static_plan_20260901_plan10 that was
251 of 271 fully-zeroed samples, and the controller then aborted with
"Failed to make progress".

This replays a candidate rule offline: project the commanded arc forward and
release thrust when the swept hull corridor clears the recorded obstacle
points.  Prints how each rule would have behaved so the change can be judged
before it is applied to the vehicle.
"""

import argparse
import bisect
import csv
import math
from collections import defaultdict


def load(prefix):
    with open(prefix + '_trajectory.csv') as f:
        rows = list(csv.DictReader(f))
    clouds = defaultdict(list)
    with open(prefix + '_obstacles.csv') as f:
        for r in csv.DictReader(f):
            if r['source'] != 'raw':
                continue
            clouds[round(float(r['t']), 3)].append(
                (float(r['x']), float(r['y'])))
    return rows, clouds


def num(row, key):
    v = row[key]
    return float(v) if v not in ('', 'None') else 0.0


def swept_clearance(points, vx, wz, half_width, horizon_s):
    """Min distance from the projected arc to any point, in base_link.

    Returns (min_distance, closest_point). The arc is the path the commanded
    (vx, wz) would actually trace, so a straight command that passes beside an
    obstacle is correctly scored as clear.
    """
    if vx <= 1e-3:
        return float('inf'), None
    steps = 24
    dt = horizon_s / steps
    x = y = th = 0.0
    path = [(0.0, 0.0)]
    for _ in range(steps):
        x += vx * math.cos(th) * dt
        y += vx * math.sin(th) * dt
        th += wz * dt
        path.append((x, y))
    best = float('inf')
    closest = None
    for px, py in points:
        # Distance to the polyline, not just its samples.
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            dx, dy = bx - ax, by - ay
            seg = dx * dx + dy * dy
            t = 0.0 if seg <= 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg))
            cx, cy = ax + t * dx, ay + t * dy
            d = math.hypot(px - cx, py - cy)
            if d < best:
                best, closest = d, (px, py)
    return best - half_width, closest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prefix', required=True)
    ap.add_argument('--half-width', type=float, default=1.1,
                    help='hull half width (footprint)')
    ap.add_argument('--margin', type=float, default=0.6,
                    help='extra clearance required to release thrust')
    ap.add_argument('--horizon-s', type=float, default=6.0)
    ap.add_argument('--min-yaw', type=float, default=0.08)
    ap.add_argument('--min-curv', type=float, default=0.05)
    args = ap.parse_args()

    rows, clouds = load(args.prefix)
    times = sorted(clouds)
    if not times:
        print('no raw obstacle points recorded')
        return 2

    stats = dict(total=0, old_pass=0, curv_pass=0, new_pass=0,
                 new_stop_real=0, no_cmd=0)
    for r in rows:
        if r['safety_stop'] != '1':
            continue
        stats['total'] += 1
        vx, wz = num(r, 'cmd_vx'), num(r, 'cmd_wz')
        if vx <= 0.0:
            stats['no_cmd'] += 1
            continue
        if abs(wz) >= args.min_yaw:
            stats['old_pass'] += 1
        if abs(wz) / vx >= args.min_curv:
            stats['curv_pass'] += 1

        t = num(r, 't')
        i = bisect.bisect_left(times, t)
        cand = [times[j] for j in (i - 1, i) if 0 <= j < len(times)]
        if not cand:
            continue
        ct = min(cand, key=lambda c: abs(c - t))
        if abs(ct - t) > 2.0:
            continue
        clear, _ = swept_clearance(clouds[ct], vx, wz,
                                   args.half_width, args.horizon_s)
        if clear >= args.margin:
            stats['new_pass'] += 1
        else:
            stats['new_stop_real'] += 1

    print(f'warning-corridor samples: {stats["total"]}')
    print(f'  Nav2 commanded nothing (vx<=0): {stats["no_cmd"]}')
    print(f'  released by yaw-rate rule   >= {args.min_yaw}: {stats["old_pass"]}')
    print(f'  released by curvature rule  >= {args.min_curv}: {stats["curv_pass"]}')
    print(f'  released by SWEPT-PATH rule (margin {args.margin} m): '
          f'{stats["new_pass"]}')
    print(f'  STOPPED by swept-path rule (真的会撞): {stats["new_stop_real"]}')
    print()
    print('The swept-path rule stops only when the commanded arc actually')
    print('approaches an obstacle, so an aligned straight run is not blocked.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
