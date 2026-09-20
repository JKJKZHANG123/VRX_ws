#!/usr/bin/env python3
"""Summarize EKF A/B CSV trials into one MATLAB-friendly table."""
import argparse
import csv
import math
from pathlib import Path


def vals(rows, key):
    out = []
    for row in rows:
        try:
            value = float(row[key])
            if math.isfinite(value):
                out.append(value)
        except (KeyError, TypeError, ValueError):
            pass
    return out


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else math.nan


def percentile(xs, p):
    if not xs:
        return math.nan
    ys = sorted(xs)
    k = (len(ys) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(ys) - 1)
    return ys[lo] + (ys[hi] - ys[lo]) * (k - lo)


def path_length(rows, x_key, y_key):
    points = []
    for row in rows:
        try:
            x, y = float(row[x_key]), float(row[y_key])
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
        except (KeyError, TypeError, ValueError):
            pass
    return sum(math.hypot(x1 - x0, y1 - y0)
               for (x0, y0), (x1, y1) in zip(points, points[1:]))


def summary(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    result = {'file': str(path), 'samples': len(rows)}
    if not rows:
        return result
    result['mode'] = rows[0].get('mode', '')
    for label in ('truth_xy_motion_m', 'lio_xy_drift_m', 'ekf_xy_drift_m',
                  'lio_truth_xy_error_m', 'ekf_truth_xy_error_m'):
        x = vals(rows, label)
        result[label + '_final'] = x[-1] if x else math.nan
        result[label + '_max'] = max(x) if x else math.nan
        result[label + '_p95'] = percentile(x, .95)
        result[label + '_rmse'] = rms(x)
    result['truth_path_length_m'] = path_length(rows, 'truth_x_rel_m', 'truth_y_rel_m')
    result['lio_path_length_m'] = path_length(rows, 'lio_x_rel_m', 'lio_y_rel_m')
    result['ekf_path_length_m'] = path_length(rows, 'ekf_camera_x_rel_m',
                                              'ekf_camera_y_rel_m')
    for label in ('lio_z_rel_m', 'ekf_z_rel_m', 'lio_yaw_rad', 'ekf_yaw_rad',
                  'lio_vx_mps', 'ekf_vx_mps', 'lio_wz_radps', 'ekf_wz_radps',
                  'cmd_vx_mps', 'cmd_wz_radps'):
        x = vals(rows, label)
        if x:
            result[label + '_mean'] = sum(x) / len(x)
            result[label + '_min'] = min(x)
            result[label + '_max'] = max(x)
    for label in ('filtered_points', 'registered_points', 'safety_cloud_points',
                  'raw_obstacle_points', 'safety_stop', 'nan_count',
                  'stamp_backwards'):
        x = vals(rows, label)
        if x:
            result[label + '_mean'] = sum(x) / len(x)
            result[label + '_max'] = max(x)
    safety = vals(rows, 'safety_stop')
    result['safety_stop_fraction'] = (sum(1 for x in safety if x > 0.5) / len(safety)
                                      if safety else math.nan)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', required=True)
    ap.add_argument('files', nargs='+')
    args = ap.parse_args()
    summaries = [summary(Path(x)) for x in args.files]
    fields = []
    for item in summaries:
        for key in item:
            if key not in fields:
                fields.append(key)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)
    print(f'WROTE {len(summaries)} summaries -> {output}')


if __name__ == '__main__':
    main()
