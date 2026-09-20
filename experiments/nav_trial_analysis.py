#!/usr/bin/env python3
"""Offline analysis helpers for repeatable USV navigation experiments.

The workspace contains two recorder generations:

* ``record_nav_path.py`` writes ``<prefix>_trajectory.csv`` and plan snapshots;
* ``usv_navigation.nav_data_logger`` writes one legacy ``<prefix>.csv``.

This module normalizes both formats so static path-vs-trajectory and dynamic
tracker A/B reports use the same metrics without copying or editing raw data.
All geometry is expected to be in ``camera_init`` coordinates.
"""

from __future__ import annotations

import csv
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

XY = Tuple[float, float]
TIMED_XY = Tuple[float, XY]


def read_csv(path: Path) -> List[dict]:
    if not path.exists():
        raise FileNotFoundError(f"CSV file does not exist: {path}")
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def number(row: dict, key: str, default: float = math.nan) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def first_number(row: dict, keys: Sequence[str], default: float = math.nan) -> float:
    for key in keys:
        value = number(row, key)
        if math.isfinite(value):
            return value
    return default


def normalize_trajectory_row(row: dict) -> dict:
    """Return a copy with common names populated from both logger formats."""
    normalized = dict(row)
    aliases = {
        "left_thrust_n": ("left_thrust_n", "left_thrust"),
        "right_thrust_n": ("right_thrust_n", "right_thrust"),
        "dynamic_tracks": ("dynamic_tracks", "dynamic_track_count"),
        "dynamic_predicted_points": (
            "dynamic_predicted_points", "dynamic_point_count"),
        "dynamic_max_speed_mps": (
            "dynamic_max_speed_mps", "dynamic_max_speed"),
        "raw_obstacle_points": ("raw_obstacle_points", "raw_obstacle_pts"),
        "safety_stop": ("safety_stop",),
    }
    for target, candidates in aliases.items():
        if math.isfinite(first_number(row, candidates)):
            normalized[target] = str(first_number(row, candidates))
    if not normalized.get("goal_status"):
        code = number(row, "goal_result_code", 0.0)
        labels = {1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                  4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        if code in labels:
            normalized["goal_status"] = f"{labels[code]} code={int(code)}"
    return normalized


def path_length(points: Sequence[XY]) -> float:
    return sum(math.hypot(x2 - x1, y2 - y1)
               for (x1, y1), (x2, y2) in zip(points, points[1:]))


def point_to_segment_distance(point: XY, start: XY, end: XY) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + u * dx), py - (ay + u * dy))


def point_to_polyline_distance(point: XY, polyline: Sequence[XY]) -> float:
    if not polyline:
        return math.nan
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])
    return min(point_to_segment_distance(point, a, b)
               for a, b in zip(polyline, polyline[1:]))


def nearest_distance(point: XY, points: Sequence[XY]) -> float:
    if not points:
        return math.nan
    return min(math.hypot(point[0] - x, point[1] - y) for x, y in points)


def percentile(values: Sequence[float], fraction: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    index = int(max(0.0, min(1.0, fraction)) * (len(finite) - 1))
    return finite[index]


def prefix_file(prefix: Path, suffix: str) -> Path:
    return prefix.with_name(prefix.name + suffix)


def trajectory_file(prefix: Path) -> Optional[Path]:
    """Resolve a path-recorder prefix or a legacy logger CSV."""
    candidates = []
    if prefix.suffix == ".csv":
        candidates.append(prefix)
    candidates.extend((prefix_file(prefix, "_trajectory.csv"),
                       prefix.with_suffix(".csv")))
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def load_trajectory(prefix: Path) -> List[dict]:
    path = trajectory_file(prefix)
    if path is None:
        raise FileNotFoundError(
            f"no trajectory CSV for {prefix}; expected {prefix}_trajectory.csv "
            f"or {prefix.with_suffix('.csv')}")
    rows = [normalize_trajectory_row(row) for row in read_csv(path)]
    return [row for row in rows
            if math.isfinite(number(row, "t"))
            and math.isfinite(number(row, "pose_x"))
            and math.isfinite(number(row, "pose_y"))]


def load_plans(prefix: Path, source: str = "/plan") -> List[dict]:
    path = prefix_file(prefix, "_plans.csv")
    if not path.exists():
        return []
    rows = read_csv(path)
    if not rows:
        return []
    sources = {row.get("source", "") for row in rows}
    if source and source not in sources:
        for candidate in ("/unsmoothed_plan", "/received_global_plan", ""):
            if candidate in sources:
                source = candidate
                break
    selected = [row for row in rows if not source or row.get("source") == source]
    return [row for row in selected
            if math.isfinite(number(row, "t"))
            and math.isfinite(number(row, "x"))
            and math.isfinite(number(row, "y"))]


def grouped_plans(rows: Sequence[dict]) -> List[Tuple[float, List[XY]]]:
    grouped: Dict[str, List[Tuple[float, XY]]] = {}
    for row in rows:
        plan_id = row.get("plan_id", "")
        grouped.setdefault(plan_id, []).append(
            (number(row, "t"), (number(row, "x"), number(row, "y"))))
    snapshots = []
    for points in grouped.values():
        points.sort(key=lambda item: item[0])
        finite = [point for _time, point in points
                  if all(math.isfinite(value) for value in point)]
        if len(finite) >= 2:
            snapshots.append((points[0][0], finite))
    snapshots.sort(key=lambda item: item[0])
    return snapshots


def plan_at(snapshots: Sequence[Tuple[float, List[XY]]], t: float) -> List[XY]:
    if not snapshots:
        return []
    selected = snapshots[0][1]
    for stamp, points in snapshots:
        if stamp > t:
            break
        selected = points
    return selected


def load_scenario_samples(path: Optional[Path]) -> List[TIMED_XY]:
    """Load obstacle event samples as ``(sim_time, (x, y))`` pairs."""
    if path is None or not path.exists():
        return []
    samples = []
    events = {"spawned", "motion_start", "pose_sample", "motion_complete",
              "static_ready"}
    for row in read_csv(path):
        if row.get("event") not in events:
            continue
        x, y = number(row, "camera_x"), number(row, "camera_y")
        t = number(row, "sim_time")
        if math.isfinite(x) and math.isfinite(y):
            samples.append((t if math.isfinite(t) else math.nan, (x, y)))
    return samples


def load_scenario_points(path: Optional[Path]) -> List[XY]:
    return [point for _time, point in load_scenario_samples(path)]


def load_obstacle_points(prefix: Path) -> List[XY]:
    path = prefix_file(prefix, "_obstacles.csv")
    if not path.exists():
        return []
    return [(number(row, "x"), number(row, "y")) for row in read_csv(path)
            if math.isfinite(number(row, "x"))
            and math.isfinite(number(row, "y"))]


def infer_status(trajectory: Sequence[dict]) -> str:
    for row in reversed(trajectory):
        status = row.get("goal_status", "").strip()
        if status and not status.upper().startswith("IDLE"):
            return status
    for row in reversed(trajectory):
        status = row.get("goal_status", "").strip()
        if status:
            return status
    return "UNKNOWN"


def infer_goal(trajectory: Sequence[dict], goal: Optional[XY]) -> Optional[XY]:
    if goal is not None:
        return goal
    for row in reversed(trajectory):
        x, y = number(row, "goal_x"), number(row, "goal_y")
        if math.isfinite(x) and math.isfinite(y) and (abs(x) > 1.0e-9 or abs(y) > 1.0e-9):
            return x, y
    # A valid origin goal is uncommon but should still be recoverable.
    for row in reversed(trajectory):
        x, y = number(row, "goal_x"), number(row, "goal_y")
        if math.isfinite(x) and math.isfinite(y):
            return x, y
    return None


def summarize_trial(
    prefix: Path,
    scenario: Optional[Path] = None,
    goal: Optional[XY] = None,
    plan_source: str = "/plan",
) -> Tuple[dict, List[dict], List[Tuple[float, List[XY]]], List[XY], List[XY]]:
    trajectory = load_trajectory(prefix)
    if not trajectory:
        raise ValueError(f"no finite trajectory rows for {prefix}")
    plans = grouped_plans(load_plans(prefix, plan_source))
    scenario_samples = load_scenario_samples(scenario)
    scenario_points = [point for _time, point in scenario_samples]
    observed_points = load_obstacle_points(prefix)
    xy = [(number(row, "pose_x"), number(row, "pose_y")) for row in trajectory]
    times = [number(row, "t") for row in trajectory]
    actual_length = path_length(xy)
    start = xy[0]
    target = infer_goal(trajectory, goal)
    direct_length = (math.hypot(target[0] - start[0], target[1] - start[1])
                     if target else math.nan)
    errors = [nearest_distance(point, plan_at(plans, t))
              for point, t in zip(xy, times)]
    crosstrack = ([point_to_segment_distance(point, start, target)
                   for point in xy] if target else [])
    obstacle_clearance = [point_to_polyline_distance(point, scenario_points)
                          for point in xy]
    latest_plan = plans[-1][1] if plans else []
    final = xy[-1]
    final_error = (math.hypot(final[0] - target[0], final[1] - target[1])
                   if target else math.nan)
    statuses = [row.get("goal_status", "") for row in trajectory]
    terminal_status = infer_status(trajectory)
    dynamic_tracks = [first_number(row, ("dynamic_tracks", "dynamic_track_count"), 0.0)
                      for row in trajectory]
    dynamic_points = [first_number(
        row, ("dynamic_predicted_points", "dynamic_point_count"), 0.0)
        for row in trajectory]
    dynamic_speed = [first_number(
        row, ("dynamic_max_speed_mps", "dynamic_max_speed"), 0.0)
        for row in trajectory]
    safety = [number(row, "safety_stop", 0.0) for row in trajectory]
    left = [first_number(row, ("left_thrust_n", "left_thrust")) for row in trajectory]
    right = [first_number(row, ("right_thrust_n", "right_thrust")) for row in trajectory]
    row_path_clearance = [number(row, "path_min_clearance") for row in trajectory]
    finite_row_clearance = [v for v in row_path_clearance if math.isfinite(v)]
    scenario_is_dynamic = (len(scenario_points) >= 2 and
                           path_length(scenario_points) > 0.5)
    result = {
        "run_id": prefix.name,
        "trajectory_samples": len(trajectory),
        "duration_s": times[-1] - times[0],
        "status": terminal_status,
        "succeeded": terminal_status.upper().startswith("SUCCEEDED"),
        "start_x": start[0], "start_y": start[1],
        "goal_x": target[0] if target else math.nan,
        "goal_y": target[1] if target else math.nan,
        "final_x": final[0], "final_y": final[1],
        "final_goal_error_m": final_error,
        "direct_distance_m": direct_length,
        "actual_path_length_m": actual_length,
        "latest_plan_length_m": path_length(latest_plan),
        "plan_snapshot_count": len(plans),
        "mean_plan_tracking_error_m": (
            sum(e for e in errors if math.isfinite(e)) /
            max(1, sum(math.isfinite(e) for e in errors))),
        "p95_plan_tracking_error_m": percentile(errors, 0.95),
        "max_plan_tracking_error_m": max(
            (e for e in errors if math.isfinite(e)), default=math.nan),
        "mean_crosstrack_m": (sum(crosstrack) / len(crosstrack)
                               if crosstrack else math.nan),
        "p95_crosstrack_m": percentile(crosstrack, 0.95),
        "max_crosstrack_m": max(crosstrack, default=math.nan),
        # This is distance to the *motion polyline*, not distance to an
        # arbitrary historical position of a moving obstacle.
        "min_scenario_clearance_m": min(
            (d for d in obstacle_clearance if math.isfinite(d)), default=math.nan),
        "scenario_path_length_m": path_length(scenario_points),
        "scenario_is_dynamic": scenario_is_dynamic,
        "scenario_samples": len(scenario_points),
        "observed_obstacle_points": len(observed_points),
        "min_logged_path_clearance_m": min(finite_row_clearance, default=math.nan),
        "safety_stop_samples": int(sum(1 for value in safety if value > 0.5)),
        "max_dynamic_tracks": max(dynamic_tracks, default=0.0),
        "max_dynamic_predicted_points": max(dynamic_points, default=0.0),
        "max_dynamic_speed_mps": max(dynamic_speed, default=0.0),
        "left_peak_N": max((abs(value) for value in left if math.isfinite(value)),
                           default=math.nan),
        "right_peak_N": max((abs(value) for value in right if math.isfinite(value)),
                             default=math.nan),
        "terminal_left_N": left[-1] if left else math.nan,
        "terminal_right_N": right[-1] if right else math.nan,
        "terminal_thrust_zero": bool(
            left and right and abs(left[-1]) < 1.0e-3 and abs(right[-1]) < 1.0e-3),
        "status_samples": len(statuses),
    }
    return result, trajectory, plans, scenario_points, observed_points


def write_metrics(path: Path, metrics: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "value"])
        writer.writerows(metrics.items())
