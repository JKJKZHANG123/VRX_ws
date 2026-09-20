#!/usr/bin/env python3
"""Create static-route and dynamic-obstacle USV comparison reports.

Examples:
  python3 experiments/compare_navigation_trials.py static \
      --prefix report3/data/static_obstacle_20260830_verified3 \
      --scenario-events report3/data/static_obstacle_20260830_verified3_scenario.csv

  python3 experiments/compare_navigation_trials.py dynamic \
      --disabled-prefix report3/data/dynamic_ab_tracker_disabled_dynamic_20260829_2050_disabled \
      --enabled-prefix report3/data/dynamic_ab_tracker_enabled_dynamic_20260829_2120_enabled \
      --disabled-events report3/data/dynamic_ab_tracker_disabled_dynamic_20260829_2050_disabled_obstacle_events.csv \
      --enabled-events report3/data/dynamic_ab_tracker_enabled_dynamic_20260829_2120_enabled_obstacle_events.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav_trial_analysis import (  # noqa: E402
    XY,
    grouped_plans,
    load_plans,
    number,
    path_length,
    plan_at,
    point_to_segment_distance,
    read_csv,
    summarize_trial,
    write_metrics,
)


def finite_xy(rows: Sequence[dict], x_key: str, y_key: str) -> List[XY]:
    return [(number(row, x_key), number(row, y_key)) for row in rows
            if math.isfinite(number(row, x_key))
            and math.isfinite(number(row, y_key))]


def dump_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as stream:
        json.dump(value, stream, indent=2, allow_nan=True)


def obstacle_cloud(ax, points: Sequence[XY]) -> None:
    if not points:
        return
    stride = max(1, len(points) // 4000)
    selected = points[::stride]
    ax.scatter([p[0] for p in selected], [p[1] for p in selected],
               s=2, c="0.55", alpha=0.20, label="observed obstacle cloud")


def plan_history(ax, plans: Sequence[Tuple[float, List[XY]]]) -> None:
    if not plans:
        return
    visible = plans if len(plans) <= 80 else plans[::max(1, len(plans) // 80)]
    for _time, points in visible:
        ax.plot([p[0] for p in points], [p[1] for p in points],
                color="tab:blue", alpha=0.10, linewidth=0.8)
    first = plans[0][1]
    latest = plans[-1][1]
    ax.plot([p[0] for p in first], [p[1] for p in first],
            "--", color="tab:cyan", linewidth=1.4, label="first planned path")
    if latest != first:
        ax.plot([p[0] for p in latest], [p[1] for p in latest],
                color="tab:purple", linewidth=2.0, label="latest planned path")


def draw_trial(ax, summary: dict, trajectory: Sequence[dict],
               plans: Sequence[Tuple[float, List[XY]]],
               scenario_points: Sequence[XY], observed_points: Sequence[XY],
               title: str, obstacle_radius: float,
               path_color: str = "tab:orange",
               path_label: str = "measured USV path") -> None:
    actual = finite_xy(trajectory, "pose_x", "pose_y")
    plan_history(ax, plans)
    if actual:
        ax.plot([p[0] for p in actual], [p[1] for p in actual],
                color=path_color, linewidth=2.2, label=path_label)
        ax.scatter(*actual[0], color="green", s=45, zorder=5, label="start")
        ax.scatter(*actual[-1], color="black", marker="x", s=65, zorder=5,
                   label="measured end")
    goal = (summary["goal_x"], summary["goal_y"])
    start = (summary["start_x"], summary["start_y"])
    if all(math.isfinite(v) for v in (*start, *goal)):
        ax.plot([start[0], goal[0]], [start[1], goal[1]],
                ":", color="0.25", linewidth=1.2, label="straight baseline")
        ax.scatter(*goal, color="black", marker="*", s=90, zorder=5,
                   label="goal")
    if scenario_points:
        sx, sy = zip(*scenario_points)
        ax.plot(sx, sy, color="red", linewidth=1.5, alpha=0.65,
                label="obstacle motion path" if len(scenario_points) > 1 else
                "obstacle position")
        ax.scatter(sx, sy, c="red", marker="s", s=38, alpha=0.8)
        if obstacle_radius > 0.0:
            ax.add_patch(Circle((sx[0], sy[0]), obstacle_radius,
                                fill=False, linestyle="--", linewidth=1.0,
                                color="red", alpha=0.6,
                                label=f"clearance radius {obstacle_radius:g} m"))
    obstacle_cloud(ax, observed_points)
    ax.set_title(title)
    ax.set_xlabel("camera_init x [m]")
    ax.set_ylabel("camera_init y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="best")


def plot_static(prefix: Path, summary: dict, trajectory: Sequence[dict],
                plans: Sequence[Tuple[float, List[XY]]], scenario: Sequence[XY],
                observed: Sequence[XY], output: Path, obstacle_radius: float) -> None:
    times = [number(row, "t") for row in trajectory]
    errors = []
    for row, t in zip(trajectory, times):
        points = plan_at(plans, t)
        if points:
            errors.append((t, min(math.hypot(number(row, "pose_x") - x,
                                              number(row, "pose_y") - y)
                                  for x, y in points)))
    fig, (ax, error_ax) = plt.subplots(1, 2, figsize=(15, 6),
                                        constrained_layout=True)
    draw_trial(ax, summary, trajectory, plans, scenario, observed,
               "Static obstacle: planned path vs measured USV path",
               obstacle_radius)
    if errors:
        error_ax.plot([x[0] for x in errors], [x[1] for x in errors],
                      color="tab:orange", linewidth=1.5)
    error_ax.set_title("Measured path to current planned path")
    error_ax.set_xlabel("relative simulation time [s]")
    error_ax.set_ylabel("nearest plan distance [m]")
    error_ax.grid(True, alpha=0.3)
    error_ax.text(0.98, 0.98,
                  f"status: {summary['status']}\n"
                  f"baseline / planned / actual: "
                  f"{summary['direct_distance_m']:.2f} / "
                  f"{summary['latest_plan_length_m']:.2f} / "
                  f"{summary['actual_path_length_m']:.2f} m\n"
                  f"final goal error: {summary['final_goal_error_m']:.2f} m\n"
                  f"P95 plan error: {summary['p95_plan_tracking_error_m']:.2f} m",
                  transform=error_ax.transAxes, ha="right", va="top", fontsize=8,
                  bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
    fig.suptitle(f"Navigation comparison: {prefix.name}")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_dynamic(records: Sequence[Tuple[str, dict, Sequence[dict],
                                           Sequence[Tuple[float, List[XY]]],
                                           Sequence[XY], Sequence[XY]]],
                 output: Path) -> None:
    """Plot matched tracker A/B runs using fields from either logger format.

    Legacy dynamic runs do not contain planner snapshots, so the second panel
    falls back to straight-line cross-track error and overlays the logger's
    time-local ``path_min_clearance`` when it is available.
    """
    fig, axes = plt.subplots(2, 2, figsize=(15, 11), constrained_layout=True)
    colors = {"tracker disabled": "tab:blue", "tracker enabled": "tab:orange"}
    for index, (label, summary, trajectory, plans, scenario, observed) in enumerate(records):
        draw_trial(
            axes[0, 0], summary, trajectory, plans,
            scenario if index == 0 else [],
            observed if index == 0 else [],
            "Dynamic obstacle: matched routes", 0.0,
            path_color=colors[label], path_label=label)
        times = [number(row, "t") for row in trajectory]
        pose = [(number(row, "pose_x"), number(row, "pose_y"))
                for row in trajectory]
        plan_error = []
        cross_track = []
        clearance = []
        tracks = []
        predicted = []
        left_thrust = []
        right_thrust = []
        safety = []
        for row, t, point in zip(trajectory, times, pose):
            plan = plan_at(plans, t)
            if plan:
                plan_error.append((t, min(math.hypot(point[0] - x, point[1] - y)
                                          for x, y in plan)))
            if all(math.isfinite(value) for value in
                   (summary["start_x"], summary["start_y"],
                    summary["goal_x"], summary["goal_y"])):
                cross_track.append((
                    t, point_to_segment_distance(
                        point, (summary["start_x"], summary["start_y"]),
                        (summary["goal_x"], summary["goal_y"]))))
            value = number(row, "path_min_clearance")
            if math.isfinite(value):
                clearance.append((t, value))
            tracks.append((t, number(row, "dynamic_tracks", 0.0)))
            predicted.append((t, number(row, "dynamic_predicted_points", 0.0)))
            left_thrust.append((t, number(row, "left_thrust_n", 0.0)))
            right_thrust.append((t, number(row, "right_thrust_n", 0.0)))
            safety.append((t, number(row, "safety_stop", 0.0)))

        tracking = plan_error or cross_track
        if tracking:
            axes[0, 1].plot([x for x, _ in tracking], [y for _, y in tracking],
                            color=colors[label], label=(
                                f"{label} plan error" if plan_error else
                                f"{label} cross-track"))
        if clearance:
            axes[0, 1].plot([x for x, _ in clearance], [y for _, y in clearance],
                            color=colors[label], linestyle="--", alpha=0.75,
                            label=f"{label} logged clearance")
        axes[1, 0].plot([x for x, _ in tracks], [y for _, y in tracks],
                        color=colors[label], label=f"{label} tracks")
        axes[1, 0].plot([x for x, _ in predicted], [y for _, y in predicted],
                        color=colors[label], linestyle="--", alpha=0.75,
                        label=f"{label} predicted points")
        axes[1, 1].plot([x for x, _ in left_thrust], [y for _, y in left_thrust],
                        color=colors[label], label=f"{label} left thrust")
        axes[1, 1].plot([x for x, _ in right_thrust], [y for _, y in right_thrust],
                        color=colors[label], linestyle="--",
                        label=f"{label} right thrust")
        stop_points = [(t, thrust) for (t, stop), (_, thrust) in zip(safety, left_thrust)
                       if stop > 0.5]
        if stop_points:
            axes[1, 1].scatter([x for x, _ in stop_points],
                               [y for _, y in stop_points],
                               color=colors[label], marker="x", s=28,
                               label=f"{label} safety stop")

    axes[0, 0].set_title("Dynamic obstacle: matched routes")
    axes[0, 0].set_xlabel("camera_init x [m]")
    axes[0, 0].set_ylabel("camera_init y [m]")
    axes[0, 0].axis("equal")
    axes[0, 1].set_title("Cross-track / logged path clearance")
    axes[0, 1].set_xlabel("relative simulation time [s]")
    axes[0, 1].set_ylabel("distance [m]")
    axes[1, 0].set_title("Dynamic tracker activity")
    axes[1, 0].set_xlabel("relative simulation time [s]")
    axes[1, 0].set_ylabel("tracks / predicted points")
    axes[1, 1].set_title("Thrust and safety-stop samples")
    axes[1, 1].set_xlabel("relative simulation time [s]")
    axes[1, 1].set_ylabel("thrust [N]")
    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            axis.legend(unique.values(), unique.keys(), fontsize=7, loc="best")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def write_dynamic_markdown(path: Path, records: Sequence[Tuple[str, dict, object, object, object, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        ("status", "Status"), ("trajectory_samples", "Samples"),
        ("duration_s", "Duration [s]"), ("direct_distance_m", "Baseline [m]"),
        ("latest_plan_length_m", "Latest plan [m]"),
        ("actual_path_length_m", "Actual path [m]"),
        ("final_goal_error_m", "Final goal error [m]"),
        ("p95_crosstrack_m", "P95 cross-track [m]"),
        ("min_scenario_clearance_m", "Min scenario clearance [m]"),
        ("safety_stop_samples", "Safety stops"),
        ("max_dynamic_tracks", "Max tracks"),
        ("max_dynamic_predicted_points", "Max predicted points"),
        ("max_dynamic_speed_mps", "Max tracker speed [m/s]"),
        ("terminal_thrust_zero", "Terminal thrust zero"),
    ]
    with path.open("w") as stream:
        stream.write("# Dynamic-obstacle navigation comparison\n\n")
        stream.write("This report compares matched tracker-disabled and tracker-enabled runs. "
                     "Both trajectory and plan geometry are in `camera_init`.\n\n")
        stream.write("| Metric | " + " | ".join(label for _key, label in fields) + " |\n")
        stream.write("|---|" + "---|" * len(fields) + "\n")
        for name, summary, *_rest in records:
            values = []
            for key, _label in fields:
                value = summary.get(key, math.nan)
                if isinstance(value, bool):
                    values.append("yes" if value else "no")
                elif isinstance(value, float):
                    values.append("n/a" if not math.isfinite(value) else f"{value:.3f}")
                else:
                    values.append(str(value))
            stream.write("| " + name + " | " + " | ".join(values) + " |\n")
        stream.write("\n## Interpretation\n\n")
        stream.write("The enabled run can only be credited with prediction-driven avoidance "
                     "when `/usv/dynamic_obstacle_cloud` is configured as a Nav2 costmap "
                     "observation source and the tracker metrics are non-zero. A successful "
                     "goal alone is not sufficient evidence of dynamic prediction use.\n")


def parse_xy(text: str) -> XY:
    try:
        x, y = (float(value.strip()) for value in text.split(",", 1))
    except (ValueError, TypeError):
        raise argparse.ArgumentTypeError("goal must be written as x,y")
    return x, y


def static_command(args: argparse.Namespace) -> int:
    prefix = Path(args.prefix)
    scenario = Path(args.scenario_events) if args.scenario_events else None
    summary, trajectory, plans, scenario_points, observed = summarize_trial(
        prefix, scenario=scenario, goal=args.goal, plan_source=args.plan_source)
    output = Path(args.output) if args.output else prefix_file(prefix, "_comparison.png")
    dump_json(output.with_suffix(".json"), summary)
    write_metrics(output.with_suffix(".csv"), summary)
    plot_static(prefix, summary, trajectory, plans, scenario_points, observed,
                output, args.obstacle_radius)
    print(f"figure: {output}")
    print(f"metrics: {output.with_suffix('.json')}")
    for key, value in summary.items():
        print(f"{key}: {value}")
    return 0


def prefix_file(prefix: Path, suffix: str) -> Path:
    return prefix.with_name(prefix.name + suffix)


def dynamic_command(args: argparse.Namespace) -> int:
    records = []
    for label, prefix_text, event_text in (
            ("tracker disabled", args.disabled_prefix, args.disabled_events),
            ("tracker enabled", args.enabled_prefix, args.enabled_events)):
        prefix = Path(prefix_text)
        scenario = Path(event_text) if event_text else None
        summary, trajectory, plans, scenario_points, observed = summarize_trial(
            prefix, scenario=scenario, goal=args.goal, plan_source=args.plan_source)
        records.append((label, summary, trajectory, plans, scenario_points, observed))
    base = Path(args.output) if args.output else Path(args.enabled_prefix + "_dynamic_comparison.png")
    dump_json(base.with_suffix(".json"), {name: summary for name, summary, *_ in records})
    with base.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        keys = list(records[0][1].keys())
        writer.writerow(["run"] + keys)
        for name, summary, *_ in records:
            writer.writerow([name] + [summary.get(key, "") for key in keys])
    write_dynamic_markdown(base.with_suffix(".md"), records)
    plot_dynamic(records, base)
    print(f"figure: {base}")
    print(f"metrics: {base.with_suffix('.json')}")
    print(f"report: {base.with_suffix('.md')}")
    for name, summary, *_ in records:
        print(f"[{name}] status={summary['status']} final_error={summary['final_goal_error_m']:.3f} "
              f"actual_length={summary['actual_path_length_m']:.3f} "
              f"min_clearance={summary['min_scenario_clearance_m']:.3f}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    static = sub.add_parser("static", help="analyse one static-obstacle run")
    static.add_argument("--prefix", required=True)
    static.add_argument("--scenario-events", default="")
    static.add_argument("--goal", type=parse_xy, default=None,
                        help="override goal when the recorder did not latch it: x,y")
    static.add_argument("--plan-source", default="/plan")
    static.add_argument("--obstacle-radius", type=float, default=2.5)
    static.add_argument("--output", default="")
    static.set_defaults(handler=static_command)
    dynamic = sub.add_parser("dynamic", help="compare matched tracker A/B runs")
    dynamic.add_argument("--disabled-prefix", required=True)
    dynamic.add_argument("--enabled-prefix", required=True)
    dynamic.add_argument("--disabled-events", default="")
    dynamic.add_argument("--enabled-events", default="")
    dynamic.add_argument("--goal", type=parse_xy, default=None)
    dynamic.add_argument("--plan-source", default="/plan")
    dynamic.add_argument("--output", default="")
    dynamic.set_defaults(handler=dynamic_command)
    args = parser.parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
