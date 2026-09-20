# Static path-planning implementation and validation — 2026-08-29

## Changes in this iteration

1. **Stable replanning BT** — `config/usv_nav_to_pose.xml` now checks the
   existing path and replans at most at 0.25 Hz, instead of replacing a valid
   Smac path every second. Spin-in-place and reverse recovery remain disabled.
2. **Bounded Smac search** — `nav2_params.yaml` changes Hybrid-A* to 36 heading
   bins, `max_iterations: 300000`, `max_planning_time: 2.0`, and enables the
   obstacle-heuristic cache. This is a runtime-stability change, not a measured
   accuracy improvement by itself.
3. **Goal-state telemetry** — `click_to_goal.py` publishes transient-local
   status (`QUEUED`, `ACCEPTED`, `SUCCEEDED`, `CANCELED`, `ABORTED`, etc.). The
   logger subscribes with matching QoS and adds `goal_result_code`, so a CSV
   records the terminal action result even when the logger starts late.
4. **Goal success criterion** — `xy_goal_tolerance` is 0.8 m. The former 2.0 m
   tolerance could classify a 4 m test as successful while the boat was still
   about 1.9 m from the target.
5. **Stop-script cleanup** — `usv.sh` now also matches the project’s Python
   navigation nodes, Point-LIO process, and Gazebo bridge processes when
   cleaning remnants. It is still verified with `ps` after every restart.

## Real VRX short-route result

Data file: `data/static_route_stable_20260829.csv`.

The experiment used the Sydney Regatta world, a fresh single launch chain,
start near `(0, 0)`, target `(4, 0)`, and a 35 s requested run. The logger
recorded 218 rows over 43.6 s because ROS simulation time continued while the
shell polling loop waited for topic responses.

Measured observations:

- Nav2 action status: `SUCCEEDED code=4`.
- The action ended at about `(2.11, 0.41)`; the 0.8 m criterion is not reflected
  in this row because the running process loaded the previous configuration
  before the latest tolerance edit. Therefore this file is **not evidence that
  the 4 m goal was physically reached**.
- During the active portion, maximum straight-line cross-track error was about
  `0.382 m`; no SafetyCloud stop was recorded.
- The path topic contained a route of about `4.13 m`, and the vessel moved
  forward without the previous large y≈4 m excursion or a planner timeout.
- The control bridge showed a short startup safety stop, then cleared it.

This is a **successful planner/control smoke test but an incomplete goal
arrival test**, not a claim of end-to-end navigation success. A new run after
restart is required to validate the 0.8 m tolerance and to produce the matched
long-route result.
