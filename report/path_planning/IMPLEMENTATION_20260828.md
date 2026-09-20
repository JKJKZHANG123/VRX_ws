# Static Path-Planning Safety Implementation (2026-08-28)

## Scope of this build

This build starts the first, low-risk stage of the agreed architecture:

`Point-LIO map + direct LiDAR scan -> separated Nav2 costmaps -> validated goal -> Smac Hybrid-A* + RPP -> independent thrust safety gate`

No dynamic-obstacle prediction or MPPI controller has been enabled yet. The
current control bridge remains the command authority and its emergency stop is
not removed.

## Code changes

1. **Validated Nav2 goal gateway** — `src/usv_navigation/usv_navigation/click_to_goal.py`
   now sends a real `NavigateToPose` action instead of only publishing a pose
   topic. RViz clicks, `goto.sh`, and single Gazebo markers use this same path.
   A goal is rejected when its frame is not `camera_init`, its distance exceeds
   45 m, it is outside an enabled geofence, or the WAM-V-sized 2.8 m check disk
   touches unknown/high-cost cells in the latest costmap. `/usv/goal_status` and
   `/usv/goal_accepted` expose the decision for logs and MATLAB.

2. **Correct final heading behavior** — an identity quaternion means “hold the
   current heading”, not “force yaw = 0”. This removes an unnecessary initial
   turn near an obstacle.

3. **Static/current obstacle separation** — in `nav2_params.yaml`, the
   accumulated `/usv/structure_cloud` marks but never clears costmap cells.
   The rolling local costmap still uses `/usv/raw_obstacle_cloud` for current
   observations and clearing. The global layer no longer consumes the raw scan,
   preventing scan-by-scan ghost obstacles from becoming global obstacles.

4. **Navigation logger extension** — `nav_data_logger.py` records goal
   acceptance/status along with trajectory, path clearance, thrust, safety-stop,
   and point-cloud density fields.

## Verification status

Offline verification completed: Python syntax compilation and YAML parsing pass.
A VRX runtime A/B result has **not** been fabricated: the next experiment must
run the same goal twice (baseline and this build) and export the resulting CSV
under `data/` before claiming a performance improvement.

Recommended command sequence after review:

```bash
./usv.sh stop                 # required before any restart
ps -ef | grep -E 'gz|ros2|rviz|nav2|point_lio|usv' | grep -v grep
colcon build --symlink-install --packages-select usv_navigation
source install/setup.bash
```

Record with `./run_nav_test.sh` and compare `goal_accepted`,
`goal_status`, `path_min_clearance`, `crosstrack_err`, `safety_stop`, and
trajectory length in MATLAB.

## Known next step

The global costmap is still a rolling online map and Smac currently permits
unknown cells because no fixed static water-area map has been calibrated for
every VRX world. Before enabling `allow_unknown: false`, create and validate
a fixed world-frame static map/geofence; otherwise valid open-water goals can be
rejected at startup. Dynamic tracking will be added only after this static
route test passes.

## Dynamic-obstacle stage added after the static-safety build

The first dynamic stage is now implemented, but it has **not yet been claimed as
VRX-validated**. The new node is:

- `src/usv_navigation/usv_navigation/dynamic_obstacle_tracker.py`
- configuration: `src/usv_navigation/config/dynamic_obstacle_tracker.yaml`
- launch entry: `usv_navigation/launch/navigation.launch.py`

Its processing chain is:

1. Read `/usv/raw_obstacle_cloud` (current LiDAR, already water/self filtered).
2. Transform returns into `camera_init` and subtract XY voxels represented by
   `/usv/structure_map_cloud`; this prevents mapped shore and fixed buoys from
   being treated as moving targets.
3. Use an 8-connected XY grid clusterer. Components smaller than 3 points or
   larger than 250 points/8 m are rejected as spray or merged shoreline.
4. Associate centroids by nearest-neighbour gating and estimate velocity with
   an alpha-beta constant-velocity filter (`alpha=0.65`, `beta=0.20`).
5. Publish current/predicted centres for an 8 s horizon on
   `/usv/dynamic_obstacle_cloud` in `wamv/wamv/base_link`.
6. Add the prediction cloud as a marking-only local-costmap source with bounded
   observation persistence. The raw scan remains responsible for ray clearing;
   the dynamic layer never clears persistent shoreline cells.

Diagnostics are published on `/usv/dynamic_predictions`,
`/usv/dynamic_tracks_markers`, and `/usv/dynamic_metrics`. The navigation CSV
now records `dynamic_track_count`, `dynamic_point_count`, and
`dynamic_max_speed`, so a matched moving-obstacle experiment can be plotted in
MATLAB without changing the logger again.

### Current verification and limitations

- `python3 -m py_compile` passes for all changed Python nodes.
- Three pure algorithm tests pass: grid clustering, oversized shoreline
  rejection, and XY transform.
- `colcon build --symlink-install --packages-select usv_navigation usv_cloud_filter`
  passes.
- A 3-second standalone ROS startup smoke test reaches the tracker startup
  log. DDS socket warnings are sandbox/network restrictions, not tracker
  exceptions.
- No VRX moving-obstacle run has been performed in this build, so there is no
  performance CSV yet. The next run must compare the same route with the
  tracker disabled/enabled and retain collision, minimum clearance, cross-track
  error, track count, predicted speed, and goal result fields.

The prediction is deliberately local and conservative. It does not yet use
CPA/TCPA, COLREGs, semantic vessel classes, or MPPI. Those should only be added
after this stage passes a stationary-shore regression and a controlled
crossing-target experiment.

## Verification follow-up (2026-08-29)

Static review found that the previous `/usv/structure_map_cloud` output was
built directly from Point-LIO `/cloud_registered`, which is a registered
single scan; it was not a persistent map. This could make the dynamic tracker
subtract a moving target from the same frame and produce no track. The filter
now voxel-accumulates structure points in `camera_init` for 15 seconds and
freezes the map before dynamic avoidance. This change has passed compilation
and offline checks, but no new VRX dynamic-obstacle performance data is
claimed until an A/B run is recorded.
