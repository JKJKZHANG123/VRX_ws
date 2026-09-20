# Path-planning build artifacts

This directory documents the staged static and dynamic path-planning build.
Runtime before/after CSV files are only added after a matched VRX experiment;
configuration changes are not presented as measured improvements.

## Current implementation

- **Static route safety:** validated RViz/Gazebo/CLI goals, WAM-V footprint
  costmap checks, static structure marking, raw-scan local clearing, and Smac
  Hybrid-A* plus regulated pure pursuit.
- **Dynamic stage:** `/usv/raw_obstacle_cloud` is transformed to `camera_init`,
  mapped structure is subtracted, residual returns are grid-clustered, and
  centroids are tracked with a nearest-neighbour alpha-beta filter. Current and
  8-second predicted centres are published to
  `/usv/dynamic_obstacle_cloud` for the local costmap.
- **Safety ownership:** raw LiDAR remains the source for clearing and the
  independent SafetyCloud emergency path is unchanged. The prediction layer is
  marking-only and bounded by `observation_persistence`.

## Diagnostics and MATLAB fields

`nav_data_logger.py` records `dynamic_track_count`, `dynamic_point_count`, and
`dynamic_max_speed` in addition to `path_min_clearance`, `path_length`,
`crosstrack_err`, `safety_stop`, point-cloud counts, and goal status. The
prediction node also publishes `/usv/dynamic_predictions`,
`/usv/dynamic_tracks_markers`, and `/usv/dynamic_metrics`.

Do not call the dynamic algorithm effective until a matched VRX A/B run has
been completed. Compare tracker disabled/enabled under the same world, start
pose, target, speed, and moving-obstacle trajectory. Report collision status,
minimum path/boat clearance, route length, cross-track error, goal result,
number of confirmed tracks, maximum estimated speed, and stale-track duration.

## Validation completed on 2026-08-28

```text
colcon build --symlink-install --packages-select usv_navigation usv_cloud_filter  PASS
3 pure dynamic-obstacle tests                                               PASS
ament_flake8 changed Python files                                            PASS
VRX moving-obstacle A/B                                                      NOT RUN
```

The absence of the VRX A/B CSV is intentional; no runtime improvement is
claimed yet. See `IMPLEMENTATION_20260828.md` for algorithm details and the
next experiment protocol.

## Validation completed on 2026-08-29

The static route build was changed to stable 0.25 Hz replanning, bounded Smac search, latched goal telemetry, and a 0.8 m goal tolerance. A real short VRX run is stored as `data/static_route_stable_20260829.csv`. It produced a 0.382 m maximum cross-track error and no SafetyCloud stop, but the process loaded the prior 2.0 m tolerance and ended at about `(2.11, 0.41)` for target `(4, 0)`. It is therefore a planner/control smoke test, not proof of physical goal arrival. See `IMPLEMENTATION_20260829.md` and `data/DATA_DICTIONARY_20260829.md`.
