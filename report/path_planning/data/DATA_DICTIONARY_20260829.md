# Static route CSV fields (2026-08-29)

`static_route_stable_20260829.csv` is a real VRX recording, not synthetic data.
Coordinates are in `camera_init`; the target in this run was `(4, 0)`.

| Field | Meaning |
|---|---|
| `t` | Logger elapsed simulation time (s). |
| `pose_x`, `pose_y`, `pose_yaw` | Point-LIO pose and yaw. |
| `goal_x`, `goal_y`, `goal_active` | Last target and whether the goal is active. |
| `des_vx`, `des_wz` | Nav2 `/cmd_vel_smoothed` command. |
| `act_vx`, `act_wz` | Point-LIO odometry twist. |
| `crosstrack_err`, `along_err` | Error relative to the straight start-to-goal line. |
| `path_min_clearance`, `path_length` | Approximate path-to-structure-cloud distance and current `/plan` length. |
| `safety_stop` | Independent direct-LiDAR emergency stop flag. |
| `raw_obstacle_pts`, `safety_cloud_pts`, `lio_input_pts`, `registered_pts` | Point counts at the relevant cloud stages. |
| `dynamic_track_count`, `dynamic_point_count`, `dynamic_max_speed` | Dynamic tracker diagnostics; this run is not a dynamic A/B validation. |
| `goal_accepted`, `goal_status`, `goal_result_code` | Latched gateway/action state; action status 4 means succeeded, 5 canceled, 6 aborted. |

For MATLAB, plot `pose_x` vs `pose_y`, overlay `(goal_x,goal_y)`, and compare
`crosstrack_err`, `along_err`, `path_length`, point counts, and terminal status.
Do not infer physical arrival from `goal_status` alone: also calculate the
final Euclidean goal error from pose and target.
