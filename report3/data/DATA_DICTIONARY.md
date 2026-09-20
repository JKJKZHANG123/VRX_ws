# Navigation Data Dictionary

Coordinates use `camera_init` unless stated otherwise. CSV time `t` is elapsed
ROS simulation time from the logger's first timer tick.

## Navigation sample CSV

| Field | Meaning |
|---|---|
| `t` | Logger elapsed time in seconds. |
| `pose_x`, `pose_y`, `pose_yaw` | Point-LIO pose and yaw. |
| `goal_x`, `goal_y`, `goal_active` | Current target and active-goal latch. For M5 these track the currently reported/final target, not the full route. |
| `des_vx`, `des_wz` | Nav2 smoothed linear/angular command. |
| `act_vx`, `act_wz` | Odometry-reported body velocity. |
| `crosstrack_err`, `along_err`, `heading_err` | Straight start-to-goal tracking diagnostics. |
| `mode` | Navigation mode, normally `PLAN`. |
| `obstacle_pts` | Structure-cloud points observed by the logger. |
| `left_thrust`, `right_thrust` | Published thruster commands in newtons. |
| `safety_stop` | Independent SafetyCloud emergency-stop flag (`0/1`). |
| `path_min_clearance` | Approximate latest-plan clearance from recorded structure points. |
| `path_length` | Length of the latest `/plan` in metres. |
| `raw_obstacle_pts`, `safety_cloud_pts` | Raw and safety-cloud point counts. |
| `lio_input_pts`, `registered_pts` | Point-LIO input and registered-cloud counts. |
| `dynamic_track_count`, `dynamic_point_count`, `dynamic_max_speed` | Dynamic tracker diagnostics; not proof of avoidance. |
| `goal_accepted`, `goal_status`, `goal_result_code` | Latched action state; result codes include `4=SUCCEEDED`, `5=CANCELED`, `6=ABORTED`. |

## M5 companion files

`*_events.csv` contains wall time, simulation time, event name, and detail for
transform readiness, marker publication, action acceptance, and terminal result.
`*_waypoints.csv` contains sequence, target coordinates, acceptance tolerance,
pass/fail, first entry time, elapsed time after acceptance, and minimum error.

## Metrics summary

`metrics_summary.csv` has one row per attempt. `validity` must be `valid` before
a row is used in a claim. `traveled_distance_m` is the sum of consecutive finite
pose displacements. Cross-track percentiles, clearance, path length, peak speeds,
SafetyCloud counts, tracker maxima, and median point counts are derived from the
sample CSV. Empty fields indicate unavailable or invalid measurements.

## Interpretation rules

Use `final_goal_error_m`, waypoint acceptance, and terminal status together to
claim arrival. A dynamic-obstacle conclusion requires tracker-disabled and
tracker-enabled runs with identical world, start pose, target, commanded speed,
obstacle geometry/trajectory, and duration. Tracker point counts alone are not
an accuracy or safety guarantee.
