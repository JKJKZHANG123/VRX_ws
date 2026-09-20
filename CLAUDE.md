# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 **Jazzy** workspace (Ubuntu 24.04) with two deployment targets sharing one algorithm core:

- **Real robot** — Unitree Unilidar L1/L2 (also Livox/Velodyne/Ouster/Hesai) LiDAR-inertial
  odometry, optional GPS/UTM georeferencing, optional RGB colorization and loop closure.
- **VRX simulation** — an autonomous WAM-V USV: Point-LIO odometry → water-surface cloud
  filtering → Nav2 → differential thrust.

The core algorithm is **Point-LIO**: per-point (high-bandwidth) LiDAR-inertial odometry using
an iterated Kalman filter on manifold (IKFoM) with ikd-Tree map management.

**`.git/` is empty — this checkout has no usable git history.** `git log`/`git diff`/blame
will not explain why code looks the way it does. The reasoning lives in unusually dense code
comments (read them before "simplifying" a value) and in `report*/`. Many constants are the
result of recorded A/B experiments; changing one silently invalidates a report.

**Single workspace.** The VRX packages were merged into `src/vrx/` (the old `~/lidar_project/vrx_ws`
overlay is gone), so everything builds and sources together.

## Build & test

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon build --packages-select usv_navigation     # focused rebuild
colcon test --packages-select usv_navigation      # pytest + ament_copyright/flake8/pep257
colcon test-result --verbose
```

Only `usv_navigation` has real unit tests (`test/test_dynamic_obstacle_tracker.py`, pure
algorithm functions, no ROS graph). Run a single one with `install/setup.bash` sourced:

```bash
python3 -m pytest src/usv_navigation/test/test_dynamic_obstacle_tracker.py -k grid_clusters -v
```

Everything else is validated by live diagnostic scripts, not unit tests:
`src/lidar_timestamp_adapter/test/` holds `verify_*.py` / `measure_*.py` / `diag_*.py` for
topics, TF, timing lag, stationary LIO drift, costmap pipeline and water rejection. Run them
against a live chain with `python3`. `./test_nav2.sh` is a Nav2 dry-run (spawns all nodes,
needs no sim or LIO).

## Running the VRX USV chain

`./usv.sh start` runs the whole chain in order and is the normal entry point;
`stop` / `status` / `restart` manage it. Logs land in `.usv_logs/`, PIDs in `.usv_pids`.
The seven steps, which are also the individual scripts:

| # | Script | What it starts |
|---|--------|----------------|
| ① | `./sim.sh [world]` | VRX Gazebo sim (default `sydney_regatta`) + `clock_bridge_watchdog` |
| ② | `./lio.sh [norviz]` | `cloud_filter` (NaN/water reject) + Point-LIO + LIO RViz |
| ③ | — | water-surface cloud filter (`usv_cloud_filter`), started after a TF check |
| ④ | `./nav.sh` | Nav2 stack + `costmap_3d_markers` + `click_to_goal` goal gateway |
| ⑤ | `./bridge.sh` | `usv_control_bridge`: Nav2 velocity → differential thrust |
| ⑥ | — | `usv_click_gazebo`: click in the Gazebo view → Nav2 goal |
| ⑦ | `./view.sh [2d]` | 3D navigation RViz |

GPS anchoring (`./gps.sh`, `navsat_transform_node`) is **commented out** of `usv.sh`:
VRX's simulated GPS jumps and makes the boat "fly" in RViz. `./ekf.sh` is a separate optional
EKF chain, also not part of `usv.sh` — if used it must be cold-started after `sim.sh` and
`lio.sh` and never restarted mid-run (a Gazebo clock jump explodes the EKF `dt` and
`/odometry/filtered` diverges to NaN).

```bash
./goto.sh 20 10 [yaw]   # send a Nav2 goal; 'here' prints pose, 'cancel' aborts
./boat.sh fwd 15        # manual driving, auto-zeros thrust (fwd/back/left/right/stop)
USV_DYNAMIC_TRACKER=true ./usv.sh start   # enable the dynamic-obstacle tracker (default off)
```

`./goto.sh` and `usv.sh` deliberately verify **actual** readiness (every managed Nav2 node
`active [3]`, both action servers present) rather than trusting a live launch PID. Keep that
property when editing them — a surviving launch parent does not mean the chain is usable.

**Between experiment runs**, always `./usv.sh stop && ros2 daemon stop`, then confirm with `ps`
that gz / ros2 / rviz / nav2 / laserMapping / cloud_filter / bridge are all gone. Stale launch
parents create duplicate TF and control writers.

## Running the real robot

```bash
ros2 launch my_mapping_launcher mapping_no_gps.launch.py   # ★ driver → Point-LIO → RViz
ros2 launch my_mapping_launcher all_in_one.launch.py       # + GPS/UTM fusion
ros2 launch my_mapping_launcher with_camera.launch.py      # + camera
ros2 launch point_lio mapping_unilidar_l1.launch.py        # Point-LIO only (also _l2, avia, mid360, ouster64, velody16, horizon)
./demo.sh                                                  # interactive: 1) plain LIO 2) RGB map 3) both
```

## Core algorithm — `point_lio_ros2/`

ROS 2 port of [Point-LIO](https://github.com/hku-mars/Point-LIO) with Unitree + VRX support.

- `src/laserMapping.cpp` — main node. Consumes a LiDAR cloud + IMU, publishes odometry on
  **`/aft_mapped_to_init`** (not `/Odometry`), `/cloud_registered`, `/Laser_map`, `/path`, and
  the `camera_init → aft_mapped` TF.
- `src/Estimator.cpp/h` — the ESKF on IKFoM manifolds. Two parallel filters: IMU-rate prediction
  and LiDAR-rate correction. State = pose, velocity, IMU biases, extrinsic, gravity.
- `src/preprocess.cpp/h` — per-vendor point struct handlers + time-based deskew.
- `src/IMU_Processing.hpp` — forward/backward propagation for per-point undistortion.
- `src/parameters.cpp/h` — YAML → global variables. Add new tunables here **and** in `parameters.h`.
- `include/IKFoM/` (submodule), `include/ikd-Tree/`, `include/FOV_Checker/`.
- `Log/plot*.py` — plot logged IMU/state/timing text files.

**Local modifications beyond upstream Point-LIO** (no git history to reveal them):

- **ZUPT** — an IMU stationarity detector plus zero-velocity update, ~77 references in
  `laserMapping.cpp` and configured by the `zupt_*` keys in `config/vrx_wamv.yaml`. It exists
  because a stationary WAM-V drifted on featureless water. `zupt_position_hold_enable` stays
  `false`: ZUPT constrains velocity, not absolute position.
- PCD saving is flushed **only on graceful SIGINT** (`laserMapping.cpp:1466`). Setting
  `pcd_save_en` at runtime via `ros2 param set` will not trigger a write.

`config/vrx_wamv.yaml` drives `launch/mapping_vrx.launch.py`, which starts three nodes:
`lidar_timestamp_adapter`'s `cloud_filter` (Gazebo `gpu_lidar` emits NaNs that break Point-LIO;
it also rejects returns below `sensor_z_min` to drop the water sheet), `pointlio_mapping`, and
the static `aft_mapped → wamv/wamv/base_link` identity TF. That TF is deliberately owned here,
not by Nav2, so the TF tree is connected before navigation starts.

## Frames

Real robot:

```
camera_init (world origin, set by Point-LIO)
  └── aft_mapped (LIO odometry output)
       └── body (identity to aft_mapped; bridges to robot_localization)
            └── gps_link (measured antenna lever-arm, set in start_mapping.launch.py)
```

VRX sim — same root, different base link, no EKF/GPS by default:

```
camera_init  →  aft_mapped  →  wamv/wamv/base_link
```

`camera_init` **is** Nav2's global frame. `/aft_mapped_to_init` is the odom source directly.
Goals, costmaps, `/cloud_registered` and markers are all in `camera_init`, which is why
`./goto.sh` and RViz "Publish Point" use it unmodified. Sim topics differ from the real robot:
thrust on `/wamv/thrusters/{left,right}/thrust` (Float64), GPS on `/wamv/sensors/gps/gps/fix`.

## USV cloud topology (the part that needs a map)

`usv_cloud_filter/src/obstacle_filter.cpp` is one node with **two independent input paths and
four outputs**. Getting these confused is the easiest way to break navigation:

| Output | Frame | Built from | Consumer |
|--------|-------|-----------|----------|
| `/usv/raw_obstacle_cloud` | `wamv/wamv/base_link` | raw Gazebo scan | **both Nav2 costmaps** (`lidar_cloud`) |
| `/usv/safety_cloud` | base_link | raw Gazebo scan | control bridge emergency stop |
| `/usv/structure_map_cloud` | `camera_init` | `/cloud_registered`, accumulated then **frozen** | dynamic tracker's static-map subtraction |
| `/usv/structure_cloud` | base_link | `/cloud_registered` | `nav_data_logger`, `feature_nav_node` |
| `/usv/costmap_cloud` | base_link | `/cloud_registered` | nothing — legacy/rollback only |

The raw Gazebo scan (`/wamv/sensors/lidars/lidar_wamv_sensor/points`) feeds the costmaps and the
safety stop so obstacle avoidance does not depend on LIO registration. The registered cloud
feeds the persistent structure map. Filtering: self-mask (`self_mask_mode: wamv_geometry`, which
removes float/beam/deck/CPU-case geometry — the old central rectangle hid real bow obstacles),
height band, `max_range: 35 m` crop, 0.1 m voxel, radius outlier removal.

Two non-obvious invariants:

- **Height cropping happens after TF into base_link**, so `z≈0` is the local water surface even
  when `camera_init` drifts vertically. Thresholds (`structure_z_min: 0.10`) are tuned against
  measured buoy return heights; raising them deletes marker buoys from the input.
- **`output_frame` must stay a boat frame.** Nav2's obstacle layer treats the observation frame's
  origin as the sensor origin for range filtering and raytrace clearing. In `camera_init` the
  "sensor" would sit at the map origin and local avoidance breaks as the boat travels.
- `structure_map_*` accumulates for `structure_map_freeze_after_s` (15 s) then freezes, so the
  tracker subtracts a genuine static map rather than the current moving-target scan from itself.

`usv_navigation/dynamic_obstacle_tracker.py` (off unless `USV_DYNAMIC_TRACKER=true`) runs
static-map subtraction → XY connected components → nearest-track association → alpha-beta
velocity filter → short-horizon prediction, publishing `/usv/dynamic_obstacle_cloud` in base_link
as a *second* costmap source. Its `expected_update_rate` is `0.0` on purpose: a nonzero value
would mark the whole costmap stale in tracker-disabled runs and time out the planner.

## Nav2 configuration (`usv_navigation/config/nav2_params.yaml`)

The WAM-V is a **differential-thruster boat that cannot reverse or rotate in place**. That single
fact explains most of the config: `allow_reversing: false`, `use_rotate_to_heading: false`,
`minimum_turning_radius: 2.5`, `motion_model_for_search: DUBIN`, and `behavior_plugins` reduced
to `spin`/`drive_on_heading`/`wait` with `backup` removed.

- Planner `SmacPlannerHybrid`, `allow_unknown: true` while both costmaps set
  `track_unknown_space: true` — unobserved cells must not block the start pose on a rolling map,
  but real point-cloud obstacles remain impassable.
- Controller `RegulatedPurePursuitController`, `desired_linear_vel: 0.6` m/s. Higher speeds made
  the turning radius exceed the Dubins assumption and caused circling/shore collisions.
- Footprint `[[3.1,1.6],[3.1,-1.6],[-2.8,-1.6],[-2.8,1.6]]` with `inflation_radius: 3.8` —
  keep inflation ≥ the circumscribed footprint radius.
- Plain `nav2_costmap_2d::ObstacleLayer`, not STVL: STVL retained voxels after the scan cleared
  them, which ghosted. Both costmaps are `rolling_window` with `clear_after_reading: true`.
- `xy_goal_tolerance: 0.8` — 2.0 let a 4 m test "succeed" at x≈2.1 m.
- A custom BT (`config/usv_nav_to_pose.xml`) does bounded 0.25 Hz replanning; `navigation.launch.py`
  rewrites the placeholder path to an absolute one via `RewrittenYaml`.

`click_to_goal` is the **single goal gateway**, not a passthrough: `/clicked_point` and
`/goal_pose` are validated against `config/goal_guard.yaml` (distance limit, free-and-known
costmap cell, `reject_unknown: true`) and then submitted to the `NavigateToPose` **action**.
Publishing `/goal_pose` alone does not command Nav2. `feature_nav_node` is intentionally **not**
launched — it stays available for offline experiments only.

## Control bridge (`usv_control_bridge/`)

`control_bridge.py` maps `/cmd_vel_smoothed` to differential thrust with a **drag-model
feed-forward and deliberately no velocity PID**:

```
F  = k_v_lin*vx + k_v_quad*vx*|vx|        (coefficients from the WAM-V URDF: x_u=51.3, x_uu=72.4)
dF = k_yaw * wz * yaw_sign
left = F - dF,  right = F + dF            → /wamv/thrusters/{left,right}/thrust
```

The EKF twist diverges to ±60 m/s while the boat physically moves <1 m/s, so feeding it to an
inner velocity loop drove the thrusters full-scale every cycle. RPP already closes the outer
loop on position. If the boat circles the wrong way the bug is upstream in RPP/odometry sign —
`yaw_sign` is the one safe inverter here.

The bridge also owns an **independent forward-corridor emergency stop** driven by
`/usv/safety_cloud` (see `safety_*` params in `config/bridge_params.yaml`), separate from Nav2's
costmap so a planner fault cannot remove collision protection.

## Other packages

Real robot:

- **`unitree_lidar_ros2/`** — SDK wrapper publishing `/unilidar/cloud` + `/unilidar/imu`; links
  the prebuilt `unitree_lidar_sdk/lib/<arch>/libunitree_lidar_sdk.a`.
- **`unitree_lidar_sdk/`** — vendor SDK v1.0.10. Wire protocol is **MAVLink** over UDP
  (`include/mavlink/`, custom `SysMavlink` dialect).
- **`my_mapping_launcher/`** — pure-launch orchestration. `all_in_one.launch.py` staggers startup
  (t=0 driver + NMEA GPS, t=2 Point-LIO, t=4 UTM fusion + RViz). `start_mapping.launch.py` adds
  the `aft_mapped→body` and `body→gps_link` static TFs plus `navsat_transform_node`
  (`config/navsat_params.yaml`: `use_odometry_yaw: true` to trust LIO heading over the
  magnetometer, `zero_altitude: true` to trust LiDAR Z over GPS altitude).
  `PCD/pcd_to_utm.py` converts output PCD to UTM/LAS using a hardcoded georeferencing transform.
- **`my_gps_driver/`** — `/gps/fix` → smoothed, accuracy-filtered `/gps/path` + LINE_STRIP marker.
- **`lidar_camera_fusion/`** — projects LiDAR into the camera image; `/colored_cloud` (per-frame)
  and `/colored_map` (accumulated in `camera_init`, TRANSIENT_LOCAL). Embeds a 90° baseline
  LiDAR→camera rotation; `roll/pitch/yaw` params are degrees on top of it and are live-tunable
  via `rqt_reconfigure` (a change clears and rebuilds the map). **Extrinsics are not fully
  calibrated — color drift is expected.**
- **`visual_loop_closure/`** — ORB keyframe DB + SE(3) Gauss-Newton/LM pose graph; loop
  constraints are identity. Publishes `/optimized_path`, `/loop_status`, `/loop_markers`.
- **`camera_tools/`**, **`OrbbecSDK_ROS2/`** — Astra Pro Plus depth reader (Y11 2-byte LE) and
  the vendor camera driver.
- **`Tools/`** — Jetson-side serial agent that runs ROS 2 actions from a PC serial assistant.

Sim:

- **`usv_localization/`** — optional GPS/EKF chain (`ekf.yaml`, `navsat.yaml`,
  `covariance_injector`), disabled by default.
- **`usv_perception/`** — offline Python camera perception: buoy detector (HSV + LiDAR range
  gating), cloud colorizer, water filter.
- **`usv_click_gazebo/`** — Qt/C++ Gazebo GUI plugin, `/gazebo/click/point` → `/goal_pose`.
- **`vrx/`** — 5 upstream packages: `vrx_gz`, `vrx_ros`, and `vrx_urdf/{wamv_description,
  wamv_gazebo,vrx_gazebo}`.
- **`usv_navigation`** extra nodes: `costmap_3d_markers` (lifts lethal local-costmap cells into
  3D boxes for RViz; visualization only), `clock_bridge_watchdog` (restarts the Gazebo→ROS
  `/clock` bridge if it stalls), `nav_data_logger` (fixed-cadence CSV of pose/goal/commanded vs
  achieved velocity/cross-track error/all cloud point counts — the MATLAB comparison input).

## Experiments, data and reports

Not scratch directories — they are the project's evidence base and its only change history.

- `experiments/` — trial runners and offline analysis: `run_static_obstacle_trial.sh`,
  `run_dynamic_ab_trial.sh`, `compare_navigation_trials.py` (`static` / `dynamic` subcommands),
  `record_nav_path.py`, `nav_trial_analysis.py`, `compute_ekf_ab_metrics.py`.
- `analysis/`, `report*/matlab/` — MATLAB plotting scripts.
- `data/`, `report*/data/` — raw CSVs. **Filename suffixes are load-bearing:** only `_valid`
  runs are usable for conclusions; everything else is retained for audit (disk exhaustion,
  bad `yaw_sign`, ROS param syntax errors, goal-coordinate contamination). Check the summary
  table's `validity` / `notes` columns before citing a number.
- `report/` LIO + obstacle-filter improvements → `report2/` EKF vs Point-LIO A/B →
  `report3/` current post-fix navigation results (`README.md` has the full experiment runbook,
  `USV_SYSTEM_TECHNICAL_DOCUMENTATION.md` the system write-up). Reports are written in Chinese;
  match that language when adding to them.
- `USV_NAV_TECH_PLAN.md`, `MIGRATION_NOTES.md` — the navigation technical plan and the
  vrx_ws merge notes. `AGENTS.md` holds the repository style guide.

Prefer running a matching `experiments/` runner over inventing a new measurement path, so results
stay comparable with the recorded baselines.

## Environment constraints

**This machine has no internet.** Gazebo must resolve worlds/models locally. A
"Fuel world download failed" error means `GZ_SIM_RESOURCE_PATH` is wrong — rebuild the vrx
packages (the install hooks regenerate it) rather than patching env vars by hand. The old
`vrx_ws` had pre-migration absolute paths baked into its hooks, which is exactly this failure.

**GPU rendering must use the NVIDIA dGPU.** `sim.sh`, `view.sh` and `usv.sh` export
`__NV_PRIME_RENDER_OFFLOAD=1` and `__GLX_VENDOR_LIBRARY_NAME=nvidia`. On the iGPU, `gpu_lidar`
drops to ~3 Hz and RViz's costmap rendering hits a GLSL shader crash (black map/cloud).
`GZ_SIM_RESOURCE_PATH` and `GZ_GUI_PLUGIN_PATH` are set by the launch scripts, not by hand.

Writes to `~/.ros/log` may be denied in this environment; the scripts redirect launch logs to
`.usv_logs/` accordingly. Treat `build/`, `install/`, `log/`, `Log/`, `.usv_logs/`, `.usv_pids`,
and `.manual_logs/` as generated output.

## Prerequisites

`ros-jazzy-pcl-ros`, `ros-jazzy-pcl-conversions`, `ros-jazzy-visualization-msgs`,
`libeigen3-dev`, `robot_localization`, `nmea_navsat_driver`, `livox_ros_driver2` (needed for its
custom messages even with Unitree LiDARs), and `unilidar_sdk` / `unilidar_sdk2` for L1 / L2.

## Critical configuration notes

1. **IMU parameters are hardware-specific.** `satu_acc`, `satu_gyro`, `acc_norm` must match the
   physical IMU; wrong values break the ESKF. The VRX config sets saturation to 100.0 because a
   simulated IMU never saturates.
2. **Per-point timestamps are mandatory.** `Failed to find match for field 'time'` means the
   PointCloud2 lacks them and Point-LIO cannot run. `time_lag_imu_to_lidar` compensates clock
   offset (0.0 in sim — sensors share the Gazebo clock).
3. **`extrinsic_est_en: false`** whenever extrinsics are known; enable only for online calibration.
4. **`pcd_save_en: true`** accumulates scans to `PCD/scans.pcd` on graceful shutdown;
   `interval: -1` keeps everything in one file.
5. Put new parameters in the owning package's `config/`, new launch files in `launch/`, lowercase
   descriptive names (`navigation.launch.py`). Python is 4-space PEP 8 `snake_case`; C++ follows
   each package's existing standard and brace style.

