# LiDAR-Inertial USV Workspace

A ROS 2 **Jazzy** workspace for autonomous surface vehicles, built around
[Point-LIO](https://github.com/hku-mars/Point-LIO) LiDAR-inertial odometry. One
algorithm core drives two deployments: a **real boat** on the water, and an
autonomous **WAM-V USV in VRX/Gazebo simulation**.

The project's focus is the gap between the two — making a per-point LiDAR-inertial
odometry stack that was designed for ground robots behave on a featureless water
surface, and turning its output into something Nav2 can actually plan on.

---

## Overview

```
                     ┌──────────────────────────────┐
  real robot         │  Unitree Unilidar L1/L2      │
  (or Livox /        │  + IMU                       │
   Velodyne /        └──────────────┬───────────────┘
   Ouster / Hesai)                  │
                                    ▼
                          ┌─────────────────────┐
                          │     Point-LIO       │  per-point IKFoM ESKF
                          │  /aft_mapped_to_init│  + ZUPT (this fork)
                          └─────────┬───────────┘
                                    │
                     ┌──────────────┴──────────────┐
                     ▼                             ▼
          ┌────────────────────┐        ┌────────────────────┐
          │  mapping / PCD     │        │  USV cloud filter  │
          │  GPS + UTM fusion  │        │  Nav2 navigation   │
          │  RGB colorization  │        │  differential      │
          │  loop closure      │        │  thrust control    │
          └────────────────────┘        └────────────────────┘
                   (real robot)                 (sim + real)
```

The two chains share `point_lio_ros2/` but diverge above it. Full details — frame
trees, topic topology, the reasoning behind each tuned constant — are in
[`CLAUDE.md`](CLAUDE.md).

---

## Repository layout

| Path | What it is |
|------|-----------|
| `src/point_lio_ros2/` | Point-LIO core. Main node `laserMapping.cpp`, ESKF in `Estimator.cpp`, per-vendor deskew in `preprocess.cpp`. |
| `src/unitree_lidar_ros2/`, `src/unitree_lidar_sdk/` | Unitree LiDAR driver + vendor SDK (MAVLink over UDP). |
| `src/my_mapping_launcher/` | Launch orchestration for the real robot, incl. GPS/UTM fusion. |
| `src/lidar_timestamp_adapter/` | `cloud_filter` (NaN + water-sheet reject) and the live diagnostic scripts in `test/`. |
| `src/usv_cloud_filter/` | Two-path obstacle filter feeding Nav2 costmaps, safety stop, and the structure map. |
| `src/usv_navigation/` | Nav2 config, BT, `click_to_goal` gateway, dynamic obstacle tracker, `costmap_3d_markers`. |
| `src/usv_control_bridge/` | Nav2 velocity → differential thrust, with drag-model feed-forward and an independent emergency stop. |
| `src/vrx/` | VRX simulation packages (vendored upstream). |
| `src/OrbbecSDK_ROS2/`, `src/lidar_camera_fusion/` | Camera driver + LiDAR↔camera projection. |
| `src/visual_loop_closure/` | ORB keyframe DB + SE(3) pose graph. |
| `experiments/`, `analysis/` | Trial runners and offline analysis scripts. |
| `report/` … `report4/` | The project's recorded results and change history. See below. |

---

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon build --packages-select usv_navigation     # focused rebuild
colcon test  --packages-select usv_navigation     # pytest + ament linters
```

`usv_navigation` is the only package with real unit tests (pure algorithm
functions, no ROS graph):

```bash
python3 -m pytest src/usv_navigation/test/test_dynamic_obstacle_tracker.py -k grid_clusters -v
```

Everything else is validated by live diagnostic scripts rather than unit tests —
`src/lidar_timestamp_adapter/test/verify_*.py` / `measure_*.py` / `diag_*.py`
cover topics, TF, timing lag, stationary LIO drift, the costmap pipeline and
water rejection. `./test_nav2.sh` is a Nav2 dry-run that needs no sim or LIO.

---

## Running the simulation

```bash
./usv.sh start      # full chain, in dependency order
./usv.sh status     # stop | restart also available
```

Logs land in `.usv_logs/`, PIDs in `.usv_pids`. The chain is started by seven
scripts, each of which also runs standalone:

| # | Script | Starts |
|---|--------|--------|
| 1 | `./sim.sh [world]` | VRX Gazebo (default `sydney_regatta`) + clock watchdog |
| 2 | `./lio.sh [norviz]` | `cloud_filter` + Point-LIO + LIO RViz |
| 3 | — | water-surface cloud filter, after a TF check |
| 4 | `./nav.sh` | Nav2 + `costmap_3d_markers` + `click_to_goal` |
| 5 | `./bridge.sh` | `usv_control_bridge` (Nav2 velocity → differential thrust) |
| 6 | — | `usv_click_gazebo` (click in Gazebo → Nav2 goal) |
| 7 | `./view.sh [2d]` | 3D navigation RViz |

```bash
./goto.sh 20 10 [yaw]   # send a Nav2 goal; 'here' prints pose, 'cancel' aborts
./boat.sh fwd 15        # manual driving (fwd/back/left/right/stop)
USV_DYNAMIC_TRACKER=true ./usv.sh start   # enable the dynamic obstacle tracker
```

GPS anchoring (`./gps.sh`) is deliberately **not** part of `usv.sh`: VRX's simulated
GPS jumps and makes the boat fly in RViz. `./ekf.sh` is likewise separate and must
be cold-started after `sim.sh` and `lio.sh`, never restarted mid-run.

Between experiment runs, always `./usv.sh stop && ros2 daemon stop` and confirm with
`ps` that every node is gone — stale launch parents create duplicate TF and control
writers.

## Running the real robot

```bash
ros2 launch my_mapping_launcher mapping_no_gps.launch.py   # driver → Point-LIO → RViz
ros2 launch my_mapping_launcher all_in_one.launch.py       # + GPS/UTM fusion
ros2 launch my_mapping_launcher with_camera.launch.py      # + camera
ros2 launch point_lio mapping_unilidar_l1.launch.py        # Point-LIO only
./demo.sh                                                  # interactive menu
```

---

## What this fork adds to upstream Point-LIO

- **ZUPT** — an IMU stationarity detector plus zero-velocity update (~77 references
  in `laserMapping.cpp`, configured by the `zupt_*` keys in `config/vrx_wamv.yaml`).
  It exists because a stationary WAM-V drifted on featureless water. ZUPT constrains
  velocity, not absolute position, which is why `zupt_position_hold_enable` stays
  `false`.
- **VRX support** — `config/vrx_wamv.yaml` + `launch/mapping_vrx.launch.py`, which
  start the Gazebo cloud filter, the mapping node, and the static
  `aft_mapped → wamv/wamv/base_link` TF that connects the tree before Nav2 starts.
- PCD saving is flushed **only on graceful SIGINT** — setting `pcd_save_en` at
  runtime via `ros2 param set` will not trigger a write.

A few non-obvious properties worth knowing before changing anything:

- Odometry is published on **`/aft_mapped_to_init`**, not `/Odometry`.
- `camera_init` **is** Nav2's global frame. Goals, costmaps and `/cloud_registered`
  are all in it, which is why `goto.sh` and RViz "Publish Point" need no conversion.
- The obstacle filter has two independent input paths and four outputs. The raw
  Gazebo scan feeds the costmaps and safety stop (avoidance does not depend on LIO
  registration); the registered cloud feeds a structure map that accumulates then
  **freezes**, so the tracker subtracts a genuine static map rather than the current
  scan from itself.
- Height cropping happens *after* TF into base_link, so `z≈0` is the local water
  surface even when `camera_init` drifts vertically.
- The WAM-V **cannot reverse or rotate in place**. That single fact drives most of
  `nav2_params.yaml`: `allow_reversing: false`, `use_rotate_to_heading: false`,
  Dubins motion model, `backup` removed from the behavior plugins.
- Several constants are the result of recorded A/B experiments. Changing one
  silently invalidates a report — check `report*/` before tuning.

---

## Experiments, data and reports

`experiments/`, `data/`, `analysis/` and `report*/` are **not scratch directories** —
they are the project's evidence base and its only change history.

- `experiments/run_static_obstacle_trial.sh`, `run_dynamic_ab_trial.sh`,
  `compare_navigation_trials.py`, `record_nav_path.py`, `nav_trial_analysis.py`
- `data/`, `report*/data/` — raw CSVs. **Filename suffixes are load-bearing:** only
  `_valid` runs are usable for conclusions; the rest are retained for audit. Check
  the summary table's `validity` / `notes` columns before citing a number.
- `report/` LIO + obstacle-filter work → `report2/` EKF-vs-Point-LIO A/B →
  `report3/` post-fix static navigation trials — its `README.md` holds the full
  experiment runbook and `USV_SYSTEM_TECHNICAL_DOCUMENTATION.md` the system
  write-up → `report4/` sudden-obstacle trials, where the obstacle appears only
  after the boat is already under way, so what is recorded is the real
  detect → replan → avoid transient rather than a pre-computed detour.

The reports are written in Chinese.

Prefer running a matching `experiments/` runner over inventing a new measurement
path, so results stay comparable with the recorded baselines.

---

## Requirements

ROS 2 Jazzy on Ubuntu 24.04, plus `ros-jazzy-pcl-ros`,
`ros-jazzy-pcl-conversions`, `ros-jazzy-visualization-msgs`, `libeigen3-dev`,
`robot_localization`, `nmea_navsat_driver`, and `livox_ros_driver2` (needed for its
custom messages even when using Unitree LiDARs).

The simulation needs a working Gazebo install with the worlds resolving locally, and
**GPU rendering should use the NVIDIA dGPU** — on an iGPU, `gpu_lidar` drops to ~3 Hz
and RViz's costmap rendering hits a GLSL shader crash. `sim.sh` / `view.sh` / `usv.sh`
export the PRIME render-offload variables for this.

---

## Licensing

This repository is distributed under **GPL-2.0** (see [`LICENSE`](LICENSE)), because
it bundles a modified copy of Point-LIO. Vendored third-party packages keep their own
licenses:

| Component | License | Origin |
|-----------|---------|--------|
| `src/point_lio_ros2/` | GPL-2.0 | fork of `dfloreaa/point_lio_ros2` (upstream: HKU-MARS Point-LIO) |
| `src/vrx/` | Apache-2.0 | fork of [osrf/vrx](https://github.com/osrf/vrx) |
| `src/OrbbecSDK_ROS2/` | Apache-2.0 | fork of [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) |
| `src/my_mapping_launcher/`, `src/my_gps_driver/` | Apache-2.0 | this project |
| `src/unitree_lidar_sdk/`, `src/unitree_lidar_ros2/` | vendor SDK | Unitree |
| remaining `src/usv_*`, `src/lidar_*`, `src/*_tools` | GPL-2.0 | this project |

The three vendored forks are committed as **plain source with their upstream `.git`
removed**, so a single clone is buildable. Their modifications and base upstream
commits are documented in the license table and in `MIGRATION_NOTES.md`.

> **Not included in this repository:** the large point-cloud products, which exceed
> GitHub's 100 MB per-file limit — `my_mapping_launcher/PCD/` (2.0 GB of `.pcd`/`.las`),
> `point_lio_ros2/PCD/scans.pcd` (103 MB), `Log/zupt_metrics.csv` (236 MB), and the
> four upstream Point-LIO README gifs (216 MB). See [`.gitignore`](.gitignore).
> Experiment data under `report*/data/` and `data/` is retained in full.
