# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Jazzy workspace for LiDAR-inertial mapping, camera fusion, GPS localization, and USV simulation. Application and ROS packages live under `src/`, including:

- `point_lio_ros2/`: Point-LIO odometry and mapping core.
- `unitree_lidar_ros2/`, `unitree_lidar_sdk/`, and `OrbbecSDK_ROS2/`: sensor drivers and camera support.
- `my_mapping_launcher/`, `my_gps_driver/`, `lidar_camera_fusion/`, and `visual_loop_closure/`: real-robot mapping components.
- `usv_*` packages and `vrx/`: filtering, localization, Nav2 control, and Gazebo/VRX simulation.

Launch files, YAML parameters, RViz layouts, URDFs, worlds, and meshes stay with their owning package. Root scripts (`sim.sh`, `lio.sh`, `nav.sh`, `usv.sh`, etc.) provide repeatable workflows. Treat `build/`, `install/`, `log/`, `.usv_logs/`, and `.usv_pids` as generated/runtime output; do not edit or commit them.

## Build, Test, and Development Commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Use `colcon build --packages-select <package>` for a focused rebuild. For simulation, run `./sim.sh`, `./lio.sh`, `./nav.sh`, and `./bridge.sh` in order, or use `./usv.sh start`; stop with `./usv.sh stop`. The real-robot smoke path is `ros2 launch my_mapping_launcher mapping_no_gps.launch.py`.

## Coding Style & Naming Conventions

Use four-space indentation for Python and spaces rather than tabs throughout. Follow PEP 8, descriptive `snake_case` Python names, and ROS package naming conventions. C++ uses the standard configured by each package (commonly C++14 or C++17); keep existing brace and naming style when modifying legacy code. Put new parameters in the package’s `config/` directory and new launch files in `launch/`, using lowercase descriptive filenames such as `navigation.launch.py`.

## Testing Guidelines

Run `colcon test` after changes and inspect failures with `colcon test-result --verbose`. Packages declare `ament_lint`, `ament_flake8`, `ament_pep257`, and/or `pytest` checks; add focused pytest tests where practical. Diagnostic scripts in `src/lidar_timestamp_adapter/test/` are useful for validating topics, transforms, timing, and navigation behavior. No repository-wide coverage threshold is currently documented.

## Commit & Pull Request Guidelines

This checkout has no usable Git history, so no existing commit convention can be verified. Use short, imperative subjects (for example, `Fix VRX point-cloud remap`) and keep unrelated changes separate. Pull requests should explain the affected ROS nodes/topics or launch parameters, list validation commands and results, link an issue when applicable, and include RViz/Gazebo screenshots or recordings for visualization and simulation changes.
