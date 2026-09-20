#!/bin/bash
# Nav2 navigation stack launcher
# Usage: ./nav.sh
# Requires: VRX sim running, Point-LIO running

source install/setup.bash

ros2 launch usv_navigation navigation.launch.py use_sim_time:=true
