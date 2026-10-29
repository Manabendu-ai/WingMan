#!/usr/bin/env bash
set -e

PX4_DIR="$HOME/PX4-Autopilot"
WINGMAN_DIR="$HOME/ws_wingman"

source /opt/ros/humble/setup.bash

# Make WingMan assets and PX4 models discoverable by Gazebo.
export GZ_SIM_RESOURCE_PATH="$WINGMAN_DIR/simulation/models:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

cd "$PX4_DIR"

export PX4_GZ_WORLD="maze_exploration"
export PX4_GZ_MODEL_POSE="0,-17.2,0.3,0,0,1.5708"

make px4_sitl gz_x500_gps_denied
