#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source ~/ws_wingman/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

LOGDIR="$HOME/ws_wingman/logs"
mkdir -p "$LOGDIR"

start_node() {
    local name="$1"
    shift

    if [[ -f "$LOGDIR/$name.pid" ]] &&
       kill -0 "$(cat "$LOGDIR/$name.pid")" 2>/dev/null; then
        echo "$name is already running; skipping."
        return
    fi

    echo "Starting $name ..."
    nohup "$@" > "$LOGDIR/$name.log" 2>&1 &
    echo $! > "$LOGDIR/$name.pid"
    sleep 2
}

start_node vslam_odometry \
    ros2 launch wingman_control vslam_odometry.launch.py \
    use_sim_time:=true

start_node cmdvel_to_px4 \
    ros2 run wingman_control cmdvel_to_px4 --ros-args \
    -p use_sim_time:=false \
    -p allow_takeoff:=false \
    -p odom_topic:=/odom \
    -p local_position_topic:=/fmu/out/vehicle_local_position_v1 \
    -p target_alt:=1.0 \
    -p climb_rate:=0.2 \
    -p vmax:=0.22 \
    -p max_yaw_rate:=0.28

echo
echo "Startup commands issued."
echo "Logs are in: $LOGDIR"
echo "Takeoff remains disabled."
echo
tail -n 20 "$LOGDIR/vslam_odometry.log" || true
tail -n 20 "$LOGDIR/cmdvel_to_px4.log" || true
