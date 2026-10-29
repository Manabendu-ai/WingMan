#!/usr/bin/env bash
# Run after bridge_lidar.sh + launch. Prints the facts that decide whether localization works.
source /opt/ros/humble/setup.bash; source ~/ws_ros2/install/setup.bash 2>/dev/null; source ~/ws_wingman/install/setup.bash 2>/dev/null
echo "== /scan: frame, angles, number of beams (must be > 100, NOT 1)"
ros2 topic echo --once /scan --field header.frame_id
ros2 topic echo --once /scan --field angle_min; ros2 topic echo --once /scan --field angle_max
ros2 topic echo --once /scan --field ranges | tr ',' '\n' | wc -l
echo "== /scan rate"; timeout 4 ros2 topic hz /scan | tail -2
echo "== /odom rate + covariance (diag[0] must be small, not 9999)"
timeout 4 ros2 topic hz /odom | tail -2
ros2 topic echo --once /odom --field pose.covariance | head -3
echo "== odometry quality"; ros2 topic echo --once /odom_info --field lost 2>/dev/null
echo "== who publishes TF odom->base_link (must be ONE: icp_odometry)"; ros2 topic info /tf -v | grep -E "Node name|Publisher count"
echo "== PX4 side"; ros2 topic list | grep -E "vehicle_local_position|vehicle_visual_odometry"
echo "xy_valid / z_valid:"; ros2 topic echo --once /fmu/out/vehicle_local_position_v1 --field xy_valid 2>/dev/null
ros2 topic echo --once /fmu/out/vehicle_local_position_v1 --field z_valid 2>/dev/null
