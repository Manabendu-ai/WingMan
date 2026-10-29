#!/usr/bin/env bash
# usage: scripts/bridge_lidar.sh <gz_lidar_topic> <scan|cloud>   (start ONE bridge only)
GZ_TOPIC="$1"; KIND="${2:-scan}"
[ -z "$GZ_TOPIC" ] && { echo "usage: $0 <gz_lidar_topic> <scan|cloud>"; exit 1; }
if [ "$KIND" = "scan" ]; then TYPE='sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
else TYPE='sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'; fi
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  "${GZ_TOPIC}@${TYPE}" \
  --ros-args -r "${GZ_TOPIC}:=/scan"
