#!/usr/bin/env bash
# Bridge the forward RGB camera in the WingMan maze Gazebo model to ROS 2.
set -euo pipefail
source /opt/ros/humble/setup.bash
IMAGE='/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image'
INFO='/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/camera_info'
echo "Bridging camera image to ${IMAGE}"
echo "Camera topic must exist in Gazebo before starting this bridge."
exec ros2 run ros_gz_bridge parameter_bridge \
  "${IMAGE}@sensor_msgs/msg/Image[gz.msgs.Image" \
  "${INFO}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
