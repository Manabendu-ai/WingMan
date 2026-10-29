#!/usr/bin/env bash
# Read-only input audit. Source the ROS/workspace environments before running.
set +e
RGB_TOPIC="${1:-/rgbd/image}"
DEPTH_TOPIC="${2:-/rgbd/depth_image}"
INFO_TOPIC="${3:-/rgbd/camera_info}"

printf '\n=== Available camera/depth/odometry topics ===\n'
ros2 topic list -t | grep -Ei 'image|camera_info|depth|point|odom|imu|rgbd' || true

for topic in "$RGB_TOPIC" "$DEPTH_TOPIC" "$INFO_TOPIC"; do
  printf '\n=== %s ===\n' "$topic"
  ros2 topic type "$topic" 2>&1
  ros2 topic info "$topic" -v 2>&1
 done

printf '\n=== Approximate frequencies (wait 3 seconds per stream, Ctrl+C if needed) ===\n'
for topic in "$RGB_TOPIC" "$DEPTH_TOPIC" "$INFO_TOPIC"; do
  echo "--- $topic"
  timeout 3s ros2 topic hz "$topic" 2>&1 || true
done

printf '\n=== Required conditions before launching RGB-D odometry ===\n'
echo '1. RGB and depth topics must both be sensor_msgs/msg/Image.'
echo '2. Camera info must be sensor_msgs/msg/CameraInfo.'
echo '3. Depth must be registered/aligned to the RGB image and use the correct scale.'
echo '4. Image stamps should be close enough for synchronization.'
echo '5. Image/camera frame IDs must be connected to base_link by a verified TF.'
echo 'If the simulator publishes only PointCloud2 and no depth Image, this launch is not ready; do not remap PointCloud2 as a depth Image.'
