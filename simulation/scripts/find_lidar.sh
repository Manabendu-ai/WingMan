#!/usr/bin/env bash
# Run with PX4 SITL + Gazebo started. Lists lidar/camera topics and their Gazebo message types.
gz topic -l | grep -i -E "lidar|scan|laser|points|depth|camera|image"
for t in $(gz topic -l | grep -i -E "scan|laser|lidar"); do
  echo "=== $t"; gz topic -i -t "$t" | head -6
done
echo "gz.msgs.LaserScan = 2D lidar (scan_kind:=scan planar:=true) | gz.msgs.PointCloudPacked = 3D (scan_kind:=cloud planar:=false)"
