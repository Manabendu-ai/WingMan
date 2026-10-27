# WINGMAN interfaces: VLM <-> ROS 2 / PX4 / EGO-Planner

STATUS: ROS 2 Humble integration baseline. Items marked TBD still block
autonomous command publication.

## Conventions
- Frames: ROS standard. x forward, y left, z up. Goals are in the WORLD frame.
- Units: meters, seconds. Depth in meters (float32).
- RGB and depth must be ALIGNED (same field of view).

## Sim -> VLM (inputs)
| Data        | Topic (TBD)                      | Type                  | Rate   |
|-------------|----------------------------------|-----------------------|--------|
| Map         | /map                             | nav_msgs/OccupancyGrid | on change |
| Drone pose  | /odom                            | nav_msgs/Odometry     | >=10 Hz |
| RGB-D data  | /odom_rgbd_image                 | rtabmap_msgs/RGBDImage | >=5 Hz |
| Odom info   | /odom_info                       | rtabmap_msgs/OdomInfo | diagnostics |
| LiDAR scan  | /scan                            | sensor_msgs/LaserScan | >=5 Hz |
| TF          | /tf                              | tf2_msgs/TFMessage | as published |
| PX4 position| /fmu/out/vehicle_local_position_v1 | px4_msgs/VehicleLocalPosition | >=5 Hz |
| PX4 status  | /fmu/out/vehicle_status_v1       | px4_msgs/VehicleStatus | >=5 Hz |
| Battery     | /fmu/out/battery_status_v1       | px4_msgs/BatteryStatus | >=1 Hz |
| Landing     | /fmu/out/vehicle_land_detected   | px4_msgs/VehicleLandDetected | verify |

## VLM -> Sim (outputs)
| Data         | Topic                     | Type                    | Rate   |
|--------------|---------------------------|-------------------------|--------|
| Dashboard state | /wingman/dashboard_state | std_msgs/String (JSON) | 2 Hz |
| Map JSON     | /wingman/map_json         | std_msgs/String (JSON) | 2 Hz |
| Telemetry JSON | /wingman/telemetry_json | std_msgs/String (JSON) | 2 Hz |
| RGB-D info   | /wingman/rgbd_info        | std_msgs/String (JSON) | 2 Hz |
| Health       | /wingman/health           | std_msgs/String (JSON) | 2 Hz |
| Observation  | /wingman/vlm/observation  | std_msgs/String (JSON)  | TBD |
| Goal         | TBD                       | TBD | blocked |

Goal input topic and PX4 mission-command interfaces: TBD. The ROS bridge
does not publish autonomous flight commands until these are finalized.

## Task input
- /wingman/task (std_msgs/String), e.g. "find the exit". Default: "find the exit".

## Observation JSON
{"scene": str, "target_visible": bool,
 "target_x": int|null (0-1000), "target_y": int|null (0-1000),
 "recommended_direction": "forward|left|right|back|none",
 "confidence": 0.0-1.0, "uncertainties": [str],
 "suggested_next_step": "continue|search_again|approach_target|uncertain"}

## Rules
- The VLM only SUGGESTS. A goal is published only if confidence >= 0.6 and a
  valid depth reading exists at the target pixel.
- If the VLM is slow/offline, exploration continues without it.

## Remaining integration decisions
1. RGB-D image encodings, depth units, and camera-info topic.
2. TF frame names and the authoritative world frame.
3. EGO-Planner goal topic/message and goal-reached feedback.
4. PX4 arm/takeoff/hover/return/land command interfaces.
5. Authentication and authorization for remote commands.

## Recording sample data (ROS 2)
ros2 bag record -o maze_sample /map /odom /odom_rgbd_image \
  /odom_info /scan /tf \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_status_v1 /fmu/out/battery_status_v1
Fly through the maze for 1-2 minutes, including junctions and dead ends.
