# WINGMAN interfaces: VLM <-> Sim/EGO-Planner

STATUS: DRAFT. Items marked TBD need the sim owner to confirm.

## Conventions
- Frames: ROS standard. x forward, y left, z up. Goals are in the WORLD frame.
- Units: meters, seconds. Depth in meters (float32).
- RGB and depth must be ALIGNED (same field of view).

## Sim -> VLM (inputs)
| Data        | Topic (TBD)                      | Type                  | Rate   |
|-------------|----------------------------------|-----------------------|--------|
| RGB image   | /camera/color/image_raw          | sensor_msgs/Image     | >=5 Hz |
| Depth image | /camera/depth/image_raw          | sensor_msgs/Image     | >=5 Hz |
| Camera info | /camera/color/camera_info        | sensor_msgs/CameraInfo| once   |
| Drone pose  | /odom                            | nav_msgs/Odometry     | >=10Hz |

## VLM -> Sim (outputs)
| Data         | Topic                     | Type                    | Rate   |
|--------------|---------------------------|-------------------------|--------|
| Observation  | /wingman/vlm/observation  | std_msgs/String (JSON)  | ~1 Hz  |
| Goal         | /wingman/vlm/goal         | geometry_msgs/PoseStamped (world frame) | on change |
| Health       | /wingman/vlm/health       | std_msgs/String (JSON)  | 1 Hz   |

Goal input topic that EGO-Planner listens to: TBD

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

## Questions for sim owner (TBD)
1. ROS 1 or ROS 2? Ubuntu version?
2. Is the sim camera aligned RGB-D (OAK-D Lite model)?
3. Exact topic names and depth encoding (32FC1 meters?).
4. EGO-Planner goal topic and message type?
5. Please record frames from the maze (command below).

## Recording sample data (ROS 2)
ros2 bag record -o maze_sample /camera/color/image_raw \
  /camera/depth/image_raw /camera/color/camera_info /odom
Fly through the maze for 1-2 minutes, including junctions and dead ends.
