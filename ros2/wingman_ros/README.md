# WingMan ROS 2 bridge

This package is the ROS 2 Humble integration boundary for WingMan. It
subscribes to the interfaces currently confirmed by the simulator owner and
publishes JSON channels for a dashboard or `rosbridge_server`.

It intentionally does **not** publish flight commands. The EGO-Planner goal
topic and PX4 command/feedback interface are not finalized yet, so command
publication remains disabled until those interfaces are implemented and
tested.

## Confirmed inputs

| Parameter | Default topic | Type |
|---|---|---|
| `map_topic` | `/map` | `nav_msgs/msg/OccupancyGrid` |
| `odom_topic` | `/odom` | `nav_msgs/msg/Odometry` |
| `rgbd_topic` | `/odom_rgbd_image` | `rtabmap_msgs/msg/RGBDImage` |
| `local_position_topic` | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` |
| `vehicle_status_topic` | `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` |
| `battery_topic` | `/fmu/out/battery_status_v1` | `px4_msgs/msg/BatteryStatus` |
| `landing_topic` | `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` |

PX4 message packages are optional for startup. If they are unavailable, the
bridge keeps publishing map and odometry data and reports the missing
telemetry inputs in its health message.

## Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/wingman/dashboard_state` | `std_msgs/msg/String` | Combined map, telemetry, RGB-D metadata, and health |
| `/wingman/map_json` | `std_msgs/msg/String` | Occupancy grid snapshot |
| `/wingman/telemetry_json` | `std_msgs/msg/String` | Pose, velocity, PX4 state, and localization health |
| `/wingman/health` | `std_msgs/msg/String` | Staleness and missing-interface diagnostics |
| `/wingman/rgbd_info` | `std_msgs/msg/String` | Latest RGB-D encodings, dimensions, and frame IDs |

The JSON is deliberately versioned and contains timestamps so a website can
show stale-data warnings rather than displaying old telemetry as current.

## Build and run

From a sourced ROS 2 Humble shell:

```bash
cd ros2
colcon build --symlink-install
source install/setup.bash
ros2 run wingman_ros bridge
```

Verify the inputs:

```bash
ros2 topic list
ros2 topic type /map
ros2 topic type /odom
ros2 topic type /odom_rgbd_image
ros2 topic echo --once /map
ros2 topic echo --once /odom
```

Inspect the JSON channels:

```bash
ros2 topic echo /wingman/dashboard_state
ros2 topic echo /wingman/health
```

## Safety gate

This bridge is telemetry/perception integration only. Do not connect
autonomous commands until the following are supplied:

1. EGO-Planner goal topic and message type.
2. Goal frame and acceptance/reached semantics.
3. PX4 arm, takeoff, hover, return, and land command interfaces.
4. Command acknowledgement and fault feedback.
5. A tested manual override and geofence.
