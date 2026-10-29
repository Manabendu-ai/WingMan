# WingMan custom maze planner: ROS visualization adapter

This adapter connects the pure Python global planner to a live `nav_msgs/OccupancyGrid` and the TF transform from the map frame to `base_link`.

## Safety properties

- It does **not** publish `/cmd_vel` or PX4 setpoints.
- It does **not** send Nav2 `NavigateToPose` actions.
- It does **not** call takeoff or landing services.
- It starts stopped and requires an explicit `Trigger` service call.
- Outputs are visualization/debug topics only. They are not flight commands.
- Unknown cells and uncertain/occupied cells are not traversable; only known-free cells can be used by A*.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ws_wingman
colcon build --packages-select wingman_exploration --symlink-install
source install/setup.bash
```

## Inspect live inputs before starting

Make sure Gazebo, ROS-Gazebo bridges, and the mapping/localization nodes are already running. Do not start takeoff for this test.

```bash
ros2 topic list | grep -E '^/rtabmap/map$|^/map$|^/odom$|^/tf$|^/tf_static$'
ros2 topic info /rtabmap/map -v
ros2 topic echo /rtabmap/map --once --field header
ros2 run tf2_ros tf2_echo map base_link
```

The defaults match the existing `frontier_explorer` map topic. If your live map uses `/map`, pass `map_topic:=/map` to the node. The node uses the incoming map's `header.frame_id` as the TF target frame; it falls back to `map` only if the header frame is empty.

## Run node (starts stopped, no movement)

```bash
ros2 run wingman_exploration maze_planner_node
```

Check outputs:

```bash
ros2 topic list | grep '^/wingman/planner_debug/'
ros2 topic echo /wingman/planner_debug/decision
ros2 topic echo /wingman/planner_debug/path --once
```

Start the planning state machine only after the map and TF are verified:

```bash
ros2 service call /wingman/custom_planner/start std_srvs/srv/Trigger '{}'
```

Pause its route calculation:

```bash
ros2 service call /wingman/custom_planner/stop std_srvs/srv/Trigger '{}'
```

**Pause only stops this planner's publications; it does not brake, land, or otherwise command the aircraft.**

## RViz

Add a `Path` display on `/wingman/planner_debug/path` and a `Marker` display on `/wingman/planner_debug/goal_marker`. Select the same fixed frame as the occupancy grid, normally `map`.

## Current limitation

The node currently visualizes the global route. It does not dispatch route waypoints to any flight controller. Before any closed-loop navigation work, validate actual map semantics, frame alignment, odometry freshness, trajectory clearance and a separate safety/controller interface in simulation.
