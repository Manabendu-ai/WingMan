# WingMan custom maze planner — first implementation

This patch adds a ROS-independent global planning core to `wingman_exploration`.
It is intentionally not wired to PX4, velocity setpoints, takeoff, or landing.

## Included

- `wingman_exploration/maze_planner.py`: four-connected A* over known-free cells,
  frontier detection/grouping, information-gain-aware target choice, active-goal
  retention, frontier bookkeeping, and return-to-entrance decisions.
- `wingman_exploration/maze_planner_demo.py`: deterministic offline demonstration.
- `test/test_maze_planner.py`: route, wall, unreachable, frontier, return, and
  unsafe-current-cell tests.
- `maze_planner_demo` console entry point in `setup.py`.

## Grid assumptions

The planner consumes a row-major occupancy grid using ROS map values: `-1` is
unknown, `0` is free, and any positive value is treated as occupied/uncertain.
Only known-free cells are traversable. Cells are `(x, y)` = `(column, row)`.
The current and entrance cells must be known-free before a route is generated.

## Build and test in the existing workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/ws_wingman
colcon build --packages-select wingman_exploration --symlink-install
source install/setup.bash
python3 -m pytest -q src/wingman_exploration/test/test_maze_planner.py
ros2 run wingman_exploration maze_planner_demo
```

The first two commands build only the exploration package. The demo runs purely
on a sample grid; it does not connect to Gazebo and does not command the drone.

## Important limitations before live integration

- This is a grid-based global planner, not yet a 3D local trajectory planner.
- A real ROS adapter still needs to convert `nav_msgs/OccupancyGrid` data and a
  valid pose estimate into `GridMap` and `(x, y)` cells, with correct origin,
  resolution, and coordinate-frame handling.
- Frontier arrival is currently represented as reaching the selected grid cell.
  In live mapping, new map updates must be allowed to add newly exposed frontiers.
- No collision clearance, drone dynamics, vertical clearance, or PX4 interface is
  implemented by this module. Keep flight disabled while integrating and testing.
