WingMan ROS planner adapter patch

Contains:
- maze_planner_node.py: ROS OccupancyGrid + TF adapter that publishes route/goal visualization only
- setup.py entrypoint
- package.xml runtime dependencies
- docs/CUSTOM_MAZE_PLANNER_ROS.md

Apply from your home directory so archive paths merge into ~/ws_wingman:
  cd ~
  unzip -o ~/Downloads/wingman_live_map_planner_patch.zip

Then:
  source /opt/ros/humble/setup.bash
  cd ~/ws_wingman
  colcon build --packages-select wingman_exploration --symlink-install
  source install/setup.bash
  ros2 run wingman_exploration maze_planner_node

The node starts stopped and does not send flight commands. Its route is published
on /wingman/planner_debug/path, not /cmd_vel or any PX4 topic. Start planning only
with the explicit /wingman/custom_planner/start Trigger service after verifying
that the map and map->base_link TF are valid.
