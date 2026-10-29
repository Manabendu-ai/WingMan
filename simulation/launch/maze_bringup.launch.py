"""Localization + mapping stack (LiDAR ICP odometry + RTAB-Map + RViz + vio_to_px4).
Start AFTER PX4+Gazebo, the XRCE agent and scripts/bridge_lidar.sh.
  ros2 launch ~/ws_wingman/launch/maze_bringup.launch.py with_nav:=false lidar_frame:=link lidar_xyz:="0 0 0.075"
ALWAYS use with_nav:=false with the mission autopilot: with_nav:=true also starts Nav2, frontier_explorer and
cmdvel_to_px4, which would fight the autopilot for /cmd_vel and offboard control."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    with_nav = LaunchConfiguration('with_nav')

    def node(pkg, exe, delay, cond=None):
        return TimerAction(period=delay, actions=[Node(
            package=pkg, executable=exe, output='screen', parameters=[{'use_sim_time': True}],
            condition=cond)])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'lidar_slam.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in
                          ['scan_kind', 'planar', 'lidar_frame', 'lidar_xyz', 'rviz']}.items())
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'nav2_stack.launch.py')),
        condition=IfCondition(with_nav))

    return LaunchDescription([
        DeclareLaunchArgument('scan_kind', default_value='scan'),
        DeclareLaunchArgument('planar', default_value='true'),
        DeclareLaunchArgument('lidar_frame', default_value='link'),
        DeclareLaunchArgument('lidar_xyz', default_value='0 0 0.075'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('with_nav', default_value='false'),
        slam,
        node('wingman_control', 'vio_to_px4', 5.0),
        TimerAction(period=10.0, actions=[nav2]),
        node('wingman_control', 'cmdvel_to_px4', 12.0, IfCondition(with_nav)),
        node('wingman_exploration', 'frontier_explorer', 12.0, IfCondition(with_nav)),
    ])