"""Flight stack on top of an already-running PX4 + XRCE agent + LiDAR bridge + maze_bringup (with_nav:=false).

  ros2 launch wingman_exploration mission.launch.py
  ros2 launch wingman_exploration mission.launch.py vlm:=false cruise_speed:=1.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

CAM = "/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image"


def generate_launch_description():
    sim = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("vlm", default_value="true"),
        DeclareLaunchArgument("model", default_value="gemma3:4b"),
        DeclareLaunchArgument("camera_topic", default_value=CAM),
        DeclareLaunchArgument("cruise_speed", default_value="0.8"),
        DeclareLaunchArgument("search_speed", default_value="0.6"),
        DeclareLaunchArgument("takeoff_alt", default_value="1.0"),
        Node(package="wingman_control", executable="cmdvel_to_px4", name="cmdvel_to_px4", output="screen",
             parameters=[sim, {"target_alt": LC("takeoff_alt"), "allow_takeoff": True, "vmax": 1.2,
                               "max_yaw_rate": 1.2, "climb_rate": 0.5}]),
        Node(package="wingman_exploration", executable="mission_autopilot", name="mission_autopilot", output="screen",
             parameters=[sim, {"cruise_speed": LC("cruise_speed"), "search_speed": LC("search_speed"),
                               "map_topic": "/map", "scan_topic": "/scan"}]),
        Node(package="wingman_vlm", executable="vlm_node", name="wingman_vlm", output="screen",
             condition=IfCondition(LC("vlm")),
             parameters=[{"model": LC("model"), "camera_topic": LC("camera_topic")}]),
    ])
