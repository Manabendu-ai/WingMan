"""Everything the friend's website talks to. Start it after mission.launch.py (or before, order does not matter).

  ros2 launch wingman_exploration website.launch.py
  ros2 launch wingman_exploration website.launch.py camera_bridge:=false     # if you already run the camera bridge

Website side (this laptop's IP, e.g. 192.168.128.220):
  rosbridge (roslibjs)   ws://<ip>:9090      commands in, telemetry / map out
  camera video (MJPEG)   http://<ip>:8080/stream?topic=<CAMERA_TOPIC>&type=mjpeg&quality=60
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

CAM = "/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_bridge", default_value="true"),
        DeclareLaunchArgument("rosbridge", default_value="true"),      # false if you already run rosbridge_websocket
        DeclareLaunchArgument("video", default_value="true"),          # false if you already run web_video_server
        DeclareLaunchArgument("port", default_value="9090"),
        DeclareLaunchArgument("video_port", default_value="8080"),
        # Gazebo camera -> ROS (same topic name the VLM node uses; skip with camera_bridge:=false if already running)
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="camera_bridge", output="screen",
             condition=IfCondition(LC("camera_bridge")), arguments=[CAM + "@sensor_msgs/msg/Image[gz.msgs.Image"]),
        # websocket for the website (commands in, telemetry / map / path out)
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket",
             output="screen", condition=IfCondition(LC("rosbridge")), parameters=[{"port": LC("port"), "address": "", "use_compression": False,
                                           "send_action_goals_in_new_thread": True}]),
        # MJPEG camera streams for <img src=...>
        Node(package="web_video_server", executable="web_video_server", name="web_video_server", output="screen",
             condition=IfCondition(LC("video")),
             parameters=[{"port": LC("video_port"), "address": "0.0.0.0"}]),
        # validates commands from the website's LLM and publishes one compact telemetry topic
        Node(package="wingman_exploration", executable="web_gateway", name="web_gateway", output="screen",
             parameters=[{"use_sim_time": False, "telemetry_hz": 2.0}])   # wall clock: telemetry keeps flowing if the sim pauses,
    ])
