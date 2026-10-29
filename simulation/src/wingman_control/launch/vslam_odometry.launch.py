"""Stage 1: RGB-D visual odometry only. No mapping, Nav2, or PX4 control.

Expected input types:
  rgb_topic: sensor_msgs/msg/Image (RGB image)
  depth_topic: sensor_msgs/msg/Image (depth registered/aligned to RGB)
  camera_info_topic: sensor_msgs/msg/CameraInfo

A valid TF from base_frame to the camera's optical frame must already exist.
Do not launch alongside lidar_slam.launch.py or any other publisher of /odom.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/rgbd/image'),
        DeclareLaunchArgument('depth_topic', default_value='/rgbd/depth_image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/rgbd/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='vslam_odom'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': LaunchConfiguration('base_frame'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'publish_tf': True,
                'wait_for_transform': 0.5,
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'queue_size': 10,
                'Odom/Strategy': '0',
                'Vis/MinInliers': '12',
                'OdomF2M/MaxSize': '1000',
                'Odom/GuessMotion': 'true',
            }],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('odom', '/odom_vslam'),
                ('odom_info', '/vslam/odom_info'),
            ],
        ),
    ])
