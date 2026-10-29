"""Stage 2: RTAB-Map RGB-D mapping; requires validated /odom_vslam first.

Launch this only after vslam_odometry.launch.py produces valid, changing poses.
This node consumes the same RGB/depth/camera_info streams and /odom_vslam; it
must not start another odometry producer.
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
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('database_path', default_value='/tmp/wingman_rtabmap_vslam.db'),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': LaunchConfiguration('base_frame'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'map_frame_id': LaunchConfiguration('map_frame'),
                'publish_tf': True,
                'subscribe_rgb': True,
                'subscribe_depth': True,
                'subscribe_scan': False,
                'subscribe_scan_cloud': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'queue_size': 10,
                'database_path': LaunchConfiguration('database_path'),
                'Grid/FromDepth': 'true',
                'Grid/RangeMax': '8.0',
                'Grid/CellSize': '0.05',
                'Grid/3D': 'false',
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Reg/Strategy': '0',
                'Mem/IncrementalMemory': 'true',
            }],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('odom', '/odom_vslam'),
            ],
            arguments=['-d'],
        ),
    ])
