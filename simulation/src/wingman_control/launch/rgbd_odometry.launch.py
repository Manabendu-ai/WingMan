from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'frame_id': 'camera_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'approx_sync': True,
                'queue_size': 10,
                'wait_for_transform': 0.2,
                'Vis/MaxFeatures': '1500',
                'Vis/MinInliers': '10',
                'OdomF2M/MaxFeatures': '1000',
            }],
            remappings=[
                ('rgb/image', '/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/depth_camera'),
            ],
        ),
    ])
