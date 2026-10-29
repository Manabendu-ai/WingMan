"""Nav2 navigation servers (no map_server/AMCL: RTAB-Map provides map->odom)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    nav2 = get_package_share_directory('nav2_bringup')

    # Path to your custom nav2_params.yaml
    params_file = os.path.join(here, '..', 'config', 'nav2', 'nav2_params.yaml')

    return LaunchDescription([
        # Remap default /cmd_vel from Nav2 servers (controller & behavior server) to /cmd_vel_nav
        # This guarantees only velocity_smoother publishes to /cmd_vel
        SetRemap(src='/cmd_vel', dst='cmd_vel_nav'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': 'true',
            }.items()
        ),
    ])