import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wingman_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # INSTALL CONFIG FILES:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='archisman',
    maintainer_email='archisman@localhost',
    description='PX4 vision-odometry and Nav2 cmd_vel bridges',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_to_px4 = wingman_control.vio_to_px4:main',
            'cmdvel_to_px4 = wingman_control.cmdvel_to_px4:main',
            'vo_node = wingman_control.vo_node:main',
            'maze_navigator = wingman_control.maze_navigator:main',
        ],
    },
)