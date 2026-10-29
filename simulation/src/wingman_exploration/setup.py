import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wingman_exploration'
setup(
    name=package_name, version='0.2.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='archisman', maintainer_email='cavendish.rana@gmail.com',
    description='Frontier exploration + mission autopilot for the WingMan maze drone', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mission_autopilot = wingman_exploration.mission_autopilot:main',
        'web_gateway = wingman_exploration.web_gateway:main',
        'explorer_sim = wingman_exploration.offline_sim:main',
        'frontier_explorer = wingman_exploration.frontier_explorer:main',
        'maze_planner_demo = wingman_exploration.maze_planner_demo:main',
        'maze_planner_node = wingman_exploration.maze_planner_node:main',
    ]},
)
