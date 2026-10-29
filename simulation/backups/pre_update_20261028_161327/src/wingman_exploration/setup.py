from setuptools import find_packages, setup

package_name = 'wingman_exploration'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='archisman', maintainer_email='cavendish.rana@gmail.com',
    description='VLM-guided custom A* maze exploration planner', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'frontier_explorer = wingman_exploration.frontier_explorer:main',
        'maze_planner_demo = wingman_exploration.maze_planner_demo:main',
        'maze_planner_node = wingman_exploration.maze_planner_node:main',
    ]},
)
