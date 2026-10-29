from setuptools import find_packages, setup

package_name = 'wingman_autonomy'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='archisman', maintainer_email='cavendish.rana@gmail.com',
    description='Gated VLM-guided mission-to-path-following integration for WingMan',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'path_follower = wingman_autonomy.path_follower:main',
    ]},
)
