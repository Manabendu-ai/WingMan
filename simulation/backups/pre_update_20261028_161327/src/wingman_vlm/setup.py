from setuptools import find_packages, setup

package_name = 'wingman_vlm'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='archisman',
    maintainer_email='cavendish.rana@gmail.com',
    description='WingMan prompt-triggered VLM perception and mission decisions using local Ollama',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'vlm_node = wingman_vlm.vlm_node:main',
        'prompt_cli = wingman_vlm.prompt_cli:main',
        'mission_manager = wingman_vlm.mission_manager:main',
    ]},
)
