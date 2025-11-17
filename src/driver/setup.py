#!/usr/bin/env python3
from setuptools import setup, find_packages

package_name = 'robot_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_driver']),
        ('share/' + package_name, ['package.xml', 'robot_description.yaml']),
        ('share/' + package_name + '/config', ['config/robot_driver_config.yaml', 'config/safe_pose_default.yaml']),
        ('share/' + package_name + '/launch', ['launch/robot_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='L2 Driver Team',
    maintainer_email='support@example.com',
    description='ROS 2 driver node for robotic actuators with CAN buses and diagnostic tooling.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver_node = driver.robot_driver_node:main',
        ],
    },
)
