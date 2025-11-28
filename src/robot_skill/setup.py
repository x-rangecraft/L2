import os
from glob import glob
from setuptools import setup

package_name = 'robot_skill'
python_package = 'robot_skill_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=[python_package],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_skill']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot_skill_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/robot_skill.launch.py']),
        ('share/' + package_name + '/skill_sets', glob('skill_sets/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML', 'open3d', 'scipy'],
    zip_safe=True,
    maintainer='L2 Robotics',
    maintainer_email='robot@l2.local',
    description='Skill planning node for the YAM manipulator.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_skill_node = robot_skill_core.robot_skill:main',
        ],
    },
)
