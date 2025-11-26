from glob import glob
from pathlib import Path

from setuptools import setup

package_name = 'web_interactive_gui'

share_dir = Path('web_interactive_gui')
templates_dir = share_dir / package_name / 'templates'
static_dir = share_dir / package_name / 'static'

def glob_files(path_pattern):
    return glob(str(path_pattern)) if path_pattern.exists() else []

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/templates', glob_files(templates_dir / '*')),
    ('share/' + package_name + '/static', glob_files(static_dir / '*')),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'flask',
        'flask-cors',
        'flask-socketio',
    ],
    zip_safe=False,
    maintainer='L2 Team',
    maintainer_email='maintainer@example.com',
    description='Reusable web interface node for image streaming and click interactions.',
    license='MIT',
    tests_require=['pytest'],
    package_data={
        package_name: [
            'templates/*.html',
            'static/*',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_interactive_gui = web_interactive_gui.node:main',
        ],
    },
)
