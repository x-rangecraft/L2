from pathlib import Path
from glob import glob
from setuptools import setup

package_name = "description"
root = Path(__file__).parent
share_dirs = ("config", "launch", "urdf", "meshes")

data_files = [
    ("share/ament_index/resource_index/packages", [str(root / "resource" / package_name)]),
    (f"share/{package_name}", ["package.xml", "description.md"]),
]

for directory in share_dirs:
    files = glob(f"{directory}/*")
    if files:
        data_files.append((f"share/{package_name}/{directory}", files))

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="jetson",
    maintainer_email="jetson@example.com",
    description="Python node that publishes the YAM manipulator URDF on /robot_description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_desc_node = description.robot_desc_node:main",
        ],
    },
)
