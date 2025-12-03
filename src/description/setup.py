from pathlib import Path
from glob import glob
import os
from setuptools import setup

package_name = "description"
root = Path(__file__).parent
share_dirs = ("config", "launch", "urdf", "meshes")

data_files = [
    ("share/ament_index/resource_index/packages", [str(root / "resource" / package_name)]),
    (f"share/{package_name}", ["package.xml", "description.md"]),
]

# 需要排除的目录名（构建产物目录）
excluded_dirs = {"__pycache__", "build", "install", ".git"}

for directory in share_dirs:
    files = glob(f"{directory}/*")
    # 过滤：只保留文件，排除目录和不需要的文件
    filtered_files = []
    for f in files:
        # 只包含文件，不包含目录
        if not os.path.isfile(f):
            continue
        # 排除 __pycache__ 和 .pyc 文件
        if "__pycache__" in f or f.endswith(".pyc"):
            continue
        # 排除构建产物目录（检查路径中是否包含这些目录名）
        path_parts = Path(f).parts
        if any(excluded in path_parts for excluded in excluded_dirs):
            continue
        filtered_files.append(f)
    
    if filtered_files:
        data_files.append((f"share/{package_name}/{directory}", filtered_files))

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
            "robot_desc_check = description.asset_check:main",
        ],
    },
)
