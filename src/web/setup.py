"""Web Interactive GUI 包配置"""

from setuptools import setup

package_name = "web_interactive_gui"
python_package = "web_interactive_gui_core"

setup(
    name=package_name,
    version="0.2.0",
    packages=[python_package],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "flask",
        "flask-cors",
        "flask-socketio",
    ],
    zip_safe=False,
    maintainer="L2 Team",
    maintainer_email="maintainer@example.com",
    description="Web interactive GUI for image streaming and object segmentation.",
    license="MIT",
    tests_require=["pytest"],
    package_data={
        python_package: [
            "templates/*.html",
            "static/*",
        ],
    },
    include_package_data=True,
    entry_points={
        "console_scripts": [
            "web_interactive_gui = web_interactive_gui_core.web_gui:main",
        ],
    },
)
