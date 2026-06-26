from glob import glob

from setuptools import find_packages, setup


package_name = "ego_training_demo"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot-a",
    maintainer_email="robot-a@example.com",
    description="Six-stage EGO-Planner training demo package for Isaac Sim, Pegasus, and PX4.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cloud_bridge = ego_training_demo.cloud_bridge:main",
            "goal_sender = ego_training_demo.goal_sender:main",
            "position_cmd_monitor = ego_training_demo.position_cmd_monitor:main",
            "ego_px4_bridge = ego_training_demo.ego_px4_bridge:main",
        ]
    },
)
