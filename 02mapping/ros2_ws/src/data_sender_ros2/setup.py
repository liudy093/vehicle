from setuptools import setup


package_name = "data_sender_ros2"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/sender.launch.py", "launch/autodrive_bridges.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="ROS2 sender for RoboSense point clouds.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sender_node = data_sender_ros2.sender_node:main",
            "mock_cloud_server = data_sender_ros2.mock_cloud_server:main",
            "gps_pose_bridge_node = data_sender_ros2.gps_pose_bridge_node:main",
            "gps_imu_bridge_node = data_sender_ros2.gps_imu_bridge_node:main",
            "cgi410_raw_imu_bridge_node = data_sender_ros2.cgi410_raw_imu_bridge_node:main",
            "cgi410_serial_bridge_node = data_sender_ros2.cgi410_serial_bridge_node:main",
        ],
    },
)
