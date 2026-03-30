from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_id", default_value="vehicle2"),
            DeclareLaunchArgument("cloud_server_url", default_value="http://127.0.0.1:5000"),
            DeclareLaunchArgument("cloud_topic", default_value="/forward/rslidar_points"),
            DeclareLaunchArgument(
                "cloud_topics",
                default_value="/forward/rslidar_points,/left/rslidar_points,/right/rslidar_points",
            ),
            DeclareLaunchArgument("gps_topic", default_value="/gps"),
            DeclareLaunchArgument("send_interval_sec", default_value="1.0"),
            DeclareLaunchArgument("fastlio_cloud_topic", default_value=""),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("fastlio_send_interval_sec", default_value="0.5"),
            Node(
                package="data_sender_ros2",
                executable="sender_node",
                name="sender_node",
                output="screen",
                parameters=[
                    {
                        "vehicle_id": LaunchConfiguration("vehicle_id"),
                        "cloud_server_url": LaunchConfiguration("cloud_server_url"),
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "cloud_topics": LaunchConfiguration("cloud_topics"),
                        "gps_topic": LaunchConfiguration("gps_topic"),
                        "send_interval_sec": LaunchConfiguration("send_interval_sec"),
                        "fastlio_cloud_topic": LaunchConfiguration("fastlio_cloud_topic"),
                        "fastlio_odom_topic": LaunchConfiguration("fastlio_odom_topic"),
                        "fastlio_send_interval_sec": LaunchConfiguration("fastlio_send_interval_sec"),
                    }
                ],
            ),
        ]
    )
