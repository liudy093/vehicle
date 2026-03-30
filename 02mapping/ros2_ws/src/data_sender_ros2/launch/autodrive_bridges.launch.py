from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gps_input_topic", default_value="/gps_data"),
            DeclareLaunchArgument("gps_output_topic", default_value="/gps"),
            DeclareLaunchArgument("imu_input_topic", default_value="/gps_data"),
            DeclareLaunchArgument("imu_output_topic", default_value="/imu"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("angles_in_degrees", default_value="true"),
            DeclareLaunchArgument("angular_velocity_in_degrees", default_value="true"),
            DeclareLaunchArgument("angular_velocity_axes", default_value="['x', 'y', 'z']"),
            DeclareLaunchArgument("angular_velocity_signs", default_value="[1.0, 1.0, 1.0]"),
            DeclareLaunchArgument("linear_acceleration_axes", default_value="['x', 'y', 'z']"),
            DeclareLaunchArgument("linear_acceleration_signs", default_value="[1.0, 1.0, 1.0]"),
            DeclareLaunchArgument("linear_acceleration_scale", default_value="9.8"),
            Node(
                package="data_sender_ros2",
                executable="gps_pose_bridge_node",
                name="gps_pose_bridge_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("gps_input_topic"),
                        "output_topic": LaunchConfiguration("gps_output_topic"),
                    }
                ],
            ),
            Node(
                package="data_sender_ros2",
                executable="gps_imu_bridge_node",
                name="gps_imu_bridge_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("imu_input_topic"),
                        "output_topic": LaunchConfiguration("imu_output_topic"),
                        "frame_id": LaunchConfiguration("imu_frame_id"),
                        "angles_in_degrees": LaunchConfiguration("angles_in_degrees"),
                        "angular_velocity_in_degrees": LaunchConfiguration(
                            "angular_velocity_in_degrees"
                        ),
                        "angular_velocity_axes": LaunchConfiguration("angular_velocity_axes"),
                        "angular_velocity_signs": LaunchConfiguration("angular_velocity_signs"),
                        "linear_acceleration_axes": LaunchConfiguration(
                            "linear_acceleration_axes"
                        ),
                        "linear_acceleration_signs": LaunchConfiguration(
                            "linear_acceleration_signs"
                        ),
                        "linear_acceleration_scale": LaunchConfiguration(
                            "linear_acceleration_scale"
                        ),
                    }
                ],
            ),
        ]
    )
