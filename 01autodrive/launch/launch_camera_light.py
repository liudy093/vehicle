import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autodrive_root = os.environ.get("AUTODRIVE_ROOT", "/home/nvidia/AutoDrive")
    use_trajectory = LaunchConfiguration('use_trajectory', default=False)
    default_map_path = os.path.join(autodrive_root, "datas", "maps", "map_tju8.json")
    file_path = LaunchConfiguration('file_path', default=default_map_path)


    return LaunchDescription([
        DeclareLaunchArgument('use_trajectory', default_value=use_trajectory),
        DeclareLaunchArgument('file_path', default_value=file_path),
        Node(
            package='match_map',
            executable='match_map',
            name='match_map',
            parameters=[{
                'use_trajectory':use_trajectory,
                'file_path':file_path
            }]
        ),
        Node(
            package='light_recognition',
            executable='light_recognition',
            name='light_recognition'
        )
    ])
