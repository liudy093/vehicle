from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_trajectory = LaunchConfiguration('use_trajectory', default=False)
    # file_path = LaunchConfiguration('file_path', default='')
    file_path = LaunchConfiguration('file_path', default='/home/nvidia/AutoDrive/datas/maps/map_0716.json')
    #file_path = LaunchConfiguration('file_path', default='/home/nvidia/AutoDrive/datas/maps/lane_e_w.txt')


    lc_ = LaunchDescription([
        DeclareLaunchArgument('use_trajectory', default_value=use_trajectory),
        DeclareLaunchArgument('file_path', default_value=file_path),
        Node(
            package='global_path_planning',
            executable='global_path_planning',
            name='global_path_planning',
            parameters=[{
                'use_trajectory':use_trajectory,
                'file_path':file_path
            }]
        )
    ])
    return lc_
