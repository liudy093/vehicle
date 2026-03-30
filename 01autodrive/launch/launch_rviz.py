from launch import LaunchDescription
from launch_ros.actions import Node
# import numpy
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    # rviz_config='/home/self_driving/tji_auto_drive/src/lidar_obstacle/rviz2.rviz'
    rviz_config = os.getcwd() + '/src/lidar_obstacle/rviz2.rviz'
    
    
    nodes_ = []
    nodes_.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rviz_config]
        ))


    return LaunchDescription(nodes_)
