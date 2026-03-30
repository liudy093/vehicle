from launch import LaunchDescription
from launch_ros.actions import Node

all_nodes = ()

def generate_launch_description():

    nodes_ = [Node(
        package=n_,
        node_executable=n_,
        name=n_
    ) for n_ in all_nodes]
    
    nodes_.append(Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen'
        ))
    
    
    return LaunchDescription(nodes_)
