from launch import LaunchDescription
from launch_ros.actions import Node

all_nodes = (
    'camera_ori',
)

def generate_launch_description():

    nodes_ = [Node(
        package=n_,
        node_executable=n_,
        name=n_
    ) for n_ in all_nodes]
    
    return LaunchDescription(nodes_)
