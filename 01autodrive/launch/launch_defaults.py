from launch import LaunchDescription
from launch_ros.actions import Node

all_nodes = (
    'car_ori',
    'sonic_obstacle',
    'regulator',
    'gps',
    'imu',
    'fusion',
    'net_work',
)

def generate_launch_description():

    nodes_ = [Node(
        package=n_,
        executable=n_,
        name=n_
    ) for n_ in all_nodes]
    
    return LaunchDescription(nodes_)
