# Copyright (c) 2021 Geekplus Inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # config = os.path.join(
    #     get_package_share_directory('start_autocar'),
    #     'config',
    #     'test.yaml'
    # )    # 默认改参数也需要编译，如果改完参数不想编译，可用以下
    
    config = os.getcwd() + "/src/start_autocar/config/"+ 'test.yaml'
    ld = LaunchDescription()

    hmi = Node(
        package='hmi',
        name='hmi',
        executable='hmi',
        output='screen',
    )
    ld.add_action(hmi)
    
    gps = Node(
        package='gps',
        name='gps',
        executable='gps',
        output='screen',
    )
    ld.add_action(gps)   

    car_ori = Node(
        package='car_ori',
        name='car_ori',
        executable='car_ori',
        output='screen',
    )
    ld.add_action(car_ori)   
    
    planner = Node(
            package='planner',
            name='planner',
            executable='planner',
            output='screen',
            parameters=[config]
    )
    ld.add_action(planner)    

    car_control = Node(
        package='car_control',
        name='car_control',
        executable='car_control',
        output='screen',
    )
    ld.add_action(car_control)     




    return ld
