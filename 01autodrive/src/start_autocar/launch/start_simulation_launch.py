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
    #     'simulation.yaml'
    # )    # 默认改参数也需要编译，如果改完参数不想编译，可用以下
    
    config = os.getcwd() + "/src/start_autocar/config/"+ 'simulation.yaml'
    ld = LaunchDescription()

    hmi = Node(
        package='paking_hmi',
        name='paking_hmi',
        executable='paking_hmi',
        output='screen',
    )

    ld.add_action(hmi)

    sim_model = Node(
        package='sim_model',
        name='sim_model',
        executable='sim_model',
        output='screen',
        parameters=[config]
    )

    ld.add_action(sim_model)    
    
    # planner = Node(
    #         package='planner',
    #         name='planner',
    #         executable='planner',
    #         output='screen',
    #         parameters=[config]
    # )

    # ld.add_action(planner)  

    # obca_planner = Node(
    #         package='obca_planner',
    #         name='obca_planner',
    #         executable='obca_planner',
    #         output='screen',
    #         parameters=[config]
    # )
   
    # ld.add_action(obca_planner)  

    parking_path_planning = Node(
            package='parking_path_planning',
            name='parking_path_planning',
            executable='parking_path_planning',
            output='screen',
            parameters=[config]
    )
   
    ld.add_action(parking_path_planning) 

    return ld
