# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# Author: Ryan Shim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def launch_setup(context, *args, **kwargs):
    # Obtener valor real en runtime
    scene_num = context.launch_configurations['scene_num']

    base_path = os.path.join(
        get_package_share_directory('turtlebot3_coppeliasim'),
        'scenes',
        f'turtlebot3_{TURTLEBOT3_MODEL}_ROS2_dqn'
    )

    # Mapeo de escenas
    suffix_map = {
        '1': '',
        '2': '_static_obs',
        '3': '_moving_obs',
        '4': '_map'
    }

    suffix = suffix_map.get(scene_num, '')

    scene_file = f"{base_path}{suffix}.ttt"

    return [
        LogInfo(msg=f"Scene selected: {scene_num}"),
        LogInfo(msg=f"Loading scene: {scene_file}"),

        ExecuteProcess(
            cmd=[f'coppeliaSim.sh -f {scene_file} -s 0'],
            shell=True
        )
    ]

def generate_launch_description():

    return LaunchDescription([
        LogInfo(msg=['Execute Turtlebot3 CoppeliaSim!!']),
            
        LogInfo(msg='Execute Turtlebot3 CoppeliaSim!!'),
            
        DeclareLaunchArgument(
            'scene_num',
            default_value='1',
            description='Scene number (1 to 4)'
        ),
        
        OpaqueFunction(function=launch_setup)
       
    ])
