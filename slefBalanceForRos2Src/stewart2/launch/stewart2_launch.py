# Copyright (c) 2019 Intel Corporation
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
import sys
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    
    #world = launch.substitutions.LaunchConfiguration('world')

    return LaunchDescription([
        
        #launch.actions.DeclareLaunchArgument(
        #    'world',  default_value=os.path.join('/home/he/dev_ws/src/stewart2/world/     stewart.world'),description='Full path to world file to load'),
        #launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', world],output='screen')


           launch_ros.actions.Node(
            package='stewart2', node_executable='ImuAnalyse', output='screen',
            name='ImuAnalyse'),

            launch_ros.actions.Node(
            package='stewart2', node_executable='InverseKinematicsStewart2', output='screen',
            name='InverseKinematicsStewart2'),

            launch_ros.actions.Node(
            package='stewart2', node_executable='ForwardKinematicsStewart2', output='screen',
            name='ForwardKinematicsStewart2'),

    ])
