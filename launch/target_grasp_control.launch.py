# Copyright (c) 2022，Horizon Robotics.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'task_input',
            default_value='False',
            description='Is there any task input'),
        DeclareLaunchArgument(
            'fixed_rel_pos',
            default_value='True',
            description='Is the relative position fixed'),
        DeclareLaunchArgument(
            'target_type',
            default_value='red_ball',
            description='Target type'),
        DeclareLaunchArgument(
            'task_type',
            default_value='catch',
            description='Task type'),
        DeclareLaunchArgument(
            'platform_h',
            default_value='189',
            description='platform height'),
        Node(
            package='tonypi_obj_grasp_control',
            executable='tonypi_obj_grasp_control',
            output='screen',
            parameters=[
                {"task_input": LaunchConfiguration('task_input')},
                {"fixed_rel_pos": LaunchConfiguration('fixed_rel_pos')},
                {"target_type": LaunchConfiguration('target_type')},
                {"task_type": LaunchConfiguration('task_type')},
                {"platform_h": LaunchConfiguration('platform_h')},,
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])