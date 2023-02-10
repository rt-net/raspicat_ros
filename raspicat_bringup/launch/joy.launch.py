# Copyright 2023 RT Corporation
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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, GroupAction

from launch_ros.actions import Node


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration(
        'config_filepath')

    joy = GroupAction(
        actions=[
            DeclareLaunchArgument(
                'joy_vel', default_value='cmd_vel'),
            DeclareLaunchArgument(
                'joy_dev', default_value='/dev/input/js0'),
            DeclareLaunchArgument('config_filepath', default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('raspicat_bringup'), 'config', '')),
                'joy', TextSubstitution(text='.config.yaml')]),

            Node(
                package='joy', executable='joy_node', name='joy_node',
                parameters=[{
                    'dev': joy_dev,
                    'deadzone': 0.05,
                    'autorepeat_rate': 20.0,
                }]),
            Node(
                package='teleop_twist_joy', executable='teleop_node',
                name='teleop_twist_joy_node', parameters=[config_filepath],
                remappings={
                    ('/cmd_vel', LaunchConfiguration('joy_vel'))},
            ),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(joy)

    return ld
