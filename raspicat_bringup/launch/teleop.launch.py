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

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    teleop = LaunchConfiguration('teleop', default='key')
    joy_config_filepath = LaunchConfiguration(
        'joy_config_filepath')
    joy_dev = LaunchConfiguration('joy_dev')
    output_vel = LaunchConfiguration('output_vel')

    joy = GroupAction(
        condition=LaunchConfigurationEquals('teleop', 'joy'),
        actions=[
            DeclareLaunchArgument('teleop', default_value='joy'),
            DeclareLaunchArgument('joy_config_filepath', default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('raspicat_bringup'), 'config', '')),
                'joy', TextSubstitution(text='.param.yaml')]),
            DeclareLaunchArgument(
                'joy_dev', default_value='/dev/input/js0'),
            DeclareLaunchArgument(
                'output_vel', default_value='cmd_vel'),

            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                parameters=[{
                    'dev': joy_dev,
                    'deadzone': 0.05,
                    'autorepeat_rate': 20.0,
                }]),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[joy_config_filepath],
                remappings={
                    ('/cmd_vel', str(teleop.perform(LaunchContext())) + '_vel')},
            ),
            Node(
                package='raspicat',
                executable='velocity_smoother_controller',
                name='velocity_smoother_controller_node',
                parameters=[joy_config_filepath],
                remappings={
                    ('/input_vel',  str(teleop.perform(LaunchContext())) + '_vel'),
                    ('/output_vel', output_vel)},
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother_node',
                parameters=[joy_config_filepath],
                remappings={
                    ('/cmd_vel', 'control_vel')},
            ),
        ]
    )

    key = GroupAction(
        condition=LaunchConfigurationEquals('teleop', 'key'),
        actions=[
            DeclareLaunchArgument('teleop', default_value='key'),
            DeclareLaunchArgument(
                'output_vel', default_value='cmd_vel'),
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_key_node',
                output='screen',
                prefix='xterm -e',
                remappings={
                    ('/cmd_vel',  str(teleop.perform(LaunchContext())) + '_vel')},
            ),
            Node(
                package='raspicat',
                executable='velocity_smoother_controller',
                name='velocity_smoother_controller_node',
                remappings={
                    ('/input_vel',  str(teleop.perform(LaunchContext())) + '_vel'),
                    ('/output_vel', output_vel)},
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother_node',
                remappings={
                    ('/cmd_vel', 'control_vel')},
            ),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(joy)
    ld.add_action(key)

    return ld
