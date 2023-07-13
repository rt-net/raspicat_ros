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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    raspicat_bringup_dir = get_package_share_directory('raspicat_bringup')

    urg_interface = LaunchConfiguration('urg_interface')
    params_file = LaunchConfiguration('params_file')

    declare_urg_interface = DeclareLaunchArgument(
        'urg_interface',
        default_value='serial',
        description='urg_interface: supported: serial, ethernet')
    declare_params_file = DeclareLaunchArgument('params_file', default_value=[
        TextSubstitution(text=os.path.join(
            raspicat_bringup_dir, 'config', '')),
        'urg_', urg_interface, TextSubstitution(text='.param.yaml')])

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        output='screen',
        parameters=[params_file]
    )

    ld = LaunchDescription()

    ld.add_action(declare_urg_interface)
    ld.add_action(declare_params_file)

    ld.add_action(urg_node)

    return ld
