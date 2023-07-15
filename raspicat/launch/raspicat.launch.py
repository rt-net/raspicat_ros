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
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory(
        'raspicat_bringup') + '/launch/')

    use_urg = LaunchConfiguration('use_urg')
    urg_interface = LaunchConfiguration('urg')

    declare_use_urg = DeclareLaunchArgument(
        'use_urg',
        default_value='True',
        description='Whether or not to launch urg_node')
    declare_urg_interface = DeclareLaunchArgument(
        'urg',
        default_value='serial',
        description='urg_interface: supported: serial, ethernet')

    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            launch_dir, 'urg.launch.py')), condition=IfCondition(use_urg),
        launch_arguments={'urg_interface': urg_interface}.items()
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            launch_dir, 'robot_state_publisher.launch.py')),
    )

    mouse_node = LifecycleNode(
        namespace='',
        name='raspimouse',
        package='raspimouse', executable='raspimouse', output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'raspicat'), 'config', 'raspicat.param.yaml')]
    )

    emit_configuring_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(mouse_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_activating_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(mouse_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    emit_shutdown_event = EmitEvent(
        event=Shutdown()
    )

    register_activating_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mouse_node,
            goal_state='inactive',
            entities=[
                emit_activating_event
            ],
        )
    )

    register_shutting_down_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mouse_node,
            goal_state='finalized',
            entities=[
                emit_shutdown_event
            ],
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_urg)
    ld.add_action(declare_urg_interface)

    ld.add_action(urg_launch)
    ld.add_action(robot_state_publisher_launch)

    ld.add_action(mouse_node)
    ld.add_action(register_activating_transition)
    ld.add_action(register_shutting_down_transition)
    ld.add_action(emit_configuring_event)

    return ld
