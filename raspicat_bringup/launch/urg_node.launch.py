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
    
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'urg_interface',
            default_value='serial',
            description='urg_interface: supported: serial, ethernet'),
        
        DeclareLaunchArgument('params_file', default_value=[
            TextSubstitution(text=os.path.join(
                raspicat_bringup_dir, 'config','')),
            'urg_', urg_interface, TextSubstitution(text='.param.yaml')]),
    ])
    
    urg_node = Node(
        package='urg_node', 
        executable='urg_node_driver', 
        output='screen',
        parameters=[params_file]
    )

    launch_description.add_action(urg_node)
    return launch_description
