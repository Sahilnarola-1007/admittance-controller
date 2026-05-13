"""
Launch the admittance controller node with parameters from YAML.

Usage:
  ros2 launch admittance_controller admittance.launch.py
  ros2 launch admittance_controller admittance.launch.py robot_ip:=192.168.1.20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('admittance_controller')
    default_params = os.path.join(pkg_dir, 'config', 'admittance_params.yaml')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.10',
            description='IP address of the Kinova Gen3'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the YAML parameter file'
        ),

        # The admittance controller node
        Node(
            package='admittance_controller',
            executable='admittance_node',
            name='admittance_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'robot_ip': LaunchConfiguration('robot_ip')},
            ],
            # Remap if needed:
            # remappings=[
            #     ('~/wrench_corrected', '/kinova/wrench_corrected'),
            # ],
        ),
    ])
