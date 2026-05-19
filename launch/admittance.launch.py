"""
Launch the admittance controller and MAE sensor driver.

Starts mae_sensor_node first (sensor must stream before controller reads),
then admittance_node.

Usage:
  ros2 launch admittance_controller admittance.launch.py
  ros2 launch admittance_controller admittance.launch.py robot_ip:=192.168.1.20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('admittance_controller')
    default_params = os.path.join(pkg_dir, 'config', 'admittance_params.yaml')

    return LaunchDescription([
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

        # MAE sensor driver (Python lifecycle node) — starts first
        Node(
            package='mae_sensor_driver',
            executable='mae_sensor_node',
            name='mae_sensor_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # Admittance controller (C++ node)
        Node(
            package='admittance_controller',
            executable='admittance_node',
            name='admittance_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'robot_ip': LaunchConfiguration('robot_ip')},
            ],
        ),
    ])
