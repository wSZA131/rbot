"""
rlai_safety/launch/safety_monitor.launch.py

Launches the forward-sector safety monitor.

Usage:
  ros2 launch rlai_safety safety_monitor.launch.py
  ros2 launch rlai_safety safety_monitor.launch.py safety_distance:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('rlai_safety'), 'config', 'safety_monitor.yaml']
    )

    declared_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('safety_distance', default_value='0.35'),
        DeclareLaunchArgument('half_angle_deg', default_value='30.0'),
    ]

    monitor = Node(
        package='rlai_safety',
        executable='safety_monitor_node',
        name='safety_monitor',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'safety_distance': LaunchConfiguration('safety_distance'),
                'half_angle_deg': LaunchConfiguration('half_angle_deg'),
            },
        ],
    )

    return LaunchDescription(declared_args + [monitor])
