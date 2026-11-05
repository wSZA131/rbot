"""
rlai_search/launch/search.launch.py

Starts the search stack:
  1. target_mapper   — detections + depth + TF -> map poses and RViz markers
  2. search_manager  — FindObject action server sequencing Nav2 and observation

The detector (rlai_object_detection) and Nav2 are launched separately.

Usage:
  ros2 launch rlai_search search.launch.py
  ros2 launch rlai_search search.launch.py search_points_file:=/path/points.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_points = PathJoinSubstitution(
        [FindPackageShare('rlai_search'), 'config', 'search_points.yaml']
    )

    declared_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'search_points_file',
            default_value=default_points,
            description='Search points YAML with the regions to visit',
        ),
        DeclareLaunchArgument('min_confidence', default_value='0.4'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    search_points_file = LaunchConfiguration('search_points_file')

    target_mapper = Node(
        package='rlai_search',
        executable='target_mapper',
        name='target_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
        }],
    )

    search_manager = Node(
        package='rlai_search',
        executable='search_manager',
        name='search_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_confidence': LaunchConfiguration('min_confidence'),
            'search_points_file': search_points_file,
        }],
    )

    return LaunchDescription(declared_args + [target_mapper, search_manager])
