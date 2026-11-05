"""
rlai_bringup/launch/search_demo.launch.py

Full autonomous-search bringup: simulation + AMCL/localisation + Nav2 +
open-vocabulary detector + search manager + LiDAR safety monitor.

Compared with simulation.launch.py this file additionally starts Nav2 and the
search stack, and it requires a saved map because AMCL needs one.

Usage:
  ros2 launch rlai_bringup search_demo.launch.py \
    map_yaml_file:=$HOME/projects/rbot-main/maps/small_warehouse.yaml \
    model_path:=$HOME/models/yoloe-11s-seg.pt

  ros2 launch rlai_bringup search_demo.launch.py headless:=true \
    map_yaml_file:=... model_path:=... device:=cpu

Then send a task:
  ros2 action send_goal /find_object rlai_interfaces/action/FindObject \
    "{target_label: backpack}" --feedback
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _validate_map(context):
    map_file = LaunchConfiguration('map_yaml_file').perform(context)
    if not map_file:
        raise RuntimeError(
            'map_yaml_file is required: build a map first, then pass the saved YAML'
        )
    if not os.path.isfile(map_file):
        raise RuntimeError(f'map file not found: {map_file}')
    return []


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('world', default_value='warehouse_search'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='',
            description='Saved map used by AMCL and the Nav2 static layer (required)',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='YOLOE checkpoint; detection stays disabled when empty',
        ),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('min_confidence', default_value='0.4'),
        DeclareLaunchArgument(
            'search_points_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('rlai_search'), 'config', 'search_points.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'nav_delay',
            default_value='12.0',
            description='Seconds to wait for AMCL before starting Nav2',
        ),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rlai_bringup'), 'launch', 'simulation.launch.py']
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'localization_enabled': 'true',
            'use_amcl': 'true',
            'map_yaml_file': LaunchConfiguration('map_yaml_file'),
            # The search stack consumes RGB + depth only; skip the extra
            # depth/stereo point-cloud processing to save CPU.
            'camera_processing_enabled': 'false',
        }.items(),
    )

    safety = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('rlai_safety'), 'launch', 'safety_monitor.launch.py']
                    )
                ),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ],
    )

    detector = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('rlai_object_detection'),
                            'launch',
                            'detector.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'model_path': LaunchConfiguration('model_path'),
                    'device': LaunchConfiguration('device'),
                }.items(),
            )
        ],
    )

    navigation = TimerAction(
        period=LaunchConfiguration('nav_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('rlai_navigation'), 'launch', 'navigation.launch.py']
                    )
                ),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ],
    )

    search = TimerAction(
        period=16.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('rlai_search'), 'launch', 'search.launch.py']
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'search_points_file': LaunchConfiguration('search_points_file'),
                    'min_confidence': LaunchConfiguration('min_confidence'),
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        declared_args + [
            OpaqueFunction(function=_validate_map),
            simulation,
            safety,
            detector,
            navigation,
            search,
        ]
    )
