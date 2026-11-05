"""
rlai_object_detection/launch/detector.launch.py

Launches the open-vocabulary detector.

Usage:
  ros2 launch rlai_object_detection detector.launch.py model_path:=/path/yoloe-11s-seg.pt
  ros2 launch rlai_object_detection detector.launch.py device:=cpu imgsz:=480
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rlai_object_detection')
    config_file = PathJoinSubstitution([pkg, 'config', 'detector.yaml'])

    declared_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Path to the YOLOE checkpoint; detection is disabled when empty',
        ),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('confidence', default_value='0.35'),
    ]

    detector = Node(
        package='rlai_object_detection',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'imgsz': LaunchConfiguration('imgsz'),
                'confidence': LaunchConfiguration('confidence'),
            },
        ],
    )

    return LaunchDescription(declared_args + [detector])
