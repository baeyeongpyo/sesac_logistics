"""Run MentorPi's standard bringup inside a per-vehicle ROS namespace.

Example:
    ros2 launch bringup wrap_bringup.launch.py robot_name:=robot_1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    """Include the existing bringup launch under the requested vehicle namespace."""
    robot_name = LaunchConfiguration('robot_name')
    bringup_launch = os.path.join(
        get_package_share_directory('bringup'),
        'launch',
        'bringup.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            description='Vehicle namespace, for example robot_1 or robot_2.',
        ),
        GroupAction([
            PushRosNamespace(robot_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
            ),
        ]),
    ])
