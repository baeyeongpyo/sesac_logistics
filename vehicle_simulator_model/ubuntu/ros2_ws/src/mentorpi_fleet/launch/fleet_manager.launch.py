from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('registry'),
        Node(
            package='mentorpi_fleet',
            executable='fleet_manager.py',
            name='fleet_manager',
            arguments=['--registry', LaunchConfiguration('registry')],
        ),
    ])
