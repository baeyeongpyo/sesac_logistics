"""LaserScan 필터 노드만 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_input', default_value='/scan_raw'),
        DeclareLaunchArgument('scan_output', default_value='/scan_filtered'),
        DeclareLaunchArgument('first_end_index', default_value='30'),
        DeclareLaunchArgument('second_start_index', default_value='470'),
        Node(
            package='mentorpi_scan_filter',
            executable='scan_filter',
            name='mentorpi_scan_filter',
            output='screen',
            parameters=[{
                'first_end_index': LaunchConfiguration('first_end_index'),
                'second_start_index': LaunchConfiguration('second_start_index'),
            }],
            remappings=[
                ('scan_raw', LaunchConfiguration('scan_input')),
                ('scan_filtered', LaunchConfiguration('scan_output')),
            ],
        ),
    ])
