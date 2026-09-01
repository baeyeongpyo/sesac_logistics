"""원본 MentorPi SLAM에 포크 각도가 제거된 LaserScan을 연결한다."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _topic(robot_name, suffix):
    name = robot_name.strip('/')
    return f'/{name}/{suffix}' if name else f'/{suffix}'


def _launch_setup(context):
    robot_name = LaunchConfiguration('robot_name').perform(context).strip('/')
    master_name = LaunchConfiguration('master_name').perform(context).strip('/')
    raw_topic = _topic(robot_name, 'scan_raw')
    filtered_topic = _topic(robot_name, 'scan_filtered')
    slam_share = get_package_share_directory('slam')

    scan_filter = Node(
        package='mentorpi_scan_filter',
        executable='scan_filter',
        namespace=robot_name,
        name='mentorpi_scan_filter',
        output='screen',
        parameters=[{
            'first_end_index': LaunchConfiguration('first_end_index'),
            'second_start_index': LaunchConfiguration('second_start_index'),
        }],
        remappings=[
            ('scan_raw', raw_topic),
            ('scan_filtered', filtered_topic),
        ],
    )

    original_slam = GroupAction(actions=[
        # LiDAR publisher는 raw를 유지하고 slam_toolbox 구독만 변경한다.
        SetRemap(src='slam_toolbox:scan_raw', dst=filtered_topic),
        SetRemap(src='slam_toolbox:/scan_raw', dst=filtered_topic),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'slam.launch.py')),
            launch_arguments={
                'robot_name': robot_name or '/',
                'master_name': master_name or '/',
                'sim': LaunchConfiguration('sim'),
                'enable_save': LaunchConfiguration('enable_save'),
                'slam_method': 'slam_toolbox',
            }.items(),
        ),
    ])
    return [scan_filter, original_slam]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value=os.environ.get('HOST', '/')),
        DeclareLaunchArgument(
            'master_name', default_value=os.environ.get('MASTER', '/')),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('enable_save', default_value='false'),
        DeclareLaunchArgument('first_end_index', default_value='30'),
        DeclareLaunchArgument('second_start_index', default_value='470'),
        OpaqueFunction(function=_launch_setup),
    ])
