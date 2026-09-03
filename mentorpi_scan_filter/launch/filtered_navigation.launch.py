"""원본 Navigation의 AMCL·costmap에 필터 LaserScan을 연결한다."""

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


def _scan_remaps(node_name, filtered_topic):
    """상대·절대 scan_raw 설정을 모두 필터 토픽으로 연결한다."""
    return [
        SetRemap(src=f'{node_name}:scan_raw', dst=filtered_topic),
        SetRemap(src=f'{node_name}:/scan_raw', dst=filtered_topic),
    ]


def _launch_setup(context):
    robot_name = LaunchConfiguration('robot_name').perform(context).strip('/')
    master_name = LaunchConfiguration('master_name').perform(context).strip('/')
    raw_topic = _topic(robot_name, 'scan_raw')
    filtered_topic = _topic(robot_name, 'scan_filtered')
    navigation_share = get_package_share_directory('navigation')

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

    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'output_frame': 'depth_scan_frame',
            'scan_height': 11,
        }],
        remappings=[
            ('depth', '/ascamera/camera_publisher/depth0/image_raw'),
            ('depth_camera_info', '/ascamera/camera_publisher/depth0/camera_info'),
            ('scan', '/depth_scan'),
        ],
    )

    depth_scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_scan_static_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '1.5708',
            '--pitch', '-1.5708',
            '--roll', '0',
            '--frame-id', 'ascamera_color_0',
            '--child-frame-id', 'depth_scan_frame',
        ],
    )

    remap_actions = []
    # AMCL과 local/global costmap에서 원시 스캔을 사용하지 않게 한다.
    for node_name in ('amcl', 'controller_server', 'planner_server'):
        remap_actions.extend(_scan_remaps(node_name, filtered_topic))

    filtered_navigation = GroupAction(actions=[
        *remap_actions,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    navigation_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'robot_name': robot_name or '/',
                'master_name': master_name or '/',
                'sim': LaunchConfiguration('sim'),
                'map': LaunchConfiguration('map'),
                'use_teb': LaunchConfiguration('use_teb'),
            }.items(),
        ),
    ])
    return [scan_filter, depthimage_to_laserscan, depth_scan_tf, filtered_navigation]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value=os.environ.get('HOST', '/')),
        DeclareLaunchArgument(
            'master_name', default_value=os.environ.get('MASTER', '/')),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('map', default_value='map_01'),
        DeclareLaunchArgument('use_teb', default_value='false'),
        DeclareLaunchArgument('first_end_index', default_value='30'),
        DeclareLaunchArgument('second_start_index', default_value='470'),
        OpaqueFunction(function=_launch_setup),
    ])