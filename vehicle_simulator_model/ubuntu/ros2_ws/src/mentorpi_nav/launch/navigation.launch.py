from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map_yaml')
    nav_share = Path(get_package_share_directory('mentorpi_nav'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    slam_share = Path(get_package_share_directory('mentorpi_slam'))
    nav_params = str(nav_share / 'config' / 'nav2.yaml')
    slam_params = str(slam_share / 'config' / 'slam.yaml')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'localization_launch.py')),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav_params,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    mapping = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
        output='screen', parameters=[slam_params],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav_params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    goal_bridge = Node(
        package='mentorpi_nav', executable='goal_bridge.py', name='goal_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    velocity_relay = Node(
        package='mentorpi_nav', executable='cmd_vel_relay.py', name='cmd_vel_relay', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['localization', 'mapping']),
        DeclareLaunchArgument('map_yaml', default_value=''),
        localization,
        mapping,
        navigation,
        goal_bridge,
        velocity_relay,
    ])
