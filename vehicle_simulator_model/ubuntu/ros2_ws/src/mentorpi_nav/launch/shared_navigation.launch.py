from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def robot_stack(robot_id, params_file):
    configured = ParameterFile(RewrittenYaml(
        source_file=params_file,
        root_key=robot_id,
        param_rewrites={
            'use_sim_time': 'true',
            'base_frame_id': f'{robot_id}/base_footprint',
            'odom_frame_id': f'{robot_id}/odom',
            'robot_base_frame': f'{robot_id}/base_footprint',
            'local_frame': f'{robot_id}/odom',
            'odom_topic': 'odom',
            'scan_topic': 'scan_raw',
            'topic': 'scan_raw',
            'cmd_vel_topic': 'cmd_vel_nav',
            'global_frame_id': 'map',
            'global_frame': 'map',
        },
        convert_types=True,
    ), allow_substs=True)
    remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static'), ('map', '/map')]
    navigation_nodes = [
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', []),
        ('nav2_bt_navigator', 'bt_navigator', []),
    ]
    actions = [
        Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=robot_id,
             parameters=[configured], remappings=remappings, output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', namespace=robot_id,
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['amcl']}]),
    ]
    for package, executable, extra_remappings in navigation_nodes:
        actions.append(Node(package=package, executable=executable, name=executable,
                            namespace=robot_id, parameters=[configured],
                            remappings=remappings + extra_remappings, output='screen'))
    actions.extend([
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=robot_id,
             parameters=[{
                 'use_sim_time': True,
                 'autostart': True,
                 'node_names': [name for _, name, _ in navigation_nodes],
             }]),
        Node(package='mentorpi_nav', executable='goal_bridge.py', name='goal_bridge', namespace=robot_id,
             parameters=[{
                 'use_sim_time': True,
                 'goal_topic': f'/{robot_id}/move_base_simple/goal',
                 'command_topic': f'/{robot_id}/controller/cmd_vel',
                 'status_topic': f'/{robot_id}/navigation/status',
                 'action_name': 'navigate_to_pose',
             }], output='screen'),
        Node(package='mentorpi_nav', executable='cmd_vel_relay.py', name='cmd_vel_relay', namespace=robot_id,
             parameters=[{
                 'use_sim_time': True,
                 'input_topic': f'/{robot_id}/cmd_vel_nav',
                 'output_topic': f'/{robot_id}/controller/cmd_vel',
             }], output='screen'),
    ])
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory('mentorpi_nav'))
    map_yaml = LaunchConfiguration('map_yaml')
    warehouse_x = LaunchConfiguration('warehouse_x')
    warehouse_y = LaunchConfiguration('warehouse_y')
    warehouse_yaw = LaunchConfiguration('warehouse_yaw')
    params_file = str(package_share / 'config' / 'nav2.yaml')
    actions = [
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('warehouse_x', default_value='0.0'),
        DeclareLaunchArgument('warehouse_y', default_value='0.0'),
        DeclareLaunchArgument('warehouse_yaw', default_value='0.0'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', parameters=[{
                 'use_sim_time': True, 'autostart': True, 'node_names': ['map_server'],
             }]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_warehouse',
             arguments=['--x', warehouse_x, '--y', warehouse_y, '--z', '0', '--roll', '0',
                        '--pitch', '0', '--yaw', warehouse_yaw, '--frame-id', 'map',
                        '--child-frame-id', 'warehouse'], parameters=[{'use_sim_time': True}], output='screen'),
    ]
    actions.extend(robot_stack('robot_1', params_file))
    actions.extend(robot_stack('robot_2', params_file))
    return LaunchDescription(actions)
