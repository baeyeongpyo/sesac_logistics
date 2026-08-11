from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _vehicle_nodes(context):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    package_share = Path(get_package_share_directory('mentorpi_nav'))
    configured = ParameterFile(RewrittenYaml(
        source_file=str(package_share / 'config' / 'nav2.yaml'),
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
            'cmd_vel_topic': 'navigation/cmd_vel',
        },
        convert_types=True,
    ), allow_substs=True)
    remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static'), ('map', '/map')]
    navigation_nodes = [
        ('nav2_controller', 'controller_server', [('cmd_vel', 'navigation/cmd_vel')]),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', []),
        ('nav2_bt_navigator', 'bt_navigator', []),
    ]
    actions = [
        Node(
            package=package, executable=executable, name=executable, namespace=robot_id,
            parameters=[configured], remappings=remappings + extra_remappings, output='screen',
        )
        for package, executable, extra_remappings in navigation_nodes
    ]
    actions.extend([
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace=robot_id,
            parameters=[{
                'use_sim_time': True, 'autostart': True,
                'node_names': [name for _, name, _ in navigation_nodes],
            }],
        ),
        Node(
            package='mentorpi_nav', executable='goal_bridge.py', name='goal_bridge', namespace=robot_id,
            parameters=[{
                'use_sim_time': True,
                'goal_topic': f'/{robot_id}/move_base_simple/goal',
                'command_topic': f'/{robot_id}/navigation/cmd_vel',
                'cancel_topic': f'/{robot_id}/navigation/cancel',
                'status_topic': f'/{robot_id}/navigation/status',
                'action_name': 'navigate_to_pose',
            }], output='screen',
        ),
        Node(
            package='mentorpi_nav', executable='cmd_vel_relay.py', name='cmd_vel_relay', namespace=robot_id,
            parameters=[{
                'use_sim_time': True,
                'input_topic': f'/{robot_id}/cmd_vel_nav',
                'output_topic': f'/{robot_id}/navigation/cmd_vel',
            }], output='screen',
        ),
        Node(
            package='mentorpi_nav', executable='cmd_vel_mux.py', name='cmd_vel_mux', namespace=robot_id,
            parameters=[{
                'use_sim_time': True,
                'manual_topic': f'/{robot_id}/manual/cmd_vel',
                'nav_topic': f'/{robot_id}/navigation/cmd_vel',
                'stop_topic': f'/{robot_id}/safety/stop',
                'output_topic': f'/{robot_id}/controller/cmd_vel',
            }], output='screen',
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id'),
        OpaqueFunction(function=_vehicle_nodes),
    ])
