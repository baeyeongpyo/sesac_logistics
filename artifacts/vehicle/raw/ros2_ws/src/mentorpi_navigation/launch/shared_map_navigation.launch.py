from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def robot_stack(name, params_file, safety_file, initial_y):
    rewrites = {
        'use_sim_time': 'true',
        'base_frame_id': f'{name}/base_footprint',
        'odom_frame_id': f'{name}/odom',
        'robot_base_frame': f'{name}/base_footprint',
        'odom_frame': f'{name}/odom',
        'global_frame_id': 'map',
        'global_frame': 'map',
        'scan_topic': 'scan_raw',
        'odom_topic': 'odom',
        'y': str(initial_y),
    }
    configured = ParameterFile(RewrittenYaml(
        source_file=params_file, root_key=name,
        param_rewrites=rewrites, convert_types=True), allow_substs=True)
    safety_configured = ParameterFile(RewrittenYaml(
        source_file=safety_file, root_key=name,
        param_rewrites={
            'use_sim_time': 'true',
            'base_frame_id': f'{name}/base_footprint',
            'odom_frame_id': f'{name}/odom',
        }, convert_types=True), allow_substs=True)
    remaps = [('/tf', '/tf'), ('/tf_static', '/tf_static'), ('map', '/map')]
    nav_nodes = [
        ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', []),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
    ]
    actions = [
        Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=name,
             parameters=[configured], remappings=remaps, output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', namespace=name,
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['amcl']}]),
    ]
    for package, executable, node_name, extra in nav_nodes:
        actions.append(Node(package=package, executable=executable, name=node_name,
                            namespace=name, parameters=[configured],
                            remappings=remaps + extra, output='screen'))
    actions.append(Node(package='nav2_collision_monitor', executable='collision_monitor',
                        name='collision_monitor', namespace=name,
                        parameters=[safety_configured], remappings=remaps, output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager_navigation', namespace=name,
                        parameters=[{'use_sim_time': True, 'autostart': True,
                                     'node_names': [item[2] for item in nav_nodes] + ['collision_monitor']}]))
    return actions


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params = PathJoinSubstitution([FindPackageShare('mentorpi_navigation'), 'config', 'nav2.yaml'])
    safety = PathJoinSubstitution([FindPackageShare('mentorpi_safety'), 'config', 'collision_monitor.yaml'])
    actions = [
        DeclareLaunchArgument('map'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
             remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', parameters=[{
                 'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]),
    ]
    actions.extend(robot_stack('robot_1', params, safety, -0.8))
    actions.extend(robot_stack('robot_2', params, safety, 0.8))
    return LaunchDescription(actions)
