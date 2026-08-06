from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params = PathJoinSubstitution([FindPackageShare('mentorpi_safety'), 'config', 'collision_monitor.yaml'])
    configured = ParameterFile(RewrittenYaml(
        source_file=params, root_key=namespace,
        param_rewrites={'use_sim_time': 'true'}, convert_types=True), allow_substs=True)
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', namespace=namespace, parameters=[configured],
             remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_safety', namespace=namespace,
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['collision_monitor']}]),
    ])
