from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _robot_nodes(name, xyz, package_share):
    description_path = Path(get_package_share_directory('mentorpi_description')) / 'urdf' / 'mecanum.xacro'
    sdf_path = Path(package_share) / 'models' / 'mentorpi_m1' / 'model.sdf.xacro'
    description = xacro.process_file(str(description_path)).toxml()
    sdf = xacro.process_file(str(sdf_path), mappings={'robot_name': name}).toxml()
    bridge = str(Path(package_share) / 'config' / f'{name}_bridge.yaml')
    image_topic = f'/{name}/depth/image_raw'
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=name,
             parameters=[{'robot_description': description, 'frame_prefix': f'{name}/', 'use_sim_time': True}],
             remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'mentorpi_warehouse', '-name', name, '-string', sdf,
                        '-x', xyz[0], '-y', xyz[1], '-z', xyz[2]]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{name}_bridge',
             parameters=[{'config_file': bridge}], output='screen'),
        Node(package='ros_gz_image', executable='image_bridge', name=f'{name}_depth_image_bridge',
             arguments=[image_topic], output='screen'),
        Node(package='mentorpi_gz_sim', executable='gz_pose_to_odom.py', namespace=name,
             name='gz_pose_to_odom', output='screen',
             parameters=[{'robot_name': name, 'publish_frequency': 30.0, 'use_sim_time': True}]),
    ]


def _launch(context):
    package_share = get_package_share_directory('mentorpi_gz_sim')
    world = str(Path(package_share) / 'worlds' / 'warehouse.sdf')
    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes')
    gz_args = f'-r -s --headless-rendering -v 2 {world}' if headless else f'-r -v 2 {world}'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())
    return [gz_sim] + _robot_nodes('robot_1', ('-1.5', '-0.8', '0.05'), package_share) \
        + _robot_nodes('robot_2', ('-1.5', '0.8', '0.05'), package_share)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        OpaqueFunction(function=_launch),
    ])
