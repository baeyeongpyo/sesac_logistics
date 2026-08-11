from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _vehicle_nodes(context):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    package_share = Path(get_package_share_directory('mentorpi_gz_sim'))
    description_path = (
        Path(get_package_share_directory('mentorpi_description'))
        / 'urdf' / 'mecanum_forklift.xacro'
    )
    sdf_path = package_share / 'models' / 'mentorpi_m1' / 'model.sdf.xacro'
    description = xacro.process_file(str(description_path)).toxml()
    sdf = xacro.process_file(str(sdf_path), mappings={'robot_name': robot_id}).toxml()
    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', namespace=robot_id,
            parameters=[{
                'robot_description': description,
                'frame_prefix': f'{robot_id}/',
                'use_sim_time': True,
            }],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        ),
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-world', 'mentorpi_warehouse', '-name', robot_id, '-string', sdf,
                '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw'),
            ],
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'{robot_id}_bridge',
            parameters=[{'config_file': LaunchConfiguration('bridge_config')}], output='screen',
        ),
        Node(
            package='ros_gz_image', executable='image_bridge', name=f'{robot_id}_depth_image_bridge',
            arguments=[f'/{robot_id}/depth/image_raw'], output='screen',
        ),
        Node(
            package='mentorpi_gz_sim', executable='gz_pose_to_odom.py', namespace=robot_id,
            name='gz_pose_to_odom', output='screen',
            parameters=[{'robot_name': robot_id, 'publish_frequency': 30.0, 'use_sim_time': True}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id'),
        DeclareLaunchArgument('x'),
        DeclareLaunchArgument('y'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('bridge_config'),
        OpaqueFunction(function=_vehicle_nodes),
    ])
