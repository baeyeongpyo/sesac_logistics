from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def _robot_nodes(name, xyz, yaw, package_share):
    description_path = Path(get_package_share_directory('mentorpi_description')) / 'urdf' / 'mecanum_forklift.xacro'
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
                        '-x', xyz[0], '-y', xyz[1], '-z', xyz[2], '-Y', yaw]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{name}_bridge',
             parameters=[{'config_file': bridge}], output='screen'),
        Node(package='ros_gz_image', executable='image_bridge', name=f'{name}_depth_image_bridge',
             arguments=[image_topic], output='screen'),
        Node(package='mentorpi_gz_sim', executable='gz_pose_to_odom.py', namespace=name,
             name='gz_pose_to_odom', output='screen',
             parameters=[{'robot_name': name, 'publish_frequency': 30.0, 'use_sim_time': True}]),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('mentorpi_gz_sim')
    return LaunchDescription(
        _robot_nodes('robot_1', ('1.8', '-2.8', '0.05'), '1.5708', package_share)
        + _robot_nodes('robot_2', ('3.2', '-2.8', '0.05'), '1.5708', package_share)
    )
