"""Launch the ASCamera RGB-D driver used by the MentorPi peripherals package."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    config_path = LaunchConfiguration('config_path')

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='depth_cam'),
        DeclareLaunchArgument(
            'config_path',
            default_value='/home/ubuntu/third_party_ros2/third_party_ws/src/ascamera/configurationfiles',
        ),
        Node(
            namespace='ascamera',
            package='ascamera',
            executable='ascamera_node',
            name='ascamera_node',
            respawn=True,
            respawn_delay=2.0,
            output='screen',
            parameters=[{
                'usb_bus_no': -1,
                'usb_path': 'null',
                'confiPath': config_path,
                # Point-cloud generation is expensive on the Raspberry Pi.  The GUI
                # only needs aligned RGB and depth images.
                'color_pcl': False,
                'pub_tfTree': True,
                'depth_width': 640,
                'depth_height': 480,
                'rgb_width': 640,
                'rgb_height': 480,
                'fps': 10,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_camera_link_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.57', '--pitch', '0', '--yaw', '-1.57',
                '--frame-id', camera_name,
                '--child-frame-id', 'ascamera_camera_link_0',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_camera_color_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.57', '--pitch', '0', '--yaw', '-1.57',
                '--frame-id', camera_name,
                '--child-frame-id', 'ascamera_color_0',
            ],
        ),
    ])
