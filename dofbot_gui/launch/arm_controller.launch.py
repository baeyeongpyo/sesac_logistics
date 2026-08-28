"""Launch the maintained DOFBOT I2C driver used by the GUI applications."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dofbot',
            executable='dofbot',
            name='dofbot_arm_controller',
            output='screen',
            parameters=[{
                'motion_time_ms': 1000,
                'limits_file': '/home/intelions/ros2_ws/config/dofbot_limits.json',
            }],
        ),
    ])
