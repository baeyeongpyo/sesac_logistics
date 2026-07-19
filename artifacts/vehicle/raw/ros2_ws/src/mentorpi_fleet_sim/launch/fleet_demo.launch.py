from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mentorpi_fleet_sim', executable='dispatcher', name='fleet_dispatcher',
             parameters=[{'map_version': 'v001', 'use_sim_time': True}], output='screen')
    ])
