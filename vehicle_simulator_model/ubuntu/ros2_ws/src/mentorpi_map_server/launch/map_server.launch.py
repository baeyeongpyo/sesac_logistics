from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            description='Absolute path to the map YAML file that references map.pgm.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Foxglove Studio must receive the visualization frame on `/tf`.
        # This does not attach the map tree to any vehicle odom/base frame.
        Node(
            package='mentorpi_map_server',
            executable='map_visualization_tf.py',
            name='map_visualization_tf',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'topic_name': '/controller_server/map',
                'frame_id': 'map',
            }],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
            output='screen',
        ),
    ])
