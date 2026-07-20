from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    frame_prefix = LaunchConfiguration('frame_prefix')
    model = LaunchConfiguration('model')
    description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot_1'),
        DeclareLaunchArgument('frame_prefix', default_value='robot_1/'),
        DeclareLaunchArgument('model', default_value=[
            '/opt/mentorpi_ws/install/mentorpi_description/share/mentorpi_description/urdf/mecanum.xacro']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=robot_name, parameters=[{'robot_description': description,
                                                'frame_prefix': frame_prefix,
                                                'use_sim_time': True}]),
    ])
