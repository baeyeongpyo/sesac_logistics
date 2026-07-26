from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(get_package_share_directory('mentorpi_gz_sim'))
    launch_directory = package_share / 'launch'

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_directory / 'gazebo_server.launch.py')),
        launch_arguments={'verbosity': LaunchConfiguration('verbosity')}.items(),
    )
    sim_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_directory / 'sim_adapter.launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('verbosity', default_value='2'),
        gazebo_server,
        sim_adapter,
    ])
