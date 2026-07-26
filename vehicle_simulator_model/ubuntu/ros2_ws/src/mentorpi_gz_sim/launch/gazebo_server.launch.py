from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('mentorpi_gz_sim')
    world = str(Path(package_share) / 'worlds' / 'warehouse.sdf')
    gz_args = ['-r -s --headless-rendering -v ', LaunchConfiguration('verbosity'), ' ', world]

    return LaunchDescription([
        DeclareLaunchArgument('verbosity', default_value='2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
    ])
