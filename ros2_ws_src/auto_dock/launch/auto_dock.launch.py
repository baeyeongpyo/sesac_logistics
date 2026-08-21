from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="1"),
        DeclareLaunchArgument("ros_domain_id", default_value="215"),
        DeclareLaunchArgument("pose_config", default_value="/shared/vehicle_pose_config.json"),
        DeclareLaunchArgument(
            "search_linear_speed_m_s", default_value="0.0",
            description="Temporary search speed override in m/s; 0 uses pose config.",
        ),
        DeclareLaunchArgument(
            "config_overrides", default_value="{}",
            description="Temporary JSON object merged over pose config for this run only.",
        ),
        # The runner only selects a YOLO target through UDP; it cannot detect
        # symbols by itself.  Keep the detector in this launch so one
        # `ros2 launch auto_dock ...` command is sufficient.
        ExecuteProcess(
            cmd=["/usr/bin/python3", "/shared/yolo_symbol_seg_node.py"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "/usr/bin/python3",
                "/home/ubuntu/ros2_ws/tools/auto_dock_runner.py",
                "--vehicle", LaunchConfiguration("vehicle"),
                "--ros-domain-id", LaunchConfiguration("ros_domain_id"),
                "--pose-config", LaunchConfiguration("pose_config"),
                "--search-linear-speed", LaunchConfiguration("search_linear_speed_m_s"),
                "--config-overrides", LaunchConfiguration("config_overrides"),
            ],
            additional_env={"QT_QPA_PLATFORM": "offscreen"},
            output="screen",
        ),
    ])
