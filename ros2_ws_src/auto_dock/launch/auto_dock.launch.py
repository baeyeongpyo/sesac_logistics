from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        # Keep every child in the vehicle DDS domain even when the invoking
        # shell has not exported ROS_DOMAIN_ID.
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        # YOLO stays an independent node; auto_dock sends only its selected
        # tag pair through the existing local UDP target-control interface.
        ExecuteProcess(
            cmd=["/usr/bin/python3", "/shared/yolo_symbol_seg_node.py"],
            output="screen",
        ),
        Node(
            package="auto_dock",
            executable="auto_dock_node",
            name="auto_dock",
            output="screen",
            parameters=[{
                "vehicle": LaunchConfiguration("vehicle"),
                "pose_config": LaunchConfiguration("pose_config"),
                "config_overrides": LaunchConfiguration("config_overrides"),
                # Keep the short speed override for quick field tests; it is
                # merged by launch into the same JSON override mechanism.
                "search_linear_speed_m_s": LaunchConfiguration("search_linear_speed_m_s"),
            }],
        ),
    ])
