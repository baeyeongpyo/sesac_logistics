import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import rclpy


def yolo_process(context):
    """Reuse a live YOLO ROS node, otherwise start the detector."""
    requested_vehicle = int(LaunchConfiguration("vehicle").perform(context))
    domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
    vehicle = requested_vehicle or {215: 1, 216: 2}.get(domain_id)
    if vehicle not in (1, 2):
        raise RuntimeError(
            "vehicle cannot be inferred: ROS_DOMAIN_ID must be 215 or 216"
        )
    detection_topic = f"/robot_{vehicle}/symbol_seg/detections"
    probe = None
    try:
        rclpy.init()
        probe = rclpy.create_node("_auto_dock_yolo_probe")
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            publishers = probe.get_publishers_info_by_topic(detection_topic)
            if any(info.node_name == "yolo_tag" for info in publishers):
                return [LogInfo(msg=f"YOLO already publishes {detection_topic}; reusing it.")]
    finally:
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return [
        LogInfo(msg=f"No YOLO publisher on {detection_topic}; starting it."),
        ExecuteProcess(
            cmd=["/usr/bin/python3", "/shared/yolo_symbol_seg_node.py"],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "vehicle", default_value="0",
            description="0 maps ROS_DOMAIN_ID 215→vehicle 1 and 216→vehicle 2.",
        ),
        DeclareLaunchArgument("pose_config", default_value="/shared/vehicle_pose_config.json"),
        DeclareLaunchArgument(
            "search_linear_speed_m_s", default_value="0.0",
            description="Temporary search speed override in m/s; 0 uses pose config.",
        ),
        DeclareLaunchArgument(
            "config_overrides", default_value="{}",
            description="Temporary JSON object merged over pose config for this run only.",
        ),
        # YOLO stays an independent node; auto_dock sends only its selected
        # tag pair through the existing local UDP target-control interface.
        OpaqueFunction(function=yolo_process),
        Node(
            package="auto_dock",
            executable="auto_dock_node",
            name="auto_dock",
            output="screen",
            parameters=[{
                "vehicle": LaunchConfiguration("vehicle"),
                "pose_config": LaunchConfiguration("pose_config"),
                # ROS launch otherwise YAML-parses `{}` into a dict before it
                # reaches rclpy; the controller intentionally parses JSON.
                "config_overrides": ParameterValue(
                    LaunchConfiguration("config_overrides"), value_type=str
                ),
                # Keep the short speed override for quick field tests; it is
                # merged by launch into the same JSON override mechanism.
                "search_linear_speed_m_s": LaunchConfiguration("search_linear_speed_m_s"),
            }],
        ),
        Node(
            package="auto_dock",
            executable="tag_entity_mapper",
            name="tag_entity_mapper",
            output="screen",
            parameters=[{"vehicle": LaunchConfiguration("vehicle")}],
        ),
        Node(
            package="auto_dock",
            executable="target_nav_bridge",
            name="target_nav_bridge",
            output="screen",
            parameters=[{"vehicle": LaunchConfiguration("vehicle")}],
        ),
    ])
