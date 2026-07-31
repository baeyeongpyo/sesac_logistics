from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mentorpi_apriltag_control",
            executable="apriltag_detector",
            name="ascamera_apriltag_detector",
            output="screen",
            parameters=[{
                "image_topic": "/ascamera_hp60c/camera_publisher/rgb0/image",
                "camera_info_topic": "/ascamera_hp60c/camera_publisher/rgb0/camera_info",
                "target_topic": "/ascamera_hp60c/apriltag/target",
                "tag_size": 0.047,
            }],
        ),
        Node(
            package="mentorpi_apriltag_control",
            executable="loading_controller",
            name="loading_controller",
            output="screen",
            parameters=[{
                "target_topic": "/ascamera_hp60c/apriltag/target",
                "cmd_vel_topic": "/cmd_vel",
                "scan_topic": "/scan",
                "lift_topic": "/fork/command",
                "status_topic": "/loading/status",
                "target_id": 1,
                "stop_distance": 0.19,
                "safety_stop_distance": 0.0,
                "wheelbase": 0.22,
                "max_steering_angle": 1.5708,
                "search_linear_speed": 0.04,
                "search_steering_angle": 0.65,
                "search_leg_duration": 2.0,
                "search_timeout": 12.0,
                "align_tolerance": 0.03,
                "final_align_tolerance": 0.02,
                "final_align_hold_time": 0.5,
                "insert_duration": 3.0,
                "insert_speed": 0.03,
            }],
        ),
    ])
