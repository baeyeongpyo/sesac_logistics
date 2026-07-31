from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="usb_camera",
            output="screen",
            parameters=[{
                "video_device": "/dev/video0",
                "image_size": [640, 480],
                "camera_frame_id": "usb_camera",
            }],
            remappings=[
                ("image_raw", "/usb_camera/image_raw"),
                ("camera_info", "/usb_camera/camera_info"),
            ],
        ),
        Node(
            package="mentorpi_apriltag_control",
            executable="apriltag_detector",
            name="usb_apriltag_detector",
            output="screen",
            parameters=[{
                "image_topic": "/usb_camera/image_raw",
                "camera_info_topic": "/usb_camera/camera_info",
                "target_topic": "/usb_camera/apriltag/target",
                "tag_size": 0.047,
            }],
        ),
        Node(
            package="mentorpi_apriltag_control",
            executable="loading_controller",
            name="gazebo_loading_controller",
            output="screen",
            parameters=[{
                "target_topic": "/usb_camera/apriltag/target",
                "cmd_vel_topic": "/robot_1/controller/cmd_vel",
                "scan_topic": "/robot_1/scan_raw",
                "lift_topic": "/robot_1/fork/command",
                "target_id": 1,
                "stop_distance": 0.19,
                "stop_tag_width_px": 114.0,
                "stop_tag_width_tolerance_px": 8.0,
                "safety_stop_distance": 0.0,
                "wheelbase": 0.22,
                "max_steering_angle": 1.5708,
            }],
        ),
    ])
