from launch import LaunchDescription
from launch_ros.actions import Node


def robot_nodes(name):
    return [
        Node(
            package="mentorpi_apriltag_control",
            executable="apriltag_detector",
            name=f"{name}_apriltag_detector",
            output="screen",
            parameters=[{
                "image_topic": f"/{name}/color/image_raw",
                "camera_info_topic": f"/{name}/color/camera_info",
                "target_topic": f"/{name}/apriltag/target",
                "tag_size": 0.30,
            }],
        ),
        Node(
            package="mentorpi_apriltag_control",
            executable="apriltag_follower",
            name=f"{name}_apriltag_follower",
            output="screen",
            parameters=[{
                "target_topic": f"/{name}/apriltag/target",
                "cmd_vel_topic": f"/{name}/controller/cmd_vel",
                "target_id": 0,
                "target_distance": 0.45,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(robot_nodes("robot_1") + robot_nodes("robot_2"))
