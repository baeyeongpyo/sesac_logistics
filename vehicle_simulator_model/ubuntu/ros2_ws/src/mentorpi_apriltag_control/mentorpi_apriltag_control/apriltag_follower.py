import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class AprilTagFollower(Node):
    def __init__(self):
        super().__init__("apriltag_follower")
        self.declare_parameter("target_topic", "/robot_1/apriltag/target")
        self.declare_parameter("cmd_vel_topic", "/robot_1/controller/cmd_vel")
        self.declare_parameter("target_id", 0)
        self.declare_parameter("target_distance", 0.45)
        self.declare_parameter("max_linear_speed", 0.18)
        self.declare_parameter("max_angular_speed", 0.7)
        self.declare_parameter("angular_gain", 1.1)
        self.declare_parameter("linear_gain", 0.35)
        self.declare_parameter("lost_timeout", 0.5)

        self.target_id = int(self.get_parameter("target_id").value)
        self.target_distance = float(self.get_parameter("target_distance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout").value)
        self.last_seen = 0.0
        self.last_detection = None

        self.publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.create_subscription(
            String, self.get_parameter("target_topic").value, self.on_target, 10
        )
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info("AprilTag follower started")

    def on_target(self, msg):
        payload = json.loads(msg.data)
        matches = [
            detection
            for detection in payload.get("detections", [])
            if detection.get("id") == self.target_id
        ]
        self.last_detection = matches[0] if matches else None
        if self.last_detection:
            self.last_seen = time.monotonic()

    def on_timer(self):
        cmd = Twist()
        if self.last_detection and time.monotonic() - self.last_seen <= self.lost_timeout:
            error_x = float(self.last_detection["error_x"])
            distance = float(self.last_detection.get("distance", self.target_distance))
            cmd.angular.z = self.clamp(-self.angular_gain * error_x, self.max_angular_speed)

            if abs(error_x) < 0.18:
                distance_error = distance - self.target_distance
                cmd.linear.x = self.clamp(
                    self.linear_gain * distance_error, self.max_linear_speed
                )
        self.publisher.publish(cmd)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
