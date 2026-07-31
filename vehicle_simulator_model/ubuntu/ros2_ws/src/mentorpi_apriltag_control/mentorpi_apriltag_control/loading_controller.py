import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LoadingController(Node):
    def __init__(self):
        super().__init__("loading_controller")
        self.declare_parameter("target_topic", "/usb_camera/apriltag/target")
        self.declare_parameter("cmd_vel_topic", "/robot_1/controller/cmd_vel")
        self.declare_parameter("scan_topic", "/robot_1/scan_raw")
        self.declare_parameter("lift_topic", "/robot_1/fork/command")
        self.declare_parameter("status_topic", "/loading/status")
        self.declare_parameter("target_id", 1)
        self.declare_parameter("stop_distance", 0.19)
        self.declare_parameter("stop_tag_width_px", 114.0)
        self.declare_parameter("stop_tag_width_tolerance_px", 8.0)
        self.declare_parameter("safety_stop_distance", 0.30)
        self.declare_parameter("search_linear_speed", 0.04)
        self.declare_parameter("search_steering_angle", 0.65)
        self.declare_parameter("search_leg_duration", 2.0)
        self.declare_parameter("search_timeout", 12.0)
        self.declare_parameter("align_tolerance", 0.08)
        self.declare_parameter("linear_gain", 0.30)
        self.declare_parameter("angular_gain", 1.1)
        self.declare_parameter("max_linear_speed", 0.16)
        self.declare_parameter("max_reverse_speed", 0.08)
        self.declare_parameter("wheelbase", 0.22)
        self.declare_parameter("max_steering_angle", 1.5708)
        self.declare_parameter("lift_command", "UP")
        self.declare_parameter("insert_distance", 0.19)
        self.declare_parameter("insert_speed", 0.03)
        self.declare_parameter("lost_timeout", 0.5)

        self.target_id = int(self.get_parameter("target_id").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.stop_tag_width_px = float(self.get_parameter("stop_tag_width_px").value)
        self.stop_tag_width_tolerance_px = float(
            self.get_parameter("stop_tag_width_tolerance_px").value
        )
        self.safety_stop_distance = float(self.get_parameter("safety_stop_distance").value)
        self.search_linear_speed = float(self.get_parameter("search_linear_speed").value)
        self.search_steering_angle = float(self.get_parameter("search_steering_angle").value)
        self.search_leg_duration = float(self.get_parameter("search_leg_duration").value)
        self.search_timeout = float(self.get_parameter("search_timeout").value)
        self.align_tolerance = float(self.get_parameter("align_tolerance").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_reverse_speed = float(self.get_parameter("max_reverse_speed").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.lift_command = str(self.get_parameter("lift_command").value)
        self.insert_distance = float(self.get_parameter("insert_distance").value)
        self.insert_speed = float(self.get_parameter("insert_speed").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout").value)

        self.state = "TAG_SEARCH"
        self.last_seen = 0.0
        self.last_detection = None
        self.front_obstacle_distance = math.inf
        self.lift_sent = False
        self.insert_started_at = None
        self.search_started_at = None
        self.last_published_state = None

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.lift_pub = self.create_publisher(String, self.get_parameter("lift_topic").value, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.create_subscription(String, self.get_parameter("target_topic").value, self.on_target, 10)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info("Loading controller started")

    def on_target(self, msg):
        payload = json.loads(msg.data)
        detections = payload.get("detections", [])
        matches = [d for d in detections if d.get("id") == self.target_id]
        self.last_detection = (
            max(matches, key=lambda detection: detection.get("tag_width_px", 0.0))
            if matches else None
        )
        if self.last_detection:
            self.last_seen = time.monotonic()

    def on_scan(self, msg):
        ranges = []
        for index, value in enumerate(msg.ranges):
            if not math.isfinite(value):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) <= math.radians(25):
                ranges.append(value)
        self.front_obstacle_distance = min(ranges) if ranges else math.inf

    def on_timer(self):
        cmd = Twist()
        detection = self.current_detection()

        if self.front_obstacle_distance < self.safety_stop_distance:
            self.state = "SAFETY_STOP"
        elif self.state == "FORK_INSERT_FORWARD":
            if self.insert_elapsed() < self.insert_duration():
                cmd.linear.x = self.insert_speed
            else:
                self.state = "LIFT_UP"
                if not self.lift_sent:
                    self.lift_pub.publish(String(data=self.lift_command))
                    self.lift_sent = True
        elif detection is None:
            cmd = self.search_command()
        else:
            self.search_started_at = None
            error_x = float(detection["error_x"])
            distance = float(detection.get("distance", self.stop_distance))
            tag_width_px = float(detection.get("tag_width_px", 0.0))
            steering_angle = self.clamp(
                self.angular_gain * error_x,
                self.max_steering_angle,
            )

            if abs(error_x) > self.align_tolerance:
                self.state = "TAG_ALIGN"
                cmd.linear.x = self.max_linear_speed * 0.35
                cmd.angular.z = self.angular_from_steering(cmd.linear.x, steering_angle)
            elif self.should_approach(distance, tag_width_px):
                self.state = "APPROACH"
                cmd.linear.x = self.approach_speed(distance, tag_width_px)
                cmd.angular.z = self.angular_from_steering(cmd.linear.x, steering_angle)
            elif self.should_back_off(distance, tag_width_px):
                self.state = "BACK_OFF"
                cmd.linear.x = self.approach_speed(distance, tag_width_px)
                cmd.angular.z = self.angular_from_steering(cmd.linear.x, steering_angle)
            else:
                self.state = "STOP_AT_DISTANCE"
                cmd = Twist()
                self.insert_started_at = time.monotonic()
                self.state = "FORK_INSERT_FORWARD"

        self.cmd_pub.publish(cmd)
        self.publish_status()

    def current_detection(self):
        if self.last_detection and time.monotonic() - self.last_seen <= self.lost_timeout:
            return self.last_detection
        return None

    def should_approach(self, distance, tag_width_px):
        if distance > 0.0:
            return distance > self.stop_distance
        return tag_width_px < self.stop_tag_width_px - self.stop_tag_width_tolerance_px

    def should_back_off(self, distance, tag_width_px):
        if distance > 0.0:
            return distance < self.stop_distance
        return tag_width_px > self.stop_tag_width_px + self.stop_tag_width_tolerance_px

    def approach_speed(self, distance, tag_width_px):
        if distance > 0.0:
            return self.clamp_range(
                self.linear_gain * (distance - self.stop_distance),
                -self.max_reverse_speed,
                self.max_linear_speed,
            )
        pixel_error = (self.stop_tag_width_px - tag_width_px) / self.stop_tag_width_px
        return self.clamp_range(
            self.max_linear_speed * pixel_error,
            -self.max_reverse_speed,
            self.max_linear_speed,
        )

    def angular_from_steering(self, linear_speed, steering_angle):
        if abs(linear_speed) < 1e-6 or self.wheelbase <= 0.0:
            return 0.0
        return linear_speed * math.tan(steering_angle) / self.wheelbase

    def insert_duration(self):
        if self.insert_speed <= 0.0:
            return 0.0
        return self.insert_distance / self.insert_speed

    def insert_elapsed(self):
        if self.insert_started_at is None:
            return 0.0
        return time.monotonic() - self.insert_started_at

    def search_command(self):
        if self.state == "TAG_SEARCH_FAILED":
            return Twist()

        now = time.monotonic()
        if self.search_started_at is None:
            self.search_started_at = now

        if now - self.search_started_at >= self.search_timeout:
            self.state = "TAG_SEARCH_FAILED"
            return Twist()

        self.state = "TAG_SEARCH"
        cmd = Twist()
        cmd.linear.x = self.search_linear_speed
        leg = int((now - self.search_started_at) / max(self.search_leg_duration, 0.1))
        steering_sign = 1.0 if leg % 2 == 0 else -1.0
        steering_angle = self.clamp(
            steering_sign * self.search_steering_angle,
            self.max_steering_angle,
        )
        cmd.angular.z = self.angular_from_steering(cmd.linear.x, steering_angle)
        return cmd

    def publish_status(self):
        if self.state == self.last_published_state:
            return
        self.status_pub.publish(String(data=self.state))
        self.last_published_state = self.state

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def clamp_range(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = LoadingController()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
