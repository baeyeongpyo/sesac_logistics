#!/usr/bin/env python3
"""Forward Nav2 velocity commands and fail safe to zero when they time out."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/robot_1/controller/cmd_vel')
        self.declare_parameter('timeout_seconds', 0.35)
        self.timeout = float(self.get_parameter('timeout_seconds').value)
        self.publisher = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.subscription = self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.on_command, 10)
        self.last_command_time = self.get_clock().now()
        self.timed_out = True
        self.timer = self.create_timer(0.05, self.enforce_timeout)

    def on_command(self, command):
        self.last_command_time = self.get_clock().now()
        self.timed_out = False
        self.publisher.publish(command)

    def enforce_timeout(self):
        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        if not self.timed_out and elapsed > self.timeout:
            self.publisher.publish(Twist())
            self.timed_out = True


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
