#!/usr/bin/env python3
"""Vehicle-local safe velocity mux: stop > manual > Nav2 > zero."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from velocity_arbitration import VelocityArbitrator


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('manual_topic', '/manual/cmd_vel')
        self.declare_parameter('nav_topic', '/navigation/cmd_vel')
        self.declare_parameter('stop_topic', '/safety/stop')
        self.declare_parameter('output_topic', '/controller/cmd_vel')
        self.declare_parameter('timeout_seconds', 0.35)
        self._arbitrator = VelocityArbitrator(float(self.get_parameter('timeout_seconds').value))
        self._publisher = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('manual_topic').value, self._manual, 10)
        self.create_subscription(Twist, self.get_parameter('nav_topic').value, self._nav, 10)
        self.create_subscription(Empty, self.get_parameter('stop_topic').value, self._stop, 10)
        self.create_timer(0.05, self._publish_selected)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _manual(self, command: Twist) -> None:
        self._arbitrator.record('manual', command, self._now())

    def _nav(self, command: Twist) -> None:
        self._arbitrator.record('nav', command, self._now())

    def _stop(self, message: Empty) -> None:
        self._arbitrator.stop(self._now())

    def _publish_selected(self) -> None:
        self._publisher.publish(self._arbitrator.select(self._now()) or Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
