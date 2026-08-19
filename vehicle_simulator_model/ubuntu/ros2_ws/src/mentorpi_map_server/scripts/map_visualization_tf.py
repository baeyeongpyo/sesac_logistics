#!/usr/bin/env python3
"""Publish the visualization-only map frame on the dynamic TF topic."""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MapVisualizationTfPublisher(Node):
    def __init__(self):
        super().__init__('map_visualization_tf')
        self._broadcaster = TransformBroadcaster(self)
        # The observation stack has no /clock publisher. Keep this
        # visualization-only transform independent from each vehicle's
        # simulated clock, even if use_sim_time is accidentally enabled.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._timer = self.create_timer(
            0.1,
            self._publish_transform,
            clock=self._steady_clock,
        )
        self._logged_first_publish = False
        self._publish_transform()

    def _publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self._system_clock.now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'map_visualization'
        transform.transform.rotation.w = 1.0
        self._broadcaster.sendTransform(transform)
        if not self._logged_first_publish:
            self.get_logger().info(
                'Publishing map -> map_visualization on /tf',
            )
            self._logged_first_publish = True


def main():
    rclpy.init()
    node = MapVisualizationTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
