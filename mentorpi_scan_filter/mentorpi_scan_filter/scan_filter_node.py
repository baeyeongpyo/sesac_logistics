#!/usr/bin/env python3
"""포크 자기반사 각도를 제거한 LaserScan을 별도 토픽으로 발행한다."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from mentorpi_scan_filter.filter_core import filter_ranges


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('mentorpi_scan_filter')
        self.declare_parameter('first_end_index', 30)
        self.declare_parameter('second_start_index', 470)
        self.first_end_index = int(
            self.get_parameter('first_end_index').value)
        self.second_start_index = int(
            self.get_parameter('second_start_index').value)
        if self.first_end_index < 0:
            raise ValueError('first_end_index must be non-negative')
        if self.second_start_index <= self.first_end_index:
            raise ValueError(
                'second_start_index must be greater than first_end_index')
        self.output = self.create_publisher(
            LaserScan, 'scan_filtered', qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'filtering LaserScan indices 0..{self.first_end_index} and '
            f'{self.second_start_index}..end')

    def scan_callback(self, message):
        if not message.ranges:
            self.get_logger().warning('ignoring malformed LaserScan')
            return
        filtered = LaserScan()
        filtered.header = message.header
        filtered.angle_min = message.angle_min
        filtered.angle_max = message.angle_max
        filtered.angle_increment = message.angle_increment
        filtered.time_increment = message.time_increment
        filtered.scan_time = message.scan_time
        filtered.range_min = message.range_min
        filtered.range_max = message.range_max
        filtered.ranges = filter_ranges(
            message.ranges, self.first_end_index, self.second_start_index)
        filtered.intensities = list(message.intensities)
        self.output.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
