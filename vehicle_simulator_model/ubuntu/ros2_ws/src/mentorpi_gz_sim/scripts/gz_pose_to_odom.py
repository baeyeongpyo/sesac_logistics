#!/usr/bin/env python3
"""Convert Gazebo model pose output into planar ROS odometry and TF."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class GazeboPoseToOdom(Node):
    def __init__(self):
        super().__init__('gz_pose_to_odom')
        default_name = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_name', default_name)
        self.declare_parameter('publish_frequency', 30.0)
        self.robot_name = self.get_parameter('robot_name').value
        frequency = float(self.get_parameter('publish_frequency').value)
        if frequency <= 0.0:
            raise ValueError('publish_frequency must be positive')

        self.odom_frame = f'{self.robot_name}/odom'
        self.base_frame = f'{self.robot_name}/base_footprint'
        self.latest_transform = None
        self.previous = None
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_subscription = self.create_subscription(
            TFMessage,
            'ground_truth/pose',
            self.pose_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / frequency, self.publish_odom)

    def pose_callback(self, msg):
        for candidate in msg.transforms:
            if candidate.child_frame_id == self.robot_name:
                self.latest_transform = candidate.transform
                return

    def publish_odom(self):
        if self.latest_transform is None:
            return

        now = self.get_clock().now()
        pose = self.latest_transform
        x = pose.translation.x
        y = pose.translation.y
        yaw = yaw_from_quaternion(pose.rotation)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        vx = vy = wz = 0.0
        if self.previous is not None:
            previous_time, previous_x, previous_y, previous_yaw = self.previous
            dt = (now - previous_time).nanoseconds * 1e-9
            if 1e-4 < dt < 1.0:
                world_vx = (x - previous_x) / dt
                world_vy = (y - previous_y) / dt
                vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
                vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
                wz = angle_delta(yaw, previous_yaw) / dt
        self.previous = (now, x, y, yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 1e-3
        odom.pose.covariance[7] = 1e-3
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e-3
        odom.twist.covariance[:] = odom.pose.covariance
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseToOdom()
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
