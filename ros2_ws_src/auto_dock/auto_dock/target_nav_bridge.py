#!/usr/bin/env python3
"""Send detected pallet approach poses to the Nav2 NavigateToPose action."""

import json
import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_pose_2d(pose, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    transform_yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    c, s = math.cos(transform_yaw), math.sin(transform_yaw)
    return {
        "x": float(translation.x) + c * float(pose["x"]) - s * float(pose["y"]),
        "y": float(translation.y) + s * float(pose["x"]) + c * float(pose["y"]),
        "yaw": normalize_angle(transform_yaw + float(pose["yaw"])),
    }


class TargetNavBridge(Node):
    def __init__(self):
        super().__init__("target_nav_bridge")
        self.declare_parameter("vehicle", 0)
        self.declare_parameter("target_found_topic", "")
        self.declare_parameter("approach_result_topic", "")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")

        requested_vehicle = int(self.get_parameter("vehicle").value)
        domain_vehicle = {215: 1, 216: 2}.get(
            int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
        )
        self.vehicle = requested_vehicle or domain_vehicle
        if self.vehicle not in (1, 2):
            raise RuntimeError("vehicle must be 1/2, or ROS_DOMAIN_ID must be 215/216")
        robot = f"/robot_{self.vehicle}"
        target_topic = (
            str(self.get_parameter("target_found_topic").value).strip()
            or f"{robot}/auto_dock/target_found"
        )
        result_topic = (
            str(self.get_parameter("approach_result_topic").value).strip()
            or f"{robot}/nav2/approach_result"
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.create_subscription(String, target_topic, self.on_target_found, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
        )
        self.goal_active = False
        self.source_stamp_ns = 0

    def publish_result(self, status, **extra):
        self.result_pub.publish(String(data=json.dumps({
            "status": status,
            "source_stamp_ns": self.source_stamp_ns,
            **extra,
        }, ensure_ascii=False)))

    def on_target_found(self, message):
        if self.goal_active:
            self.get_logger().warning("Nav2 approach goal already active; target ignored")
            return
        try:
            payload = json.loads(message.data)
            target = payload["target"]
            source_frame = str(payload.get("frame_id", "odom"))
            standoff = max(0.20, min(2.0, float(payload["approach_standoff_m"])))
            self.source_stamp_ns = int(payload.get("source_stamp_ns", 0) or 0)
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, Time()
            )
            map_target = transform_pose_2d(target, transform)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError, TransformException) as exc:
            self.get_logger().warning(f"target-found conversion failed: {exc}")
            self.publish_result("invalid_target")
            return

        approach_x = map_target["x"] - standoff * math.cos(map_target["yaw"])
        approach_y = map_target["y"] - standoff * math.sin(map_target["yaw"])
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.publish_result("server_unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = approach_x
        goal.pose.pose.position.y = approach_y
        goal.pose.pose.orientation.z = math.sin(map_target["yaw"] * 0.5)
        goal.pose.pose.orientation.w = math.cos(map_target["yaw"] * 0.5)
        self.goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info(
            f"Nav2 approach goal sent: ({approach_x:.3f}, {approach_y:.3f})"
        )

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.goal_active = False
            self.get_logger().error(f"Nav2 goal request failed: {exc}")
            self.publish_result("request_failed")
            return
        if not goal_handle.accepted:
            self.goal_active = False
            self.publish_result("rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        self.goal_active = False
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f"Nav2 result failed: {exc}")
            self.publish_result("result_failed")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_result("succeeded")
        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_result("canceled")
        else:
            self.publish_result("failed", nav2_status=int(status))


def main(args=None):
    rclpy.init(args=args)
    node = TargetNavBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
