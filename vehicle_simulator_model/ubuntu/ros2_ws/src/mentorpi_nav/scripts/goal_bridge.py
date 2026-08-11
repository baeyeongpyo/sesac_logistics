#!/usr/bin/env python3
"""Translate Foxglove PoseStamped goals into Nav2 NavigateToPose actions."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        self.declare_parameter('goal_topic', '/move_base_simple/goal')
        self.declare_parameter('command_topic', '/robot_1/controller/cmd_vel')
        self.declare_parameter('cancel_topic', '/robot_1/navigation/cancel')
        self.declare_parameter('status_topic', '/robot_1/navigation/status')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.goal_client = ActionClient(self, NavigateToPose, self.get_parameter('action_name').value)
        self.stop_publisher = self.create_publisher(Twist, self.get_parameter('command_topic').value, 10)
        self.status_publisher = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.subscription = self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self.on_goal, 10)
        self.cancel_subscription = self.create_subscription(
            Empty, self.get_parameter('cancel_topic').value, self.on_cancel, 10)
        self.active_goal = None
        self.pending_goal = None

    def publish_status(self, value):
        message = String()
        message.data = value
        self.status_publisher.publish(message)

    def stop(self):
        self.stop_publisher.publish(Twist())

    def on_goal(self, pose):
        if pose.header.frame_id != 'map':
            self.publish_status('rejected: goal frame must be map')
            self.stop()
            return
        self.pending_goal = pose
        if self.active_goal is not None:
            self.publish_status('preempting active goal')
            self.active_goal.cancel_goal_async().add_done_callback(self.send_pending_goal)
            self.active_goal = None
            return
        self.send_pending_goal()

    def on_cancel(self, _message):
        self.pending_goal = None
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
            self.active_goal = None
        self.publish_status('canceled')
        self.stop()

    def send_pending_goal(self, _future=None):
        if self.pending_goal is None:
            return
        if not self.goal_client.wait_for_server(timeout_sec=1.0):
            self.publish_status('rejected: NavigateToPose server is unavailable')
            self.stop()
            return
        request = NavigateToPose.Goal()
        request.pose = self.pending_goal
        self.pending_goal = None
        self.goal_client.send_goal_async(request).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.publish_status('rejected: NavigateToPose goal was not accepted')
            self.stop()
            return
        self.active_goal = handle
        self.publish_status('accepted')
        handle.get_result_async().add_done_callback(
            lambda future, goal_handle=handle: self.on_result(goal_handle, future))

    def on_result(self, goal_handle, future):
        result = future.result()
        if goal_handle is not self.active_goal:
            return
        self.active_goal = None
        labels = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'failed',
        }
        self.publish_status(labels.get(result.status, 'failed'))
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
