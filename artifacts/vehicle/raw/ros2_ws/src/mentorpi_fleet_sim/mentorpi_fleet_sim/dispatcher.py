import math
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from .corridor_reservation import CorridorReservation


class FleetDispatcher(Node):
    def __init__(self):
        super().__init__('fleet_dispatcher')
        self.declare_parameter('map_version', 'v001')
        version = self.get_parameter('map_version').value
        if version != 'v001':
            raise ValueError(f'unsupported active map version: {version}')
        self._reservation = CorridorReservation(ttl_seconds=120.0)
        self._action_clients = {
            'robot_1': ActionClient(self, NavigateToPose, '/robot_1/navigate_to_pose'),
            'robot_2': ActionClient(self, NavigateToPose, '/robot_2/navigate_to_pose'),
        }
        self._tasks = [('robot_1', -0.8, -0.8, 0.0), ('robot_2', -0.8, 0.8, 0.0)]
        self._index = 0
        self._busy = False
        self.create_timer(1.0, self._dispatch_next)
        self.get_logger().info(f'fleet ready map_version={version} corridor=corridor_a')

    def _dispatch_next(self):
        if self._busy or self._index >= len(self._tasks):
            return
        robot_id, x, y, yaw = self._tasks[self._index]
        client = self._action_clients[robot_id]
        if not client.wait_for_server(timeout_sec=0.1):
            return
        now = time.monotonic()
        if not self._reservation.acquire('corridor_a', robot_id, now):
            return
        self._busy = True
        self.get_logger().info(f'reservation acquired corridor_a holder={robot_id}')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda result, robot=robot_id: self._goal_response(robot, result))

    def _goal_response(self, robot_id, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'goal rejected robot={robot_id}')
            self._finish_task(robot_id)
            return
        self.get_logger().info(f'goal accepted robot={robot_id}')
        handle.get_result_async().add_done_callback(
            lambda result, robot=robot_id: self._goal_result(robot, result))

    def _goal_result(self, robot_id, future):
        self.get_logger().info(f'goal finished robot={robot_id} status={future.result().status}')
        self._finish_task(robot_id)

    def _finish_task(self, robot_id):
        if self._reservation.release('corridor_a', robot_id):
            self.get_logger().info(f'reservation released corridor_a holder={robot_id}')
        self._index += 1
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = FleetDispatcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
