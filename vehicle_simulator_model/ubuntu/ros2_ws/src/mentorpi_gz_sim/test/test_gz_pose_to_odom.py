import importlib.util
from pathlib import Path
from unittest import TestCase
from unittest.mock import MagicMock, patch

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'gz_pose_to_odom.py'
SPEC = importlib.util.spec_from_file_location('gz_pose_to_odom', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GazeboPoseToOdomMainTest(TestCase):
    def test_keyboard_interrupt_does_not_shutdown_an_already_closed_context(self):
        node = MagicMock()

        with patch.object(MODULE, 'GazeboPoseToOdom', return_value=node), \
             patch.object(MODULE.rclpy, 'init'), \
             patch.object(MODULE.rclpy, 'spin', side_effect=KeyboardInterrupt), \
             patch.object(MODULE.rclpy, 'ok', return_value=False), \
             patch.object(MODULE.rclpy, 'shutdown') as shutdown:
            try:
                MODULE.main()
            except KeyboardInterrupt as error:
                self.fail(f'KeyboardInterrupt must be treated as normal shutdown: {error!r}')

        node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()


class GazeboPoseToOdomTest(TestCase):
    @classmethod
    def setUpClass(cls):
        MODULE.rclpy.init(args=['--ros-args', '-r', '__ns:=/robot_1'])

    @classmethod
    def tearDownClass(cls):
        if MODULE.rclpy.ok():
            MODULE.rclpy.shutdown()

    def setUp(self):
        self.node = MODULE.GazeboPoseToOdom()

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def transform(child_frame_id, x):
        transform = TransformStamped()
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = x
        transform.transform.rotation.w = 1.0
        return transform

    def test_subscribes_to_namespaced_ground_truth_pose(self):
        self.assertEqual(
            self.node.pose_subscription.topic_name,
            '/robot_1/ground_truth/pose',
        )

    def test_selects_transform_for_its_robot(self):
        message = TFMessage()
        message.transforms = [
            self.transform('robot_2', 2.0),
            self.transform('', 3.0),
            self.transform('robot_1', 1.0),
        ]

        self.node.pose_callback(message)

        self.assertEqual(self.node.latest_transform.translation.x, 1.0)

    def test_ignores_other_robot_and_empty_child_frames(self):
        message = TFMessage()
        message.transforms = [
            self.transform('robot_2', 2.0),
            self.transform('', 3.0),
        ]

        self.node.pose_callback(message)

        self.assertIsNone(self.node.latest_transform)

    def test_publishes_existing_odom_and_tf_frame_contract(self):
        self.node.latest_transform = self.transform('robot_1', 1.25).transform
        self.node.odom_publisher = MagicMock()
        self.node.tf_broadcaster = MagicMock()

        self.node.publish_odom()

        odom = self.node.odom_publisher.publish.call_args.args[0]
        self.assertEqual(odom.header.frame_id, 'robot_1/odom')
        self.assertEqual(odom.child_frame_id, 'robot_1/base_footprint')
        self.assertEqual(odom.pose.pose.position.x, 1.25)
        self.assertEqual(odom.twist.twist.linear.x, 0.0)
        self.assertEqual(odom.twist.twist.linear.y, 0.0)
        self.assertEqual(odom.twist.twist.angular.z, 0.0)
        self.assertEqual(list(odom.twist.covariance), list(odom.pose.covariance))

        transform = self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(transform.header.frame_id, 'robot_1/odom')
        self.assertEqual(transform.child_frame_id, 'robot_1/base_footprint')
        self.assertEqual(transform.transform.translation.x, 1.25)


if __name__ == '__main__':
    import unittest

    unittest.main()
