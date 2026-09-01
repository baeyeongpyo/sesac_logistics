"""ROS 2 adapter tests, run in the server image where rclpy is installed."""

import importlib.util
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

HAS_RCLPY = importlib.util.find_spec('rclpy') is not None

if HAS_RCLPY:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    from fleet_bridge_worker.telemetry import (
        FleetTelemetryWriter,
        TelemetryStore,
    )


@unittest.skipUnless(HAS_RCLPY, 'requires the ROS 2 server image')
class FleetTelemetryWriterRosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.temporary_directory = TemporaryDirectory()
        self.store = TelemetryStore(Path(self.temporary_directory.name) / 'telemetry.db')
        self.now = [100.0]
        self.writer = FleetTelemetryWriter(
            self.store,
            ('robot_1', 'robot_2'),
            pose_write_rate_hz=5.0,
            tf_freshness_timeout_sec=10.0,
            monotonic_time=lambda: self.now[0],
            received_at=lambda: '2026-08-31T00:00:00Z',
        )

    def tearDown(self):
        self.writer.destroy_node()
        self.temporary_directory.cleanup()
        rclpy.shutdown()

    def transform(self, parent, child, *, x=0.0, y=0.0, yaw=0.0):
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.header.stamp.sec = 12
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        return transform

    def record_robot_1_pose(self):
        self.writer._dynamic_tf_callback('robot_1', TFMessage(transforms=[
            self.transform('map', 'robot_1/odom', x=10.0, y=5.0, yaw=0.1),
            self.transform(
                'robot_1/odom',
                'robot_1/base_footprint',
                x=1.0,
                yaw=0.2,
            ),
        ]))
        self.writer._record_all_poses()

    def test_rejects_a_different_robot_frame_from_the_source_topic(self):
        self.record_robot_1_pose()

        self.writer._dynamic_tf_callback('robot_2', TFMessage(transforms=[
            self.transform('map', 'robot_1/odom', x=999.0),
        ]))
        self.writer._record_all_poses()

        pose = self.store.fetch_pose('robot_1')
        self.assertAlmostEqual(pose['x_m'], 10.9950041653)
        self.assertAlmostEqual(pose['y_m'], 5.0998334166)
        self.assertAlmostEqual(pose['yaw_rad'], 0.3)
        self.assertEqual(pose['received_at'], '2026-08-31T00:00:00Z')
        self.assertFalse(self.writer._tf_freshness.is_fresh('robot_2'))

    def test_stale_tf_does_not_overwrite_the_last_current_pose(self):
        self.record_robot_1_pose()
        initial_pose = self.store.fetch_pose('robot_1')

        self.now[0] = 110.1
        self.writer._record_all_poses()

        self.assertEqual(self.store.fetch_pose('robot_1'), initial_pose)


if __name__ == '__main__':
    unittest.main()
