from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
BUNDLE = PACKAGE.parents[3]
COMMON = BUNDLE / 'common/fleet_bridge_config'
sys.path[:0] = [str(PACKAGE), str(COMMON)]

from fleet_telemetry_filter.node import cleanup_rclpy


class RosInitializationTest(unittest.TestCase):
    def test_interrupt_cleanup_does_not_destroy_an_already_interrupted_node(self):
        class FakeNode:
            def __init__(self):
                self.destroy_count = 0

            def destroy_node(self):
                self.destroy_count += 1

        class FakeRclpy:
            def __init__(self):
                self.shutdown_count = 0

            def ok(self):
                return True

            def shutdown(self):
                self.shutdown_count += 1

        node = FakeNode()
        rclpy = FakeRclpy()

        cleanup_rclpy(node, rclpy, interrupted=True)

        self.assertEqual(node.destroy_count, 0)
        self.assertEqual(rclpy.shutdown_count, 1)


if __name__ == '__main__':
    unittest.main()
