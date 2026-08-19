from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from mentorpi_fleet.bridge_config import bridge_document, write_bridge_config
from mentorpi_fleet.registry import VehicleSpec


class BridgeConfigTest(unittest.TestCase):
    def setUp(self):
        self.robot_1 = VehicleSpec(
            'robot_1', 'physical', 215, '/robot_1', 'mentorpi', True,
        )

    def test_bridges_rosbag_telemetry_from_vehicle_to_central_domain(self):
        document = bridge_document(self.robot_1, control_domain=225)

        expected = {
            '/robot_1/odom': 'nav_msgs/msg/Odometry',
            '/robot_1/scan_raw': 'sensor_msgs/msg/LaserScan',
            '/robot_1/imu/data_raw': 'sensor_msgs/msg/Imu',
            '/robot_1/depth/image_raw': 'sensor_msgs/msg/Image',
            '/robot_1/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
            '/robot_1/controller/cmd_vel': 'geometry_msgs/msg/Twist',
        }
        for name, message_type in expected.items():
            with self.subTest(topic=name):
                self.assertEqual(document['topics'][name], {
                    'type': message_type,
                    'from_domain': 215,
                    'to_domain': 225,
                })
        self.assertNotIn('/robot_1/manual/cmd_vel', [
            name for name, config in document['topics'].items()
            if config['from_domain'] == 1
        ])

    def test_bridges_only_safe_commands_from_central_to_matching_vehicle_domain(self):
        document = bridge_document(self.robot_1, control_domain=225)

        for name in ('/robot_1/manual/cmd_vel', '/robot_1/cmd_vel_nav', '/robot_1/safety/stop'):
            with self.subTest(topic=name):
                self.assertEqual(document['topics'][name]['from_domain'], 225)
                self.assertEqual(document['topics'][name]['to_domain'], 215)
        self.assertNotIn('/robot_2/manual/cmd_vel', document['topics'])
        self.assertNotIn('/cmd_vel', document['topics'])
        self.assertNotIn('/robot_1/navigate_to_pose', document['topics'])
        self.assertEqual(document['topics']['/robot_1/controller/cmd_vel']['from_domain'], 215)

    def test_config_is_written_atomically_as_valid_yaml(self):
        with TemporaryDirectory() as directory:
            target = Path(directory) / 'robot_1' / 'domain_bridge.yaml'
            written = write_bridge_config(self.robot_1, 225, target)
            parsed = yaml.safe_load(written.read_text())

        self.assertEqual(written, target)
        self.assertEqual(parsed['topics']['/robot_1/safety/stop']['to_domain'], 215)


if __name__ == '__main__':
    unittest.main()
