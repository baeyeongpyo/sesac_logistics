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
            'robot_1', 'physical', 1, '/robot_1', 'mentorpi', True,
        )

    def test_telemetry_flows_only_from_vehicle_domain_to_central_domain(self):
        document = bridge_document(self.robot_1, control_domain=215)

        odom = document['topics']['/robot_1/odom']
        self.assertEqual(odom['type'], 'nav_msgs/msg/Odometry')
        self.assertEqual(odom['from_domain'], 1)
        self.assertEqual(odom['to_domain'], 215)
        self.assertNotIn('/robot_1/manual/cmd_vel', [
            name for name, config in document['topics'].items()
            if config['from_domain'] == 1
        ])

    def test_commands_flow_only_from_central_domain_to_the_same_vehicle_domain(self):
        document = bridge_document(self.robot_1, control_domain=215)

        manual = document['topics']['/robot_1/manual/cmd_vel']
        self.assertEqual(manual['type'], 'geometry_msgs/msg/Twist')
        self.assertEqual(manual['from_domain'], 215)
        self.assertEqual(manual['to_domain'], 1)
        self.assertNotIn('/robot_2/manual/cmd_vel', document['topics'])
        self.assertNotIn('/cmd_vel', document['topics'])

    def test_config_is_written_atomically_as_valid_yaml(self):
        with TemporaryDirectory() as directory:
            target = Path(directory) / 'robot_1' / 'domain_bridge.yaml'
            written = write_bridge_config(self.robot_1, 215, target)
            parsed = yaml.safe_load(written.read_text())

        self.assertEqual(written, target)
        self.assertEqual(parsed['topics']['/robot_1/safety/stop']['to_domain'], 1)


if __name__ == '__main__':
    unittest.main()
