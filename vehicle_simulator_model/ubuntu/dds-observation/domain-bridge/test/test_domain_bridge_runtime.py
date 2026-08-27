import json
from pathlib import Path
import sys
import unittest


RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

from publish_bridge_status import status_payload
from render_bridge_config import render_bridge_config


class DomainBridgeRuntimeTest(unittest.TestCase):
    def test_renderer_prefixes_telemetry_and_restores_command_namespace(self):
        document = render_bridge_config(
            vehicle_domain=215,
            control_domain=225,
            source_namespace='/',
            central_prefix='/robot_1',
        )

        self.assertEqual(document['topics']['/odom'], {
            'type': 'nav_msgs/msg/Odometry',
            'from_domain': 215,
            'to_domain': 225,
            'remap': '/robot_1/odom',
        })
        self.assertEqual(document['topics']['/robot_1/manual/cmd_vel'], {
            'type': 'geometry_msgs/msg/Twist',
            'from_domain': 225,
            'to_domain': 215,
            'remap': '/manual/cmd_vel',
        })
        self.assertNotIn('/robot_2/odom', document['topics'])
        self.assertNotIn('/cmd_vel', document['topics'])

    def test_status_payload_contains_only_state_and_timestamp(self):
        payload = json.loads(status_payload('active', '2026-08-19T12:00:00Z'))

        self.assertEqual(payload, {
            'state': 'active',
            'timestamp': '2026-08-19T12:00:00Z',
        })


if __name__ == '__main__':
    unittest.main()
