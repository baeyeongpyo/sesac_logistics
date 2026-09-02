import unittest

from operations.control_center.app.command_relay import CommandRelay, RelayResponse


class FakeTransport:
    def __init__(self):
        self.calls = []

    def __call__(self, method, url, payload, timeout):
        self.calls.append(
            {
                'method': method,
                'url': url,
                'payload': payload,
                'timeout': timeout,
            }
        )
        return RelayResponse(status_code=202, body={'state': 'ACCEPTED'})


class CommandRelayTests(unittest.TestCase):
    def setUp(self):
        self.transport = FakeTransport()
        self.relay = CommandRelay(
            'http://fleet-bridge:8080/',
            request_timeout_sec=2.5,
            transport=self.transport,
        )

    def test_manual_command_forwards_native_lateral_velocity(self):
        response = self.relay.send_manual(
            'R1', linear_x=0.0, linear_y=0.8, angular_z=0.0,
        )

        self.assertEqual(response.status_code, 202)
        self.assertEqual(
            self.transport.calls,
            [
                {
                    'method': 'POST',
                    'url': 'http://fleet-bridge:8080/api/v1/vehicle-command/R1/cmd-vel',
                    'payload': {
                        'linear_x': 0.0,
                        'linear_y': 0.8,
                        'angular_z': 0.0,
                        'hold_ms': 300,
                    },
                    'timeout': 2.5,
                }
            ],
        )

    def test_initial_pose_and_idle_use_their_allowlisted_vehicle_api_paths(self):
        self.relay.send_initial_pose('R1', x=1.2, y=-0.3, yaw=0.75)
        self.relay.send_operation_idle('R1')

        self.assertEqual(
            [call['url'] for call in self.transport.calls],
            [
                'http://fleet-bridge:8080/api/v1/vehicle-command/R1/localization/initial-pose',
                'http://fleet-bridge:8080/api/v1/vehicle-command/R1/operation/idle',
            ],
        )
        self.assertEqual(
            self.transport.calls[0]['payload'],
            {'frame_id': 'map', 'x': 1.2, 'y': -0.3, 'yaw': 0.75},
        )
        self.assertEqual(
            self.transport.calls[1]['payload'],
            {'reason': 'OPERATOR_CONFIRMED'},
        )

    def test_invalid_control_input_never_reaches_transport(self):
        with self.assertRaises(ValueError):
            self.relay.send_manual('R1', linear_x=0.1, angular_z=0.0, hold_ms=50)
        with self.assertRaises(ValueError):
            self.relay.send_manual('R1', linear_x=0.0, linear_y=1.1, angular_z=0.0)
        with self.assertRaisesRegex(ValueError, '10 degrees'):
            self.relay.send_manual('R1', linear_x=0.0, linear_y=0.0, angular_z=1.0, hold_ms=300)
        with self.assertRaises(ValueError):
            self.relay.send_initial_pose('R1', x=0.0, y=0.0, yaw=4.0)
        with self.assertRaises(ValueError):
            self.relay.send_stop('../inventory.db')

        self.assertEqual(self.transport.calls, [])
