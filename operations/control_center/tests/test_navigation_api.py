from pathlib import Path
import unittest

from operations.control_center.app.command_relay import RelayResponse
from operations.control_center.app.main import create_app
from operations.control_center.app.settings import Settings


class RecordingRelay:
    def __init__(self):
        self.calls = []

    def send_navigation_goal(self, robot_id, *, x, y, yaw):
        self.calls.append(('goal', robot_id, {'x': x, 'y': y, 'yaw': yaw}))
        return RelayResponse(202, {'state': 'DRIVE'})

    def send_navigation_waypoints(self, robot_id, *, waypoints):
        self.calls.append(('waypoints', robot_id, {'waypoints': waypoints}))
        return RelayResponse(202, {'state': 'DRIVE'})

    def send_fork_up(self, robot_id):
        self.calls.append(('fork_up', robot_id))
        return RelayResponse(202, {'command': 'UP', 'state': 'FORK_COMMAND_PUBLISHED'})

    def send_fork_down(self, robot_id):
        self.calls.append(('fork_down', robot_id))
        return RelayResponse(202, {'command': 'DOWN', 'state': 'FORK_COMMAND_PUBLISHED'})


class NavigationApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from fastapi.testclient import TestClient

        cls.TestClient = TestClient

    def setUp(self):
        self.relay = RecordingRelay()
        self.client = self.TestClient(create_app(
            Settings(
                data_directory=Path('/unused-data'),
                map_directory=Path('/unused-map'),
                fleet_bridge_command_url='http://fleet-bridge:8080',
                request_timeout_sec=3.0,
            ),
            relay=self.relay,
        ))

    def test_selected_goal_and_ordered_waypoints_are_relayed_as_navigation_requests(self):
        goal = self.client.post(
            '/api/vehicles/robot_1/navigation/goal',
            json={'x': 1.2, 'y': -0.3, 'yaw': 0.75},
        )
        waypoints = self.client.post(
            '/api/vehicles/robot_1/navigation/waypoints',
            json={'waypoints': [
                {'x': 1.2, 'y': -0.3, 'yaw': 0.75},
                {'x': 2.0, 'y': 0.5, 'yaw': -0.25},
            ]},
        )

        self.assertEqual(goal.status_code, 202)
        self.assertEqual(waypoints.status_code, 202)
        self.assertEqual(
            self.relay.calls,
            [
                ('goal', 'robot_1', {'x': 1.2, 'y': -0.3, 'yaw': 0.75}),
                ('waypoints', 'robot_1', {'waypoints': [
                    {'x': 1.2, 'y': -0.3, 'yaw': 0.75},
                    {'x': 2.0, 'y': 0.5, 'yaw': -0.25},
                ]}),
            ],
        )

    def test_fork_controls_relay_to_the_selected_vehicle(self):
        up = self.client.post('/api/vehicles/robot_1/fork/up')
        down = self.client.post('/api/vehicles/robot_1/fork/down')

        self.assertEqual(up.status_code, 202)
        self.assertEqual(down.status_code, 202)
        self.assertEqual(
            self.relay.calls,
            [('fork_up', 'robot_1'), ('fork_down', 'robot_1')],
        )
