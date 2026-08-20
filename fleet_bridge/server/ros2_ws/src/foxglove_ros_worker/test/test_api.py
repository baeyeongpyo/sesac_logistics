from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from fleet_bridge_config.models import (
    CommandApiConfig,
    CommandConfig,
    FleetConfig,
    ServerConfig,
    VehicleConfig,
)
from foxglove_ros_worker.api import create_app


def fleet(*, enabled=True):
    return FleetConfig(
        server=ServerConfig(
            domain_id=225,
            foxglove_port=8765,
            command_api=CommandApiConfig(host='127.0.0.1', port=8080),
        ),
        vehicles=(VehicleConfig(
            id='robot_1',
            foxglove_uri='ws://10.0.0.11:8766',
            enabled=enabled,
            command=CommandConfig(
                topic='/cmd_vel',
                message_type='geometry_msgs/msg/Twist',
                max_linear_x=0.3,
                max_angular_z=1.0,
                max_hold_ms=1000,
                publish_rate_hz=10.0,
            ),
        ),),
    )


class RecordingCommandClient:
    def __init__(self, error=None):
        self.error = error
        self.twists = []
        self.stops = []

    async def send_twist(self, vehicle, linear_x, angular_z, hold_ms):
        if self.error:
            raise self.error
        self.twists.append((vehicle.id, linear_x, angular_z, hold_ms))

    async def stop(self, vehicle):
        if self.error:
            raise self.error
        self.stops.append(vehicle.id)


class CommandApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from fastapi.testclient import TestClient

        cls.TestClient = TestClient

    def client(self, *, enabled=True, error=None):
        commands = RecordingCommandClient(error=error)
        return self.TestClient(create_app(fleet(enabled=enabled), commands)), commands

    def test_cmd_vel_returns_accepted_and_delivers_valid_request(self):
        client, commands = self.client()

        response = client.post('/api/v1/robots/robot_1/cmd_vel', json={
            'linear_x': 0.1,
            'angular_z': 0.0,
            'hold_ms': 300,
        })

        self.assertEqual(response.status_code, 202)
        self.assertEqual(response.json()['robot_id'], 'robot_1')
        self.assertEqual(commands.twists, [('robot_1', 0.1, 0.0, 300)])

    def test_cmd_vel_rejects_invalid_or_unsafe_request(self):
        client, _commands = self.client()

        for payload in (
            {'linear_x': 0.1, 'angular_z': 0.0, 'hold_ms': 0},
            {'linear_x': 0.31, 'angular_z': 0.0, 'hold_ms': 300},
        ):
            with self.subTest(payload=payload):
                response = client.post('/api/v1/robots/robot_1/cmd_vel', json=payload)
                self.assertEqual(response.status_code, 422)

    def test_api_rejects_unknown_or_disabled_robot(self):
        client, _commands = self.client()
        unknown = client.post('/api/v1/robots/robot_x/stop')
        disabled, _commands = self.client(enabled=False)
        disabled_response = disabled.post('/api/v1/robots/robot_1/stop')

        self.assertEqual(unknown.status_code, 404)
        self.assertEqual(disabled_response.status_code, 404)

    def test_api_returns_service_unavailable_when_delivery_fails(self):
        client, _commands = self.client(error=ConnectionError('vehicle unavailable'))

        response = client.post('/api/v1/robots/robot_1/stop')

        self.assertEqual(response.status_code, 503)

    def test_stop_docs_and_openapi_are_available(self):
        client, commands = self.client()
        stop = client.post('/api/v1/robots/robot_1/stop')
        docs = client.get('/docs')
        schema = client.get('/openapi.json').json()

        self.assertEqual(stop.status_code, 202)
        self.assertEqual(commands.stops, ['robot_1'])
        self.assertEqual(client.get('/healthz').json(), {'status': 'ok'})
        self.assertEqual(docs.status_code, 200)
        self.assertIn('swagger-ui', docs.text)
        self.assertIn('/api/v1/robots/{robot_id}/cmd_vel', schema['paths'])
        self.assertIn('/api/v1/robots/{robot_id}/stop', schema['paths'])


if __name__ == '__main__':
    unittest.main()
