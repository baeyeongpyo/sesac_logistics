from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from fleet_bridge_config.models import (
    CommandApiConfig,
    FleetConfig,
    ServerConfig,
    VehicleConfig,
)
from foxglove_ros_worker.api import create_app
from foxglove_ros_worker.command import (
    VehicleCommandApiError,
    VehicleCommandResponse,
    VehicleCommandTransportError,
)


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
            command_api_url='http://10.0.0.11:8082',
            enabled=enabled,
        ),),
    )


class RecordingVehicleApiClient:
    def __init__(self, error=None, response=None):
        self.error = error
        self.response = response or VehicleCommandResponse(
            status_code=200,
            body={'vehicle_response': 'preserved'},
        )
        self.calls = []

    async def relay(self, vehicle, method, path, payload=None):
        if self.error:
            raise self.error
        self.calls.append((vehicle.id, method, path, payload))
        return self.response


class CommandApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from fastapi.testclient import TestClient

        cls.TestClient = TestClient

    def client(self, *, enabled=True, error=None, response=None):
        vehicle_api = RecordingVehicleApiClient(error=error, response=response)
        return self.TestClient(create_app(fleet(enabled=enabled), vehicle_api)), vehicle_api

    def test_vehicle_command_relay_exposes_every_vehicle_api_path(self):
        client, vehicle_api = self.client(
            response=VehicleCommandResponse(
                status_code=202,
                body={'operation_id': 'vehicle-generated', 'state': 'ACCEPTED'},
            ),
        )
        requests = [
            ('GET', '/healthz', None, '/healthz'),
            ('GET', '/openapi.json', None, '/openapi.json'),
            ('GET', '/operation-status', None, '/v1/operation-status'),
            ('GET', '/vehicle-status', None, '/v1/vehicle-status'),
            ('POST', '/cmd-vel', {
                'linear_x': 0.1,
                'angular_z': 0.0,
                'hold_ms': 300,
            }, '/v1/cmd-vel'),
            ('POST', '/navigation/goals', {
                'x': 1.5,
                'y': 0.0,
                'yaw': 0.0,
            }, '/v1/navigation/goals'),
            ('POST', '/navigation/cancel', {
                'operation_id': 'vehicle-generated',
            }, '/v1/navigation/cancel'),
            ('POST', '/localization/initial-pose', {
                'x': 1.5,
                'y': 0.0,
                'yaw': 0.0,
            }, '/v1/localization/initial-pose'),
            ('POST', '/stop', None, '/v1/stop'),
        ]

        for method, suffix, payload, _vehicle_path in requests:
            with self.subTest(method=method, suffix=suffix):
                response = client.request(
                    method,
                    f'/api/v1/vehicle-command/robot_1{suffix}',
                    json=payload,
                )
                self.assertEqual(response.status_code, 202)
                self.assertEqual(response.json(), {
                    'operation_id': 'vehicle-generated',
                    'state': 'ACCEPTED',
                })

        self.assertEqual(vehicle_api.calls, [
            ('robot_1', method, vehicle_path, payload)
            for method, _suffix, payload, vehicle_path in requests
        ])

    def test_vehicle_command_relay_preserves_vehicle_error_response(self):
        client, _vehicle_api = self.client(
            error=VehicleCommandApiError(
                409,
                'VEHICLE_MOTION_ACTIVE',
                {'error': 'VEHICLE_MOTION_ACTIVE'},
            ),
        )

        response = client.post(
            '/api/v1/vehicle-command/robot_1/localization/initial-pose',
            json={'x': 1.5, 'y': 0.0, 'yaw': 0.0},
        )

        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json(), {'error': 'VEHICLE_MOTION_ACTIVE'})

    def test_vehicle_command_relay_returns_service_unavailable_for_transport_error(self):
        client, _vehicle_api = self.client(
            error=VehicleCommandTransportError('vehicle unavailable'),
        )

        response = client.post('/api/v1/vehicle-command/robot_1/stop')

        self.assertEqual(response.status_code, 503)

    def test_vehicle_command_relay_rejects_unknown_or_disabled_robot(self):
        client, _vehicle_api = self.client()
        unknown = client.post('/api/v1/vehicle-command/robot_x/stop')
        disabled, _vehicle_api = self.client(enabled=False)
        disabled_response = disabled.post('/api/v1/vehicle-command/robot_1/stop')

        self.assertEqual(unknown.status_code, 404)
        self.assertEqual(disabled_response.status_code, 404)

    def test_legacy_robot_command_paths_are_not_exposed(self):
        client, _vehicle_api = self.client()
        requests = [
            ('/api/v1/robots/robot_1/cmd_vel', {
                'linear_x': 0.1,
                'angular_z': 0.0,
                'hold_ms': 300,
            }),
            ('/api/v1/robots/robot_1/nav2/goal_pose', {
                'header': {
                    'stamp': {'sec': 0, 'nanosec': 0},
                    'frame_id': 'map',
                },
                'pose': {
                    'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                },
            }),
            ('/api/v1/robots/robot_1/nav2/cancel', None),
            ('/api/v1/robots/robot_1/stop', None),
        ]

        for path, payload in requests:
            with self.subTest(path=path):
                response = client.post(path, json=payload)
                self.assertEqual(response.status_code, 404)

        schema = client.get('/openapi.json').json()
        for path, _payload in requests:
            with self.subTest(path=path):
                self.assertNotIn(path.replace('robot_1', '{robot_id}'), schema['paths'])

    def test_docs_expose_vehicle_command_relay(self):
        client, _vehicle_api = self.client()
        docs = client.get('/docs')
        schema = client.get('/openapi.json').json()

        self.assertEqual(client.get('/healthz').json(), {'status': 'ok'})
        self.assertEqual(docs.status_code, 200)
        self.assertIn('swagger-ui', docs.text)
        self.assertIn('/api/v1/vehicle-command/{robot_id}/healthz', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/openapi.json', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/operation-status', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/vehicle-status', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/cmd-vel', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/navigation/goals', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/navigation/cancel', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/localization/initial-pose', schema['paths'])
        self.assertIn('/api/v1/vehicle-command/{robot_id}/stop', schema['paths'])

    def test_openapi_describes_relay_and_provides_vehicle_native_request_examples(self):
        """A missing relay explanation or sample payload would leave Swagger unusable."""

        client, _vehicle_api = self.client()
        schema = client.get('/openapi.json').json()

        self.assertIn('중계', schema['info']['description'])
        self.assertEqual(
            schema['tags'][0],
            {
                'name': 'vehicle-command relay',
                'description': (
                    'Fleet Manager는 등록된 차량의 vehicle_command_api로 요청과 응답을 '
                    '변경하지 않고 중계합니다.'
                ),
            },
        )

        prefix = '/api/v1/vehicle-command/{robot_id}'
        expected_examples = {
            '/cmd-vel': {
                'linear_x': 0.2,
                'angular_z': 0.0,
                'hold_ms': 500,
            },
            '/navigation/goals': {
                'frame_id': 'map',
                'x': 1.5,
                'y': 0.0,
                'yaw': 0.0,
            },
            '/navigation/cancel': {
                'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
            },
            '/localization/initial-pose': {
                'frame_id': 'map',
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
            },
        }
        for suffix, expected_value in expected_examples.items():
            with self.subTest(suffix=suffix):
                operation = schema['paths'][prefix + suffix]['post']
                self.assertIn('차량', operation['description'])
                self.assertEqual(
                    operation['parameters'][0]['description'],
                    '명령을 전달하거나 상태를 조회할 차량 ID입니다.',
                )
                examples = operation['requestBody']['content']['application/json']['examples']
                self.assertIn('sample', examples)
                self.assertEqual(examples['sample']['value'], expected_value)

        stop = schema['paths'][prefix + '/stop']['post']
        self.assertIn('요청 본문 없이', stop['description'])
        self.assertEqual(
            stop['responses']['200']['content']['application/json']['example'],
            {
                'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
                'state': 'STOPPED',
                'cancel_requested': True,
            },
        )

        operation_status = schema['paths'][prefix + '/operation-status']['get']
        self.assertEqual(
            operation_status['responses']['200']['content']['application/json']['example'],
            {
                'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
                'state': 'NAVIGATING',
                'detail': 'NAVIGATION_GOAL_ACCEPTED',
            },
        )


if __name__ == '__main__':
    unittest.main()
