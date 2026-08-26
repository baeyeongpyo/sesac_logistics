import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request
from urllib.request import urlopen
import unittest
import uuid


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts' / 'vehicle_command_api.py'


def load_server_module():
    if not SCRIPT.exists():
        return SimpleNamespace()
    spec = importlib.util.spec_from_file_location('vehicle_command_api_for_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def post_json(url, payload):
    request = Request(
        url,
        data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    try:
        with urlopen(request, timeout=2) as response:
            return response.status, json.load(response)
    except HTTPError as error:
        body = error.read()
        error.close()
        return error.code, json.loads(body)


class RecordingVelocity:
    def __init__(self):
        self.messages = []

    def publish(self, linear_x, angular_z):
        self.messages.append((linear_x, angular_z))


class FakeNavigation:
    def __init__(self, available=True):
        self.available = available
        self.goals = []
        self.cancel_requests = []
        self._callbacks = {}

    def submit_goal(self, operation_id, goal, on_terminal):
        if not self.available:
            return {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}
        self.goals.append((operation_id, goal))
        self._callbacks[operation_id] = on_terminal
        return {'accepted': True}

    def cancel(self, operation_id):
        self.cancel_requests.append(operation_id)
        return {'accepted': operation_id in self._callbacks}

    def complete(self, operation_id, state):
        self._callbacks[operation_id](operation_id, state)


class ClosingFakeAdapter(FakeNavigation, RecordingVelocity):
    def __init__(self):
        FakeNavigation.__init__(self)
        RecordingVelocity.__init__(self)
        self.closed = False

    def close(self):
        self.closed = True


class ReturningHttpServer:
    def __init__(self):
        self.served = False
        self.closed = False

    def serve_forever(self):
        self.served = True

    def server_close(self):
        self.closed = True


class VehicleCommandApiServerTest(unittest.TestCase):
    def setUp(self):
        self.module = load_server_module()
        self.assertTrue(
            hasattr(self.module, 'VehicleCommandService'),
            'the standalone vehicle command service must exist',
        )
        self.velocity = RecordingVelocity()
        self.navigation = FakeNavigation()
        self.service = self.module.VehicleCommandService(
            velocity=self.velocity,
            navigation=self.navigation,
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
        )
        self.server = self.module.create_http_server('127.0.0.1', 0, self.service)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f'http://127.0.0.1:{self.server.server_address[1]}'

    def tearDown(self):
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=2)
        if hasattr(self, 'service'):
            self.service.close()

    def get_json(self, path):
        with urlopen(f'{self.base_url}{path}', timeout=2) as response:
            return response.status, json.load(response)

    def navigation_goal(self):
        return post_json(
            f'{self.base_url}/v1/navigation/goals',
            {'frame_id': 'map', 'x': 1.50, 'y': 0.0, 'yaw': 0.0},
        )

    def test_health_openapi_and_operation_status_are_discoverable(self):
        """Removing a public endpoint must fail the vehicle integration contract."""
        health_status, health = self.get_json('/healthz')
        openapi_status, openapi = self.get_json('/openapi.json')
        status_code, operation_status = self.get_json('/v1/operation-status')

        self.assertEqual(health_status, 200)
        self.assertEqual(health, {'status': 'ok'})
        self.assertEqual(openapi_status, 200)
        self.assertEqual(openapi['openapi'], '3.0.3')
        self.assertEqual(
            set(openapi['paths']),
            {
                '/healthz',
                '/openapi.json',
                '/v1/operation-status',
                '/v1/cmd-vel',
                '/v1/navigation/goals',
                '/v1/navigation/cancel',
                '/v1/stop',
            },
        )
        self.assertEqual(status_code, 200)
        self.assertEqual(operation_status, {
            'operation_id': None,
            'state': 'IDLE',
            'detail': 'READY',
        })

    def test_navigation_goal_returns_vehicle_generated_operation_id_and_tracks_goal(self):
        """Removing vehicle-side IDs or changing coordinates must fail the navigation contract."""
        status, body = self.navigation_goal()

        self.assertEqual(status, 202)
        self.assertEqual(body['state'], 'NAVIGATING')
        operation_id = body['operation_id']
        self.assertIsInstance(uuid.UUID(operation_id), uuid.UUID)
        self.assertEqual(self.navigation.goals, [(
            operation_id,
            {'frame_id': 'map', 'x': 1.5, 'y': 0.0, 'yaw': 0.0},
        )])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': operation_id,
            'state': 'NAVIGATING',
            'detail': 'NAVIGATION_GOAL_ACCEPTED',
        })

    def test_navigation_terminal_result_updates_the_current_operation_status(self):
        """Dropping Nav2 terminal callbacks must fail status monitoring."""
        _, body = self.navigation_goal()
        self.navigation.complete(body['operation_id'], 'COMPLETED')

        status, operation_status = self.get_json('/v1/operation-status')

        self.assertEqual(status, 200)
        self.assertEqual(operation_status, {
            'operation_id': body['operation_id'],
            'state': 'COMPLETED',
            'detail': 'NAVIGATION_SUCCEEDED',
        })

    def test_cancel_marks_target_operation_cancelled_after_nav2_result(self):
        """Ignoring the requested operation ID must fail targeted cancellation."""
        _, body = self.navigation_goal()
        status, response = post_json(
            f'{self.base_url}/v1/navigation/cancel',
            {'operation_id': body['operation_id']},
        )
        self.navigation.complete(body['operation_id'], 'CANCELLED')
        _, operation_status = self.get_json('/v1/operation-status')

        self.assertEqual(status, 202)
        self.assertEqual(response, {
            'operation_id': body['operation_id'],
            'state': 'CANCELLING',
        })
        self.assertEqual(self.navigation.cancel_requests, [body['operation_id']])
        self.assertEqual(operation_status, {
            'operation_id': body['operation_id'],
            'state': 'CANCELLED',
            'detail': 'NAVIGATION_CANCELLED',
        })

    def test_stop_zeroes_velocity_before_requesting_navigation_cancel(self):
        """Removing the zero command before cancel must fail the immediate stop contract."""
        _, body = self.navigation_goal()
        post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.08, 'angular_z': 0.10, 'hold_ms': 1000},
        )

        status, response = post_json(f'{self.base_url}/v1/stop', {})

        self.assertEqual(status, 200)
        self.assertEqual(response, {
            'operation_id': body['operation_id'],
            'state': 'STOPPED',
            'cancel_requested': True,
        })
        self.assertEqual(self.velocity.messages[-1], (0.0, 0.0))
        self.assertEqual(self.navigation.cancel_requests, [body['operation_id'], body['operation_id']])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': body['operation_id'],
            'state': 'STOPPED',
            'detail': 'STOP_REQUESTED',
        })

    def test_invalid_goal_and_unavailable_navigation_do_not_start_an_operation(self):
        """Removing goal validation or treating unavailable Nav2 as accepted must fail safely."""
        status, body = post_json(
            f'{self.base_url}/v1/navigation/goals',
            {'frame_id': 'odom', 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        )

        self.assertEqual(status, 422)
        self.assertEqual(body, {'error': 'frame_id must be map'})
        self.assertEqual(self.navigation.goals, [])

        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.navigation = FakeNavigation(available=False)
        self.service.close()
        self.service = self.module.VehicleCommandService(
            velocity=self.velocity,
            navigation=self.navigation,
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
        )
        self.server = self.module.create_http_server('127.0.0.1', 0, self.service)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f'http://127.0.0.1:{self.server.server_address[1]}'

        status, body = self.navigation_goal()

        self.assertEqual(status, 503)
        self.assertEqual(body, {'error': 'NAVIGATION_SERVER_UNAVAILABLE'})
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': None,
            'state': 'FAILED',
            'detail': 'NAVIGATION_SERVER_UNAVAILABLE',
        })

    def test_manual_velocity_is_bounded_and_expires_to_idle(self):
        """Removing command bounds or the hold-expiry zero must fail direct control safety."""
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.05, 'angular_z': -0.25, 'hold_ms': 20},
        )

        self.assertEqual(status, 202)
        self.assertEqual(body, {
            'state': 'MANUAL',
            'linear_x': 0.05,
            'angular_z': -0.25,
            'hold_ms': 20,
        })
        deadline = time.monotonic() + 1
        while len(self.velocity.messages) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.velocity.messages, [(0.05, -0.25), (0.0, 0.0)])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': None,
            'state': 'IDLE',
            'detail': 'MANUAL_COMMAND_EXPIRED',
        })

    def test_manual_velocity_cancels_an_active_navigation_before_direct_control(self):
        """Allowing manual velocity while Nav2 stays active must fail control handoff safety."""
        _, body = self.navigation_goal()

        status, response = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.05, 'angular_z': 0.0, 'hold_ms': 1000},
        )

        self.assertEqual(status, 202)
        self.assertEqual(response['state'], 'MANUAL')
        self.assertEqual(self.navigation.cancel_requests, [body['operation_id']])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': None,
            'state': 'MANUAL',
            'detail': 'MANUAL_COMMAND_SENT',
        })


class VehicleCommandApiCliTest(unittest.TestCase):
    def setUp(self):
        self.module = load_server_module()

    def test_cli_exposes_direct_runner_and_action_timeout_configuration(self):
        """Removing a runtime flag must fail the vehicle deployment contract."""
        result = subprocess.run(
            [sys.executable, str(SCRIPT), '--help'],
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        for option in (
            '--host', '--port', '--cmd-vel-topic', '--action-name',
            '--max-linear-x', '--max-angular-z', '--max-hold-ms',
            '--action-server-timeout-sec', '--goal-response-timeout-sec',
            '--cancel-response-timeout-sec',
        ):
            self.assertIn(option, result.stdout)

    def test_direct_runner_zeroes_velocity_and_closes_its_adapter(self):
        """Removing shutdown zeroing or adapter cleanup must fail the process lifecycle contract."""
        self.assertTrue(
            hasattr(self.module, 'run_server'),
            'the standalone direct Python runner must exist',
        )
        adapter = ClosingFakeAdapter()
        http_server = ReturningHttpServer()
        arguments = SimpleNamespace(
            host='0.0.0.0',
            port=8082,
            cmd_vel_topic='/cmd_vel',
            action_name='/navigate_to_pose',
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
            action_server_timeout_sec=1.0,
            goal_response_timeout_sec=3.0,
            cancel_response_timeout_sec=3.0,
        )

        self.module.run_server(
            arguments,
            adapter_factory=lambda args: adapter,
            http_server_factory=lambda host, port, service: http_server,
        )

        self.assertTrue(http_server.served)
        self.assertTrue(http_server.closed)
        self.assertEqual(adapter.messages, [(0.0, 0.0)])
        self.assertTrue(adapter.closed)


if __name__ == '__main__':
    unittest.main()
