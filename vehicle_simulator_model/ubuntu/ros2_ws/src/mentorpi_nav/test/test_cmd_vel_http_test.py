import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request
from urllib.request import urlopen


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts' / 'cmd_vel_http_test.py'


def load_server_module():
    spec = importlib.util.spec_from_file_location('cmd_vel_http_test_for_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, linear_x, angular_z):
        self.messages.append((linear_x, angular_z))


class ClosingRecordingPublisher(RecordingPublisher):
    def __init__(self, topic):
        super().__init__()
        self.topic = topic
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
        try:
            return error.code, json.loads(body)
        except json.JSONDecodeError:
            return error.code, {'error': 'non_json_error'}


class CmdVelHttpTestCliTest(unittest.TestCase):
    def test_cli_exposes_direct_http_and_cmd_vel_configuration(self):
        """Removing a direct-run argument must fail this deployment contract."""
        result = subprocess.run(
            [sys.executable, str(SCRIPT), '--help'],
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--host', result.stdout)
        self.assertIn('--port', result.stdout)
        self.assertIn('--topic', result.stdout)
        self.assertIn('--max-linear-x', result.stdout)
        self.assertIn('--max-angular-z', result.stdout)
        self.assertIn('--max-hold-ms', result.stdout)


class CmdVelHttpTestServerTest(unittest.TestCase):
    def setUp(self):
        self.module = load_server_module()
        self.publisher = RecordingPublisher()

        self.assertTrue(
            hasattr(self.module, 'CmdVelCommandService'),
            'the HTTP command service must exist',
        )
        self.service = self.module.CmdVelCommandService(
            publisher=self.publisher,
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

    def test_health_and_openapi_are_available_without_ros_launch(self):
        """Removing either public discovery endpoint must fail this HTTP contract."""
        with urlopen(f'{self.base_url}/healthz', timeout=2) as response:
            health = json.load(response)
            health_status = response.status
        with urlopen(f'{self.base_url}/openapi.json', timeout=2) as response:
            openapi = json.load(response)
            openapi_status = response.status

        self.assertEqual(health_status, 200)
        self.assertEqual(health, {'status': 'ok'})
        self.assertEqual(openapi_status, 200)
        self.assertIn('/v1/cmd-vel', openapi['paths'])
        self.assertIn('/v1/stop', openapi['paths'])

    def test_openapi_defines_the_complete_velocity_request_contract(self):
        """Removing required velocity fields from OpenAPI must fail client-generation contract."""
        with urlopen(f'{self.base_url}/openapi.json', timeout=2) as response:
            openapi = json.load(response)

        operation = openapi['paths']['/v1/cmd-vel']['post']
        request_body = operation.get('requestBody')
        self.assertIsNotNone(request_body)
        schema = request_body['content']['application/json']['schema']
        self.assertEqual(schema['required'], ['linear_x', 'angular_z', 'hold_ms'])
        self.assertEqual(schema['properties']['linear_x']['maximum'], 0.1)
        self.assertEqual(schema['properties']['angular_z']['minimum'], -0.5)
        self.assertEqual(schema['properties']['hold_ms']['maximum'], 1000)

    def test_valid_velocity_request_publishes_the_requested_bounded_velocity(self):
        """Dropping or swapping either velocity field must fail the command contract."""
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.08, 'angular_z': -0.25, 'hold_ms': 1000},
        )

        self.assertEqual(status, 202)
        self.assertEqual(body, {
            'state': 'COMMAND_SENT',
            'linear_x': 0.08,
            'angular_z': -0.25,
            'hold_ms': 1000,
        })
        self.assertEqual(self.publisher.messages, [(0.08, -0.25)])

    def test_velocity_is_zeroed_when_hold_time_expires(self):
        """Removing the timeout stop must fail the physical-motion safety contract."""
        status, _ = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.05, 'angular_z': 0.0, 'hold_ms': 20},
        )

        deadline = time.monotonic() + 1
        while len(self.publisher.messages) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertEqual(status, 202)
        self.assertEqual(self.publisher.messages, [(0.05, 0.0), (0.0, 0.0)])

    def test_stop_immediately_zeroes_an_active_velocity_command(self):
        """Removing the stop endpoint or its zero velocity must fail the stop contract."""
        post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.08, 'angular_z': 0.10, 'hold_ms': 1000},
        )
        status, body = post_json(f'{self.base_url}/v1/stop', {})

        self.assertEqual(status, 200)
        self.assertEqual(body, {'state': 'STOPPED'})
        self.assertEqual(self.publisher.messages, [(0.08, 0.10), (0.0, 0.0)])

    def test_excessive_linear_velocity_is_rejected_without_publishing(self):
        """Removing the configured linear speed limit must fail the safety contract."""
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.11, 'angular_z': 0.0, 'hold_ms': 1000},
        )

        self.assertEqual(status, 422)
        self.assertEqual(body, {'error': 'linear_x must be between -0.1 and 0.1'})
        self.assertEqual(self.publisher.messages, [])

    def test_excessive_hold_time_is_rejected_without_publishing(self):
        """Removing the configured duration limit must fail the safety contract."""
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.05, 'angular_z': 0.0, 'hold_ms': 1001},
        )

        self.assertEqual(status, 422)
        self.assertEqual(body, {'error': 'hold_ms must be between 1 and 1000'})
        self.assertEqual(self.publisher.messages, [])

    def test_unknown_velocity_field_is_rejected_to_match_openapi(self):
        """Ignoring an unspecified JSON field must fail the published API contract."""
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.05, 'angular_z': 0.0, 'hold_ms': 1000, 'speed': 1},
        )

        self.assertEqual(status, 422)
        self.assertEqual(body, {'error': 'unknown fields: speed'})
        self.assertEqual(self.publisher.messages, [])

    def test_direct_runner_closes_with_zero_velocity_before_releasing_publisher(self):
        """Removing shutdown zeroing or publisher cleanup must fail this lifecycle contract."""
        self.assertTrue(
            hasattr(self.module, 'run_server'),
            'the direct Python runner must exist',
        )
        publisher = ClosingRecordingPublisher('/cmd_vel')
        http_server = ReturningHttpServer()
        arguments = SimpleNamespace(
            host='0.0.0.0',
            port=8082,
            topic='/cmd_vel',
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
        )

        self.module.run_server(
            arguments,
            publisher_factory=lambda topic: publisher,
            http_server_factory=lambda host, port, service: http_server,
        )

        self.assertTrue(http_server.served)
        self.assertTrue(http_server.closed)
        self.assertEqual(publisher.messages, [(0.0, 0.0)])
        self.assertTrue(publisher.closed)


if __name__ == '__main__':
    unittest.main()
