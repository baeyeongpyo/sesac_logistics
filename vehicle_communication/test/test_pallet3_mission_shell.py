import json
import os
import subprocess
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


SCRIPT = Path(__file__).resolve().parents[1] / 'tools' / 'pallet3_mission.sh'


class Pallet3MissionShellTests(unittest.TestCase):
    def test_shell_defines_the_required_phases_in_the_mandated_order(self):
        script = SCRIPT.read_text()

        for function in (
            'request_json',
            'wait_vehicle_reply',
            'report_mission_event',
            'run_auto_dock_pick',
            'run_manual_pick',
            'complete_pick',
            'drive_to_p3',
            'lower_fork',
            'complete_place',
            'reverse_after_place',
            'return_to_docker',
            'abort_mission',
        ):
            self.assertIn(f'{function}() {{', script)

        main = script[script.index('main() {'):]
        expected_steps = (
            'complete_pick',
            'drive_to_p3',
            'lower_fork',
            'complete_place',
            'reverse_after_place',
            'return_to_docker',
        )
        indexes = [main.index(step) for step in expected_steps]
        self.assertEqual(indexes, sorted(indexes))

    def test_shell_uses_direct_vehicle_api_and_releases_inventory_operation_before_reverse(self):
        script = SCRIPT.read_text()

        self.assertIn('/v1/missions/pallet3/${OPERATION_ID}/placed', script)
        self.assertIn('"linear_x":-0.18', script)
        self.assertIn('"hold_ms":1000', script)
        self.assertIn('report_mission_event "PICK_COMPLETED"', script)
        self.assertIn('report_mission_event "PLACE_READY"', script)
        self.assertIn('report_mission_event "RETURN_COMPLETED"', script)
        self.assertLess(
            script.index('/v1/missions/pallet3/${OPERATION_ID}/placed'),
            script.index('"linear_x":-0.18'),
        )

    def test_manual_mission_calls_pick_place_then_reverse_and_return_without_inventory_operation_id(self):
        operation_id = '73d5b9af-5a12-4f34-a96c-5de116df1e8e'

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if urlparse(self.path).path == '/v1/operation-status':
                    self.send_json(200, self.server.operation)
                    return
                self.send_json(404, {'error': 'not_found'})

            def do_POST(self):
                size = int(self.headers.get('Content-Length', '0'))
                payload = json.loads(self.rfile.read(size) or b'{}')
                path = urlparse(self.path).path
                self.server.calls.append((path, payload))
                if path.startswith('/api/v1/vehicles/robot_1/missions/pallet3/'):
                    self.server.events.append(payload['event_type'])
                    self.send_json(200, {'accepted': True})
                    return
                if path == '/v1/fork/up':
                    self.server.operation = {'operation_id': operation_id, 'detail': 'FORK_UP_COMPLETE'}
                    self.send_json(202, {})
                    return
                if path.endswith('/picked'):
                    self.server.operation = {'operation_id': operation_id, 'detail': 'MANUAL_PICK_COMPLETED'}
                    self.send_json(200, {})
                    return
                if path == '/v1/navigation/waypoints':
                    next_operation = payload.get('operation_id', 'return-navigation-id')
                    self.server.operation = {'operation_id': next_operation, 'detail': 'NAVIGATION_SUCCEEDED'}
                    self.send_json(202, {})
                    return
                if path == '/v1/fork/down':
                    self.server.operation = {'operation_id': operation_id, 'detail': 'FORK_DOWN_COMPLETE'}
                    self.send_json(202, {})
                    return
                if path.endswith('/placed'):
                    self.server.operation = {'operation_id': None, 'detail': 'PLACE_COMPLETED'}
                    self.send_json(200, {})
                    return
                if path == '/v1/cmd-vel':
                    self.server.operation = {'operation_id': None, 'detail': 'MANUAL_COMMAND_EXPIRED'}
                    self.send_json(202, {})
                    return
                self.send_json(404, {'error': 'not_found'})

            def send_json(self, status, body):
                encoded = json.dumps(body).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def log_message(self, _format, *_args):
                return

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        server.calls = []
        server.events = []
        server.operation = {'operation_id': operation_id, 'detail': 'PALLET3_MISSION_STARTED'}
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        environment = os.environ | {
            'VEHICLE_COMMAND_API_URL': f'http://127.0.0.1:{server.server_port}',
            'PALLET3_STEP_TIMEOUT_SEC': '3',
        }
        try:
            result = subprocess.run(
                [
                    str(SCRIPT),
                    '--operation-id', operation_id,
                    '--robot-id', 'robot_1',
                    '--pick-mode', 'manual',
                    '--fleet-manager-url', f'http://127.0.0.1:{server.server_port}',
                ],
                cwd=SCRIPT.parent,
                env=environment,
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

        self.assertEqual(result.returncode, 0, result.stderr)
        command_calls = [call for call in server.calls if call[0].startswith('/v1/') and call[0] != '/v1/operation-status']
        self.assertEqual(
            [path for path, _payload in command_calls],
            [
                '/v1/fork/up',
                f'/v1/missions/pallet3/{operation_id}/picked',
                '/v1/navigation/waypoints',
                '/v1/fork/down',
                f'/v1/missions/pallet3/{operation_id}/placed',
                '/v1/cmd-vel',
                '/v1/navigation/waypoints',
            ],
        )
        reverse = command_calls[5][1]
        returned = command_calls[6][1]
        self.assertEqual(reverse, {'linear_x': -0.18, 'linear_y': 0.0, 'angular_z': 0.0, 'hold_ms': 1000})
        self.assertNotIn('operation_id', reverse)
        self.assertNotIn('operation_id', returned)
        self.assertEqual(server.events, ['PICK_COMPLETED', 'PLACE_READY', 'RETURN_COMPLETED'])


if __name__ == '__main__':
    unittest.main()
