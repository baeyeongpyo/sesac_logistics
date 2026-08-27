import asyncio
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from foxglove_ros_worker.command import VehicleCommandApiError, VehicleCommandClient


class RecordingRequest:
    def __init__(self, response):
        self.response = response
        self.calls = []

    async def __call__(self, method, url, payload):
        self.calls.append((method, url, payload))
        return self.response


class VehicleCommandClientTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = SimpleNamespace(
            id='robot_2',
            command_api_url='http://192.168.100.35:8082/',
            command=SimpleNamespace(
                max_linear_x=0.3,
                max_angular_z=1.0,
                max_hold_ms=1000,
            ),
        )

    def test_posts_goal_to_configured_vehicle_api_with_planar_pose(self):
        request = RecordingRequest({
            'operation_id': '3c4fed6d-4de6-4679-af67-ef8278f9a771',
            'state': 'NAVIGATING',
        })
        client = VehicleCommandClient(request=request)
        pose_stamped = {
            'header': {
                'stamp': {'sec': 12, 'nanosec': 34},
                'frame_id': 'map',
            },
            'pose': {
                'position': {'x': 1.25, 'y': -0.5, 'z': 0.0},
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': math.sqrt(0.5),
                    'w': math.sqrt(0.5),
                },
            },
        }

        result = asyncio.run(client.send_goal_pose(self.vehicle, pose_stamped))

        self.assertEqual(result['state'], 'NAVIGATING')
        self.assertEqual(len(request.calls), 1)
        method, url, payload = request.calls[0]
        self.assertEqual(method, 'POST')
        self.assertEqual(url, 'http://192.168.100.35:8082/v1/navigation/goals')
        self.assertEqual(payload['frame_id'], 'map')
        self.assertEqual(payload['x'], 1.25)
        self.assertEqual(payload['y'], -0.5)
        self.assertAlmostEqual(payload['yaw'], math.pi / 2)

    def test_posts_bounded_manual_command_to_vehicle_api(self):
        request = RecordingRequest({'state': 'MANUAL'})
        client = VehicleCommandClient(request=request)

        result = asyncio.run(client.send_twist(self.vehicle, 0.1, -0.2, 300))

        self.assertEqual(result, {'state': 'MANUAL'})
        self.assertEqual(request.calls, [(
            'POST',
            'http://192.168.100.35:8082/v1/cmd-vel',
            {'linear_x': 0.1, 'angular_z': -0.2, 'hold_ms': 300},
        )])

    def test_cancel_and_stop_use_vehicle_api_without_foxglove_request(self):
        request = RecordingRequest({'state': 'STOPPED'})
        client = VehicleCommandClient(request=request)

        asyncio.run(client.cancel_navigation(
            self.vehicle,
            '3c4fed6d-4de6-4679-af67-ef8278f9a771',
        ))
        asyncio.run(client.stop(self.vehicle))

        self.assertEqual(request.calls, [
            (
                'POST',
                'http://192.168.100.35:8082/v1/navigation/cancel',
                {'operation_id': '3c4fed6d-4de6-4679-af67-ef8278f9a771'},
            ),
            ('POST', 'http://192.168.100.35:8082/v1/stop', None),
        ])

    def test_default_transport_posts_json_to_vehicle_http_server(self):
        received = []

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                size = int(self.headers['Content-Length'])
                received.append((self.path, json.loads(self.rfile.read(size))))
                body = b'{"state":"MANUAL"}'
                self.send_response(202)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                return

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        vehicle = SimpleNamespace(
            id='robot_2',
            command_api_url=f'http://127.0.0.1:{server.server_port}',
            command=self.vehicle.command,
        )
        try:
            result = asyncio.run(
                VehicleCommandClient().send_twist(vehicle, 0.1, 0.0, 300),
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

        self.assertEqual(result, {'state': 'MANUAL'})
        self.assertEqual(received, [(
            '/v1/cmd-vel',
            {'linear_x': 0.1, 'angular_z': 0.0, 'hold_ms': 300},
        )])

    def test_relay_preserves_vehicle_status_code_and_response_body(self):
        """Changing a relayed vehicle response must fail Fleet Manager API parity."""
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                body = b'{"robot_id":"robot_2","operation":{"state":"IDLE"}}'
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                return

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        vehicle = SimpleNamespace(
            id='robot_2',
            command_api_url=f'http://127.0.0.1:{server.server_port}',
            command=self.vehicle.command,
        )
        try:
            response = asyncio.run(VehicleCommandClient().relay(
                vehicle,
                'GET',
                '/v1/vehicle-status',
            ))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.body, {
            'robot_id': 'robot_2',
            'operation': {'state': 'IDLE'},
        })

    def test_relay_preserves_vehicle_error_status_and_body(self):
        """Replacing a vehicle error response must fail Fleet Manager API parity."""
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = b'{"error":"VEHICLE_MOTION_ACTIVE"}'
                self.send_response(409)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                return

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        vehicle = SimpleNamespace(
            id='robot_2',
            command_api_url=f'http://127.0.0.1:{server.server_port}',
            command=self.vehicle.command,
        )
        try:
            with self.assertRaises(VehicleCommandApiError) as raised:
                asyncio.run(VehicleCommandClient().relay(
                    vehicle,
                    'POST',
                    '/v1/localization/initial-pose',
                    {'x': 1.5, 'y': 0.0, 'yaw': 0.0},
                ))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

        self.assertEqual(raised.exception.status_code, 409)
        self.assertEqual(raised.exception.detail, 'VEHICLE_MOTION_ACTIVE')
        self.assertEqual(raised.exception.body, {'error': 'VEHICLE_MOTION_ACTIVE'})
        self.assertTrue(raised.exception.__cause__.closed)


if __name__ == '__main__':
    unittest.main()
