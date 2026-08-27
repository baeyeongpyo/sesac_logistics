import asyncio
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from foxglove_ros_worker.command import VehicleCommandApiError, VehicleCommandClient


class VehicleCommandClientTest(unittest.TestCase):
    def vehicle(self, port):
        return SimpleNamespace(
            id='robot_2',
            command_api_url=f'http://127.0.0.1:{port}',
        )

    def test_relay_preserves_vehicle_status_code_and_response_body(self):
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                size = int(self.headers['Content-Length'])
                self.server.received = (self.path, json.loads(self.rfile.read(size)))
                body = b'{"operation_id":"vehicle-generated","state":"NAVIGATING"}'
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
        try:
            response = asyncio.run(VehicleCommandClient().relay(
                self.vehicle(server.server_port),
                'POST',
                '/v1/navigation/goals',
                {'x': 1.5, 'y': 0.0, 'yaw': 0.0},
            ))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

        self.assertEqual(server.received, (
            '/v1/navigation/goals',
            {'x': 1.5, 'y': 0.0, 'yaw': 0.0},
        ))
        self.assertEqual(response.status_code, 202)
        self.assertEqual(response.body, {
            'operation_id': 'vehicle-generated',
            'state': 'NAVIGATING',
        })

    def test_relay_preserves_vehicle_error_status_and_body(self):
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
        try:
            with self.assertRaises(VehicleCommandApiError) as raised:
                asyncio.run(VehicleCommandClient().relay(
                    self.vehicle(server.server_port),
                    'POST',
                    '/v1/localization/initial-pose',
                    {'x': 1.5, 'y': 0.0, 'yaw': 0.0},
                ))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

        self.assertEqual(raised.exception.status_code, 409)
        self.assertEqual(raised.exception.body, {'error': 'VEHICLE_MOTION_ACTIVE'})
        self.assertTrue(raised.exception.__cause__.closed)


if __name__ == '__main__':
    unittest.main()
