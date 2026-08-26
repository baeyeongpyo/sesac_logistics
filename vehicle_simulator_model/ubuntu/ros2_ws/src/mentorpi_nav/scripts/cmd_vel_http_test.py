#!/usr/bin/env python3
"""Run a temporary HTTP-to-/cmd_vel test server without a ROS 2 launch file."""

import argparse
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


class CommandValidationError(ValueError):
    pass


class CmdVelCommandService:
    def __init__(self, publisher, max_linear_x, max_angular_z, max_hold_ms):
        self.publisher = publisher
        self.max_linear_x = max_linear_x
        self.max_angular_z = max_angular_z
        self.max_hold_ms = max_hold_ms
        self._lock = threading.Lock()
        self._timer = None
        self._command_number = 0

    def command(self, payload):
        unknown_fields = sorted(set(payload) - {'linear_x', 'angular_z', 'hold_ms'})
        if unknown_fields:
            raise CommandValidationError(f'unknown fields: {", ".join(unknown_fields)}')
        linear_x = self._bounded_number(payload, 'linear_x', self.max_linear_x)
        angular_z = self._bounded_number(payload, 'angular_z', self.max_angular_z)
        hold_ms = self._hold_ms(payload)

        with self._lock:
            self._command_number += 1
            command_number = self._command_number
            self._cancel_timer()
            self.publisher.publish(linear_x, angular_z)
            self._timer = threading.Timer(hold_ms / 1000, self._expire, [command_number])
            self._timer.daemon = True
            self._timer.start()

        return {
            'state': 'COMMAND_SENT',
            'linear_x': linear_x,
            'angular_z': angular_z,
            'hold_ms': hold_ms,
        }

    def stop(self):
        with self._lock:
            self._command_number += 1
            self._cancel_timer()
            self.publisher.publish(0.0, 0.0)
        return {'state': 'STOPPED'}

    def close(self):
        self.stop()

    def _expire(self, command_number):
        with self._lock:
            if command_number != self._command_number:
                return
            self._timer = None
            self.publisher.publish(0.0, 0.0)

    def _cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _bounded_number(self, payload, field, maximum):
        value = payload.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CommandValidationError(f'{field} must be a number')
        value = float(value)
        if not math.isfinite(value) or abs(value) > maximum:
            raise CommandValidationError(f'{field} must be between {-maximum} and {maximum}')
        return value

    def _hold_ms(self, payload):
        value = payload.get('hold_ms')
        if isinstance(value, bool) or not isinstance(value, int):
            raise CommandValidationError('hold_ms must be an integer')
        if value < 1 or value > self.max_hold_ms:
            raise CommandValidationError(f'hold_ms must be between 1 and {self.max_hold_ms}')
        return value


def openapi_document(service):
    return {
        'openapi': '3.0.3',
        'info': {
            'title': 'MentorPi cmd_vel test API',
            'version': '1.0.0',
        },
        'paths': {
            '/healthz': {'get': {'responses': {'200': {'description': 'Ready'}}}},
            '/v1/cmd-vel': {
                'post': {
                    'requestBody': {
                        'required': True,
                        'content': {
                            'application/json': {
                                'schema': {
                                    'type': 'object',
                                    'additionalProperties': False,
                                    'required': ['linear_x', 'angular_z', 'hold_ms'],
                                    'properties': {
                                        'linear_x': {
                                            'type': 'number',
                                            'minimum': -service.max_linear_x,
                                            'maximum': service.max_linear_x,
                                        },
                                        'angular_z': {
                                            'type': 'number',
                                            'minimum': -service.max_angular_z,
                                            'maximum': service.max_angular_z,
                                        },
                                        'hold_ms': {
                                            'type': 'integer',
                                            'minimum': 1,
                                            'maximum': service.max_hold_ms,
                                        },
                                    },
                                },
                            },
                        },
                    },
                    'responses': {
                        '202': {'description': 'Velocity command published'},
                        '400': {'description': 'Invalid JSON'},
                        '422': {'description': 'Invalid velocity command'},
                    },
                },
            },
            '/v1/stop': {'post': {'responses': {'200': {'description': 'Stopped'}}}},
        },
    }


def create_http_server(host, port, service):
    class RequestHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/healthz':
                self._write_json(200, {'status': 'ok'})
                return
            if path == '/openapi.json':
                self._write_json(200, openapi_document(service))
                return
            self._write_json(404, {'error': 'not_found'})

        def do_POST(self):
            path = urlparse(self.path).path
            if path == '/v1/cmd-vel':
                try:
                    response = service.command(self._read_json())
                except CommandValidationError as error:
                    self._write_json(422, {'error': str(error)})
                    return
                except json.JSONDecodeError:
                    self._write_json(400, {'error': 'invalid_json'})
                    return
                self._write_json(202, response)
                return
            if path == '/v1/stop':
                self._write_json(200, service.stop())
                return
            self._write_json(404, {'error': 'not_found'})

        def _read_json(self):
            content_length = self.headers.get('Content-Length')
            if content_length is None:
                raise CommandValidationError('Content-Length header is required')
            try:
                body_length = int(content_length)
            except ValueError as error:
                raise CommandValidationError('Content-Length must be an integer') from error
            if body_length < 1 or body_length > 4096:
                raise CommandValidationError('request body size must be between 1 and 4096 bytes')
            payload = json.loads(self.rfile.read(body_length))
            if not isinstance(payload, dict):
                raise CommandValidationError('request body must be a JSON object')
            return payload

        def _write_json(self, status, payload):
            body = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format_string, *args):
            return

    return ThreadingHTTPServer((host, port), RequestHandler)


class RosCmdVelPublisher:
    """Small ROS 2 adapter kept separate so the HTTP API can be tested without ROS."""

    def __init__(self, topic):
        try:
            import rclpy
            from geometry_msgs.msg import Twist
        except ImportError as error:
            raise RuntimeError(
                'ROS 2 Python packages are unavailable. Source the ROS 2 environment first.',
            ) from error

        rclpy.init(args=None)
        self._rclpy = rclpy
        self._twist_type = Twist
        self._node = rclpy.create_node('cmd_vel_http_test')
        self._publisher = self._node.create_publisher(Twist, topic, 10)

    def publish(self, linear_x, angular_z):
        message = self._twist_type()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self._publisher.publish(message)

    def close(self):
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish bounded test velocities to a ROS 2 cmd_vel topic.',
    )
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--max-linear-x', type=float, default=0.10)
    parser.add_argument('--max-angular-z', type=float, default=0.50)
    parser.add_argument('--max-hold-ms', type=int, default=1000)
    return parser.parse_args(argv)


def run_server(arguments, publisher_factory=RosCmdVelPublisher, http_server_factory=create_http_server):
    publisher = publisher_factory(arguments.topic)
    service = CmdVelCommandService(
        publisher=publisher,
        max_linear_x=arguments.max_linear_x,
        max_angular_z=arguments.max_angular_z,
        max_hold_ms=arguments.max_hold_ms,
    )
    http_server = http_server_factory(arguments.host, arguments.port, service)
    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        service.close()
        http_server.server_close()
        publisher.close()


def main(argv=None):
    run_server(parse_args(argv))


if __name__ == '__main__':
    main()
