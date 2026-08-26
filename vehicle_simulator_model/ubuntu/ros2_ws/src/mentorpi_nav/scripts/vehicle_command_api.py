#!/usr/bin/env python3
"""Serve direct vehicle commands against an already-running global Nav2 stack."""

import json
import math
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


class CommandValidationError(ValueError):
    pass


class NavigationUnavailableError(RuntimeError):
    pass


class NavigationCancelError(RuntimeError):
    pass


class VehicleCommandService:
    def __init__(self, velocity, navigation, max_linear_x, max_angular_z, max_hold_ms):
        self.velocity = velocity
        self.navigation = navigation
        self.max_linear_x = max_linear_x
        self.max_angular_z = max_angular_z
        self.max_hold_ms = max_hold_ms
        self._lock = threading.Lock()
        self._manual_timer = None
        self._manual_generation = 0
        self._active_navigation_operation = None
        self._manual_cancelled_operation = None
        self._stopped_operation = None
        self._status = {
            'operation_id': None,
            'state': 'IDLE',
            'detail': 'READY',
        }

    def operation_status(self):
        with self._lock:
            return dict(self._status)

    def command(self, payload):
        self._validate_fields(payload, {'linear_x', 'angular_z', 'hold_ms'})
        linear_x = self._bounded_number(payload, 'linear_x', self.max_linear_x)
        angular_z = self._bounded_number(payload, 'angular_z', self.max_angular_z)
        hold_ms = self._hold_ms(payload)

        with self._lock:
            active_operation = self._active_navigation_operation
        if active_operation is not None:
            self.navigation.cancel(active_operation)

        with self._lock:
            self._active_navigation_operation = None
            self._manual_cancelled_operation = active_operation
            self._stopped_operation = None
            self._manual_generation += 1
            generation = self._manual_generation
            self._cancel_manual_timer()
            self.velocity.publish(linear_x, angular_z)
            self._set_status(None, 'MANUAL', 'MANUAL_COMMAND_SENT')
            self._manual_timer = threading.Timer(
                hold_ms / 1000,
                self._expire_manual_command,
                [generation],
            )
            self._manual_timer.daemon = True
            self._manual_timer.start()

        return {
            'state': 'MANUAL',
            'linear_x': linear_x,
            'angular_z': angular_z,
            'hold_ms': hold_ms,
        }

    def navigation_goal(self, payload):
        goal = self._goal(payload)
        operation_id = str(uuid.uuid4())

        with self._lock:
            previous_operation = self._active_navigation_operation
        if previous_operation is not None:
            self.navigation.cancel(previous_operation)

        with self._lock:
            self._cancel_manual_timer()
            self._manual_generation += 1
            self._active_navigation_operation = operation_id
            self._manual_cancelled_operation = None
            self._stopped_operation = None
            self._set_status(operation_id, 'NAVIGATING', 'NAVIGATION_GOAL_ACCEPTED')

        try:
            response = self.navigation.submit_goal(operation_id, goal, self._on_navigation_terminal)
        except NavigationUnavailableError:
            response = {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}

        if not response.get('accepted'):
            detail = response.get('error', 'NAVIGATION_GOAL_REJECTED')
            with self._lock:
                if self._active_navigation_operation == operation_id:
                    self._active_navigation_operation = None
                    self._set_status(None, 'FAILED', detail)
            raise NavigationUnavailableError(detail)

        return {
            'operation_id': operation_id,
            'state': 'NAVIGATING',
        }

    def navigation_cancel(self, payload):
        self._validate_fields(payload, {'operation_id'})
        requested_operation = payload.get('operation_id')
        if requested_operation is not None and not isinstance(requested_operation, str):
            raise CommandValidationError('operation_id must be a string')
        with self._lock:
            active_operation = self._active_navigation_operation
        operation_id = requested_operation or active_operation
        if operation_id is None or operation_id != active_operation:
            raise NavigationCancelError('NAVIGATION_OPERATION_NOT_ACTIVE')

        response = self.navigation.cancel(operation_id)
        if not response.get('accepted'):
            raise NavigationCancelError(response.get('error', 'NAVIGATION_CANCEL_REJECTED'))
        with self._lock:
            if self._active_navigation_operation == operation_id:
                self._set_status(operation_id, 'CANCELLING', 'NAVIGATION_CANCEL_REQUESTED')
        return {
            'operation_id': operation_id,
            'state': 'CANCELLING',
        }

    def stop(self):
        with self._lock:
            self._manual_generation += 1
            self._cancel_manual_timer()
            operation_id = self._active_navigation_operation or self._manual_cancelled_operation

        self.velocity.publish(0.0, 0.0)

        cancel_requested = False
        if operation_id is not None:
            response = self.navigation.cancel(operation_id)
            cancel_requested = bool(response.get('accepted'))

        with self._lock:
            if operation_id == self._active_navigation_operation:
                self._stopped_operation = operation_id
            self._manual_cancelled_operation = None
            self._set_status(operation_id, 'STOPPED', 'STOP_REQUESTED')
        return {
            'operation_id': operation_id,
            'state': 'STOPPED',
            'cancel_requested': cancel_requested,
        }

    def close(self):
        self.stop()

    def _on_navigation_terminal(self, operation_id, terminal_state):
        mapping = {
            'COMPLETED': ('COMPLETED', 'NAVIGATION_SUCCEEDED'),
            'CANCELLED': ('CANCELLED', 'NAVIGATION_CANCELLED'),
            'FAILED': ('FAILED', 'NAVIGATION_FAILED'),
        }
        state, detail = mapping.get(terminal_state, ('FAILED', 'NAVIGATION_FAILED'))
        with self._lock:
            if operation_id != self._active_navigation_operation:
                return
            self._active_navigation_operation = None
            if operation_id == self._stopped_operation:
                self._set_status(operation_id, 'STOPPED', 'STOP_REQUESTED')
                return
            self._set_status(operation_id, state, detail)

    def _expire_manual_command(self, generation):
        with self._lock:
            if generation != self._manual_generation:
                return
            self._manual_timer = None
            self._manual_cancelled_operation = None
            self.velocity.publish(0.0, 0.0)
            self._set_status(None, 'IDLE', 'MANUAL_COMMAND_EXPIRED')

    def _set_status(self, operation_id, state, detail):
        self._status = {
            'operation_id': operation_id,
            'state': state,
            'detail': detail,
        }

    def _cancel_manual_timer(self):
        if self._manual_timer is not None:
            self._manual_timer.cancel()
            self._manual_timer = None

    def _goal(self, payload):
        self._validate_fields(payload, {'frame_id', 'x', 'y', 'yaw'})
        frame_id = payload.get('frame_id', 'map')
        if frame_id != 'map':
            raise CommandValidationError('frame_id must be map')
        return {
            'frame_id': frame_id,
            'x': self._finite_number(payload, 'x'),
            'y': self._finite_number(payload, 'y'),
            'yaw': self._finite_number(payload, 'yaw'),
        }

    def _validate_fields(self, payload, allowed_fields):
        unknown_fields = sorted(set(payload) - allowed_fields)
        if unknown_fields:
            raise CommandValidationError(f'unknown fields: {", ".join(unknown_fields)}')

    def _bounded_number(self, payload, field, maximum):
        value = self._finite_number(payload, field)
        if abs(value) > maximum:
            raise CommandValidationError(f'{field} must be between {-maximum} and {maximum}')
        return value

    def _finite_number(self, payload, field):
        value = payload.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CommandValidationError(f'{field} must be a number')
        value = float(value)
        if not math.isfinite(value):
            raise CommandValidationError(f'{field} must be finite')
        return value

    def _hold_ms(self, payload):
        value = payload.get('hold_ms')
        if isinstance(value, bool) or not isinstance(value, int):
            raise CommandValidationError('hold_ms must be an integer')
        if value < 1 or value > self.max_hold_ms:
            raise CommandValidationError(f'hold_ms must be between 1 and {self.max_hold_ms}')
        return value


def openapi_document(service):
    operation_status_schema = {
        'type': 'object',
        'required': ['operation_id', 'state', 'detail'],
        'properties': {
            'operation_id': {'type': 'string', 'nullable': True, 'format': 'uuid'},
            'state': {
                'type': 'string',
                'enum': [
                    'IDLE', 'MANUAL', 'NAVIGATING', 'CANCELLING', 'CANCELLED',
                    'COMPLETED', 'FAILED', 'STOPPED',
                ],
            },
            'detail': {'type': 'string'},
        },
    }
    return {
        'openapi': '3.0.3',
        'info': {
            'title': 'MentorPi Vehicle Command API',
            'version': '1.0.0',
            'description': 'HTTP gateway for the vehicle global Nav2 action and cmd_vel topic.',
        },
        'paths': {
            '/healthz': {
                'get': {'responses': {'200': {'description': 'API process is ready'}}},
            },
            '/openapi.json': {
                'get': {'responses': {'200': {'description': 'OpenAPI document'}}},
            },
            '/v1/operation-status': {
                'get': {
                    'responses': {
                        '200': {
                            'description': 'Current command operation state',
                            'content': {'application/json': {'schema': operation_status_schema}},
                        },
                    },
                },
            },
            '/v1/cmd-vel': {
                'post': {
                    'requestBody': {
                        'required': True,
                        'content': {'application/json': {'schema': {
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
                        }}},
                    },
                    'responses': {
                        '202': {'description': 'Manual velocity published'},
                        '422': {'description': 'Invalid command'},
                    },
                },
            },
            '/v1/navigation/goals': {
                'post': {
                    'requestBody': {
                        'required': True,
                        'content': {'application/json': {'schema': {
                            'type': 'object',
                            'additionalProperties': False,
                            'required': ['x', 'y', 'yaw'],
                            'properties': {
                                'frame_id': {'type': 'string', 'default': 'map'},
                                'x': {'type': 'number'},
                                'y': {'type': 'number'},
                                'yaw': {'type': 'number'},
                            },
                        }}},
                    },
                    'responses': {
                        '202': {'description': 'Navigation goal accepted'},
                        '422': {'description': 'Invalid goal'},
                        '503': {'description': 'Navigation unavailable'},
                    },
                },
            },
            '/v1/navigation/cancel': {
                'post': {
                    'requestBody': {
                        'content': {'application/json': {'schema': {
                            'type': 'object',
                            'additionalProperties': False,
                            'properties': {'operation_id': {'type': 'string', 'format': 'uuid'}},
                        }}},
                    },
                    'responses': {
                        '202': {'description': 'Cancel accepted by Nav2'},
                        '409': {'description': 'Operation is not active'},
                    },
                },
            },
            '/v1/stop': {
                'post': {
                    'responses': {'200': {'description': 'Velocity zeroed and Nav2 cancel requested'}},
                },
            },
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
            if path == '/v1/operation-status':
                self._write_json(200, service.operation_status())
                return
            self._write_json(404, {'error': 'not_found'})

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                if path == '/v1/cmd-vel':
                    self._write_json(202, service.command(self._read_json()))
                    return
                if path == '/v1/navigation/goals':
                    self._write_json(202, service.navigation_goal(self._read_json()))
                    return
                if path == '/v1/navigation/cancel':
                    self._write_json(202, service.navigation_cancel(self._read_optional_json()))
                    return
                if path == '/v1/stop':
                    self._read_optional_json()
                    self._write_json(200, service.stop())
                    return
            except CommandValidationError as error:
                self._write_json(422, {'error': str(error)})
                return
            except NavigationUnavailableError as error:
                self._write_json(503, {'error': str(error)})
                return
            except NavigationCancelError as error:
                self._write_json(409, {'error': str(error)})
                return
            except json.JSONDecodeError:
                self._write_json(400, {'error': 'invalid_json'})
                return
            self._write_json(404, {'error': 'not_found'})

        def _read_json(self):
            content_length = self.headers.get('Content-Length')
            if content_length is None:
                raise CommandValidationError('Content-Length header is required')
            return self._decode_json(content_length)

        def _read_optional_json(self):
            content_length = self.headers.get('Content-Length')
            if content_length is None or content_length == '0':
                return {}
            return self._decode_json(content_length)

        def _decode_json(self, content_length):
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
