#!/usr/bin/env python3
"""Serve direct vehicle commands against an already-running global Nav2 stack."""

import argparse
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
            try:
                response = self.navigation.cancel(operation_id)
                cancel_requested = bool(response.get('accepted'))
            except Exception:
                cancel_requested = False

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


class RosVehicleAdapter:
    """ROS adapter that joins the vehicle's existing global Nav2 graph."""

    def __init__(
        self,
        cmd_vel_topic,
        action_name,
        action_server_timeout_sec,
        goal_response_timeout_sec,
        cancel_response_timeout_sec,
    ):
        try:
            import rclpy
            from action_msgs.msg import GoalStatus
            from geometry_msgs.msg import PoseStamped, Twist
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as error:
            raise RuntimeError(
                'ROS 2 Python packages are unavailable. Source the ROS 2 environment first.',
            ) from error

        self._rclpy = rclpy
        self._goal_status = GoalStatus
        self._pose_stamped_type = PoseStamped
        self._twist_type = Twist
        self._navigate_to_pose_type = NavigateToPose
        self._action_client_type = ActionClient
        self._action_server_timeout_sec = action_server_timeout_sec
        self._goal_response_timeout_sec = goal_response_timeout_sec
        self._cancel_response_timeout_sec = cancel_response_timeout_sec
        self._lock = threading.Lock()
        self._goal_handles = {}
        self._closed = False

        self._context = Context()
        self._rclpy.init(args=None, context=self._context)
        self._node = self._rclpy.create_node('vehicle_command_api', context=self._context)
        self._publisher = self._node.create_publisher(self._twist_type, cmd_vel_topic, 10)
        self._goal_client = self._action_client_type(
            self._node,
            self._navigate_to_pose_type,
            action_name,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

    def publish(self, linear_x, angular_z):
        message = self._twist_type()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self._publisher.publish(message)

    def submit_goal(self, operation_id, goal, on_terminal):
        if not self._goal_client.wait_for_server(timeout_sec=self._action_server_timeout_sec):
            return {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}

        request = self._navigate_to_pose_type.Goal()
        request.pose = self._pose_from_goal(goal)
        try:
            goal_handle = self._wait_for_future(
                self._goal_client.send_goal_async(request),
                self._goal_response_timeout_sec,
                'NAVIGATION_GOAL_RESPONSE_TIMEOUT',
            )
        except NavigationUnavailableError as error:
            return {'accepted': False, 'error': str(error)}
        if not goal_handle.accepted:
            return {'accepted': False, 'error': 'NAVIGATION_GOAL_REJECTED'}

        with self._lock:
            self._goal_handles[operation_id] = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda future: self._on_navigation_result(operation_id, goal_handle, on_terminal, future),
        )
        return {'accepted': True}

    def cancel(self, operation_id):
        with self._lock:
            goal_handle = self._goal_handles.get(operation_id)
        if goal_handle is None:
            return {'accepted': False, 'error': 'NAVIGATION_OPERATION_NOT_ACTIVE'}
        try:
            response = self._wait_for_future(
                goal_handle.cancel_goal_async(),
                self._cancel_response_timeout_sec,
                'NAVIGATION_CANCEL_RESPONSE_TIMEOUT',
            )
        except NavigationUnavailableError as error:
            return {'accepted': False, 'error': str(error)}
        return {
            'accepted': bool(getattr(response, 'goals_canceling', [])),
            'error': 'NAVIGATION_CANCEL_REJECTED',
        }

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            self.publish(0.0, 0.0)
        except Exception:
            pass
        self._executor.shutdown()
        self._executor_thread.join(timeout=2)
        self._node.destroy_node()
        if self._context.ok():
            self._rclpy.shutdown(context=self._context)

    def _pose_from_goal(self, goal):
        pose = self._pose_stamped_type()
        pose.header.frame_id = goal['frame_id']
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = goal['x']
        pose.pose.position.y = goal['y']
        pose.pose.orientation.z = math.sin(goal['yaw'] / 2)
        pose.pose.orientation.w = math.cos(goal['yaw'] / 2)
        return pose

    def _wait_for_future(self, future, timeout_sec, timeout_error):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=timeout_sec):
            raise NavigationUnavailableError(timeout_error)
        try:
            return future.result()
        except Exception as error:
            raise NavigationUnavailableError(timeout_error) from error

    def _on_navigation_result(self, operation_id, goal_handle, on_terminal, future):
        try:
            result = future.result()
            mapping = {
                self._goal_status.STATUS_SUCCEEDED: 'COMPLETED',
                self._goal_status.STATUS_CANCELED: 'CANCELLED',
            }
            terminal_state = mapping.get(result.status, 'FAILED')
        except Exception:
            terminal_state = 'FAILED'
        with self._lock:
            if self._goal_handles.get(operation_id) is not goal_handle:
                return
            self._goal_handles.pop(operation_id, None)
        on_terminal(operation_id, terminal_state)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run a standalone HTTP gateway for the vehicle global Nav2 action.',
    )
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--action-name', default='/navigate_to_pose')
    parser.add_argument('--max-linear-x', type=float, default=0.10)
    parser.add_argument('--max-angular-z', type=float, default=0.50)
    parser.add_argument('--max-hold-ms', type=int, default=1000)
    parser.add_argument('--action-server-timeout-sec', type=float, default=1.0)
    parser.add_argument('--goal-response-timeout-sec', type=float, default=3.0)
    parser.add_argument('--cancel-response-timeout-sec', type=float, default=3.0)
    return parser.parse_args(argv)


def create_ros_vehicle_adapter(arguments):
    return RosVehicleAdapter(
        cmd_vel_topic=arguments.cmd_vel_topic,
        action_name=arguments.action_name,
        action_server_timeout_sec=arguments.action_server_timeout_sec,
        goal_response_timeout_sec=arguments.goal_response_timeout_sec,
        cancel_response_timeout_sec=arguments.cancel_response_timeout_sec,
    )


def run_server(arguments, adapter_factory=None, http_server_factory=create_http_server):
    adapter = create_ros_vehicle_adapter(arguments) if adapter_factory is None else adapter_factory(arguments)
    service = VehicleCommandService(
        velocity=adapter,
        navigation=adapter,
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
        adapter.close()


def main(argv=None):
    run_server(parse_args(argv))


if __name__ == '__main__':
    main()
