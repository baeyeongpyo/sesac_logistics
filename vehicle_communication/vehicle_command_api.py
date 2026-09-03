#!/usr/bin/env python3
"""Serve direct vehicle commands against an already-running global Nav2 stack."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import queue
import signal
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from urllib.request import Request, urlopen


MAX_MANUAL_ROTATION_DEGREES = 10.0
MAX_MANUAL_ROTATION_RADIANS = math.radians(MAX_MANUAL_ROTATION_DEGREES)


class CommandValidationError(ValueError):
    pass


class NavigationUnavailableError(RuntimeError):
    pass


class NavigationCancelError(RuntimeError):
    pass


class InitialPoseMotionError(RuntimeError):
    pass


class OperationConflictError(RuntimeError):
    pass


class FleetStatusReporter:
    """Send vehicle observations to Fleet Bridge without blocking vehicle control."""

    def __init__(self, robot_id, relay_url, timeout_sec=2.0):
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError('robot_id must be a non-empty string')
        if not isinstance(relay_url, str) or not relay_url.strip():
            raise ValueError('relay_url must be a non-empty string')
        if timeout_sec <= 0:
            raise ValueError('timeout_sec must be greater than zero')
        self._endpoint = (
            f'{relay_url.rstrip("/")}/api/v1/vehicle-status/{robot_id}'
        )
        self._timeout_sec = timeout_sec
        self._queue = queue.Queue()
        self._closed = False
        self._lock = threading.Lock()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def report(self, payload):
        with self._lock:
            if self._closed:
                return
            self._queue.put(dict(payload))

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._queue.put(None)
        self._worker.join(timeout=self._timeout_sec + 1)

    def _run(self):
        while True:
            payload = self._queue.get()
            if payload is None:
                return
            try:
                request = Request(
                    self._endpoint,
                    data=json.dumps(payload).encode('utf-8'),
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                with urlopen(request, timeout=self._timeout_sec) as response:
                    response.read()
            except Exception:
                pass


class VehicleStatus:
    """Keep the configured identity and most recent battery reading thread-safe."""

    def __init__(self, robot_id, battery_stale_sec, clock=None):
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError('robot_id must be a non-empty string')
        if battery_stale_sec <= 0:
            raise ValueError('battery_stale_sec must be greater than zero')
        self._robot_id = robot_id
        self._battery_stale_sec = battery_stale_sec
        self._clock = clock or time.time
        self._lock = threading.Lock()
        self._battery_raw_value = None
        self._battery_received_at = None

    def update_battery(self, raw_value):
        with self._lock:
            self._battery_raw_value = int(raw_value)
            self._battery_received_at = self._clock()

    def snapshot(self, operation):
        now = self._clock()
        with self._lock:
            raw_value = self._battery_raw_value
            received_at = self._battery_received_at
        stale = received_at is None or (now - received_at) > self._battery_stale_sec
        return {
            'robot_id': self._robot_id,
            'battery': {
                'raw_value': raw_value,
                'received_at': self._format_timestamp(received_at),
                'stale': stale,
            },
            'operation': operation,
        }

    @staticmethod
    def _format_timestamp(timestamp):
        if timestamp is None:
            return None
        return datetime.fromtimestamp(timestamp, timezone.utc).isoformat(
            timespec='milliseconds',
        ).replace('+00:00', 'Z')


class VehicleCommandService:
    def __init__(
        self,
        velocity,
        navigation,
        max_linear_x,
        max_angular_z,
        max_hold_ms,
        initial_pose=None,
        auto_dock=None,
        initial_pose_position_variance=0.25,
        initial_pose_yaw_variance=0.0685,
        vehicle_status=None,
        status_reporter=None,
        max_linear_y=None,
        fork_state_timeout_sec=15.0,
    ):
        self.velocity = velocity
        self.navigation = navigation
        self.max_linear_x = max_linear_x
        self.max_linear_y = max_linear_x if max_linear_y is None else max_linear_y
        self.max_angular_z = max_angular_z
        self.max_hold_ms = max_hold_ms
        if initial_pose_position_variance <= 0 or initial_pose_yaw_variance <= 0:
            raise ValueError('initial pose covariance variances must be greater than zero')
        self._initial_pose_publisher = initial_pose
        self._auto_dock = auto_dock
        self._initial_pose_position_variance = initial_pose_position_variance
        self._initial_pose_yaw_variance = initial_pose_yaw_variance
        self._vehicle_status = vehicle_status or VehicleStatus('unknown', 3.0)
        self._status_reporter = status_reporter
        self._lock = threading.Lock()
        self._manual_timer = None
        self._manual_generation = 0
        self._manual_operation_id = None
        if fork_state_timeout_sec <= 0:
            raise ValueError('fork_state_timeout_sec must be greater than zero')
        self._fork_state_timeout_sec = fork_state_timeout_sec
        self._fork_timer = None
        self._fork_generation = 0
        self._pending_fork_command = None
        self._pending_fork_operation_id = None
        self._active_navigation_operation = None
        self._active_navigation_attempt = None
        self._navigation_origin_state = None
        self._manual_restore_status = None
        self._auto_dock_active = False
        self._auto_dock_operation = None
        self._last_reported_external_state = None
        self._status = {
            'operation_id': None,
            'previous_operation_id': None,
            'state': 'INIT',
            'previous_state': None,
            'detail': 'VEHICLE_BOOTED',
        }

    def operation_status(self):
        with self._lock:
            return dict(self._status)

    def vehicle_status(self):
        return self._vehicle_status.snapshot(self.operation_status())

    def report_current_status(self):
        with self._lock:
            self._report_external_state('VEHICLE')

    def initial_pose(self, payload):
        pose = self._goal(payload)
        with self._lock:
            if (
                self._active_navigation_operation is not None
                or self._auto_dock_active
                or self._status['state'] == 'MANUAL'
            ):
                raise InitialPoseMotionError('VEHICLE_MOTION_ACTIVE')
            if self._initial_pose_publisher is None:
                raise NavigationUnavailableError('INITIAL_POSE_PUBLISHER_UNAVAILABLE')
            self._initial_pose_publisher.publish_initial_pose(
                pose,
                self._initial_pose_position_variance,
                self._initial_pose_yaw_variance,
            )
        return {
            'operation_id': str(uuid.uuid4()),
            'state': 'INITIAL_POSE_PUBLISHED',
            **pose,
        }

    def mark_idle(self, payload):
        self._validate_fields(payload, {'reason'})
        reason = payload.get('reason')
        if not isinstance(reason, str) or not reason:
            raise CommandValidationError('reason must be a non-empty string')
        with self._lock:
            if self._active_navigation_operation is not None or self._auto_dock_active:
                raise OperationConflictError('VEHICLE_MOTION_ACTIVE')
            state = self._status['state']
            if state not in {'INIT', 'CANCELLED', 'FAILED'}:
                raise OperationConflictError('IDLE_TRANSITION_NOT_ALLOWED')
            self._complete_to_idle('OPERATOR_READY', state, source='API')
            return dict(self._status)

    def command(self, payload):
        self._validate_fields(
            payload,
            {'linear_x', 'linear_y', 'angular_z', 'hold_ms', 'operation_id'},
        )
        linear_x = self._bounded_number(payload, 'linear_x', self.max_linear_x)
        linear_y = self._optional_bounded_number(payload, 'linear_y', self.max_linear_y)
        angular_z = self._bounded_number(payload, 'angular_z', self.max_angular_z)
        hold_ms = self._hold_ms(payload)
        operation_id = self._optional_operation_id(payload)
        self._validate_manual_rotation(angular_z, hold_ms)

        with self._lock:
            active_operation = self._active_navigation_operation
            active_attempt = self._active_navigation_attempt
            if self._auto_dock_active:
                raise OperationConflictError('AUTO_DOCK_ACTIVE')
        if active_operation is not None:
            response = self.navigation.cancel(active_attempt)
            if not response.get('accepted'):
                raise NavigationCancelError(response.get('error', 'NAVIGATION_CANCEL_REJECTED'))

        with self._lock:
            if (
                active_operation is not None
                and active_operation == self._active_navigation_operation
            ):
                self._active_navigation_operation = None
                self._active_navigation_attempt = None
                self._navigation_origin_state = None
                self._set_status(
                    active_operation,
                    'CANCELLED',
                    'NAVIGATION_CANCELLED',
                    'DRIVE',
                )
            self._manual_restore_status = dict(self._status)
            self._manual_operation_id = operation_id or self._status['operation_id']
            self._manual_generation += 1
            generation = self._manual_generation
            self._cancel_manual_timer()
            self.velocity.publish(linear_x, linear_y, angular_z)
            self._set_status(
                self._status['operation_id'],
                'MANUAL',
                'MANUAL_COMMAND_SENT',
                self._manual_restore_status['state'],
            )
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
            'linear_y': linear_y,
            'angular_z': angular_z,
            'hold_ms': hold_ms,
        }

    def _fork_command(self, command, payload):
        if command not in {'UP', 'DOWN'}:
            raise CommandValidationError('fork command must be UP or DOWN')
        self._validate_fields(payload, {'operation_id'})
        operation_id = self._optional_operation_id(payload)
        if not hasattr(self.velocity, 'publish_fork_command'):
            raise NavigationUnavailableError('FORK_COMMAND_PUBLISHER_UNAVAILABLE')
        if operation_id is not None:
            with self._lock:
                if self._pending_fork_command is not None:
                    raise OperationConflictError('FORK_COMMAND_ACTIVE')
                self._fork_generation += 1
                generation = self._fork_generation
                self._pending_fork_command = command
                self._pending_fork_operation_id = operation_id
                self._cancel_fork_timer()
                self._fork_timer = threading.Timer(
                    self._fork_state_timeout_sec,
                    self._expire_fork_state,
                    [generation],
                )
                self._fork_timer.daemon = True
                self._fork_timer.start()
        self.velocity.publish_fork_command(command)
        return {'command': command, 'state': 'FORK_COMMAND_PUBLISHED'}

    def fork_command(self, command, payload=None):
        return self._fork_command(command, payload or {})

    def on_fork_state(self, raw_data):
        try:
            payload = json.loads(raw_data)
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = None
        with self._lock:
            command = self._pending_fork_command
            operation_id = self._pending_fork_operation_id
            if command is None or operation_id is None:
                return
            self._clear_pending_fork()

        state = payload.get('state') if isinstance(payload, dict) else None
        error = payload.get('error') if isinstance(payload, dict) else 'INVALID_FORK_STATE'
        if not isinstance(error, str):
            error = 'INVALID_FORK_STATE'
        if state == f'{command}_COMPLETE' and not error:
            self._report_fork_state(operation_id, f'FORK_{command}_COMPLETE', False)
            return
        detail = f'FORK_{command}_ERROR'
        if error:
            detail = f'{detail}:{error}'
        elif state:
            detail = f'{detail}:{state}'
        else:
            detail = f'{detail}:INVALID_FORK_STATE'
        self._report_fork_state(operation_id, detail, True)

    def navigation_goal(self, payload):
        goal = self._goal(payload, {'operation_id', 'purpose'})
        return self._start_navigation(
            payload,
            lambda attempt_id: self.navigation.submit_goal(
                attempt_id,
                goal,
                self._on_navigation_terminal,
            ),
        )

    def navigation_waypoints(self, payload):
        waypoints = self._waypoints(payload)
        return self._start_navigation(
            payload,
            lambda attempt_id: self.navigation.submit_waypoints(
                attempt_id,
                waypoints,
                self._on_navigation_terminal,
            ),
        )

    def _start_navigation(self, payload, submit):
        operation_id = self._operation_id(payload.get('operation_id'))
        purpose = self._navigation_purpose(payload.get('purpose'))
        attempt_id = str(uuid.uuid4())

        with self._lock:
            origin_state = self._status['state']
            if origin_state not in {'IDLE', 'PICK_COMPLETE'}:
                raise OperationConflictError('OPERATION_NOT_READY_FOR_DRIVE')
            if self._active_navigation_operation is not None:
                raise OperationConflictError('NAVIGATION_OPERATION_ACTIVE')
            if (
                origin_state == 'PICK_COMPLETE'
                and operation_id != self._status['operation_id']
            ):
                raise OperationConflictError('OPERATION_ID_MISMATCH')
            if origin_state == 'PICK_COMPLETE' and purpose != 'PLACE':
                raise OperationConflictError('DRIVE_PURPOSE_MUST_BE_PLACE')
            self._cancel_manual_timer()
            self._manual_generation += 1
            self._active_navigation_operation = operation_id
            self._active_navigation_attempt = attempt_id
            self._navigation_origin_state = origin_state
            self._manual_restore_status = None

        try:
            response = submit(attempt_id)
        except NavigationUnavailableError:
            response = {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}

        if not response.get('accepted'):
            detail = response.get('error', 'NAVIGATION_GOAL_REJECTED')
            with self._lock:
                if self._active_navigation_attempt == attempt_id:
                    self._active_navigation_operation = None
                    self._active_navigation_attempt = None
                    self._navigation_origin_state = None
                    self._set_status(
                        operation_id,
                        'FAILED',
                        detail,
                        'DRIVE',
                        source='NAV2',
                        attempt_id=attempt_id,
                    )
            raise NavigationUnavailableError(detail)

        with self._lock:
            if self._active_navigation_attempt == attempt_id:
                self._set_status(
                    operation_id,
                    'DRIVE',
                    'NAVIGATION_GOAL_ACCEPTED',
                    origin_state,
                    source='NAV2',
                    attempt_id=attempt_id,
                )

        return {
            'operation_id': operation_id,
            'attempt_id': attempt_id,
            'state': 'DRIVE',
        }

    def auto_dock_command(self, payload):
        command = self._auto_dock_payload(payload)
        with self._lock:
            state = self._status['state']
            expected_state = 'IDLE' if command['operation'] == 'PICK' else 'PICK_COMPLETE'
            if state != expected_state:
                raise OperationConflictError('OPERATION_NOT_READY_FOR_AUTO_DOCK')
            if (
                command['operation'] == 'PLACE'
                and command['operation_id'] != self._status['operation_id']
            ):
                raise OperationConflictError('OPERATION_ID_MISMATCH')
            if self._auto_dock is None:
                raise NavigationUnavailableError('AUTO_DOCK_PUBLISHER_UNAVAILABLE')
            self._auto_dock.publish_arrival({
                'status': 'SUCCEEDED',
                'location': command['location'],
                'operation': command['operation'],
                'product_type': command['product_type'],
                'target': command['target'],
            })
            self._auto_dock_active = True
            self._auto_dock_operation = command['operation']
            self._set_status(
                command['operation_id'],
                'PICKING' if command['operation'] == 'PICK' else 'PLACE',
                'AUTO_DOCK_COMMAND_ACCEPTED',
                state,
            )
        return {
            'operation_id': command['operation_id'],
            'state': 'PICKING' if command['operation'] == 'PICK' else 'PLACE',
        }

    def on_auto_dock_status(self, payload):
        if not isinstance(payload, dict):
            return
        raw_state = str(payload.get('state', '')).strip().upper()
        reason = str(payload.get('reason', '')).strip()
        with self._lock:
            if not self._auto_dock_active:
                return
            if raw_state == 'ERROR' or self._auto_dock_rejected(raw_state, reason):
                operation_id = self._status['operation_id']
                previous_state = self._status['state']
                self._auto_dock_active = False
                self._auto_dock_operation = None
                detail = 'AUTO_DOCK_ERROR'
                if reason:
                    detail = f'{detail}:{reason}'
                self._set_status(
                    operation_id,
                    'FAILED',
                    detail,
                    previous_state,
                    source='AUTO_DOCK',
                )
                return
            if raw_state in {
                'SEARCHING', 'ALIGNING', 'INSERTING', 'WAIT_UP_COMPLETE',
                'WAIT_DOWN_COMPLETE', 'REVERSING', 'TURNING',
            }:
                detail = f'AUTO_DOCK_{raw_state}'
                if reason:
                    detail = f'{detail}:{reason}'
                self._set_status(
                    self._status['operation_id'],
                    self._status['state'],
                    detail,
                    self._status['previous_state'],
                    source='AUTO_DOCK',
                )

    def on_auto_dock_drive_ready(self):
        with self._lock:
            if not self._auto_dock_active:
                return
            operation_id = self._status['operation_id']
            operation = self._auto_dock_operation
            previous_state = self._status['state']
            self._auto_dock_active = False
            self._auto_dock_operation = None
            if operation == 'PICK':
                self._set_status(
                    operation_id,
                    'PICK_COMPLETE',
                    'AUTO_DOCK_PICK_COMPLETED',
                    previous_state,
                    source='AUTO_DOCK',
                )
                return
            self._set_status(
                operation_id,
                'PLACE_COMPLETE',
                'AUTO_DOCK_PLACE_COMPLETED',
                previous_state,
                source='AUTO_DOCK',
            )
            self._complete_to_idle('AUTO_DOCK_PLACE_COMPLETED', 'PLACE_COMPLETE')

    def navigation_cancel(self, payload):
        self._validate_fields(payload, {'operation_id'})
        requested_operation = payload.get('operation_id')
        if requested_operation is not None and not isinstance(requested_operation, str):
            raise CommandValidationError('operation_id must be a string')
        with self._lock:
            active_operation = self._active_navigation_operation
            active_attempt = self._active_navigation_attempt
        operation_id = requested_operation or active_operation
        if operation_id is None or operation_id != active_operation:
            raise NavigationCancelError('NAVIGATION_OPERATION_NOT_ACTIVE')

        response = self.navigation.cancel(active_attempt)
        if not response.get('accepted'):
            raise NavigationCancelError(response.get('error', 'NAVIGATION_CANCEL_REJECTED'))
        with self._lock:
            if self._active_navigation_operation == operation_id:
                self._active_navigation_operation = None
                self._active_navigation_attempt = None
                self._navigation_origin_state = None
                self._set_status(
                    operation_id,
                    'CANCELLED',
                    'NAVIGATION_CANCELLED',
                    'DRIVE',
                    source='API',
                    attempt_id=active_attempt,
                )
        return {
            'operation_id': operation_id,
            'state': 'CANCELLED',
        }

    def stop(self):
        with self._lock:
            self._manual_generation += 1
            self._cancel_manual_timer()
            self._clear_pending_fork()
            navigation_attempt = self._active_navigation_attempt
            auto_dock_active = self._auto_dock_active
            status_before_stop = dict(self._status)

        self.velocity.publish(0.0, 0.0, 0.0)

        cancel_requested = False
        if navigation_attempt is not None:
            try:
                response = self.navigation.cancel(navigation_attempt)
                cancel_requested = bool(response.get('accepted'))
            except Exception:
                cancel_requested = False
        if auto_dock_active:
            try:
                self._auto_dock.stop()
                cancel_requested = True
            except Exception:
                cancel_requested = False

        with self._lock:
            if navigation_attempt == self._active_navigation_attempt:
                self._active_navigation_operation = None
                self._active_navigation_attempt = None
                self._navigation_origin_state = None
            if auto_dock_active:
                self._auto_dock_active = False
                self._auto_dock_operation = None
            self._manual_restore_status = None
            if status_before_stop['operation_id'] is not None:
                self._set_status(
                    status_before_stop['operation_id'],
                    'CANCELLED',
                    'STOP_REQUESTED',
                    status_before_stop['state'],
                    source='API',
                    attempt_id=navigation_attempt,
                )
            elif status_before_stop['state'] == 'MANUAL':
                self._set_status(
                    None,
                    'FAILED',
                    'STOP_REQUESTED',
                    'MANUAL',
                    source='API',
                )
            else:
                self._set_status(
                    None,
                    'FAILED',
                    'STOP_REQUESTED',
                    status_before_stop['state'],
                    source='API',
                )
        return {
            'operation_id': status_before_stop['operation_id'],
            'state': self.operation_status()['state'],
            'cancel_requested': cancel_requested,
        }

    def close(self):
        with self._lock:
            self._clear_pending_fork()
        self.stop()

    def _on_navigation_terminal(self, attempt_id, terminal_state):
        with self._lock:
            if attempt_id != self._active_navigation_attempt:
                return
            operation_id = self._active_navigation_operation
            origin_state = self._navigation_origin_state
            self._active_navigation_operation = None
            self._active_navigation_attempt = None
            self._navigation_origin_state = None
            if terminal_state == 'COMPLETED':
                if origin_state == 'PICK_COMPLETE':
                    self._set_status(
                        operation_id,
                        'PICK_COMPLETE',
                        'NAVIGATION_SUCCEEDED',
                        'DRIVE',
                        source='NAV2',
                        attempt_id=attempt_id,
                    )
                    return
                self._complete_to_idle(
                    'NAVIGATION_SUCCEEDED',
                    'DRIVE',
                    source='NAV2',
                    attempt_id=attempt_id,
                )
                return
            if terminal_state == 'CANCELLED':
                self._set_status(
                    operation_id,
                    'CANCELLED',
                    'NAVIGATION_CANCELLED',
                    'DRIVE',
                    source='NAV2',
                    attempt_id=attempt_id,
                )
                return
            self._set_status(
                operation_id,
                'FAILED',
                'NAVIGATION_FAILED',
                'DRIVE',
                source='NAV2',
                attempt_id=attempt_id,
            )

    def _expire_manual_command(self, generation):
        with self._lock:
            if generation != self._manual_generation:
                return
            self._manual_timer = None
            self.velocity.publish(0.0, 0.0, 0.0)
            restored = self._manual_restore_status or {
                'operation_id': None,
                'previous_operation_id': self._status['previous_operation_id'],
                'state': 'INIT',
                'previous_state': 'MANUAL',
                'detail': 'MANUAL_COMMAND_EXPIRED',
            }
            self._manual_restore_status = None
            operation_id = self._manual_operation_id
            self._manual_operation_id = None
            self._status = {
                **restored,
                'detail': 'MANUAL_COMMAND_EXPIRED',
            }
            if operation_id is not None:
                self._report_external_state('API', operation_id=operation_id)

    def _set_status(
        self,
        operation_id,
        state,
        detail,
        previous_state=None,
        source=None,
        attempt_id=None,
    ):
        self._status = {
            'operation_id': operation_id,
            'previous_operation_id': self._status['previous_operation_id'],
            'state': state,
            'previous_state': previous_state,
            'detail': detail,
        }

        if source is not None:
            self._report_external_state(source, attempt_id)

    def _complete_to_idle(self, detail, previous_state, source=None, attempt_id=None):
        operation_id = self._status['operation_id']
        self._status = {
            'operation_id': None,
            'previous_operation_id': operation_id or self._status['previous_operation_id'],
            'state': 'IDLE',
            'previous_state': previous_state,
            'detail': detail,
        }
        if source is not None:
            self._report_external_state(source, attempt_id, operation_id)

    def _report_external_state(self, source, attempt_id=None, operation_id=None):
        if self._status_reporter is None:
            return
        external_state = {
            'INIT': 'INIT',
            'IDLE': 'WAIT',
            'DRIVE': 'DRIVE',
            'PICKING': 'PICK',
            'PICK_COMPLETE': 'WAIT',
            'PLACE': 'PLACE',
            'PLACE_COMPLETE': 'WAIT',
            'FAILED': 'FAIL',
            'CANCELLED': 'FAIL',
        }.get(self._status['state'])
        if external_state is None:
            return
        detail = self._status['detail']
        if detail == 'NAVIGATION_GOAL_ACCEPTED':
            detail = 'NAVIGATION_STARTED'
        elif detail == 'STOP_REQUESTED':
            detail = 'API_STOP'
        elif detail == 'NAVIGATION_CANCELLED' and source == 'API':
            detail = 'API_NAVIGATION_CANCEL'
        payload = {
            'state': external_state,
            'previous_state': self._last_reported_external_state,
            'operation_id': (
                self._status['operation_id']
                if operation_id is None
                else operation_id
            ),
            'attempt_id': attempt_id,
            'source': source,
            'detail': detail,
            'observed_at': datetime.now(timezone.utc).isoformat(
                timespec='milliseconds',
            ).replace('+00:00', 'Z'),
        }
        try:
            self._status_reporter.report(payload)
        except Exception:
            return
        self._last_reported_external_state = external_state

    def _cancel_manual_timer(self):
        if self._manual_timer is not None:
            self._manual_timer.cancel()
            self._manual_timer = None

    def _expire_fork_state(self, generation):
        with self._lock:
            if generation != self._fork_generation:
                return
            command = self._pending_fork_command
            operation_id = self._pending_fork_operation_id
            self._clear_pending_fork()
        if command is not None and operation_id is not None:
            self._report_fork_state(operation_id, f'FORK_{command}_TIMEOUT', True)

    def _clear_pending_fork(self):
        self._fork_generation += 1
        self._cancel_fork_timer()
        self._pending_fork_command = None
        self._pending_fork_operation_id = None

    def _cancel_fork_timer(self):
        if self._fork_timer is not None:
            self._fork_timer.cancel()
            self._fork_timer = None

    def _report_fork_state(self, operation_id, detail, failed):
        if self._status_reporter is None:
            return
        state = 'FAIL' if failed else 'WAIT'
        payload = {
            'state': state,
            'previous_state': self._last_reported_external_state,
            'operation_id': operation_id,
            'attempt_id': None,
            'source': 'FORK',
            'detail': detail,
            'observed_at': datetime.now(timezone.utc).isoformat(
                timespec='milliseconds',
            ).replace('+00:00', 'Z'),
        }
        try:
            self._status_reporter.report(payload)
        except Exception:
            return
        self._last_reported_external_state = state

    def _goal(self, payload, extra_fields=None):
        allowed_fields = {'frame_id', 'x', 'y', 'yaw'}
        if extra_fields:
            allowed_fields.update(extra_fields)
        self._validate_fields(payload, allowed_fields)
        frame_id = payload.get('frame_id', 'map')
        if frame_id != 'map':
            raise CommandValidationError('frame_id must be map')
        return {
            'frame_id': frame_id,
            'x': self._finite_number(payload, 'x'),
            'y': self._finite_number(payload, 'y'),
            'yaw': self._finite_number(payload, 'yaw'),
        }

    @staticmethod
    def _optional_operation_id(payload):
        operation_id = payload.get('operation_id')
        if operation_id is None:
            return None
        if not isinstance(operation_id, str) or not operation_id.strip():
            raise CommandValidationError('operation_id must be a non-empty string')
        try:
            return str(uuid.UUID(operation_id))
        except (ValueError, AttributeError) as error:
            raise CommandValidationError('operation_id must be a UUID') from error

    def _waypoints(self, payload):
        self._validate_fields(payload, {'operation_id', 'purpose', 'waypoints'})
        raw_waypoints = payload.get('waypoints')
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise CommandValidationError('waypoints must be a non-empty array')
        waypoints = []
        for index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, dict):
                raise CommandValidationError(f'waypoints[{index}] must be an object')
            waypoints.append(self._goal(waypoint))
        return waypoints

    def _auto_dock_payload(self, payload):
        self._validate_fields(
            payload,
            {'operation_id', 'operation', 'product_type', 'location', 'target'},
        )
        operation = self._required_uppercase(payload, 'operation', {'PICK', 'PLACE'})
        product_type = self._required_uppercase(payload, 'product_type', {'NORMAL', 'FRESH'})
        location = payload.get('location')
        if not isinstance(location, str) or not location.strip():
            raise CommandValidationError('location must be a non-empty string')
        target = payload.get('target')
        if not isinstance(target, dict):
            raise CommandValidationError('target must be an object')
        return {
            'operation_id': self._operation_id(payload.get('operation_id'), required=True),
            'operation': operation,
            'product_type': product_type,
            'location': location.strip().upper(),
            'target': target,
        }

    def _navigation_purpose(self, value):
        if value is None:
            return None
        return self._required_uppercase({'purpose': value}, 'purpose', {'PICK', 'PLACE'})

    @staticmethod
    def _auto_dock_rejected(raw_state, reason):
        return (
            raw_state == 'REJECTED'
            or reason.startswith('arrival_')
            or reason.startswith('pick_requires_')
            or reason.startswith('place_requires_')
        )

    def _operation_id(self, value, required=False):
        if value is None and not required:
            return str(uuid.uuid4())
        if not isinstance(value, str) or not value:
            raise CommandValidationError('operation_id must be a non-empty string')
        try:
            return str(uuid.UUID(value))
        except (ValueError, AttributeError) as error:
            raise CommandValidationError('operation_id must be a UUID') from error

    def _required_uppercase(self, payload, field, allowed_values):
        value = payload.get(field)
        if not isinstance(value, str):
            raise CommandValidationError(f'{field} must be a string')
        value = value.strip().upper()
        if value not in allowed_values:
            allowed = ', '.join(sorted(allowed_values))
            raise CommandValidationError(f'{field} must be one of {allowed}')
        return value

    def _validate_fields(self, payload, allowed_fields):
        unknown_fields = sorted(set(payload) - allowed_fields)
        if unknown_fields:
            raise CommandValidationError(f'unknown fields: {", ".join(unknown_fields)}')

    def _bounded_number(self, payload, field, maximum):
        value = self._finite_number(payload, field)
        if abs(value) > maximum:
            raise CommandValidationError(f'{field} must be between {-maximum} and {maximum}')
        return value

    def _optional_bounded_number(self, payload, field, maximum):
        if field not in payload:
            return 0.0
        return self._bounded_number(payload, field, maximum)

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

    @staticmethod
    def _validate_manual_rotation(angular_z, hold_ms):
        if abs(angular_z) * hold_ms / 1000 > MAX_MANUAL_ROTATION_RADIANS:
            raise CommandValidationError(
                'angular_z and hold_ms must not exceed 10 degrees per command',
            )


def openapi_document(service):
    operation_status_schema = {
        'type': 'object',
        'required': [
            'operation_id', 'previous_operation_id', 'state', 'previous_state', 'detail',
        ],
        'properties': {
            'operation_id': {'type': 'string', 'nullable': True, 'format': 'uuid'},
            'previous_operation_id': {'type': 'string', 'nullable': True, 'format': 'uuid'},
            'state': {
                'type': 'string',
                'enum': [
                    'INIT', 'IDLE', 'DRIVE', 'PICKING', 'PICK_COMPLETE', 'PLACE',
                    'PLACE_COMPLETE', 'FAILED', 'CANCELLED', 'MANUAL',
                ],
            },
            'previous_state': {'type': 'string', 'nullable': True},
            'detail': {'type': 'string'},
        },
    }
    vehicle_status_schema = {
        'type': 'object',
        'required': ['robot_id', 'battery', 'operation'],
        'properties': {
            'robot_id': {'type': 'string'},
            'battery': {
                'type': 'object',
                'required': ['raw_value', 'received_at', 'stale'],
                'properties': {
                    'raw_value': {'type': 'integer', 'nullable': True},
                    'received_at': {
                        'type': 'string',
                        'format': 'date-time',
                        'nullable': True,
                    },
                    'stale': {'type': 'boolean'},
                },
            },
            'operation': operation_status_schema,
        },
    }
    initial_pose_schema = {
        'type': 'object',
        'additionalProperties': False,
        'required': ['x', 'y', 'yaw'],
        'properties': {
            'frame_id': {'type': 'string', 'default': 'map'},
            'x': {'type': 'number'},
            'y': {'type': 'number'},
            'yaw': {'type': 'number'},
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
            '/v1/vehicle-status': {
                'get': {
                    'responses': {
                        '200': {
                            'description': 'Configured vehicle identity, battery and operation state',
                            'content': {'application/json': {'schema': vehicle_status_schema}},
                        },
                    },
                },
            },
            '/v1/operation/idle': {
                'post': {
                    'requestBody': {
                        'required': True,
                        'content': {'application/json': {'schema': {
                            'type': 'object',
                            'additionalProperties': False,
                            'required': ['reason'],
                            'properties': {'reason': {'type': 'string'}},
                        }}},
                    },
                    'responses': {
                        '200': {'description': 'Operator authorised a ready vehicle'},
                        '409': {'description': 'Operation is active or cannot become idle'},
                        '422': {'description': 'Invalid idle transition request'},
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
                                'linear_y': {
                                    'type': 'number',
                                    'default': 0.0,
                                    'minimum': -service.max_linear_y,
                                    'maximum': service.max_linear_y,
                                },
                                'operation_id': {'type': 'string', 'format': 'uuid'},
                                'angular_z': {
                                    'type': 'number',
                                    'description': (
                                        'One command is limited to 10 degrees: '
                                        'abs(angular_z) * hold_ms / 1000 <= 0.174533.'
                                    ),
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
            '/v1/fork/up': {
                'post': {
                    'requestBody': {
                        'content': {'application/json': {'schema': {
                            'type': 'object',
                            'additionalProperties': False,
                            'properties': {
                                'operation_id': {'type': 'string', 'format': 'uuid'},
                            },
                        }}},
                    },
                    'responses': {
                        '202': {'description': 'UP published to the fork command topic'},
                        '503': {'description': 'Fork command topic publisher unavailable'},
                    },
                },
            },
            '/v1/fork/down': {
                'post': {
                    'requestBody': {
                        'content': {'application/json': {'schema': {
                            'type': 'object',
                            'additionalProperties': False,
                            'properties': {
                                'operation_id': {'type': 'string', 'format': 'uuid'},
                            },
                        }}},
                    },
                    'responses': {
                        '202': {'description': 'DOWN published to the fork command topic'},
                        '503': {'description': 'Fork command topic publisher unavailable'},
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
                                'operation_id': {'type': 'string', 'format': 'uuid'},
                                'purpose': {'type': 'string', 'enum': ['PICK', 'PLACE']},
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
            '/v1/navigation/waypoints': {
                'post': {
                    'requestBody': {
                        'required': True,
                        'content': {'application/json': {'schema': {
                            'type': 'object',
                            'additionalProperties': False,
                            'required': ['waypoints'],
                            'properties': {
                                'operation_id': {'type': 'string', 'format': 'uuid'},
                                'purpose': {'type': 'string', 'enum': ['PICK', 'PLACE']},
                                'waypoints': {
                                    'type': 'array',
                                    'minItems': 1,
                                    'items': {
                                        'type': 'object',
                                        'additionalProperties': False,
                                        'required': ['x', 'y', 'yaw'],
                                        'properties': {
                                            'frame_id': {'type': 'string', 'default': 'map'},
                                            'x': {'type': 'number'},
                                            'y': {'type': 'number'},
                                            'yaw': {'type': 'number'},
                                        },
                                    },
                                },
                            },
                        }}},
                    },
                    'responses': {
                        '202': {'description': 'FollowWaypoints route accepted'},
                        '422': {'description': 'Invalid waypoint route'},
                        '503': {'description': 'Navigation unavailable'},
                    },
                },
            },
            '/v1/auto-dock': {
                'post': {
                    'requestBody': {
                        'required': True,
                        'content': {'application/json': {'schema': {
                            'type': 'object',
                            'additionalProperties': False,
                            'required': [
                                'operation_id', 'operation', 'product_type', 'location', 'target',
                            ],
                            'properties': {
                                'operation_id': {'type': 'string', 'format': 'uuid'},
                                'operation': {'type': 'string', 'enum': ['PICK', 'PLACE']},
                                'product_type': {'type': 'string', 'enum': ['NORMAL', 'FRESH']},
                                'location': {'type': 'string'},
                                'target': {'type': 'object'},
                            },
                        }}},
                    },
                    'responses': {
                        '202': {'description': 'Auto Dock command published'},
                        '409': {'description': 'Vehicle state cannot accept this dock operation'},
                        '422': {'description': 'Invalid Auto Dock command'},
                        '503': {'description': 'Auto Dock topic publisher unavailable'},
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
            '/v1/localization/initial-pose': {
                'post': {
                    'requestBody': {
                        'required': True,
                        'content': {'application/json': {'schema': initial_pose_schema}},
                    },
                    'responses': {
                        '202': {'description': 'Initial pose published to AMCL'},
                        '409': {'description': 'Vehicle has an active navigation or manual motion'},
                        '422': {'description': 'Invalid initial pose'},
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
            if path == '/v1/vehicle-status':
                self._write_json(200, service.vehicle_status())
                return
            self._write_json(404, {'error': 'not_found'})

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                if path == '/v1/cmd-vel':
                    self._write_json(202, service.command(self._read_json()))
                    return
                if path == '/v1/fork/up':
                    self._write_json(
                        202,
                        service.fork_command('UP', self._read_optional_json()),
                    )
                    return
                if path == '/v1/fork/down':
                    self._write_json(
                        202,
                        service.fork_command('DOWN', self._read_optional_json()),
                    )
                    return
                if path == '/v1/navigation/goals':
                    self._write_json(202, service.navigation_goal(self._read_json()))
                    return
                if path == '/v1/navigation/waypoints':
                    self._write_json(202, service.navigation_waypoints(self._read_json()))
                    return
                if path == '/v1/auto-dock':
                    self._write_json(202, service.auto_dock_command(self._read_json()))
                    return
                if path == '/v1/operation/idle':
                    self._write_json(200, service.mark_idle(self._read_json()))
                    return
                if path == '/v1/navigation/cancel':
                    self._write_json(202, service.navigation_cancel(self._read_optional_json()))
                    return
                if path == '/v1/localization/initial-pose':
                    self._write_json(202, service.initial_pose(self._read_json()))
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
            except InitialPoseMotionError as error:
                self._write_json(409, {'error': str(error)})
                return
            except OperationConflictError as error:
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
        robot_id,
        cmd_vel_topic,
        action_name,
        follow_waypoints_action_name,
        action_server_timeout_sec,
        goal_response_timeout_sec,
        cancel_response_timeout_sec,
        fork_command_topic=None,
        fork_state_topic=None,
        auto_dock_arrival_topic=None,
        auto_dock_status_topic=None,
        auto_dock_stop_topic=None,
        auto_dock_drive_ready_topic=None,
    ):
        try:
            import rclpy
            from action_msgs.msg import GoalStatus
            from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
            from nav2_msgs.action import FollowWaypoints, NavigateToPose
            from rclpy.action import ActionClient
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                DurabilityPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from std_msgs.msg import Empty, String, UInt16
        except ImportError as error:
            raise RuntimeError(
                'ROS 2 Python packages are unavailable. Source the ROS 2 environment first.',
            ) from error

        self._rclpy = rclpy
        self._goal_status = GoalStatus
        self._pose_stamped_type = PoseStamped
        self._pose_with_covariance_stamped_type = PoseWithCovarianceStamped
        self._twist_type = Twist
        self._navigate_to_pose_type = NavigateToPose
        self._follow_waypoints_type = FollowWaypoints
        self._action_client_type = ActionClient
        self._uint16_type = UInt16
        self._string_type = String
        self._empty_type = Empty
        self._battery_qos = qos_profile_sensor_data
        self._auto_dock_status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._action_server_timeout_sec = action_server_timeout_sec
        self._goal_response_timeout_sec = goal_response_timeout_sec
        self._cancel_response_timeout_sec = cancel_response_timeout_sec
        self._lock = threading.Lock()
        self._goal_handles = {}
        self._closed = False
        self._battery_subscription = None
        self._initial_pose_publisher = None
        self._auto_dock_status_subscription = None
        self._auto_dock_drive_ready_subscription = None
        self._fork_state_subscription = None

        robot_name = robot_id.strip('/')
        if not robot_name:
            raise ValueError('robot_id must be a non-empty string')
        self._auto_dock_arrival_topic = (
            auto_dock_arrival_topic or f'/{robot_name}/nav2/arrival'
        )
        self._auto_dock_status_topic = (
            auto_dock_status_topic or f'/{robot_name}/auto_dock/status'
        )
        self._auto_dock_stop_topic = (
            auto_dock_stop_topic or f'/{robot_name}/auto_dock/stop'
        )
        self._auto_dock_drive_ready_topic = (
            auto_dock_drive_ready_topic or f'/{robot_name}/auto_dock/drive_ready'
        )
        self._fork_command_topic = fork_command_topic or '/fork/command'
        self._fork_state_topic = fork_state_topic or '/fork/state'

        self._context = Context()
        self._rclpy.init(args=None, context=self._context)
        self._node = self._rclpy.create_node('vehicle_command_api', context=self._context)
        self._publisher = self._node.create_publisher(self._twist_type, cmd_vel_topic, 10)
        self._fork_command_publisher = self._node.create_publisher(
            self._string_type,
            self._fork_command_topic,
            10,
        )
        self._auto_dock_arrival_publisher = self._node.create_publisher(
            self._string_type,
            self._auto_dock_arrival_topic,
            10,
        )
        self._auto_dock_stop_publisher = self._node.create_publisher(
            self._empty_type,
            self._auto_dock_stop_topic,
            10,
        )
        self._goal_client = self._action_client_type(
            self._node,
            self._navigate_to_pose_type,
            action_name,
        )
        self._waypoints_client = self._action_client_type(
            self._node,
            self._follow_waypoints_type,
            follow_waypoints_action_name,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

    def subscribe_battery(self, battery_topic, on_battery):
        if self._battery_subscription is not None:
            raise RuntimeError('battery subscription is already configured')
        self._battery_subscription = self._node.create_subscription(
            self._uint16_type,
            battery_topic,
            lambda message: on_battery(message.data),
            self._battery_qos,
        )

    def configure_initial_pose_publisher(self, initial_pose_topic):
        if self._initial_pose_publisher is not None:
            raise RuntimeError('initial pose publisher is already configured')
        self._initial_pose_publisher = self._node.create_publisher(
            self._pose_with_covariance_stamped_type,
            initial_pose_topic,
            10,
        )

    def configure_auto_dock(self, on_status, on_drive_ready):
        if self._auto_dock_status_subscription is not None:
            raise RuntimeError('auto dock subscriptions are already configured')

        def status_callback(message):
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                return
            if isinstance(payload, dict):
                on_status(payload)

        self._auto_dock_status_subscription = self._node.create_subscription(
            self._string_type,
            self._auto_dock_status_topic,
            status_callback,
            self._auto_dock_status_qos,
        )
        self._auto_dock_drive_ready_subscription = self._node.create_subscription(
            self._empty_type,
            self._auto_dock_drive_ready_topic,
            lambda _message: on_drive_ready(),
            10,
        )

    def configure_fork_state_subscription(self, on_fork_state):
        if self._fork_state_subscription is not None:
            raise RuntimeError('fork state subscription is already configured')
        self._fork_state_subscription = self._node.create_subscription(
            self._string_type,
            self._fork_state_topic,
            lambda message: on_fork_state(message.data),
            10,
        )

    def publish_initial_pose(self, pose, position_variance, yaw_variance):
        if self._initial_pose_publisher is None:
            raise NavigationUnavailableError('INITIAL_POSE_PUBLISHER_UNAVAILABLE')
        message = self._pose_with_covariance_stamped_type()
        message.header.frame_id = pose['frame_id']
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = pose['x']
        message.pose.pose.position.y = pose['y']
        message.pose.pose.orientation.z = math.sin(pose['yaw'] / 2)
        message.pose.pose.orientation.w = math.cos(pose['yaw'] / 2)
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[35] = yaw_variance
        self._initial_pose_publisher.publish(message)

    def publish(self, linear_x, linear_y, angular_z):
        message = self._twist_type()
        message.linear.x = linear_x
        message.linear.y = linear_y
        message.angular.z = angular_z
        self._publisher.publish(message)

    def publish_fork_command(self, command):
        message = self._string_type()
        message.data = command
        self._fork_command_publisher.publish(message)

    def publish_arrival(self, payload):
        message = self._string_type()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self._auto_dock_arrival_publisher.publish(message)

    def stop(self):
        self._auto_dock_stop_publisher.publish(self._empty_type())

    def submit_goal(self, operation_id, goal, on_terminal):
        if not self._goal_client.wait_for_server(timeout_sec=self._action_server_timeout_sec):
            return {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}

        request = self._navigate_to_pose_type.Goal()
        request.pose = self._pose_from_goal(goal)
        return self._submit_action_goal(
            self._goal_client,
            operation_id,
            request,
            on_terminal,
        )

    def submit_waypoints(self, operation_id, waypoints, on_terminal):
        if not self._waypoints_client.wait_for_server(timeout_sec=self._action_server_timeout_sec):
            return {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}

        request = self._follow_waypoints_type.Goal()
        request.poses = [self._pose_from_goal(waypoint) for waypoint in waypoints]
        return self._submit_action_goal(
            self._waypoints_client,
            operation_id,
            request,
            on_terminal,
        )

    def _submit_action_goal(self, action_client, operation_id, request, on_terminal):
        try:
            goal_handle = self._wait_for_future(
                action_client.send_goal_async(request),
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
            self.publish(0.0, 0.0, 0.0)
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
            if (
                terminal_state == 'COMPLETED'
                and getattr(getattr(result, 'result', None), 'missed_waypoints', ())
            ):
                terminal_state = 'FAILED'
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
    parser.add_argument('--robot-id', required=True)
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--fork-command-topic', default='/fork/command')
    parser.add_argument('--fork-state-topic', default='/fork/state')
    parser.add_argument('--fork-state-timeout-sec', type=float, default=15.0)
    parser.add_argument('--action-name', default='/navigate_to_pose')
    parser.add_argument('--follow-waypoints-action-name', default='/follow_waypoints')
    parser.add_argument('--battery-topic', default='/ros_robot_controller/battery')
    parser.add_argument('--battery-stale-sec', type=float, default=3.0)
    parser.add_argument('--initial-pose-topic', default='/initialpose')
    parser.add_argument('--initial-pose-position-variance', type=float, default=0.25)
    parser.add_argument('--initial-pose-yaw-variance', type=float, default=0.0685)
    parser.add_argument('--auto-dock-arrival-topic')
    parser.add_argument('--auto-dock-status-topic')
    parser.add_argument('--auto-dock-stop-topic')
    parser.add_argument('--auto-dock-drive-ready-topic')
    parser.add_argument('--max-linear-x', type=float, default=1.0)
    parser.add_argument('--max-linear-y', type=float, default=1.0)
    parser.add_argument('--max-angular-z', type=float, default=1.0)
    parser.add_argument('--max-hold-ms', type=int, default=1000)
    parser.add_argument('--action-server-timeout-sec', type=float, default=1.0)
    parser.add_argument('--goal-response-timeout-sec', type=float, default=3.0)
    parser.add_argument('--cancel-response-timeout-sec', type=float, default=3.0)
    parser.add_argument(
        '--fleet-status-relay-url',
        default=os.environ.get('FLEET_STATUS_RELAY_URL'),
    )
    return parser.parse_args(argv)


def create_ros_vehicle_adapter(arguments):
    return RosVehicleAdapter(
        robot_id=arguments.robot_id,
        cmd_vel_topic=arguments.cmd_vel_topic,
        fork_command_topic=getattr(arguments, 'fork_command_topic', None),
        fork_state_topic=getattr(arguments, 'fork_state_topic', None),
        action_name=arguments.action_name,
        follow_waypoints_action_name=arguments.follow_waypoints_action_name,
        action_server_timeout_sec=arguments.action_server_timeout_sec,
        goal_response_timeout_sec=arguments.goal_response_timeout_sec,
        cancel_response_timeout_sec=arguments.cancel_response_timeout_sec,
        auto_dock_arrival_topic=arguments.auto_dock_arrival_topic,
        auto_dock_status_topic=arguments.auto_dock_status_topic,
        auto_dock_stop_topic=arguments.auto_dock_stop_topic,
        auto_dock_drive_ready_topic=arguments.auto_dock_drive_ready_topic,
    )


def run_server(arguments, adapter_factory=None, http_server_factory=create_http_server):
    vehicle_status = VehicleStatus(
        robot_id=arguments.robot_id,
        battery_stale_sec=arguments.battery_stale_sec,
    )
    adapter = create_ros_vehicle_adapter(arguments) if adapter_factory is None else adapter_factory(arguments)
    relay_url = getattr(arguments, 'fleet_status_relay_url', None)
    status_reporter = (
        FleetStatusReporter(arguments.robot_id, relay_url)
        if relay_url
        else None
    )
    if hasattr(adapter, 'subscribe_battery'):
        adapter.subscribe_battery(arguments.battery_topic, vehicle_status.update_battery)
    if hasattr(adapter, 'configure_initial_pose_publisher'):
        adapter.configure_initial_pose_publisher(arguments.initial_pose_topic)
    service = VehicleCommandService(
        velocity=adapter,
        navigation=adapter,
        initial_pose=adapter,
        auto_dock=adapter,
        max_linear_x=arguments.max_linear_x,
        max_linear_y=getattr(arguments, 'max_linear_y', arguments.max_linear_x),
        max_angular_z=arguments.max_angular_z,
        max_hold_ms=arguments.max_hold_ms,
        fork_state_timeout_sec=getattr(arguments, 'fork_state_timeout_sec', 15.0),
        initial_pose_position_variance=arguments.initial_pose_position_variance,
        initial_pose_yaw_variance=arguments.initial_pose_yaw_variance,
        vehicle_status=vehicle_status,
        status_reporter=status_reporter,
    )
    if hasattr(adapter, 'configure_auto_dock'):
        adapter.configure_auto_dock(
            service.on_auto_dock_status,
            service.on_auto_dock_drive_ready,
        )
    if hasattr(adapter, 'configure_fork_state_subscription'):
        adapter.configure_fork_state_subscription(service.on_fork_state)
    service.report_current_status()
    http_server = http_server_factory(arguments.host, arguments.port, service)
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def handle_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, handle_sigterm)
    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            service.close()
            http_server.server_close()
            adapter.close()
        finally:
            try:
                if status_reporter is not None:
                    status_reporter.close()
            finally:
                signal.signal(signal.SIGTERM, previous_sigterm_handler)


def main(argv=None):
    run_server(parse_args(argv))


if __name__ == '__main__':
    main()
