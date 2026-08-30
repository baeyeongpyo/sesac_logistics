import importlib.util
import json
from pathlib import Path
import signal
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
SCRIPT = PACKAGE / 'vehicle_command_api.py'
INVENTORY_OPERATION_ID = '73d5b9af-5a12-4f34-a96c-5de116df1e8e'


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


class RecordingInitialPose:
    def __init__(self):
        self.messages = []

    def publish_initial_pose(self, pose, position_variance, yaw_variance):
        self.messages.append((pose, position_variance, yaw_variance))


class RecordingAutoDock:
    def __init__(self):
        self.arrival_messages = []
        self.stop_requests = 0

    def publish_arrival(self, payload):
        self.arrival_messages.append(payload)

    def stop(self):
        self.stop_requests += 1


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


class SigtermHttpServer(ReturningHttpServer):
    def __init__(self, handlers):
        super().__init__()
        self._handlers = handlers

    def serve_forever(self):
        self.served = True
        self._handlers[signal.SIGTERM](signal.SIGTERM, None)


class VehicleCommandApiServerTest(unittest.TestCase):
    def setUp(self):
        self.module = load_server_module()
        self.assertTrue(
            hasattr(self.module, 'VehicleCommandService'),
            'the standalone vehicle command service must exist',
        )
        self.now = [1700000000.125]
        self.vehicle_status = self.module.VehicleStatus(
            robot_id='robot_2',
            battery_stale_sec=3.0,
            clock=lambda: self.now[0],
        )
        self.velocity = RecordingVelocity()
        self.navigation = FakeNavigation()
        self.initial_pose_publisher = RecordingInitialPose()
        self.auto_dock = RecordingAutoDock()
        self.service = self.module.VehicleCommandService(
            velocity=self.velocity,
            navigation=self.navigation,
            initial_pose=self.initial_pose_publisher,
            auto_dock=self.auto_dock,
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
            vehicle_status=self.vehicle_status,
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

    def navigation_goal(self, operation_id=None, purpose=None):
        _, operation = self.get_json('/v1/operation-status')
        if operation['state'] == 'INIT':
            idle_status, idle = self.mark_idle()
            self.assertEqual(idle_status, 200)
            self.assertEqual(idle['state'], 'IDLE')
        payload = {'frame_id': 'map', 'x': 1.50, 'y': 0.0, 'yaw': 0.0}
        if operation_id is not None:
            payload['operation_id'] = operation_id
        if purpose is not None:
            payload['purpose'] = purpose
        return post_json(
            f'{self.base_url}/v1/navigation/goals',
            payload,
        )

    def initial_pose(self):
        return post_json(
            f'{self.base_url}/v1/localization/initial-pose',
            {'x': 1.50, 'y': 0.0, 'yaw': 0.0},
        )

    def mark_idle(self):
        return post_json(
            f'{self.base_url}/v1/operation/idle',
            {'reason': 'OPERATOR_CONFIRMED'},
        )

    def auto_dock_command(self, operation, operation_id=INVENTORY_OPERATION_ID):
        return post_json(
            f'{self.base_url}/v1/auto-dock',
            {
                'operation_id': operation_id,
                'operation': operation,
                'product_type': 'NORMAL',
                'location': 'DOCK_1',
                'target': {'type': 'NEAREST'},
            },
        )

    def test_startup_stays_init_until_operator_marks_idle(self):
        """AMCL initial pose alone must not authorise a newly started vehicle."""
        _, initial_status = self.get_json('/v1/operation-status')
        initial_pose_status, _ = self.initial_pose()
        _, after_pose = self.get_json('/v1/operation-status')
        idle_status, idle = self.mark_idle()

        self.assertEqual(initial_status['state'], 'INIT')
        self.assertEqual(initial_pose_status, 202)
        self.assertEqual(after_pose['state'], 'INIT')
        self.assertEqual(idle_status, 200)
        self.assertEqual(idle, {
            'operation_id': None,
            'previous_operation_id': None,
            'state': 'IDLE',
            'previous_state': 'INIT',
            'detail': 'OPERATOR_READY',
        })

    def test_failed_operation_keeps_current_id_until_idle_is_explicit(self):
        """Failure must retain the server operation ID until an operator clears it."""
        _, goal = self.navigation_goal()
        self.navigation.complete(goal['attempt_id'], 'FAILED')

        _, failed = self.get_json('/v1/operation-status')
        idle_status, idle = self.mark_idle()

        self.assertEqual(failed, {
            'operation_id': goal['operation_id'],
            'previous_operation_id': None,
            'state': 'FAILED',
            'previous_state': 'DRIVE',
            'detail': 'NAVIGATION_FAILED',
        })
        self.assertEqual(idle_status, 200)
        self.assertEqual(idle, {
            'operation_id': None,
            'previous_operation_id': goal['operation_id'],
            'state': 'IDLE',
            'previous_state': 'FAILED',
            'detail': 'OPERATOR_READY',
        })

    def test_inventory_operation_id_uses_a_distinct_nav2_attempt(self):
        """One Inventory workflow ID may be retried through different Nav2 attempts."""
        _, drive = self.navigation_goal(INVENTORY_OPERATION_ID)

        self.assertEqual(drive, {
            'operation_id': INVENTORY_OPERATION_ID,
            'attempt_id': drive['attempt_id'],
            'state': 'DRIVE',
        })
        self.assertNotEqual(drive['attempt_id'], INVENTORY_OPERATION_ID)
        self.assertIsInstance(uuid.UUID(drive['attempt_id']), uuid.UUID)
        self.assertEqual(self.navigation.goals, [(
            drive['attempt_id'],
            {'frame_id': 'map', 'x': 1.5, 'y': 0.0, 'yaw': 0.0},
        )])
        self.navigation.complete(drive['attempt_id'], 'COMPLETED')
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status['previous_operation_id'], INVENTORY_OPERATION_ID)

    def test_auto_dock_uses_drive_ready_not_ready_status_for_pick_completion(self):
        """Auto Dock READY is not complete until its drive_ready event arrives."""
        self.mark_idle()
        status, response = self.auto_dock_command('PICK')

        self.assertEqual(status, 202)
        self.assertEqual(response, {
            'operation_id': INVENTORY_OPERATION_ID,
            'state': 'PICKING',
        })
        self.assertEqual(self.auto_dock.arrival_messages, [{
            'status': 'SUCCEEDED',
            'location': 'DOCK_1',
            'operation': 'PICK',
            'product_type': 'NORMAL',
            'target': {'type': 'NEAREST'},
        }])
        self.service.on_auto_dock_status({'state': 'READY', 'operation': 'PICK'})
        self.assertEqual(self.service.operation_status()['state'], 'PICKING')
        self.service.on_auto_dock_drive_ready()
        self.assertEqual(self.service.operation_status(), {
            'operation_id': INVENTORY_OPERATION_ID,
            'previous_operation_id': None,
            'state': 'PICK_COMPLETE',
            'previous_state': 'PICKING',
            'detail': 'AUTO_DOCK_PICK_COMPLETED',
        })

    def test_auto_dock_place_completion_returns_to_idle(self):
        """Place completion is reported before the snapshot is cleared to IDLE."""
        self.mark_idle()
        self.auto_dock_command('PICK')
        self.service.on_auto_dock_drive_ready()
        self.auto_dock_command('PLACE')
        self.service.on_auto_dock_drive_ready()

        self.assertEqual(self.service.operation_status(), {
            'operation_id': None,
            'previous_operation_id': INVENTORY_OPERATION_ID,
            'state': 'IDLE',
            'previous_state': 'PLACE_COMPLETE',
            'detail': 'AUTO_DOCK_PLACE_COMPLETED',
        })

    def test_loaded_vehicle_requires_place_purpose_before_drive(self):
        """A loaded vehicle must not accept an unspecified or Pick-purpose drive."""
        self.mark_idle()
        self.auto_dock_command('PICK')
        self.service.on_auto_dock_drive_ready()

        rejected_status, rejected = self.navigation_goal(INVENTORY_OPERATION_ID)
        accepted_status, accepted = self.navigation_goal(
            INVENTORY_OPERATION_ID,
            purpose='PLACE',
        )

        self.assertEqual(rejected_status, 409)
        self.assertEqual(rejected, {'error': 'DRIVE_PURPOSE_MUST_BE_PLACE'})
        self.assertEqual(accepted_status, 202)
        self.assertEqual(accepted['state'], 'DRIVE')

    def test_auto_dock_error_keeps_operation_for_operator_recovery(self):
        """Dock errors must retain the Inventory operation until explicit recovery."""
        self.mark_idle()
        self.auto_dock_command('PICK')
        self.service.on_auto_dock_status({
            'state': 'ERROR',
            'reason': 'fork_failed',
            'operation': 'PICK',
        })

        self.assertEqual(self.service.operation_status(), {
            'operation_id': INVENTORY_OPERATION_ID,
            'previous_operation_id': None,
            'state': 'FAILED',
            'previous_state': 'PICKING',
            'detail': 'AUTO_DOCK_ERROR:fork_failed',
        })

    def test_stop_cancels_auto_dock_through_its_stop_topic(self):
        """A stop command must signal the running Auto Dock FSM before cancellation."""
        self.mark_idle()
        self.auto_dock_command('PICK')

        status, response = post_json(f'{self.base_url}/v1/stop', {})

        self.assertEqual(status, 200)
        self.assertEqual(self.auto_dock.stop_requests, 1)
        self.assertEqual(response, {
            'operation_id': INVENTORY_OPERATION_ID,
            'state': 'CANCELLED',
            'cancel_requested': True,
        })
        self.assertEqual(self.service.operation_status(), {
            'operation_id': INVENTORY_OPERATION_ID,
            'previous_operation_id': None,
            'state': 'CANCELLED',
            'previous_state': 'PICKING',
            'detail': 'STOP_REQUESTED',
        })

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
                '/v1/vehicle-status',
                '/v1/operation/idle',
                '/v1/cmd-vel',
                '/v1/navigation/goals',
                '/v1/auto-dock',
                '/v1/navigation/cancel',
                '/v1/localization/initial-pose',
                '/v1/stop',
            },
        )
        self.assertEqual(status_code, 200)
        self.assertEqual(operation_status, {
            'operation_id': None,
            'previous_operation_id': None,
            'state': 'INIT',
            'previous_state': None,
            'detail': 'VEHICLE_BOOTED',
        })

    def test_initial_pose_publishes_map_pose_with_configured_covariance(self):
        """Removing initial pose publishing must fail AMCL initialization."""
        status, response = self.initial_pose()

        self.assertEqual(status, 202)
        self.assertIsInstance(uuid.UUID(response['operation_id']), uuid.UUID)
        self.assertEqual(response, {
            'operation_id': response['operation_id'],
            'state': 'INITIAL_POSE_PUBLISHED',
            'frame_id': 'map',
            'x': 1.5,
            'y': 0.0,
            'yaw': 0.0,
        })
        self.assertEqual(self.initial_pose_publisher.messages, [(
            {'frame_id': 'map', 'x': 1.5, 'y': 0.0, 'yaw': 0.0},
            0.25,
            0.0685,
        )])

    def test_initial_pose_rejects_while_navigation_is_active(self):
        """Repositioning AMCL during Nav2 motion must not publish a new pose."""
        self.navigation_goal()

        status, response = self.initial_pose()

        self.assertEqual(status, 409)
        self.assertEqual(response, {'error': 'VEHICLE_MOTION_ACTIVE'})
        self.assertEqual(self.initial_pose_publisher.messages, [])

    def test_initial_pose_rejects_while_manual_velocity_is_active(self):
        """Repositioning AMCL during manual motion must not publish a new pose."""
        post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.05, 'angular_z': 0.0, 'hold_ms': 1000},
        )

        status, response = self.initial_pose()

        self.assertEqual(status, 409)
        self.assertEqual(response, {'error': 'VEHICLE_MOTION_ACTIVE'})
        self.assertEqual(self.initial_pose_publisher.messages, [])

    def test_vehicle_status_reports_configured_robot_battery_and_operation(self):
        """Dropping configured identity or battery freshness must fail fleet monitoring."""
        self.vehicle_status.update_battery(8354)
        _, goal = self.navigation_goal()

        status, vehicle_status = self.get_json('/v1/vehicle-status')

        self.assertEqual(status, 200)
        self.assertEqual(vehicle_status, {
            'robot_id': 'robot_2',
            'battery': {
                'raw_value': 8354,
                'received_at': '2023-11-14T22:13:20.125Z',
                'stale': False,
            },
            'operation': {
                'operation_id': goal['operation_id'],
                'previous_operation_id': None,
                'state': 'DRIVE',
                'previous_state': 'IDLE',
                'detail': 'NAVIGATION_GOAL_ACCEPTED',
            },
        })

    def test_vehicle_status_marks_missing_or_expired_battery_stale(self):
        """Treating missing telemetry as fresh must fail operational monitoring."""
        _, missing_battery = self.get_json('/v1/vehicle-status')
        self.vehicle_status.update_battery(8354)
        self.now[0] += 3.001
        _, expired_battery = self.get_json('/v1/vehicle-status')

        self.assertEqual(missing_battery['battery'], {
            'raw_value': None,
            'received_at': None,
            'stale': True,
        })
        self.assertEqual(expired_battery['battery'], {
            'raw_value': 8354,
            'received_at': '2023-11-14T22:13:20.125Z',
            'stale': True,
        })

    def test_navigation_goal_returns_vehicle_generated_operation_id_and_tracks_goal(self):
        """Removing vehicle-side IDs or changing coordinates must fail the navigation contract."""
        status, body = self.navigation_goal()

        self.assertEqual(status, 202)
        self.assertEqual(body['state'], 'DRIVE')
        operation_id = body['operation_id']
        self.assertIsInstance(uuid.UUID(operation_id), uuid.UUID)
        self.assertIsInstance(uuid.UUID(body['attempt_id']), uuid.UUID)
        self.assertNotEqual(body['attempt_id'], operation_id)
        self.assertEqual(self.navigation.goals, [(
            body['attempt_id'],
            {'frame_id': 'map', 'x': 1.5, 'y': 0.0, 'yaw': 0.0},
        )])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': operation_id,
            'previous_operation_id': None,
            'state': 'DRIVE',
            'previous_state': 'IDLE',
            'detail': 'NAVIGATION_GOAL_ACCEPTED',
        })

    def test_navigation_terminal_result_updates_the_current_operation_status(self):
        """Dropping Nav2 terminal callbacks must fail status monitoring."""
        _, body = self.navigation_goal()
        self.navigation.complete(body['attempt_id'], 'COMPLETED')

        status, operation_status = self.get_json('/v1/operation-status')

        self.assertEqual(status, 200)
        self.assertEqual(operation_status, {
            'operation_id': None,
            'previous_operation_id': body['operation_id'],
            'state': 'IDLE',
            'previous_state': 'DRIVE',
            'detail': 'NAVIGATION_SUCCEEDED',
        })

    def test_cancel_marks_target_operation_cancelled_after_nav2_result(self):
        """Ignoring the requested operation ID must fail targeted cancellation."""
        _, body = self.navigation_goal()
        status, response = post_json(
            f'{self.base_url}/v1/navigation/cancel',
            {'operation_id': body['operation_id']},
        )
        _, operation_status = self.get_json('/v1/operation-status')

        self.assertEqual(status, 202)
        self.assertEqual(response, {
            'operation_id': body['operation_id'],
            'state': 'CANCELLED',
        })
        self.assertEqual(self.navigation.cancel_requests, [body['attempt_id']])
        self.assertEqual(operation_status, {
            'operation_id': body['operation_id'],
            'previous_operation_id': None,
            'state': 'CANCELLED',
            'previous_state': 'DRIVE',
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
            'state': 'CANCELLED',
            'cancel_requested': False,
        })
        self.assertEqual(self.velocity.messages[-1], (0.0, 0.0))
        self.assertEqual(self.navigation.cancel_requests, [body['attempt_id']])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': body['operation_id'],
            'previous_operation_id': None,
            'state': 'CANCELLED',
            'previous_state': 'MANUAL',
            'detail': 'STOP_REQUESTED',
        })

    def test_stop_returns_stopped_when_navigation_cancel_raises(self):
        """Propagating a cancel exception after zeroing velocity must fail the stop safety contract."""
        _, body = self.navigation_goal()

        def raise_cancel_error(operation_id):
            raise RuntimeError(f'cancel failed for {operation_id}')

        original_cancel = self.navigation.cancel
        self.navigation.cancel = raise_cancel_error
        try:
            response = self.service.stop()
        except RuntimeError as error:
            self.fail(f'stop must absorb a navigation cancel failure: {error}')
        finally:
            self.navigation.cancel = original_cancel

        self.assertEqual(response, {
            'operation_id': body['operation_id'],
            'state': 'CANCELLED',
            'cancel_requested': False,
        })
        self.assertEqual(self.velocity.messages[-1], (0.0, 0.0))
        self.assertEqual(self.service.operation_status(), {
            'operation_id': body['operation_id'],
            'previous_operation_id': None,
            'state': 'CANCELLED',
            'previous_state': 'DRIVE',
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
        self.mark_idle()

        status, body = self.navigation_goal()

        self.assertEqual(status, 503)
        self.assertEqual(body, {'error': 'NAVIGATION_SERVER_UNAVAILABLE'})
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertIsInstance(uuid.UUID(operation_status['operation_id']), uuid.UUID)
        self.assertEqual(operation_status, {
            'operation_id': operation_status['operation_id'],
            'previous_operation_id': None,
            'state': 'FAILED',
            'previous_state': 'DRIVE',
            'detail': 'NAVIGATION_SERVER_UNAVAILABLE',
        })

    def test_manual_velocity_is_bounded_and_expires_to_idle(self):
        """Removing command bounds or the hold-expiry zero must fail direct control safety."""
        self.mark_idle()
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
            'previous_operation_id': None,
            'state': 'IDLE',
            'previous_state': 'INIT',
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
        self.assertEqual(self.navigation.cancel_requests, [body['attempt_id']])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': body['operation_id'],
            'previous_operation_id': None,
            'state': 'MANUAL',
            'previous_state': 'CANCELLED',
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
            '--host', '--port', '--robot-id', '--cmd-vel-topic', '--action-name',
            '--battery-topic', '--battery-stale-sec',
            '--initial-pose-topic', '--initial-pose-position-variance',
            '--initial-pose-yaw-variance',
            '--auto-dock-arrival-topic', '--auto-dock-status-topic',
            '--auto-dock-stop-topic', '--auto-dock-drive-ready-topic',
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
            robot_id='robot_2',
            cmd_vel_topic='/cmd_vel',
            action_name='/navigate_to_pose',
            battery_topic='/ros_robot_controller/battery',
            battery_stale_sec=3.0,
            initial_pose_topic='/initialpose',
            initial_pose_position_variance=0.25,
            initial_pose_yaw_variance=0.0685,
            auto_dock_arrival_topic=None,
            auto_dock_status_topic=None,
            auto_dock_stop_topic=None,
            auto_dock_drive_ready_topic=None,
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

    def test_sigterm_runs_the_same_safe_shutdown_path_as_keyboard_interrupt(self):
        """Skipping service cleanup on SIGTERM can leave a moving vehicle unmanaged."""
        self.assertTrue(
            hasattr(self.module, 'signal'),
            'the direct runner must install a SIGTERM handler for safe cleanup',
        )
        adapter = ClosingFakeAdapter()
        handlers = {}
        http_server = SigtermHttpServer(handlers)
        arguments = SimpleNamespace(
            host='0.0.0.0',
            port=8082,
            robot_id='robot_2',
            cmd_vel_topic='/cmd_vel',
            action_name='/navigate_to_pose',
            battery_topic='/ros_robot_controller/battery',
            battery_stale_sec=3.0,
            initial_pose_topic='/initialpose',
            initial_pose_position_variance=0.25,
            initial_pose_yaw_variance=0.0685,
            auto_dock_arrival_topic=None,
            auto_dock_status_topic=None,
            auto_dock_stop_topic=None,
            auto_dock_drive_ready_topic=None,
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
            action_server_timeout_sec=1.0,
            goal_response_timeout_sec=3.0,
            cancel_response_timeout_sec=3.0,
        )
        original_signal = self.module.signal.signal

        def capture_signal(signum, handler):
            previous = handlers.get(signum, signal.SIG_DFL)
            handlers[signum] = handler
            return previous

        self.module.signal.signal = capture_signal
        try:
            self.module.run_server(
                arguments,
                adapter_factory=lambda args: adapter,
                http_server_factory=lambda host, port, service: http_server,
            )
        finally:
            self.module.signal.signal = original_signal

        self.assertTrue(http_server.served)
        self.assertTrue(http_server.closed)
        self.assertEqual(adapter.messages, [(0.0, 0.0)])
        self.assertTrue(adapter.closed)

    def test_default_runner_expands_cli_arguments_for_ros_adapter(self):
        """Passing the argparse object directly to RosVehicleAdapter must fail startup."""
        adapter = ClosingFakeAdapter()
        http_server = ReturningHttpServer()
        arguments = SimpleNamespace(
            host='0.0.0.0',
            port=8082,
            robot_id='robot_2',
            cmd_vel_topic='/cmd_vel',
            action_name='/navigate_to_pose',
            battery_topic='/ros_robot_controller/battery',
            battery_stale_sec=3.0,
            initial_pose_topic='/initialpose',
            initial_pose_position_variance=0.25,
            initial_pose_yaw_variance=0.0685,
            auto_dock_arrival_topic=None,
            auto_dock_status_topic=None,
            auto_dock_stop_topic=None,
            auto_dock_drive_ready_topic=None,
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
            action_server_timeout_sec=1.0,
            goal_response_timeout_sec=3.0,
            cancel_response_timeout_sec=3.0,
        )
        captured = {}

        def construct_adapter(
            robot_id,
            cmd_vel_topic,
            action_name,
            action_server_timeout_sec,
            goal_response_timeout_sec,
            cancel_response_timeout_sec,
            auto_dock_arrival_topic,
            auto_dock_status_topic,
            auto_dock_stop_topic,
            auto_dock_drive_ready_topic,
        ):
            captured.update({
                'robot_id': robot_id,
                'cmd_vel_topic': cmd_vel_topic,
                'action_name': action_name,
                'action_server_timeout_sec': action_server_timeout_sec,
                'goal_response_timeout_sec': goal_response_timeout_sec,
                'cancel_response_timeout_sec': cancel_response_timeout_sec,
                'auto_dock_arrival_topic': auto_dock_arrival_topic,
                'auto_dock_status_topic': auto_dock_status_topic,
                'auto_dock_stop_topic': auto_dock_stop_topic,
                'auto_dock_drive_ready_topic': auto_dock_drive_ready_topic,
            })
            return adapter

        original_adapter = self.module.RosVehicleAdapter
        self.module.RosVehicleAdapter = construct_adapter
        try:
            self.module.run_server(
                arguments,
                http_server_factory=lambda host, port, service: http_server,
            )
        except Exception as error:
            self.fail(f'default runner must construct the ROS adapter: {error}')
        finally:
            self.module.RosVehicleAdapter = original_adapter

        self.assertEqual(captured, {
            'robot_id': 'robot_2',
            'cmd_vel_topic': '/cmd_vel',
            'action_name': '/navigate_to_pose',
            'action_server_timeout_sec': 1.0,
            'goal_response_timeout_sec': 3.0,
            'cancel_response_timeout_sec': 3.0,
            'auto_dock_arrival_topic': None,
            'auto_dock_status_topic': None,
            'auto_dock_stop_topic': None,
            'auto_dock_drive_ready_topic': None,
        })


if __name__ == '__main__':
    unittest.main()
