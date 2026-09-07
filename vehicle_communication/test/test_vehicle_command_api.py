import importlib.util
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request
from urllib.request import urlopen
import unittest
from unittest.mock import patch
import uuid


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'vehicle_command_api.py'
INVENTORY_OPERATION_ID = '73d5b9af-5a12-4f34-a96c-5de116df1e8e'
POC_MISSION_ID = '3e829a02-7601-4b9f-afdf-3dbd84737828'


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
        self.fork_messages = []

    def publish(self, *values):
        self.messages.append(values)

    def publish_fork_command(self, command):
        self.fork_messages.append(command)


class FakeNavigation:
    def __init__(self, available=True):
        self.available = available
        self.goals = []
        self.waypoints = []
        self.cancel_requests = []
        self._callbacks = {}

    def submit_goal(self, operation_id, goal, on_terminal):
        if not self.available:
            return {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}
        self.goals.append((operation_id, goal))
        self._callbacks[operation_id] = on_terminal
        return {'accepted': True}

    def submit_waypoints(self, operation_id, waypoints, on_terminal):
        if not self.available:
            return {'accepted': False, 'error': 'NAVIGATION_SERVER_UNAVAILABLE'}
        self.waypoints.append((operation_id, waypoints))
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


class RecordingStatusReporter:
    def __init__(self):
        self.reports = []

    def report(self, payload):
        self.reports.append(payload)


class RecordingPallet3Supervisor:
    def __init__(self):
        self.starts = []
        self.stop_calls = 0

    def start(self, operation_id, robot_id, pick_mode, fleet_manager_url):
        self.starts.append((operation_id, robot_id, pick_mode, fleet_manager_url))
        return 1234

    def stop(self):
        self.stop_calls += 1


class FleetStatusReporterTest(unittest.TestCase):
    def test_posts_state_payload_to_bridge_for_configured_robot(self):
        module = load_server_module()

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                size = int(self.headers['Content-Length'])
                self.server.received = (self.path, json.loads(self.rfile.read(size)))
                body = b'{"status":"ok"}'
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
        payload = {
            'state': 'INIT',
            'previous_state': None,
            'operation_id': None,
            'attempt_id': None,
            'source': 'VEHICLE',
            'detail': 'VEHICLE_BOOTED',
            'observed_at': '2026-08-31T12:00:00Z',
        }
        reporter = None
        try:
            reporter = module.FleetStatusReporter(
                'robot_2', f'http://127.0.0.1:{server.server_port}'
            )
            reporter.report(payload)
            deadline = time.monotonic() + 1
            while not hasattr(server, 'received') and time.monotonic() < deadline:
                time.sleep(0.01)
        finally:
            if reporter is not None:
                reporter.close()
            server.shutdown()
            server.server_close()
            thread.join()

        self.assertEqual(
            server.received,
            ('/api/v1/vehicle-status/robot_2', payload),
        )


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
        self.status_reporter = RecordingStatusReporter()
        self.pallet3_supervisor = RecordingPallet3Supervisor()
        self.service = self.module.VehicleCommandService(
            velocity=self.velocity,
            navigation=self.navigation,
            initial_pose=self.initial_pose_publisher,
            auto_dock=self.auto_dock,
            max_linear_x=1.0,
            max_linear_y=1.0,
            max_angular_z=1.0,
            max_hold_ms=1000,
            vehicle_status=self.vehicle_status,
            status_reporter=self.status_reporter,
        )
        self.service._pallet3_supervisor = self.pallet3_supervisor
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

    def test_manual_pallet3_pick_requires_fork_up_and_place_clears_operation_before_return(self):
        idle_status, _idle = self.mark_idle()
        self.assertEqual(idle_status, 200)
        start_status, started = post_json(
            f'{self.base_url}/v1/missions/pallet3',
            {
                'operation_id': INVENTORY_OPERATION_ID,
                'robot_id': 'robot_2',
                'pick_mode': 'manual',
                'fleet_manager_url': 'http://fleet.example:8090',
            },
        )
        self.assertEqual(start_status, 202)
        self.assertEqual(started['state'], 'STARTING')
        self.assertEqual(self.service.operation_status()['operation_id'], INVENTORY_OPERATION_ID)
        self.assertEqual(self.service.operation_status()['detail'], 'PALLET3_MISSION_STARTED')
        self.assertEqual(
            self.pallet3_supervisor.starts,
            [(INVENTORY_OPERATION_ID, 'robot_2', 'manual', 'http://fleet.example:8090')],
        )

        self.assertEqual(
            post_json(f'{self.base_url}/v1/missions/pallet3/{INVENTORY_OPERATION_ID}/picked', {})[0],
            409,
        )
        post_json(f'{self.base_url}/v1/fork/up', {'operation_id': INVENTORY_OPERATION_ID})
        self.service.on_fork_state('{"state":"UP_COMPLETE","error":""}')
        self.assertEqual(self.service.operation_status()['detail'], 'FORK_UP_COMPLETE')
        picked_status, picked = post_json(
            f'{self.base_url}/v1/missions/pallet3/{INVENTORY_OPERATION_ID}/picked', {}
        )
        self.assertEqual(picked_status, 200)
        self.assertEqual(picked['state'], 'PICK_COMPLETE')

        post_json(f'{self.base_url}/v1/fork/down', {'operation_id': INVENTORY_OPERATION_ID})
        self.service.on_fork_state('{"state":"DOWN_COMPLETE","error":""}')
        placed_status, placed = post_json(
            f'{self.base_url}/v1/missions/pallet3/{INVENTORY_OPERATION_ID}/placed', {}
        )
        self.assertEqual(placed_status, 200)
        self.assertIsNone(placed['operation_id'])
        self.assertEqual(placed['detail'], 'PLACE_COMPLETED')

    def test_stop_terminates_active_pallet3_shell_process_group(self):
        self.mark_idle()
        post_json(
            f'{self.base_url}/v1/missions/pallet3',
            {
                'operation_id': INVENTORY_OPERATION_ID,
                'robot_id': 'robot_2',
                'pick_mode': 'auto_dock',
                'fleet_manager_url': 'http://fleet.example:8090',
            },
        )

        stop_status, _stopped = post_json(f'{self.base_url}/v1/stop', {})

        self.assertEqual(stop_status, 200)
        self.assertEqual(self.pallet3_supervisor.stop_calls, 1)

    def test_navigation_cancel_terminates_active_pallet3_shell_process_group(self):
        self.mark_idle()
        post_json(
            f'{self.base_url}/v1/missions/pallet3',
            {
                'operation_id': INVENTORY_OPERATION_ID,
                'robot_id': 'robot_2',
                'pick_mode': 'manual',
                'fleet_manager_url': 'http://fleet.example:8090',
            },
        )
        route_status, route = post_json(
            f'{self.base_url}/v1/navigation/waypoints',
            {
                'operation_id': INVENTORY_OPERATION_ID,
                'purpose': 'PLACE',
                'waypoints': [{'frame_id': 'map', 'x': -0.44, 'y': -0.9, 'yaw': 0.0}],
            },
        )
        self.assertEqual(route_status, 202)

        cancel_status, _cancelled = post_json(
            f'{self.base_url}/v1/navigation/cancel',
            {'operation_id': route['operation_id']},
        )

        self.assertEqual(cancel_status, 202)
        self.assertEqual(self.pallet3_supervisor.stop_calls, 1)

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

    def navigation_waypoints(self, operation_id=None, purpose=None):
        _, operation = self.get_json('/v1/operation-status')
        if operation['state'] == 'INIT':
            idle_status, idle = self.mark_idle()
            self.assertEqual(idle_status, 200)
            self.assertEqual(idle['state'], 'IDLE')
        payload = {
            'waypoints': [
                {'frame_id': 'map', 'x': 1.50, 'y': 0.0, 'yaw': 0.0},
                {'frame_id': 'map', 'x': 2.00, 'y': 0.5, 'yaw': 1.57},
            ],
        }
        if operation_id is not None:
            payload['operation_id'] = operation_id
        if purpose is not None:
            payload['purpose'] = purpose
        return post_json(
            f'{self.base_url}/v1/navigation/waypoints',
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

    def wait_until(self, predicate):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail('timed out waiting for expected vehicle status')

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

    def test_current_init_state_can_be_reported_after_startup(self):
        self.service.report_current_status()

        report = self.status_reporter.reports[-1]

        self.assertEqual(report['state'], 'INIT')
        self.assertIsNone(report['previous_state'])
        self.assertIsNone(report['operation_id'])
        self.assertIsNone(report['attempt_id'])
        self.assertEqual(report['source'], 'VEHICLE')
        self.assertEqual(report['detail'], 'VEHICLE_BOOTED')

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

    def test_waypoint_navigation_submits_all_poses_to_follow_waypoints(self):
        """A route must use FollowWaypoints without changing legacy goal handling."""
        status, drive = self.navigation_waypoints()

        self.assertEqual(status, 202)
        self.assertEqual(drive, {
            'operation_id': drive['operation_id'],
            'attempt_id': drive['attempt_id'],
            'state': 'DRIVE',
        })
        self.assertIsInstance(uuid.UUID(drive['operation_id']), uuid.UUID)
        self.assertIsInstance(uuid.UUID(drive['attempt_id']), uuid.UUID)
        self.assertEqual(self.navigation.goals, [])
        self.assertEqual(self.navigation.waypoints, [(
            drive['attempt_id'],
            [
                {'frame_id': 'map', 'x': 1.5, 'y': 0.0, 'yaw': 0.0},
                {'frame_id': 'map', 'x': 2.0, 'y': 0.5, 'yaw': 1.57},
            ],
        )])

        self.navigation.complete(drive['attempt_id'], 'COMPLETED')

        _, operation = self.get_json('/v1/operation-status')
        self.assertEqual(operation, {
            'operation_id': None,
            'previous_operation_id': drive['operation_id'],
            'state': 'IDLE',
            'previous_state': 'DRIVE',
            'detail': 'NAVIGATION_SUCCEEDED',
        })

    def test_nav2_reports_drive_only_after_goal_acceptance(self):
        reporter = RecordingStatusReporter()
        test_case = self

        class AcceptanceInspectingNavigation(FakeNavigation):
            def submit_goal(self, operation_id, goal, on_terminal):
                test_case.assertEqual(reporter.reports, [])
                return super().submit_goal(operation_id, goal, on_terminal)

        navigation = AcceptanceInspectingNavigation()
        service = self.module.VehicleCommandService(
            velocity=RecordingVelocity(),
            navigation=navigation,
            max_linear_x=0.10,
            max_angular_z=0.50,
            max_hold_ms=1000,
            status_reporter=reporter,
        )
        service.mark_idle({'reason': 'OPERATOR_CONFIRMED'})
        reporter.reports.clear()

        response = service.navigation_goal({
            'operation_id': INVENTORY_OPERATION_ID,
            'purpose': 'PICK',
            'x': 1.5,
            'y': 0.0,
            'yaw': 0.0,
        })

        self.assertEqual(response['state'], 'DRIVE')
        self.assertEqual(reporter.reports[-1]['state'], 'DRIVE')
        self.assertEqual(reporter.reports[-1]['source'], 'NAV2')
        self.assertEqual(reporter.reports[-1]['detail'], 'NAVIGATION_STARTED')
        self.assertEqual(reporter.reports[-1]['operation_id'], INVENTORY_OPERATION_ID)
        self.assertEqual(reporter.reports[-1]['attempt_id'], response['attempt_id'])

    def test_nav2_success_reports_wait_with_the_completed_operation(self):
        _, goal = self.navigation_goal(INVENTORY_OPERATION_ID)
        self.navigation.complete(goal['attempt_id'], 'COMPLETED')

        report = self.status_reporter.reports[-1]

        self.assertEqual(report['state'], 'WAIT')
        self.assertEqual(report['previous_state'], 'DRIVE')
        self.assertEqual(report['operation_id'], INVENTORY_OPERATION_ID)
        self.assertEqual(report['attempt_id'], goal['attempt_id'])
        self.assertEqual(report['source'], 'NAV2')
        self.assertEqual(report['detail'], 'NAVIGATION_SUCCEEDED')

    def test_nav2_failure_reports_fail(self):
        _, goal = self.navigation_goal(INVENTORY_OPERATION_ID)
        self.navigation.complete(goal['attempt_id'], 'FAILED')

        report = self.status_reporter.reports[-1]

        self.assertEqual(report['state'], 'FAIL')
        self.assertEqual(report['previous_state'], 'DRIVE')
        self.assertEqual(report['operation_id'], INVENTORY_OPERATION_ID)
        self.assertEqual(report['attempt_id'], goal['attempt_id'])
        self.assertEqual(report['source'], 'NAV2')
        self.assertEqual(report['detail'], 'NAVIGATION_FAILED')

    def test_navigation_cancel_reports_fail_from_api(self):
        _, goal = self.navigation_goal(INVENTORY_OPERATION_ID)

        status, _ = post_json(
            f'{self.base_url}/v1/navigation/cancel',
            {'operation_id': INVENTORY_OPERATION_ID},
        )
        report = self.status_reporter.reports[-1]

        self.assertEqual(status, 202)
        self.assertEqual(report['state'], 'FAIL')
        self.assertEqual(report['previous_state'], 'DRIVE')
        self.assertEqual(report['operation_id'], INVENTORY_OPERATION_ID)
        self.assertEqual(report['attempt_id'], goal['attempt_id'])
        self.assertEqual(report['source'], 'API')
        self.assertEqual(report['detail'], 'API_NAVIGATION_CANCEL')

    def test_stop_reports_fail_from_api(self):
        _, goal = self.navigation_goal(INVENTORY_OPERATION_ID)

        status, _ = post_json(f'{self.base_url}/v1/stop', {})
        report = self.status_reporter.reports[-1]

        self.assertEqual(status, 200)
        self.assertEqual(report['state'], 'FAIL')
        self.assertEqual(report['previous_state'], 'DRIVE')
        self.assertEqual(report['operation_id'], INVENTORY_OPERATION_ID)
        self.assertEqual(report['attempt_id'], goal['attempt_id'])
        self.assertEqual(report['source'], 'API')
        self.assertEqual(report['detail'], 'API_STOP')

    def test_goal_navigation_restarts_after_an_operator_stop(self):
        _, stopped_goal = self.navigation_goal()
        stop_status, stop = post_json(f'{self.base_url}/v1/stop', {})
        restart_status, restarted_goal = self.navigation_goal()

        self.assertEqual(stop_status, 200)
        self.assertEqual(stop['state'], 'CANCELLED')
        self.assertEqual(restart_status, 202)
        self.assertEqual(restarted_goal['state'], 'DRIVE')
        self.assertNotEqual(restarted_goal['attempt_id'], stopped_goal['attempt_id'])

    def test_follow_waypoints_restarts_after_an_operator_stop(self):
        _, stopped_goal = self.navigation_goal()
        stop_status, stop = post_json(f'{self.base_url}/v1/stop', {})
        restart_status, restarted_route = self.navigation_waypoints()

        self.assertEqual(stop_status, 200)
        self.assertEqual(stop['state'], 'CANCELLED')
        self.assertEqual(restart_status, 202)
        self.assertEqual(restarted_route['state'], 'DRIVE')
        self.assertNotEqual(restarted_route['attempt_id'], stopped_goal['attempt_id'])

    def test_navigation_cancel_does_not_authorize_a_new_drive(self):
        _, goal = self.navigation_goal()
        cancel_status, _ = post_json(
            f'{self.base_url}/v1/navigation/cancel',
            {'operation_id': goal['operation_id']},
        )
        restart_status, restart = self.navigation_goal()

        self.assertEqual(cancel_status, 202)
        self.assertEqual(restart_status, 409)
        self.assertEqual(restart['error'], 'OPERATION_NOT_READY_FOR_DRIVE')

    def test_navigation_failure_does_not_authorize_a_new_drive(self):
        _, goal = self.navigation_goal()
        self.navigation.complete(goal['attempt_id'], 'FAILED')

        goal_restart_status, goal_restart = self.navigation_goal()
        route_restart_status, route_restart = self.navigation_waypoints()

        self.assertEqual(goal_restart_status, 409)
        self.assertEqual(goal_restart['error'], 'OPERATION_NOT_READY_FOR_DRIVE')
        self.assertEqual(route_restart_status, 409)
        self.assertEqual(route_restart['error'], 'OPERATION_NOT_READY_FOR_DRIVE')

    def test_stop_from_wait_reports_fail_from_api(self):
        self.mark_idle()

        status, _ = post_json(f'{self.base_url}/v1/stop', {})
        report = self.status_reporter.reports[-1]

        self.assertEqual(status, 200)
        self.assertEqual(report['state'], 'FAIL')
        self.assertEqual(report['previous_state'], 'WAIT')
        self.assertIsNone(report['operation_id'])
        self.assertIsNone(report['attempt_id'])
        self.assertEqual(report['source'], 'API')
        self.assertEqual(report['detail'], 'API_STOP')

    def test_auto_dock_reports_pick_only_after_running_status(self):
        self.mark_idle()
        reports_before_command = len(self.status_reporter.reports)

        status, _ = self.auto_dock_command('PICK')

        self.assertEqual(status, 202)
        self.assertEqual(len(self.status_reporter.reports), reports_before_command)

        self.service.on_auto_dock_status({
            'state': 'SEARCHING',
            'operation': 'PICK',
        })
        report = self.status_reporter.reports[-1]

        self.assertEqual(report['state'], 'PICK')
        self.assertEqual(report['previous_state'], 'WAIT')
        self.assertEqual(report['operation_id'], INVENTORY_OPERATION_ID)
        self.assertIsNone(report['attempt_id'])
        self.assertEqual(report['source'], 'AUTO_DOCK')
        self.assertEqual(report['detail'], 'AUTO_DOCK_SEARCHING')

    def test_auto_dock_drive_ready_reports_wait(self):
        self.mark_idle()
        self.auto_dock_command('PICK')
        self.service.on_auto_dock_status({'state': 'ALIGNING', 'operation': 'PICK'})

        self.service.on_auto_dock_drive_ready()
        report = self.status_reporter.reports[-1]

        self.assertEqual(report['state'], 'WAIT')
        self.assertEqual(report['previous_state'], 'PICK')
        self.assertEqual(report['operation_id'], INVENTORY_OPERATION_ID)
        self.assertIsNone(report['attempt_id'])
        self.assertEqual(report['source'], 'AUTO_DOCK')
        self.assertEqual(report['detail'], 'AUTO_DOCK_PICK_COMPLETED')

    def test_auto_dock_error_reports_fail(self):
        self.mark_idle()
        self.auto_dock_command('PICK')
        self.service.on_auto_dock_status({'state': 'SEARCHING', 'operation': 'PICK'})

        self.service.on_auto_dock_status({
            'state': 'ERROR',
            'reason': 'fork_failed',
            'operation': 'PICK',
        })
        report = self.status_reporter.reports[-1]

        self.assertEqual(report['state'], 'FAIL')
        self.assertEqual(report['previous_state'], 'PICK')
        self.assertEqual(report['operation_id'], INVENTORY_OPERATION_ID)
        self.assertIsNone(report['attempt_id'])
        self.assertEqual(report['source'], 'AUTO_DOCK')
        self.assertEqual(report['detail'], 'AUTO_DOCK_ERROR:fork_failed')

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

    def test_fork_up_and_down_publish_exact_commands_to_the_fork_topic_adapter(self):
        """The HTTP controls must preserve the vehicle's UP/DOWN topic contract."""
        up_status, up = post_json(f'{self.base_url}/v1/fork/up', {})
        down_status, down = post_json(f'{self.base_url}/v1/fork/down', {})

        self.assertEqual(up_status, 202)
        self.assertEqual(up, {'command': 'UP', 'state': 'FORK_COMMAND_PUBLISHED'})
        self.assertEqual(down_status, 202)
        self.assertEqual(down, {'command': 'DOWN', 'state': 'FORK_COMMAND_PUBLISHED'})
        self.assertEqual(self.velocity.fork_messages, ['UP', 'DOWN'])

    def test_fork_down_complete_is_relayed_only_for_the_pending_poc_mission(self):
        """A stale fork-state message must not release a POC mission into reverse."""
        self.service.on_fork_state('{"state":"DOWN_COMPLETE","error":""}')
        self.assertEqual(self.status_reporter.reports, [])

        status, response = post_json(
            f'{self.base_url}/v1/fork/down',
            {'operation_id': POC_MISSION_ID},
        )
        self.service.on_fork_state('{"state":"DOWN_COMPLETE","error":""}')

        self.assertEqual(status, 202)
        self.assertEqual(response, {'command': 'DOWN', 'state': 'FORK_COMMAND_PUBLISHED'})
        self.assertEqual(self.status_reporter.reports[-1]['state'], 'WAIT')
        self.assertEqual(self.status_reporter.reports[-1]['operation_id'], POC_MISSION_ID)
        self.assertEqual(self.status_reporter.reports[-1]['source'], 'FORK')
        self.assertEqual(self.status_reporter.reports[-1]['detail'], 'FORK_DOWN_COMPLETE')

    def test_poc_fork_down_completion_releases_the_completed_auto_dock_pick(self):
        """The pallet POC must not retain the old PICK operation after unloading."""
        self.mark_idle()
        self.auto_dock_command('PICK')
        self.service.on_auto_dock_drive_ready()
        self.status_reporter.reports.clear()

        post_json(
            f'{self.base_url}/v1/fork/down',
            {'operation_id': INVENTORY_OPERATION_ID},
        )
        self.service.on_fork_state('{"state":"DOWN_COMPLETE","error":""}')

        self.assertEqual(self.service.operation_status(), {
            'operation_id': None,
            'previous_operation_id': INVENTORY_OPERATION_ID,
            'state': 'IDLE',
            'previous_state': 'PICK_COMPLETE',
            'detail': 'FORK_DOWN_COMPLETE',
        })
        self.assertEqual(self.status_reporter.reports[-1]['state'], 'WAIT')
        self.assertEqual(self.status_reporter.reports[-1]['operation_id'], INVENTORY_OPERATION_ID)
        self.assertEqual(self.status_reporter.reports[-1]['detail'], 'FORK_DOWN_COMPLETE')

    def test_fork_down_error_fails_the_pending_poc_mission(self):
        """An error must never be treated as the down-complete gate for reverse."""
        post_json(
            f'{self.base_url}/v1/fork/down',
            {'operation_id': POC_MISSION_ID},
        )

        self.service.on_fork_state('{"state":"DOWN_COMPLETE","error":"hydraulic_fault"}')

        self.assertEqual(self.status_reporter.reports[-1]['state'], 'FAIL')
        self.assertEqual(self.status_reporter.reports[-1]['operation_id'], POC_MISSION_ID)
        self.assertEqual(self.status_reporter.reports[-1]['source'], 'FORK')
        self.assertEqual(
            self.status_reporter.reports[-1]['detail'],
            'FORK_DOWN_ERROR:hydraulic_fault',
        )

    def test_fork_down_timeout_fails_the_pending_poc_mission(self):
        """A missing fork completion cannot leave the mission eligible to reverse."""
        reporter = RecordingStatusReporter()
        service = self.module.VehicleCommandService(
            velocity=RecordingVelocity(),
            navigation=FakeNavigation(),
            max_linear_x=1.0,
            max_angular_z=1.0,
            max_hold_ms=1000,
            fork_state_timeout_sec=0.01,
            status_reporter=reporter,
        )
        try:
            service.fork_command('DOWN', {'operation_id': POC_MISSION_ID})
            self.wait_until(lambda: bool(reporter.reports))
            reports = list(reporter.reports)
        finally:
            service.close()

        self.assertEqual(reports[-1]['state'], 'FAIL')
        self.assertEqual(reports[-1]['operation_id'], POC_MISSION_ID)
        self.assertEqual(reports[-1]['source'], 'FORK')
        self.assertEqual(reports[-1]['detail'], 'FORK_DOWN_TIMEOUT')

    def test_manual_expiry_relays_the_optional_poc_mission_id(self):
        """The POC cannot start its return leg until the exact reverse has stopped."""
        self.mark_idle()
        self.status_reporter.reports.clear()
        status, response = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {
                'operation_id': POC_MISSION_ID,
                'linear_x': -0.18,
                'linear_y': 0.0,
                'angular_z': 0.0,
                'hold_ms': 1,
            },
        )
        self.wait_until(lambda: bool(self.status_reporter.reports))

        self.assertEqual(status, 202)
        self.assertEqual(response['linear_x'], -0.18)
        self.assertEqual(self.status_reporter.reports[-1]['state'], 'WAIT')
        self.assertEqual(self.status_reporter.reports[-1]['operation_id'], POC_MISSION_ID)
        self.assertEqual(self.status_reporter.reports[-1]['source'], 'API')
        self.assertEqual(self.status_reporter.reports[-1]['detail'], 'MANUAL_COMMAND_EXPIRED')

    def test_health_openapi_and_operation_status_are_discoverable(self):
        """Removing a public endpoint must fail the vehicle integration contract."""
        health_status, health = self.get_json('/healthz')
        openapi_status, openapi = self.get_json('/openapi.json')
        status_code, operation_status = self.get_json('/v1/operation-status')

        self.assertEqual(health_status, 200)
        self.assertEqual(health, {'status': 'ok'})
        self.assertEqual(openapi_status, 200)
        self.assertEqual(openapi['openapi'], '3.0.3')
        velocity_schema = openapi['paths']['/v1/cmd-vel']['post']['requestBody'][
            'content'
        ]['application/json']['schema']
        self.assertEqual(velocity_schema['properties']['linear_y'], {
            'type': 'number',
            'default': 0.0,
            'minimum': -1.0,
            'maximum': 1.0,
        })
        self.assertEqual(
            velocity_schema['properties']['operation_id'],
            {'type': 'string', 'format': 'uuid'},
        )
        fork_schema = openapi['paths']['/v1/fork/down']['post']['requestBody'][
            'content'
        ]['application/json']['schema']
        self.assertEqual(
            fork_schema['properties']['operation_id'],
            {'type': 'string', 'format': 'uuid'},
        )
        self.assertEqual(
            set(openapi['paths']),
            {
                '/healthz',
                '/openapi.json',
                '/v1/operation-status',
                '/v1/vehicle-status',
                '/v1/operation/idle',
                '/v1/missions/pallet3',
                '/v1/missions/pallet3/{operation_id}/picked',
                '/v1/missions/pallet3/{operation_id}/placed',
                '/v1/cmd-vel',
                '/v1/fork/up',
                '/v1/fork/down',
                '/v1/navigation/goals',
                '/v1/navigation/waypoints',
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
        self.assertEqual(self.velocity.messages[-1], (0.0, 0.0, 0.0))
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
        self.assertEqual(self.velocity.messages[-1], (0.0, 0.0, 0.0))
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

    def test_manual_velocity_supports_bounded_lateral_motion_and_expires_to_idle(self):
        """A lateral command must publish linear.y and expire with a complete zero Twist."""
        self.mark_idle()
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.0, 'linear_y': 0.1, 'angular_z': -0.25, 'hold_ms': 20},
        )

        self.assertEqual(status, 202)
        self.assertEqual(body, {
            'state': 'MANUAL',
            'linear_x': 0.0,
            'linear_y': 0.1,
            'angular_z': -0.25,
            'hold_ms': 20,
        })
        deadline = time.monotonic() + 1
        while len(self.velocity.messages) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.velocity.messages, [(0.0, 0.1, -0.25), (0.0, 0.0, 0.0)])
        _, operation_status = self.get_json('/v1/operation-status')
        self.assertEqual(operation_status, {
            'operation_id': None,
            'previous_operation_id': None,
            'state': 'IDLE',
            'previous_state': 'INIT',
            'detail': 'MANUAL_COMMAND_EXPIRED',
        })

    def test_manual_velocity_accepts_the_configured_one_meter_per_second_limit(self):
        """The vehicle API must expose the dashboard's 1.0 m/s translational limit."""
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 1.0, 'linear_y': -1.0, 'angular_z': 0.0, 'hold_ms': 20},
        )

        self.assertEqual(status, 202)
        self.assertEqual(body['linear_x'], 1.0)
        self.assertEqual(body['linear_y'], -1.0)

    def test_manual_velocity_rejects_a_turn_larger_than_ten_degrees(self):
        """One cmd_vel request must not rotate a vehicle more than ten degrees."""
        status, body = post_json(
            f'{self.base_url}/v1/cmd-vel',
            {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 1.0, 'hold_ms': 300},
        )

        self.assertEqual(status, 422)
        self.assertEqual(body, {
            'error': 'angular_z and hold_ms must not exceed 10 degrees per command',
        })
        self.assertEqual(self.velocity.messages, [])

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
            '--follow-waypoints-action-name',
            '--fork-command-topic', '--fork-state-topic', '--fork-state-timeout-sec',
            '--battery-topic', '--battery-stale-sec',
            '--initial-pose-topic', '--initial-pose-position-variance',
            '--initial-pose-yaw-variance',
            '--auto-dock-arrival-topic', '--auto-dock-status-topic',
            '--auto-dock-stop-topic', '--auto-dock-drive-ready-topic',
            '--max-linear-x', '--max-linear-y', '--max-angular-z', '--max-hold-ms',
            '--action-server-timeout-sec', '--goal-response-timeout-sec',
            '--cancel-response-timeout-sec',
            '--fleet-status-relay-url',
        ):
            self.assertIn(option, result.stdout)
        arguments = self.module.parse_args(['--robot-id', 'robot_2'])
        self.assertEqual(arguments.max_linear_x, 1.0)
        self.assertEqual(arguments.max_linear_y, 1.0)
        self.assertEqual(arguments.max_angular_z, 1.0)
        self.assertEqual(arguments.fork_state_topic, '/fork/state')
        self.assertEqual(arguments.fork_state_timeout_sec, 15.0)

    def test_direct_runner_reports_init_to_the_configured_status_relay(self):
        adapter = ClosingFakeAdapter()
        http_server = ReturningHttpServer()
        arguments = SimpleNamespace(
            host='0.0.0.0',
            port=8082,
            robot_id='robot_2',
            cmd_vel_topic='/cmd_vel',
            fork_command_topic='/fork/command',
            fork_state_topic='/fork/state',
            fork_state_timeout_sec=15.0,
            action_name='/navigate_to_pose',
            follow_waypoints_action_name='/follow_waypoints',
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
            fleet_status_relay_url='http://fleet-bridge:8080',
        )
        reporters = []

        class CapturingReporter:
            def __init__(self, robot_id, relay_url):
                self.robot_id = robot_id
                self.relay_url = relay_url
                self.reports = []
                self.closed = False
                reporters.append(self)

            def report(self, payload):
                self.reports.append(payload)

            def close(self):
                self.closed = True

        original_reporter = self.module.FleetStatusReporter
        self.module.FleetStatusReporter = CapturingReporter
        try:
            self.module.run_server(
                arguments,
                adapter_factory=lambda _args: adapter,
                http_server_factory=lambda _host, _port, _service: http_server,
            )
        finally:
            self.module.FleetStatusReporter = original_reporter

        self.assertEqual(len(reporters), 1)
        self.assertEqual(reporters[0].robot_id, 'robot_2')
        self.assertEqual(reporters[0].relay_url, 'http://fleet-bridge:8080')
        self.assertEqual(reporters[0].reports[0]['state'], 'INIT')
        self.assertEqual(reporters[0].reports[0]['source'], 'VEHICLE')
        self.assertTrue(reporters[0].closed)

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
            fork_command_topic='/fork/command',
            action_name='/navigate_to_pose',
            follow_waypoints_action_name='/follow_waypoints',
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
        self.assertEqual(adapter.messages, [(0.0, 0.0, 0.0)])
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
            fork_command_topic='/fork/command',
            action_name='/navigate_to_pose',
            follow_waypoints_action_name='/follow_waypoints',
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
        self.assertEqual(adapter.messages, [(0.0, 0.0, 0.0)])
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
            fork_command_topic='/fork/command',
            fork_state_topic='/fork/state',
            fork_state_timeout_sec=15.0,
            action_name='/navigate_to_pose',
            follow_waypoints_action_name='/follow_waypoints',
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
            follow_waypoints_action_name,
            action_server_timeout_sec,
            goal_response_timeout_sec,
            cancel_response_timeout_sec,
            fork_command_topic,
            fork_state_topic,
            auto_dock_arrival_topic,
            auto_dock_status_topic,
            auto_dock_stop_topic,
            auto_dock_drive_ready_topic,
        ):
            captured.update({
                'robot_id': robot_id,
                'cmd_vel_topic': cmd_vel_topic,
                'action_name': action_name,
                'follow_waypoints_action_name': follow_waypoints_action_name,
                'action_server_timeout_sec': action_server_timeout_sec,
                'goal_response_timeout_sec': goal_response_timeout_sec,
                'cancel_response_timeout_sec': cancel_response_timeout_sec,
                'fork_command_topic': fork_command_topic,
                'fork_state_topic': fork_state_topic,
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
            'follow_waypoints_action_name': '/follow_waypoints',
            'action_server_timeout_sec': 1.0,
            'goal_response_timeout_sec': 3.0,
            'cancel_response_timeout_sec': 3.0,
            'fork_command_topic': '/fork/command',
            'fork_state_topic': '/fork/state',
            'auto_dock_arrival_topic': None,
            'auto_dock_status_topic': None,
            'auto_dock_stop_topic': None,
            'auto_dock_drive_ready_topic': None,
        })


class RosVehicleAdapterTopicTest(unittest.TestCase):
    def create_adapter(self):
        module = load_server_module()
        publisher_topics = []
        subscription_topics = []

        class FakeContext:
            def ok(self):
                return True

        class FakeNode:
            def create_publisher(self, _message_type, topic, _qos):
                publisher_topics.append(topic)
                return SimpleNamespace(publish=lambda _message: None)

            def create_subscription(
                self, _message_type, topic, _callback, _qos,
            ):
                subscription_topics.append(topic)
                return object()

            def destroy_node(self):
                return None

        class FakeExecutor:
            def __init__(self, context):
                self.context = context

            def add_node(self, _node):
                return None

            def spin(self):
                return None

            def shutdown(self):
                return None

        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0)
                self.angular = SimpleNamespace(z=0.0)

        class FakeQosProfile:
            def __init__(self, **_kwargs):
                pass

        fake_rclpy = ModuleType('rclpy')
        fake_rclpy.init = lambda **_kwargs: None
        fake_rclpy.shutdown = lambda **_kwargs: None
        fake_rclpy.create_node = lambda *_args, **_kwargs: FakeNode()

        fake_modules = {
            'rclpy': fake_rclpy,
            'rclpy.action': SimpleNamespace(ActionClient=lambda *_args: object()),
            'rclpy.context': SimpleNamespace(Context=FakeContext),
            'rclpy.executors': SimpleNamespace(SingleThreadedExecutor=FakeExecutor),
            'rclpy.qos': SimpleNamespace(
                DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=object()),
                QoSProfile=FakeQosProfile,
                ReliabilityPolicy=SimpleNamespace(RELIABLE=object()),
                qos_profile_sensor_data=object(),
            ),
            'action_msgs.msg': SimpleNamespace(GoalStatus=object()),
            'geometry_msgs.msg': SimpleNamespace(
                PoseStamped=object(),
                PoseWithCovarianceStamped=object(),
                Twist=FakeTwist,
            ),
            'nav2_msgs.action': SimpleNamespace(
                FollowWaypoints=object(),
                NavigateToPose=object(),
            ),
            'std_msgs.msg': SimpleNamespace(Empty=object(), String=object(), UInt16=object()),
        }

        with patch.dict(sys.modules, fake_modules):
            adapter = module.RosVehicleAdapter(
                robot_id='robot_2',
                cmd_vel_topic='/cmd_vel',
                action_name='/navigate_to_pose',
                follow_waypoints_action_name='/follow_waypoints',
                action_server_timeout_sec=1.0,
                goal_response_timeout_sec=3.0,
                cancel_response_timeout_sec=3.0,
            )
            return adapter, publisher_topics, subscription_topics

    def test_default_auto_dock_arrival_publishes_to_global_topic(self):
        """A namespaced arrival topic would prevent the vehicle Auto Dock from receiving commands."""
        adapter, publisher_topics, _subscription_topics = self.create_adapter()
        try:
            self.assertIn('/nav2/arrival', publisher_topics)
            self.assertNotIn('/robot_2/nav2/arrival', publisher_topics)
        finally:
            adapter.close()

    def test_default_auto_dock_drive_ready_subscribes_to_global_topic(self):
        """The Auto Dock completion event is published on the global ROS topic."""
        adapter, _publisher_topics, subscription_topics = self.create_adapter()
        try:
            adapter.configure_auto_dock(lambda _status: None, lambda: None)

            self.assertIn('/auto_dock/drive_ready', subscription_topics)
            self.assertNotIn('/robot_2/auto_dock/drive_ready', subscription_topics)
        finally:
            adapter.close()


class RosVehicleAdapterResultTest(unittest.TestCase):
    def test_follow_waypoints_missed_waypoint_is_a_failed_navigation(self):
        """Nav2 may succeed overall while reporting individual waypoint failures."""
        module = load_server_module()
        adapter = module.RosVehicleAdapter.__new__(module.RosVehicleAdapter)
        adapter._goal_status = SimpleNamespace(
            STATUS_SUCCEEDED=4,
            STATUS_CANCELED=5,
        )
        adapter._lock = threading.Lock()
        goal_handle = object()
        adapter._goal_handles = {'attempt-1': goal_handle}
        terminal = []

        class ResultFuture:
            def result(self):
                return SimpleNamespace(
                    status=4,
                    result=SimpleNamespace(missed_waypoints=[1]),
                )

        adapter._on_navigation_result(
            'attempt-1',
            goal_handle,
            lambda attempt_id, state: terminal.append((attempt_id, state)),
            ResultFuture(),
        )

        self.assertEqual(terminal, [('attempt-1', 'FAILED')])
        self.assertEqual(adapter._goal_handles, {})


if __name__ == '__main__':
    unittest.main()
