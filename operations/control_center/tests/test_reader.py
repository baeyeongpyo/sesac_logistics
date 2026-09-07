import sqlite3
import tempfile
import unittest
from datetime import UTC, datetime
from pathlib import Path
from unittest.mock import Mock, patch

from operations.control_center.app.reader import SnapshotReader


class SnapshotReaderTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.data_directory = Path(self.tempdir.name)
        self._create_fleet_telemetry()
        self._create_fleet_manager()
        self._create_inventory()
        self._create_orchestrator()

    def tearDown(self):
        self.tempdir.cleanup()

    def test_snapshot_combines_vehicle_task_and_inventory_data(self):
        snapshot = SnapshotReader(self.data_directory).snapshot()

        vehicle = snapshot['vehicles'][0]
        self.assertEqual(vehicle['robot_id'], 'R1')
        self.assertEqual(vehicle['pose'], {'x_m': 1.25, 'y_m': -0.5, 'yaw_rad': 0.4})
        self.assertEqual(vehicle['battery']['battery_raw'], 87)
        self.assertEqual(vehicle['fleet_state']['state'], 'IDLE')
        self.assertEqual(vehicle['pallet_state']['payload_type'], 'BOX_A')
        self.assertEqual(vehicle['active_task']['operation_id'], 'op-1')
        self.assertEqual(snapshot['inventory']['stocks'][0]['available_quantity'], 7)
        self.assertEqual(
            snapshot['inventory']['zone_inventory'],
            [
                {
                    'zone_id': 'A-01',
                    'name': '입고 A',
                    'capacity': 20,
                    'current_quantity': 12,
                    'items': [
                        {'payload_type': 'BOX_A', 'quantity': 10},
                        {'payload_type': 'BOX_B', 'quantity': 2},
                    ],
                },
                {
                    'zone_id': 'B-01',
                    'name': '출고 B',
                    'capacity': 8,
                    'current_quantity': 0,
                    'items': [],
                },
            ],
        )
        self.assertEqual(snapshot['orchestrator']['steps'][0]['phase'], 'TO_PICK')
        self.assertTrue(all(source['available'] for source in snapshot['sources'].values()))

    def test_snapshot_displays_matching_inventory_navigation_as_auto_drive(self):
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'DRIVE', detail = 'NAVIGATION_STARTED' "
                "WHERE robot_id = 'R1'"
            )

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['fleet_state']['state'], 'DRIVE')
        self.assertEqual(vehicle['display_state'], 'AUTO_DRIVE')

    def test_current_p3_work_is_not_replaced_by_completed_history(self):
        with self._connect('inventory.db') as db:
            db.execute("UPDATE transport_operations SET status = 'COMPLETED'")
            db.execute(
                "INSERT INTO transport_operations VALUES "
                "('p3-current', 'R1', 'NORMAL', 'docker', 'p3', 'TO_PLACE', "
                "0, NULL, 2, '2026-09-05T08:00:00Z', '2026-09-05T08:01:00Z', NULL)"
            )
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'DRIVE', operation_id = 'p3-current', "
                "source = 'NAV2', detail = 'NAVIGATION_STARTED'"
            )

        snapshot = SnapshotReader(self.data_directory).snapshot()
        vehicle = snapshot['vehicles'][0]

        self.assertEqual(vehicle['active_task']['operation_id'], 'p3-current')
        self.assertEqual(vehicle['active_task']['destination_zone_id'], 'p3')
        self.assertEqual(vehicle['display_state'], 'AUTO_DRIVE')
        self.assertIsNone(vehicle['orchestrator_step'])
        self.assertEqual(len(snapshot['inventory']['transport_operations']), 2)

    def test_completed_history_is_not_an_active_task_after_stop_or_return(self):
        with self._connect('inventory.db') as db:
            db.execute("UPDATE transport_operations SET status = 'COMPLETED'")
        for state, operation_id in [('FAIL', 'return-route'), ('WAIT', 'op-1')]:
            with self.subTest(state=state):
                with self._connect('fleet_manager.db') as db:
                    db.execute(
                        "UPDATE vehicle_states SET state = ?, operation_id = ?, "
                        "source = 'API', detail = 'API_STOP'",
                        (state, operation_id),
                    )

                snapshot = SnapshotReader(self.data_directory).snapshot()

                self.assertIsNone(snapshot['vehicles'][0]['active_task'])
                self.assertIsNone(snapshot['vehicles'][0]['orchestrator_step'])
                self.assertEqual(len(snapshot['inventory']['transport_operations']), 1)

    def test_active_task_survives_a_newer_update_to_completed_history(self):
        with self._connect('inventory.db') as db:
            db.execute(
                "INSERT INTO transport_operations VALUES "
                "('old-completed', 'R1', 'NORMAL', 'docker', 'p3', 'COMPLETED', "
                "0, NULL, 3, '2026-08-31T08:00:00Z', '2026-09-05T08:01:00Z', "
                "'2026-09-05T08:01:00Z')"
            )
        with self._connect('fleet_manager.db') as db:
            db.execute("UPDATE vehicle_states SET operation_id = NULL")

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['active_task']['operation_id'], 'op-1')

    def test_arrival_wait_is_not_displayed_as_driving(self):
        with self._connect('inventory.db') as db:
            db.execute("UPDATE transport_operations SET status = 'TO_PLACE'")
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'WAIT', source = 'NAV2', "
                "detail = 'NAVIGATION_SUCCEEDED'"
            )

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['display_state'], 'WAIT')

    def test_missing_inventory_preserves_fleet_operation_reference(self):
        (self.data_directory / 'inventory.db').unlink()

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['active_task'], {'operation_id': 'op-1'})

    def test_snapshot_displays_navigation_without_inventory_operation_as_manual_drive(self):
        with self._connect('inventory.db') as db:
            db.execute("DELETE FROM transport_operations WHERE operation_id = 'op-1'")
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'DRIVE', operation_id = 'manual-nav', "
                "detail = 'NAVIGATION_STARTED' WHERE robot_id = 'R1'"
            )

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['fleet_state']['state'], 'DRIVE')
        self.assertEqual(vehicle['display_state'], 'MANUAL_DRIVE')

    def test_snapshot_displays_navigation_as_manual_drive_when_only_completed_inventory_work_exists(self):
        with self._connect('inventory.db') as db:
            db.execute(
                "UPDATE transport_operations SET status = 'COMPLETED', "
                "completed_at = '2026-09-01T09:01:00Z' WHERE operation_id = 'op-1'"
            )
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'DRIVE', operation_id = 'manual-nav', "
                "detail = 'NAVIGATION_STARTED' WHERE robot_id = 'R1'"
            )

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['fleet_state']['state'], 'DRIVE')
        self.assertEqual(vehicle['display_state'], 'MANUAL_DRIVE')

    def test_snapshot_keeps_wait_after_navigation_without_inventory_operation_completes(self):
        with self._connect('inventory.db') as db:
            db.execute("DELETE FROM transport_operations WHERE operation_id = 'op-1'")
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'WAIT', operation_id = 'manual-nav', "
                "detail = 'NAVIGATION_SUCCEEDED' WHERE robot_id = 'R1'"
            )

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['fleet_state']['state'], 'WAIT')
        self.assertEqual(vehicle['display_state'], 'WAIT')

    def test_snapshot_keeps_operator_ready_wait_available_to_the_server(self):
        with self._connect('inventory.db') as db:
            db.execute("DELETE FROM transport_operations WHERE operation_id = 'op-1'")
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'WAIT', source = 'API', "
                "detail = 'OPERATOR_READY' WHERE robot_id = 'R1'"
            )

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['fleet_state']['state'], 'WAIT')
        self.assertEqual(vehicle['display_state'], 'WAIT')

    def test_snapshot_displays_operator_api_stop_without_overwriting_raw_failure(self):
        """Changing the API stop signature must not turn a commanded stop into an error badge."""

        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'FAIL', previous_state = 'DRIVE', "
                "source = 'API', detail = 'API_STOP' WHERE robot_id = 'R1'"
            )

        vehicle = SnapshotReader(self.data_directory).snapshot()['vehicles'][0]

        self.assertEqual(vehicle['fleet_state']['state'], 'FAIL')
        self.assertEqual(vehicle['fleet_state']['source'], 'API')
        self.assertEqual(vehicle['fleet_state']['detail'], 'API_STOP')
        self.assertEqual(vehicle['display_state'], 'STOPPED')

    def test_snapshot_preserves_fleet_state_when_inventory_source_is_unavailable(self):
        (self.data_directory / 'inventory.db').unlink()
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "UPDATE vehicle_states SET state = 'DRIVE', detail = 'NAVIGATION_STARTED' "
                "WHERE robot_id = 'R1'"
            )

        snapshot = SnapshotReader(self.data_directory).snapshot()
        vehicle = snapshot['vehicles'][0]

        self.assertFalse(snapshot['sources']['inventory']['available'])
        self.assertEqual(vehicle['display_state'], 'DRIVE')

    def test_missing_database_is_reported_without_breaking_other_sources(self):
        (self.data_directory / 'orchestrator.db').unlink()

        snapshot = SnapshotReader(self.data_directory).snapshot()

        self.assertFalse(snapshot['sources']['orchestrator']['available'])
        self.assertEqual(snapshot['orchestrator']['steps'], [])
        self.assertEqual(snapshot['vehicles'][0]['robot_id'], 'R1')

    def test_snapshot_marks_connectivity_from_the_newest_telemetry_receipt(self):
        with self._connect('fleet_telemetry.db') as db:
            db.execute(
                "UPDATE robot_battery SET received_at = '2026-09-01T09:00:06Z' "
                "WHERE robot_id = 'R1'"
            )

        online = SnapshotReader(
            self.data_directory,
            clock=lambda: datetime(2026, 9, 1, 9, 0, 15, tzinfo=UTC),
        ).snapshot()['vehicles'][0]['connectivity']
        stale = SnapshotReader(
            self.data_directory,
            clock=lambda: datetime(2026, 9, 1, 9, 0, 20, tzinfo=UTC),
        ).snapshot()['vehicles'][0]['connectivity']
        offline = SnapshotReader(
            self.data_directory,
            clock=lambda: datetime(2026, 9, 1, 9, 0, 37, tzinfo=UTC),
        ).snapshot()['vehicles'][0]['connectivity']

        self.assertEqual(
            online,
            {
                'state': 'online',
                'last_received_at': '2026-09-01T09:00:06Z',
                'age_sec': 9,
            },
        )
        self.assertEqual(stale['state'], 'stale')
        self.assertEqual(offline['state'], 'offline')

    def test_snapshot_marks_a_vehicle_without_telemetry_as_unconfirmed(self):
        with self._connect('fleet_manager.db') as db:
            db.execute(
                "INSERT INTO vehicle_states VALUES "
                "('R2', 'WAIT', NULL, NULL, NULL, 'demo', '{}', "
                "'2026-09-01T09:00:00Z', '2026-09-01T09:00:01Z')"
            )

        snapshot = SnapshotReader(self.data_directory).snapshot()
        vehicle = next(vehicle for vehicle in snapshot['vehicles'] if vehicle['robot_id'] == 'R2')

        self.assertEqual(
            vehicle['connectivity'],
            {'state': 'unconfirmed', 'last_received_at': None, 'age_sec': None},
        )

    def test_connections_are_read_only(self):
        reader = SnapshotReader(self.data_directory)

        connection = reader._connect('fleet_telemetry.db')
        try:
            with self.assertRaises(sqlite3.OperationalError):
                connection.execute(
                    "UPDATE robot_pose SET x_m = 10 WHERE robot_id = 'R1'"
                )
        finally:
            connection.close()

    def test_connections_use_immutable_read_only_uri_for_read_only_docker_mounts(self):
        reader = SnapshotReader(self.data_directory)
        connection = Mock()

        with patch('operations.control_center.app.reader.sqlite3.connect', return_value=connection) as connect:
            reader._connect('fleet_telemetry.db')

        self.assertIn('?mode=ro&immutable=1', connect.call_args.args[0])
        self.assertTrue(connect.call_args.kwargs['uri'])
        connection.execute.assert_called_once_with('PRAGMA query_only = ON')

    def _connect(self, name: str) -> sqlite3.Connection:
        connection = sqlite3.connect(self.data_directory / name)
        self.addCleanup(connection.close)
        return connection

    def _create_fleet_telemetry(self):
        with self._connect('fleet_telemetry.db') as db:
            db.executescript(
                '''
                CREATE TABLE robot_pose (
                    robot_id TEXT PRIMARY KEY, x_m REAL, y_m REAL, yaw_rad REAL,
                    observed_at TEXT, received_at TEXT
                );
                CREATE TABLE robot_battery (
                    robot_id TEXT PRIMARY KEY, battery_raw INTEGER, received_at TEXT
                );
                INSERT INTO robot_pose VALUES ('R1', 1.25, -0.5, 0.4, '2026-09-01T09:00:00Z', '2026-09-01T09:00:01Z');
                INSERT INTO robot_battery VALUES ('R1', 87, '2026-09-01T09:00:01Z');
                '''
            )

    def _create_fleet_manager(self):
        with self._connect('fleet_manager.db') as db:
            db.executescript(
                '''
                CREATE TABLE vehicle_states (
                    robot_id TEXT PRIMARY KEY, state TEXT, previous_state TEXT,
                    operation_id TEXT, attempt_id TEXT, source TEXT, detail TEXT,
                    observed_at TEXT, updated_at TEXT
                );
                INSERT INTO vehicle_states VALUES ('R1', 'IDLE', NULL, 'op-1', NULL, 'demo', '{}', '2026-09-01T09:00:00Z', '2026-09-01T09:00:01Z');
                '''
            )

    def _create_inventory(self):
        with self._connect('inventory.db') as db:
            db.executescript(
                '''
                CREATE TABLE zones (
                    zone_id TEXT PRIMARY KEY, name TEXT, map_name TEXT, nav_x REAL,
                    nav_y REAL, nav_yaw REAL, capacity INTEGER, enabled INTEGER,
                    created_at TEXT, updated_at TEXT
                );
                CREATE TABLE pallet_stocks (
                    zone_id TEXT, payload_type TEXT, quantity INTEGER,
                    reserved_quantity INTEGER, version INTEGER, updated_at TEXT,
                    PRIMARY KEY (zone_id, payload_type)
                );
                CREATE TABLE robot_pallet_states (
                    robot_id TEXT PRIMARY KEY, has_pallet INTEGER, payload_type TEXT,
                    version INTEGER, reported_at TEXT, updated_at TEXT
                );
                CREATE TABLE transport_operations (
                    operation_id TEXT PRIMARY KEY, robot_id TEXT, payload_type TEXT,
                    source_zone_id TEXT, destination_zone_id TEXT, status TEXT,
                    priority INTEGER, failure_code TEXT, version INTEGER,
                    created_at TEXT, updated_at TEXT, completed_at TEXT
                );
                INSERT INTO zones VALUES ('A-01', '입고 A', 'map_0825', 1, 2, 0, 20, 1, '2026-09-01T09:00:00Z', '2026-09-01T09:00:00Z');
                INSERT INTO zones VALUES ('B-01', '출고 B', 'map_0825', 3, 4, 0, 8, 1, '2026-09-01T09:00:00Z', '2026-09-01T09:00:00Z');
                INSERT INTO pallet_stocks VALUES ('A-01', 'BOX_A', 10, 3, 1, '2026-09-01T09:00:00Z');
                INSERT INTO pallet_stocks VALUES ('A-01', 'BOX_B', 2, 0, 1, '2026-09-01T09:00:00Z');
                INSERT INTO robot_pallet_states VALUES ('R1', 1, 'BOX_A', 1, '2026-09-01T09:00:00Z', '2026-09-01T09:00:00Z');
                INSERT INTO transport_operations VALUES ('op-1', 'R1', 'BOX_A', 'A-01', 'B-01', 'TO_PICK', 10, NULL, 1, '2026-09-01T09:00:00Z', '2026-09-01T09:00:00Z', NULL);
                '''
            )

    def _create_orchestrator(self):
        with self._connect('orchestrator.db') as db:
            db.executescript(
                '''
                CREATE TABLE orchestrator_steps (
                    operation_id TEXT PRIMARY KEY, robot_id TEXT, phase TEXT,
                    source_zone_id TEXT, destination_zone_id TEXT, payload_type TEXT,
                    updated_at TEXT
                );
                CREATE TABLE command_outbox (
                    command_id TEXT PRIMARY KEY, operation_id TEXT, robot_id TEXT,
                    command_type TEXT, payload_json TEXT, status TEXT, last_error TEXT,
                    created_at TEXT, sent_at TEXT
                );
                CREATE TABLE orchestrator_errors (
                    error_key TEXT PRIMARY KEY, message TEXT, updated_at TEXT
                );
                CREATE TABLE operation_recoveries (
                    operation_id TEXT PRIMARY KEY, robot_id TEXT, marked_at TEXT
                );
                INSERT INTO orchestrator_steps VALUES ('op-1', 'R1', 'TO_PICK', 'A-01', 'B-01', 'BOX_A', '2026-09-01T09:00:00Z');
                INSERT INTO command_outbox VALUES ('cmd-1', 'op-1', 'R1', 'navigation/goals', '{}', 'PENDING', NULL, '2026-09-01T09:00:00Z', NULL);
                '''
            )
