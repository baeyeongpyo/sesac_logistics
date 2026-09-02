import sqlite3
import tempfile
import unittest
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

    def test_missing_database_is_reported_without_breaking_other_sources(self):
        (self.data_directory / 'orchestrator.db').unlink()

        snapshot = SnapshotReader(self.data_directory).snapshot()

        self.assertFalse(snapshot['sources']['orchestrator']['available'])
        self.assertEqual(snapshot['orchestrator']['steps'], [])
        self.assertEqual(snapshot['vehicles'][0]['robot_id'], 'R1')

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
