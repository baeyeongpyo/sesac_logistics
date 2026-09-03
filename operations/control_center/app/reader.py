"""Read-only SQLite aggregation for the operations dashboard."""

from __future__ import annotations

from contextlib import closing
from datetime import UTC, datetime
import json
from pathlib import Path
import sqlite3
from typing import Any, Callable


ONLINE_TELEMETRY_AGE_SEC = 10
OFFLINE_TELEMETRY_AGE_SEC = 30


class SnapshotReader:
    """Build one dashboard snapshot from the operations SQLite databases.

    Every database connection is opened using SQLite's ``mode=ro`` URI mode and
    has ``query_only`` enabled.  This process is intentionally unable to make a
    state change through the mounted monitoring data directory.
    """

    def __init__(
        self,
        data_directory: Path,
        *,
        clock: Callable[[], datetime] = lambda: datetime.now(UTC),
    ):
        self.data_directory = data_directory
        self._clock = clock

    def snapshot(self) -> dict[str, Any]:
        """Return a partial snapshot even when one source is unavailable."""

        telemetry, telemetry_status = self._read_source(
            'telemetry', self._read_telemetry
        )
        fleet, fleet_status = self._read_source('fleet_manager', self._read_fleet)
        inventory, inventory_status = self._read_source(
            'inventory', self._read_inventory
        )
        orchestrator, orchestrator_status = self._read_source(
            'orchestrator', self._read_orchestrator
        )
        telemetry = {'poses': [], 'batteries': [], **telemetry}
        fleet = {'states': [], **fleet}
        inventory = {
            'zones': [],
            'stocks': [],
            'pallet_states': [],
            'transport_operations': [],
            **inventory,
        }
        orchestrator = {
            'steps': [],
            'commands': [],
            'errors': [],
            'recoveries': [],
            **orchestrator,
        }

        sources = {
            'telemetry': telemetry_status,
            'fleet_manager': fleet_status,
            'inventory': inventory_status,
            'orchestrator': orchestrator_status,
        }
        vehicles = self._build_vehicles(
            telemetry,
            fleet,
            inventory,
            orchestrator,
            inventory_available=inventory_status['available'],
            now=self._clock().astimezone(UTC),
        )
        return {
            'generated_at': datetime.now(UTC).isoformat(),
            'sources': sources,
            'vehicles': vehicles,
            'inventory': inventory,
            'orchestrator': orchestrator,
        }

    def _connect(self, filename: str) -> sqlite3.Connection:
        """Open one known database in read-only mode."""

        path = (self.data_directory / filename).resolve()
        if not path.is_file():
            raise FileNotFoundError(filename)
        connection = sqlite3.connect(
            f'{path.as_uri()}?mode=ro&immutable=1',
            uri=True,
        )
        connection.row_factory = sqlite3.Row
        connection.execute('PRAGMA query_only = ON')
        return connection

    def _read_source(
        self,
        source_name: str,
        reader: Callable[[], dict[str, list[dict[str, Any]]]],
    ) -> tuple[dict[str, list[dict[str, Any]]], dict[str, Any]]:
        try:
            return reader(), {'available': True, 'name': source_name}
        except (OSError, sqlite3.Error, ValueError):
            return {}, {
                'available': False,
                'name': source_name,
                'message': '데이터 원본에 연결할 수 없습니다.',
            }

    def _read_telemetry(self) -> dict[str, list[dict[str, Any]]]:
        with closing(self._connect('fleet_telemetry.db')) as connection:
            poses = [
                {
                    'robot_id': row['robot_id'],
                    'x_m': row['x_m'],
                    'y_m': row['y_m'],
                    'yaw_rad': row['yaw_rad'],
                    'observed_at': row['observed_at'],
                    'received_at': row['received_at'],
                }
                for row in connection.execute(
                    'SELECT robot_id, x_m, y_m, yaw_rad, observed_at, received_at '
                    'FROM robot_pose ORDER BY robot_id'
                )
            ]
            batteries = [
                {
                    'robot_id': row['robot_id'],
                    'battery_raw': row['battery_raw'],
                    'received_at': row['received_at'],
                }
                for row in connection.execute(
                    'SELECT robot_id, battery_raw, received_at '
                    'FROM robot_battery ORDER BY robot_id'
                )
            ]
        return {'poses': poses, 'batteries': batteries}

    def _read_fleet(self) -> dict[str, list[dict[str, Any]]]:
        with closing(self._connect('fleet_manager.db')) as connection:
            states = [
                {
                    'robot_id': row['robot_id'],
                    'state': row['state'],
                    'previous_state': row['previous_state'],
                    'operation_id': row['operation_id'],
                    'attempt_id': row['attempt_id'],
                    'source': row['source'],
                    'detail': self._json_value(row['detail']),
                    'observed_at': row['observed_at'],
                    'updated_at': row['updated_at'],
                }
                for row in connection.execute(
                    'SELECT robot_id, state, previous_state, operation_id, attempt_id, '
                    'source, detail, observed_at, updated_at '
                    'FROM vehicle_states ORDER BY robot_id'
                )
            ]
        return {'states': states}

    def _read_inventory(self) -> dict[str, list[dict[str, Any]]]:
        with closing(self._connect('inventory.db')) as connection:
            zones = [
                {
                    **dict(row),
                    'enabled': bool(row['enabled']),
                }
                for row in connection.execute(
                    'SELECT zone_id, name, map_name, nav_x, nav_y, nav_yaw, capacity, '
                    'enabled, created_at, updated_at FROM zones ORDER BY zone_id'
                )
            ]
            stocks = [
                {
                    **dict(row),
                    'available_quantity': row['quantity'] - row['reserved_quantity'],
                }
                for row in connection.execute(
                    'SELECT zone_id, payload_type, quantity, reserved_quantity, version, '
                    'updated_at FROM pallet_stocks ORDER BY zone_id, payload_type'
                )
            ]
            stocks_by_zone: dict[str, list[dict[str, Any]]] = {}
            for stock in stocks:
                stocks_by_zone.setdefault(stock['zone_id'], []).append(stock)
            zone_inventory = [
                {
                    'zone_id': zone['zone_id'],
                    'name': zone['name'],
                    'capacity': zone['capacity'],
                    'current_quantity': sum(stock['quantity'] for stock in zone_stocks),
                    'items': [
                        {
                            'payload_type': stock['payload_type'],
                            'quantity': stock['quantity'],
                        }
                        for stock in zone_stocks
                    ],
                }
                for zone in zones
                for zone_stocks in [stocks_by_zone.get(zone['zone_id'], [])]
            ]
            pallet_states = [
                {
                    **dict(row),
                    'has_pallet': bool(row['has_pallet']),
                }
                for row in connection.execute(
                    'SELECT robot_id, has_pallet, payload_type, version, reported_at, '
                    'updated_at FROM robot_pallet_states ORDER BY robot_id'
                )
            ]
            operations = [
                dict(row)
                for row in connection.execute(
                    'SELECT operation_id, robot_id, payload_type, source_zone_id, '
                    'destination_zone_id, status, priority, failure_code, version, '
                    'created_at, updated_at, completed_at FROM transport_operations '
                    'ORDER BY updated_at DESC, operation_id'
                )
            ]
        return {
            'zones': zones,
            'stocks': stocks,
            'zone_inventory': zone_inventory,
            'pallet_states': pallet_states,
            'transport_operations': operations,
        }

    def _read_orchestrator(self) -> dict[str, list[dict[str, Any]]]:
        with closing(self._connect('orchestrator.db')) as connection:
            steps = [
                dict(row)
                for row in connection.execute(
                    'SELECT operation_id, robot_id, phase, source_zone_id, '
                    'destination_zone_id, payload_type, updated_at '
                    'FROM orchestrator_steps ORDER BY updated_at DESC, operation_id'
                )
            ]
            commands = [
                {
                    **dict(row),
                    'payload': self._json_value(row['payload_json']),
                }
                for row in connection.execute(
                    'SELECT command_id, operation_id, robot_id, command_type, '
                    'payload_json, status, last_error, created_at, sent_at '
                    'FROM command_outbox ORDER BY created_at DESC, command_id'
                )
            ]
            errors = [
                dict(row)
                for row in connection.execute(
                    'SELECT error_key, message, updated_at FROM orchestrator_errors '
                    'ORDER BY updated_at DESC, error_key'
                )
            ]
            recoveries = [
                dict(row)
                for row in connection.execute(
                    'SELECT operation_id, robot_id, marked_at FROM operation_recoveries '
                    'ORDER BY marked_at DESC, operation_id'
                )
            ]
        return {
            'steps': steps,
            'commands': commands,
            'errors': errors,
            'recoveries': recoveries,
        }

    def _build_vehicles(
        self,
        telemetry: dict[str, list[dict[str, Any]]],
        fleet: dict[str, list[dict[str, Any]]],
        inventory: dict[str, list[dict[str, Any]]],
        orchestrator: dict[str, list[dict[str, Any]]],
        *,
        inventory_available: bool,
        now: datetime,
    ) -> list[dict[str, Any]]:
        poses = self._by_robot(telemetry.get('poses', []))
        batteries = self._by_robot(telemetry.get('batteries', []))
        states = self._by_robot(fleet.get('states', []))
        pallet_states = self._by_robot(inventory.get('pallet_states', []))
        operations = self._by_robot(inventory.get('transport_operations', []))
        steps = self._by_robot(orchestrator.get('steps', []))
        commands = self._by_robot(orchestrator.get('commands', []), multiple=True)
        robot_ids = set(poses) | set(batteries) | set(states) | set(pallet_states)
        robot_ids |= set(operations) | set(steps) | set(commands)

        vehicles = []
        for robot_id in sorted(robot_ids):
            fleet_state = states.get(robot_id)
            inventory_task = operations.get(robot_id)
            active_task = inventory_task
            if active_task is None and fleet_state and fleet_state.get('operation_id'):
                active_task = {'operation_id': fleet_state['operation_id']}
            pose = poses.get(robot_id)
            compact_pose = None
            if pose:
                compact_pose = {
                    'x_m': pose['x_m'],
                    'y_m': pose['y_m'],
                    'yaw_rad': pose['yaw_rad'],
                }
            vehicles.append(
                {
                    'robot_id': robot_id,
                    'pose': compact_pose,
                    'battery': batteries.get(robot_id),
                    'connectivity': self._connectivity(
                        pose,
                        batteries.get(robot_id),
                        now,
                    ),
                    'fleet_state': fleet_state,
                    'display_state': self._display_state(
                        fleet_state,
                        inventory_task,
                        inventory_available,
                    ),
                    'pallet_state': pallet_states.get(robot_id),
                    'active_task': active_task,
                    'orchestrator_step': steps.get(robot_id),
                    'pending_commands': commands.get(robot_id, []),
                }
            )
        return vehicles

    @staticmethod
    def _display_state(
        fleet_state: dict[str, Any] | None,
        inventory_task: dict[str, Any] | None,
        inventory_available: bool,
    ) -> str | None:
        """Derive a dashboard-only state without changing vehicle control state."""

        if fleet_state is None:
            return None
        state = fleet_state.get('state')
        if (
            state in {'FAIL', 'FAILED'}
            and fleet_state.get('source') == 'API'
            and fleet_state.get('detail') == 'API_STOP'
        ):
            return 'STOPPED'
        if not inventory_available:
            return state

        operation_id = fleet_state.get('operation_id')
        is_active_inventory_task = (
            inventory_task is not None
            and inventory_task.get('operation_id') == operation_id
            and inventory_task.get('status')
            in {'TO_PICK', 'PICKING', 'TO_PLACE', 'PLACING'}
        )
        if is_active_inventory_task and state in {'WAIT', 'DRIVE'}:
            return 'AUTO_DRIVE'
        if not is_active_inventory_task and state == 'DRIVE':
            return 'MANUAL_DRIVE'
        return state

    @staticmethod
    def _connectivity(
        pose: dict[str, Any] | None,
        battery: dict[str, Any] | None,
        now: datetime,
    ) -> dict[str, str | int | None]:
        receipts = [
            received_at
            for received_at in (
                pose.get('received_at') if pose else None,
                battery.get('received_at') if battery else None,
            )
            if isinstance(received_at, str)
        ]
        timestamps = []
        for received_at in receipts:
            try:
                timestamp = datetime.fromisoformat(received_at.replace('Z', '+00:00'))
            except ValueError:
                continue
            if timestamp.tzinfo is not None:
                timestamps.append((timestamp.astimezone(UTC), received_at))

        if not timestamps:
            return {
                'state': 'unconfirmed',
                'last_received_at': None,
                'age_sec': None,
            }

        latest_timestamp, latest_received_at = max(timestamps)
        age_sec = max(0, int((now - latest_timestamp).total_seconds()))
        if age_sec <= ONLINE_TELEMETRY_AGE_SEC:
            state = 'online'
        elif age_sec <= OFFLINE_TELEMETRY_AGE_SEC:
            state = 'stale'
        else:
            state = 'offline'
        return {
            'state': state,
            'last_received_at': latest_received_at,
            'age_sec': age_sec,
        }

    @staticmethod
    def _by_robot(
        rows: list[dict[str, Any]],
        *,
        multiple: bool = False,
    ) -> dict[str, Any]:
        if multiple:
            grouped: dict[str, list[dict[str, Any]]] = {}
            for row in rows:
                robot_id = row.get('robot_id')
                if robot_id:
                    grouped.setdefault(robot_id, []).append(row)
            return grouped
        return {
            row['robot_id']: row
            for row in rows
            if row.get('robot_id') is not None
        }

    @staticmethod
    def _json_value(value: str) -> Any:
        try:
            return json.loads(value)
        except (TypeError, json.JSONDecodeError):
            return value
