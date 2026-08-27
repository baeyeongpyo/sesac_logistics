from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
import sqlite3
from typing import Iterator
from uuid import uuid4


class PayloadType(str, Enum):
    EMPTY = "EMPTY"
    FRESH = "FRESH"
    NORMAL = "NORMAL"


class OperationStatus(str, Enum):
    QUEUED = "QUEUED"
    TO_PICK = "TO_PICK"
    PICKING = "PICKING"
    TO_PLACE = "TO_PLACE"
    PLACING = "PLACING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class Zone:
    zone_id: str
    name: str
    map_name: str
    nav_x: float
    nav_y: float
    nav_yaw: float
    capacity: int | None
    enabled: bool


@dataclass(frozen=True)
class Stock:
    zone_id: str
    payload_type: PayloadType
    quantity: int
    reserved_quantity: int
    version: int
    updated_at: str

    @property
    def available_quantity(self) -> int:
        return self.quantity - self.reserved_quantity


@dataclass(frozen=True)
class RobotPalletState:
    robot_id: str
    has_pallet: bool
    payload_type: PayloadType | None
    version: int
    reported_at: str | None
    updated_at: str | None


@dataclass(frozen=True)
class Operation:
    operation_id: str
    robot_id: str | None
    payload_type: PayloadType
    source_zone_id: str
    destination_zone_id: str
    status: OperationStatus
    priority: int
    failure_code: str | None
    version: int
    created_at: str
    updated_at: str
    completed_at: str | None


@dataclass(frozen=True)
class NextInstruction:
    robot_id: str
    operation_id: str
    action: str
    zone_id: str
    payload_type: PayloadType
    map_name: str
    nav_x: float
    nav_y: float
    nav_yaw: float


@dataclass(frozen=True)
class InventoryEvent:
    event_id: str
    idempotency_key: str
    operation_id: str | None
    robot_id: str | None
    event_type: str
    zone_id: str
    payload_type: PayloadType
    quantity_delta: int
    occurred_at: str
    created_at: str


class InventoryError(Exception):
    """Base class for inventory domain errors."""


class NotFoundError(InventoryError):
    """Raised when an inventory resource does not exist."""


class ConflictError(InventoryError):
    """Raised when an operation conflicts with the current inventory state."""


class InsufficientStockError(ConflictError):
    """Raised when there is no unreserved pallet stock to transport."""


class InventoryStore:
    def __init__(self, database_path: str | Path) -> None:
        self._database_path = Path(database_path)

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self._database_path, timeout=5, isolation_level=None)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA busy_timeout = 5000")
        connection.execute("PRAGMA synchronous = FULL")
        return connection

    @contextmanager
    def _write_connection(self) -> Iterator[sqlite3.Connection]:
        connection = self._connect()
        try:
            connection.execute("BEGIN IMMEDIATE")
            yield connection
            connection.execute("COMMIT")
        except BaseException:
            if connection.in_transaction:
                connection.execute("ROLLBACK")
            raise
        finally:
            connection.close()

    def initialize(self) -> None:
        self._database_path.parent.mkdir(parents=True, exist_ok=True)
        connection = self._connect()
        try:
            connection.execute("PRAGMA journal_mode = WAL")
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS zones (
                    zone_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    map_name TEXT NOT NULL,
                    nav_x REAL NOT NULL,
                    nav_y REAL NOT NULL,
                    nav_yaw REAL NOT NULL,
                    capacity INTEGER,
                    enabled INTEGER NOT NULL CHECK (enabled IN (0, 1)),
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    CHECK (capacity IS NULL OR capacity >= 0)
                );

                CREATE TABLE IF NOT EXISTS payload_types (
                    payload_type TEXT PRIMARY KEY,
                    enabled INTEGER NOT NULL CHECK (enabled IN (0, 1)),
                    created_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS pallet_stocks (
                    zone_id TEXT NOT NULL,
                    payload_type TEXT NOT NULL,
                    quantity INTEGER NOT NULL CHECK (quantity >= 0),
                    reserved_quantity INTEGER NOT NULL DEFAULT 0,
                    version INTEGER NOT NULL DEFAULT 1,
                    updated_at TEXT NOT NULL,
                    PRIMARY KEY (zone_id, payload_type),
                    FOREIGN KEY (zone_id) REFERENCES zones(zone_id),
                    FOREIGN KEY (payload_type) REFERENCES payload_types(payload_type),
                    CHECK (reserved_quantity >= 0 AND reserved_quantity <= quantity)
                );

                CREATE TABLE IF NOT EXISTS robot_pallet_states (
                    robot_id TEXT PRIMARY KEY,
                    has_pallet INTEGER NOT NULL CHECK (has_pallet IN (0, 1)),
                    payload_type TEXT,
                    version INTEGER NOT NULL DEFAULT 1,
                    reported_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    FOREIGN KEY (payload_type) REFERENCES payload_types(payload_type),
                    CHECK (
                        (has_pallet = 0 AND payload_type IS NULL)
                        OR (has_pallet = 1 AND payload_type IS NOT NULL)
                    )
                );

                CREATE TABLE IF NOT EXISTS transport_operations (
                    operation_id TEXT PRIMARY KEY,
                    robot_id TEXT,
                    payload_type TEXT NOT NULL,
                    source_zone_id TEXT NOT NULL,
                    destination_zone_id TEXT NOT NULL,
                    status TEXT NOT NULL,
                    priority INTEGER NOT NULL DEFAULT 0,
                    failure_code TEXT,
                    version INTEGER NOT NULL DEFAULT 1,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    completed_at TEXT,
                    FOREIGN KEY (payload_type) REFERENCES payload_types(payload_type),
                    FOREIGN KEY (source_zone_id) REFERENCES zones(zone_id),
                    FOREIGN KEY (destination_zone_id) REFERENCES zones(zone_id),
                    CHECK (source_zone_id <> destination_zone_id),
                    CHECK (status IN (
                        'QUEUED', 'TO_PICK', 'PICKING', 'TO_PLACE', 'PLACING',
                        'COMPLETED', 'FAILED', 'RECOVERY_REQUIRED', 'CANCELLED'
                    ))
                );

                CREATE TABLE IF NOT EXISTS inventory_events (
                    event_id TEXT PRIMARY KEY,
                    idempotency_key TEXT NOT NULL UNIQUE,
                    operation_id TEXT,
                    robot_id TEXT,
                    event_type TEXT NOT NULL,
                    zone_id TEXT NOT NULL,
                    payload_type TEXT NOT NULL,
                    quantity_delta INTEGER NOT NULL,
                    occurred_at TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    FOREIGN KEY (operation_id) REFERENCES transport_operations(operation_id),
                    FOREIGN KEY (zone_id) REFERENCES zones(zone_id),
                    FOREIGN KEY (payload_type) REFERENCES payload_types(payload_type),
                    CHECK (event_type IN ('PICK_COMPLETED', 'PLACE_COMPLETED', 'STOCK_ADJUSTED')),
                    CHECK (
                        (event_type = 'PICK_COMPLETED' AND quantity_delta = -1)
                        OR (event_type = 'PLACE_COMPLETED' AND quantity_delta = 1)
                        OR (event_type = 'STOCK_ADJUSTED' AND quantity_delta <> 0)
                    )
                );
                """
            )
            timestamp = self._timestamp()
            connection.executemany(
                """
                INSERT OR IGNORE INTO payload_types (payload_type, enabled, created_at)
                VALUES (?, 1, ?)
                """,
                [(payload_type.value, timestamp) for payload_type in PayloadType],
            )
        finally:
            connection.close()

    def upsert_zone(self, zone: Zone) -> Zone:
        self._validate_zone(zone)
        timestamp = self._timestamp()
        with self._write_connection() as connection:
            if zone.capacity is not None:
                occupied = connection.execute(
                    "SELECT COALESCE(SUM(quantity), 0) AS occupied FROM pallet_stocks WHERE zone_id = ?",
                    (zone.zone_id,),
                ).fetchone()["occupied"]
                inbound_reservations = self._destination_reservation_count(
                    connection, zone.zone_id
                )
                if occupied + inbound_reservations > zone.capacity:
                    raise ConflictError(
                        "zone capacity cannot be lower than current stock and inbound reservations"
                    )
            connection.execute(
                """
                INSERT INTO zones (
                    zone_id, name, map_name, nav_x, nav_y, nav_yaw, capacity,
                    enabled, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(zone_id) DO UPDATE SET
                    name = excluded.name,
                    map_name = excluded.map_name,
                    nav_x = excluded.nav_x,
                    nav_y = excluded.nav_y,
                    nav_yaw = excluded.nav_yaw,
                    capacity = excluded.capacity,
                    enabled = excluded.enabled,
                    updated_at = excluded.updated_at
                """,
                (
                    zone.zone_id,
                    zone.name,
                    zone.map_name,
                    zone.nav_x,
                    zone.nav_y,
                    zone.nav_yaw,
                    zone.capacity,
                    int(zone.enabled),
                    timestamp,
                    timestamp,
                ),
            )
        return zone

    def list_zones(self) -> list[Zone]:
        connection = self._connect()
        try:
            rows = connection.execute(
                """
                SELECT zone_id, name, map_name, nav_x, nav_y, nav_yaw, capacity, enabled
                FROM zones
                ORDER BY zone_id
                """
            ).fetchall()
            return [self._zone_from_row(row) for row in rows]
        finally:
            connection.close()

    def set_stock(
        self, zone_id: str, payload_type: PayloadType, quantity: int
    ) -> Stock:
        if quantity < 0:
            raise ValueError("quantity must be non-negative")
        timestamp = self._timestamp()
        with self._write_connection() as connection:
            zone = connection.execute(
                "SELECT zone_id, capacity FROM zones WHERE zone_id = ?", (zone_id,)
            ).fetchone()
            if zone is None:
                raise NotFoundError(f"unknown zone: {zone_id}")
            if zone["capacity"] is not None:
                other_quantity = connection.execute(
                    """
                    SELECT COALESCE(SUM(quantity), 0) AS quantity
                    FROM pallet_stocks
                    WHERE zone_id = ? AND payload_type <> ?
                    """,
                    (zone_id, payload_type.value),
                ).fetchone()["quantity"]
                inbound_reservations = self._destination_reservation_count(
                    connection, zone_id
                )
                if other_quantity + quantity + inbound_reservations > zone["capacity"]:
                    raise ConflictError("zone capacity would be exceeded")
            existing = connection.execute(
                """
                SELECT reserved_quantity FROM pallet_stocks
                WHERE zone_id = ? AND payload_type = ?
                """,
                (zone_id, payload_type.value),
            ).fetchone()
            if existing is not None and quantity < existing["reserved_quantity"]:
                raise ConflictError("quantity cannot be lower than reserved_quantity")
            connection.execute(
                """
                INSERT INTO pallet_stocks (
                    zone_id, payload_type, quantity, reserved_quantity, version, updated_at
                ) VALUES (?, ?, ?, 0, 1, ?)
                ON CONFLICT(zone_id, payload_type) DO UPDATE SET
                    quantity = excluded.quantity,
                    version = pallet_stocks.version + 1,
                    updated_at = excluded.updated_at
                """,
                (zone_id, payload_type.value, quantity, timestamp),
            )
            row = connection.execute(
                """
                SELECT zone_id, payload_type, quantity, reserved_quantity, version, updated_at
                FROM pallet_stocks
                WHERE zone_id = ? AND payload_type = ?
                """,
                (zone_id, payload_type.value),
            ).fetchone()
        return self._stock_from_row(row)

    def list_stocks(self) -> list[Stock]:
        connection = self._connect()
        try:
            rows = connection.execute(
                """
                SELECT zone_id, payload_type, quantity, reserved_quantity, version, updated_at
                FROM pallet_stocks
                ORDER BY zone_id, payload_type
                """
            ).fetchall()
            return [self._stock_from_row(row) for row in rows]
        finally:
            connection.close()

    def stock(self, zone_id: str, payload_type: PayloadType) -> Stock:
        connection = self._connect()
        try:
            row = connection.execute(
                """
                SELECT zone_id, payload_type, quantity, reserved_quantity, version, updated_at
                FROM pallet_stocks
                WHERE zone_id = ? AND payload_type = ?
                """,
                (zone_id, payload_type.value),
            ).fetchone()
            if row is None:
                raise NotFoundError(
                    f"stock does not exist: {zone_id}/{payload_type.value}"
                )
            return self._stock_from_row(row)
        finally:
            connection.close()

    def create_operation(
        self,
        payload_type: PayloadType,
        source_zone_id: str,
        destination_zone_id: str,
        priority: int = 0,
    ) -> Operation:
        if source_zone_id == destination_zone_id:
            raise ValueError("source and destination zones must differ")

        timestamp = self._timestamp()
        with self._write_connection() as connection:
            self._require_zone(connection, source_zone_id)
            self._require_zone(connection, destination_zone_id)
            destination = connection.execute(
                "SELECT capacity FROM zones WHERE zone_id = ?", (destination_zone_id,)
            ).fetchone()
            if destination["capacity"] is not None:
                occupied = connection.execute(
                    "SELECT COALESCE(SUM(quantity), 0) AS occupied FROM pallet_stocks WHERE zone_id = ?",
                    (destination_zone_id,),
                ).fetchone()["occupied"]
                inbound_reservations = self._destination_reservation_count(
                    connection, destination_zone_id
                )
                if occupied + inbound_reservations + 1 > destination["capacity"]:
                    raise ConflictError("destination zone capacity is full")
            reservation = connection.execute(
                """
                UPDATE pallet_stocks
                SET reserved_quantity = reserved_quantity + 1,
                    version = version + 1,
                    updated_at = ?
                WHERE zone_id = ?
                  AND payload_type = ?
                  AND quantity > reserved_quantity
                """,
                (timestamp, source_zone_id, payload_type.value),
            )
            if reservation.rowcount != 1:
                raise InsufficientStockError(
                    f"no available {payload_type.value} stock in {source_zone_id}"
                )
            operation_id = str(uuid4())
            try:
                connection.execute(
                    """
                    INSERT INTO transport_operations (
                        operation_id, robot_id, payload_type, source_zone_id,
                        destination_zone_id, status, priority, failure_code, version,
                        created_at, updated_at, completed_at
                    ) VALUES (?, NULL, ?, ?, ?, ?, ?, NULL, 1, ?, ?, NULL)
                    """,
                    (
                        operation_id,
                        payload_type.value,
                        source_zone_id,
                        destination_zone_id,
                        OperationStatus.QUEUED.value,
                        priority,
                        timestamp,
                        timestamp,
                    ),
                )
            except sqlite3.IntegrityError as error:
                raise ConflictError(f"operation already exists: {operation_id}") from error
            row = connection.execute(
                "SELECT * FROM transport_operations WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return self._operation_from_row(row)

    def assign_operation(self, operation_id: str, robot_id: str) -> Operation:
        if not robot_id:
            raise ValueError("robot_id must not be empty")
        timestamp = self._timestamp()
        with self._write_connection() as connection:
            operation = connection.execute(
                "SELECT * FROM transport_operations WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if operation is None:
                raise NotFoundError(f"unknown operation: {operation_id}")
            if operation["status"] != OperationStatus.QUEUED.value:
                raise ConflictError(f"operation is not queueable: {operation_id}")

            active_operation = connection.execute(
                """
                SELECT operation_id FROM transport_operations
                WHERE robot_id = ?
                  AND status IN ('TO_PICK', 'PICKING', 'TO_PLACE', 'PLACING', 'RECOVERY_REQUIRED')
                """,
                (robot_id,),
            ).fetchone()
            if active_operation is not None:
                raise ConflictError(f"robot already has an active operation: {robot_id}")

            robot_state = connection.execute(
                "SELECT has_pallet FROM robot_pallet_states WHERE robot_id = ?",
                (robot_id,),
            ).fetchone()
            if robot_state is not None and robot_state["has_pallet"]:
                raise ConflictError(f"robot already carries a pallet: {robot_id}")

            connection.execute(
                """
                UPDATE transport_operations
                SET robot_id = ?, status = ?, version = version + 1, updated_at = ?
                WHERE operation_id = ?
                """,
                (robot_id, OperationStatus.TO_PICK.value, timestamp, operation_id),
            )
            row = connection.execute(
                "SELECT * FROM transport_operations WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return self._operation_from_row(row)

    def get_robot_pallet_state(self, robot_id: str) -> RobotPalletState:
        connection = self._connect()
        try:
            row = connection.execute(
                "SELECT * FROM robot_pallet_states WHERE robot_id = ?", (robot_id,)
            ).fetchone()
            if row is None:
                return RobotPalletState(robot_id, False, None, 0, None, None)
            return self._robot_state_from_row(row)
        finally:
            connection.close()

    def next_instruction(self, robot_id: str) -> NextInstruction | None:
        robot_state = self.get_robot_pallet_state(robot_id)
        target_status = (
            OperationStatus.TO_PLACE if robot_state.has_pallet else OperationStatus.TO_PICK
        )
        zone_column = (
            "destination_zone_id" if robot_state.has_pallet else "source_zone_id"
        )
        action = "PLACE" if robot_state.has_pallet else "PICK"
        connection = self._connect()
        try:
            operation = connection.execute(
                """
                SELECT * FROM transport_operations
                WHERE robot_id = ? AND status = ?
                ORDER BY priority DESC, created_at, operation_id
                LIMIT 1
                """,
                (robot_id, target_status.value),
            ).fetchone()
            if operation is None:
                return None
            zone = connection.execute(
                f"""
                SELECT zone_id, map_name, nav_x, nav_y, nav_yaw
                FROM zones
                WHERE zone_id = ?
                """,
                (operation[zone_column],),
            ).fetchone()
            if zone is None:
                raise NotFoundError(f"unknown zone: {operation[zone_column]}")
            return NextInstruction(
                robot_id=robot_id,
                operation_id=operation["operation_id"],
                action=action,
                zone_id=zone["zone_id"],
                payload_type=PayloadType(operation["payload_type"]),
                map_name=zone["map_name"],
                nav_x=zone["nav_x"],
                nav_y=zone["nav_y"],
                nav_yaw=zone["nav_yaw"],
            )
        finally:
            connection.close()

    def complete_pick(
        self,
        operation_id: str,
        robot_id: str,
        idempotency_key: str,
        occurred_at: str | None = None,
    ) -> InventoryEvent:
        if not idempotency_key:
            raise ValueError("idempotency_key must not be empty")
        event_timestamp = occurred_at or self._timestamp()
        created_at = self._timestamp()
        with self._write_connection() as connection:
            existing = connection.execute(
                "SELECT * FROM inventory_events WHERE idempotency_key = ?",
                (idempotency_key,),
            ).fetchone()
            if existing is not None:
                self._validate_duplicate_event(
                    existing, operation_id, robot_id, "PICK_COMPLETED"
                )
                return self._event_from_row(existing)

            operation = connection.execute(
                "SELECT * FROM transport_operations WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if operation is None:
                raise NotFoundError(f"unknown operation: {operation_id}")
            if operation["robot_id"] != robot_id:
                raise ConflictError("pick robot does not own this operation")
            if operation["status"] != OperationStatus.TO_PICK.value:
                raise ConflictError("operation is not ready to pick")

            robot_state = connection.execute(
                "SELECT has_pallet FROM robot_pallet_states WHERE robot_id = ?",
                (robot_id,),
            ).fetchone()
            if robot_state is not None and robot_state["has_pallet"]:
                raise ConflictError("robot already carries a pallet")

            stock_update = connection.execute(
                """
                UPDATE pallet_stocks
                SET quantity = quantity - 1,
                    reserved_quantity = reserved_quantity - 1,
                    version = version + 1,
                    updated_at = ?
                WHERE zone_id = ?
                  AND payload_type = ?
                  AND quantity > 0
                  AND reserved_quantity > 0
                """,
                (
                    created_at,
                    operation["source_zone_id"],
                    operation["payload_type"],
                ),
            )
            if stock_update.rowcount != 1:
                raise ConflictError("reserved source stock is no longer available")

            connection.execute(
                """
                INSERT INTO robot_pallet_states (
                    robot_id, has_pallet, payload_type, version, reported_at, updated_at
                ) VALUES (?, 1, ?, 1, ?, ?)
                ON CONFLICT(robot_id) DO UPDATE SET
                    has_pallet = 1,
                    payload_type = excluded.payload_type,
                    version = robot_pallet_states.version + 1,
                    reported_at = excluded.reported_at,
                    updated_at = excluded.updated_at
                """,
                (robot_id, operation["payload_type"], event_timestamp, created_at),
            )
            connection.execute(
                """
                UPDATE transport_operations
                SET status = ?, version = version + 1, updated_at = ?
                WHERE operation_id = ?
                """,
                (OperationStatus.TO_PLACE.value, created_at, operation_id),
            )
            event = InventoryEvent(
                event_id=str(uuid4()),
                idempotency_key=idempotency_key,
                operation_id=operation_id,
                robot_id=robot_id,
                event_type="PICK_COMPLETED",
                zone_id=operation["source_zone_id"],
                payload_type=PayloadType(operation["payload_type"]),
                quantity_delta=-1,
                occurred_at=event_timestamp,
                created_at=created_at,
            )
            connection.execute(
                """
                INSERT INTO inventory_events (
                    event_id, idempotency_key, operation_id, robot_id, event_type,
                    zone_id, payload_type, quantity_delta, occurred_at, created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    event.event_id,
                    event.idempotency_key,
                    event.operation_id,
                    event.robot_id,
                    event.event_type,
                    event.zone_id,
                    event.payload_type.value,
                    event.quantity_delta,
                    event.occurred_at,
                    event.created_at,
                ),
            )
            return event

    def complete_place(
        self,
        operation_id: str,
        robot_id: str,
        idempotency_key: str,
        occurred_at: str | None = None,
    ) -> InventoryEvent:
        if not idempotency_key:
            raise ValueError("idempotency_key must not be empty")
        event_timestamp = occurred_at or self._timestamp()
        created_at = self._timestamp()
        with self._write_connection() as connection:
            existing = connection.execute(
                "SELECT * FROM inventory_events WHERE idempotency_key = ?",
                (idempotency_key,),
            ).fetchone()
            if existing is not None:
                self._validate_duplicate_event(
                    existing, operation_id, robot_id, "PLACE_COMPLETED"
                )
                return self._event_from_row(existing)

            operation = connection.execute(
                "SELECT * FROM transport_operations WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if operation is None:
                raise NotFoundError(f"unknown operation: {operation_id}")
            if operation["robot_id"] != robot_id:
                raise ConflictError("place robot does not own this operation")
            if operation["status"] != OperationStatus.TO_PLACE.value:
                raise ConflictError("operation is not ready to place")

            robot_state = connection.execute(
                "SELECT * FROM robot_pallet_states WHERE robot_id = ?", (robot_id,)
            ).fetchone()
            if robot_state is None or not robot_state["has_pallet"]:
                raise ConflictError("robot does not carry a pallet")
            if robot_state["payload_type"] != operation["payload_type"]:
                raise ConflictError("robot pallet type does not match the operation")

            destination = connection.execute(
                "SELECT capacity FROM zones WHERE zone_id = ?",
                (operation["destination_zone_id"],),
            ).fetchone()
            if destination is None:
                raise NotFoundError(
                    f"unknown zone: {operation['destination_zone_id']}"
                )
            if destination["capacity"] is not None:
                occupied = connection.execute(
                    "SELECT COALESCE(SUM(quantity), 0) AS occupied FROM pallet_stocks WHERE zone_id = ?",
                    (operation["destination_zone_id"],),
                ).fetchone()["occupied"]
                if occupied + 1 > destination["capacity"]:
                    raise ConflictError("destination zone capacity is full")

            connection.execute(
                """
                INSERT INTO pallet_stocks (
                    zone_id, payload_type, quantity, reserved_quantity, version, updated_at
                ) VALUES (?, ?, 1, 0, 1, ?)
                ON CONFLICT(zone_id, payload_type) DO UPDATE SET
                    quantity = pallet_stocks.quantity + 1,
                    version = pallet_stocks.version + 1,
                    updated_at = excluded.updated_at
                """,
                (
                    operation["destination_zone_id"],
                    operation["payload_type"],
                    created_at,
                ),
            )
            connection.execute(
                """
                UPDATE robot_pallet_states
                SET has_pallet = 0,
                    payload_type = NULL,
                    version = version + 1,
                    reported_at = ?,
                    updated_at = ?
                WHERE robot_id = ?
                """,
                (event_timestamp, created_at, robot_id),
            )
            connection.execute(
                """
                UPDATE transport_operations
                SET status = ?, version = version + 1, updated_at = ?, completed_at = ?
                WHERE operation_id = ?
                """,
                (
                    OperationStatus.COMPLETED.value,
                    created_at,
                    created_at,
                    operation_id,
                ),
            )
            event = InventoryEvent(
                event_id=str(uuid4()),
                idempotency_key=idempotency_key,
                operation_id=operation_id,
                robot_id=robot_id,
                event_type="PLACE_COMPLETED",
                zone_id=operation["destination_zone_id"],
                payload_type=PayloadType(operation["payload_type"]),
                quantity_delta=1,
                occurred_at=event_timestamp,
                created_at=created_at,
            )
            connection.execute(
                """
                INSERT INTO inventory_events (
                    event_id, idempotency_key, operation_id, robot_id, event_type,
                    zone_id, payload_type, quantity_delta, occurred_at, created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    event.event_id,
                    event.idempotency_key,
                    event.operation_id,
                    event.robot_id,
                    event.event_type,
                    event.zone_id,
                    event.payload_type.value,
                    event.quantity_delta,
                    event.occurred_at,
                    event.created_at,
                ),
            )
            return event

    def operation(self, operation_id: str) -> Operation:
        connection = self._connect()
        try:
            row = connection.execute(
                "SELECT * FROM transport_operations WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if row is None:
                raise NotFoundError(f"unknown operation: {operation_id}")
            return self._operation_from_row(row)
        finally:
            connection.close()

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    @staticmethod
    def _validate_zone(zone: Zone) -> None:
        if not zone.zone_id:
            raise ValueError("zone_id must not be empty")
        if not zone.name:
            raise ValueError("name must not be empty")
        if not zone.map_name:
            raise ValueError("map_name must not be empty")
        if zone.capacity is not None and zone.capacity < 0:
            raise ValueError("capacity must be non-negative")

    @staticmethod
    def _stock_from_row(row: sqlite3.Row) -> Stock:
        return Stock(
            zone_id=row["zone_id"],
            payload_type=PayloadType(row["payload_type"]),
            quantity=row["quantity"],
            reserved_quantity=row["reserved_quantity"],
            version=row["version"],
            updated_at=row["updated_at"],
        )

    @staticmethod
    def _zone_from_row(row: sqlite3.Row) -> Zone:
        return Zone(
            zone_id=row["zone_id"],
            name=row["name"],
            map_name=row["map_name"],
            nav_x=row["nav_x"],
            nav_y=row["nav_y"],
            nav_yaw=row["nav_yaw"],
            capacity=row["capacity"],
            enabled=bool(row["enabled"]),
        )

    @staticmethod
    def _require_zone(connection: sqlite3.Connection, zone_id: str) -> None:
        row = connection.execute(
            "SELECT zone_id FROM zones WHERE zone_id = ?", (zone_id,)
        ).fetchone()
        if row is None:
            raise NotFoundError(f"unknown zone: {zone_id}")

    @staticmethod
    def _destination_reservation_count(
        connection: sqlite3.Connection, zone_id: str
    ) -> int:
        return connection.execute(
            """
            SELECT COUNT(*) AS reserved
            FROM transport_operations
            WHERE destination_zone_id = ?
              AND status IN (
                  'QUEUED', 'TO_PICK', 'PICKING', 'TO_PLACE', 'PLACING',
                  'RECOVERY_REQUIRED'
              )
            """,
            (zone_id,),
        ).fetchone()["reserved"]

    @staticmethod
    def _operation_from_row(row: sqlite3.Row) -> Operation:
        return Operation(
            operation_id=row["operation_id"],
            robot_id=row["robot_id"],
            payload_type=PayloadType(row["payload_type"]),
            source_zone_id=row["source_zone_id"],
            destination_zone_id=row["destination_zone_id"],
            status=OperationStatus(row["status"]),
            priority=row["priority"],
            failure_code=row["failure_code"],
            version=row["version"],
            created_at=row["created_at"],
            updated_at=row["updated_at"],
            completed_at=row["completed_at"],
        )

    @staticmethod
    def _robot_state_from_row(row: sqlite3.Row) -> RobotPalletState:
        payload_type = row["payload_type"]
        return RobotPalletState(
            robot_id=row["robot_id"],
            has_pallet=bool(row["has_pallet"]),
            payload_type=PayloadType(payload_type) if payload_type is not None else None,
            version=row["version"],
            reported_at=row["reported_at"],
            updated_at=row["updated_at"],
        )

    @staticmethod
    def _event_from_row(row: sqlite3.Row) -> InventoryEvent:
        return InventoryEvent(
            event_id=row["event_id"],
            idempotency_key=row["idempotency_key"],
            operation_id=row["operation_id"],
            robot_id=row["robot_id"],
            event_type=row["event_type"],
            zone_id=row["zone_id"],
            payload_type=PayloadType(row["payload_type"]),
            quantity_delta=row["quantity_delta"],
            occurred_at=row["occurred_at"],
            created_at=row["created_at"],
        )

    @staticmethod
    def _validate_duplicate_event(
        event: sqlite3.Row,
        operation_id: str,
        robot_id: str,
        event_type: str,
    ) -> None:
        if (
            event["operation_id"] != operation_id
            or event["robot_id"] != robot_id
            or event["event_type"] != event_type
        ):
            raise ConflictError("idempotency key was already used by another event")
