import json
import sqlite3
import threading
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta
from enum import Enum
from pathlib import Path
from typing import Any, Callable
from urllib.error import URLError
from urllib.request import Request, urlopen
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, field_validator


class VehicleState(str, Enum):
    INIT = "INIT"
    WAIT = "WAIT"
    DRIVE = "DRIVE"
    PICK = "PICK"
    PLACE = "PLACE"
    FAIL = "FAIL"


class StateSource(str, Enum):
    VEHICLE = "VEHICLE"
    NAV2 = "NAV2"
    AUTO_DOCK = "AUTO_DOCK"
    API = "API"
    FORK = "FORK"


class VehicleStateReport(BaseModel):
    model_config = ConfigDict(extra="forbid")

    state: VehicleState
    previous_state: VehicleState | None
    operation_id: str | None
    attempt_id: str | None
    source: StateSource
    detail: str
    observed_at: datetime

    @field_validator("observed_at")
    @classmethod
    def observed_at_must_be_utc(cls, value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() != UTC.utcoffset(value):
            raise ValueError("observed_at must be UTC")
        return value.astimezone(UTC)


class VehicleSnapshot(VehicleStateReport):
    robot_id: str
    updated_at: datetime


class VehicleStateLog(VehicleStateReport):
    id: int
    robot_id: str
    created_at: datetime


@dataclass(frozen=True)
class DirectPallet3Mission:
    operation_id: str
    robot_id: str
    pick_mode: str
    phase: str
    pick_idempotency_key: str | None
    place_idempotency_key: str | None
    return_idempotency_key: str | None
    created_at: str
    updated_at: str


class FleetOutboxEvent(BaseModel):
    model_config = ConfigDict(frozen=True)

    event_id: str
    event_type: str
    occurred_at: datetime
    payload: dict[str, Any]
    attempt_count: int
    next_attempt_at: datetime
    last_error: str | None


class VehicleStateStore:
    def __init__(self, database_path: str | Path) -> None:
        self._connection = sqlite3.connect(str(database_path), check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        self._initialize()

    def _initialize(self) -> None:
        with self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS vehicle_states (
                    robot_id TEXT PRIMARY KEY,
                    state TEXT NOT NULL,
                    previous_state TEXT,
                    operation_id TEXT,
                    attempt_id TEXT,
                    source TEXT NOT NULL,
                    detail TEXT NOT NULL,
                    observed_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS vehicle_state_logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_id TEXT NOT NULL,
                    state TEXT NOT NULL,
                    previous_state TEXT,
                    operation_id TEXT,
                    attempt_id TEXT,
                    source TEXT NOT NULL,
                    detail TEXT NOT NULL,
                    observed_at TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS vehicle_state_logs_robot_id_id
                    ON vehicle_state_logs (robot_id, id DESC);
                CREATE TABLE IF NOT EXISTS fleet_event_outbox (
                    event_id TEXT PRIMARY KEY,
                    event_type TEXT NOT NULL,
                    occurred_at TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    attempt_count INTEGER NOT NULL DEFAULT 0,
                    next_attempt_at TEXT NOT NULL,
                    last_error TEXT,
                    delivered_at TEXT
                );
                CREATE INDEX IF NOT EXISTS fleet_event_outbox_pending
                    ON fleet_event_outbox (delivered_at, next_attempt_at);
                CREATE TABLE IF NOT EXISTS direct_pallet3_missions (
                    operation_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    pick_mode TEXT NOT NULL CHECK (pick_mode IN ('auto_dock', 'manual')),
                    phase TEXT NOT NULL CHECK (phase IN ('STARTING', 'STARTED', 'PICKED', 'PLACED', 'RETURNED')),
                    pick_idempotency_key TEXT,
                    place_idempotency_key TEXT,
                    return_idempotency_key TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS direct_pallet3_replies (
                    operation_id TEXT NOT NULL,
                    detail TEXT NOT NULL,
                    recorded_at TEXT NOT NULL,
                    PRIMARY KEY (operation_id, detail),
                    FOREIGN KEY (operation_id) REFERENCES direct_pallet3_missions(operation_id)
                );
                """
            )

    def close(self) -> None:
        with self._lock:
            self._connection.close()

    def record_state(self, robot_id: str, report: VehicleStateReport) -> VehicleSnapshot:
        with self._lock, self._connection:
            current = self._fetch_vehicle(robot_id)
            updated_at = _now()
            values = _report_values(report)
            self._connection.execute(
                """
                INSERT INTO vehicle_states (
                    robot_id, state, previous_state, operation_id, attempt_id,
                    source, detail, observed_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(robot_id) DO UPDATE SET
                    state = excluded.state,
                    previous_state = excluded.previous_state,
                    operation_id = excluded.operation_id,
                    attempt_id = excluded.attempt_id,
                    source = excluded.source,
                    detail = excluded.detail,
                    observed_at = excluded.observed_at,
                    updated_at = excluded.updated_at
                """,
                (robot_id, *values, _format_time(updated_at)),
            )
            if current is None or current.state != report.state:
                self._connection.execute(
                    """
                    INSERT INTO vehicle_state_logs (
                        robot_id, state, previous_state, operation_id, attempt_id,
                        source, detail, observed_at, created_at
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (robot_id, *values, _format_time(updated_at)),
                )
            self._enqueue_outbox(robot_id, report)
            return self._fetch_vehicle(robot_id)

    def get_vehicle(self, robot_id: str) -> VehicleSnapshot | None:
        with self._lock:
            return self._fetch_vehicle(robot_id)

    def list_vehicles(self) -> list[VehicleSnapshot]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT * FROM vehicle_states ORDER BY robot_id"
            ).fetchall()
        return [_snapshot_from_row(row) for row in rows]

    def list_logs(
        self, robot_id: str, *, limit: int = 100, before_id: int | None = None
    ) -> list[VehicleStateLog]:
        query = "SELECT * FROM vehicle_state_logs WHERE robot_id = ?"
        values: list[object] = [robot_id]
        if before_id is not None:
            query += " AND id < ?"
            values.append(before_id)
        query += " ORDER BY id DESC LIMIT ?"
        values.append(limit)
        with self._lock:
            rows = self._connection.execute(query, values).fetchall()
        return [_log_from_row(row) for row in rows]

    def create_direct_pallet3_mission(
        self, operation_id: str, robot_id: str, pick_mode: str
    ) -> DirectPallet3Mission:
        timestamp = _format_time(_now())
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO direct_pallet3_missions (
                    operation_id, robot_id, pick_mode, phase, created_at, updated_at
                ) VALUES (?, ?, ?, 'STARTING', ?, ?)
                """,
                (operation_id, robot_id, pick_mode, timestamp, timestamp),
            )
            return self._fetch_direct_pallet3_mission(operation_id)

    def get_direct_pallet3_mission(
        self, operation_id: str
    ) -> DirectPallet3Mission | None:
        with self._lock:
            return self._fetch_direct_pallet3_mission(operation_id)

    def get_active_direct_pallet3_mission(
        self, robot_id: str
    ) -> DirectPallet3Mission | None:
        with self._lock:
            row = self._connection.execute(
                """
                SELECT * FROM direct_pallet3_missions
                WHERE robot_id = ? AND phase != 'RETURNED'
                ORDER BY created_at DESC
                LIMIT 1
                """,
                (robot_id,),
            ).fetchone()
        return _direct_pallet3_mission_from_row(row) if row is not None else None

    def mark_direct_pallet3_started(self, operation_id: str) -> DirectPallet3Mission:
        return self._transition_direct_pallet3_mission(
            operation_id, "STARTING", "STARTED", None, None
        )

    def record_direct_pallet3_reply(
        self, robot_id: str, operation_id: str, detail: str
    ) -> None:
        with self._lock, self._connection:
            mission = self._fetch_direct_pallet3_mission(operation_id)
            if mission is None or mission.robot_id != robot_id:
                return
            self._connection.execute(
                """
                INSERT OR IGNORE INTO direct_pallet3_replies (
                    operation_id, detail, recorded_at
                ) VALUES (?, ?, ?)
                """,
                (operation_id, detail, _format_time(_now())),
            )

    def has_direct_pallet3_reply(self, operation_id: str, detail: str) -> bool:
        with self._lock:
            row = self._connection.execute(
                """
                SELECT 1 FROM direct_pallet3_replies
                WHERE operation_id = ? AND detail = ?
                """,
                (operation_id, detail),
            ).fetchone()
        return row is not None

    def complete_direct_pallet3_event(
        self,
        operation_id: str,
        expected_phase: str,
        next_phase: str,
        event_column: str,
        idempotency_key: str,
    ) -> DirectPallet3Mission:
        if event_column not in {
            "pick_idempotency_key",
            "place_idempotency_key",
            "return_idempotency_key",
        }:
            raise ValueError(f"unsupported direct pallet3 event column: {event_column}")
        return self._transition_direct_pallet3_mission(
            operation_id, expected_phase, next_phase, event_column, idempotency_key
        )

    def list_pending_outbox_events(
        self, *, include_scheduled: bool = False
    ) -> list[FleetOutboxEvent]:
        query = "SELECT * FROM fleet_event_outbox WHERE delivered_at IS NULL"
        values: tuple[object, ...] = ()
        if not include_scheduled:
            query += " AND next_attempt_at <= ?"
            values = (_format_time(_now()),)
        query += " ORDER BY rowid"
        with self._lock:
            rows = self._connection.execute(query, values).fetchall()
        return [_outbox_event_from_row(row) for row in rows]

    def _fetch_direct_pallet3_mission(
        self, operation_id: str
    ) -> DirectPallet3Mission | None:
        row = self._connection.execute(
            "SELECT * FROM direct_pallet3_missions WHERE operation_id = ?",
            (operation_id,),
        ).fetchone()
        return _direct_pallet3_mission_from_row(row) if row is not None else None

    def _transition_direct_pallet3_mission(
        self,
        operation_id: str,
        expected_phase: str,
        next_phase: str,
        event_column: str | None,
        idempotency_key: str | None,
    ) -> DirectPallet3Mission:
        with self._lock, self._connection:
            mission = self._fetch_direct_pallet3_mission(operation_id)
            if mission is None:
                raise KeyError(f"unknown pallet3 mission: {operation_id}")
            current_key = (
                getattr(mission, event_column) if event_column is not None else None
            )
            if mission.phase == next_phase:
                if event_column is not None and current_key == idempotency_key:
                    return mission
                raise ValueError(
                    f"pallet3 mission {operation_id} already reached {next_phase}"
                )
            if mission.phase != expected_phase:
                raise ValueError(
                    f"pallet3 mission {operation_id} is {mission.phase}, expected {expected_phase}"
                )
            assignments = ["phase = ?", "updated_at = ?"]
            values: list[object] = [next_phase, _format_time(_now())]
            if event_column is not None:
                assignments.append(f"{event_column} = ?")
                values.append(idempotency_key)
            values.append(operation_id)
            self._connection.execute(
                f"UPDATE direct_pallet3_missions SET {', '.join(assignments)} WHERE operation_id = ?",
                values,
            )
            return self._fetch_direct_pallet3_mission(operation_id)

    def mark_outbox_delivered(self, event_id: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                UPDATE fleet_event_outbox
                SET delivered_at = ?, last_error = NULL
                WHERE event_id = ? AND delivered_at IS NULL
                """,
                (_format_time(_now()), event_id),
            )

    def record_outbox_delivery_failure(
        self, event: FleetOutboxEvent, error: str, retry_interval_sec: float
    ) -> None:
        attempts = event.attempt_count + 1
        delay = retry_interval_sec * (2 ** min(attempts - 1, 8))
        next_attempt = _now() + timedelta(seconds=delay)
        with self._lock, self._connection:
            self._connection.execute(
                """
                UPDATE fleet_event_outbox
                SET attempt_count = ?, next_attempt_at = ?, last_error = ?
                WHERE event_id = ? AND delivered_at IS NULL
                """,
                (attempts, _format_time(next_attempt), error[:1000], event.event_id),
            )

    def _enqueue_outbox(self, robot_id: str, report: VehicleStateReport) -> None:
        payload = {"robot_id": robot_id, **report.model_dump(mode="json")}
        observed_at = _format_time(report.observed_at)
        self._connection.execute(
            """
            INSERT INTO fleet_event_outbox (
                event_id, event_type, occurred_at, payload_json,
                attempt_count, next_attempt_at, last_error, delivered_at
            ) VALUES (?, 'fleet.vehicle_reported', ?, ?, 0, ?, NULL, NULL)
            """,
            (
                str(uuid4()),
                observed_at,
                json.dumps(payload, sort_keys=True, separators=(",", ":")),
                _format_time(_now()),
            ),
        )

    def _fetch_vehicle(self, robot_id: str) -> VehicleSnapshot | None:
        row = self._connection.execute(
            "SELECT * FROM vehicle_states WHERE robot_id = ?", (robot_id,)
        ).fetchone()
        return _snapshot_from_row(row) if row is not None else None


def _report_values(report: VehicleStateReport) -> tuple[object, ...]:
    return (
        report.state.value,
        report.previous_state.value if report.previous_state else None,
        report.operation_id,
        report.attempt_id,
        report.source.value,
        report.detail,
        _format_time(report.observed_at),
    )


def _snapshot_from_row(row: sqlite3.Row) -> VehicleSnapshot:
    return VehicleSnapshot(
        robot_id=row["robot_id"],
        state=row["state"],
        previous_state=row["previous_state"],
        operation_id=row["operation_id"],
        attempt_id=row["attempt_id"],
        source=row["source"],
        detail=row["detail"],
        observed_at=row["observed_at"],
        updated_at=row["updated_at"],
    )


def _log_from_row(row: sqlite3.Row) -> VehicleStateLog:
    return VehicleStateLog(
        id=row["id"],
        robot_id=row["robot_id"],
        state=row["state"],
        previous_state=row["previous_state"],
        operation_id=row["operation_id"],
        attempt_id=row["attempt_id"],
        source=row["source"],
        detail=row["detail"],
        observed_at=row["observed_at"],
        created_at=row["created_at"],
    )


def _direct_pallet3_mission_from_row(row: sqlite3.Row) -> DirectPallet3Mission:
    return DirectPallet3Mission(
        operation_id=row["operation_id"],
        robot_id=row["robot_id"],
        pick_mode=row["pick_mode"],
        phase=row["phase"],
        pick_idempotency_key=row["pick_idempotency_key"],
        place_idempotency_key=row["place_idempotency_key"],
        return_idempotency_key=row["return_idempotency_key"],
        created_at=row["created_at"],
        updated_at=row["updated_at"],
    )


def _outbox_event_from_row(row: sqlite3.Row) -> FleetOutboxEvent:
    return FleetOutboxEvent(
        event_id=row["event_id"],
        event_type=row["event_type"],
        occurred_at=row["occurred_at"],
        payload=json.loads(row["payload_json"]),
        attempt_count=row["attempt_count"],
        next_attempt_at=row["next_attempt_at"],
        last_error=row["last_error"],
    )


def _now() -> datetime:
    return datetime.now(UTC)


def _format_time(value: datetime) -> str:
    return value.astimezone(UTC).isoformat().replace("+00:00", "Z")


class FleetEventDispatcher:
    def __init__(
        self,
        store: VehicleStateStore,
        destination_url: str,
        *,
        retry_interval_sec: float,
    ) -> None:
        self._store = store
        self._destination_url = destination_url.rstrip("/")
        self._retry_interval_sec = max(retry_interval_sec, 0.1)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._run, name="fleet-event-outbox", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(self._retry_interval_sec * 2, 1.0))

    def flush_once(
        self, deliver: Callable[[FleetOutboxEvent], bool] | None = None
    ) -> int:
        delivery = deliver or self._deliver
        delivered_count = 0
        for event in self._store.list_pending_outbox_events():
            try:
                delivered = delivery(event)
            except Exception as error:
                self._store.record_outbox_delivery_failure(
                    event, str(error), self._retry_interval_sec
                )
                continue
            if delivered:
                self._store.mark_outbox_delivered(event.event_id)
                delivered_count += 1
            else:
                self._store.record_outbox_delivery_failure(
                    event, "orchestrator did not return 2xx", self._retry_interval_sec
                )
        return delivered_count

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self.flush_once()
            self._stop_event.wait(self._retry_interval_sec)

    def _deliver(self, event: FleetOutboxEvent) -> bool:
        body = json.dumps(
            {
                "event_id": event.event_id,
                "event_type": event.event_type,
                "occurred_at": _format_time(event.occurred_at),
                "payload": event.payload,
            }
        ).encode("utf-8")
        request = Request(
            self._destination_url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=3) as response:
                return 200 <= response.status < 300
        except URLError:
            return False
