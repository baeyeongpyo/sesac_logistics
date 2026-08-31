import sqlite3
import threading
from datetime import UTC, datetime
from enum import Enum
from pathlib import Path

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


def _now() -> datetime:
    return datetime.now(UTC)


def _format_time(value: datetime) -> str:
    return value.astimezone(UTC).isoformat().replace("+00:00", "Z")
