from __future__ import annotations

from datetime import UTC, datetime
import json
from pathlib import Path
import sqlite3
import threading
from uuid import uuid4

from .models import (
    CommandRecord,
    EventEnvelope,
    OperationStep,
    Pallet3Mission,
    Pallet3OperationWorkflow,
)


class OrchestratorStore:
    def __init__(self, database_path: str | Path) -> None:
        self._database_path = Path(database_path)
        self._database_path.parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(
            str(self._database_path), check_same_thread=False
        )
        self._connection.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        self._initialize()

    def _initialize(self) -> None:
        with self._lock, self._connection:
            self._connection.execute("PRAGMA journal_mode = WAL")
            self._connection.execute("PRAGMA foreign_keys = ON")
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS orchestrator_inbox (
                    source TEXT NOT NULL,
                    event_id TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    occurred_at TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    received_at TEXT NOT NULL,
                    PRIMARY KEY (source, event_id)
                );
                CREATE TABLE IF NOT EXISTS orchestrator_steps (
                    operation_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    phase TEXT NOT NULL,
                    source_zone_id TEXT NOT NULL,
                    destination_zone_id TEXT NOT NULL,
                    payload_type TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS command_outbox (
                    command_id TEXT PRIMARY KEY,
                    operation_id TEXT NOT NULL,
                    robot_id TEXT NOT NULL,
                    command_type TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    status TEXT NOT NULL,
                    last_error TEXT,
                    created_at TEXT NOT NULL,
                    sent_at TEXT,
                    UNIQUE (operation_id, command_type)
                );
                CREATE TABLE IF NOT EXISTS orchestrator_errors (
                    error_key TEXT PRIMARY KEY,
                    message TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS operation_recoveries (
                    operation_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    marked_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS pallet3_poc_missions (
                    mission_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    phase TEXT NOT NULL,
                    failure_detail TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    unload_confirmed_at TEXT,
                    completed_at TEXT
                );
                CREATE UNIQUE INDEX IF NOT EXISTS pallet3_poc_active_robot
                    ON pallet3_poc_missions (robot_id)
                    WHERE phase NOT IN ('COMPLETED', 'FAILED');
                CREATE TABLE IF NOT EXISTS pallet3_operation_workflows (
                    operation_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    phase TEXT NOT NULL,
                    manual_pick_confirmed_at TEXT,
                    failure_detail TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    completed_at TEXT
                );
                """
            )

    def close(self) -> None:
        with self._lock:
            self._connection.close()

    def record_event(self, source: str, envelope: EventEnvelope) -> bool:
        with self._lock, self._connection:
            cursor = self._connection.execute(
                """
                INSERT OR IGNORE INTO orchestrator_inbox (
                    source, event_id, event_type, occurred_at, payload_json, received_at
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    source,
                    envelope.event_id,
                    envelope.event_type,
                    _format_time(envelope.occurred_at),
                    json.dumps(envelope.payload, sort_keys=True, separators=(",", ":")),
                    _format_time(_now()),
                ),
            )
        return cursor.rowcount == 1

    def enqueue_command(
        self,
        *,
        operation_id: str,
        robot_id: str,
        command_type: str,
        payload: dict,
    ) -> CommandRecord:
        timestamp = _format_time(_now())
        with self._lock, self._connection:
            existing = self._connection.execute(
                """
                SELECT * FROM command_outbox
                WHERE operation_id = ? AND command_type = ?
                """,
                (operation_id, command_type),
            ).fetchone()
            if existing is not None:
                return _command_from_row(existing)
            command_id = str(uuid4())
            self._connection.execute(
                """
                INSERT INTO command_outbox (
                    command_id, operation_id, robot_id, command_type, payload_json,
                    status, last_error, created_at, sent_at
                ) VALUES (?, ?, ?, ?, ?, 'PENDING', NULL, ?, NULL)
                """,
                (
                    command_id,
                    operation_id,
                    robot_id,
                    command_type,
                    json.dumps(payload, sort_keys=True, separators=(",", ":")),
                    timestamp,
                ),
            )
            row = self._connection.execute(
                "SELECT * FROM command_outbox WHERE command_id = ?", (command_id,)
            ).fetchone()
        return _command_from_row(row)

    def upsert_step(
        self,
        *,
        operation_id: str,
        robot_id: str,
        phase: str,
        source_zone_id: str,
        destination_zone_id: str,
        payload_type: str,
    ) -> OperationStep:
        timestamp = _format_time(_now())
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO orchestrator_steps (
                    operation_id, robot_id, phase, source_zone_id,
                    destination_zone_id, payload_type, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(operation_id) DO UPDATE SET
                    robot_id = excluded.robot_id,
                    phase = excluded.phase,
                    source_zone_id = excluded.source_zone_id,
                    destination_zone_id = excluded.destination_zone_id,
                    payload_type = excluded.payload_type,
                    updated_at = excluded.updated_at
                """,
                (
                    operation_id,
                    robot_id,
                    phase,
                    source_zone_id,
                    destination_zone_id,
                    payload_type,
                    timestamp,
                ),
            )
            row = self._connection.execute(
                "SELECT * FROM orchestrator_steps WHERE operation_id = ?", (operation_id,)
            ).fetchone()
        return _step_from_row(row)

    def get_step(self, operation_id: str) -> OperationStep | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM orchestrator_steps WHERE operation_id = ?", (operation_id,)
            ).fetchone()
        return _step_from_row(row) if row is not None else None

    def has_command(self, operation_id: str, command_type: str) -> bool:
        with self._lock:
            row = self._connection.execute(
                """
                SELECT 1 FROM command_outbox
                WHERE operation_id = ? AND command_type = ?
                """,
                (operation_id, command_type),
            ).fetchone()
        return row is not None

    def mark_recovery_required(self, operation_id: str, robot_id: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO operation_recoveries (operation_id, robot_id, marked_at)
                VALUES (?, ?, ?)
                ON CONFLICT(operation_id) DO UPDATE SET
                    robot_id = excluded.robot_id, marked_at = excluded.marked_at
                """,
                (operation_id, robot_id, _format_time(_now())),
            )

    def recovery_required(self, operation_id: str) -> bool:
        with self._lock:
            row = self._connection.execute(
                "SELECT 1 FROM operation_recoveries WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return row is not None

    def clear_recovery_required(self, operation_id: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                "DELETE FROM operation_recoveries WHERE operation_id = ?", (operation_id,)
            )

    def create_or_get_pallet3_workflow(
        self, operation_id: str, robot_id: str
    ) -> tuple[Pallet3OperationWorkflow, bool]:
        timestamp = _format_time(_now())
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if row is not None:
                return _pallet3_workflow_from_row(row), False
            self._connection.execute(
                """
                INSERT INTO pallet3_operation_workflows (
                    operation_id, robot_id, phase, manual_pick_confirmed_at,
                    failure_detail, created_at, updated_at, completed_at
                ) VALUES (?, ?, 'PICK_PENDING', NULL, NULL, ?, ?, NULL)
                """,
                (operation_id, robot_id, timestamp, timestamp),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return _pallet3_workflow_from_row(row), True

    def get_pallet3_workflow(self, operation_id: str) -> Pallet3OperationWorkflow | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return _pallet3_workflow_from_row(row) if row is not None else None

    def list_active_pallet3_workflows(self) -> list[Pallet3OperationWorkflow]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT * FROM pallet3_operation_workflows
                WHERE phase NOT IN ('COMPLETED', 'FAILED')
                ORDER BY created_at, operation_id
                """
            ).fetchall()
        return [_pallet3_workflow_from_row(row) for row in rows]

    def confirm_pallet3_manual_pick(
        self, operation_id: str
    ) -> tuple[Pallet3OperationWorkflow | None, bool]:
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if row is None:
                return None, False
            workflow = _pallet3_workflow_from_row(row)
            if workflow.manual_pick_confirmed_at is not None:
                return workflow, False
            timestamp = _format_time(_now())
            cursor = self._connection.execute(
                """
                UPDATE pallet3_operation_workflows
                SET manual_pick_confirmed_at = ?, updated_at = ?
                WHERE operation_id = ? AND manual_pick_confirmed_at IS NULL
                """,
                (timestamp, timestamp, operation_id),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return _pallet3_workflow_from_row(row), cursor.rowcount == 1

    def transition_pallet3_workflow(
        self, operation_id: str, expected_phase: str, phase: str
    ) -> tuple[Pallet3OperationWorkflow | None, bool]:
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if row is None:
                return None, False
            workflow = _pallet3_workflow_from_row(row)
            if workflow.phase != expected_phase or workflow.phase in {"COMPLETED", "FAILED"}:
                return workflow, False
            timestamp = _format_time(_now())
            completed_at = timestamp if phase == "COMPLETED" else None
            cursor = self._connection.execute(
                """
                UPDATE pallet3_operation_workflows
                SET phase = ?, updated_at = ?, completed_at = COALESCE(?, completed_at)
                WHERE operation_id = ? AND phase = ?
                """,
                (phase, timestamp, completed_at, operation_id, expected_phase),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return _pallet3_workflow_from_row(row), cursor.rowcount == 1

    def fail_pallet3_workflow(
        self, operation_id: str, detail: str
    ) -> Pallet3OperationWorkflow | None:
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
            if row is None:
                return None
            workflow = _pallet3_workflow_from_row(row)
            if workflow.phase in {"COMPLETED", "FAILED"}:
                return workflow
            self._connection.execute(
                """
                UPDATE pallet3_operation_workflows
                SET phase = 'FAILED', failure_detail = ?, updated_at = ?
                WHERE operation_id = ?
                """,
                (detail[:1000], _format_time(_now()), operation_id),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_operation_workflows WHERE operation_id = ?",
                (operation_id,),
            ).fetchone()
        return _pallet3_workflow_from_row(row)

    def create_or_get_pallet3_mission(
        self,
        robot_id: str,
        mission_id: str | None = None,
        *,
        replace_active: bool = False,
    ) -> tuple[Pallet3Mission, bool]:
        timestamp = _format_time(_now())
        with self._lock, self._connection:
            active = self._connection.execute(
                """
                SELECT * FROM pallet3_poc_missions
                WHERE robot_id = ? AND phase NOT IN ('COMPLETED', 'FAILED')
                """,
                (robot_id,),
            ).fetchone()
            if active is not None:
                if not replace_active:
                    return _pallet3_mission_from_row(active), False
                self._connection.execute(
                    """
                    UPDATE pallet3_poc_missions
                    SET phase = 'FAILED', failure_detail = ?, updated_at = ?
                    WHERE mission_id = ?
                    """,
                    (
                        "SUPERSEDED_BY_NEW_POC_REQUEST",
                        timestamp,
                        active["mission_id"],
                    ),
                )
            mission_id = mission_id or str(uuid4())
            self._connection.execute(
                """
                INSERT INTO pallet3_poc_missions (
                    mission_id, robot_id, phase, failure_detail, created_at, updated_at,
                    unload_confirmed_at, completed_at
                ) VALUES (?, ?, 'OUTBOUND_SENT', NULL, ?, ?, NULL, NULL)
                """,
                (mission_id, robot_id, timestamp, timestamp),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
        return _pallet3_mission_from_row(row), True

    def get_pallet3_mission(self, mission_id: str) -> Pallet3Mission | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
        return _pallet3_mission_from_row(row) if row is not None else None

    def list_active_pallet3_missions(self) -> list[Pallet3Mission]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT * FROM pallet3_poc_missions
                WHERE phase NOT IN ('COMPLETED', 'FAILED')
                ORDER BY created_at
                """
            ).fetchall()
        return [_pallet3_mission_from_row(row) for row in rows]

    def confirm_pallet3_unload(
        self, mission_id: str
    ) -> tuple[Pallet3Mission | None, bool]:
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
            if row is None:
                return None, False
            mission = _pallet3_mission_from_row(row)
            if mission.phase != "AWAIT_UNLOAD_CONFIRMATION":
                return mission, False
            timestamp = _format_time(_now())
            self._connection.execute(
                """
                UPDATE pallet3_poc_missions
                SET phase = 'FORK_DOWN_SENT', unload_confirmed_at = ?, updated_at = ?
                WHERE mission_id = ? AND phase = 'AWAIT_UNLOAD_CONFIRMATION'
                """,
                (timestamp, timestamp, mission_id),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
        return _pallet3_mission_from_row(row), True

    def transition_pallet3_mission(
        self, mission_id: str, expected_phase: str, phase: str
    ) -> tuple[Pallet3Mission | None, bool]:
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
            if row is None:
                return None, False
            mission = _pallet3_mission_from_row(row)
            if mission.phase != expected_phase:
                return mission, False
            timestamp = _format_time(_now())
            completed_at = timestamp if phase == "COMPLETED" else None
            self._connection.execute(
                """
                UPDATE pallet3_poc_missions
                SET phase = ?, updated_at = ?, completed_at = COALESCE(?, completed_at)
                WHERE mission_id = ? AND phase = ?
                """,
                (phase, timestamp, completed_at, mission_id, expected_phase),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
        return _pallet3_mission_from_row(row), True

    def fail_pallet3_mission(self, mission_id: str, detail: str) -> Pallet3Mission | None:
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
            if row is None:
                return None
            mission = _pallet3_mission_from_row(row)
            if mission.phase in {"COMPLETED", "FAILED"}:
                return mission
            timestamp = _format_time(_now())
            self._connection.execute(
                """
                UPDATE pallet3_poc_missions
                SET phase = 'FAILED', failure_detail = ?, updated_at = ?
                WHERE mission_id = ?
                """,
                (detail[:1000], timestamp, mission_id),
            )
            row = self._connection.execute(
                "SELECT * FROM pallet3_poc_missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
        return _pallet3_mission_from_row(row)

    def get_command(self, command_id: str) -> CommandRecord | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM command_outbox WHERE command_id = ?", (command_id,)
            ).fetchone()
        return _command_from_row(row) if row is not None else None

    def list_pending_commands(self) -> list[CommandRecord]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT * FROM command_outbox WHERE status = 'PENDING' ORDER BY created_at"
            ).fetchall()
        return [_command_from_row(row) for row in rows]

    def mark_command_sent(self, command_id: str) -> None:
        self._set_command_status(command_id, "SENT", None, sent_at=_now())

    def mark_command_delivery_unknown(self, command_id: str, error: str) -> None:
        self._set_command_status(command_id, "DELIVERY_UNKNOWN", error)

    def mark_command_failed(self, command_id: str, error: str) -> None:
        self._set_command_status(command_id, "FAILED", error)

    def _set_command_status(
        self,
        command_id: str,
        status: str,
        error: str | None,
        *,
        sent_at: datetime | None = None,
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                UPDATE command_outbox
                SET status = ?, last_error = ?, sent_at = COALESCE(?, sent_at)
                WHERE command_id = ?
                """,
                (status, error, _format_time(sent_at) if sent_at else None, command_id),
            )

    def set_error(self, error_key: str, message: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO orchestrator_errors (error_key, message, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(error_key) DO UPDATE SET
                    message = excluded.message, updated_at = excluded.updated_at
                """,
                (error_key, message, _format_time(_now())),
            )

    def clear_error(self, error_key: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                "DELETE FROM orchestrator_errors WHERE error_key = ?", (error_key,)
            )

    def status(self) -> dict:
        with self._lock:
            inbox_event_count = self._connection.execute(
                "SELECT COUNT(*) FROM orchestrator_inbox"
            ).fetchone()[0]
            step_count = self._connection.execute(
                "SELECT COUNT(*) FROM orchestrator_steps"
            ).fetchone()[0]
            command_rows = self._connection.execute(
                "SELECT status, COUNT(*) AS count FROM command_outbox GROUP BY status"
            ).fetchall()
            error_rows = self._connection.execute(
                "SELECT error_key, message, updated_at FROM orchestrator_errors ORDER BY error_key"
            ).fetchall()
            poc_rows = self._connection.execute(
                "SELECT phase, COUNT(*) AS count FROM pallet3_poc_missions GROUP BY phase"
            ).fetchall()
        return {
            "inbox_event_count": inbox_event_count,
            "step_count": step_count,
            "command_counts": {row["status"]: row["count"] for row in command_rows},
            "errors": [dict(row) for row in error_rows],
            "pallet3_mission_counts": {row["phase"]: row["count"] for row in poc_rows},
        }


def _command_from_row(row: sqlite3.Row) -> CommandRecord:
    return CommandRecord(
        command_id=row["command_id"],
        operation_id=row["operation_id"],
        robot_id=row["robot_id"],
        command_type=row["command_type"],
        payload=json.loads(row["payload_json"]),
        status=row["status"],
        last_error=row["last_error"],
        created_at=row["created_at"],
        sent_at=row["sent_at"],
    )


def _step_from_row(row: sqlite3.Row) -> OperationStep:
    return OperationStep(
        operation_id=row["operation_id"],
        robot_id=row["robot_id"],
        phase=row["phase"],
        source_zone_id=row["source_zone_id"],
        destination_zone_id=row["destination_zone_id"],
        payload_type=row["payload_type"],
        updated_at=row["updated_at"],
    )


def _pallet3_mission_from_row(row: sqlite3.Row) -> Pallet3Mission:
    return Pallet3Mission(
        mission_id=row["mission_id"],
        robot_id=row["robot_id"],
        phase=row["phase"],
        failure_detail=row["failure_detail"],
        created_at=row["created_at"],
        updated_at=row["updated_at"],
        unload_confirmed_at=row["unload_confirmed_at"],
        completed_at=row["completed_at"],
    )


def _pallet3_workflow_from_row(row: sqlite3.Row) -> Pallet3OperationWorkflow:
    return Pallet3OperationWorkflow(
        operation_id=row["operation_id"],
        robot_id=row["robot_id"],
        phase=row["phase"],
        manual_pick_confirmed_at=row["manual_pick_confirmed_at"],
        failure_detail=row["failure_detail"],
        created_at=row["created_at"],
        updated_at=row["updated_at"],
        completed_at=row["completed_at"],
    )


def _now() -> datetime:
    return datetime.now(UTC)


def _format_time(value: datetime) -> str:
    return value.astimezone(UTC).isoformat().replace("+00:00", "Z")
