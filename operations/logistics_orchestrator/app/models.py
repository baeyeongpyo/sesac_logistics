from __future__ import annotations

from datetime import UTC, datetime
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, field_validator


class EventEnvelope(BaseModel):
    model_config = ConfigDict(extra="forbid")

    event_id: str = Field(min_length=1)
    event_type: str = Field(min_length=1)
    occurred_at: datetime
    payload: dict[str, Any]

    @field_validator("occurred_at")
    @classmethod
    def occurred_at_must_be_utc(cls, value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() != UTC.utcoffset(value):
            raise ValueError("occurred_at must be UTC")
        return value.astimezone(UTC)


class CommandRecord(BaseModel):
    model_config = ConfigDict(frozen=True)

    command_id: str
    operation_id: str
    robot_id: str
    command_type: str
    payload: dict[str, Any]
    status: str
    last_error: str | None
    created_at: datetime
    sent_at: datetime | None


class OperationStep(BaseModel):
    model_config = ConfigDict(frozen=True)

    operation_id: str
    robot_id: str
    phase: str
    source_zone_id: str
    destination_zone_id: str
    payload_type: str
    updated_at: datetime


class Pallet3Mission(BaseModel):
    model_config = ConfigDict(frozen=True)

    mission_id: str
    robot_id: str
    phase: str
    failure_detail: str | None
    created_at: datetime
    updated_at: datetime
    unload_confirmed_at: datetime | None
    completed_at: datetime | None


class Pallet3OperationWorkflow(BaseModel):
    model_config = ConfigDict(frozen=True)

    operation_id: str
    robot_id: str
    phase: str
    manual_pick_confirmed_at: datetime | None
    failure_detail: str | None
    created_at: datetime
    updated_at: datetime
    completed_at: datetime | None


class Pallet3MissionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    robot_id: str = Field(min_length=1)
    bypass_pick: bool = False
    new_mission: bool = False


class Pallet3ManualPickRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    operator_confirmed: bool


class Pallet3ForceCompletionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    operator_confirmed: bool
