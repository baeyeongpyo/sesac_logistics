from __future__ import annotations

from contextlib import asynccontextmanager
from dataclasses import asdict
import os

from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field

from .inventory import (
    ConflictError,
    InventoryEvent,
    InventoryStore,
    NotFoundError,
    Operation,
    PayloadType,
    RobotPalletState,
    Stock,
    Zone,
)


class RequestModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class ZoneRequest(RequestModel):
    name: str = Field(min_length=1)
    map_name: str = Field(min_length=1)
    nav_x: float
    nav_y: float
    nav_yaw: float
    capacity: int | None = Field(default=None, ge=0)
    enabled: bool = True


class StockRequest(RequestModel):
    quantity: int = Field(ge=0)


class OperationRequest(RequestModel):
    operation_id: str = Field(min_length=1)
    payload_type: PayloadType
    source_zone_id: str = Field(min_length=1)
    destination_zone_id: str = Field(min_length=1)
    priority: int = 0


class AssignmentRequest(RequestModel):
    robot_id: str = Field(min_length=1)


class CompletionRequest(RequestModel):
    robot_id: str = Field(min_length=1)
    idempotency_key: str = Field(min_length=1)
    occurred_at: str | None = None


def _response(value: Zone | Stock | Operation | InventoryEvent | RobotPalletState) -> dict:
    response = asdict(value)
    if isinstance(value, Stock):
        response["available_quantity"] = value.available_quantity
    return response


def _store(request: Request) -> InventoryStore:
    return request.app.state.store


def create_app(database_path: str) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        store = InventoryStore(database_path)
        store.initialize()
        app.state.store = store
        yield

    app = FastAPI(title="Pallet Inventory API", version="1.0.0", lifespan=lifespan)

    @app.exception_handler(ConflictError)
    def conflict_error_handler(_: Request, error: ConflictError):
        return JSONResponse(
            status_code=status.HTTP_409_CONFLICT, content={"detail": str(error)}
        )

    @app.exception_handler(NotFoundError)
    def not_found_error_handler(_: Request, error: NotFoundError):
        return JSONResponse(
            status_code=status.HTTP_404_NOT_FOUND, content={"detail": str(error)}
        )

    @app.get("/healthz")
    def healthz() -> dict[str, str]:
        return {"status": "ok"}

    @app.put("/api/v1/zones/{zone_id}")
    def upsert_zone(zone_id: str, body: ZoneRequest, request: Request) -> dict:
        zone = _store(request).upsert_zone(
            Zone(
                zone_id,
                body.name,
                body.map_name,
                body.nav_x,
                body.nav_y,
                body.nav_yaw,
                body.capacity,
                body.enabled,
            )
        )
        return _response(zone)

    @app.get("/api/v1/zones")
    def list_zones(request: Request) -> list[dict]:
        return [_response(zone) for zone in _store(request).list_zones()]

    @app.put("/api/v1/stocks/{zone_id}/{payload_type}")
    def set_stock(
        zone_id: str, payload_type: PayloadType, body: StockRequest, request: Request
    ) -> dict:
        return _response(_store(request).set_stock(zone_id, payload_type, body.quantity))

    @app.get("/api/v1/stocks")
    def list_stocks(request: Request) -> list[dict]:
        return [_response(stock) for stock in _store(request).list_stocks()]

    @app.post("/api/v1/operations", status_code=status.HTTP_201_CREATED)
    def create_operation(body: OperationRequest, request: Request) -> dict:
        operation = _store(request).create_operation(
            body.operation_id,
            body.payload_type,
            body.source_zone_id,
            body.destination_zone_id,
            body.priority,
        )
        return _response(operation)

    @app.post("/api/v1/operations/{operation_id}/assignments")
    def assign_operation(
        operation_id: str, body: AssignmentRequest, request: Request
    ) -> dict:
        return _response(_store(request).assign_operation(operation_id, body.robot_id))

    @app.post("/api/v1/operations/{operation_id}/pick-completions")
    def complete_pick(
        operation_id: str, body: CompletionRequest, request: Request
    ) -> dict:
        event = _store(request).complete_pick(
            operation_id, body.robot_id, body.idempotency_key, body.occurred_at
        )
        return _response(event)

    @app.post("/api/v1/operations/{operation_id}/place-completions")
    def complete_place(
        operation_id: str, body: CompletionRequest, request: Request
    ) -> dict:
        event = _store(request).complete_place(
            operation_id, body.robot_id, body.idempotency_key, body.occurred_at
        )
        return _response(event)

    @app.get("/api/v1/robots/{robot_id}/next-instruction")
    def next_instruction(robot_id: str, request: Request) -> dict | None:
        instruction = _store(request).next_instruction(robot_id)
        return asdict(instruction) if instruction is not None else None

    @app.get("/api/v1/robots/{robot_id}/pallet-state")
    def robot_pallet_state(robot_id: str, request: Request) -> dict:
        return _response(_store(request).get_robot_pallet_state(robot_id))

    return app


app = create_app(os.environ.get("INVENTORY_DB_PATH", "/data/inventory.db"))
