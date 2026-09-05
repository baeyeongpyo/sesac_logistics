from contextlib import asynccontextmanager
import os
from pathlib import Path
from typing import Any, Literal

from fastapi import Body, FastAPI, HTTPException, Query, Request, Response, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field

from .commands import (
    BridgeCommandClient,
    BridgeCommandGateway,
    BridgeUnavailableError,
    UnknownVehicleError,
    UnsupportedCapabilityError,
    VehicleRegistry,
    load_vehicle_registry,
)

from .fleet import (
    FleetEventDispatcher,
    VehicleSnapshot,
    VehicleStateLog,
    VehicleStateReport,
    VehicleStateStore,
)
from .pallet3 import (
    InventoryClient,
    InventoryGateway,
    InventoryRequestError,
    Pallet3MissionService,
)


class Pallet3StartRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    pick_mode: Literal["auto_dock", "manual"]


class Pallet3EventRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    event_type: Literal["PICK_COMPLETED", "PLACE_READY", "RETURN_COMPLETED"]
    idempotency_key: str = Field(min_length=1)


def _store(request: Request) -> VehicleStateStore:
    return request.app.state.store


def _registry(request: Request) -> VehicleRegistry:
    return request.app.state.vehicle_registry


def _bridge_client(request: Request) -> BridgeCommandGateway:
    return request.app.state.bridge_client


def _pallet3_service(request: Request) -> Pallet3MissionService:
    return request.app.state.pallet3_service


def create_app(
    database_path: str,
    *,
    orchestrator_event_url: str | None = None,
    retry_interval_sec: float = 2.0,
    vehicle_registry_path: str | Path | None = None,
    bridge_client: BridgeCommandGateway | None = None,
    inventory_client: InventoryGateway | None = None,
    inventory_url: str = "http://127.0.0.1:8081",
    fleet_manager_url: str = "http://192.168.100.27:8090",
) -> FastAPI:
    registry_path = vehicle_registry_path or (
        Path(__file__).resolve().parents[1] / "config" / "vehicles.yaml"
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        store = VehicleStateStore(database_path)
        command_client = bridge_client or BridgeCommandClient()
        app.state.store = store
        app.state.vehicle_registry = load_vehicle_registry(registry_path)
        app.state.bridge_client = command_client
        app.state.pallet3_service = Pallet3MissionService(
            store,
            inventory_client or InventoryClient(inventory_url),
            app.state.vehicle_registry,
            command_client,
            fleet_manager_url=fleet_manager_url,
        )
        destination_url = (orchestrator_event_url or "").strip()
        dispatcher = (
            FleetEventDispatcher(
                store, destination_url, retry_interval_sec=retry_interval_sec
            )
            if destination_url
            else None
        )
        app.state.event_dispatcher = dispatcher
        if dispatcher is not None:
            dispatcher.start()
        yield
        if dispatcher is not None:
            dispatcher.stop()
        close = getattr(command_client, "close", None)
        if callable(close):
            close()
        store.close()

    app = FastAPI(title="Fleet Manager API", version="1.0.0", lifespan=lifespan)

    @app.get("/healthz")
    def healthz() -> dict[str, str]:
        return {"status": "ok"}

    @app.post("/api/v1/vehicles/{robot_id}/state", response_model=VehicleSnapshot)
    def record_vehicle_state(
        robot_id: str, report: VehicleStateReport, request: Request
    ) -> VehicleSnapshot:
        snapshot = _store(request).record_state(robot_id, report)
        if report.operation_id is not None and report.detail in {
            "AUTO_DOCK_PICK_COMPLETED",
            "FORK_UP_COMPLETE",
            "FORK_DOWN_COMPLETE",
        }:
            _pallet3_service(request).record_vehicle_reply(
                robot_id, report.operation_id, report.detail
            )
        return snapshot

    @app.get("/api/v1/vehicles", response_model=list[VehicleSnapshot])
    def list_vehicles(request: Request) -> list[VehicleSnapshot]:
        return _store(request).list_vehicles()

    @app.get(
        "/api/v1/vehicles/{robot_id}/logs", response_model=list[VehicleStateLog]
    )
    def list_vehicle_logs(
        robot_id: str,
        request: Request,
        limit: int = Query(default=100, ge=1, le=500),
        before_id: int | None = Query(default=None, ge=1),
    ) -> list[VehicleStateLog]:
        if _store(request).get_vehicle(robot_id) is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"unknown vehicle: {robot_id}",
            )
        return _store(request).list_logs(
            robot_id, limit=limit, before_id=before_id
        )

    @app.get("/api/v1/vehicles/{robot_id}", response_model=VehicleSnapshot)
    def get_vehicle(robot_id: str, request: Request) -> VehicleSnapshot:
        snapshot = _store(request).get_vehicle(robot_id)
        if snapshot is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"unknown vehicle: {robot_id}",
            )
        return snapshot

    @app.post("/api/v1/vehicles/{robot_id}/commands/navigation/goals")
    def navigate_vehicle(
        robot_id: str,
        payload: dict[str, Any] = Body(...),
        request: Request = None,
    ) -> Response:
        return _relay_command(
            request,
            robot_id,
            "navigate",
            f"/api/v1/vehicle-command/{robot_id}/navigation/goals",
            payload,
        )

    @app.post("/api/v1/vehicles/{robot_id}/commands/navigation/waypoints")
    def navigate_vehicle_waypoints(
        robot_id: str,
        payload: dict[str, Any] = Body(...),
        request: Request = None,
    ) -> Response:
        return _relay_command(
            request,
            robot_id,
            "navigate",
            f"/api/v1/vehicle-command/{robot_id}/navigation/waypoints",
            payload,
        )

    @app.post("/api/v1/vehicles/{robot_id}/commands/fork/down")
    def lower_vehicle_fork(
        robot_id: str,
        payload: dict[str, Any] = Body(...),
        request: Request = None,
    ) -> Response:
        return _relay_command(
            request,
            robot_id,
            "fork",
            f"/api/v1/vehicle-command/{robot_id}/fork/down",
            payload,
        )

    @app.post("/api/v1/vehicles/{robot_id}/commands/cmd-vel")
    def command_vehicle_velocity(
        robot_id: str,
        payload: dict[str, Any] = Body(...),
        request: Request = None,
    ) -> Response:
        return _relay_command(
            request,
            robot_id,
            "manual_drive",
            f"/api/v1/vehicle-command/{robot_id}/cmd-vel",
            payload,
        )

    @app.post("/api/v1/vehicles/{robot_id}/commands/auto-dock")
    def auto_dock_vehicle(
        robot_id: str,
        payload: dict[str, Any] = Body(...),
        request: Request = None,
    ) -> Response:
        return _relay_command(
            request,
            robot_id,
            "auto_dock",
            f"/api/v1/vehicle-command/{robot_id}/auto-dock",
            payload,
        )

    @app.post("/api/v1/vehicles/{robot_id}/commands/stop")
    def stop_vehicle(robot_id: str, request: Request) -> Response:
        return _relay_command(
            request,
            robot_id,
            "stop",
            f"/api/v1/vehicle-command/{robot_id}/stop",
            {},
        )

    @app.post("/api/v1/vehicles/{robot_id}/missions/pallet3", status_code=202)
    def start_pallet3_mission(
        robot_id: str, body: Pallet3StartRequest, request: Request
    ) -> dict[str, Any]:
        try:
            return _pallet3_service(request).start(robot_id, body.pick_mode)
        except UnknownVehicleError as error:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"unknown vehicle: {robot_id}",
            ) from error
        except UnsupportedCapabilityError as error:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT, detail=str(error)
            ) from error
        except InventoryRequestError as error:
            raise HTTPException(
                status_code=(
                    status.HTTP_409_CONFLICT
                    if error.status_code == status.HTTP_409_CONFLICT
                    else status.HTTP_503_SERVICE_UNAVAILABLE
                ),
                detail=str(error),
            ) from error
        except ValueError as error:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT, detail=str(error)
            ) from error
        except RuntimeError as error:
            raise HTTPException(
                status_code=status.HTTP_502_BAD_GATEWAY, detail=str(error)
            ) from error

    @app.post("/api/v1/vehicles/{robot_id}/missions/pallet3/{operation_id}/events")
    def record_pallet3_event(
        robot_id: str,
        operation_id: str,
        body: Pallet3EventRequest,
        request: Request,
    ) -> dict[str, Any]:
        try:
            return _pallet3_service(request).record_event(
                robot_id, operation_id, body.event_type, body.idempotency_key
            )
        except KeyError as error:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail=str(error)
            ) from error
        except InventoryRequestError as error:
            raise HTTPException(
                status_code=(
                    status.HTTP_409_CONFLICT
                    if error.status_code == status.HTTP_409_CONFLICT
                    else status.HTTP_503_SERVICE_UNAVAILABLE
                ),
                detail=str(error),
            ) from error
        except ValueError as error:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT, detail=str(error)
            ) from error

    return app


def _relay_command(
    request: Request,
    robot_id: str,
    capability: str,
    bridge_path: str,
    payload: dict[str, Any],
) -> Response:
    try:
        vehicle = _registry(request).require(robot_id, capability)
    except UnknownVehicleError as error:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"unknown vehicle: {robot_id}",
        ) from error
    except UnsupportedCapabilityError as error:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(error),
        ) from error
    try:
        relay = _bridge_client(request).relay(vehicle, bridge_path, payload)
    except BridgeUnavailableError as error:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(error),
        ) from error
    if relay.body is None:
        return Response(status_code=relay.status_code)
    return JSONResponse(status_code=relay.status_code, content=relay.body)


app = create_app(
    os.getenv("FLEET_MANAGER_DB_PATH", "/data/fleet_manager.db"),
    orchestrator_event_url=os.getenv("ORCHESTRATOR_EVENT_URL", ""),
    retry_interval_sec=float(os.getenv("EVENT_RETRY_INTERVAL_SEC", "2")),
    vehicle_registry_path=os.getenv("VEHICLE_REGISTRY_PATH") or None,
    inventory_url=os.getenv("INVENTORY_URL", "http://127.0.0.1:8081"),
    fleet_manager_url=os.getenv(
        "FLEET_MANAGER_PUBLIC_URL", "http://192.168.100.27:8090"
    ),
)
