from contextlib import asynccontextmanager
import os
from pathlib import Path
from typing import Any

from fastapi import Body, FastAPI, HTTPException, Query, Request, Response, status
from fastapi.responses import JSONResponse

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


def _store(request: Request) -> VehicleStateStore:
    return request.app.state.store


def _registry(request: Request) -> VehicleRegistry:
    return request.app.state.vehicle_registry


def _bridge_client(request: Request) -> BridgeCommandGateway:
    return request.app.state.bridge_client


def create_app(
    database_path: str,
    *,
    orchestrator_event_url: str | None = None,
    retry_interval_sec: float = 2.0,
    vehicle_registry_path: str | Path | None = None,
    bridge_client: BridgeCommandGateway | None = None,
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
        return _store(request).record_state(robot_id, report)

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
)
