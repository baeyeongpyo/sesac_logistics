from contextlib import asynccontextmanager
import os

from fastapi import FastAPI, HTTPException, Query, Request, status

from .fleet import (
    FleetEventDispatcher,
    VehicleSnapshot,
    VehicleStateLog,
    VehicleStateReport,
    VehicleStateStore,
)


def _store(request: Request) -> VehicleStateStore:
    return request.app.state.store


def create_app(
    database_path: str,
    *,
    orchestrator_event_url: str | None = None,
    retry_interval_sec: float = 2.0,
) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        store = VehicleStateStore(database_path)
        app.state.store = store
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

    return app


app = create_app(
    os.getenv("FLEET_MANAGER_DB_PATH", "/data/fleet_manager.db"),
    orchestrator_event_url=os.getenv("ORCHESTRATOR_EVENT_URL", ""),
    retry_interval_sec=float(os.getenv("EVENT_RETRY_INTERVAL_SEC", "2")),
)
