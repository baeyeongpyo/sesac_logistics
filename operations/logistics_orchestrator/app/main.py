from __future__ import annotations

from contextlib import asynccontextmanager
import os
from typing import Any, Callable

from fastapi import FastAPI, HTTPException, Request, Response, status

from .models import EventEnvelope, Pallet3Mission, Pallet3MissionRequest
from .store import OrchestratorStore


def _store(request: Request) -> OrchestratorStore:
    return request.app.state.store


def create_app(
    database_path: str,
    *,
    service_factory: Callable[[OrchestratorStore], Any] | None = None,
) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        store = OrchestratorStore(database_path)
        app.state.store = store
        app.state.service = service_factory(store) if service_factory is not None else None
        yield
        store.close()

    app = FastAPI(title="Logistics Orchestrator API", version="1.0.0", lifespan=lifespan)

    @app.get("/healthz")
    def healthz() -> dict[str, str]:
        return {"status": "ok"}

    @app.get("/api/v1/status")
    def service_status(request: Request) -> dict:
        return _store(request).status()

    @app.post("/api/v1/reconcile")
    def reconcile(request: Request) -> dict[str, bool]:
        service = request.app.state.service
        if service is not None:
            service.reconcile()
        return {"accepted": True}

    @app.post(
        "/api/v1/poc/pallet-3-missions",
        response_model=Pallet3Mission,
        status_code=status.HTTP_201_CREATED,
    )
    def start_pallet3_mission(
        body: Pallet3MissionRequest, request: Request
    ) -> Pallet3Mission:
        service = _require_service(request)
        try:
            return service.start_pallet3_mission(
                body.robot_id, bypass_pick=body.bypass_pick
            )
        except KeyError as error:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail=str(error)
            ) from error
        except ValueError as error:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT, detail=str(error)
            ) from error

    @app.get(
        "/api/v1/poc/pallet-3-missions/{mission_id}",
        response_model=Pallet3Mission,
    )
    def get_pallet3_mission(mission_id: str, request: Request) -> Pallet3Mission:
        mission = _store(request).get_pallet3_mission(mission_id)
        if mission is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"unknown pallet 3 mission: {mission_id}",
            )
        return mission

    @app.post(
        "/api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation",
        response_model=Pallet3Mission,
    )
    def confirm_pallet3_unload(mission_id: str, request: Request) -> Pallet3Mission:
        service = _require_service(request)
        try:
            return service.confirm_pallet3_unload(mission_id)
        except KeyError as error:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail=str(error)
            ) from error
        except ValueError as error:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT, detail=str(error)
            ) from error

    @app.post(
        "/api/v1/events/fleet",
        status_code=status.HTTP_204_NO_CONTENT,
        response_class=Response,
    )
    def receive_fleet_event(event: EventEnvelope, request: Request) -> Response:
        _record_event("fleet", event, request)
        return Response(status_code=status.HTTP_204_NO_CONTENT)

    @app.post(
        "/api/v1/events/inventory",
        status_code=status.HTTP_204_NO_CONTENT,
        response_class=Response,
    )
    def receive_inventory_event(event: EventEnvelope, request: Request) -> Response:
        _record_event("inventory", event, request)
        return Response(status_code=status.HTTP_204_NO_CONTENT)

    return app


def _record_event(source: str, event: EventEnvelope, request: Request) -> None:
    if _store(request).record_event(source, event):
        service = request.app.state.service
        if service is not None:
            service.handle_recorded_event(source, event)


def _require_service(request: Request) -> Any:
    service = request.app.state.service
    if service is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="orchestrator service is unavailable",
        )
    return service


def _runtime_service_factory(store: OrchestratorStore) -> Any:
    from .clients import HttpFleetManagerClient, HttpInventoryClient
    from .service import OrchestratorService

    fleet_manager = HttpFleetManagerClient(
        os.getenv("FLEET_MANAGER_URL", "http://host.docker.internal:8090")
    )
    return OrchestratorService(
        store,
        HttpInventoryClient(os.getenv("INVENTORY_URL", "http://host.docker.internal:8081")),
        fleet_manager,
    )


app = create_app(
    os.getenv("ORCHESTRATOR_DB_PATH", "/data/orchestrator.db"),
    service_factory=_runtime_service_factory,
)
