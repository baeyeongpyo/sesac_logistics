from __future__ import annotations

from contextlib import asynccontextmanager
import os
from typing import Any, Callable

from fastapi import FastAPI, Request, Response, status

from .models import EventEnvelope
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


def _runtime_service_factory(store: OrchestratorStore) -> Any:
    from .clients import HttpFleetBridgeClient, HttpFleetManagerClient, HttpInventoryClient
    from .service import OrchestratorService

    return OrchestratorService(
        store,
        HttpInventoryClient(os.getenv("INVENTORY_URL", "http://host.docker.internal:8081")),
        HttpFleetManagerClient(
            os.getenv("FLEET_MANAGER_URL", "http://host.docker.internal:8090")
        ),
        HttpFleetBridgeClient(
            os.getenv("FLEET_BRIDGE_URL", "http://host.docker.internal:8080")
        ),
    )


app = create_app(
    os.getenv("ORCHESTRATOR_DB_PATH", "/data/orchestrator.db"),
    service_factory=_runtime_service_factory,
)
