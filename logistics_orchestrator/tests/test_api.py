from pathlib import Path
import tempfile
import unittest

from fastapi.testclient import TestClient

from logistics_orchestrator.app.main import create_app


class OrchestratorApiTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        database_path = Path(self.temporary_directory.name) / "orchestrator.db"
        self.client = TestClient(create_app(str(database_path)))
        self.client.__enter__()

    def tearDown(self) -> None:
        self.client.__exit__(None, None, None)
        self.temporary_directory.cleanup()

    def test_healthz_reports_service_ready(self) -> None:
        # This catches an unhealthy Docker container being reported as ready.
        response = self.client.get("/healthz")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "ok"})

    def test_fleet_event_returns_no_content_only_after_durable_write(self) -> None:
        # This catches acknowledging a Fleet event before its inbox row is durable.
        event = {
            "event_id": "fleet-event-1",
            "event_type": "fleet.vehicle_reported",
            "occurred_at": "2026-08-31T12:00:00Z",
            "payload": {"robot_id": "robot_1", "state": "WAIT"},
        }

        response = self.client.post("/api/v1/events/fleet", json=event)
        duplicate = self.client.post("/api/v1/events/fleet", json=event)
        status = self.client.get("/api/v1/status")

        self.assertEqual(response.status_code, 204)
        self.assertEqual(duplicate.status_code, 204)
        self.assertEqual(status.json()["inbox_event_count"], 1)

    def test_bootstrap_endpoint_is_not_exposed(self) -> None:
        # This catches moving Inventory environment ownership into the Orchestrator.
        response = self.client.post("/api/v1/bootstrap")

        self.assertEqual(response.status_code, 404)

    def test_durable_event_triggers_reconciliation_once_and_manual_reconcile_is_available(self) -> None:
        # This catches either acknowledging before dispatch or running an idempotent replay twice.
        calls: list[str] = []

        class RecordingService:
            def handle_recorded_event(self, source, event) -> None:
                calls.append(f"event:{source}:{event.event_id}")

            def reconcile(self) -> None:
                calls.append("reconcile")

        app = create_app(
            str(Path(self.temporary_directory.name) / "service.db"),
            service_factory=lambda _: RecordingService(),
        )
        event = {
            "event_id": "inventory-event-1",
            "event_type": "inventory.stock_changed",
            "occurred_at": "2026-08-31T12:00:00Z",
            "payload": {"zone_id": "docker"},
        }

        with TestClient(app) as client:
            self.assertEqual(client.post("/api/v1/events/inventory", json=event).status_code, 204)
            self.assertEqual(client.post("/api/v1/events/inventory", json=event).status_code, 204)
            self.assertEqual(client.post("/api/v1/reconcile").status_code, 200)

        self.assertEqual(calls, ["event:inventory:inventory-event-1", "reconcile"])


if __name__ == "__main__":
    unittest.main()
