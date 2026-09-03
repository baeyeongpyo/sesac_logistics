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

    def test_pallet3_poc_routes_create_read_and_confirm_the_same_mission(self) -> None:
        calls: list[tuple[str, str]] = []
        start_bypass_values: list[bool] = []
        start_new_mission_values: list[bool] = []

        class RecordingPocService:
            def __init__(self, store) -> None:
                self.store = store

            def start_pallet3_mission(
                self,
                robot_id: str,
                *,
                bypass_pick: bool = False,
                new_mission: bool = False,
            ):
                calls.append(("start", robot_id))
                start_bypass_values.append(bypass_pick)
                start_new_mission_values.append(new_mission)
                mission, _created = self.store.create_or_get_pallet3_mission(robot_id)
                return mission

            def confirm_pallet3_unload(self, mission_id: str):
                calls.append(("confirm", mission_id))
                mission = self.store.get_pallet3_mission(mission_id)
                if mission is None:
                    raise KeyError(mission_id)
                return mission

        app = create_app(
            str(Path(self.temporary_directory.name) / "poc.db"),
            service_factory=lambda store: RecordingPocService(store),
        )
        with TestClient(app) as client:
            created = client.post(
                "/api/v1/poc/pallet-3-missions",
                json={"robot_id": "robot_1"},
            )
            mission_id = created.json()["mission_id"]
            retrieved = client.get(f"/api/v1/poc/pallet-3-missions/{mission_id}")
            confirmation = client.post(
                f"/api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation"
            )

        self.assertEqual(created.status_code, 201)
        self.assertEqual(retrieved.status_code, 200)
        self.assertEqual(confirmation.status_code, 200)
        self.assertEqual(retrieved.json()["mission_id"], mission_id)
        self.assertEqual(calls, [("start", "robot_1"), ("confirm", mission_id)])
        self.assertEqual(start_bypass_values, [False])
        self.assertEqual(start_new_mission_values, [False])

    def test_pallet3_poc_start_requires_the_runtime_service(self) -> None:
        response = self.client.post(
            "/api/v1/poc/pallet-3-missions",
            json={"robot_id": "robot_1"},
        )

        self.assertEqual(response.status_code, 503)

    def test_pallet3_poc_forwards_explicit_pick_bypass_to_the_runtime_service(self) -> None:
        # This catches silently rejecting or dropping the explicit POC-only PICK bypass.
        calls: list[tuple[str, bool]] = []

        class RecordingPocService:
            def __init__(self, store) -> None:
                self.store = store

            def start_pallet3_mission(
                self,
                robot_id: str,
                *,
                bypass_pick: bool = False,
                new_mission: bool = False,
            ):
                calls.append((robot_id, bypass_pick))
                mission, _created = self.store.create_or_get_pallet3_mission(robot_id)
                return mission

        app = create_app(
            str(Path(self.temporary_directory.name) / "poc-bypass.db"),
            service_factory=lambda store: RecordingPocService(store),
        )
        with TestClient(app) as client:
            response = client.post(
                "/api/v1/poc/pallet-3-missions",
                json={"robot_id": "robot_1", "bypass_pick": True},
            )

        self.assertEqual(response.status_code, 201)
        self.assertEqual(calls, [("robot_1", True)])

    def test_pallet3_poc_forwards_explicit_new_mission_request(self) -> None:
        # This catches dropping the explicit request to supersede an idle POC run.
        calls: list[tuple[str, bool, bool]] = []

        class RecordingPocService:
            def __init__(self, store) -> None:
                self.store = store

            def start_pallet3_mission(
                self,
                robot_id: str,
                *,
                bypass_pick: bool = False,
                new_mission: bool = False,
            ):
                calls.append((robot_id, bypass_pick, new_mission))
                mission, _created = self.store.create_or_get_pallet3_mission(robot_id)
                return mission

        app = create_app(
            str(Path(self.temporary_directory.name) / "poc-new-mission.db"),
            service_factory=lambda store: RecordingPocService(store),
        )
        with TestClient(app) as client:
            response = client.post(
                "/api/v1/poc/pallet-3-missions",
                json={"robot_id": "robot_1", "bypass_pick": True, "new_mission": True},
            )

        self.assertEqual(response.status_code, 201)
        self.assertEqual(calls, [("robot_1", True, True)])


if __name__ == "__main__":
    unittest.main()
