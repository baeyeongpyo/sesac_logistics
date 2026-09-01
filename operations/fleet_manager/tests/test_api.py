import tempfile
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from fleet_manager.app.fleet import FleetEventDispatcher
from fleet_manager.app.main import create_app


class FleetManagerApiTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        database_path = Path(self.tempdir.name) / "fleet_manager.db"
        self.client = TestClient(create_app(str(database_path)))
        self.client.__enter__()

    def tearDown(self) -> None:
        self.client.__exit__(None, None, None)
        self.tempdir.cleanup()

    def test_healthz_reports_service_ready(self) -> None:
        response = self.client.get("/healthz")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "ok"})

    def test_state_report_creates_vehicle_snapshot(self) -> None:
        response = self.client.post(
            "/api/v1/vehicles/robot-1/state",
            json={
                "state": "DRIVE",
                "previous_state": "WAIT",
                "operation_id": "operation-1",
                "attempt_id": "attempt-1",
                "source": "NAV2",
                "detail": "NAVIGATION_STARTED",
                "observed_at": "2026-08-31T12:00:00Z",
            },
        )

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["robot_id"], "robot-1")
        self.assertEqual(response.json()["state"], "DRIVE")
        self.assertEqual(response.json()["previous_state"], "WAIT")
        self.assertEqual(response.json()["operation_id"], "operation-1")

    def test_vehicle_snapshot_is_available_by_robot_id(self) -> None:
        self.client.post(
            "/api/v1/vehicles/robot-1/state",
            json={
                "state": "WAIT",
                "previous_state": "INIT",
                "operation_id": None,
                "attempt_id": None,
                "source": "API",
                "detail": "OPERATOR_APPROVED",
                "observed_at": "2026-08-31T12:00:00Z",
            },
        )

        response = self.client.get("/api/v1/vehicles/robot-1")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["state"], "WAIT")
        self.assertEqual(response.json()["detail"], "OPERATOR_APPROVED")

    def test_vehicle_list_returns_each_latest_snapshot(self) -> None:
        for robot_id, state in (("robot-2", "WAIT"), ("robot-1", "INIT")):
            self.assertEqual(
                self.client.post(
                    f"/api/v1/vehicles/{robot_id}/state",
                    json={
                        "state": state,
                        "previous_state": None,
                        "operation_id": None,
                        "attempt_id": None,
                        "source": "VEHICLE",
                        "detail": "VEHICLE_BOOTED",
                        "observed_at": "2026-08-31T12:00:00Z",
                    },
                ).status_code,
                200,
            )

        response = self.client.get("/api/v1/vehicles")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            [snapshot["robot_id"] for snapshot in response.json()],
            ["robot-1", "robot-2"],
        )

    def test_vehicle_logs_page_from_newest_to_oldest(self) -> None:
        for state, previous_state in (("INIT", None), ("DRIVE", "INIT")):
            self.assertEqual(
                self.client.post(
                    "/api/v1/vehicles/robot-1/state",
                    json={
                        "state": state,
                        "previous_state": previous_state,
                        "operation_id": None,
                        "attempt_id": None,
                        "source": "VEHICLE",
                        "detail": f"{state}_EVENT",
                        "observed_at": "2026-08-31T12:00:00Z",
                    },
                ).status_code,
                200,
        )

        newest_page = self.client.get("/api/v1/vehicles/robot-1/logs?limit=1")
        self.assertEqual(newest_page.status_code, 200)
        older_page = self.client.get(
            f"/api/v1/vehicles/robot-1/logs?before_id={newest_page.json()[0]['id']}&limit=1"
        )

        self.assertEqual(newest_page.json()[0]["state"], "DRIVE")
        self.assertEqual(older_page.status_code, 200)
        self.assertEqual(older_page.json()[0]["state"], "INIT")

    def test_state_report_rejects_unknown_fields(self) -> None:
        response = self.client.post(
            "/api/v1/vehicles/robot-1/state",
            json={
                "state": "INIT",
                "previous_state": None,
                "operation_id": None,
                "attempt_id": None,
                "source": "VEHICLE",
                "detail": "VEHICLE_BOOTED",
                "observed_at": "2026-08-31T12:00:00Z",
                "unexpected": True,
            },
        )

        self.assertEqual(response.status_code, 422)

    def test_empty_orchestrator_event_url_does_not_start_a_dispatcher(self) -> None:
        # This catches a stand-alone Fleet Manager continuously posting to no destination.
        app = create_app(
            str(Path(self.tempdir.name) / "unconfigured-outbox.db"),
            orchestrator_event_url="",
        )

        with TestClient(app):
            self.assertIsNone(app.state.event_dispatcher)

    def test_configured_orchestrator_event_url_starts_a_dispatcher(self) -> None:
        # This catches dropping Fleet reports when Orchestrator delivery is configured.
        app = create_app(
            str(Path(self.tempdir.name) / "configured-outbox.db"),
            orchestrator_event_url="http://127.0.0.1:9999/api/v1/events/fleet",
            retry_interval_sec=60,
        )

        with TestClient(app):
            self.assertIsInstance(app.state.event_dispatcher, FleetEventDispatcher)


if __name__ == "__main__":
    unittest.main()
