import json
import tempfile
import unittest
from pathlib import Path

import httpx
from fastapi.testclient import TestClient

from fleet_manager.app.commands import (
    BridgeCommandClient,
    BridgeUnavailableError,
    RelayResponse,
    load_vehicle_registry,
)
from fleet_manager.app.main import create_app


class RecordingBridgeClient:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, str, dict]] = []
        self.response = RelayResponse(status_code=202, body={"state": "DRIVE"})
        self.error: Exception | None = None

    def relay(self, vehicle, path: str, payload: dict) -> RelayResponse:
        if self.error is not None:
            raise self.error
        self.calls.append((vehicle.id, vehicle.model.id, path, payload))
        return self.response


class FleetManagerCommandApiTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.database_path = Path(self.temporary_directory.name) / "fleet_manager.db"
        self.registry_path = Path(self.temporary_directory.name) / "vehicles.yaml"
        self.registry_path.write_text(
            """models:
  - id: mentorpi
    bridge_url: http://command-api:8080
    capabilities: [navigate, auto_dock, stop, report_status]
  - id: limited
    bridge_url: http://limited-bridge:8080
    capabilities: [navigate]
vehicles:
  - id: robot_1
    model: mentorpi
  - id: robot_2
    model: limited
"""
        )
        self.bridge = RecordingBridgeClient()
        self.client = TestClient(
            create_app(
                str(self.database_path),
                vehicle_registry_path=str(self.registry_path),
                bridge_client=self.bridge,
            )
        )
        self.client.__enter__()

    def tearDown(self) -> None:
        self.client.__exit__(None, None, None)
        self.temporary_directory.cleanup()

    def test_navigation_resolves_robot_model_and_relays_to_its_bridge(self) -> None:
        payload = {
            "operation_id": "operation-1",
            "purpose": "PICK",
            "frame_id": "map",
            "x": 1.0,
            "y": 2.0,
            "yaw": 0.0,
        }

        response = self.client.post(
            "/api/v1/vehicles/robot_1/commands/navigation/goals",
            json=payload,
        )

        self.assertEqual(response.status_code, 202)
        self.assertEqual(response.json(), {"state": "DRIVE"})
        self.assertEqual(
            self.bridge.calls,
            [
                (
                    "robot_1",
                    "mentorpi",
                    "/api/v1/vehicle-command/robot_1/navigation/goals",
                    payload,
                )
            ],
        )

    def test_auto_dock_relays_the_existing_vehicle_payload(self) -> None:
        payload = {
            "operation_id": "operation-1",
            "operation": "PICK",
            "product_type": "FRESH",
            "location": "DOCKER",
            "target": {"type": "NEAREST"},
        }

        response = self.client.post(
            "/api/v1/vehicles/robot_1/commands/auto-dock",
            json=payload,
        )

        self.assertEqual(response.status_code, 202)
        self.assertEqual(
            self.bridge.calls[0][2:],
            ("/api/v1/vehicle-command/robot_1/auto-dock", payload),
        )

    def test_unsupported_capability_is_rejected_before_bridge_delivery(self) -> None:
        response = self.client.post(
            "/api/v1/vehicles/robot_2/commands/auto-dock",
            json={"operation_id": "operation-1"},
        )

        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.bridge.calls, [])

    def test_unknown_vehicle_is_rejected_before_bridge_delivery(self) -> None:
        response = self.client.post(
            "/api/v1/vehicles/robot_x/commands/navigation/goals",
            json={"operation_id": "operation-1"},
        )

        self.assertEqual(response.status_code, 404)
        self.assertEqual(self.bridge.calls, [])

    def test_bridge_rejection_status_and_body_are_preserved(self) -> None:
        self.bridge.response = RelayResponse(
            status_code=409,
            body={"detail": "vehicle is already driving"},
        )

        response = self.client.post(
            "/api/v1/vehicles/robot_1/commands/navigation/goals",
            json={"operation_id": "operation-1"},
        )

        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json(), {"detail": "vehicle is already driving"})

    def test_bridge_timeout_is_reported_as_service_unavailable(self) -> None:
        self.bridge.error = BridgeUnavailableError("bridge timed out")

        response = self.client.post(
            "/api/v1/vehicles/robot_1/commands/navigation/goals",
            json={"operation_id": "operation-1"},
        )

        self.assertEqual(response.status_code, 503)
        self.assertIn("bridge timed out", response.json()["detail"])

    def test_http_bridge_client_posts_to_the_registered_model_endpoint(self) -> None:
        received: dict[str, object] = {}

        def respond(request: httpx.Request) -> httpx.Response:
            received["url"] = str(request.url)
            received["body"] = json.loads(request.content)
            return httpx.Response(202, json={"state": "DRIVE"})

        client = httpx.Client(transport=httpx.MockTransport(respond))
        bridge = BridgeCommandClient(client=client)
        vehicle = load_vehicle_registry(self.registry_path).require("robot_1", "navigate")
        payload = {"operation_id": "operation-1", "purpose": "PICK"}

        response = bridge.relay(
            vehicle,
            "/api/v1/vehicle-command/robot_1/navigation/goals",
            payload,
        )

        self.assertEqual(response.status_code, 202)
        self.assertEqual(response.body, {"state": "DRIVE"})
        self.assertEqual(
            received["url"],
            "http://command-api:8080/api/v1/vehicle-command/robot_1/navigation/goals",
        )
        self.assertEqual(received["body"], payload)


if __name__ == "__main__":
    unittest.main()
