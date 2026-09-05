import tempfile
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from fleet_manager.app.commands import ModelBridge, RegisteredVehicle, RelayResponse, VehicleRegistry
from fleet_manager.app.fleet import StateSource, VehicleState, VehicleStateReport, VehicleStateStore
from fleet_manager.app.pallet3 import Pallet3MissionService
from fleet_manager.app.main import create_app


class RecordingInventory:
    def __init__(self) -> None:
        self.created: list[tuple[str, str, str, str]] = []
        self.pick_calls: list[tuple[str, str, str]] = []
        self.place_calls: list[tuple[str, str, str]] = []

    def create_operation(
        self, payload_type: str, source_zone_id: str, destination_zone_id: str, robot_id: str
    ) -> str:
        self.created.append((payload_type, source_zone_id, destination_zone_id, robot_id))
        return "operation-1"

    def complete_pick(self, operation_id: str, robot_id: str, idempotency_key: str) -> None:
        self.pick_calls.append((operation_id, robot_id, idempotency_key))

    def complete_place(self, operation_id: str, robot_id: str, idempotency_key: str) -> None:
        self.place_calls.append((operation_id, robot_id, idempotency_key))


class RecordingBridge:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, dict]] = []

    def relay(self, vehicle: RegisteredVehicle, path: str, payload: dict) -> RelayResponse:
        self.calls.append((vehicle.id, path, payload))
        return RelayResponse(status_code=202, body={"accepted": True})


class Pallet3MissionServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        self.store = VehicleStateStore(Path(self.tempdir.name) / "fleet_manager.db")
        model = ModelBridge(
            "mentorpi", "http://bridge.example", frozenset({"pallet3_mission"})
        )
        self.registry = VehicleRegistry({"robot_1": RegisteredVehicle("robot_1", model)})
        self.inventory = RecordingInventory()
        self.bridge = RecordingBridge()
        self.service = Pallet3MissionService(
            self.store,
            self.inventory,
            self.registry,
            self.bridge,
            fleet_manager_url="http://fleet.example:8090",
        )
        self.record_state(VehicleState.WAIT, "OPERATOR_READY", operation_id=None)

    def tearDown(self) -> None:
        self.store.close()
        self.tempdir.cleanup()

    def record_state(self, state: VehicleState, detail: str, *, operation_id: str | None) -> None:
        self.store.record_state(
            "robot_1",
            VehicleStateReport(
                state=state,
                previous_state=None,
                operation_id=operation_id,
                attempt_id=None,
                source=StateSource.API,
                detail=detail,
                observed_at="2026-09-05T00:00:00Z",
            ),
        )

    def test_start_creates_normal_docker_to_p3_operation_then_relays_shell_request(self) -> None:
        result = self.service.start("robot_1", "manual")

        self.assertEqual(
            self.inventory.created, [("NORMAL", "docker", "p3", "robot_1")]
        )
        self.assertEqual(self.bridge.calls[0][1], "/api/v1/vehicle-command/robot_1/missions/pallet3")
        self.assertEqual(self.bridge.calls[0][2]["operation_id"], result["operation_id"])
        self.assertEqual(self.bridge.calls[0][2]["robot_id"], "robot_1")
        self.assertEqual(self.bridge.calls[0][2]["pick_mode"], "manual")

    def test_pick_and_place_events_require_matching_vehicle_replies_and_complete_inventory_once(self) -> None:
        operation_id = self.service.start("robot_1", "auto_dock")["operation_id"]
        self.service.record_vehicle_reply(
            "robot_1", operation_id, "AUTO_DOCK_PICK_COMPLETED"
        )

        picked = self.service.record_event(
            "robot_1", operation_id, "PICK_COMPLETED", operation_id + ":pick"
        )

        self.service.record_vehicle_reply("robot_1", operation_id, "FORK_DOWN_COMPLETE")
        placed = self.service.record_event(
            "robot_1", operation_id, "PLACE_READY", operation_id + ":place"
        )
        repeated = self.service.record_event(
            "robot_1", operation_id, "PLACE_READY", operation_id + ":place"
        )

        self.assertEqual(picked["phase"], "PICKED")
        self.assertEqual(placed["phase"], "PLACED")
        self.assertEqual(repeated["phase"], "PLACED")
        self.assertEqual(
            self.inventory.pick_calls, [(operation_id, "robot_1", operation_id + ":pick")]
        )
        self.assertEqual(
            self.inventory.place_calls, [(operation_id, "robot_1", operation_id + ":place")]
        )

    def test_manual_pick_is_rejected_until_fork_up_completion_is_recorded(self) -> None:
        operation_id = self.service.start("robot_1", "manual")["operation_id"]

        with self.assertRaisesRegex(ValueError, "FORK_UP_COMPLETE"):
            self.service.record_event(
                "robot_1", operation_id, "PICK_COMPLETED", operation_id + ":pick"
            )

        self.assertEqual(self.inventory.pick_calls, [])

    def test_second_pallet3_start_is_rejected_before_creating_another_inventory_operation(self) -> None:
        self.service.start("robot_1", "manual")

        with self.assertRaisesRegex(ValueError, "already active"):
            self.service.start("robot_1", "auto_dock")

        self.assertEqual(self.inventory.created, [("NORMAL", "docker", "p3", "robot_1")])


class Pallet3MissionApiTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        root = Path(self.tempdir.name)
        registry_path = root / "vehicles.yaml"
        registry_path.write_text(
            """models:
  - id: mentorpi
    bridge_url: http://bridge.example
    capabilities: [navigate, auto_dock, stop, report_status, fork, manual_drive, pallet3_mission]
vehicles:
  - id: robot_1
    model: mentorpi
"""
        )
        self.inventory = RecordingInventory()
        self.bridge = RecordingBridge()
        self.client = TestClient(
            create_app(
                str(root / "fleet_manager.db"),
                vehicle_registry_path=registry_path,
                bridge_client=self.bridge,
                inventory_client=self.inventory,
            )
        )
        self.client.__enter__()
        self.client.post(
            "/api/v1/vehicles/robot_1/state",
            json={
                "state": "WAIT",
                "previous_state": "INIT",
                "operation_id": None,
                "attempt_id": None,
                "source": "API",
                "detail": "OPERATOR_READY",
                "observed_at": "2026-09-05T00:00:00Z",
            },
        )

    def tearDown(self) -> None:
        self.client.__exit__(None, None, None)
        self.tempdir.cleanup()

    def test_start_requires_pick_mode_and_events_complete_inventory_after_vehicle_reply(self) -> None:
        self.assertEqual(
            self.client.post("/api/v1/vehicles/robot_1/missions/pallet3", json={}).status_code,
            422,
        )
        started = self.client.post(
            "/api/v1/vehicles/robot_1/missions/pallet3",
            json={"pick_mode": "manual"},
        )
        self.assertEqual(started.status_code, 202)
        operation_id = started.json()["operation_id"]
        self.assertEqual(
            self.bridge.calls[0][2]['fleet_manager_url'],
            'http://192.168.100.27:8090',
        )
        event_url = f"/api/v1/vehicles/robot_1/missions/pallet3/{operation_id}/events"

        self.assertEqual(
            self.client.post(
                event_url,
                json={"event_type": "PICK_COMPLETED", "idempotency_key": operation_id + ":pick"},
            ).status_code,
            409,
        )
        self.client.post(
            "/api/v1/vehicles/robot_1/state",
            json={
                "state": "WAIT",
                "previous_state": "PICK",
                "operation_id": operation_id,
                "attempt_id": None,
                "source": "FORK",
                "detail": "FORK_UP_COMPLETE",
                "observed_at": "2026-09-05T00:00:01Z",
            },
        )
        picked = self.client.post(
            event_url,
            json={"event_type": "PICK_COMPLETED", "idempotency_key": operation_id + ":pick"},
        )

        self.assertEqual(picked.status_code, 200)
        self.assertEqual(self.inventory.pick_calls, [(operation_id, "robot_1", operation_id + ":pick")])


if __name__ == "__main__":
    unittest.main()
