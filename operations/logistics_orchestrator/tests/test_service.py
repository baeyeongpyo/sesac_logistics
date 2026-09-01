from pathlib import Path
import tempfile
import unittest

from logistics_orchestrator.app.service import OrchestratorService
from logistics_orchestrator.app.store import OrchestratorStore


def _zones(*, disabled: set[str] | None = None) -> list[dict]:
    disabled = disabled or set()
    identifiers = ["docker", "p1", "p2", "p3"]
    identifiers += [f"f{index}" for index in range(1, 10)]
    identifiers += [f"n{index}" for index in range(1, 10)]
    return [
        {
            "zone_id": zone_id,
            "name": zone_id.upper(),
            "map_name": "map_0825",
            "nav_x": 0.11 if zone_id == "docker" else float(int(zone_id[1:])),
            "nav_y": -0.98 if zone_id == "docker" else -1.0,
            "nav_yaw": 0.0,
            "capacity": 24 if zone_id == "docker" else 1,
            "enabled": zone_id not in disabled,
        }
        for zone_id in identifiers
    ]


def _stock(zone_id: str, payload_type: str, quantity: int, reserved: int = 0) -> dict:
    return {
        "zone_id": zone_id,
        "payload_type": payload_type,
        "quantity": quantity,
        "reserved_quantity": reserved,
        "available_quantity": quantity - reserved,
    }


def _operation(status: str = "TO_PICK") -> dict:
    return {
        "operation_id": "operation-1",
        "robot_id": "robot_1",
        "payload_type": "FRESH",
        "source_zone_id": "docker",
        "destination_zone_id": "p1",
        "status": status,
        "priority": 0,
    }


class FakeInventory:
    def __init__(self, *, stocks: list[dict], active_operations: list[dict]) -> None:
        self.zones = _zones()
        self.stocks = stocks
        self.active_operations = active_operations
        self.created: list[dict] = []
        self.pick_completion_keys: list[str] = []
        self.place_completion_keys: list[str] = []

    def snapshot(self) -> dict:
        return {
            "zones": self.zones,
            "stocks": self.stocks,
            "active_operations": self.active_operations,
        }

    def create_operation(self, body: dict) -> dict:
        operation = {
            "operation_id": "created-operation",
            "robot_id": body["robot_id"],
            "payload_type": body["payload_type"],
            "source_zone_id": body["source_zone_id"],
            "destination_zone_id": body["destination_zone_id"],
            "status": "TO_PICK",
            "priority": body["priority"],
        }
        self.created.append(operation)
        self.active_operations.append(operation)
        return operation

    def complete_pick(self, operation_id: str, robot_id: str, idempotency_key: str) -> None:
        self.pick_completion_keys.append(idempotency_key)
        for operation in self.active_operations:
            if operation["operation_id"] == operation_id:
                operation["status"] = "TO_PLACE"

    def complete_place(self, operation_id: str, robot_id: str, idempotency_key: str) -> None:
        self.place_completion_keys.append(idempotency_key)
        self.active_operations = [
            operation
            for operation in self.active_operations
            if operation["operation_id"] != operation_id
        ]


class FakeFleet:
    def __init__(self, vehicles: list[dict], *, timeout: bool = False) -> None:
        self.vehicles = vehicles
        self.timeout = timeout
        self.commands: list[tuple[str, str, dict]] = []

    def list_vehicles(self) -> list[dict]:
        return self.vehicles

    def navigate(self, robot_id: str, payload: dict) -> None:
        if self.timeout:
            raise TimeoutError("timed out")
        self.commands.append(("navigate", robot_id, payload))

    def auto_dock(self, robot_id: str, payload: dict) -> None:
        if self.timeout:
            raise TimeoutError("timed out")
        self.commands.append(("auto_dock", robot_id, payload))


class OrchestratorServiceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        database_path = Path(self.temporary_directory.name) / "orchestrator.db"
        self.store = OrchestratorStore(database_path)

    def tearDown(self) -> None:
        self.store.close()
        self.temporary_directory.cleanup()

    def test_waiting_vehicle_creates_docker_fresh_pick_operation_then_navigates(self) -> None:
        # This catches a ready vehicle remaining idle while Docker Fresh cargo is available.
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1)], active_operations=[])
        fleet = FakeFleet([{"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}])
        OrchestratorService(self.store, inventory, fleet).reconcile()

        self.assertEqual(inventory.created[0]["source_zone_id"], "docker")
        self.assertEqual(inventory.created[0]["destination_zone_id"], "p1")
        self.assertEqual(fleet.commands[0][0:2], ("navigate", "robot_1"))
        self.assertEqual(fleet.commands[0][2]["purpose"], "PICK")

    def test_navigation_success_for_to_pick_sends_pick_auto_dock(self) -> None:
        # This catches arriving at source without starting the physical PICK action.
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1, 1)], active_operations=[_operation()])
        fleet = FakeFleet(
            [{"robot_id": "robot_1", "state": "WAIT", "detail": "NAVIGATION_SUCCEEDED", "operation_id": "operation-1"}]
        )
        OrchestratorService(self.store, inventory, fleet).reconcile()

        command = fleet.commands[0]
        self.assertEqual(command[0:2], ("auto_dock", "robot_1"))
        self.assertEqual(command[2]["operation"], "PICK")
        self.assertEqual(command[2]["location"], "DOCKER")

    def test_pick_complete_commits_inventory_once_then_navigates_to_destination(self) -> None:
        # This catches moving to PLACE before the Inventory source decrement is durable.
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1, 1)], active_operations=[_operation()])
        fleet = FakeFleet(
            [{"robot_id": "robot_1", "state": "WAIT", "detail": "AUTO_DOCK_PICK_COMPLETED", "operation_id": "operation-1"}]
        )
        OrchestratorService(self.store, inventory, fleet).reconcile()

        self.assertEqual(inventory.pick_completion_keys, ["operation-1:pick"])
        self.assertEqual(fleet.commands[0][0], "navigate")
        self.assertEqual(fleet.commands[0][2]["purpose"], "PLACE")
        self.assertEqual(fleet.commands[0][2]["x"], 1.0)

    def test_place_complete_commits_inventory_without_duplicate_vehicle_command(self) -> None:
        # This catches a completed placement retaining robot cargo or reissuing a stale command.
        inventory = FakeInventory(stocks=[], active_operations=[_operation("TO_PLACE")])
        fleet = FakeFleet(
            [{"robot_id": "robot_1", "state": "WAIT", "detail": "AUTO_DOCK_PLACE_COMPLETED", "operation_id": "operation-1"}]
        )
        OrchestratorService(self.store, inventory, fleet).reconcile()

        self.assertEqual(inventory.place_completion_keys, ["operation-1:place"])
        self.assertEqual(fleet.commands, [])

    def test_fail_does_not_command_until_operator_ready_then_resumes_active_operation(self) -> None:
        # This catches automatic movement while a human recovery acknowledgement is still absent.
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1, 1)], active_operations=[_operation()])
        fleet = FakeFleet([{"robot_id": "robot_1", "state": "FAIL", "detail": "NAVIGATION_FAILED"}])
        service = OrchestratorService(self.store, inventory, fleet)

        service.reconcile()
        fleet.vehicles[0] = {"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}
        service.reconcile()

        self.assertEqual(fleet.commands[0][0], "navigate")
        self.assertEqual(fleet.commands[0][2]["purpose"], "PICK")

    def test_invalid_inventory_environment_records_error_and_sends_no_command(self) -> None:
        # This catches commanding a vehicle to an unconfigured physical zone.
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1)], active_operations=[])
        inventory.zones = _zones(disabled={"f1"})
        fleet = FakeFleet([{"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}])
        OrchestratorService(self.store, inventory, fleet).reconcile()

        self.assertEqual(fleet.commands, [])
        self.assertIn("f1 disabled", self.store.status()["errors"][0]["message"])

    def test_navigation_timeout_is_delivery_unknown_and_is_not_sent_again(self) -> None:
        # This catches a timed-out Nav2 request being replayed before vehicle state confirms it.
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1)], active_operations=[])
        fleet = FakeFleet(
            [{"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}],
            timeout=True,
        )
        service = OrchestratorService(self.store, inventory, fleet)

        service.reconcile()
        service.reconcile()

        self.assertEqual(fleet.commands, [])
        self.assertEqual(self.store.status()["command_counts"], {"DELIVERY_UNKNOWN": 1})


if __name__ == "__main__":
    unittest.main()
