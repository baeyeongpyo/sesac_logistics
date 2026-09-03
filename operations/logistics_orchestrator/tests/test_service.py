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

    def navigate_waypoints(self, robot_id: str, payload: dict) -> None:
        if self.timeout:
            raise TimeoutError("timed out")
        self.commands.append(("navigate_waypoints", robot_id, payload))

    def fork_down(self, robot_id: str, payload: dict) -> None:
        if self.timeout:
            raise TimeoutError("timed out")
        self.commands.append(("fork_down", robot_id, payload))

    def command_velocity(self, robot_id: str, payload: dict) -> None:
        if self.timeout:
            raise TimeoutError("timed out")
        self.commands.append(("command_velocity", robot_id, payload))

    def stop(self, robot_id: str) -> None:
        self.commands.append(("stop", robot_id, {}))


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

    def test_pallet3_mission_runs_waypoints_then_confirmed_unload_reverse_and_return(self) -> None:
        inventory = FakeInventory(stocks=[], active_operations=[])
        vehicle = {"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}
        fleet = FakeFleet([vehicle])
        service = OrchestratorService(self.store, inventory, fleet)

        mission = service.start_pallet3_mission("robot_1")

        self.assertEqual(mission.phase, "OUTBOUND_SENT")
        self.assertEqual(fleet.commands[0][0:2], ("navigate_waypoints", "robot_1"))
        self.assertEqual(
            fleet.commands[0][2]["waypoints"],
            [
                {"frame_id": "map", "x": -0.44, "y": -0.9, "yaw": 0.0},
                {"frame_id": "map", "x": -0.42, "y": -2.0, "yaw": -1.5707963267948966},
                {"frame_id": "map", "x": -0.42, "y": -2.4, "yaw": -1.5707963267948966},
            ],
        )

        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "operation_id": mission.mission_id,
            "source": "NAV2", "detail": "NAVIGATION_SUCCEEDED",
        }
        service.reconcile()
        arrived = self.store.get_pallet3_mission(mission.mission_id)
        self.assertEqual(arrived.phase, "AWAIT_UNLOAD_CONFIRMATION")
        self.assertEqual(len(fleet.commands), 1)

        after_confirmation = service.confirm_pallet3_unload(mission.mission_id)
        self.assertEqual(after_confirmation.phase, "FORK_DOWN_SENT")
        self.assertEqual(fleet.commands[-1], (
            "fork_down", "robot_1", {"operation_id": mission.mission_id},
        ))

        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "operation_id": mission.mission_id,
            "source": "FORK", "detail": "FORK_DOWN_COMPLETE",
        }
        service.reconcile()
        self.assertEqual(fleet.commands[-1], (
            "command_velocity", "robot_1", {
                "operation_id": mission.mission_id,
                "linear_x": -0.18, "linear_y": 0.0, "angular_z": 0.0, "hold_ms": 1000,
            },
        ))

        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "operation_id": mission.mission_id,
            "source": "API", "detail": "MANUAL_COMMAND_EXPIRED",
        }
        service.reconcile()
        self.assertEqual(fleet.commands[-1][0:2], ("navigate_waypoints", "robot_1"))
        self.assertEqual(
            fleet.commands[-1][2]["waypoints"][-1],
            {"frame_id": "map", "x": 0.085, "y": -0.905, "yaw": 0.0},
        )

        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "operation_id": mission.mission_id,
            "source": "NAV2", "detail": "NAVIGATION_SUCCEEDED",
        }
        service.reconcile()

        self.assertEqual(self.store.get_pallet3_mission(mission.mission_id).phase, "COMPLETED")
        self.assertEqual(inventory.pick_completion_keys, [])
        self.assertEqual(inventory.place_completion_keys, [])

    def test_pallet3_mission_requires_arrival_confirmation_and_ignores_stale_fork_event(self) -> None:
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1)], active_operations=[])
        fleet = FakeFleet([{"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}])
        service = OrchestratorService(self.store, inventory, fleet)
        mission = service.start_pallet3_mission("robot_1")

        service.reconcile()

        with self.assertRaisesRegex(ValueError, "unload confirmation"):
            service.confirm_pallet3_unload(mission.mission_id)
        self.assertEqual([command[0] for command in fleet.commands], ["navigate_waypoints"])

        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "operation_id": mission.mission_id,
            "source": "NAV2", "detail": "NAVIGATION_SUCCEEDED",
        }
        service.reconcile()
        service.confirm_pallet3_unload(mission.mission_id)
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "operation_id": "other-operation",
            "source": "FORK", "detail": "FORK_DOWN_COMPLETE",
        }
        service.reconcile()

        self.assertEqual(self.store.get_pallet3_mission(mission.mission_id).phase, "FORK_DOWN_SENT")
        self.assertEqual([command[0] for command in fleet.commands], ["navigate_waypoints", "fork_down"])

    def test_pallet3_command_delivery_timeout_fails_and_requests_stop(self) -> None:
        inventory = FakeInventory(stocks=[], active_operations=[])
        fleet = FakeFleet(
            [{"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}],
            timeout=True,
        )

        mission = OrchestratorService(self.store, inventory, fleet).start_pallet3_mission("robot_1")

        self.assertEqual(mission.phase, "FAILED")
        self.assertIn("POC_OUTBOUND_WAYPOINTS", mission.failure_detail)
        self.assertEqual(fleet.commands, [("stop", "robot_1", {})])

    def test_pallet3_mission_fails_and_stops_on_fork_failure(self) -> None:
        inventory = FakeInventory(stocks=[], active_operations=[])
        fleet = FakeFleet([{"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}])
        service = OrchestratorService(self.store, inventory, fleet)
        mission = service.start_pallet3_mission("robot_1")
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "operation_id": mission.mission_id,
            "source": "NAV2", "detail": "NAVIGATION_SUCCEEDED",
        }
        service.reconcile()
        service.confirm_pallet3_unload(mission.mission_id)
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "FAIL", "operation_id": mission.mission_id,
            "source": "FORK", "detail": "FORK_DOWN_TIMEOUT",
        }
        service.reconcile()

        failed = self.store.get_pallet3_mission(mission.mission_id)
        self.assertEqual(failed.phase, "FAILED")
        self.assertEqual(failed.failure_detail, "FORK_DOWN_TIMEOUT")
        self.assertEqual(fleet.commands[-1], ("stop", "robot_1", {}))


if __name__ == "__main__":
    unittest.main()
