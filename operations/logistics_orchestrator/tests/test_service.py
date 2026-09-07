from pathlib import Path
import tempfile
import unittest

from logistics_orchestrator.app.service import (
    OUTBOUND_PALLET3_WAYPOINTS,
    OrchestratorService,
)
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


def _pallet3_operation(status: str = "TO_PICK") -> dict:
    return {
        **_operation(status),
        "destination_zone_id": "p3",
    }


class FakeInventory:
    def __init__(self, *, stocks: list[dict], active_operations: list[dict]) -> None:
        self.zones = _zones()
        self.stocks = stocks
        self.active_operations = active_operations
        self.created: list[dict] = []
        self.pick_completion_keys: list[str] = []
        self.place_completion_keys: list[str] = []
        self.force_completion_keys: list[str] = []
        self.pallet_states: dict[str, dict] = {}

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

    def pallet_state(self, robot_id: str) -> dict:
        return self.pallet_states.get(
            robot_id,
            {"robot_id": robot_id, "has_pallet": False, "payload_type": None},
        )

    def complete_pick(self, operation_id: str, robot_id: str, idempotency_key: str) -> None:
        self.pick_completion_keys.append(idempotency_key)
        for operation in self.active_operations:
            if operation["operation_id"] == operation_id:
                operation["status"] = "TO_PLACE"
                for stock in self.stocks:
                    if (
                        stock["zone_id"] == operation["source_zone_id"]
                        and stock["payload_type"] == operation["payload_type"]
                    ):
                        stock["quantity"] -= 1
                        stock["reserved_quantity"] -= 1
                        stock["available_quantity"] = (
                            stock["quantity"] - stock["reserved_quantity"]
                        )
                self.pallet_states[robot_id] = {
                    "robot_id": robot_id,
                    "has_pallet": True,
                    "payload_type": operation["payload_type"],
                }

    def complete_place(self, operation_id: str, robot_id: str, idempotency_key: str) -> None:
        self.place_completion_keys.append(idempotency_key)
        operation = next(
            item
            for item in self.active_operations
            if item["operation_id"] == operation_id
        )
        for stock in self.stocks:
            if (
                stock["zone_id"] == operation["destination_zone_id"]
                and stock["payload_type"] == operation["payload_type"]
            ):
                stock["quantity"] += 1
                stock["available_quantity"] = (
                    stock["quantity"] - stock["reserved_quantity"]
                )
        self.active_operations = [
            operation
            for operation in self.active_operations
            if operation["operation_id"] != operation_id
        ]
        self.pallet_states[robot_id] = {
            "robot_id": robot_id,
            "has_pallet": False,
            "payload_type": None,
        }

    def force_complete(self, operation_id: str, robot_id: str, idempotency_key: str) -> None:
        self.force_completion_keys.append(idempotency_key)
        operation = next(
            item
            for item in self.active_operations
            if item["operation_id"] == operation_id
        )
        if operation["status"] in {"TO_PICK", "PICKING"}:
            for stock in self.stocks:
                if (
                    stock["zone_id"] == operation["source_zone_id"]
                    and stock["payload_type"] == operation["payload_type"]
                ):
                    stock["reserved_quantity"] -= 1
                    stock["available_quantity"] = (
                        stock["quantity"] - stock["reserved_quantity"]
                    )
        elif operation["status"] in {"TO_PLACE", "PLACING", "RECOVERY_REQUIRED"}:
            for stock in self.stocks:
                if (
                    stock["zone_id"] == operation["destination_zone_id"]
                    and stock["payload_type"] == operation["payload_type"]
                ):
                    stock["quantity"] += 1
                    stock["available_quantity"] = (
                        stock["quantity"] - stock["reserved_quantity"]
                    )
        self.active_operations = [
            item
            for item in self.active_operations
            if item["operation_id"] != operation_id
        ]
        self.pallet_states[robot_id] = {
            "robot_id": robot_id,
            "has_pallet": False,
            "payload_type": None,
        }


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
        self.assertEqual(command[2]["location"], "DOCK_1")

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

    def test_manual_pallet3_pick_commits_inventory_once_without_vehicle_command(self) -> None:
        # This catches treating manual loading as a route-only POC that never changes stock.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "INIT", "detail": "VEHICLE_BOOTED"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        workflow = service.confirm_pallet3_manual_pick("operation-1", True)
        replayed = service.confirm_pallet3_manual_pick("operation-1", True)

        self.assertEqual(workflow.operation_id, "operation-1")
        self.assertEqual(replayed.operation_id, "operation-1")
        self.assertEqual(inventory.pick_completion_keys, ["operation-1:manual-pick"])
        self.assertEqual(inventory.active_operations[0]["status"], "TO_PLACE")
        self.assertTrue(inventory.pallet_state("robot_1")["has_pallet"])
        self.assertEqual(fleet.commands, [])

    def test_manual_pallet3_pick_wait_starts_outbound_without_docker_navigation(self) -> None:
        # This catches WAIT after manual loading restarting the normal Docker Auto Dock route.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "INIT", "detail": "VEHICLE_BOOTED"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.confirm_pallet3_manual_pick("operation-1", True)
        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "source": "API",
            "detail": "OPERATOR_READY",
        }
        service.reconcile()

        self.assertEqual([command[0] for command in fleet.commands], ["navigate_waypoints"])
        self.assertEqual(fleet.commands[0][2]["operation_id"], "operation-1")
        self.assertEqual(fleet.commands[0][2]["waypoints"][-1]["y"], -2.34)
        self.assertEqual(self.store.get_pallet3_workflow("operation-1").phase, "OUTBOUND_SENT")

    def test_force_complete_pallet3_closes_a_loaded_failed_transfer_even_while_vehicle_drives(
        self,
    ) -> None:
        # This catches using the current Fleet state as a gate for server-ledger closure.
        inventory = FakeInventory(
            stocks=[_stock("docker", "FRESH", 0), _stock("p3", "FRESH", 0)],
            active_operations=[_pallet3_operation("TO_PLACE")],
        )
        inventory.pallet_states["robot_1"] = {
            "robot_id": "robot_1",
            "has_pallet": True,
            "payload_type": "FRESH",
        }
        fleet = FakeFleet([
            {
                "robot_id": "robot_1",
                "state": "DRIVE",
                "detail": "NAVIGATION_STARTED",
            }
        ])
        self.store.create_or_get_pallet3_workflow("operation-1", "robot_1")
        self.store.transition_pallet3_workflow(
            "operation-1", "PICK_PENDING", "OUTBOUND_SENT"
        )
        self.store.fail_pallet3_workflow("operation-1", "P3_OUTBOUND_WAYPOINTS")
        self.store.mark_recovery_required("operation-1", "robot_1")
        service = OrchestratorService(self.store, inventory, fleet)

        workflow = service.force_complete_pallet3_operation("operation-1", True)
        replayed = service.force_complete_pallet3_operation("operation-1", True)

        self.assertEqual(inventory.force_completion_keys, ["operation-1:force-complete"])
        self.assertEqual(inventory.active_operations, [])
        self.assertEqual(inventory.stocks[1]["quantity"], 1)
        self.assertFalse(inventory.pallet_state("robot_1")["has_pallet"])
        self.assertEqual(workflow.phase, "COMPLETED")
        self.assertEqual(workflow.failure_detail, "FORCE_COMPLETED_BY_OPERATOR")
        self.assertIsNotNone(workflow.completed_at)
        self.assertEqual(replayed.phase, "COMPLETED")
        self.assertFalse(self.store.recovery_required("operation-1"))
        self.assertEqual(fleet.commands, [])

    def test_manual_pallet3_pick_is_rejected_after_automatic_pick_navigation_started(self) -> None:
        # This catches bypass racing an already dispatched Docker approach command.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.reconcile()

        with self.assertRaisesRegex(ValueError, "automatic pick has already started"):
            service.confirm_pallet3_manual_pick("operation-1", True)
        self.assertEqual(inventory.pick_completion_keys, [])

    def test_auto_pallet3_recipe_down_completes_inventory_then_reverses_and_returns(self) -> None:
        # This catches p3 stock changing before the physical Fork DOWN completion.
        inventory = FakeInventory(
            stocks=[_stock("docker", "FRESH", 1, 1), _stock("p3", "FRESH", 0)],
            active_operations=[_pallet3_operation()],
        )
        fleet = FakeFleet([
            {
                "robot_id": "robot_1",
                "state": "WAIT",
                "operation_id": "operation-1",
                "source": "AUTO_DOCK",
                "detail": "AUTO_DOCK_PICK_COMPLETED",
            }
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.reconcile()

        self.assertEqual(inventory.pick_completion_keys, ["operation-1:pick"])
        self.assertEqual(inventory.stocks[0]["quantity"], 0)
        self.assertEqual(fleet.commands[-1][0:2], ("navigate_waypoints", "robot_1"))
        self.assertEqual(fleet.commands[-1][2]["operation_id"], "operation-1")
        self.assertEqual(self.store.get_pallet3_workflow("operation-1").phase, "OUTBOUND_SENT")

        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "operation_id": "operation-1",
            "source": "NAV2",
            "detail": "NAVIGATION_SUCCEEDED",
        }
        service.reconcile()

        self.assertEqual(fleet.commands[-1], ("fork_down", "robot_1", {"operation_id": "operation-1"}))
        self.assertEqual(inventory.place_completion_keys, [])

        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "operation_id": "operation-1",
            "source": "FORK",
            "detail": "FORK_DOWN_COMPLETE",
        }
        service.reconcile()

        self.assertEqual(inventory.place_completion_keys, ["operation-1:place"])
        self.assertEqual(inventory.stocks[1]["quantity"], 1)
        self.assertEqual(
            fleet.commands[-1],
            (
                "command_velocity",
                "robot_1",
                {
                    "operation_id": "operation-1",
                    "linear_x": -0.18,
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                    "hold_ms": 1000,
                },
            ),
        )

        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "operation_id": "operation-1",
            "source": "API",
            "detail": "MANUAL_COMMAND_EXPIRED",
        }
        service.reconcile()

        self.assertEqual(fleet.commands[-1][0:2], ("navigate_waypoints", "robot_1"))
        self.assertEqual(fleet.commands[-1][2]["waypoints"][-1], {
            "frame_id": "map", "x": 0.085, "y": -0.905, "yaw": 0.0,
        })

        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "operation_id": "operation-1",
            "source": "NAV2",
            "detail": "NAVIGATION_SUCCEEDED",
        }
        service.reconcile()

        workflow = self.store.get_pallet3_workflow("operation-1")
        self.assertEqual(workflow.phase, "COMPLETED")
        self.assertEqual(inventory.place_completion_keys, ["operation-1:place"])

    def test_pallet3_operation_uses_its_own_zone_validation(self) -> None:
        # This catches unrelated f/n configuration blocking an already reserved docker-to-p3 run.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        inventory.zones = _zones(disabled={"f1"})
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}
        ])

        OrchestratorService(self.store, inventory, fleet).reconcile()

        self.assertEqual(fleet.commands[0][0:2], ("navigate", "robot_1"))
        self.assertEqual(fleet.commands[0][2]["purpose"], "PICK")

    def test_pallet3_pick_phase_recovers_only_after_operator_ready(self) -> None:
        # This catches recovery emitting a second Docker command before the operator makes it safe.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.reconcile()
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "FAIL", "detail": "NAVIGATION_FAILED"
        }
        service.reconcile()
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"
        }
        service.reconcile()

        self.assertEqual([command[0] for command in fleet.commands], ["navigate", "navigate"])
        self.assertEqual(fleet.commands[-1][2]["purpose"], "PICK")

    def test_pallet3_outbound_recovers_after_operator_ready(self) -> None:
        # This catches a failed route to p3 becoming unrecoverable even though cargo is still loaded.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "INIT", "detail": "VEHICLE_BOOTED"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.confirm_pallet3_manual_pick("operation-1", True)
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"
        }
        service.reconcile()
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "FAIL", "detail": "NAVIGATION_FAILED"
        }
        service.reconcile()
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"
        }
        service.reconcile()

        self.assertEqual(
            [command[0] for command in fleet.commands],
            ["navigate_waypoints", "navigate_waypoints"],
        )
        self.assertEqual(self.store.get_pallet3_workflow("operation-1").phase, "OUTBOUND_SENT")

    def test_manual_pallet3_recovery_resends_outbound_each_time_without_changing_inventory(self) -> None:
        # This catches a second manual stop being acknowledged without a fresh Nav2 command.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "INIT", "detail": "VEHICLE_BOOTED"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.confirm_pallet3_manual_pick("operation-1", True)
        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "detail": "OPERATOR_READY",
            "observed_at": "2026-09-04T01:30:00Z",
        }
        service.reconcile()

        recovered = service.recover_pallet3_operation("operation-1")
        with self.assertRaisesRegex(ValueError, "new OPERATOR_READY report"):
            service.recover_pallet3_operation("operation-1")
        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "detail": "OPERATOR_READY",
            "observed_at": "2026-09-04T01:31:00Z",
        }
        recovered_again = service.recover_pallet3_operation("operation-1")
        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "detail": "OPERATOR_READY",
            "observed_at": "2026-09-04T01:30:00Z",
        }
        with self.assertRaisesRegex(ValueError, "new OPERATOR_READY report"):
            service.recover_pallet3_operation("operation-1")

        self.assertEqual(recovered.phase, "OUTBOUND_SENT")
        self.assertEqual(recovered_again.phase, "OUTBOUND_SENT")
        self.assertEqual(
            [command[0] for command in fleet.commands],
            ["navigate_waypoints", "navigate_waypoints", "navigate_waypoints"],
        )
        self.assertEqual(
            fleet.commands[-1][2]["waypoints"],
            OUTBOUND_PALLET3_WAYPOINTS,
        )
        self.assertEqual(inventory.pick_completion_keys, ["operation-1:manual-pick"])
        self.assertEqual(inventory.place_completion_keys, [])
        self.assertTrue(self.store.has_command("operation-1", "P3_OUTBOUND_WAYPOINTS_RECOVERY"))
        self.assertTrue(self.store.has_command("operation-1", "P3_OUTBOUND_WAYPOINTS_RECOVERY_2"))
        self.assertEqual(fleet.commands[-1][2]["operation_id"], "operation-1")

    def test_manual_pallet3_recovery_refuses_inconsistent_inventory_and_workflow_states(self) -> None:
        # This catches recovery restarting Docker PICK after cargo was already recorded outbound.
        fleet = FakeFleet([
            {
                "robot_id": "robot_1",
                "state": "WAIT",
                "detail": "OPERATOR_READY",
                "observed_at": "2026-09-04T01:30:00Z",
            }
        ])

        with self.subTest("outbound workflow with TO_PICK"):
            inventory = FakeInventory(
                stocks=[], active_operations=[_pallet3_operation("TO_PICK")]
            )
            self.store.create_or_get_pallet3_workflow("operation-1", "robot_1")
            self.store.transition_pallet3_workflow(
                "operation-1", "PICK_PENDING", "OUTBOUND_SENT"
            )
            service = OrchestratorService(self.store, inventory, fleet)

            with self.assertRaisesRegex(ValueError, "not ready for pallet 3 recover"):
                service.recover_pallet3_operation("operation-1")

            self.assertEqual(fleet.commands, [])

        with self.subTest("manual pick marked but still TO_PICK"):
            operation = _pallet3_operation("TO_PICK")
            operation["operation_id"] = "operation-2"
            inventory = FakeInventory(
                stocks=[], active_operations=[operation]
            )
            self.store.create_or_get_pallet3_workflow("operation-2", "robot_1")
            self.store.confirm_pallet3_manual_pick("operation-2")
            service = OrchestratorService(self.store, inventory, fleet)

            with self.assertRaisesRegex(ValueError, "not ready for pallet 3 recover"):
                service.recover_pallet3_operation("operation-2")

            self.assertEqual(fleet.commands, [])

    def test_manual_pallet3_recovery_requires_an_existing_workflow(self) -> None:
        # This catches recovery manufacturing a new pre-Fork DOWN workflow after state was lost.
        inventory = FakeInventory(
            stocks=[], active_operations=[_pallet3_operation("TO_PLACE")]
        )
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        with self.assertRaisesRegex(ValueError, "workflow was not found"):
            service.recover_pallet3_operation("operation-1")

        self.assertIsNone(self.store.get_pallet3_workflow("operation-1"))
        self.assertFalse(self.store.recovery_required("operation-1"))
        self.assertEqual(fleet.commands, [])

    def test_manual_pallet3_recovery_refuses_unsafe_inventory_state(self) -> None:
        # This catches recovery advancing a transfer while Inventory is in an intermediate state.
        inventory = FakeInventory(
            stocks=[], active_operations=[_pallet3_operation("PICKING")]
        )
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"}
        ])
        self.store.create_or_get_pallet3_workflow("operation-1", "robot_1")
        service = OrchestratorService(self.store, inventory, fleet)

        with self.assertRaisesRegex(ValueError, "not ready for pallet 3 recover"):
            service.recover_pallet3_operation("operation-1")

        self.assertFalse(self.store.recovery_required("operation-1"))
        self.assertEqual(fleet.commands, [])

    def test_manual_pallet3_recovery_requires_operator_ready(self) -> None:
        # This catches recover reissuing a route while the vehicle may still be moving.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation()])
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "DRIVE", "detail": "NAVIGATION_STARTED"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.confirm_pallet3_manual_pick("operation-1", True)

        with self.assertRaisesRegex(ValueError, "OPERATOR_READY"):
            service.recover_pallet3_operation("operation-1")
        self.assertEqual(fleet.commands, [])

    def test_pallet3_fork_down_failure_stops_without_reissuing_the_fork_command(self) -> None:
        # This catches automatic retry of a Fork DOWN whose physical execution is unknown.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation("TO_PLACE")])
        self.store.create_or_get_pallet3_workflow("operation-1", "robot_1")
        self.store.transition_pallet3_workflow(
            "operation-1", "PICK_PENDING", "OUTBOUND_SENT"
        )
        self.store.transition_pallet3_workflow(
            "operation-1", "OUTBOUND_SENT", "FORK_DOWN_SENT"
        )
        fleet = FakeFleet([
            {"robot_id": "robot_1", "state": "FAIL", "detail": "FORK_DOWN_TIMEOUT"}
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.reconcile()
        fleet.vehicles[0] = {
            "robot_id": "robot_1", "state": "WAIT", "detail": "OPERATOR_READY"
        }
        service.reconcile()

        workflow = self.store.get_pallet3_workflow("operation-1")
        self.assertEqual(workflow.phase, "FAILED")
        self.assertEqual(workflow.failure_detail, "FORK_DOWN_TIMEOUT")
        self.assertEqual(fleet.commands, [("stop", "robot_1", {})])

    def test_pallet3_ignores_navigation_and_fork_events_for_other_operations(self) -> None:
        # This catches a stale vehicle report changing another pallet's inventory or fork state.
        inventory = FakeInventory(stocks=[], active_operations=[_pallet3_operation("TO_PLACE")])
        self.store.create_or_get_pallet3_workflow("operation-1", "robot_1")
        self.store.transition_pallet3_workflow(
            "operation-1", "PICK_PENDING", "OUTBOUND_SENT"
        )
        fleet = FakeFleet([
            {
                "robot_id": "robot_1",
                "state": "WAIT",
                "operation_id": "other-operation",
                "source": "NAV2",
                "detail": "NAVIGATION_SUCCEEDED",
            }
        ])
        service = OrchestratorService(self.store, inventory, fleet)

        service.reconcile()
        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "operation_id": "operation-1",
            "source": "NAV2",
            "detail": "NAVIGATION_SUCCEEDED",
        }
        service.reconcile()
        fleet.vehicles[0] = {
            "robot_id": "robot_1",
            "state": "WAIT",
            "operation_id": "other-operation",
            "source": "FORK",
            "detail": "FORK_DOWN_COMPLETE",
        }
        service.reconcile()

        self.assertEqual([command[0] for command in fleet.commands], ["fork_down"])
        self.assertEqual(inventory.place_completion_keys, [])
        self.assertEqual(self.store.get_pallet3_workflow("operation-1").phase, "FORK_DOWN_SENT")

    def test_pallet3_mission_runs_waypoints_then_confirmed_unload_reverse_and_return(self) -> None:
        inventory = FakeInventory(stocks=[], active_operations=[])
        vehicle = {
            "robot_id": "robot_1", "state": "WAIT",
            "operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
            "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
        }
        fleet = FakeFleet([vehicle])
        service = OrchestratorService(self.store, inventory, fleet)

        mission = service.start_pallet3_mission("robot_1")

        self.assertEqual(mission.phase, "OUTBOUND_SENT")
        self.assertEqual(mission.mission_id, vehicle["operation_id"])
        self.assertEqual(fleet.commands[0][0:2], ("navigate_waypoints", "robot_1"))
        self.assertEqual(
            fleet.commands[0][2]["waypoints"],
            [
                {"frame_id": "map", "x": -0.44, "y": -0.9, "yaw": 0.0},
                {"frame_id": "map", "x": -0.44, "y": -1.69, "yaw": -1.5707963267948966},
                {"frame_id": "map", "x": -0.44, "y": -2.34, "yaw": -1.5707963267948966},
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

    def test_pallet3_mission_bypasses_pick_completion_only_when_explicitly_requested(self) -> None:
        # This catches requiring an unavailable Auto Dock PICK completion in the explicit POC bypass.
        inventory = FakeInventory(stocks=[], active_operations=[])
        fleet = FakeFleet([{
            "robot_id": "robot_1", "state": "WAIT",
            "source": "API", "detail": "OPERATOR_READY",
        }])

        mission = OrchestratorService(self.store, inventory, fleet).start_pallet3_mission(
            "robot_1", bypass_pick=True
        )

        self.assertEqual(mission.phase, "OUTBOUND_SENT")
        self.assertEqual(fleet.commands[0][0:2], ("navigate_waypoints", "robot_1"))
        self.assertEqual(fleet.commands[0][2]["operation_id"], mission.mission_id)
        self.assertEqual(
            fleet.commands[0][2]["waypoints"],
            [
                {"frame_id": "map", "x": -0.44, "y": -0.9, "yaw": 0.0},
                {"frame_id": "map", "x": -0.44, "y": -1.69, "yaw": -1.5707963267948966},
                {"frame_id": "map", "x": -0.44, "y": -2.34, "yaw": -1.5707963267948966},
            ],
        )

    def test_new_pallet3_poc_mission_supersedes_an_idle_bypass_mission(self) -> None:
        # This catches repeated POC execution reusing a stale mission instead of sending a fresh route.
        inventory = FakeInventory(stocks=[], active_operations=[])
        fleet = FakeFleet([{
            "robot_id": "robot_1", "state": "WAIT",
            "source": "API", "detail": "OPERATOR_READY",
        }])
        service = OrchestratorService(self.store, inventory, fleet)
        original = service.start_pallet3_mission("robot_1", bypass_pick=True)

        replacement = service.start_pallet3_mission(
            "robot_1", bypass_pick=True, new_mission=True
        )

        replaced = self.store.get_pallet3_mission(original.mission_id)
        self.assertNotEqual(replacement.mission_id, original.mission_id)
        self.assertEqual(replaced.phase, "FAILED")
        self.assertEqual(replaced.failure_detail, "SUPERSEDED_BY_NEW_POC_REQUEST")
        self.assertEqual(
            [command[2]["operation_id"] for command in fleet.commands],
            [original.mission_id, replacement.mission_id],
        )

    def test_new_pallet3_poc_mission_requires_explicit_pick_bypass(self) -> None:
        # This catches replacing an Auto Dock-linked mission without a safe new operation identity.
        inventory = FakeInventory(stocks=[], active_operations=[])
        fleet = FakeFleet([{
            "robot_id": "robot_1", "state": "WAIT",
            "operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
            "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
        }])

        with self.assertRaisesRegex(ValueError, "only with bypass_pick"):
            OrchestratorService(self.store, inventory, fleet).start_pallet3_mission(
                "robot_1", new_mission=True
            )

        self.assertEqual(fleet.commands, [])

    def test_pallet3_mission_requires_arrival_confirmation_and_ignores_stale_fork_event(self) -> None:
        inventory = FakeInventory(stocks=[_stock("docker", "FRESH", 1)], active_operations=[])
        fleet = FakeFleet([{
            "robot_id": "robot_1", "state": "WAIT",
            "operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
            "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
        }])
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
            [{
                "robot_id": "robot_1", "state": "WAIT",
                "operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
                "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
            }],
            timeout=True,
        )

        mission = OrchestratorService(self.store, inventory, fleet).start_pallet3_mission("robot_1")

        self.assertEqual(mission.phase, "FAILED")
        self.assertIn("POC_OUTBOUND_WAYPOINTS", mission.failure_detail)
        self.assertEqual(fleet.commands, [("stop", "robot_1", {})])

    def test_pallet3_mission_fails_and_stops_on_fork_failure(self) -> None:
        inventory = FakeInventory(stocks=[], active_operations=[])
        fleet = FakeFleet([{
            "robot_id": "robot_1", "state": "WAIT",
            "operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
            "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
        }])
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
