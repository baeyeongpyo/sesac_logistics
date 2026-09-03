import json
import unittest

import httpx

from logistics_orchestrator.app.clients import (
    HttpFleetManagerClient,
    HttpInventoryClient,
)


class HttpClientContractTest(unittest.TestCase):
    def test_inventory_snapshot_reads_zones_stocks_and_active_operations(self) -> None:
        # This catches planner decisions using only one stale Inventory resource.
        paths: list[str] = []

        def respond(request: httpx.Request) -> httpx.Response:
            paths.append(request.url.path)
            return httpx.Response(200, json=[])

        client = httpx.Client(transport=httpx.MockTransport(respond))
        inventory = HttpInventoryClient("http://inventory", client=client)

        snapshot = inventory.snapshot()

        self.assertEqual(paths, ["/api/v1/zones", "/api/v1/stocks", "/api/v1/operations/active"])
        self.assertEqual(snapshot, {"zones": [], "stocks": [], "active_operations": []})

    def test_fleet_manager_navigation_uses_the_command_endpoint_and_body(self) -> None:
        # This catches bypassing Fleet Manager or losing operation identity on Nav2 commands.
        received: dict = {}

        def respond(request: httpx.Request) -> httpx.Response:
            received["path"] = request.url.path
            received["body"] = json.loads(request.content)
            return httpx.Response(202)

        client = httpx.Client(transport=httpx.MockTransport(respond))
        fleet = HttpFleetManagerClient("http://fleet", client=client)
        command = {"operation_id": "operation-1", "purpose": "PICK", "frame_id": "map", "x": 1, "y": 2, "yaw": 0}

        fleet.navigate("robot_1", command)

        self.assertEqual(
            received["path"],
            "/api/v1/vehicles/robot_1/commands/navigation/goals",
        )
        self.assertEqual(received["body"], command)

    def test_fleet_manager_auto_dock_uses_the_command_endpoint_and_body(self) -> None:
        received: dict = {}

        def respond(request: httpx.Request) -> httpx.Response:
            received["path"] = request.url.path
            received["body"] = json.loads(request.content)
            return httpx.Response(202)

        client = httpx.Client(transport=httpx.MockTransport(respond))
        fleet = HttpFleetManagerClient("http://fleet", client=client)
        command = {"operation_id": "operation-1", "operation": "PICK"}

        fleet.auto_dock("robot_1", command)

        self.assertEqual(
            received["path"],
            "/api/v1/vehicles/robot_1/commands/auto-dock",
        )
        self.assertEqual(received["body"], command)

    def test_fleet_manager_poc_controls_use_only_fleet_manager_command_endpoints(self) -> None:
        received: list[tuple[str, dict]] = []

        def respond(request: httpx.Request) -> httpx.Response:
            received.append((request.url.path, json.loads(request.content)))
            return httpx.Response(202)

        client = httpx.Client(transport=httpx.MockTransport(respond))
        fleet = HttpFleetManagerClient("http://fleet", client=client)
        mission_id = "3e829a02-7601-4b9f-afdf-3dbd84737828"
        fleet.navigate_waypoints("robot_1", {"operation_id": mission_id, "waypoints": []})
        fleet.fork_down("robot_1", {"operation_id": mission_id})
        fleet.command_velocity(
            "robot_1",
            {"operation_id": mission_id, "linear_x": -0.18, "angular_z": 0.0, "hold_ms": 1000},
        )
        fleet.stop("robot_1")

        self.assertEqual(
            received,
            [
                ("/api/v1/vehicles/robot_1/commands/navigation/waypoints", {"operation_id": mission_id, "waypoints": []}),
                ("/api/v1/vehicles/robot_1/commands/fork/down", {"operation_id": mission_id}),
                ("/api/v1/vehicles/robot_1/commands/cmd-vel", {
                    "operation_id": mission_id, "linear_x": -0.18, "angular_z": 0.0, "hold_ms": 1000,
                }),
                ("/api/v1/vehicles/robot_1/commands/stop", {}),
            ],
        )

    def test_fleet_manager_command_service_unavailable_is_delivery_unknown(self) -> None:
        client = httpx.Client(
            transport=httpx.MockTransport(
                lambda _: httpx.Response(503, json={"detail": "bridge timed out"})
            )
        )
        fleet = HttpFleetManagerClient("http://fleet", client=client)

        with self.assertRaises(TimeoutError):
            fleet.navigate("robot_1", {"operation_id": "operation-1"})

    def test_fleet_manager_returns_latest_vehicle_snapshots(self) -> None:
        # This catches deciding from an event payload instead of the Fleet Manager ledger.
        client = httpx.Client(
            transport=httpx.MockTransport(
                lambda _: httpx.Response(200, json=[{"robot_id": "robot_1", "state": "WAIT"}])
            )
        )

        vehicles = HttpFleetManagerClient("http://fleet", client=client).list_vehicles()

        self.assertEqual(vehicles, [{"robot_id": "robot_1", "state": "WAIT"}])


if __name__ == "__main__":
    unittest.main()
