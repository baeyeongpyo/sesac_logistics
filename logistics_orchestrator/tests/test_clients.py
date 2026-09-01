import json
import unittest

import httpx

from logistics_orchestrator.app.clients import (
    HttpFleetBridgeClient,
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

    def test_bridge_navigation_uses_the_relay_endpoint_and_body(self) -> None:
        # This catches bypassing Fleet Bridge or losing operation identity on Nav2 commands.
        received: dict = {}

        def respond(request: httpx.Request) -> httpx.Response:
            received["path"] = request.url.path
            received["body"] = json.loads(request.content)
            return httpx.Response(202)

        client = httpx.Client(transport=httpx.MockTransport(respond))
        bridge = HttpFleetBridgeClient("http://bridge", client=client)
        command = {"operation_id": "operation-1", "purpose": "PICK", "frame_id": "map", "x": 1, "y": 2, "yaw": 0}

        bridge.navigate("robot_1", command)

        self.assertEqual(received["path"], "/api/v1/vehicle-command/robot_1/navigation/goals")
        self.assertEqual(received["body"], command)

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
