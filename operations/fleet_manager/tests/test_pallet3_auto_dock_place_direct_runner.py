import importlib.util
import unittest
from dataclasses import dataclass
from typing import Any
from urllib.parse import urlparse

import pallet3_direct_runner as base_runner_module


@dataclass(frozen=True)
class RecordedCall:
    method: str
    path: str
    body: dict[str, Any] | None


class ScriptedTransport:
    def __init__(self, responses: list[base_runner_module.ApiResponse]):
        self._responses = responses
        self.calls: list[RecordedCall] = []

    def request(
        self,
        method: str,
        url: str,
        body: dict[str, Any] | None,
        timeout_sec: float,
    ) -> base_runner_module.ApiResponse:
        del timeout_sec
        self.calls.append(RecordedCall(method, url, body))
        return self._responses.pop(0)

    def post_paths(self) -> list[str]:
        return [urlparse(call.path).path for call in self.calls if call.method == "POST"]

    def body_for(self, path: str) -> dict[str, Any] | None:
        return next(
            call.body for call in self.calls if urlparse(call.path).path == path
        )

    def last_body_for(self, path: str) -> dict[str, Any] | None:
        return next(
            call.body
            for call in reversed(self.calls)
            if urlparse(call.path).path == path
        )


def response(status: int, body: Any) -> base_runner_module.ApiResponse:
    return base_runner_module.ApiResponse(status=status, body=body)


class AutoDockPlacePallet3RunnerTests(unittest.TestCase):
    def test_auto_dock_place_runner_module_is_available(self) -> None:
        self.assertIsNotNone(
            importlib.util.find_spec("pallet3_auto_dock_place_direct_runner")
        )

    def test_run_returns_to_docker_after_auto_dock_place_completion(self) -> None:
        runner_module = __import__("pallet3_auto_dock_place_direct_runner")
        runner_class = getattr(runner_module, "AutoDockPlacePallet3Runner", None)
        self.assertIsNotNone(runner_class)

        operation_id = "op-1"
        transport = ScriptedTransport([
            response(200, {"state": "WAIT"}),
            response(201, {"operation_id": operation_id}),
            response(202, {}),
            response(200, [{"operation_id": operation_id, "status": "TO_PICK"}]),
            response(200, {
                "operation_id": operation_id,
                "source": "AUTO_DOCK",
                "detail": "AUTO_DOCK_PICK_COMPLETED",
            }),
            response(200, {}),
            response(202, {}),
            response(200, [{"operation_id": operation_id, "status": "TO_PLACE"}]),
            response(200, {
                "operation_id": operation_id,
                "source": "NAV2",
                "detail": "NAVIGATION_SUCCEEDED",
            }),
            response(202, {}),
            response(200, [{"operation_id": operation_id, "status": "TO_PLACE"}]),
            response(200, {
                "operation_id": operation_id,
                "source": "AUTO_DOCK",
                "detail": "AUTO_DOCK_PLACE_COMPLETED",
            }),
            response(200, {}),
            response(202, {}),
        ])
        runner = runner_class(
            base_runner_module.RunnerConfig(
                robot_id="robot_1",
                inventory_url="http://inventory.example",
                fleet_url="http://fleet.example",
            ),
            transport,
            sleep=lambda _: None,
        )

        runner.run()

        self.assertEqual(transport.post_paths(), [
            "/api/v1/operations",
            "/api/v1/vehicles/robot_1/commands/auto-dock",
            "/api/v1/operations/op-1/pick-completions",
            "/api/v1/vehicles/robot_1/commands/navigation/waypoints",
            "/api/v1/vehicles/robot_1/commands/auto-dock",
            "/api/v1/operations/op-1/place-completions",
            "/api/v1/vehicles/robot_1/commands/navigation/waypoints",
        ])
        auto_dock_path = "/api/v1/vehicles/robot_1/commands/auto-dock"
        self.assertEqual(transport.last_body_for(auto_dock_path), {
            "operation_id": operation_id,
            "operation": "PLACE",
            "product_type": "FRESH",
            "location": "Y",
            "target": {"type": "NEAREST"},
        })
        outbound_path = "/api/v1/vehicles/robot_1/commands/navigation/waypoints"
        self.assertEqual(transport.body_for(outbound_path)["waypoints"], [
            {"frame_id": "map", "x": -0.440, "y": -0.900, "yaw": 0.0},
            {
                "frame_id": "map",
                "x": -0.440,
                "y": -1.690,
                "yaw": -1.5707963267948966,
            },
        ])
        self.assertNotIn(
            "/api/v1/vehicles/robot_1/commands/fork/down",
            transport.post_paths(),
        )
        self.assertNotIn(
            "/api/v1/vehicles/robot_1/commands/cmd-vel",
            transport.post_paths(),
        )


if __name__ == "__main__":
    unittest.main()
