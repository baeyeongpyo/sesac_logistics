import unittest
from dataclasses import dataclass
from typing import Any
from urllib.parse import urlparse

from pallet3_direct_runner import ApiResponse, DirectPallet3Runner, RunnerConfig, RunnerError


@dataclass(frozen=True)
class RecordedCall:
    method: str
    path: str
    body: dict[str, Any] | None


class ScriptedTransport:
    def __init__(self, responses: list[ApiResponse]):
        self._responses = responses
        self.calls: list[RecordedCall] = []

    def request(
        self,
        method: str,
        url: str,
        body: dict[str, Any] | None,
        timeout_sec: float,
    ) -> ApiResponse:
        del timeout_sec
        self.calls.append(RecordedCall(method, url, body))
        return self._responses.pop(0)

    def post_paths(self) -> list[str]:
        return [urlparse(call.path).path for call in self.calls if call.method == "POST"]

    def get_paths(self) -> list[str]:
        return [urlparse(call.path).path for call in self.calls if call.method == "GET"]

    def body_for(self, path: str) -> dict[str, Any] | None:
        for call in self.calls:
            if urlparse(call.path).path == path:
                return call.body
        return None

    def last_body_for(self, path: str) -> dict[str, Any] | None:
        for call in reversed(self.calls):
            if urlparse(call.path).path == path:
                return call.body
        return None


def response(method: str, path: str, status: int, body: dict[str, Any]) -> ApiResponse:
    del method, path
    return ApiResponse(status=status, body=body)


def config() -> RunnerConfig:
    return RunnerConfig(
        robot_id="robot_1",
        inventory_url="http://inventory.example",
        fleet_url="http://fleet.example",
    )


class DirectPallet3RunnerTests(unittest.TestCase):
    def runner_with_successful_p3_sequence(self, operation_id: str) -> DirectPallet3Runner:
        transport = ScriptedTransport([
            response("GET", "/vehicles/robot_1", 200, {"state": "WAIT"}),
            response("POST", "/operations", 201, {"operation_id": operation_id}),
            response("POST", "/auto-dock", 202, {}),
            response("GET", "/operations/active", 200, [
                {"operation_id": operation_id, "status": "TO_PICK"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": operation_id,
                "source": "AUTO_DOCK",
                "detail": "AUTO_DOCK_PICK_COMPLETED",
            }),
            response("POST", "/pick-completions", 200, {}),
            response("POST", "/navigation/waypoints", 202, {}),
            response("GET", "/operations/active", 200, [
                {"operation_id": operation_id, "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": operation_id,
                "source": "NAV2",
                "detail": "NAVIGATION_SUCCEEDED",
            }),
            response("POST", "/fork", 202, {}),
            response("GET", "/operations/active", 200, [
                {"operation_id": operation_id, "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": operation_id,
                "source": "FORK",
                "detail": "FORK_DOWN_COMPLETE",
            }),
            response("POST", "/manual", 202, {}),
            response("GET", "/operations/active", 200, [
                {"operation_id": operation_id, "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": operation_id,
                "source": "API",
                "detail": "MANUAL_COMMAND_EXPIRED",
            }),
            response("POST", "/place-completions", 200, {}),
            response("POST", "/navigation/waypoints", 202, {}),
        ])
        self.transport = transport
        return DirectPallet3Runner(config(), transport, sleep=lambda _: None)

    def test_run_places_after_reverse_then_returns_without_transport_operation_id(self):
        runner = self.runner_with_successful_p3_sequence("op-1")

        runner.run()

        self.assertEqual(self.transport.body_for("/api/v1/operations/op-1/place-completions"), {
            "robot_id": "robot_1", "idempotency_key": "op-1:place",
        })
        return_body = self.transport.last_body_for(
            "/api/v1/vehicles/robot_1/commands/navigation/waypoints"
        )
        self.assertNotIn("operation_id", return_body)
        self.assertEqual(return_body["waypoints"][0], {
            "frame_id": "map", "x": -0.420, "y": -2.000, "yaw": -1.5707963268,
        })

    def test_auto_dock_completion_gates_pick_completion_and_outbound_route(self):
        transport = ScriptedTransport([
            response("POST", "/auto-dock", 202, {}),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PICK"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "state": "WAIT", "operation_id": "op-1",
                "source": "NAV2", "detail": "AUTO_DOCK_PICK_COMPLETED",
            }),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PICK"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "state": "WAIT", "operation_id": "another-op",
                "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
            }),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "state": "WAIT", "operation_id": "op-1",
                "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
            }),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PICK"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "state": "WAIT", "operation_id": "op-1",
                "source": "AUTO_DOCK", "detail": "AUTO_DOCK_PICK_COMPLETED",
            }),
            response("POST", "/pick-completions", 200, {}),
            response("POST", "/navigation/waypoints", 202, {}),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        runner.send_auto_dock_pick("op-1")
        runner.wait_for_report("op-1", "AUTO_DOCK_PICK_COMPLETED", "TO_PICK")
        runner.complete_pick("op-1")
        runner.send_outbound_route("op-1")

        self.assertEqual(transport.post_paths(), [
            "/api/v1/vehicles/robot_1/commands/auto-dock",
            "/api/v1/operations/op-1/pick-completions",
            "/api/v1/vehicles/robot_1/commands/navigation/waypoints",
        ])
        self.assertEqual(transport.get_paths(), [
            "/api/v1/operations/active", "/api/v1/vehicles/robot_1",
            "/api/v1/operations/active", "/api/v1/vehicles/robot_1",
            "/api/v1/operations/active", "/api/v1/vehicles/robot_1",
            "/api/v1/operations/active", "/api/v1/vehicles/robot_1",
        ])
        self.assertEqual(transport.calls[-1].body, {
            "operation_id": "op-1",
            "purpose": "PLACE",
            "waypoints": [
                {"frame_id": "map", "x": -0.440, "y": -0.900, "yaw": 0.0},
                {"frame_id": "map", "x": -0.440, "y": -1.690, "yaw": -1.5707963267948966},
                {"frame_id": "map", "x": -0.440, "y": -2.340, "yaw": -1.5707963267948966},
            ],
        })

    def test_allows_wait_vehicle_with_existing_operation_id(self):
        transport = ScriptedTransport([
            response("GET", "/api/v1/vehicles/robot_1", 200, {
                "state": "WAIT", "operation_id": "existing-op",
                "source": "API", "detail": "OPERATOR_READY",
            }),
            response("POST", "/api/v1/operations", 201, {"operation_id": "op-2"}),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        self.assertEqual(runner.create_operation(), "op-2")
        self.assertEqual(transport.calls[1], RecordedCall(
            "POST", "http://inventory.example/api/v1/operations", {
                "robot_id": "robot_1", "payload_type": "NORMAL",
                "source_zone_id": "docker", "destination_zone_id": "p3", "priority": 0,
            },
        ))

    def test_creates_normal_docker_to_p3_operation_after_wait_state_check(self):
        transport = ScriptedTransport([
            response("GET", "/api/v1/vehicles/robot_1", 200, {
                "state": "WAIT", "operation_id": None,
                "source": "API", "detail": "OPERATOR_READY",
            }),
            response("POST", "/api/v1/operations", 201, {"operation_id": "op-1"}),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        self.assertEqual(runner.create_operation(), "op-1")
        self.assertEqual(transport.calls[0], RecordedCall(
            "GET", "http://fleet.example/api/v1/vehicles/robot_1", None,
        ))
        self.assertEqual(transport.calls[1], RecordedCall(
            "POST", "http://inventory.example/api/v1/operations", {
                "robot_id": "robot_1", "payload_type": "NORMAL",
                "source_zone_id": "docker", "destination_zone_id": "p3", "priority": 0,
            },
        ))

    def test_rejects_non_wait_vehicle_without_creating_operation(self):
        transport = ScriptedTransport([
            response("GET", "/api/v1/vehicles/robot_1", 200, {
                "state": "DRIVING", "operation_id": "existing-op",
                "source": "API", "detail": "NAVIGATING",
            }),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        with self.assertRaises(RunnerError):
            runner.create_operation()

        self.assertEqual(transport.calls, [RecordedCall(
            "GET", "http://fleet.example/api/v1/vehicles/robot_1", None,
        )])
