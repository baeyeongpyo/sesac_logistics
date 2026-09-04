import unittest
from dataclasses import dataclass
from typing import Any

from pallet3_direct_runner import ApiResponse, DirectPallet3Runner, RunnerConfig


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
