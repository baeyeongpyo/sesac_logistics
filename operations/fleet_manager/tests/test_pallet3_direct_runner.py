import signal
import unittest
from dataclasses import dataclass
from typing import Any
from unittest.mock import patch
from urllib.parse import urlparse

try:
    from fleet_manager import pallet3_direct_runner as runner_module
except ModuleNotFoundError:
    import pallet3_direct_runner as runner_module

ApiResponse = runner_module.ApiResponse
CommunicationLost = runner_module.CommunicationLost
DirectPallet3Runner = runner_module.DirectPallet3Runner
FleetFailure = runner_module.FleetFailure
OperationStateMismatch = runner_module.OperationStateMismatch
RunnerConfig = runner_module.RunnerConfig
RunnerError = runner_module.RunnerError
RunnerInterrupted = runner_module.RunnerInterrupted
TransportError = runner_module.TransportError
main = runner_module.main
parse_args = runner_module.parse_args


@dataclass(frozen=True)
class RecordedCall:
    method: str
    path: str
    body: dict[str, Any] | None


class ScriptedTransport:
    def __init__(self, responses: list[ApiResponse | Exception]):
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
        result = self._responses.pop(0)
        if isinstance(result, Exception):
            raise result
        return result

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


def response(method: str, path: str, status: int, body: Any) -> ApiResponse:
    del method, path
    return ApiResponse(status=status, body=body)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds


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

    def test_run_does_not_return_when_place_completion_is_not_successful(self):
        runner = self.runner_with_successful_p3_sequence("op-1")
        self.transport._responses[-2] = response("POST", "/place-completions", 500, {})

        with self.assertRaises(RunnerError):
            runner.run()

        self.assertEqual(
            self.transport.post_paths().count(
                "/api/v1/vehicles/robot_1/commands/navigation/waypoints"
            ),
            1,
        )

    def test_navigation_gate_waits_for_matching_to_place_report_before_fork_down(self):
        transport = ScriptedTransport([
            response("POST", "/navigation/waypoints", 202, {}),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": "op-1",
                "source": "FORK",
                "detail": "NAVIGATION_SUCCEEDED",
            }),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": "op-1",
                "source": "NAV2",
                "detail": "NAVIGATION_SUCCEEDED",
            }),
            response("POST", "/fork/down", 202, {}),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        runner.send_outbound_route("op-1")
        runner.wait_for_report("op-1", "NAVIGATION_SUCCEEDED", "TO_PLACE")
        runner.send_fork_down("op-1")

        self.assertEqual(transport.post_paths(), [
            "/api/v1/vehicles/robot_1/commands/navigation/waypoints",
            "/api/v1/vehicles/robot_1/commands/fork/down",
        ])
        self.assertEqual(len(transport.get_paths()), 4)

    def test_navigation_gate_blocks_fork_until_operation_detail_and_to_place_all_match(self):
        transport = ScriptedTransport([
            response("POST", "/navigation/waypoints", 202, {}),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": "other-op",
                "source": "NAV2",
                "detail": "NAVIGATION_SUCCEEDED",
            }),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": "op-1",
                "source": "NAV2",
                "detail": "NAVIGATION_FAILED",
            }),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": "op-1",
                "source": "NAV2",
                "detail": "NAVIGATION_SUCCEEDED",
            }),
            response("POST", "/fork/down", 202, {}),
        ])
        fork_path = "/api/v1/vehicles/robot_1/commands/fork/down"
        blocked_polls = 0

        def assert_fork_is_blocked(_: float) -> None:
            nonlocal blocked_polls
            blocked_polls += 1
            self.assertNotIn(fork_path, transport.post_paths())

        runner = DirectPallet3Runner(config(), transport, sleep=assert_fork_is_blocked)
        runner.send_outbound_route("op-1")
        runner.wait_for_report("op-1", "NAVIGATION_SUCCEEDED", "TO_PLACE")
        runner.send_fork_down("op-1")

        self.assertEqual(blocked_polls, 2)
        self.assertEqual(transport.post_paths(), [
            "/api/v1/vehicles/robot_1/commands/navigation/waypoints",
            fork_path,
        ])

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

    def test_active_operation_outage_allows_five_seconds_then_stops(self):
        clock = FakeClock()
        transport = ScriptedTransport([
            TransportError("inventory unavailable") for _ in range(11)
        ])
        runner = DirectPallet3Runner(
            config(), transport, sleep=clock.sleep, monotonic=clock.monotonic
        )

        with self.assertRaisesRegex(CommunicationLost, "5.0 seconds"):
            runner.wait_for_report("op-1", "NAVIGATION_SUCCEEDED", "TO_PLACE")

        self.assertEqual(clock.sleeps, [0.5] * 10)
        self.assertEqual(transport.get_paths(), [
            "/api/v1/operations/active",
        ] * 11)

    def test_successful_inventory_response_resets_outage_timer(self):
        clock = FakeClock()
        transport = ScriptedTransport([
            TransportError("inventory unavailable"),
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": "other-op", "state": "WAIT",
                "source": "NAV2", "detail": "NAVIGATION_SUCCEEDED",
            }),
            *[TransportError("inventory unavailable") for _ in range(10)],
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PLACE"},
            ]),
            response("GET", "/vehicles/robot_1", 200, {
                "operation_id": "op-1", "state": "WAIT",
                "source": "NAV2", "detail": "NAVIGATION_SUCCEEDED",
            }),
        ])
        runner = DirectPallet3Runner(
            config(), transport, sleep=clock.sleep, monotonic=clock.monotonic
        )

        runner.wait_for_report("op-1", "NAVIGATION_SUCCEEDED", "TO_PLACE")

        self.assertEqual(clock.sleeps, [0.5] * 12)

    def test_active_operation_state_mismatch_stops_immediately(self):
        transport = ScriptedTransport([
            response("GET", "/operations/active", 200, [
                {"operation_id": "op-1", "status": "TO_PICK"},
            ]),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        with self.assertRaises(OperationStateMismatch):
            runner.wait_for_report("op-1", "NAVIGATION_SUCCEEDED", "TO_PLACE")

        self.assertEqual(transport.get_paths(), ["/api/v1/operations/active"])

    def test_matching_fleet_fail_stops_run_before_next_command_and_completion(self):
        runner = self.runner_with_successful_p3_sequence("op-1")
        self.transport._responses[11] = response("GET", "/vehicles/robot_1", 200, {
            "state": "FAIL", "operation_id": "op-1",
            "source": "API", "detail": "API_STOP",
        })

        with self.assertRaises(FleetFailure):
            runner.run()

        self.assertNotIn(
            "/api/v1/vehicles/robot_1/commands/cmd-vel",
            self.transport.post_paths(),
        )
        self.assertNotIn(
            "/api/v1/operations/op-1/place-completions",
            self.transport.post_paths(),
        )

    def test_fleet_snapshot_must_be_an_object(self):
        transport = ScriptedTransport([
            response("GET", "/vehicles/robot_1", 200, []),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        with self.assertRaisesRegex(RunnerError, "Fleet.*object"):
            runner.create_operation()

        self.assertEqual(transport.get_paths(), [
            "/api/v1/vehicles/robot_1",
        ])

    def test_signal_stop_is_best_effort_and_sent_only_once(self):
        transport = ScriptedTransport([
            response("POST", "/commands/stop", 503, {}),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        with self.assertRaises(RunnerInterrupted):
            runner.request_stop(signal.SIGINT, None)
        with self.assertRaises(RunnerInterrupted):
            runner.request_stop(signal.SIGTERM, None)

        self.assertEqual(transport.post_paths(), [
            "/api/v1/vehicles/robot_1/commands/stop",
        ])

    def test_command_delivery_unknown_is_not_retried(self):
        transport = ScriptedTransport([
            response("POST", "/commands/fork/down", 503, {}),
        ])
        runner = DirectPallet3Runner(config(), transport, sleep=lambda _: None)

        with self.assertRaises(TransportError):
            runner.send_fork_down("op-1")

        self.assertEqual(transport.post_paths(), [
            "/api/v1/vehicles/robot_1/commands/fork/down",
        ])

    def test_parse_args_accepts_all_runtime_overrides(self):
        args = parse_args([
            "--robot-id", "robot_2",
            "--inventory-url", "http://inventory.local:18081",
            "--fleet-url", "http://fleet.local:18090",
            "--request-timeout-sec", "2.5",
        ])

        self.assertEqual(args.robot_id, "robot_2")
        self.assertEqual(args.inventory_url, "http://inventory.local:18081")
        self.assertEqual(args.fleet_url, "http://fleet.local:18090")
        self.assertEqual(args.request_timeout_sec, 2.5)

    @patch.object(runner_module.signal, "signal")
    @patch.object(runner_module, "DirectPallet3Runner")
    def test_main_registers_interrupt_handlers(self, runner_type, register_signal):
        runner = runner_type.return_value

        self.assertEqual(main(["--robot-id", "robot_1"]), 0)

        register_signal.assert_any_call(signal.SIGINT, runner.request_stop)
        register_signal.assert_any_call(signal.SIGTERM, runner.request_stop)
        runner.run.assert_called_once_with()
