"""Direct Inventory operation creation for a Pallet 3 delivery."""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, Protocol
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class RunnerError(RuntimeError):
    """Base error raised by the direct runner."""


class TransportError(RunnerError):
    """An HTTP request could not complete successfully."""


class CommunicationLost(RunnerError):
    """Inventory connectivity stayed unavailable past the safety grace period."""


class OperationStateMismatch(RunnerError):
    """The tracked Inventory operation is absent or in an unexpected state."""

    def __init__(
        self, operation_id: str, expected_status: str, operation: Any
    ) -> None:
        actual_status = operation.get("status") if isinstance(operation, dict) else None
        super().__init__(
            f"operation {operation_id} expected active status {expected_status}, "
            f"got {actual_status!r}"
        )


class FleetFailure(RunnerError):
    """The Fleet snapshot reports failure for the tracked operation."""


class RunnerInterrupted(RunnerError):
    """The direct runner was interrupted after a best-effort vehicle stop."""


@dataclass(frozen=True)
class ApiResponse:
    status: int
    body: Any


class ApiTransport(Protocol):
    def request(
        self,
        method: str,
        url: str,
        body: dict[str, Any] | None,
        timeout_sec: float,
    ) -> ApiResponse:
        """Issue one JSON HTTP request."""


class UrllibTransport:
    """Standard-library HTTP transport for the Fleet and Inventory APIs."""

    def request(
        self,
        method: str,
        url: str,
        body: dict[str, Any] | None,
        timeout_sec: float,
    ) -> ApiResponse:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = Request(
            url,
            data=data,
            method=method,
            headers={"Accept": "application/json", "Content-Type": "application/json"},
        )
        try:
            with urlopen(request, timeout=timeout_sec) as response:
                raw_body = response.read()
                try:
                    parsed_body = json.loads(raw_body.decode("utf-8")) if raw_body else {}
                except (UnicodeDecodeError, json.JSONDecodeError) as error:
                    raise TransportError(f"invalid JSON response from {url}") from error
                if not isinstance(parsed_body, (dict, list)):
                    raise TransportError(f"JSON response from {url} must be an object or array")
                return ApiResponse(status=response.status, body=parsed_body)
        except HTTPError as error:
            raise TransportError(f"HTTP {error.code} from {url}") from error
        except (URLError, TimeoutError, socket.timeout) as error:
            raise TransportError(f"request to {url} failed: {error}") from error


@dataclass(frozen=True)
class RunnerConfig:
    robot_id: str
    inventory_url: str = "http://127.0.0.1:8081"
    fleet_url: str = "http://127.0.0.1:8090"
    request_timeout_sec: float = 1.0


class DirectPallet3Runner:
    def __init__(
        self,
        config: RunnerConfig,
        transport: ApiTransport | None = None,
        sleep: Callable[[float], None] | None = None,
        monotonic: Callable[[], float] | None = None,
    ) -> None:
        self.config = config
        self._transport = transport or UrllibTransport()
        self._sleep = sleep or time.sleep
        self._monotonic = monotonic or time.monotonic
        self._inventory_outage_started_at: float | None = None
        self._stop_requested = False

    def create_operation(self) -> str:
        self._require_vehicle_wait()
        response = self._request(
            "POST",
            self._inventory("/api/v1/operations"),
            {
                "robot_id": self.config.robot_id,
                "payload_type": "NORMAL",
                "source_zone_id": "docker",
                "destination_zone_id": "p3",
                "priority": 0,
            },
        )
        return self._required_string(response.body, "operation_id")

    def send_auto_dock_pick(self, operation_id: str) -> None:
        self._post_fleet("/commands/auto-dock", {
            "operation_id": operation_id,
            "operation": "PICK",
            "product_type": "NORMAL",
            "location": "DOCK_1",
            "target": {"type": "NEAREST"},
        })

    def wait_for_report(
        self, operation_id: str, detail: str, expected_inventory_status: str
    ) -> None:
        expected_source = {
            "AUTO_DOCK_PICK_COMPLETED": "AUTO_DOCK",
            "NAVIGATION_SUCCEEDED": "NAV2",
            "FORK_DOWN_COMPLETE": "FORK",
            "MANUAL_COMMAND_EXPIRED": "API",
        }.get(detail)
        if expected_source is None:
            raise RunnerError(f"unsupported completion detail: {detail}")

        while True:
            self._require_active_operation(operation_id, expected_inventory_status)
            vehicle = self._require_object(
                self._request(
                    "GET",
                    self._fleet(f"/api/v1/vehicles/{self.config.robot_id}"),
                    None,
                ).body,
                "Fleet vehicle snapshot",
            )
            if (
                vehicle.get("operation_id") == operation_id
                and vehicle.get("state") == "FAIL"
            ):
                raise FleetFailure(
                    f"vehicle {self.config.robot_id} reported FAIL for operation "
                    f"{operation_id}"
                )
            if (
                vehicle.get("operation_id") == operation_id
                and vehicle.get("source") == expected_source
                and vehicle.get("detail") == detail
            ):
                return
            self._sleep(0.5)

    def complete_pick(self, operation_id: str) -> None:
        self._post_inventory(f"/api/v1/operations/{operation_id}/pick-completions", {
            "robot_id": self.config.robot_id,
            "idempotency_key": f"{operation_id}:pick",
        })

    def send_outbound_route(self, operation_id: str) -> None:
        self._post_fleet("/commands/navigation/waypoints", {
            "operation_id": operation_id,
            "purpose": "PLACE",
            "waypoints": [
                {"frame_id": "map", "x": -0.440, "y": -0.900, "yaw": 0.0},
                {"frame_id": "map", "x": -0.440, "y": -1.690, "yaw": -1.5707963267948966},
                {"frame_id": "map", "x": -0.440, "y": -2.340, "yaw": -1.5707963267948966},
            ],
        })

    def send_fork_down(self, operation_id: str) -> None:
        self._post_fleet("/commands/fork/down", {"operation_id": operation_id})

    def send_reverse(self, operation_id: str) -> None:
        self._post_fleet("/commands/cmd-vel", {
            "operation_id": operation_id,
            "linear_x": -0.18,
            "linear_y": 0.0,
            "angular_z": 0.0,
            "hold_ms": 1000,
        })

    def complete_place(self, operation_id: str) -> None:
        self._post_inventory(f"/api/v1/operations/{operation_id}/place-completions", {
            "robot_id": self.config.robot_id,
            "idempotency_key": f"{operation_id}:place",
        })

    def send_return_route(self) -> None:
        self._post_fleet("/commands/navigation/waypoints", {
            "purpose": "PLACE",
            "waypoints": [
                {"frame_id": "map", "x": -0.420, "y": -2.000, "yaw": -1.5707963268},
                {"frame_id": "map", "x": -0.440, "y": -0.900, "yaw": 0.0},
                {"frame_id": "map", "x": 0.085, "y": -0.905, "yaw": 0.0},
            ],
        })

    def run(self) -> None:
        operation_id = self.create_operation()
        self.send_auto_dock_pick(operation_id)
        self.wait_for_report(operation_id, "AUTO_DOCK_PICK_COMPLETED", "TO_PICK")
        self.complete_pick(operation_id)
        self.send_outbound_route(operation_id)
        self.wait_for_report(operation_id, "NAVIGATION_SUCCEEDED", "TO_PLACE")
        self.send_fork_down(operation_id)
        self.wait_for_report(operation_id, "FORK_DOWN_COMPLETE", "TO_PLACE")
        self.send_reverse(operation_id)
        self.wait_for_report(operation_id, "MANUAL_COMMAND_EXPIRED", "TO_PLACE")
        self.complete_place(operation_id)
        self.send_return_route()

    def request_stop(self, _signum: int | None = None, _frame: Any = None) -> None:
        self._best_effort_stop()
        raise RunnerInterrupted("direct pallet 3 runner interrupted")

    def _best_effort_stop(self) -> None:
        if not self._stop_requested:
            self._stop_requested = True
            try:
                self._post_fleet("/commands/stop", {})
            except Exception:
                pass

    def _require_vehicle_wait(self) -> None:
        vehicle = self._require_object(
            self._request(
                "GET", self._fleet(f"/api/v1/vehicles/{self.config.robot_id}"), None
            ).body,
            "Fleet vehicle snapshot",
        )
        if vehicle.get("state") != "WAIT":
            raise RunnerError("vehicle is not ready: expected state WAIT")

    def _require_active_operation(
        self, operation_id: str, expected_status: str
    ) -> None:
        while True:
            try:
                operations = self._request(
                    "GET", self._inventory("/api/v1/operations/active"), None
                ).body
            except TransportError as error:
                self._record_inventory_outage(error)
                continue

            self._inventory_outage_started_at = None
            if not isinstance(operations, list):
                raise RunnerError("Inventory active operations response must be an array")
            operation = next(
                (
                    item
                    for item in operations
                    if isinstance(item, dict)
                    and item.get("operation_id") == operation_id
                ),
                None,
            )
            if operation is None or operation.get("status") != expected_status:
                raise OperationStateMismatch(operation_id, expected_status, operation)
            return

    def _record_inventory_outage(self, error: TransportError) -> None:
        now = self._monotonic()
        if self._inventory_outage_started_at is None:
            self._inventory_outage_started_at = now
        if now - self._inventory_outage_started_at >= 5.0:
            self._best_effort_stop()
            raise CommunicationLost(
                f"Inventory communication lost for 5.0 seconds: {error}"
            ) from error
        self._sleep(0.5)

    def _request(self, method: str, url: str, body: dict[str, Any] | None) -> ApiResponse:
        response = self._transport.request(
            method, url, body, self.config.request_timeout_sec
        )
        if not 200 <= response.status < 300:
            raise TransportError(f"HTTP {response.status} from {url}")
        return response

    def _post_inventory(self, path: str, body: dict[str, Any]) -> None:
        self._request("POST", self._inventory(path), body)

    def _post_fleet(self, path: str, body: dict[str, Any]) -> None:
        self._request(
            "POST",
            self._fleet(f"/api/v1/vehicles/{self.config.robot_id}{path}"),
            body,
        )

    def _inventory(self, path: str) -> str:
        return f"{self.config.inventory_url.rstrip('/')}{path}"

    def _fleet(self, path: str) -> str:
        return f"{self.config.fleet_url.rstrip('/')}{path}"

    @staticmethod
    def _require_object(body: Any, context: str) -> dict[str, Any]:
        if not isinstance(body, dict):
            raise RunnerError(f"{context} response must be a JSON object")
        return body

    @classmethod
    def _required_string(cls, body: Any, key: str) -> str:
        body = cls._require_object(body, "API")
        value = body.get(key)
        if not isinstance(value, str) or not value:
            raise RunnerError(f"response is missing required string field: {key}")
        return value


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one direct NORMAL pallet delivery from docker to P3."
    )
    parser.add_argument("--robot-id", required=True, help="Fleet robot identifier")
    parser.add_argument(
        "--inventory-url",
        default=RunnerConfig.inventory_url,
        help="Inventory API base URL (default: %(default)s)",
    )
    parser.add_argument(
        "--fleet-url",
        default=RunnerConfig.fleet_url,
        help="Fleet Manager API base URL (default: %(default)s)",
    )
    parser.add_argument(
        "--request-timeout-sec",
        type=float,
        default=RunnerConfig.request_timeout_sec,
        help="HTTP request timeout in seconds (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    runner = DirectPallet3Runner(
        RunnerConfig(
            robot_id=args.robot_id,
            inventory_url=args.inventory_url,
            fleet_url=args.fleet_url,
            request_timeout_sec=args.request_timeout_sec,
        )
    )
    signal.signal(signal.SIGINT, runner.request_stop)
    signal.signal(signal.SIGTERM, runner.request_stop)
    try:
        runner.run()
    except RunnerError as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
