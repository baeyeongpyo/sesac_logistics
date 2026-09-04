"""Direct Inventory operation creation for a Pallet 3 delivery."""

from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from typing import Any, Protocol
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class RunnerError(RuntimeError):
    """Base error raised by the direct runner."""


class TransportError(RunnerError):
    """An HTTP request could not complete successfully."""


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
    inventory_url: str = "http://192.168.100.27:8081"
    fleet_url: str = "http://192.168.100.27:8090"
    request_timeout_sec: float = 1.0


class DirectPallet3Runner:
    def __init__(
        self,
        config: RunnerConfig,
        transport: ApiTransport | None = None,
        sleep: Any = None,
    ) -> None:
        self.config = config
        self._transport = transport or UrllibTransport()
        self._sleep = sleep or time.sleep

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
            active_operations = self._request(
                "GET", self._inventory("/api/v1/operations/active"), None
            ).body
            vehicle = self._request(
                "GET", self._fleet(f"/api/v1/vehicles/{self.config.robot_id}"), None
            ).body
            if (
                self._has_active_operation(
                    active_operations, operation_id, expected_inventory_status
                )
                and vehicle.get("operation_id") == operation_id
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

    def _require_vehicle_wait(self) -> None:
        response = self._request(
            "GET", self._fleet(f"/api/v1/vehicles/{self.config.robot_id}"), None
        )
        if response.body.get("state") != "WAIT":
            raise RunnerError("vehicle is not ready: expected state WAIT")

    def _request(self, method: str, url: str, body: dict[str, Any] | None) -> ApiResponse:
        return self._transport.request(method, url, body, self.config.request_timeout_sec)

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
    def _has_active_operation(
        active_operations: Any, operation_id: str, expected_status: str
    ) -> bool:
        return isinstance(active_operations, list) and any(
            operation.get("operation_id") == operation_id
            and operation.get("status") == expected_status
            for operation in active_operations
            if isinstance(operation, dict)
        )

    @staticmethod
    def _required_string(body: dict[str, Any], key: str) -> str:
        value = body.get(key)
        if not isinstance(value, str) or not value:
            raise RunnerError(f"response is missing required string field: {key}")
        return value
