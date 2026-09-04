"""Direct Inventory operation creation for a Pallet 3 delivery."""

from __future__ import annotations

import json
import socket
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
    body: dict[str, Any]


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
                if not isinstance(parsed_body, dict):
                    raise TransportError(f"JSON response from {url} must be an object")
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
        self._sleep = sleep

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

    def _require_vehicle_wait(self) -> None:
        response = self._request(
            "GET", self._fleet(f"/api/v1/vehicles/{self.config.robot_id}"), None
        )
        if response.body.get("state") != "WAIT":
            raise RunnerError("vehicle is not ready: expected state WAIT")

    def _request(self, method: str, url: str, body: dict[str, Any] | None) -> ApiResponse:
        return self._transport.request(method, url, body, self.config.request_timeout_sec)

    def _inventory(self, path: str) -> str:
        return f"{self.config.inventory_url.rstrip('/')}{path}"

    def _fleet(self, path: str) -> str:
        return f"{self.config.fleet_url.rstrip('/')}{path}"

    @staticmethod
    def _required_string(body: dict[str, Any], key: str) -> str:
        value = body.get(key)
        if not isinstance(value, str) or not value:
            raise RunnerError(f"response is missing required string field: {key}")
        return value
