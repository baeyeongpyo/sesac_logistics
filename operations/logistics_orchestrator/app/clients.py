from __future__ import annotations

from typing import Any

import httpx


class _HttpClient:
    def __init__(self, base_url: str, *, client: httpx.Client | None = None) -> None:
        self._base_url = base_url.rstrip("/")
        self._client = client or httpx.Client(timeout=5)
        self._owns_client = client is None

    def close(self) -> None:
        if self._owns_client:
            self._client.close()

    def _get(self, path: str) -> Any:
        return self._request("GET", path).json()

    def _post(self, path: str, body: dict[str, Any]) -> Any:
        response = self._request("POST", path, json=body)
        return response.json() if response.content else None

    def _request(self, method: str, path: str, **kwargs: Any) -> httpx.Response:
        try:
            response = self._client.request(method, f"{self._base_url}{path}", **kwargs)
        except httpx.TimeoutException as error:
            raise TimeoutError(str(error)) from error
        response.raise_for_status()
        return response


class HttpInventoryClient(_HttpClient):
    def snapshot(self) -> dict[str, list[dict[str, Any]]]:
        return {
            "zones": self._get("/api/v1/zones"),
            "stocks": self._get("/api/v1/stocks"),
            "active_operations": self._get("/api/v1/operations/active"),
        }

    def create_operation(self, body: dict[str, Any]) -> dict[str, Any]:
        return self._post("/api/v1/operations", body)

    def pallet_state(self, robot_id: str) -> dict[str, Any]:
        return self._get(f"/api/v1/robots/{robot_id}/pallet-state")

    def complete_pick(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None:
        self._post(
            f"/api/v1/operations/{operation_id}/pick-completions",
            {"robot_id": robot_id, "idempotency_key": idempotency_key},
        )

    def complete_place(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None:
        self._post(
            f"/api/v1/operations/{operation_id}/place-completions",
            {"robot_id": robot_id, "idempotency_key": idempotency_key},
        )


class HttpFleetManagerClient(_HttpClient):
    def list_vehicles(self) -> list[dict[str, Any]]:
        return self._get("/api/v1/vehicles")

    def navigate(self, robot_id: str, payload: dict[str, Any]) -> None:
        self._post_command(
            f"/api/v1/vehicles/{robot_id}/commands/navigation/goals", payload
        )

    def auto_dock(self, robot_id: str, payload: dict[str, Any]) -> None:
        self._post_command(
            f"/api/v1/vehicles/{robot_id}/commands/auto-dock", payload
        )

    def navigate_waypoints(self, robot_id: str, payload: dict[str, Any]) -> None:
        self._post_command(
            f"/api/v1/vehicles/{robot_id}/commands/navigation/waypoints", payload
        )

    def fork_down(self, robot_id: str, payload: dict[str, Any]) -> None:
        self._post_command(
            f"/api/v1/vehicles/{robot_id}/commands/fork/down", payload
        )

    def command_velocity(self, robot_id: str, payload: dict[str, Any]) -> None:
        self._post_command(
            f"/api/v1/vehicles/{robot_id}/commands/cmd-vel", payload
        )

    def stop(self, robot_id: str) -> None:
        self._post_command(f"/api/v1/vehicles/{robot_id}/commands/stop", {})

    def _post_command(self, path: str, payload: dict[str, Any]) -> None:
        try:
            self._post(path, payload)
        except httpx.HTTPStatusError as error:
            if error.response.status_code == 503:
                raise TimeoutError(error.response.text) from error
            raise
