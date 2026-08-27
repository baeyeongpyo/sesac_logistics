"""Deliver Fleet Manager commands through each vehicle's HTTP command API."""

import asyncio
from collections.abc import Awaitable, Callable, Mapping
import json
import math
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fleet_bridge_config.models import VehicleConfig


class CommandValidationError(ValueError):
    """Raised when a command violates a configured Fleet Manager bound."""


class VehicleCommandApiError(RuntimeError):
    """The vehicle API returned a non-success HTTP response."""

    def __init__(self, status_code: int, detail: str) -> None:
        self.status_code = status_code
        self.detail = detail
        super().__init__(detail)


class VehicleCommandTransportError(RuntimeError):
    """The vehicle API could not be reached or returned invalid JSON."""


def validate_command(
    vehicle: VehicleConfig,
    linear_x: float,
    angular_z: float,
    hold_ms: int,
) -> None:
    """Reject malformed or unsafe manual velocity before vehicle delivery."""

    if (
        isinstance(linear_x, bool)
        or isinstance(angular_z, bool)
        or not isinstance(linear_x, (int, float))
        or not isinstance(angular_z, (int, float))
    ):
        raise CommandValidationError('linear_x and angular_z must be numbers')
    if not math.isfinite(linear_x) or not math.isfinite(angular_z):
        raise CommandValidationError('linear_x and angular_z must be finite')
    if abs(linear_x) > vehicle.command.max_linear_x:
        raise CommandValidationError('linear_x exceeds configured limit')
    if abs(angular_z) > vehicle.command.max_angular_z:
        raise CommandValidationError('angular_z exceeds configured limit')
    if isinstance(hold_ms, bool) or not isinstance(hold_ms, int):
        raise CommandValidationError('hold_ms must be an integer')
    if hold_ms < 1 or hold_ms > vehicle.command.max_hold_ms:
        raise CommandValidationError('hold_ms exceeds configured limit')


class VehicleCommandClient:
    """Send commands only to the configured vehicle command API endpoint."""

    def __init__(
        self,
        *,
        request: Callable[[str, str, dict[str, Any] | None], Awaitable[dict[str, Any]]]
        | None = None,
        timeout_sec: float = 5.0,
    ) -> None:
        if timeout_sec <= 0:
            raise ValueError('timeout_sec must be greater than zero')
        self._request = request or self._default_request
        self._timeout_sec = timeout_sec

    async def send_twist(
        self,
        vehicle: VehicleConfig,
        linear_x: float,
        angular_z: float,
        hold_ms: int,
    ) -> dict[str, Any]:
        validate_command(vehicle, linear_x, angular_z, hold_ms)
        return await self._post(
            vehicle,
            '/v1/cmd-vel',
            {
                'linear_x': float(linear_x),
                'angular_z': float(angular_z),
                'hold_ms': hold_ms,
            },
        )

    async def send_goal_pose(
        self,
        vehicle: VehicleConfig,
        goal_pose: Mapping[str, Any],
    ) -> dict[str, Any]:
        return await self._post(
            vehicle,
            '/v1/navigation/goals',
            self._vehicle_goal(goal_pose),
        )

    async def cancel_navigation(
        self,
        vehicle: VehicleConfig,
        operation_id: str | None = None,
    ) -> dict[str, Any]:
        payload = {} if operation_id is None else {'operation_id': operation_id}
        return await self._post(vehicle, '/v1/navigation/cancel', payload)

    async def stop(self, vehicle: VehicleConfig) -> dict[str, Any]:
        return await self._post(vehicle, '/v1/stop', None)

    async def _post(
        self,
        vehicle: VehicleConfig,
        path: str,
        payload: dict[str, Any] | None,
    ) -> dict[str, Any]:
        return await self._request(
            'POST',
            f'{vehicle.command_api_url.rstrip("/")}{path}',
            payload,
        )

    async def _default_request(
        self,
        method: str,
        url: str,
        payload: dict[str, Any] | None,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(self._request_sync, method, url, payload)

    def _request_sync(
        self,
        method: str,
        url: str,
        payload: dict[str, Any] | None,
    ) -> dict[str, Any]:
        data = None if payload is None else json.dumps(payload).encode('utf-8')
        headers = {} if data is None else {'Content-Type': 'application/json'}
        request = Request(url, data=data, headers=headers, method=method)
        try:
            with urlopen(request, timeout=self._timeout_sec) as response:
                return self._decode_response(response.read(), response.status)
        except HTTPError as error:
            detail = self._error_detail(error.read())
            raise VehicleCommandApiError(error.code, detail) from error
        except (OSError, URLError) as error:
            raise VehicleCommandTransportError(
                f'vehicle command API unavailable: {error}',
            ) from error

    @staticmethod
    def _decode_response(body: bytes, status_code: int) -> dict[str, Any]:
        try:
            response = json.loads(body)
        except json.JSONDecodeError as error:
            raise VehicleCommandTransportError(
                f'vehicle command API returned invalid JSON (HTTP {status_code})',
            ) from error
        if not isinstance(response, dict):
            raise VehicleCommandTransportError(
                f'vehicle command API returned a non-object response (HTTP {status_code})',
            )
        return response

    @staticmethod
    def _error_detail(body: bytes) -> str:
        try:
            response = json.loads(body)
        except json.JSONDecodeError:
            return 'vehicle command API returned an error response'
        if isinstance(response, dict) and isinstance(response.get('error'), str):
            return response['error']
        return 'vehicle command API returned an error response'

    @staticmethod
    def _vehicle_goal(goal_pose: Mapping[str, Any]) -> dict[str, Any]:
        header = goal_pose['header']
        pose = goal_pose['pose']
        position = pose['position']
        orientation = pose['orientation']
        x = float(orientation['x'])
        y = float(orientation['y'])
        z = float(orientation['z'])
        w = float(orientation['w'])
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return {
            'frame_id': header['frame_id'],
            'x': float(position['x']),
            'y': float(position['y']),
            'yaw': yaw,
        }
