"""Deliver Fleet Manager commands through each vehicle's HTTP command API."""

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fleet_bridge_config.models import VehicleConfig


class VehicleCommandApiError(RuntimeError):
    """The vehicle API returned a non-success HTTP response."""

    def __init__(
        self,
        status_code: int,
        detail: str,
        body: dict[str, Any] | None = None,
    ) -> None:
        self.status_code = status_code
        self.detail = detail
        self.body = body or {'error': detail}
        super().__init__(detail)


class VehicleCommandTransportError(RuntimeError):
    """The vehicle API could not be reached or returned invalid JSON."""


@dataclass(frozen=True)
class VehicleCommandResponse:
    """The exact successful HTTP status and JSON object returned by a vehicle."""

    status_code: int
    body: dict[str, Any]


class VehicleCommandClient:
    """Send commands only to the configured vehicle command API endpoint."""

    def __init__(
        self,
        *,
        request: Callable[
            [str, str, Any | None],
            Awaitable[VehicleCommandResponse | dict[str, Any]],
        ] | None = None,
        timeout_sec: float = 5.0,
    ) -> None:
        if timeout_sec <= 0:
            raise ValueError('timeout_sec must be greater than zero')
        self._request = request or self._default_request
        self._timeout_sec = timeout_sec

    async def relay(
        self,
        vehicle: VehicleConfig,
        method: str,
        path: str,
        payload: Any | None = None,
    ) -> VehicleCommandResponse:
        response = await self._request(
            method,
            f'{vehicle.command_api_url.rstrip("/")}{path}',
            payload,
        )
        if isinstance(response, VehicleCommandResponse):
            return response
        if isinstance(response, dict):
            return VehicleCommandResponse(status_code=200, body=response)
        raise VehicleCommandTransportError('vehicle command API returned a non-object response')

    async def _default_request(
        self,
        method: str,
        url: str,
        payload: Any | None,
    ) -> VehicleCommandResponse:
        return await asyncio.to_thread(self._request_sync, method, url, payload)

    def _request_sync(
        self,
        method: str,
        url: str,
        payload: Any | None,
    ) -> VehicleCommandResponse:
        data = None if payload is None else json.dumps(payload).encode('utf-8')
        headers = {} if data is None else {'Content-Type': 'application/json'}
        request = Request(url, data=data, headers=headers, method=method)
        try:
            with urlopen(request, timeout=self._timeout_sec) as response:
                return VehicleCommandResponse(
                    status_code=response.status,
                    body=self._decode_response(response.read(), response.status),
                )
        except HTTPError as error:
            try:
                body = self._error_body(error.read())
            finally:
                error.close()
            detail = self._error_detail(body)
            raise VehicleCommandApiError(error.code, detail, body) from error
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
    def _error_body(body: bytes) -> dict[str, Any]:
        try:
            response = json.loads(body)
        except json.JSONDecodeError:
            return {}
        if isinstance(response, dict):
            return response
        return {}

    @staticmethod
    def _error_detail(body: dict[str, Any]) -> str:
        if isinstance(body.get('error'), str):
            return body['error']
        return 'vehicle command API returned an error response'
