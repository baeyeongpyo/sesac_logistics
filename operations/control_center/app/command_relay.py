"""Allowlisted command relay for the vehicle command API."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Any, Callable, Protocol
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


ROBOT_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
MAX_MANUAL_SPEED = 1.0
MAX_MANUAL_ROTATION_RADIANS = math.radians(10.0)


@dataclass(frozen=True)
class RelayResponse:
    """Response returned by the Fleet Bridge command endpoint."""

    status_code: int
    body: Any


class CommandRelayError(RuntimeError):
    """Fleet Bridge was unreachable before a vehicle response was received."""


class Transport(Protocol):
    def __call__(
        self,
        method: str,
        url: str,
        payload: dict[str, Any] | None,
        timeout: float,
    ) -> RelayResponse: ...


class CommandRelay:
    """Forward only explicit operator controls to the Fleet Bridge API."""

    def __init__(
        self,
        base_url: str,
        *,
        request_timeout_sec: float,
        transport: Transport | None = None,
    ):
        self.base_url = base_url.rstrip('/')
        self.request_timeout_sec = request_timeout_sec
        self._transport = transport or _http_post

    def send_manual(
        self,
        robot_id: str,
        *,
        linear_x: float,
        linear_y: float = 0.0,
        angular_z: float,
        hold_ms: int = 300,
    ) -> RelayResponse:
        """Send a bounded forward/reverse/lateral/rotation command to one vehicle."""

        self._validate_robot_id(robot_id)
        self._validate_number('linear_x', linear_x, -MAX_MANUAL_SPEED, MAX_MANUAL_SPEED)
        self._validate_number('linear_y', linear_y, -MAX_MANUAL_SPEED, MAX_MANUAL_SPEED)
        self._validate_number('angular_z', angular_z, -1.0, 1.0)
        if not isinstance(hold_ms, int) or not 100 <= hold_ms <= 1000:
            raise ValueError('hold_ms must be between 100 and 1000 milliseconds')
        if abs(angular_z) * hold_ms / 1000 > MAX_MANUAL_ROTATION_RADIANS:
            raise ValueError('angular_z and hold_ms must not exceed 10 degrees per command')
        return self._post(
            robot_id,
            'cmd-vel',
            {
                'linear_x': linear_x,
                'linear_y': linear_y,
                'angular_z': angular_z,
                'hold_ms': hold_ms,
            },
        )

    def send_initial_pose(
        self,
        robot_id: str,
        *,
        x: float,
        y: float,
        yaw: float,
    ) -> RelayResponse:
        """Publish an AMCL initial pose in the fixed ``map`` frame."""

        self._validate_robot_id(robot_id)
        self._validate_number('x', x, -1000.0, 1000.0)
        self._validate_number('y', y, -1000.0, 1000.0)
        self._validate_number('yaw', yaw, -math.pi, math.pi)
        return self._post(
            robot_id,
            'localization/initial-pose',
            {'frame_id': 'map', 'x': x, 'y': y, 'yaw': yaw},
        )

    def send_navigation_goal(
        self,
        robot_id: str,
        *,
        x: float,
        y: float,
        yaw: float,
    ) -> RelayResponse:
        """Send one operator-selected Nav2 goal in the fixed ``map`` frame."""

        self._validate_robot_id(robot_id)
        return self._post(robot_id, 'navigation/goals', self._navigation_pose(x, y, yaw))

    def send_navigation_waypoints(
        self,
        robot_id: str,
        *,
        waypoints: list[dict[str, float]],
    ) -> RelayResponse:
        """Send an ordered, non-empty Nav2 FollowWaypoints request."""

        self._validate_robot_id(robot_id)
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError('waypoints must be a non-empty list')
        normalized_waypoints = []
        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                raise ValueError(f'waypoints[{index}] must be an object')
            normalized_waypoints.append(self._navigation_pose(
                waypoint.get('x'),
                waypoint.get('y'),
                waypoint.get('yaw'),
            ))
        return self._post(
            robot_id,
            'navigation/waypoints',
            {'waypoints': normalized_waypoints},
        )

    def send_fork_up(self, robot_id: str) -> RelayResponse:
        """Publish the vehicle-native ``UP`` command to ``/fork/command``."""

        self._validate_robot_id(robot_id)
        return self._post(robot_id, 'fork/up', None)

    def send_fork_down(self, robot_id: str) -> RelayResponse:
        """Publish the vehicle-native ``DOWN`` command to ``/fork/command``."""

        self._validate_robot_id(robot_id)
        return self._post(robot_id, 'fork/down', None)

    def send_operation_idle(self, robot_id: str) -> RelayResponse:
        """Explicitly request the vehicle-native transition to IDLE."""

        self._validate_robot_id(robot_id)
        return self._post(
            robot_id,
            'operation/idle',
            {'reason': 'OPERATOR_CONFIRMED'},
        )

    def send_stop(self, robot_id: str) -> RelayResponse:
        """Immediately request a vehicle stop; never persist a command in SQLite."""

        self._validate_robot_id(robot_id)
        return self._post(robot_id, 'stop', None)

    def send_navigation_cancel(self, robot_id: str) -> RelayResponse:
        """Cancel the vehicle's current navigation operation."""

        self._validate_robot_id(robot_id)
        return self._post(robot_id, 'navigation/cancel', {})

    def _post(
        self,
        robot_id: str,
        command_path: str,
        payload: dict[str, Any] | None,
    ) -> RelayResponse:
        url = (
            f'{self.base_url}/api/v1/vehicle-command/{robot_id}/{command_path}'
        )
        return self._transport('POST', url, payload, self.request_timeout_sec)

    @staticmethod
    def _validate_robot_id(robot_id: str) -> None:
        if not isinstance(robot_id, str) or not ROBOT_ID_PATTERN.fullmatch(robot_id):
            raise ValueError('robot_id has an invalid format')

    @classmethod
    def _navigation_pose(cls, x: float, y: float, yaw: float) -> dict[str, float | str]:
        cls._validate_number('x', x, -1000.0, 1000.0)
        cls._validate_number('y', y, -1000.0, 1000.0)
        cls._validate_number('yaw', yaw, -math.pi, math.pi)
        return {'frame_id': 'map', 'x': x, 'y': y, 'yaw': yaw}

    @staticmethod
    def _validate_number(name: str, value: float, minimum: float, maximum: float) -> None:
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
            or value < minimum
            or value > maximum
        ):
            raise ValueError(f'{name} must be between {minimum} and {maximum}')


def _http_post(
    method: str,
    url: str,
    payload: dict[str, Any] | None,
    timeout: float,
) -> RelayResponse:
    """Perform one JSON request while preserving Fleet Bridge HTTP responses."""

    body = None if payload is None else json.dumps(payload).encode('utf-8')
    request = Request(
        url,
        data=body,
        headers={'Content-Type': 'application/json'} if body else {},
        method=method,
    )
    try:
        with urlopen(request, timeout=timeout) as response:  # noqa: S310 - fixed LAN URL
            return RelayResponse(response.status, _decode_json(response.read()))
    except HTTPError as error:
        return RelayResponse(error.code, _decode_json(error.read()))
    except (OSError, URLError) as error:
        raise CommandRelayError('Fleet Bridge command API에 연결할 수 없습니다.') from error


def _decode_json(raw: bytes) -> Any:
    if not raw:
        return None
    try:
        return json.loads(raw.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return {'detail': 'Fleet Bridge가 JSON이 아닌 응답을 반환했습니다.'}
