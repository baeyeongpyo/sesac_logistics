"""Bounded `cmd_vel` delivery through a vehicle Foxglove Bridge."""

import asyncio
import math
from collections.abc import Awaitable, Callable
from typing import Any

from fleet_bridge_config.models import VehicleConfig

from .protocol import (
    ProtocolError,
    ServerInfo,
    client_advertise_message,
    client_message_frame,
    parse_server_message,
)


SUBPROTOCOL = 'foxglove.websocket.v1'
COMMAND_CHANNEL_ID = 1


class CommandValidationError(ValueError):
    """Raised when a command violates a configured vehicle safety bound."""


def validate_command(
    vehicle: VehicleConfig,
    linear_x: float,
    angular_z: float,
    hold_ms: int,
) -> None:
    """Reject unsafe or malformed command values before opening a socket."""

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


def serialize_twist(linear_x: float, angular_z: float) -> bytes:
    """Serialize a planar Twist to the ROS 2 CDR wire representation."""

    from geometry_msgs.msg import Twist
    from rclpy.serialization import serialize_message

    message = Twist()
    message.linear.x = float(linear_x)
    message.angular.z = float(angular_z)
    return serialize_message(message)


class FoxgloveCommandClient:
    """Open one short-lived verified client-publish connection per command."""

    def __init__(
        self,
        *,
        connect_factory: Callable[..., Any] | None = None,
        serialize_twist: Callable[[float, float], bytes] = serialize_twist,
        sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
    ) -> None:
        self._connect_factory = connect_factory
        self._serialize_twist = serialize_twist
        self._sleep = sleep

    def _open_connection(self, vehicle: VehicleConfig):
        connect_factory = self._connect_factory
        if connect_factory is None:
            import websockets

            connect_factory = websockets.connect
        return connect_factory(
            vehicle.foxglove_uri,
            subprotocols=[SUBPROTOCOL],
            max_size=8 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=20,
            close_timeout=5,
        )

    async def _prepare(self, websocket: Any, vehicle: VehicleConfig) -> None:
        if getattr(websocket, 'subprotocol', None) != SUBPROTOCOL:
            raise ProtocolError(
                f'Foxglove server did not negotiate {SUBPROTOCOL}',
            )
        payload = await websocket.recv()
        if not isinstance(payload, str):
            raise ProtocolError('Foxglove server did not send serverInfo text')
        server_info = parse_server_message(payload)
        if not isinstance(server_info, ServerInfo):
            raise ProtocolError('Foxglove server did not send serverInfo first')
        if 'clientPublish' not in server_info.capabilities:
            raise ProtocolError('Foxglove server does not support clientPublish')
        if 'cdr' not in server_info.supported_encodings:
            raise ProtocolError('Foxglove server does not support CDR encoding')
        await websocket.send(client_advertise_message(
            COMMAND_CHANNEL_ID,
            vehicle.command.topic,
            vehicle.command.message_type,
        ))

    async def _send_frame(self, websocket: Any, payload: bytes) -> None:
        await websocket.send(client_message_frame(COMMAND_CHANNEL_ID, payload))

    async def send_twist(
        self,
        vehicle: VehicleConfig,
        linear_x: float,
        angular_z: float,
        hold_ms: int,
    ) -> None:
        """Send a bounded command repeatedly, then always send a zero Twist."""

        validate_command(vehicle, linear_x, angular_z, hold_ms)
        command_payload = self._serialize_twist(linear_x, angular_z)
        zero_payload = self._serialize_twist(0.0, 0.0)
        duration = hold_ms / 1000.0
        interval = 1.0 / vehicle.command.publish_rate_hz

        async with self._open_connection(vehicle) as websocket:
            await self._prepare(websocket, vehicle)
            remaining = duration
            try:
                while remaining > 0:
                    await self._send_frame(websocket, command_payload)
                    delay = min(interval, remaining)
                    await self._sleep(delay)
                    remaining -= delay
            finally:
                await self._send_frame(websocket, zero_payload)

    async def stop(self, vehicle: VehicleConfig) -> None:
        """Send one immediate zero Twist command to a verified vehicle bridge."""

        zero_payload = self._serialize_twist(0.0, 0.0)
        async with self._open_connection(vehicle) as websocket:
            await self._prepare(websocket, vehicle)
            await self._send_frame(websocket, zero_payload)
