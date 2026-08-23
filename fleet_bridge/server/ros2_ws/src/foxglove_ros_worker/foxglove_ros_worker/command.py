"""Bounded `cmd_vel` delivery through a vehicle Foxglove Bridge."""

import asyncio
import math
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from typing import Any

from fleet_bridge_config.models import VehicleConfig

from .protocol import (
    AdvertiseServices,
    ProtocolError,
    ServerInfo,
    Service,
    ServiceCallFailure,
    SUPPORTED_SUBPROTOCOLS,
    client_advertise_message,
    client_message_frame,
    client_service_call_frame,
    parse_service_call_response_frame,
    parse_server_message,
)


COMMAND_CHANNEL_ID = 1
SERVICE_CALL_ID = 1


class CommandValidationError(ValueError):
    """Raised when a command violates a configured vehicle safety bound."""


@dataclass(frozen=True)
class NavigationCancelResult:
    return_code: int
    goals_canceling: int


class StopDeliveryError(RuntimeError):
    """Raised after the combined stop attempted both Nav2 and cmd_vel paths."""

    def __init__(self, failures: tuple[tuple[str, Exception], ...]) -> None:
        self.failures = failures
        details = '; '.join(f'{name}: {error}' for name, error in failures)
        super().__init__(f'stop delivery failed ({details})')


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


def serialize_goal_pose(goal_pose: Mapping[str, Any]) -> bytes:
    """Serialize a ROS-shaped goal pose request as ``PoseStamped`` CDR."""

    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import serialize_message

    message = PoseStamped()
    header = goal_pose['header']
    stamp = header['stamp']
    pose = goal_pose['pose']
    position = pose['position']
    orientation = pose['orientation']
    message.header.stamp.sec = stamp['sec']
    message.header.stamp.nanosec = stamp['nanosec']
    message.header.frame_id = header['frame_id']
    message.pose.position.x = position['x']
    message.pose.position.y = position['y']
    message.pose.position.z = position['z']
    message.pose.orientation.x = orientation['x']
    message.pose.orientation.y = orientation['y']
    message.pose.orientation.z = orientation['z']
    message.pose.orientation.w = orientation['w']
    return serialize_message(message)


def serialize_cancel_request() -> bytes:
    """Serialize the all-zero ``CancelGoal`` request that cancels all goals."""

    from action_msgs.srv import CancelGoal
    from rclpy.serialization import serialize_message

    return serialize_message(CancelGoal.Request())


def deserialize_cancel_response(payload: bytes) -> NavigationCancelResult:
    """Deserialize a ``CancelGoal`` response into API-neutral summary data."""

    from action_msgs.srv import CancelGoal
    from rclpy.serialization import deserialize_message

    response = deserialize_message(payload, CancelGoal.Response)
    return NavigationCancelResult(
        return_code=response.return_code,
        goals_canceling=len(response.goals_canceling),
    )


class FoxgloveCommandClient:
    """Open one short-lived verified client-publish connection per command."""

    def __init__(
        self,
        *,
        connect_factory: Callable[..., Any] | None = None,
        serialize_twist: Callable[[float, float], bytes] = serialize_twist,
        serialize_goal_pose: Callable[[Mapping[str, Any]], bytes] = serialize_goal_pose,
        serialize_cancel_request: Callable[[], bytes] = serialize_cancel_request,
        deserialize_cancel_response: Callable[
            [bytes], NavigationCancelResult
        ] = deserialize_cancel_response,
        service_timeout_sec: float = 5.0,
        sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
    ) -> None:
        self._connect_factory = connect_factory
        self._serialize_twist = serialize_twist
        self._serialize_goal_pose = serialize_goal_pose
        self._serialize_cancel_request = serialize_cancel_request
        self._deserialize_cancel_response = deserialize_cancel_response
        self._service_timeout_sec = service_timeout_sec
        self._sleep = sleep

    def _open_connection(self, vehicle: VehicleConfig):
        connect_factory = self._connect_factory
        if connect_factory is None:
            import websockets

            connect_factory = websockets.connect
        return connect_factory(
            vehicle.foxglove_uri,
            subprotocols=list(SUPPORTED_SUBPROTOCOLS),
            max_size=8 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=20,
            close_timeout=5,
        )

    async def _server_info(self, websocket: Any) -> ServerInfo:
        selected_subprotocol = getattr(websocket, 'subprotocol', None)
        if selected_subprotocol not in SUPPORTED_SUBPROTOCOLS:
            raise ProtocolError(
                'Foxglove server did not negotiate a supported subprotocol '
                f'({", ".join(SUPPORTED_SUBPROTOCOLS)})',
            )
        payload = await websocket.recv()
        if not isinstance(payload, str):
            raise ProtocolError('Foxglove server did not send serverInfo text')
        server_info = parse_server_message(payload)
        if not isinstance(server_info, ServerInfo):
            raise ProtocolError('Foxglove server did not send serverInfo first')
        if 'cdr' not in server_info.supported_encodings:
            raise ProtocolError('Foxglove server does not support CDR encoding')
        return server_info

    async def _server_info_with_timeout(self, websocket: Any) -> ServerInfo:
        try:
            return await asyncio.wait_for(
                self._server_info(websocket),
                timeout=self._service_timeout_sec,
            )
        except asyncio.TimeoutError as error:
            raise ProtocolError('Foxglove serverInfo timed out') from error

    async def _prepare_publish(
        self,
        websocket: Any,
        topic: str,
        message_type: str,
    ) -> None:
        server_info = await self._server_info_with_timeout(websocket)
        if 'clientPublish' not in server_info.capabilities:
            raise ProtocolError('Foxglove server does not support clientPublish')
        await websocket.send(client_advertise_message(
            COMMAND_CHANNEL_ID,
            topic,
            message_type,
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
            await self._prepare_publish(
                websocket,
                vehicle.command.topic,
                vehicle.command.message_type,
            )
            remaining = duration
            try:
                while remaining > 0:
                    await self._send_frame(websocket, command_payload)
                    delay = min(interval, remaining)
                    await self._sleep(delay)
                    remaining -= delay
            finally:
                await self._send_frame(websocket, zero_payload)

    async def send_goal_pose(
        self,
        vehicle: VehicleConfig,
        goal_pose: Mapping[str, Any],
    ) -> None:
        """Publish one ROS-shaped ``PoseStamped`` to the vehicle Nav2 goal topic."""

        payload = self._serialize_goal_pose(goal_pose)
        async with self._open_connection(vehicle) as websocket:
            await self._prepare_publish(
                websocket,
                vehicle.navigation.goal_topic,
                vehicle.navigation.goal_message_type,
            )
            await self._send_frame(websocket, payload)

    async def _find_cancel_service(
        self,
        websocket: Any,
        vehicle: VehicleConfig,
    ) -> Service:
        while True:
            payload = await websocket.recv()
            if not isinstance(payload, str):
                raise ProtocolError(
                    'Foxglove server did not advertise cancel service as text',
                )
            message = parse_server_message(payload)
            if not isinstance(message, AdvertiseServices):
                continue
            for service in message.services:
                if service.name != vehicle.navigation.cancel_service:
                    continue
                if service.type != vehicle.navigation.cancel_service_type:
                    raise ProtocolError(
                        'Nav2 cancel service type does not match configuration',
                    )
                if service.request_encoding not in (None, 'cdr'):
                    raise ProtocolError('Nav2 cancel service does not accept CDR')
                if service.response_encoding not in (None, 'cdr'):
                    raise ProtocolError('Nav2 cancel service does not return CDR')
                return service

    async def cancel_navigation(
        self,
        vehicle: VehicleConfig,
    ) -> NavigationCancelResult:
        """Cancel every active goal on the configured Nav2 NavigateToPose action."""

        async with self._open_connection(vehicle) as websocket:
            server_info = await self._server_info_with_timeout(websocket)
            if 'services' not in server_info.capabilities:
                raise ProtocolError('Foxglove server does not support services')
            try:
                service = await asyncio.wait_for(
                    self._find_cancel_service(websocket, vehicle),
                    timeout=self._service_timeout_sec,
                )
            except asyncio.TimeoutError as error:
                raise ProtocolError(
                    'Nav2 cancel service advertisement timed out',
                ) from error
            await websocket.send(client_service_call_frame(
                service.id,
                SERVICE_CALL_ID,
                'cdr',
                self._serialize_cancel_request(),
            ))
            try:
                return await asyncio.wait_for(
                    self._wait_cancel_response(websocket, service),
                    timeout=self._service_timeout_sec,
                )
            except asyncio.TimeoutError as error:
                raise ProtocolError('Nav2 cancel service response timed out') from error

    async def _wait_cancel_response(
        self,
        websocket: Any,
        service: Service,
    ) -> NavigationCancelResult:
        while True:
            payload = await websocket.recv()
            if isinstance(payload, str):
                message = parse_server_message(payload)
                if (
                    isinstance(message, ServiceCallFailure)
                    and message.service_id == service.id
                    and message.call_id == SERVICE_CALL_ID
                ):
                    raise ProtocolError(
                        f'Nav2 cancel service call failed: {message.message}',
                    )
                continue
            if not isinstance(payload, bytes):
                raise ProtocolError('Foxglove service response must be bytes')
            response = parse_service_call_response_frame(payload)
            if (
                response.service_id != service.id
                or response.call_id != SERVICE_CALL_ID
            ):
                continue
            if response.encoding != 'cdr':
                raise ProtocolError('Nav2 cancel service response is not CDR')
            try:
                return self._deserialize_cancel_response(response.payload)
            except Exception as error:
                raise ProtocolError(
                    'invalid Nav2 cancel CDR response',
                ) from error

    async def stop_cmd_vel(self, vehicle: VehicleConfig) -> None:
        """Send one immediate zero Twist command to a verified vehicle bridge."""

        zero_payload = self._serialize_twist(0.0, 0.0)
        async with self._open_connection(vehicle) as websocket:
            await self._prepare_publish(
                websocket,
                vehicle.command.topic,
                vehicle.command.message_type,
            )
            await self._send_frame(websocket, zero_payload)

    async def stop(self, vehicle: VehicleConfig) -> NavigationCancelResult:
        """Attempt Nav2 cancellation and zero ``cmd_vel`` without short-circuiting."""

        nav_result, cmd_vel_result = await asyncio.gather(
            self.cancel_navigation(vehicle),
            self.stop_cmd_vel(vehicle),
            return_exceptions=True,
        )
        failures = []
        if isinstance(nav_result, Exception):
            failures.append(('Nav2 cancel', nav_result))
        if isinstance(cmd_vel_result, Exception):
            failures.append(('cmd_vel stop', cmd_vel_result))
        if failures:
            raise StopDeliveryError(tuple(failures))
        return nav_result
