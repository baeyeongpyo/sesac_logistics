"""Vehicle Foxglove WebSocket to server ROS 2 worker runtime."""

import argparse
import asyncio
import logging
import os
import time
from typing import Any, Callable

from fleet_bridge_config.loader import load_fleet, load_telemetry
from fleet_bridge_config.models import TopicConfig

from .protocol import (
    Advertise,
    IgnoredMessage,
    ProtocolError,
    ServerInfo,
    SUPPORTED_SUBPROTOCOLS,
    Unadvertise,
    parse_message_frame,
    parse_server_message,
    subscribe_message,
)
from .republisher import ChannelSelector, RateGate, RosRepublisher
from .state import ReconnectBackoff, WorkerState


LOGGER = logging.getLogger('foxglove_ros_worker')


class FoxgloveWorker:
    def __init__(
        self,
        robot_id: str,
        uri: str,
        topics: tuple[TopicConfig, ...],
        republisher: Any,
        state: WorkerState,
        *,
        connect_factory: Callable[..., Any] | None = None,
        wall_clock: Callable[[], float] = time.time,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], Any] = asyncio.sleep,
    ) -> None:
        self._robot_id = robot_id
        self._uri = uri
        self._selector = ChannelSelector(topics)
        self._republisher = republisher
        self._state = state
        self._connect_factory = connect_factory
        self._wall_clock = wall_clock
        self._monotonic_ns = monotonic_ns
        self._sleep = sleep
        self._rate_gate = RateGate()
        self._backoff = ReconnectBackoff()
        self._channel_subscriptions: dict[int, int] = {}
        self._subscriptions: dict[int, TopicConfig] = {}
        self._next_subscription_id = 1

    def _reset_subscriptions(self) -> None:
        self._channel_subscriptions.clear()
        self._subscriptions.clear()
        self._next_subscription_id = 1

    async def handle_text(self, payload: str, websocket: Any) -> None:
        message = parse_server_message(payload)
        if isinstance(message, ServerInfo):
            if 'cdr' not in message.supported_encodings:
                raise ProtocolError('Foxglove server does not support CDR encoding')
            return

        if isinstance(message, Advertise):
            new_subscriptions = []
            for channel in message.channels:
                if channel.id in self._channel_subscriptions:
                    continue
                topic = self._selector.select(channel)
                if topic is None:
                    continue
                subscription_id = self._next_subscription_id
                self._next_subscription_id += 1
                self._channel_subscriptions[channel.id] = subscription_id
                self._subscriptions[subscription_id] = topic
                new_subscriptions.append((subscription_id, channel.id))
            if new_subscriptions:
                await websocket.send(subscribe_message(new_subscriptions))
            return

        if isinstance(message, Unadvertise):
            for channel_id in message.channel_ids:
                subscription_id = self._channel_subscriptions.pop(channel_id, None)
                if subscription_id is not None:
                    self._subscriptions.pop(subscription_id, None)
            return

        if isinstance(message, IgnoredMessage):
            return

        raise ProtocolError(f'unsupported server message: {type(message).__name__}')

    async def handle_binary(self, payload: bytes) -> None:
        frame = parse_message_frame(payload)
        topic = self._subscriptions.get(frame.subscription_id)
        if topic is None:
            return
        if not self._rate_gate.allow(topic, self._monotonic_ns()):
            return
        self._republisher.publish(topic, frame.payload)
        self._state.record_message(self._wall_clock())

    def _open_connection(self):
        connect_factory = self._connect_factory
        if connect_factory is None:
            import websockets

            connect_factory = websockets.connect
        return connect_factory(
            self._uri,
            subprotocols=list(SUPPORTED_SUBPROTOCOLS),
            max_size=8 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=20,
            close_timeout=5,
        )

    async def run_once(self) -> None:
        self._reset_subscriptions()
        async with self._open_connection() as websocket:
            selected_subprotocol = getattr(websocket, 'subprotocol', None)
            if selected_subprotocol not in SUPPORTED_SUBPROTOCOLS:
                raise ProtocolError(
                    'Foxglove server did not negotiate a supported subprotocol '
                    f'({", ".join(SUPPORTED_SUBPROTOCOLS)})',
                )
            self._state.connected()
            self._backoff.reset()
            LOGGER.info('%s connected to %s', self._robot_id, self._uri)
            async for payload in websocket:
                if isinstance(payload, str):
                    await self.handle_text(payload, websocket)
                elif isinstance(payload, bytes):
                    await self.handle_binary(payload)
                else:
                    raise ProtocolError('WebSocket message must be text or bytes')
        raise ConnectionError('Foxglove WebSocket closed')

    async def run_forever(self) -> None:
        while True:
            try:
                await self.run_once()
            except asyncio.CancelledError:
                self._state.disconnected('worker cancelled')
                raise
            except Exception as error:  # connection failures must not stop other workers
                self._state.disconnected(str(error), reconnect=True)
                delay = self._backoff.next_delay()
                LOGGER.warning(
                    '%s connection failed (%s); retrying in %.1fs',
                    self._robot_id,
                    error,
                    delay,
                )
                await self._sleep(delay)


def _arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Republish one vehicle Foxglove stream into the server ROS domain.',
    )
    parser.add_argument('--robot-id', default=os.environ.get('ROBOT_ID'))
    parser.add_argument(
        '--fleet-config',
        default=os.environ.get('FLEET_CONFIG', '/config/fleet.yaml'),
    )
    parser.add_argument(
        '--telemetry-config',
        default=os.environ.get('TELEMETRY_CONFIG', '/config/telemetry.yaml'),
    )
    parser.add_argument('--uri', default=os.environ.get('FOXGLOVE_URI'))
    parser.add_argument(
        '--freshness-timeout',
        type=float,
        default=float(os.environ.get('FRESHNESS_TIMEOUT_SEC', '10')),
    )
    return parser.parse_args(argv)


def main(argv=None) -> None:
    logging.basicConfig(
        level=os.environ.get('LOG_LEVEL', 'INFO'),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    args = _arguments(argv)
    if not args.robot_id:
        raise SystemExit('--robot-id or ROBOT_ID is required')

    fleet = load_fleet(args.fleet_config, os.environ)
    vehicle = fleet.vehicle(args.robot_id)
    if not vehicle.enabled:
        raise SystemExit(f'vehicle {args.robot_id} is disabled')
    configured_domain = os.environ.get('ROS_DOMAIN_ID')
    if configured_domain != str(fleet.server.domain_id):
        raise SystemExit(
            f'ROS_DOMAIN_ID must be server domain {fleet.server.domain_id}, '
            f'got {configured_domain!r}',
        )

    topics = load_telemetry(args.telemetry_config, args.robot_id)
    state = WorkerState(args.robot_id, args.freshness_timeout)
    republisher = RosRepublisher(args.robot_id, topics, state)
    worker = FoxgloveWorker(
        args.robot_id,
        args.uri or vehicle.foxglove_uri,
        topics,
        republisher,
        state,
    )
    republisher.start()
    try:
        asyncio.run(worker.run_forever())
    except KeyboardInterrupt:
        state.disconnected('worker interrupted')
    finally:
        republisher.close()


if __name__ == '__main__':
    main()
