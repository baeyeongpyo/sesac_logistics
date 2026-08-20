"""Minimal Foxglove WebSocket v1 protocol parsing used by the worker."""

from dataclasses import dataclass
import json
import struct
from typing import Any, Iterable


class ProtocolError(ValueError):
    """Raised when a Foxglove protocol message is malformed."""


@dataclass(frozen=True)
class ServerInfo:
    name: str
    capabilities: tuple[str, ...]
    supported_encodings: tuple[str, ...]
    metadata: dict[str, str]
    session_id: str | None


@dataclass(frozen=True)
class Channel:
    id: int
    topic: str
    encoding: str
    schema_name: str
    schema: str
    schema_encoding: str | None


@dataclass(frozen=True)
class Advertise:
    channels: tuple[Channel, ...]


@dataclass(frozen=True)
class Unadvertise:
    channel_ids: tuple[int, ...]


@dataclass(frozen=True)
class IgnoredMessage:
    operation: str


@dataclass(frozen=True)
class MessageFrame:
    subscription_id: int
    timestamp_ns: int
    payload: bytes


def _require_dict(value: Any, description: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProtocolError(f'{description} must be an object')
    return value


def _require_string(value: Any, field: str) -> str:
    if not isinstance(value, str):
        raise ProtocolError(f'{field} must be a string')
    return value


def _require_int(value: Any, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ProtocolError(f'{field} must be a non-negative integer')
    return value


def _string_tuple(value: Any, field: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise ProtocolError(f'{field} must be an array')
    return tuple(_require_string(item, field) for item in value)


def parse_server_message(payload: str) -> ServerInfo | Advertise | Unadvertise | IgnoredMessage:
    """Parse a Foxglove server JSON message into a typed value."""

    try:
        message = _require_dict(json.loads(payload), 'message')
    except (json.JSONDecodeError, TypeError) as error:
        raise ProtocolError('message must be valid JSON') from error

    operation = _require_string(message.get('op'), 'op')
    if operation == 'serverInfo':
        metadata = _require_dict(message.get('metadata', {}), 'metadata')
        session_id = message.get('sessionId')
        if session_id is not None:
            session_id = _require_string(session_id, 'sessionId')
        return ServerInfo(
            name=_require_string(message.get('name'), 'name'),
            capabilities=_string_tuple(message.get('capabilities'), 'capabilities'),
            supported_encodings=_string_tuple(
                message.get('supportedEncodings'),
                'supportedEncodings',
            ),
            metadata={
                _require_string(key, 'metadata key'): _require_string(value, 'metadata value')
                for key, value in metadata.items()
            },
            session_id=session_id,
        )

    if operation == 'advertise':
        raw_channels = message.get('channels')
        if not isinstance(raw_channels, list):
            raise ProtocolError('channels must be an array')
        channels = []
        for index, raw_channel in enumerate(raw_channels):
            channel = _require_dict(raw_channel, f'channels[{index}]')
            schema_encoding = channel.get('schemaEncoding')
            if schema_encoding is not None:
                schema_encoding = _require_string(
                    schema_encoding,
                    f'channels[{index}].schemaEncoding',
                )
            channels.append(Channel(
                id=_require_int(channel.get('id'), f'channels[{index}].id'),
                topic=_require_string(channel.get('topic'), f'channels[{index}].topic'),
                encoding=_require_string(channel.get('encoding'), f'channels[{index}].encoding'),
                schema_name=_require_string(
                    channel.get('schemaName'),
                    f'channels[{index}].schemaName',
                ),
                schema=_require_string(channel.get('schema'), f'channels[{index}].schema'),
                schema_encoding=schema_encoding,
            ))
        return Advertise(tuple(channels))

    if operation == 'unadvertise':
        raw_channel_ids = message.get('channelIds')
        if not isinstance(raw_channel_ids, list):
            raise ProtocolError('channelIds must be an array')
        return Unadvertise(tuple(
            _require_int(channel_id, 'channelIds')
            for channel_id in raw_channel_ids
        ))

    return IgnoredMessage(operation)


def subscribe_message(subscriptions: Iterable[tuple[int, int]]) -> str:
    """Build a Foxglove subscribe operation.

    Each pair contains ``(client_subscription_id, server_channel_id)``.
    """

    records = []
    for subscription_id, channel_id in subscriptions:
        records.append({
            'id': _require_int(subscription_id, 'subscription id'),
            'channelId': _require_int(channel_id, 'channel id'),
        })
    return json.dumps(
        {'op': 'subscribe', 'subscriptions': records},
        separators=(',', ':'),
    )


def client_advertise_message(
    channel_id: int,
    topic: str,
    schema_name: str,
) -> str:
    """Build a client channel advertisement for Foxglove client publishing."""

    return json.dumps(
        {
            'op': 'advertise',
            'channels': [{
                'id': _require_int(channel_id, 'channel id'),
                'topic': _require_string(topic, 'topic'),
                'encoding': 'cdr',
                'schemaName': _require_string(schema_name, 'schema name'),
            }],
        },
        separators=(',', ':'),
    )


def client_message_frame(channel_id: int, payload: bytes) -> bytes:
    """Build a client message-data binary frame (opcode 1)."""

    if not isinstance(payload, bytes):
        raise ProtocolError('client message payload must be bytes')
    return b'\x01' + struct.pack('<I', _require_int(channel_id, 'channel id')) + payload


def parse_message_frame(payload: bytes) -> MessageFrame:
    """Parse a Foxglove message-data binary frame (opcode 1)."""

    if not isinstance(payload, bytes):
        raise ProtocolError('message frame must be bytes')
    if len(payload) < 13:
        raise ProtocolError('message frame is shorter than its 13-byte header')
    if payload[0] != 1:
        raise ProtocolError(f'unsupported binary opcode: {payload[0]}')
    subscription_id, timestamp_ns = struct.unpack_from('<IQ', payload, 1)
    return MessageFrame(subscription_id, timestamp_ns, payload[13:])
