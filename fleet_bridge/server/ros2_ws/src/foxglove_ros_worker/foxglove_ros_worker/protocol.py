"""Foxglove WebSocket protocol handling shared by supported Bridge versions."""

from dataclasses import dataclass
import json
import struct
from typing import Any, Iterable


SUPPORTED_SUBPROTOCOLS = (
    'foxglove.sdk.v1',
    'foxglove.websocket.v1',
)


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
class Service:
    id: int
    name: str
    type: str
    request_encoding: str | None
    response_encoding: str | None


@dataclass(frozen=True)
class AdvertiseServices:
    services: tuple[Service, ...]


@dataclass(frozen=True)
class ServiceCallFailure:
    service_id: int
    call_id: int
    message: str


@dataclass(frozen=True)
class ServiceCallResponse:
    service_id: int
    call_id: int
    encoding: str
    payload: bytes


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


def parse_server_message(
    payload: str,
) -> ServerInfo | Advertise | AdvertiseServices | ServiceCallFailure | Unadvertise | IgnoredMessage:
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

    if operation == 'advertiseServices':
        raw_services = message.get('services')
        if not isinstance(raw_services, list):
            raise ProtocolError('services must be an array')
        services = []
        for index, raw_service in enumerate(raw_services):
            service = _require_dict(raw_service, f'services[{index}]')
            request = service.get('request')
            response = service.get('response')
            request_encoding = None
            response_encoding = None
            if request is not None:
                request = _require_dict(request, f'services[{index}].request')
                request_encoding = _require_string(
                    request.get('encoding'),
                    f'services[{index}].request.encoding',
                )
            if response is not None:
                response = _require_dict(response, f'services[{index}].response')
                response_encoding = _require_string(
                    response.get('encoding'),
                    f'services[{index}].response.encoding',
                )
            services.append(Service(
                id=_require_int(service.get('id'), f'services[{index}].id'),
                name=_require_string(service.get('name'), f'services[{index}].name'),
                type=_require_string(service.get('type'), f'services[{index}].type'),
                request_encoding=request_encoding,
                response_encoding=response_encoding,
            ))
        return AdvertiseServices(tuple(services))

    if operation == 'serviceCallFailure':
        return ServiceCallFailure(
            service_id=_require_int(message.get('serviceId'), 'serviceId'),
            call_id=_require_int(message.get('callId'), 'callId'),
            message=_require_string(message.get('message'), 'message'),
        )

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


def client_service_call_frame(
    service_id: int,
    call_id: int,
    encoding: str,
    payload: bytes,
) -> bytes:
    """Build a service-call request binary frame (client opcode 2)."""

    if not isinstance(payload, bytes):
        raise ProtocolError('service call payload must be bytes')
    encoding_bytes = _require_string(encoding, 'service encoding').encode('utf-8')
    return (
        b'\x02'
        + struct.pack(
            '<III',
            _require_int(service_id, 'service id'),
            _require_int(call_id, 'call id'),
            len(encoding_bytes),
        )
        + encoding_bytes
        + payload
    )


def parse_service_call_response_frame(payload: bytes) -> ServiceCallResponse:
    """Parse a service-call response binary frame (server opcode 3)."""

    if not isinstance(payload, bytes):
        raise ProtocolError('service response frame must be bytes')
    if len(payload) < 13:
        raise ProtocolError('service response frame is shorter than its header')
    if payload[0] != 3:
        raise ProtocolError(f'unsupported service response opcode: {payload[0]}')
    service_id, call_id, encoding_length = struct.unpack_from('<III', payload, 1)
    encoding_end = 13 + encoding_length
    if encoding_end > len(payload):
        raise ProtocolError('service response encoding exceeds frame length')
    try:
        encoding = payload[13:encoding_end].decode('utf-8')
    except UnicodeDecodeError as error:
        raise ProtocolError('service response encoding must be UTF-8') from error
    return ServiceCallResponse(
        service_id=service_id,
        call_id=call_id,
        encoding=encoding,
        payload=payload[encoding_end:],
    )


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
