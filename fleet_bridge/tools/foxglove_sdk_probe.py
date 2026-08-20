#!/usr/bin/env python3
"""Probe a vehicle Foxglove SDK Bridge from the command-api container."""

import argparse
import asyncio
import json
import os
import struct
import sys
from collections.abc import Callable
from typing import Any


DEFAULT_PROTOCOL = 'foxglove.sdk.v1'
COMMAND_CHANNEL_ID = 1


class ProbeError(RuntimeError):
    """Raised when a Foxglove Bridge does not satisfy the probe contract."""


def _server_info(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, str):
        raise ProbeError('Foxglove Bridge did not send a text serverInfo message')
    try:
        info = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ProbeError('Foxglove Bridge sent malformed serverInfo JSON') from error
    if not isinstance(info, dict) or info.get('op') != 'serverInfo':
        raise ProbeError('Foxglove Bridge did not send serverInfo first')
    return info


def _zero_twist_frame() -> bytes:
    payload = json.dumps({
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    }, separators=(',', ':')).encode('utf-8')
    return b'\x01' + struct.pack('<I', COMMAND_CHANNEL_ID) + payload


def _advertise_twist() -> str:
    return json.dumps({
        'op': 'advertise',
        'channels': [{
            'id': COMMAND_CHANNEL_ID,
            'topic': '/cmd_vel',
            'encoding': 'json',
            'schemaName': 'geometry_msgs/msg/Twist',
        }],
    }, separators=(',', ':'))


async def probe(
    uri: str,
    *,
    send_zero_cmd_vel: bool = False,
    connect_factory: Callable[..., Any] | None = None,
    emit: Callable[[str], None] = print,
) -> dict[str, Any]:
    """Report server capabilities and optionally publish exactly one zero Twist."""

    if connect_factory is None:
        import websockets

        connect_factory = websockets.connect

    async with connect_factory(
        uri,
        subprotocols=[DEFAULT_PROTOCOL],
        open_timeout=5,
    ) as websocket:
        if getattr(websocket, 'subprotocol', None) != DEFAULT_PROTOCOL:
            raise ProbeError(f'Bridge did not negotiate {DEFAULT_PROTOCOL}')

        info = _server_info(await websocket.recv())
        emit(f'connected: {uri}')
        emit(f'subprotocol: {DEFAULT_PROTOCOL}')
        emit(json.dumps(info, ensure_ascii=False, sort_keys=True))

        if not send_zero_cmd_vel:
            return info

        capabilities = info.get('capabilities', [])
        encodings = info.get('supportedEncodings', [])
        if 'clientPublish' not in capabilities:
            raise ProbeError('Bridge does not advertise clientPublish')
        if 'json' not in encodings:
            raise ProbeError('Bridge does not advertise JSON client publishing')

        await websocket.send(_advertise_twist())
        await websocket.send(_zero_twist_frame())
        emit('sent: zero Twist to /cmd_vel')
        return info


def _arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Probe a Foxglove SDK Bridge from the command-api container.',
    )
    parser.add_argument(
        '--uri',
        default=os.environ.get('ROBOT_2_FOXGLOVE_URI'),
        help='Vehicle Bridge URI (default: ROBOT_2_FOXGLOVE_URI).',
    )
    parser.add_argument(
        '--send-zero-cmd-vel',
        action='store_true',
        help='Advertise /cmd_vel and publish one zero geometry_msgs/msg/Twist.',
    )
    arguments = parser.parse_args(argv)
    if not arguments.uri:
        parser.error('--uri or ROBOT_2_FOXGLOVE_URI is required')
    return arguments


def main(argv: list[str] | None = None) -> int:
    arguments = _arguments(argv)
    try:
        asyncio.run(probe(
            arguments.uri,
            send_zero_cmd_vel=arguments.send_zero_cmd_vel,
        ))
    except Exception as error:
        print(f'ERROR: {type(error).__name__}: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
