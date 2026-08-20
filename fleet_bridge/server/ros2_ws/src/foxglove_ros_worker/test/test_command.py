import asyncio
import json
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from fleet_bridge_config.models import CommandConfig, VehicleConfig
from foxglove_ros_worker.command import FoxgloveCommandClient
from foxglove_ros_worker.protocol import ProtocolError


class FakeWebSocket:
    def __init__(self, server_info, subprotocol='foxglove.websocket.v1'):
        self.subprotocol = subprotocol
        self._messages = [json.dumps(server_info)]
        self.sent = []

    async def recv(self):
        return self._messages.pop(0)

    async def send(self, payload):
        self.sent.append(payload)


class FakeConnection:
    def __init__(self, websocket):
        self.websocket = websocket

    async def __aenter__(self):
        return self.websocket

    async def __aexit__(self, exc_type, exc, traceback):
        return False


def vehicle():
    return VehicleConfig(
        id='robot_1',
        foxglove_uri='ws://10.0.0.11:8766',
        enabled=True,
        command=CommandConfig(
            topic='/cmd_vel',
            message_type='geometry_msgs/msg/Twist',
            max_linear_x=0.3,
            max_angular_z=1.0,
            max_hold_ms=1000,
            publish_rate_hz=10.0,
        ),
    )


def server_info(capabilities=('clientPublish',), encodings=('cdr',)):
    return {
        'op': 'serverInfo',
        'name': 'foxglove_bridge',
        'capabilities': list(capabilities),
        'supportedEncodings': list(encodings),
    }


class FoxgloveCommandClientTest(unittest.TestCase):
    def test_sends_advertise_nonzero_and_final_zero_twist(self):
        websocket = FakeWebSocket(server_info())
        sleeps = []
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_twist=lambda linear_x, angular_z: (
                f'{linear_x}:{angular_z}'.encode('ascii')
            ),
            sleep=lambda duration: sleeps.append(duration) or _completed(),
        )

        asyncio.run(client.send_twist(vehicle(), 0.1, 0.2, 100))

        self.assertEqual(json.loads(websocket.sent[0])['op'], 'advertise')
        self.assertEqual(websocket.sent[1], b'\x01\x01\x00\x00\x000.1:0.2')
        self.assertEqual(websocket.sent[-1], b'\x01\x01\x00\x00\x000.0:0.0')
        self.assertEqual(sleeps, [0.1])

    def test_rejects_bridge_without_client_publish_before_advertising(self):
        websocket = FakeWebSocket(server_info(capabilities=()))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_twist=lambda linear_x, angular_z: b'cdr',
        )

        with self.assertRaisesRegex(ProtocolError, 'clientPublish'):
            asyncio.run(client.send_twist(vehicle(), 0.1, 0.2, 100))

        self.assertEqual(websocket.sent, [])

    def test_stop_sends_only_zero_twist(self):
        websocket = FakeWebSocket(server_info())
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_twist=lambda linear_x, angular_z: (
                f'{linear_x}:{angular_z}'.encode('ascii')
            ),
        )

        asyncio.run(client.stop(vehicle()))

        self.assertEqual(len(websocket.sent), 2)
        self.assertEqual(websocket.sent[-1], b'\x01\x01\x00\x00\x000.0:0.0')

    def test_sdk_bridge_is_accepted_and_requested_before_legacy_fallback(self):
        websocket = FakeWebSocket(
            server_info(),
            subprotocol='foxglove.sdk.v1',
        )
        connect_calls = []

        def connect(uri, **kwargs):
            connect_calls.append((uri, kwargs))
            return FakeConnection(websocket)

        client = FoxgloveCommandClient(
            connect_factory=connect,
            serialize_twist=lambda linear_x, angular_z: b'cdr',
        )

        try:
            asyncio.run(client.stop(vehicle()))
        except ProtocolError:
            delivered = False
        else:
            delivered = True

        self.assertTrue(delivered, 'SDK Bridge stop command must be accepted')

        self.assertEqual(connect_calls, [(
            'ws://10.0.0.11:8766',
            {
                'subprotocols': ['foxglove.sdk.v1', 'foxglove.websocket.v1'],
                'max_size': 8 * 1024 * 1024,
                'ping_interval': 20,
                'ping_timeout': 20,
                'close_timeout': 5,
            },
        )])
        self.assertEqual(len(websocket.sent), 2)


async def _completed():
    return None


if __name__ == '__main__':
    unittest.main()
