"""Tests for the host-side Foxglove SDK probe helper."""

import asyncio
import importlib.util
import json
from pathlib import Path
import unittest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / 'tools' / 'foxglove_sdk_probe.py'


def load_probe_module():
    if not SCRIPT_PATH.exists():
        return None
    spec = importlib.util.spec_from_file_location('foxglove_sdk_probe', SCRIPT_PATH)
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeWebSocket:
    def __init__(self, server_info, subprotocol='foxglove.sdk.v1'):
        self.subprotocol = subprotocol
        self._received = [json.dumps(server_info)]
        self.sent = []

    async def recv(self):
        return self._received.pop(0)

    async def send(self, message):
        self.sent.append(message)


class FakeConnection:
    def __init__(self, websocket):
        self.websocket = websocket

    async def __aenter__(self):
        return self.websocket

    async def __aexit__(self, exc_type, exc, traceback):
        return False


def sdk_server_info(capabilities=('clientPublish',), encodings=('cdr', 'json')):
    return {
        'op': 'serverInfo',
        'name': 'robot_2',
        'capabilities': list(capabilities),
        'supportedEncodings': list(encodings),
        'metadata': {'fg-library': 'foxglove-sdk-cpp/v0.15.1'},
        'sessionId': 'session-1',
    }


class FoxgloveSdkProbeTest(unittest.TestCase):
    def setUp(self):
        self.probe = load_probe_module()
        self.assertIsNotNone(
            self.probe,
            'tools/foxglove_sdk_probe.py must provide the host-side probe command',
        )

    def test_default_probe_only_reports_sdk_server_info(self):
        websocket = FakeWebSocket(sdk_server_info())
        connect_calls = []
        output = []

        def connect(uri, **kwargs):
            connect_calls.append((uri, kwargs))
            return FakeConnection(websocket)

        result = asyncio.run(self.probe.probe(
            'ws://robot-2:8765',
            connect_factory=connect,
            emit=output.append,
        ))

        self.assertEqual(result['metadata']['fg-library'], 'foxglove-sdk-cpp/v0.15.1')
        self.assertEqual(websocket.sent, [])
        self.assertEqual(connect_calls, [(
            'ws://robot-2:8765',
            {'subprotocols': ['foxglove.sdk.v1'], 'open_timeout': 5},
        )])
        self.assertTrue(any('clientPublish' in line for line in output))

    def test_zero_command_advertises_json_twist_and_sends_zero_frame(self):
        websocket = FakeWebSocket(sdk_server_info())

        def connect(*args, **kwargs):
            return FakeConnection(websocket)

        asyncio.run(self.probe.probe(
            'ws://robot-2:8765',
            send_zero_cmd_vel=True,
            connect_factory=connect,
            emit=lambda message: None,
        ))

        self.assertEqual(json.loads(websocket.sent[0]), {
            'op': 'advertise',
            'channels': [{
                'id': 1,
                'topic': '/cmd_vel',
                'encoding': 'json',
                'schemaName': 'geometry_msgs/msg/Twist',
            }],
        })
        self.assertEqual(websocket.sent[1], (
            b'\x01\x01\x00\x00\x00'
            b'{"linear":{"x":0.0,"y":0.0,"z":0.0},'
            b'"angular":{"x":0.0,"y":0.0,"z":0.0}}'
        ))

    def test_zero_command_requires_json_client_publish_capability(self):
        websocket = FakeWebSocket(sdk_server_info(capabilities=(), encodings=('cdr',)))

        def connect(*args, **kwargs):
            return FakeConnection(websocket)

        with self.assertRaisesRegex(self.probe.ProbeError, 'clientPublish'):
            asyncio.run(self.probe.probe(
                'ws://robot-2:8765',
                send_zero_cmd_vel=True,
                connect_factory=connect,
                emit=lambda message: None,
            ))

        self.assertEqual(websocket.sent, [])


if __name__ == '__main__':
    unittest.main()
