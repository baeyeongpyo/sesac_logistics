import asyncio
import json
from pathlib import Path
import struct
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from fleet_bridge_config.models import CommandConfig, VehicleConfig
from foxglove_ros_worker.command import (
    FoxgloveCommandClient,
    NavigationCancelResult,
    StopDeliveryError,
)
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


class HangingWebSocket(FakeWebSocket):
    async def recv(self):
        if self._messages:
            return self._messages.pop(0)
        await asyncio.Event().wait()


class FailingSendWebSocket(FakeWebSocket):
    async def send(self, payload):
        raise ConnectionError('publish failed')


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


def cancel_service_advertisement():
    return {
        'op': 'advertiseServices',
        'services': [{
            'id': 7,
            'name': '/navigate_to_pose/_action/cancel_goal',
            'type': 'action_msgs/srv/CancelGoal',
            'request': {
                'encoding': 'cdr',
                'schemaName': 'action_msgs/srv/CancelGoal_Request',
                'schemaEncoding': 'ros2msg',
                'schema': 'action_msgs/GoalInfo goal_info',
            },
            'response': {
                'encoding': 'cdr',
                'schemaName': 'action_msgs/srv/CancelGoal_Response',
                'schemaEncoding': 'ros2msg',
                'schema': 'int8 return_code',
            },
        }],
    }


def service_response(payload=b'cancel-response'):
    return b'\x03' + struct.pack('<III', 7, 1, 3) + b'cdr' + payload


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

    def test_sends_pose_stamped_to_configured_nav2_goal_topic(self):
        websocket = FakeWebSocket(server_info())
        goal_pose = {
            'header': {
                'stamp': {'sec': 12, 'nanosec': 34},
                'frame_id': 'map',
            },
            'pose': {
                'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            },
        }
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_goal_pose=lambda value: b'pose-stamped' if value == goal_pose else b'',
        )

        asyncio.run(client.send_goal_pose(vehicle(), goal_pose))

        advertisement = json.loads(websocket.sent[0])
        self.assertEqual(advertisement['channels'][0]['topic'], '/goal_pose')
        self.assertEqual(
            advertisement['channels'][0]['schemaName'],
            'geometry_msgs/msg/PoseStamped',
        )
        self.assertEqual(websocket.sent[1], b'\x01\x01\x00\x00\x00pose-stamped')

    def test_rejects_bridge_without_client_publish_before_advertising(self):
        websocket = FakeWebSocket(server_info(capabilities=()))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_twist=lambda linear_x, angular_z: b'cdr',
        )

        with self.assertRaisesRegex(ProtocolError, 'clientPublish'):
            asyncio.run(client.send_twist(vehicle(), 0.1, 0.2, 100))

        self.assertEqual(websocket.sent, [])

    def test_nav2_cancel_calls_advertised_cancel_service(self):
        websocket = FakeWebSocket(server_info(capabilities=('services',)))
        websocket._messages.extend([
            json.dumps(cancel_service_advertisement()),
            service_response(),
        ])
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_cancel_request=lambda: b'cancel-request',
            deserialize_cancel_response=lambda payload: NavigationCancelResult(
                return_code=0,
                goals_canceling=1 if payload == b'cancel-response' else 0,
            ),
        )

        result = asyncio.run(client.cancel_navigation(vehicle()))

        self.assertEqual(websocket.sent, [
            b'\x02' + struct.pack('<III', 7, 1, 3) + b'cdr' + b'cancel-request',
        ])
        self.assertEqual(result, NavigationCancelResult(0, 1))

    def test_nav2_cancel_times_out_when_hidden_service_is_not_advertised(self):
        websocket = HangingWebSocket(server_info(capabilities=('services',)))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            service_timeout_sec=0.001,
        )

        with self.assertRaisesRegex(ProtocolError, 'timed out'):
            asyncio.run(client.cancel_navigation(vehicle()))

    def test_nav2_cancel_times_out_when_server_info_is_not_received(self):
        websocket = HangingWebSocket(server_info(capabilities=('services',)))
        websocket._messages.clear()
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            service_timeout_sec=0.001,
        )

        with self.assertRaisesRegex(ProtocolError, 'serverInfo timed out'):
            asyncio.run(asyncio.wait_for(
                client.cancel_navigation(vehicle()),
                timeout=0.05,
            ))

    def test_nav2_cancel_times_out_when_service_does_not_respond(self):
        websocket = HangingWebSocket(server_info(capabilities=('services',)))
        websocket._messages.append(json.dumps(cancel_service_advertisement()))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_cancel_request=lambda: b'cancel-request',
            service_timeout_sec=0.001,
        )

        with self.assertRaisesRegex(ProtocolError, 'response timed out'):
            asyncio.run(asyncio.wait_for(
                client.cancel_navigation(vehicle()),
                timeout=0.05,
            ))

    def test_nav2_cancel_rejects_malformed_cdr_response(self):
        websocket = FakeWebSocket(
            server_info(capabilities=('services',)),
            subprotocol='foxglove.sdk.v1',
        )
        websocket._messages.extend([
            json.dumps(cancel_service_advertisement()),
            service_response(payload=b'not-cdr'),
        ])
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(websocket),
            serialize_cancel_request=lambda: b'cancel-request',
            deserialize_cancel_response=lambda payload: _raise(ValueError('bad CDR')),
        )

        with self.assertRaisesRegex(ProtocolError, 'invalid Nav2 cancel CDR response'):
            asyncio.run(client.cancel_navigation(vehicle()))

    def test_stop_cancels_nav2_and_sends_zero_twist(self):
        cancel_socket = FakeWebSocket(server_info(capabilities=('services',)))
        cancel_socket._messages.extend([
            json.dumps(cancel_service_advertisement()),
            service_response(),
        ])
        cmd_vel_socket = FakeWebSocket(server_info())
        sockets = iter((cancel_socket, cmd_vel_socket))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(next(sockets)),
            serialize_twist=lambda linear_x, angular_z: (
                f'{linear_x}:{angular_z}'.encode('ascii')
            ),
            serialize_cancel_request=lambda: b'cancel-request',
            deserialize_cancel_response=lambda payload: NavigationCancelResult(0, 1),
        )

        asyncio.run(client.stop(vehicle()))

        self.assertEqual(len(cancel_socket.sent), 1)
        self.assertEqual(cmd_vel_socket.sent[-1], b'\x01\x01\x00\x00\x000.0:0.0')

    def test_stop_still_sends_zero_twist_when_nav2_cancel_fails(self):
        cancel_socket = FakeWebSocket(server_info(capabilities=()))
        cmd_vel_socket = FakeWebSocket(server_info())
        sockets = iter((cancel_socket, cmd_vel_socket))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(next(sockets)),
            serialize_twist=lambda linear_x, angular_z: (
                f'{linear_x}:{angular_z}'.encode('ascii')
            ),
            serialize_cancel_request=lambda: b'cancel-request',
        )

        with self.assertRaisesRegex(StopDeliveryError, 'Nav2 cancel'):
            asyncio.run(client.stop(vehicle()))

        self.assertEqual(cmd_vel_socket.sent[-1], b'\x01\x01\x00\x00\x000.0:0.0')

    def test_stop_still_cancels_nav2_when_cmd_vel_stop_fails(self):
        cancel_socket = FakeWebSocket(server_info(capabilities=('services',)))
        cancel_socket._messages.extend([
            json.dumps(cancel_service_advertisement()),
            service_response(),
        ])
        cmd_vel_socket = FailingSendWebSocket(server_info())
        sockets = iter((cancel_socket, cmd_vel_socket))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(next(sockets)),
            serialize_twist=lambda linear_x, angular_z: b'zero-twist',
            serialize_cancel_request=lambda: b'cancel-request',
            deserialize_cancel_response=lambda payload: NavigationCancelResult(0, 1),
        )

        with self.assertRaisesRegex(StopDeliveryError, 'cmd_vel stop'):
            asyncio.run(client.stop(vehicle()))

        self.assertEqual(len(cancel_socket.sent), 1)

    def test_stop_finishes_when_nav2_server_info_is_not_received(self):
        cancel_socket = HangingWebSocket(server_info(capabilities=('services',)))
        cancel_socket._messages.clear()
        cmd_vel_socket = FakeWebSocket(server_info())
        sockets = iter((cancel_socket, cmd_vel_socket))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(next(sockets)),
            serialize_twist=lambda linear_x, angular_z: b'zero-twist',
            service_timeout_sec=0.001,
        )

        with self.assertRaisesRegex(StopDeliveryError, 'serverInfo timed out'):
            asyncio.run(asyncio.wait_for(client.stop(vehicle()), timeout=0.05))

        self.assertEqual(cmd_vel_socket.sent[-1], b'\x01\x01\x00\x00\x00zero-twist')

    def test_stop_finishes_when_cmd_vel_server_info_is_not_received(self):
        cancel_socket = FakeWebSocket(server_info(capabilities=('services',)))
        cancel_socket._messages.extend([
            json.dumps(cancel_service_advertisement()),
            service_response(),
        ])
        cmd_vel_socket = HangingWebSocket(server_info())
        cmd_vel_socket._messages.clear()
        sockets = iter((cancel_socket, cmd_vel_socket))
        client = FoxgloveCommandClient(
            connect_factory=lambda *args, **kwargs: FakeConnection(next(sockets)),
            serialize_twist=lambda linear_x, angular_z: b'zero-twist',
            serialize_cancel_request=lambda: b'cancel-request',
            deserialize_cancel_response=lambda payload: NavigationCancelResult(0, 1),
            service_timeout_sec=0.001,
        )

        with self.assertRaisesRegex(StopDeliveryError, 'serverInfo timed out'):
            asyncio.run(asyncio.wait_for(client.stop(vehicle()), timeout=0.05))

        self.assertEqual(len(cancel_socket.sent), 1)

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
            asyncio.run(client.stop_cmd_vel(vehicle()))
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


def _raise(error):
    raise error


if __name__ == '__main__':
    unittest.main()
