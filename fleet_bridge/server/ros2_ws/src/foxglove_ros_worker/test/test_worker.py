import json
from pathlib import Path
import struct
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path.insert(0, str(COMMON))
sys.path.insert(0, str(PACKAGE))

from fleet_bridge_config.models import (
    QosConfig,
    RateConfig,
    TopicConfig,
)
from foxglove_ros_worker.main import FoxgloveWorker
from foxglove_ros_worker.protocol import ProtocolError
from foxglove_ros_worker.state import WorkerState


def odom_topic(worker_rate=None):
    return TopicConfig(
        id='odom',
        enabled=True,
        source='/odom',
        target='/robot_1/odom',
        message_type='nav_msgs/msg/Odometry',
        worker_rate=RateConfig(max_rate_hz=worker_rate),
        qos=QosConfig('best_effort', 'volatile', 'keep_last', 5),
    )


class FakeWebSocket:
    def __init__(self):
        self.sent = []

    async def send(self, payload):
        self.sent.append(payload)


class FakeRepublisher:
    def __init__(self):
        self.messages = []

    def publish(self, topic, payload):
        self.messages.append((topic.id, payload))


class FoxgloveWorkerTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.socket = FakeWebSocket()
        self.republisher = FakeRepublisher()
        self.state = WorkerState('robot_1', freshness_timeout_sec=10.0)
        self.state.connected()
        self.worker = FoxgloveWorker(
            robot_id='robot_1',
            uri='ws://robot-1:8766',
            topics=(odom_topic(),),
            republisher=self.republisher,
            state=self.state,
            wall_clock=lambda: 100.0,
            monotonic_ns=lambda: 1_000_000_000,
        )

    async def test_advertise_subscribes_matching_cdr_and_republishes_binary(self):
        await self.worker.handle_text(json.dumps({
            'op': 'advertise',
            'channels': [{
                'id': 3,
                'topic': '/odom',
                'encoding': 'cdr',
                'schemaName': 'nav_msgs/msg/Odometry',
                'schema': 'schema body',
                'schemaEncoding': 'ros2msg',
            }],
        }), self.socket)

        subscribe = json.loads(self.socket.sent[0])
        self.assertEqual(subscribe['op'], 'subscribe')
        self.assertEqual(subscribe['subscriptions'], [{'id': 1, 'channelId': 3}])

        await self.worker.handle_binary(
            b'\x01' + struct.pack('<IQ', 1, 42) + b'cdr',
        )

        self.assertEqual(self.republisher.messages, [('odom', b'cdr')])
        self.assertEqual(self.state.snapshot(now=100.0)['state'], 'online')

    async def test_mismatched_type_is_not_subscribed(self):
        await self.worker.handle_text(json.dumps({
            'op': 'advertise',
            'channels': [{
                'id': 4,
                'topic': '/odom',
                'encoding': 'cdr',
                'schemaName': 'std_msgs/msg/String',
                'schema': 'schema body',
            }],
        }), self.socket)

        self.assertEqual(self.socket.sent, [])

    async def test_unadvertise_drops_mapping_and_unknown_frame_is_ignored(self):
        await self.worker.handle_text(json.dumps({
            'op': 'advertise',
            'channels': [{
                'id': 3,
                'topic': '/robot_1/odom',
                'encoding': 'cdr',
                'schemaName': 'nav_msgs/msg/Odometry',
                'schema': 'schema body',
            }],
        }), self.socket)
        await self.worker.handle_text(json.dumps({
            'op': 'unadvertise',
            'channelIds': [3],
        }), self.socket)

        await self.worker.handle_binary(
            b'\x01' + struct.pack('<IQ', 1, 42) + b'cdr',
        )

        self.assertEqual(self.republisher.messages, [])

    async def test_server_without_cdr_and_malformed_frame_are_rejected(self):
        with self.assertRaises(ProtocolError):
            await self.worker.handle_text(json.dumps({
                'op': 'serverInfo',
                'name': 'bridge',
                'capabilities': [],
                'supportedEncodings': ['json'],
            }), self.socket)

        with self.assertRaises(ProtocolError):
            await self.worker.handle_binary(b'bad')


if __name__ == '__main__':
    unittest.main()
