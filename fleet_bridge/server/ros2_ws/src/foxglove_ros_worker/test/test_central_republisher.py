from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path.insert(0, str(COMMON))
sys.path.insert(0, str(PACKAGE))

from fleet_bridge_config.models import CentralTopicConfig, QosConfig
from foxglove_ros_worker.central_republisher import CentralTopicRelay


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}
        self.timers = []

    def create_publisher(self, _message_type, topic, _qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, _message_type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return callback

    def create_timer(self, interval, callback):
        self.timers.append((interval, callback))
        return callback


class CentralTopicRelayTest(unittest.TestCase):
    def test_publishes_cached_map_once_without_periodic_replay(self):
        """A durable map needs one publish; late readers receive its retained sample."""
        topic = CentralTopicConfig(
            id='controller_map',
            enabled=True,
            source='/controller_server/map',
            target='/map',
            message_type='nav_msgs/msg/OccupancyGrid',
            replay_rate_hz=None,
            qos=QosConfig('reliable', 'transient_local', 'keep_last', 1),
        )
        node = FakeNode()
        CentralTopicRelay((topic,), node, lambda _name: object, lambda qos: qos)

        node.subscriptions['/controller_server/map']('map-v1')
        self.assertEqual(node.publishers['/map'].messages, ['map-v1'])
        self.assertEqual(node.timers, [])


if __name__ == '__main__':
    unittest.main()
