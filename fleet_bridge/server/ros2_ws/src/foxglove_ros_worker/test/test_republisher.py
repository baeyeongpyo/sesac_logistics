from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import patch


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path.insert(0, str(COMMON))
sys.path.insert(0, str(PACKAGE))

from fleet_bridge_config.models import (
    QosConfig,
    RateConfig,
    TopicConfig,
)
from foxglove_ros_worker.protocol import Channel
from foxglove_ros_worker.republisher import (
    ChannelSelector,
    LatestMessageReplay,
    RateGate,
    RosRepublisher,
    initialize_rclpy,
    qos_kwargs,
)


def topic(
    *,
    enabled=True,
    worker_rate=None,
    reliability='best_effort',
    durability='volatile',
):
    return TopicConfig(
        id='odom',
        enabled=enabled,
        source='/odom',
        target='/robot_1/odom',
        message_type='nav_msgs/msg/Odometry',
        worker_rate=RateConfig(max_rate_hz=worker_rate),
        qos=QosConfig(
            reliability=reliability,
            durability=durability,
            history='keep_last',
            depth=5,
        ),
    )


def channel(
    *,
    topic_name='/odom',
    encoding='cdr',
    schema_name='nav_msgs/msg/Odometry',
):
    return Channel(
        id=3,
        topic=topic_name,
        encoding=encoding,
        schema_name=schema_name,
        schema='schema body',
        schema_encoding='ros2msg',
    )


class ChannelSelectorTest(unittest.TestCase):
    def test_accepts_only_enabled_cdr_channel_with_matching_type(self):
        selector = ChannelSelector((topic(),))

        self.assertEqual(selector.select(channel()).id, 'odom')
        self.assertIsNone(selector.select(channel(topic_name='/robot_1/odom')))
        self.assertIsNone(selector.select(channel(encoding='json')))
        self.assertIsNone(selector.select(channel(schema_name='std_msgs/msg/String')))
        self.assertIsNone(ChannelSelector((topic(enabled=False),)).select(channel()))

    def test_qos_mapping_preserves_configured_ros_policies(self):
        config = topic(reliability='reliable', durability='transient_local').qos

        self.assertEqual(qos_kwargs(config), {
            'reliability': 'reliable',
            'durability': 'transient_local',
            'history': 'keep_last',
            'depth': 5,
        })


class RateGateTest(unittest.TestCase):
    def test_rate_limit_is_independent_per_topic_and_allows_boundary(self):
        gate = RateGate()
        limited = topic(worker_rate=2.0)

        self.assertTrue(gate.allow(limited, now_ns=1_000_000_000))
        self.assertFalse(gate.allow(limited, now_ns=1_499_999_999))
        self.assertTrue(gate.allow(limited, now_ns=1_500_000_000))

        other = TopicConfig(**{**limited.__dict__, 'id': 'other'})
        self.assertTrue(gate.allow(other, now_ns=1_100_000_000))

    def test_topic_without_worker_rate_is_never_dropped(self):
        gate = RateGate()
        unlimited = topic()

        self.assertTrue(gate.allow(unlimited, now_ns=1))
        self.assertTrue(gate.allow(unlimited, now_ns=1))


class LatestMessageReplayTest(unittest.TestCase):
    def test_replays_only_the_latest_received_message(self):
        published = []
        replay = LatestMessageReplay(published.append)

        self.assertFalse(replay.replay())

        replay.update('first')
        replay.update('latest')

        self.assertTrue(replay.replay())
        self.assertEqual(published, ['latest'])


class FrameIdNamespaceTest(unittest.TestCase):
    def test_publish_namespaces_vehicle_frames_but_preserves_shared_map(self):
        class Header:
            __slots__ = ('frame_id',)

            def __init__(self, frame_id):
                self.frame_id = frame_id

        class Transform:
            __slots__ = ('header', 'child_frame_id')

            def __init__(self, parent, child):
                self.header = Header(parent)
                self.child_frame_id = child

        class TfMessage:
            __slots__ = ('transforms',)

            def __init__(self):
                self.transforms = [
                    Transform('map', 'odom'),
                    Transform('odom', 'base_footprint'),
                ]

        class SensorMessage:
            __slots__ = ('header',)

            def __init__(self):
                self.header = Header('lidar_frame')

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        tf_message = TfMessage()
        sensor_message = SensorMessage()
        tf_publisher = Publisher()
        sensor_publisher = Publisher()
        republisher = type('Republisher', (), {
            '_robot_id': 'robot_2',
            '_message_types': {'tf': object(), 'scan': object()},
            '_deserialize_message': lambda _self, payload, _type: {
                b'tf': tf_message,
                b'scan': sensor_message,
            }[payload],
            '_publishers': {'tf': tf_publisher, 'scan': sensor_publisher},
            '_replays': {},
        })()

        RosRepublisher.publish(republisher, type('Topic', (), {'id': 'tf'})(), b'tf')
        RosRepublisher.publish(republisher, type('Topic', (), {'id': 'scan'})(), b'scan')

        self.assertEqual(tf_message.transforms[0].header.frame_id, 'map')
        self.assertEqual(tf_message.transforms[0].child_frame_id, 'robot_2/odom')
        self.assertEqual(tf_message.transforms[1].header.frame_id, 'robot_2/odom')
        self.assertEqual(
            tf_message.transforms[1].child_frame_id,
            'robot_2/base_footprint',
        )
        self.assertEqual(sensor_message.header.frame_id, 'robot_2/lidar_frame')
        self.assertEqual(tf_publisher.messages, [tf_message])
        self.assertEqual(sensor_publisher.messages, [sensor_message])


class RosRepublisherConstructionTest(unittest.TestCase):
    def test_constructor_retains_robot_id_for_frame_namespace_publishing(self):
        class Header:
            __slots__ = ('_frame_id',)

            def __init__(self, frame_id):
                self._frame_id = frame_id

            @property
            def frame_id(self):
                return self._frame_id

        class Message:
            __slots__ = ('_header',)

            def __init__(self):
                self._header = Header('base_footprint')

            @property
            def header(self):
                return self._header

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        class Node:
            instance = None

            def __init__(self, _name):
                self.publishers = []
                Node.instance = self

            def create_publisher(self, *_args):
                publisher = Publisher()
                self.publishers.append(publisher)
                return publisher

            def create_timer(self, *_args):
                return object()

        class Executor:
            def add_node(self, _node):
                pass

        message = Message()
        rclpy_module = ModuleType('rclpy')
        rclpy_module.ok = lambda: True
        executors_module = ModuleType('rclpy.executors')
        executors_module.SingleThreadedExecutor = Executor
        node_module = ModuleType('rclpy.node')
        node_module.Node = Node
        serialization_module = ModuleType('rclpy.serialization')
        serialization_module.deserialize_message = lambda _payload, _type: message
        signals_module = ModuleType('rclpy.signals')
        signals_module.SignalHandlerOptions = type('SignalHandlerOptions', (), {
            'NO': object(),
        })
        qos_module = ModuleType('rclpy.qos')
        qos_module.DurabilityPolicy = type('DurabilityPolicy', (), {
            'TRANSIENT_LOCAL': object(),
            'VOLATILE': object(),
        })
        qos_module.HistoryPolicy = type('HistoryPolicy', (), {
            'KEEP_LAST': object(),
        })
        qos_module.ReliabilityPolicy = type('ReliabilityPolicy', (), {
            'BEST_EFFORT': object(),
            'RELIABLE': object(),
        })
        qos_module.QoSProfile = lambda **_kwargs: object()
        utilities_module = ModuleType('rosidl_runtime_py.utilities')
        utilities_module.get_message = lambda _message_type: object()
        std_msgs_module = ModuleType('std_msgs.msg')
        std_msgs_module.String = object

        with patch.dict(sys.modules, {
            'rclpy': rclpy_module,
            'rclpy.executors': executors_module,
            'rclpy.node': node_module,
            'rclpy.serialization': serialization_module,
            'rclpy.signals': signals_module,
            'rclpy.qos': qos_module,
            'rosidl_runtime_py.utilities': utilities_module,
            'std_msgs.msg': std_msgs_module,
        }):
            telemetry = topic()
            republisher = RosRepublisher('robot_2', (telemetry,), object())
            republisher.publish(telemetry, b'cdr')

        self.assertEqual(message.header.frame_id, 'robot_2/base_footprint')
        self.assertEqual(Node.instance.publishers[0].messages, [message])


class RosInitializationTest(unittest.TestCase):
    def test_background_executor_disables_rclpy_process_signal_handlers(self):
        class FakeRclpy:
            def __init__(self):
                self.init_calls = []

            def ok(self):
                return False

            def init(self, **kwargs):
                self.init_calls.append(kwargs)

        rclpy = FakeRclpy()
        no_signal_handlers = object()

        owns_rclpy = initialize_rclpy(rclpy, no_signal_handlers)

        self.assertTrue(owns_rclpy)
        self.assertEqual(rclpy.init_calls, [{
            'args': None,
            'signal_handler_options': no_signal_handlers,
        }])


if __name__ == '__main__':
    unittest.main()
