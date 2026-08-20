from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path.insert(0, str(COMMON))
sys.path.insert(0, str(PACKAGE))

from fleet_bridge_config.models import (
    FilterConfig,
    QosConfig,
    RateConfig,
    TopicConfig,
)
from foxglove_ros_worker.protocol import Channel
from foxglove_ros_worker.republisher import (
    ChannelSelector,
    RateGate,
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
        uplink='/robot_1/odom',
        target='/robot_1/odom',
        message_type='nav_msgs/msg/Odometry',
        filter=FilterConfig(mode='passthrough'),
        worker_rate=RateConfig(max_rate_hz=worker_rate),
        qos=QosConfig(
            reliability=reliability,
            durability=durability,
            history='keep_last',
            depth=5,
        ),
        debug=True,
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
