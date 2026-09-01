"""Channel selection, rate limiting, and ROS 2 CDR republishing."""

import json
from threading import Thread
import time
from typing import Any

from fleet_bridge_config.models import QosConfig, TopicConfig

from .protocol import Channel


def qos_kwargs(config: QosConfig) -> dict[str, object]:
    """Return a stable, ROS-independent QoS representation for validation."""

    return {
        'reliability': config.reliability,
        'durability': config.durability,
        'history': config.history,
        'depth': config.depth,
    }


def initialize_rclpy(rclpy_module, no_signal_handlers) -> bool:
    """Initialize a background ROS context without owning process signals."""

    owns_rclpy = not rclpy_module.ok()
    if owns_rclpy:
        rclpy_module.init(
            args=None,
            signal_handler_options=no_signal_handlers,
        )
    return owns_rclpy


def namespace_frame_ids(message: Any, robot_id: str) -> None:
    """Prefix vehicle frame IDs while retaining the fleet-wide map frame."""

    prefix = f'{robot_id}/'
    visited: set[int] = set()

    def namespaced(frame_id: str) -> str:
        normalized = frame_id.lstrip('/')
        if not normalized or normalized == 'map' or normalized.startswith(prefix):
            return normalized
        return f'{prefix}{normalized}'

    def visit(value: Any) -> None:
        if value is None or isinstance(value, (bool, bytes, float, int, str)):
            return
        identity = id(value)
        if identity in visited:
            return
        visited.add(identity)

        if isinstance(value, dict):
            for field, nested in value.items():
                if field in ('frame_id', 'child_frame_id') and isinstance(nested, str):
                    value[field] = namespaced(nested)
                else:
                    visit(nested)
            return

        if isinstance(value, (list, tuple)):
            for nested in value:
                visit(nested)
            return

        fields = set()
        for slot in getattr(type(value), '__slots__', ()):
            if not isinstance(slot, str) or not hasattr(value, slot):
                continue
            fields.add(slot)
            field = slot.lstrip('_')
            nested = getattr(value, slot)
            if field in ('frame_id', 'child_frame_id') and isinstance(nested, str):
                setattr(value, slot, namespaced(nested))
            else:
                visit(nested)

        for field, nested in vars(value).items() if hasattr(value, '__dict__') else ():
            if field in fields:
                continue
            if field in ('frame_id', 'child_frame_id') and isinstance(nested, str):
                setattr(value, field, namespaced(nested))
            else:
                visit(nested)

    visit(message)


class LatestMessageReplay:
    """Retain the latest message and publish it again on a timer."""

    def __init__(self, publish) -> None:
        self._publish = publish
        self._latest = None

    def update(self, message: Any) -> None:
        self._latest = message

    def replay(self) -> bool:
        if self._latest is None:
            return False
        self._publish(self._latest)
        return True


class ChannelSelector:
    """Select only explicitly enabled, type-safe CDR telemetry channels."""

    def __init__(self, topics: tuple[TopicConfig, ...]) -> None:
        self._by_source = {
            topic.source: topic
            for topic in topics
            if topic.enabled
        }

    def select(self, channel: Channel) -> TopicConfig | None:
        topic = self._by_source.get(channel.topic)
        if topic is None:
            return None
        if channel.encoding != 'cdr' or channel.schema_name != topic.message_type:
            return None
        return topic


class RateGate:
    """Apply the optional server-side maximum rate independently per topic."""

    def __init__(self) -> None:
        self._last_allowed_ns: dict[str, int] = {}

    def allow(self, topic: TopicConfig, now_ns: int) -> bool:
        maximum = topic.worker_rate.max_rate_hz
        if maximum is None:
            return True
        minimum_interval_ns = int(1_000_000_000 / maximum)
        previous = self._last_allowed_ns.get(topic.id)
        if previous is not None and now_ns >= previous:
            if now_ns - previous < minimum_interval_ns:
                return False
        self._last_allowed_ns[topic.id] = now_ns
        return True


def _qos_profile(config: QosConfig):
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=config.depth,
        reliability=(
            ReliabilityPolicy.BEST_EFFORT
            if config.reliability == 'best_effort'
            else ReliabilityPolicy.RELIABLE
        ),
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if config.durability == 'transient_local'
            else DurabilityPolicy.VOLATILE
        ),
    )


class RosRepublisher:
    """Deserialize CDR and publish configured ROS messages on Domain 225."""

    def __init__(self, robot_id: str, topics: tuple[TopicConfig, ...], state: Any) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.serialization import deserialize_message
        from rclpy.signals import SignalHandlerOptions
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._deserialize_message = deserialize_message
        self._string_type = String
        self._robot_id = robot_id
        self._state = state
        self._owns_rclpy = initialize_rclpy(
            rclpy,
            SignalHandlerOptions.NO,
        )

        self._node = Node(f'foxglove_ros_worker_{robot_id}')
        self._publishers = {}
        self._message_types = {}
        self._replays = {}
        self._replay_timers = []
        for topic in topics:
            if not topic.enabled:
                continue
            message_type = get_message(topic.message_type)
            self._message_types[topic.id] = message_type
            self._publishers[topic.id] = self._node.create_publisher(
                message_type,
                topic.target,
                _qos_profile(topic.qos),
            )
            if topic.replay_rate_hz is not None:
                self._replays[topic.id] = LatestMessageReplay(
                    self._publishers[topic.id].publish,
                )
                self._replay_timers.append(self._node.create_timer(
                    1.0 / topic.replay_rate_hz,
                    lambda topic_id=topic.id: self._replays[topic_id].replay(),
                ))

        status_qos = QosConfig('reliable', 'volatile', 'keep_last', 1)
        self._status_publisher = self._node.create_publisher(
            String,
            f'/{robot_id}/fleet_bridge/status',
            _qos_profile(status_qos),
        )
        self._status_timer = self._node.create_timer(1.0, self._publish_status)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread: Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = Thread(
            target=self._executor.spin,
            name='foxglove-ros-executor',
            daemon=True,
        )
        self._thread.start()

    def publish(self, topic: TopicConfig, payload: bytes) -> None:
        message_type = self._message_types[topic.id]
        message = self._deserialize_message(payload, message_type)
        namespace_frame_ids(message, self._robot_id)
        self._publishers[topic.id].publish(message)
        replay = self._replays.get(topic.id)
        if replay is not None:
            replay.update(message)

    def _publish_status(self) -> None:
        payload = json.dumps(
            self._state.snapshot(now=time.time()),
            separators=(',', ':'),
            sort_keys=True,
        )
        self._status_publisher.publish(self._string_type(data=payload))

    def close(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()
