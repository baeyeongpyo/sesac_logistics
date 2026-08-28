"""Republish central server ROS topics, with optional cached replay."""

import argparse
import logging
import os
from threading import Thread
import time
from typing import Callable

from fleet_bridge_config import load_central_topics
from fleet_bridge_config.models import CentralTopicConfig

from .republisher import LatestMessageReplay, _qos_profile, initialize_rclpy


LOGGER = logging.getLogger('foxglove_ros_worker.central_republisher')


class CentralTopicRelay:
    """Republish configured central topics without subscribing to its own output."""

    def __init__(
        self,
        topics: tuple[CentralTopicConfig, ...],
        node,
        message_resolver: Callable[[str], object],
        qos_profile: Callable[[object], object],
    ) -> None:
        self._replays = {}
        self._publishers = {}
        self._subscriptions = []
        self._timers = []
        for topic in topics:
            if not topic.enabled:
                continue
            message_type = message_resolver(topic.message_type)
            publisher = node.create_publisher(
                message_type,
                topic.target,
                qos_profile(topic.qos),
            )
            replay = LatestMessageReplay(publisher.publish)
            self._publishers[topic.id] = publisher
            self._replays[topic.id] = replay
            self._subscriptions.append(node.create_subscription(
                message_type,
                topic.source,
                lambda message, topic_id=topic.id: self._receive(topic_id, message),
                qos_profile(topic.qos),
            ))
            if topic.replay_rate_hz is not None:
                self._timers.append(node.create_timer(
                    1.0 / topic.replay_rate_hz,
                    lambda topic_id=topic.id: self._replays[topic_id].replay(),
                ))

    def _receive(self, topic_id: str, message) -> None:
        self._publishers[topic_id].publish(message)
        self._replays[topic_id].update(message)


class CentralTopicRepublisher:
    """Run central topic relays in a background ROS executor."""

    def __init__(self, topics: tuple[CentralTopicConfig, ...]) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rosidl_runtime_py.utilities import get_message

        self._rclpy = rclpy
        self._owns_rclpy = initialize_rclpy(rclpy, SignalHandlerOptions.NO)
        self._node = Node('fleet_central_topic_republisher')
        self._relay = CentralTopicRelay(
            topics,
            self._node,
            get_message,
            _qos_profile,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread: Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = Thread(
            target=self._executor.spin,
            name='fleet-central-topic-republisher',
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()


def _arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Republish central ROS topics with optional cached replay.',
    )
    parser.add_argument(
        '--central-topics-config',
        default=os.environ.get(
            'CENTRAL_TOPICS_CONFIG',
            '/config/central_topics.yaml',
        ),
    )
    return parser.parse_args(argv)


def main(argv=None) -> None:
    logging.basicConfig(
        level=os.environ.get('LOG_LEVEL', 'INFO'),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    arguments = _arguments(argv)
    republisher = CentralTopicRepublisher(
        load_central_topics(arguments.central_topics_config),
    )
    republisher.start()
    LOGGER.info('central topic republisher started')
    try:
        while republisher._rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        republisher.close()
