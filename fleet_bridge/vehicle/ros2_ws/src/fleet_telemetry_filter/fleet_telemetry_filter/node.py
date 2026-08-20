import time

from fleet_bridge_config.loader import load_telemetry

from .launch_config import forwarded_topics
from .policy import ForwardPolicy


def cleanup_rclpy(node, rclpy_module, interrupted):
    """Avoid double-destroying waitables already interrupted by rclpy."""

    if not interrupted:
        node.destroy_node()
    if rclpy_module.ok():
        rclpy_module.shutdown()


def _qos_profile(config):
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


def _callback(publisher, policy):
    def forward(message):
        if policy.should_forward(message, time.monotonic_ns()):
            publisher.publish(message)

    return forward


def create_node():
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    class TelemetryFilterNode(Node):
        def __init__(self):
            super().__init__('fleet_telemetry_filter')
            self.declare_parameter('robot_id', '')
            self.declare_parameter('telemetry_config', '/config/telemetry.yaml')
            robot_id = self.get_parameter('robot_id').value
            config_path = self.get_parameter('telemetry_config').value
            if not robot_id:
                raise ValueError('robot_id parameter is required')
            topics = load_telemetry(config_path, robot_id)
            self._publishers = []
            self._subscriptions = []

            for topic in forwarded_topics(topics):
                message_type = get_message(topic.message_type)
                qos = _qos_profile(topic.qos)
                publisher = self.create_publisher(message_type, topic.uplink, qos)
                subscription = self.create_subscription(
                    message_type,
                    topic.source,
                    _callback(publisher, ForwardPolicy(topic.filter)),
                    qos,
                )
                self._publishers.append(publisher)
                self._subscriptions.append(subscription)
                self.get_logger().info(
                    f'filtering {topic.source} -> {topic.uplink} '
                    f'with mode={topic.filter.mode}',
                )

    return TelemetryFilterNode()


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = create_node()
    interrupted = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        cleanup_rclpy(node, rclpy, interrupted)


if __name__ == '__main__':
    main()
