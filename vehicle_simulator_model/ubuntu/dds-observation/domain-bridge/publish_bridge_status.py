"""Publish bridge liveness based on telemetry observed after Domain Bridge remapping."""

import argparse
from datetime import datetime, timezone
import json
import os
import time

from render_bridge_config import TELEMETRY_TOPICS, topic_path


VALID_STATES = frozenset({'active', 'idle', 'error'})


def status_payload(state: str, timestamp: str) -> str:
    """Return the intentionally narrow public bridge status payload."""
    if state not in VALID_STATES:
        raise ValueError(f'unsupported bridge state: {state}')
    return json.dumps({'state': state, 'timestamp': timestamp}, separators=(',', ':'))


class BridgeStatusPublisher:
    def __init__(self, bridge_pid: int, central_prefix: str, active_window_seconds: float):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._bridge_pid = bridge_pid
        self._active_window_seconds = active_window_seconds
        self._last_forwarded_at: float | None = None
        self.running = True
        self._node = rclpy.create_node('domain_bridge_status')
        self._publisher = self._node.create_publisher(
            String,
            topic_path(central_prefix, 'fleet/status'),
            10,
        )
        self._string_type = String
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for suffix, message_type in TELEMETRY_TOPICS.items():
            self._node.create_subscription(
                get_message(message_type),
                topic_path(central_prefix, suffix),
                self._on_forwarded_telemetry,
                qos,
            )
        self._node.create_timer(1.0, self._publish_status)

    def _on_forwarded_telemetry(self, _message) -> None:
        self._last_forwarded_at = time.monotonic()

    def _bridge_is_running(self) -> bool:
        try:
            os.kill(self._bridge_pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _publish_status(self) -> None:
        if not self._bridge_is_running():
            state = 'error'
            self.running = False
        elif (
            self._last_forwarded_at is not None
            and time.monotonic() - self._last_forwarded_at <= self._active_window_seconds
        ):
            state = 'active'
        else:
            state = 'idle'
        message = self._string_type()
        message.data = status_payload(
            state,
            datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
        )
        self._publisher.publish(message)

    def close(self) -> None:
        self._node.destroy_node()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--bridge-pid', required=True, type=int)
    parser.add_argument('--central-prefix', required=True)
    parser.add_argument('--active-window-seconds', default=3.0, type=float)
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    publisher = BridgeStatusPublisher(
        args.bridge_pid,
        args.central_prefix,
        args.active_window_seconds,
    )
    try:
        while rclpy.ok() and publisher.running:
            rclpy.spin_once(publisher._node, timeout_sec=0.2)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        publisher.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
