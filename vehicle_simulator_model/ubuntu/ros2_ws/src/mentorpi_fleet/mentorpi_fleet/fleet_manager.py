#!/usr/bin/env python3
"""Central Domain 215 manager for physical fleet bridge workers."""

import argparse
import json
import time
from pathlib import Path

from mentorpi_fleet.fleet_state import FleetPresence
from mentorpi_fleet.registry import RegistryValidationError, enabled_vehicles, load_registry
from mentorpi_fleet.worker_manager import BridgeWorkerManager


class FleetManagerNode:
    """rclpy node wrapper kept separate from testable registry/state logic."""

    def __init__(self, registry_path: Path, runtime_dir: Path, timeout_seconds: float):
        import rclpy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._node = rclpy.create_node('fleet_manager')
        self._registry_path = registry_path
        self._last_registry_mtime: int | None = None
        self._subscriptions = {}
        self._presence = FleetPresence(timeout_seconds)
        self._workers: BridgeWorkerManager | None = None
        self._control_domain: int | None = None
        self._odom_type = Odometry
        self._status_publisher = self._node.create_publisher(String, '/fleet/status', 10)
        self._string_type = String
        self._node.create_timer(1.0, self._tick)
        self._reload_registry()

    @property
    def node(self):
        return self._node

    def _reload_registry(self) -> None:
        try:
            mtime = self._registry_path.stat().st_mtime_ns
        except OSError as error:
            self._node.get_logger().error(f'fleet registry unavailable: {error}')
            return
        if self._last_registry_mtime == mtime:
            return
        try:
            registry = load_registry(self._registry_path)
        except RegistryValidationError as error:
            self._node.get_logger().error(f'fleet registry rejected; keeping current workers: {error}')
            return
        if self._control_domain is not None and registry.control_domain != self._control_domain:
            self._node.get_logger().error(
                'fleet registry control_domain changed at runtime; keeping current workers'
            )
            return
        if self._workers is None:
            self._control_domain = registry.control_domain
            self._workers = BridgeWorkerManager(registry.control_domain, self._runtime_dir)
        vehicles = enabled_vehicles(registry, kind='physical')
        self._workers.reconcile(vehicles)
        self._presence.reconcile(vehicles, kind='physical')
        desired_ids = {vehicle.vehicle_id for vehicle in vehicles}
        for vehicle_id in tuple(self._subscriptions):
            if vehicle_id not in desired_ids:
                self._node.destroy_subscription(self._subscriptions.pop(vehicle_id))
        for vehicle in vehicles:
            if vehicle.vehicle_id not in self._subscriptions:
                self._subscriptions[vehicle.vehicle_id] = self._node.create_subscription(
                    self._odom_type,
                    f'{vehicle.namespace}/odom',
                    lambda message, vehicle_id=vehicle.vehicle_id: self._on_odom(vehicle_id),
                    10,
                )
        self._last_registry_mtime = mtime

    def _on_odom(self, vehicle_id: str) -> None:
        self._presence.record_odom(vehicle_id, time.monotonic())

    def _tick(self) -> None:
        self._reload_registry()
        message = self._string_type()
        message.data = json.dumps({'vehicles': self._presence.snapshot(time.monotonic())}, separators=(',', ':'))
        self._status_publisher.publish(message)

    def shutdown(self) -> None:
        if self._workers is not None:
            self._workers.stop_all()
        self._node.destroy_node()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--registry', required=True, type=Path)
    parser.add_argument('--runtime-dir', default=Path('/run/mentorpi-fleet'), type=Path)
    parser.add_argument('--odom-timeout', default=3.0, type=float)
    args = parser.parse_args(argv)

    import rclpy
    rclpy.init()
    manager = FleetManagerNode(args.registry, args.runtime_dir, args.odom_timeout)
    try:
        rclpy.spin(manager.node)
    except KeyboardInterrupt:
        pass
    finally:
        manager.shutdown()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
