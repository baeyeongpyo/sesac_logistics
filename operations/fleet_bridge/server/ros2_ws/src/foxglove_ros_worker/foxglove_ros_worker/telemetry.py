"""Persist the latest fleet pose and raw battery telemetry."""

import argparse
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import sqlite3
import threading
import time
from typing import Any, Callable

from fleet_bridge_config.loader import load_fleet
from fleet_bridge_config.models import FleetConfig


TF_AUTHORITY = 'fleet_telemetry_writer'


def active_robot_ids(fleet: FleetConfig) -> tuple[str, ...]:
    """Return only the configured vehicles whose telemetry is enabled."""

    return tuple(vehicle.id for vehicle in fleet.vehicles if vehicle.enabled)


def quaternion_to_yaw(*, x: float, y: float, z: float, w: float) -> float:
    """Return the Z-axis heading represented by a quaternion in radians."""

    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class TfFreshness:
    """Track when each robot last supplied an accepted dynamic TF message."""

    def __init__(
        self,
        *,
        timeout_sec: float,
        monotonic_time: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('TF freshness timeout must be greater than zero')
        self._timeout_sec = timeout_sec
        self._monotonic_time = monotonic_time
        self._last_received_at: dict[str, float] = {}

    def record(self, robot_id: str) -> None:
        self._last_received_at[robot_id] = self._monotonic_time()

    def is_fresh(self, robot_id: str) -> bool:
        last_received_at = self._last_received_at.get(robot_id)
        return (
            last_received_at is not None
            and self._monotonic_time() - last_received_at <= self._timeout_sec
        )


def transform_belongs_to_robot(transform: Any, robot_id: str) -> bool:
    """Allow only the shared map or frames owned by the source robot."""

    header = getattr(transform, 'header', None)
    parent = getattr(header, 'frame_id', None)
    child = getattr(transform, 'child_frame_id', None)
    if not isinstance(parent, str) or not isinstance(child, str):
        return False

    parent = parent.lstrip('/')
    child = child.lstrip('/')
    prefix = f'{robot_id}/'
    return child.startswith(prefix) and (
        parent == 'map' or parent.startswith(prefix)
    )


class TelemetryStore:
    """Store one current pose and one current battery value per robot."""

    def __init__(self, database_path: str | Path) -> None:
        self._connection = sqlite3.connect(
            str(database_path),
            check_same_thread=False,
        )
        self._connection.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        with self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS robot_pose (
                    robot_id TEXT PRIMARY KEY,
                    x_m REAL NOT NULL,
                    y_m REAL NOT NULL,
                    yaw_rad REAL NOT NULL,
                    observed_at TEXT,
                    received_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS robot_battery (
                    robot_id TEXT PRIMARY KEY,
                    battery_raw INTEGER NOT NULL,
                    received_at TEXT NOT NULL
                );
                """
            )

    def close(self) -> None:
        with self._lock:
            self._connection.close()

    def upsert_pose(
        self,
        robot_id: str,
        *,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        observed_at: str | None,
        received_at: str,
    ) -> None:
        _validate_robot_id(robot_id)
        _validate_finite('x_m', x_m)
        _validate_finite('y_m', y_m)
        _validate_finite('yaw_rad', yaw_rad)
        _validate_timestamp('received_at', received_at)
        if observed_at is not None:
            _validate_timestamp('observed_at', observed_at)
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO robot_pose (
                    robot_id, x_m, y_m, yaw_rad, observed_at, received_at
                ) VALUES (?, ?, ?, ?, ?, ?)
                ON CONFLICT(robot_id) DO UPDATE SET
                    x_m = excluded.x_m,
                    y_m = excluded.y_m,
                    yaw_rad = excluded.yaw_rad,
                    observed_at = excluded.observed_at,
                    received_at = excluded.received_at
                """,
                (robot_id, x_m, y_m, yaw_rad, observed_at, received_at),
            )

    def upsert_battery(
        self,
        robot_id: str,
        *,
        battery_raw: int,
        received_at: str,
    ) -> None:
        _validate_robot_id(robot_id)
        if (
            isinstance(battery_raw, bool)
            or not isinstance(battery_raw, int)
            or battery_raw < 0
            or battery_raw > 65535
        ):
            raise ValueError('battery_raw must be an unsigned 16-bit integer')
        _validate_timestamp('received_at', received_at)
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO robot_battery (robot_id, battery_raw, received_at)
                VALUES (?, ?, ?)
                ON CONFLICT(robot_id) DO UPDATE SET
                    battery_raw = excluded.battery_raw,
                    received_at = excluded.received_at
                """,
                (robot_id, battery_raw, received_at),
            )

    def fetch_pose(self, robot_id: str) -> dict[str, object] | None:
        return self._fetch('robot_pose', robot_id)

    def fetch_battery(self, robot_id: str) -> dict[str, object] | None:
        return self._fetch('robot_battery', robot_id)

    def _fetch(self, table: str, robot_id: str) -> dict[str, object] | None:
        with self._lock:
            row = self._connection.execute(
                f'SELECT * FROM {table} WHERE robot_id = ?',
                (robot_id,),
            ).fetchone()
        return dict(row) if row is not None else None


class FleetTelemetryRecorder:
    """Convert current ROS-shaped messages into the two telemetry rows."""

    def __init__(
        self,
        store: TelemetryStore,
        *,
        lookup_transform: Callable[[str], Any],
        received_at: Callable[[], str],
    ) -> None:
        self._store = store
        self._lookup_transform = lookup_transform
        self._received_at = received_at

    @property
    def store(self) -> TelemetryStore:
        return self._store

    def record_latest_pose(
        self,
        robot_id: str,
        *,
        received_at: str | None = None,
    ) -> None:
        transform_stamped = self._lookup_transform(robot_id)
        transform = transform_stamped.transform
        translation = transform.translation
        rotation = transform.rotation
        self._store.upsert_pose(
            robot_id,
            x_m=translation.x,
            y_m=translation.y,
            yaw_rad=quaternion_to_yaw(
                x=rotation.x,
                y=rotation.y,
                z=rotation.z,
                w=rotation.w,
            ),
            observed_at=_stamp_to_utc_iso(transform_stamped.header.stamp),
            received_at=received_at if received_at is not None else self._received_at(),
        )

    def record_battery(self, robot_id: str, message: Any) -> None:
        self._store.upsert_battery(
            robot_id,
            battery_raw=message.data,
            received_at=self._received_at(),
        )


def utc_now_iso() -> str:
    """Return the writer's current wall-clock time in a stable UTC format."""

    return datetime.now(timezone.utc).isoformat(timespec='microseconds').replace('+00:00', 'Z')


def _positive_rate(value: str) -> float:
    try:
        rate = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('pose write rate must be a number') from error
    if not math.isfinite(rate) or rate <= 0:
        raise argparse.ArgumentTypeError('pose write rate must be greater than zero')
    return rate


def _positive_timeout(value: str) -> float:
    try:
        timeout = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            'TF freshness timeout must be a number',
        ) from error
    if not math.isfinite(timeout) or timeout <= 0:
        raise argparse.ArgumentTypeError(
            'TF freshness timeout must be greater than zero',
        )
    return timeout


def _arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Persist the latest namespaced fleet pose and battery telemetry.',
    )
    parser.add_argument(
        '--fleet-config',
        default=os.environ.get('FLEET_CONFIG', '/config/fleet.yaml'),
    )
    parser.add_argument(
        '--database-path',
        default=os.environ.get('TELEMETRY_DB_PATH', '/data/fleet_telemetry.db'),
    )
    parser.add_argument(
        '--pose-write-rate-hz',
        type=_positive_rate,
        default=_positive_rate(os.environ.get('TELEMETRY_POSE_WRITE_RATE_HZ', '5')),
    )
    parser.add_argument(
        '--tf-freshness-timeout-sec',
        type=_positive_timeout,
        default=_positive_timeout(
            os.environ.get('TELEMETRY_TF_FRESHNESS_TIMEOUT_SEC', '10'),
        ),
    )
    return parser.parse_known_args(argv)


class FleetTelemetryWriter:
    """ROS 2 adapter that records only the configured fleet's latest telemetry."""

    def __init__(
        self,
        store: TelemetryStore,
        robot_ids: tuple[str, ...],
        *,
        pose_write_rate_hz: float,
        tf_freshness_timeout_sec: float,
        monotonic_time: Callable[[], float] = time.monotonic,
        received_at: Callable[[], str] = utc_now_iso,
    ) -> None:
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from rclpy.time import Time
        from std_msgs.msg import UInt16
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformException

        self._node = Node('fleet_telemetry_writer')
        self._robot_ids = robot_ids
        self._time_type = Time
        self._transform_exception = TransformException
        self._tf_freshness = TfFreshness(
            timeout_sec=tf_freshness_timeout_sec,
            monotonic_time=monotonic_time,
        )
        self._stale_robot_ids: set[str] = set()
        self._lookup_failed_robot_ids: set[str] = set()
        self._rejected_transform_frames: set[tuple[str, str, str]] = set()
        self._pose_received_at: dict[str, str] = {}
        self._buffer = Buffer(node=self._node)
        self._recorder = FleetTelemetryRecorder(
            store,
            lookup_transform=self._lookup_transform,
            received_at=received_at,
        )
        self._received_at = received_at
        dynamic_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        battery_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = []
        for robot_id in robot_ids:
            self._subscriptions.extend((
                self._node.create_subscription(
                    TFMessage,
                    f'/{robot_id}/tf',
                    lambda message, robot_id=robot_id: self._dynamic_tf_callback(
                        robot_id,
                        message,
                    ),
                    dynamic_tf_qos,
                ),
                self._node.create_subscription(
                    TFMessage,
                    f'/{robot_id}/tf_static',
                    lambda message, robot_id=robot_id: self._static_tf_callback(
                        robot_id,
                        message,
                    ),
                    static_tf_qos,
                ),
                self._node.create_subscription(
                    UInt16,
                    f'/{robot_id}/ros_robot_controller/battery',
                    lambda message, robot_id=robot_id: self._record_battery(
                        robot_id,
                        message,
                    ),
                    battery_qos,
                ),
            ))
        self._pose_timer = self._node.create_timer(
            1.0 / pose_write_rate_hz,
            self._record_all_poses,
        )

    @property
    def node(self):
        return self._node

    def destroy_node(self) -> None:
        self._recorder.store.close()
        self._node.destroy_node()

    def _dynamic_tf_callback(self, robot_id: str, message: Any) -> None:
        self._accept_transforms(robot_id, message, static=False)

    def _static_tf_callback(self, robot_id: str, message: Any) -> None:
        self._accept_transforms(robot_id, message, static=True)

    def _accept_transforms(
        self,
        robot_id: str,
        message: Any,
        *,
        static: bool,
    ) -> None:
        setter = (
            self._buffer.set_transform_static
            if static
            else self._buffer.set_transform
        )
        accepted_transform = False
        for transform in message.transforms:
            if not transform_belongs_to_robot(transform, robot_id):
                self._warn_rejected_transform(robot_id, transform)
                continue
            setter(transform, TF_AUTHORITY)
            accepted_transform = True
        if accepted_transform and not static:
            self._tf_freshness.record(robot_id)
            self._pose_received_at[robot_id] = self._received_at()

    def _warn_rejected_transform(self, robot_id: str, transform: Any) -> None:
        header = getattr(transform, 'header', None)
        parent = str(getattr(header, 'frame_id', ''))
        child = str(getattr(transform, 'child_frame_id', ''))
        rejection_key = (robot_id, parent, child)
        if rejection_key not in self._rejected_transform_frames:
            self._rejected_transform_frames.add(rejection_key)
            self._node.get_logger().warning(
                f'ignored TF on {robot_id} source: {parent} -> {child}',
            )

    def _lookup_transform(self, robot_id: str) -> Any:
        return self._buffer.lookup_transform(
            'map',
            f'{robot_id}/base_footprint',
            self._time_type(),
        )

    def _record_all_poses(self) -> None:
        for robot_id in self._robot_ids:
            if not self._tf_freshness.is_fresh(robot_id):
                if robot_id not in self._stale_robot_ids:
                    self._stale_robot_ids.add(robot_id)
                    self._node.get_logger().warning(
                        f'pose stale for {robot_id}: no accepted dynamic TF within '
                        'the configured freshness timeout',
                    )
                continue
            self._stale_robot_ids.discard(robot_id)
            try:
                self._recorder.record_latest_pose(
                    robot_id,
                    received_at=self._pose_received_at[robot_id],
                )
            except self._transform_exception as error:
                if robot_id not in self._lookup_failed_robot_ids:
                    self._lookup_failed_robot_ids.add(robot_id)
                    self._node.get_logger().warning(
                        f'pose unavailable for {robot_id}: {error}',
                    )
            except ValueError as error:
                self._node.get_logger().warning(
                    f'pose rejected for {robot_id}: {error}',
                )
            else:
                self._lookup_failed_robot_ids.discard(robot_id)

    def _record_battery(self, robot_id: str, message: Any) -> None:
        try:
            self._recorder.record_battery(robot_id, message)
        except ValueError as error:
            self._node.get_logger().warning(
                f'battery rejected for {robot_id}: {error}',
            )


def main(argv=None) -> None:
    import rclpy

    arguments, ros_arguments = _arguments(argv)
    fleet = load_fleet(arguments.fleet_config, os.environ)
    writer = None
    rclpy.init(args=ros_arguments)
    try:
        writer = FleetTelemetryWriter(
            TelemetryStore(arguments.database_path),
            active_robot_ids(fleet),
            pose_write_rate_hz=arguments.pose_write_rate_hz,
            tf_freshness_timeout_sec=arguments.tf_freshness_timeout_sec,
        )
        rclpy.spin(writer.node)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _stamp_to_utc_iso(stamp: Any) -> str:
    seconds = stamp.sec
    nanoseconds = stamp.nanosec
    if (
        isinstance(seconds, bool)
        or not isinstance(seconds, int)
        or isinstance(nanoseconds, bool)
        or not isinstance(nanoseconds, int)
        or nanoseconds < 0
        or nanoseconds >= 1_000_000_000
    ):
        raise ValueError('TF stamp must contain integer sec and nanosec values')
    base = datetime.fromtimestamp(seconds, timezone.utc).strftime('%Y-%m-%dT%H:%M:%S')
    return f'{base}.{nanoseconds:09d}Z'


def _validate_robot_id(robot_id: str) -> None:
    if not isinstance(robot_id, str) or not robot_id:
        raise ValueError('robot_id must be a non-empty string')


def _validate_finite(name: str, value: float) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    if not math.isfinite(float(value)):
        raise ValueError(f'{name} must be a finite number')


def _validate_timestamp(name: str, value: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f'{name} must be a non-empty string')
