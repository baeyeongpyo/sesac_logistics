"""Publish the Fleet Bridge's static central map without a Nav2 relay."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any

import yaml


class MapConfigError(ValueError):
    """Raised when a checked-in map YAML or PGM cannot be published safely."""


@dataclass(frozen=True)
class LoadedMap:
    width: int
    height: int
    resolution: float
    origin: tuple[float, float, float]
    data: tuple[int, ...]


def _mapping(value: object, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise MapConfigError(f'{location} must be a mapping')
    return value


def _number(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapConfigError(f'{location} must be a number')
    number = float(value)
    if not math.isfinite(number):
        raise MapConfigError(f'{location} must be finite')
    return number


def _positive_number(value: object, location: str) -> float:
    number = _number(value, location)
    if number <= 0:
        raise MapConfigError(f'{location} must be greater than zero')
    return number


def _origin(value: object) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise MapConfigError('map.origin must contain x, y, and yaw')
    return tuple(
        _number(component, f'map.origin[{index}]')
        for index, component in enumerate(value)
    )  # type: ignore[return-value]


def _read_token(data: bytes, offset: int) -> tuple[bytes | None, int]:
    """Read an ASCII PGM header/token while accepting comments between tokens."""

    length = len(data)
    while True:
        while offset < length and chr(data[offset]).isspace():
            offset += 1
        if offset < length and data[offset] == ord('#'):
            newline = data.find(b'\n', offset)
            offset = length if newline < 0 else newline + 1
            continue
        break
    if offset >= length:
        return None, offset
    start = offset
    while offset < length and not chr(data[offset]).isspace() and data[offset] != ord('#'):
        offset += 1
    if start == offset:
        raise MapConfigError('PGM token is empty')
    return data[start:offset], offset


def _read_positive_integer(data: bytes, offset: int, label: str) -> tuple[int, int]:
    token, offset = _read_token(data, offset)
    if token is None:
        raise MapConfigError(f'PGM {label} is missing')
    try:
        value = int(token)
    except ValueError as error:
        raise MapConfigError(f'PGM {label} must be an integer') from error
    if value <= 0:
        raise MapConfigError(f'PGM {label} must be greater than zero')
    return value, offset


def _read_pgm(path: Path) -> tuple[int, int, int, tuple[int, ...]]:
    data = path.read_bytes()
    magic, offset = _read_token(data, 0)
    if magic not in (b'P2', b'P5'):
        raise MapConfigError('map image must be an 8-bit P2 or P5 PGM')
    width, offset = _read_positive_integer(data, offset, 'width')
    height, offset = _read_positive_integer(data, offset, 'height')
    max_value, offset = _read_positive_integer(data, offset, 'max value')
    if max_value > 255:
        raise MapConfigError('16-bit PGM maps are not supported')
    pixel_count = width * height

    if magic == b'P5':
        if offset >= len(data) or not chr(data[offset]).isspace():
            raise MapConfigError('PGM binary header must end with whitespace')
        if data[offset:offset + 2] == b'\r\n':
            offset += 2
        else:
            offset += 1
        pixels = tuple(data[offset:])
    else:
        values = []
        while True:
            token, offset = _read_token(data, offset)
            if token is None:
                break
            try:
                value = int(token)
            except ValueError as error:
                raise MapConfigError('PGM pixel must be an integer') from error
            values.append(value)
        pixels = tuple(values)

    if len(pixels) != pixel_count:
        raise MapConfigError(
            f'PGM pixel count is {len(pixels)} but expected {pixel_count}',
        )
    if any(pixel < 0 or pixel > max_value for pixel in pixels):
        raise MapConfigError('PGM pixel is outside its declared max value')
    return width, height, max_value, pixels


def _occupancy_value(
    pixel: int,
    max_value: int,
    *,
    negate: bool,
    occupied_threshold: float,
    free_threshold: float,
) -> int:
    occupancy = pixel / max_value if negate else 1.0 - (pixel / max_value)
    if occupancy > occupied_threshold:
        return 100
    if occupancy < free_threshold:
        return 0
    return -1


def load_map(path: Path | str) -> LoadedMap:
    """Load one Nav2-compatible trinary YAML/PGM map into OccupancyGrid data."""

    yaml_path = Path(path)
    with yaml_path.open('r', encoding='utf-8') as stream:
        document = _mapping(yaml.safe_load(stream), 'map YAML')
    image = document.get('image')
    if not isinstance(image, str) or not image:
        raise MapConfigError('map.image must be a non-empty path')
    mode = document.get('mode', 'trinary')
    if mode != 'trinary':
        raise MapConfigError('map.mode must be trinary for the direct publisher')
    resolution = _positive_number(document.get('resolution'), 'map.resolution')
    origin = _origin(document.get('origin'))
    negate_value = _number(document.get('negate', 0), 'map.negate')
    if negate_value not in (0.0, 1.0):
        raise MapConfigError('map.negate must be 0 or 1')
    occupied_threshold = _number(document.get('occupied_thresh'), 'map.occupied_thresh')
    free_threshold = _number(document.get('free_thresh'), 'map.free_thresh')
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise MapConfigError('map free/occupied thresholds must satisfy 0 <= free < occupied <= 1')

    width, height, max_value, pixels = _read_pgm(yaml_path.parent / image)
    rows_from_bottom = range(height - 1, -1, -1)
    grid_data = tuple(
        _occupancy_value(
            pixels[(row * width) + column],
            max_value,
            negate=bool(negate_value),
            occupied_threshold=occupied_threshold,
            free_threshold=free_threshold,
        )
        for row in rows_from_bottom
        for column in range(width)
    )
    return LoadedMap(
        width=width,
        height=height,
        resolution=resolution,
        origin=origin,
        data=grid_data,
    )


class CentralMapPublisher:
    """Keep the map and its visualization frame available to late Foxglove clients."""

    def __init__(self, map_yaml: Path | str):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from tf2_msgs.msg import TFMessage

        class _Node(Node):
            pass

        self._node = _Node('fleet_map_publisher')
        self._map = load_map(map_yaml)
        self._occupancy_grid_type = OccupancyGrid
        self._transform_stamped_type = TransformStamped
        self._tf_message_type = TFMessage
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._map_publisher = self._node.create_publisher(OccupancyGrid, '/map', map_qos)
        self._tf_publisher = self._node.create_publisher(TFMessage, '/tf', tf_qos)
        self._node.create_timer(1.0, self.publish_map)
        self._node.create_timer(0.1, self.publish_transform)
        self.publish_map()
        self.publish_transform()

    @property
    def node(self):
        return self._node

    def publish_map(self) -> None:
        message = self._occupancy_grid_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.info.map_load_time = message.header.stamp
        message.info.resolution = self._map.resolution
        message.info.width = self._map.width
        message.info.height = self._map.height
        message.info.origin.position.x = self._map.origin[0]
        message.info.origin.position.y = self._map.origin[1]
        message.info.origin.orientation.z = math.sin(self._map.origin[2] / 2.0)
        message.info.origin.orientation.w = math.cos(self._map.origin[2] / 2.0)
        message.data = self._map.data
        self._map_publisher.publish(message)

    def publish_transform(self) -> None:
        transform = self._transform_stamped_type()
        transform.header.stamp = self._node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'map_visualization'
        transform.transform.rotation.w = 1.0
        self._tf_publisher.publish(self._tf_message_type(transforms=[transform]))

    def destroy_node(self) -> None:
        self._node.destroy_node()


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description='Publish the Fleet Bridge static map for Foxglove.')
    parser.add_argument(
        '--map-yaml',
        default=os.environ.get('MAP_YAML', '/maps/map_0825.yaml'),
    )
    return parser.parse_known_args(argv)


def main(argv=None) -> None:
    import rclpy

    arguments, ros_arguments = _arguments(argv)
    rclpy.init(args=ros_arguments)
    publisher = CentralMapPublisher(arguments.map_yaml)
    try:
        rclpy.spin(publisher.node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
