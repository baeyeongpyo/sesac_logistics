"""Publish the fixed warehouse-point overlay for Foxglove observation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any

import yaml


Color = tuple[float, float, float]
Point = tuple[float, float]
Scale = tuple[float, float, float]


@dataclass(frozen=True)
class WarehousePointGroup:
    id: str
    label: str
    color: Color


@dataclass(frozen=True)
class WarehousePoint:
    id: str
    label: str
    group: str
    center: Point


@dataclass(frozen=True)
class WarehouseMarkerStyle:
    point_center_z_m: float
    point_diameter_m: float
    point_height_m: float
    label_z_m: float
    label_size_m: float


@dataclass(frozen=True)
class WarehouseLayout:
    frame_id: str
    topic: str
    goal_center_offset_m: float
    marker_style: WarehouseMarkerStyle
    point_groups: tuple[WarehousePointGroup, ...]
    points: tuple[WarehousePoint, ...]

    def point_group(self, group_id: str) -> WarehousePointGroup:
        for group in self.point_groups:
            if group.id == group_id:
                return group
        raise KeyError(group_id)


@dataclass(frozen=True)
class MarkerSpec:
    id: str
    kind: str
    color: Color
    text: str | None = None
    position: Point | None = None
    z: float = 0.0
    scale: Scale = (0.0, 0.0, 0.0)


def _mapping(value: object, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f'{location} must be a mapping')
    return value


def _identifier(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f'{location} must be a non-empty string')
    return value


def _number(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{location} must be a number')
    return float(value)


def _positive_number(value: object, location: str) -> float:
    result = _number(value, location)
    if result <= 0:
        raise ValueError(f'{location} must be greater than zero')
    return result


def _color(value: object, location: str) -> Color:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{location} must contain three RGB values')
    result = tuple(_number(component, f'{location}[{index}]') for index, component in enumerate(value))
    if any(component < 0.0 or component > 1.0 for component in result):
        raise ValueError(f'{location} values must be between zero and one')
    return result  # type: ignore[return-value]


def _center(value: object, location: str) -> Point:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f'{location} must contain x and y')
    return (
        _number(value[0], f'{location}[0]'),
        _number(value[1], f'{location}[1]'),
    )


def _warehouse_point(raw: dict[str, Any], index: int) -> WarehousePoint:
    location = f'points[{index}]'
    point_id = _identifier(raw.get('id'), f'{location}.id')
    return WarehousePoint(
        id=point_id,
        label=_identifier(raw.get('label', point_id), f'{location}.label'),
        group=_identifier(raw.get('group'), f'{location}.group'),
        center=_center(raw.get('center'), f'{location}.center'),
    )


def load_warehouse_layout(path: Path | str) -> WarehouseLayout:
    """Load and validate the editable static zone layout."""

    with Path(path).open('r', encoding='utf-8') as stream:
        document = _mapping(yaml.safe_load(stream), 'warehouse layout')
    if document.get('version') != 2:
        raise ValueError('warehouse layout.version must be 2')
    frame_id = _identifier(document.get('frame_id'), 'warehouse layout.frame_id')
    topic = _identifier(document.get('topic'), 'warehouse layout.topic')
    if not topic.startswith('/'):
        raise ValueError('warehouse layout.topic must be an absolute ROS topic')
    goal_center_offset_m = _number(
        document.get('goal_center_offset_m'),
        'warehouse layout.goal_center_offset_m',
    )
    if goal_center_offset_m <= 0:
        raise ValueError('warehouse layout.goal_center_offset_m must be greater than zero')

    raw_marker_style = _mapping(document.get('marker_style'), 'warehouse layout.marker_style')
    marker_style = WarehouseMarkerStyle(
        point_center_z_m=_number(
            raw_marker_style.get('point_center_z_m'),
            'warehouse layout.marker_style.point_center_z_m',
        ),
        point_diameter_m=_positive_number(
            raw_marker_style.get('point_diameter_m'),
            'warehouse layout.marker_style.point_diameter_m',
        ),
        point_height_m=_positive_number(
            raw_marker_style.get('point_height_m'),
            'warehouse layout.marker_style.point_height_m',
        ),
        label_z_m=_number(
            raw_marker_style.get('label_z_m'),
            'warehouse layout.marker_style.label_z_m',
        ),
        label_size_m=_positive_number(
            raw_marker_style.get('label_size_m'),
            'warehouse layout.marker_style.label_size_m',
        ),
    )

    raw_groups = document.get('point_groups')
    if not isinstance(raw_groups, list) or not raw_groups:
        raise ValueError('warehouse layout.point_groups must not be empty')
    point_groups = tuple(
        WarehousePointGroup(
            id=_identifier(raw.get('id'), f'point_groups[{index}].id'),
            label=_identifier(raw.get('label'), f'point_groups[{index}].label'),
            color=_color(raw.get('color'), f'point_groups[{index}].color'),
        )
        for index, value in enumerate(raw_groups)
        for raw in (_mapping(value, f'point_groups[{index}]'),)
    )
    if len({group.id for group in point_groups}) != len(point_groups):
        raise ValueError('warehouse layout.point_groups contains duplicate ids')

    raw_points = document.get('points')
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError('warehouse layout.points must not be empty')
    points = tuple(
        _warehouse_point(_mapping(value, f'points[{index}]'), index)
        for index, value in enumerate(raw_points)
    )
    if len({point.id for point in points}) != len(points):
        raise ValueError('warehouse layout.points contains duplicate ids')
    group_ids = {group.id for group in point_groups}
    unknown_groups = {point.group for point in points} - group_ids
    if unknown_groups:
        raise ValueError(f'warehouse layout.points contains unknown groups: {sorted(unknown_groups)}')
    return WarehouseLayout(
        frame_id=frame_id,
        topic=topic,
        goal_center_offset_m=goal_center_offset_m,
        marker_style=marker_style,
        point_groups=point_groups,
        points=points,
    )


def build_marker_specs(layout: WarehouseLayout) -> tuple[MarkerSpec, ...]:
    """Create a ROS-independent representation that is straightforward to test."""

    specs = []
    for point in layout.points:
        group = layout.point_group(point.group)
        style = layout.marker_style
        specs.extend((
            MarkerSpec(
                point.id,
                'warehouse_point',
                group.color,
                position=point.center,
                z=style.point_center_z_m,
                scale=(style.point_diameter_m, style.point_diameter_m, style.point_height_m),
            ),
            MarkerSpec(
                point.id,
                'warehouse_point_label',
                group.color,
                text=point.label,
                position=point.center,
                z=style.label_z_m,
                scale=(0.0, 0.0, style.label_size_m),
            ),
        ))
    return tuple(specs)


def build_ros_markers(layout: WarehouseLayout, stamp):
    """Translate the tested marker specs into the ROS 2 messages Foxglove renders."""

    from visualization_msgs.msg import Marker, MarkerArray

    markers = MarkerArray()
    for marker_id, spec in enumerate(build_marker_specs(layout)):
        marker = Marker()
        marker.header.frame_id = layout.frame_id
        marker.header.stamp = stamp
        marker.ns = spec.kind
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.color.r, marker.color.g, marker.color.b = spec.color
        if spec.kind == 'warehouse_point':
            x, y = spec.position
            marker.type = Marker.CYLINDER
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = spec.z
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = spec.scale
            marker.color.a = 0.9
        elif spec.kind == 'warehouse_point_label':
            x, y = spec.position
            marker.type = Marker.TEXT_VIEW_FACING
            marker.text = spec.text
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = spec.z
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = spec.scale
            marker.color.a = 1.0
        else:
            raise ValueError(f'unknown warehouse marker kind: {spec.kind}')
        markers.markers.append(marker)
    return markers


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description='Publish the warehouse zoning overlay for Foxglove.')
    parser.add_argument(
        '--warehouse-zones-config',
        default=os.environ.get('WAREHOUSE_ZONES_CONFIG', '/config/warehouse_zones.yaml'),
    )
    return parser.parse_args(argv)


def main(argv=None) -> None:
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from visualization_msgs.msg import MarkerArray

    layout = load_warehouse_layout(_arguments(argv).warehouse_zones_config)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('warehouse_zone_publisher')
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(MarkerArray, layout.topic, qos)
    publisher.publish(build_ros_markers(layout, node.get_clock().now().to_msg()))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
