"""Render an explicit, namespaced ROS 2 Domain Bridge configuration."""

import argparse
from pathlib import Path

import yaml


TELEMETRY_TOPICS = {
    'odom': 'nav_msgs/msg/Odometry',
    'tf': 'tf2_msgs/msg/TFMessage',
    'tf_static': 'tf2_msgs/msg/TFMessage',
    'scan': 'sensor_msgs/msg/LaserScan',
    'scan_raw': 'sensor_msgs/msg/LaserScan',
    'imu': 'sensor_msgs/msg/Imu',
    'imu/data_raw': 'sensor_msgs/msg/Imu',
    'depth/image_raw': 'sensor_msgs/msg/Image',
    'depth/camera_info': 'sensor_msgs/msg/CameraInfo',
    'controller/cmd_vel': 'geometry_msgs/msg/Twist',
    'ground_truth/pose': 'geometry_msgs/msg/PoseStamped',
}
COMMAND_TOPICS = {
    'manual/cmd_vel': 'geometry_msgs/msg/Twist',
    'cmd_vel_nav': 'geometry_msgs/msg/Twist',
    'safety/stop': 'std_msgs/msg/Empty',
}


def topic_path(namespace: str, suffix: str) -> str:
    """Join a ROS namespace and topic suffix into one absolute topic path."""
    parts = [part.strip('/') for part in (namespace, suffix) if part.strip('/')]
    return '/' + '/'.join(parts)


def render_bridge_config(
    vehicle_domain: int,
    control_domain: int,
    source_namespace: str,
    central_prefix: str,
) -> dict:
    """Return a two-way, allowlisted Domain Bridge document for one vehicle."""
    if not source_namespace.startswith('/') or not central_prefix.startswith('/'):
        raise ValueError('source_namespace and central_prefix must be absolute ROS namespaces')

    topics = {}
    for suffix, message_type in TELEMETRY_TOPICS.items():
        source_topic = topic_path(source_namespace, suffix)
        topics[source_topic] = {
            'type': message_type,
            'from_domain': vehicle_domain,
            'to_domain': control_domain,
            'remap': topic_path(central_prefix, suffix),
        }
    for suffix, message_type in COMMAND_TOPICS.items():
        central_topic = topic_path(central_prefix, suffix)
        topics[central_topic] = {
            'type': message_type,
            'from_domain': control_domain,
            'to_domain': vehicle_domain,
            'remap': topic_path(source_namespace, suffix),
        }
    return {'topics': topics}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--vehicle-domain', required=True, type=int)
    parser.add_argument('--control-domain', required=True, type=int)
    parser.add_argument('--source-namespace', required=True)
    parser.add_argument('--central-prefix', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    args.output.write_text(
        yaml.safe_dump(
            render_bridge_config(
                args.vehicle_domain,
                args.control_domain,
                args.source_namespace,
                args.central_prefix,
            ),
            sort_keys=True,
        ),
        encoding='utf-8',
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
