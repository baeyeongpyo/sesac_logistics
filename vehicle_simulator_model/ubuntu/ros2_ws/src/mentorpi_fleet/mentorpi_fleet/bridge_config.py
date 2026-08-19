"""Render isolated Domain Bridge configurations for one fleet vehicle."""

from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml

from .registry import VehicleSpec


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


def bridge_document(vehicle: VehicleSpec, control_domain: int) -> dict:
    """Return the complete two-way bridge document for exactly one vehicle."""
    topics = {}
    for suffix, message_type in TELEMETRY_TOPICS.items():
        topics[f'{vehicle.namespace}/{suffix}'] = {
            'type': message_type,
            'from_domain': vehicle.domain_id,
            'to_domain': control_domain,
        }
    for suffix, message_type in COMMAND_TOPICS.items():
        topics[f'{vehicle.namespace}/{suffix}'] = {
            'type': message_type,
            'from_domain': control_domain,
            'to_domain': vehicle.domain_id,
        }
    return {'topics': topics}


def write_bridge_config(vehicle: VehicleSpec, control_domain: int, target: Path) -> Path:
    """Atomically write a bridge config so readers never see a partial YAML file."""
    target.parent.mkdir(parents=True, exist_ok=True)
    serialized = yaml.safe_dump(bridge_document(vehicle, control_domain), sort_keys=True)
    with NamedTemporaryFile('w', encoding='utf-8', dir=target.parent, delete=False) as temporary:
        temporary.write(serialized)
        temporary_path = Path(temporary.name)
    temporary_path.replace(target)
    return target
