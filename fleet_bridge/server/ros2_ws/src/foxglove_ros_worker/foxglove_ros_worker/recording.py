import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import re
from typing import Sequence

from fleet_bridge_config import load_central_topics, load_telemetry


ROBOT_ID_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
SESSION_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]*$')


def parse_robot_ids(value: str) -> tuple[str, ...]:
    robot_ids = tuple(item.strip() for item in value.split(',') if item.strip())
    if not robot_ids:
        raise ValueError('robot_ids_empty')
    if len(robot_ids) != len(set(robot_ids)):
        raise ValueError('duplicate_robot_id')
    for robot_id in robot_ids:
        if not ROBOT_ID_PATTERN.fullmatch(robot_id):
            raise ValueError(f'invalid_robot_id: {robot_id}')
    return robot_ids


def record_topics(
    telemetry_path: Path | str,
    central_topics_path: Path | str,
    robot_ids: Sequence[str],
) -> tuple[str, ...]:
    topics = []
    for robot_id in robot_ids:
        topics.extend(
            topic.target
            for topic in load_telemetry(telemetry_path, robot_id)
            if topic.enabled
        )
        topics.append(f'/{robot_id}/fleet_bridge/status')
    topics.extend(
        topic.target
        for topic in load_central_topics(central_topics_path)
        if topic.enabled
    )
    return tuple(dict.fromkeys(topics))


def new_session_path(root: Path, requested_id: str, now: datetime) -> Path:
    if requested_id:
        if not SESSION_ID_PATTERN.fullmatch(requested_id):
            raise ValueError(f'invalid_session_id: {requested_id}')
        output_path = root / requested_id
        if output_path.exists():
            raise ValueError(f'session_exists: {output_path}')
        return output_path

    session_id = now.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output_path = root / session_id
    suffix = 1
    while output_path.exists():
        output_path = root / f'{session_id}-{suffix:02d}'
        suffix += 1
    return output_path


def parse_arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Record enabled fleet telemetry to a rosbag2 session.',
    )
    parser.add_argument(
        '--rosbag-root',
        default=os.environ.get('ROSBAG_ROOT', '/rosbag'),
    )
    parser.add_argument(
        '--rosbag-session-id',
        default=os.environ.get('ROSBAG_SESSION_ID', ''),
    )
    parser.add_argument(
        '--robot-ids',
        default=os.environ.get('ROBOT_IDS', 'robot_1,robot_2'),
    )
    parser.add_argument(
        '--telemetry-config',
        default=os.environ.get('TELEMETRY_CONFIG', '/config/telemetry.yaml'),
    )
    parser.add_argument(
        '--central-topics-config',
        default=os.environ.get(
            'CENTRAL_TOPICS_CONFIG',
            '/config/central_topics.yaml',
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    arguments = parse_arguments(argv)
    robot_ids = parse_robot_ids(arguments.robot_ids)
    topics = record_topics(
        arguments.telemetry_config,
        arguments.central_topics_config,
        robot_ids,
    )
    root = Path(arguments.rosbag_root)
    root.mkdir(parents=True, exist_ok=True)
    output_path = new_session_path(
        root,
        arguments.rosbag_session_id,
        datetime.now(timezone.utc),
    )
    command = ['ros2', 'bag', 'record', '--output', str(output_path), *topics]
    os.execvp('ros2', command)
