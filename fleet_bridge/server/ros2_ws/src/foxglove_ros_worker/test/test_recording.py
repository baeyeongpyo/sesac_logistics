from datetime import datetime, timezone
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path.insert(0, str(COMMON))
sys.path.insert(0, str(PACKAGE))

from foxglove_ros_worker.recording import new_session_path, record_topics


def telemetry_topic(identifier, enabled, source, target, message_type):
    return {
        'id': identifier,
        'enabled': enabled,
        'source': source,
        'target': target,
        'type': message_type,
        'worker_rate': {},
        'qos': {
            'reliability': 'best_effort',
            'durability': 'volatile',
            'history': 'keep_last',
            'depth': 1,
        },
    }


class RecordingTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.now = datetime(2026, 8, 21, tzinfo=timezone.utc)
        self.telemetry_path = self.write_yaml('telemetry.yaml', {
            'version': 1,
            'topics': [
                telemetry_topic(
                    'odom',
                    True,
                    '/odom',
                    '/{robot}/odom',
                    'nav_msgs/msg/Odometry',
                ),
                telemetry_topic(
                    'depth_image_raw',
                    True,
                    '/depth/image_raw',
                    '/{robot}/depth/image_raw',
                    'sensor_msgs/msg/Image',
                ),
            ],
        })
        self.telemetry_with_disabled_path = self.write_yaml('disabled-telemetry.yaml', {
            'version': 1,
            'topics': [
                telemetry_topic(
                    'odom',
                    True,
                    '/odom',
                    '/{robot}/odom',
                    'nav_msgs/msg/Odometry',
                ),
                telemetry_topic(
                    'depth_image_raw',
                    False,
                    '/depth/image_raw',
                    '/{robot}/depth/image_raw',
                    'sensor_msgs/msg/Image',
                ),
            ],
        })
        self.central_path = self.write_yaml('central.yaml', {
            'version': 1,
            'topics': [
                {
                    'id': 'controller_map',
                    'enabled': True,
                    'source': '/controller_server/map',
                    'target': '/fleet/map',
                    'type': 'nav_msgs/msg/OccupancyGrid',
                    'replay_rate_hz': 1.0,
                    'qos': {
                        'reliability': 'reliable',
                        'durability': 'transient_local',
                        'history': 'keep_last',
                        'depth': 1,
                    },
                },
            ],
        })
        self.central_with_disabled_path = self.write_yaml('disabled-central.yaml', {
            'version': 1,
            'topics': [
                {
                    'id': 'controller_map',
                    'enabled': False,
                    'source': '/controller_server/map',
                    'target': '/fleet/map',
                    'type': 'nav_msgs/msg/OccupancyGrid',
                    'replay_rate_hz': 1.0,
                    'qos': {
                        'reliability': 'reliable',
                        'durability': 'transient_local',
                        'history': 'keep_last',
                        'depth': 1,
                    },
                },
            ],
        })

    def tearDown(self):
        self.temporary_directory.cleanup()

    def write_yaml(self, name, value):
        path = self.root / name
        path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
        return path

    def test_record_topics_expands_targets_status_and_map(self):
        self.assertEqual(
            record_topics(
                self.telemetry_path,
                self.central_path,
                ('robot_1', 'robot_2'),
            ),
            (
                '/robot_1/odom',
                '/robot_1/depth/image_raw',
                '/robot_1/fleet_bridge/status',
                '/robot_2/odom',
                '/robot_2/depth/image_raw',
                '/robot_2/fleet_bridge/status',
                '/fleet/map',
            ),
        )

    def test_record_topics_omits_disabled_entries(self):
        self.assertEqual(
            record_topics(
                self.telemetry_with_disabled_path,
                self.central_with_disabled_path,
                ('robot_1',),
            ),
            ('/robot_1/odom', '/robot_1/fleet_bridge/status'),
        )

    def test_new_session_path_preserves_existing_directory(self):
        (self.root / 'manual').mkdir()
        with self.assertRaisesRegex(ValueError, 'session_exists'):
            new_session_path(self.root, 'manual', self.now)

        (self.root / '20260821T000000Z').mkdir()
        self.assertEqual(
            new_session_path(self.root, '', self.now).name,
            '20260821T000000Z-01',
        )


if __name__ == '__main__':
    unittest.main()
