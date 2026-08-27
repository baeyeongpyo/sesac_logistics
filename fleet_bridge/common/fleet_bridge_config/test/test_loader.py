from pathlib import Path
import re
import sys
from tempfile import TemporaryDirectory
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
BUNDLE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE))

from fleet_bridge_config.loader import (
    ConfigError,
    load_central_topics,
    load_fleet,
    load_telemetry,
)


VALID_TELEMETRY = {
    'version': 1,
    'topics': [
        {
            'id': 'odom',
            'enabled': True,
            'source': '/odom',
            'target': '/{robot}/odom',
            'type': 'nav_msgs/msg/Odometry',
            'worker_rate': {},
            'qos': {
                'reliability': 'best_effort',
                'durability': 'volatile',
                'history': 'keep_last',
                'depth': 5,
            },
        },
        {
            'id': 'scan',
            'enabled': False,
            'source': '/scan_filtered',
            'target': '/{robot}/scan',
            'type': 'sensor_msgs/msg/LaserScan',
            'worker_rate': {'max_rate_hz': 2.0},
            'qos': {
                'reliability': 'best_effort',
                'durability': 'volatile',
                'history': 'keep_last',
                'depth': 1,
            },
        },
    ],
}

VEHICLE_TOPIC_PATTERN = re.compile(
    r'^(?P<topic>/\S+) \[(?P<message_type>[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*)\]$',
)
EXPECTED_VEHICLE_TOPIC_COUNT = 76


def load_vehicle_topic_snapshot():
    snapshot_path = BUNDLE / 'config/tmp/vehicle_node_topic'
    entries = []
    for line_number, raw_line in enumerate(
        snapshot_path.read_text(encoding='utf-8').splitlines(),
        start=1,
    ):
        match = VEHICLE_TOPIC_PATTERN.fullmatch(raw_line)
        if match is None:
            raise AssertionError(
                f'invalid vehicle topic snapshot line {line_number}: {raw_line!r}',
            )
        entries.append((match['topic'], match['message_type']))
    return entries


class ConfigLoaderTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def write_yaml(self, name, value):
        path = self.root / name
        path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
        return path

    def test_load_telemetry_maps_source_to_namespaced_target_and_preserves_qos(self):
        topics = load_telemetry(
            self.write_yaml('telemetry.yaml', VALID_TELEMETRY),
            'robot_1',
        )

        self.assertEqual(len(topics), 2)
        scan = topics[1]
        self.assertEqual(scan.id, 'scan')
        self.assertEqual(scan.source, '/scan_filtered')
        self.assertEqual(scan.target, '/robot_1/scan')
        self.assertEqual(scan.worker_rate.max_rate_hz, 2.0)
        self.assertEqual(scan.qos.reliability, 'best_effort')
        self.assertEqual(scan.qos.depth, 1)
        self.assertFalse(scan.enabled)

    def test_load_telemetry_keeps_root_source_for_remote_bridge_matching(self):
        topics = load_telemetry(
            self.write_yaml('telemetry.yaml', VALID_TELEMETRY),
            'robot_1',
        )

        odom = topics[0]
        self.assertEqual(odom.source, '/odom')
        self.assertEqual(odom.target, '/robot_1/odom')

    def test_load_telemetry_has_no_vehicle_relay_fields(self):
        topics = load_telemetry(
            self.write_yaml('telemetry.yaml', VALID_TELEMETRY),
            'robot_1',
        )

        self.assertFalse(hasattr(topics[0], 'uplink'))
        self.assertFalse(hasattr(topics[0], 'filter'))
        self.assertFalse(hasattr(topics[0], 'debug'))

    def test_load_telemetry_rejects_removed_vehicle_relay_keys(self):
        legacy = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        legacy['topics'][0]['uplink'] = '/{robot}/odom'
        with self.assertRaisesRegex(ConfigError, 'unknown keys'):
            load_telemetry(
                self.write_yaml('telemetry.yaml', legacy),
                'robot_1',
            )

    def test_load_fleet_expands_only_declared_environment_values(self):
        fleet = {
            'server': {
                'domain_id': 225,
                'foxglove_port': 8765,
                'command_api': {'host': '127.0.0.1', 'port': 8080},
            },
            'vehicles': [
                {
                    'id': 'robot_1',
                    'foxglove_uri': '${ROBOT_1_FOXGLOVE_URI}',
                    'command_api_url': '${ROBOT_1_COMMAND_API_URL}',
                    'enabled': True,
                    'command': {
                        'topic': '/cmd_vel',
                        'type': 'geometry_msgs/msg/Twist',
                        'max_linear_x': 0.3,
                        'max_angular_z': 1.0,
                        'max_hold_ms': 1000,
                        'publish_rate_hz': 10,
                    },
                },
            ],
        }

        loaded = load_fleet(
            self.write_yaml('fleet.yaml', fleet),
            {
                'ROBOT_1_FOXGLOVE_URI': 'ws://10.0.0.11:8766',
                'ROBOT_1_COMMAND_API_URL': 'http://10.0.0.11:8082',
            },
        )

        self.assertEqual(loaded.server.domain_id, 225)
        self.assertEqual(loaded.server.command_api.host, '127.0.0.1')
        self.assertEqual(loaded.server.command_api.port, 8080)
        self.assertEqual(loaded.vehicles[0].foxglove_uri, 'ws://10.0.0.11:8766')
        self.assertEqual(loaded.vehicles[0].command_api_url, 'http://10.0.0.11:8082')
        self.assertEqual(loaded.vehicles[0].namespace, '/robot_1')
        self.assertEqual(loaded.vehicles[0].command.topic, '/cmd_vel')
        self.assertEqual(loaded.vehicles[0].command.max_hold_ms, 1000)

    def test_load_fleet_expands_vehicle_command_api_url(self):
        fleet = {
            'server': {
                'domain_id': 225,
                'foxglove_port': 8765,
                'command_api': {'host': '127.0.0.1', 'port': 8080},
            },
            'vehicles': [{
                'id': 'robot_1',
                'foxglove_uri': '${ROBOT_1_FOXGLOVE_URI}',
                'command_api_url': '${ROBOT_1_COMMAND_API_URL}',
                'enabled': True,
                'command': {
                    'topic': '/cmd_vel',
                    'type': 'geometry_msgs/msg/Twist',
                    'max_linear_x': 0.3,
                    'max_angular_z': 1.0,
                    'max_hold_ms': 1000,
                    'publish_rate_hz': 10,
                },
            }],
        }

        loaded = load_fleet(
            self.write_yaml('fleet-command-api.yaml', fleet),
            {
                'ROBOT_1_FOXGLOVE_URI': 'ws://10.0.0.11:8766',
                'ROBOT_1_COMMAND_API_URL': 'http://10.0.0.11:8082/',
            },
        )

        self.assertEqual(
            loaded.vehicles[0].command_api_url,
            'http://10.0.0.11:8082',
        )

    def test_load_fleet_rejects_missing_environment_value(self):
        fleet = {
            'server': {
                'domain_id': 225,
                'foxglove_port': 8765,
                'command_api': {'host': '127.0.0.1', 'port': 8080},
            },
            'vehicles': [
                {
                    'id': 'robot_1',
                    'foxglove_uri': '${ROBOT_1_FOXGLOVE_URI}',
                    'command_api_url': '${ROBOT_1_COMMAND_API_URL}',
                    'enabled': True,
                    'command': {
                        'topic': '/cmd_vel',
                        'type': 'geometry_msgs/msg/Twist',
                        'max_linear_x': 0.3,
                        'max_angular_z': 1.0,
                        'max_hold_ms': 1000,
                        'publish_rate_hz': 10,
                    },
                },
            ],
        }

        with self.assertRaisesRegex(ConfigError, 'ROBOT_1_FOXGLOVE_URI'):
            load_fleet(self.write_yaml('fleet.yaml', fleet), {})

    def test_load_fleet_rejects_unsafe_command_values(self):
        fleet = {
            'server': {
                'domain_id': 225,
                'foxglove_port': 8765,
                'command_api': {'host': '127.0.0.1', 'port': 8080},
            },
            'vehicles': [{
                'id': 'robot_1',
                'foxglove_uri': 'ws://10.0.0.11:8766',
                'command_api_url': 'http://10.0.0.11:8082',
                'enabled': True,
                'command': {
                    'topic': '/cmd_vel',
                    'type': 'geometry_msgs/msg/Twist',
                    'max_linear_x': 0.3,
                    'max_angular_z': 1.0,
                    'max_hold_ms': 1000,
                    'publish_rate_hz': 10,
                },
            }],
        }
        cases = [
            ('topic', 'cmd_vel'),
            ('type', 'std_msgs/msg/String'),
            ('max_linear_x', 0),
            ('max_angular_z', 0),
            ('max_hold_ms', 0),
            ('publish_rate_hz', 0),
        ]

        for field, value in cases:
            with self.subTest(field=field):
                invalid = yaml.safe_load(yaml.safe_dump(fleet))
                invalid['vehicles'][0]['command'][field] = value
                with self.assertRaisesRegex(ConfigError, field):
                    load_fleet(self.write_yaml(f'{field}.yaml', invalid), {})

    def test_load_fleet_rejects_non_http_vehicle_command_api_url(self):
        fleet = {
            'server': {
                'domain_id': 225,
                'foxglove_port': 8765,
                'command_api': {'host': '127.0.0.1', 'port': 8080},
            },
            'vehicles': [{
                'id': 'robot_1',
                'foxglove_uri': 'ws://10.0.0.11:8766',
                'command_api_url': 'http://10.0.0.11:8082',
                'enabled': True,
                'command': {
                    'topic': '/cmd_vel',
                    'type': 'geometry_msgs/msg/Twist',
                    'max_linear_x': 0.3,
                    'max_angular_z': 1.0,
                    'max_hold_ms': 1000,
                    'publish_rate_hz': 10,
                },
            }],
        }
        cases = (
            'ws://10.0.0.11:8766',
            'http://10.0.0.11:8082/v1',
        )

        for value in cases:
            with self.subTest(value=value):
                invalid = yaml.safe_load(yaml.safe_dump(fleet))
                invalid['vehicles'][0]['command_api_url'] = value
                with self.assertRaisesRegex(ConfigError, 'command_api_url'):
                    load_fleet(self.write_yaml('invalid-command-api-url.yaml', invalid), {})

    def test_load_telemetry_rejects_duplicate_target(self):
        duplicated = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        duplicated['topics'][1]['enabled'] = True
        duplicated['topics'][1]['target'] = '/{robot}/odom'

        with self.assertRaisesRegex(ConfigError, 'duplicate target'):
            load_telemetry(self.write_yaml('telemetry.yaml', duplicated), 'robot_1')

    def test_load_telemetry_rejects_duplicate_source(self):
        duplicated = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        duplicated['topics'][1]['enabled'] = True
        duplicated['topics'][1]['source'] = '/odom'

        with self.assertRaisesRegex(ConfigError, 'duplicate source'):
            load_telemetry(self.write_yaml('telemetry.yaml', duplicated), 'robot_1')

    def test_load_telemetry_rejects_unknown_keys_and_invalid_values(self):
        cases = []

        unknown = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        unknown['topics'][0]['unexpected'] = True
        cases.append((unknown, 'unknown keys'))

        invalid_rate = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        invalid_rate['topics'][1]['worker_rate']['max_rate_hz'] = 0
        cases.append((invalid_rate, 'max_rate_hz'))

        invalid_qos = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        invalid_qos['topics'][0]['qos']['reliability'] = 'sometimes'
        cases.append((invalid_qos, 'reliability'))

        invalid_type = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        invalid_type['topics'][0]['type'] = 'Odometry'
        cases.append((invalid_type, 'message type'))

        for index, (document, message) in enumerate(cases):
            with self.subTest(message=message):
                with self.assertRaisesRegex(ConfigError, message):
                    load_telemetry(
                        self.write_yaml(f'invalid-{index}.yaml', document),
                        'robot_1',
                    )

    def test_disabled_topics_may_share_no_active_routing_identity(self):
        disabled = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        disabled['topics'].append({
            **disabled['topics'][1],
            'id': 'scan_alternative',
        })

        loaded = load_telemetry(
            self.write_yaml('disabled.yaml', disabled),
            'robot_1',
        )

        self.assertEqual(len(loaded), 3)

    def test_load_central_topics_validates_enabled_entries(self):
        document = {
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
                {
                    'id': 'disabled_copy',
                    'enabled': False,
                    'source': '/controller_server/map',
                    'target': '/fleet/map_copy',
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
        }

        topics = load_central_topics(self.write_yaml('central.yaml', document))

        self.assertEqual(
            [
                (
                    topic.id,
                    topic.enabled,
                    topic.source,
                    topic.target,
                    topic.replay_rate_hz,
                )
                for topic in topics
            ],
            [
                (
                    'controller_map',
                    True,
                    '/controller_server/map',
                    '/fleet/map',
                    1.0,
                ),
                (
                    'disabled_copy',
                    False,
                    '/controller_server/map',
                    '/fleet/map_copy',
                    1.0,
                ),
            ],
        )

    def test_load_central_topics_rejects_duplicate_active_path(self):
        document = {
            'version': 1,
            'topics': [
                {
                    'id': 'first',
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
                {
                    'id': 'second',
                    'enabled': True,
                    'source': '/controller_server/map',
                    'target': '/fleet/map_copy',
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
        }

        with self.assertRaisesRegex(ConfigError, 'duplicate central topic'):
            load_central_topics(self.write_yaml('duplicate.yaml', document))

    def test_load_telemetry_preserves_camera_pair_and_replay_rate(self):
        document = {
            'version': 1,
            'topics': [
                {
                    'id': 'rgb_image_raw',
                    'enabled': True,
                    'source': '/ascamera/camera_publisher/rgb0/image',
                    'target': '/{robot}/rgb/image_raw',
                    'type': 'sensor_msgs/msg/Image',
                    'paired_with': 'rgb_camera_info',
                    'worker_rate': {},
                    'qos': {
                        'reliability': 'best_effort',
                        'durability': 'volatile',
                        'history': 'keep_last',
                        'depth': 1,
                    },
                },
                {
                    'id': 'rgb_camera_info',
                    'enabled': True,
                    'source': '/ascamera/camera_publisher/rgb0/camera_info',
                    'target': '/{robot}/rgb/camera_info',
                    'type': 'sensor_msgs/msg/CameraInfo',
                    'paired_with': 'rgb_image_raw',
                    'replay_rate_hz': 1.0,
                    'worker_rate': {},
                    'qos': {
                        'reliability': 'reliable',
                        'durability': 'transient_local',
                        'history': 'keep_last',
                        'depth': 1,
                    },
                },
            ],
        }

        image, camera_info = load_telemetry(
            self.write_yaml('camera-pair.yaml', document),
            'robot_1',
        )

        self.assertEqual(image.paired_with, 'rgb_camera_info')
        self.assertEqual(camera_info.paired_with, 'rgb_image_raw')
        self.assertEqual(camera_info.replay_rate_hz, 1.0)
        self.assertEqual(camera_info.target, '/robot_1/rgb/camera_info')

    def test_repository_configs_define_two_vehicles_and_safe_default_telemetry(self):
        fleet = load_fleet(
            BUNDLE / 'config/fleet.yaml',
            {
                'ROBOT_1_FOXGLOVE_URI': 'ws://10.0.0.11:8766',
                'ROBOT_2_FOXGLOVE_URI': 'ws://10.0.0.12:8766',
                'ROBOT_1_COMMAND_API_URL': 'http://10.0.0.11:8082',
                'ROBOT_2_COMMAND_API_URL': 'http://10.0.0.12:8082',
            },
        )
        topics = load_telemetry(BUNDLE / 'config/telemetry.yaml', 'robot_1')
        central_topics = load_central_topics(BUNDLE / 'config/central_topics.yaml')

        self.assertEqual([vehicle.id for vehicle in fleet.vehicles], ['robot_1', 'robot_2'])
        self.assertEqual(
            [vehicle.namespace for vehicle in fleet.vehicles],
            ['/robot_1', '/robot_2'],
        )
        self.assertEqual(fleet.server.domain_id, 225)
        self.assertEqual(fleet.server.command_api.port, 8080)
        self.assertEqual(fleet.vehicles[0].command.topic, '/cmd_vel')
        self.assertEqual(fleet.vehicles[0].command_api_url, 'http://10.0.0.11:8082')
        self.assertTrue(all(topic.enabled for topic in topics))
        battery = next(
            topic for topic in topics
            if topic.id == 'ros_robot_controller_battery'
        )
        scan_filtered = next(topic for topic in topics if topic.id == 'scan_filtered')
        self.assertEqual(battery.worker_rate.max_rate_hz, 0.2)
        self.assertEqual(battery.message_type, 'std_msgs/msg/UInt16')
        self.assertEqual(scan_filtered.worker_rate.max_rate_hz, 2.0)
        rgb_image = next(topic for topic in topics if topic.id == 'ascamera_rgb0_image')
        rgb_camera_info = next(
            topic for topic in topics if topic.id == 'ascamera_rgb0_camera_info'
        )
        depth_image = next(
            topic for topic in topics if topic.id == 'ascamera_depth0_image_raw'
        )
        depth_camera_info = next(
            topic for topic in topics if topic.id == 'ascamera_depth0_camera_info'
        )
        self.assertEqual(rgb_image.paired_with, 'ascamera_rgb0_camera_info')
        self.assertEqual(rgb_camera_info.paired_with, 'ascamera_rgb0_image')
        self.assertEqual(depth_image.paired_with, 'ascamera_depth0_camera_info')
        self.assertEqual(depth_camera_info.paired_with, 'ascamera_depth0_image_raw')
        self.assertEqual(rgb_image.worker_rate.max_rate_hz, 5.0)
        self.assertEqual(depth_image.worker_rate.max_rate_hz, 5.0)
        self.assertEqual(rgb_camera_info.replay_rate_hz, 1.0)
        self.assertEqual(depth_camera_info.replay_rate_hz, 1.0)
        self.assertEqual(rgb_camera_info.qos.durability, 'transient_local')
        self.assertEqual(depth_camera_info.qos.durability, 'transient_local')
        self.assertEqual(
            [
                (
                    topic.source,
                    topic.target,
                    topic.message_type,
                    topic.replay_rate_hz,
                )
                for topic in central_topics
                if topic.enabled
            ],
            [
                (
                    '/controller_server/map',
                    '/fleet/map',
                    'nav_msgs/msg/OccupancyGrid',
                    1.0,
                ),
            ],
        )
        vehicle_map = next(topic for topic in topics if topic.id == 'map')
        self.assertEqual(vehicle_map.source, '/map')
        self.assertEqual(vehicle_map.target, '/robot_1/map')
        odom = next(topic for topic in topics if topic.id == 'odom')
        self.assertEqual(odom.source, '/odom')
        self.assertEqual(odom.target, '/robot_1/odom')

    def test_repository_telemetry_maps_every_vehicle_topic_with_exact_type(self):
        snapshot = load_vehicle_topic_snapshot()
        topics = load_telemetry(BUNDLE / 'config/telemetry.yaml', 'robot_1')

        self.assertEqual(len(snapshot), EXPECTED_VEHICLE_TOPIC_COUNT)
        self.assertEqual(len(snapshot), len(set(snapshot)))
        self.assertEqual(
            {
                (
                    topic.source,
                    topic.message_type,
                    topic.target,
                    topic.enabled,
                )
                for topic in topics
            },
            {
                (source, message_type, f'/robot_1{source}', True)
                for source, message_type in snapshot
            },
        )


if __name__ == '__main__':
    unittest.main()
