from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
BUNDLE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE))

from fleet_bridge_config.loader import ConfigError, load_fleet, load_telemetry


VALID_TELEMETRY = {
    'version': 1,
    'topics': [
        {
            'id': 'odom',
            'enabled': True,
            'source': '/odom',
            'uplink': '/{robot}/odom',
            'target': '/{robot}/odom',
            'type': 'nav_msgs/msg/Odometry',
            'filter': {'mode': 'passthrough'},
            'worker_rate': {},
            'qos': {
                'reliability': 'best_effort',
                'durability': 'volatile',
                'history': 'keep_last',
                'depth': 5,
            },
            'debug': True,
        },
        {
            'id': 'scan',
            'enabled': False,
            'source': '/scan_filtered',
            'uplink': '/{robot}/fleet_bridge/scan',
            'target': '/{robot}/scan',
            'type': 'sensor_msgs/msg/LaserScan',
            'filter': {'mode': 'rate', 'max_rate_hz': 2.0},
            'worker_rate': {'max_rate_hz': 2.0},
            'qos': {
                'reliability': 'best_effort',
                'durability': 'volatile',
                'history': 'keep_last',
                'depth': 1,
            },
            'debug': True,
        },
    ],
}


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

    def test_load_telemetry_prefixes_uplink_and_preserves_filter_and_qos(self):
        topics = load_telemetry(
            self.write_yaml('telemetry.yaml', VALID_TELEMETRY),
            'robot_1',
        )

        self.assertEqual(len(topics), 2)
        scan = topics[1]
        self.assertEqual(scan.id, 'scan')
        self.assertEqual(scan.source, '/scan_filtered')
        self.assertEqual(scan.uplink, '/robot_1/fleet_bridge/scan')
        self.assertEqual(scan.target, '/robot_1/scan')
        self.assertEqual(scan.filter.mode, 'rate')
        self.assertEqual(scan.filter.max_rate_hz, 2.0)
        self.assertEqual(scan.worker_rate.max_rate_hz, 2.0)
        self.assertEqual(scan.qos.reliability, 'best_effort')
        self.assertEqual(scan.qos.depth, 1)
        self.assertFalse(scan.enabled)
        self.assertTrue(scan.debug)

    def test_load_telemetry_allows_passthrough_relay_from_root_topic(self):
        topics = load_telemetry(
            self.write_yaml('telemetry.yaml', VALID_TELEMETRY),
            'robot_1',
        )

        odom = topics[0]
        self.assertEqual(odom.source, '/odom')
        self.assertEqual(odom.uplink, '/robot_1/odom')
        self.assertEqual(odom.target, '/robot_1/odom')
        self.assertEqual(odom.filter.mode, 'passthrough')

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
            {'ROBOT_1_FOXGLOVE_URI': 'ws://10.0.0.11:8766'},
        )

        self.assertEqual(loaded.server.domain_id, 225)
        self.assertEqual(loaded.server.command_api.host, '127.0.0.1')
        self.assertEqual(loaded.server.command_api.port, 8080)
        self.assertEqual(loaded.vehicles[0].foxglove_uri, 'ws://10.0.0.11:8766')
        self.assertEqual(loaded.vehicles[0].namespace, '/robot_1')
        self.assertEqual(loaded.vehicles[0].command.topic, '/cmd_vel')
        self.assertEqual(loaded.vehicles[0].command.max_hold_ms, 1000)

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

    def test_load_telemetry_rejects_duplicate_uplink(self):
        duplicated = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        duplicated['topics'][1]['enabled'] = True
        duplicated['topics'][1]['uplink'] = '/{robot}/odom'

        with self.assertRaisesRegex(ConfigError, 'duplicate uplink'):
            load_telemetry(self.write_yaml('telemetry.yaml', duplicated), 'robot_1')

    def test_load_telemetry_rejects_unknown_keys_and_invalid_values(self):
        cases = []

        unknown = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        unknown['topics'][0]['unexpected'] = True
        cases.append((unknown, 'unknown keys'))

        invalid_rate = yaml.safe_load(yaml.safe_dump(VALID_TELEMETRY))
        invalid_rate['topics'][1]['filter']['max_rate_hz'] = 0
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

    def test_repository_configs_define_two_vehicles_and_safe_default_telemetry(self):
        fleet = load_fleet(
            BUNDLE / 'config/fleet.yaml',
            {
                'ROBOT_1_FOXGLOVE_URI': 'ws://10.0.0.11:8766',
                'ROBOT_2_FOXGLOVE_URI': 'ws://10.0.0.12:8766',
            },
        )
        topics = load_telemetry(BUNDLE / 'config/telemetry.yaml', 'robot_1')

        self.assertEqual([vehicle.id for vehicle in fleet.vehicles], ['robot_1', 'robot_2'])
        self.assertEqual(
            [vehicle.namespace for vehicle in fleet.vehicles],
            ['/robot_1', '/robot_2'],
        )
        self.assertEqual(fleet.server.domain_id, 225)
        self.assertEqual(fleet.server.command_api.port, 8080)
        self.assertEqual(fleet.vehicles[0].command.topic, '/cmd_vel')
        battery = next(topic for topic in topics if topic.id == 'battery')
        scan = next(topic for topic in topics if topic.id == 'scan')
        self.assertEqual(battery.filter.mode, 'on_change')
        self.assertEqual(dict(battery.filter.thresholds), {
            'percentage': 0.01,
            'voltage': 0.1,
        })
        self.assertFalse(battery.enabled)
        self.assertFalse(scan.enabled)
        self.assertEqual(scan.filter.max_rate_hz, 2.0)
        odom = next(topic for topic in topics if topic.id == 'odom')
        self.assertEqual(odom.source, '/odom')
        self.assertEqual(odom.uplink, '/robot_1/odom')


if __name__ == '__main__':
    unittest.main()
