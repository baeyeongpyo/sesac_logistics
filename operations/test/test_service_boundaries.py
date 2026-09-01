"""Regression contracts for independent operations service ownership."""

from pathlib import Path
import unittest

import yaml


OPERATIONS = Path(__file__).resolve().parents[1]
FLEET_BRIDGE = OPERATIONS / 'fleet_bridge'
MONITORING = OPERATIONS / 'monitoring'
MAP_SERVER = MONITORING / 'map_server'
WAREHOUSE_SERVER = MONITORING / 'warehouse_server'


def compose(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


class ServiceBoundaryContractTest(unittest.TestCase):
    def test_integrated_compose_includes_fleet_bridge_and_monitoring_bundles(self):
        content = (OPERATIONS / 'compose.local.yaml').read_text(encoding='utf-8')

        for bundle in (
            './fleet_bridge/docker-compose.yaml',
            './monitoring/docker-compose.yaml',
        ):
            with self.subTest(bundle=bundle):
                self.assertIn(bundle, content)
        self.assertNotIn('./map_server/docker-compose.yaml', content)
        self.assertNotIn('./warehouse_server/docker-compose.yaml', content)
        self.assertNotIn('./foxglove_assert_server/docker-compose.yaml', content)

    def test_fleet_bridge_owns_only_vehicle_communication_services(self):
        services = compose(FLEET_BRIDGE / 'docker-compose.yaml')['services']

        self.assertEqual(set(services), {
            'worker-robot-1',
            'worker-robot-2',
            'command-api',
            'telemetry-writer',
        })

    def test_map_server_owns_the_central_map_publisher(self):
        service = compose(MONITORING / 'docker-compose.yaml')['services']['map-publisher']

        self.assertEqual(service['network_mode'], 'host')
        self.assertEqual(service['ipc'], 'host')
        self.assertEqual(service['environment']['ROS_DOMAIN_ID'], '${SERVER_ROS_DOMAIN_ID:-225}')
        self.assertEqual(
            service['command'][:4],
            ['ros2', 'run', 'central_map_server', 'central_map_publisher'],
        )

    def test_warehouse_server_owns_the_zone_overlay_publisher(self):
        service = compose(MONITORING / 'docker-compose.yaml')['services'][
            'warehouse-zone-publisher'
        ]

        self.assertEqual(service['network_mode'], 'host')
        self.assertEqual(service['ipc'], 'host')
        self.assertEqual(
            service['environment']['WAREHOUSE_ZONES_CONFIG'],
            '/config/warehouse_zones.yaml',
        )

    def test_monitoring_owns_foxglove_assets_map_and_warehouse(self):
        services = compose(MONITORING / 'docker-compose.yaml')['services']

        self.assertEqual(set(services), {
            'foxglove-bridge',
            'asset-server',
            'map-publisher',
            'warehouse-zone-publisher',
        })
        self.assertEqual(services['foxglove-bridge']['network_mode'], 'host')
        self.assertEqual(services['foxglove-bridge']['ipc'], 'host')
        self.assertEqual(services['asset-server']['network_mode'], 'host')
        self.assertEqual(services['asset-server']['command'][0], 'python3')

    def test_legacy_ownership_paths_are_absent(self):
        for path in (
            OPERATIONS / 'foxglove_assert_server',
            OPERATIONS / 'map_server',
            OPERATIONS / 'warehouse_server',
            FLEET_BRIDGE / 'maps',
            FLEET_BRIDGE / 'config' / 'server_foxglove.yaml',
            FLEET_BRIDGE / 'config' / 'warehouse_zones.yaml',
        ):
            with self.subTest(path=path):
                self.assertFalse(path.exists())


if __name__ == '__main__':
    unittest.main()
