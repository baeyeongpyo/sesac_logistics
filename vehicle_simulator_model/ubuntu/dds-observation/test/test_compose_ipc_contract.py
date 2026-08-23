from pathlib import Path
import json
import subprocess
import unittest


COMPOSE_FILE = Path(__file__).resolve().parents[1] / 'docker-compose.yaml'


class ComposeIpcContractTest(unittest.TestCase):
    def test_map_server_and_bridge_share_host_ipc_for_fast_dds(self):
        compose = COMPOSE_FILE.read_text(encoding='utf-8')

        foxglove = compose.split('  foxglove-bridge:', 1)[1].split(
            '  map-server:', 1,
        )[0]
        map_server = compose.split('  map-server:', 1)[1].split(
            '  rosbag-recorder:', 1,
        )[0]

        self.assertIn('network_mode: host', foxglove)
        self.assertIn('ipc: host', foxglove)
        self.assertIn('network_mode: host', map_server)
        self.assertIn('ipc: host', map_server)

    def test_rosbag_recorder_uses_the_configured_host_directory(self):
        result = subprocess.run(
            ['docker', 'compose', '--env-file', '.env.example', 'config'],
            cwd=COMPOSE_FILE.parent,
            check=True,
            capture_output=True,
            encoding='utf-8',
        )
        recorder = result.stdout.split('  rosbag-recorder:', 1)[1].split(
            'networks:', 1,
        )[0]

        self.assertIn('type: bind', recorder)
        self.assertIn('source: /home/litcoder/logistics_database', recorder)
        self.assertIn('target: /rosbag', recorder)

    def test_static_bridge_services_use_one_common_image_with_distinct_domains(self):
        result = subprocess.run(
            ['docker', 'compose', '--env-file', '.env.example', 'config', '--format', 'json'],
            cwd=COMPOSE_FILE.parent,
            check=True,
            capture_output=True,
            encoding='utf-8',
        )
        services = json.loads(result.stdout)['services']

        self.assertNotIn('fleet-manager', services)
        for name, vehicle_domain, central_prefix in (
            ('bridge-robot-1', '215', '/robot_1'),
            ('bridge-robot-2', '216', '/robot_2'),
        ):
            with self.subTest(service=name):
                bridge = services[name]
                self.assertEqual(bridge['image'], 'mentorpi-domain-bridge:humble')
                self.assertEqual(bridge['network_mode'], 'host')
                self.assertEqual(bridge['ipc'], 'host')
                self.assertEqual(bridge['environment'], {
                    'CENTRAL_PREFIX': central_prefix,
                    'CONTROL_DOMAIN': '225',
                    'SOURCE_NAMESPACE': '/',
                    'VEHICLE_DOMAIN': vehicle_domain,
                })

        for name in ('foxglove-bridge', 'map-server', 'rosbag-recorder'):
            self.assertEqual(
                services[name]['depends_on'],
                {
                    'bridge-robot-1': {'condition': 'service_started', 'required': True},
                    'bridge-robot-2': {'condition': 'service_started', 'required': True},
                },
            )


if __name__ == '__main__':
    unittest.main()
