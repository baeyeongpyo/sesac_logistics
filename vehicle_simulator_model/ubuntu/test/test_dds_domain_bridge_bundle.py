import json
import os
import subprocess
import time
import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1] / 'dds-domain-bridge'


class DdsDomainBridgeBundleTest(unittest.TestCase):
    def test_standalone_compose_exposes_only_discovery_and_telemetry_bridge(self):
        environment = os.environ.copy()
        environment['DDS_DOMAIN_BRIDGE_IMAGE'] = 'mentorpi-dds-domain-bridge:test'
        result = subprocess.run(
            [
                'docker', 'compose',
                '--env-file', str(BUNDLE / '.env.example'),
                '-f', str(BUNDLE / 'docker-compose.yaml'),
                'config', '--format', 'json',
            ],
            text=True,
            capture_output=True,
            env=environment,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        services = json.loads(result.stdout)['services']
        self.assertEqual(set(services), {'dds-discovery', 'dds-domain-bridge'})

        discovery = services['dds-discovery']
        self.assertEqual(discovery['network_mode'], 'host')
        self.assertEqual(
            discovery['command'],
            ['fastdds', 'discovery', '-i', '0', '-l', '0.0.0.0', '-p', '11811'],
        )

        bridge = services['dds-domain-bridge']
        self.assertEqual(bridge['network_mode'], 'host')
        self.assertEqual(bridge['environment']['ROS_DOMAIN_ID'], '215')
        self.assertEqual(bridge['environment']['DDS_DISCOVERY_HOST'], '127.0.0.1')
        self.assertEqual(bridge['environment']['FASTDDS_BUILTIN_TRANSPORTS'], 'UDPv4')
        bridge_template = next(
            volume for volume in bridge['volumes']
            if volume['target'] == '/etc/dds-domain-bridge/bridge.yaml.template'
        )
        self.assertTrue(bridge_template['read_only'])
        self.assertEqual(
            bridge['command'],
            ['ros2', 'run', 'domain_bridge', 'domain_bridge', '/tmp/dds-domain-bridge.yaml'],
        )

    def test_image_entrypoint_renders_central_bridge_configuration(self):
        result = subprocess.run(
            [
                'docker', 'compose',
                '--env-file', str(BUNDLE / '.env.example'),
                '-f', str(BUNDLE / 'docker-compose.yaml'),
                'run', '--rm', '--no-deps', 'dds-domain-bridge',
                'bash', '-lc',
                'ros2 pkg prefix domain_bridge '
                '&& grep -Fx "    from_domain: 1" /tmp/dds-domain-bridge.yaml '
                '&& grep -Fx "    to_domain: 215" /tmp/dds-domain-bridge.yaml',
            ],
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)

    def test_observer_removes_its_temporary_super_client_profile(self):
        compose_command = [
            'docker', 'compose',
            '--env-file', str(BUNDLE / '.env.example'),
            '-f', str(BUNDLE / 'docker-compose.yaml'),
        ]
        up = subprocess.run(
            [*compose_command, 'up', '-d'],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(up.returncode, 0, up.stderr)
        try:
            observed = subprocess.run(
                [*compose_command, 'exec', '-T', 'dds-domain-bridge', 'dds-observe', 'topics'],
                text=True,
                capture_output=True,
                check=False,
            )
            profiles = subprocess.run(
                [
                    *compose_command, 'exec', '-T', 'dds-domain-bridge', 'bash', '-lc',
                    'find /tmp -maxdepth 1 -name "dds-domain-bridge-super-client-*.xml" -print',
                ],
                text=True,
                capture_output=True,
                check=False,
            )
        finally:
            down = subprocess.run(
                [*compose_command, 'down'],
                text=True,
                capture_output=True,
                check=False,
            )

        self.assertEqual(observed.returncode, 0, observed.stderr)
        self.assertEqual(profiles.returncode, 0, profiles.stderr)
        self.assertEqual(profiles.stdout, '')

    def test_domain_scoped_observer_does_not_receive_other_domain_payload(self):
        compose_command = [
            'docker', 'compose',
            '--env-file', str(BUNDLE / '.env.example'),
            '-f', str(BUNDLE / 'docker-compose.yaml'),
        ]
        up = subprocess.run(
            [*compose_command, 'up', '-d'],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(up.returncode, 0, up.stderr)
        publisher = None
        try:
            publisher = subprocess.Popen(
                [
                    *compose_command, 'exec', '-T', 'dds-domain-bridge', 'bash', '-lc',
                    'source /opt/ros/humble/setup.bash '
                    '&& source /usr/local/bin/dds-domain-bridge-env '
                    '&& export ROS_DOMAIN_ID=215 '
                    '&& timeout 8 ros2 topic pub -r 2 /central_domain_probe '
                    'std_msgs/msg/String "{data: central}"',
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            time.sleep(2)
            observed = subprocess.run(
                [
                    *compose_command, 'exec', '-T',
                    '-e', 'DDS_OBSERVE_DOMAIN_ID=1',
                    'dds-domain-bridge', 'bash', '-lc',
                    'timeout 3 dds-observe echo /central_domain_probe',
                ],
                text=True,
                capture_output=True,
                check=False,
            )
        finally:
            if publisher is not None:
                publisher.communicate(timeout=15)
            down = subprocess.run(
                [*compose_command, 'down'],
                text=True,
                capture_output=True,
                check=False,
            )

        self.assertEqual(down.returncode, 0, down.stderr)
        self.assertNotIn('data: central', observed.stdout)
        self.assertIn('Could not determine the type', observed.stdout + observed.stderr)


if __name__ == '__main__':
    unittest.main()
