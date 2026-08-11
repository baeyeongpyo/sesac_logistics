import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]


class ObservationBundleTest(unittest.TestCase):
    def test_foxglove_bridge_has_no_browser_proxy_dependencies(self):
        bridge_path = BUNDLE / 'compose.foxglove.yaml'
        self.assertTrue(bridge_path.is_file())

        bridge = bridge_path.read_text()
        lan = (BUNDLE / 'compose.lan.yaml').read_text()

        for required in (
            'foxglove-bridge:',
            '127.0.0.1:${FOXGLOVE_PORT-8765}:8765',
            'DDS_SUPER_CLIENT: "1"',
            'fleet-manager:',
            'fleet-scene:',
            'condition: service_started',
        ):
            self.assertIn(required, bridge)
        for required in (
            'foxglove-bridge:',
            'network_mode: host',
            'DDS_DISCOVERY_HOST: 127.0.0.1',
            'ports: !reset []',
        ):
            self.assertIn(required, lan)

    def test_lan_profile_uses_host_network_without_changing_base_compose(self):
        base = (BUNDLE / 'compose.yaml').read_text()
        lan = (BUNDLE / 'compose.lan.yaml').read_text()

        self.assertIn('internal: true', base)
        self.assertNotIn('network_mode: host', base)
        for service in (
            'dds-discovery:',
            'gazebo-server:',
            'sim-adapter:',
            'foxglove-bridge:',
            'slam-mapper:',
            'slam-inspector:',
        ):
            self.assertIn(service, lan)
        self.assertIn('networks: !reset []', lan)
        self.assertIn('GZ_IP: "${GZ_SERVER_IP:?', lan)

    def test_lan_compose_uses_host_bridge_without_published_ports(self):
        environment = os.environ.copy()
        environment.update({
            'GZ_SERVER_IP': '192.168.50.10',
            'FOXGLOVE_PORT': '8765',
        })
        result = subprocess.run(
            [
                'docker', 'compose',
                '-f', str(BUNDLE / 'compose.yaml'),
                '-f', str(BUNDLE / 'compose.foxglove.yaml'),
                '-f', str(BUNDLE / 'compose.lan.yaml'),
                'config', '--format', 'json',
            ],
            text=True,
            capture_output=True,
            env=environment,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        bridge = json.loads(result.stdout)['services']['foxglove-bridge']
        self.assertEqual(bridge['network_mode'], 'host')
        self.assertNotIn('ports', bridge)

    def test_lan_mode_requires_server_ip_before_calling_docker(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            shutil.copy2(BUNDLE / 'run.sh', temp_path / 'run.sh')
            (temp_path / '.env.test').write_text('SIM_NETWORK_MODE=lan\n')
            marker = temp_path / 'docker-called'
            fake_docker = temp_path / 'docker'
            fake_docker.write_text(
                '#!/usr/bin/env bash\n'
                'printf called > "$DOCKER_MARKER"\n'
            )
            fake_docker.chmod(0o755)
            environment = os.environ.copy()
            environment.update({
                'PATH': f'{temp_path}{os.pathsep}{environment["PATH"]}',
                'DOCKER_MARKER': str(marker),
            })
            environment.pop('SIM_NETWORK_MODE', None)
            environment.pop('GZ_SERVER_IP', None)

            result = subprocess.run(
                [str(temp_path / 'run.sh'), '--env', 'test', 'down'],
                text=True,
                capture_output=True,
                env=environment,
                check=False,
            )

        self.assertEqual(result.returncode, 2)
        self.assertIn('GZ_SERVER_IP is required when SIM_NETWORK_MODE=lan', result.stderr)
        self.assertFalse(marker.exists())

    def test_native_gui_connector_remains_development_pc_only(self):
        script = (BUNDLE / 'scripts/gz-gui-connect.sh').read_text()
        self.assertIn('gz sim --force-version 8 -g', script)
        self.assertNotIn('gz sim -s', script)

    def test_run_sh_test_executes_observation_bundle_contract(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            shutil.copy2(BUNDLE / 'run.sh', temp_path / 'run.sh')
            (temp_path / '.env.test').write_text('SIM_NETWORK_MODE=internal\n')
            python_log = temp_path / 'python-commands'
            fake_python = temp_path / 'python3'
            fake_python.write_text(
                '#!/usr/bin/env bash\n'
                'printf \'%s\\n\' "$@" >> "$PYTHON_LOG"\n'
            )
            fake_python.chmod(0o755)
            fake_docker = temp_path / 'docker'
            fake_docker.write_text('#!/usr/bin/env bash\nexit 0\n')
            fake_docker.chmod(0o755)
            environment = os.environ.copy()
            environment.update({
                'PATH': f'{temp_path}{os.pathsep}{environment["PATH"]}',
                'PYTHON_LOG': str(python_log),
            })

            result = subprocess.run(
                [str(temp_path / 'run.sh'), '--env', 'test', 'test'],
                text=True,
                capture_output=True,
                env=environment,
                check=False,
            )
            python_commands = python_log.read_text()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertRegex(
            python_commands, r'test/test_observation_bundle\.py\n'
        )


if __name__ == '__main__':
    unittest.main()
