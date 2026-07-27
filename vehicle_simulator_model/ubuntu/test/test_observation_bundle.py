import os
import subprocess
import tempfile
import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]


class ObservationBundleTest(unittest.TestCase):
    def test_lan_profile_uses_host_network_without_changing_base_compose(self):
        base = (BUNDLE / 'compose.yaml').read_text()
        lan = (BUNDLE / 'compose.lan.yaml').read_text()
        self.assertIn('internal: true', base)
        self.assertNotIn('network_mode: host', base)
        for service in ('dds-discovery:', 'gazebo-server:', 'sim-adapter:',
                        'slam-mapper:', 'slam-inspector:'):
            self.assertIn(service, lan)
        self.assertIn('network_mode: host', lan)
        self.assertIn('networks: !reset []', lan)
        self.assertIn('GZ_IP: "${GZ_SERVER_IP:?', lan)
        self.assertIn('DDS_DISCOVERY_HOST: 127.0.0.1', lan)
        self.assertIn('GZ_RELAY_HOST: ""', lan)

    def test_mac_client_sets_one_partition_and_runs_gui_only(self):
        script = (BUNDLE / 'scripts/gz-gui-connect.sh').read_text()
        self.assertIn('GZ_PARTITION="${GZ_PARTITION:-mentorpi-sim}"', script)
        self.assertIn('GZ_RELAY="$server_ip"', script)
        self.assertIn('GZ_IP="$client_ip"', script)
        self.assertIn('gz topic -l', script)
        self.assertIn('gz sim --force-version 8 -g', script)
        self.assertNotIn('gz sim -s', script)

    def test_lan_mode_requires_server_ip_before_calling_docker(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            marker = temp_path / 'docker-called'
            fake_docker = temp_path / 'docker'
            fake_docker.write_text(
                '#!/usr/bin/env bash\n'
                'printf called > "$DOCKER_MARKER"\n'
            )
            fake_docker.chmod(0o755)
            env = os.environ.copy()
            env.update({
                'SIM_NETWORK_MODE': 'lan',
                'PATH': f'{temp_path}:{env["PATH"]}',
                'DOCKER_MARKER': str(marker),
            })
            env.pop('GZ_SERVER_IP', None)

            result = subprocess.run(
                [str(BUNDLE / 'run.sh'), 'down'],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn(
                'GZ_SERVER_IP is required when SIM_NETWORK_MODE=lan', result.stderr
            )
            self.assertFalse(marker.exists())

    def test_unknown_network_mode_is_rejected_before_calling_docker(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            marker = temp_path / 'docker-called'
            fake_docker = temp_path / 'docker'
            fake_docker.write_text(
                '#!/usr/bin/env bash\n'
                'printf called > "$DOCKER_MARKER"\n'
            )
            fake_docker.chmod(0o755)
            env = os.environ.copy()
            env.update({
                'SIM_NETWORK_MODE': 'unsupported',
                'PATH': f'{temp_path}:{env["PATH"]}',
                'DOCKER_MARKER': str(marker),
            })

            result = subprocess.run(
                [str(BUNDLE / 'run.sh'), 'down'],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn(
                'SIM_NETWORK_MODE must be internal or lan', result.stderr
            )
            self.assertFalse(marker.exists())

    def test_lan_mode_adds_the_lan_profile_to_docker_compose(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            fake_docker = temp_path / 'docker'
            fake_docker.write_text(
                '#!/usr/bin/env bash\n'
                'printf \'%s\\n\' "$@"\n'
            )
            fake_docker.chmod(0o755)
            env = os.environ.copy()
            env.update({
                'SIM_NETWORK_MODE': 'lan',
                'GZ_SERVER_IP': '192.168.50.10',
                'PATH': f'{temp_path}:{env["PATH"]}',
            })

            result = subprocess.run(
                [str(BUNDLE / 'run.sh'), 'down'],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 0)
            self.assertIn(str(BUNDLE / 'compose.lan.yaml'), result.stdout)

    def test_run_sh_test_executes_observation_bundle_contract(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
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
            env = os.environ.copy()
            env.update({
                'PATH': f'{temp_path}:{env["PATH"]}',
                'PYTHON_LOG': str(python_log),
            })

            result = subprocess.run(
                [str(BUNDLE / 'run.sh'), 'test'],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 0)
            self.assertIn(
                str(BUNDLE / 'test/test_observation_bundle.py'),
                python_log.read_text(),
            )

    def test_mac_client_preflight_connects_and_starts_gui(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            fake_gz = temp_path / 'gz'
            fake_gz.write_text(
                '#!/usr/bin/env bash\n'
                'case "${1:-} ${2:-}" in\n'
                "  'sim --versions') printf '8.14.0\\n' ;;\n"
                "  'topic -l') printf '/world/mentorpi_warehouse/stats\\n' ;;\n"
                "  'sim --force-version') printf '%s|%s|%s\\n' \"$GZ_IP\" \"$GZ_RELAY\" \"$GZ_PARTITION\" ;;\n"
                'esac\n'
            )
            fake_gz.chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{temp_path}:{env["PATH"]}'

            result = subprocess.run(
                [
                    str(BUNDLE / 'scripts/gz-gui-connect.sh'),
                    '192.168.50.10',
                    '192.168.50.20',
                ],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 0)
            self.assertIn(
                '192.168.50.20|192.168.50.10|mentorpi-sim', result.stdout
            )

    def test_mac_client_rejects_another_gazebo_version(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            fake_gz = temp_path / 'gz'
            fake_gz.write_text(
                '#!/usr/bin/env bash\n'
                "printf '8.13.0\\n'\n"
            )
            fake_gz.chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{temp_path}:{env["PATH"]}'

            result = subprocess.run(
                [
                    str(BUNDLE / 'scripts/gz-gui-connect.sh'),
                    '192.168.50.10',
                    '192.168.50.20',
                ],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 3)
            self.assertIn(
                'Gazebo Sim 8.14.0 is required on the GUI client', result.stderr
            )

    def test_mac_client_rejects_an_unreachable_gazebo_server(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            fake_gz = temp_path / 'gz'
            fake_gz.write_text(
                '#!/usr/bin/env bash\n'
                'case "${1:-} ${2:-}" in\n'
                "  'sim --versions') printf '8.14.0\\n' ;;\n"
                "  'topic -l') printf '/other/topic\\n' ;;\n"
                'esac\n'
            )
            fake_gz.chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{temp_path}:{env["PATH"]}'

            result = subprocess.run(
                [
                    str(BUNDLE / 'scripts/gz-gui-connect.sh'),
                    '192.168.50.10',
                    '192.168.50.20',
                ],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                'Gazebo server is not reachable at 192.168.50.10 on partition '
                'mentorpi-sim',
                result.stderr,
            )
