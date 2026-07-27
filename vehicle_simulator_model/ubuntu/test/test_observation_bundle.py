import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]


class ObservationBundleTest(unittest.TestCase):
    def run_with_fake_docker(self, args, extra_env=None):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            docker_log = temp_path / 'docker.log'
            fake_docker = temp_path / 'docker'
            fake_docker.write_text(
                '#!/usr/bin/env bash\n'
                'printf \'VIEWER_DOMAIN=%s\\n\' "${VIEWER_DOMAIN:-}" '
                '>> "$FAKE_DOCKER_LOG"\n'
                'printf \'VIEWER_ALLOW_CIDRS=%s\\n\' '
                '"${VIEWER_ALLOW_CIDRS:-}" >> "$FAKE_DOCKER_LOG"\n'
                'printf \'ARGS=%s\\n\' "$*" >> "$FAKE_DOCKER_LOG"\n'
            )
            fake_docker.chmod(0o755)
            env = os.environ.copy()
            env.update({
                'PATH': f'{temp_path}:{env["PATH"]}',
                'FAKE_DOCKER_LOG': str(docker_log),
                'SIM_NETWORK_MODE': 'internal',
            })
            env.pop('VIEWER_DOMAIN', None)
            env.pop('VIEWER_ALLOW_CIDRS', None)
            if extra_env:
                env.update(extra_env)

            result = subprocess.run(
                [str(BUNDLE / 'run.sh'), *args],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            log = docker_log.read_text() if docker_log.exists() else ''
            return result, log

    def test_gateway_is_the_only_viewer_service_with_host_ports(self):
        viewer = (BUNDLE / 'compose.viewer.yaml').read_text()
        public = (BUNDLE / 'compose.viewer-public.yaml').read_text()
        caddy = (BUNDLE / 'Caddyfile.viewer').read_text()

        self.assertIn('web-gateway:', viewer)
        self.assertIn('127.0.0.1:${VIEWER_PORT:-8080}:8080', viewer)
        self.assertIn('ports: !override', public)
        self.assertIn('"80:80"', public)
        self.assertIn('"443:443"', public)
        self.assertIn('@allowed remote_ip {$VIEWER_ALLOW_CIDRS}', caddy)
        self.assertIn('reverse_proxy gazebo-viewer:6080', caddy)
        self.assertIn('respond 403', caddy)
        self.assertNotIn('basic_auth', caddy)
        for forbidden in ('10317:', '10318:', '11811:', '5900:', '6080:6080'):
            self.assertNotIn(forbidden, viewer + public)

    def test_public_compose_gives_only_gateway_https_ports_and_egress(self):
        base_files = [
            '-f', str(BUNDLE / 'compose.yaml'),
            '-f', str(BUNDLE / 'compose.viewer.yaml'),
        ]
        local_result = subprocess.run(
            [
                'docker', 'compose', *base_files, '--profile', 'viewer',
                'config', '--format', 'json',
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        public_env = os.environ.copy()
        public_env.update({
            'VIEWER_DOMAIN': 'sim.example.com',
            'VIEWER_ALLOW_CIDRS': '203.0.113.10/32 203.0.113.11/32',
        })
        public_result = subprocess.run(
            [
                'docker', 'compose', *base_files,
                '-f', str(BUNDLE / 'compose.viewer-public.yaml'),
                '--profile', 'viewer', 'config', '--format', 'json',
            ],
            text=True,
            capture_output=True,
            env=public_env,
            check=False,
        )

        self.assertEqual(local_result.returncode, 0, local_result.stderr)
        self.assertEqual(public_result.returncode, 0, public_result.stderr)
        local = json.loads(local_result.stdout)
        public = json.loads(public_result.stdout)
        self.assertNotIn('viewer-edge', local['networks'])
        self.assertEqual(
            set(public['services']['web-gateway']['networks']),
            {'mentorpi', 'viewer-edge'},
        )
        self.assertEqual(
            public['services']['web-gateway']['environment']['VIEWER_SITE'],
            'https://sim.example.com',
        )
        for service in ('dds-discovery', 'gazebo-server', 'sim-adapter',
                        'gazebo-viewer'):
            self.assertNotIn(
                'viewer-edge', public['services'][service]['networks']
            )
        self.assertEqual(
            [
                (port['published'], port['target'])
                for port in public['services']['web-gateway']['ports']
            ],
            [('80', 80), ('443', 443)],
        )
        for service, definition in public['services'].items():
            if service != 'web-gateway':
                self.assertNotIn('ports', definition)

    def test_public_viewer_rejects_unsafe_configuration_before_docker(self):
        valid_domain = 'sim.example.com'
        valid_allowlist = '203.0.113.10/32 203.0.113.11/32'
        cases = (
            (
                'empty domain',
                {'VIEWER_DOMAIN': '', 'VIEWER_ALLOW_CIDRS': valid_allowlist},
                'VIEWER_DOMAIN is required for public viewer',
            ),
            (
                'empty allowlist',
                {'VIEWER_DOMAIN': valid_domain, 'VIEWER_ALLOW_CIDRS': ''},
                'VIEWER_ALLOW_CIDRS is required for public viewer',
            ),
            (
                'whitespace-only allowlist',
                {'VIEWER_DOMAIN': valid_domain, 'VIEWER_ALLOW_CIDRS': ' \t '},
                'VIEWER_ALLOW_CIDRS is required for public viewer',
            ),
            (
                'all IPv4 sources',
                {
                    'VIEWER_DOMAIN': valid_domain,
                    'VIEWER_ALLOW_CIDRS': '0.0.0.0/0',
                },
                'does not allow unrestricted CIDRs',
            ),
            (
                'all IPv6 sources',
                {'VIEWER_DOMAIN': valid_domain, 'VIEWER_ALLOW_CIDRS': '::/0'},
                'does not allow unrestricted CIDRs',
            ),
            (
                'tab-separated unrestricted source',
                {
                    'VIEWER_DOMAIN': valid_domain,
                    'VIEWER_ALLOW_CIDRS': '203.0.113.10/32\t0.0.0.0/0',
                },
                'does not allow unrestricted CIDRs',
            ),
        )

        for label, viewer_env, message in cases:
            with self.subTest(label):
                result, docker_log = self.run_with_fake_docker(
                    ['viewer-up', 'public'], viewer_env
                )

                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
                self.assertEqual(docker_log, '')

    def test_public_viewer_accepts_tab_separator_and_exports_spaces(self):
        result, docker_log = self.run_with_fake_docker(
            ['viewer-up', 'public'],
            {
                'VIEWER_DOMAIN': 'sim.example.com',
                'VIEWER_ALLOW_CIDRS': (
                    '10.0.0.0/8\t192.168.0.0/16'
                ),
            },
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(
            'VIEWER_ALLOW_CIDRS=10.0.0.0/8 192.168.0.0/16\n',
            docker_log,
        )

    def test_public_viewer_rejects_non_hostname_domains_before_docker(self):
        invalid_domains = (
            'http://sim.example.com',
            'sim.example.com:443',
            'sim.example.com/viewer',
            'sim example.com',
            'sim.example.com\nrespond',
            'sim.example.com.',
            '시뮬레이터.example.com',
        )

        for domain in invalid_domains:
            with self.subTest(domain=repr(domain)):
                result, docker_log = self.run_with_fake_docker(
                    ['viewer-up', 'public'],
                    {
                        'VIEWER_DOMAIN': domain,
                        'VIEWER_ALLOW_CIDRS': '203.0.113.10/32',
                    },
                )

                self.assertEqual(result.returncode, 2)
                self.assertIn(
                    'VIEWER_DOMAIN must be an ASCII DNS hostname',
                    result.stderr,
                )
                self.assertEqual(docker_log, '')

    def test_public_viewer_rejects_malformed_allowlist_before_docker(self):
        invalid_allowlists = (
            'not-an-address',
            '203.0.113.0/24,2001:db8::/64',
            '203.0.113.0/33',
            '2001:db8::/129',
            '203.0.113.7/24',
            '203.0.113.10/32\x1b',
            '203.0.113.10/32\nrespond',
            'private_ranges',
        )

        for allowlist in invalid_allowlists:
            with self.subTest(allowlist=repr(allowlist)):
                result, docker_log = self.run_with_fake_docker(
                    ['viewer-up', 'public'],
                    {
                        'VIEWER_DOMAIN': 'sim.example.com',
                        'VIEWER_ALLOW_CIDRS': allowlist,
                    },
                )

                self.assertEqual(result.returncode, 2)
                self.assertIn(
                    'VIEWER_ALLOW_CIDRS must be a space- or tab-separated '
                    'list of valid IP addresses or CIDRs',
                    result.stderr,
                )
                self.assertEqual(docker_log, '')

    def test_public_viewer_normalizes_valid_hostname_and_allowlist(self):
        result, docker_log = self.run_with_fake_docker(
            ['viewer-up', 'public'],
            {
                'VIEWER_DOMAIN': 'SIM.Example.COM',
                'VIEWER_ALLOW_CIDRS': (
                    '  203.0.113.0/24  198.51.100.7 '
                    '2001:0db8::/64  2001:db8::1  '
                ),
            },
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('VIEWER_DOMAIN=sim.example.com\n', docker_log)
        self.assertIn(
            'VIEWER_ALLOW_CIDRS=203.0.113.0/24 198.51.100.7 '
            '2001:db8::/64 2001:db8::1\n',
            docker_log,
        )

    def test_viewer_up_rejects_lan_mode_before_docker(self):
        for mode in ('local', 'public'):
            with self.subTest(mode):
                result, docker_log = self.run_with_fake_docker(
                    ['viewer-up', mode],
                    {
                        'SIM_NETWORK_MODE': 'lan',
                        'GZ_SERVER_IP': '192.168.50.10',
                        'VIEWER_DOMAIN': 'sim.example.com',
                        'VIEWER_ALLOW_CIDRS': '203.0.113.10/32',
                    },
                )

                self.assertEqual(result.returncode, 2)
                self.assertIn(
                    'viewer-up requires SIM_NETWORK_MODE=internal', result.stderr
                )
                self.assertEqual(docker_log, '')

    def test_viewer_up_uses_local_gateway_unless_public_is_selected(self):
        local_result, local_log = self.run_with_fake_docker(
            ['viewer-up'], {'VIEWER_MODE': 'local'}
        )
        public_result, public_log = self.run_with_fake_docker(
            ['viewer-up'],
            {
                'VIEWER_MODE': 'public',
                'VIEWER_DOMAIN': 'sim.example.com',
                'VIEWER_ALLOW_CIDRS': '203.0.113.10/32',
            },
        )

        self.assertEqual(local_result.returncode, 0)
        self.assertIn(str(BUNDLE / 'compose.viewer.yaml'), local_log)
        self.assertNotIn(str(BUNDLE / 'compose.viewer-public.yaml'), local_log)
        self.assertIn(
            '--profile viewer up -d dds-discovery gazebo-server sim-adapter '
            'gazebo-viewer web-gateway',
            local_log,
        )
        self.assertEqual(public_result.returncode, 0)
        self.assertIn(str(BUNDLE / 'compose.viewer-public.yaml'), public_log)
        self.assertIn(
            'up -d dds-discovery gazebo-server sim-adapter gazebo-viewer '
            'web-gateway',
            public_log,
        )

    def test_viewer_down_and_logs_target_only_viewer_services(self):
        down_result, down_log = self.run_with_fake_docker(['viewer-down'])
        logs_result, logs_log = self.run_with_fake_docker(['viewer-logs'])

        self.assertEqual(down_result.returncode, 0)
        self.assertIn(
            '--profile viewer stop web-gateway gazebo-viewer', down_log
        )
        self.assertNotIn('gazebo-server', down_log)
        self.assertNotIn('sim-adapter', down_log)
        self.assertEqual(logs_result.returncode, 0)
        self.assertIn(
            '--profile viewer logs -f gazebo-viewer web-gateway', logs_log
        )
        self.assertNotIn('gazebo-server', logs_log)
        self.assertNotIn('sim-adapter', logs_log)

    def test_viewer_runs_real_gazebo_gui_read_only_and_shared(self):
        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        script = (BUNDLE / 'viewer-entrypoint.sh').read_text()
        compose = (BUNDLE / 'compose.viewer.yaml').read_text()

        for package in ('xvfb', 'x11vnc', 'novnc', 'websockify'):
            self.assertIn(package, dockerfile)
        self.assertIn('Xvfb :99', script)
        self.assertIn('gz sim --force-version 8 -g', script)
        self.assertIn('-viewonly', script)
        self.assertIn('-shared', script)
        self.assertIn('-forever', script)
        self.assertIn('websockify --web=/usr/share/novnc 6080', script)
        self.assertIn('gazebo-viewer:', compose)
        self.assertIn("expose: ['6080']", compose)
        self.assertNotIn('6080:6080', compose)

    def test_mac_preflight_unsupported_defers_to_task_4_browser_viewer(self):
        readme = (BUNDLE / 'README.md').read_text()
        self.assertIn('exit 4', readme)
        self.assertIn('UNSUPPORTED', readme)
        self.assertIn('Task 4', readme)
        self.assertIn('browser viewer', readme)
        self.assertIn('제공 후', readme)

    def test_lan_smoke_checks_two_clients_and_both_robots(self):
        script = (BUNDLE / 'test/smoke_observation.sh').read_text()
        self.assertIn('client-a', script)
        self.assertIn('client-b', script)
        self.assertIn('/world/mentorpi_warehouse/stats', script)
        self.assertIn('/robot_1/scan_raw', script)
        self.assertIn('/robot_2/scan_raw', script)

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
