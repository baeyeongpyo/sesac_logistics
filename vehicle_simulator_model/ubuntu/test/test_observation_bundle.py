import base64
import hashlib
import json
import os
import shutil
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]


class ObservationBundleTest(unittest.TestCase):
    def start_fake_viewer_server(self, mode):
        listener = socket.socket()
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        listener.settimeout(5)
        errors = []

        def receive_http(connection):
            request = b''
            while b'\r\n\r\n' not in request:
                chunk = connection.recv(4096)
                if not chunk:
                    raise RuntimeError('client closed during HTTP handshake')
                request += chunk
            headers = {}
            for line in request.decode('iso-8859-1').split('\r\n')[1:]:
                if ':' in line:
                    name, value = line.split(':', 1)
                    headers[name.lower()] = value.strip()
            key = headers['sec-websocket-key']
            accept = base64.b64encode(hashlib.sha1(
                (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()
            ).digest()).decode()
            connection.sendall((
                'HTTP/1.1 101 Switching Protocols\r\n'
                'Upgrade: websocket\r\n'
                'Connection: Upgrade\r\n'
                f'Sec-WebSocket-Accept: {accept}\r\n'
                'Sec-WebSocket-Protocol: binary\r\n'
                '\r\n'
            ).encode())

        def receive_exact(connection, length):
            data = b''
            while len(data) < length:
                chunk = connection.recv(length - len(data))
                if not chunk:
                    raise EOFError('WebSocket closed')
                data += chunk
            return data

        def receive_frame(connection):
            first, second = receive_exact(connection, 2)
            length = second & 0x7f
            if length == 126:
                length = struct.unpack('>H', receive_exact(connection, 2))[0]
            elif length == 127:
                length = struct.unpack('>Q', receive_exact(connection, 8))[0]
            mask = receive_exact(connection, 4) if second & 0x80 else b''
            payload = receive_exact(connection, length)
            if mask:
                payload = bytes(
                    value ^ mask[index % 4]
                    for index, value in enumerate(payload)
                )
            return first & 0x0f, payload

        def send_frame(connection, payload):
            header = b'\x82'
            if len(payload) < 126:
                header += bytes([len(payload)])
            elif len(payload) < 65536:
                header += b'\x7e' + struct.pack('>H', len(payload))
            else:
                header += b'\x7f' + struct.pack('>Q', len(payload))
            connection.sendall(header + payload)

        def serve_early_close():
            connection, _ = listener.accept()
            with connection:
                receive_http(connection)

        def serve_coupled_close():
            connections = []
            ready = threading.Barrier(2)

            def initialize(connection):
                try:
                    receive_http(connection)
                    send_frame(connection, b'RFB 003.008\n')
                    self.assertEqual(
                        receive_frame(connection),
                        (2, b'RFB 003.008\n'),
                    )
                    send_frame(connection, b'\x01\x01')
                    self.assertEqual(receive_frame(connection), (2, b'\x01'))
                    send_frame(connection, b'\x00\x00\x00\x00')
                    self.assertEqual(receive_frame(connection), (2, b'\x01'))
                    pixel_format = struct.pack(
                        '>BBBBHHHBBBxxx',
                        32, 24, 0, 1, 255, 255, 255, 16, 8, 0,
                    )
                    name = b'coupled-close'
                    send_frame(
                        connection,
                        struct.pack('>HH', 2, 2)
                        + pixel_format
                        + struct.pack('>I', len(name))
                        + name,
                    )
                    ready.wait(timeout=5)
                    time.sleep(0.2)
                except Exception as error:
                    errors.append(str(error))

            workers = []
            for _ in range(2):
                connection, _ = listener.accept()
                connections.append(connection)
                worker = threading.Thread(
                    target=initialize,
                    args=(connection,),
                    daemon=True,
                )
                worker.start()
                workers.append(worker)
            for worker in workers:
                worker.join(timeout=5)
            for connection in connections:
                connection.close()

        target = (
            serve_early_close if mode == 'early-close'
            else serve_coupled_close
        )

        def serve():
            try:
                target()
            except (OSError, RuntimeError) as error:
                errors.append(str(error))
            finally:
                listener.close()

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        return listener.getsockname()[1], thread, errors

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

    def test_viewer_compose_gives_only_gateway_ports_and_edge_egress(self):
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
        for mode in (local, public):
            self.assertIn('viewer-edge', mode['networks'])
            self.assertFalse(
                mode['networks']['viewer-edge'].get('internal', False)
            )
            self.assertEqual(
                set(mode['services']['web-gateway']['networks']),
                {'mentorpi', 'viewer-edge'},
            )
            self.assertEqual(
                mode['services']['web-gateway']
                .get('healthcheck', {})
                .get('test'),
                [
                    'CMD', 'wget', '-q', '-O', '/dev/null',
                    'http://127.0.0.1:2019/config/',
                ],
            )
            for service in ('dds-discovery', 'gazebo-server', 'sim-adapter',
                            'gazebo-viewer'):
                self.assertNotIn(
                    'viewer-edge', mode['services'][service]['networks']
                )
        self.assertEqual(
            [
                (port.get('host_ip'), port['published'], port['target'])
                for port in local['services']['web-gateway']['ports']
            ],
            [('127.0.0.1', '8080', 8080)],
        )
        self.assertEqual(
            public['services']['web-gateway']['environment']['VIEWER_SITE'],
            'https://sim.example.com',
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
            '--profile viewer up -d --wait dds-discovery gazebo-server '
            'sim-adapter gazebo-viewer web-gateway',
            local_log,
        )
        self.assertEqual(public_result.returncode, 0)
        self.assertIn(str(BUNDLE / 'compose.viewer-public.yaml'), public_log)
        self.assertIn(
            'up -d --wait dds-discovery gazebo-server sim-adapter '
            'gazebo-viewer web-gateway',
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

    def test_viewer_supervisor_preserves_failed_child_exit_status(self):
        bash = shutil.which('bash')
        version = subprocess.run(
            [bash, '-c', 'printf "%s.%s" "$BASH_VERSINFO" '
             '"${BASH_VERSINFO[1]}"'],
            text=True,
            capture_output=True,
            check=True,
        ).stdout
        if tuple(map(int, version.split('.'))) < (4, 3):
            self.skipTest('viewer runtime requires Bash 4.3+ for wait -n')

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            for name, body in {
                'Xvfb': '#!/usr/bin/env bash\nexec /bin/sleep 60\n',
                'xdpyinfo': '#!/usr/bin/env bash\nexit 0\n',
                'gz': '#!/usr/bin/env bash\nexit 23\n',
                'x11vnc': '#!/usr/bin/env bash\nexec /bin/sleep 60\n',
                'websockify': '#!/usr/bin/env bash\nexec /bin/sleep 60\n',
            }.items():
                command = temp_path / name
                command.write_text(body)
                command.chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{temp_path}:{env["PATH"]}'

            result = subprocess.run(
                [bash, str(BUNDLE / 'viewer-entrypoint.sh')],
                text=True,
                capture_output=True,
                env=env,
                check=False,
                timeout=10,
            )

            self.assertEqual(result.returncode, 23, result.stderr)

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

    def test_viewer_smoke_checks_two_websocket_clients_and_isolation(self):
        script = (BUNDLE / 'test/smoke_observation.sh').read_text()
        self.assertIn('vnc.html?view_only=1', script)
        self.assertIn('websockify', script)
        self.assertIn('gz sim --force-version 8 -g', script)
        self.assertIn('mentorpi-healthcheck server', script)
        self.assertIn('mentorpi-healthcheck adapter', script)

    def test_viewer_smoke_does_not_require_lan_environment(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            for name, body in {
                'curl': (
                    '#!/usr/bin/env bash\n'
                    "printf '<title>noVNC</title>\\n'\n"
                ),
                'docker': '#!/usr/bin/env bash\nexit 0\n',
                'python3': '#!/usr/bin/env bash\nexit 0\n',
            }.items():
                command = temp_path / name
                command.write_text(body)
                command.chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{temp_path}:{env["PATH"]}'
            env.pop('GZ_SERVER_IP', None)
            env.pop('GZ_CLIENT_IP', None)

            result = subprocess.run(
                [str(BUNDLE / 'test/smoke_observation.sh'), 'viewer'],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stderr)

    def test_viewer_process_probe_rejects_its_own_command_text(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            fake_docker = temp_path / 'docker'
            fake_docker.write_text(
                '#!/usr/bin/env bash\n'
                'if [[ "$*" == *"exec -T gazebo-viewer python3 -"* ]]; then\n'
                '  exec python3 -\n'
                'fi\n'
                'exit 0\n'
            )
            fake_docker.chmod(0o755)
            env = os.environ.copy()
            env['PATH'] = f'{temp_path}:{env["PATH"]}'
            result = subprocess.run(
                [
                    'bash', '-c',
                    'source "$1"; check_viewer_processes',
                    '_', str(BUNDLE / 'test/smoke_observation.sh'),
                ],
                text=True,
                capture_output=True,
                env=env,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn('missing viewer process', result.stderr)

    def test_viewer_websocket_rejects_101_without_rfb_banner(self):
        port, thread, errors = self.start_fake_viewer_server('early-close')
        result = subprocess.run(
            [
                'bash', '-c',
                'source "$1"; check_viewer_websockets "$2"',
                '_', str(BUNDLE / 'test/smoke_observation.sh'),
                f'ws://127.0.0.1:{port}/websockify',
            ],
            text=True,
            capture_output=True,
            check=False,
            timeout=15,
        )
        thread.join(timeout=5)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn('RFB banner', result.stderr)
        self.assertEqual(errors, [])

    def test_viewer_websocket_rejects_coupled_close_a_then_b(self):
        port, thread, errors = self.start_fake_viewer_server('coupled-close')
        result = subprocess.run(
            [
                'bash', '-c',
                'source "$1"; check_viewer_websockets "$2" "$3"',
                '_', str(BUNDLE / 'test/smoke_observation.sh'),
                f'ws://127.0.0.1:{port}/websockify',
                'client-a-first',
            ],
            text=True,
            capture_output=True,
            check=False,
            timeout=15,
        )
        thread.join(timeout=5)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn('survivor=client-b after=client-a', result.stderr)
        self.assertEqual(errors, [])

    def test_viewer_websocket_rejects_coupled_close_b_then_a(self):
        port, thread, errors = self.start_fake_viewer_server('coupled-close')
        result = subprocess.run(
            [
                'bash', '-c',
                'source "$1"; check_viewer_websockets "$2" "$3"',
                '_', str(BUNDLE / 'test/smoke_observation.sh'),
                f'ws://127.0.0.1:{port}/websockify',
                'client-b-first',
            ],
            text=True,
            capture_output=True,
            check=False,
            timeout=15,
        )
        thread.join(timeout=5)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn('survivor=client-a after=client-b', result.stderr)
        self.assertEqual(errors, [])

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


if __name__ == '__main__':
    unittest.main()
