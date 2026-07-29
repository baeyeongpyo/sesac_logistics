import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


WORLD = Path(__file__).resolve().parent / 'worlds' / 'pallet_manager_test.sdf'
COMMAND_SERVICE = '/warehouse/pallet/command'
BROKEN_COMMAND_SERVICE = '/warehouse/pallet/broken_command'
POSE_TOPIC = '/world/pallet_manager_test/pose/info'


class PalletManagerIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.env = os.environ.copy()
        self.env['GZ_PARTITION'] = f'mentorpi-pallet-manager-{os.getpid()}'
        self.stdout = tempfile.TemporaryFile(mode='w+', encoding='utf-8')
        self.stderr = tempfile.TemporaryFile(mode='w+', encoding='utf-8')
        self.server = subprocess.Popen(
            ['gz', 'sim', '-r', '-s', '-v', '4', str(WORLD)],
            env=self.env,
            stdout=self.stdout,
            stderr=self.stderr,
            start_new_session=True,
            text=True,
        )
        self._cleaned_up = False
        self.addCleanup(self._cleanup_server)
        self.wait_for_services({COMMAND_SERVICE, BROKEN_COMMAND_SERVICE}, timeout=10)

    def tearDown(self):
        self._cleanup_server()
        output = self.server_output()
        if output.strip():
            sys.stderr.write(f'\n--- Gazebo server stdout/stderr ---\n{output[-20000:]}\n')

    def _cleanup_server(self):
        if self._cleaned_up:
            return
        self._cleaned_up = True
        if self.server.poll() is None:
            try:
                os.killpg(self.server.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.server.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.server.wait(timeout=5)

    def server_output(self):
        chunks = []
        for stream in (self.stdout, self.stderr):
            stream.flush()
            stream.seek(0)
            chunks.append(stream.read())
        return '\n'.join(chunks)

    def wait_for_services(self, expected, timeout):
        deadline = time.monotonic() + timeout
        last_output = ''
        while time.monotonic() < deadline:
            if self.server.poll() is not None:
                self.fail(
                    f'Gazebo exited before advertising services\n{self.server_output()}')
            try:
                result = subprocess.run(
                    ['gz', 'service', '--list'],
                    env=self.env,
                    capture_output=True,
                    text=True,
                    timeout=5,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                last_output = 'gz service --list timed out'
                time.sleep(0.1)
                continue
            last_output = result.stdout + result.stderr
            services = set(result.stdout.splitlines())
            if expected <= services:
                return
            time.sleep(0.1)
        self.fail(
            f'timed out waiting for services {sorted(expected)}; '
            f'last service output:\n{last_output}\n{self.server_output()}')

    def command(self, text, service=COMMAND_SERVICE):
        result = subprocess.run(
            [
                'gz', 'service',
                '--service', service,
                '--reqtype', 'gz.msgs.StringMsg',
                '--reptype', 'gz.msgs.StringMsg',
                '--timeout', '6000',
                '--req', f'data: {json.dumps(text)}',
            ],
            env=self.env,
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
        if result.returncode != 0:
            self.fail(
                f'command failed ({text!r}):\n{result.stdout}{result.stderr}\n'
                f'{self.server_output()}')
        match = re.search(
            r'data:\s*("(?:\\.|[^"\\])*")', result.stdout + result.stderr)
        if match is None:
            self.fail(
                f'command response has no StringMsg data ({text!r}):\n'
                f'{result.stdout}{result.stderr}\n{self.server_output()}')
        return json.loads(match.group(1))

    def pose_snapshot(self):
        result = subprocess.run(
            [
                'gz', 'topic', '--echo', '--topic', POSE_TOPIC,
                '--num', '1', '--json-output',
            ],
            env=self.env,
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        if result.returncode != 0:
            return {}
        start = result.stdout.find('{')
        if start < 0:
            return {}
        try:
            message, _ = json.JSONDecoder().raw_decode(result.stdout[start:])
        except json.JSONDecodeError:
            return {}

        poses = {}
        for pose in message.get('pose', []):
            position = pose.get('position', {})
            poses[pose.get('name', '')] = SimpleNamespace(
                x=float(position.get('x', 0.0)),
                y=float(position.get('y', 0.0)),
            )
        return poses

    def try_model_pose(self, name, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pose = self.pose_snapshot().get(name)
            if pose is not None:
                return pose
            time.sleep(0.05)
        return None

    def model_pose(self, name):
        pose = self.try_model_pose(name)
        if pose is None:
            self.fail(f'model pose not found for {name!r}\n{self.server_output()}')
        return pose

    def test_default_spawn_state_transition_and_remove(self):
        self.assertEqual(self.command('list'), (
            'ok|list|pallet_01:fresh:loaded,'
            'pallet_02:fresh:loaded,'
            'pallet_03:fresh:loaded,'
            'pallet_04:normal:loaded,'
            'pallet_05:normal:loaded,'
            'pallet_06:normal:loaded'
        ))
        self.assertEqual(
            self.command('remove|missing_pallet'),
            'error|NOT_FOUND|missing_pallet')
        self.assertEqual(
            self.command(
                'spawn|pallet_bad|fresh|empty|-4.0|-3.0|0',
                service=BROKEN_COMMAND_SERVICE),
            'error|MODEL_TEMPLATE_INVALID|pallet_bad')

        self.assertTrue(self.command(
            'spawn|pallet_07|fresh|loaded|-2.5|-2.8|0').startswith('ok|spawn|'))
        spawned_pose = self.model_pose('pallet_07')
        self.assertAlmostEqual(spawned_pose.x, -2.5, delta=0.01)
        self.assertAlmostEqual(spawned_pose.y, -2.8, delta=0.01)
        self.assertEqual(
            self.command('spawn|pallet_08|normal|empty|-2.5|-2.8|0'),
            'error|SPAWN_POSE_OCCUPIED|pallet_08')
        self.assertEqual(
            self.command('spawn|pallet_09|normal|empty|1.8|-2.8|0'),
            'error|SPAWN_POSE_OCCUPIED|pallet_09')

        self.assertTrue(
            self.command('state|pallet_07|empty|').startswith('ok|state|'))
        self.assertIsNone(self.try_model_pose('pallet_07_payload'))
        self.assertTrue(
            self.command('state|pallet_07|loaded|normal').startswith('ok|state|'))
        self.assertIsNotNone(self.model_pose('pallet_07_payload'))
        self.assertIn('pallet_07:normal:loaded', self.command('list'))

        self.assertTrue(
            self.command('remove|pallet_07').startswith('ok|remove|'))
        self.assertIsNone(self.try_model_pose('pallet_07'))
        self.assertIsNone(self.try_model_pose('pallet_07_payload'))
        self.assertNotIn('pallet_07:', self.command('list'))


if __name__ == '__main__':
    unittest.main()
