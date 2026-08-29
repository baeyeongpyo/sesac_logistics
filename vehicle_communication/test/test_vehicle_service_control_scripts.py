"""Process-discovery contract for the vehicle communication shell scripts."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
FOXGLOVE_PROCESS_PREFIX = 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'


class VehicleCommunicationScriptTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.temporary_path = Path(self.temporary_directory.name)
        self.package = self.temporary_path / 'vehicle_communication'
        shutil.copytree(PACKAGE, self.package, ignore=shutil.ignore_patterns('__pycache__'))
        self.log_file = self.temporary_path / 'invocations.log'

        (self.package / 'runtime.env').write_text(
            '\n'.join((
                'VEHICLE_ROBOT_ID=test_vehicle',
                'VEHICLE_COMMAND_API_PORT=18082',
                'FOXGLOVE_PORT=18765',
                '',
            )),
            encoding='utf-8',
        )

        fake_bin = self.temporary_path / 'bin'
        fake_bin.mkdir()
        self.fake_bin = fake_bin
        self._write_fake_command(fake_bin / 'ros2', 'ros2')
        self._write_fake_command(fake_bin / 'python3', 'python3')
        self.environment = {
            **os.environ,
            'HOME': str(self.temporary_path / 'home'),
            'PATH': f'{fake_bin}:{os.environ["PATH"]}',
            'TEST_INVOCATION_LOG': str(self.log_file),
            'TEST_ROS_SETUP_LOADED': 'from_zshrc',
        }

    def tearDown(self):
        for name in ('vehicle_communication_stop.sh', 'foxglove_stop.sh', 'command_api_stop.sh'):
            script = self.package / 'tools' / name
            if script.is_file():
                subprocess.run(
                    [str(script)],
                    env=self.environment,
                    check=False,
                    capture_output=True,
                    text=True,
                )
        self.temporary_directory.cleanup()

    def _write_fake_command(self, path, name):
        path.write_text(
            '\n'.join((
                '#!/usr/bin/env bash',
                'set -euo pipefail',
                f'printf "{name}_setup=%s\\n" "${{TEST_ROS_SETUP_LOADED:-}}" >> "$TEST_INVOCATION_LOG"',
                f'printf "{name}_arg=%s\\n" "$@" >> "$TEST_INVOCATION_LOG"',
                'child=""',
                'stop() { [[ -z "$child" ]] || kill "$child" 2>/dev/null || true; exit 0; }',
                'trap stop TERM INT',
                'while true; do sleep 60 & child=$!; wait "$child"; done',
                '',
            )),
            encoding='utf-8',
        )
        path.chmod(0o755)

    def command(self, name):
        script = self.package / 'tools' / name
        self.assertTrue(script.is_file(), f'tools/{name} must provide the lifecycle command')
        return subprocess.run(
            [str(script)],
            env=self.environment,
            check=False,
            capture_output=True,
            text=True,
        )

    @property
    def foxglove_process(self):
        return (FOXGLOVE_PROCESS_PREFIX, 'port:=18765 topic_whitelist')

    @property
    def command_api_process(self):
        return (f'python3 {self.package / "vehicle_command_api.py"} --host',)

    def start_other_process(self, command):
        process = subprocess.Popen(command, env=self.environment)
        self.addCleanup(self.stop_process, process)
        return process

    @staticmethod
    def stop_process(process):
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=2)

    @staticmethod
    def matching_process_ids(process_text):
        if isinstance(process_text, str):
            process_text = (process_text,)
        result = subprocess.run(
            ['ps', '-eo', 'pid=,args='],
            check=True,
            capture_output=True,
            text=True,
        )
        return {
            int(line.split(maxsplit=1)[0])
            for line in result.stdout.splitlines()
            if all(text in line for text in process_text)
        }

    def wait_for_processes(self, process_text):
        for _ in range(20):
            process_ids = self.matching_process_ids(process_text)
            if process_ids:
                return process_ids
            time.sleep(0.1)
        self.fail(f'process did not start: {" ".join(process_text)}')

    def wait_for_no_processes(self, process_text):
        for _ in range(20):
            if not self.matching_process_ids(process_text):
                return
            time.sleep(0.1)
        self.fail(f'process did not stop: {" ".join(process_text)}')

    def wait_for_invocation(self):
        for _ in range(20):
            if self.log_file.is_file():
                return self.log_file.read_text(encoding='utf-8')
            time.sleep(0.1)
        self.fail('the started process did not record its invocation')

    def test_foxglove_start_uses_ps_to_avoid_duplicate_bridge_and_stop_ends_it(self):
        started = self.command('foxglove_start.sh')
        self.assertEqual(started.returncode, 0, started.stderr)
        original_process_ids = self.wait_for_processes(self.foxglove_process)
        invocation = self.wait_for_invocation()
        self.assertIn('ros2_setup=from_zshrc', invocation)
        self.assertIn('ros2_arg=port:=18765', invocation)
        self.assertIn(
            'ros2_arg=topic_whitelist:=["^/(map|tf|tf_static|joint_states|amcl_pose|odom|ros_robot_controller/battery)$"]',
            invocation,
        )

        repeated_start = self.command('foxglove_start.sh')
        self.assertEqual(repeated_start.returncode, 0, repeated_start.stderr)
        self.assertEqual(self.matching_process_ids(self.foxglove_process), original_process_ids)

        stopped = self.command('foxglove_stop.sh')
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.wait_for_no_processes(self.foxglove_process)
        self.assertFalse((Path(self.environment['HOME']) / 'log' / 'foxglove_bridge_pid').exists())

    def test_command_api_start_uses_ps_to_avoid_duplicate_api_and_stop_ends_it(self):
        started = self.command('command_api_start.sh')
        self.assertEqual(started.returncode, 0, started.stderr)
        original_process_ids = self.wait_for_processes(self.command_api_process)
        invocation = self.wait_for_invocation()
        self.assertIn('python3_setup=from_zshrc', invocation)
        self.assertIn('python3_arg=--port', invocation)
        self.assertIn('python3_arg=18082', invocation)
        self.assertIn('python3_arg=--robot-id', invocation)
        self.assertIn('python3_arg=test_vehicle', invocation)

        repeated_start = self.command('command_api_start.sh')
        self.assertEqual(repeated_start.returncode, 0, repeated_start.stderr)
        self.assertEqual(self.matching_process_ids(self.command_api_process), original_process_ids)

        stopped = self.command('command_api_stop.sh')
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.wait_for_no_processes(self.command_api_process)
        self.assertFalse((Path(self.environment['HOME']) / 'log' / 'vehicle_command_api_pid').exists())

    def test_combined_start_and_stop_delegate_to_the_independent_scripts(self):
        started = self.command('vehicle_communication_start.sh')
        self.assertEqual(started.returncode, 0, started.stderr)
        self.wait_for_processes(self.foxglove_process)
        self.wait_for_processes(self.command_api_process)

        stopped = self.command('vehicle_communication_stop.sh')
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.wait_for_no_processes(self.foxglove_process)
        self.wait_for_no_processes(self.command_api_process)

    def test_stop_scripts_succeed_when_their_service_is_not_running(self):
        for name in ('foxglove_stop.sh', 'command_api_stop.sh', 'vehicle_communication_stop.sh'):
            stopped = self.command(name)
            self.assertEqual(stopped.returncode, 0, stopped.stderr)

    def test_scripts_ignore_other_deployments_and_foxglove_ports(self):
        other_foxglove = self.start_other_process((
            str(self.fake_bin / 'ros2'),
            'launch',
            'foxglove_bridge',
            'foxglove_bridge_launch.xml',
            'port:=187650',
        ))
        other_api = self.start_other_process((
            str(self.fake_bin / 'python3'),
            str(self.package / 'vehicle_command_api.py.bak'),
        ))
        self.wait_for_processes((FOXGLOVE_PROCESS_PREFIX, 'port:=187650'))
        self.wait_for_processes(f'python3 {self.package / "vehicle_command_api.py.bak"}')

        self.assertEqual(self.command('foxglove_start.sh').returncode, 0)
        self.assertEqual(self.command('command_api_start.sh').returncode, 0)
        self.wait_for_processes(self.foxglove_process)
        self.wait_for_processes(self.command_api_process)

        self.assertEqual(self.command('foxglove_stop.sh').returncode, 0)
        self.assertEqual(self.command('command_api_stop.sh').returncode, 0)
        self.wait_for_no_processes(self.foxglove_process)
        self.wait_for_no_processes(self.command_api_process)
        self.assertIsNone(other_foxglove.poll())
        self.assertIsNone(other_api.poll())


if __name__ == '__main__':
    unittest.main()
