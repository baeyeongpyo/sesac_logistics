"""Lifecycle contract for the standalone Foxglove Bridge control script."""

import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / 'tools' / 'foxglove_bridge_ctl.sh'


class FoxgloveBridgeControlScriptTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.temporary_path = Path(self.temporary_directory.name)
        fake_bin = self.temporary_path / 'bin'
        fake_bin.mkdir()
        ros2 = fake_bin / 'ros2'
        ros2.write_text('#!/usr/bin/env bash\nexec sleep 60\n', encoding='utf-8')
        ros2.chmod(0o755)
        self.environment = {
            **os.environ,
            'PATH': f'{fake_bin}:{os.environ["PATH"]}',
            'TMPDIR': str(self.temporary_path),
        }

    def tearDown(self):
        if not SCRIPT_PATH.is_file():
            self.temporary_directory.cleanup()
            return
        subprocess.run(
            [str(SCRIPT_PATH), 'stop'],
            env=self.environment,
            check=False,
            capture_output=True,
            text=True,
        )
        self.temporary_directory.cleanup()

    def command(self, action: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(SCRIPT_PATH), action],
            env=self.environment,
            check=False,
            capture_output=True,
            text=True,
        )

    def test_start_status_and_stop_manage_the_recorded_process(self):
        self.assertTrue(
            SCRIPT_PATH.is_file(),
            'tools/foxglove_bridge_ctl.sh must provide the lifecycle command',
        )
        started = self.command('start')
        self.assertEqual(started.returncode, 0, started.stderr)

        pid_file = self.temporary_path / 'foxglove-bridge.pid'
        pid = int(pid_file.read_text(encoding='utf-8').strip())
        os.kill(pid, 0)

        status = self.command('status')
        self.assertEqual(status.returncode, 0, status.stderr)
        self.assertIn(f'PID={pid}', status.stdout)

        stop_started_at = time.monotonic()
        stopped = self.command('stop')
        self.assertLess(time.monotonic() - stop_started_at, 2.0)
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertFalse(pid_file.exists())

        for _ in range(20):
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)
        else:
            self.fail(f'process {pid} remained after stop')


if __name__ == '__main__':
    unittest.main()
