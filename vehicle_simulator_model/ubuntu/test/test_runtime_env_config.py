import os
import shutil
import subprocess
import textwrap
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


BUNDLE = Path(__file__).resolve().parents[1]


class RuntimeEnvConfigTest(unittest.TestCase):
    def run_command(self, dotenv, environment_overrides=None):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            script = root / 'run.sh'
            bin_dir = root / 'bin'
            docker_log = root / 'docker.log'
            shutil.copy2(BUNDLE / 'run.sh', script)
            (root / '.env').write_text(dotenv)
            bin_dir.mkdir()
            fake_docker = bin_dir / 'docker'
            fake_docker.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                printf '%s\\n' "$*" >> "$DOCKER_LOG"
            '''))
            fake_docker.chmod(0o755)

            environment = os.environ.copy()
            environment.pop('SIM_NETWORK_MODE', None)
            environment.pop('GZ_SERVER_IP', None)
            environment.update({
                'PATH': f'{bin_dir}{os.pathsep}{environment["PATH"]}',
                'DOCKER_LOG': str(docker_log),
            })
            if environment_overrides:
                environment.update(environment_overrides)

            result = subprocess.run(
                [str(script), 'sim-up'],
                cwd=root,
                text=True,
                capture_output=True,
                env=environment,
            )
            return result, docker_log.read_text() if docker_log.exists() else ''

    def test_dotenv_selects_lan_profile(self):
        result, docker_log = self.run_command(
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n'
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('compose.lan.yaml', docker_log)

    def test_exported_environment_overrides_dotenv(self):
        result, docker_log = self.run_command(
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
            {'SIM_NETWORK_MODE': 'internal'},
        )
        self.assertEqual(result.returncode, 0)
        self.assertNotIn('compose.lan.yaml', docker_log)

    def test_invalid_dotenv_fails_before_docker(self):
        result, docker_log = self.run_command('export SIM_NETWORK_MODE=lan\n')
        self.assertEqual(result.returncode, 2)
        self.assertIn('.env:1: expected NAME=value', result.stderr)
        self.assertEqual(docker_log, '')


if __name__ == '__main__':
    unittest.main()
