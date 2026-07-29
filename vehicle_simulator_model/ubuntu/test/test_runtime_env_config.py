import os
import shutil
import subprocess
import textwrap
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


BUNDLE = Path(__file__).resolve().parents[1]


class RuntimeEnvConfigTest(unittest.TestCase):
    def run_command(
        self,
        profile=None,
        dotenv=None,
        environment_overrides=None,
        selector_args=None,
        command=('sim-up',),
    ):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            script = root / 'run.sh'
            bin_dir = root / 'bin'
            docker_log = root / 'docker.log'
            shutil.copy2(BUNDLE / 'run.sh', script)
            if dotenv is not None:
                (root / f'.env.{profile}').write_text(dotenv)
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

            arguments = [str(script)]
            if selector_args is not None:
                arguments.extend(selector_args)
            elif profile is not None:
                arguments.extend(('--env', profile))
            arguments.extend(command)

            result = subprocess.run(
                arguments,
                cwd=root,
                text=True,
                capture_output=True,
                env=environment,
            )
            return result, docker_log.read_text() if docker_log.exists() else ''

    def test_arbitrary_profile_selects_lan_compose_and_env_file(self):
        result, docker_log = self.run_command(
            'dev1', 'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n'
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('.env.dev1', docker_log)
        self.assertIn('compose.lan.yaml', docker_log)

    def test_profile_overrides_inherited_environment(self):
        result, docker_log = self.run_command(
            'dev2',
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
            {'SIM_NETWORK_MODE': 'internal'},
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('compose.lan.yaml', docker_log)

    def test_missing_profile_fails_before_docker(self):
        result, docker_log = self.run_command('dev3')
        self.assertEqual(result.returncode, 2)
        self.assertIn('.env.dev3', result.stderr)
        self.assertEqual(docker_log, '')

    def test_invalid_profile_name_fails_before_docker(self):
        result, docker_log = self.run_command('../server')
        self.assertEqual(result.returncode, 2)
        self.assertIn('profile name', result.stderr)
        self.assertEqual(docker_log, '')

    def test_missing_selector_value_fails_before_docker(self):
        result, docker_log = self.run_command(selector_args=('--env',), command=())
        self.assertEqual(result.returncode, 2)
        self.assertIn('--env', result.stderr)
        self.assertNotIn('Unknown command', result.stderr)
        self.assertEqual(docker_log, '')

    def test_duplicate_selector_fails_before_docker(self):
        result, docker_log = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=internal\n',
            selector_args=('--env', 'dev1', '--env', 'dev2'),
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn('--env', result.stderr)
        self.assertNotIn('Unknown command', result.stderr)
        self.assertEqual(docker_log, '')

    def test_malformed_profile_fails_with_line_number_before_docker(self):
        result, docker_log = self.run_command(
            'dev4', '# profile settings\nexport SIM_NETWORK_MODE=lan\n'
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn('.env.dev4:2: expected NAME=value', result.stderr)
        self.assertEqual(docker_log, '')

    def test_bare_command_fails_before_docker(self):
        result, docker_log = self.run_command()
        self.assertEqual(result.returncode, 2)
        self.assertIn('--env', result.stderr)
        self.assertEqual(docker_log, '')


if __name__ == '__main__':
    unittest.main()
