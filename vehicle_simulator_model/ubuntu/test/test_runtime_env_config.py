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
        bare_dotenv=None,
        selector_args=None,
        command=('sim-up',),
    ):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            script = root / 'run.sh'
            bin_dir = root / 'bin'
            docker_log = root / 'docker.log'
            shutil.copy2(BUNDLE / 'run.sh', script)
            (root / 'ros2_ws/src/mentorpi_gz_sim/models').mkdir(parents=True)
            (root / 'ros2_ws/src/mentorpi_gz_sim/worlds').mkdir(parents=True)
            (root / 'ros2_ws/src/mentorpi_gz_sim/worlds/warehouse.sdf').touch()
            if dotenv is not None:
                (root / f'.env.{profile}').write_text(dotenv)
            if bare_dotenv is not None:
                (root / '.env').write_text(bare_dotenv)
            bin_dir.mkdir()
            fake_docker = bin_dir / 'docker'
            fake_docker.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                if [[ "${@: -2}" == 'version --short' ]]; then
                  printf '2.24.4\\n'
                  exit 0
                fi
                printf '<%s>\\n' "$@" >> "$DOCKER_LOG"
            '''))
            fake_docker.chmod(0o755)
            fake_gz = bin_dir / 'gz'
            fake_gz.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                printf 'args=%s\n' "$*" >> "$GZ_LOG"
                printf 'ip=%s\npartition=%s\nresource=%s\n' \\
                  "$GZ_IP" "$GZ_PARTITION" "$GZ_SIM_RESOURCE_PATH" >> "$GZ_LOG"
            '''))
            fake_gz.chmod(0o755)

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
        self.assertRegex(
            docker_log, r'<--env-file>\n<[^>\n]*/\.env\.dev1>\n'
        )
        self.assertIn('compose.lan.yaml', docker_log)

    def test_profile_overrides_inherited_environment(self):
        result, docker_log = self.run_command(
            'dev2',
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
            {'SIM_NETWORK_MODE': 'internal'},
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('compose.lan.yaml', docker_log)

    def test_profile_ignores_conflicting_bare_dotenv(self):
        result, docker_log = self.run_command(
            'dev5',
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
            bare_dotenv='SIM_NETWORK_MODE=internal\n',
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('compose.lan.yaml', docker_log)

    def test_profile_does_not_parse_malformed_bare_dotenv(self):
        result, docker_log = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
            bare_dotenv='export SIM_NETWORK_MODE=internal\n',
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('compose.lan.yaml', docker_log)
        self.assertNotEqual(docker_log, '')

    def test_missing_profile_fails_before_docker(self):
        result, docker_log = self.run_command('dev3')
        self.assertEqual(result.returncode, 2)
        self.assertIn('.env.dev3', result.stderr)
        self.assertEqual(docker_log, '')

    def test_invalid_profile_names_fail_before_docker(self):
        for profile in ('-dev', 'dev.name', 'dev space', '../server'):
            with self.subTest(profile=profile):
                result, docker_log = self.run_command(profile)
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

    def test_duplicate_selector_after_command_fails_before_docker(self):
        result, docker_log = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=internal\n',
            command=('down', '--env', 'server'),
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn('--env', result.stderr)
        self.assertEqual(docker_log, '')

    def test_commands_without_arguments_reject_unexpected_tail_before_docker(self):
        for command in (
            'build',
            'down',
            'logs',
            'topics',
            'test',
            'fork-up',
            'foxglove-down',
            'foxglove-logs',
            'mapping-stop',
            'help',
        ):
            with self.subTest(command=command):
                result, docker_log = self.run_command(
                    'dev1',
                    'SIM_NETWORK_MODE=internal\n',
                    command=(command, 'unexpected'),
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn('does not accept arguments', result.stderr)
                self.assertEqual(docker_log, '')

    def test_sim_up_rejects_more_than_optional_gpu_before_docker(self):
        result, docker_log = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=internal\n',
            command=('sim-up', 'gpu', 'unexpected'),
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn('optional gpu', result.stderr)
        self.assertEqual(docker_log, '')

    def test_sim_up_rejects_explicit_empty_tail_before_docker(self):
        result, docker_log = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=internal\n',
            command=('sim-up', ''),
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn('optional gpu', result.stderr)
        self.assertEqual(docker_log, '')

    def test_sim_up_uses_bridge_compose_and_starts_foxglove(self):
        result, docker_log = self.run_command(
            'server',
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('compose.foxglove.yaml', docker_log)
        self.assertIn('<foxglove-bridge>', docker_log)

    def test_sim_up_starts_core_observation_without_simulation_adapter(self):
        result, docker_log = self.run_command(
            'dev',
            'SIM_NETWORK_MODE=internal\n',
            command=('sim-up',),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        for service in ('dds-discovery', 'gazebo-server', 'fleet-manager', 'fleet-scene', 'foxglove-bridge'):
            self.assertIn(f'<{service}>', docker_log)
        self.assertNotIn('<sim-adapter>', docker_log)

    def test_sim_adapter_lifecycle_targets_only_optional_adapter_service(self):
        result, up_log = self.run_command(
            'dev', 'SIM_NETWORK_MODE=internal\n', command=('sim-adapter-up',),
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('<up>', up_log)
        self.assertIn('<sim-adapter>', up_log)
        self.assertNotIn('<fleet-manager>', up_log)

        result, down_log = self.run_command(
            'dev', 'SIM_NETWORK_MODE=internal\n', command=('sim-adapter-down',),
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('<kill>\n<-s>\n<SIGTERM>\n<sim-adapter>', down_log)
        self.assertIn('<stop>\n<sim-adapter>', down_log)
        self.assertNotIn('<gazebo-server>', down_log)

    def test_nav_up_requires_an_already_running_simulation_adapter(self):
        result, docker_log = self.run_command(
            'dev', 'SIM_NETWORK_MODE=internal\n', command=('nav-up',),
        )

        self.assertEqual(result.returncode, 3)
        self.assertIn('sim-adapter-up', result.stderr)
        self.assertNotIn('<navigation>', docker_log)

    def test_mapping_up_requires_an_already_running_simulation_adapter(self):
        result, docker_log = self.run_command(
            'server',
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
            {'GIT_COMMIT': 'test-commit'},
            command=('mapping-up', 'warehouse-01'),
        )

        self.assertEqual(result.returncode, 3)
        self.assertIn('sim-adapter-up', result.stderr)
        self.assertNotIn('<slam-mapper>', docker_log)

    def test_mapping_up_does_not_require_a_git_checkout(self):
        result, _ = self.run_command(
            'server',
            'SIM_NETWORK_MODE=lan\nGZ_SERVER_IP=192.168.50.10\n',
            command=('mapping-up', 'warehouse-01'),
        )

        self.assertEqual(result.returncode, 3)
        self.assertIn('sim-adapter-up', result.stderr)
        self.assertNotIn('fatal: not a git repository', result.stderr)

    def test_foxglove_lifecycle_targets_only_the_bridge(self):
        result, docker_log = self.run_command(
            'dev',
            'SIM_NETWORK_MODE=internal\n',
            command=('foxglove-down',),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('<stop>\n<foxglove-bridge>', docker_log)
        self.assertNotIn('<gazebo-server>', docker_log)

    def test_down_removes_orphaned_browser_viewer_containers(self):
        result, docker_log = self.run_command(
            'dev',
            'SIM_NETWORK_MODE=internal\n',
            command=('down',),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('<down>\n<--remove-orphans>', docker_log)

    def test_blank_profile_image_overrides_inherited_image_and_default(self):
        result, docker_log = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=internal\nMENTORPI_IMAGE=\n',
            {'MENTORPI_IMAGE': 'registry.example/inherited:latest'},
            command=('build',),
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('<--tag>\n<>\n', docker_log)
        self.assertNotIn('registry.example/inherited:latest', docker_log)
        self.assertNotIn('mentorpi-sim:harmonic', docker_log)

    def test_fork_up_error_uses_named_profile_sim_up_guidance(self):
        result, _ = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=internal\n',
            command=('fork-up',),
        )
        self.assertEqual(result.returncode, 3)
        self.assertIn('./run.sh --env <profile> sim-up', result.stderr)

    def test_native_gz_server_uses_selected_profile_without_shell_exports(self):
        with TemporaryDirectory() as directory:
            gz_log = Path(directory) / 'gz.log'
            result, docker_log = self.run_command(
                'dev',
                'SIM_NETWORK_MODE=internal\n'
                'GZ_IP=127.0.0.1\n'
                'GZ_PARTITION=mentorpi-native\n'
                'NATIVE_GZ_RESOURCE_PATH=ros2_ws/src/mentorpi_gz_sim/models\n',
                {'GZ_LOG': str(gz_log)},
                command=('gz-server',),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(docker_log, '')
            output = gz_log.read_text()
            self.assertIn('args=sim -s -r ', output)
            self.assertIn('worlds/warehouse.sdf', output)
            self.assertIn('ip=127.0.0.1', output)
            self.assertIn('partition=mentorpi-native', output)
            self.assertIn('ros2_ws/src/mentorpi_gz_sim/models', output)

    def test_native_gz_gui_uses_selected_profile_without_shell_exports(self):
        with TemporaryDirectory() as directory:
            gz_log = Path(directory) / 'gz.log'
            result, docker_log = self.run_command(
                'dev',
                'SIM_NETWORK_MODE=internal\n'
                'GZ_IP=127.0.0.1\n'
                'GZ_PARTITION=mentorpi-native\n'
                'NATIVE_GZ_RESOURCE_PATH=ros2_ws/src/mentorpi_gz_sim/models\n',
                {'GZ_LOG': str(gz_log)},
                command=('gz-gui',),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(docker_log, '')
            self.assertIn('args=sim -g', gz_log.read_text())

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

    def test_selector_after_command_fails_before_docker(self):
        result, docker_log = self.run_command(
            'dev1',
            'SIM_NETWORK_MODE=internal\n',
            selector_args=('sim-up', '--env', 'dev1'),
            command=(),
        )
        self.assertEqual(result.returncode, 2)
        self.assertEqual(docker_log, '')


if __name__ == '__main__':
    unittest.main()
