import json
import os
import subprocess
import textwrap
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


BUNDLE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = BUNDLE.parents[1]
GAZEBO_CMAKE = BUNDLE / 'ros2_ws/src/mentorpi_gz_sim/CMakeLists.txt'


class DeployOnlyBundleTest(unittest.TestCase):
    def test_odom_and_launch_contracts_are_registered_with_ctest(self):
        cmake = GAZEBO_CMAKE.read_text()
        for name, path in (
            ('test_harmonic_launch_contract', 'test/test_harmonic_launch_contract.py'),
            ('test_gz_pose_to_odom', 'test/test_gz_pose_to_odom.py'),
        ):
            self.assertIn(f'ament_add_pytest_test({name}', cmake)
            self.assertIn(path, cmake)

    def test_slam_lifecycle_contracts_are_registered_with_ctest(self):
        slam_cmake = (
            BUNDLE / 'ros2_ws/src/mentorpi_slam/CMakeLists.txt'
        ).read_text()
        for name, path in (
            ('test_slam_contract', 'test/test_slam_contract.py'),
            ('test_session_artifacts', 'test/test_session_artifacts.py'),
            ('test_atomic_publish', 'test/test_atomic_publish.py'),
            ('test_mapping_session_script', 'test/test_mapping_session_script.py'),
        ):
            self.assertIn(f'ament_add_pytest_test({name}', slam_cmake)
            self.assertIn(path, slam_cmake)

    def test_test_command_separates_host_static_and_runtime_ros_checks(self):
        script = (BUNDLE / 'run.sh').read_text()
        test_command = script.split('  test)', 1)[1].split('  fork-up)', 1)[0]

        for static_test in (
            'test/test_bundle.py',
            'test/test_original_model.py',
            'test_harmonic_launch_contract.py',
        ):
            self.assertIn(static_test, test_command)
        self.assertIn('python3 "$BUNDLE_DIR/test/test_bundle.py" -v', test_command)
        self.assertIn('python3 -m unittest discover', test_command)
        self.assertIn("-p 'test_harmonic_launch_contract.py'", test_command)
        self.assertNotIn('test/test_gz_pose_to_odom.py', test_command)
        for runtime_check in (
            'gz sim --versions',
            'ros2 pkg prefix mentorpi_description',
            'ros2 pkg prefix mentorpi_gz_sim',
            'ros2 pkg prefix mentorpi_slam',
            'cd /opt/mentorpi_ws',
            'colcon test --packages-select mentorpi_gz_sim mentorpi_slam',
            'colcon test-result --verbose',
        ):
            self.assertIn(runtime_check, test_command)
        for stage in ('host-static', 'compose-config', 'runtime-ctest'):
            self.assertIn(f'mentorpi test stage={stage}', test_command)

    def test_test_command_validates_base_and_gpu_compose_configs(self):
        script = (BUNDLE / 'run.sh').read_text()
        test_command = script.split('  test)', 1)[1].split('  fork-up)', 1)[0]

        self.assertGreaterEqual(test_command.count('config --quiet'), 2)
        self.assertIn('compose.gpu.yaml', test_command)
        self.assertIn('RENDER_GID="${RENDER_GID:-0}"', test_command)

    def test_sim_adapter_configures_one_way_relay_host(self):
        compose = (BUNDLE / 'compose.yaml').read_text()
        gazebo_server = compose.split('  gazebo-server:', 1)[1].split(
            '  sim-adapter:', 1
        )[0]
        sim_adapter = compose.split('  sim-adapter:', 1)[1].split('\nnetworks:', 1)[0]
        self.assertIn('GZ_RELAY_HOST: gazebo-server', sim_adapter)
        self.assertNotIn('GZ_RELAY_HOST', gazebo_server)
        for forbidden in ('GZ_RELAY:', 'ipv4_address:', 'network_mode: host'):
            self.assertNotIn(forbidden, compose)

    def test_entrypoint_resolves_relay_ipv4_or_fails_fast(self):
        entrypoint = (BUNDLE / 'entrypoint.sh').read_text()
        for required in (
            'if [[ -n "${GZ_RELAY_HOST:-}" ]]',
            'getent ahostsv4',
            'GZ_RELAY_RESOLVE_ATTEMPTS',
            'valid_ipv4',
            'export GZ_RELAY=',
            'relay_target=%s',
            'relay_error=resolve_failed',
            'sleep 1',
            'exit 70',
        ):
            self.assertIn(required, entrypoint)

    def test_dds_environment_helper_resolves_docker_dns_for_every_process(self):
        helper = BUNDLE / 'dds_env.sh'
        self.assertTrue(helper.is_file())
        with TemporaryDirectory() as directory:
            bin_dir = Path(directory)
            fake_getent = bin_dir / 'getent'
            fake_getent.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                printf '10.22.0.5 STREAM dds-discovery\\n'
            '''))
            fake_getent.chmod(0o755)
            environment = os.environ.copy()
            environment.update({
                'PATH': f'{bin_dir}{os.pathsep}{environment["PATH"]}',
                'DDS_DISCOVERY_HOST': 'dds-discovery',
                'DDS_DISCOVERY_PORT': '11811',
            })

            result = subprocess.run(
                [
                    'bash', '-c',
                    'source "$1" && printf "%s" "$ROS_DISCOVERY_SERVER"',
                    'bash', str(helper),
                ],
                text=True,
                capture_output=True,
                env=environment,
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, '10.22.0.5:11811')

    def test_dds_environment_helper_builds_super_client_profile_for_cli(self):
        helper = BUNDLE / 'dds_env.sh'
        with TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / 'bin'
            profile = root / 'super-client.xml'
            bin_dir.mkdir()
            fake_getent = bin_dir / 'getent'
            fake_getent.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                printf '10.22.0.5 STREAM dds-discovery\\n'
            '''))
            fake_getent.chmod(0o755)
            environment = os.environ.copy()
            environment.update({
                'PATH': f'{bin_dir}{os.pathsep}{environment["PATH"]}',
                'DDS_DISCOVERY_HOST': 'dds-discovery',
                'DDS_DISCOVERY_PORT': '11811',
                'DDS_SUPER_CLIENT': '1',
                'DDS_SUPER_CLIENT_PROFILE': str(profile),
            })

            result = subprocess.run(
                [
                    'bash', '-c',
                    'source "$1" && printf "%s\\n%s\\n%s" '
                    '"${ROS_DISCOVERY_SERVER-unset}" '
                    '"$FASTRTPS_DEFAULT_PROFILES_FILE" '
                    '"$FASTDDS_DEFAULT_PROFILES_FILE"',
                    'bash', str(helper),
                ],
                text=True,
                capture_output=True,
                env=environment,
            )

            profile_text = profile.read_text()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            result.stdout.splitlines(),
            ['unset', str(profile), str(profile)],
        )
        for required in (
            'is_default_profile="true"',
            '<discoveryProtocol>SUPER_CLIENT</discoveryProtocol>',
            'prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41"',
            '<address>10.22.0.5</address>',
            '<port>11811</port>',
        ):
            self.assertIn(required, profile_text)

    def test_healthchecks_require_payload_progression_for_both_robots(self):
        compose = (BUNDLE / 'compose.yaml').read_text()
        health_path = BUNDLE / 'healthcheck.sh'
        self.assertTrue(health_path.is_file(), 'healthcheck.sh')
        health = health_path.read_text()
        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        self.assertIn('set -o pipefail', health)
        self.assertNotIn('set -uo pipefail', health)
        self.assertIn(
            'COPY healthcheck.sh /usr/local/bin/mentorpi-healthcheck',
            dockerfile,
        )

        gazebo_server = compose.split('  gazebo-server:', 1)[1].split(
            '  sim-adapter:', 1
        )[0]
        sim_adapter = compose.split('  sim-adapter:', 1)[1].split('\nnetworks:', 1)[0]
        self.assertIn('mentorpi-healthcheck', gazebo_server)
        self.assertIn('server', gazebo_server)
        self.assertIn('mentorpi-healthcheck', sim_adapter)
        self.assertIn('adapter', sim_adapter)
        for section in (gazebo_server, sim_adapter):
            self.assertIn('start_period:', section)
        for required in (
            'source /opt/ros/humble/setup.bash',
            'source /opt/mentorpi_ws/install/setup.bash',
            '/world/mentorpi_warehouse/stats',
            'gz topic',
            '-n 2',
            'iterations:',
            'check_ros_payload robot_1 scan_raw',
            'check_ros_payload robot_1 odom',
            'check_ros_payload robot_2 scan_raw',
            'check_ros_payload robot_2 odom',
            'sensor_msgs/msg/LaserScan',
            'nav_msgs/msg/Odometry',
            'rosgraph_msgs/msg/Clock',
            'check_clock_progress',
            'tf2_echo',
            'check_robot_tf robot_1',
            'check_robot_tf robot_2',
            'local parent="${robot}/odom"',
            'local child="${robot}/base_footprint"',
            'timeout',
        ):
            self.assertIn(required, health)
        self.assertNotIn('ros2 topic list', health)

    def test_runtime_image_contains_fastdds_discovery_cli(self):
        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        self.assertIn('fastdds-tools', dockerfile)

    def test_gpu_profile_requires_readable_render_gid_preflight(self):
        gpu_compose = (BUNDLE / 'compose.gpu.yaml').read_text()
        script = (BUNDLE / 'run.sh').read_text()
        self.assertIn('group_add:', gpu_compose)
        self.assertIn('${RENDER_GID', gpu_compose)
        for required in (
            "uname -s",
            '/dev/dri/renderD',
            '-r "$render_node"',
            "stat -c '%g'",
            'export RENDER_GID',
            'Linux',
        ):
            self.assertIn(required, script)
        for forbidden in ('privileged:', 'chmod 666', 'network_mode: host'):
            self.assertNotIn(forbidden, gpu_compose + script)

    def test_fork_up_uses_healthy_running_adapter(self):
        script = (BUNDLE / 'run.sh').read_text()
        fork_up = script.split('  fork-up)', 1)[1].split('  mapping-up)', 1)[0]
        for required in (
            'ps -q sim-adapter',
            'State.Health.Status',
            'healthy',
            'exec -T sim-adapter bash -lc',
            'timeout',
            'ros2 topic pub --once',
        ):
            self.assertIn(required, fork_up)
        self.assertNotIn(' run --rm', fork_up)

    def test_topic_diagnostic_sources_ros_environment(self):
        script = (BUNDLE / 'run.sh').read_text()
        for required in (
            'topics',
            'DDS_SUPER_CLIENT=1',
            'source /opt/ros/humble/setup.bash',
            'source /opt/mentorpi_ws/install/setup.bash',
            'exec sim-adapter bash -lc',
            'ros2 topic list --no-daemon',
        ):
            self.assertIn(required, script)

    def test_mapping_profile_uses_one_shot_mapper_and_persistent_slam_volume(self):
        compose = (BUNDLE / 'compose.yaml').read_text()
        self.assertIn('  slam-mapper:', compose)
        mapper = compose.split('  slam-mapper:', 1)[1].split('\nnetworks:', 1)[0]

        for required in (
            'profiles: [mapping]',
            'restart: "no"',
            'SLAM_DATA_ROOT: /slam-data',
            'slam-data:/slam-data',
            'condition: service_healthy',
            '/opt/mentorpi_ws/install/mentorpi_slam/lib/mentorpi_slam/run_mapping_session.sh',
            'GIT_COMMIT: "${GIT_COMMIT:-unknown}"',
            'WORLD_VERSION: "${WORLD_VERSION:-unknown}"',
            'MODEL_VERSION: "${MODEL_VERSION:-unknown}"',
            'TF_CALIBRATION_VERSION: "${TF_CALIBRATION_VERSION:-unknown}"',
        ):
            self.assertIn(required, mapper)
        self.assertNotIn('GZ_RELAY_HOST', mapper)
        self.assertIn('slam-data:', compose)
        self.assertIn('name: mentorpi-slam-data', compose)

    def test_ros_services_use_stable_discovery_and_udp_without_namespace_sharing(self):
        result = subprocess.run(
            [
                'docker',
                'compose',
                '-f',
                str(BUNDLE / 'compose.yaml'),
                '--profile',
                'mapping',
                'config',
                '--format',
                'json',
            ],
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        services = json.loads(result.stdout)['services']
        discovery = services['dds-discovery']
        gazebo = services['gazebo-server']
        adapter = services['sim-adapter']
        mapper = services['slam-mapper']

        self.assertEqual(
            discovery['command'],
            [
                'fastdds', 'discovery', '-i', '0',
                '-l', '0.0.0.0', '-p', '11811',
            ],
        )
        self.assertNotIn('ports', discovery)
        self.assertEqual(discovery.get('restart'), 'unless-stopped')
        for service in (adapter, mapper):
            self.assertIn('mentorpi', service.get('networks', {}))
            self.assertNotIn('ipc', service)
            self.assertNotIn('network_mode', service)
            self.assertEqual(
                service['environment']['DDS_DISCOVERY_HOST'],
                'dds-discovery',
            )
            self.assertEqual(service['environment']['DDS_DISCOVERY_PORT'], '11811')
            self.assertNotIn('ROS_DISCOVERY_SERVER', service['environment'])
            self.assertEqual(
                service['environment']['FASTDDS_BUILTIN_TRANSPORTS'],
                'UDPv4',
            )
        for non_client in (discovery, gazebo, services['slam-data-init'],
                           services['slam-inspector']):
            self.assertNotIn('DDS_DISCOVERY_HOST', non_client['environment'])
            self.assertNotIn('ROS_DISCOVERY_SERVER', non_client['environment'])

        self.assertEqual(adapter.get('restart'), 'unless-stopped')
        self.assertEqual(mapper.get('restart'), 'no')
        self.assertEqual(mapper.get('stop_grace_period'), '30s')

    def test_mapping_volume_initializer_only_owns_volume_root_before_mapper(self):
        compose = (BUNDLE / 'compose.yaml').read_text()
        self.assertIn('  slam-data-init:', compose)
        initializer = compose.split('  slam-data-init:', 1)[1].split(
            '  slam-mapper:', 1
        )[0]
        mapper = compose.split('  slam-mapper:', 1)[1].split(
            '  slam-inspector:', 1
        )[0]

        for required in (
            'profiles: [mapping]',
            'restart: "no"',
            'user: "0:0"',
            'chown 1000:1000 /slam-data',
            'slam-data:/slam-data',
        ):
            self.assertIn(required, initializer)
        self.assertNotIn('chown -R', initializer)
        self.assertNotIn('chmod -R', initializer)
        self.assertIn('slam-data-init:', mapper)
        self.assertIn('condition: service_completed_successfully', mapper)
        self.assertIn('sim-adapter:', mapper)
        self.assertIn('condition: service_healthy', mapper)

    def test_mapper_execs_installed_lifecycle_script_as_pid_one(self):
        compose = (BUNDLE / 'compose.yaml').read_text()
        mapper = compose.split('  slam-mapper:', 1)[1].split(
            '  slam-inspector:', 1
        )[0]
        entrypoint = (BUNDLE / 'entrypoint.sh').read_text()

        self.assertIn(
            'command: /opt/mentorpi_ws/install/mentorpi_slam/lib/mentorpi_slam/run_mapping_session.sh',
            mapper,
        )
        self.assertNotIn('ros2 run mentorpi_slam run_mapping_session.sh', mapper)
        for required in (
            'source /opt/ros/humble/setup.bash',
            'source /opt/mentorpi_ws/install/setup.bash',
            'exec "$@"',
        ):
            self.assertIn(required, entrypoint)

    def test_mapping_commands_validate_session_and_preserve_finalization(self):
        script = (BUNDLE / 'run.sh').read_text()
        self.assertIn('  mapping-up)', script)
        self.assertIn('  mapping-stop)', script)
        self.assertIn('  mapping-status)', script)
        mapping_up = script.split('  mapping-up)', 1)[1].split('  mapping-stop)', 1)[0]
        mapping_stop = script.split('  mapping-stop)', 1)[1].split(
            '  mapping-status)', 1
        )[0]
        mapping_status = script.split('  mapping-status)', 1)[1].split('  -h|', 1)[0]

        for required in (
            'validate_session_id',
            '[[ "$#" -ne 2 ]]',
            'export SESSION_ID="$2"',
            'export IMAGE_VERSION="${IMAGE_VERSION:-mentorpi-sim:harmonic}"',
            'export GIT_COMMIT="${GIT_COMMIT:-$(git -C "$BUNDLE_DIR" rev-parse HEAD)}"',
            'export WORLD_VERSION="${WORLD_VERSION:-warehouse-v1}"',
            'export MODEL_VERSION="${MODEL_VERSION:-mentorpi-m1-v1}"',
            'export TF_CALIBRATION_VERSION="${TF_CALIBRATION_VERSION:-ground-truth-v1}"',
            '--profile mapping up -d',
            'gazebo-server sim-adapter slam-mapper',
        ):
            self.assertIn(required, mapping_up)

        for required in (
            'kill -s SIGINT slam-mapper',
            'MAPPING_STOP_TIMEOUT_SECONDS',
            'stop gazebo-server sim-adapter',
            'mapping finalization succeeded',
            'mapping finalization failed',
        ):
            self.assertIn(required, mapping_stop)
        self.assertIn('State.Running', script)
        self.assertIn('State.ExitCode', script)

        for required in (
            'validate_session_id "$2"',
            '--profile mapping run --rm --no-deps slam-inspector',
            'session_dir="/slam-data/${SESSION_ID}"',
            'checksums.sha256',
            'sha256sum -c checksums.sha256',
        ):
            self.assertIn(required, mapping_status)

    def test_mapping_stop_cleans_support_after_mapper_already_exited(self):
        result, operations = self.run_mapping_stop_with_fake_docker(
            mapper_state='false 0'
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('mapping finalization succeeded', result.stdout)
        self.assertEqual(operations, ['stop'])

    def test_mapping_stop_timeout_leaves_mapper_and_support_running(self):
        result, operations = self.run_mapping_stop_with_fake_docker(
            mapper_state='true 0', sigint_state='true 0', timeout='0'
        )

        self.assertEqual(result.returncode, 5, result.stderr)
        self.assertIn('finalization status is unknown', result.stderr)
        self.assertEqual(operations, ['sigint'])

    def test_mapping_stop_sigint_after_current_adapter_is_healthy(self):
        result, operations = self.run_mapping_stop_with_fake_docker(
            mapper_state='true 0',
            sigint_state='false 0',
            adapter_health='healthy',
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('mapping finalization succeeded', result.stdout)
        self.assertEqual(operations, ['sigint', 'stop'])

    def test_mapping_stop_waits_for_recreated_adapter_to_recover(self):
        result, operations = self.run_mapping_stop_with_fake_docker(
            mapper_state='true 0',
            sigint_state='false 0',
            adapter_health='starting,unhealthy,healthy',
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('adapter_state=reconnecting', result.stdout)
        self.assertIn('adapter_state=recovered', result.stdout)
        self.assertEqual(operations, ['sigint', 'stop'])

    def test_mapping_stop_recovery_timeout_keeps_mapping_services_running(self):
        result, operations = self.run_mapping_stop_with_fake_docker(
            mapper_state='true 0',
            adapter_health='unhealthy',
            timeout='0',
        )

        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertIn('adapter recovery timed out', result.stderr)
        self.assertEqual(operations, [])

    def test_mapping_commands_reject_dot_session_ids_before_docker(self):
        for command in ('mapping-up', 'mapping-status'):
            for session_id in ('.', '..'):
                with self.subTest(command=command, session_id=session_id):
                    result = subprocess.run(
                        ['bash', str(BUNDLE / 'run.sh'), command, session_id],
                        text=True,
                        capture_output=True,
                    )
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn('session ID may contain', result.stderr)

    def run_mapping_stop_with_fake_docker(
        self,
        mapper_state,
        sigint_state=None,
        sigterm_state=None,
        sigkill_state=None,
        adapter_health='healthy',
        timeout='5',
    ):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            state_path = root / 'mapper-state'
            log_path = root / 'operations'
            health_index_path = root / 'health-index'
            state_path.write_text(mapper_state)
            health_index_path.write_text('0')
            docker = bin_dir / 'docker'
            docker.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                set -euo pipefail
                if [[ "$1" == inspect ]]; then
                  format="$3"
                  container_id="$4"
                  if [[ "$container_id" == adapter-current ]]; then
                    IFS=',' read -r -a health_values <<< "$FAKE_DOCKER_ADAPTER_HEALTH"
                    health_index="$(cat "$FAKE_DOCKER_HEALTH_INDEX_PATH")"
                    last_index=$((${#health_values[@]} - 1))
                    if ((health_index > last_index)); then health_index=$last_index; fi
                    printf 'true %s\\n' "${health_values[$health_index]}"
                    printf '%s' "$((health_index + 1))" > "$FAKE_DOCKER_HEALTH_INDEX_PATH"
                  else
                    cat "$FAKE_DOCKER_STATE_PATH"
                  fi
                  exit 0
                fi
                [[ "$1" == compose ]] || exit 99
                case " $* " in
                  *' ps -q --all slam-mapper '*)
                    printf '%s\\n' mapper-test
                    ;;
                  *' ps -q sim-adapter '*)
                    printf '%s\\n' adapter-current
                    ;;
                  *' kill -s SIGINT slam-mapper '*)
                    printf '%s\\n' sigint >> "$FAKE_DOCKER_LOG_PATH"
                    if [[ -n "$FAKE_DOCKER_SIGINT_STATE" ]]; then
                      printf '%s' "$FAKE_DOCKER_SIGINT_STATE" > "$FAKE_DOCKER_STATE_PATH"
                    fi
                    ;;
                  *' kill -s SIGTERM slam-mapper '*)
                    printf '%s\\n' sigterm >> "$FAKE_DOCKER_LOG_PATH"
                    if [[ -n "$FAKE_DOCKER_SIGTERM_STATE" ]]; then
                      printf '%s' "$FAKE_DOCKER_SIGTERM_STATE" > "$FAKE_DOCKER_STATE_PATH"
                    fi
                    ;;
                  *' kill -s SIGKILL slam-mapper '*)
                    printf '%s\\n' sigkill >> "$FAKE_DOCKER_LOG_PATH"
                    if [[ -n "$FAKE_DOCKER_SIGKILL_STATE" ]]; then
                      printf '%s' "$FAKE_DOCKER_SIGKILL_STATE" > "$FAKE_DOCKER_STATE_PATH"
                    fi
                    ;;
                  *' stop gazebo-server sim-adapter dds-discovery '*)
                    printf '%s\\n' stop >> "$FAKE_DOCKER_LOG_PATH"
                    ;;
                  *) exit 98 ;;
                esac
            '''))
            docker.chmod(0o755)
            environment = os.environ.copy()
            environment.update({
                'PATH': f'{bin_dir}{os.pathsep}{environment["PATH"]}',
                'FAKE_DOCKER_STATE_PATH': str(state_path),
                'FAKE_DOCKER_LOG_PATH': str(log_path),
                'FAKE_DOCKER_HEALTH_INDEX_PATH': str(health_index_path),
                'FAKE_DOCKER_ADAPTER_HEALTH': adapter_health,
                'FAKE_DOCKER_SIGINT_STATE': sigint_state or '',
                'FAKE_DOCKER_SIGTERM_STATE': sigterm_state or '',
                'FAKE_DOCKER_SIGKILL_STATE': sigkill_state or '',
                'MAPPING_STOP_TIMEOUT_SECONDS': timeout,
                'MAPPING_RECONNECT_TIMEOUT_SECONDS': timeout,
            })
            result = subprocess.run(
                ['bash', str(BUNDLE / 'run.sh'), 'mapping-stop'],
                text=True,
                capture_output=True,
                env=environment,
            )
            operations = log_path.read_text().splitlines() if log_path.exists() else []
            return result, operations

    def test_bundle_contains_all_runtime_assets(self):
        for relative_path in (
            'Dockerfile',
            'compose.yaml',
            'compose.gpu.yaml',
            'entrypoint.sh',
            'healthcheck.sh',
            'run.sh',
            'README.md',
            'ros2_ws/src/mentorpi_description/package.xml',
            'ros2_ws/src/mentorpi_gz_sim/package.xml',
        ):
            self.assertTrue((BUNDLE / relative_path).is_file(), relative_path)

        compose = (BUNDLE / 'compose.yaml').read_text()
        for service in ('gazebo-server:', 'sim-adapter:'):
            self.assertIn(service, compose)
        for required in (
            'GZ_PARTITION: mentorpi-sim',
            'condition: service_healthy',
            'LIBGL_ALWAYS_SOFTWARE',
            'ros2 launch mentorpi_gz_sim gazebo_server.launch.py',
            'ros2 launch mentorpi_gz_sim sim_adapter.launch.py',
            'mentorpi:',
        ):
            self.assertIn(required, compose)
        for removed in (
            'mentorpi-gui:',
            'DISPLAY:',
            'XAUTHORITY:',
            'VirtualGL',
            '/dev/dri/renderD128:/dev/dri/renderD128',
            'network_mode: host',
            './ros2_ws:/ws',
        ):
            self.assertNotIn(removed, compose)
        for forbidden_port in ('10317:', '10318:', '9002:'):
            self.assertNotIn(forbidden_port, compose)
        self.assertNotIn('build:', compose)
        self.assertIn('image: "${MENTORPI_IMAGE:-mentorpi-sim:harmonic}"', compose)
        self.assertIn(
            'IMAGE_VERSION: "${MENTORPI_IMAGE:-mentorpi-sim:harmonic}"',
            compose,
        )

        gpu_compose = (BUNDLE / 'compose.gpu.yaml').read_text()
        self.assertIn('/dev/dri:/dev/dri', gpu_compose)
        self.assertIn('LIBGL_ALWAYS_SOFTWARE: "0"', gpu_compose)

        script = (BUNDLE / 'run.sh').read_text()
        for command in ('build', 'sim-up', 'down', 'logs', 'test', 'fork-up'):
            self.assertIn(command, script)
        self.assertIn('up -d dds-discovery gazebo-server sim-adapter', script)
        self.assertIn('MENTORPI_IMAGE', script)
        self.assertIn('docker build --platform', script)
        self.assertNotIn('"${COMPOSE[@]}" build', script)
        for removed in ('ssh -Y', 'vglrun', 'DISPLAY'):
            self.assertNotIn(removed, script)

        entrypoint = (BUNDLE / 'entrypoint.sh').read_text()
        for required in ('SERVICE_NAME', 'IMAGE_VERSION', 'SESSION_ID', 'ROBOT_IDS'):
            self.assertIn(required, entrypoint)

    def test_runtime_image_uses_humble_with_harmonic(self):
        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        self.assertIn('FROM ros:humble-ros-base-jammy AS runtime', dockerfile)
        self.assertNotIn('humble-desktop-full', dockerfile)
        self.assertIn('https://packages.osrfoundation.org/gazebo.gpg', dockerfile)
        self.assertIn('gz-harmonic', dockerfile)
        for required in (
            'ros-humble-robot-state-publisher',
            'ros-humble-ros-gzharmonic',
            'ros-humble-tf2-ros',
            'ros-humble-xacro',
        ):
            self.assertIn(required, dockerfile)
        for removed in ('ros-humble-ros-gz \\', 'VirtualGL', 'x11-apps', 'xauth', 'dbus-x11'):
            self.assertNotIn(removed, dockerfile)
        self.assertFalse((BUNDLE / 'vendor/virtualgl_3.1.4_amd64.deb').exists())

    def test_repository_has_no_duplicate_root_runtime_layout(self):
        for legacy_path in ('docker', 'ros2_ws', 'compose.yaml', 'test'):
            self.assertFalse((REPOSITORY_ROOT / legacy_path).exists(), legacy_path)

    def test_operator_docs_describe_in_place_bundle_changes(self):
        readme = (BUNDLE / 'README.md').read_text()
        for removed in ('ssh -Y', 'XAUTHORITY', 'VirtualGL', 'XQuartz'):
            self.assertNotIn(removed, readme)
        for text in (
            'linux/amd64',
            './run.sh build',
            './run.sh sim-up',
            './run.sh logs',
            './run.sh down',
            './run.sh fork-up',
            './run.sh topics',
            'MENTORPI_IMAGE',
            'sha256:',
            '브라우저',
            '오프스크린',
            'native Ubuntu',
            'release gate',
            './run.sh mapping-up',
            './run.sh mapping-stop',
            './run.sh mapping-status',
            '.inprogress',
            'mentorpi-slam-data',
            'mentorpi-slam-data:/slam-data:ro',
            '첫 mapping-up',
            '기존 세션 내용을 변경하지 않는다',
        ):
            self.assertIn(text, readme)


if __name__ == '__main__':
    unittest.main()
