import json
import os
import shutil
import subprocess
import textwrap
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml


BUNDLE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = BUNDLE.parents[1]
GAZEBO_CMAKE = BUNDLE / 'ros2_ws/src/mentorpi_gz_sim/CMakeLists.txt'
OBSERVATION_BUNDLE = BUNDLE / 'dds-observation'
OBSERVATION_COMPOSE = OBSERVATION_BUNDLE / 'docker-compose.yaml'
ROSBAG_RECORDER = OBSERVATION_BUNDLE / 'rosbag-recorder/rosbag_recorder.sh'
ROSBAG_BOOTSTRAP = OBSERVATION_BUNDLE / 'rosbag-recorder/rosbag_recorder_bootstrap.sh'


class DeployOnlyBundleTest(unittest.TestCase):
    def test_physical_observation_map_server_mounts_the_live_package_and_map_data(self):
        """Map updates must be deployed from host mounts without rebuilding Foxglove."""
        with TemporaryDirectory() as directory:
            map_directory = Path(directory)
            environment = os.environ.copy()
            environment.update({
                'DDS_OBSERVATION_IMAGE': 'mentorpi-dds-observation:test',
                'MAP_DIRECTORY': str(map_directory),
                'MAP_SERVER_OVERLAY_VOLUME': 'mentorpi-map-server-overlay-test',
            })
            result = subprocess.run(
                [
                    'docker', 'compose',
                    '--env-file', str(OBSERVATION_BUNDLE / '.env.example'),
                    '-f', str(OBSERVATION_COMPOSE),
                    'config', '--format', 'json',
                ],
                text=True,
                capture_output=True,
                env=environment,
                check=False,
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        services = json.loads(result.stdout)['services']
        self.assertIn('map-server', services)
        map_server = services['map-server']
        self.assertEqual(map_server['network_mode'], 'host')
        self.assertEqual(map_server['user'], '0:0')
        self.assertEqual(map_server['environment']['ROS_DOMAIN_ID'], '225')
        self.assertEqual(map_server['environment']['MAP_YAML'], '/maps/map.yaml')
        self.assertEqual(
            map_server['command'],
            ['/usr/local/bin/mentorpi-map-server-bootstrap'],
        )

        mounts = {mount['target']: mount for mount in map_server['volumes']}
        self.assertEqual(mounts['/ws']['type'], 'volume')
        self.assertEqual(mounts['/ws/src/mentorpi_map_server']['type'], 'bind')
        self.assertTrue(mounts['/ws/src/mentorpi_map_server']['read_only'])
        self.assertEqual(mounts['/maps']['type'], 'bind')
        self.assertEqual(mounts['/maps']['source'], str(map_directory))
        self.assertTrue(mounts['/maps']['read_only'])

    def test_physical_observation_exposes_foxglove_on_all_interfaces_with_configured_port(self):
        """A trusted LAN client must be able to use the configured Foxglove port."""
        environment = os.environ.copy()
        environment.update({
            'DDS_OBSERVATION_IMAGE': 'mentorpi-dds-observation:test',
            'FOXGLOVE_PORT': '9234',
        })
        result = subprocess.run(
            [
                'docker', 'compose',
                '--env-file', str(OBSERVATION_BUNDLE / '.env.example'),
                '-f', str(OBSERVATION_COMPOSE),
                'config', '--format', 'json',
            ],
            text=True,
            capture_output=True,
            env=environment,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        bridge = json.loads(result.stdout)['services']['foxglove-bridge']
        self.assertEqual(
            bridge['command'],
            [
                'ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml',
                'address:=0.0.0.0', 'port:=9234',
            ],
        )

    def test_physical_observation_compose_uses_static_vehicle_bridges(self):
        """Each physical vehicle must have one isolated bridge into Domain 225."""
        self.assertTrue((OBSERVATION_BUNDLE / 'Dockerfile').is_file())
        self.assertTrue((OBSERVATION_BUNDLE / 'README.md').is_file())
        result = subprocess.run(
            [
                'docker', 'compose',
                '--env-file', str(OBSERVATION_BUNDLE / '.env.example'),
                '-f', str(OBSERVATION_COMPOSE),
                'config', '--format', 'json',
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        services = json.loads(result.stdout)['services']

        self.assertEqual(
            set(services),
            {
                'bridge-robot-1', 'bridge-robot-2',
                'foxglove-bridge', 'map-server', 'rosbag-recorder',
            },
        )

        for name, domain, prefix in (
            ('bridge-robot-1', '215', '/robot_1'),
            ('bridge-robot-2', '216', '/robot_2'),
        ):
            with self.subTest(bridge=name):
                bridge = services[name]
                self.assertEqual(bridge['image'], 'mentorpi-domain-bridge:humble')
                self.assertEqual(bridge['network_mode'], 'host')
                self.assertEqual(bridge['environment'], {
                    'CENTRAL_PREFIX': prefix,
                    'CONTROL_DOMAIN': '225',
                    'SOURCE_NAMESPACE': '/',
                    'VEHICLE_DOMAIN': domain,
                })

        for name in ('foxglove-bridge', 'map-server', 'rosbag-recorder'):
            with self.subTest(service=name):
                service = services[name]
                self.assertEqual(service['network_mode'], 'host')
                self.assertEqual(service['environment']['ROS_DOMAIN_ID'], '225')
                self.assertNotIn('DDS_DISCOVERY_HOST', service['environment'])
                self.assertNotIn('DDS_DISCOVERY_PORT', service['environment'])
                self.assertNotIn('DDS_SUPER_CLIENT', service['environment'])

        for name in ('foxglove-bridge', 'map-server', 'rosbag-recorder'):
            with self.subTest(dependent_service=name):
                self.assertEqual(
                    services[name]['depends_on']['bridge-robot-1']['condition'],
                    'service_started',
                )
                self.assertEqual(
                    services[name]['depends_on']['bridge-robot-2']['condition'],
                    'service_started',
                )

        self.assertEqual(
            services['rosbag-recorder']['command'],
            ['/usr/local/bin/mentorpi-rosbag-recorder-bootstrap'],
        )
        self.assertEqual(services['rosbag-recorder']['user'], '0:0')
        self.assertTrue(ROSBAG_BOOTSTRAP.is_file())
        bootstrap = ROSBAG_BOOTSTRAP.read_text()
        self.assertIn('chown ros:ros "$rosbag_root"', bootstrap)
        self.assertIn('runuser -u ros --preserve-environment --', bootstrap)
        self.assertFalse((BUNDLE / 'compose.observation.yaml').exists())
        self.assertFalse((BUNDLE / 'rosbag_recorder.sh').exists())

    def test_rosbag_recorder_creates_a_session_with_both_vehicle_telemetry_topics(self):
        """A recorder restart must create an independent bag for the two live vehicles."""
        with TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            invocation = root / 'ros2-invocation'
            fake_ros2 = bin_dir / 'ros2'
            fake_ros2.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                printf '%s\\n' "$@" > "$ROSBAG_INVOCATION"
            '''))
            fake_ros2.chmod(0o755)
            environment = os.environ.copy()
            environment.update({
                'PATH': f'{bin_dir}{os.pathsep}{environment["PATH"]}',
                'ROSBAG_INVOCATION': str(invocation),
                'ROSBAG_ROOT': str(root / 'bags'),
                'ROSBAG_SESSION_ID': 'live-20260816-01',
            })

            result = subprocess.run(
                ['bash', str(ROSBAG_RECORDER)],
                text=True,
                capture_output=True,
                env=environment,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(
                invocation.read_text().splitlines(),
                [
                    'bag', 'record', '--output', str(root / 'bags/live-20260816-01'),
                    '/robot_1/tf', '/robot_1/tf_static',
                    '/robot_2/tf', '/robot_2/tf_static',
                    '/robot_1/fleet/status', '/robot_2/fleet/status',
                    '/controller_server/map',
                    '/robot_1/odom', '/robot_1/scan_raw', '/robot_1/imu/data_raw',
                    '/robot_1/depth/image_raw', '/robot_1/depth/camera_info',
                    '/robot_1/cmd_vel_nav', '/robot_1/controller/cmd_vel',
                    '/robot_1/navigation/status',
                    '/robot_2/odom', '/robot_2/scan_raw', '/robot_2/imu/data_raw',
                    '/robot_2/depth/image_raw', '/robot_2/depth/camera_info',
                    '/robot_2/cmd_vel_nav', '/robot_2/controller/cmd_vel',
                    '/robot_2/navigation/status',
                ],
            )

    def test_rosbag_recorder_suffixes_an_automatic_session_that_already_exists(self):
        """A quick recorder restart must retain the previous bag instead of overwriting it."""
        with TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            bag_root = root / 'bags'
            (bag_root / '20260816T120000Z').mkdir(parents=True)
            invocation = root / 'ros2-invocation'
            fake_date = bin_dir / 'date'
            fake_date.write_text('#!/usr/bin/env bash\nprintf "20260816T120000Z\\n"\n')
            fake_date.chmod(0o755)
            fake_ros2 = bin_dir / 'ros2'
            fake_ros2.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                printf '%s\\n' "$@" > "$ROSBAG_INVOCATION"
            '''))
            fake_ros2.chmod(0o755)
            environment = os.environ.copy()
            environment.pop('ROSBAG_SESSION_ID', None)
            environment.update({
                'PATH': f'{bin_dir}{os.pathsep}{environment["PATH"]}',
                'ROSBAG_INVOCATION': str(invocation),
                'ROSBAG_ROOT': str(bag_root),
            })

            result = subprocess.run(
                ['bash', str(ROSBAG_RECORDER)],
                text=True,
                capture_output=True,
                env=environment,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(str(bag_root / '20260816T120000Z-01'), invocation.read_text())

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
            'colcon test --packages-select mentorpi_gz_sim mentorpi_foxglove_scene mentorpi_fleet mentorpi_slam',
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
        self.assertIn('RENDER_GID="${RENDER_GID-0}"', test_command)

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

    def test_gazebo_services_bind_runtime_sdf_assets_read_only(self):
        compose = yaml.safe_load((BUNDLE / 'compose.yaml').read_text())
        expected_mounts = [
            {
                'type': 'bind',
                'source': './ros2_ws/src/mentorpi_gz_sim/worlds',
                'target': (
                    '/opt/mentorpi_ws/install/mentorpi_gz_sim/share/'
                    'mentorpi_gz_sim/worlds'
                ),
                'read_only': True,
            },
            {
                'type': 'bind',
                'source': './ros2_ws/src/mentorpi_gz_sim/models',
                'target': (
                    '/opt/mentorpi_ws/install/mentorpi_gz_sim/share/'
                    'mentorpi_gz_sim/models'
                ),
                'read_only': True,
            },
        ]

        self.assertEqual(compose['services']['gazebo-server'].get('volumes', []), expected_mounts)
        adapter_mounts = compose['services']['sim-adapter'].get('volumes', [])
        for expected in expected_mounts:
            self.assertIn(expected, adapter_mounts)
        self.assertIn({
            'type': 'bind',
            'source': './ros2_ws/src/mentorpi_fleet/config',
            'target': '/etc/mentorpi-fleet',
            'read_only': True,
        }, adapter_mounts)

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

    def test_healthchecks_separate_shared_world_and_optional_adapter_liveness(self):
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
            '/world/mentorpi_warehouse/stats',
            'gz topic',
            '-n 2',
            'iterations:',
            'check_adapter_manager',
            'simulation_manager.py',
            'pgrep',
        ):
            self.assertIn(required, health)
        self.assertNotIn('check_ros_payload robot_1', health)

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
            'exec fleet-manager bash -lc',
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
            'GIT_COMMIT: "${GIT_COMMIT-unknown}"',
            'WORLD_VERSION: "${WORLD_VERSION-unknown}"',
            'MODEL_VERSION: "${MODEL_VERSION-unknown}"',
            'TF_CALIBRATION_VERSION: "${TF_CALIBRATION_VERSION-unknown}"',
        ):
            self.assertIn(required, mapper)
        self.assertNotIn('GZ_RELAY_HOST', mapper)
        self.assertIn('slam-data:', compose)
        self.assertIn(
            'name: "${SLAM_VOLUME_NAME-mentorpi-slam-data}"',
            compose,
        )

    def test_mapping_session_does_not_change_foxglove_simulation_services(self):
        base_files = ['-f', str(BUNDLE / 'compose.yaml')]
        mapping_env = os.environ.copy()
        mapping_env.update({
            'SESSION_ID': 'active-mapping-session',
            'SLAM_VOLUME_NAME': 'mentorpi-test-isolated-slam-data',
        })
        foxglove_env = mapping_env.copy()
        foxglove_env.pop('SESSION_ID')

        mapping_result = subprocess.run(
            [
                'docker', 'compose', *base_files, '--profile', 'mapping',
                'config', '--format', 'json',
            ],
            env=mapping_env,
            text=True,
            capture_output=True,
            check=False,
        )
        foxglove_result = subprocess.run(
            [
                'docker', 'compose', *base_files,
                '-f', str(BUNDLE / 'compose.foxglove.yaml'),
                'config', '--format', 'json',
            ],
            env=foxglove_env,
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertEqual(mapping_result.returncode, 0, mapping_result.stderr)
        self.assertEqual(foxglove_result.returncode, 0, foxglove_result.stderr)
        mapping = json.loads(mapping_result.stdout)
        foxglove = json.loads(foxglove_result.stdout)
        for service in ('dds-discovery', 'gazebo-server', 'sim-adapter'):
            self.assertEqual(
                mapping['services'][service],
                foxglove['services'][service],
            )
            self.assertNotIn(
                'SESSION_ID',
                mapping['services'][service]['environment'],
            )
        self.assertNotIn(
            'SESSION_ID',
            mapping['services']['slam-data-init']['environment'],
        )
        self.assertNotIn('SESSION_ID', foxglove['services']['foxglove-bridge']['environment'])
        for service in ('slam-mapper', 'slam-inspector'):
            self.assertEqual(
                mapping['services'][service]['environment']['SESSION_ID'],
                'active-mapping-session',
            )
        self.assertEqual(
            mapping['volumes']['slam-data']['name'],
            'mentorpi-test-isolated-slam-data',
        )

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
            '[[ "${#RUN_COMMAND[@]}" -ne 2 ]]',
            'export SESSION_ID="${RUN_COMMAND[1]}"',
            'export IMAGE_VERSION="${IMAGE_VERSION-mentorpi-sim:harmonic}"',
            'export GIT_COMMIT="${GIT_COMMIT-$(git -C "$BUNDLE_DIR" rev-parse HEAD 2>/dev/null || printf \'unknown\')}"',
            'export WORLD_VERSION="${WORLD_VERSION-warehouse-v1}"',
            'export MODEL_VERSION="${MODEL_VERSION-mentorpi-m1-v1}"',
            'export TF_CALIBRATION_VERSION="${TF_CALIBRATION_VERSION-ground-truth-v1}"',
            '--profile mapping up -d',
            'require_healthy_sim_adapter',
            'up -d slam-mapper',
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
            'validate_session_id "${RUN_COMMAND[1]}"',
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
        with TemporaryDirectory() as directory:
            root = Path(directory)
            launcher = self.copy_test_launcher(root)
            for command in ('mapping-up', 'mapping-status'):
                for session_id in ('.', '..'):
                    with self.subTest(command=command, session_id=session_id):
                        result = subprocess.run(
                            [
                                'bash', str(launcher), '--env', 'test', command,
                                session_id,
                            ],
                            text=True,
                            capture_output=True,
                        )
                        self.assertEqual(result.returncode, 2, result.stderr)
                        self.assertIn('session ID may contain', result.stderr)

    def copy_test_launcher(self, root):
        launcher = root / 'run.sh'
        shutil.copy2(BUNDLE / 'run.sh', launcher)
        (root / '.env.test').write_text('SIM_NETWORK_MODE=internal\n')
        return launcher

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
            launcher = self.copy_test_launcher(root)
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
                ['bash', str(launcher), '--env', 'test', 'mapping-stop'],
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
            '.env.server.example',
            'compose.foxglove.yaml',
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
            'ros2 run mentorpi_fleet simulation_manager.py',
            'fleet-manager:',
            'fleet-scene:',
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
        self.assertIn('image: "${MENTORPI_IMAGE-mentorpi-sim:harmonic}"', compose)
        self.assertIn(
            'IMAGE_VERSION: "${MENTORPI_IMAGE-mentorpi-sim:harmonic}"',
            compose,
        )

        gpu_compose = (BUNDLE / 'compose.gpu.yaml').read_text()
        self.assertIn('/dev/dri:/dev/dri', gpu_compose)
        self.assertIn('LIBGL_ALWAYS_SOFTWARE: "0"', gpu_compose)

        script = (BUNDLE / 'run.sh').read_text()
        for command in ('build', 'sim-up', 'down', 'logs', 'test', 'fork-up'):
            self.assertIn(command, script)
        self.assertIn(
            'up -d dds-discovery gazebo-server fleet-manager fleet-scene foxglove-bridge',
            script,
        )
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

    def test_runtime_contains_only_direct_foxglove_observation_assets(self):
        for removed in (
            'Caddyfile.viewer',
            'compose.viewer.yaml',
            'compose.viewer-public.yaml',
            'viewer-entrypoint.sh',
            '.env.server-viewer.example',
        ):
            self.assertFalse((BUNDLE / removed).exists(), removed)

        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        for removed_package in ('novnc', 'websockify', 'x11vnc', 'xvfb'):
            self.assertNotIn(removed_package, dockerfile)

        readme = (BUNDLE / 'README.md').read_text()
        self.assertIn('ws://<server-lan-ip>:8765', readme)
        self.assertIn('TCP 8765', readme)

    def test_repository_has_no_duplicate_root_runtime_layout(self):
        for legacy_path in ('docker', 'ros2_ws', 'compose.yaml', 'test'):
            self.assertFalse((REPOSITORY_ROOT / legacy_path).exists(), legacy_path)

    def test_operator_docs_describe_in_place_bundle_changes(self):
        readme = (BUNDLE / 'README.md').read_text()
        for removed in ('ssh -Y', 'XAUTHORITY', 'VirtualGL', 'XQuartz'):
            self.assertNotIn(removed, readme)
        for text in (
            'linux/amd64',
            './run.sh --env dev build',
            './run.sh --env dev sim-up',
            './run.sh --env server logs',
            './run.sh --env server down',
            './run.sh --env server fork-up',
            './run.sh --env server topics',
            'MENTORPI_IMAGE',
            '버전 tag는 레지스트리에서 다른 이미지로 이동할 수 있으므로 그 자체로 불변하지 않다',
            '검증된 digest를 pull하여 사용',
            '브라우저',
            '오프스크린',
            'native Ubuntu',
            'release gate',
            './run.sh --env dev mapping-up',
            './run.sh --env dev mapping-stop',
            './run.sh --env dev mapping-status',
            '.inprogress',
            'mentorpi-slam-data',
            'SLAM_VOLUME_NAME',
            'mentorpi-slam-data:/slam-data:ro',
            '첫 mapping-up',
            '기존 세션 내용을 변경하지 않는다',
            '개발 PC: 네이티브 Gazebo GUI',
            '개발 PC: Docker 통합 테스트',
            '서버 PC: 시뮬레이션만 실행',
            '서버 PC: SLAM 지도 생성',
            '서비스별 실행 범위',
            'sim-adapter-up',
            'slam-mapper',
            'docker compose --profile mapping logs -f slam-mapper',
            '--force-recreate sim-adapter',
            '현재 지원하지 않는다',
            'GZ_IP=127.0.0.1',
            './run.sh --env dev gz-server',
            './run.sh --env dev gz-gui',
            'GZ_SIM_RESOURCE_PATH',
        ):
            self.assertIn(text, readme)
        self.assertNotIn(
            'gz sim ros2_ws/src/mentorpi_gz_sim/worlds/warehouse.sdf',
            readme.splitlines(),
        )

    def test_operator_docs_describe_runtime_sdf_asset_mount_contract(self):
        readme = (BUNDLE / 'README.md').read_text()

        for asset_path in (
            '`ros2_ws/src/mentorpi_gz_sim/worlds`',
            '`ros2_ws/src/mentorpi_gz_sim/models`',
        ):
            self.assertIn(asset_path, readme)
        for contract_text in (
            '서버 배포 시에는 이미지와 Compose 파일뿐 아니라 위 두 디렉터리도',
            '읽기 전용으로 bind mount한다.',
            '일반 ROS 코드와\nGazebo 플러그인 바이너리는 이 예외에 포함하지 않으며',
            'SDF·mesh·model 자산을 변경한 뒤에는 사용하는 profile로 전체 simulation 서비스를 다시\n시작한다.',
            './run.sh --env <profile> down',
            './run.sh --env <profile> sim-up',
            'Gazebo와 SceneUpdate publisher가 변경된 자산을 다시 읽도록 한다.',
            'ROS 코드나 Gazebo 플러그인을 변경한 경우에는 bind mount로 반영되지 않으므로 이미지를 다시\n빌드한 뒤 서비스를 시작해야 한다.',
        ):
            self.assertIn(contract_text, readme)

    def test_operator_docs_describe_shared_observation_operations(self):
        readme = (BUNDLE / 'README.md').read_text()
        for text in (
            '.env.server',
            'GZ_SERVER_IP',
            './run.sh --env server foxglove-logs',
            './run.sh --env server foxglove-down',
            'ws://<server-lan-ip>:8765',
            'TCP 8765',
            'Gazebo Transport와 ROS DDS discovery는 개발 PC나 공용 인터넷에 직접',
            'Foxglove Studio',
            '별도 web image',
        ):
            self.assertIn(text, readme)

        for removed in ('server-viewer', 'viewer-up', 'viewer-down'):
            self.assertNotIn(removed, readme)

        old_plan = (
            REPOSITORY_ROOT
            / 'docs/superpowers/plans/2026-07-26-mentorpi-gazebo-web-monitor.md'
        ).read_text()
        self.assertTrue(
            old_plan.startswith(
                '# MentorPi Gazebo Web Monitor Implementation Plan\n\n'
                '> **Superseded:** 이 계획의 웹 전용·basic-auth 전제는\n'
                '> `docs/superpowers/plans/2026-07-28-mentorpi-gazebo-shared-observation.md`로\n'
            ),
            'old plan must start with the shared-observation superseded notice',
        )

    def test_operator_docs_distinguish_profile_network_modes_and_image_pull(self):
        readme = (BUNDLE / 'README.md').read_text()

        for text in (
            '`--env server`는 LAN 모드',
            'host networking',
            '`--env dev`는 Docker 내부 모드',
            '`.env.server`의 `MENTORPI_IMAGE`',
            '그 정확한 reference를 `docker pull`',
        ):
            self.assertIn(text, readme)
        self.assertNotIn('registry tag/digest를 export', readme)

    def test_server_operator_docs_describe_lan_host_networking(self):
        readme = (BUNDLE / 'README.md').read_text()
        server_section = readme.split('## 서버 운영\n', 1)[1].split(
            '## 공유 관찰 운영', 1
        )[0]

        for text in (
            '`--env server`',
            'LAN',
            'host networking',
            'DDS_DISCOVERY_HOST=127.0.0.1',
            'loopback',
            'GZ_SERVER_IP',
            'firewall',
        ):
            self.assertIn(text, server_section)
        for stale_text in (
            '내부 `mentorpi` 네트워크',
            '내부 bridge network',
            'Docker DNS의 `dds-discovery`',
            '외부 Gazebo Transport 포트와 ROS DDS 포트는 공개하지 않는다',
        ):
            self.assertNotIn(stale_text, server_section)


if __name__ == '__main__':
    unittest.main()
