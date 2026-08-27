from pathlib import Path
import re
import unittest

import yaml


BUNDLE = Path(__file__).resolve().parents[1]
PINNED_COMMIT = '41f96cc6053632a472d9a821989952771b1117f2'
MENTORPI_COMMIT = 'fb6d9969e935eb0e31966185158e33347951e761'
VEHICLE_MESSAGE_PACKAGES = (
    'ros-humble-bond',
    'ros-humble-dwb-msgs',
    'ros-humble-lifecycle-msgs',
    'ros-humble-map-msgs',
    'ros-humble-nav2-msgs',
    'ros-humble-rcl-interfaces',
    'ros-humble-rosidl-default-runtime',
    'ros-humble-std-msgs',
    'ros-humble-visualization-msgs',
)
EXPECTED_VEHICLE_TOPIC_COUNT = 76


class DockerImageContractTest(unittest.TestCase):
    def test_vehicle_delivery_bundle_is_absent(self):
        for relative in (
            'vehicle',
            'docker-compose.vehicle.yaml',
        ):
            with self.subTest(relative=relative):
                self.assertFalse((BUNDLE / relative).exists())

    def test_server_image_is_humble_multistage_and_pins_bridge_source(self):
        content = (BUNDLE / 'server/Dockerfile').read_text(encoding='utf-8')

        self.assertGreaterEqual(content.count('FROM ros:humble-ros-base-jammy'), 2)
        self.assertNotIn('jazzy', content.lower())
        self.assertIn(PINNED_COMMIT, content)
        self.assertIn('checkout "${FOXGLOVE_BRIDGE_COMMIT}"', content)
        self.assertIn('colcon build', content)
        self.assertIn('rmw-fastrtps-cpp', content)
        self.assertGreaterEqual(
            content.count('ros-humble-resource-retriever'),
            2,
        )
        self.assertGreaterEqual(content.count('ros-humble-rosbag2'), 2)
        self.assertGreaterEqual(content.count('ros-humble-action-msgs'), 2)

    def test_server_entrypoint_sources_ros_and_workspace_then_exec(self):
        content = (BUNDLE / 'server/entrypoint.sh').read_text(encoding='utf-8')

        self.assertIn('/opt/ros/humble/setup.bash', content)
        self.assertIn('/opt/fleet_bridge_ws/install/setup.bash', content)
        self.assertIn('exec "$@"', content)

    def test_server_pins_command_api_and_websocket_dependencies(self):
        content = (BUNDLE / 'server/Dockerfile').read_text(encoding='utf-8')

        self.assertIn('websockets==10.4', content)
        self.assertIn('fastapi==0.115.12', content)
        self.assertIn('httpx==0.28.1', content)
        self.assertIn('uvicorn==0.34.0', content)
        self.assertIn('ros-humble-geometry-msgs', content)
        self.assertIn('/opt/python', content)

    def test_server_image_can_resolve_every_vehicle_message_package(self):
        dockerfile = (BUNDLE / 'server/Dockerfile').read_text(encoding='utf-8')
        package_xml = (
            BUNDLE / 'server/ros2_ws/src/foxglove_ros_worker/package.xml'
        ).read_text(encoding='utf-8')

        for package in VEHICLE_MESSAGE_PACKAGES:
            with self.subTest(package=package):
                self.assertGreaterEqual(dockerfile.count(package), 2)
        self.assertIn(MENTORPI_COMMIT, dockerfile)
        self.assertIn('driver/ros_robot_controller_msgs', dockerfile)
        self.assertIn('<exec_depend>ros_robot_controller_msgs</exec_depend>', package_xml)
        for dependency in (
            'action_msgs',
            'bond',
            'diagnostic_msgs',
            'dwb_msgs',
            'geometry_msgs',
            'lifecycle_msgs',
            'map_msgs',
            'nav2_msgs',
            'nav_msgs',
            'rcl_interfaces',
            'sensor_msgs',
            'std_msgs',
            'tf2_msgs',
            'visualization_msgs',
        ):
            with self.subTest(dependency=dependency):
                self.assertIn(f'<exec_depend>{dependency}</exec_depend>', package_xml)


class ConfigurationContractTest(unittest.TestCase):
    def test_server_foxglove_allows_an_eight_mib_client_send_buffer(self):
        document = yaml.safe_load(
            (BUNDLE / 'config/server_foxglove.yaml').read_text(encoding='utf-8'),
        )
        parameters = document['foxglove_bridge']['ros__parameters']

        self.assertEqual(parameters['send_buffer_limit'], 8 * 1024 * 1024)

    def test_server_foxglove_is_observation_only_and_namespaced(self):
        document = yaml.safe_load(
            (BUNDLE / 'config/server_foxglove.yaml').read_text(encoding='utf-8'),
        )
        parameters = document['foxglove_bridge']['ros__parameters']

        self.assertEqual(parameters['port'], 8765)
        self.assertEqual(parameters['capabilities'], ['none'])
        self.assertTrue({
            'clientPublish', 'parameters', 'parametersSubscribe', 'services',
            'connectionGraph', 'assets',
        }.isdisjoint(parameters['capabilities']))
        self.assertEqual(parameters['client_topic_whitelist'], ['(?!)'])
        self.assertEqual(parameters['service_whitelist'], ['(?!)'])
        self.assertEqual(parameters['param_whitelist'], ['(?!)'])
        whitelist = parameters['topic_whitelist']

        def allowed(topic):
            return any(re.fullmatch(pattern, topic) for pattern in whitelist)

        self.assertTrue(allowed('/robot_1/ascamera/camera_publisher/rgb0/image'))
        self.assertTrue(allowed('/robot_1/ascamera/camera_publisher/depth0/image_raw'))
        self.assertTrue(allowed('/robot_2/goal_pose'))
        self.assertTrue(allowed('/fleet/map'))
        self.assertFalse(allowed('/robot_3/odom'))
        self.assertFalse(allowed('/controller_server/map'))

    def test_server_foxglove_exposes_every_namespaced_vehicle_topic(self):
        document = yaml.safe_load(
            (BUNDLE / 'config/server_foxglove.yaml').read_text(encoding='utf-8'),
        )
        whitelist = document['foxglove_bridge']['ros__parameters']['topic_whitelist']

        def allowed(topic):
            return any(re.fullmatch(pattern, topic) for pattern in whitelist)

        snapshot_pattern = re.compile(r'^(?P<topic>/\S+) \[[^]]+\]$')
        snapshot_lines = (
            BUNDLE / 'config/tmp/vehicle_node_topic'
        ).read_text(encoding='utf-8').splitlines()
        self.assertEqual(len(snapshot_lines), EXPECTED_VEHICLE_TOPIC_COUNT)
        sources = []
        for line_number, line in enumerate(snapshot_lines, start=1):
            match = snapshot_pattern.fullmatch(line)
            self.assertIsNotNone(
                match,
                f'invalid vehicle topic snapshot line {line_number}: {line!r}',
            )
            sources.append(match['topic'])

        for robot_id in ('robot_1', 'robot_2'):
            for source in sources:
                with self.subTest(robot_id=robot_id, source=source):
                    self.assertTrue(allowed(f'/{robot_id}{source}'))

    def test_server_foxglove_matches_every_best_effort_republished_topic(self):
        foxglove = yaml.safe_load(
            (BUNDLE / 'config/server_foxglove.yaml').read_text(encoding='utf-8'),
        )
        patterns = foxglove['foxglove_bridge']['ros__parameters'][
            'best_effort_qos_topic_whitelist'
        ]
        telemetry = yaml.safe_load(
            (BUNDLE / 'config/telemetry.yaml').read_text(encoding='utf-8'),
        )

        for robot_id in ('robot_1', 'robot_2'):
            for topic in telemetry['topics']:
                if topic['qos']['reliability'] != 'best_effort':
                    continue
                target = topic['target'].replace('{robot}', robot_id)
                with self.subTest(robot_id=robot_id, target=target):
                    self.assertTrue(any(
                        re.fullmatch(pattern, target)
                        for pattern in patterns
                    ))

    def test_example_environment_documents_server_domains_and_uris(self):
        content = (BUNDLE / '.env.example').read_text(encoding='utf-8')

        for required in (
            'SERVER_ROS_DOMAIN_ID=225',
            'ROBOT_1_FOXGLOVE_URI=',
            'ROBOT_2_FOXGLOVE_URI=',
            'ROBOT_1_COMMAND_API_URL=',
            'ROBOT_2_COMMAND_API_URL=',
            'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            'FASTDDS_BUILTIN_TRANSPORTS=DEFAULT',
            'COMMAND_API_HOST=',
            'COMMAND_API_PORT=8080',
            'ROSBAG_HOST_DIRECTORY=',
            'ROSBAG_SESSION_ID=',
        ):
            self.assertIn(required, content)
        for removed in (
            'ROBOT_ID=robot_1',
            'ROS_DOMAIN_ID=215',
            'VEHICLE_IMAGE=',
        ):
            self.assertNotIn(removed, content)

    def test_fleet_namespace_is_derived_from_vehicle_id(self):
        document = yaml.safe_load(
            (BUNDLE / 'config/fleet.yaml').read_text(encoding='utf-8'),
        )

        for vehicle in document['vehicles']:
            self.assertNotIn('namespace', vehicle)
            self.assertNotIn('domain_id', vehicle)
            self.assertEqual(vehicle['command']['topic'], '/cmd_vel')
            self.assertEqual(vehicle['command']['type'], 'geometry_msgs/msg/Twist')
        self.assertEqual(document['server']['command_api'], {
            'host': '127.0.0.1',
            'port': 8080,
        })


class ReadmeContractTest(unittest.TestCase):
    def test_readme_covers_server_deployment_and_network_verification(self):
        content = (BUNDLE / 'README.md').read_text(encoding='utf-8')

        for required in (
            'docker compose --env-file',
            'scan',
            'battery',
            'worker_rate',
            'qos',
            'ping',
            'docker stats',
            'ros2 topic hz',
            'Domain Bridge',
            'ws://<server-ip>:8765',
            'http://<server-ip>:8080/docs',
            'POST /api/v1/robots/{robot_id}/cmd_vel',
            'POST /api/v1/robots/{robot_id}/stop',
            'GET /api/v1/vehicle-command/{robot_id}/vehicle-status',
            'POST /api/v1/vehicle-command/{robot_id}/localization/initial-pose',
            'clientPublish',
            'zero Twist',
            '8766',
            'ascamera/camera_publisher/rgb0/image',
            '/goal_pose',
            '/fleet/map',
            'replay_rate_hz',
        ):
            with self.subTest(required=required):
                self.assertIn(required, content)


if __name__ == '__main__':
    unittest.main()
