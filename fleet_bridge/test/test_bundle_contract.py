from pathlib import Path
import re
import unittest

import yaml


BUNDLE = Path(__file__).resolve().parents[1]
PINNED_COMMIT = '41f96cc6053632a472d9a821989952771b1117f2'


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
        self.assertIn('uvicorn==0.34.0', content)
        self.assertIn('ros-humble-geometry-msgs', content)
        self.assertIn('/opt/python', content)


class ConfigurationContractTest(unittest.TestCase):
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

        self.assertTrue(allowed('/robot_1/rgb/image_raw'))
        self.assertTrue(allowed('/controller_server/map'))
        self.assertFalse(allowed('/robot_1/depth/image_raw'))
        self.assertFalse(allowed('/robot_1/depth/camera_info'))

    def test_example_environment_documents_server_domains_and_uris(self):
        content = (BUNDLE / '.env.example').read_text(encoding='utf-8')

        for required in (
            'SERVER_ROS_DOMAIN_ID=225',
            'ROBOT_1_FOXGLOVE_URI=ws://192.168.10.215:8766',
            'ROBOT_2_FOXGLOVE_URI=ws://192.168.10.216:8766',
            'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            'FASTDDS_BUILTIN_TRANSPORTS=DEFAULT',
            'COMMAND_API_HOST=127.0.0.1',
            'COMMAND_API_PORT=8080',
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
            'clientPublish',
            'zero Twist',
            '8766',
        ):
            with self.subTest(required=required):
                self.assertIn(required, content)


if __name__ == '__main__':
    unittest.main()
