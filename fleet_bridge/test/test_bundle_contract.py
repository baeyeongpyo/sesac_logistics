from pathlib import Path
import unittest

import yaml


BUNDLE = Path(__file__).resolve().parents[1]
PINNED_COMMIT = '41f96cc6053632a472d9a821989952771b1117f2'


class DockerImageContractTest(unittest.TestCase):
    def test_both_images_are_humble_multistage_and_pin_bridge_source(self):
        for relative in ('vehicle/Dockerfile', 'server/Dockerfile'):
            with self.subTest(dockerfile=relative):
                content = (BUNDLE / relative).read_text(encoding='utf-8')
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

    def test_entrypoints_source_ros_and_workspace_then_exec(self):
        for relative in ('vehicle/entrypoint.sh', 'server/entrypoint.sh'):
            with self.subTest(entrypoint=relative):
                content = (BUNDLE / relative).read_text(encoding='utf-8')
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
        self.assertIn('^/robot_1/.*$', parameters['topic_whitelist'])
        self.assertIn('^/robot_2/.*$', parameters['topic_whitelist'])

    def test_example_environment_documents_vehicle_domains_and_uris(self):
        content = (BUNDLE / '.env.example').read_text(encoding='utf-8')

        for required in (
            'ROBOT_ID=robot_1',
            'ROS_DOMAIN_ID=215',
            'SERVER_ROS_DOMAIN_ID=225',
            'ROBOT_1_FOXGLOVE_URI=ws://192.168.10.215:8766',
            'ROBOT_2_FOXGLOVE_URI=ws://192.168.10.216:8766',
            'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            'FASTDDS_BUILTIN_TRANSPORTS=DEFAULT',
            'COMMAND_API_HOST=127.0.0.1',
            'COMMAND_API_PORT=8080',
        ):
            self.assertIn(required, content)

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
    def test_readme_covers_deployment_tuning_and_network_verification(self):
        content = (BUNDLE / 'README.md').read_text(encoding='utf-8')

        for required in (
            'docker compose --env-file',
            'ROBOT_ID',
            'ROS_DOMAIN_ID',
            'scan',
            'battery',
            'worker_rate',
            'qos',
            'ping',
            'docker stats',
            'ros2 topic hz',
            'Domain Bridge',
            'foxglove-debug',
            'ws://<server-ip>:8765',
            'http://<server-ip>:8080/docs',
            'POST /api/v1/robots/{robot_id}/cmd_vel',
            'POST /api/v1/robots/{robot_id}/stop',
            'clientPublish',
            'zero Twist',
            '주행 컨테이너 내부',
        ):
            with self.subTest(required=required):
                self.assertIn(required, content)


if __name__ == '__main__':
    unittest.main()
