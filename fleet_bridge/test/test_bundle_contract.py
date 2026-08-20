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

    def test_server_pins_python_310_compatible_websockets(self):
        content = (BUNDLE / 'server/Dockerfile').read_text(encoding='utf-8')

        self.assertIn('websockets==10.4', content)
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
        ):
            self.assertIn(required, content)


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
        ):
            with self.subTest(required=required):
                self.assertIn(required, content)


if __name__ == '__main__':
    unittest.main()
