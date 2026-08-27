import json
from pathlib import Path
import subprocess
import unittest


BUNDLE = Path(__file__).resolve().parents[1]
ENV_FILE = BUNDLE / '.env.example'


def environment_values():
    return {
        key: value
        for line in ENV_FILE.read_text(encoding='utf-8').splitlines()
        if line and not line.startswith('#') and '=' in line
        for key, value in [line.split('=', 1)]
    }


def compose_config(filename):
    result = subprocess.run(
        [
            'docker', 'compose',
            '--env-file', str(ENV_FILE),
            '-f', str(BUNDLE / filename),
            '--profile', '*',
            'config', '--format', 'json',
        ],
        cwd=BUNDLE,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise AssertionError(result.stderr)
    return json.loads(result.stdout)


class ServerComposeContractTest(unittest.TestCase):
    def test_workers_are_isolated_per_vehicle_on_server_domain(self):
        services = compose_config('docker-compose.server.yaml')['services']
        environment = environment_values()

        for name, robot_id, variable in (
            ('worker-robot-1', 'robot_1', 'ROBOT_1_FOXGLOVE_URI'),
            ('worker-robot-2', 'robot_2', 'ROBOT_2_FOXGLOVE_URI'),
        ):
            with self.subTest(service=name):
                worker = services[name]
                self.assertEqual(worker['environment']['ROBOT_ID'], robot_id)
                self.assertEqual(worker['environment']['FOXGLOVE_URI'], environment[variable])
                self.assertEqual(worker['environment']['ROS_DOMAIN_ID'], '225')
                self.assertEqual(worker['environment']['ROS_LOCALHOST_ONLY'], '1')
                self.assertEqual(worker['network_mode'], 'host')
                self.assertEqual(worker['ipc'], 'host')
                self.assertEqual(worker['stop_signal'], 'SIGINT')

    def test_server_bridge_exposes_only_host_port_and_read_only_config(self):
        bridge = compose_config('docker-compose.server.yaml')['services']['server-foxglove']

        self.assertEqual(bridge['network_mode'], 'host')
        self.assertEqual(bridge['ipc'], 'host')
        self.assertEqual(bridge['environment']['ROS_DOMAIN_ID'], '225')
        self.assertEqual(bridge['stop_signal'], 'SIGINT')
        self.assertNotIn('ports', bridge)
        config_mount = next(
            mount for mount in bridge['volumes']
            if mount['target'] == '/config/server_foxglove.yaml'
        )
        self.assertTrue(config_mount['read_only'])

    def test_command_api_publishes_configured_port_without_host_network(self):
        api = compose_config('docker-compose.server.yaml')['services']['command-api']
        environment = environment_values()

        self.assertNotIn('network_mode', api)
        self.assertEqual(len(api['ports']), 1)
        published_port = api['ports'][0]
        self.assertEqual(published_port['target'], int(environment['COMMAND_API_PORT']))
        self.assertEqual(published_port['published'], environment['COMMAND_API_PORT'])
        self.assertEqual(published_port['protocol'], 'tcp')
        self.assertEqual(
            api['environment']['COMMAND_API_HOST'],
            environment['COMMAND_API_HOST'],
        )
        self.assertEqual(
            api['environment']['COMMAND_API_PORT'],
            environment['COMMAND_API_PORT'],
        )
        self.assertEqual(
            api['environment']['ROBOT_1_COMMAND_API_URL'],
            environment['ROBOT_1_COMMAND_API_URL'],
        )
        self.assertEqual(
            api['environment']['ROBOT_2_COMMAND_API_URL'],
            environment['ROBOT_2_COMMAND_API_URL'],
        )
        self.assertEqual(
            api['command'],
            ['ros2', 'run', 'foxglove_ros_worker', 'fleet_command_api'],
        )
        config_mount = next(
            mount for mount in api['volumes']
            if mount['target'] == '/config/fleet.yaml'
        )
        self.assertTrue(config_mount['read_only'])

    def test_rosbag_recorder_uses_server_domain_and_read_only_configs(self):
        recorder = compose_config('docker-compose.server.yaml')['services']['rosbag-recorder']

        self.assertEqual(recorder['profiles'], ['recording'])
        self.assertEqual(recorder['network_mode'], 'host')
        self.assertEqual(recorder['ipc'], 'host')
        self.assertEqual(recorder['environment']['ROS_DOMAIN_ID'], '225')
        self.assertEqual(recorder['environment']['ROBOT_IDS'], 'robot_1,robot_2')
        self.assertEqual(
            recorder['command'],
            ['ros2', 'run', 'foxglove_ros_worker', 'fleet_rosbag_recorder'],
        )
        for target in (
            '/config/fleet.yaml',
            '/config/telemetry.yaml',
            '/config/central_topics.yaml',
        ):
            mount = next(
                mount for mount in recorder['volumes']
                if mount['target'] == target
            )
            self.assertTrue(mount['read_only'])

    def test_central_topic_republisher_replays_map_on_server_domain(self):
        relay = compose_config('docker-compose.server.yaml')['services'][
            'central-topic-republisher'
        ]

        self.assertEqual(relay['network_mode'], 'host')
        self.assertEqual(relay['ipc'], 'host')
        self.assertEqual(relay['environment']['ROS_DOMAIN_ID'], '225')
        self.assertEqual(
            relay['command'],
            [
                'ros2',
                'run',
                'foxglove_ros_worker',
                'fleet_central_topic_republisher',
            ],
        )
        config_mount = next(
            mount for mount in relay['volumes']
            if mount['target'] == '/config/central_topics.yaml'
        )
        self.assertTrue(config_mount['read_only'])


if __name__ == '__main__':
    unittest.main()
