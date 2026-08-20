import json
from pathlib import Path
import subprocess
import unittest


BUNDLE = Path(__file__).resolve().parents[1]
ENV_FILE = BUNDLE / '.env.example'


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


class VehicleComposeContractTest(unittest.TestCase):
    def test_vehicle_raw_bridge_uses_host_network_ipc_and_cmd_vel_policy(self):
        services = compose_config('docker-compose.vehicle.yaml')['services']
        fleet = services['foxglove-fleet']

        self.assertEqual(fleet['network_mode'], 'host')
        self.assertEqual(fleet['ipc'], 'host')
        self.assertEqual(fleet['environment']['ROS_LOCALHOST_ONLY'], '1')
        self.assertEqual(fleet['environment']['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp')
        self.assertEqual(fleet['environment']['FOXGLOVE_MODE'], 'raw')
        self.assertEqual(fleet['environment']['FOXGLOVE_PORT'], '8766')
        self.assertEqual(fleet['stop_signal'], 'SIGINT')
        self.assertNotIn('ports', fleet)

    def test_vehicle_debug_endpoint_is_opt_in_and_uses_port_8765(self):
        services = compose_config('docker-compose.vehicle.yaml')['services']
        debug = services['foxglove-debug']

        self.assertEqual(debug['profiles'], ['debug'])
        self.assertEqual(debug['environment']['FOXGLOVE_MODE'], 'debug')
        self.assertEqual(debug['environment']['FOXGLOVE_PORT'], '8765')
        self.assertEqual(debug['network_mode'], 'host')
        self.assertEqual(debug['ipc'], 'host')
        self.assertEqual(debug['stop_signal'], 'SIGINT')

    def test_vehicle_config_mount_is_read_only(self):
        fleet = compose_config('docker-compose.vehicle.yaml')['services']['foxglove-fleet']
        telemetry_mount = next(
            mount for mount in fleet['volumes']
            if mount['target'] == '/config/telemetry.yaml'
        )
        self.assertTrue(telemetry_mount['read_only'])


class ServerComposeContractTest(unittest.TestCase):
    def test_workers_are_isolated_per_vehicle_on_server_domain(self):
        services = compose_config('docker-compose.server.yaml')['services']

        for name, robot_id, uri in (
            ('worker-robot-1', 'robot_1', 'ws://192.168.10.215:8766'),
            ('worker-robot-2', 'robot_2', 'ws://192.168.10.216:8766'),
        ):
            with self.subTest(service=name):
                worker = services[name]
                self.assertEqual(worker['environment']['ROBOT_ID'], robot_id)
                self.assertEqual(worker['environment']['FOXGLOVE_URI'], uri)
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

    def test_command_api_uses_vehicle_uri_and_configured_swagger_port(self):
        api = compose_config('docker-compose.server.yaml')['services']['command-api']

        self.assertEqual(api['network_mode'], 'host')
        self.assertEqual(api['environment']['COMMAND_API_HOST'], '127.0.0.1')
        self.assertEqual(api['environment']['COMMAND_API_PORT'], '8080')
        self.assertEqual(
            api['environment']['ROBOT_1_FOXGLOVE_URI'],
            'ws://192.168.10.215:8766',
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


if __name__ == '__main__':
    unittest.main()
