from pathlib import Path
import subprocess
import unittest


COMPOSE_FILE = Path(__file__).resolve().parents[1] / 'docker-compose.yaml'


class ComposeIpcContractTest(unittest.TestCase):
    def test_map_server_and_bridge_share_host_ipc_for_fast_dds(self):
        compose = COMPOSE_FILE.read_text(encoding='utf-8')

        foxglove = compose.split('  foxglove-bridge:', 1)[1].split(
            '  map-server:', 1,
        )[0]
        map_server = compose.split('  map-server:', 1)[1].split(
            '  rosbag-recorder:', 1,
        )[0]

        self.assertIn('network_mode: host', foxglove)
        self.assertIn('ipc: host', foxglove)
        self.assertIn('network_mode: host', map_server)
        self.assertIn('ipc: host', map_server)

    def test_rosbag_recorder_uses_the_configured_host_directory(self):
        result = subprocess.run(
            ['docker', 'compose', '--env-file', '.env.example', 'config'],
            cwd=COMPOSE_FILE.parent,
            check=True,
            capture_output=True,
            encoding='utf-8',
        )
        recorder = result.stdout.split('  rosbag-recorder:', 1)[1].split(
            'networks:', 1,
        )[0]

        self.assertIn('type: bind', recorder)
        self.assertIn('source: /home/litcoder/logistics_database', recorder)
        self.assertIn('target: /rosbag', recorder)


if __name__ == '__main__':
    unittest.main()
