from pathlib import Path
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


if __name__ == '__main__':
    unittest.main()
