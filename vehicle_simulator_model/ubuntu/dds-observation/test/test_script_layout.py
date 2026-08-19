from pathlib import Path
import unittest


BUNDLE = Path(__file__).resolve().parents[1]
DOCKERFILE = BUNDLE / 'Dockerfile'


class ScriptLayoutTest(unittest.TestCase):
    def test_map_server_and_rosbag_scripts_are_grouped_by_service(self):
        expected_scripts = {
            'map-server': {'map_server.sh', 'map_server_bootstrap.sh'},
            'rosbag-recorder': {
                'rosbag_recorder.sh',
                'rosbag_recorder_bootstrap.sh',
            },
        }
        dockerfile = DOCKERFILE.read_text(encoding='utf-8')

        for directory, scripts in expected_scripts.items():
            with self.subTest(directory=directory):
                for script in scripts:
                    self.assertTrue((BUNDLE / directory / script).is_file())
                    self.assertIn(f'COPY {directory}/{script} ', dockerfile)

        for script in set().union(*expected_scripts.values()):
            self.assertFalse((BUNDLE / script).exists())


if __name__ == '__main__':
    unittest.main()
