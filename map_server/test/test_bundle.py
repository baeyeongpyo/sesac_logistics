import json
import os
import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


BUNDLE = Path(__file__).resolve().parents[1]
COMPOSE = BUNDLE / 'docker-compose.yaml'


class MapServerBundleTest(unittest.TestCase):
    def test_default_compose_configuration_mounts_the_bundle_maps_directory(self):
        """The bundle must start from its checked-in map files without host paths."""
        environment = os.environ.copy()
        for name in ('MAP_DIRECTORY', 'MAP_YAML', 'MAP_USE_SIM_TIME', 'ROS_DOMAIN_ID'):
            environment.pop(name, None)
        result = subprocess.run(
            [
                'docker', 'compose',
                '--env-file', str(BUNDLE / '.env.example'),
                '-f', str(COMPOSE),
                'config', '--format', 'json',
            ],
            text=True,
            capture_output=True,
            env=environment,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        service = json.loads(result.stdout)['services']['map-server']
        mount = {item['target']: item for item in service['volumes']}['/maps']
        self.assertEqual(mount['source'], str(BUNDLE / 'maps'))
        self.assertTrue(mount['read_only'])
        self.assertTrue((BUNDLE / 'maps' / 'map_0825.yaml').is_file())
        self.assertTrue((BUNDLE / 'maps' / 'map_0825.pgm').is_file())

    def test_map_0825_is_mounted_read_only_and_published_on_central_domain(self):
        """A static map must be available to the central DDS domain without copies."""
        self.assertTrue(COMPOSE.is_file())
        with TemporaryDirectory() as directory:
            map_directory = Path(directory)
            (map_directory / 'map_0825.yaml').write_text(
                'image: map_0825.pgm\n',
                encoding='utf-8',
            )
            (map_directory / 'map_0825.pgm').write_bytes(b'P5\n1 1\n255\n\0')
            environment = os.environ.copy()
            environment.update({
                'MAP_DIRECTORY': directory,
                'MAP_USE_SIM_TIME': 'false',
                'ROS_DOMAIN_ID': '225',
            })
            result = subprocess.run(
                [
                    'docker', 'compose',
                    '--env-file', str(BUNDLE / '.env.example'),
                    '-f', str(COMPOSE),
                    'config', '--format', 'json',
                ],
                text=True,
                capture_output=True,
                env=environment,
                check=False,
            )

        self.assertEqual(result.returncode, 0, result.stderr)
        service = json.loads(result.stdout)['services']['map-server']
        self.assertEqual(service['network_mode'], 'host')
        self.assertEqual(service['ipc'], 'host')
        self.assertEqual(service['command'], ['/usr/local/bin/mentorpi-map-server'])
        self.assertEqual(service['environment']['ROS_DOMAIN_ID'], '225')
        self.assertEqual(service['environment']['MAP_YAML'], '/maps/map_0825.yaml')
        mounts = {mount['target']: mount for mount in service['volumes']}
        self.assertEqual(mounts['/maps']['source'], directory)
        self.assertTrue(mounts['/maps']['read_only'])
        self.assertEqual(set(mounts), {'/maps'})

    def test_built_image_exposes_the_map_visualization_tf_executable(self):
        """The launch file must be able to execute the installed TF publisher."""
        build = subprocess.run(
            [
                'docker', 'compose',
                '--env-file', str(BUNDLE / '.env.example'),
                '-f', str(COMPOSE),
                'build', 'map-server',
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(build.returncode, 0, build.stderr)
        executable = subprocess.run(
            [
                'docker', 'run', '--rm',
                '--entrypoint', 'test',
                'mentorpi-map-server:humble',
                '-x', '/ws/install/mentorpi_map_server/lib/mentorpi_map_server/map_visualization_tf.py',
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(executable.returncode, 0, executable.stderr)


if __name__ == '__main__':
    unittest.main()
