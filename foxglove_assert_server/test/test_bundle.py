import importlib.util
import json
import os
import subprocess
import unittest
from xml.etree import ElementTree
from functools import partial
from http.client import HTTPConnection
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread


BUNDLE = Path(__file__).resolve().parents[1]
COMPOSE = BUNDLE / 'docker-compose.yaml'
SERVER = BUNDLE / 'serve_assets.py'


class FoxgloveAssetServerBundleTest(unittest.TestCase):
    def test_default_compose_configuration_mounts_the_bundle_assets_directory(self):
        """The asset server must start from the URDF and meshes stored with it."""
        environment = os.environ.copy()
        for name in ('ASSET_DIRECTORY', 'ASSET_BIND_ADDRESS', 'ASSET_PORT'):
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
        service = json.loads(result.stdout)['services']['asset-server']
        mount = {item['target']: item for item in service['volumes']}['/assets']
        self.assertEqual(mount['source'], str(BUNDLE / 'assets'))
        self.assertTrue(mount['read_only'])
        assets = BUNDLE / 'assets' / 'hiwonder_mecanum_forklift'
        model = assets / 'urdf' / 'hiwonder_mecanum_forklift.urdf'
        self.assertTrue(model.is_file())
        self.assertTrue((assets / 'meshes' / 'mentorpi' / 'base_link.STL').is_file())
        self.assertTrue((assets / 'meshes' / 'mentorpi' / 'cam_Link.STL').is_file())
        self.assertTrue((assets / 'meshes' / 'mentorpi' / 'lidar_Link.STL').is_file())

        robot = ElementTree.parse(model).getroot()
        self.assertEqual(robot.tag, 'robot')
        self.assertEqual(robot.attrib['name'], 'hiwonder_mecanum_forklift')
        self.assertEqual(
            {link.attrib['name'] for link in robot.findall('link')},
            {
                'base_link', 'front_left_wheel', 'rear_left_wheel',
                'front_right_wheel', 'rear_right_wheel', 'lift_mast',
                'fork_carriage',
            },
        )
        mesh_uris = {
            mesh.attrib['filename']
            for mesh in robot.findall('.//mesh')
        }
        self.assertEqual(
            mesh_uris,
            {
                'package://hiwonder_mecanum_forklift/meshes/mentorpi/base_link.STL',
                'package://hiwonder_mecanum_forklift/meshes/mentorpi/cam_Link.STL',
                'package://hiwonder_mecanum_forklift/meshes/mentorpi/lidar_Link.STL',
            },
        )

    def test_standalone_bundle_mounts_only_the_configured_asset_directory(self):
        """The asset server must not depend on vehicle_simulator_model files."""
        self.assertTrue(COMPOSE.is_file())
        with TemporaryDirectory() as directory:
            environment = os.environ.copy()
            environment.update({
                'ASSET_DIRECTORY': directory,
                'ASSET_BIND_ADDRESS': '0.0.0.0',
                'ASSET_PORT': '8081',
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
        service = json.loads(result.stdout)['services']['asset-server']
        self.assertEqual(service['network_mode'], 'host')
        self.assertEqual(service['command'], [
            'python3', '/app/serve_assets.py', '--port', '8081', '--bind',
            '0.0.0.0', '--directory', '/assets',
        ])
        mounts = {mount['target']: mount for mount in service['volumes']}
        self.assertEqual(mounts['/assets']['source'], directory)
        self.assertTrue(mounts['/assets']['read_only'])
        self.assertEqual(set(mounts), {'/assets'})

    def test_standalone_server_supports_cors_and_byte_ranges(self):
        """Foxglove web clients must receive CORS-enabled 206 asset responses."""
        specification = importlib.util.spec_from_file_location(
            'foxglove_asset_server',
            SERVER,
        )
        self.assertIsNotNone(specification)
        self.assertIsNotNone(specification.loader)
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)

        with TemporaryDirectory() as directory:
            asset = Path(directory) / 'robot.urdf'
            asset.write_bytes(b'robot-model')
            server = module.ThreadingHTTPServer(
                ('127.0.0.1', 0),
                partial(module.FoxgloveAssetRequestHandler, directory=directory),
            )
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                connection = HTTPConnection('127.0.0.1', server.server_port)
                connection.request(
                    'GET',
                    '/robot.urdf',
                    headers={'Range': 'bytes=2-6'},
                )
                response = connection.getresponse()
                self.assertEqual(response.status, 206)
                self.assertEqual(response.read(), b'bot-m')
                self.assertEqual(response.getheader('Content-Range'), 'bytes 2-6/11')
                self.assertEqual(response.getheader('Access-Control-Allow-Origin'), '*')
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2.0)


if __name__ == '__main__':
    unittest.main()
