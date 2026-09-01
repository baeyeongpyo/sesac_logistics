from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
MAP_SERVER = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PACKAGE))

from central_map_server.publisher import MapConfigError, _arguments, load_map


class DirectMapLoaderTest(unittest.TestCase):
    def write_map(self, directory: Path, pgm: bytes, *, mode: str = 'trinary') -> Path:
        (directory / 'map.pgm').write_bytes(pgm)
        yaml_path = directory / 'map.yaml'
        yaml_path.write_text(
            '\n'.join((
                'image: map.pgm',
                f'mode: {mode}',
                'resolution: 0.5',
                'origin: [-1.0, -2.0, 0.0]',
                'negate: 0',
                'occupied_thresh: 0.65',
                'free_thresh: 0.25',
            )),
            encoding='utf-8',
        )
        return yaml_path

    def test_p5_map_uses_nav2_occupancy_values_and_bottom_left_row_order(self):
        with TemporaryDirectory() as temporary_directory:
            yaml_path = self.write_map(
                Path(temporary_directory),
                b'P5\n2 2\n255\n' + bytes((0, 255, 128, 0)),
            )
            loaded = load_map(yaml_path)

        self.assertEqual((loaded.width, loaded.height), (2, 2))
        self.assertEqual(loaded.resolution, 0.5)
        self.assertEqual(loaded.origin, (-1.0, -2.0, 0.0))
        # PGM is top-to-bottom; OccupancyGrid starts at the bottom-left origin.
        self.assertEqual(loaded.data, (-1, 100, 100, 0))

    def test_p2_map_and_yaml_image_relative_path_are_supported(self):
        with TemporaryDirectory() as temporary_directory:
            yaml_path = self.write_map(
                Path(temporary_directory),
                b'P2\n# direct central map\n1 2\n255\n0\n255\n',
            )
            loaded = load_map(yaml_path)

        self.assertEqual((loaded.width, loaded.height), (1, 2))
        self.assertEqual(loaded.data, (0, 100))

    def test_non_trinary_map_mode_is_rejected_before_ros_publication(self):
        with TemporaryDirectory() as temporary_directory:
            yaml_path = self.write_map(
                Path(temporary_directory),
                b'P5\n1 1\n255\n\xff',
                mode='scale',
            )
            with self.assertRaisesRegex(MapConfigError, 'mode'):
                load_map(yaml_path)

    def test_checked_in_map_preserves_the_existing_nav2_geometry(self):
        loaded = load_map(MAP_SERVER / 'maps' / 'map_0825.yaml')

        self.assertEqual((loaded.width, loaded.height), (196, 128))
        self.assertEqual(loaded.resolution, 0.05)
        self.assertEqual(loaded.origin, (-5.04, -4.03, 0.0))
        self.assertEqual(len(loaded.data), 196 * 128)

    def test_map_cli_preserves_ros_arguments_for_rclpy(self):
        arguments, ros_arguments = _arguments([
            '--map-yaml', '/maps/test.yaml',
            '--ros-args', '-p', 'use_sim_time:=false',
        ])

        self.assertEqual(arguments.map_yaml, '/maps/test.yaml')
        self.assertEqual(ros_arguments, ['--ros-args', '-p', 'use_sim_time:=false'])


if __name__ == '__main__':
    unittest.main()
