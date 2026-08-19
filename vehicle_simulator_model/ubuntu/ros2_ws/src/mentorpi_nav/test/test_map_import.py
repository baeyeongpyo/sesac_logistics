import hashlib
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'map_import.py'


def write_source_map(root: Path, map_id: str) -> Path:
    source = root / map_id
    source.mkdir(parents=True)
    (source / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (source / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding='utf-8',
    )
    return source


class MapImportTest(unittest.TestCase):
    def test_imports_named_map_as_a_verified_immutable_session(self):
        """A selected import ID must become a Nav2-readable session without mutating its source."""
        with TemporaryDirectory() as directory:
            root = Path(directory)
            source_root = root / 'map-import'
            destination_root = root / 'slam-data'
            source = write_source_map(source_root, 'warehouse-real-v1')

            result = subprocess.run(
                [
                    sys.executable, str(SCRIPT),
                    '--source-root', str(source_root),
                    '--destination-root', str(destination_root),
                    '--map-id', 'warehouse-real-v1',
                ],
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            session = destination_root / 'warehouse-real-v1'
            self.assertEqual((session / 'map.pgm').read_bytes(), (source / 'map.pgm').read_bytes())
            self.assertEqual((session / 'map.yaml').read_text(), (source / 'map.yaml').read_text())
            manifest = json.loads((session / 'manifest.json').read_text())
            self.assertEqual(manifest['session_id'], 'warehouse-real-v1')
            self.assertEqual(manifest['map_source'], 'external-import')
            checksums = (session / 'checksums.sha256').read_text().splitlines()
            expected = hashlib.sha256((session / 'map.pgm').read_bytes()).hexdigest()
            self.assertIn(f'{expected}  map.pgm', checksums)
            self.assertTrue((source / 'map.pgm').is_file())

    def test_refuses_to_overwrite_an_existing_published_map_id(self):
        """Re-importing an ID must not silently replace the map Nav2 is using."""
        with TemporaryDirectory() as directory:
            root = Path(directory)
            source_root = root / 'map-import'
            destination_root = root / 'slam-data'
            write_source_map(source_root, 'warehouse-real-v1')
            destination_root.mkdir()
            published = destination_root / 'warehouse-real-v1'
            published.mkdir()
            (published / 'map.yaml').write_text('original-map\n', encoding='utf-8')

            result = subprocess.run(
                [
                    sys.executable, str(SCRIPT),
                    '--source-root', str(source_root),
                    '--destination-root', str(destination_root),
                    '--map-id', 'warehouse-real-v1',
                ],
                text=True,
                capture_output=True,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn('already exists', result.stderr)
            self.assertEqual((published / 'map.yaml').read_text(), 'original-map\n')

    def test_rejects_a_map_yaml_that_points_outside_the_imported_session(self):
        """An imported map must not retain an absolute or parent-directory image reference."""
        with TemporaryDirectory() as directory:
            root = Path(directory)
            source_root = root / 'map-import'
            destination_root = root / 'slam-data'
            source = write_source_map(source_root, 'warehouse-real-v2')
            (source / 'map.yaml').write_text(
                'image: ../another-map.pgm\nresolution: 0.05\n', encoding='utf-8'
            )

            result = subprocess.run(
                [
                    sys.executable, str(SCRIPT),
                    '--source-root', str(source_root),
                    '--destination-root', str(destination_root),
                    '--map-id', 'warehouse-real-v2',
                ],
                text=True,
                capture_output=True,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn('image must reference map.pgm', result.stderr)
            self.assertFalse((destination_root / 'warehouse-real-v2').exists())


if __name__ == '__main__':
    unittest.main()
