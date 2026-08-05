import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'map_session.py'
SPEC = importlib.util.spec_from_file_location('map_session', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_session(root, session_id, created_at):
    session = root / session_id
    session.mkdir()
    (session / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\n')
    (session / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (session / 'manifest.json').write_text(json.dumps({
        'session_id': session_id,
        'created_at': created_at,
    }) + '\n')
    checksums = []
    for path in ('manifest.json', 'map.pgm', 'map.yaml'):
        digest = hashlib.sha256((session / path).read_bytes()).hexdigest()
        checksums.append(f'{digest}  {path}')
    (session / 'checksums.sha256').write_text('\n'.join(checksums) + '\n')
    return session


class MapSessionTest(unittest.TestCase):
    def test_selects_requested_valid_session(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            write_session(root, 'map-a', '2026-08-05T00:00:00Z')

            selected = MODULE.find_valid_session(root, 'map-a')

        self.assertEqual(selected.session_id, 'map-a')
        self.assertEqual(selected.map_yaml.name, 'map.yaml')

    def test_selects_newest_valid_session_when_request_is_auto(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            write_session(root, 'old-map', '2026-08-04T00:00:00Z')
            write_session(root, 'new-map', '2026-08-05T00:00:00Z')

            selected = MODULE.find_valid_session(root, None)

        self.assertEqual(selected.session_id, 'new-map')

    def test_rejects_missing_map_or_invalid_checksum(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            session = write_session(root, 'bad-map', '2026-08-05T00:00:00Z')
            (session / 'map.pgm').write_bytes(b'changed')

            selected = MODULE.find_valid_session(root, 'bad-map')

        self.assertIsNone(selected)

    def test_rejects_checksum_path_traversal(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            session = write_session(root, 'unsafe-map', '2026-08-05T00:00:00Z')
            digest = hashlib.sha256(b'outside').hexdigest()
            (session / 'checksums.sha256').write_text(f'{digest}  ../outside\n')

            selected = MODULE.find_valid_session(root, 'unsafe-map')

        self.assertIsNone(selected)


if __name__ == '__main__':
    unittest.main()
