import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/session_artifacts.py'
SPEC = importlib.util.spec_from_file_location('session_artifacts', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


METADATA = {
    'session_id': 'session-001',
    'robot_id': 'robot_1',
    'image_version': 'sha-f471f23',
    'git_commit': 'f471f23',
    'world_version': 'warehouse-v1',
    'model_version': 'mentorpi-m1-v1',
    'slam_params_sha256': 'a' * 64,
    'tf_calibration_version': 'ground-truth-v1',
    'created_at': '2026-07-26T00:00:00Z',
}


class SessionArtifactsTest(unittest.TestCase):
    def test_manifest_and_checksums_are_deterministic(self):
        with TemporaryDirectory() as directory:
            session = Path(directory)
            (session / 'map.yaml').write_text('resolution: 0.05\n')
            (session / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')

            manifest_path = MODULE.write_manifest(session, METADATA)
            checksum_path = MODULE.write_checksums(session)

            manifest = json.loads(manifest_path.read_text())
            self.assertEqual(manifest, METADATA)
            self.assertEqual(
                [line.split('  ', 1)[1] for line in checksum_path.read_text().splitlines()],
                ['manifest.json', 'map.pgm', 'map.yaml'],
            )
            self.assertEqual(
                manifest_path.read_text(),
                json.dumps(METADATA, indent=2, sort_keys=True) + '\n',
            )

    def test_manifest_rejects_missing_or_empty_required_metadata(self):
        with TemporaryDirectory() as directory:
            missing = METADATA.copy()
            missing.pop('session_id')
            empty = METADATA.copy()
            empty['robot_id'] = ''

            with self.assertRaises(ValueError):
                MODULE.write_manifest(Path(directory), missing)
            with self.assertRaises(ValueError):
                MODULE.write_manifest(Path(directory), empty)

    def test_checksums_hash_nested_regular_files_and_exclude_output_files(self):
        with TemporaryDirectory() as directory:
            session = Path(directory)
            nested = session / 'rosbag' / 'posegraph'
            nested.mkdir(parents=True)
            (nested / 'data.db3').write_bytes(b'bag data')
            (session / 'snapshot.tmp').write_text('transient')
            (session / 'checksums.sha256').write_text('stale')

            checksum_path = MODULE.write_checksums(session)

            self.assertEqual(
                [line.split('  ', 1)[1] for line in checksum_path.read_text().splitlines()],
                ['rosbag/posegraph/data.db3'],
            )


if __name__ == '__main__':
    unittest.main()
