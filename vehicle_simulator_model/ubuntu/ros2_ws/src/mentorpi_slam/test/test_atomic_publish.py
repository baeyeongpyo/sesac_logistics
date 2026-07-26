import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest


HELPER = Path(__file__).resolve().parents[1] / 'scripts' / 'atomic_publish.py'


@unittest.skipUnless(sys.platform.startswith('linux'), 'renameat2 is a Linux-only publication primitive')
class AtomicPublishTest(unittest.TestCase):
    def run_helper(self, stage, final):
        return subprocess.run(
            [sys.executable, str(HELPER), str(stage), str(final)],
            text=True, capture_output=True, timeout=5,
        )

    def test_publishes_stage_when_target_is_absent(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            stage = root / '.inprogress' / 'session-1'
            final = root / 'session-1'
            stage.mkdir(parents=True)
            (stage / 'artifact').write_text('complete')

            result = self.run_helper(stage, final)

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(stage.exists())
            self.assertEqual((final / 'artifact').read_text(), 'complete')

    def test_target_created_immediately_before_publish_is_never_overwritten_or_nested(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            stage = root / '.inprogress' / 'session-1'
            final = root / 'session-1'
            stage.mkdir(parents=True)
            (stage / 'artifact').write_text('stage')
            final.mkdir()
            (final / 'owner').write_text('other publisher')

            result = self.run_helper(stage, final)

            self.assertNotEqual(result.returncode, 0)
            self.assertTrue(stage.is_dir())
            self.assertEqual((stage / 'artifact').read_text(), 'stage')
            self.assertEqual((final / 'owner').read_text(), 'other publisher')
            self.assertFalse((final / 'session-1').exists())


@unittest.skipIf(sys.platform.startswith('linux'), 'Linux exercises the actual renameat2 path above')
class AtomicPublishUnsupportedPlatformTest(unittest.TestCase):
    def test_unsupported_platform_retains_stage_and_never_creates_final(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            stage = root / '.inprogress' / 'session-1'
            final = root / 'session-1'
            stage.mkdir(parents=True)
            (stage / 'artifact').write_text('stage')

            result = subprocess.run(
                [sys.executable, str(HELPER), str(stage), str(final)],
                text=True, capture_output=True, timeout=5,
            )

            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertTrue(stage.is_dir())
            self.assertFalse(final.exists())


if __name__ == '__main__':
    unittest.main()
