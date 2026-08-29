"""Repository contract for the vehicle communication deployment bundle."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


class VehicleCommunicationBundleLayoutTest(unittest.TestCase):
    def test_bundle_uses_vehicle_communication_directory_name(self):
        """Keeping the command-only directory name would hide the Foxglove bundle role."""
        self.assertTrue((REPOSITORY_ROOT / 'vehicle_communication').is_dir())
        self.assertFalse((REPOSITORY_ROOT / 'vehicle_command_api').exists())


if __name__ == '__main__':
    unittest.main()
