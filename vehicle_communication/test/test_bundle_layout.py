"""Repository contract for the vehicle communication deployment bundle."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


class VehicleCommunicationBundleLayoutTest(unittest.TestCase):
    def test_bundle_uses_vehicle_communication_directory_name(self):
        """Keeping the command-only directory name would hide the Foxglove bundle role."""
        self.assertTrue((REPOSITORY_ROOT / 'vehicle_communication').is_dir())
        self.assertFalse((REPOSITORY_ROOT / 'vehicle_command_api').exists())

    def test_ros2_package_installs_vehicle_command_api_executable(self):
        """The vehicle HTTP gateway must be runnable with ``ros2 run``."""
        package_root = REPOSITORY_ROOT / 'vehicle_communication'
        package_xml = package_root / 'package.xml'
        setup_py = package_root / 'setup.py'
        setup_cfg = package_root / 'setup.cfg'

        self.assertTrue(package_xml.is_file())
        self.assertTrue((package_root / 'resource' / 'vehicle_command_api').is_file())
        self.assertTrue(setup_py.is_file())
        self.assertTrue(setup_cfg.is_file())

        self.assertIn('<name>vehicle_command_api</name>', package_xml.read_text())
        self.assertIn('<build_type>ament_python</build_type>', package_xml.read_text())
        self.assertIn(
            'vehicle_command_api = vehicle_command_api:main',
            setup_py.read_text(),
        )
        self.assertIn(
            'script_dir=$base/lib/vehicle_command_api',
            setup_cfg.read_text(),
        )


if __name__ == '__main__':
    unittest.main()
