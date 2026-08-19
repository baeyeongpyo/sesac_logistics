import unittest
from pathlib import Path

import yaml


BUNDLE = Path(__file__).resolve().parents[1]


class MapImportBundleTest(unittest.TestCase):
    def test_map_importer_reads_a_dedicated_volume_and_writes_only_runtime_sessions(self):
        """External maps must be isolated from Nav2's checksum-verified runtime volume."""
        compose = yaml.safe_load((BUNDLE / 'compose.yaml').read_text())

        self.assertIn('map-data-init', compose['services'])
        initializer = compose['services']['map-data-init']
        self.assertEqual(initializer['profiles'], ['map-import'])
        self.assertEqual(initializer['user'], '0:0')
        self.assertEqual(initializer['command'], 'chown 1000:1000 /slam-data')
        self.assertIn('map-importer', compose['services'])
        service = compose['services']['map-importer']
        self.assertEqual(service['profiles'], ['map-import'])
        self.assertEqual(
            service['command'],
            '/opt/mentorpi_ws/install/mentorpi_nav/lib/mentorpi_nav/map_import.py',
        )
        mounts = {mount['target']: mount for mount in service['volumes']}
        self.assertTrue(mounts['/map-import']['read_only'])
        self.assertFalse(mounts['/slam-data'].get('read_only', False))
        self.assertEqual(
            service['depends_on']['map-data-init']['condition'],
            'service_completed_successfully',
        )
        self.assertEqual(
            compose['volumes']['map-import']['name'],
            '${MAP_IMPORT_VOLUME_NAME-mentorpi-map-import}',
        )

    def test_launcher_imports_the_explicit_map_id_without_starting_simulation(self):
        """Choosing a map version must run only the one-shot importer with that version."""
        script = (BUNDLE / 'run.sh').read_text()

        self.assertIn('map-import <id>', script)
        self.assertIn('map-import)', script)
        self.assertIn('MAP_IMPORT_ID="${RUN_COMMAND[1]}"', script)
        self.assertIn('--profile map-import run --rm map-importer', script)
        self.assertNotIn('require_healthy_sim_adapter\n    export MAP_IMPORT_ID', script)

    def test_nav_package_installs_import_command_and_its_contract_test(self):
        """The container image must contain the importer and exercise its behavior in CTest."""
        cmake = (BUNDLE / 'ros2_ws/src/mentorpi_nav/CMakeLists.txt').read_text()

        self.assertIn('scripts/map_import.py', cmake)
        self.assertIn('ament_add_pytest_test(test_map_import', cmake)


if __name__ == '__main__':
    unittest.main()
