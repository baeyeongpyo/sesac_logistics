import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]


class NavigationBundleTest(unittest.TestCase):
    def test_image_installs_nav2_runtime(self):
        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        self.assertIn('ros-humble-navigation2', dockerfile)
        self.assertIn('ros-humble-nav2-bringup', dockerfile)

    def test_navigation_service_uses_read_only_map_volume(self):
        compose = (BUNDLE / 'compose.yaml').read_text()
        service = compose.split('  navigation:', 1)[1].split('\nnetworks:', 1)[0]
        for required in (
            'profiles: [navigation]',
            'SLAM_DATA_ROOT: /slam-data',
            'NAV_SESSION_ID:',
            'run_navigation.sh',
            'condition: service_healthy',
            'read_only: true',
        ):
            self.assertIn(required, service)

    def test_launcher_exposes_auto_map_selection_lifecycle(self):
        script = (BUNDLE / 'run.sh').read_text()
        for required in (
            'nav-up auto [id]',
            'nav-down',
            'nav-status',
            '--profile navigation up -d --wait',
            'unset NAV_SESSION_ID',
            'NAV_SESSION_ID="${RUN_COMMAND[2]}"',
            'ros2 topic list --no-daemon | grep -E "^/(cmd_vel_nav|map|move_base_simple/goal)$"',
        ):
            self.assertIn(required, script)

    def test_nav_package_registers_its_contract_tests(self):
        package = BUNDLE / 'ros2_ws/src/mentorpi_nav'
        cmake = (package / 'CMakeLists.txt').read_text()
        for name in ('test_map_session', 'test_navigation_contract'):
            self.assertIn(name, cmake)
        self.assertIn('goal_bridge.py', cmake)
        self.assertIn('cmd_vel_relay.py', cmake)


if __name__ == '__main__':
    unittest.main()
