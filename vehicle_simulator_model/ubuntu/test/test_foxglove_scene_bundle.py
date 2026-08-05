import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]
SCENE_PACKAGE = BUNDLE / 'ros2_ws/src/mentorpi_foxglove_scene'


class FoxgloveSceneBundleTest(unittest.TestCase):
    def test_image_installs_the_foxglove_scene_message_package(self):
        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        self.assertIn('ros-humble-foxglove-msgs', dockerfile)

    def test_scene_package_exposes_the_publisher_executable(self):
        package = (SCENE_PACKAGE / 'package.xml').read_text()
        setup = (SCENE_PACKAGE / 'setup.py').read_text()
        self.assertIn('<exec_depend>foxglove_msgs</exec_depend>', package)
        self.assertIn('sdf_scene_publisher = mentorpi_foxglove_scene.sdf_scene_publisher:main', setup)

    def test_test_command_runs_scene_contracts_and_builds_the_package(self):
        script = (BUNDLE / 'run.sh').read_text()
        test_command = script.split('  test)', 1)[1].split('  fork-up)', 1)[0]
        self.assertIn('test_foxglove_scene_bundle.py', test_command)
        self.assertIn('mentorpi_foxglove_scene/test', test_command)
        self.assertIn('ros2 pkg prefix mentorpi_foxglove_scene', test_command)
        self.assertIn('--packages-select mentorpi_gz_sim mentorpi_foxglove_scene mentorpi_slam mentorpi_nav', test_command)

    def test_readme_describes_the_two_scene_topics(self):
        readme = (BUNDLE / 'README.md').read_text()
        self.assertIn('/warehouse_scene/static', readme)
        self.assertIn('/warehouse_scene/dynamic', readme)
        self.assertIn('/warehouse/entity_poses', readme)


if __name__ == '__main__':
    unittest.main()
