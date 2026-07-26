import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch'


class HarmonicLaunchContractTest(unittest.TestCase):
    def test_server_launch_only_starts_gazebo(self):
        text = (LAUNCH / 'gazebo_server.launch.py').read_text()
        self.assertIn("'-r -s --headless-rendering", text)
        self.assertIn("get_package_share_directory('ros_gz_sim')", text)
        self.assertNotIn('robot_state_publisher', text)
        self.assertNotIn("executable='create'", text)

    def test_adapter_launch_owns_spawn_and_bridges(self):
        text = (LAUNCH / 'sim_adapter.launch.py').read_text()
        for token in ('robot_state_publisher', "executable='create'", 'parameter_bridge',
                      'image_bridge', 'gz_pose_to_odom.py'):
            self.assertIn(token, text)
        self.assertNotIn('gz_sim.launch.py', text)

    def test_combined_launch_includes_both_boundaries(self):
        text = (LAUNCH / 'two_robot_sim.launch.py').read_text()
        self.assertIn('gazebo_server.launch.py', text)
        self.assertIn('sim_adapter.launch.py', text)
