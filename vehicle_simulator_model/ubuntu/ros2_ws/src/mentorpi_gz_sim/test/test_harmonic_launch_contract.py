import ast
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch'
CMAKE = PACKAGE / 'CMakeLists.txt'
PACKAGE_XML = PACKAGE / 'package.xml'


class HarmonicLaunchContractTest(unittest.TestCase):
    def test_server_preserves_verbosity_as_a_launch_substitution(self):
        tree = ast.parse((LAUNCH / 'gazebo_server.launch.py').read_text())
        assignments = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == 'gz_args' for target in node.targets)
        ]

        self.assertEqual(len(assignments), 1)
        value = assignments[0].value
        self.assertIsInstance(value, (ast.List, ast.Tuple))
        self.assertTrue(any(
            isinstance(element, ast.Call)
            and isinstance(element.func, ast.Name)
            and element.func.id == 'LaunchConfiguration'
            and len(element.args) == 1
            and isinstance(element.args[0], ast.Constant)
            and element.args[0].value == 'verbosity'
            for element in value.elts
        ))

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

    def test_contract_test_is_registered_with_ctest(self):
        cmake = CMAKE.read_text()
        package_xml = PACKAGE_XML.read_text()
        self.assertIn('find_package(ament_cmake_pytest REQUIRED)', cmake)
        self.assertIn('ament_add_pytest_test(test_harmonic_launch_contract', cmake)
        self.assertIn('test/test_harmonic_launch_contract.py', cmake)
        self.assertIn('<test_depend>ament_cmake_pytest</test_depend>', package_xml)
