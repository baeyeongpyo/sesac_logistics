import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


class VehicleNavigationContractTest(unittest.TestCase):
    def test_launch_scopes_goal_relay_and_mux_to_one_vehicle_argument(self):
        launch = (PACKAGE / 'launch' / 'vehicle_navigation.launch.py').read_text()

        self.assertIn("DeclareLaunchArgument('robot_id')", launch)
        self.assertIn("executable='goal_bridge.py'", launch)
        self.assertIn("executable='cmd_vel_relay.py'", launch)
        self.assertIn("executable='cmd_vel_mux.py'", launch)
        self.assertIn("f'/{robot_id}/manual/cmd_vel'", launch)
        self.assertIn("f'/{robot_id}/controller/cmd_vel'", launch)
        self.assertNotIn("robot_stack('robot_1'", launch)
        self.assertNotIn("robot_stack('robot_2'", launch)


if __name__ == '__main__':
    unittest.main()
