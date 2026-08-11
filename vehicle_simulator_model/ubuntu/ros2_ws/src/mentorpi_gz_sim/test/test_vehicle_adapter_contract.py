import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


class VehicleAdapterContractTest(unittest.TestCase):
    def test_single_vehicle_adapter_exposes_vehicle_and_pose_arguments(self):
        launch = (PACKAGE / 'launch' / 'vehicle_adapter.launch.py').read_text()

        for argument in ('robot_id', 'x', 'y', 'z', 'yaw', 'bridge_config'):
            self.assertIn(f"DeclareLaunchArgument('{argument}'", launch)
        self.assertIn("executable='create'", launch)
        self.assertIn("executable='parameter_bridge'", launch)
        self.assertIn("executable='gz_pose_to_odom.py'", launch)
        self.assertNotIn("_robot_nodes('robot_1'", launch)
        self.assertNotIn("_robot_nodes('robot_2'", launch)

    def test_generic_bridge_template_routes_clock_and_one_vehicle_namespace(self):
        template = (PACKAGE / 'config' / 'vehicle_bridge.yaml.in').read_text()

        self.assertIn('ros_topic_name: /clock', template)
        self.assertIn('/__ROBOT_ID__/controller/cmd_vel', template)
        self.assertIn('/__ROBOT_ID__/ground_truth/pose', template)
        self.assertNotIn('/robot_1/', template)
        self.assertNotIn('/robot_2/', template)


if __name__ == '__main__':
    unittest.main()
