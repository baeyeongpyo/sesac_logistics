from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]


class NavigationContractTest(unittest.TestCase):
    def test_nav2_configuration_uses_robot_1_frames_and_lidar(self):
        config = (PACKAGE / 'config' / 'nav2.yaml').read_text()
        for required in (
            'global_frame: map',
            'robot_base_frame: robot_1/base_footprint',
            'odom_frame_id: robot_1/odom',
            'odom_topic: /robot_1/odom',
            'topic: /robot_1/scan_raw',
            'plugin: "nav2_costmap_2d::ObstacleLayer"',
            'plugin: "nav2_costmap_2d::StaticLayer"',
            'plugin: "nav2_costmap_2d::InflationLayer"',
            'plugin: nav2_navfn_planner/NavfnPlanner',
            'width: 4',
            'height: 4',
        ):
            self.assertIn(required, config)

    def test_launch_has_mutually_exclusive_localization_and_mapping_providers(self):
        launch = (PACKAGE / 'launch' / 'navigation.launch.py').read_text()
        self.assertIn("IfCondition(PythonExpression([\"'\", mode, \"' == 'localization'\"]))", launch)
        self.assertIn("IfCondition(PythonExpression([\"'\", mode, \"' == 'mapping'\"]))", launch)
        self.assertIn("package='slam_toolbox'", launch)
        self.assertIn("navigation_launch.py", launch)

    def test_goal_bridge_and_velocity_relay_are_launched(self):
        launch = (PACKAGE / 'launch' / 'navigation.launch.py').read_text()
        for executable in ('goal_bridge.py', 'cmd_vel_relay.py'):
            self.assertIn(executable, launch)

    def test_velocity_relay_subscribes_to_nav2_launch_output(self):
        relay = (PACKAGE / 'scripts' / 'cmd_vel_relay.py').read_text()
        self.assertIn("input_topic', '/cmd_vel_nav'", relay)

    def test_goal_bridge_ignores_results_from_preempted_goals(self):
        bridge = (PACKAGE / 'scripts' / 'goal_bridge.py').read_text()
        self.assertIn('lambda future, goal_handle=handle: self.on_result(goal_handle, future)', bridge)
        self.assertIn('if goal_handle is not self.active_goal:', bridge)


if __name__ == '__main__':
    unittest.main()
