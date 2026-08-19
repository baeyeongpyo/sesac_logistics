from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE / 'launch' / 'map_server.launch.py'
TF_PUBLISHER_FILE = PACKAGE / 'scripts' / 'map_visualization_tf.py'


class MapServerLaunchContractTest(unittest.TestCase):
    def test_launch_starts_only_map_server_on_controller_map_topic(self):
        self.assertTrue(LAUNCH_FILE.is_file())

        launch = LAUNCH_FILE.read_text(encoding='utf-8')

        self.assertIn("package='nav2_map_server'", launch)
        self.assertIn("executable='map_server'", launch)
        self.assertIn("'topic_name': '/controller_server/map'", launch)
        self.assertIn("'yaml_filename': map_yaml", launch)
        self.assertIn("package='nav2_lifecycle_manager'", launch)
        self.assertNotIn("package='nav2_amcl'", launch)
        self.assertNotIn("package='nav2_controller'", launch)
        self.assertNotIn("package='nav2_planner'", launch)

    def test_launch_publishes_visualization_tf_on_the_dynamic_tf_topic(self):
        """Foxglove must receive `map` frames without attaching vehicle TF trees."""
        launch = LAUNCH_FILE.read_text(encoding='utf-8')

        self.assertIn("package='mentorpi_map_server'", launch)
        self.assertIn("executable='map_visualization_tf.py'", launch)
        self.assertNotIn("executable='static_transform_publisher'", launch)
        self.assertNotIn("'--child-frame-id', 'odom'", launch)

    def test_visualization_tf_uses_wall_time_independent_of_ros_sim_time(self):
        publisher = TF_PUBLISHER_FILE.read_text(encoding='utf-8')

        self.assertIn('ClockType.STEADY_TIME', publisher)
        self.assertIn('ClockType.SYSTEM_TIME', publisher)
        self.assertIn('clock=self._steady_clock', publisher)


if __name__ == '__main__':
    unittest.main()
