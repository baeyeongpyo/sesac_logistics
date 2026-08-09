import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SCENE_PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = SCENE_PACKAGE.parent
SIM_PACKAGE = WORKSPACE_SRC / 'mentorpi_gz_sim'
WORLD = SIM_PACKAGE / 'worlds/warehouse.sdf'
LAUNCH = SIM_PACKAGE / 'launch/sim_adapter.launch.py'
PUBLISHER = SCENE_PACKAGE / 'mentorpi_foxglove_scene/sdf_scene_publisher.py'


class SceneContractTest(unittest.TestCase):
    def test_world_uses_builtin_dynamic_pose_stream_not_model_only_pose_publisher(self):
        world = ET.parse(WORLD).getroot().find('world')
        plugin = world.find("plugin[@name='gz::sim::systems::PosePublisher']")

        self.assertIsNone(plugin)

    def test_scene_publisher_uses_named_ground_truth_streams_for_each_robot(self):
        text = PUBLISHER.read_text()

        self.assertIn("for robot_id in ('robot_1', 'robot_2')", text)
        self.assertIn("f'/{robot_id}/ground_truth/pose'", text)
        self.assertIn('self._poses.update(_poses_from_tf(message))', text)

    def test_adapter_launch_starts_scene_publisher_without_robot_owned_warehouse_tf(self):
        text = LAUNCH.read_text()

        self.assertIn("package='mentorpi_foxglove_scene'", text)
        self.assertIn("executable='sdf_scene_publisher'", text)
        self.assertIn("'frame_id': 'warehouse'", text)
        self.assertNotIn("name='warehouse_frame'", text)
        self.assertNotIn("'--frame-id', 'robot_1/odom'", text)
        self.assertNotIn("'--child-frame-id', 'warehouse'", text)
        self.assertNotIn("warehouse_scene_bridge.yaml", text)

    def test_scene_publisher_defaults_to_the_shared_warehouse_frame(self):
        text = PUBLISHER.read_text()

        self.assertIn("self.declare_parameter('frame_id', 'warehouse')", text)

    def test_static_scene_is_republished_for_late_foxglove_subscribers(self):
        text = PUBLISHER.read_text()

        self.assertIn('self._publish_static()', text)
        self.assertIn('self.create_timer(1.0, self._publish_static)', text)
        self.assertIn('def _publish_static(self) -> None:', text)


if __name__ == '__main__':
    unittest.main()
