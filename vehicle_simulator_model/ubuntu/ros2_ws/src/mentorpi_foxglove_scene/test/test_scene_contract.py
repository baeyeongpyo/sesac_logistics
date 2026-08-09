import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SCENE_PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = SCENE_PACKAGE.parent
SIM_PACKAGE = WORKSPACE_SRC / 'mentorpi_gz_sim'
WORLD = SIM_PACKAGE / 'worlds/warehouse.sdf'
BRIDGE = SIM_PACKAGE / 'config/warehouse_scene_bridge.yaml'
LAUNCH = SIM_PACKAGE / 'launch/sim_adapter.launch.py'
PUBLISHER = SCENE_PACKAGE / 'mentorpi_foxglove_scene/sdf_scene_publisher.py'


class SceneContractTest(unittest.TestCase):
    def test_world_publishes_model_poses_on_a_dedicated_pose_vector_topic(self):
        world = ET.parse(WORLD).getroot().find('world')
        plugin = world.find("plugin[@name='gz::sim::systems::PosePublisher']")

        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.attrib['filename'], 'gz-sim-pose-publisher-system')
        self.assertEqual(plugin.findtext('topic'), '/warehouse/entity_poses')
        self.assertEqual(plugin.findtext('publish_model_pose'), 'true')
        self.assertEqual(plugin.findtext('use_pose_vector_msg'), 'true')
        self.assertEqual(plugin.findtext('update_frequency'), '10')

    def test_pose_vector_bridge_is_kept_separate_from_tf(self):
        text = BRIDGE.read_text()

        self.assertIn('ros_topic_name: /warehouse/entity_poses', text)
        self.assertIn('gz_topic_name: /warehouse/entity_poses', text)
        self.assertIn('ros_type_name: tf2_msgs/msg/TFMessage', text)
        self.assertIn('gz_type_name: gz.msgs.Pose_V', text)
        self.assertIn('direction: GZ_TO_ROS', text)

    def test_adapter_launch_starts_scene_publisher_without_robot_owned_warehouse_tf(self):
        text = LAUNCH.read_text()

        self.assertIn("warehouse_scene_bridge.yaml", text)
        self.assertIn("package='mentorpi_foxglove_scene'", text)
        self.assertIn("executable='sdf_scene_publisher'", text)
        self.assertIn("'frame_id': 'warehouse'", text)
        self.assertNotIn("name='warehouse_frame'", text)
        self.assertNotIn("'--frame-id', 'robot_1/odom'", text)
        self.assertNotIn("'--child-frame-id', 'warehouse'", text)

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
