import math
import tempfile
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(PACKAGE))

from mentorpi_foxglove_scene.sdf_scene import (
    Cube,
    Cylinder,
    SceneParseError,
    Sphere,
    static_scene_from_sdf,
)


class SdfSceneTest(unittest.TestCase):
    def write_file(self, directory, relative_path, content):
        path = directory / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    def test_converts_inline_visual_geometries_to_primitives(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            world = self.write_file(root, 'warehouse.sdf', '''
<sdf version="1.8"><world name="warehouse"><model name="fixtures"><link name="link">
  <visual name="box"><pose>1 2 3 0 0 0</pose><geometry><box><size>2 3 4</size></box></geometry><material><diffuse>0.1 0.2 0.3 1</diffuse></material></visual>
  <visual name="cylinder"><geometry><cylinder><radius>0.5</radius><length>2</length></cylinder></geometry></visual>
  <visual name="sphere"><geometry><sphere><radius>0.4</radius></sphere></geometry></visual>
  <visual name="floor"><geometry><plane><normal>0 0 1</normal><size>6 8</size></plane></geometry></visual>
</link></model></world></sdf>''')

            entities = static_scene_from_sdf(world, root / 'models')
            primitives = {primitive.id: primitive for entity in entities for primitive in entity.primitives}

            self.assertIsInstance(primitives['fixtures/link/box'], Cube)
            self.assertEqual(primitives['fixtures/link/box'].size, (2.0, 3.0, 4.0))
            self.assertEqual(primitives['fixtures/link/box'].pose.position, (1.0, 2.0, 3.0))
            self.assertEqual(primitives['fixtures/link/box'].color, (0.1, 0.2, 0.3, 1.0))
            self.assertIsInstance(primitives['fixtures/link/cylinder'], Cylinder)
            self.assertEqual(primitives['fixtures/link/cylinder'].radius, 0.5)
            self.assertIsInstance(primitives['fixtures/link/sphere'], Sphere)
            self.assertEqual(primitives['fixtures/link/sphere'].radius, 0.4)
            self.assertEqual(primitives['fixtures/link/floor'].size, (6.0, 8.0, 0.01))

    def test_expands_warehouse_model_include_with_composed_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            world = self.write_file(root, 'warehouse.sdf', '''
<sdf version="1.8"><world name="warehouse"><include>
  <uri>model://warehouse_fixture</uri><name>fixture_a</name><pose>10 20 0 0 0 1.57079632679</pose>
</include></world></sdf>''')
            self.write_file(root, 'models/warehouse_fixture/model.sdf', '''
<sdf version="1.8"><model name="warehouse_fixture"><link name="body">
  <visual name="panel"><pose>1 0 2 0 0 0</pose><geometry><box><size>1 2 3</size></box></geometry></visual>
</link></model></sdf>''')

            entities = static_scene_from_sdf(world, root / 'models')
            panel = entities[0].primitives[0]

            self.assertEqual(panel.id, 'fixture_a/body/panel')
            self.assertAlmostEqual(panel.pose.position[0], 10.0)
            self.assertAlmostEqual(panel.pose.position[1], 21.0)
            self.assertEqual(panel.pose.position[2], 2.0)
            self.assertAlmostEqual(panel.pose.yaw, math.pi / 2)

    def test_rejects_model_uri_outside_the_warehouse_models_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            world = self.write_file(root, 'warehouse.sdf', '''
<sdf version="1.8"><world name="warehouse"><include><uri>model://../../outside</uri></include></world></sdf>''')

            with self.assertRaises(SceneParseError):
                static_scene_from_sdf(world, root / 'models')

    def test_skips_unsupported_mesh_visual_without_dropping_supported_visuals(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            world = self.write_file(root, 'warehouse.sdf', '''
<sdf version="1.8"><world name="warehouse"><model name="fixture"><link name="link">
  <visual name="mesh"><geometry><mesh><uri>model://mesh.dae</uri></mesh></geometry></visual>
  <visual name="box"><geometry><box><size>1 1 1</size></box></geometry></visual>
</link></model></world></sdf>''')

            entities = static_scene_from_sdf(world, root / 'models')

            self.assertEqual([p.id for p in entities[0].primitives], ['fixture/link/box'])


if __name__ == '__main__':
    unittest.main()
