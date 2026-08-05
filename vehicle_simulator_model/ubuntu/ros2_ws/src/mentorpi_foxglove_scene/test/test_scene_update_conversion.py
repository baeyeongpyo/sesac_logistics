import unittest
from pathlib import Path
import sys


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from mentorpi_foxglove_scene.sdf_scene import Cube, Cylinder, Pose, SceneEntity, Sphere
from mentorpi_foxglove_scene.sdf_scene_publisher import RosMessageTypes, scene_update_from_entities


class Message:
    def __init__(self):
        pass

    def __getattr__(self, name):
        nested = Message()
        setattr(self, name, nested)
        return nested


class Deletion(Message):
    MATCHING_ID = 0


class SceneUpdateConversionTest(unittest.TestCase):
    def setUp(self):
        self.messages = RosMessageTypes(
            scene_update=Message,
            scene_entity=Message,
            scene_entity_deletion=Deletion,
            cube=Message,
            cylinder=Message,
            sphere=Message,
            pose=Message,
            vector3=Message,
            color=Message,
        )
        self.pose = Pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.5, 0.8660254))

    def test_serializes_each_primitive_in_its_foxglove_field(self):
        entity = SceneEntity('fixture', (
            Cube('cube', self.pose, (0.1, 0.2, 0.3, 1.0), (2.0, 3.0, 4.0)),
            Cylinder('cylinder', self.pose, (0.4, 0.5, 0.6, 1.0), 0.5, 2.0),
            Sphere('sphere', self.pose, (0.7, 0.8, 0.9, 1.0), 0.4),
        ))

        update = scene_update_from_entities((entity,), (), 'robot_1/odom', 'stamp', self.messages)

        self.assertEqual(update.entities[0].id, 'fixture')
        self.assertEqual(update.entities[0].frame_id, 'robot_1/odom')
        self.assertEqual(update.entities[0].timestamp, 'stamp')
        self.assertEqual(update.entities[0].cubes[0].size.x, 2.0)
        self.assertEqual(update.entities[0].cylinders[0].size.x, 1.0)
        self.assertEqual(update.entities[0].cylinders[0].size.z, 2.0)
        self.assertEqual(update.entities[0].cylinders[0].bottom_scale, 1.0)
        self.assertEqual(update.entities[0].spheres[0].size.z, 0.8)
        self.assertEqual(update.entities[0].spheres[0].color.b, 0.9)

    def test_serializes_matching_id_deletions_with_the_current_timestamp(self):
        update = scene_update_from_entities((), ('pallet_01_payload',), 'robot_1/odom', 'stamp', self.messages)

        self.assertEqual(len(update.deletions), 1)
        self.assertEqual(update.deletions[0].type, Deletion.MATCHING_ID)
        self.assertEqual(update.deletions[0].id, 'pallet_01_payload')
        self.assertEqual(update.deletions[0].timestamp, 'stamp')


if __name__ == '__main__':
    unittest.main()
