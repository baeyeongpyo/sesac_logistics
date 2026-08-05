import unittest
from pathlib import Path
import sys


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from mentorpi_foxglove_scene.dynamic_scene import DynamicScene
from mentorpi_foxglove_scene.sdf_scene import Cube, Pose


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    import math
    return Pose((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


class DynamicSceneTest(unittest.TestCase):
    def test_robot_pose_creates_chassis_mast_and_two_forks(self):
        snapshot = DynamicScene().snapshot({'robot_1': pose(1.0, 2.0)})

        self.assertEqual([entity.id for entity in snapshot.entities], ['robot_1'])
        robot = snapshot.entities[0]
        self.assertEqual([primitive.id for primitive in robot.primitives], [
            'robot_1/chassis', 'robot_1/mast', 'robot_1/fork_left', 'robot_1/fork_right',
        ])
        self.assertTrue(all(isinstance(primitive, Cube) for primitive in robot.primitives))
        self.assertEqual(robot.primitives[0].size, (0.30, 0.20, 0.12))
        self.assertEqual(robot.primitives[0].pose.position, (1.0, 2.0, 0.06))

    def test_pallet_payload_is_rendered_only_when_payload_model_exists(self):
        scene = DynamicScene()
        snapshot = scene.snapshot({
            'pallet_01': pose(-1.0, 0.6),
            'pallet_01_payload': pose(-1.0, 0.6, 0.03),
        })

        self.assertEqual([entity.id for entity in snapshot.entities], ['pallet_01', 'pallet_01_payload'])
        self.assertEqual(len(snapshot.entities[0].primitives), 4)
        payload = snapshot.entities[1].primitives
        self.assertEqual(len(payload), 2)
        self.assertEqual(payload[0].color, (0.55, 0.25, 0.75, 1.0))

    def test_missing_payload_is_reported_as_a_deleted_entity(self):
        scene = DynamicScene()
        scene.snapshot({'pallet_01': pose(), 'pallet_01_payload': pose(z=0.03)})

        snapshot = scene.snapshot({'pallet_01': pose()})

        self.assertEqual([entity.id for entity in snapshot.entities], ['pallet_01'])
        self.assertEqual(snapshot.deleted_ids, ('pallet_01_payload',))

    def test_unknown_gazebo_entities_are_ignored(self):
        snapshot = DynamicScene().snapshot({'conveyor_joint': pose(), 'ground': pose()})

        self.assertEqual(snapshot.entities, ())
        self.assertEqual(snapshot.deleted_ids, ())


if __name__ == '__main__':
    unittest.main()
