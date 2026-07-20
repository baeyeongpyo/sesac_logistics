import hashlib
import tarfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
ARCHIVE = ROOT / 'promotion-shelf/20260708-163950-mentorpi/raw/mentorpi-ros2-ws-group-control-2026-07-08.tar.gz'
UPSTREAM = 'mentorpi-ros2-ws-group-control-2026-07-08/src/simulations/mentorpi_description/urdf/mecanum.xacro'
MECANUM = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mecanum.xacro'
MECANUM_URDF = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mecanum.urdf'
LEGACY_MODEL = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro'
DESCRIPTION_LAUNCH = ROOT / 'ros2_ws/src/mentorpi_description/launch/description.launch.py'
SIM_LAUNCH = ROOT / 'ros2_ws/src/mentorpi_gz_sim/launch/two_robot_sim.launch.py'
SDF = ROOT / 'ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro'


class MecanumSourceModelTest(unittest.TestCase):
    def test_description_uses_the_exact_archived_mecanum_model(self):
        with tarfile.open(ARCHIVE) as archive:
            expected = archive.extractfile(UPSTREAM).read()

        self.assertTrue(MECANUM.is_file())
        self.assertEqual(hashlib.sha256(MECANUM.read_bytes()).digest(), hashlib.sha256(expected).digest())
        self.assertTrue(MECANUM_URDF.is_symlink())
        self.assertEqual(MECANUM_URDF.readlink(), Path('mecanum.xacro'))
        self.assertFalse(LEGACY_MODEL.exists())
        description_launch = DESCRIPTION_LAUNCH.read_text()
        simulation_launch = SIM_LAUNCH.read_text()
        self.assertIn('mecanum.xacro', description_launch)
        self.assertIn('mecanum.xacro', simulation_launch)
        self.assertNotIn("' robot_name:='", description_launch)
        self.assertNotIn("' frame_prefix:='", description_launch)
        self.assertNotIn("mappings={'robot_name': name, 'frame_prefix': f'{name}/'}", simulation_launch)

    def test_gazebo_matches_source_mecanum_sensor_and_drive_contract(self):
        root = ET.parse(SDF).getroot()
        model = root.find('model')
        links = {link.attrib['name']: link for link in model.findall('link')}
        joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

        self.assertEqual(links['lidar_frame'].findtext('pose'), '-0.012242 -0.00008533 0.162501 0 0 0')
        self.assertEqual(links['depth_cam'].findtext('pose'), '0.061376 -0.00013463 0.121154 0 0 0')
        self.assertEqual(links['imu_link'].findtext('pose'), '0 0 0.07 0 0 0')
        self.assertEqual(joints['lidar_Joint'].findtext('child'), 'lidar_frame')
        self.assertEqual(joints['cam_Joint'].findtext('child'), 'depth_cam')

        text = SDF.read_text()
        self.assertIn('<xacro:wheel name="front_left" x="0.067052" y="0.07591" z="0.051592"', text)
        self.assertIn('<xacro:wheel name="front_right" x="0.067052" y="-0.07591" z="0.051592"', text)
        self.assertIn('<xacro:wheel name="back_left" x="-0.06764" y="0.07591" z="0.051586"', text)
        self.assertIn('<xacro:wheel name="back_right" x="-0.067621" y="-0.07591" z="0.051592"', text)
        self.assertIn('<wheelbase>0.13468</wheelbase><wheel_separation>0.15182</wheel_separation>', text)
        self.assertIn('<gz_frame_id>${robot_name}/lidar_frame</gz_frame_id>', text)
        self.assertIn('<gz_frame_id>${robot_name}/depth_cam</gz_frame_id>', text)


if __name__ == '__main__':
    unittest.main()
