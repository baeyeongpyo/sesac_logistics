import hashlib
import tarfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
ARCHIVE = ROOT / 'promotion-shelf/20260708-163950-mentorpi/raw/mentorpi-ros2-ws-group-control-2026-07-08.tar.gz'
URDF = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro'
SDF = ROOT / 'ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro'
MESH_ROOT = ROOT / 'ros2_ws/src/mentorpi_description/meshes/mecanum'
UPSTREAM = 'mentorpi-ros2-ws-group-control-2026-07-08/src/simulations/mentorpi_description/meshes/mecanum/'


class OriginalModelTest(unittest.TestCase):
    def test_original_meshes_and_public_sensor_transforms_are_restored(self):
        root = ET.parse(URDF).getroot()
        meshes = [element.attrib['filename'] for element in root.findall('.//mesh')]
        self.assertIn('package://mentorpi_description/meshes/mecanum/base_link.STL', meshes)
        self.assertIn('package://mentorpi_description/meshes/mecanum/lidar_Link.STL', meshes)
        joints = {element.attrib['name']: element for element in root.findall('joint')}
        self.assertEqual(joints['base_footprint_to_base_link'].find('origin').attrib['xyz'], '0 0 0.07')
        self.assertEqual(joints['laser_joint'].find('origin').attrib['xyz'], '-0.012242 -0.00008533 0.092501')
        self.assertEqual(joints['depth_camera_joint'].find('origin').attrib['xyz'], '0.061376 -0.00013463 0.051154')
        with tarfile.open(ARCHIVE) as archive:
            for name in ('base_link.STL', 'wheel_lf_Link.STL', 'wheel_rf_Link.STL', 'wheel_lb_Link.STL', 'wheel_rb_Link.STL', 'lidar_Link.STL', 'cam_Link.STL'):
                expected = archive.extractfile(UPSTREAM + name).read()
                self.assertEqual(hashlib.sha256((MESH_ROOT / name).read_bytes()).digest(), hashlib.sha256(expected).digest())
        sdf_text = SDF.read_text()
        self.assertIn('<link name="base_laser"><pose>-0.012242 -0.00008533 0.162501 0 0 0</pose>', sdf_text)
        self.assertIn('<link name="depth_camera_link"><pose>0.061376 -0.00013463 0.121154 0 0 0</pose>', sdf_text)


if __name__ == '__main__':
    unittest.main()
