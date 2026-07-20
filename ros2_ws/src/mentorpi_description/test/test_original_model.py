import hashlib
import struct
import tarfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
ARCHIVE = ROOT / 'promotion-shelf/20260708-163950-mentorpi/raw/mentorpi-ros2-ws-group-control-2026-07-08.tar.gz'
UPSTREAM = 'mentorpi-ros2-ws-group-control-2026-07-08/src/simulations/mentorpi_description/urdf/mecanum.xacro'
MECANUM = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mecanum.xacro'
MECANUM_URDF = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mecanum.urdf'
FORKLIFT = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mecanum_forklift.xacro'
FORKLIFT_URDF = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mecanum_forklift.urdf'
CAMERA_CONFIG = ROOT / 'ros2_ws/src/mentorpi_description/urdf/forklift_camera_config.xacro'
CAMERA_MESH = ROOT / 'ros2_ws/src/mentorpi_description/meshes/mecanum/cam_Link.STL'
LEGACY_MODEL = ROOT / 'ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro'
DESCRIPTION_LAUNCH = ROOT / 'ros2_ws/src/mentorpi_description/launch/description.launch.py'
SIM_LAUNCH = ROOT / 'ros2_ws/src/mentorpi_gz_sim/launch/two_robot_sim.launch.py'
SDF = ROOT / 'ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro'
BRIDGE_CONFIGS = [
    ROOT / 'ros2_ws/src/mentorpi_gz_sim/config/robot_1_bridge.yaml',
    ROOT / 'ros2_ws/src/mentorpi_gz_sim/config/robot_2_bridge.yaml',
]
GAZEBO_PACKAGE_XML = ROOT / 'ros2_ws/src/mentorpi_gz_sim/package.xml'


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
        self.assertIn('mecanum_forklift.xacro', description_launch)
        self.assertIn('mecanum_forklift.xacro', simulation_launch)
        self.assertNotIn("' robot_name:='", description_launch)
        self.assertNotIn("' frame_prefix:='", description_launch)
        self.assertNotIn("mappings={'robot_name': name, 'frame_prefix': f'{name}/'}", simulation_launch)

    def test_gazebo_matches_forklift_sensor_and_drive_contract(self):
        root = ET.parse(SDF).getroot()
        model = root.find('model')
        links = {link.attrib['name']: link for link in model.findall('link')}
        joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

        self.assertEqual(links['lidar_frame'].findtext('pose'), '-0.012242 -0.00008533 0.162501 0 0 0')
        self.assertNotIn('relative_to', links['lidar_frame'].find('pose').attrib)
        self.assertEqual(
            links['depth_cam'].findtext('pose'),
            '${camera_mount_x} ${camera_mount_y} ${camera_mount_z} 0 ${camera_pitch} 0',
        )
        self.assertEqual(links['depth_cam'].find('pose').attrib['relative_to'], 'fork_mast')
        self.assertEqual(links['imu_link'].findtext('pose'), '0 0 0.07 0 0 0')
        self.assertEqual(joints['lidar_Joint'].findtext('child'), 'lidar_frame')
        self.assertEqual(joints['lidar_Joint'].findtext('parent'), 'base_footprint')
        self.assertEqual(joints['cam_Joint'].findtext('child'), 'depth_cam')
        self.assertEqual(joints['cam_Joint'].findtext('parent'), 'fork_mast')

        text = SDF.read_text()
        self.assertIn('<xacro:wheel name="front_left" x="0.067052" y="0.07591" z="0.051592"', text)
        self.assertIn('<xacro:wheel name="front_right" x="0.067052" y="-0.07591" z="0.051592"', text)
        self.assertIn('<xacro:wheel name="back_left" x="-0.06764" y="0.07591" z="0.051586"', text)
        self.assertIn('<xacro:wheel name="back_right" x="-0.067621" y="-0.07591" z="0.051592"', text)
        self.assertIn('<wheelbase>0.13468</wheelbase><wheel_separation>0.15182</wheel_separation>', text)
        self.assertIn('<gz_frame_id>${robot_name}/lidar_frame</gz_frame_id>', text)
        self.assertIn('<gz_frame_id>${robot_name}/depth_cam</gz_frame_id>', text)

    def test_forklift_model_has_a_lifted_carriage_and_configurable_camera(self):
        root = ET.parse(FORKLIFT).getroot()
        links = {link.attrib['name']: link for link in root.findall('link')}
        joints = {joint.attrib['name']: joint for joint in root.findall('joint')}

        self.assertTrue(FORKLIFT_URDF.is_file())
        self.assertFalse(FORKLIFT_URDF.is_symlink())
        self.assertNotIn('${camera_', FORKLIFT_URDF.read_text())
        preview_root = ET.parse(FORKLIFT_URDF).getroot()
        preview_joints = {joint.attrib['name']: joint for joint in preview_root.findall('joint')}
        preview_camera_origin = preview_joints['cam_Joint'].find('origin').attrib
        self.assertEqual(preview_camera_origin['xyz'], '0.025 0.0 0.1496473521750886')
        self.assertEqual(preview_camera_origin['rpy'], '0 0.0 0')
        mesh = CAMERA_MESH.read_bytes()
        triangle_count = struct.unpack_from('<I', mesh, 80)[0]
        camera_mesh_top_z = max(
            struct.unpack_from('<f', mesh, 84 + triangle * 50 + 12 + vertex * 12 + 8)[0]
            for triangle in range(triangle_count)
            for vertex in range(3)
        )
        mast_height = float(links['fork_mast'].find('visual/geometry/box').attrib['size'].split()[2])
        self.assertAlmostEqual(float(preview_camera_origin['xyz'].split()[2]) + camera_mesh_top_z,
                               mast_height / 2, places=9)
        self.assertTrue({'fork_mast', 'fork_carriage', 'fork_left', 'fork_right',
                         'fork_left_tip', 'fork_right_tip'} <= links.keys())
        self.assertEqual(joints['fork_carriage_joint'].attrib['type'], 'prismatic')
        self.assertEqual(joints['fork_carriage_joint'].find('axis').attrib['xyz'], '0 0 1')
        limit = joints['fork_carriage_joint'].find('limit').attrib
        self.assertEqual(limit['lower'], '0.0')
        self.assertEqual(limit['upper'], '0.11')
        self.assertEqual(joints['cam_Joint'].find('parent').attrib['link'], 'fork_mast')
        camera_joint_origin = joints['cam_Joint'].find('origin').attrib
        self.assertEqual(camera_joint_origin['xyz'], '${camera_mount_x} ${camera_mount_y} ${camera_mount_z}')
        self.assertEqual(camera_joint_origin['rpy'], '0 ${camera_pitch} 0')
        camera_config = CAMERA_CONFIG.read_text()
        self.assertIn('name="camera_pitch" value="0.0"', camera_config)
        self.assertIn('name="camera_mount_x" value="0.025"', camera_config)
        self.assertIn('name="camera_mount_y" value="0.0"', camera_config)
        self.assertIn('name="camera_mesh_top_z" value="0.0003526478249114007"', camera_config)
        self.assertIn('name="camera_mount_z" value="${fork_mast_height / 2 - camera_mesh_top_z}"', camera_config)
        self.assertEqual(joints['lidar_Joint'].find('parent').attrib['link'], 'base_link')
        self.assertEqual(joints['lidar_Joint'].find('origin').attrib['xyz'], '-0.012242 -8.533E-05 0.092501')

        sdf_root = ET.parse(SDF).getroot()
        model = sdf_root.find('model')
        sdf_links = {link.attrib['name']: link for link in model.findall('link')}
        sdf_joints = {joint.attrib['name']: joint for joint in model.findall('joint')}
        self.assertTrue({'fork_mast', 'fork_carriage', 'fork_left', 'fork_right'} <= sdf_links.keys())
        self.assertEqual(sdf_joints['fork_carriage_joint'].attrib['type'], 'prismatic')
        self.assertEqual(sdf_joints['fork_carriage_joint'].findtext('axis/xyz'), '0 0 1')
        self.assertEqual(sdf_joints['fork_carriage_joint'].findtext('axis/limit/lower'), '0.0')
        self.assertEqual(sdf_joints['fork_carriage_joint'].findtext('axis/limit/upper'), '0.11')
        sdf_text = SDF.read_text()
        self.assertIn('<topic>/${robot_name}/fork/command</topic>', sdf_text)
        self.assertIn('<gz_frame_id>${robot_name}/depth_cam</gz_frame_id>', sdf_text)
        for robot_name, bridge_path in zip(('robot_1', 'robot_2'), BRIDGE_CONFIGS):
            bridge_text = bridge_path.read_text()
            self.assertIn(
                f'ros_topic_name: /{robot_name}/fork/command, '
                f'gz_topic_name: /{robot_name}/fork/command, '
                'ros_type_name: std_msgs/msg/Float64, '
                'gz_type_name: ignition.msgs.Double, direction: ROS_TO_GZ',
                bridge_text,
            )
        self.assertIn('<exec_depend>std_msgs</exec_depend>', GAZEBO_PACKAGE_XML.read_text())


if __name__ == '__main__':
    unittest.main()
