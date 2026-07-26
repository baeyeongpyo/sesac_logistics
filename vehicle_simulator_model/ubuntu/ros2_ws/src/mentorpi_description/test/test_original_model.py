import hashlib
import struct
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / 'src'
MECANUM = SOURCE / 'mentorpi_description/urdf/mecanum.xacro'
MECANUM_URDF = SOURCE / 'mentorpi_description/urdf/mecanum.urdf'
FORKLIFT = SOURCE / 'mentorpi_description/urdf/mecanum_forklift.xacro'
FORKLIFT_URDF = SOURCE / 'mentorpi_description/urdf/mecanum_forklift.urdf'
CAMERA_CONFIG = SOURCE / 'mentorpi_description/urdf/forklift_camera_config.xacro'
CAMERA_MESH = SOURCE / 'mentorpi_description/meshes/mecanum/cam_Link.STL'
LIDAR_MESH = SOURCE / 'mentorpi_description/meshes/mecanum/lidar_Link.STL'
LEGACY_MODEL = SOURCE / 'mentorpi_description/urdf/mentorpi_m1.urdf.xacro'
DESCRIPTION_LAUNCH = SOURCE / 'mentorpi_description/launch/description.launch.py'
SIM_ADAPTER_LAUNCH = SOURCE / 'mentorpi_gz_sim/launch/sim_adapter.launch.py'
SDF = SOURCE / 'mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro'
BRIDGE_CONFIGS = [
    SOURCE / 'mentorpi_gz_sim/config/robot_1_bridge.yaml',
    SOURCE / 'mentorpi_gz_sim/config/robot_2_bridge.yaml',
]
GAZEBO_PACKAGE_XML = SOURCE / 'mentorpi_gz_sim/package.xml'
EXPECTED_MECANUM_SHA256 = 'f787f807714c78888dbe4afe755308ddb9e7f29c2778c6cb3bd5e56f960614ee'


class MecanumSourceModelTest(unittest.TestCase):
    def test_description_uses_the_pinned_mecanum_model(self):
        self.assertTrue(MECANUM.is_file())
        self.assertEqual(hashlib.sha256(MECANUM.read_bytes()).hexdigest(), EXPECTED_MECANUM_SHA256)
        self.assertTrue(MECANUM_URDF.is_symlink())
        self.assertEqual(MECANUM_URDF.readlink(), Path('mecanum.xacro'))
        self.assertFalse(LEGACY_MODEL.exists())
        description_launch = DESCRIPTION_LAUNCH.read_text()
        simulation_adapter_launch = SIM_ADAPTER_LAUNCH.read_text()
        self.assertIn('mecanum_forklift.xacro', description_launch)
        self.assertIn('mecanum_forklift.xacro', simulation_adapter_launch)
        self.assertIn("mappings={'robot_name': name}", simulation_adapter_launch)
        self.assertIn("executable='gz_pose_to_odom.py'", simulation_adapter_launch)
        self.assertNotIn("' robot_name:='", description_launch)
        self.assertNotIn("' frame_prefix:='", description_launch)
        self.assertNotIn(
            "mappings={'robot_name': name, 'frame_prefix': f'{name}/'}",
            simulation_adapter_launch,
        )

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
        drive_plugin = next(
            plugin for plugin in model.findall('plugin')
            if plugin.attrib['name'] == 'gz::sim::systems::MecanumDrive'
        )
        self.assertEqual(drive_plugin.findtext('wheelbase'), '0.13468')
        self.assertEqual(drive_plugin.findtext('wheel_separation'), '0.15182')
        self.assertIn('<gz_frame_id>${robot_name}/lidar_frame</gz_frame_id>', text)
        self.assertIn('<gz_frame_id>${robot_name}/depth_cam</gz_frame_id>', text)

    def test_gazebo_uses_original_sensor_meshes_without_guide_markers(self):
        root = ET.parse(SDF).getroot()
        model = root.find('model')
        links = {link.attrib['name']: link for link in model.findall('link')}

        expected_meshes = {
            'lidar_frame': ('lidar_visual', 'lidar_Link.STL'),
            'depth_cam': ('depth_camera_visual', 'cam_Link.STL'),
        }
        for link_name, (visual_name, mesh_name) in expected_meshes.items():
            visuals = {visual.attrib['name']: visual for visual in links[link_name].findall('visual')}
            self.assertEqual(set(visuals), {visual_name})
            self.assertEqual(links[link_name].findall('collision'), [])
            mesh = visuals[visual_name].find('geometry/mesh')
            self.assertIsNotNone(mesh)
            self.assertEqual(mesh.findtext('uri'), f'file://$(find mentorpi_description)/meshes/mecanum/{mesh_name}')
            self.assertIsNone(mesh.find('scale'))

        self.assertEqual(links['imu_link'].findall('visual'), [])

        for link_name in ('lidar_frame', 'depth_cam'):
            self.assertEqual(links[link_name].findtext('sensor/visualize'), 'false')
        self.assertEqual(links['imu_link'].findtext('sensor/visualize'), None)
        self.assertEqual(links['lidar_frame'].findtext('sensor/gz_frame_id'), '${robot_name}/lidar_frame')
        self.assertEqual(links['depth_cam'].findtext('sensor/gz_frame_id'), '${robot_name}/depth_cam')
        self.assertEqual(links['imu_link'].findtext('sensor/gz_frame_id'), '${robot_name}/imu_link')

    def test_gazebo_chassis_is_below_the_lidar_mesh(self):
        root = ET.parse(SDF).getroot()
        model = root.find('model')
        links = {link.attrib['name']: link for link in model.findall('link')}

        chassis = links['base_footprint']
        collision = chassis.find("collision[@name='chassis_collision']")
        visual = chassis.find("visual[@name='chassis_visual']")
        self.assertEqual(collision.findtext('pose'), '0 0 0.0765 0 0 0')
        self.assertEqual(visual.findtext('pose'), '0 0 0.0765 0 0 0')
        self.assertEqual(collision.findtext('geometry/box/size'), '0.180 0.145 0.090')
        self.assertEqual(visual.findtext('geometry/box/size'), '0.180 0.145 0.090')

        lidar_mesh = LIDAR_MESH.read_bytes()
        triangle_count = struct.unpack_from('<I', lidar_mesh, 80)[0]
        lidar_mesh_min_z = min(
            struct.unpack_from('<f', lidar_mesh, 84 + triangle * 50 + 12 + vertex * 12 + 8)[0]
            for triangle in range(triangle_count)
            for vertex in range(3)
        )
        chassis_top = 0.0765 + 0.090 / 2
        lidar_bottom = float(links['lidar_frame'].findtext('pose').split()[2]) + lidar_mesh_min_z
        self.assertLess(chassis_top, lidar_bottom)

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
                'gz_type_name: gz.msgs.Double, direction: ROS_TO_GZ',
                bridge_text,
            )
        gazebo_package = GAZEBO_PACKAGE_XML.read_text()
        self.assertIn('<exec_depend>std_msgs</exec_depend>', gazebo_package)
        self.assertIn('<description>Gazebo Harmonic simulation</description>', gazebo_package)

    def test_gazebo_assets_use_harmonic_names(self):
        world = (SOURCE / 'mentorpi_gz_sim/worlds/warehouse.sdf').read_text()
        model = SDF.read_text()
        bridges = '\n'.join(path.read_text() for path in BRIDGE_CONFIGS)
        combined = '\n'.join((world, model, bridges))

        self.assertIn('gz-sim-physics-system', world)
        self.assertIn('gz::sim::systems::MecanumDrive', model)
        self.assertIn('gz.msgs.LaserScan', bridges)
        self.assertIn('xmlns:gz="http://gazebosim.org/schema"', model)
        self.assertNotIn('ignition-gazebo', combined)
        self.assertNotIn('ignition::gazebo', combined)
        self.assertNotIn('ignition.msgs', combined)
        self.assertNotIn('ignition:expressed_in', combined)

    def test_ground_truth_uses_robot_specific_model_pose_publishers(self):
        model = ET.parse(SDF).getroot().find('model')
        pose_publisher = next(
            plugin for plugin in model.findall('plugin')
            if plugin.attrib.get('name') == 'gz::sim::systems::PosePublisher'
        )

        self.assertEqual(pose_publisher.attrib['filename'], 'gz-sim-pose-publisher-system')
        expected = {
            'publish_link_pose': 'false',
            'publish_visual_pose': 'false',
            'publish_collision_pose': 'false',
            'publish_sensor_pose': 'false',
            'publish_nested_model_pose': 'false',
            'publish_model_pose': 'true',
            'use_pose_vector_msg': 'true',
            'update_frequency': '30',
            'topic': '/${robot_name}/ground_truth/pose',
        }
        for tag, value in expected.items():
            self.assertEqual(pose_publisher.findtext(tag), value)

        bridges = '\n'.join(path.read_text() for path in BRIDGE_CONFIGS)
        for robot_name, bridge_path in zip(('robot_1', 'robot_2'), BRIDGE_CONFIGS):
            self.assertIn(
                f'ros_topic_name: /{robot_name}/ground_truth/pose, '
                f'gz_topic_name: /{robot_name}/ground_truth/pose, '
                'ros_type_name: tf2_msgs/msg/TFMessage, '
                'gz_type_name: gz.msgs.Pose_V, direction: GZ_TO_ROS',
                bridge_path.read_text(),
            )
        self.assertNotIn('/gz/dynamic_pose', bridges)
        self.assertNotIn('/world/mentorpi_warehouse/dynamic_pose/info', bridges)


if __name__ == '__main__':
    unittest.main()
