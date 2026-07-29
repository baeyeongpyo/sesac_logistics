import unittest
import xml.etree.ElementTree as ET
import subprocess
import sys
import tempfile
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
PALLET_TEMPLATE = PACKAGE / 'models/pallet/pallet.sdf.in'
FRESH_PAYLOAD_TEMPLATE = PACKAGE / 'models/pallet/payload_fresh.sdf.in'
NORMAL_PAYLOAD_TEMPLATE = PACKAGE / 'models/pallet/payload_normal.sdf.in'
WORLD = PACKAGE / 'worlds/warehouse.sdf'
MARKINGS = PACKAGE / 'models/warehouse_markings/model.sdf'
MARKINGS_GENERATOR = PACKAGE / 'tools/generate_floor_markings.py'
ENV_HOOK = PACKAGE / 'env-hooks/mentorpi_gz_sim.sh.in'
STATIC_EQUIPMENT = (
    PACKAGE / 'models/warehouse_conveyor/model.sdf',
    PACKAGE / 'models/warehouse_robot_arm/model.sdf',
    PACKAGE / 'models/warehouse_charger/model.sdf',
    PACKAGE / 'models/warehouse_rack/model.sdf',
)

PALLET_SIZE = (0.135, 0.135, 0.030)
FORK_CENTER_Z = 0.018
FORK_THICKNESS = 0.010
CHANNEL_BOTTOM = 0.008
CHANNEL_TOP = 0.024


def render_sdf(template, tokens):
    rendered = template.read_text()
    for token, value in tokens.items():
        rendered = rendered.replace(token, value)
    return ET.fromstring(rendered)


class WarehouseAssetTest(unittest.TestCase):
    def test_pallet_envelope_and_fork_channels(self):
        root = render_sdf(PALLET_TEMPLATE, {
            '@PALLET_ID@': 'pallet_test',
            '@POSE@': '0 0 0 0 0 0',
        })
        model = root.find('model')
        self.assertEqual(model.attrib['name'], 'pallet_test')
        self.assertEqual(
            tuple(map(float, model.findtext("link/visual[@name='deck']/geometry/box/size").split())),
            (0.135, 0.135, 0.006),
        )
        self.assertLess(CHANNEL_BOTTOM, FORK_CENTER_Z - FORK_THICKNESS / 2)
        self.assertGreater(CHANNEL_TOP, FORK_CENTER_Z + FORK_THICKNESS / 2)

        link = model.find("link[@name='pallet_link']")
        support_collisions = [
            collision for collision in link.findall('collision')
            if collision.attrib['name'].startswith('support_')
        ]
        self.assertEqual(len(support_collisions), 3)
        self.assertEqual(
            [float(collision.findtext('pose').split()[1]) for collision in support_collisions],
            [-0.063, 0.0, 0.063],
        )
        for collision in support_collisions:
            self.assertEqual(collision.findtext('geometry/box/size'), '0.135 0.009 0.008')
            self.assertEqual(collision.findtext('surface/friction/ode/mu'), '1.2')
            self.assertEqual(collision.findtext('surface/friction/ode/mu2'), '1.2')

    def test_pallet_attachment_topics_reference_the_pallet_id(self):
        root = render_sdf(PALLET_TEMPLATE, {
            '@PALLET_ID@': 'pallet_test',
            '@POSE@': '0 0 0 0 0 0',
        })
        plugin = root.find('model/plugin')
        self.assertEqual(plugin.attrib['filename'], 'gz-sim-detachable-joint-system')
        self.assertEqual(plugin.attrib['name'], 'gz::sim::systems::DetachableJoint')
        self.assertEqual(plugin.findtext('parent_link'), 'pallet_link')
        self.assertEqual(plugin.findtext('child_model'), 'pallet_test_payload')
        self.assertEqual(plugin.findtext('child_link'), 'payload_link')
        self.assertEqual(plugin.findtext('attach_topic'), '/warehouse/pallet/pallet_test/attach')
        self.assertEqual(plugin.findtext('detach_topic'), '/warehouse/pallet/pallet_test/detach')
        self.assertEqual(plugin.findtext('output_topic'), '/warehouse/pallet/pallet_test/attached')
        self.assertEqual(plugin.findtext('suppress_child_warning'), 'true')

    def test_payload_templates_use_the_pallet_payload_contract(self):
        expected_colors = {
            FRESH_PAYLOAD_TEMPLATE: '0.1 0.7 0.2 1',
            NORMAL_PAYLOAD_TEMPLATE: '0.1 0.3 0.8 1',
        }
        for template, color in expected_colors.items():
            with self.subTest(template=template.name):
                root = render_sdf(template, {
                    '@PALLET_ID@': 'pallet_test',
                    '@PAYLOAD_POSE@': '0 0 0.03 0 0 0',
                })
                model = root.find('model')
                self.assertEqual(model.attrib['name'], 'pallet_test_payload')
                self.assertEqual(model.findtext('pose'), '0 0 0.03 0 0 0')
                link = model.find("link[@name='payload_link']")
                self.assertEqual(link.findtext('inertial/mass'), '0.25')
                collision_size = tuple(map(float, link.findtext('collision/geometry/box/size').split()))
                self.assertLessEqual(collision_size[0], 0.125)
                self.assertLessEqual(collision_size[1], 0.125)
                visuals = link.findall('visual')
                self.assertGreater(len(visuals), 1)
                self.assertTrue(all(
                    visual.findtext('material/diffuse') == color
                    for visual in visuals
                ))

    def test_reference_layout_and_static_equipment(self):
        world = ET.parse(WORLD).getroot().find('world')
        self.assertEqual(world.attrib['name'], 'mentorpi_warehouse')
        includes = world.findall('include')
        names = {
            include.findtext('name')
            for include in includes
        }
        self.assertTrue({
            'left_conveyor', 'pico_conveyor',
            'robot_arm_upper', 'robot_arm_lower',
            'charging_station', 'fresh_rack', 'normal_rack',
            'floor_markings',
        } <= names)
        self.assertTrue(all(include.attrib == {} for include in includes))
        text = WORLD.read_text()
        self.assertNotIn('joint-controller', text)
        self.assertNotIn('trajectory-controller', text)

    def test_world_has_six_configured_default_pallets(self):
        world = ET.parse(WORLD).getroot().find('world')
        plugin = world.find(
            "plugin[@name='mentorpi_gz_sim::WarehousePalletManager']")
        self.assertIsNotNone(plugin)
        self.assertEqual(
            plugin.attrib['filename'],
            'mentorpi_warehouse_pallet_manager',
        )
        defaults = plugin.findall('default_pallet')
        self.assertEqual(len(defaults), 6)
        self.assertEqual(
            [(p.attrib['kind'], p.attrib['state']) for p in defaults],
            [('fresh', 'loaded')] * 3 + [('normal', 'loaded')] * 3,
        )
        self.assertEqual(
            [p.attrib['pose'] for p in defaults],
            [
                '-1.0 0.6 0 0 0 0',
                '-0.5 0.6 0 0 0 0',
                '0.0 0.6 0 0 0 0',
                '2.9 2.4 0 0 0 0',
                '3.4 2.4 0 0 0 0',
                '3.9 2.4 0 0 0 0',
            ],
        )

    def test_equipment_models_are_static_with_simple_collisions(self):
        for model_path in STATIC_EQUIPMENT:
            with self.subTest(model=model_path.parent.name):
                model = ET.parse(model_path).getroot().find('model')
                self.assertEqual(model.findtext('static'), 'true')
                collisions = model.findall('.//collision')
                self.assertGreaterEqual(len(collisions), 1)
                self.assertLessEqual(len(collisions), 4)

        arm = ET.parse(STATIC_EQUIPMENT[1]).getroot().find('model')
        joints = arm.findall('joint')
        self.assertGreaterEqual(len(joints), 1)
        self.assertTrue(all(joint.attrib['type'] == 'fixed' for joint in joints))

    def test_floor_markings_are_visual_only_and_deterministic(self):
        root = ET.parse(MARKINGS).getroot()
        self.assertEqual(root.findtext('model/static'), 'true')
        self.assertEqual(root.findall('.//collision'), [])
        self.assertGreater(len(root.findall('.//visual')), 100)

        with tempfile.TemporaryDirectory() as tmp:
            generated = Path(tmp) / 'model.sdf'
            subprocess.run(
                [sys.executable, str(MARKINGS_GENERATOR), str(generated)],
                check=True,
            )
            self.assertEqual(generated.read_bytes(), MARKINGS.read_bytes())

    def test_install_hook_uses_the_active_colcon_prefix(self):
        result = subprocess.run(
            [
                'bash',
                '-c',
                '''
ament_prepend_unique_value() {
  eval "export $1=\\"$2\\""
}
export AMENT_CURRENT_PREFIX=/opt/ros/humble
export COLCON_CURRENT_PREFIX=/opt/mentorpi_ws/install/mentorpi_gz_sim
source "$1"
printf '%s\\n%s\\n' "$GZ_SIM_SYSTEM_PLUGIN_PATH" "$GZ_SIM_RESOURCE_PATH"
''',
                'bash',
                str(ENV_HOOK),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.stdout.splitlines(), [
            '/opt/mentorpi_ws/install/mentorpi_gz_sim/lib',
            '/opt/mentorpi_ws/install/mentorpi_gz_sim/share/'
            'mentorpi_gz_sim/models',
        ])


if __name__ == '__main__':
    unittest.main()
