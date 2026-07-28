import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
PALLET_TEMPLATE = PACKAGE / 'models/pallet/pallet.sdf.in'
FRESH_PAYLOAD_TEMPLATE = PACKAGE / 'models/pallet/payload_fresh.sdf.in'
NORMAL_PAYLOAD_TEMPLATE = PACKAGE / 'models/pallet/payload_normal.sdf.in'

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


if __name__ == '__main__':
    unittest.main()
