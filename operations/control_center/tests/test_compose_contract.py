from pathlib import Path
import unittest

import yaml


OPERATIONS = Path(__file__).resolve().parents[2]
CONTROL_CENTER = OPERATIONS / 'control_center'


def compose(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


class ControlCenterComposeContractTests(unittest.TestCase):
    def test_data_and_map_mounts_are_read_only(self):
        service = compose(CONTROL_CENTER / 'docker-compose.yaml')['services'][
            'control-center'
        ]
        mounts = {item['target']: item for item in service['volumes']}

        self.assertTrue(mounts['/data']['read_only'])
        self.assertTrue(mounts['/maps']['read_only'])
        self.assertEqual(
            mounts['/maps']['source'],
            '${CONTROL_CENTER_MAP_DIRECTORY:-../monitoring/map_server/maps}',
        )

    def test_local_compose_includes_control_center(self):
        content = (OPERATIONS / 'compose.local.yaml').read_text(encoding='utf-8')

        self.assertIn('./control_center/docker-compose.yaml', content)

    def test_image_includes_the_static_dashboard_assets(self):
        dockerfile = (CONTROL_CENTER / 'Dockerfile').read_text(encoding='utf-8')

        self.assertIn('COPY static ./static', dockerfile)
