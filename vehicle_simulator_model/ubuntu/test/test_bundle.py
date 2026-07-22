import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = BUNDLE.parents[1]


class DeployOnlyBundleTest(unittest.TestCase):
    def test_bundle_contains_all_runtime_assets(self):
        for relative_path in (
            'Dockerfile',
            'compose.yaml',
            'entrypoint.sh',
            'run.sh',
            'README.md',
            'vendor/virtualgl_3.1.4_amd64.deb',
            'ros2_ws/src/mentorpi_description/package.xml',
            'ros2_ws/src/mentorpi_gz_sim/package.xml',
        ):
            self.assertTrue((BUNDLE / relative_path).is_file(), relative_path)

        compose = (BUNDLE / 'compose.yaml').read_text()
        self.assertIn('context: .', compose)
        self.assertIn('mentorpi-sim:', compose)
        self.assertIn('mentorpi-gui:', compose)
        self.assertIn('./ros2_ws:/ws', compose)
        self.assertIn('/dev/dri/renderD128:/dev/dri/renderD128', compose)
        self.assertIn('network_mode: host', compose)
        self.assertIn('/tmp/.Xauthority:ro', compose)
        self.assertIn('RENDER_GID', compose)
        self.assertIn('IGN_IP: 127.0.0.1', compose)

        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        self.assertIn('ARG VIRTUALGL_VERSION=3.1.4', dockerfile)
        self.assertIn('COPY vendor/virtualgl_${VIRTUALGL_VERSION}_amd64.deb', dockerfile)
        self.assertIn('02edc6b599571c385389af1a006f07a70c298e1d97c580a9bfd4b39d835c51e6', dockerfile)

        script = (BUNDLE / 'run.sh').read_text()
        for command in ('build', 'shell', 'headless', 'gui', 'test', 'fork-up'):
            self.assertIn(command, script)
        self.assertIn('/opt/VirtualGL/bin/vglrun -d egl -c proxy', script)
        self.assertIn('source install/setup.bash', script)

    def test_repository_has_no_duplicate_root_runtime_layout(self):
        for legacy_path in ('docker', 'ros2_ws', 'compose.yaml', 'test'):
            self.assertFalse((REPOSITORY_ROOT / legacy_path).exists(), legacy_path)

    def test_operator_docs_describe_in_place_bundle_changes(self):
        readme = (BUNDLE / 'README.md').read_text()
        self.assertNotIn('refresh-workspace.sh', readme)
        self.assertIn('deploy/ubuntu', readme)
        for text in ('scp -r', 'ssh -Y', 'XAUTHORITY', './run.sh build', './run.sh headless', './run.sh gui'):
            self.assertIn(text, readme)


if __name__ == '__main__':
    unittest.main()
