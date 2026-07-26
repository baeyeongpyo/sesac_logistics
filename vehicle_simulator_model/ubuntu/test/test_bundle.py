import unittest
from pathlib import Path


BUNDLE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = BUNDLE.parents[1]


class DeployOnlyBundleTest(unittest.TestCase):
    def test_bundle_contains_all_runtime_assets(self):
        for relative_path in (
            'Dockerfile',
            'compose.yaml',
            'compose.gpu.yaml',
            'entrypoint.sh',
            'run.sh',
            'README.md',
            'ros2_ws/src/mentorpi_description/package.xml',
            'ros2_ws/src/mentorpi_gz_sim/package.xml',
        ):
            self.assertTrue((BUNDLE / relative_path).is_file(), relative_path)

        compose = (BUNDLE / 'compose.yaml').read_text()
        for service in ('gazebo-server:', 'sim-adapter:'):
            self.assertIn(service, compose)
        for required in (
            'GZ_PARTITION: mentorpi-sim',
            'condition: service_healthy',
            'LIBGL_ALWAYS_SOFTWARE',
            'ros2 launch mentorpi_gz_sim gazebo_server.launch.py',
            'ros2 launch mentorpi_gz_sim sim_adapter.launch.py',
            'mentorpi:',
        ):
            self.assertIn(required, compose)
        for removed in (
            'mentorpi-gui:',
            'DISPLAY:',
            'XAUTHORITY:',
            'VirtualGL',
            '/dev/dri/renderD128:/dev/dri/renderD128',
            'network_mode: host',
            './ros2_ws:/ws',
        ):
            self.assertNotIn(removed, compose)
        for forbidden_port in ('10317:', '10318:', '9002:'):
            self.assertNotIn(forbidden_port, compose)
        self.assertNotIn('build:', compose)
        self.assertIn('image: "${MENTORPI_IMAGE:-mentorpi-sim:harmonic}"', compose)
        self.assertIn(
            'IMAGE_VERSION: "${MENTORPI_IMAGE:-mentorpi-sim:harmonic}"',
            compose,
        )

        gpu_compose = (BUNDLE / 'compose.gpu.yaml').read_text()
        self.assertIn('/dev/dri:/dev/dri', gpu_compose)
        self.assertIn('LIBGL_ALWAYS_SOFTWARE: "0"', gpu_compose)

        script = (BUNDLE / 'run.sh').read_text()
        for command in ('build', 'sim-up', 'down', 'logs', 'test', 'fork-up'):
            self.assertIn(command, script)
        self.assertIn('up -d gazebo-server sim-adapter', script)
        self.assertIn('MENTORPI_IMAGE', script)
        self.assertIn('docker build --platform', script)
        self.assertNotIn('"${COMPOSE[@]}" build', script)
        for removed in ('ssh -Y', 'vglrun', 'DISPLAY'):
            self.assertNotIn(removed, script)

        entrypoint = (BUNDLE / 'entrypoint.sh').read_text()
        for required in ('SERVICE_NAME', 'IMAGE_VERSION', 'SESSION_ID', 'ROBOT_IDS'):
            self.assertIn(required, entrypoint)

    def test_runtime_image_uses_humble_with_harmonic(self):
        dockerfile = (BUNDLE / 'Dockerfile').read_text()
        self.assertIn('FROM ros:humble-ros-base-jammy AS runtime', dockerfile)
        self.assertNotIn('humble-desktop-full', dockerfile)
        self.assertIn('https://packages.osrfoundation.org/gazebo.gpg', dockerfile)
        self.assertIn('gz-harmonic', dockerfile)
        for required in (
            'ros-humble-robot-state-publisher',
            'ros-humble-ros-gzharmonic',
            'ros-humble-tf2-ros',
            'ros-humble-xacro',
        ):
            self.assertIn(required, dockerfile)
        for removed in ('ros-humble-ros-gz \\', 'VirtualGL', 'x11-apps', 'xauth', 'dbus-x11'):
            self.assertNotIn(removed, dockerfile)
        self.assertFalse((BUNDLE / 'vendor/virtualgl_3.1.4_amd64.deb').exists())

    def test_repository_has_no_duplicate_root_runtime_layout(self):
        for legacy_path in ('docker', 'ros2_ws', 'compose.yaml', 'test'):
            self.assertFalse((REPOSITORY_ROOT / legacy_path).exists(), legacy_path)

    def test_operator_docs_describe_in_place_bundle_changes(self):
        readme = (BUNDLE / 'README.md').read_text()
        for removed in ('ssh -Y', 'XAUTHORITY', 'VirtualGL', 'XQuartz'):
            self.assertNotIn(removed, readme)
        for text in (
            'linux/amd64',
            './run.sh build',
            './run.sh sim-up',
            './run.sh logs',
            './run.sh down',
            './run.sh fork-up',
            'MENTORPI_IMAGE',
            'sha256:',
            '브라우저',
            '오프스크린',
        ):
            self.assertIn(text, readme)


if __name__ == '__main__':
    unittest.main()
