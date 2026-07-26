import os
from contextlib import contextmanager
from pathlib import Path
import signal
import subprocess
from tempfile import TemporaryDirectory
import textwrap
import time
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts' / 'run_mapping_session.sh'


class MappingSessionScriptTest(unittest.TestCase):
    def test_script_records_required_topics_and_saves_artifacts(self):
        text = SCRIPT.read_text()
        for topic in ('/clock', '/tf', '/tf_static', '/robot_1/scan_raw',
                      '/robot_1/imu/data_raw', '/robot_1/odom'):
            self.assertIn(topic, text)
        self.assertIn('/slam_toolbox/save_map', text)
        self.assertIn('/slam_toolbox/serialize_map', text)
        self.assertIn('.inprogress', text)
        self.assertIn('session_artifacts.py', text)
        self.assertIn('mv \"$stage_dir\" \"$final_dir\"', text)

    def test_rejects_invalid_session_id_without_creating_a_final_session(self):
        self.require_script()
        with self.fake_ros_environment() as environment:
            result = self.run_script(environment, SESSION_ID='invalid/session')
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((environment['data_root'] / 'invalid').exists())

    def test_rejects_existing_stage_or_final_path(self):
        self.require_script()
        with self.fake_ros_environment() as environment:
            stage = environment['data_root'] / '.inprogress' / 'session-1'
            stage.mkdir(parents=True)
            result = self.run_script(environment)
            self.assertNotEqual(result.returncode, 0)

        with self.fake_ros_environment() as environment:
            (environment['data_root'] / 'session-1').mkdir(parents=True)
            result = self.run_script(environment)
            self.assertNotEqual(result.returncode, 0)

    def test_sigint_finalizes_then_atomically_publishes_complete_session(self):
        self.require_script()
        with self.fake_ros_environment() as environment:
            process = self.start_script(environment)
            self.wait_for_ready_processes(environment)
            self.assertFalse((environment['data_root'] / 'session-1').exists())

            _, errors = self.interrupt_and_wait(process)

            final_dir = environment['data_root'] / 'session-1'
            self.assertEqual(process.returncode, 0, errors)
            self.assertTrue((final_dir / 'map.yaml').is_file())
            self.assertGreater((final_dir / 'map.pgm').stat().st_size, 0)
            self.assertTrue((final_dir / 'posegraph' / 'mentorpi.posegraph').is_file())
            self.assertTrue((final_dir / 'rosbag2' / 'mapping' / 'data.db3').is_file())
            self.assertTrue((final_dir / 'manifest.json').is_file())
            self.assertTrue((final_dir / 'checksums.sha256').is_file())
            self.assertFalse((environment['data_root'] / '.inprogress' / 'session-1').exists())

    def test_finalization_failure_retains_only_inprogress_session(self):
        self.require_script()
        with self.fake_ros_environment() as environment:
            environment['env']['FAKE_ROS_FAIL_SAVE'] = '1'
            process = self.start_script(environment)
            self.wait_for_ready_processes(environment)
            _, errors = self.interrupt_and_wait(process)

            self.assertNotEqual(process.returncode, 0, errors)
            self.assertTrue((environment['data_root'] / '.inprogress' / 'session-1').is_dir())
            self.assertFalse((environment['data_root'] / 'session-1').exists())

    @contextmanager
    def fake_ros_environment(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / 'bin'
            ready_dir = root / 'ready'
            data_root = root / 'sessions'
            bin_dir.mkdir()
            ready_dir.mkdir()
            fake_ros = bin_dir / 'ros2'
            fake_ros.write_text(textwrap.dedent('''\
            #!/usr/bin/env bash
            set -euo pipefail
            case "$1 $2" in
              'service list')
                printf '%s\\n' /slam_toolbox/save_map /slam_toolbox/serialize_map
                ;;
              'service call')
                if [[ "$3" == /slam_toolbox/save_map ]]; then
                  if [[ "${FAKE_ROS_FAIL_SAVE:-}" == 1 ]]; then exit 17; fi
                  printf 'image: map.pgm\\n' > "$FAKE_STAGE/map.yaml"
                  printf 'P5\\n1 1\\n255\\n' > "$FAKE_STAGE/map.pgm"
                  printf '\\0' >> "$FAKE_STAGE/map.pgm"
                else
                  printf 'posegraph' > "$FAKE_STAGE/posegraph/mentorpi.posegraph"
                fi
                ;;
              'bag record')
                mkdir -p "$FAKE_STAGE/rosbag2/mapping"
                printf 'bag' > "$FAKE_STAGE/rosbag2/mapping/data.db3"
                : > "$FAKE_READY_DIR/bag"
                exec python3 -c 'import signal
import sys
import time
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
while True:
    time.sleep(1)'
                ;;
              'launch mentorpi_slam')
                : > "$FAKE_READY_DIR/slam"
                trap 'exit 0' INT TERM
                while :; do sleep 1; done
                ;;
              *)
                printf 'unexpected ros2 invocation: %s %s\\n' "$1" "$2" >&2
                exit 99
                ;;
            esac
            '''))
            fake_ros.chmod(0o755)
            environment = os.environ.copy()
            environment.update({
                'PATH': f'{bin_dir}{os.pathsep}{environment["PATH"]}',
                'SLAM_DATA_ROOT': str(data_root),
                'SESSION_ID': 'session-1',
                'IMAGE_VERSION': 'image-v1',
                'GIT_COMMIT': 'deadbeef',
                'WORLD_VERSION': 'world-v1',
                'MODEL_VERSION': 'model-v1',
                'TF_CALIBRATION_VERSION': 'tf-v1',
                'FAKE_READY_DIR': str(ready_dir),
                'FAKE_STAGE': str(data_root / '.inprogress' / 'session-1'),
                'SLAM_SERVICE_WAIT_SECONDS': '0',
                'SLAM_CONFIG_PATH': str(PACKAGE / 'config' / 'slam.yaml'),
            })
            yield {'env': environment, 'data_root': data_root, 'ready_dir': ready_dir}

    def run_script(self, environment, **updates):
        child_environment = environment['env'].copy()
        child_environment.update(updates)
        return subprocess.run(
            ['bash', str(SCRIPT)], env=child_environment, text=True,
            capture_output=True, timeout=10,
        )

    def require_script(self):
        if not SCRIPT.exists():
            self.skipTest('run_mapping_session.sh has not been implemented yet')

    def start_script(self, environment):
        return subprocess.Popen(
            ['bash', str(SCRIPT)], env=environment['env'], text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            preexec_fn=self.restore_signal_handlers,
        )

    @staticmethod
    def restore_signal_handlers():
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

    def interrupt_and_wait(self, process):
        process.send_signal(signal.SIGINT)
        try:
            return process.communicate(timeout=10)
        except subprocess.TimeoutExpired as error:
            process.terminate()
            output, errors = process.communicate(timeout=5)
            self.fail(
                'mapping session did not finalize within 10 seconds; '
                f'partial stdout={error.output!r}, partial stderr={error.stderr!r}, '
                f'cleanup stdout={output!r}, cleanup stderr={errors!r}'
            )

    def wait_for_ready_processes(self, environment):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            ready_dir = environment['ready_dir']
            if (ready_dir / 'bag').exists() and (ready_dir / 'slam').exists():
                return
            time.sleep(0.05)
        self.fail('fake rosbag and SLAM launch did not start')


if __name__ == '__main__':
    unittest.main()
