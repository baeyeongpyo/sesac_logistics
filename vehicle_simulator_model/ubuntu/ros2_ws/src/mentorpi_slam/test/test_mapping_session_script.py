import os
from contextlib import contextmanager
import json
import hashlib
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
        self.assertIn('ROS_COMMAND_TIMEOUT_SECONDS', text)
        self.assertIn('PROCESS_STOP_TIMEOUT_SECONDS', text)
        self.assertIn('PROCESS_KILL_TIMEOUT_SECONDS', text)
        self.assertIn('rosbag_metadata_is_nonempty', text)
        self.assertIn('rosbag_storage_is_nonempty', text)
        self.assertIn('posegraph_is_nonempty', text)
        self.assertIn('finalization_in_progress', text)
        self.assertIn('json.dumps', text)
        self.assertIn('atomic_publish.py', text)
        self.assertLess(text.index('trap cleanup EXIT'), text.index('session path appeared while acquiring lock'))
        self.assertNotIn('mv -T -n', text)

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
            manifest = json.loads((final_dir / 'manifest.json').read_text())
            self.assertEqual(
                manifest['slam_params_sha256'],
                hashlib.sha256((PACKAGE / 'config' / 'slam.yaml').read_bytes()).hexdigest(),
            )
            checksum_paths = [line.split('  ', 1)[1] for line in (final_dir / 'checksums.sha256').read_text().splitlines()]
            self.assertEqual(checksum_paths, sorted(checksum_paths))
            self.assertIn('map.yaml', checksum_paths)
            self.assertIn('posegraph/mentorpi.posegraph', checksum_paths)
            self.assertIn('rosbag2/mapping/metadata.yaml', checksum_paths)
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

    def test_rejects_missing_posegraph_and_metadata_only_bag(self):
        for failure_flag in ('FAKE_ROS_NO_POSEGRAPH', 'FAKE_ROS_METADATA_ONLY'):
            with self.subTest(failure_flag), self.fake_ros_environment() as environment:
                environment['env'][failure_flag] = '1'
                process = self.start_script(environment)
                self.wait_for_ready_processes(environment)
                _, errors = self.interrupt_and_wait(process)
                self.assertNotEqual(process.returncode, 0, errors)
                self.assertTrue((environment['data_root'] / '.inprogress' / 'session-1').is_dir())
                self.assertFalse((environment['data_root'] / 'session-1').exists())

    def test_rejects_unsafe_root_and_invalid_timeout_before_spawning(self):
        with self.fake_ros_environment() as environment:
            for updates in (
                {'SLAM_DATA_ROOT': 'relative/session-root'},
                {'SLAM_DATA_ROOT': str(environment['data_root']) + "\nunsafe"},
                {'ROS_COMMAND_TIMEOUT_SECONDS': 'one'},
            ):
                result = self.run_script(environment, **updates)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((environment['ready_dir'] / 'bag').exists())

    def test_service_timeout_and_ignoring_children_are_bounded_and_cleaned_up(self):
        for hang_flag in ('FAKE_ROS_HANG_LIST', 'FAKE_ROS_HANG_CALL'):
            with self.subTest(hang_flag), self.fake_ros_environment() as environment:
                environment['env'].update({
                    hang_flag: '1',
                    'ROS_COMMAND_TIMEOUT_SECONDS': '1',
                    'PROCESS_STOP_TIMEOUT_SECONDS': '1',
                    'PROCESS_KILL_TIMEOUT_SECONDS': '1',
                })
                process = self.start_script(environment)
                self.wait_for_ready_processes(environment)
                _, errors = self.interrupt_and_wait(process)
                self.assertNotEqual(process.returncode, 0, errors)
                self.assertFalse((environment['data_root'] / 'session-1').exists())

        with self.fake_ros_environment() as environment:
            environment['env'].update({
                'FAKE_ROS_IGNORE_CHILD_SIGNALS': '1',
                'PROCESS_STOP_TIMEOUT_SECONDS': '1',
                'PROCESS_KILL_TIMEOUT_SECONDS': '1',
            })
            process = self.start_script(environment)
            self.wait_for_ready_processes(environment)
            _, errors = self.interrupt_and_wait(process)
            self.assertEqual(process.returncode, 0, errors)
            for pid_path in environment['pid_dir'].iterdir():
                with self.assertRaises(ProcessLookupError):
                    os.kill(int(pid_path.read_text()), 0)

    def test_repeated_signal_keeps_finalization_active_and_requests_are_json(self):
        with self.fake_ros_environment() as environment:
            environment['env']['FAKE_ROS_CALL_DELAY'] = '1'
            process = self.start_script(environment)
            self.wait_for_ready_processes(environment)
            process.send_signal(signal.SIGINT)
            time.sleep(0.1)
            process.send_signal(signal.SIGTERM)
            _, errors = process.communicate(timeout=10)
            self.assertEqual(process.returncode, 0, errors)
            save_request = json.loads((environment['request_dir'] / 'save_map.json').read_text())
            graph_request = json.loads((environment['request_dir'] / 'serialize_map.json').read_text())
            self.assertEqual(save_request['name']['data'], str((environment['stage'] / 'map').resolve()))
            self.assertEqual(graph_request['filename'], str((environment['stage'] / 'posegraph' / 'mentorpi').resolve()))

    def test_final_target_race_never_nests_staging_directory(self):
        with self.fake_ros_environment() as environment:
            environment['env']['FAKE_ROS_CREATE_FINAL'] = '1'
            process = self.start_script(environment)
            self.wait_for_ready_processes(environment)
            _, errors = self.interrupt_and_wait(process)
            final_dir = environment['data_root'] / 'session-1'
            self.assertNotEqual(process.returncode, 0, errors)
            self.assertTrue((environment['data_root'] / '.inprogress' / 'session-1').is_dir())
            self.assertFalse((final_dir / 'session-1').exists())
            self.assertFalse((environment['data_root'] / '.session-locks' / 'session-1.lock').exists())

    def test_retains_stage_when_atomic_noreplace_publisher_is_unavailable(self):
        with self.fake_ros_environment() as environment:
            environment['env']['FAKE_ATOMIC_PUBLISH_UNSUPPORTED'] = '1'
            process = self.start_script(environment)
            self.wait_for_ready_processes(environment)
            _, errors = self.interrupt_and_wait(process)
            self.assertNotEqual(process.returncode, 0, errors)
            self.assertTrue((environment['data_root'] / '.inprogress' / 'session-1').is_dir())
            self.assertFalse((environment['data_root'] / '.session-locks' / 'session-1.lock').exists())

    @contextmanager
    def fake_ros_environment(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            bin_dir = root / 'bin'
            ready_dir = root / 'ready'
            pid_dir = root / 'pids'
            request_dir = root / 'requests'
            data_root = root / 'sessions'
            bin_dir.mkdir()
            ready_dir.mkdir()
            pid_dir.mkdir()
            request_dir.mkdir()
            fake_ros = bin_dir / 'ros2'
            fake_ros.write_text(textwrap.dedent('''\
            #!/usr/bin/env bash
            set -euo pipefail
            case "$1 $2" in
              'service list')
                if [[ "${FAKE_ROS_HANG_LIST:-}" == 1 ]]; then exec python3 -c 'import time; time.sleep(60)'; fi
                printf '%s\\n' /slam_toolbox/save_map /slam_toolbox/serialize_map
                ;;
              'service call')
                if [[ "${FAKE_ROS_HANG_CALL:-}" == 1 ]]; then exec python3 -c 'import time; time.sleep(60)'; fi
                if [[ -n "${FAKE_ROS_CALL_DELAY:-}" ]]; then sleep "$FAKE_ROS_CALL_DELAY"; fi
                printf '%s' "$5" > "$FAKE_REQUEST_DIR/${3##*/}.json"
                python3 - "$3" "$5" <<'PY'
import json
import sys

service, request = sys.argv[1:]
payload = json.loads(request)
if service == '/slam_toolbox/save_map':
    valid = isinstance(payload.get('name'), dict) and isinstance(payload['name'].get('data'), str)
elif service == '/slam_toolbox/serialize_map':
    valid = isinstance(payload.get('filename'), str)
else:
    valid = False
raise SystemExit(0 if valid else 42)
PY
                if [[ "$3" == /slam_toolbox/save_map ]]; then
                  if [[ "${FAKE_ROS_FAIL_SAVE:-}" == 1 ]]; then exit 17; fi
                  printf 'image: map.pgm\\n' > "$FAKE_STAGE/map.yaml"
                  printf 'P5\\n1 1\\n255\\n' > "$FAKE_STAGE/map.pgm"
                  printf '\\0' >> "$FAKE_STAGE/map.pgm"
                else
                  if [[ "${FAKE_ROS_CREATE_FINAL:-}" == 1 ]]; then mkdir "$FAKE_FINAL"; fi
                  if [[ "${FAKE_ROS_NO_POSEGRAPH:-}" != 1 ]]; then printf 'posegraph' > "$FAKE_STAGE/posegraph/mentorpi.posegraph"; fi
                fi
                ;;
              'bag record')
                mkdir -p "$FAKE_STAGE/rosbag2/mapping"
                printf 'rosbag2_bagfile_information:\\n' > "$FAKE_STAGE/rosbag2/mapping/metadata.yaml"
                if [[ "${FAKE_ROS_METADATA_ONLY:-}" != 1 ]]; then printf 'bag' > "$FAKE_STAGE/rosbag2/mapping/data.db3"; fi
                : > "$FAKE_READY_DIR/bag"
                printf '%s' "$$" > "$FAKE_PID_DIR/bag"
                if [[ "${FAKE_ROS_IGNORE_CHILD_SIGNALS:-}" == 1 ]]; then exec python3 -c 'import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'; fi
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
                printf '%s' "$$" > "$FAKE_PID_DIR/slam"
                if [[ "${FAKE_ROS_IGNORE_CHILD_SIGNALS:-}" == 1 ]]; then exec python3 -c 'import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'; fi
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
            fake_atomic_publisher = bin_dir / 'atomic_publish.py'
            fake_atomic_publisher.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                set -euo pipefail
                if [[ "${FAKE_ATOMIC_PUBLISH_UNSUPPORTED:-}" == 1 ]]; then exit 2; fi
                [[ -d "$1" && ! -e "$2" ]] || exit 1
                exec /bin/mv "$1" "$2"
            '''))
            fake_atomic_publisher.chmod(0o755)
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
                'FAKE_PID_DIR': str(pid_dir),
                'FAKE_REQUEST_DIR': str(request_dir),
                'FAKE_STAGE': str(data_root / '.inprogress' / 'session-1'),
                'FAKE_FINAL': str(data_root / 'session-1'),
                'ATOMIC_PUBLISHER': str(fake_atomic_publisher),
                'SLAM_SERVICE_WAIT_SECONDS': '0',
                'SLAM_CONFIG_PATH': str(PACKAGE / 'config' / 'slam.yaml'),
            })
            yield {
                'env': environment, 'data_root': data_root, 'ready_dir': ready_dir,
                'pid_dir': pid_dir, 'request_dir': request_dir,
                'stage': data_root / '.inprogress' / 'session-1',
            }

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
