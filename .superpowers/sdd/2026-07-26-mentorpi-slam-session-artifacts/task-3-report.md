# Task 3 Report: atomic mapping session lifecycle

## TDD evidence

1. Added the script contract and fake-ROS behavior tests before creating
   `run_mapping_session.sh`.
2. RED: `python3 -m unittest ...test_mapping_session_script.py -v` failed with
   `FileNotFoundError` for the deliberately absent script.
3. GREEN: implemented staging, bounded service discovery, signal finalization,
   metadata/checksums, map checks, and atomic publication. A fake rosbag initially
   inherited ignored `SIGINT`; the test fixture now replaces it with a process that
   explicitly restores the signal handler. A failure test then exposed an incorrect
   zero exit status from the trap; the trap now preserves the finalization failure.

## Verification

```text
bash -n vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py -v

Ran 8 tests in 4.228s
OK
```

`git diff --check` also completed without whitespace errors.

## Scope and remaining concern

The lifecycle tests use fake `ros2` processes; a live ROS/Gazebo end-to-end run is
still required to validate the installed `slam_toolbox` service behavior and exact
rosbag output layout. The production default hashes the installed `slam.yaml`; the
source config fallback exists only so the source-tree script can be tested before
installation.

## Fix round 1: review findings

### Root cause and changes

The prior lifecycle accepted any non-empty rosbag file, did not verify serialized
posegraph output, directly blocked in `ros2`/`wait`, reset signal handling during
finalization, and used a plain rename without a session reservation. The fix adds:

- non-empty map, posegraph, `metadata.yaml`, and `.db3`/`.mcap` checks before
  manifest/checksum/publication;
- validated timeout environment values, bounded `ros2 service list`/`call`, and
  bounded SIGINT/TERM/KILL process escalation;
- an EXIT cleanup path plus a coalescing finalization guard that keeps INT/TERM
  trapped during critical finalization;
- absolute, normalized, quote/control-character-safe data roots and JSON-encoded
  service request bodies;
- per-session `mkdir` reservation and GNU `mv -T -n` no-clobber publication.
  This is an atomic same-filesystem rename because stage and final are siblings
  beneath one normalized data root. The BSD fallback relies on the reservation and
  immediate target check; production ROS is the Ubuntu/GNU path.

### Fix round 1 verification

```text
bash -n vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py -v

Ran 15 tests in 16.775s
OK
```

The fake ROS tests cover missing posegraph, metadata-only bag, service list and
call hangs, signal-ignoring children with KILL escalation, repeated INT/TERM,
unsafe root/timeout validation, JSON request parsing, target race no-nesting, and
manifest SHA/checksum ordering.

## Fix round 2: service schema and publication safety

`SerializePoseGraph` takes a plain string `filename`; it is not a ROS string
wrapper. The lifecycle now uses separate JSON encoders: SaveMap retains
`{"name":{"data":"..."}}`, while SerializePoseGraph emits
`{"filename":"..."}`. The fake ROS service parses each JSON request and rejects
the wrong schema, so this is an executable service-type regression test.

Lock cleanup is now armed immediately after successful lock creation, before the
post-lock path recheck. The unsafe BSD check-then-rename fallback was removed:
production requires GNU `mv -T -n` and safely fails before startup when it is not
available. Tests include lock release after a target race and GNU-mv unavailability.

```text
bash -n vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py -v

Ran 16 tests in 19.426s
OK
```

## Fix round 3: kernel-atomic no-replace publication

GNU `mv -T -n` cannot provide the required no-replace guarantee across its user
space check/rename sequence. Publication now delegates to installed
`scripts/atomic_publish.py`, which calls Linux `renameat2` via `ctypes` with
`RENAME_NOREPLACE`. `EEXIST`, unsupported architectures, unsupported kernels, and
cross-filesystem publication return failure without moving the stage; there is no
fallback rename path.

`test_atomic_publish.py` directly invokes the helper. On Linux it verifies a
successful publish and the deterministic already-created-target race, including
the absence of overwrite or nested stage. On this macOS test host, that Linux-only
pair is skipped and the helper's unsupported-platform stage-retention test passes.
The lifecycle fake publisher is used only to exercise the surrounding signal and
artifact lifecycle on a non-Linux host.

```text
bash -n vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
python3 -m py_compile vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/atomic_publish.py
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_atomic_publish.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py -v

Ran 19 tests in 18.342s
OK (skipped=2 Linux-only renameat2 tests on macOS)
```
