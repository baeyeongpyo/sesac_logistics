# Task 4 Report: Compose mapping profile and operator commands

## Result

`slam-mapper` one-shot Compose profile, operator commands, read-only final-session
inspection, and operator documentation were added.

## TDD evidence

1. Added mapping profile/operator command assertions to
   `vehicle_simulator_model/ubuntu/test/test_bundle.py`.
2. Confirmed RED: the new assertions failed because `slam-mapper` and the mapping
   commands did not exist.
3. Added the Compose services, `run.sh` lifecycle commands, and README instructions.
4. During self-review, added a failing assertion for Compose propagation of required
   mapping metadata, then added the missing environment mappings.

## Implementation

- `slam-mapper` shares the existing internal network and ROS domain, waits for
  healthy `sim-adapter`, mounts `mentorpi-slam-data` at `/slam-data`, and overrides
  inherited restart behavior with `restart: "no"`.
- `mapping-up <session-id>` enforces the mapping session-ID grammar and exports the
  required metadata before starting Gazebo, adapter, and mapper under the mapping
  profile.
- `mapping-stop` sends `SIGINT`, polls the mapper container with a bounded timeout,
  reports its finalization exit status, then stops Gazebo and adapter. A timeout
  leaves the mapper untouched so its finalization is not cut off.
- `mapping-status <session-id>` validates the same grammar and uses the separate
  `slam-inspector` service, whose named-volume mount is read-only, to list only the
  published final directory and run `sha256sum -c` inside it.
- README documents operation, successful layout, `.inprogress` behavior, and named
  volume backup/restore commands.

## Verification

Passed on 2026-07-26:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml --profile mapping config --quiet
bash -n vehicle_simulator_model/ubuntu/run.sh
```

Also verified invalid `mapping-up` and `mapping-status` IDs reject before Docker
execution, Compose renders exported mapping metadata into `slam-mapper`, and
`git diff --check` is clean.

## Self-review and remaining concern

The final-session inspector is read-only and never selects `.inprogress`; mapper
finalization is not interrupted by an immediate `down`. Linux container lifecycle
E2E remains intentionally unexecuted on this macOS host and is deferred to Task 5.

## Fix round 1/5 (2026-07-26)

Reviewer findings were addressed with boundary behavior tests before implementation:

- `mapping-stop` now queries `docker compose ps -q --all slam-mapper`, so an exited
  mapper is still found, its exit code is reported, and support services are
  cleaned up. A failed `SIGINT` send re-checks that container state before deciding.
- A mapper that remains running through the bounded stop wait now returns non-zero
  without stopping Gazebo or the adapter.
- Both the operator command and mapping lifecycle runner reject `.` and `..` before
  creating a session/staging path.
- The backup command mounts `mentorpi-slam-data` read-only; the restore command is
  explicitly documented as writable.

Fresh verification output summary:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
# Ran 34 tests in 19.705s — OK

bash -n vehicle_simulator_model/ubuntu/run.sh \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml --profile mapping config --quiet
git diff --check
# all exited 0
```

## Fix round 2/5 (2026-07-26)

Task 5 exposed a fresh named-volume ownership failure: Docker created
`mentorpi-slam-data` as `root:root` while the mapper runs as uid/gid 1000. The
mapping profile now has one-shot `slam-data-init`, the only root-running service.
It executes non-recursive `chown 1000:1000 /slam-data`; `slam-mapper` remains
non-root and depends on both its successful completion and a healthy adapter.
No session subtree is recursively chowned or otherwise modified. The inspector
continues to mount the volume read-only.

The first mapping-up performs this root-directory ownership setup. Repeated runs
only reassert the mount-root owner and do not alter existing session contents.

Fresh verification output summary:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
# Ran 35 tests in 19.125s — OK

bash -n vehicle_simulator_model/ubuntu/run.sh \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml --profile mapping config --quiet
git diff --check
# all exited 0
```

A bounded (30-second per container) temporary-volume smoke check used
`mentorpi-task4-init-check-r2`, not `mentorpi-slam-data`:

```bash
docker run --rm --user 0:0 -v mentorpi-task4-init-check-r2:/slam-data \
  --entrypoint /bin/bash mentorpi-sim:harmonic -lc 'chown 1000:1000 /slam-data'
docker run --rm --user 1000:1000 -v mentorpi-task4-init-check-r2:/slam-data \
  --entrypoint /bin/bash mentorpi-sim:harmonic -lc \
  'test "$(stat -c %u:%g /slam-data)" = 1000:1000 && test -w /slam-data && \
   mkdir /slam-data/.initializer-write-check && rmdir /slam-data/.initializer-write-check'
# temporary volume initializer check: uid_gid=1000:1000 mapper_write=ok
```

The temporary volume was removed afterward; `docker volume inspect mentorpi-slam-data`
reported `preserved volume=mentorpi-slam-data`.
