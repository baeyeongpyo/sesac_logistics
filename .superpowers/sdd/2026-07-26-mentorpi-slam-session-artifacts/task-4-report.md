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

## Fix round 3/5 (2026-07-26)

Task 5 retry showed that `ros2 run` was container PID 1 and the lifecycle Bash was
its child, so `docker compose kill -s SIGINT` did not reliably reach the lifecycle
trap. `slam-mapper` now directly commands the installed executable:

```text
/opt/mentorpi_ws/install/mentorpi_slam/lib/mentorpi_slam/run_mapping_session.sh
```

The image entrypoint sources ROS/workspace setup and ends with `exec "$@"`, so the
kernel executes this shebang Bash as PID 1. During runtime probing, the direct Bash
could still defer the trap while blocked in `wait "$slam_pid"`; the lifecycle now
polls child completion in 100 ms intervals before reaping it, which leaves PID 1
available to process SIGINT.

Fresh verification output summary:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
# Ran 37 tests in 20.789s — OK

bash -n vehicle_simulator_model/ubuntu/run.sh \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml --profile mapping config --quiet
git diff --check
# all exited 0
```

A bounded isolated signal smoke used temporary volume/container names
`mentorpi-task4-signal-diagnose-r3` (removed afterward), mounted the current
lifecycle scripts read-only, and set all lifecycle timeout variables to two seconds.
It verified all of the following before no-stack cleanup:

```text
PID1: bash /tmp/mentorpi-slam-scripts/run_mapping_session.sh
services: /slam_toolbox/save_map, /slam_toolbox/serialize_map
SIGINT result: exited 1 (bounded failure because the isolated smoke has no map data)
logs: SaveMap request, bounded ros2 service timeout, recorder shutdown
volume: only .inprogress/signal-diagnose-r3 remained; no final session was published
```

This proves PID 1 signal delivery and bounded lifecycle failure, not a replacement
for the full Task 5 mapping-success E2E. The existing `mentorpi-slam-data` volume
was inspected as `preserved volume=mentorpi-slam-data`; `smoke-002` was not touched.

## Fix round 4/5 (2026-07-26)

Task 5's `smoke-003` staging bag had zero messages because ROS 2 traffic stopped at
the `sim-adapter` container boundary. The failure was reproduced before changing
Compose, using only bounded debug projects:

| Adapter/probe boundary | Discovery | scan/odom/TF/clock payload |
|---|---:|---:|
| Same internal bridge, separate network and IPC namespaces | no | no |
| Same internal bridge, shared IPC only | no | no |
| Shared adapter network namespace, separate IPC namespaces | yes | no (all echoes timed out with 124) |
| Shared adapter network and IPC namespaces | yes | yes (all echoes exited 0) |

In the baseline, `sim-adapter` was healthy and its own bounded
`ros2 topic echo --once` received real `/robot_1/scan_raw`, `/robot_1/odom`,
`/tf`, and `/clock` payloads. A separate runtime container on the same Compose
bridge saw only `/parameter_events` and `/rosout`. Sharing only the network
namespace exposed all publishers and their QoS (the scan publisher was RELIABLE)
but still delivered no data. Sharing both namespaces delivered all four payloads.
The containers use `rmw_fastrtps_cpp` and the same image machine ID. The observed
root cause is therefore the combined Fast DDS discovery/network and shared-memory
IPC boundary, not a topic-name, ROS domain, or scan QoS mismatch.

The regression contract was added first and runs the real Compose renderer:

```bash
python3 -m unittest \
  vehicle_simulator_model.ubuntu.test.test_bundle.DeployOnlyBundleTest.test_mapper_shares_adapter_network_and_ipc_for_cross_container_ros_payload -v
# RED: sim-adapter ipc was None instead of shareable
```

Compose now makes `sim-adapter` a project-local `ipc: shareable` owner and joins
only `slam-mapper` to it with both `network_mode: service:sim-adapter` and
`ipc: service:sim-adapter`. The shared runtime anchor was split so the mapper has
no mutually exclusive separate network attachment. No host network, host IPC,
published DDS/Gazebo port, static IP, or wider privilege was added.
`gazebo-server` remains on the internal `mentorpi` network and the existing
`GZ_RELAY_HOST=gazebo-server` health path is unchanged.

The post-change runtime gate used Compose project
`mentorpi-task4-r4-map-gate`, session `task4-r4-map-001`, and temporary volume
`mentorpi-task4-r4-slam-data`. The production `mentorpi-slam-data` volume and
`smoke-001`/`002`/`003` were not mounted:

```text
adapter hostname=fcdda3fcf986 ipc=ipc:[4026532561] net=net:[4026532567]
mapper  hostname=fcdda3fcf986 ipc=ipc:[4026532561] net=net:[4026532567]
scan_raw status=0; odom status=0; tf status=0; clock status=0
/map frame_id=map resolution=0.05 width=184 height=158
rosbag recorder: all six requested topics subscribed; recording
```

`/map` arrived on the first bounded poll without driving. Every diagnostic echo
had a 15-second timeout and every map poll had a 5-second timeout. Cleanup sent a
final zero Twist, signalled the mapper with SIGINT, removed the debug Compose
containers/network, and removed only `mentorpi-task4-r4-slam-data`. No debug
container, network, volume, override file, or session was retained.

Fresh verification output:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
# Ran 38 tests in 20.569s — OK

docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml --profile mapping config --quiet
# both exited 0
```

Only Compose, its contract test, operator documentation, and this report changed,
so the already-built `mentorpi-sim:harmonic` image did not require rebuilding.
The `/map` payload gate is complete; full Task 5 artifact finalization remains a
separate E2E responsibility.

## Fix round 5/5 (2026-07-27)

The mapper lifecycle now treats only operator `SIGINT` as permission to finalize
and publish. `SIGTERM` sets an abort request, never starts ROS save/serialize
service calls, performs bounded recorder/SLAM child cleanup, preserves staging,
and exits nonzero. A `SIGTERM` received during `SIGINT` finalization prevents
publication; the publish boundary rolls a just-renamed final directory back to
staging if the abort arrives across that boundary. An unexpected natural
`slam_toolbox` exit also preserves staging instead of implicitly finalizing.
Compose gives the mapper a 30-second stop grace for its bounded child cleanup.

`mapping-stop` now performs a host-side runtime gate before sending `SIGINT`:

1. inspect the current Compose `sim-adapter` full container ID and require it to
   be running and healthy;
2. require the running mapper's `HostConfig.NetworkMode` and `IpcMode` to both
   name that exact current container ID;
3. read `/proc/1/ns/net` and `/proc/1/ns/ipc` in both running containers and
   require the actual namespace identities to match.

Any failed gate invalidates the session. `mapping-stop` sends `SIGTERM`, waits
`MAPPING_ABORT_TIMEOUT_SECONDS` (default 10), uses `SIGKILL` only as a bounded
fallback, stops support services after confirmed mapper exit, and returns
nonzero. The final session is not published and `.inprogress` remains.

The behavior tests were added before implementation. RED showed that lifecycle
`SIGTERM` published with exit 0, unhealthy/recreated owners still received
`SIGINT`, and a same-ID owner with replaced actual namespaces still entered the
finalization wait. GREEN covers standalone and mid-finalization `SIGTERM`,
healthy current-owner `SIGINT`, unhealthy/new-ID/same-ID-stale aborts, already
exited cleanup, normal finalization timeout preservation, and forced bounded
abort cleanup. The real Compose renderer also checks `unless-stopped` adapter,
one-shot mapper, network+IPC service ownership, and the 30-second stop grace.

The runtime-gate image was built from the official Dockerfile as
`mentorpi-sim:harmonic` (image `7cf7e6495046`). After adding the atomic-boundary
abort regression, the same cached official build completed again as final image
`2a9e44f481fa`. The first ordinary legacy build
was bounded and cancelled after `docker-credential-desktop list` stalled. The
documented empty `DOCKER_CONFIG` plus `DOCKER_BUILDKIT=0` recovery reused the
local `ros:humble-ros-base-jammy` and apt layers, rebuilt all three colcon
packages, and completed all 14 Dockerfile steps.

Three isolated runtime gates used only temporary projects, volumes, and session
IDs:

- Automatic crash restart: project `mentorpi-task4-r5-auto4`, volume
  `mentorpi-task4-r5-auto4-data`, session `task4-r5-auto-004`. A temporary
  adapter PID-1 wrapper exited 137 after bounded child cleanup so Docker, rather
  than a manual `docker stop/kill`, exercised `unless-stopped`. The adapter
  recovered healthy with the same full ID and restart count 1, but the actual
  namespaces changed: adapter `net:[4026532825] ipc:[4026532567]`, mapper
  `net:[4026532570] ipc:[4026532563]`. Mapper-side scan timed out with 124 and
  odom/TF/clock all returned nonzero. `mapping-stop` detected the actual identity
  mismatch, aborted the mapper with exit 143, returned 7, and left only staging.
- Explicit Compose recreate: project `mentorpi-task4-r5-recreate2`, volume
  `mentorpi-task4-r5-recreate2-data`, session `task4-r5-recreate-002`.
  `docker compose up -d --no-deps --force-recreate sim-adapter` exited 0 and
  changed the adapter ID from `3331436...` to `313d7e4...`; the still-running
  mapper named the old ID for both namespaces. `mapping-stop` sent no `SIGINT`,
  aborted the mapper with exit 143, returned 7, and mapper logs contained no
  SaveMap, serialize-map, or service-call request. Read-only inspection found
  final absent and staging present.
- Normal current owner: project `mentorpi-task4-r5-normal`, volume
  `mentorpi-task4-r5-normal-data`, session `task4-r5-normal-001`. Adapter and
  mapper both reported `net:[4026532563] ipc:[4026532560]`; mapper received a
  `/map` payload; `mapping-stop` used `SIGINT`, returned 0, published final,
  removed staging, and passed `sha256sum -c`.

Every runtime cleanup sent a final zero Twist best-effort, removed only its exact
temporary Compose resources and volume, and retained no debug artifact. The
production `mentorpi-slam-data` volume was inspected by name and preserved;
`smoke-001` through `smoke-003` were never mounted or modified.

Fresh verification:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
# Ran 44 tests — OK

python3 vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_atomic_publish.py -v
# Ran 3 tests — OK (skipped=2 Linux-only cases on this Darwin host)

vehicle_simulator_model/ubuntu/run.sh test
# host/static suites passed; runtime ctest: 12 tests, 0 errors, 0 failures

docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml --profile mapping config --quiet
RENDER_GID=0 docker compose -f vehicle_simulator_model/ubuntu/compose.yaml \
  -f vehicle_simulator_model/ubuntu/compose.gpu.yaml config --quiet
bash -n vehicle_simulator_model/ubuntu/run.sh \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
git diff --check
# all exited 0
```
