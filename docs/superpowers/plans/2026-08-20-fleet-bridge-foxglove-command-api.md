# Fleet Bridge Foxglove Command API Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 차량 Foxglove Bridge의 원본 telemetry를 서버 namespace로 중계하고 Swagger UI에서 시험할 수 있는 `cmd_vel`·`stop` REST API를 제공한다.

**Architecture:** 서버 telemetry worker는 remote channel의 원본 `source` topic을 선택해 Domain 225의 `target` namespace로 재발행한다. 독립 command API 서비스는 설정된 차량 URI에 WebSocket v1 client로 연결하고 `geometry_msgs/msg/Twist` CDR을 client-publish channel에 보내며, FastAPI가 OpenAPI와 Swagger UI를 생성한다.

**Tech Stack:** ROS 2 Humble, Python 3.10, FastAPI, Uvicorn, websockets 10.4, Foxglove WebSocket v1, Docker Compose.

**Spec:** `docs/superpowers/specs/2026-08-20-fleet-bridge-foxglove-command-design.md`

## Global Constraints

- 차량 ROS Domain은 `robot_1=215`, `robot_2=216`, 서버 ROS Domain은 `225`다.
- 차량과 서버 사이 DDS/domain bridge는 추가하지 않고 `foxglove.websocket.v1`만 사용한다.
- 원격 telemetry 선택은 `TopicConfig.source`, 서버 publisher는 `TopicConfig.target`이다.
- vehicle identity는 server-side `fleet.yaml` URI 매핑만 신뢰한다.
- command는 `geometry_msgs/msg/Twist` `/cmd_vel`, `stop` zero Twist만 제공한다.
- API 기본 bind host는 `127.0.0.1`이며 `hold_ms` 완료와 stop에서 zero Twist를 반드시 발행한다.
- Swagger UI는 `/docs`, OpenAPI schema는 `/openapi.json`, 상태 확인은 `/healthz`다.
- 기존 pin인 Foxglove Bridge 0.8.5와 `websockets==10.4`를 바꾸지 않는다.

---

### Task 1: Command 설정 모델과 raw telemetry 선택

**Files:**
- Modify: `fleet_bridge/common/fleet_bridge_config/fleet_bridge_config/models.py`
- Modify: `fleet_bridge/common/fleet_bridge_config/fleet_bridge_config/loader.py`
- Modify: `fleet_bridge/config/fleet.yaml`
- Modify: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/republisher.py`
- Modify: `fleet_bridge/common/fleet_bridge_config/test/test_loader.py`
- Modify: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_republisher.py`

**Interfaces:**
- Consumes: `fleet.yaml` server/vehicle configuration and `TopicConfig.source`, `TopicConfig.target`.
- Produces: immutable `CommandApiConfig(host, port)`, `CommandConfig(topic, message_type, max_linear_x, max_angular_z, max_hold_ms, publish_rate_hz)`, `VehicleConfig.command`, and `ChannelSelector.select()` matching raw source topics.

- [ ] **Step 1: Write the failing config and selector tests**

  Extend `test_loader.py` with a valid command block and assert `loaded.server.command_api.port == 8080`, `loaded.vehicles[0].command.topic == '/cmd_vel'`, plus invalid zero speed/hold, invalid command topic, and non-Twist message type cases. Change `test_republisher.py` so a configured source `/odom`, an uplink `/{robot}/odom`, and a channel `/odom` select the topic while a channel `/{robot}/odom` does not.

- [ ] **Step 2: Run the new tests to verify failure**

  Run: `python3 -m unittest fleet_bridge.common.fleet_bridge_config.test.test_loader fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_republisher -v`

  Expected: FAIL because `command_api`, `command`, and raw source selection are not implemented.

- [ ] **Step 3: Implement the minimal model, parser, and selector change**

  Add immutable `CommandApiConfig` and `CommandConfig` dataclasses. Require `server.command_api` (`host`, `port`) and each vehicle `command` (`topic`, `type`, `max_linear_x`, `max_angular_z`, `max_hold_ms`, `publish_rate_hz`) in `load_fleet`; validate a loopback-or-IP host, port 1–65535, absolute topic, `geometry_msgs/msg/Twist`, positive finite limits, `max_hold_ms` 1–60000, and rate 1–100 Hz. Make `ChannelSelector` index enabled topics by `topic.source` rather than `topic.uplink`, retaining CDR and schema-name matching. Update `fleet.yaml` with `127.0.0.1:8080`, `/cmd_vel`, 0.3 m/s, 1.0 rad/s, 1000 ms, and 10 Hz safe defaults.

- [ ] **Step 4: Run focused tests to verify pass**

  Run: `python3 -m unittest fleet_bridge.common.fleet_bridge_config.test.test_loader fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_republisher -v`

  Expected: PASS.

- [ ] **Step 5: Commit**

  Run: `git add fleet_bridge/common/fleet_bridge_config fleet_bridge/config/fleet.yaml fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/republisher.py fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_republisher.py && git commit -m "feat: configure raw Foxglove telemetry and commands"`

### Task 2: Vehicle raw Bridge launch policy

**Files:**
- Modify: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/fleet_telemetry_filter/launch_config.py`
- Modify: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/launch/vehicle_foxglove.launch.py`
- Modify: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/test/test_launch_config.py`
- Modify: `fleet_bridge/docker-compose.vehicle.yaml`

**Interfaces:**
- Consumes: `TopicConfig.source`, `VehicleConfig.command`, environment `FOXGLOVE_MODE=raw`.
- Produces: raw vehicle Bridge params exposing enabled source telemetry and admitting only the configured `/cmd_vel` through `clientPublish`.

- [ ] **Step 1: Write the failing raw Bridge parameter tests**

  Add a `raw` mode test that asserts `^/odom$` and `^/tf$` are whitelist entries, `^/robot_1/odom$` is absent, `capabilities == ['clientPublish']`, and `client_topic_whitelist == ['^/cmd_vel$']`. Add an invalid mode case for any mode except `raw` or legacy `fleet`/`debug`.

- [ ] **Step 2: Run the vehicle configuration test to verify failure**

  Run: `python3 -m unittest fleet_bridge.vehicle.ros2_ws.src.fleet_telemetry_filter.test.test_launch_config -v`

  Expected: FAIL because raw mode and command whitelist do not exist.

- [ ] **Step 3: Implement raw mode without starting the sidecar filter**

  Pass `VehicleConfig.command` to `bridge_parameters`. For `raw`, use enabled `source` topics and the configured command topic, use `clientPublish`, and keep service/parameter/asset deny lists. In `vehicle_foxglove.launch.py`, default `FOXGLOVE_MODE` to `raw`, accept the fleet configuration path, and only create `fleet_telemetry_filter` in explicit legacy `fleet` mode. Set the Compose service default to raw, while documenting that it must execute within the driving container's ROS graph rather than pretending an isolated sidecar can observe it.

- [ ] **Step 4: Run the test to verify pass**

  Run: `python3 -m unittest fleet_bridge.vehicle.ros2_ws.src.fleet_telemetry_filter.test.test_launch_config -v`

  Expected: PASS.

- [ ] **Step 5: Commit**

  Run: `git add fleet_bridge/vehicle fleet_bridge/docker-compose.vehicle.yaml && git commit -m "feat: expose raw telemetry and cmd vel through vehicle bridge"`

### Task 3: Foxglove client-publish protocol and command worker

**Files:**
- Modify: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/protocol.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/command.py`
- Modify: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_protocol.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_command.py`

**Interfaces:**
- Consumes: `VehicleConfig`, `CommandConfig`, Foxglove `serverInfo`, a serialized ROS 2 Twist payload.
- Produces: `client_advertise_message(channel_id, topic, schema_name) -> str`, `client_message_frame(channel_id, payload) -> bytes`, and async `FoxgloveCommandClient.send_twist(robot_id, linear_x, angular_z, hold_ms)` / `stop(robot_id)`.

- [ ] **Step 1: Write failing protocol and command worker tests**

  Extend `test_protocol.py` to assert an advertise JSON object with `op: 'advertise'`, id `1`, topic `/cmd_vel`, encoding `cdr`, and schema name `geometry_msgs/msg/Twist`, then assert a client binary frame equals `b'\\x01' + struct.pack('<I', 1) + payload`. In `test_command.py`, use a fake async WebSocket that sends a `serverInfo` containing `clientPublish` and `cdr`; assert command ordering is advertise, nonzero Twist frame(s), zero Twist frame; assert missing capability raises `ProtocolError` and produces no command frame.

- [ ] **Step 2: Run command tests to verify failure**

  Run: `python3 -m unittest fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_protocol fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_command -v`

  Expected: FAIL because client publish functions and command client do not exist.

- [ ] **Step 3: Implement protocol builders and bounded command worker**

  Add JSON advertise and little-endian client-message builders according to Foxglove WebSocket v1. The command worker must negotiate `foxglove.websocket.v1`, wait for `serverInfo`, require `clientPublish` and `cdr`, advertise channel id `1`, serialize `Twist` through `rclpy.serialization.serialize_message`, publish at configured rate for `hold_ms`, and always send a zero Twist in `finally`. Clamp neither input nor protocol silently; expose a `CommandValidationError` before calling the worker when the API receives unsafe values. Close the socket after each short test command so an interrupted process cannot retain a stale command channel.

- [ ] **Step 4: Run command tests to verify pass**

  Run: `python3 -m unittest fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_protocol fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_command -v`

  Expected: PASS.

- [ ] **Step 5: Commit**

  Run: `git add fleet_bridge/server/ros2_ws/src/foxglove_ros_worker && git commit -m "feat: publish bounded Twist commands through Foxglove"`

### Task 4: FastAPI, OpenAPI, and Swagger UI

**Files:**
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/api.py`
- Modify: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/setup.py`
- Modify: `fleet_bridge/server/Dockerfile`
- Modify: `fleet_bridge/docker-compose.server.yaml`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_api.py`

**Interfaces:**
- Consumes: loaded `FleetConfig` and `FoxgloveCommandClient` passed through a FastAPI application factory.
- Produces: `create_app(fleet, command_client)`, `POST /api/v1/robots/{robot_id}/cmd_vel`, `POST /api/v1/robots/{robot_id}/stop`, `GET /healthz`, `/docs`, and `/openapi.json`.

- [ ] **Step 1: Write failing API tests**

  In `test_api.py`, create an app using a recording command client. Assert valid POST `/api/v1/robots/robot_1/cmd_vel` returns 202 and records `(0.1, 0.0, 300)`; invalid `hold_ms=0` and speed above configured limit return 422; unknown and disabled robots return 404; a command connection error returns 503; `/docs` returns 200 with `swagger-ui`; `/openapi.json` contains both command paths; and stop calls the client's `stop('robot_1')`.

- [ ] **Step 2: Run API tests to verify failure**

  Run: `python3 -m unittest fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_api -v`

  Expected: FAIL because FastAPI and the application factory do not exist.

- [ ] **Step 3: Implement the minimal API and container entry point**

  Use Pydantic request fields `linear_x`, `angular_z`, and `hold_ms` with documented Swagger examples; check vehicle existence/enabled state and configured limits before awaiting the command client. Map operational WebSocket failures to 503. Add `fleet_command_api = foxglove_ros_worker.api:main` console script. Install pinned `fastapi==0.115.12` and `uvicorn==0.34.0` alongside existing `websockets==10.4` into `/opt/python`. Add `command-api` Compose service with host networking, read-only fleet config mount, `COMMAND_API_HOST`, `COMMAND_API_PORT`, and no ROS Domain requirement.

- [ ] **Step 4: Run API tests to verify pass**

  Run: `python3 -m unittest fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_api -v`

  Expected: PASS.

- [ ] **Step 5: Commit**

  Run: `git add fleet_bridge/server fleet_bridge/docker-compose.server.yaml && git commit -m "feat: add Swagger command test API"`

### Task 5: Deployment contracts and documentation

**Files:**
- Modify: `fleet_bridge/.env.example`
- Modify: `fleet_bridge/README.md`
- Modify: `fleet_bridge/test/test_compose_contract.py`
- Modify: `fleet_bridge/test/test_bundle_contract.py`

**Interfaces:**
- Consumes: completed config, vehicle launch, command API, Docker Compose services.
- Produces: repeatable raw vehicle Bridge and server Swagger test instructions.

- [ ] **Step 1: Write failing bundle contract tests**

  Extend Compose/bundle tests to require a `command-api` service, `COMMAND_API_HOST`/`COMMAND_API_PORT`, raw vehicle mode, `clientPublish`, the `/cmd_vel` command whitelist, and README mentions for `http://<server-ip>:8080/docs`, the two REST paths, vehicle-in-container requirement, and zero-Twist behavior.

- [ ] **Step 2: Run contract tests to verify failure**

  Run: `python3 -m unittest fleet_bridge.test.test_compose_contract fleet_bridge.test.test_bundle_contract -v`

  Expected: FAIL because deployment files do not expose command API or raw mode instructions.

- [ ] **Step 3: Update env sample and operational documentation**

  Add `COMMAND_API_HOST=127.0.0.1`, `COMMAND_API_PORT=8080`, and raw vehicle Foxglove URI examples. Replace the legacy filter-first architecture diagram with direct raw telemetry and command direction. Document build/run commands, Swagger URL, curl examples for `cmd_vel`/`stop`, explicit no-auth test warning, firewall requirement, and verification commands for server worker plus API health.

- [ ] **Step 4: Run contracts and full Python suite**

  Run: `python3 -m unittest discover -s fleet_bridge -p 'test_*.py' -v`

  Expected: PASS.

- [ ] **Step 5: Build and inspect the server image**

  Run: `docker build -f fleet_bridge/server/Dockerfile -t mentorpi-fleet-bridge-server:humble fleet_bridge && docker run --rm --entrypoint bash mentorpi-fleet-bridge-server:humble -lc 'source /opt/ros/humble/setup.bash && source /opt/fleet_bridge_ws/install/setup.bash && python3 -c "import fastapi, uvicorn; print(fastapi.__version__)"'`

  Expected: build succeeds and FastAPI imports from the runtime image.

- [ ] **Step 6: Commit**

  Run: `git add fleet_bridge/.env.example fleet_bridge/README.md fleet_bridge/test && git commit -m "docs: document Foxglove command API deployment"`
