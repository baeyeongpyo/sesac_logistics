# Nav2 저장 지도 우선·SLAM 대체 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 저장 지도 우선으로 Nav2 목표 주행을 실행하고, 유효한 지도가 없으면 SLAM mapping 모드로 자동 전환한다.

**Architecture:** `mentorpi_nav` 패키지는 지도 세션 검증, Nav2 launch, Foxglove goal bridge를 소유한다. 기존 sim adapter는 센서·TF·Gazebo 구동만 계속 담당한다. localization 모드는 map_server+AMCL, mapping 모드는 slam_toolbox를 단독 map→odom 발행자로 사용한다.

**Tech Stack:** ROS 2 Humble, Navigation2, AMCL, SLAM Toolbox, rclpy, Docker Compose, unittest/pytest.

## Global Constraints

- 자율주행 대상은 `robot_1`이며 구동 토픽은 `/robot_1/controller/cmd_vel`이다.
- `map → robot_1/odom`은 AMCL 또는 SLAM Toolbox 중 하나만 발행한다.
- 지도 선택은 `map.yaml`, `map.pgm`, `manifest.json`, `checksums.sha256`의 checksum 검증을 통과해야 한다.
- goal 성공·실패·취소·유효하지 않은 goal은 모두 zero `Twist`로 종료한다.

---

### Task 1: Nav2 패키지와 지도 세션 선택기

**Files:**
- Create: `ros2_ws/src/mentorpi_nav/{CMakeLists.txt,package.xml}`
- Create: `ros2_ws/src/mentorpi_nav/mentorpi_nav/map_session.py`
- Create: `ros2_ws/src/mentorpi_nav/test/test_map_session.py`
- Modify: `Dockerfile`

**Interfaces:**
- Produces: `MapSession(path: Path, session_id: str)`, `find_valid_session(root, requested_id) -> MapSession | None`.

- [ ] Write tests for valid explicit selection, newest valid automatic selection, missing file, and checksum mismatch.
- [ ] Implement strict relative-path checksum parser and session selection.
- [ ] Add Navigation2, Nav2 bringup, AMCL dependencies and install the package.
- [ ] Run package unit tests and image build.

### Task 2: Nav2 parameters and launch modes

**Files:**
- Create: `ros2_ws/src/mentorpi_nav/config/nav2.yaml`
- Create: `ros2_ws/src/mentorpi_nav/launch/navigation.launch.py`
- Create: `ros2_ws/src/mentorpi_nav/test/test_navigation_launch.py`

**Interfaces:**
- Consumes: `mode` (`localization` or `mapping`), `map_yaml`, existing `/map`, `/robot_1/scan_raw`, TF.
- Produces: Nav2 lifecycle servers and `/navigate_to_pose` action.

- [ ] Write launch contract tests for frames, scan, costmap layers, controller remap, and mutually exclusive AMCL/SLAM providers.
- [ ] Configure conservative Mecanum footprint, velocity limits, static/inflation global layers and scan obstacle/inflation local layers.
- [ ] Implement localization launch with `nav2_bringup` map_server/AMCL and mapping launch without these nodes.
- [ ] Run static and ROS launch tests.

### Task 3: Foxglove goal bridge and safety stop

**Files:**
- Create: `ros2_ws/src/mentorpi_nav/mentorpi_nav/goal_bridge.py`
- Create: `ros2_ws/src/mentorpi_nav/test/test_goal_bridge.py`
- Modify: `ros2_ws/src/mentorpi_nav/setup.py`

**Interfaces:**
- Consumes: `/move_base_simple/goal` (`geometry_msgs/PoseStamped`).
- Produces: `nav2_msgs/action/NavigateToPose` goals and zero `Twist` on `/robot_1/controller/cmd_vel` on every terminal path.

- [ ] Write node-method tests for map-frame validation, active goal cancellation, rejected server, terminal stop, and result publication.
- [ ] Implement asynchronous action client, status topic, frame validation, preemption and stop helper.
- [ ] Run goal bridge tests.

### Task 4: Compose and command lifecycle

**Files:**
- Modify: `compose.yaml`
- Modify: `run.sh`
- Modify: `README.md`
- Modify: `test/test_bundle.py`

**Interfaces:**
- Adds: `nav-up auto [session-id]`, `nav-down`, `nav-status`.
- Produces: service environment variables `NAV_MODE`, `NAV_MAP_YAML`, `NAV_SESSION_ID`.

- [ ] Write static command and compose tests.
- [ ] Add `navigation` service with read-only `slam-data` volume and sim-adapter dependency.
- [ ] Implement deterministic `nav-up` selection and mode log; mapping fallback starts mapping provider only when no session validates.
- [ ] Ensure map publishing transition stops navigation before mapping session changes TF provider.
- [ ] Run bundle tests and compose config validation.

### Task 5: Runtime verification

**Files:**
- Modify: `README.md`
- Modify: `healthcheck.sh` only if a bounded navigation health probe is necessary.

- [ ] Build image and run tests.
- [ ] Start localization using valid `sim-map-01`; verify lifecycle, AMCL map→odom, goal action, cmd_vel and terminal zero command.
- [ ] Start mapping fallback using a volume with no valid session; verify SLAM mode and absence of AMCL.
- [ ] Publish a safe map-frame Foxglove-compatible goal and verify a result or explicit planner rejection without residual vehicle motion.
- [ ] Document launch, stop, mode selection, and Foxglove click workflow.
