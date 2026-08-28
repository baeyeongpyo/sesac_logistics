# Foxglove 사내망 직접 접속 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** headless Gazebo 서버의 Foxglove Bridge를 사내망 개발 PC의 로컬 Foxglove Studio에 직접 제공한다.

**Architecture:** browser viewer Compose에서 Foxglove Bridge를 `compose.foxglove.yaml`으로 분리한다. Docker 내부 모드에서는 loopback 8765 포트를, LAN 모드에서는 host networking과 host loopback DDS discovery를 사용한다. Caddy, noVNC, X11 browser viewer 구성은 삭제한다.

**Tech Stack:** Docker Compose, Gazebo Harmonic, ROS 2 Humble, Fast DDS Discovery Server, Foxglove Bridge, Python `unittest`, Bash.

## Global Constraints

- `SIM_NETWORK_MODE=lan`은 `GZ_SERVER_IP`가 필수이며 Gazebo·DDS·Bridge가 Linux server host network를 사용한다.
- Foxglove Studio는 사내망에서 `ws://<server-lan-ip>:8765`으로 직접 접속한다.
- host firewall은 신뢰된 개발자 CIDR만 TCP 8765에 허용하며 Gazebo Transport와 DDS discovery는 개발 PC에 공개하지 않는다.
- 외부 공개용 reverse proxy, TLS, 인증은 이 변경의 범위 밖이다.
- `llm-wiki-core` Project/Team artifact는 수정하지 않는다.

---

### Task 1: Bridge 전용 Compose 계약

**Files:**
- Create: `vehicle_simulator_model/ubuntu/compose.foxglove.yaml`
- Modify: `vehicle_simulator_model/ubuntu/compose.lan.yaml`
- Modify: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`

**Interfaces:**
- Consumes: `compose.yaml`의 `dds-discovery`, `sim-adapter`, `mentorpi` network와 `.env.<profile>` 환경 변수.
- Produces: `foxglove-bridge` 서비스. internal 모드에서는 `127.0.0.1:${FOXGLOVE_PORT-8765}:8765`, LAN 모드에서는 host network의 TCP 8765을 사용한다.

- [ ] **Step 1: Write the failing test**

`test_observation_bundle.py`에 `test_foxglove_bridge_has_no_browser_proxy_dependencies`를 추가한다. 이 테스트는 Bridge file의 `DDS_SUPER_CLIENT: "1"`, `sim-adapter` healthy dependency, internal loopback port와 LAN file의 host network/loopback discovery/empty ports를 검증한다.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 vehicle_simulator_model/ubuntu/test/test_observation_bundle.py -v`

Expected: FAIL because `compose.foxglove.yaml` does not exist.

- [ ] **Step 3: Write minimal implementation**

`compose.foxglove.yaml`에 `foxglove-bridge`만 정의한다. image/platform/restart, ROS/DDS 환경, `DDS_SUPER_CLIENT`, `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`, `sim-adapter` healthy dependency, internal loopback port를 둔다. `compose.lan.yaml`에는 Bridge의 `network_mode: host`, `networks: !reset []`, `DDS_DISCOVERY_HOST: 127.0.0.1`, `ports: !reset []`를 추가한다.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 vehicle_simulator_model/ubuntu/test/test_observation_bundle.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

Run: `git add vehicle_simulator_model/ubuntu/compose.foxglove.yaml vehicle_simulator_model/ubuntu/compose.lan.yaml vehicle_simulator_model/ubuntu/test/test_observation_bundle.py && git commit -m "feat: add direct LAN Foxglove bridge"`

### Task 2: Launcher의 Bridge lifecycle 통합

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/run.sh`
- Modify: `vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py`

**Interfaces:**
- Consumes: `compose.foxglove.yaml`과 선택한 `.env.<profile>`.
- Produces: `sim-up`이 Bridge를 포함해 시작하며 `foxglove-down`, `foxglove-logs`가 Bridge 단독 lifecycle을 제공한다.

- [ ] **Step 1: Write the failing test**

`test_runtime_env_config.py`에 `test_sim_up_uses_bridge_compose_and_starts_foxglove`와 `test_foxglove_lifecycle_targets_only_the_bridge`를 추가한다. 첫 테스트는 LAN `sim-up` Docker log에 `compose.foxglove.yaml`와 `<foxglove-bridge>`가 있는지, 둘째 테스트는 `foxglove-down`이 `<stop>`과 `<foxglove-bridge>`만 기록하는지 확인한다. `viewer-*` 명령 테스트는 제거한다.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py -v`

Expected: FAIL because `sim-up` does not select the Bridge Compose file and `foxglove-down` is unknown.

- [ ] **Step 3: Write minimal implementation**

`FOXGLOVE_COMPOSE` 배열에 base Compose와 `compose.foxglove.yaml`을 포함한다. LAN mode에서는 base 배열과 Bridge 배열 모두에 `compose.lan.yaml`을 추가한다. `sim-up`, `down`, `logs`, `test`에 Bridge lifecycle을 포함하고 `foxglove-down`/`foxglove-logs`를 추가한다. `viewer-*` command, public viewer validation, Compose version probe를 제거한다.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

Run: `git add vehicle_simulator_model/ubuntu/run.sh vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py && git commit -m "feat: manage Foxglove bridge with simulation"`

### Task 3: Browser viewer 런타임 제거와 운영 문서 갱신

**Files:**
- Delete: `vehicle_simulator_model/ubuntu/Caddyfile.viewer`
- Delete: `vehicle_simulator_model/ubuntu/compose.viewer.yaml`
- Delete: `vehicle_simulator_model/ubuntu/compose.viewer-public.yaml`
- Delete: `vehicle_simulator_model/ubuntu/viewer-entrypoint.sh`
- Delete: `vehicle_simulator_model/ubuntu/.env.server-viewer.example`
- Modify: `vehicle_simulator_model/ubuntu/Dockerfile`
- Modify: `vehicle_simulator_model/ubuntu/.env.server.example`
- Modify: `vehicle_simulator_model/ubuntu/README.md`
- Modify: `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- Modify: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`

**Interfaces:**
- Consumes: Bridge lifecycle from Task 2 and server profile’s `GZ_SERVER_IP`.
- Produces: browser-specific packages/files/commands가 없는 headless runtime 및 사내망 접속 절차 문서.

- [ ] **Step 1: Write the failing test**

`test_bundle.py`에 `test_runtime_contains_only_direct_foxglove_observation_assets`를 추가한다. 이 테스트는 browser viewer 파일이 없고 Dockerfile에 `novnc`, `websockify`, `x11vnc`, `xvfb`가 없으며 README에 `ws://<server-lan-ip>:8765`과 `TCP 8765`이 있는지 검증한다.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 vehicle_simulator_model/ubuntu/test/test_bundle.py -v && python3 vehicle_simulator_model/ubuntu/test/test_observation_bundle.py -v`

Expected: FAIL because browser viewer files and packages still exist.

- [ ] **Step 3: Write minimal implementation**

Dockerfile에서 `novnc`, `websockify`, `x11-utils`, `x11vnc`, `xvfb`, viewer entrypoint 복사를 제거한다. viewer Compose/Caddy/profile 파일을 삭제한다. `.env.server.example`에 `FOXGLOVE_PORT=8765`을 명시한다. README의 browser viewer 절을 사내망 Foxglove Studio 접속과 TCP 8765 firewall 규칙으로 교체한다.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 vehicle_simulator_model/ubuntu/test/test_bundle.py -v && python3 vehicle_simulator_model/ubuntu/test/test_observation_bundle.py -v`

Expected: PASS

- [ ] **Step 5: Commit**

Run: `git add -u vehicle_simulator_model/ubuntu && git add vehicle_simulator_model/ubuntu/Dockerfile vehicle_simulator_model/ubuntu/.env.server.example vehicle_simulator_model/ubuntu/README.md vehicle_simulator_model/ubuntu/test/test_bundle.py vehicle_simulator_model/ubuntu/test/test_observation_bundle.py && git commit -m "refactor: remove browser viewer runtime"`

### Task 4: 전체 Compose와 runtime 검증

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/run.sh` (검증 실패 시에만)
- Modify: `vehicle_simulator_model/ubuntu/test/*.py` (검증 실패 시에만)

**Interfaces:**
- Consumes: Tasks 1–3의 Compose, launcher, profile과 documentation 계약.
- Produces: Docker Compose 렌더링과 이미지 내 ROS package test를 통과한 headless bundle.

- [ ] **Step 1: Run internal과 LAN Compose 렌더링 검증**

Run: `cd vehicle_simulator_model/ubuntu && docker compose --env-file .env.dev.example -f compose.yaml -f compose.foxglove.yaml config --quiet && docker compose --env-file .env.server.example -f compose.yaml -f compose.foxglove.yaml -f compose.lan.yaml config --quiet`

Expected: PASS; LAN render에서 Bridge는 host network이고 `ports`가 없다.

- [ ] **Step 2: Run 전체 검증**

Run: `cd vehicle_simulator_model/ubuntu && ./run.sh --env dev test`

Expected: 모든 host-static, compose-config, runtime-ctest 단계가 PASS.

- [ ] **Step 3: Commit**

Run: `git add vehicle_simulator_model/ubuntu && git commit -m "test: verify direct LAN Foxglove runtime"`
