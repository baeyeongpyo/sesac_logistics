# MentorPi Gazebo Shared Observation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Docker가 실행하는 단일 MentorPi Gazebo 월드를 동일 LAN의 Mac 네이티브 `gz sim -g`와 외부 팀원의 read-only 웹 화면에서 동시에 관찰할 수 있게 한다.

**Architecture:** 기본 `sim-up`은 현재처럼 외부 포트가 없는 내부 Docker 네트워크에서 동작한다. 선택적인 LAN 모드는 Gazebo Transport를 라우팅 가능한 호스트 네트워크에 연결하고, 선택적인 viewer 모드는 Docker 내부에서 실제 `gz sim -g`를 Xvfb로 실행해 noVNC로 전달한 뒤 HTTPS gateway에서 IP allowlist를 적용한다.

**Tech Stack:** Gazebo Harmonic 8.14.0, ROS 2 Humble, Docker Compose 2.24.4+, Docker Desktop 4.34+, Xvfb, x11vnc, noVNC, websockify, Caddy 2.10, Bash, Python `unittest`

## Global Constraints

- 기본 `./run.sh sim-up`은 현재의 `internal: true` 네트워크와 외부 포트 미공개 계약을 유지한다.
- authoritative simulation은 Docker의 `gazebo-server` 한 개이며 별도의 Mac Gazebo server를 실행하지 않는다.
- 월드 이름은 `mentorpi_warehouse`, 로봇 이름은 `robot_1`, `robot_2`를 유지한다.
- 차량 spawn, ROS bridge, odom, TF 책임은 계속 `sim-adapter`가 소유한다.
- LAN 네이티브 GUI는 같은 LAN처럼 양방향 IP 라우팅이 가능한 네트워크에서만 지원한다.
- LAN 모드는 신뢰된 네트워크에서만 활성화하고 Linux host firewall은 승인된 개발 CIDR의 트래픽만 허용한다.
- LAN 네이티브 GUI는 입력을 기술적으로 차단하지 않으므로 한 명만 operator로 지정하고 나머지는 화면 관찰만 수행한다.
- 공용 인터넷에 Gazebo Transport, ROS 2 DDS, VNC 5900, noVNC 6080을 직접 공개하지 않는다.
- 외부 웹 모니터링은 Caddy의 80/443만 공개하며, production 실행은 비어 있지 않은 IP allowlist를 필수로 한다.
- 웹 viewer는 `x11vnc -viewonly -shared`를 사용해 여러 팀원이 동시에 볼 수 있지만 Gazebo GUI 입력은 전달하지 않는다.
- 애플리케이션 로그인은 추가하지 않는다. 허용 대상은 방화벽과 Caddy source CIDR allowlist로 제한한다.
- GUI와 viewer 장애는 `gazebo-server`, `sim-adapter`, `slam-mapper`를 중단시키지 않는다.
- Mac과 Docker의 Gazebo major version은 8로 고정하고 release smoke test에서는 양쪽 `gz sim --versions`의 첫 줄이 `8.14.0`인지 확인한다.
- Nav2 자동 주행, 실제 차량 목적지 전달, 운영 지도 전환 조건은 이 계획 범위에 포함하지 않는다.
- 기존 사용자 변경인 `vehicle_simulator_model/ubuntu/README.md`, `vehicle_simulator_model/ubuntu/test/test_bundle.py`, `vehicle_simulator_model/ubuntu/ros2_ws/model.sdf`는 구현 시작 시 재검토하고 덮어쓰지 않는다.
- `docs/superpowers/plans/2026-07-26-mentorpi-gazebo-web-monitor.md`의 웹 전용·basic-auth 전제는 이 계획으로 대체한다.

---

## Confirmed Baseline

현재 구현과 런타임에서 다음 사실을 확인했다.

- `./run.sh sim-up`은 `dds-discovery`, `gazebo-server`, `sim-adapter`를 실행한다.
- `gazebo-server`와 `sim-adapter` healthcheck가 모두 `healthy`이다.
- `/robot_1/scan_raw`, `/robot_1/odom`, `/robot_2/scan_raw`, `/robot_2/odom`, `/tf`, `/clock` payload가 진행된다.
- Docker 내부 `gz topic -l`에서는 차량과 센서 토픽이 보인다.
- 현재 Mac에서 Docker bridge IP를 `GZ_RELAY`로 지정한 `gz topic -l`은 토픽을 찾지 못한다.
- 현재 Compose는 `internal: true`이며 Gazebo Transport와 viewer 포트를 공개하지 않는다.
- Mac과 Docker의 `gz sim --versions` 첫 줄은 모두 `8.14.0`이다.
- 차량 SDF의 LiDAR와 depth camera mesh는 Docker 설치 경로로 확장되는 `file://$(find ...)` URI를 사용한다.
- 현재 이미지에는 `Xvfb`, `x11vnc`, `websockify`, noVNC가 없다.

## Target Use Cases

| Use case | 실행 위치 | 관찰 방법 | 접근 경계 |
| --- | --- | --- | --- |
| 모델·월드 편집 | Mac 개발 PC | Mac 네이티브 Gazebo | 로컬 파일 |
| 로컬 Docker 통합 테스트 | Mac Docker Desktop | 검증된 경우 Mac `gz sim -g`, 아니면 local viewer | localhost |
| 사내 LAN 공유 시뮬레이션 | Linux Docker 서버 | 여러 Mac의 `gz sim -g` | 같은 LAN CIDR |
| 외부 팀 모니터링 | Linux Docker 서버 | read-only browser viewer | HTTPS 443 + source IP allowlist |
| SLAM 데이터 생성 | Docker 서버 | `mapping-up`, healthcheck, logs | Docker 내부 |

## File Map

- Create `vehicle_simulator_model/ubuntu/compose.lan.yaml`: Gazebo/ROS 서비스의 host-network LAN override
- Create `vehicle_simulator_model/ubuntu/compose.viewer.yaml`: internal viewer와 local gateway 서비스
- Create `vehicle_simulator_model/ubuntu/compose.viewer-public.yaml`: 80/443 production exposure override
- Create `vehicle_simulator_model/ubuntu/viewer-entrypoint.sh`: Xvfb, `gz sim -g`, x11vnc, websockify lifecycle
- Create `vehicle_simulator_model/ubuntu/Caddyfile.viewer`: source CIDR allowlist와 noVNC reverse proxy
- Create `vehicle_simulator_model/ubuntu/scripts/gz-gui-connect.sh`: Mac native GUI preflight와 실행
- Create `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`: LAN/viewer/security static contract
- Create `vehicle_simulator_model/ubuntu/test/smoke_observation.sh`: running stack 관찰 경로 smoke test
- Modify `vehicle_simulator_model/ubuntu/Dockerfile`: viewer runtime packages와 entrypoint 설치
- Modify `vehicle_simulator_model/ubuntu/entrypoint.sh`: portable Gazebo resource path export
- Modify `vehicle_simulator_model/ubuntu/run.sh`: network mode와 viewer 운영 명령
- Modify `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro`: portable mesh URI
- Modify `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py`: portable URI contract
- Modify `vehicle_simulator_model/ubuntu/test/test_bundle.py`: 새 파일과 기본 network isolation 계약
- Modify `vehicle_simulator_model/ubuntu/README.md`: 환경별 실행·접속·종료 가이드
- Modify `docs/superpowers/plans/2026-07-26-mentorpi-gazebo-web-monitor.md`: superseded 안내만 추가

---

### Task 1: Portable Gazebo scene resources

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro`
- Modify: `vehicle_simulator_model/ubuntu/entrypoint.sh`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py`
- Test: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py`

**Interfaces:**
- Consumes: source package directory `ros2_ws/src/mentorpi_description`
- Produces: `model://mentorpi_description/...` mesh URIs resolvable in Docker, Mac checkout, and viewer
- Produces: `GZ_SIM_RESOURCE_PATH` containing installed package share parents

- [ ] **Step 1: Write the failing portable-resource contract**

Add this test:

```python
def test_gazebo_mesh_uris_are_portable_between_server_and_gui(self):
    model = SDF.read_text()
    self.assertIn(
        'model://mentorpi_description/meshes/mecanum/lidar_Link.STL',
        model,
    )
    self.assertIn(
        'model://mentorpi_description/meshes/mecanum/cam_Link.STL',
        model,
    )
    self.assertNotIn('file://$(find mentorpi_description)', model)
    self.assertNotIn('/opt/mentorpi_ws', model)
```

- [ ] **Step 2: Run the test and verify the current absolute-resource contract fails**

Run:

```bash
python3 \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py \
  -v
```

Expected: FAIL because both mesh URIs still start with `file://$(find mentorpi_description)`.

- [ ] **Step 3: Replace both mesh URIs**

Use these exact values:

```xml
<uri>model://mentorpi_description/meshes/mecanum/lidar_Link.STL</uri>
```

```xml
<uri>model://mentorpi_description/meshes/mecanum/cam_Link.STL</uri>
```

- [ ] **Step 4: Export installed resource paths from the common entrypoint**

Insert before `exec "$@"`:

```bash
resource_paths=(
  /opt/mentorpi_ws/install/mentorpi_description/share
  /opt/mentorpi_ws/install/mentorpi_gz_sim/share
)
if [[ -d /ws/install/mentorpi_description/share ]]; then
  resource_paths+=(/ws/install/mentorpi_description/share)
fi
if [[ -d /ws/install/mentorpi_gz_sim/share ]]; then
  resource_paths+=(/ws/install/mentorpi_gz_sim/share)
fi
resource_path_value="$(IFS=:; printf '%s' "${resource_paths[*]}")"
if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
  resource_path_value="${resource_path_value}:${GZ_SIM_RESOURCE_PATH}"
fi
export GZ_SIM_RESOURCE_PATH="$resource_path_value"
```

- [ ] **Step 5: Add an expanded-SDF runtime assertion**

Extend the image test command to run:

```bash
xacro \
  /opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro \
  robot_name:=robot_1 \
  | tee /tmp/robot_1.sdf \
  | grep -F 'model://mentorpi_description/meshes/mecanum/lidar_Link.STL'
```

Expected: the expanded SDF contains portable URIs and no Docker absolute mesh path.

- [ ] **Step 6: Run model and bundle tests**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add \
  vehicle_simulator_model/ubuntu/entrypoint.sh \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py
git commit -m "fix: Gazebo GUI용 모델 리소스 경로 이식"
```

---

### Task 2: Optional LAN transport profile and Mac GUI client

**Files:**
- Create: `vehicle_simulator_model/ubuntu/compose.lan.yaml`
- Create: `vehicle_simulator_model/ubuntu/scripts/gz-gui-connect.sh`
- Create: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- Modify: `vehicle_simulator_model/ubuntu/run.sh`
- Test: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`

**Interfaces:**
- Consumes: `SIM_NETWORK_MODE=internal|lan`
- Consumes: `GZ_SERVER_IP=<IP local to the Docker host>`
- Produces: the existing `COMPOSE` array plus `compose.lan.yaml` only in LAN mode
- Produces: `gz-gui-connect.sh <server-ip> <client-ip>`
- Preserves: default `SIM_NETWORK_MODE=internal`

- [ ] **Step 1: Write the failing LAN profile contract**

Create `test_observation_bundle.py` with:

```python
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]


class ObservationBundleTest(unittest.TestCase):
    def test_lan_profile_uses_host_network_without_changing_base_compose(self):
        base = (BUNDLE / 'compose.yaml').read_text()
        lan = (BUNDLE / 'compose.lan.yaml').read_text()
        self.assertIn('internal: true', base)
        self.assertNotIn('network_mode: host', base)
        for service in ('dds-discovery:', 'gazebo-server:', 'sim-adapter:',
                        'slam-mapper:', 'slam-inspector:'):
            self.assertIn(service, lan)
        self.assertIn('network_mode: host', lan)
        self.assertIn('networks: !reset []', lan)
        self.assertIn('GZ_IP: "${GZ_SERVER_IP:?', lan)
        self.assertIn('DDS_DISCOVERY_HOST: 127.0.0.1', lan)
        self.assertIn('GZ_RELAY_HOST: ""', lan)

    def test_mac_client_sets_one_partition_and_runs_gui_only(self):
        script = (BUNDLE / 'scripts/gz-gui-connect.sh').read_text()
        self.assertIn('GZ_PARTITION="${GZ_PARTITION:-mentorpi-sim}"', script)
        self.assertIn('GZ_RELAY="$server_ip"', script)
        self.assertIn('GZ_IP="$client_ip"', script)
        self.assertIn('gz topic -l', script)
        self.assertIn('gz sim --force-version 8 -g', script)
        self.assertNotIn('gz sim -s', script)
```

- [ ] **Step 2: Run the contract and verify missing LAN assets**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py \
  -v
```

Expected: ERROR because `compose.lan.yaml` and `scripts/gz-gui-connect.sh` do not exist.

- [ ] **Step 3: Implement the LAN override**

Create `compose.lan.yaml`:

```yaml
services:
  dds-discovery:
    network_mode: host
    networks: !reset []

  gazebo-server:
    network_mode: host
    networks: !reset []
    environment:
      GZ_IP: "${GZ_SERVER_IP:?set GZ_SERVER_IP to a Docker-host IP}"

  sim-adapter:
    network_mode: host
    networks: !reset []
    environment:
      GZ_IP: "${GZ_SERVER_IP:?set GZ_SERVER_IP to a Docker-host IP}"
      GZ_RELAY_HOST: ""
      DDS_DISCOVERY_HOST: 127.0.0.1

  slam-data-init:
    network_mode: host
    networks: !reset []

  slam-mapper:
    network_mode: host
    networks: !reset []
    environment:
      DDS_DISCOVERY_HOST: 127.0.0.1

  slam-inspector:
    network_mode: host
    networks: !reset []
```

Compose 2.24.4+ is required because the observation overrides use the `!reset`
and `!override` tags.

- [ ] **Step 4: Add network-mode selection to `run.sh`**

Add:

```bash
configure_network_mode() {
  case "${SIM_NETWORK_MODE:-internal}" in
    internal)
      ;;
    lan)
      if [[ -z "${GZ_SERVER_IP:-}" ]]; then
        echo 'GZ_SERVER_IP is required when SIM_NETWORK_MODE=lan' >&2
        exit 2
      fi
      COMPOSE+=( -f "$BUNDLE_DIR/compose.lan.yaml" )
      ;;
    *)
      echo 'SIM_NETWORK_MODE must be internal or lan' >&2
      exit 2
      ;;
  esac
}

configure_network_mode
```

Call it after the base `COMPOSE` array is created and before command dispatch so `sim-up`, `mapping-up`, `mapping-stop`, logs, topics, and down all use the same selected mode.

- [ ] **Step 5: Implement the Mac GUI client preflight**

Create executable `scripts/gz-gui-connect.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 2 ]]; then
  echo 'Usage: gz-gui-connect.sh <server-ip> <client-ip>' >&2
  exit 2
fi

server_ip="$1"
client_ip="$2"
bundle_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export GZ_IP="$client_ip"
export GZ_RELAY="$server_ip"
export GZ_PARTITION="${GZ_PARTITION:-mentorpi-sim}"
export GZ_SIM_RESOURCE_PATH="$bundle_dir/ros2_ws/src${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if ! gz sim --versions | grep -Fxq '8.14.0'; then
  echo 'Gazebo Sim 8.14.0 is required on the GUI client' >&2
  exit 3
fi

topics="$(gz topic -l)"
if ! grep -Fxq '/world/mentorpi_warehouse/stats' <<<"$topics"; then
  echo "Gazebo server is not reachable at ${server_ip} on partition ${GZ_PARTITION}" >&2
  exit 4
fi

exec gz sim --force-version 8 -g
```

- [ ] **Step 6: Test invalid modes and GUI preflight with a fake `gz`**

Add tests that place a fake `gz` first in `PATH`.

The fake returns:

```bash
case "${1:-} ${2:-}" in
  'sim --versions') printf '8.14.0\n' ;;
  'topic -l') printf '/world/mentorpi_warehouse/stats\n' ;;
  'sim --force-version') printf '%s|%s|%s\n' "$GZ_IP" "$GZ_RELAY" "$GZ_PARTITION" ;;
esac
```

Assert:

```python
self.assertEqual(result.returncode, 0)
self.assertIn('192.168.50.20|192.168.50.10|mentorpi-sim', result.stdout)
```

Also assert that `SIM_NETWORK_MODE=lan` without `GZ_SERVER_IP` exits 2 before calling Docker.

- [ ] **Step 7: Validate Compose**

Run:

```bash
docker compose \
  -f vehicle_simulator_model/ubuntu/compose.yaml \
  config --quiet

GZ_SERVER_IP=192.168.50.10 docker compose \
  -f vehicle_simulator_model/ubuntu/compose.yaml \
  -f vehicle_simulator_model/ubuntu/compose.lan.yaml \
  config --quiet
```

Expected: both configurations pass; the base config remains on the internal network.

- [ ] **Step 8: Commit**

```bash
git add \
  vehicle_simulator_model/ubuntu/compose.lan.yaml \
  vehicle_simulator_model/ubuntu/scripts/gz-gui-connect.sh \
  vehicle_simulator_model/ubuntu/run.sh \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py
git commit -m "feat: LAN Gazebo GUI 연결 프로필 추가"
```

---

### Task 3: Linux LAN and Mac Docker Desktop release gates

**Files:**
- Create: `vehicle_simulator_model/ubuntu/test/smoke_observation.sh`
- Modify: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- Test: `vehicle_simulator_model/ubuntu/test/smoke_observation.sh`

**Interfaces:**
- Consumes: a running `SIM_NETWORK_MODE=lan` stack
- Consumes: `GZ_SERVER_IP`, `GZ_CLIENT_IP`
- Produces: exit 0 only when two independent Gazebo clients discover the same world
- Produces: an explicit Mac Docker Desktop supported/unsupported result

- [ ] **Step 1: Add the smoke-script static contract**

Add:

```python
def test_lan_smoke_checks_two_clients_and_both_robots(self):
    script = (BUNDLE / 'test/smoke_observation.sh').read_text()
    self.assertIn('client-a', script)
    self.assertIn('client-b', script)
    self.assertIn('/world/mentorpi_warehouse/stats', script)
    self.assertIn('/robot_1/scan_raw', script)
    self.assertIn('/robot_2/scan_raw', script)
```

- [ ] **Step 2: Run the contract and verify the smoke script is missing**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py \
  -v
```

Expected: ERROR for `test/smoke_observation.sh`.

- [ ] **Step 3: Implement the two-client discovery smoke**

Create executable `test/smoke_observation.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != lan || "$#" -ne 1 ]]; then
  echo 'Usage: smoke_observation.sh lan' >&2
  exit 2
fi

: "${GZ_SERVER_IP:?set GZ_SERVER_IP}"
: "${GZ_CLIENT_IP:?set GZ_CLIENT_IP}"

bundle_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

check_client() {
  local name="$1"
  local topics
  topics="$(
    GZ_IP="$GZ_CLIENT_IP" \
    GZ_RELAY="$GZ_SERVER_IP" \
    GZ_PARTITION=mentorpi-sim \
    gz topic -l
  )"
  for topic in \
    /world/mentorpi_warehouse/stats \
    /robot_1/scan_raw \
    /robot_2/scan_raw; do
    grep -Fxq "$topic" <<<"$topics" || {
      printf '%s missing %s\n' "$name" "$topic" >&2
      return 1
    }
  done
}

check_client client-a &
pid_a="$!"
check_client client-b &
pid_b="$!"
wait "$pid_a"
wait "$pid_b"

docker compose \
  -f "$bundle_dir/compose.yaml" \
  -f "$bundle_dir/compose.lan.yaml" \
  exec -T gazebo-server /usr/local/bin/mentorpi-healthcheck server
docker compose \
  -f "$bundle_dir/compose.yaml" \
  -f "$bundle_dir/compose.lan.yaml" \
  exec -T sim-adapter /usr/local/bin/mentorpi-healthcheck adapter
```

- [ ] **Step 4: Run the native Linux LAN gate**

On the Ubuntu release host:

```bash
cd vehicle_simulator_model/ubuntu
export SIM_NETWORK_MODE=lan
export GZ_SERVER_IP="$(hostname -I | awk '{print $1}')"
./run.sh sim-up

GZ_CLIENT_IP="$GZ_SERVER_IP" \
  GZ_SERVER_IP="$GZ_SERVER_IP" \
  ./test/smoke_observation.sh lan
```

Expected: both clients find the world and both robot scans; both healthchecks return `status=ok`.

- [ ] **Step 5: Run the Mac Docker Desktop gate**

Enable Docker Desktop host networking, then run:

```bash
cd vehicle_simulator_model/ubuntu
export SIM_NETWORK_MODE=lan
export GZ_SERVER_IP=127.0.0.1
./run.sh sim-up
./scripts/gz-gui-connect.sh 127.0.0.1 127.0.0.1
```

Expected: the preflight finds `/world/mentorpi_warehouse/stats`, then the GUI displays `warehouse`, `robot_1`, and `robot_2`.

If the preflight exits 4, Mac Docker Desktop direct GUI is marked unsupported in README and local developers use Task 4's browser viewer. Do not weaken the base internal-network contract to force this gate.

- [ ] **Step 6: Verify GUI disconnect isolation**

Close both GUI clients and run:

```bash
docker compose \
  -f compose.yaml \
  -f compose.lan.yaml \
  exec -T gazebo-server /usr/local/bin/mentorpi-healthcheck server

docker compose \
  -f compose.yaml \
  -f compose.lan.yaml \
  exec -T sim-adapter /usr/local/bin/mentorpi-healthcheck adapter
```

Expected: both checks still return `status=ok`.

- [ ] **Step 7: Commit**

```bash
git add \
  vehicle_simulator_model/ubuntu/test/smoke_observation.sh \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py
git commit -m "test: 다중 Gazebo GUI LAN 연결 검증"
```

---

### Task 4: Read-only multi-client Gazebo viewer

**Files:**
- Create: `vehicle_simulator_model/ubuntu/viewer-entrypoint.sh`
- Create: `vehicle_simulator_model/ubuntu/compose.viewer.yaml`
- Modify: `vehicle_simulator_model/ubuntu/Dockerfile`
- Modify: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- Test: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`

**Interfaces:**
- Consumes: internal `mentorpi` network, `GZ_PARTITION=mentorpi-sim`, `GZ_RELAY_HOST=gazebo-server`
- Produces: internal `gazebo-viewer:6080`
- Produces: view-only shared VNC session backed by the actual `gz sim -g`

- [ ] **Step 1: Write the failing viewer contract**

Add:

```python
def test_viewer_runs_real_gazebo_gui_read_only_and_shared(self):
    dockerfile = (BUNDLE / 'Dockerfile').read_text()
    script = (BUNDLE / 'viewer-entrypoint.sh').read_text()
    compose = (BUNDLE / 'compose.viewer.yaml').read_text()

    for package in ('xvfb', 'x11vnc', 'novnc', 'websockify'):
        self.assertIn(package, dockerfile)
    self.assertIn('Xvfb :99', script)
    self.assertIn('gz sim --force-version 8 -g', script)
    self.assertIn('-viewonly', script)
    self.assertIn('-shared', script)
    self.assertIn('-forever', script)
    self.assertIn('websockify --web=/usr/share/novnc 6080', script)
    self.assertIn('gazebo-viewer:', compose)
    self.assertIn("expose: ['6080']", compose)
    self.assertNotIn('6080:6080', compose)
```

- [ ] **Step 2: Run the contract and verify viewer assets are absent**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py \
  -v
```

Expected: ERROR because the viewer files do not exist and the packages are not installed.

- [ ] **Step 3: Install viewer runtime packages**

Add these packages to the existing apt install block:

```dockerfile
novnc \
websockify \
x11vnc \
xvfb \
```

Copy and make the viewer entrypoint executable:

```dockerfile
COPY viewer-entrypoint.sh /usr/local/bin/mentorpi-viewer
RUN chmod +x /usr/local/bin/mentorpi-viewer
```

- [ ] **Step 4: Implement the viewer process supervisor**

Create `viewer-entrypoint.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

export DISPLAY=:99
pids=()

cleanup() {
  local pid
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM

Xvfb :99 -screen 0 "${VIEWER_GEOMETRY:-1920x1080x24}" -nolisten tcp &
pids+=("$!")

for _ in $(seq 1 50); do
  xdpyinfo -display :99 >/dev/null 2>&1 && break
  sleep 0.1
done
xdpyinfo -display :99 >/dev/null 2>&1

gz sim --force-version 8 -g &
pids+=("$!")

x11vnc \
  -display :99 \
  -rfbport 5900 \
  -localhost \
  -nopw \
  -forever \
  -shared \
  -viewonly &
pids+=("$!")

websockify --web=/usr/share/novnc 6080 localhost:5900 &
pids+=("$!")

wait -n "${pids[@]}"
```

Add `x11-utils` to the Dockerfile because the readiness loop uses `xdpyinfo`.

- [ ] **Step 5: Add the internal viewer service**

Create `compose.viewer.yaml`:

```yaml
services:
  gazebo-viewer:
    image: "${MENTORPI_IMAGE:-mentorpi-sim:harmonic}"
    platform: ${TARGET_PLATFORM:-linux/amd64}
    profiles: [viewer]
    restart: unless-stopped
    networks: [mentorpi]
    environment:
      GZ_PARTITION: mentorpi-sim
      GZ_RELAY_HOST: gazebo-server
      SERVICE_NAME: gazebo-viewer
      IMAGE_VERSION: "${MENTORPI_IMAGE:-mentorpi-sim:harmonic}"
      SESSION_ID: none
      ROBOT_IDS: robot_1,robot_2
      LIBGL_ALWAYS_SOFTWARE: "1"
    command: /usr/local/bin/mentorpi-viewer
    depends_on:
      sim-adapter:
        condition: service_healthy
    expose: ['6080']
    healthcheck:
      test: ['CMD', 'curl', '-fsS', 'http://127.0.0.1:6080/vnc.html']
      interval: 10s
      timeout: 5s
      retries: 12
      start_period: 15s
```

- [ ] **Step 6: Add shell and Compose validation**

Run:

```bash
bash -n vehicle_simulator_model/ubuntu/viewer-entrypoint.sh

docker compose \
  -f vehicle_simulator_model/ubuntu/compose.yaml \
  -f vehicle_simulator_model/ubuntu/compose.viewer.yaml \
  --profile viewer \
  config --quiet

python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py \
  -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add \
  vehicle_simulator_model/ubuntu/Dockerfile \
  vehicle_simulator_model/ubuntu/viewer-entrypoint.sh \
  vehicle_simulator_model/ubuntu/compose.viewer.yaml \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py
git commit -m "feat: read-only Gazebo noVNC viewer 추가"
```

---

### Task 5: HTTPS gateway and source-IP allowlist

**Files:**
- Create: `vehicle_simulator_model/ubuntu/Caddyfile.viewer`
- Create: `vehicle_simulator_model/ubuntu/compose.viewer-public.yaml`
- Modify: `vehicle_simulator_model/ubuntu/compose.viewer.yaml`
- Modify: `vehicle_simulator_model/ubuntu/run.sh`
- Modify: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- Test: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`

**Interfaces:**
- Consumes: `VIEWER_MODE=local|public`
- Consumes: local `VIEWER_PORT`, default `8080`
- Consumes: public `VIEWER_DOMAIN` and whitespace-separated `VIEWER_ALLOW_CIDRS`
- Produces: local `127.0.0.1:${VIEWER_PORT}` or public HTTPS 443
- Preserves: internal-only `gazebo-viewer:6080`

- [ ] **Step 1: Write the failing gateway boundary contract**

Add:

```python
def test_gateway_is_the_only_viewer_service_with_host_ports(self):
    viewer = (BUNDLE / 'compose.viewer.yaml').read_text()
    public = (BUNDLE / 'compose.viewer-public.yaml').read_text()
    caddy = (BUNDLE / 'Caddyfile.viewer').read_text()

    self.assertIn('web-gateway:', viewer)
    self.assertIn('127.0.0.1:${VIEWER_PORT:-8080}:8080', viewer)
    self.assertIn('ports: !override', public)
    self.assertIn('"80:80"', public)
    self.assertIn('"443:443"', public)
    self.assertIn('@allowed remote_ip {$VIEWER_ALLOW_CIDRS}', caddy)
    self.assertIn('reverse_proxy gazebo-viewer:6080', caddy)
    self.assertIn('respond 403', caddy)
    self.assertNotIn('basic_auth', caddy)
    for forbidden in ('10317:', '10318:', '11811:', '5900:', '6080:6080'):
        self.assertNotIn(forbidden, viewer + public)
```

- [ ] **Step 2: Run the contract and verify gateway files are missing**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py \
  -v
```

Expected: ERROR for `Caddyfile.viewer` and `compose.viewer-public.yaml`.

- [ ] **Step 3: Implement the no-auth allowlist gateway**

Create `Caddyfile.viewer`:

```caddyfile
{$VIEWER_SITE:http://:8080} {
  @allowed remote_ip {$VIEWER_ALLOW_CIDRS:private_ranges}

  handle @allowed {
    reverse_proxy gazebo-viewer:6080
  }

  respond 403
}
```

This plan supports direct router/NAT forwarding to Caddy. If a CDN or another reverse proxy is later inserted, trusted proxy handling requires a separate security review because `remote_ip` would otherwise see the proxy address.

- [ ] **Step 4: Add the local gateway service**

Append to `compose.viewer.yaml`:

```yaml
  web-gateway:
    image: caddy:2.10-alpine
    profiles: [viewer]
    restart: unless-stopped
    networks: [mentorpi]
    environment:
      VIEWER_SITE: http://:8080
      VIEWER_ALLOW_CIDRS: private_ranges
    volumes:
      - ./Caddyfile.viewer:/etc/caddy/Caddyfile:ro
      - viewer-caddy-data:/data
      - viewer-caddy-config:/config
    ports:
      - "127.0.0.1:${VIEWER_PORT:-8080}:8080"
    depends_on:
      gazebo-viewer:
        condition: service_healthy

volumes:
  viewer-caddy-data:
  viewer-caddy-config:
```

- [ ] **Step 5: Add the public exposure override**

Create `compose.viewer-public.yaml`:

```yaml
services:
  web-gateway:
    environment:
      VIEWER_SITE: "${VIEWER_DOMAIN:?set VIEWER_DOMAIN for automatic HTTPS}"
      VIEWER_ALLOW_CIDRS: "${VIEWER_ALLOW_CIDRS:?set one or more team CIDRs}"
    ports: !override
      - "80:80"
      - "443:443"
```

- [ ] **Step 6: Add `viewer-up`, `viewer-down`, and `viewer-logs`**

Add usage:

```text
viewer-up [local|public] Start read-only Gazebo browser monitoring.
viewer-down              Stop viewer services without stopping simulation.
viewer-logs              Follow viewer and gateway logs.
```

Implement public validation before Docker is called:

```bash
validate_public_viewer() {
  : "${VIEWER_DOMAIN:?VIEWER_DOMAIN is required for public viewer}"
  : "${VIEWER_ALLOW_CIDRS:?VIEWER_ALLOW_CIDRS is required for public viewer}"
  case " $VIEWER_ALLOW_CIDRS " in
    *' 0.0.0.0/0 '*|*' ::/0 '*)
      echo 'public viewer does not allow unrestricted CIDRs' >&2
      exit 2
      ;;
  esac
}
```

`viewer-up` must reject `SIM_NETWORK_MODE=lan`; the public viewer uses the base internal network and does not need raw Gazebo Transport exposure.

Compose commands:

```bash
viewer_compose=(
  docker compose
  -f "$BUNDLE_DIR/compose.yaml"
  -f "$BUNDLE_DIR/compose.viewer.yaml"
  --profile viewer
)
```

For `public`, append `-f "$BUNDLE_DIR/compose.viewer-public.yaml"` before `up`.

Start:

```bash
"${viewer_compose[@]}" up -d \
  dds-discovery gazebo-server sim-adapter gazebo-viewer web-gateway
```

Stop only viewer services:

```bash
"${viewer_compose[@]}" stop web-gateway gazebo-viewer
```

- [ ] **Step 7: Test allowlist rejection before Docker invocation**

Use the existing fake-Docker test pattern and assert:

```python
self.assertNotEqual(result.returncode, 0)
self.assertIn('does not allow unrestricted CIDRs', result.stderr)
self.assertEqual(fake_docker_log, '')
```

Cover empty `VIEWER_DOMAIN`, empty `VIEWER_ALLOW_CIDRS`, `0.0.0.0/0`, and `::/0`.

- [ ] **Step 8: Validate local and public Compose**

Run:

```bash
docker compose \
  -f vehicle_simulator_model/ubuntu/compose.yaml \
  -f vehicle_simulator_model/ubuntu/compose.viewer.yaml \
  --profile viewer \
  config --quiet

VIEWER_DOMAIN=sim.example.com \
VIEWER_ALLOW_CIDRS='203.0.113.10/32 203.0.113.11/32' \
docker compose \
  -f vehicle_simulator_model/ubuntu/compose.yaml \
  -f vehicle_simulator_model/ubuntu/compose.viewer.yaml \
  -f vehicle_simulator_model/ubuntu/compose.viewer-public.yaml \
  --profile viewer \
  config --quiet
```

Expected: both pass; only the gateway has host-published ports.

- [ ] **Step 9: Commit**

```bash
git add \
  vehicle_simulator_model/ubuntu/Caddyfile.viewer \
  vehicle_simulator_model/ubuntu/compose.viewer.yaml \
  vehicle_simulator_model/ubuntu/compose.viewer-public.yaml \
  vehicle_simulator_model/ubuntu/run.sh \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py
git commit -m "feat: IP 제한 Gazebo viewer gateway 추가"
```

---

### Task 6: Runtime isolation and multi-viewer verification

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/test/smoke_observation.sh`
- Modify: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- Test: `vehicle_simulator_model/ubuntu/test/smoke_observation.sh`

**Interfaces:**
- Consumes: local viewer at `http://127.0.0.1:${VIEWER_PORT:-8080}`
- Verifies: actual `gz sim -g`, two concurrent noVNC clients, view-only mode, simulation failure isolation

- [ ] **Step 1: Add the viewer smoke contract**

Add:

```python
def test_viewer_smoke_checks_two_websocket_clients_and_isolation(self):
    script = (BUNDLE / 'test/smoke_observation.sh').read_text()
    self.assertIn('vnc.html?view_only=1', script)
    self.assertIn('websockify', script)
    self.assertIn('gz sim --force-version 8 -g', script)
    self.assertIn('mentorpi-healthcheck server', script)
    self.assertIn('mentorpi-healthcheck adapter', script)
```

- [ ] **Step 2: Extend the smoke script with an explicit viewer mode**

Replace the initial single-mode validation with:

```bash
mode="${1:-}"
if [[ "$#" -ne 1 || "$mode" != lan && "$mode" != viewer ]]; then
  echo 'Usage: smoke_observation.sh <lan|viewer>' >&2
  exit 2
fi
```

Wrap the existing LAN checks in `smoke_lan()` and add:

Move the `GZ_SERVER_IP` and `GZ_CLIENT_IP` required-variable checks inside
`smoke_lan()` so viewer mode does not require LAN variables.

```bash
smoke_viewer() {
viewer_url="http://127.0.0.1:${VIEWER_PORT:-8080}/vnc.html?view_only=1&autoconnect=1"
curl -fsS "$viewer_url" | grep -Fq '<title>noVNC</title>'

docker compose \
  -f "$bundle_dir/compose.yaml" \
  -f "$bundle_dir/compose.viewer.yaml" \
  --profile viewer \
  exec -T gazebo-viewer \
  bash -lc '
    pgrep -f "gz sim --force-version 8 -g"
    pgrep -x Xvfb
    pgrep -x x11vnc
    pgrep -f websockify
  '
}

case "$mode" in
  lan) smoke_lan ;;
  viewer) smoke_viewer ;;
esac
```

Use two concurrent HTTP/WebSocket browser sessions in the manual gate below; the automated smoke proves that the multi-client `-shared` server and all processes remain alive.

- [ ] **Step 3: Build and start the local viewer**

Run:

```bash
cd vehicle_simulator_model/ubuntu
./run.sh build
./run.sh viewer-up local
docker compose \
  -f compose.yaml \
  -f compose.viewer.yaml \
  --profile viewer \
  ps
```

Expected: `gazebo-server`, `sim-adapter`, `gazebo-viewer`, and `web-gateway` are healthy/running.

- [ ] **Step 4: Open two independent browser clients**

Open the following URL in a normal browser window and a private window:

```text
http://127.0.0.1:8080/vnc.html?view_only=1&autoconnect=1
```

Expected in both windows:

- `warehouse` is rendered.
- `robot_1` and `robot_2` are rendered.
- Camera movement and keyboard/mouse input are not forwarded because the server is view-only.
- Closing either window does not disconnect the other.

- [ ] **Step 5: Prove viewer failure isolation**

Run:

```bash
docker compose \
  -f compose.yaml \
  -f compose.viewer.yaml \
  --profile viewer \
  stop web-gateway gazebo-viewer

docker compose exec -T gazebo-server \
  /usr/local/bin/mentorpi-healthcheck server
docker compose exec -T sim-adapter \
  /usr/local/bin/mentorpi-healthcheck adapter
```

Expected: both healthchecks still return `status=ok`.

- [ ] **Step 6: Prove viewer recovery**

Run:

```bash
./run.sh viewer-up local
curl -fsS \
  'http://127.0.0.1:8080/vnc.html?view_only=1&autoconnect=1' \
  >/dev/null
```

Expected: viewer recovers without restarting Gazebo or the adapter.

- [ ] **Step 7: Commit**

```bash
git add \
  vehicle_simulator_model/ubuntu/test/smoke_observation.sh \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py
git commit -m "test: Gazebo viewer 다중 접속과 장애 격리 검증"
```

---

### Task 7: Operator documentation and superseded-plan notice

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/README.md`
- Modify: `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- Modify: `docs/superpowers/plans/2026-07-26-mentorpi-gazebo-web-monitor.md`
- Test: `vehicle_simulator_model/ubuntu/test/test_bundle.py`

**Interfaces:**
- Documents: internal, LAN native GUI, local viewer, public viewer, mapping combinations
- Documents: multi-client behavior, IP allowlist, shutdown and recovery

- [ ] **Step 1: Add failing documentation assertions**

Add these required strings:

```python
for text in (
    'SIM_NETWORK_MODE=lan',
    'GZ_SERVER_IP',
    'scripts/gz-gui-connect.sh',
    './run.sh viewer-up local',
    './run.sh viewer-up public',
    'VIEWER_DOMAIN',
    'VIEWER_ALLOW_CIDRS',
    'read-only',
    '동시에 접속',
    'Gazebo Transport를 공용 인터넷에 공개하지 않는다',
):
    self.assertIn(text, readme)
```

Assert the existing plan starts with a superseded notice linking to this plan.

- [ ] **Step 2: Run documentation tests and verify failure**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  -v
```

Expected: FAIL because the new operation guide is absent.

- [ ] **Step 3: Document the execution matrix**

Add this exact matrix:

| 목적 | 서버 실행 | 개발 PC 접속 |
| --- | --- | --- |
| Headless 통합 검증 | `./run.sh sim-up` | `topics`, logs, healthcheck |
| 같은 LAN 네이티브 GUI | `SIM_NETWORK_MODE=lan GZ_SERVER_IP=<server-lan-ip> ./run.sh sim-up` | `scripts/gz-gui-connect.sh <server-lan-ip> <client-lan-ip>` |
| 로컬 브라우저 viewer | `./run.sh viewer-up local` | `http://127.0.0.1:8080/vnc.html?view_only=1&autoconnect=1` |
| 외부 팀 viewer | `VIEWER_DOMAIN=... VIEWER_ALLOW_CIDRS='...' ./run.sh viewer-up public` | `https://<VIEWER_DOMAIN>/vnc.html?view_only=1&autoconnect=1` |
| 지도 생성 | `./run.sh mapping-up <session-id>` | logs와 `mapping-status` |

- [ ] **Step 4: Document LAN native GUI**

Include:

```bash
# Linux server
export SIM_NETWORK_MODE=lan
export GZ_SERVER_IP=192.168.50.10
./run.sh sim-up

# Mac A
./scripts/gz-gui-connect.sh 192.168.50.10 192.168.50.20

# Mac B
./scripts/gz-gui-connect.sh 192.168.50.10 192.168.50.21
```

State that all GUI clients share the same world and that closing every GUI does not stop the simulation.

- [ ] **Step 5: Document public viewer without application auth**

Include:

```bash
export VIEWER_DOMAIN=sim.example.com
export VIEWER_ALLOW_CIDRS='203.0.113.10/32 203.0.113.11/32'
./run.sh viewer-up public
```

State:

- Router/NAT forwards public 80 and 443 to the Linux server.
- Caddy uses 80 for ACME redirect/challenge and serves the viewer on 443.
- Only listed CIDRs receive the viewer; all others receive HTTP 403.
- Dynamic team IPs require updating the allowlist or choosing a separate authenticated access method.
- `0.0.0.0/0` and `::/0` are rejected.
- noVNC 6080, VNC 5900, Gazebo Transport, ROS DDS are never port-forwarded.

- [ ] **Step 6: Add operations and recovery commands**

Document:

```bash
./run.sh viewer-logs
./run.sh viewer-down
./run.sh logs
./run.sh topics
./run.sh down
```

Clarify that `viewer-down` preserves the simulation, while `down` stops the simulation stack.

- [ ] **Step 7: Mark the old plan as superseded**

Insert immediately after its title:

```markdown
> **Superseded:** 이 계획의 웹 전용·basic-auth 전제는
> `docs/superpowers/plans/2026-07-28-mentorpi-gazebo-shared-observation.md`로
> 대체되었다. 새 계획은 LAN 네이티브 `gz sim -g`와 IP allowlist 기반
> read-only viewer를 함께 정의한다.
```

- [ ] **Step 8: Run documentation tests**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py \
  -v
```

Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add \
  docs/superpowers/plans/2026-07-26-mentorpi-gazebo-web-monitor.md \
  vehicle_simulator_model/ubuntu/README.md \
  vehicle_simulator_model/ubuntu/test/test_bundle.py
git commit -m "docs: Gazebo 공유 관찰 운영 가이드 추가"
```

---

### Task 8: Full regression and release acceptance

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- Test: all existing runtime, model, SLAM, observation tests

**Interfaces:**
- Verifies: default internal mode, LAN mode, local viewer, public viewer config, SLAM regression
- Produces: release evidence for Mac and Ubuntu

- [ ] **Step 1: Run all host-side tests**

Run:

```bash
python3 \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  -v

python3 \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py \
  -v

python3 -m unittest discover \
  -s vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test \
  -p 'test_*.py' \
  -v

python3 -m unittest discover \
  -s vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test \
  -p 'test_*.py' \
  -v

python3 \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py \
  -v
```

Expected: PASS.

- [ ] **Step 2: Validate all Compose variants**

Run:

```bash
cd vehicle_simulator_model/ubuntu

docker compose -f compose.yaml config --quiet

GZ_SERVER_IP=192.168.50.10 \
docker compose \
  -f compose.yaml \
  -f compose.lan.yaml \
  config --quiet

docker compose \
  -f compose.yaml \
  -f compose.viewer.yaml \
  --profile viewer \
  config --quiet

VIEWER_DOMAIN=sim.example.com \
VIEWER_ALLOW_CIDRS='203.0.113.10/32 203.0.113.11/32' \
docker compose \
  -f compose.yaml \
  -f compose.viewer.yaml \
  -f compose.viewer-public.yaml \
  --profile viewer \
  config --quiet
```

Expected: PASS.

- [ ] **Step 3: Build and run the immutable image tests**

Run:

```bash
./run.sh build
./run.sh test
```

Expected: image build and all container-side `colcon test` checks pass.

- [ ] **Step 4: Verify default mode has no new host exposure**

Run:

```bash
./run.sh sim-up
docker compose ps
docker compose port gazebo-server 10317 && exit 1 || true
docker compose port sim-adapter 11811 && exit 1 || true
./run.sh topics
```

Expected: server and adapter are healthy; raw transport and DDS have no host-published port.

- [ ] **Step 5: Verify local viewer release gate**

Run:

```bash
./run.sh viewer-up local
./test/smoke_observation.sh viewer
```

Expected: browser endpoint and actual GUI processes are healthy.

- [ ] **Step 6: Verify SLAM regression**

Run:

```bash
session_id="observation-release-$(date +%Y%m%d%H%M%S)"
./run.sh mapping-up "$session_id"
./run.sh mapping-stop
./run.sh mapping-status "$session_id"
```

Expected: mapping finalization succeeds and all checksums pass.

- [ ] **Step 7: Run manual Mac and Ubuntu acceptance**

Record:

```text
Mac Gazebo version: 8.14.0
Docker Gazebo version: 8.14.0
Mac Docker Desktop direct GUI: PASS or UNSUPPORTED
Ubuntu LAN Mac GUI A: PASS
Ubuntu LAN Mac GUI B: PASS
Local noVNC clients x2: PASS
Public allowlisted source: PASS
Public non-allowlisted source: HTTP 403
Viewer stopped while simulation healthchecks pass: PASS
```

Release is accepted only when every line has a concrete result. `UNSUPPORTED` is permitted only for Mac Docker Desktop direct GUI; Linux LAN native GUI and local/public viewer gates must pass.

- [ ] **Step 8: Commit final test adjustments**

```bash
git add vehicle_simulator_model/ubuntu/test/test_observation_bundle.py
git commit -m "test: Gazebo 공유 관찰 release gate 완성"
```

---

## Completion Criteria

- Default `sim-up` remains headless, isolated, and healthy.
- Same-LAN Linux Docker server is visible from at least two simultaneous Mac `gz sim -g` clients.
- Closing native GUI clients does not stop Gazebo, adapter, or mapper.
- Browser viewer shows the actual Docker-connected Gazebo GUI and accepts at least two simultaneous viewers.
- Browser viewer cannot send mouse or keyboard input to Gazebo.
- Public mode exposes only 80/443 and rejects sources outside `VIEWER_ALLOW_CIDRS`.
- Public mode cannot start with an empty or unrestricted allowlist.
- Viewer failure and restart do not restart simulation services.
- Both robot sensor, odom, TF, SLAM session, checksum contracts continue to pass.
- README provides complete commands for internal, LAN, local viewer, public viewer, mapping, logs, and shutdown cases.

## Out of Scope

- Public-internet native `gz sim -g` through raw port forwarding
- VPN-based native Gazebo Transport
- Browser-based model, URDF, Xacro, or SDF editing
- Per-user application accounts and passwords
- Multiple operator arbitration inside Gazebo GUI
- Nav2 autonomous driving and goal dispatch
- Real-vehicle SLAM upload, server map merge, and safe map switching
- ROS 2 Jazzy migration

## Reference Documentation

- Gazebo Harmonic server/GUI split: `https://gazebosim.org/docs/harmonic/getstarted/`
- Gazebo Transport environment variables: `https://gazebosim.org/api/transport/14/envvars.html`
- Gazebo Transport relay and endpoint reachability: `https://gazebosim.org/api/transport/12/relay.html`
- Docker host networking: `https://docs.docker.com/engine/network/drivers/host/`
- Docker published-port behavior: `https://docs.docker.com/engine/network/port-publishing/`
