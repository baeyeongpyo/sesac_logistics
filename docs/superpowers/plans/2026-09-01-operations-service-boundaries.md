# 운영 서비스 경계 분리 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 차량 통신, 지도, 창고 오버레이, 관제 서비스를 독립 Docker bundle로 분리한다.

**Architecture:** `fleet_bridge`는 차량 WebSocket/HTTP 중계와 telemetry 저장만 유지한다. `map_server`와 `warehouse_server`는 각각 ROS topic 하나의 소유자가 되고, `monitoring`은 중앙 Foxglove Bridge와 3D asset HTTP server를 함께 제공한다. 모든 ROS bundle은 동일 host-network Domain 225에서 topic으로 통신한다.

**Tech Stack:** ROS 2 Humble, ament Python, Docker Compose, Python `unittest`, PyYAML, Foxglove Bridge, Python HTTP server.

**Spec:** `docs/superpowers/specs/2026-09-01-operations-service-boundaries-design.md`

## Global Constraints

- `fleet_bridge` Compose에는 `worker-robot-1`, `worker-robot-2`, `command-api`, `telemetry-writer`만 둔다.
- `map_server`는 `/map` 및 `map -> map_visualization` `/tf`를 유일하게 발행한다.
- `warehouse_server`는 `/warehouse/zones` MarkerArray를 reliable/transient-local QoS로 발행한다.
- `monitoring`은 observation-only Foxglove Bridge `:8765` 및 asset HTTP `:8088`을 제공한다.
- ROS bundle은 `ROS_DOMAIN_ID=225`, host network, host IPC를 유지한다.
- `:8080`, `:8765`, `:8088`, ROS topic 이름, QoS, map/warehouse YAML 형식은 변경하지 않는다.
- `operations/foxglove_assert_server`와 `fleet_bridge` 내부 map/warehouse/central Foxglove 파일은 최종 tree에 남기지 않는다.
- 사용자 변경 상태인 `.gitignore`, `vehicle_communication/runtime.env`, 기존 untracked docs는 수정하거나 stage하지 않는다.

---

### Task 1: 새 서비스 경계 contract test를 추가한다

**Files:**
- Create: `operations/test/__init__.py`
- Create: `operations/test/test_service_boundaries.py`
- Modify: `operations/fleet_bridge/test/test_compose_contract.py`
- Modify: `operations/fleet_bridge/test/test_bundle_contract.py`

**Interfaces:**
- Consumes: 각 bundle의 최종 `docker-compose.yaml`, Dockerfile, ROS package directory
- Produces: 서비스 책임이 다시 섞이는 변경을 막는 cross-bundle contract test

- [ ] **Step 1: 실패 테스트를 작성한다**

```python
def test_integrated_compose_includes_each_service_bundle(self):
    content = (OPERATIONS / 'compose.local.yaml').read_text(encoding='utf-8')
    for path in (
        './fleet_bridge/docker-compose.yaml',
        './map_server/docker-compose.yaml',
        './warehouse_server/docker-compose.yaml',
        './monitoring/docker-compose.yaml',
    ):
        self.assertIn(path, content)
    self.assertNotIn('./foxglove_assert_server/docker-compose.yaml', content)

def test_fleet_bridge_owns_only_vehicle_communication_services(self):
    services = compose(FLEET_BRIDGE / 'docker-compose.yaml')['services']
    self.assertEqual(set(services), {
        'worker-robot-1', 'worker-robot-2', 'command-api', 'telemetry-writer',
    })
```

- [ ] **Step 2: 실패를 확인한다**

Run: `python3 -m unittest operations.test.test_service_boundaries -v`

Expected: 새 test module과 세 신규 bundle이 없어 실패한다.

- [ ] **Step 3: 기존 fleet contract가 제거된 책임을 요구하도록 수정한다**

```python
for service in ('server-foxglove', 'map-publisher', 'warehouse-zone-publisher'):
    self.assertNotIn(service, services)
for relative in ('maps', 'config/server_foxglove.yaml', 'config/warehouse_zones.yaml'):
    self.assertFalse((BUNDLE / relative).exists())
```

- [ ] **Step 4: 아직 실패하는 contract test를 다시 실행한다**

Run: `python3 -m unittest operations.test.test_service_boundaries operations.fleet_bridge.test.test_compose_contract operations.fleet_bridge.test.test_bundle_contract -v`

Expected: production layout을 옮기기 전이므로 실패한다.

### Task 2: Monitoring 하위 map_server 구성요소를 추출한다

**Files:**
- Create: `operations/monitoring/map_server/{README.md,Dockerfile,entrypoint.sh}`
- Create: `operations/monitoring/map_server/maps/{map_0825.yaml,map_0825.pgm}`
- Create: `operations/monitoring/map_server/ros2_ws/src/central_map_server/{package.xml,setup.cfg,setup.py,resource/central_map_server,central_map_server/__init__.py,central_map_server/publisher.py}`
- Create: `operations/monitoring/map_server/ros2_ws/src/central_map_server/test/test_publisher.py`
- Delete: `operations/fleet_bridge/maps/**`
- Delete: `operations/fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/map_publisher.py`
- Delete: `operations/fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_map_publisher.py`

**Interfaces:**
- Consumes: existing map YAML/PGM bytes and `MAP_YAML`, `MAP_USE_SIM_TIME`
- Produces: `central_map_server.publisher:main`, `/map`, and `/tf` map visualization transform

- [ ] **Step 1: 이동 후 map geometry를 요구하는 실패 테스트를 작성한다**

```python
from central_map_server.publisher import load_map

def test_checked_in_map_keeps_nav2_geometry(self):
    loaded = load_map(MAP_SERVER / 'maps' / 'map_0825.yaml')
    self.assertEqual((loaded.width, loaded.height), (196, 128))
    self.assertEqual(loaded.resolution, 0.05)
    self.assertEqual(loaded.origin, (-5.04, -4.03, 0.0))

def test_compose_uses_central_map_entry_point(self):
    service = compose(MONITORING / 'docker-compose.yaml')['services']['map-publisher']
    self.assertEqual(service['command'][:4], [
        'ros2', 'run', 'central_map_server', 'central_map_publisher',
    ])
```

- [ ] **Step 2: 실패를 확인한다**

Run: `PYTHONPATH=operations/monitoring/map_server/ros2_ws/src/central_map_server python3 -m unittest discover -s operations/monitoring/map_server/ros2_ws/src/central_map_server/test -v`

Expected: `central_map_server`가 없어 실패한다.

- [ ] **Step 3: publisher와 map data를 새 bundle로 옮긴다**

```python
# operations/monitoring/map_server/ros2_ws/src/central_map_server/setup.py
entry_points={'console_scripts': [
    'central_map_publisher = central_map_server.publisher:main',
]}
```

```yaml
# operations/monitoring/docker-compose.yaml
services:
  map-publisher:
    build: {context: ./map_server, dockerfile: Dockerfile}
    network_mode: host
    ipc: host
    environment:
      ROS_DOMAIN_ID: ${SERVER_ROS_DOMAIN_ID:-225}
      MAP_YAML: ${MAP_YAML:-/maps/map_0825.yaml}
```

- [ ] **Step 4: map tests를 green으로 만든다**

Run: `PYTHONPATH=operations/monitoring/map_server/ros2_ws/src/central_map_server python3 -m unittest discover -s operations/monitoring/map_server/ros2_ws/src/central_map_server/test -v`

Expected: PASS.

### Task 3: Monitoring 하위 warehouse_server 구성요소를 추출한다

**Files:**
- Create: `operations/monitoring/warehouse_server/{README.md,Dockerfile,entrypoint.sh}`
- Create: `operations/monitoring/warehouse_server/config/warehouse_zones.yaml`
- Create: `operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server/{package.xml,setup.cfg,setup.py,resource/warehouse_overlay_server,warehouse_overlay_server/__init__.py,warehouse_overlay_server/publisher.py}`
- Create: `operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server/test/test_publisher.py`
- Delete: `operations/fleet_bridge/config/warehouse_zones.yaml`
- Delete: `operations/fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/warehouse_zone_publisher.py`
- Delete: `operations/fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_warehouse_zone_publisher.py`

**Interfaces:**
- Consumes: version 2 warehouse zone YAML
- Produces: `warehouse_overlay_server.publisher:main` and `/warehouse/zones` MarkerArray

- [ ] **Step 1: 독립 layout import와 Compose contract의 실패 테스트를 작성한다**

```python
from warehouse_overlay_server.publisher import build_marker_specs, load_warehouse_layout

def test_layout_keeps_every_operator_point(self):
    layout = load_warehouse_layout(WAREHOUSE_SERVER / 'config' / 'warehouse_zones.yaml')
    self.assertEqual(layout.topic, '/warehouse/zones')
    self.assertEqual(len(layout.points), 22)

def test_compose_mounts_layout_read_only(self):
    service = compose(MONITORING / 'docker-compose.yaml')['services']['warehouse-zone-publisher']
    self.assertEqual(service['environment']['WAREHOUSE_ZONES_CONFIG'], '/config/warehouse_zones.yaml')
```

- [ ] **Step 2: 실패를 확인한다**

Run: `PYTHONPATH=operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server python3 -m unittest discover -s operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server/test -v`

Expected: `warehouse_overlay_server`가 없어 실패한다.

- [ ] **Step 3: publisher, config, image, Compose를 새 bundle로 옮긴다**

```python
# operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server/setup.py
entry_points={'console_scripts': [
    'warehouse_zone_publisher = warehouse_overlay_server.publisher:main',
]}
```

```yaml
# operations/monitoring/docker-compose.yaml
services:
  warehouse-zone-publisher:
    build: {context: ./warehouse_server, dockerfile: Dockerfile}
    network_mode: host
    ipc: host
    environment:
      ROS_DOMAIN_ID: ${SERVER_ROS_DOMAIN_ID:-225}
      WAREHOUSE_ZONES_CONFIG: /config/warehouse_zones.yaml
```

- [ ] **Step 4: warehouse tests를 green으로 만든다**

Run: `PYTHONPATH=operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server python3 -m unittest discover -s operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server/test -v`

Expected: PASS.

### Task 4: Monitoring bundle에 Foxglove, asset, map, warehouse를 모은다

**Files:**
- Create: `operations/monitoring/{.env.example,README.md,docker-compose.yaml}`
- Create: `operations/monitoring/foxglove/{Dockerfile,entrypoint.sh}`
- Create: `operations/monitoring/config/server_foxglove.yaml`
- Create: `operations/monitoring/assets/{Dockerfile,serve_assets.py,hiwonder_mecanum_forklift/**}`
- Create: `operations/monitoring/map_server/**`
- Create: `operations/monitoring/warehouse_server/**`
- Create: `operations/monitoring/test/test_monitoring_bundle.py`
- Delete: `operations/fleet_bridge/config/server_foxglove.yaml`
- Delete: `operations/fleet_bridge/tools/foxglove_bridge_ctl.sh`
- Delete: `operations/fleet_bridge/test/test_foxglove_bridge_ctl.py`
- Delete: `operations/foxglove_assert_server/**`

**Interfaces:**
- Consumes: current Foxglove whitelist, pinned bridge commit, current URDF/mesh files
- Produces: `foxglove-bridge` at `:8765` and `asset-server` at `:8088`

- [ ] **Step 1: monitoring Compose와 asset URL 동작의 실패 테스트를 작성한다**

```python
def test_monitoring_owns_bridge_and_asset_services(self):
    services = compose(MONITORING / 'docker-compose.yaml')['services']
    self.assertEqual(set(services), {
        'foxglove-bridge', 'asset-server', 'map-publisher', 'warehouse-zone-publisher',
    })
    self.assertEqual(services['foxglove-bridge']['network_mode'], 'host')
    self.assertEqual(services['asset-server']['command'][0], 'python3')

def test_monitoring_bridge_stays_observation_only(self):
    params = load_yaml(MONITORING / 'config' / 'server_foxglove.yaml')['foxglove_bridge']['ros__parameters']
    self.assertEqual(params['port'], 8765)
    self.assertEqual(params['capabilities'], ['none'])
```

- [ ] **Step 2: 실패를 확인한다**

Run: `python3 -m unittest discover -s operations/monitoring/test -p 'test_*.py' -v`

Expected: `operations/monitoring`이 없어 실패한다.

- [ ] **Step 3: Bridge source/config과 current asset server를 monitoring으로 이동한다**

```yaml
# operations/monitoring/docker-compose.yaml
services:
  foxglove-bridge:
    build: {context: ./foxglove, dockerfile: Dockerfile}
    network_mode: host
    ipc: host
  asset-server:
    build: {context: ./assets, dockerfile: Dockerfile}
    network_mode: host
  map-publisher:
    build: {context: ./map_server, dockerfile: Dockerfile}
    network_mode: host
  warehouse-zone-publisher:
    build: {context: ./warehouse_server, dockerfile: Dockerfile}
    network_mode: host
```

```dockerfile
# operations/monitoring/foxglove/Dockerfile
RUN git clone https://github.com/foxglove/ros-foxglove-bridge.git src/foxglove_bridge \
 && git -C src/foxglove_bridge checkout "${FOXGLOVE_BRIDGE_COMMIT}"
```

- [ ] **Step 4: monitoring tests를 green으로 만든다**

Run: `python3 -m unittest discover -s operations/monitoring/test -p 'test_*.py' -v`

Expected: PASS, including migrated CORS/range asset tests.

### Task 5: fleet_bridge를 차량 중개 전용으로 정리한다

**Files:**
- Modify: `operations/fleet_bridge/docker-compose.yaml`
- Modify: `operations/fleet_bridge/server/Dockerfile`
- Modify: `operations/fleet_bridge/server/entrypoint.sh`
- Rename: `operations/fleet_bridge/server/ros2_ws/src/foxglove_ros_worker` to `operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker`
- Modify: `operations/fleet_bridge/README.md`
- Modify: `operations/fleet_bridge/test/{test_bundle_contract.py,test_compose_contract.py}`

**Interfaces:**
- Consumes: fleet.yaml, telemetry.yaml, vehicle Foxglove URIs and vehicle Command API URLs
- Produces: current `:8080` Command API, per-robot telemetry topics, latest telemetry DB

- [ ] **Step 1: package rename과 image contract의 실패 테스트를 작성한다**

```python
def test_fleet_bridge_image_builds_no_central_foxglove_server(self):
    content = (BUNDLE / 'server' / 'Dockerfile').read_text(encoding='utf-8')
    self.assertNotIn('ros-foxglove-bridge.git', content)
    self.assertIn('COPY server/ros2_ws/src/fleet_bridge_worker src/fleet_bridge_worker', content)

def test_fleet_bridge_worker_exports_only_vehicle_communication_entry_points(self):
    setup = (WORKER / 'setup.py').read_text(encoding='utf-8')
    self.assertIn('fleet_bridge_worker = fleet_bridge_worker.main:main', setup)
    self.assertNotIn('central_map_publisher', setup)
```

- [ ] **Step 2: 실패를 확인한다**

Run: `python3 -m unittest operations.fleet_bridge.test.test_bundle_contract operations.fleet_bridge.test.test_compose_contract -v`

Expected: old package and central Foxglove build 때문에 실패한다.

- [ ] **Step 3: worker package를 rename하고 Compose/Dockerfile에서 다른 책임을 제거한다**

```python
# fleet_bridge_worker/setup.py
entry_points={'console_scripts': [
    'fleet_bridge_worker = fleet_bridge_worker.main:main',
    'fleet_command_api = fleet_bridge_worker.api:main',
    'fleet_telemetry_writer = fleet_bridge_worker.telemetry:main',
]}
```

```yaml
# fleet_bridge/docker-compose.yaml final services
services:
  worker-robot-1: {}
  worker-robot-2: {}
  command-api: {}
  telemetry-writer: {}
```

- [ ] **Step 4: vehicle bridge tests와 fleet contracts를 green으로 만든다**

Run: `PYTHONPATH=operations/fleet_bridge/common/fleet_bridge_config:operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker python3 -m unittest discover -s operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/test -p 'test_*.py' -v && python3 -m unittest discover -s operations/fleet_bridge/test -p 'test_*.py' -v`

Expected: FastAPI가 없는 local environment에서는 API module tests만 baseline처럼 load되지 않는다.

### Task 6: 통합 Compose와 운영 문서를 새 경계로 정리한다

**Files:**
- Modify: `operations/compose.local.yaml`
- Modify: `operations/fleet_manager/README.md`
- Modify: `operations/logistics_orchestrator/README.md`
- Modify: `docs/2026-08-31-logistics-orchestration-review.md`
- Modify: `operations/test/test_service_boundaries.py`

**Interfaces:**
- Consumes: four bundle Compose paths and external endpoint contracts
- Produces: one integrated local deployment definition and accurate ownership docs

- [ ] **Step 1: legacy bundle/paths 부재를 요구하는 실패 테스트를 보강한다**

```python
def test_legacy_operation_bundles_are_absent(self):
    self.assertFalse((OPERATIONS / 'foxglove_assert_server').exists())
    self.assertFalse((FLEET_BRIDGE / 'maps').exists())
    self.assertFalse((FLEET_BRIDGE / 'config' / 'server_foxglove.yaml').exists())
```

- [ ] **Step 2: 실패를 확인한다**

Run: `python3 -m unittest operations.test.test_service_boundaries -v`

Expected: old directories와 files가 남아 실패한다.

- [ ] **Step 3: Compose include와 documentation의 owner/명령을 새 경계로 바꾼다**

```yaml
# operations/compose.local.yaml
include:
  - ./fleet_bridge/docker-compose.yaml
  - ./monitoring/docker-compose.yaml
  - ./fleet_manager/docker-compose.yaml
  - ./inventory_db/docker-compose.yaml
  - ./logistics_orchestrator/docker-compose.yaml
```

- [ ] **Step 4: boundary contract와 네 Compose config를 실행한다**

Run: `python3 -m unittest operations.test.test_service_boundaries -v && for service in fleet_bridge map_server warehouse_server monitoring; do docker compose --env-file "operations/$service/.env.example" -f "operations/$service/docker-compose.yaml" config --quiet; done`

Expected: PASS.

### Task 7: 전체 검증 및 검토

**Files:**
- Verify only

**Interfaces:**
- Consumes: final source tree and all service contracts
- Produces: reproducible verification evidence

- [ ] **Step 1: 이동된 Python test suites를 실행한다**

Run: `PYTHONPATH=operations/fleet_bridge/common/fleet_bridge_config python3 -m unittest discover -s operations/fleet_bridge/common/fleet_bridge_config/test -p 'test_*.py' -v && PYTHONPATH=operations/fleet_bridge/common/fleet_bridge_config:operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker python3 -m unittest discover -s operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/test -p 'test_*.py' -v && PYTHONPATH=operations/monitoring/map_server/ros2_ws/src/central_map_server python3 -m unittest discover -s operations/monitoring/map_server/ros2_ws/src/central_map_server/test -p 'test_*.py' -v && PYTHONPATH=operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server python3 -m unittest discover -s operations/monitoring/warehouse_server/ros2_ws/src/warehouse_overlay_server/test -p 'test_*.py' -v && python3 -m unittest discover -s operations/monitoring/test -p 'test_*.py' -v && python3 -m unittest discover -s operations/test -p 'test_*.py' -v`

Expected: PASS except documented optional local FastAPI/ROS integration cases.

- [ ] **Step 2: Compose rendering과 diff를 검토한다**

Run: `docker compose --env-file operations/fleet_bridge/.env.example -f operations/fleet_bridge/docker-compose.yaml config --quiet && docker compose --env-file operations/monitoring/.env.example -f operations/monitoring/docker-compose.yaml config --quiet && git diff --check && git status --short`

Expected: Compose render and whitespace check PASS; user-owned modifications remain unstaged.
