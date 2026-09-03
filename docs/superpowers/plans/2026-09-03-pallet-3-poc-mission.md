# Pallet 3 POC Mission Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `dock_1`에서 픽업된 차량을 `pallet_3`에 이동시켜 하역 확인, 포크 하강 완료, 1초 후진, `dock_1` 복귀를 수행하는 영속 POC API를 만든다.

**Architecture:** Orchestrator는 POC 미션과 command outbox를 SQLite에 저장하고 Fleet 이벤트로 phase를 진행한다. Vehicle Command API는 `/fork/state`를 구독해 포크 완료를 mission ID와 함께 relay하며, 명령은 Fleet Manager와 Fleet Bridge를 거친다.

**Tech Stack:** Python, FastAPI, Pydantic, SQLite, httpx, ROS 2 Humble/rclpy, unittest.

**Spec:** `docs/superpowers/specs/2026-09-03-pallet-3-poc-mission-design.md`

## Global Constraints

- POC는 `pallet_3`만 지원하며 기존 inventory 예약, pick/place, 자동 배차를 변경하지 않는다.
- `DOWN_COMPLETE` 이전에는 후진·복귀 명령을 보내지 않는다.
- 후진 payload는 정확히 `linear_x=-0.18`, `linear_y=0.0`, `angular_z=0.0`, `hold_ms=1000`이다.
- 실패·취소·포크 오류·타임아웃에는 stop을 요청하고 mission을 `FAILED`로 끝낸다.

---

### Task 1: 차량 포크 상태를 Fleet 이벤트로 relay

**Files:**
- Modify: `vehicle_communication/vehicle_command_api.py`
- Modify: `vehicle_communication/test/test_vehicle_command_api.py`
- Modify: `vehicle_communication/README.md`

**Interfaces:**
- Consumes: `/fork/state` String JSON `{"state":"DOWN_COMPLETE","error":""}`.
- Produces: Fleet status `source="FORK"`, `detail="FORK_DOWN_COMPLETE"`, `operation_id=<mission_id>`.
- Produces: optional `operation_id` on `/v1/fork/down` and `/v1/cmd-vel`.

- [ ] **Step 1: DOWN 완료와 수동 만료의 failing test를 작성한다.**

```python
def test_down_complete_reports_only_the_pending_down_mission(self):
    self.service.on_fork_state('{"state":"DOWN_COMPLETE","error":""}')
    post_json(f'{self.base_url}/v1/fork/down', {'operation_id': 'poc-1'})
    self.service.on_fork_state('{"state":"DOWN_COMPLETE","error":""}')
    self.assertEqual(self.status_reporter.reports[-1]['operation_id'], 'poc-1')
    self.assertEqual(self.status_reporter.reports[-1]['detail'], 'FORK_DOWN_COMPLETE')
```

```python
def test_manual_expiry_reports_the_optional_poc_operation_id(self):
    post_json(f'{self.base_url}/v1/cmd-vel', {'operation_id': 'poc-1', 'linear_x': -0.18, 'linear_y': 0.0, 'angular_z': 0.0, 'hold_ms': 1})
    self.wait_for_manual_timer()
    self.assertEqual(self.status_reporter.reports[-1]['detail'], 'MANUAL_COMMAND_EXPIRED')
    self.assertEqual(self.status_reporter.reports[-1]['operation_id'], 'poc-1')
```

- [ ] **Step 2: 새 테스트가 누락 인터페이스로 실패하는지 확인한다.** Run `python3 -m unittest test.test_vehicle_command_api.VehicleCommandApiServerTest -v`; expected: `on_fork_state` 또는 operation ID validation missing failure.
- [ ] **Step 3: 최소 구현을 작성한다.** `VehicleCommandService`에 pending fork command·mission ID·15초 timer를 둔다. `fork_command(command, payload)`는 optional string operation ID만 받고 `DOWN` 후 pending을 설정한다. `on_fork_state(raw)`는 pending `DOWN` 후에 들어온 `DOWN_COMPLETE`만 `FORK_DOWN_COMPLETE`로 보고하고 malformed JSON, nonempty error, timeout은 `FAIL`로 보고한다. `command()`와 `_expire_manual_command()`는 operation ID를 보존한다. `RosVehicleAdapter.configure_fork_state_subscription(topic, callback)`와 CLI `--fork-state-topic=/fork/state`, `--fork-state-timeout-sec=15.0`을 추가한다. fork HTTP routes는 optional body를 service에 전달한다.
- [ ] **Step 4: 차량 API 회귀를 확인한다.** Run `python3 -m unittest discover -s test -v`; expected: 기존 fork·navigation·manual safety 및 새 relay tests PASS.
- [ ] **Step 5: 커밋한다.** `git add vehicle_communication && git commit -m 'feat(vehicle): relay fork completion state'`.

### Task 2: Fleet Manager에 POC 제어 명령을 추가

**Files:**
- Modify: `operations/fleet_manager/app/commands.py`
- Modify: `operations/fleet_manager/app/main.py`
- Modify: `operations/fleet_manager/app/fleet.py`
- Modify: `operations/fleet_manager/config/vehicles.yaml`
- Modify: `operations/fleet_manager/tests/test_commands.py`
- Modify: `operations/fleet_manager/tests/test_api.py`
- Modify: `operations/logistics_orchestrator/app/clients.py`
- Modify: `operations/logistics_orchestrator/tests/test_clients.py`

**Interfaces:**
- Produces: `/api/v1/vehicles/{robot_id}/commands/navigation/waypoints`, `/fork/down`, `/cmd-vel`, `/stop`.
- Produces: `HttpFleetManagerClient.navigate_waypoints`, `fork_down`, `manual_velocity`, `stop`.
- Consumes: Fleet Bridge의 같은 vehicle-native route.

- [ ] **Step 1: Fleet Manager relay의 failing test를 작성한다.**

```python
def test_poc_commands_relay_exact_waypoint_fork_and_reverse_payloads(self):
    waypoint = self.client.post('/api/v1/vehicles/robot_1/commands/navigation/waypoints', json={'operation_id': 'poc-1', 'purpose': 'POC_PALLET_3_OUTBOUND', 'waypoints': [{'frame_id': 'map', 'x': -0.44, 'y': -0.9, 'yaw': 0.0}]})
    fork = self.client.post('/api/v1/vehicles/robot_1/commands/fork/down', json={'operation_id': 'poc-1'})
    reverse = self.client.post('/api/v1/vehicles/robot_1/commands/cmd-vel', json={'operation_id': 'poc-1', 'linear_x': -0.18, 'linear_y': 0.0, 'angular_z': 0.0, 'hold_ms': 1000})
    self.assertEqual([waypoint.status_code, fork.status_code, reverse.status_code], [202, 202, 202])
```

- [ ] **Step 2: route 또는 capability 부재로 실패하는지 확인한다.** Run `python3 -m unittest operations.fleet_manager.tests.test_commands -v`; expected: `404` 또는 unsupported capability failure.
- [ ] **Step 3: 최소 구현을 작성한다.** `SUPPORTED_CAPABILITIES`에 `fork`, `manual_drive`를 추가하고 MentorPi registry에 넣는다. waypoint는 existing `navigate` capability를 재사용한다. `StateSource`에 `FORK`를 추가한다. Fleet Manager routes는 payload를 변형하지 않고 Fleet Bridge에 relay한다. HTTP client method path는 각각 `/commands/navigation/waypoints`, `/commands/fork/down`, `/commands/cmd-vel`, `/commands/stop`이다.
- [ ] **Step 4: command boundary를 확인한다.** Run `python3 -m unittest discover -s operations/fleet_manager/tests -v`; then run `python3 -m unittest operations.logistics_orchestrator.tests.test_clients -v`; expected: new and existing command contracts PASS.
- [ ] **Step 5: 커밋한다.** `git add operations/fleet_manager operations/logistics_orchestrator/app/clients.py operations/logistics_orchestrator/tests/test_clients.py && git commit -m 'feat(fleet): relay poc vehicle controls'`.

### Task 3: 영속 POC 상태 머신과 자동 배차 격리

**Files:**
- Create: `operations/logistics_orchestrator/app/poc.py`
- Modify: `operations/logistics_orchestrator/app/models.py`
- Modify: `operations/logistics_orchestrator/app/store.py`
- Modify: `operations/logistics_orchestrator/app/service.py`
- Create: `operations/logistics_orchestrator/tests/test_poc.py`
- Modify: `operations/logistics_orchestrator/tests/test_store.py`
- Modify: `operations/logistics_orchestrator/tests/test_service.py`

**Interfaces:**
- Produces: `Pallet3PocService.start(robot_id)`, `confirm_unload(mission_id)`, `handle_fleet_event(payload)`.
- Produces: `PocMission(mission_id, robot_id, phase, last_error, created_at, updated_at)`.
- Consumes: Fleet event `robot_id`, `operation_id`, `source`, `detail`.

- [ ] **Step 1: POC phase transition failing test를 작성한다.**

```python
def test_down_complete_sends_exact_reverse_then_manual_expiry_sends_return_route(self):
    mission = self.create_mission_waiting_for_fork()
    self.service.handle_fleet_event(_event(mission, source='FORK', detail='FORK_DOWN_COMPLETE'))
    self.assertEqual(self.fleet.commands[-1], ('manual_velocity', 'robot_2', {'operation_id': mission.mission_id, 'linear_x': -0.18, 'linear_y': 0.0, 'angular_z': 0.0, 'hold_ms': 1000}))
    self.service.handle_fleet_event(_event(mission, source='API', detail='MANUAL_COMMAND_EXPIRED'))
    self.assertEqual(self.fleet.commands[-1][2]['purpose'], 'POC_DOCK_1_RETURN')
```

- [ ] **Step 2: module 부재로 실패하는지 확인한다.** Run `python3 -m unittest operations.logistics_orchestrator.tests.test_poc -v`; expected: `ModuleNotFoundError`.
- [ ] **Step 3: store·models·state machine을 구현한다.** `poc_missions` table은 mission ID primary key, robot ID, phase, last error, timestamps를 저장하고 terminal 이외 phase에는 robot 하나만 허용하는 partial unique index를 둔다. existing `command_outbox`에는 mission ID와 `POC_NAV_TO_PALLET_3`, `POC_FORK_DOWN`, `POC_REVERSE`, `POC_NAV_TO_DOCK_1`, `POC_STOP`을 사용한다. start route는 `(-0.440,-0.900,0)`, `(-0.420,-2.000,-π/2)`, `(-0.420,-2.400,-π/2)`이다. return route는 `(-0.420,-2.000,-π/2)`, `(-0.440,-0.900,0)`, `(0.085,-0.905,0)`이다. outbound `NAVIGATION_SUCCEEDED`는 confirmation 대기, confirmation은 fork down, `FORK_DOWN_COMPLETE`는 exact reverse, `MANUAL_COMMAND_EXPIRED`는 return route, inbound navigation success는 completed가 된다. failure event는 failed+idempotent stop이 된다. `OrchestratorService`에는 active POC predicate를 주입하여 POC 차량 자동 배차를 건너뛴다.
- [ ] **Step 4: POC·store·기존 배차 회귀를 확인한다.** Run `python3 -m unittest discover -s operations/logistics_orchestrator/tests -v`; expected: phase guard, duplicate event, duplicate confirmation, restart persistence, stop failure 및 existing tests PASS.
- [ ] **Step 5: 커밋한다.** `git add operations/logistics_orchestrator/app operations/logistics_orchestrator/tests && git commit -m 'feat(orchestrator): add pallet 3 poc mission'`.

### Task 4: Orchestrator HTTP API와 Fleet event wiring

**Files:**
- Modify: `operations/logistics_orchestrator/app/main.py`
- Modify: `operations/logistics_orchestrator/tests/test_api.py`
- Modify: `operations/logistics_orchestrator/README.md`

**Interfaces:**
- Produces: `POST /api/v1/poc/pallet-3-missions`, `GET /api/v1/poc/pallet-3-missions/{mission_id}`, `POST /api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation`.
- Consumes: durable `fleet.vehicle_reported` event after inbox deduplication.

- [ ] **Step 1: public API failing test를 작성한다.**

```python
def test_start_then_allows_unload_confirmation_only_after_arrival(self):
    started = self.client.post('/api/v1/poc/pallet-3-missions', json={'robot_id': 'robot_2'})
    mission_id = started.json()['mission_id']
    self.assertEqual(started.status_code, 201)
    self.assertEqual(self.client.post(f'/api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation').status_code, 409)
    self.client.post('/api/v1/events/fleet', json=_arrival_event(mission_id))
    self.assertEqual(self.client.post(f'/api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation').status_code, 202)
```

- [ ] **Step 2: route 부재로 실패하는지 확인한다.** Run `python3 -m unittest operations.logistics_orchestrator.tests.test_api -v`; expected: start request `404`.
- [ ] **Step 3: endpoint와 runtime wiring을 구현한다.** `create_app`에 `poc_service_factory`를 추가하여 lifespan에서 POC service를 만든다. start는 201, status는 200/404, confirmation은 202/404/409을 반환한다. `_record_event`는 existing behavior를 유지하고 fleet event만 POC service에도 전달한다. README에는 start, mission status, confirmation curl과 stop 절차를 기록한다.
- [ ] **Step 4: 전체 관련 회귀를 확인한다.** Run `python3 -m unittest discover -s operations/logistics_orchestrator/tests -v`; run `python3 -m unittest discover -s vehicle_communication/test -v`; run `python3 -m unittest discover -s operations/fleet_manager/tests -v`; run `npm test --prefix operations/control_center`; expected: all PASS. FastAPI가 host에 없으면 services requirements-dev를 설치한 isolated Python environment에서 같은 tests를 실행한다.
- [ ] **Step 5: 커밋한다.** `git add operations/logistics_orchestrator docs/superpowers && git commit -m 'feat(orchestrator): expose pallet 3 poc api'`.

## Plan Self-Review

- Spec coverage: fixed routes, unload confirmation, DOWN_COMPLETE gate, exact reverse, Fleet Manager boundary, persistence/idempotency, auto dispatch isolation, stop failures, public execution API를 Task 1~4가 다룬다.
- Placeholder scan: API path, payload, phase, expected failure, test command을 명시했으며 미결정 표기가 없다.
- Type consistency: vehicle `operation_id`, Fleet event `operation_id`, POC `mission_id`는 동일 string correlation key다.
