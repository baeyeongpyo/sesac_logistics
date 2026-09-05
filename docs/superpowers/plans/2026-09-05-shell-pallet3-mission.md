# Shell Pallet 3 Mission Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** 차량 shell이 Auto Dock 또는 수동 PICK부터 P3 PLACE·후진·Docker 복귀까지 실행하고, Fleet Manager가 Inventory 원장과 대시보드 보고를 소유하게 한다.

**Architecture:** Fleet Manager는 Inventory operation을 만들고 차량 mission event를 원장 전이로 변환한다. Fleet Bridge는 시작 요청만 차량 API로 전달하며, 차량 API는 shell process group과 기존 ROS callback을 연결한다. shell은 로컬 Vehicle Command API의 완료 상태와 Fleet Manager event 응답을 확인해야 다음 단계로 진행한다.

**Tech Stack:** Python 3.12, FastAPI, SQLite, urllib, Bash, curl, jq, ROS 2/Nav2, unittest/pytest.

**Spec:** docs/superpowers/specs/2026-09-05-shell-pallet3-mission-design.md

## Global Constraints

- pick_mode는 기본값 없이 auto_dock 또는 manual만 허용한다.
- Inventory는 Fleet Manager만 호출하며, shell과 차량 API는 호출하지 않는다.
- PLACE 완료 후 모든 후진·복귀 차량 명령에는 operation ID를 넣지 않는다.
- Fleet Manager event 요청은 0.5초 간격으로 최대 5초 재시도하고 실패하면 차량을 stop한다.
- Fleet Manager stop은 Vehicle Command API stop relay와 shell process group SIGTERM을 모두 수행해야 한다.
- 기존 Fleet Manager 상태 relay와 dashboard 로그를 유지한다.
- Logistics Orchestrator는 이 direct mission을 소유하지 않는다.

---

## 파일 구조

| 파일 | 책임 |
| --- | --- |
| operations/fleet_manager/app/pallet3.py | Inventory client, 미션 SQLite 저장소, 시작·이벤트 상태 전이 |
| operations/fleet_manager/app/main.py | P3 mission HTTP endpoint와 앱 의존성 연결 |
| operations/fleet_manager/app/commands.py | pallet3_mission 차량 capability |
| operations/fleet_manager/config/vehicles.yaml | MentorPi capability 선언 |
| operations/fleet_manager/tests/test_pallet3.py | 서버의 Inventory·이벤트 계약 검증 |
| operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/fleet_bridge_worker/api.py | P3 시작 요청을 차량 API로 전달 |
| vehicle_communication/vehicle_command_api.py | shell supervisor, P3 endpoint, stop 통합 |
| vehicle_communication/tools/pallet3_mission.sh | 물리 명령·완료 대기 순서 |

### Task 1: Fleet Manager P3 mission domain과 Inventory 연동

**Files:**

- Create: operations/fleet_manager/app/pallet3.py
- Create: operations/fleet_manager/tests/test_pallet3.py
- Modify: operations/fleet_manager/app/fleet.py

**Interfaces:**

- Consumes: VehicleStateStore.get_snapshot(robot_id), BridgeCommandClient.relay(robot_id, path, payload).
- Produces: Pallet3MissionService.start(robot_id: str, pick_mode: str) -> dict, record_event(robot_id: str, operation_id: str, event_type: str, idempotency_key: str) -> dict, record_vehicle_reply(robot_id: str, operation_id: str, detail: str) -> None.

- [ ] **Step 1: 시작·이벤트 실패 테스트를 작성한다**

    def test_start_creates_inventory_operation_then_relays_shell_request(service, inventory, bridge):
        service.store.record_state("robot_1", "WAIT", "READY", False)
        result = service.start("robot_1", "manual")
        assert inventory.created == [("NORMAL", "docker", "p3", "robot_1")]
        assert bridge.calls[0][1] == "/missions/pallet3"
        assert bridge.calls[0][2]["operation_id"] == result["operation_id"]
        assert bridge.calls[0][2]["robot_id"] == "robot_1"

    def test_place_event_completes_inventory_once(service, inventory):
        operation_id = service.start("robot_1", "auto_dock")["operation_id"]
        service.record_vehicle_reply("robot_1", operation_id, "AUTO_DOCK_PICK_COMPLETED")
        service.record_event("robot_1", operation_id, "PICK_COMPLETED", operation_id + ":pick")
        result = service.record_event("robot_1", operation_id, "PLACE_READY", operation_id + ":place")
        assert result["phase"] == "PLACED"
        assert inventory.place_calls == [operation_id]

- [ ] **Step 2: 테스트가 실패함을 확인한다**

Run: cd operations/fleet_manager && .venv/bin/python -m pytest tests/test_pallet3.py -q

Expected: ModuleNotFoundError 또는 Pallet3MissionService import 오류.

- [ ] **Step 3: 최소 Inventory client·미션 저장소·서비스를 구현한다**

    class Pallet3MissionService:
        def start(self, robot_id: str, pick_mode: str) -> dict:
            if pick_mode not in {"auto_dock", "manual"}:
                raise ValueError("pick_mode must be auto_dock or manual")
            self.require_wait(robot_id)
            operation_id = self.inventory.create_operation("NORMAL", "docker", "p3", robot_id)
            self.store.create_direct_pallet3_mission(operation_id, robot_id, pick_mode)
            self.bridge.relay(robot_id, "/missions/pallet3", self.vehicle_payload(operation_id, robot_id, pick_mode))
            return {"operation_id": operation_id, "pick_mode": pick_mode}

        def record_event(self, robot_id: str, operation_id: str, event_type: str, idempotency_key: str) -> dict:
            return self.events.apply(robot_id, operation_id, event_type, idempotency_key)

direct_pallet3_missions에는 operation_id primary key, robot_id, pick_mode, phase, picked_at, placed_at, returned_at와 event별 idempotency key를 저장한다. 이벤트 전이는 PICK_COMPLETED -> PICKED, PLACE_READY -> PLACED, RETURN_COMPLETED -> RETURNED만 허용한다. PICK에는 mode별 AUTO_DOCK_PICK_COMPLETED 또는 FORK_UP_COMPLETE reply 증거를, PLACE에는 FORK_DOWN_COMPLETE 증거를 요구한다.

- [ ] **Step 4: 테스트를 통과시킨다**

Run: cd operations/fleet_manager && .venv/bin/python -m pytest tests/test_pallet3.py -q

Expected: PASS. WAIT 이외 상태, 중복 start, 잘못된 mode, robot 불일치, 잘못된 event 순서, Inventory 409도 각각 거절하는 테스트를 추가한다.

- [ ] **Step 5: 커밋한다**

    git add operations/fleet_manager/app/pallet3.py operations/fleet_manager/app/fleet.py operations/fleet_manager/tests/test_pallet3.py
    git commit -m "feat: add pallet 3 mission domain"

### Task 2: Fleet Manager HTTP 계약과 capability 연결

**Files:**

- Modify: operations/fleet_manager/app/main.py
- Modify: operations/fleet_manager/app/commands.py
- Modify: operations/fleet_manager/config/vehicles.yaml
- Modify: operations/fleet_manager/.env.example
- Modify: operations/fleet_manager/tests/test_api.py

**Interfaces:**

- Consumes: Pallet3MissionService.start()와 record_event().
- Produces: POST /api/v1/vehicles/{robot_id}/missions/pallet3, POST /api/v1/vehicles/{robot_id}/missions/pallet3/{operation_id}/events.

- [ ] **Step 1: HTTP 계약의 실패 테스트를 작성한다**

    def test_pallet3_start_requires_explicit_pick_mode(client):
        response = client.post("/api/v1/vehicles/robot_1/missions/pallet3", json={})
        assert response.status_code == 422

    def test_pick_event_requires_vehicle_reply_evidence(client, mission):
        response = client.post(mission.event_url, json={"event_type": "PICK_COMPLETED", "idempotency_key": "pick"})
        assert response.status_code == 409

- [ ] **Step 2: 테스트가 실패함을 확인한다**

Run: cd operations/fleet_manager && .venv/bin/python -m pytest tests/test_api.py -q

Expected: P3 route가 404이거나 event precondition이 구현되지 않아 실패.

- [ ] **Step 3: endpoint와 구성 값을 구현한다**

    @app.post("/api/v1/vehicles/{robot_id}/missions/pallet3", status_code=202)
    def start_pallet3_mission(robot_id: str, body: Pallet3StartRequest):
        return pallet3_service.start(robot_id, body.pick_mode)

    @app.post("/api/v1/vehicles/{robot_id}/missions/pallet3/{operation_id}/events")
    def record_pallet3_event(robot_id: str, operation_id: str, body: Pallet3EventRequest):
        return pallet3_service.record_event(robot_id, operation_id, body.event_type, body.idempotency_key)

상태 relay가 AUTO_DOCK_PICK_COMPLETED, FORK_UP_COMPLETE, FORK_DOWN_COMPLETE를 받으면 같은 operation ID의 record_vehicle_reply()를 호출한다. INVENTORY_URL 기본값은 http://127.0.0.1:8081로 두고 pallet3_mission capability를 registry에 추가한다.

- [ ] **Step 4: API·기존 Fleet Manager 테스트를 통과시킨다**

Run: cd operations/fleet_manager && .venv/bin/python -m pytest tests -q

Expected: PASS.

- [ ] **Step 5: 커밋한다**

    git add operations/fleet_manager/app/main.py operations/fleet_manager/app/commands.py operations/fleet_manager/config/vehicles.yaml operations/fleet_manager/.env.example operations/fleet_manager/tests/test_api.py
    git commit -m "feat: expose pallet 3 mission api"

### Task 3: Fleet Bridge의 P3 시작 proxy

**Files:**

- Modify: operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/fleet_bridge_worker/api.py
- Modify: operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/test/test_api.py

**Interfaces:**

- Consumes: Fleet Manager POST /api/v1/vehicle-command/{robot_id}/missions/pallet3 요청.
- Produces: Vehicle Command API POST /v1/missions/pallet3로의 body 보존 proxy.

- [ ] **Step 1: proxy 실패 테스트를 작성한다**

    def test_pallet3_mission_proxy_preserves_identity(client, command_client):
        body = {"operation_id": "op-1", "robot_id": "robot_1", "pick_mode": "manual", "fleet_manager_url": "http://fm:8090"}
        response = client.post("/api/v1/vehicle-command/robot_1/missions/pallet3", json=body)
        assert response.status_code == 202
        assert command_client.calls == [("robot_1", "POST", "/v1/missions/pallet3", body)]

- [ ] **Step 2: 테스트가 실패함을 확인한다**

Run: cd operations/fleet_bridge/server/ros2_ws && python3 -m pytest src/fleet_bridge_worker/test/test_api.py -q

Expected: route 404.

- [ ] **Step 3: 단일 relay route를 구현한다**

    @app.post("/api/v1/vehicle-command/{robot_id}/missions/pallet3", status_code=202)
    def pallet3_mission(robot_id: str, body: dict):
        return relay_vehicle_command(robot_id, "POST", "/v1/missions/pallet3", body)

기존 stop route는 변경하지 않는다. stop은 이미 /v1/stop을 proxy하므로 차량 API의 supervisor 통합으로 shell도 종료된다.

- [ ] **Step 4: Bridge 테스트를 통과시킨다**

Run: cd operations/fleet_bridge/server/ros2_ws && python3 -m pytest src/fleet_bridge_worker/test/test_api.py -q

Expected: PASS.

- [ ] **Step 5: 커밋한다**

    git add operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/fleet_bridge_worker/api.py operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/test/test_api.py
    git commit -m "feat: relay pallet 3 mission through fleet bridge"

### Task 4: Vehicle Command API shell supervisor와 상태 전이

**Files:**

- Modify: vehicle_communication/vehicle_command_api.py
- Modify: vehicle_communication/test/test_vehicle_command_api.py
- Modify: vehicle_communication/runtime.env

**Interfaces:**

- Consumes: POST /v1/missions/pallet3 body {operation_id, robot_id, pick_mode, fleet_manager_url}.
- Produces: Pallet3MissionSupervisor.start(...), stop(), POST /v1/missions/pallet3/{operation_id}/picked, POST /v1/missions/pallet3/{operation_id}/placed.

- [ ] **Step 1: process group·수동 PICK·place 해제의 실패 테스트를 작성한다**

    def test_stop_terminates_active_pallet3_process_group(service, supervisor):
        service.start_pallet3_mission({"operation_id": "op-1", "robot_id": "robot_1", "pick_mode": "manual", "fleet_manager_url": "http://fm:8090"})
        service.stop({})
        assert supervisor.terminated == ["op-1"]

    def test_manual_pick_requires_fork_up_reply_then_place_clears_context(service):
        service.start_pallet3_mission(MANUAL_MISSION)
        assert service.mark_pallet3_picked("op-1") == 409
        service.on_fork_state('{"state":"UP_COMPLETE"}')
        assert service.mark_pallet3_picked("op-1") == 200
        service.on_fork_state('{"state":"DOWN_COMPLETE"}')
        assert service.mark_pallet3_placed("op-1") == 200
        assert service.operation_status()["operation_id"] is None

- [ ] **Step 2: 테스트가 실패함을 확인한다**

Run: python3 -m pytest vehicle_communication/test/test_vehicle_command_api.py -q

Expected: P3 service method 또는 endpoint 부재로 실패.

- [ ] **Step 3: supervisor와 endpoint를 구현한다**

    class Pallet3MissionSupervisor:
        def start(self, operation_id: str, robot_id: str, pick_mode: str, fleet_manager_url: str) -> None:
            self.process = subprocess.Popen(
                [self.script, "--operation-id", operation_id, "--robot-id", robot_id, "--pick-mode", pick_mode, "--fleet-manager-url", fleet_manager_url],
                start_new_session=True,
            )

        def stop(self) -> None:
            if self.process and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGTERM)

on_fork_state()는 FORK_UP_COMPLETE와 FORK_DOWN_COMPLETE를 같은 operation ID의 내부 완료 detail과 기존 status relay 양쪽에 기록한다. /picked는 manual mission과 UP 완료 증거만 수락해 PICK_COMPLETE로 전이한다. /placed는 DOWN 완료 증거와 활성 ROS 명령 없음이 확인된 경우 WAIT/PLACE_COMPLETED를 보고하고 operation context를 비운다. 기존 /v1/stop은 supervisor stop, velocity zero, Nav2 cancel, Auto Dock stop을 한 번의 idempotent stop으로 수행한다.

- [ ] **Step 4: 차량 API 테스트를 통과시킨다**

Run: python3 -m pytest vehicle_communication/test/test_vehicle_command_api.py -q

Expected: PASS.

- [ ] **Step 5: 커밋한다**

    git add vehicle_communication/vehicle_command_api.py vehicle_communication/test/test_vehicle_command_api.py vehicle_communication/runtime.env
    git commit -m "feat: supervise pallet 3 vehicle mission"

### Task 5: 순차 shell mission과 운영 문서

**Files:**

- Create: vehicle_communication/tools/pallet3_mission.sh
- Create: vehicle_communication/test/test_pallet3_mission_shell.py
- Modify: vehicle_communication/README.md

**Interfaces:**

- Consumes: Vehicle Command API P3 endpoints와 Fleet Manager mission events.
- Produces: 실행 가능한 pallet3_mission.sh --operation-id OP --robot-id robot_1 --pick-mode auto_dock|manual --fleet-manager-url URL.

- [ ] **Step 1: shell 순서 계약 테스트를 작성한다**

    def test_place_clears_operation_id_before_reverse_and_return(script_text):
        assert script_text.index("complete_place") < script_text.index("reverse_after_place")
        assert script_text.index("reverse_after_place") < script_text.index("return_to_docker")
        assert '"linear_x":-0.18' in script_text
        assert '"hold_ms":1000' in script_text
        assert 'report_mission_event "RETURN_COMPLETED"' in script_text

- [ ] **Step 2: 테스트가 실패함을 확인한다**

Run: python3 -m pytest vehicle_communication/test/test_pallet3_mission_shell.py -q

Expected: shell 파일 부재로 실패.

- [ ] **Step 3: 명시적 단계 함수를 구현한다**

    main() {
      case "$PICK_MODE" in
        auto_dock) run_auto_dock_pick ;;
        manual) run_manual_pick ;;
      esac
      complete_pick
      drive_to_p3
      lower_fork
      complete_place
      reverse_after_place
      return_to_docker
    }

함수는 request_json, wait_vehicle_reply, report_mission_event, run_auto_dock_pick, run_manual_pick, complete_pick, drive_to_p3, lower_fork, complete_place, reverse_after_place, return_to_docker, abort_mission 이름을 정확히 사용한다. report_mission_event는 0.5초 interval·10회 시도 뒤 abort_mission을 호출한다. EXIT, INT, TERM trap은 이미 성공 복귀한 경우를 제외하고 local /v1/stop을 best-effort 호출한다.

- [ ] **Step 4: shell·차량 전체 테스트를 통과시킨다**

Run: python3 -m pytest vehicle_communication/test/test_pallet3_mission_shell.py vehicle_communication/test/test_vehicle_command_api.py -q && bash -n vehicle_communication/tools/pallet3_mission.sh

Expected: PASS 및 shell syntax 오류 없음.

- [ ] **Step 5: 운영 문서와 커밋을 완료한다**

    git add vehicle_communication/tools/pallet3_mission.sh vehicle_communication/test/test_pallet3_mission_shell.py vehicle_communication/README.md
    git commit -m "feat: add shell pallet 3 mission runner"

### Task 6: 전체 통합 회귀 검증

**Files:**

- Verify: operations/fleet_manager/tests/
- Verify: operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/test/
- Verify: vehicle_communication/test/

**Interfaces:**

- Consumes: Task 1부터 Task 5까지의 HTTP 계약.
- Produces: 실행 전 검증 결과와 운영 절차.

- [ ] **Step 1: 세 서비스의 전체 테스트를 실행한다**

    cd operations/fleet_manager && .venv/bin/python -m pytest tests -q
    cd ../fleet_bridge/server/ros2_ws && python3 -m pytest src/fleet_bridge_worker/test -q
    cd ../../../.. && python3 -m pytest vehicle_communication/test -q

- [ ] **Step 2: 정적 계약과 shell 구문을 검증한다**

    bash -n vehicle_communication/tools/pallet3_mission.sh
    git diff --check
    git status --short

- [ ] **Step 3: 실행 절차를 검토한다**

Fleet Manager에서 POST /api/v1/vehicles/robot_1/missions/pallet3 body로 {"pick_mode":"auto_dock"} 또는 {"pick_mode":"manual"}을 보낸다. Orchestrator는 중지하고 Fleet Manager의 ORCHESTRATOR_EVENT_URL은 비운다. Fleet Manager와 Inventory는 중앙 서버의 127.0.0.1:8090, 127.0.0.1:8081이며, 차량 API는 127.0.0.1:8082이다.

- [ ] **Step 4: 최종 커밋을 확인한다**

    git log --oneline -5
    git status --short

