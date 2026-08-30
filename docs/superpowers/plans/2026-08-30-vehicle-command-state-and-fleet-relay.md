# Vehicle Command State and Fleet Relay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 차량 API가 Nav2·Auto Dock의 작업 상태를 안전하게 관리하고 Fleet Bridge가 새 명령과 상태 계약을 그대로 중계하게 한다.

**Architecture:** `VehicleCommandService` 안에 현재 작업, 직전 작업, 실행 중인 Nav2 attempt를 구분하는 상태 전이 로직을 둔다. ROS adapter는 Auto Dock의 arrival/status/drive_ready topic을 HTTP 서비스에 연결하고, Fleet Bridge는 payload를 해석·변환하지 않는 HTTP relay만 추가한다.

**Tech Stack:** Python 3, ROS 2 (`rclpy`, Nav2 action, `std_msgs`), `unittest`, FastAPI.

**Spec:** `docs/superpowers/specs/2026-08-30-vehicle-operation-state-design.md`

## Global Constraints

- 공개 물류 상태는 `INIT`, `IDLE`, `DRIVE`, `PICKING`, `PICK_COMPLETE`, `PLACE`, `PLACE_COMPLETE`, `FAILED`, `CANCELLED`다.
- `MANUAL`은 기존 `/v1/cmd-vel` 검증 호환성을 위한 일시적 유지보수 상태이며, 타이머 만료 뒤 명령 전 상태로 복귀한다.
- `INIT_POSE`는 `/initialpose`만 발행하고 `INIT`을 `IDLE`로 바꾸지 않는다.
- `CANCELLED`와 `FAILED`는 명시적 `POST /v1/operation/idle` 전까지 새 DRIVE/Auto Dock 작업을 거부한다.
- 중앙 서버가 만든 `operation_id`를 물류 명령에 사용하며 차량은 재기동 뒤 작업을 임의 복원하지 않는다.
- Auto Dock 완료는 `/{robot_id}/auto_dock/drive_ready`만으로 판정한다. `READY` status만으로 완료를 판정하지 않는다.
- Fleet Bridge는 차량 응답 본문과 상태 코드를 그대로 중계한다.

---

### Task 1: 차량 작업 상태 모델과 HTTP 계약

**Files:**
- Modify: `vehicle_communication/vehicle_command_api.py:77-515`
- Modify: `vehicle_communication/test/test_vehicle_command_api.py:117-477`

**Interfaces:**
- Produces: `VehicleCommandService.operation_status() -> dict` with `operation_id`, `previous_operation_id`, `state`, `previous_state`, `detail`.
- Produces: `VehicleCommandService.mark_idle(payload) -> dict` for `POST /v1/operation/idle`.
- Consumes: `navigation` adapter's existing `submit_goal(attempt_id, goal, on_terminal)` and `cancel(attempt_id)` methods.

- [ ] **Step 1: Write failing state-contract tests**

```python
def test_startup_stays_init_until_operator_marks_idle(self):
    self.assertEqual(self.get_json('/v1/operation-status')[1]['state'], 'INIT')
    self.assertEqual(self.initial_pose()[0], 202)
    self.assertEqual(self.get_json('/v1/operation-status')[1]['state'], 'INIT')
    self.assertEqual(post_json(self.base_url + '/v1/operation/idle', {'reason': 'OPERATOR_CONFIRMED'})[1]['state'], 'IDLE')

def test_failed_operation_keeps_current_id_until_idle_is_explicit(self):
    _, goal = self.navigation_goal(operation_id=INVENTORY_OPERATION_ID)
    self.navigation.complete(goal['attempt_id'], 'FAILED')
    self.assertEqual(self.get_json('/v1/operation-status')[1]['operation_id'], INVENTORY_OPERATION_ID)
    self.assertEqual(post_json(self.base_url + '/v1/operation/idle', {'reason': 'OPERATOR_CONFIRMED'})[1]['previous_operation_id'], INVENTORY_OPERATION_ID)
```

- [ ] **Step 2: Run the focused tests and confirm they fail because `idle` and prior-state fields do not exist**

Run: `python3 -m unittest vehicle_communication.test.test_vehicle_command_api.VehicleCommandApiServerTest.test_startup_stays_init_until_operator_marks_idle -v`

Expected: FAIL because the API initially returns `IDLE` and `POST /v1/operation/idle` is unknown.

- [ ] **Step 3: Implement minimal transition helpers and routes**

```python
def _set_operation(self, operation_id, state, detail, previous_state=None):
    self._status = {
        'operation_id': operation_id,
        'previous_operation_id': self._status['previous_operation_id'],
        'state': state,
        'previous_state': previous_state,
        'detail': detail,
    }

def mark_idle(self, payload):
    self._validate_fields(payload, {'reason'})
    if self._active_navigation_attempt is not None or self._auto_dock_active:
        raise OperationConflictError('VEHICLE_MOTION_ACTIVE')
    if self._status['state'] not in {'INIT', 'CANCELLED', 'FAILED'}:
        raise OperationConflictError('IDLE_TRANSITION_NOT_ALLOWED')
    self._complete_to_idle('OPERATOR_READY')
    return self.operation_status()
```

Initialize in `INIT`; preserve the current operation ID on cancellation/failure; move it to `previous_operation_id` only when an explicit idle transition or a completed workflow clears it. Add `POST /v1/operation/idle`, mapped conflicts to HTTP 409, and expand OpenAPI schemas/examples.

- [ ] **Step 4: Run all vehicle API tests and verify the revised old-state expectations**

Run: `python3 -m unittest discover -s vehicle_communication/test -v`

Expected: PASS; all status snapshots include the five operation fields.

- [ ] **Step 5: Commit the independently working state-contract change**

```bash
git add vehicle_communication/vehicle_command_api.py vehicle_communication/test/test_vehicle_command_api.py
git commit -m "feat(vehicle-command-api): add guarded operation states"
```

### Task 2: Nav2 IDs, Auto Dock topic bridge, and state transitions

**Files:**
- Modify: `vehicle_communication/vehicle_command_api.py:77-858`
- Modify: `vehicle_communication/test/test_vehicle_command_api.py:36-477`
- Modify: `vehicle_communication/package.xml`

**Interfaces:**
- Produces: `POST /v1/navigation/goals` accepts optional caller `operation_id` and `purpose`; `PICK_COMPLETE` requires `purpose: 'PLACE'`, and returns `{operation_id, attempt_id, state: 'DRIVE'}`.
- Produces: `POST /v1/auto-dock` accepts `operation_id`, `operation` (`PICK`/`PLACE`), `product_type`, `location`, `target` and publishes an Auto Dock arrival JSON message.
- Consumes: Auto Dock status `String` JSON and `drive_ready` `Empty` from `/{robot_id}/auto_dock/status` and `/{robot_id}/auto_dock/drive_ready`.

- [ ] **Step 1: Write failing Nav2/Auto Dock behavior tests**

```python
def test_inventory_operation_id_drives_and_preserves_loaded_context(self):
    self.mark_idle()
    _, drive = self.navigation_goal(operation_id=INVENTORY_OPERATION_ID)
    self.assertEqual(drive['state'], 'DRIVE')
    self.navigation.complete(drive['attempt_id'], 'COMPLETED')
    self.assertEqual(self.get_json('/v1/operation-status')[1]['state'], 'IDLE')

def test_drive_ready_is_the_only_pick_completion_signal(self):
    self.mark_idle()
    self.auto_dock(operation='PICK', operation_id=INVENTORY_OPERATION_ID)
    self.auto_dock_status({'state': 'READY', 'operation': 'PICK'})
    self.assertEqual(self.get_json('/v1/operation-status')[1]['state'], 'PICKING')
    self.auto_dock_drive_ready()
    self.assertEqual(self.get_json('/v1/operation-status')[1]['state'], 'PICK_COMPLETE')
```

- [ ] **Step 2: Run focused tests and confirm the missing API/topic behavior fails**

Run: `python3 -m unittest vehicle_communication.test.test_vehicle_command_api.VehicleCommandApiServerTest.test_drive_ready_is_the_only_pick_completion_signal -v`

Expected: FAIL because `/v1/auto-dock` and Auto Dock callbacks do not exist.

- [ ] **Step 3: Implement transport-safe drive and docking transitions**

```python
def auto_dock_command(self, payload):
    command = self._auto_dock_payload(payload)
    self._assert_can_start_dock(command['operation'], command['operation_id'])
    self._auto_dock.publish_arrival({
        'status': 'SUCCEEDED',
        'location': command['location'],
        'operation': command['operation'],
        'product_type': command['product_type'],
        'target': command['target'],
    })
    self._set_operation(command['operation_id'], 'PICKING' if command['operation'] == 'PICK' else 'PLACE', 'AUTO_DOCK_COMMAND_ACCEPTED')
```

Generate an internal Nav2 `attempt_id` separately from caller-supplied `operation_id`; use the attempt only for adapter handle/cancel callbacks. Permit DRIVE only from `IDLE` or `PICK_COMPLETE`; in `PICK_COMPLETE`, require the same operation ID and `purpose: 'PLACE'`. Require Auto Dock `PICK` from `IDLE` and `PLACE` from `PICK_COMPLETE`, transition rejected/error Auto Dock status to `FAILED`, and use `drive_ready` to reach `PICK_COMPLETE` or transient `PLACE_COMPLETE` then `IDLE`.

Extend `RosVehicleAdapter` with `String` arrival publisher, `String` status subscriber, and `Empty` ready subscriber, configured from `--robot-id`; preserve explicit topic flags if the package already exposes them. Its status callback must decode JSON before it calls the service and ignore malformed/irrelevant messages safely.

- [ ] **Step 4: Run all vehicle tests and exercise ROS adapter argument wiring tests**

Run: `python3 -m unittest discover -s vehicle_communication/test -v`

Expected: PASS; no existing direct-control or shutdown safety tests regress.

- [ ] **Step 5: Commit the ROS and command change**

```bash
git add vehicle_communication/vehicle_command_api.py vehicle_communication/test/test_vehicle_command_api.py vehicle_communication/package.xml
git commit -m "feat(vehicle-command-api): bridge auto dock operations"
```

### Task 3: Fleet Bridge relay endpoints and API documentation

**Files:**
- Modify: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/api.py:199-510`
- Modify: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_api.py:68-259`

**Interfaces:**
- Produces: `POST /api/v1/vehicle-command/{robot_id}/operation/idle` forwarding unchanged to `/v1/operation/idle`.
- Produces: `POST /api/v1/vehicle-command/{robot_id}/auto-dock` forwarding unchanged to `/v1/auto-dock`.
- Consumes: vehicle API status contract from Task 1 and payload contracts from Task 2.

- [ ] **Step 1: Write failing proxy and OpenAPI tests**

```python
('POST', '/operation/idle', {'reason': 'OPERATOR_CONFIRMED'}, '/v1/operation/idle'),
('POST', '/auto-dock', {
    'operation_id': INVENTORY_OPERATION_ID,
    'operation': 'PICK',
    'product_type': 'NORMAL',
    'location': 'DOCK_1',
    'target': {'type': 'NEAREST'},
}, '/v1/auto-dock'),

self.assertIn('/api/v1/vehicle-command/{robot_id}/operation/idle', schema['paths'])
self.assertIn('/api/v1/vehicle-command/{robot_id}/auto-dock', schema['paths'])
```

- [ ] **Step 2: Run focused fleet test and confirm it fails on the absent paths**

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker python3 -m unittest fleet_bridge.server.ros2_ws.src.foxglove_ros_worker.test.test_api -v`

Expected: FAIL after FastAPI is available, because the two routes are not registered.

- [ ] **Step 3: Add transparent endpoints and update OpenAPI examples**

```python
@app.post('/api/v1/vehicle-command/{robot_id}/operation/idle', tags=['vehicle-command relay'])
async def vehicle_operation_idle(robot_id: RobotId, payload: Any = Body(default=None)):
    return await relay_vehicle_command(robot_id, 'POST', '/v1/operation/idle', payload)

@app.post('/api/v1/vehicle-command/{robot_id}/auto-dock', tags=['vehicle-command relay'])
async def vehicle_auto_dock(robot_id: RobotId, payload: Any = Body(default=None)):
    return await relay_vehicle_command(robot_id, 'POST', '/v1/auto-dock', payload)
```

Update the two status examples with `previous_operation_id` and `previous_state`, and navigation examples/descriptions so Fleet Manager identifies Inventory as the `operation_id` source rather than the vehicle.

- [ ] **Step 4: Run Fleet Bridge API tests**

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker python3 -m unittest discover -s fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test -p 'test_api.py' -v`

Expected: PASS; each new route sends the exact payload and preserves vehicle responses.

- [ ] **Step 5: Commit the relay change**

```bash
git add fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/api.py fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_api.py
git commit -m "feat(fleet-bridge): relay vehicle operation commands"
```

### Task 4: Operational documentation and full verification

**Files:**
- Modify: `docs/superpowers/specs/2026-08-30-vehicle-operation-state-design.md`
- Modify: `vehicle_communication/README.md`
- Modify: `fleet_bridge/README.md`

**Interfaces:**
- Documents: exact state recovery sequence, independent DRIVE/Auto Dock commands, Fleet Bridge routes, and the fact that `MANUAL` is a temporary maintenance state.

- [ ] **Step 1: Update documented status and command examples**

```markdown
1. `POST /v1/localization/initial-pose`
2. 현장 위치 확인
3. `POST /v1/operation/idle` with `OPERATOR_CONFIRMED`
4. 중앙 서버가 같은 Inventory `operation_id`로 DRIVE 또는 AUTO DOCK을 새로 전송
```

Describe that `INIT_POSE` does not authorize work, `FAILED`/`CANCELLED` require the idle endpoint, Auto Dock completion requires `drive_ready`, and neither vehicle API nor Fleet Bridge offers a resume or raw-state-injection endpoint.

- [ ] **Step 2: Run full available verification**

Run: `python3 -m unittest discover -s vehicle_communication/test -v`

Expected: PASS.

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker python3 -m unittest discover -s fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test -p 'test_*.py' -v`

Expected: PASS when FastAPI is installed; otherwise record the missing local dependency separately from code results.

- [ ] **Step 3: Inspect the change set and commit documentation**

```bash
git diff --check
git status --short
git add -f docs/superpowers/specs/2026-08-30-vehicle-operation-state-design.md docs/superpowers/plans/2026-08-30-vehicle-command-state-and-fleet-relay.md
git add vehicle_communication/README.md fleet_bridge/README.md
git commit -m "docs: describe vehicle operation relay workflow"
```
