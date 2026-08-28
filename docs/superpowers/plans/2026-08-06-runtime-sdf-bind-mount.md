# Runtime SDF Bind Mount Implementation Plan

> **For implementation:** follow this plan with the execution-plan workflow, completing and verifying each task in order.

**Goal:** 개발 PC와 서버 모두에서 Gazebo warehouse 월드 및 모델 SDF 자산을 읽기 전용 bind mount로 제공하여, 이미지 재빌드 없이 자산 변경을 반영할 수 있게 한다.

**Architecture:** 기본 Compose 파일의 `gazebo-server`와 `sim-adapter` 서비스에 동일한 `worlds/` 및 `models/` bind mount를 선언한다. 두 서비스가 설치된 `mentorpi_gz_sim` 패키지 공유 경로에서 같은 자산을 읽으므로 Gazebo 실행 장면과 Foxglove SceneUpdate 장면이 일치한다. 운영 문서는 서버 배포 번들에 두 자산 디렉터리를 포함해야 함과, 자산 변경 후 시뮬레이터 재시작이 필요함을 명확히 한다.

**Tech Stack:** Docker Compose, Gazebo Harmonic, ROS 2 Humble, Python `unittest`

---

## Task 1: Compose bind mount 계약을 실패하는 테스트로 고정

**Files:**

- Modify: `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- Modify: `vehicle_simulator_model/ubuntu/compose.yaml`

**Step 1: 실패하는 테스트 작성**

`test_bundle.py`의 Compose 검증 클래스에 `test_gazebo_services_mount_runtime_sdf_assets_read_only`를 추가한다. `compose.yaml`을 YAML로 읽어 다음 두 서비스 각각에 아래 두 long-syntax volume이 존재하는지 검증한다.

```python
expected_mounts = [
    {
        "type": "bind",
        "source": "./ros2_ws/src/mentorpi_gz_sim/worlds",
        "target": "/opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/worlds",
        "read_only": True,
    },
    {
        "type": "bind",
        "source": "./ros2_ws/src/mentorpi_gz_sim/models",
        "target": "/opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/models",
        "read_only": True,
    },
]
```

검증 대상 서비스는 `gazebo-server`, `sim-adapter`다. 서비스별 `volumes`에서 각 기대 mount의 모든 필드가 일치해야 하며, 코드나 플러그인 라이브러리 경로를 mount하는 항목은 추가하지 않는다.

**Step 2: 테스트가 실패하는지 확인**

Run:

```bash
cd vehicle_simulator_model/ubuntu && python3 test/test_bundle.py -v
```

Expected: 새 테스트가 `gazebo-server`와 `sim-adapter`의 누락된 bind mount를 이유로 실패한다.

**Step 3: 최소 Compose 구현**

`compose.yaml`의 `gazebo-server`와 `sim-adapter`에 각각 `volumes`를 추가한다. 아래의 world와 model 디렉터리를 설치된 패키지 share 경로의 동명 디렉터리에 long syntax bind mount하고 `read_only: true`를 설정한다.

```yaml
- type: bind
  source: ./ros2_ws/src/mentorpi_gz_sim/worlds
  target: /opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/worlds
  read_only: true
- type: bind
  source: ./ros2_ws/src/mentorpi_gz_sim/models
  target: /opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/models
  read_only: true
```

기본 Compose 파일에만 선언하여 `dev`, `server`, `server-viewer` 프로필이 모두 같은 정책을 상속하도록 한다.

**Step 4: 단위 테스트 재실행**

Run:

```bash
cd vehicle_simulator_model/ubuntu && python3 test/test_bundle.py -v
```

Expected: 모든 bundle 검증 테스트가 통과한다.

**Step 5: 변경 커밋**

```bash
git add vehicle_simulator_model/ubuntu/compose.yaml vehicle_simulator_model/ubuntu/test/test_bundle.py
git commit -m "feat: mount runtime Gazebo SDF assets"
```

## Task 2: 서버 배포 및 갱신 절차 문서화

**Files:**

- Modify: `vehicle_simulator_model/ubuntu/README.md`
- Modify: `vehicle_simulator_model/ubuntu/test/test_bundle.py`

**Step 1: 실패하는 문서 계약 테스트 작성**

기존 README 검증 테스트에 다음 운영 조건을 확인하는 assertion을 추가한다.

- 서버 배포 시 `ros2_ws/src/mentorpi_gz_sim/worlds`와 `ros2_ws/src/mentorpi_gz_sim/models`를 Compose 파일과 함께 배포해야 한다.
- 위 두 경로만 런타임 읽기 전용 bind mount 예외이며 ROS 소스 코드나 플러그인 바이너리는 mount하지 않는다.
- SDF, mesh, model 자산 변경 후 `./run.sh --env <profile> sim-down` 및 `sim-up`으로 Gazebo와 SceneUpdate publisher를 재시작한다.
- 코드 또는 Gazebo 플러그인 변경에는 이미지 재빌드가 필요하다.

**Step 2: 테스트가 실패하는지 확인**

Run:

```bash
cd vehicle_simulator_model/ubuntu && python3 test/test_bundle.py -v
```

Expected: README에 아직 없는 운영 문구를 이유로 문서 계약 테스트가 실패한다.

**Step 3: README를 구현 정책에 맞게 수정**

README의 배포 원칙 및 server 배포 절차를 수정한다.

- 기존의 “ROS 소스를 bind mount하지 않는다”는 설명을 유지하되, `mentorpi_gz_sim`의 `worlds/`와 `models/`은 SDF/mesh 자산 갱신용 읽기 전용 예외라고 명시한다.
- 서버에 이미지와 Compose 파일만 배포한다는 설명은 두 자산 디렉터리도 함께 배포해야 한다고 수정한다.
- 자산 수정 시 적용 절차와 코드/플러그인 수정 시 재빌드 절차를 분리해 적는다.

**Step 4: 문서 계약 테스트 재실행**

Run:

```bash
cd vehicle_simulator_model/ubuntu && python3 test/test_bundle.py -v
```

Expected: 문서 계약을 포함한 모든 bundle 테스트가 통과한다.

**Step 5: 변경 커밋**

```bash
git add vehicle_simulator_model/ubuntu/README.md vehicle_simulator_model/ubuntu/test/test_bundle.py
git commit -m "docs: document runtime SDF asset deployment"
```

## Task 3: Compose 렌더링과 회귀 범위를 검증

**Files:**

- Verify only: `vehicle_simulator_model/ubuntu/compose.yaml`
- Verify only: `vehicle_simulator_model/ubuntu/compose.lan.yaml`
- Verify only: `vehicle_simulator_model/ubuntu/compose.viewer.yaml`

**Step 1: dev 및 server Compose 구성을 렌더링**

Run:

```bash
cd vehicle_simulator_model/ubuntu && docker compose --env-file .env.dev -f compose.yaml config --quiet
cd vehicle_simulator_model/ubuntu && docker compose --env-file .env.server -f compose.yaml -f compose.lan.yaml config --quiet
```

Expected: 두 프로필 모두 Compose 문법과 mount 경로를 정상적으로 해석한다.

**Step 2: viewer 프로필 상속 확인**

Run:

```bash
cd vehicle_simulator_model/ubuntu && docker compose --env-file .env.server -f compose.yaml -f compose.lan.yaml -f compose.viewer.yaml config --quiet
```

Expected: viewer 프로필이 기본 `gazebo-server`와 `sim-adapter` mount를 유지하며 별도 충돌이 없다.

**Step 3: 전체 테스트 실행**

Run:

```bash
./run.sh --env dev test
git diff --check
git status --short
```

Expected: 프로젝트 테스트와 whitespace 검증이 통과한다. 이미 존재한 `llm-wiki-core/llm_wiki_core/__pycache__/` 변경은 이번 작업과 무관하므로 커밋에 포함하지 않는다.

**Step 4: 계획 커밋**

```bash
git add -f docs/superpowers/plans/2026-08-06-runtime-sdf-bind-mount.md
git commit -m "docs: add SDF asset mount implementation plan"
```
