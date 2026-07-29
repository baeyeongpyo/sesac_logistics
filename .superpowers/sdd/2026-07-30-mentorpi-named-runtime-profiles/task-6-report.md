# Task 6 구현 보고서: named runtime profile 계약 보완

## 구현 내용

- launcher가 profile 뒤에 다시 나타나는 `--env`를 command 위치와 관계없이
  Docker 호출 전에 거부하도록 했다.
- `build`, `down`, `logs`, `topics`, `test`, `fork-up`, `viewer-down`,
  `viewer-logs`, `mapping-stop`, help alias는 추가 인자를 거부한다.
  `sim-up`은 선택적 `gpu`, `viewer-up`은 선택적 `local|public`,
  `mapping-up`/`mapping-status`는 정확히 하나의 session ID만 허용한다.
  optional tail 판별은 값의 non-empty 여부가 아니라 `RUN_COMMAND` 배열 길이를
  사용하므로 명시적으로 전달된 빈 문자열도 Docker 호출 전에 거부한다.
- 선택 profile의 명시적 빈 값을 유지하도록 Bash와 Compose의 설정 기본값을
  `${VAR-default}` 형식으로 바꿨다. 위치 인자 판독과 오류 진단 표시용
  fallback은 기존 `:-` 의미를 유지했다.
- internal networking, 정상 image/platform,
  `COMPOSE_PROJECT_NAME=mentorpi-server-viewer`를 갖는
  `.env.server-viewer.example`을 추가했다.
- browser viewer 문서를 `--env server-viewer` 전용 흐름으로 바꾸고, LAN
  native GUI는 계속 `--env server`를 사용하며 두 환경은 서로 다른 Compose
  project와 stack임을 명시했다.
- 실행 중인 adapter가 없을 때 `fork-up`이
  `./run.sh --env <profile> sim-up`을 안내하도록 바로잡았다.

## TDD RED 확인

production/config 변경 전에 회귀 테스트를 추가하고 다음 실패를 확인했다.

```text
python3 vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py -v
Ran 17 tests in 9.670s
FAILED (failures=12)
```

실패에는 빈 `MENTORPI_IMAGE`, `down --env server`, 무인자 명령 tail,
`sim-up gpu unexpected`, server-viewer template 부재, fork-up의 이전 안내가
각각 포함됐다.

```text
python3 vehicle_simulator_model/ubuntu/test/test_bundle.py -v
Ran 30 tests in 5.004s
FAILED (failures=3)
```

bundle RED는 새 template 부재, unset-only metadata 기대, server-viewer 문서
계약 부재에서 발생했다.

리뷰 blocker follow-up도 production 수정 전에 두 회귀 테스트로 재현했다.

```text
python3 -m unittest -v \
  ...test_sim_up_rejects_explicit_empty_tail_before_docker \
  ...test_viewer_up_rejects_explicit_empty_tail_before_docker
Ran 2 tests in 1.099s
FAILED (failures=2)
```

두 실패 모두 명시적 빈 tail이 종료 코드 0으로 Docker를 호출한 기존 결함에서
발생했다.

## 검증 결과

집중 회귀 스위트:

```text
python3 vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py -v
Ran 19 tests in 9.325s
OK

python3 vehicle_simulator_model/ubuntu/test/test_bundle.py -v
Ran 30 tests in 4.550s
OK

python3 vehicle_simulator_model/ubuntu/test/test_observation_bundle.py -v
Ran 31 tests in 14.804s
OK (skipped=1)
```

observation skip은 macOS Bash 3.2에서 `wait -n`을 지원하지 않는 기존 조건부
viewer-supervisor 테스트다.

저장소 밖 임시 dev/server/server-viewer profile을 각 tracked template에서
복사해 Compose를 검증했다.

```text
docker compose --env-file <tmp>/.env.dev \
  -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
compose_dev=PASS

docker compose --env-file <tmp>/.env.server \
  -f vehicle_simulator_model/ubuntu/compose.yaml \
  -f vehicle_simulator_model/ubuntu/compose.lan.yaml config --quiet
compose_server=PASS

docker compose --env-file <tmp>/.env.server-viewer \
  -f vehicle_simulator_model/ubuntu/compose.yaml \
  -f vehicle_simulator_model/ubuntu/compose.viewer.yaml \
  --profile viewer config --quiet
compose_server_viewer=PASS
viewer_project=mentorpi-server-viewer
```

syntax 검증:

```text
bash -n vehicle_simulator_model/ubuntu/run.sh
python3 -m py_compile \
  vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/test/test_observation_bundle.py
syntax_checks=PASS
```

tracked template에서 임시 `.env.dev`를 만든 뒤 전체 launcher 검증을 수행하고,
종료 직후 해당 파일만 삭제했다.

```text
cp vehicle_simulator_model/ubuntu/.env.dev.example \
  vehicle_simulator_model/ubuntu/.env.dev
./vehicle_simulator_model/ubuntu/run.sh --env dev test
exit 0

host static:
- test_bundle.py: 30 passed
- test_runtime_env_config.py: 17 passed
- test_observation_bundle.py: 30 passed, 1 skipped
- test_original_model.py: 9 passed
- test_harmonic_launch_contract.py: 5 passed

compose-config:
- base config passed
- GPU overlay config passed

runtime-ctest:
Summary: 58 tests, 0 errors, 0 failures, 1 skipped

rm -- vehicle_simulator_model/ubuntu/.env.dev
temporary_dev_profile=removed
```

## 변경 범위

- `vehicle_simulator_model/ubuntu/run.sh`
- `vehicle_simulator_model/ubuntu/compose.yaml`
- `vehicle_simulator_model/ubuntu/compose.viewer.yaml`
- `vehicle_simulator_model/ubuntu/.env.server-viewer.example`
- `vehicle_simulator_model/ubuntu/README.md`
- `vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py`
- `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- `.superpowers/sdd/2026-07-30-mentorpi-named-runtime-profiles/task-6-report.md`

`test_observation_bundle.py`는 Task 6 brief에 명시된 허용 범위 안에서
`VIEWER_PORT`의 unset-only Compose 계약을 검증하는 기존 source assertion 한
줄만 갱신했다.

실제 local `.env` 파일은 수정하지 않았다. 전체 검증용 `.env.dev`는 tracked
template에서만 생성했고 검증 후 삭제했으며 커밋 대상에 포함하지 않는다.
