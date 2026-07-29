# Task 7 구현 보고서: profile-bound viewer preflight 및 서버 운영 문서

## 구현 내용

- `viewer-up`의 Docker Compose 최소 버전 확인을 선택 profile에 묶었다.
  이제 probe는 `docker compose --env-file "$PROFILE_FILE" version --short`를
  실행하므로 stack operation과 동일한 `.env.<profile>`을 사용한다.
- fake Docker는 앞쪽 Compose 옵션과 무관하게 마지막 `version --short` 인자를
  인식하도록 갱신했고, 회귀 테스트는 선택한 `.env.test`가 version probe에
  정확히 전달되는지 검증한다. 기존의 지원 버전 거부와 `--wait` 실패 전파
  검증도 profile-bound 인자를 확인한다.
- README는 `--env server`를 LAN mode 및 affected simulation services의 host
  networking으로, `--env dev`와 `--env server-viewer`만 Docker 내부 mode로
  구분한다.
- 서버 이미지 변경은 `.env.server`의 `MENTORPI_IMAGE`를 편집한 뒤 그 정확한
  tag 또는 digest reference를 `docker pull`하도록 문서화했다. 선택 profile이
  shell export보다 우선하므로 inherited `MENTORPI_IMAGE` export에 의존하지
  않도록 명시했다.

## 리뷰 후속 수정

- P2 리뷰 지적에 따라 `## 서버 운영` 섹션에서 internal `mentorpi` bridge,
  Docker DNS, 외부 포트 비공개라는 `dev` 전제 설명을 제거했다.
- `--env server`는 affected simulation services가 host networking을 쓰는 LAN
  profile임을 명시했다. host-network containers의 discovery client는
  `DDS_DISCOVERY_HOST=127.0.0.1`로 연결되어 control traffic이 loopback을
  사용하며, `GZ_SERVER_IP`의 LAN Gazebo Transport 노출은 host firewall의
  신뢰된 CIDR 제한으로 관리한다고 설명했다.
- Docker internal `mentorpi` bridge와 Docker DNS locator 설명은
  `--env dev` 및 `--env server-viewer` profile 범위로 옮겼다.

## TDD RED 확인

production/documentation 변경 전에 아래 테스트를 추가·갱신했다.

- selected profile을 받지 않는 기존 `compose version --short` 호출은
  `test_viewer_compose_version_probe_uses_selected_profile`과 기존 version
  preflight 기대에서 실패했다.
- README 계약 테스트는 production 문구를 바꾸기 전에 추가했다. 구현 후
  `host networking`을 의도적으로 `host network`로 변이하여 해당 단일 테스트가
  실패하는 것도 확인했다.

```text
python3 vehicle_simulator_model/ubuntu/test/test_observation_bundle.py -v
FAILED (failures=3, skipped=1)
```

세 실패는 모두 기존 `ARGS=compose version --short`가 선택 profile 없이
호출된 데서 발생했다. 이 실행은 `&&` 뒤의 bundle suite를 시작하지 않았으므로,
문서 회귀 테스트는 위의 의도적 mutation으로 실패를 확인한 뒤 원상 복구했다.

리뷰 후속의 server-section 회귀 테스트도 문서 변경 전에 추가했다.

```text
python3 -m unittest -v \
  vehicle_simulator_model.ubuntu.test.test_bundle.DeployOnlyBundleTest.test_server_operator_docs_describe_lan_host_networking
FAILED (failures=1)
```

기존 문서에는 standalone `--env server` LAN/host-network 설명과 loopback,
firewall 경계가 없고 stale internal-bridge 문구가 남아 있어 실패했다.

## 검증 결과

집중 스위트:

```text
python3 vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py -v
Ran 19 tests
OK

python3 vehicle_simulator_model/ubuntu/test/test_observation_bundle.py -v
Ran 32 tests
OK (skipped=1)

python3 vehicle_simulator_model/ubuntu/test/test_bundle.py -v
Ran 31 tests
OK
```

skip은 현재 macOS Bash가 `wait -n`을 지원하지 않을 때만 건너뛰는 기존
viewer-supervisor 테스트다.

리뷰 후속 문서 수정 뒤의 관련 suite:

```text
python3 vehicle_simulator_model/ubuntu/test/test_bundle.py -v
Ran 32 tests
OK
```

tracked template에서만 일시 `.env.dev`를 생성하여 전체 launcher 검증을
수행한 뒤 즉시 삭제했다.

```text
./run.sh --env dev test
exit 0

temporary vehicle_simulator_model/ubuntu/.env.dev=removed
```

## 변경 범위

- `vehicle_simulator_model/ubuntu/run.sh`
- `vehicle_simulator_model/ubuntu/README.md`
- `vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py`
- `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`
- `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- `.superpowers/sdd/2026-07-30-mentorpi-named-runtime-profiles/task-7-report.md`

profile template 및 실제 local `.env` 파일은 변경하거나 커밋하지 않았다.
