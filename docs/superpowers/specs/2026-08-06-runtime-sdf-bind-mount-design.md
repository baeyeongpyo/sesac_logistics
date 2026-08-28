# Runtime SDF Bind Mount 설계

## 목적

개발 PC와 서버 모두에서 warehouse SDF와 Gazebo model asset을 이미지 재빌드 없이 교체한다.

## 설계

`compose.yaml`의 `gazebo-server`와 `sim-adapter`에 다음 host 디렉터리를 read-only bind mount한다.

- `./ros2_ws/src/mentorpi_gz_sim/worlds` → `/opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/worlds`
- `./ros2_ws/src/mentorpi_gz_sim/models` → `/opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/models`

`gazebo-server`는 mounted `warehouse.sdf`를 launch에서 package share 경로로 읽는다. `sim-adapter`는
warehouse scene publisher가 같은 mounted SDF와 models를 읽으므로 Gazebo와 Foxglove 장면이 일치한다.

## 운영 계약

- 적용 범위는 `dev`, `server`, `server-viewer`를 포함한 모든 Compose profile이다.
- host bundle에는 `ros2_ws/src/mentorpi_gz_sim/worlds`와 `models`가 반드시 존재해야 한다.
- mount는 `:ro`로 고정한다. 컨테이너는 SDF/model asset을 수정하지 않는다.
- SDF 또는 model SDF·mesh를 바꾼 뒤 `./run.sh --env <profile> sim-up`을 다시 실행한다. 코드 또는 plugin 변경은 여전히 이미지 재빌드가 필요하다.
- source bind mount를 사용하지 않는다는 기존 문서는 SDF/model asset의 이 예외를 명확히 기록한다.

## 검증

1. Compose config가 두 서비스에 정확한 read-only mount를 렌더링한다.
2. host SDF/model directory가 누락되면 launcher가 Docker 호출 전에 실패한다.
3. README가 서버 배포 시 asset directory 동반과 재시작 절차를 설명한다.
