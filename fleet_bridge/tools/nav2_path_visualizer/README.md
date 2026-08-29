# NAV / Path Lab

차량·ROS 2·Foxglove·FastAPI 없이 `global plan`, `transformed global plan`, `local plan`의 관계를 지도 위에서 확인하는 정적 브라우저 도구입니다.

모든 계산과 PGM/YAML 파일 읽기는 브라우저 안에서 실행됩니다. 서버 실행, 패키지 설치, 네트워크 연결이 필요 없습니다.

## 실행

Finder 또는 브라우저에서 [index.html](./index.html)을 엽니다.

1. `PGM 지도`와 `YAML metadata`를 선택하고 **선택 지도 읽기**를 누릅니다. P2/P5 PGM과 map_server 형식 YAML을 지원합니다.
2. 차량 시작점과 목표점의 `x`, `y`, `yaw°`를 입력합니다.
3. **경로 시뮬레이션**을 누릅니다.

프로젝트의 기본 지도는 다음 파일입니다.

- [`map_server/maps/map_0825.pgm`](../../../map_server/maps/map_0825.pgm)
- [`map_server/maps/map_0825.yaml`](../../../map_server/maps/map_0825.yaml)

이 지도에서 예시로 시작 `(-4.0, -3.0, 0°)`, 목표 `(3.8, 2.0, 0°)`를 넣으면 경로를 확인할 수 있습니다.

## 표시되는 레이어

| 레이어 | 의미 |
| --- | --- |
| `Global plan` | 시작점에서 목표점까지 지도 전체를 사용해 계산한 전역 경로입니다. |
| `Transformed plan` | 현재 차량 위치 근처의 3 m 로컬 영역에 맞춰 잘라낸 전역 경로입니다. 이미 지난 구간을 포함하지 않는 controller 입력의 근사입니다. |
| `Local plan` | 300개의 `(vx, vtheta)` 후보를 1.5초 동안 적분하고, 충돌·경로 정렬·목표 거리 점수를 평가해 고른 궤적입니다. |

경로 색상은 각각 연한 파랑, 노랑, 초록입니다. 노란 점선 원은 로컬 계획에 쓰는 3 m 영역의 원형 근사입니다.

## 모델 경계

- `nav2.yaml`의 NavFn/DWB 주요 값(로봇 반경 `0.16 m`, inflation `0.28 m`, 15×20 속도 후보, `sim_time: 1.5`)을 참고합니다.
- 실제 Nav2의 Costmap, TF, progress checker, DWB critic C++ 구현을 호출하지 않습니다. 따라서 이 결과는 **판단 과정 학습·시각 테스트용 근사 결과**이며 실제 차량 명령이 아닙니다.
- ROS 토픽을 발행하거나 구독하지 않으며 FastAPI, Docker, npm 의존성이 없습니다.

## 개발 확인

Node.js가 있으면 순수 모델과 정적 화면 계약을 확인할 수 있습니다.

```bash
node --test fleet_bridge/tools/nav2_path_visualizer/test/model.test.mjs \
  fleet_bridge/tools/nav2_path_visualizer/test/page-contract.test.mjs
```
