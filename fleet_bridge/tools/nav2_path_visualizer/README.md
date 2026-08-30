# NAV / Path Lab

차량·ROS 2·Foxglove·FastAPI 없이 `global plan`, `transformed global plan`, `local plan`의 관계를 지도 위에서 확인하는 정적 브라우저 도구입니다.

모든 계산과 PGM/YAML 파일 읽기는 브라우저 안에서 실행됩니다. 서버 실행, 패키지 설치, 네트워크 연결이 필요 없습니다.

`index.html`에는 실행용 JavaScript가 인라인으로 포함되어 있어 Chrome에서도 `file://`로 바로 열 수 있습니다. 소스인 `model.mjs` 또는 `app.mjs`를 수정한 개발자는 배포 전에 아래 명령으로 `index.html` 번들을 갱신합니다. 이 명령은 최종 사용자의 실행 과정에는 필요 없습니다.

```bash
node fleet_bridge/tools/nav2_path_visualizer/build-standalone.mjs
```

## 실행

Finder 또는 브라우저에서 [index.html](./index.html)을 엽니다.

## Graph Router / Graph Planner 설명 화면

창고의 Lane Graph에서 출발지와 도착지를 클릭해, Dijkstra 기반의 Graph Router와 Graph Planner가 각각 무엇을 만드는지 확인하려면 [graph-route-explainer.html](./graph-route-explainer.html)을 엽니다.

- 첫 waypoint 클릭: 차량의 현재 위치(출발 Node)를 지정합니다.
- 두 번째 waypoint 클릭: 도착 Node를 지정합니다.
- Graph Router: 주황색 Lane 중 선택된 Lane ID 순서인 `RoutePlan`을 만듭니다.
- Graph Planner: 선택된 Lane geometry를 연결해 파란 점선의 `nav_msgs/Path`를 만듭니다.

이 화면도 정적 HTML이며 실제 ROS 토픽을 발행하거나 차량을 제어하지 않습니다.

1. `PGM 지도`와 `YAML metadata`를 선택하고 **선택 지도 읽기**를 누릅니다. P2/P5 PGM과 map_server 형식 YAML을 지원합니다.
2. 지도 위에 커서를 올려 `x`, `y`, grid cell, 주행 가능/장애물 상태를 확인합니다. 이 hover 정보는 입력값을 바꾸지 않습니다.
3. **Start 선택** 또는 **Goal 선택**을 누른 뒤 지도에서 입력합니다.
   - 짧게 클릭하면 해당 `x`, `y`만 채우고 기존 `yaw°`는 유지합니다.
   - 클릭한 채로 드래그하면 시작점은 위치, 드래그 방향은 heading이 되어 `yaw°`까지 채웁니다.
4. 지도 위의 `−`, `+`, `맞춤` 버튼이나 마우스 휠로 확대·축소합니다. 휠 확대는 커서가 가리키는 지도 좌표를 기준으로 동작합니다.
5. **경로 시뮬레이션**을 누릅니다.

프로젝트의 기본 지도는 다음 파일입니다.

- [`maps/map_0825.pgm`](../../maps/map_0825.pgm)
- [`maps/map_0825.yaml`](../../maps/map_0825.yaml)

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
