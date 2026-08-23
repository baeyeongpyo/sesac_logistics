# Auto Dock ↔ Nav2 연동 포맷

## 전체 플로우

1. Auto Dock이 목표 팔레트를 검출한다.
2. Auto Dock이 태그맵의 `map` 좌표와 정면 yaw를 사용해 `nav_approach_standoff_m`만큼 떨어진 진입 준비 Pose를 하나 계산한다.
3. Auto Dock이 Nav2 담당 노드에 진입 준비 Pose를 발행한다.
   - Topic: `/robot_N/nav2/approach_goal`
   - Type: `geometry_msgs/msg/PoseStamped`
4. Nav2가 진입 준비 Pose까지 이동한다.
5. Nav2 담당 노드가 Auto Dock에 이동 결과를 발행한다.
   - Topic: `/robot_N/nav2/approach_result`
   - Type: `std_msgs/msg/String` (JSON)
6. 성공하면 Auto Dock이 제어권을 넘겨받아 카메라 기반 정렬과 최종 진입을 수행한다.

## 설정

`/shared/vehicle_pose_config.json`에서 다음 값을 사용한다.

```json
{
  "nav_approach_standoff_m": 0.45
}
```

단위는 m이며, 팔레트 정면으로부터 진입 준비 Pose까지의 거리다. Auto Dock은 목표를 확정할 때 이 값을 읽으며, 값이 없으면 `0.45`를 기본값으로 사용한다.

## Auto Dock → Nav2 목표 포맷

```yaml
header:
  frame_id: map
  stamp: <현재 ROS time>
pose:
  position:
    x: <진입 준비점 x, meter>
    y: <진입 준비점 y, meter>
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: sin(yaw / 2)
    w: cos(yaw / 2)
```

Nav2 담당 노드는 이 `PoseStamped`를 받아 내부적으로 `/navigate_to_pose` 액션 goal로 전달한다.

## Nav2 → Auto Dock 결과 포맷

```json
{
  "status": "succeeded"
}
```

실패 결과 예시:

```json
{
  "status": "failed",
  "reason": "no_path"
}
```

`status`는 `succeeded`, `failed`, `canceled` 중 하나다. `succeeded`일 때만 Auto Dock이 최종 정렬 단계로 진행하며, 나머지는 실패로 처리하고 차량을 정지한다.

## 내부 토픽

`/robot_N/tag_entity_map`은 Auto Dock 내부 입력이다. Nav2 담당자는 태그맵을 읽거나 좌표계를 변환할 필요가 없으며, 위 `approach_goal`과 `approach_result` 두 토픽만 연동한다.
