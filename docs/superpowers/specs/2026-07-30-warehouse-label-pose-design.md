# 창고 바닥 텍스트 표식 위치 분리 설계

## 목표

`FRESH`, `NORMAL`, `PICO`, `ROAD 2`, 작업장 `1`~`4` 표식을 하나의
`warehouse_markings` 모델 내부 visual에서 분리한다. 사용자는
`warehouse.sdf`의 각 `<include><pose>`만 수정해 표식의 위치와 yaw를
독립적으로 변경할 수 있어야 한다.

## 선택한 방식

각 텍스트 표식은 독립적인 정적 Gazebo 모델로 제공한다. 색상 구역 판,
테두리, 충전 번개는 기존 `warehouse_markings` 모델에 유지한다. 텍스트
모델은 visual만 가지며 collision을 만들지 않는다.

월드에는 다음과 같은 include를 둔다.

```xml
<include name="fresh_label">
  <uri>model://warehouse_label_fresh</uri>
  <pose>-0.5 0.7 0.003 0 0 0</pose>
</include>
```

동일한 형태로 `normal_label`, `pico_label`, `road_2_label`,
`workstation_1_label`~`workstation_4_label`을 배치한다.

## 생성과 파일 경계

기존 `generate_floor_markings.py`는 각 글리프의 5×7 visual 생성 로직을
재사용하되, 구역 판/테두리 SDF와 표식 모델 SDF를 각각 결정적으로
생성한다. 각 표식 모델은 `model.config`, `model.sdf`를 가지며 원점이
텍스트 중앙이다. 따라서 월드의 pose가 표식 위치를 유일하게 결정한다.

## 동작과 안전성

- 모든 표식 모델은 `<static>true</static>`이며 collision이 없다.
- 표식 이동은 센서, 경로 계획, 포크, 파렛트 물리에 영향을 주지 않는다.
- Mac 네이티브 GUI와 Linux Docker Gazebo 모두 동일한 SDF resource를
  렌더링한다.
- 기존 기본 파렛트 6개, 창고 설비, 파렛트 관리 플러그인 설정은 변경하지
  않는다.

## 검증

정적 테스트는 모든 표식 모델의 존재, collision 부재, 월드 include 이름,
그리고 각 include의 pose를 검사한다. 생성기 테스트는 생성 결과가 커밋된
SDF와 byte-for-byte 일치하는지 확인한다. `gz sdf -k`로 모든 표식 SDF와
월드를 검증하고, Mac GUI에서 Entity Tree의 개별 표식을 선택해 위치
변경이 월드 pose만으로 가능한지 확인한다.
