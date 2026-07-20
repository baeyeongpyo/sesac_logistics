# MentorPi Mecanum Description

이 패키지는 MentorPi M1의 원본 Mecanum 차체와 전면 승강 포크 모델을 사용한다.

- `urdf/mecanum.xacro`: 보존용 원본 URDF/Xacro 모델이다. 업스트림 파일과 바이트 단위로 일치해야 한다.
- `urdf/mecanum.urdf`: 위 파일을 가리키는 심볼릭 링크. ROS/xacro가 없는 환경에서 URDF 뷰어 입력으로 사용한다.
- `urdf/mecanum_forklift.xacro`: 실제 ROS 2 및 Gazebo 기본 모델이다. 고정 마스트, 승강 캐리지, 두 개의 포크, 마스트 상단 카메라를 포함한다.
- `urdf/mecanum_forklift.urdf`: ROS 없이 확인할 수 있도록 기본 카메라 설정을 적용해 생성한 정적 URDF다.
- `meshes/mecanum/`: 원본 CAD STL 메시이다. 바퀴와 뎁스 카메라는 고밀도 메시다.

`mecanum.urdf`는 원본 Xacro를 가리키는 심볼릭 링크이므로 직접 수정하지 않는다. 포크 구조와 카메라 장착 위치는 `mecanum_forklift.xacro`에서 관리한다. `mecanum_forklift.urdf`는 아래 설정의 기본값을 렌더링한 뷰어 전용 파일이다.

## 포크 구조와 승강 범위

포크 구조는 1:12 RC 지게차 어셈블리와 실차 사진을 기준으로 단순화했다.

| 항목 | 값 |
| --- | --- |
| 마스트 | 차체 전면에 고정, 폭 120 mm, 높이 300 mm |
| 포크 | 2개, 길이 180 mm, 중심 간격 100 mm |
| 캐리지 최저 높이 | 지면 기준 25 mm |
| 승강 범위 | 0.00–0.11 m |
| 뎁스 카메라 | 마스트 상단 전방 브래킷 |
| 라이다 | 기존 차체 장착 위치 유지 |

`fork_carriage_joint`는 Z축 prismatic joint다. URDF 뷰어에서는 joint 제어로 확인하고, Gazebo에서는 목표 높이를 지정하는 position controller로 동작한다.

## 카메라 위치와 각도 설정

[`urdf/forklift_camera_config.xacro`](urdf/forklift_camera_config.xacro)에 카메라 장착 파라미터를 분리했다. 기본값에서 카메라 본체의 상단은 마스트 상단과 같은 높이이고, `camera_pitch`는 라디안 단위 피치 각도다. 기본값 `0.0`은 수평 전방을 향한다.

| 설정 | 기본값 | 의미 |
| --- | --- | --- |
| `camera_mount_x` | 0.025 m | 마스트 전면 방향 오프셋 |
| `camera_mount_y` | 0.0 m | 좌우 중심 정렬 |
| `camera_mount_z` | 0.149647 m | 마스트 기준 카메라 링크 높이. STL의 기준점 오프셋을 보정해 카메라 상단과 마스트 상단을 일치시킴 |
| `camera_pitch` | 0.0 rad | 카메라 피치 각도 |

ROS 2와 Gazebo는 이 Xacro 설정 파일을 공통으로 읽는다. `urdf-viz`용 `mecanum_forklift.urdf`는 위 기본값을 포함하므로, 설정값을 바꾼 뒤에는 ROS 환경에서 Xacro를 다시 렌더링한 URDF로 확인한다.

카메라 각도를 변경한 뒤의 모델을 뷰어에서 확인하려면 다음처럼 임시 URDF를 생성한다. `camera_pitch`만 수정하고, X/Y/Z 위치나 메시 기준점 보정값은 실제 장착 위치를 바꿀 때만 수정한다.

```bash
ros2 run xacro xacro \
  ros2_ws/src/mentorpi_description/urdf/mecanum_forklift.xacro \
  > /tmp/mecanum_forklift.urdf

urdf-viz-large-mesh /tmp/mecanum_forklift.urdf \
  --axis-scale 0.01 \
  --web-server-port 7782 \
  --package-path mentorpi_description="$PWD/ros2_ws/src/mentorpi_description"
```

`camera_pitch`는 라디안 값이다. 예를 들어 `0.174533`은 약 10도다. 포트가 사용 중이면 `--web-server-port`의 값을 다른 번호로 바꾼다.

## ROS 없이 URDF 확인하기

일반 `urdf-viz` 0.46.1은 16비트 메시 인덱스 제한 때문에 MentorPi의 고밀도 바퀴와 뎁스 카메라 STL을 표시하지 못한다. 이 개발 환경에는 대용량 메시를 분할 렌더링하도록 빌드한 `urdf-viz-large-mesh` 명령을 설치했다.

프로젝트 루트에서 다음을 실행한다.

```bash
urdf-viz-large-mesh \
  ros2_ws/src/mentorpi_description/urdf/mecanum_forklift.urdf \
  --axis-scale 0.01 \
  --web-server-port 7778 \
  --package-path mentorpi_description="$PWD/ros2_ws/src/mentorpi_description"
```

- `--package-path`는 `package://mentorpi_description/...` 메시 URI를 실제 패키지 경로로 해석한다.
- `--axis-scale 0.01`은 큰 XYZ 축이 로봇을 가리지 않게 한다.
- 포트 `7778`이 이미 사용 중이면 사용 가능한 다른 포트 번호로 바꾼다.
- 반드시 일반 `urdf-viz`가 아닌 `urdf-viz-large-mesh`를 사용한다.

## ROS 2 및 Gazebo 실행 기준

ROS 2가 설치된 환경에서는 원본 Xacro를 그대로 사용한다.

```bash
ros2 launch mentorpi_description description.launch.py
ros2 launch mentorpi_gz_sim two_robot_sim.launch.py
```

`description.launch.py`와 `two_robot_sim.launch.py`는 모두 `urdf/mecanum_forklift.xacro`를 로봇 설명으로 사용한다. Gazebo SDF의 바퀴 위치, IMU, 라이다, 뎁스 카메라, 포크 캐리지 프레임은 이 모델의 좌표계와 맞춰져 있다.

## Gazebo 포크 승강 명령

두 로봇 시뮬레이션을 실행한 뒤, ROS 2에서 목표 포크 높이를 metre 단위로 publish한다. `0.0`은 최저 위치이고 `0.11`은 최대 상승 위치다.

```bash
ros2 topic pub --once /robot_1/fork/command std_msgs/msg/Float64 "{data: 0.11}"
ros2 topic pub --once /robot_1/fork/command std_msgs/msg/Float64 "{data: 0.0}"
```

두 번째 로봇에는 `/robot_2/fork/command`를 사용한다. 범위를 벗어난 목표값은 보내지 않는다.
