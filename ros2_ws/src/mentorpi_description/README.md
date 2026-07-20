# MentorPi Mecanum Description

이 패키지는 MentorPi M1의 원본 Mecanum 모델을 사용한다.

- `urdf/mecanum.xacro`: 원본 URDF/Xacro 모델. 수정 기준 파일이다.
- `urdf/mecanum.urdf`: 위 파일을 가리키는 심볼릭 링크. ROS/xacro가 없는 환경에서 URDF 뷰어 입력으로 사용한다.
- `meshes/mecanum/`: 원본 CAD STL 메시이다. 바퀴와 뎁스 카메라는 고밀도 메시다.

`mecanum.urdf`는 별도 복사본이 아니므로 직접 수정하지 않는다. 모델 변경은 반드시 `mecanum.xacro`에서 수행한다.

## ROS 없이 URDF 확인하기

일반 `urdf-viz` 0.46.1은 16비트 메시 인덱스 제한 때문에 MentorPi의 고밀도 바퀴와 뎁스 카메라 STL을 표시하지 못한다. 이 개발 환경에는 대용량 메시를 분할 렌더링하도록 빌드한 `urdf-viz-large-mesh` 명령을 설치했다.

프로젝트 루트에서 다음을 실행한다.

```bash
urdf-viz-large-mesh \
  ros2_ws/src/mentorpi_description/urdf/mecanum.urdf \
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

`description.launch.py`와 `two_robot_sim.launch.py`는 모두 `urdf/mecanum.xacro`를 로봇 설명으로 사용한다. Gazebo SDF의 바퀴 위치, IMU, 라이다, 뎁스 카메라 프레임은 이 모델의 좌표계와 맞춰져 있다.
