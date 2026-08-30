# Foxglove Asset Server

Foxglove 3D Panel의 URDF와 mesh만 제공하는 독립 Docker bundle이다. ROS, DDS,
`vehicle_simulator_model`에는 의존하지 않는다.

## 자산 구조

기본 자산은 이 bundle의 `assets/`에 포함된다. Compose는 해당 디렉터리를 이미지에
복사하지 않고 `/assets`로 읽기 전용 마운트한다.

```text
assets/
└── hiwonder_mecanum_forklift/
    ├── urdf/hiwonder_mecanum_forklift.urdf
    ├── meshes/mentorpi/{base_link,cam_Link,lidar_Link}.STL
    └── model.sdf
```

`model.sdf`와 STL은 `agent/prj-araneum-canonical` 브랜치의
`models/hiwonder_mecanum_forklift`에서 복구했다. Foxglove용 URDF는 그 SDF의
시각 모델을 기반으로 변환한 파일이며, 원본 정보는 `SOURCE.md`와
`meshes/mentorpi/SOURCE.txt`에 남긴다. 지도 PGM은 이 서버에서 제공하지 않고 ROS
`/map` `nav_msgs/msg/OccupancyGrid` topic을 사용한다.

## 시작

```bash
cd foxglove_assert_server
cp .env.example .env
docker compose up -d --build
docker compose ps
```

Foxglove 3D Panel의 URDF Custom Layer URL은 다음과 같다.

```text
http://<관제-서버-LAN-IP>:8088/hiwonder_mecanum_forklift/urdf/hiwonder_mecanum_forklift.urdf
```

기본 `ASSET_PORT`는 `8088`이며, 변경했다면 URL의 포트도 같은 값으로 바꾼다.
URDF 내부 mesh는 URDF 디렉터리를 기준으로 한 `../meshes/...` 상대 경로여서 같은 서버의
mesh URL로 해석된다. 서버는 Foxglove 웹 앱에 필요한 CORS와 HTTP byte-range 응답을
제공한다. `ASSET_PORT`는 신뢰된 LAN PC에서만 접근하도록 방화벽을 설정한다.
