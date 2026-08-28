# Map Server

`map_server`는 중앙 ROS 2 Domain 225에서 정적 지도만 발행하는 독립 컨테이너입니다.
Nav2 map server가 `map_0825.yaml`과 해당 PGM을 읽어 `/controller_server/map`
(`nav_msgs/msg/OccupancyGrid`)으로 발행합니다. 지도 파일은 컨테이너에 복사하지 않고
읽기 전용으로 마운트합니다.

`fleet_bridge/config/central_topics.yaml`은 이 토픽을 `/map`으로 전달합니다.
그 토픽은 `reliable + transient_local + keep_last(1)`로 설정되어 있어 Foxglove가
나중에 접속해도 마지막 지도를 받을 수 있습니다. 지도는 주기적으로 재발행하지 않습니다.

## 시작

현재 기본 지도는 `map_server` 폴더 안의 다음 두 파일입니다.

```text
map_server/maps/map_0825.yaml
map_server/maps/map_0825.pgm
```

```bash
cd map_server
cp .env.example .env
docker compose up -d --build
```

Compose는 이 폴더를 컨테이너의 `/maps`에 읽기 전용으로 마운트합니다. 지도 파일은
이미지에는 포함하지 않습니다. 다른 지도를 쓰려면 `.env`의 `MAP_DIRECTORY`를
YAML/PGM이 있는 호스트 디렉터리로, `MAP_YAML`을 그 디렉터리가 컨테이너에
마운트되는 `/maps` 기준 경로로 바꿉니다. YAML의 `image`는 같은 디렉터리의 PGM
파일을 상대 경로로 가리켜야 합니다.

```bash
docker compose logs -f map-server
docker compose down
```

## Foxglove 3D Panel

Foxglove Bridge는 중앙 Domain 225에 연결합니다. 3D Panel에서 `/map`을 지도
소스로 선택하면 됩니다. URDF/mesh는 별도 `foxglove_assert_server`가 HTTP URL로
제공하며, 이 컨테이너는 지도 토픽과 `map -> map_visualization` TF만 담당합니다.
