# Central Map Server

이 bundle은 관제 PC에서 중앙 정적 지도만 발행한다. 차량과 통신하지 않으며
`/map`과 Foxglove의 지도 시각화용 `map -> map_visualization` 변환만 소유한다.

```bash
cp operations/map_server/.env.example operations/map_server/.env.server
docker compose --env-file operations/map_server/.env.server \
  -f operations/map_server/docker-compose.yaml up -d --build
```

`MAP_DIRECTORY`에는 `MAP_YAML`이 가리키는 YAML과 PGM이 있어야 한다. map 데이터는
컨테이너에 읽기 전용으로 mount되며, `map-publisher`는 관제 호스트 ROS Domain 225에서
단일 `/map` publisher로 실행한다.

`tools/nav2_path_visualizer`는 이 중앙 map data를 검토하고 Nav2 경로를 설명하는
정적 도구다. ROS와 차량 통신을 실행하지 않으며 map_server의 운영 보조 도구로 관리한다.
