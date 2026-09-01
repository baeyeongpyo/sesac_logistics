# Monitoring

관제 화면을 구성하는 서비스를 하나의 bundle로 제공한다.

- `foxglove-bridge`: 관제 PC ROS Domain 225의 `/robot_N/*`, `/map`, `/tf`,
  `/warehouse/zones`를 관측 전용 WebSocket `ws://<server-ip>:8765`으로 노출한다.
- `asset-server`: Foxglove 3D Panel용 URDF·mesh를
  `http://<server-ip>:8088/hiwonder_mecanum_forklift/urdf/hiwonder_mecanum_forklift.urdf`
  에서 제공한다.
- `map-publisher`: 중앙 정적 지도와 `map -> map_visualization` 변환을 발행한다.
- `warehouse-zone-publisher`: 창고 물류 지점 overlay를 `/warehouse/zones`로 발행한다.

```bash
cp operations/monitoring/.env.example operations/monitoring/.env.server
docker compose --env-file operations/monitoring/.env.server \
  -f operations/monitoring/docker-compose.yaml up -d --build
```

Foxglove Bridge는 `capabilities: [none]`으로 실행된다. 관제 endpoint에서는
topic publish, service 호출, parameter 변경을 수행할 수 없다. 지도와 창고 overlay는
이 bundle의 publisher가, 차량 telemetry는 Fleet Bridge가 발행한다.

지도 원본은 `map_server/maps/`, 창고 layout 원본은
`warehouse_server/config/warehouse_zones.yaml`에서 관리한다. 두 파일은 container에
읽기 전용으로 mount된다.
