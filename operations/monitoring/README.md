# Monitoring

관제용 Foxglove 서비스를 한 bundle로 제공한다.

- `foxglove-bridge`: 관제 PC ROS Domain 225의 `/robot_N/*`, `/map`, `/tf`,
  `/warehouse/zones`를 관측 전용 WebSocket `ws://<server-ip>:8765`으로 노출한다.
- `asset-server`: Foxglove 3D Panel용 URDF·mesh를
  `http://<server-ip>:8088/hiwonder_mecanum_forklift/urdf/hiwonder_mecanum_forklift.urdf`
  에서 제공한다.

```bash
cp operations/monitoring/.env.example operations/monitoring/.env.server
docker compose --env-file operations/monitoring/.env.server \
  -f operations/monitoring/docker-compose.yaml up -d --build
```

Foxglove Bridge는 `capabilities: [none]`으로 실행된다. 관제 endpoint에서는
topic publish, service 호출, parameter 변경을 수행할 수 없다. 지도와 창고 overlay,
차량 telemetry는 각각 map_server, warehouse_server, fleet_bridge가 발행한다.
