# Warehouse Overlay Server

이 Monitoring 하위 구성요소는 창고 물류 지점 layout을 `/warehouse/zones` MarkerArray로 발행한다.
차량 API, telemetry DB, 지도 파일을 소유하지 않는다.

```bash
cp operations/monitoring/.env.example operations/monitoring/.env.server
docker compose --env-file operations/monitoring/.env.server \
  -f operations/monitoring/docker-compose.yaml up -d --build warehouse-zone-publisher
```

`config/warehouse_zones.yaml`은 운영자가 관리하는 layout 원본이며, container에는
읽기 전용으로 mount된다. publisher는 관제 호스트 ROS Domain 225에서 실행한다.
