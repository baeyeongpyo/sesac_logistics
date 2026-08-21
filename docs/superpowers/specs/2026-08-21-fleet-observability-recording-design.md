# Fleet Observability and Recording Design

**Date:** 2026-08-21
**Status:** User-approved design; implementation-plan review pending

## Goal

Expand the server-only `fleet_bridge` bundle so that each vehicle's navigation,
health, RGB, and raw depth telemetry can be received in the central ROS Domain
225 and recorded to rosbag2. The central Foxglove endpoint must show operational
telemetry and RGB images, but must never expose depth images. The shared map is
not vehicle telemetry: the existing central map service remains the sole
publisher of `/controller_server/map`.

## Boundaries and Ownership

```text
vehicle ROS + vehicle Foxglove Bridge
  /odom, /tf, /scan_raw, /depth/image_raw, ...
      |  WebSocket CDR
      v
worker-robot-N (Domain 225) --> /robot_N/*
      |                                  |
      |                                  +--> rosbag-recorder
      v
server-foxglove :8765 --> central Foxglove (RGB, no depth)

central map-server (Domain 225) --> /controller_server/map
                                      |             |
                                      +--> Foxglove +--> rosbag-recorder

worker-robot-N --> /robot_N/fleet_bridge/status
```

There are three different topic owners. Their configuration must not be mixed.

| Owner | Topic shape | Configuration owner |
| --- | --- | --- |
| Vehicle relay | `/{robot}/...` | `config/telemetry.yaml` |
| Worker-derived connection state | `/{robot}/fleet_bridge/status` | worker runtime; always recorded for an enabled vehicle |
| Central server publisher | `/controller_server/map` | `config/central_topics.yaml` |

`/map` and `/{robot}/map` are not relayed as normal vehicle telemetry. A common
map is supplied once by the central map service. The `fleet_bridge` Compose file
does not create that service; it consumes its existing Domain 225 publisher.

## Vehicle Telemetry Contract

Each entry in `config/telemetry.yaml` keeps the existing contract:
`enabled`, `source`, `target`, `type`, `worker_rate`, and `qos`. `enabled: true`
means that the worker subscribes to that vehicle Bridge channel, republishes it
on the namespaced target, and the recorder includes that target. Changing the
value requires recreation of the affected worker and rosbag-recorder; hot reload
is out of scope.

The initial enabled set is below. Source names are the current physical vehicle
contract; if a vehicle Bridge does not advertise one, the worker leaves that
entry unsubscribed without failing the rest of the stream.

| ID | Vehicle source | Central target | Type | Central Foxglove | rosbag |
| --- | --- | --- | --- | --- | --- |
| `odom` | `/odom` | `/{robot}/odom` | `nav_msgs/msg/Odometry` | yes | yes |
| `tf` | `/tf` | `/{robot}/tf` | `tf2_msgs/msg/TFMessage` | yes | yes |
| `tf_static` | `/tf_static` | `/{robot}/tf_static` | `tf2_msgs/msg/TFMessage` | yes | yes |
| `amcl_pose` | `/amcl_pose` | `/{robot}/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | yes | yes |
| `scan_raw` | `/scan_raw` | `/{robot}/scan_raw` | `sensor_msgs/msg/LaserScan` | yes | yes |
| `scan_filtered` | `/scan_filtered` | `/{robot}/scan_filtered` | `sensor_msgs/msg/LaserScan` | yes | yes |
| `imu_data_raw` | `/imu/data_raw` | `/{robot}/imu/data_raw` | `sensor_msgs/msg/Imu` | yes | yes |
| `battery` | `/ros_robot_controller/battery` | `/{robot}/battery` | `sensor_msgs/msg/BatteryState` | yes | yes |
| `diagnostics` | `/diagnostics` | `/{robot}/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | yes | yes |
| `rgb_image_raw` | `/ascamera/camera_publisher/rgb0/image` | `/{robot}/rgb/image_raw` | `sensor_msgs/msg/Image` | yes | yes |
| `depth_image_raw` | `/depth/image_raw` | `/{robot}/depth/image_raw` | `sensor_msgs/msg/Image` | no | yes |
| `depth_camera_info` | `/depth/camera_info` | `/{robot}/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | no | yes |
| `navigation_goal` | `/move_base_simple/goal` | `/{robot}/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | yes | yes |
| `navigation_status` | `/navigation/status` | `/{robot}/navigation/status` | `std_msgs/msg/String` | yes | yes |
| `navigation_cmd_vel` | `/navigation/cmd_vel` | `/{robot}/navigation/cmd_vel` | `geometry_msgs/msg/Twist` | yes | yes |
| `controller_cmd_vel` | `/controller/cmd_vel` | `/{robot}/controller/cmd_vel` | `geometry_msgs/msg/Twist` | yes | yes |

The following are declared with `enabled: false` so that they can be enabled for
navigation debugging without code changes: `/plan`, `/local_plan`,
`/global_costmap/costmap`, `/local_costmap/costmap`, and
`/navigate_to_pose/_action/status`. Their respective message types are
`nav_msgs/msg/Path`, `nav_msgs/msg/Path`, `nav_msgs/msg/OccupancyGrid`,
`nav_msgs/msg/OccupancyGrid`, and `action_msgs/msg/GoalStatusArray`.

Sensor QoS is best-effort/volatile with a short queue; `tf_static` is
reliable/transient-local. Navigation, health, and status messages are
reliable/volatile. Raw image and IMU entries have no server-side rate cap so a
bag preserves the received stream. `scan_filtered` retains its 2 Hz cap for
operational display, while `scan_raw` is uncapped for bag replay.

This policy does not make raw images cheaper to transmit: depth must traverse
the vehicle Bridge when recording is enabled. Source-side image compression,
resolution reduction, and FPS limiting are a later, separate change.

## Central Display Policy

`config/server_foxglove.yaml` becomes an exact central display allowlist rather
than the current broad `^/robot_N/.*$` patterns. It permits the table's `yes`
targets, `/robot_N/fleet_bridge/status`, and `/controller_server/map`. It does
not include either depth target or disabled Nav2 debugging targets.

Therefore a direct connection to a vehicle Foxglove Bridge may inspect raw
depth, while a client connected to `ws://<central-server>:8765` can inspect RGB
but cannot discover or subscribe to depth. The central Bridge remains
observation-only: publishing, services, and parameters stay disabled.

## Recording Service

Add a `rosbag-recorder` service to `docker-compose.server.yaml`, using the
existing server image and Domain 225 with host network and host IPC. It mounts:

- `config/fleet.yaml` and `config/telemetry.yaml` read-only;
- new `config/central_topics.yaml` read-only;
- required `ROSBAG_HOST_DIRECTORY` at `/rosbag`.

The new `central_topics.yaml` contains the independently owned map entry:

```yaml
version: 1
topics:
  - id: controller_map
    enabled: true
    topic: /controller_server/map
```

A `fleet_rosbag_recorder` console entry point reads the enabled telemetry for
`ROBOT_IDS=robot_1,robot_2`, expands every target, adds each worker status
topic, and adds enabled central topics. It then invokes `ros2 bag record` with
that explicit list. It creates a UTC-named session directory when
`ROSBAG_SESSION_ID` is empty; a provided session ID must be valid and must not
already exist. Thus no existing bag is overwritten.

`ros-humble-rosbag2` is installed in the server runtime. `ros-humble-action-msgs`
is installed so the optional Nav2 action-status entry can be enabled later.

## Runtime Validation

Before enabling a new physical vehicle source, validate it on that vehicle:

```bash
ros2 topic list -t | sort
ros2 topic info -v /ascamera/camera_publisher/rgb0/image
ros2 topic info -v /depth/image_raw
ros2 topic info -v /navigation/cmd_vel
ros2 topic hz /ascamera/camera_publisher/rgb0/image
```

Nav2 itself neither starts nor stops the camera node in this bundle. If the RGB
publisher disappears during navigation, it is a vehicle camera-process or
vehicle-Bridge issue, not a central whitelist transition. The central worker
can only relay channels advertised by the vehicle Bridge.

## Tests and Acceptance Criteria

1. The configuration loader validates central topic documents, rejects unknown
   keys, duplicate active topic paths, invalid IDs, and invalid topic names.
2. Repository telemetry tests assert the enabled operational set, the disabled
   Nav2 debug set, and the absence of vehicle map relay entries.
3. Recorder unit tests assert deterministic target expansion for both robots,
   inclusion of worker status and central map, omission of disabled entries, and
   safe session-name/overwrite handling.
4. Compose contract tests assert that the recorder uses Domain 225, host
   network/IPC, read-only configuration mounts, and the required bag directory.
5. Foxglove contract tests assert `/controller_server/map` and RGB are allowed,
   while depth paths are not allowed and observation-only permissions remain.
6. `docker compose --env-file .env.example -f docker-compose.server.yaml config
   --quiet`, configuration tests, worker tests, and bundle/compose contract
   tests pass. Existing unrelated `.env.example` changes are preserved and any
   baseline contract mismatch is reported rather than overwritten.

## Non-Goals

- Modifying the vehicle-side Foxglove Bridge deployment, its whitelist, camera
  launch process, or Nav2 launch process; those are outside `fleet_bridge`.
- Adding compression, image republishers, a central map-server service, map
  generation, or TF frame rewriting.
- Making recording or telemetry settings hot-reload without container
  recreation.
