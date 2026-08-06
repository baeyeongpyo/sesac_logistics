---
title: Hiwonder MentorPi Getting Ready Implementation Guide
created: 2026-07-09
updated: 2026-07-09
type: source
status: active
tags:
  - robotics
  - mentorpi
  - ros2
  - getting-ready
  - implementation-guide
sources:
  - title: MentorPi Getting Ready documentation
    url: https://wiki.hiwonder.com/projects/MentorPi/en/latest/docs/1.getting_ready.html
    accessed: 2026-07-09
  - title: Preserved tracking vehicle ROS2 workspace raw copy
    path: llm-wiki/raw/mentorpi-ros2-ws-group-control-2026-07-08
    accessed: 2026-07-09
  - title: MentorPi ros2_ws current source verification
    path: llm-wiki/raw/mentorpi-ros2-ws-current-verification-2026-07-09.md
    accessed: 2026-07-09
  - title: Current source SHA-256 manifest
    path: llm-wiki/raw/mentorpi-ros2-ws-current-source-2026-07-09.sha256
    accessed: 2026-07-09
---

# Hiwonder MentorPi Getting Ready Implementation Guide

## Summary

Hiwonder's Getting Ready guide covers first-use safety, charging, assembly,
startup status, WonderPi app control, and wireless controller control. In the
tracked MentorPi `ros2_ws`, those user-facing behaviors are implemented mainly
through these packages:

| Area | Raw Code Path | Implementation Role |
|---|---|---|
| Full startup bringup | `src/bringup/launch/bringup.launch.py` | Starts hardware controller, odometry, sensors, rosbridge, video streaming, app features, joystick control, and startup check. |
| Startup beep / OLED status | `src/bringup/bringup/startup_check.py` | Waits for startup, publishes buzzer feedback, and writes SSID/IP to OLED. |
| App feature launch | `src/app/launch/start_app.launch.py` | Starts lidar, line-following, object-tracking, and hand-gesture app nodes. |
| App feature nodes | `src/app/app/*.py` | Implements the WonderPi app games as ROS2 services, topics, and processing nodes. |
| Wireless controller launch | `src/peripherals/launch/joystick_control.launch.py` | Starts `joy_node` and `peripherals.joystick_control`. |
| Wireless controller node | `src/peripherals/peripherals/joystick_control.py` | Converts `/joy` joystick messages into chassis velocity, buzzer, and optional steering servo commands. |
| Chassis control bridge | `src/driver/controller` | Converts `Twist` commands into Mecanum or Ackermann motor/servo outputs and odometry. |
| Low-level hardware bridge | `src/driver/ros_robot_controller` | Sends motor, buzzer, servo, OLED, LED, IMU, and battery operations to the RRC Lite / STM32 board. |
| Runtime environment | `.zshrc`, `.robotrc`, `.typerc` | Selects hardware type, camera type, ROS domain, DDS config, source-vs-installed package paths, and shell setup. |

The current external source path was rechecked on 2026-07-09. Its 430
non-`.DS_Store` source files match the preserved raw copy by SHA-256.

## Getting Ready to Code Mapping

| Guide Section | User-Facing Behavior | Code / Runtime Mapping |
|---|---|---|
| Product introduction | MentorPi M1 is the Mecanum variant; A1 is Ackermann. | `.typerc` selects `MACHINE_TYPE=MentorPi_Mecanum` for M1 or `MentorPi_Acker` for A1. `joystick_control.py`, `odom_publisher_node.py`, and app nodes branch on that environment value. |
| Charging and first use | Fully charge before first operation; use the supplied charger when voltage drops below 6.4 V. | Battery handling is hardware procedure, not implemented in this `ros2_ws`. Battery telemetry is surfaced through `ros_robot_controller_node.py`, which publishes low-level board data. |
| Startup status | After power-on, wait about one minute; LED/buzzer indicate normal startup; AP hotspot begins with `HW`. | `bringup.launch.py` includes `startup_check`. `startup_check.py` sleeps 50 seconds, publishes a 1900 Hz buzzer command, then writes `SSID:HW-...` and `IP:...` to `/ros_robot_controller/set_oled`. |
| App control | The WonderPi app connects in AP direct mode or STA/LAN mode and controls robot games. | `bringup.launch.py` starts `rosbridge_server`, `web_video_server`, sensors, `start_app.launch.py`, and controller nodes. The mobile app can then call ROS services and view camera streams through the bridge/video server path. |
| App games | Robot Control, Lidar, Object Tracking, Line Following, Gesture Control. | `start_app.launch.py` starts lidar, line-following, object-tracking, and hand-gesture nodes. Robot Control is covered by controller/joystick/app command paths. `ar_app_node_launch` exists but is commented out. |
| Wireless controller | USB receiver should be connected; handle pairs automatically; START beeps/stops; joysticks move the robot. | `joystick_control.launch.py` starts `joy_node` with `/dev/input/js0` and `autorepeat_rate=20.0`, then starts `joystick_control`. That node subscribes to `/joy`, maps axes to `controller/cmd_vel`, and maps START to `ros_robot_controller/set_buzzer`. |

## Runtime Environment

`source ~/.zshrc` loads the robot runtime chain:

```text
~/.zshrc
  -> $HOME/ros2_ws/.robotrc
  -> $HOME/ros2_ws/.typerc
```

Important `.typerc` values for MentorPi M1:

| Variable | Expected M1 Value | Runtime Effect |
|---|---|---|
| `MACHINE_TYPE` | `MentorPi_Mecanum` | Enables Mecanum movement mapping in joystick, odometry, and app motion logic. |
| `DEPTH_CAMERA_TYPE` | `ascamera` for depth camera, otherwise USB camera path | Controls `depth_camera.launch.py` behavior and image topic expectations. |
| `LIDAR_TYPE` | Raw copy defaults to `LD19` | Used by lidar app logic and launch paths. Confirm against the installed hardware, especially if the physical kit uses MS200. |
| `need_compile` | `False` in the raw copy | Launch files use source-tree paths under `/home/ubuntu/ros2_ws/src/...` instead of installed package share paths. |
| `ROS_DOMAIN_ID` | `0` by default | Must match across ROS2 participants that should communicate. |
| `CYCLONEDDS_URI` | `file:///etc/cyclonedds/config.xml` | Selects the robot's CycloneDDS configuration. |

Before debugging app or controller behavior, verify the shell actually loaded
these values:

```bash
source ~/.zshrc
echo "$MACHINE_TYPE"
echo "$DEPTH_CAMERA_TYPE"
echo "$LIDAR_TYPE"
echo "$need_compile"
echo "$ROS_DOMAIN_ID"
```

## Full Startup Launch Graph

The default app startup path is:

```bash
ros2 launch bringup bringup.launch.py
```

`bringup.launch.py` starts:

| Started Component | Code Path | Notes |
|---|---|---|
| Startup check | `bringup.startup_check` | Publishes buzzer and OLED status after a startup delay. |
| Controller stack | `driver/controller/launch/controller.launch.py` | Includes odometry, robot description, hardware controller, IMU filter, and EKF. |
| Depth camera | `peripherals/launch/depth_camera.launch.py` | Uses `DEPTH_CAMERA_TYPE`; `ascamera` path uses `ascamera.launch.py`. |
| Lidar | `peripherals/launch/lidar.launch.py` | Includes LD19 launch in the raw code; verify against the physical lidar model. |
| ROS bridge | `rosbridge_server rosbridge_websocket_launch.xml` | Bridge used by external app/web clients. |
| Web video | `web_video_server` | Provides camera stream access to clients. |
| App feature nodes | `app/launch/start_app.launch.py` | Starts the individual app game service nodes. |
| Joystick control | `peripherals/launch/joystick_control.launch.py` | Starts `joy_node` and single-robot joystick control. |
| Init pose | `controller/launch/init_pose.launch.py` | Runs initial pose action path. |

The raw `start_app_node.service` shows an auto-start service that runs
`ros2 launch bringup bringup.launch.py` as user `ubuntu` after networking and
time sync. The raw service line is:

```text
ExecStart=/bin/zsh -c 'source home/ubuntu/.zshrc; ros2 launch bringup bringup.launch.py;'
```

That line appears to omit the leading slash before `/home/ubuntu/.zshrc`.
Verify the installed service on the robot before assuming the raw file is the
active systemd unit.

## App Control Implementation

`src/app/setup.py` installs these console scripts:

| Console Script | Node Source | Getting Ready Feature |
|---|---|---|
| `lidar_controller` | `app/lidar_controller.py` | Lidar obstacle avoidance, following, and guarding. |
| `line_following` | `app/line_following.py` | Camera color pick and line following. |
| `object_tracking` | `app/object_tracking.py` | Camera color pick and object tracking. |
| `hand_trajectory` | `app/hand_trajectory_node.py` | Hand trajectory extraction from camera images. |
| `hand_gesture` | `app/hand_gesture.py` | Gesture command conversion into robot motion. |
| `ar_app` | `app/ar_app.py` | Present in setup but not launched by default because `ar_app_node_launch` is commented out in `start_app.launch.py`. |

Common app service pattern:

| Service | Meaning |
|---|---|
| `~/enter` | Subscribe to required camera/lidar topics and enter the game mode. |
| `~/exit` | Stop the game mode and unsubscribe/stop motion where implemented. |
| `~/set_running` | Enable or select the active behavior. |
| `~/init_finish` | Report node initialization status. |
| `*/heartbeat` | Heartbeat helper exits the mode if the controlling app stops sending heartbeats. |

Feature-specific behavior:

| Feature | Inputs | Outputs | Manual Debug Command |
|---|---|---|---|
| Lidar | `/scan_raw` | `/controller/cmd_vel`, steering servo topic for Ackermann | `ros2 launch app lidar_node.launch.py debug:=true` |
| Line following | `ascamera/camera_publisher/rgb0/image`, `/scan_raw` | `/controller/cmd_vel`, `~/image_result` | `ros2 launch app line_following_node.launch.py debug:=true` |
| Object tracking | `/ascamera/camera_publisher/rgb0/image` | `/controller/cmd_vel`, `~/image_result` | `ros2 launch app object_tracking_node.launch.py debug:=true` |
| Hand gesture | `/ascamera/camera_publisher/rgb0/image` through `hand_trajectory` | `/hand_trajectory/points`, `/controller/cmd_vel`, motor/servo stop commands | `ros2 launch app hand_gesture_node.launch.py debug:=true` |

Example service calls preserved in the raw `command` file:

```bash
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 1}"
ros2 service call /line_following/enter std_srvs/srv/Trigger {}
ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: True}"
ros2 service call /object_tracking/enter std_srvs/srv/Trigger {}
ros2 service call /object_tracking/set_running std_srvs/srv/SetBool "{data: True}"
ros2 service call /hand_gesture/enter std_srvs/srv/Trigger {}
ros2 service call /hand_gesture/set_running std_srvs/srv/SetBool "{data: True}"
```

## Wireless Controller Implementation

`peripherals/launch/joystick_control.launch.py` starts:

```text
joy/joy_node
peripherals/joystick_control
```

Important launch defaults:

| Setting | Value |
|---|---|
| Joystick device | `/dev/input/js0` |
| Autorepeat rate | `20.0` |
| `max_linear` | `0.5` from launch, node default is `0.7` |
| `max_angular` | `2.0` from launch, node default is `3.0` |
| Command topic | `controller/cmd_vel` |
| `disable_servo_control` | `True` in launch parameters |

`joystick_control.py` maps controller input as follows:

| Input | Mecanum M1 Output |
|---|---|
| Left stick X (`lx`) | `Twist.linear.y` lateral motion |
| Left stick Y (`ly`) | `Twist.linear.x` forward/backward motion |
| Right stick X (`rx`) | `Twist.angular.z` rotation |
| START button | Publish `BuzzerState` to `ros_robot_controller/set_buzzer` |

The runtime path is:

```text
USB handle receiver
  -> joy_node
  -> /joy
  -> peripherals.joystick_control
  -> controller/cmd_vel
  -> controller.odom_publisher
  -> ros_robot_controller/set_motor
  -> ros_robot_controller_node
  -> RRC Lite / STM32 motor driver
```

For buzzer feedback:

```text
/joy START
  -> joystick_control.py start_callback
  -> ros_robot_controller/set_buzzer
  -> ros_robot_controller_node.set_buzzer_state
  -> board.set_buzzer(...)
```

## First Hardware Bringup Checklist

Use this order when implementing or debugging on the physical MentorPi M1:

1. Charge the battery fully before first use and confirm the charger indicator
   reaches full state.
2. Confirm the USB handle receiver, lidar, camera, RRC data cable, and Raspberry
   Pi power cable are seated.
3. Power on the expansion board and wait at least one minute.
4. Confirm LED1 steady-on, LED2 AP/LAN status, lidar rotation, and a startup
   beep.
5. Connect by WonderPi app or VNC/terminal.
6. Run `source ~/.zshrc` and verify `.typerc` values.
7. Confirm ROS graph basics:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joy
ros2 topic echo /controller/cmd_vel
ros2 topic echo /ros_robot_controller/imu_raw
```

8. For app debugging, stop the auto-start path first if it conflicts with a
   manual launch:

```bash
~/.stop_ros.sh
```

9. Start one feature manually with `debug:=true` so its required controller,
   camera, or lidar dependencies are included by the feature launch file.

## Caveats

- The WonderPi mobile app itself and the Wi-Fi/AP configuration scripts are not
  part of the preserved `ros2_ws` raw tree. The official guide remains the
  source of truth for app installation and AP/LAN connection steps.
- The Getting Ready guide recommends completing VNC setup and robot version
  configuration before app control. In code terms, that means `.typerc` must
  match the physical chassis, camera, and lidar before launching app or
  controller nodes.
- The raw workspace defaults `LIDAR_TYPE=LD19`, while project notes for MentorPi
  M1 hardware mention Oradar MS200. Confirm the installed lidar and update
  `.typerc` / launch configuration before relying on lidar app behavior.
- `start_app.launch.py` starts four app feature nodes by default. If a manual
  experiment starts the same nodes again, stop the auto-start service first to
  avoid duplicate publishers or service conflicts.

