---
title: Hiwonder MentorPi M1 Product Spec and Manual Notes
created: 2026-07-08
updated: 2026-07-09
type: source
status: active
tags:
  - robotics
  - mentorpi
  - raspberry-pi
  - ros2
  - hardware
sources:
  - title: Hiwonder MentorPi M1 product page
    url: https://www.hiwonder.com/products/mentorpi-m1?variant=41285495685207
    accessed: 2026-07-08
  - title: MentorPi v2.0 documentation
    url: https://docs.hiwonder.com/projects/MentorPi/en/latest/
    accessed: 2026-07-08
  - title: MentorPi Master Slave And Group Control tutorial
    url: https://wiki.hiwonder.com/projects/MentorPi/en/latest/docs/13.MasterSlaveAndGroupControl.html
    accessed: 2026-07-08
  - title: MentorPi Getting Ready documentation
    url: https://wiki.hiwonder.com/projects/MentorPi/en/latest/docs/1.getting_ready.html
    accessed: 2026-07-09
  - title: User-provided MentorPi M1 spec screenshots
    path: llm-wiki/raw/mentorpi-m1-user-provided-spec-screenshots-2026-07-08.md
    accessed: 2026-07-08
---

# Hiwonder MentorPi M1 Product Spec and Manual Notes

## Summary

Hiwonder MentorPi M1 is a Raspberry Pi based ROS2 robot car that uses a Mecanum-wheel chassis. It is intended for AI robot development and education, including motion control, SLAM mapping, navigation, vision processing, autonomous driving, group control, and large AI model based interaction.

The official MentorPi documentation states that users should choose the `Latest` course version for MentorPi M1.

## Product Identity

| Item | Value |
|---|---|
| Product | Hiwonder MentorPi M1 Raspberry Pi Robot Car |
| Chassis | Mecanum-wheel chassis |
| Main controller | Raspberry Pi 5 controller plus RRC Lite controller |
| Robot software stack | Raspberry Pi OS, Ubuntu 22.04 LTS, ROS2 Humble in Docker |
| Main control options | App, wireless controller, PC control |
| Connectivity | Wi-Fi and Ethernet |
| Programming languages listed | Python, C, C++, JavaScript |
| Storage listed | 64GB TF card |

## Available Product Options

The product page lists three M1 kit levels and several Raspberry Pi 5 controller options.

| Option Group | Choices |
|---|---|
| Kit level | MentorPi M1 Starter, MentorPi M1 Standard, MentorPi M1 Advanced |
| Raspberry Pi 5 bundle | Without Raspberry Pi 5, With Raspberry Pi 5 2GB, 4GB, 8GB, or 16GB |

Kit differences from the official packing list:

| Kit | Camera / Interaction Notes |
|---|---|
| Starter | Monocular camera version |
| Standard | Depth camera version |
| Advanced | Depth camera version plus WonderEcho Pro AI voice interaction box and Type-C cable |

## Main Hardware Specs

| Item | Specification |
|---|---|
| Size | 212 x 171 x 147 mm for depth camera version |
| Weight | 1.2 kg |
| Chassis type | Mecanum wheel chassis |
| Motor | 310 metal gear geared motor |
| Encoder | AB-phase high-accuracy quadrature encoder |
| Chassis material | Full metal aluminum alloy chassis, anodizing process |
| ROS controller | RRC Lite controller plus Raspberry Pi 5 controller |
| Camera | Angstrong binocular 3D depth camera on depth-camera kit |
| Lidar | Oradar MS200 |
| Battery | 7.4V 2200mAh 10C LiPo battery |
| Charger in packing list | 8.4V 2A charger, DC5.5 x 2.5 male connector |
| Package size | 41 x 22 x 18 cm |
| Package weight | About 2.1 kg |

## Vehicle Dimensions

User-provided screenshots for the depth-camera version list these physical
dimensions.

| Item | Specification |
|---|---|
| Overall size | 212 x 171 x 147 mm |
| Overall length | 212 mm |
| Overall width | 171 mm |
| Overall height with depth camera | 147 mm |
| Chassis/body height shown in side diagram | 112 mm |
| Wheel diameter | 65 mm |
| Wheel width | 30 mm |

## Camera and Sensor Specs

### Depth Camera

The product page lists the depth camera model as `Nuwa-HP60C` in the camera parameter section.

| Item | Specification |
|---|---|
| Power | USB |
| Operating range | 0.2-4 m |
| Accuracy | < 2 mm at 1000 mm |
| Depth FOV | H73.8 x V58.8 x D86.4 degrees |
| Color FOV | H80.9 x V51.7 x D88.9 degrees |
| VBUS | 4.75-5.25 V |
| Supported OS | Windows, Android, Linux |
| Depth resolution / frame rate | 640 x 480 at up to 20 fps |
| RGB resolution / frame rate | 1920 x 1080 at up to 20 fps |
| Operating environment | Indoor |
| Data port | Type-C USB 2.0 |
| Size | 89.9 x 19.0 x 25.0 mm |
| Power consumption | < 2 W |

### Oradar MS200 Lidar

The user-provided product screenshot describes the Oradar MS200 as a
Time-of-Flight lidar for distance measurement up to 12 m, with 4,500 samples
per second, ROS integration support, and indoor mapping, navigation, and
obstacle avoidance use cases.

| Item | Specification |
|---|---|
| Lidar model | Oradar MS200 |
| Ranging principle | TOF ranging |
| Recommended scenarios | Indoor and outdoor |
| Supply voltage | 5 V |
| Scanning range | 360 degrees |
| Ranging radius | Black object: 12 m |
| Communication rate | 230400 bps |
| Sampling frequency | 4500 Hz |
| Scanning frequency | 7-15 Hz, 10 Hz by default |
| Angular resolution | 0.4 degrees @5 Hz, 0.8 degrees @10 Hz |
| Supply current | 260 mA |
| Output port | Standard asynchronous serial port (UART) |
| Work temperature | -10 to 50 degrees C |
| Ranging accuracy | +/-10 mm [0.1 m-2 m], +/-20 mm [0.1 m-12 m] |
| Size | 37.7 x 37.5 x 32.5 mm |

### Monocular Camera

| Item | Specification |
|---|---|
| Resolution | 30W, 640 x 480 |
| Light-sensitive chip | GC0308 |
| Camera type | HS-256-650 |
| Aperture | 2.0 |
| Focus | 1.7 mm |
| Field of view | 170 degrees |
| Distortion | -83% |
| Interface | USB |
| Size | 30 x 27 x 25 mm |

### LFD-01 Servo

| Item | Specification |
|---|---|
| Rotation range | 0-180 degrees |
| Communication | PWM pulse-width control |
| Working voltage | 4.8-6 V |
| Maximum torque | >= 1.4 kg.cm at 6 V |
| No-load speed | <= 0.11 sec / 60 degrees at 6 V |
| Stall protection | Power cut protection after 5 seconds of stall |
| Size | 22.3 x 12.0 x 23.2 mm |

## Controller, Motor, and Battery Specs

### RRC Lite Controller

Labeled board components in the user-provided screenshot include USB serial
port, power indicator, GPIO expansion, buzzer, PWM servo port, I2C expansion
port, STM32F407VET6 main control chip, RGB light, serial bus servo port,
4-channel encoder motor port, IMU pose sensor, 5V 5A external power supply,
power supply port, power switch, user indicator, reset button, and user button.

| Item | Specification |
|---|---|
| Main control chip | STM32F407VET6(100PIN) |
| Motor drive chip | SA8870C with overcurrent protection |
| IMU sensor | 3-axis acceleration and 3-axis gravity acceleration |
| Encoder motor port | 4-channel independent drive |
| Serial servo port | 2-channel, 6-12 V |
| PWM servo port | 4-channel, 5-8.4 V |
| Responsive components | Buzzer * 1; LED light * 3; RGB light * 2 |
| Power supply | 6-14 V wide voltage input |
| External power supply port | 5 V 5 A |
| Download port | Serial port one-click download |
| Circuit protection | Overheat, short circuit, and overcurrent protection |
| Board layer | Industrial-grade dual layers |
| Size | 85 x 56 x 17 mm |
| Mounting pitch | 57.5 x 48.5 mm |
| Weight | 32 g |

### Hall Encoder Geared Motor

| Item | Specification |
|---|---|
| Motor rated voltage | 7.4 V |
| Rated current | <=0.65 A |
| Stall torque | >=1.0 kg.cm |
| Gear ratio | 1:20 |
| Rated torque | 0.4 kg*cm |
| Encoder type | AB phase incremental Hall encoder |
| Rotation speed after reduction | 450+/-10 rpm |
| Encoder power supply voltage | 3.3-5 V |
| Stall current | <1.4 A |
| Weight shown in screenshot | About 70 g |

Motor feature notes from the screenshot:

- Wrapped rear tail shell protects the PCB circuit and magnetic ring at the end
  of the motor.
- Permanent magnet brushed motor provides fast starting response, large
  starting torque, and smooth speed change.
- High-precision magnetic encoder is described as high precision and strongly
  anti-interference.
- Full metal gear and metal output shaft are described as reducing power
  consumption and extending motor service life.

### 7.4V 2200mAh 10C LiPo Battery

| Item | Specification |
|---|---|
| Model | 7.4V 2200mAh LiPo battery |
| Plug | DC5.5*2.5 female / SM-2P male |
| Size | 69 x 37 x 19 mm |
| Charger | 8.4 V charger |
| Protection notes | Built-in protection board for overcharging, overcurrent, over-discharging, and short circuits |
| Listed service life | Over 300 charge cycles |

## Supported Functions

- Motion control for Mecanum-wheel chassis.
- SLAM mapping and path planning with lidar.
- Dynamic obstacle avoidance.
- 3D visual mapping and navigation with depth camera on supported kits.
- OpenCV vision functions such as color recognition, color tracking, QR code recognition, and line following.
- MediaPipe based human-robot interaction lessons.
- Machine learning lessons, including YOLOv5 model training in the official manual.
- Product page advertises YOLOv11 based road sign and traffic light recognition for autonomous driving.
- Autonomous driving lessons: lane keeping, road sign detection, traffic light recognition, turning decision making, autonomous parking, and integrated application.
- Master-slave and group control.
- Large AI model course for voice module, multimodal large model applications, and embodied AI applications.
- App control through WonderPi on iOS and Android.
- Wireless controller control over Bluetooth.

## Manual Structure

Official manual root: <https://docs.hiwonder.com/projects/MentorPi/en/latest/>

| Manual Section | Project-Relevant Content |
|---|---|
| 1. Getting Ready | Product introduction, packing list, charging and usage guide, assembly and wiring, startup status, app control, wireless controller control |
| 2. Set Development Environment | VNC connection, robot version configuration, system overview, Docker usage, servo deviation adjustment |
| 3. Motion Control Lesson | Mecanum chassis motion analysis, Ackermann reference, IMU / linear / angular velocity calibration, odometer data, speed control |
| 4. Lidar Lesson | Lidar principle, obstacle avoidance, lidar following, lidar guarding |
| 5. Depth Camera Basic Lesson | ROS2 test/configuration, depth camera configuration, ROS SDK installation, point cloud, web monitoring |
| 6. Mapping Lesson | WinSCP, URDF model, robot model, SLAM principle, slam_toolbox mapping, RTAB-VSLAM 3D mapping |
| 7. Navigation Lesson | Autonomous navigation, AMCL, DWA path planning, point-to-point and multi-point navigation, RTAB-VSLAM 3D navigation |
| 8. ROS+OpenCV Lesson | Depth camera installation, color threshold adjustment, color recognition, QR code generation/recognition, line following, color tracking, chassis tracking |
| 9. MediaPipe Human-robot Interaction | MediaPipe introduction, fingertip trajectory recognition, hand following, posture control, pose detection |
| 10. Machine Learning | Machine learning introduction, library introduction, YOLOv5 introduction and model training |
| 11. Autonomous Driving Lesson | Map and prop setup, lane keeping, road sign detection, traffic light recognition, turning decisions, autonomous parking |
| 12. Master Slave And Group Control | Master-slave setup and group control |
| 13. Robot Network Configuration | AP direct connection mode and LAN mode |
| 14. Large AI Model Course | Voice module overview, large model basics, multimodal large model applications, embodied AI applications |
| Appendix / Download | Additional reference and downloadable materials via Google Drive |

## Initial Setup Notes

- Default network mode is AP direct connection.
- Robot hotspot prefix is `HW`.
- Default Wi-Fi / AP password is `hiwonder`.
- Default VNC IP in AP mode is `192.168.149.1`.
- VNC login uses account `pi` when requested and password `raspberrypi`.
- Startup takes about 1 minute. LED1 steady blue and a buzzer beep indicate ROS configuration completed; LED2 blinking once per second indicates the hotspot is broadcasting.
- The product should be fully charged before first use. The official guide says initial charging takes about 1 hour.
- Charge the LiPo battery with the provided charger when voltage drops below 6.4 V.
- The robot version configuration tool must match the purchased hardware:
  - `MentorPi_Mecanum` for MentorPi M1.
  - `MentorPi_Acker` for MentorPi A1.
  - `ascamera` for 3D depth camera.
  - `usb_cam` for 2D monocular camera.
- Function code runs inside the Docker container. The manual identifies `ros2_ws/src/` as the main location for function packages and source code.

## Source Code / Workspace Map

The manual identifies these directories after entering the Docker container:

| Directory | Meaning |
|---|---|
| `ros2_ws` | ROS workspace for MentorPi functions |
| `share` | Shared directory between Raspberry Pi and Docker container |
| `softwave` | PC-side software and color threshold adjustment tools |
| `ros2_ws/src/app` | App game function package |
| `ros2_ws/src/example` | Game cases |
| `ros2_ws/src/bringup` | App function references |
| `ros2_ws/src/driver` | Underlying control |
| `ros2_ws/src/interfaces` | Program interfaces |
| `ros2_ws/src/peripherals` | Hardware drivers |
| `ros2_ws/src/navigation` | Navigation |
| `ros2_ws/src/slam` | Mapping |
| `ros2_ws/src/yolov5_ros2` | YOLOv5 game |
| `ros2_ws/src/simulations` | URDF description |
| `ros2_ws/src/multi` | Multi-robot master-slave / group control package from the group control lesson |

Detailed mapping between the official group-control tutorial and the actual
tracked ROS2 workspace is captured in
`llm-wiki/sources/hiwonder-mentorpi-group-control-code-map.md`.

Detailed mapping between the official Getting Ready guide and the actual
tracked ROS2 workspace startup, app-control, and wireless-controller
implementation is captured in
`llm-wiki/sources/hiwonder-mentorpi-getting-ready-implementation-guide.md`.

## Master-Slave and Group Control Tutorial Notes

The official group control lesson uses at least two MentorPi vehicles. One unit
is designated as the master and creates the Wi-Fi network; the other units are
configured as slaves on the same network. ROS namespaces / host variables
separate each robot, and a broadcast control program sends velocity commands so
multiple vehicles can be driven together from the master.

### Required Setup

- Prepare at least two MentorPi vehicles.
- Complete the VNC setup from the development environment lesson.
- Connect the USB game controller receiver to the master device.
- Before configuring multi-robot control, stop the default auto-started ROS
  gameplay on the vehicles with `~/.stop_ros.sh`.

### Network Configuration

| Device | Configuration |
|---|---|
| Master | Edit `hiwonder-toolbox/wifi_conf.py`, set the hotspot password as needed, reboot with `sudo reboot`, then connect clients to the master's Wi-Fi. The default password is `hiwonder` unless changed. |
| Slave | Edit `hiwonder-toolbox/wifi_conf.py`, set network mode to `2` for LAN mode, and set the Wi-Fi SSID/password to match the master's hotspot, for example `HW-123` / `hiwonder`. Reboot with `sudo reboot`; this reboot is required. |

Slave network behavior:

- In LAN mode, `LED2` flashes rapidly while the robot searches for the
  predefined network.
- If the target network is not found after three searches, the robot falls back
  to direct connection mode and `LED2` flashes slowly.
- This fallback does not rewrite `wifi_conf.py`; on the next boot the robot
  still attempts the LAN configuration first unless the file is manually
  changed.

### Environment Variables

| Device | Environment Variable Setup |
|---|---|
| Master | Edit `/home/ubuntu/ros2_ws/.typerc` and set the master namespace / host setting to `robot_1/`, then run `source ~/.zshrc`. |
| Slave | Use the mobile app on the master's Wi-Fi to find the slave IP, connect with VNC, edit `/home/ubuntu/ros2_ws/.typerc`, set `HOST` to `robot_2`, then run `source ~/.zshrc`. |

### Time Synchronization

Before running group control, check the master time with `date` and set the
slave time with a command like:

```bash
sudo date -s "2024-10-29 22:46:31"
```

If joystick control shows visible delay or desynchronization between vehicles,
repeat time synchronization. The tutorial recommends adding the elapsed time
between reading the master clock and applying it on the slave, then verifying
both terminals with `date`.

### Program Execution

The lesson links a `multi.zip` source package for the group-control code.

On the master device, open three terminals and run:

```bash
ros2 launch multi multi_controller.launch.py
python3 ros2_ws/src/multi/launch/joystick_control_multi.py
ros2 run joy joy_node
```

On each slave device, run:

```bash
ros2 launch multi multi_controller.launch.py
```

For later group-control runs, stop the auto-start services on both master and
slave devices first, then synchronize system time before launching the ROS
nodes.

### Controller Behavior

The controller pairs automatically after being turned on. During pairing, the
red and green LEDs blink together. After pairing, the green LED remains on and
the red LED turns off. Pressing `START` after connection should trigger buzzer
feedback on both vehicles.

| Controller Input | Function |
|---|---|
| `START` | Stop and reset the robot |
| Left joystick up/down | Move forward / backward |
| Left joystick left/right | Mecanum chassis lateral left / right movement |
| Right joystick left/right | Ackermann front-wheel steering left / right |

The lesson notes that gentle joystick movement results in slow-speed movement.

## Project Implications

- Treat MentorPi M1 as the baseline hardware for this project.
- Prefer Mecanum movement assumptions in motion control work.
- Confirm the purchased kit level before using depth-camera, monocular-camera, or WonderEcho Pro features.
- For Standard or Advanced kits, use the depth-camera path and `ascamera`; for Starter, use the monocular-camera path and `usb_cam`.
- Development and debugging should expect ROS2 Humble running in Docker on Raspberry Pi OS / Ubuntu 22.04.
- For autonomous driving and perception work, separate product-page claims from manual examples: the product page mentions YOLOv11 for road signs and traffic lights, while the manual's machine learning section documents YOLOv5 lessons.
- For multi-robot work, plan for deterministic network setup, unique ROS
  namespaces / host values per robot, explicit auto-start shutdown, and manual
  time synchronization before joystick or broadcast control tests.

## Open Questions

- Which exact kit will be used: Starter, Standard, or Advanced?
- Will the project use a bundled Raspberry Pi 5 or an existing board?
- Will the project target app/manual operation, ROS2 package development, autonomous driving, SLAM/navigation, or large-model interaction first?
- If using source code from Hiwonder, confirm whether access is delivered after purchase or through the official download link.
