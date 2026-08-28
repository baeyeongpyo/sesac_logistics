---
title: MentorPi M1 User-Provided Product, Sensor, and Vehicle Spec Screenshots
created: 2026-07-08
updated: 2026-07-08
type: raw
status: active
tags:
  - robotics
  - mentorpi
  - hardware
  - sensor-spec
  - vehicle-spec
sources:
  - title: User-provided screenshots in Codex conversation
    accessed: 2026-07-08
---

# MentorPi M1 User-Provided Product, Sensor, and Vehicle Spec Screenshots

This page is a text transcription of the product, sensor, controller, motor,
battery, and vehicle specification screenshots provided by the user on
2026-07-08. The original image paths were temporary clipboard files in the
local session and are not treated as durable repository artifacts.

## Vehicle Product Parameters

| Item | Specification |
|---|---|
| Size | 212*171*147mm (Depth Camera Version) |
| Weight | 1.2kg |
| Chassis type | Mecanum wheel chassis |
| Motor | 310 metal gear geared motor |
| Encoder | AB-phase high-accuracy quadrature encoder |
| Material | Full metal aluminum alloy chassis, anodizing process |
| ROS controller | RRC Lite controller + Raspberry Pi 5 controller |
| Control method | App, wireless handle and PC control |
| Camera | Angstrong binocular 3D depth camera |
| Lidar | Oradar MS200 |
| Battery | 7.4V 2200mAh 10C LiPo battery |
| OS | Raspberry Pi OS + Ubuntu 22.04 LTS + ROS2 Humble (Docker) |
| Software | iOS/ Android app |
| Communication method | WiFi/ Ethernet |
| Programming language | Python/ C/ C++/ JavaScript |
| Storage | 64GB TF card |

## Vehicle Dimensional Diagram

| Dimension | Value |
|---|---|
| Overall length | 212mm |
| Overall width | 171mm |
| Overall height with depth camera | 147mm |
| Chassis/body height shown in side diagram | 112mm |
| Wheel diameter | 65mm |
| Wheel width | 30mm |

## Oradar MS200 Lidar

The screenshot describes the Oradar MS200 Lidar as a Time-of-Flight (TOF)
lidar for precise distance measurement, with range up to 12 meters and a
scanning rate of 4,500 samples per second. It is described as ROS-friendly and
suited to indoor mapping, navigation, and obstacle avoidance.

| Item | Specification |
|---|---|
| Lidar model | Oradar MS200 |
| Ranging principle | TOF ranging |
| Recommended scenarios | Indoor and outdoor |
| Supply voltage | 5V |
| Scanning range | 360 degrees |
| Ranging radius | Black object: 12m |
| Communication rate | 230400bps |
| Sampling frequency | 4500Hz |
| Scanning frequency | 7-15Hz, 10Hz by default |
| Angular resolution | 0.4 degrees @5Hz, 0.8 degrees @10Hz |
| Supply current | 260mA |
| Output port | Standard asynchronous serial port (UART) |
| Work temperature | -10 to 50 degrees C |
| Ranging accuracy | +/-10mm [0.1m-2m], +/-20mm [0.1m-12m] |
| Size | 37.7*37.5*32.5mm |

## Depth Camera Parameters

| Item | Specification |
|---|---|
| Camera model | Nuwa-HP60C |
| Power supply method | USB |
| Operating range | 0.2-4m |
| Accuracy | < 2mm@1000mm |
| Depth FOV | H73.8 degrees x V58.8 degrees x D86.4 degrees |
| Color FOV | H80.9 degrees x V51.7 degrees x D88.9 degrees |
| VBUS | 4.75-5.25V |
| Supported OS | Windows, Android, Linux |
| Resolution@frame rate (Depth mode) | 640x480@20fps(Max) |
| Resolution@frame rate (RGB mode) | 1920x1080@20fps(Max) |
| Operating environment | Indoor |
| Data port | Type C USB 2.0 |
| Size | 89.9x19.0x25.0mm |
| Power consumption | < 2W |

## RRC Lite Controller

### Labeled Components

- USB serial port
- Power indicator
- GPIO expansion
- Buzzer
- PWM servo port
- I2C expansion port
- STM32F407VET6 main control chip
- RGB light
- Serial bus servo port
- 4-channel encoder motor port
- IMU pose sensor
- 5V 5A external power supply
- Power supply port
- Power switch
- User indicator
- Reset button
- User button

### Parameters

| Item | Specification |
|---|---|
| Main control chip | STM32F407VET6(100PIN) |
| Motor drive chip | SA8870C (Overcurrent protection) |
| IMU sensor | 3-axis acceleration and 3-axis gravity acceleration |
| Encoder motor port | 4-Channel (Independent drive) |
| Serial servo port | 2-Channel (6-12V) |
| PWM servo port | 4-Channel (5-8.4V) |
| Responsive component | Buzzer * 1; LED light * 3; RGB light * 2 |
| Power supply | 6-14V wide voltage input |
| External power supply port | 5V 5A |
| Download port | Serial port one-click download |
| Circuit protection | Overheat, short circuit and overcurrent protection |
| Board layer | Industrial-grade dual layers |
| Size | 85*56*17mm |
| Mounting pitch | 57.5*48.5mm |
| Weight | 32g |

## Encoder Geared Motor and Wheel

| Feature | Description |
|---|---|
| Wrapped Rear Tail Shell | Protects the PCB circuit and magnetic ring at the end of the motor from external influences, improving safety and service life. |
| Permanent Magnet Brushed Motor | Permanent magnet DC motor with fast starting response speed, large starting torque, and smooth speed change. |
| High-precision Magnetic Encoder | Motor is equipped with a high-precision magnetic encoder, strong horsepower, high precision, and strong anti-interference ability. |
| Durable Metal Gear | Full metal gear and metal output shaft reduce power consumption and extend motor service life. |

## Hall Encoder Geared Motor

| Item | Specification |
|---|---|
| Motor rated voltage | 7.4V |
| Rated current | <=0.65A |
| Stall torque | >=1.0kg.cm |
| Gear ratio | 1:20 |
| Rated torque | 0.4kg*cm |
| Encoder type | AB phase incremental Hall encoder |
| Rotation speed after reduction | 450+/-10rpm |
| Encoder power supply voltage | 3.3-5V |
| Stall current | <1.4A |
| Microcontroller weight | About 70g |

## 7.4V 2200mAh 10C LiPo Battery

The screenshot describes the battery as using high-quality 18650 cells with a
built-in protection board to prevent damage from overcharging, overcurrent,
over-discharging, and short circuits. It lists a service life of over 300
charge cycles.

| Item | Specification |
|---|---|
| Model | 7.4V 2200mAh LiPo battery |
| Plug | DC5.5*2.5 female / SM-2P male |
| Size | 69x37x19mm |
| Charger | 8.4V charger |
