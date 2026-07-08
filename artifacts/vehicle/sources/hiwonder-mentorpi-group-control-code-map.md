---
title: Hiwonder MentorPi Master-Slave Group Control Code Map
created: 2026-07-08
updated: 2026-07-09
type: source
status: active
tags:
  - robotics
  - mentorpi
  - ros2
  - multi-robot
  - group-control
sources:
  - title: MentorPi Master Slave And Group Control tutorial
    url: https://wiki.hiwonder.com/projects/MentorPi/en/latest/docs/13.MasterSlaveAndGroupControl.html
    accessed: 2026-07-08
  - title: Preserved tracking vehicle ROS2 workspace raw copy
    path: llm-wiki/raw/mentorpi-ros2-ws-group-control-2026-07-08
    accessed: 2026-07-08
  - title: Preserved tracking vehicle ROS2 workspace SHA-256 manifest
    path: llm-wiki/raw/mentorpi-ros2-ws-group-control-2026-07-08.sha256
    accessed: 2026-07-08
  - title: Current tracking vehicle ROS2 workspace verification
    path: llm-wiki/raw/mentorpi-ros2-ws-current-verification-2026-07-09.md
    accessed: 2026-07-09
---

# Hiwonder MentorPi Master-Slave Group Control Code Map

## Summary

The Master-Slave and Group Control tutorial maps to the `multi` package under
the preserved MentorPi ROS2 workspace raw copy:

```text
llm-wiki/raw/mentorpi-ros2-ws-group-control-2026-07-08/src/multi
```

This raw copy was copied from:

```text
/Users/yeongpyo/project/product/wiki-binding2/artifacts/tracking-vehicle/raw/ros2_ws
```

The copied tree was verified against the source using a relative-path SHA-256
manifest stored at:

```text
llm-wiki/raw/mentorpi-ros2-ws-group-control-2026-07-08.sha256
```

The source path was rechecked on 2026-07-09. The current external workspace has
one additional `.DS_Store` file, but all 430 non-`.DS_Store` source files match
the preserved raw copy by relative-path SHA-256 hash. The current-source
manifest is preserved at:

```text
llm-wiki/raw/mentorpi-ros2-ws-current-source-2026-07-09.sha256
```

The tutorial's simple joystick group-control flow is implemented by:

1. Per-robot controller startup through `multi/launch/multi_controller.launch.py`.
2. A direct Python group joystick broadcaster at
   `multi/launch/joystick_control_multi.py`.
3. A global joystick input node from the ROS `joy` package.
4. The lower-level `controller` and `ros_robot_controller` packages that turn
   `Twist` messages into motor, buzzer, and servo commands.

The same `multi` package also contains TF / formation-following nodes
(`formation_update`, `slave_tf_listener`, `tf_listen`, `tf_follower`,
`costmap_publish`, `set_odom`), but those nodes are not launched by the
tutorial's listed joystick-control commands.

## Key Code Paths

| Purpose | Code Path | Notes |
|---|---|---|
| Group-control package | `ros2_ws/src/multi` | Tutorial package for master/slave and group control. |
| Main controller launch | `src/multi/launch/multi_controller.launch.py` | Reads `MASTER` and `HOST`, pushes the robot namespace, includes `controller.launch.py`, and conditionally includes `joy_control.launch.py` for the master. |
| Group joystick broadcaster | `src/multi/launch/joystick_control_multi.py` | Direct Python script used by the tutorial; subscribes to `/joy` and publishes commands to `robot_1/`, `robot_2/`, and `robot_3/` topics. |
| Optional master joystick launch | `src/multi/launch/joy_control.launch.py` | Includes `peripherals/launch/joystick_control.launch.py` inside the master namespace. This is separate from the tutorial's direct `joystick_control_multi.py` step. |
| Single-robot joystick launch | `src/peripherals/launch/joystick_control.launch.py` | Starts `joy_node` and `peripherals.joystick_control`, with command velocity remapping. |
| Single-robot joystick node | `src/peripherals/peripherals/joystick_control.py` | Converts `/joy` to one robot's `controller/cmd_vel`, buzzer, and Ackermann servo commands. |
| Chassis controller launch | `src/driver/controller/launch/controller.launch.py` | Starts IMU filter, odometry publisher, and EKF. |
| Odometry / chassis node | `src/driver/controller/controller/odom_publisher_node.py` | Subscribes to `controller/cmd_vel`, converts Mecanum or Ackermann commands to motor and servo outputs, and publishes odometry. |
| Mecanum kinematics | `src/driver/controller/controller/mecanum.py` | Converts `linear.x`, `linear.y`, and `angular.z` into four motor RPS commands. |
| Ackermann kinematics | `src/driver/controller/controller/ackermann.py` | Converts speed and angular velocity into drive motor RPS plus front servo angle. |
| Hardware controller launch | `src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py` | Starts the STM32/RRC Lite bridge node. |
| Hardware controller node | `src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py` | Subscribes to motor, buzzer, LED, servo, and other hardware-control topics. |
| Environment defaults | `ros2_ws/.typerc` | Defines `MACHINE_TYPE`, `HOST`, `MASTER`, `ROS_DOMAIN_ID`, and CycloneDDS config. |
| Shell source chain | `ros2_ws/.zshrc`, `ros2_ws/.robotrc` | `.zshrc` sources `.robotrc`; `.robotrc` sources `.typerc`. |
| Auto-start reference | `src/bringup/scripts/start_app_node.service` | Starts `bringup.launch.py` as a systemd service; group control should stop this auto-start path before manual launch. |

## Tutorial Step to Code Mapping

| Tutorial Step | Code Mapping | Runtime Effect |
|---|---|---|
| Prepare at least two MentorPi vehicles | `joystick_control_multi.py` is hardcoded for `num_robots = 3`. | The script publishes to `robot_1/`, `robot_2/`, and `robot_3/` topics even if only two vehicles are present. For a different fleet size, update `num_robots`. |
| Put master and slave on the same network | Not implemented inside `ros2_ws`; the tutorial uses `hiwonder-toolbox/wifi_conf.py`, which is outside this raw `ros2_ws` artifact. | ROS2 discovery and topic traffic require the robots to be reachable on the same network. `.typerc` sets `ROS_DOMAIN_ID=0` and `CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml`. |
| Edit master/slave `.typerc` and run `source ~/.zshrc` | `.zshrc` sources `.robotrc`; `.robotrc` sources `.typerc`; `multi_controller.launch.py` reads `EnvironmentVariable('MASTER')` and `EnvironmentVariable('HOST')`. | `MASTER` selects the master namespace; `HOST` selects the current robot namespace. `source ~/.zshrc` reloads those values into the terminal before running launch commands. |
| Stop auto-started ROS gameplay with `~/.stop_ros.sh` | Raw command notes list `~/.stop_ros.sh`; `start_app_node.service` auto-runs `ros2 launch bringup bringup.launch.py`. | Prevents the normal bringup/game service from competing with manual group-control nodes for the same controller and hardware topics. |
| Synchronize time with `date` / `sudo date -s ...` | No explicit time-sync code in the tutorial launch commands. Code that uses ROS time includes odometry stamping, TF broadcasting, and TF lookup. | Time skew can break or delay TF and multi-robot coordination. The tutorial's manual time sync supports consistent ROS message and transform timing. |
| Master terminal 1: `ros2 launch multi multi_controller.launch.py` | Launches `multi_controller.launch.py` with current `MASTER` and `HOST`. | Starts this robot's namespaced `controller.launch.py`. If `MASTER == HOST`, it also includes `joy_control.launch.py`. |
| Slave terminal: `ros2 launch multi multi_controller.launch.py` | Same launch file, but `HOST` should be the slave namespace, for example `robot_2`. | Starts the slave robot's namespaced controller stack. Since `MASTER != HOST`, the master-only joystick include should not run. |
| Master terminal 2: `python3 ros2_ws/src/multi/launch/joystick_control_multi.py` | Runs `JoystickController('joystick_control', num_robots=3)` directly from source. | Subscribes to global `/joy`, converts joystick axes to `Twist`, and publishes the same movement command to each `robot_N/controller/cmd_vel`. |
| Master terminal 3: `ros2 run joy joy_node` | Runs the standard ROS `joy` package node. | Reads the USB game controller and publishes `sensor_msgs/Joy` on `/joy`, which `joystick_control_multi.py` consumes. |
| Press `START` on the controller | `joystick_control_multi.py` maps button name `start` to `start_callback`. | Publishes a short `BuzzerState` to every configured robot namespace, so all connected vehicles should beep. |
| Left joystick up/down/left/right | `joystick_control_multi.py` maps `ly` to `Twist.linear.x` and `lx` to `Twist.linear.y` when `MACHINE_TYPE=MentorPi_Mecanum`. | Mecanum vehicles move forward/backward and translate laterally. The per-robot controller converts the `Twist` into four motor RPS values. |
| Right joystick left/right | `joystick_control_multi.py` maps `rx` to `Twist.angular.z`. For Ackermann, it also computes PWM servo position. | Mecanum robots rotate; Ackermann robots steer the front servo and drive along a turn radius. |

## Runtime Topic Flow

The tutorial's intended group-control path is:

```text
USB game controller
  -> joy_node
  -> /joy
  -> joystick_control_multi.py
  -> robot_1/controller/cmd_vel
  -> robot_2/controller/cmd_vel
  -> robot_3/controller/cmd_vel
  -> each robot's namespaced controller node
  -> robot_N/ros_robot_controller/set_motor
  -> RRC Lite / STM32 motor output
```

For the `START` buzzer feedback:

```text
/joy start button
  -> joystick_control_multi.py start_callback
  -> robot_N/ros_robot_controller/set_buzzer
  -> ros_robot_controller_node.py
  -> board.set_buzzer(...)
```

For Ackermann front steering:

```text
/joy right-stick x axis
  -> joystick_control_multi.py
  -> robot_N/ros_robot_controller/pwm_servo/set_state
  -> ros_robot_controller_node.py
  -> board.pwm_servo_set_position(...)
```

## Launch and Namespace Details

`multi_controller.launch.py` declares:

- `master`, defaulting to environment variable `MASTER`.
- `robot_name`, defaulting to environment variable `HOST`.
- `cmd_vel_topic`, defaulting to `/controller/cmd_vel`.

It then pushes `robot_name` as a ROS namespace and includes:

```text
controller/launch/controller.launch.py
```

The controller launch brings up:

- `odom_publisher.launch.py`
- `peripherals/launch/imu_filter.launch.py`
- `robot_localization` EKF node

`odom_publisher.launch.py` includes:

- `mentorpi_description/launch/robot_description.launch.py`
- `ros_robot_controller/launch/ros_robot_controller.launch.py`
- `controller` package executable `odom_publisher`

Inside `odom_publisher_node.py`, the controller subscribes to:

```text
controller/cmd_vel
cmd_vel
set_odom
```

and publishes:

```text
ros_robot_controller/set_motor
ros_robot_controller/pwm_servo/set_state
odom_raw
set_pose
```

Because `multi_controller.launch.py` pushes `HOST` as a namespace, these
relative topic names become namespaced per robot.

## Environment Variable Mapping

`.typerc` defines the robot runtime identity:

| Variable | Code Usage | Tutorial Meaning |
|---|---|---|
| `MACHINE_TYPE` | Read by `joystick_control_multi.py`, `peripherals/joystick_control.py`, `odom_publisher_node.py`, and URDF xacro. | Selects Mecanum vs Ackermann motion behavior. For MentorPi M1, use `MentorPi_Mecanum`. |
| `HOST` | Used as `robot_name` default by `multi_controller.launch.py`. | The namespace for the current physical robot, for example `robot_1` or `robot_2`. |
| `MASTER` | Used by `multi_controller.launch.py` and `joy_control.launch.py`. | The namespace of the master robot. The tutorial uses the master as `robot_1`. |
| `ROS_DOMAIN_ID` | Standard ROS2 DDS domain selector. | All robots in the group must share the same domain. |
| `CYCLONEDDS_URI` | Points to `/etc/cyclonedds/config.xml`. | Controls CycloneDDS network behavior for ROS2 discovery and communication. |
| `need_compile` | Used by many launch files to decide package-share paths vs source paths. | The raw artifact defaults to source path mode with `False`. |

The source chain is:

```text
source ~/.zshrc
  -> source $HOME/ros2_ws/.robotrc
  -> source $HOME/ros2_ws/.typerc
```

## Code-Level Behavior

### `joystick_control_multi.py`

- Subscribes to absolute `/joy`.
- Creates publishers for each `robot_N/` namespace:
  - `robot_N/ros_robot_controller/pwm_servo/set_state`
  - `robot_N/ros_robot_controller/set_buzzer`
  - `robot_N/controller/cmd_vel`
- Hardcodes `num_robots = 3` in `main()`.
- Applies a joystick deadband of `0.1`.
- For `MentorPi_Mecanum`:
  - `lx` maps to `Twist.linear.y`.
  - `ly` maps to `Twist.linear.x`.
  - `rx` maps to `Twist.angular.z`.
- For `MentorPi_Acker`:
  - `ly` maps to forward/backward velocity.
  - `rx` maps to steering angle and angular velocity.
  - PWM servo ID `3` is used for front steering.
- Pressing `START` sends a 2500 Hz buzzer command to every configured robot.

### `peripherals/joystick_control.py`

This is the single-robot joystick implementation. It has the same basic axis and
button mapping as `joystick_control_multi.py`, but it publishes only to the
current robot's relative topics rather than looping through `robot_N/`
namespaces.

### `controller/odom_publisher_node.py`

Despite its name, this node is also the chassis command bridge:

- It subscribes to `controller/cmd_vel`.
- For `MentorPi_Mecanum`, it passes `linear.x`, `linear.y`, and `angular.z` to
  `MecanumChassis.set_velocity(...)`.
- For `MentorPi_Acker`, it passes speed and turn rate to
  `AckermannChassis.set_velocity(...)`, publishes motor speed, and publishes PWM
  servo commands when needed.
- It publishes motor output to `ros_robot_controller/set_motor`.
- It maintains and publishes local odometry on `odom_raw`.

### `ros_robot_controller_node.py`

This node is the hardware bridge to the RRC Lite / STM32 board:

- `ros_robot_controller/set_motor` calls `board.set_motor_speed(...)`.
- `ros_robot_controller/set_buzzer` calls `board.set_buzzer(...)`.
- `ros_robot_controller/pwm_servo/set_state` calls
  `board.pwm_servo_set_position(...)` or servo offset functions.
- It also publishes IMU, battery, button, SBUS, and other low-level data.

## Formation / TF Nodes Present but Not Used by Tutorial Commands

The `multi` package contains formation-oriented nodes installed through
`multi/setup.py`:

| Console Script | Code Path | Role |
|---|---|---|
| `formation_update` | `multi/formation_update.py` | Publishes static target transforms `point2` and `point3` relative to the master base frame. Supports row, column, and triangle layouts via services. |
| `slave_tf_listener` | `multi/slave_tf_listener.py` | Looks up a target TF relative to a slave base frame and publishes PID-generated `Twist` commands. |
| `tf_listen` | `multi/tf_listen.py` | Similar TF listener / PID follower variant with different defaults. |
| `tf_publish` | `multi/tf_publish.py` | Present in package entry points; inspect before using because it is not part of the tutorial command sequence. |
| `costmap_publish` | `multi/costmap_publish.py` | Publishes virtual wall / costmap-related messages based on TF. |

These nodes explain the package's broader formation-control direction, but the
official tutorial page section analyzed here launches only the controller,
direct group joystick broadcaster, and joystick input node.

## Code / Tutorial Caveats

- `wifi_conf.py` is part of `hiwonder-toolbox`, not this `ros2_ws` raw artifact.
  Network setup must therefore be validated on the robot image, not only in this
  workspace.
- `joystick_control_multi.py` lives under `launch/` but is executed as a direct
  Python script. It is not registered as a `console_scripts` entry point in
  `multi/setup.py`.
- `multi_controller.launch.py` conditionally includes `joy_control.launch.py`
  when `MASTER == HOST`. The tutorial also starts `joystick_control_multi.py`
  and `ros2 run joy joy_node` separately. This means the master path may start a
  single-robot joystick control path in addition to the group broadcaster,
  depending on the exact `MASTER` / `HOST` values.
- `peripherals/launch/joystick_control.launch.py` already starts `joy_node`, but
  the tutorial explicitly starts `ros2 run joy joy_node` in a separate terminal
  for the group broadcaster. Avoid running duplicate `joy_node` processes unless
  testing confirms the robot image expects that setup.
- `joystick_control_multi.py` hardcodes three robot namespaces. With only two
  vehicles, the unused `robot_3` publications are harmless if no subscribers
  exist, but the configured fleet size should be made explicit for production
  experiments.
- `.typerc` comments show `robot_1` / `robot_2`, while the tutorial text may show
  a trailing slash format such as `robot_1/`. Use one consistent namespace
  format across `MASTER`, `HOST`, and any manually constructed topic names, then
  verify actual topics with `ros2 topic list`.

## Recommended Verification on Hardware

After configuring two robots, run these checks before moving the vehicles:

```bash
source ~/.zshrc
echo "$MASTER"
echo "$HOST"
echo "$MACHINE_TYPE"
ros2 topic list | grep -E 'robot_[0-9]+/(controller/cmd_vel|ros_robot_controller/set_motor|ros_robot_controller/set_buzzer)'
ros2 topic echo /joy
```

Then test with a conservative joystick input and watch:

```bash
ros2 topic echo /robot_1/controller/cmd_vel
ros2 topic echo /robot_2/controller/cmd_vel
```

If the topics appear under a different slash pattern, adjust `MASTER`, `HOST`,
or launch arguments before running full motion tests.
