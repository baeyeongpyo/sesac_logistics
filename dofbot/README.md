# DOFBOT ROS 2 package

This package reuses Yahboom's `Arm_Lib` I2C protocol to control a DOFBOT
controller at I2C address `0x15` on bus `1`.

Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select dofbot
source install/setup.zsh
```

Start the driver without moving the arm:

```bash
ros2 run dofbot dofbot
```

The driver subscribes to `dofbot/command_joint_angles` (`std_msgs/Float64MultiArray`).
It accepts six target angles in degrees. Joints 1-4 and 6 accept 0-180; joint 5
accepts 0-270. The `motion_time_ms` parameter defaults to 1000 milliseconds.

Example command (this physically moves the arm):

```bash
ros2 topic pub --once /dofbot/command_joint_angles std_msgs/msg/Float64MultiArray "{data: [90, 90, 90, 90, 90, 90]}"
```
