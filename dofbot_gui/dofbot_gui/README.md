# DOFBOT GUI

Start the hardware driver in one terminal:

```bash
ros2 run dofbot dofbot
```

Then open the control window in another terminal:

```bash
ros2 run dofbot_gui dofbot_gui
```

Moving a slider sends the latest pose to `/dofbot/command_joint_angles` after a
short 80ms debounce. **Send pose** can also send the current pose explicitly.

For hand-guided teaching, press **Teach mode (torque off)** first. Support the
arm while moving it by hand, then press **Read current pose**. Press **Lock arm
(torque on)** to hold it again. Sending a pose turns torque back on.

Use **Start calibration (torque off)** once, then move every joint through its
safe range by hand. **Stop & save limits** stores only the observed minimum and
maximum for each joint in `config/dofbot_limits.json`. The limits are applied by
both the GUI and the driver; they do not change controller firmware or factory
calibration.
