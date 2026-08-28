"""Small desktop controller for publishing DOFBOT joint-angle commands."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dofbot.Arm_Lib import Arm_Device


JOINT_LIMITS = (180, 180, 180, 180, 270, 180)
LIMITS_FILE = Path('/home/intelions/ros2_ws/config/dofbot_limits.json')
RECORDING_FILE = Path('/home/intelions/ros2_ws/config/dofbot_recording.json')
FEEDBACK_INTERVAL_MS = 150


class DofbotGui(Node):
    def __init__(self) -> None:
        super().__init__('dofbot_gui')
        self.publisher = self.create_publisher(
            Float64MultiArray, 'dofbot/command_joint_angles', 10
        )
        self.limits_publisher = self.create_publisher(
            Float64MultiArray, 'dofbot/safety_limits', 10
        )
        self.root = tk.Tk()
        self.root.title('DOFBOT control')
        self.root.resizable(False, False)
        self.angles = [tk.IntVar(value=90) for _ in range(6)]
        self.minimums, self.maximums = self._load_limits()
        self.scales: list[tk.Scale] = []
        self.current_values = [tk.StringVar(value="Current: —") for _ in range(6)]
        self.status = tk.StringVar(value='Set a pose, then press Send pose.')
        self._pending_send: str | None = None
        self._loading_pose = False
        self._arm = Arm_Device()
        self._calibrating = False
        self._calibration_mins: list[float | None] | None = None
        self._calibration_maxs: list[float | None] | None = None
        self.torque_status = tk.StringVar(value='Torque: unknown')
        self.recording_status = tk.StringVar(value='Recording: 0 poses / 0.0 s')
        self.playback_speed = tk.DoubleVar(value=0.5)
        self._recording = False
        self._recording_started = 0.0
        self._recorded_poses: list[dict[str, object]] = []
        self._playing = False
        self._play_index = 0
        self._play_started = 0.0
        self._play_after: str | None = None
        self._feedback_after: str | None = None
        self._last_feedback: list[float | None] = [None] * 6
        self._build_window()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self._feedback_loop()

    def _build_window(self) -> None:
        frame = ttk.Frame(self.root, padding=16)
        frame.grid()

        ttk.Label(frame, text='Joint angles (degrees)', font=('', 14, 'bold')).grid(
            row=0, column=0, columnspan=2, sticky='w', pady=(0, 8)
        )
        for index, angle in enumerate(self.angles, start=1):
            ttk.Label(frame, text=f'Joint {index}').grid(row=index, column=0, sticky='w')
            scale = tk.Scale(
                frame,
                from_=self.minimums[index - 1],
                to=self.maximums[index - 1],
                orient=tk.HORIZONTAL,
                variable=angle,
                length=320,
                resolution=1,
                command=self._on_slider_change,
            )
            scale.grid(row=index, column=1, sticky='ew')
            self.scales.append(scale)
            ttk.Label(frame, textvariable=self.current_values[index - 1], width=14).grid(row=index, column=4, padx=(8, 0), sticky="w")
            ttk.Button(frame, text="Set Min", command=lambda i=index - 1: self._set_joint_limit(i, True)).grid(row=index, column=2, padx=(6, 0))
            ttk.Button(frame, text="Set Max", command=lambda i=index - 1: self._set_joint_limit(i, False)).grid(row=index, column=3, padx=(4, 0))

        buttons = ttk.Frame(frame)
        buttons.grid(row=7, column=0, columnspan=5, sticky='ew', pady=(12, 4))
        ttk.Button(buttons, text='Torque OFF', command=self._teach_mode).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(buttons, text='Start calibration (torque off)', command=self._start_calibration).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(buttons, text='Torque ON', command=self._lock_arm).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(buttons, text='Center (90°)', command=self._center).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(buttons, text='Read current pose', command=self._load_current_pose).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(buttons, text='Send pose', command=self._send_pose).pack(side=tk.LEFT)
        recorder = ttk.LabelFrame(frame, text='Joint recording / playback', padding=8)
        recorder.grid(row=8, column=0, columnspan=5, sticky='ew', pady=(10, 0))
        ttk.Button(recorder, text='Start record', command=self._start_recording).pack(side=tk.LEFT)
        ttk.Button(recorder, text='Stop record', command=self._stop_recording).pack(side=tk.LEFT, padx=4)
        ttk.Button(recorder, text='Play', command=self._start_playback).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(recorder, text='Stop play', command=self._stop_playback).pack(side=tk.LEFT)
        ttk.Label(recorder, text='Speed').pack(side=tk.LEFT, padx=(12, 0))
        tk.Scale(recorder, from_=0.25, to=2.0, resolution=0.25,
                 orient=tk.HORIZONTAL, variable=self.playback_speed,
                 length=130, showvalue=True).pack(side=tk.LEFT)
        ttk.Button(recorder, text='Save', command=self._save_recording).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(recorder, text='Load', command=self._load_recording).pack(side=tk.LEFT)
        ttk.Button(recorder, text='Clear', command=self._clear_recording).pack(side=tk.LEFT, padx=4)
        ttk.Label(recorder, textvariable=self.recording_status).pack(side=tk.LEFT, padx=10)
        calibration = ttk.LabelFrame(frame, text='Software safety calibration', padding=8)
        calibration.grid(row=9, column=0, columnspan=5, sticky='ew', pady=(10, 0))
        ttk.Label(
            calibration,
            text='Move all joints by hand through their safe ranges, then save.',
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(calibration, text='Stop & save limits', command=self._stop_calibration).pack(side=tk.LEFT)
        ttk.Button(calibration, text='Save limits', command=self._save_manual_limits).pack(side=tk.LEFT, padx=(8, 0))
        state = ttk.Frame(frame)
        state.grid(row=10, column=0, columnspan=5, sticky='ew', pady=(8, 0))
        ttk.Label(state, textvariable=self.torque_status, font=('', 10, 'bold')).pack(side=tk.LEFT)
        ttk.Label(state, textvariable=self.status).pack(side=tk.LEFT, padx=16)

    def _set_joint_limit(self, index: int, is_minimum: bool) -> None:
        value = float(self.angles[index].get())
        if is_minimum:
            if value >= self.maximums[index]:
                self.status.set(f"Joint {index + 1}: Min must be below Max.")
                return
            self.minimums[index] = value
        else:
            if value <= self.minimums[index]:
                self.status.set(f"Joint {index + 1}: Max must be above Min.")
                return
            self.maximums[index] = value
        self.scales[index].configure(from_=self.minimums[index], to=self.maximums[index])
        self.status.set(f"Joint {index + 1} {'min' if is_minimum else 'max'} set to {value:.0f}°. Click Save limits to persist.")

    def _save_manual_limits(self) -> None:
        try:
            self._save_limits()
        except OSError as error:
            self.status.set(f"Could not save limits: {error}")
            return
        self.limits_publisher.publish(Float64MultiArray(data=self.minimums + self.maximums))
        self.status.set(f"Saved limits to {LIMITS_FILE}.")

    def _center(self) -> None:
        for angle in self.angles:
            angle.set(90)
        self._schedule_send()
        self.status.set('Centered sliders at 90° and sending the pose.')

    def _start_calibration(self) -> None:
        self._arm.Arm_serial_set_torque(0)
        angles = self._read_current_angles(retries=1)
        if all(angle is None for angle in angles):
            self.status.set('Could not read the current pose from the arm.')
            return
        self._calibrating = True
        self._calibration_mins = angles.copy()
        self._calibration_maxs = angles.copy()
        self.status.set('Recording limits. Move every joint by hand; click Stop & save when done.')
        self._sample_calibration()

    def _teach_mode(self) -> None:
        self._calibrating = False
        self._stop_playback(False)
        self._arm.Arm_serial_set_torque(0)
        self.torque_status.set('Torque: OFF')
        self.status.set('Teach mode: torque is off. Move the arm by hand, then Read current pose.')

    def _lock_arm(self) -> None:
        self._arm.Arm_serial_set_torque(1)
        self.torque_status.set('Torque: ON')
        self.status.set('Torque is on. The arm is holding its current pose.')

    def _on_slider_change(self, _value: str) -> None:
        if not self._loading_pose:
            self._schedule_send()

    def _schedule_send(self) -> None:
        if self._pending_send is not None:
            self.root.after_cancel(self._pending_send)
        self._pending_send = self.root.after(80, self._send_pose)

    def _send_pose(self) -> None:
        self._pending_send = None
        values = [float(angle.get()) for angle in self.angles]
        self._arm.Arm_serial_set_torque(1)
        self.torque_status.set('Torque: ON')
        target = [int(round(value)) for value in values]
        self._arm.Arm_serial_servo_write6(*target, 200)
        self.publisher.publish(Float64MultiArray(data=values))
        self.status.set(f'Torque on. Sent: {[int(value) for value in values]}')

    def _load_current_pose(self) -> None:
        actual_angles = self._read_current_angles()
        for index, angle in enumerate(actual_angles):
            self.current_values[index].set("Current: N/A" if angle is None else f"Current: {angle:.0f}°")
        if all(angle is None for angle in actual_angles):
            self.status.set('Could not read the current pose from the arm.')
            return
        self._loading_pose = True
        try:
            for variable, angle in zip(self.angles, actual_angles):
                if angle is not None:
                    variable.set(round(angle))
        finally:
            self._loading_pose = False
        if self._pending_send is not None:
            self.root.after_cancel(self._pending_send)
            self._pending_send = None
        unavailable = [str(index) for index, angle in enumerate(actual_angles, start=1) if angle is None]
        if unavailable:
            self.status.set(f'Loaded readable joints. No feedback from Joint {", ".join(unavailable)}.')
        else:
            self.status.set('Loaded the current pose. It has not been sent.')

    def _sample_calibration(self) -> None:
        if not self._calibrating:
            return
        actual_angles = self._read_current_angles(retries=1)
        if self._calibration_mins is not None and self._calibration_maxs is not None:
            for index, current in enumerate(actual_angles):
                if current is None:
                    continue
                if self._calibration_mins[index] is None:
                    self._calibration_mins[index] = current
                    self._calibration_maxs[index] = current
                else:
                    self._calibration_mins[index] = min(self._calibration_mins[index], current)
                    self._calibration_maxs[index] = max(self._calibration_maxs[index], current)
        self.root.after(250, self._sample_calibration)

    def _stop_calibration(self) -> None:
        if not self._calibrating or self._calibration_mins is None or self._calibration_maxs is None:
            self.status.set('Start calibration before saving limits.')
            return
        self._calibrating = False
        updated_joints = 0
        for index, (minimum, maximum) in enumerate(
            zip(self._calibration_mins, self._calibration_maxs)
        ):
            if minimum is None or maximum is None or maximum - minimum < 2:
                continue
            self.minimums[index] = minimum
            self.maximums[index] = maximum
            self.scales[index].configure(from_=minimum, to=maximum)
            updated_joints += 1
        try:
            self._save_limits()
        except OSError as error:
            self.status.set(f'Could not save limits: {error}')
            return
        self.limits_publisher.publish(
            Float64MultiArray(data=self.minimums + self.maximums)
        )
        self.status.set(
            f'Saved {updated_joints} joints to {LIMITS_FILE}. Torque remains off.'
        )

    def _read_current_angles(self, retries: int = 8) -> list[float | None]:
        actual_angles = []
        for index in range(1, 7):
            angle = None
            for _attempt in range(retries):
                angle = self._arm.Arm_serial_servo_read(index)
                if angle is not None:
                    break
                time.sleep(0.015)
            actual_angles.append(angle)
        return [float(angle) if angle is not None else None for angle in actual_angles]

    def _feedback_loop(self) -> None:
        actual = self._read_current_angles(retries=1)
        for index, angle in enumerate(actual):
            if angle is not None:
                self._last_feedback[index] = angle
                self.current_values[index].set(f'Current: {angle:6.1f}°')
        if self._recording and any(value is not None for value in actual):
            elapsed = time.monotonic() - self._recording_started
            values = self._last_feedback.copy()
            if all(value is not None for value in values):
                self._recorded_poses.append({'time': elapsed, 'angles': values})
                self.recording_status.set(
                    f'Recording: {len(self._recorded_poses)} poses / {elapsed:.1f} s')
        self._feedback_after = self.root.after(FEEDBACK_INTERVAL_MS, self._feedback_loop)

    def _start_recording(self) -> None:
        self._stop_playback(False)
        self._recorded_poses = []
        self._recording_started = time.monotonic()
        self._recording = True
        self._arm.Arm_serial_set_torque(0)
        self.torque_status.set('Torque: OFF')
        self.recording_status.set('Recording: 0 poses / 0.0 s')
        self.status.set('Recording started. Move the arm by hand; torque is OFF.')

    def _stop_recording(self) -> None:
        if not self._recording:
            self.status.set('Recording is not active.')
            return
        self._recording = False
        duration = float(self._recorded_poses[-1]['time']) if self._recorded_poses else 0.0
        self.recording_status.set(f'Recorded: {len(self._recorded_poses)} poses / {duration:.1f} s')
        self.status.set('Recording stopped. Torque remains OFF.')

    def _start_playback(self) -> None:
        if not self._recorded_poses:
            self.status.set('There is no recording to play.')
            return
        self._recording = False
        self._stop_playback(False)
        self._playing = True
        self._play_index = 0
        self._play_started = time.monotonic()
        self._arm.Arm_serial_set_torque(1)
        self.torque_status.set('Torque: ON')
        self.status.set('Playing recorded motion. Torque is ON.')
        self._play_next_pose()

    def _play_next_pose(self) -> None:
        if not self._playing:
            return
        if self._play_index >= len(self._recorded_poses):
            self._playing = False
            self._play_after = None
            self.status.set('Playback finished. Torque remains ON.')
            return
        pose = self._recorded_poses[self._play_index]
        values = [float(value) for value in pose['angles']]
        target = [int(round(value)) for value in values]
        speed = max(0.25, min(2.0, float(self.playback_speed.get())))
        if self._play_index == 0:
            # Move gently to the first recorded pose before following the path.
            motion_ms = round(400 / speed)
        else:
            previous_time = float(self._recorded_poses[self._play_index - 1]['time'])
            recorded_interval = float(pose['time']) - previous_time
            motion_ms = max(40, min(4000, round(recorded_interval * 1000 / speed)))
        self._arm.Arm_serial_servo_write6(*target, motion_ms)
        self.publisher.publish(Float64MultiArray(data=values))
        self._play_index += 1
        self.recording_status.set(
            f'Playing: {self._play_index}/{len(self._recorded_poses)} poses at {speed:.2f}x')
        # The controller interpolates throughout motion_ms, eliminating stepwise playback.
        self._play_after = self.root.after(motion_ms, self._play_next_pose)

    def _stop_playback(self, update_status: bool = True) -> None:
        self._playing = False
        if self._play_after is not None:
            self.root.after_cancel(self._play_after)
            self._play_after = None
        if update_status:
            self.status.set('Playback stopped. Torque state was not changed.')

    def _clear_recording(self) -> None:
        self._recording = False
        self._stop_playback(False)
        self._recorded_poses = []
        self.recording_status.set('Recording: 0 poses / 0.0 s')
        self.status.set('Recording cleared.')

    def _save_recording(self) -> None:
        if not self._recorded_poses:
            self.status.set('There is no recording to save.')
            return
        try:
            RECORDING_FILE.parent.mkdir(parents=True, exist_ok=True)
            RECORDING_FILE.write_text(json.dumps(self._recorded_poses, indent=2) + '\n')
        except OSError as error:
            self.status.set(f'Could not save recording: {error}')
            return
        self.status.set(f'Saved {len(self._recorded_poses)} poses.')

    def _load_recording(self) -> None:
        try:
            poses = json.loads(RECORDING_FILE.read_text())
            if not isinstance(poses, list) or not poses:
                raise ValueError('recording is empty')
            for pose in poses:
                if len(pose['angles']) != 6 or float(pose['time']) < 0:
                    raise ValueError('invalid pose')
                pose['time'] = float(pose['time'])
                pose['angles'] = [float(value) for value in pose['angles']]
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.status.set(f'Could not load recording: {error}')
            return
        self._recorded_poses = poses
        duration = float(poses[-1]['time'])
        self.recording_status.set(f'Loaded: {len(poses)} poses / {duration:.1f} s')
        self.status.set(f'Loaded recording from {RECORDING_FILE}.')

    def _load_limits(self) -> tuple[list[float], list[float]]:
        try:
            data = json.loads(LIMITS_FILE.read_text())
            minimums = [float(value) for value in data['minimums']]
            maximums = [float(value) for value in data['maximums']]
            if len(minimums) == 6 and len(maximums) == 6:
                return minimums, maximums
        except (FileNotFoundError, ValueError, KeyError, json.JSONDecodeError):
            pass
        return [0.0] * 6, list(map(float, JOINT_LIMITS))

    def _save_limits(self) -> None:
        LIMITS_FILE.parent.mkdir(parents=True, exist_ok=True)
        LIMITS_FILE.write_text(
            json.dumps({'minimums': self.minimums, 'maximums': self.maximums}, indent=2)
            + '\n'
        )

    def _close(self) -> None:
        self._recording = False
        self._stop_playback(False)
        if self._feedback_after is not None:
            self.root.after_cancel(self._feedback_after)
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DofbotGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
