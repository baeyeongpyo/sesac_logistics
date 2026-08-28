"""Dedicated slow teach-and-repeat GUI for a nine-step DOFBOT pick/place cycle."""

from __future__ import annotations

import json
import time
import tkinter as tk
import threading
from pathlib import Path
from tkinter import font as tkfont, ttk

import cv2
import numpy as np
import rclpy
from PIL import Image, ImageTk
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32MultiArray


PROGRAM_DIR = Path('/home/intelions/ros2_ws/config')
STEPS = (
    ('home', '1. 기본 자세'),
    ('pick_approach', '2. 픽 위치로'),
    ('pick', '3. 픽하고'),
    ('pick_lift', '4. 다시 들어올린 위치로'),
    ('drop_approach', '5. 내려놓을 위치로'),
    ('drop_position', '6. 드랍 위치'),
    ('drop', '7. 드랍하고'),
    ('drop_lift', '8. 다시 드랍한 위치로'),
    ('return_home', '9. 다시 기본 자세로'),
)


class RosInterface(Node):
    def __init__(self) -> None:
        super().__init__('dofbot_pick_place_gui')
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._subscription = None
        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._last_frame_time = 0.0
        self._joint_lock = threading.Lock()
        self._joint_angles: list[float | None] = [None] * 6
        self._move_publisher = self.create_publisher(Float32MultiArray, '/arm/move_all', 10)
        self._torque_publisher = self.create_publisher(Bool, '/arm/torque_cmd', 10)
        self.create_subscription(Float32MultiArray, '/arm/joint_angles',
                                 self._on_joint_angles, 10)

    def _on_joint_angles(self, message: Float32MultiArray) -> None:
        if len(message.data) < 6:
            return
        values = [float(value) if value >= 0 else None for value in message.data[:6]]
        with self._joint_lock:
            self._joint_angles = values

    def latest_angles(self) -> list[float | None]:
        with self._joint_lock:
            return self._joint_angles.copy()

    def move_all(self, pose: list[float], duration_ms: int) -> None:
        values = [float(value) for value in pose[:6]]
        values.append(float(max(20, min(30000, duration_ms))))
        self._move_publisher.publish(Float32MultiArray(data=values))

    def set_torque(self, enabled: bool) -> None:
        self._torque_publisher.publish(Bool(data=enabled))

    def set_enabled(self, enabled: bool) -> None:
        if enabled and self._subscription is None:
            self._subscription = self.create_subscription(
                CompressedImage, '/camera/image/compressed', self._on_image,
                self._qos)
        elif not enabled and self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
            with self._lock:
                self._jpeg = None

    def _on_image(self, message: CompressedImage) -> None:
        now = time.monotonic()
        if now - self._last_frame_time < 0.18:
            return
        self._last_frame_time = now
        with self._lock:
            self._jpeg = bytes(message.data)

    def latest(self) -> bytes | None:
        with self._lock:
            return self._jpeg


class PickPlaceGui:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self._configure_korean_fonts()
        self.root.title('DOFBOT 초저속 Pick & Place')
        self.root.resizable(False, False)
        self._feedback_busy = False
        self._capture_busy = False
        self._closing = False
        if not rclpy.ok():
            rclpy.init()
        self.ros_node = RosInterface()
        self.camera_thread = threading.Thread(
            target=self._spin_camera, daemon=True)
        self.camera_thread.start()
        self._camera_photo = None
        self._camera_after: str | None = None
        self.camera_enabled = tk.BooleanVar(value=False)
        self.camera_button_text = tk.StringVar(value='카메라 켜기')
        self.poses: dict[str, list[float] | None] = {key: None for key, _ in STEPS}
        self.pose_text = {key: tk.StringVar(value='—') for key, _ in STEPS}
        self.current_text = tk.StringVar(value='현재 관절: 읽는 중…')
        self.status = tk.StringVar(value='Torque OFF 후 각 단계 자세를 손으로 잡고 기록하세요.')
        self.torque_text = tk.StringVar(value='Torque: unknown')
        self.transition_seconds = {key: tk.DoubleVar(value=5.0) for key, _ in STEPS}
        self.active_box = tk.IntVar(value=1)
        self._last_angles: list[float | None] = [None] * 6
        self._playing = False
        self._play_index = 0
        self._play_after: str | None = None
        self._segment_source: list[float] | None = None
        self._segment_target: list[float] | None = None
        self._segment_started = 0.0
        self._segment_duration = 0.0
        self._feedback_after: str | None = None
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self._feedback_loop()
        self._camera_loop()

    def _spin_camera(self) -> None:
        try:
            rclpy.spin(self.ros_node)
        except ExternalShutdownException:
            pass

    def _configure_korean_fonts(self) -> None:
        family = 'Noto Sans CJK KR'
        for name in (
            'TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont',
            'TkCaptionFont', 'TkSmallCaptionFont', 'TkIconFont', 'TkTooltipFont',
        ):
            try:
                tkfont.nametofont(name).configure(family=family)
            except tk.TclError:
                pass

    def _build(self) -> None:
        frame = ttk.Frame(self.root, padding=14)
        frame.grid()
        ttk.Label(frame, text='DOFBOT Pick & Place 좌표 기록', font=('', 15, 'bold')).grid(
            row=0, column=0, columnspan=5, sticky='w', pady=(0, 8))
        ttk.Label(frame, textvariable=self.current_text, font=('', 11, 'bold')).grid(
            row=1, column=0, columnspan=5, sticky='w', pady=(0, 8))

        ttk.Label(frame, text='좌표 단계').grid(row=2, column=0, sticky='w')
        ttk.Label(frame, text='관절 좌표값').grid(row=2, column=1, sticky='w')
        ttk.Label(frame, text='이전→현재 이동시간(초)').grid(row=2, column=2)
        for index, (key, label) in enumerate(STEPS):
            row = index + 3
            ttk.Label(frame, text=label, width=24).grid(row=row, column=0, sticky='w', pady=2)
            ttk.Entry(frame, textvariable=self.pose_text[key], width=42).grid(
                row=row, column=1, sticky='w', padx=6)
            tk.Spinbox(frame, from_=0.5, to=30.0, increment=0.5, width=6,
                       textvariable=self.transition_seconds[key]).grid(row=row, column=2, padx=5)
            ttk.Button(frame, text='현재값 기록', command=lambda k=key: self._capture(k)).grid(
                row=row, column=3, padx=3)
            ttk.Button(frame, text='이 위치로 이동', command=lambda k=key: self._go_to(k)).grid(
                row=row, column=4, padx=3)

        boxes = ttk.LabelFrame(frame, text='상자 1~8 프로그램 저장 / 불러오기', padding=8)
        boxes.grid(row=12, column=0, columnspan=5, sticky='ew', pady=(10, 0))
        for box in range(1, 9):
            slot = ttk.Frame(boxes)
            slot.grid(row=(box - 1) // 4, column=(box - 1) % 4, padx=5, pady=3, sticky='w')
            ttk.Radiobutton(slot, text=f'상자 {box}', variable=self.active_box,
                            value=box).pack(side=tk.LEFT)
            ttk.Button(slot, text='저장', command=lambda b=box: self._save_box(b)).pack(side=tk.LEFT, padx=2)
            ttk.Button(slot, text='불러오기', command=lambda b=box: self._load_box(b)).pack(side=tk.LEFT)
        ttk.Button(boxes, text='선택 상자 JSON 편집', command=self._open_json_editor).grid(
            row=2, column=0, columnspan=4, sticky='w', padx=5, pady=(5, 0))

        controls = ttk.LabelFrame(frame, text='실행', padding=8)
        controls.grid(row=13, column=0, columnspan=5, sticky='ew', pady=(8, 0))
        ttk.Button(controls, text='Torque OFF', command=self._torque_off).pack(side=tk.LEFT)
        ttk.Button(controls, text='Torque ON', command=self._torque_on).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text='전체 초저속 실행', command=self._start).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(controls, text='정지', command=self._stop).pack(side=tk.LEFT)

        footer = ttk.Frame(frame)
        footer.grid(row=14, column=0, columnspan=5, sticky='ew', pady=(8, 0))
        ttk.Label(footer, textvariable=self.torque_text, font=('', 10, 'bold')).pack(side=tk.LEFT)
        ttk.Label(footer, textvariable=self.status).pack(side=tk.LEFT, padx=16)

        camera = ttk.LabelFrame(self.root, text='카메라 /camera/image/compressed', padding=8)
        camera.grid(row=0, column=1, sticky='n', padx=(0, 14), pady=14)
        self.camera_view = ttk.Label(camera, text='카메라 영상을 기다리는 중…',
                                     width=60, anchor='center')
        self.camera_view.pack()
        camera_controls = ttk.Frame(camera)
        camera_controls.pack(fill=tk.X, pady=(6, 0))
        self.camera_status = tk.StringVar(value='카메라 OFF — 영상 구독/디코딩 중지')
        ttk.Button(camera_controls, textvariable=self.camera_button_text,
                   command=self._toggle_camera).pack(side=tk.LEFT)
        ttk.Label(camera_controls, textvariable=self.camera_status).pack(side=tk.LEFT, padx=8)

    def _toggle_camera(self) -> None:
        enabled = not bool(self.camera_enabled.get())
        self.camera_enabled.set(enabled)
        self.ros_node.set_enabled(enabled)
        if enabled:
            self.camera_button_text.set('카메라 끄기')
            self.camera_status.set('ROS 카메라 토픽 연결 중')
        else:
            self.camera_button_text.set('카메라 켜기')
            self.camera_status.set('카메라 OFF — 영상 구독/디코딩 중지')
            self._camera_photo = None
            self.camera_view.configure(image='', text='카메라가 꺼져 있습니다', width=60)

    def _camera_loop(self) -> None:
        if self.camera_enabled.get():
            jpeg = self.ros_node.latest()
            if jpeg:
                array = np.frombuffer(jpeg, dtype=np.uint8)
                frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
                if frame is not None:
                    height, width = frame.shape[:2]
                    scale = min(320 / width, 240 / height)
                    size = (max(1, round(width * scale)), max(1, round(height * scale)))
                    frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    self._camera_photo = ImageTk.PhotoImage(Image.fromarray(frame))
                    self.camera_view.configure(image=self._camera_photo, text='', width=0)
                    self.camera_status.set(f'저부하 표시 320×240 / 5fps (원본 {width}×{height})')
            delay = 200
        else:
            delay = 500
        self._camera_after = self.root.after(delay, self._camera_loop)

    def _read_angles(self, retries: int = 3) -> list[float | None]:
        for _ in range(max(1, retries)):
            values = self.ros_node.latest_angles()
            if all(value is not None for value in values):
                return values
            time.sleep(0.05)
        return self.ros_node.latest_angles()

    def _feedback_loop(self) -> None:
        if not self._playing and not self._feedback_busy and not self._capture_busy:
            self._feedback_busy = True
            threading.Thread(target=self._read_feedback_worker, daemon=True).start()
        self._feedback_after = self.root.after(300, self._feedback_loop)

    def _read_feedback_worker(self) -> None:
        values = self._read_angles(retries=1)
        if not self._closing:
            self.root.after(0, self._apply_feedback, values)

    def _apply_feedback(self, values: list[float | None]) -> None:
        self._feedback_busy = False
        for index, value in enumerate(values):
            if value is not None:
                self._last_angles[index] = value
        shown = ['—' if value is None else f'{value:.1f}' for value in self._last_angles]
        self.current_text.set('현재 관절: [' + ', '.join(shown) + ']')

    def _capture(self, key: str) -> None:
        if self._capture_busy:
            self.status.set('다른 좌표를 기록 중입니다. 잠시 기다려 주세요.')
            return
        self._capture_busy = True
        self.status.set(f'{dict(STEPS)[key]} 현재 관절값을 읽는 중…')
        threading.Thread(target=self._capture_worker, args=(key,), daemon=True).start()

    def _capture_worker(self, key: str) -> None:
        values = self._read_angles(retries=5)
        if not self._closing:
            self.root.after(0, self._finish_capture, key, values)

    def _finish_capture(self, key: str, values: list[float | None]) -> None:
        self._capture_busy = False
        if any(value is None for value in values):
            missing = [str(i) for i, value in enumerate(values, start=1) if value is None]
            self.status.set(f'관절 {", ".join(missing)} 값을 읽지 못했습니다. 다시 눌러 주세요.')
            return
        pose = [float(value) for value in values]
        self.poses[key] = pose
        self.pose_text[key].set(self._format_pose(pose))
        label = dict(STEPS)[key]
        self.status.set(f'{label} 좌표 기록 완료: {self._format_pose(pose)}')

    def _format_pose(self, pose: list[float]) -> str:
        return '[' + ', '.join(f'{value:.1f}' for value in pose) + ']'

    def _seconds_for(self, key: str) -> float:
        try:
            return max(0.5, min(30.0, float(self.transition_seconds[key].get())))
        except (TypeError, ValueError, tk.TclError):
            self.transition_seconds[key].set(5.0)
            return 5.0

    def _command(self, pose: list[float], seconds: float) -> None:
        duration_ms = max(100, min(30000, round(seconds * 1000)))
        self.ros_node.set_torque(True)
        self.ros_node.move_all(pose, duration_ms)
        self.torque_text.set('Torque: ON')

    def _go_to(self, key: str) -> None:
        try:
            self._sync_pose_entry(key)
        except ValueError as error:
            self.status.set(f'좌표 오류: {error}')
            return
        pose = self.poses[key]
        if pose is None:
            self.status.set('먼저 이 단계의 현재값을 기록하세요.')
            return
        self._stop(update_status=False, hold=False)
        seconds = self._seconds_for(key)
        self._command(pose, seconds)
        self.status.set(f'{dict(STEPS)[key]} 위치로 {seconds:.1f}초 동안 이동합니다.')

    def _start(self) -> None:
        try:
            self._sync_all_pose_entries()
        except ValueError as error:
            self.status.set(f'좌표 오류: {error}')
            return
        missing = [label for key, label in STEPS if self.poses[key] is None]
        if missing:
            self.status.set('미기록 단계: ' + ', '.join(missing))
            return
        current = self._read_angles(retries=5)
        if any(value is None for value in current):
            self.status.set('ROS 드라이버의 현재 관절값을 기다리는 중입니다. 잠시 후 다시 실행하세요.')
            return
        self._stop(update_status=False, hold=False)
        self.ros_node.set_torque(True)
        self.torque_text.set('Torque: ON')
        self._playing = True
        self._play_index = 0
        self._segment_source = [float(value) for value in current]
        self.status.set('단일 ROS 드라이버로 연속 보간 실행을 시작합니다.')
        self._play_next()

    def _play_next(self) -> None:
        if not self._playing:
            return
        if self._play_index >= len(STEPS):
            self._playing = False
            self._play_after = None
            self.status.set('전체 Pick & Place 동작이 완료되었습니다. Torque는 ON입니다.')
            return
        key, label = STEPS[self._play_index]
        target = self.poses[key]
        if target is None or self._segment_source is None:
            self._stop()
            return
        self._segment_target = [float(value) for value in target]
        self._segment_started = time.monotonic()
        self._segment_duration = self._seconds_for(key)
        self.status.set(
            f'연속 이동 {self._play_index + 1}/9: {label}까지 '
            f'{self._segment_duration:.1f}초')
        self._run_segment()

    def _run_segment(self) -> None:
        if not self._playing or self._segment_source is None or self._segment_target is None:
            return
        elapsed = time.monotonic() - self._segment_started
        ratio = min(1.0, elapsed / max(0.1, self._segment_duration))
        # Smoothstep removes abrupt velocity changes at both ends of each segment.
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        pose = [start + (target - start) * blend
                for start, target in zip(self._segment_source, self._segment_target)]
        self.ros_node.move_all(pose, 150)
        if ratio >= 1.0:
            self._segment_source = self._segment_target.copy()
            self._play_index += 1
            self._play_after = self.root.after(100, self._play_next)
        else:
            self._play_after = self.root.after(100, self._run_segment)

    def _stop(self, update_status: bool = True, hold: bool = True) -> None:
        self._playing = False
        if self._play_after is not None:
            self.root.after_cancel(self._play_after)
            self._play_after = None
        if hold:
            current = self._read_angles(retries=2)
            if all(value is not None for value in current):
                self._command([float(value) for value in current], 0.1)
        if update_status:
            self.status.set('실행을 정지했습니다. 현재 위치를 유지합니다.')

    def _torque_off(self) -> None:
        self._stop(update_status=False, hold=False)
        self.ros_node.set_torque(False)
        self.torque_text.set('Torque: OFF')
        self.status.set('Torque OFF: 로봇팔을 손으로 움직여 각 위치를 기록하세요.')

    def _torque_on(self) -> None:
        self._stop(update_status=False, hold=False)
        self.ros_node.set_torque(True)
        self.torque_text.set('Torque: ON')
        self.status.set('Torque ON: 현재 위치를 유지합니다.')

    def _program_file(self, box: int | None = None) -> Path:
        number = int(self.active_box.get() if box is None else box)
        return PROGRAM_DIR / f'dofbot_pick_place_box{number}.json'

    def _sync_pose_entry(self, key: str) -> None:
        text = self.pose_text[key].get().strip()
        if not text or text == '—':
            self.poses[key] = None
            return
        try:
            raw = json.loads(text) if text.startswith('[') else text.split(',')
            pose = [float(value) for value in raw]
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            raise ValueError(f'{dict(STEPS)[key]} 숫자 형식이 잘못되었습니다') from error
        if len(pose) != 6:
            raise ValueError(f'{dict(STEPS)[key]} 관절값은 정확히 6개여야 합니다')
        maximums = (180, 180, 180, 180, 270, 180)
        if any(value < 0 or value > maximums[i] for i, value in enumerate(pose)):
            raise ValueError(f'{dict(STEPS)[key]} 관절 범위를 벗어났습니다')
        self.poses[key] = pose
        self.pose_text[key].set(self._format_pose(pose))

    def _sync_all_pose_entries(self) -> None:
        for key, _ in STEPS:
            self._sync_pose_entry(key)

    def _data_from_ui(self) -> dict[str, object]:
        self._sync_all_pose_entries()
        missing = [label for key, label in STEPS if self.poses[key] is None]
        if missing:
            raise ValueError('미기록 단계: ' + ', '.join(missing))
        return {
            'transition_seconds': {key: self._seconds_for(key) for key, _ in STEPS},
            'poses': self.poses,
        }

    def _apply_data(self, data: dict[str, object]) -> None:
        loaded = data['poses']
        if not isinstance(loaded, dict):
            raise ValueError('poses는 객체여야 합니다')
        new_poses: dict[str, list[float]] = {}
        for key, _ in STEPS:
            pose = [float(value) for value in loaded[key]]
            if len(pose) != 6:
                raise ValueError(f'{key}: 관절값은 6개여야 합니다')
            maximums = (180, 180, 180, 180, 270, 180)
            if any(value < 0 or value > maximums[i] for i, value in enumerate(pose)):
                raise ValueError(f'{key}: 관절 범위를 벗어났습니다')
            new_poses[key] = pose
        legacy_seconds = float(data.get('seconds_per_step', 5.0))
        saved_times = data.get('transition_seconds', {})
        if not isinstance(saved_times, dict):
            raise ValueError('transition_seconds는 객체여야 합니다')
        for key, _ in STEPS:
            seconds = float(saved_times.get(key, legacy_seconds))
            if seconds < 0.5 or seconds > 30.0:
                raise ValueError(f'{key}: 이동시간은 0.5~30초여야 합니다')
        for key, _ in STEPS:
            self.poses[key] = new_poses[key]
            self.pose_text[key].set(self._format_pose(new_poses[key]))
            self.transition_seconds[key].set(float(saved_times.get(key, legacy_seconds)))

    def _save_box(self, box: int) -> None:
        self.active_box.set(box)
        try:
            data = self._data_from_ui()
            path = self._program_file(box)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
        except (OSError, ValueError) as error:
            self.status.set(f'상자 {box} 저장 실패: {error}')
            return
        self.status.set(f'상자 {box} 저장 완료: {path}')

    def _load_box(self, box: int) -> None:
        self.active_box.set(box)
        path = self._program_file(box)
        try:
            data = json.loads(path.read_text())
            self._apply_data(data)
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.status.set(f'상자 {box} 불러오기 실패: {error}')
            return
        self.status.set(f'상자 {box} 불러오기 완료: {path}')

    def _open_json_editor(self) -> None:
        box = int(self.active_box.get())
        path = self._program_file(box)
        try:
            initial = path.read_text()
        except OSError:
            try:
                initial = json.dumps(self._data_from_ui(), ensure_ascii=False, indent=2) + '\n'
            except ValueError:
                initial = json.dumps({'transition_seconds': {}, 'poses': self.poses},
                                     ensure_ascii=False, indent=2) + '\n'
        window = tk.Toplevel(self.root)
        window.title(f'상자 {box} JSON 편집 - {path.name}')
        editor = tk.Text(window, width=86, height=34, font=('monospace', 10), undo=True)
        editor.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        editor.insert('1.0', initial)
        bar = ttk.Frame(window)
        bar.pack(fill=tk.X, padx=8, pady=(0, 8))

        def apply_and_save() -> None:
            try:
                data = json.loads(editor.get('1.0', tk.END))
                self._apply_data(data)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
            except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
                self.status.set(f'상자 {box} JSON 오류: {error}')
                return
            self.status.set(f'상자 {box} JSON 검증·저장·적용 완료')
            window.destroy()

        ttk.Button(bar, text='검증 후 저장·적용', command=apply_and_save).pack(side=tk.LEFT)
        ttk.Button(bar, text='취소', command=window.destroy).pack(side=tk.LEFT, padx=6)

    def _close(self) -> None:
        self._closing = True
        self._playing = False
        if self._play_after is not None:
            self.root.after_cancel(self._play_after)
        if self._feedback_after is not None:
            self.root.after_cancel(self._feedback_after)
        if self._camera_after is not None:
            self.root.after_cancel(self._camera_after)
        self.ros_node.set_enabled(False)
        self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    PickPlaceGui().run()
