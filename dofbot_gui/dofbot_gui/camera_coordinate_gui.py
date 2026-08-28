"""Camera-coordinate pick/place GUI with teach-point workspace calibration."""

from __future__ import annotations

import json
import math
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import font as tkfont, ttk

import cv2
import numpy as np
import rclpy
from PIL import Image, ImageTk
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Bool, Float32MultiArray


CALIBRATION_FILE = Path('/home/intelions/ros2_ws/config/camera_pick_calibration.json')
RGB_TOPIC = '/ascamera/ascamera_node/rgb0/image'
DEPTH_TOPIC = '/ascamera/ascamera_node/depth0/image_raw'


class RosInterface(Node):
    def __init__(self) -> None:
        super().__init__('dofbot_camera_coordinate_gui')
        self._lock = threading.Lock()
        self._rgb: np.ndarray | None = None
        self._depth: np.ndarray | None = None
        self._angles: list[float | None] = [None] * 6
        self._last_rgb_time = 0.0
        self._last_depth_time = 0.0
        self.create_subscription(RosImage, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(RosImage, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/arm/joint_angles', self._on_angles, 10)
        self._move = self.create_publisher(Float32MultiArray, '/arm/move_all', 10)
        self._torque = self.create_publisher(Bool, '/arm/torque_cmd', 10)

    def _on_rgb(self, msg: RosImage) -> None:
        now = time.monotonic()
        if now - self._last_rgb_time < 0.18:
            return
        self._last_rgb_time = now
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            channels = max(1, msg.step // max(1, msg.width))
            frame = raw.reshape(msg.height, msg.step)[:, :msg.width * channels]
            frame = frame.reshape(msg.height, msg.width, channels)
            if msg.encoding.lower() in ('bgr8', 'bgra8'):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB if channels == 4 else cv2.COLOR_BGR2RGB)
            elif channels == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
            elif channels == 1:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            with self._lock:
                self._rgb = frame.copy()
        except (ValueError, cv2.error):
            return

    def _on_depth(self, msg: RosImage) -> None:
        now = time.monotonic()
        if now - self._last_depth_time < 0.10:
            return
        self._last_depth_time = now
        try:
            if msg.encoding.upper() in ('16UC1', 'MONO16'):
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
            elif msg.encoding.upper() == '32FC1':
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
                depth = depth * 1000.0
            else:
                return
            with self._lock:
                self._depth = depth.copy()
        except ValueError:
            return

    def _on_angles(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            with self._lock:
                self._angles = [float(v) if v >= 0 else None for v in msg.data[:6]]

    def snapshot(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        with self._lock:
            return (None if self._rgb is None else self._rgb.copy(),
                    None if self._depth is None else self._depth.copy())

    def angles(self) -> list[float | None]:
        with self._lock:
            return self._angles.copy()

    def torque(self, enabled: bool) -> None:
        self._torque.publish(Bool(data=enabled))

    def move(self, pose: list[float], milliseconds: int) -> None:
        self._move.publish(Float32MultiArray(
            data=[*map(float, pose[:6]), float(max(100, min(30000, milliseconds)))])
        )


class CameraCoordinateGui:
    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = RosInterface()
        self.root = tk.Tk()
        self.root.title('DOFBOT 카메라 좌표 Pick & Place')
        self.root.resizable(False, False)
        self._set_font()
        self._photo = None
        self._frame_size = (640, 480)
        self._selected: tuple[float, float, float] | None = None
        self.pick: tuple[float, float, float] | None = None
        self.drop: tuple[float, float, float] | None = None
        self.samples: list[dict[str, object]] = []
        self.status = tk.StringVar(value='카메라 영상을 기다리는 중입니다.')
        self.point_text = tk.StringVar(value='선택 좌표: —')
        self.pick_text = tk.StringVar(value='픽: —')
        self.drop_text = tk.StringVar(value='드롭: —')
        self.sample_text = tk.StringVar(value='보정점 0개 (최소 3개 필요)')
        self.move_seconds = tk.DoubleVar(value=2.5)
        self.open_angle = tk.DoubleVar(value=90.0)
        self.closed_angle = tk.DoubleVar(value=30.0)
        self._playing = False
        self._after_ids: list[str] = []
        self._build()
        self._load()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        threading.Thread(target=self._spin, daemon=True).start()
        self._update_camera()

    def _set_font(self) -> None:
        for name in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont'):
            try:
                tkfont.nametofont(name).configure(family='Noto Sans CJK KR')
            except tk.TclError:
                pass

    def _build(self) -> None:
        outer = ttk.Frame(self.root, padding=12)
        outer.grid()
        left = ttk.Frame(outer)
        left.grid(row=0, column=0, sticky='n')
        self.canvas = tk.Canvas(left, width=640, height=480, bg='#202020', cursor='crosshair')
        self.canvas.pack()
        self._image_item = self.canvas.create_image(0, 0, anchor='nw')
        self.canvas.bind('<Button-1>', self._click)
        ttk.Label(left, textvariable=self.point_text, font=('', 11, 'bold')).pack(anchor='w', pady=(6, 0))

        side = ttk.Frame(outer, padding=(12, 0, 0, 0))
        side.grid(row=0, column=1, sticky='n')
        ttk.Label(side, text='카메라 좌표 제어', font=('', 15, 'bold')).pack(anchor='w')
        ttk.Label(side, text='1. 영상 클릭 → 2. 기준 자세 기록 → 3. 픽/드롭 실행',
                  wraplength=390).pack(anchor='w', pady=(2, 10))

        select = ttk.LabelFrame(side, text='선택 좌표', padding=8)
        select.pack(fill=tk.X)
        ttk.Button(select, text='픽 좌표로 지정', command=self._set_pick).pack(side=tk.LEFT)
        ttk.Button(select, text='드롭 좌표로 지정', command=self._set_drop).pack(side=tk.LEFT, padx=5)
        ttk.Label(side, textvariable=self.pick_text).pack(anchor='w', pady=(8, 0))
        ttk.Label(side, textvariable=self.drop_text).pack(anchor='w')

        calibration = ttk.LabelFrame(side, text='작업 평면 보정', padding=8)
        calibration.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(calibration, text='각 기준점에서 위 자세와 아래 자세를 기록하세요.',
                  wraplength=360).pack(anchor='w')
        buttons = ttk.Frame(calibration)
        buttons.pack(fill=tk.X, pady=5)
        ttk.Button(buttons, text='Torque OFF', command=self._torque_off).pack(side=tk.LEFT)
        ttk.Button(buttons, text='Torque ON', command=self._torque_on).pack(side=tk.LEFT, padx=4)
        ttk.Button(calibration, text='새 보정점: 현재 자세를 위로 기록',
                   command=lambda: self._record('above')).pack(fill=tk.X, pady=2)
        ttk.Button(calibration, text='가장 가까운 보정점: 현재 자세를 아래로 기록',
                   command=lambda: self._record('down')).pack(fill=tk.X, pady=2)
        ttk.Label(calibration, textvariable=self.sample_text).pack(anchor='w', pady=(5, 0))
        ttk.Button(calibration, text='보정 전체 삭제', command=self._clear_samples).pack(anchor='w', pady=(5, 0))

        settings = ttk.LabelFrame(side, text='동작 설정', padding=8)
        settings.pack(fill=tk.X, pady=(10, 0))
        self._spinbox_row(settings, '구간 이동시간(초)', self.move_seconds, 0.5, 10, 0.5)
        self._spinbox_row(settings, '그리퍼 열림 각도', self.open_angle, 0, 180, 1)
        self._spinbox_row(settings, '그리퍼 닫힘 각도', self.closed_angle, 0, 180, 1)
        run = ttk.Frame(side)
        run.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(run, text='픽 → 드롭 실행', command=self._run).pack(side=tk.LEFT)
        ttk.Button(run, text='정지', command=self._stop).pack(side=tk.LEFT, padx=5)
        ttk.Label(side, textvariable=self.status, wraplength=390).pack(anchor='w', pady=(10, 0))

    @staticmethod
    def _spinbox_row(parent, label, variable, low, high, step) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=label, width=20).pack(side=tk.LEFT)
        tk.Spinbox(row, from_=low, to=high, increment=step, width=7,
                   textvariable=variable).pack(side=tk.LEFT)

    def _spin(self) -> None:
        try:
            rclpy.spin(self.node)
        except ExternalShutdownException:
            pass

    def _click(self, event) -> None:
        rgb, depth = self.node.snapshot()
        if rgb is None:
            self.status.set('RGB 영상이 아직 없습니다.')
            return
        sx = rgb.shape[1] / 640.0
        sy = rgb.shape[0] / 480.0
        u = max(0, min(rgb.shape[1] - 1, round(event.x * sx)))
        v = max(0, min(rgb.shape[0] - 1, round(event.y * sy)))
        z = 0.0
        if depth is not None:
            du = min(depth.shape[1] - 1, round(u * depth.shape[1] / rgb.shape[1]))
            dv = min(depth.shape[0] - 1, round(v * depth.shape[0] / rgb.shape[0]))
            patch = depth[max(0, dv - 2):dv + 3, max(0, du - 2):du + 3]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size:
                z = float(np.median(valid))
        self._selected = (float(u), float(v), z)
        self.point_text.set(f'선택 좌표: u={u}, v={v}, depth={z:.0f} mm')

    @staticmethod
    def _fmt(point) -> str:
        return '—' if point is None else f'u={point[0]:.0f}, v={point[1]:.0f}, depth={point[2]:.0f} mm'

    def _set_pick(self) -> None:
        if self._selected is not None:
            self.pick = self._selected
            self.pick_text.set('픽: ' + self._fmt(self.pick))

    def _set_drop(self) -> None:
        if self._selected is not None:
            self.drop = self._selected
            self.drop_text.set('드롭: ' + self._fmt(self.drop))

    def _read_pose(self) -> list[float] | None:
        pose = self.node.angles()
        if any(v is None for v in pose):
            return None
        return [float(v) for v in pose]

    def _record(self, kind: str) -> None:
        if self._selected is None:
            self.status.set('먼저 카메라 영상을 클릭하세요.')
            return
        pose = self._read_pose()
        if pose is None:
            self.status.set('/arm/joint_angles 현재값을 읽지 못했습니다.')
            return
        u, v, depth = self._selected
        if kind == 'above':
            self.samples.append({'u': u, 'v': v, 'depth': depth, 'above': pose, 'down': None})
            self.status.set('새 보정점의 위 자세를 기록했습니다. 같은 위치에서 아래 자세도 기록하세요.')
        else:
            candidates = [s for s in self.samples if s.get('down') is None]
            if not candidates:
                self.status.set('먼저 새 보정점의 위 자세를 기록하세요.')
                return
            sample = min(candidates, key=lambda s: (float(s['u']) - u) ** 2 + (float(s['v']) - v) ** 2)
            sample['down'] = pose
            self.status.set('보정점의 아래 자세를 기록했습니다.')
        self._save()
        self._update_sample_text()

    def _complete(self) -> list[dict[str, object]]:
        return [s for s in self.samples if s.get('above') is not None and s.get('down') is not None]

    def _inside_workspace(self, point) -> bool:
        complete = self._complete()
        if len(complete) < 3:
            return False
        cloud = np.array([[s['u'], s['v']] for s in complete], dtype=np.float32)
        hull = cv2.convexHull(cloud)
        return cv2.pointPolygonTest(hull, (float(point[0]), float(point[1])), False) >= 0

    def _interpolate(self, point, kind: str) -> list[float]:
        weighted = [0.0] * 6
        total = 0.0
        for sample in self._complete():
            distance = math.hypot(float(sample['u']) - point[0], float(sample['v']) - point[1])
            weight = 1.0 / max(4.0, distance) ** 2
            pose = sample[kind]
            for i in range(6):
                weighted[i] += weight * float(pose[i])
            total += weight
        return [value / total for value in weighted]

    def _run(self) -> None:
        if self.pick is None or self.drop is None:
            self.status.set('픽 좌표와 드롭 좌표를 모두 지정하세요.')
            return
        if len(self._complete()) < 3:
            self.status.set('완료된 보정점이 최소 3개 필요합니다.')
            return
        if not self._inside_workspace(self.pick) or not self._inside_workspace(self.drop):
            self.status.set('픽/드롭 좌표는 보정점들이 둘러싼 안전 영역 안에 있어야 합니다.')
            return
        pick_up = self._interpolate(self.pick, 'above')
        pick_down = self._interpolate(self.pick, 'down')
        drop_up = self._interpolate(self.drop, 'above')
        drop_down = self._interpolate(self.drop, 'down')
        opened, closed = float(self.open_angle.get()), float(self.closed_angle.get())
        for pose in (pick_up, pick_down, drop_up, drop_down):
            if any(v < 0 or v > (270 if i == 4 else 180) for i, v in enumerate(pose)):
                self.status.set('보간된 자세가 관절 안전 범위를 벗어났습니다.')
                return
        pick_up[5] = opened; pick_down[5] = opened
        pick_closed = pick_down.copy(); pick_closed[5] = closed
        lift_closed = pick_up.copy(); lift_closed[5] = closed
        drop_up[5] = closed; drop_down[5] = closed
        released = drop_down.copy(); released[5] = opened
        final_up = drop_up.copy(); final_up[5] = opened
        sequence = [pick_up, pick_down, pick_closed, lift_closed,
                    drop_up, drop_down, released, final_up]
        seconds = max(0.5, min(10.0, float(self.move_seconds.get())))
        self._stop(show=False)
        self.node.torque(True)
        self._playing = True
        for index, pose in enumerate(sequence):
            delay = round(index * (seconds + 0.35) * 1000)
            after_id = self.root.after(delay, self._send_step, index, pose, seconds)
            self._after_ids.append(after_id)
        self._after_ids.append(self.root.after(
            round(len(sequence) * (seconds + 0.35) * 1000), self._finish))
        self.status.set('카메라 좌표 Pick & Place를 실행합니다.')

    def _send_step(self, index, pose, seconds) -> None:
        if self._playing:
            self.node.move(pose, round(seconds * 1000))
            self.status.set(f'동작 {index + 1}/8 실행 중')

    def _finish(self) -> None:
        self._playing = False
        self._after_ids.clear()
        self.status.set('Pick & Place 완료')

    def _stop(self, show=True) -> None:
        self._playing = False
        for after_id in self._after_ids:
            try:
                self.root.after_cancel(after_id)
            except tk.TclError:
                pass
        self._after_ids.clear()
        pose = self._read_pose()
        if pose is not None:
            self.node.move(pose, 100)
        if show:
            self.status.set('정지했습니다. 현재 자세를 유지합니다.')

    def _torque_off(self) -> None:
        self._stop(show=False)
        self.node.torque(False)
        self.status.set('Torque OFF: 기준 위치로 로봇을 손으로 이동하세요.')

    def _torque_on(self) -> None:
        self.node.torque(True)
        self.status.set('Torque ON')

    def _save(self) -> None:
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        CALIBRATION_FILE.write_text(json.dumps({'samples': self.samples}, ensure_ascii=False, indent=2) + '\n')

    def _load(self) -> None:
        try:
            data = json.loads(CALIBRATION_FILE.read_text())
            self.samples = list(data.get('samples', []))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            self.samples = []
        self._update_sample_text()

    def _clear_samples(self) -> None:
        self.samples.clear()
        self._save()
        self._update_sample_text()
        self.status.set('보정점을 모두 삭제했습니다.')

    def _update_sample_text(self) -> None:
        self.sample_text.set(f'완료 보정점 {len(self._complete())}개 / 전체 {len(self.samples)}개 (최소 3개)')

    def _update_camera(self) -> None:
        rgb, _ = self.node.snapshot()
        if rgb is not None:
            shown = cv2.resize(rgb, (640, 480), interpolation=cv2.INTER_AREA)
            for sample in self.samples:
                x = round(float(sample['u']) * 640 / rgb.shape[1])
                y = round(float(sample['v']) * 480 / rgb.shape[0])
                cv2.circle(shown, (x, y), 6, (0, 255, 0), 2)
            for point, color in ((self.pick, (255, 60, 60)), (self.drop, (60, 160, 255))):
                if point is not None:
                    x = round(point[0] * 640 / rgb.shape[1]); y = round(point[1] * 480 / rgb.shape[0])
                    cv2.drawMarker(shown, (x, y), color, cv2.MARKER_CROSS, 18, 2)
            self._photo = ImageTk.PhotoImage(Image.fromarray(shown))
            self.canvas.itemconfigure(self._image_item, image=self._photo)
        self.root.after(200, self._update_camera)

    def _close(self) -> None:
        self._playing = False
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    CameraCoordinateGui().run()
