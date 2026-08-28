"""Desktop preview for the local DOFBOT camera."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import threading
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node


class CameraGui(Node):
    def __init__(self) -> None:
        super().__init__('camera_gui')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('snapshot_dir', '/home/intelions/Pictures')
        self._device = self.get_parameter('device').value
        self._capture = None
        self._last_frame = None
        self._lock = threading.Lock()
        self._running = True
        self._photo = None
        self._camera_state = 'Opening camera…'

        self.root = tk.Tk()
        self.root.title('DOFBOT camera')
        self.root.resizable(False, False)
        frame = ttk.Frame(self.root, padding=12)
        frame.grid()
        self.image_label = ttk.Label(frame, text='Opening camera…', anchor='center', width=80)
        self.image_label.grid(row=0, column=0, columnspan=2)
        self.status = tk.StringVar(value=f'Camera: {self._device}')
        ttk.Label(frame, textvariable=self.status).grid(row=1, column=0, sticky='w', pady=(8, 0))
        ttk.Button(frame, text='Save snapshot', command=self._save_snapshot).grid(
            row=1, column=1, sticky='e', pady=(8, 0)
        )
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        threading.Thread(target=self._camera_worker, daemon=True).start()
        self._update_window()

    def _camera_worker(self) -> None:
        capture = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not capture.isOpened():
            self._camera_state = f'Could not open {self._device}. Check camera connection.'
            capture.release()
            return
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('width').value)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('height').value)
        self._capture = capture
        self._camera_state = 'Waiting for camera frames…'
        while self._running:
            ok, frame = capture.read()
            if not ok:
                self._camera_state = 'No frame received from camera.'
                continue
            with self._lock:
                self._last_frame = frame
            self._camera_state = f'{frame.shape[1]} × {frame.shape[0]}  |  live'
        capture.release()

    def _update_window(self) -> None:
        with self._lock:
            frame = None if self._last_frame is None else self._last_frame.copy()
        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self._photo = ImageTk.PhotoImage(Image.fromarray(rgb))
            self.image_label.configure(image=self._photo, text='')
        else:
            self.image_label.configure(text=self._camera_state)
        self.status.set(self._camera_state)
        if self._running:
            self.root.after(50, self._update_window)

    def _save_snapshot(self) -> None:
        with self._lock:
            frame = None if self._last_frame is None else self._last_frame.copy()
        if frame is None:
            self.status.set('No frame available to save.')
            return
        directory = Path(self.get_parameter('snapshot_dir').value)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"dofbot_{datetime.now():%Y%m%d_%H%M%S}.png"
        self.status.set(f'Saved: {path}' if cv2.imwrite(str(path), frame) else 'Could not save snapshot.')

    def _close(self) -> None:
        self._running = False
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
