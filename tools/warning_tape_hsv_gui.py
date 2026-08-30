#!/usr/bin/env python3
"""Interactive HSV mask tuner for a vehicle's raw ROS camera image."""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QDoubleSpinBox,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


DEFAULTS = {
    "h_min": 15,
    "h_max": 42,
    "s_min": 90,
    "s_max": 255,
    "v_min": 70,
    "v_max": 255,
    "r_min": 0,
    "r_max": 255,
    "g_min": 0,
    "g_max": 255,
    "b_min": 0,
    "b_max": 255,
    "open_kernel": 3,
    "close_kernel": 1,
    "min_component_pixels": 200,
}

DOCK_END_DEFAULTS = {
    **DEFAULTS,
    # H min > H max intentionally means hue wrap:
    # 168..179 OR 0..12, the two OpenCV red ranges.
    "h_min": 168,
    "h_max": 12,
    "s_min": 120,
    "v_min": 65,
    "open_kernel": 1,
    "min_component_pixels": 30,
}


def filtered_mask(frame, values):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    lower = np.asarray(
        (values["h_min"], values["s_min"], values["v_min"]),
        dtype=np.uint8,
    )
    upper = np.asarray(
        (values["h_max"], values["s_max"], values["v_max"]),
        dtype=np.uint8,
    )
    if values["h_min"] <= values["h_max"]:
        hsv_mask = cv2.inRange(hsv, lower, upper)
    else:
        hsv_mask = cv2.bitwise_or(
            cv2.inRange(
                hsv,
                np.asarray((0, values["s_min"], values["v_min"]), np.uint8),
                np.asarray((values["h_max"], values["s_max"], values["v_max"]), np.uint8),
            ),
            cv2.inRange(
                hsv,
                np.asarray((values["h_min"], values["s_min"], values["v_min"]), np.uint8),
                np.asarray((179, values["s_max"], values["v_max"]), np.uint8),
            ),
        )
    rgb_mask = cv2.inRange(
        rgb,
        np.asarray(
            (values["r_min"], values["g_min"], values["b_min"]),
            dtype=np.uint8,
        ),
        np.asarray(
            (values["r_max"], values["g_max"], values["b_max"]),
            dtype=np.uint8,
        ),
    )
    mode = values.get("filter_mode", "HSV")
    if mode == "RGB":
        mask = rgb_mask
    elif mode == "HSV_AND_RGB":
        mask = cv2.bitwise_and(hsv_mask, rgb_mask)
    elif mode == "HSV_OR_RGB":
        mask = cv2.bitwise_or(hsv_mask, rgb_mask)
    else:
        mask = hsv_mask
    for operation, key in (
        (cv2.MORPH_OPEN, "open_kernel"),
        (cv2.MORPH_CLOSE, "close_kernel"),
    ):
        size = int(values[key])
        if size > 1:
            if size % 2 == 0:
                size += 1
            mask = cv2.morphologyEx(
                mask, operation, np.ones((size, size), dtype=np.uint8)
            )
    minimum_area = int(values.get("min_component_pixels", 0))
    if minimum_area > 1:
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        filtered = np.zeros_like(mask)
        for label in range(1, count):
            if int(stats[label, cv2.CC_STAT_AREA]) >= minimum_area:
                filtered[labels == label] = 255
        mask = filtered
    return hsv, mask


class RawCameraNode(Node):
    def __init__(self, topic, cmd_vel_topic):
        super().__init__("warning_tape_hsv_gui")
        self.bridge = CvBridge()
        self.frame = None
        self.frame_stamp = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Image, topic, self.on_image, qos)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

    def on_image(self, message):
        self.frame = self.bridge.imgmsg_to_cv2(
            message, desired_encoding="bgr8"
        )
        self.frame_stamp = time.monotonic()

    def drive(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        message = Twist()
        message.linear.x = float(linear_x)
        message.linear.y = float(linear_y)
        message.angular.z = float(angular_z)
        self.cmd_pub.publish(message)

    def stop(self, repeats=1):
        for _index in range(repeats):
            self.drive()


class ImageView(QLabel):
    def __init__(self, title):
        super().__init__(title)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("background:#111; color:#ddd; border:1px solid #444")
        self.source = None

    def set_bgr(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.source = QImage(
            rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0],
            QImage.Format_RGB888,
        ).copy()
        self.refresh()

    def set_mask(self, mask):
        self.source = QImage(
            mask.data, mask.shape[1], mask.shape[0], mask.strides[0],
            QImage.Format_Grayscale8,
        ).copy()
        self.refresh()

    def refresh(self):
        if self.source is None:
            return
        self.setPixmap(QPixmap.fromImage(self.source).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.refresh()


class HsvTuner(QMainWindow):
    MOVEMENT_KEYS = {Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D, Qt.Key_Q, Qt.Key_E}

    def __init__(self, node, args):
        super().__init__()
        self.node = node
        self.args = args
        self.defaults = (
            DOCK_END_DEFAULTS if args.target == "dock-end" else DEFAULTS
        )
        self.config_path = Path(args.config).expanduser()
        self.latest_frame = None
        self.controls = {}
        self.pressed = set()
        self.drive_was_active = False
        self.setFocusPolicy(Qt.StrongFocus)
        QApplication.instance().installEventFilter(self)
        target_title = "DOCK END" if args.target == "dock-end" else "WARNING TAPE"
        self.setWindowTitle(f"Vehicle {args.vehicle} {target_title} colour tuner")

        root = QWidget()
        layout = QVBoxLayout(root)
        views = QHBoxLayout()
        self.original = ImageView("RAW CAMERA")
        self.mask = ImageView("HSV MASK")
        self.masked = ImageView("MASKED RESULT")
        views.addWidget(self.original)
        views.addWidget(self.mask)
        views.addWidget(self.masked)
        layout.addLayout(views, 1)

        controls_row = QHBoxLayout()
        form = QFormLayout()
        specs = (
            ("h_min", "H min", 0, 179),
            ("h_max", "H max", 0, 179),
            ("s_min", "S min", 0, 255),
            ("s_max", "S max", 0, 255),
            ("v_min", "V min", 0, 255),
            ("v_max", "V max", 0, 255),
            ("r_min", "R min", 0, 255),
            ("r_max", "R max", 0, 255),
            ("g_min", "G min", 0, 255),
            ("g_max", "G max", 0, 255),
            ("b_min", "B min", 0, 255),
            ("b_max", "B max", 0, 255),
            ("open_kernel", "Open kernel", 1, 21),
            ("close_kernel", "Close kernel", 1, 21),
            ("min_component_pixels", "Min component px", 0, 5000),
        )
        for key, label, minimum, maximum in specs:
            row = QHBoxLayout()
            slider = QSlider(Qt.Horizontal)
            slider.setRange(minimum, maximum)
            spin = QSpinBox()
            spin.setRange(minimum, maximum)
            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)
            slider.valueChanged.connect(self.render)
            row.addWidget(slider, 1)
            row.addWidget(spin)
            form.addRow(label, row)
            self.controls[key] = (slider, spin)
        controls_row.addLayout(form, 1)

        buttons = QVBoxLayout()
        self.status = QLabel(
            "Waiting for raw camera frame\n"
            "Drive: W/S forward/reverse · A/D left/right · Q/E rotate · Space stop"
        )
        self.status.setWordWrap(True)
        self.linear_speed = QDoubleSpinBox()
        self.linear_speed.setRange(0.10, 0.30)
        self.linear_speed.setSingleStep(0.01)
        self.linear_speed.setValue(args.linear_speed)
        self.linear_speed.setSuffix(" m/s")
        self.angular_speed = QDoubleSpinBox()
        self.angular_speed.setRange(0.05, 2.0)
        self.angular_speed.setSingleStep(0.05)
        self.angular_speed.setValue(args.angular_speed)
        self.angular_speed.setSuffix(" rad/s")
        self.filter_mode = QComboBox()
        self.filter_mode.addItem("HSV only", "HSV")
        self.filter_mode.addItem("RGB only", "RGB")
        self.filter_mode.addItem("HSV AND RGB", "HSV_AND_RGB")
        self.filter_mode.addItem("HSV OR RGB", "HSV_OR_RGB")
        self.filter_mode.currentIndexChanged.connect(self.render)
        save = QPushButton("Save HSV JSON")
        load = QPushButton("Load HSV JSON")
        reset = QPushButton(
            "Reset red defaults" if args.target == "dock-end"
            else "Reset yellow defaults"
        )
        shot = QPushButton("Save raw + mask")
        save.clicked.connect(self.save_config)
        load.clicked.connect(self.load_config)
        reset.clicked.connect(lambda: self.set_values(self.defaults))
        shot.clicked.connect(self.save_snapshot)
        buttons.addWidget(self.status)
        buttons.addWidget(QLabel("Mask mode"))
        buttons.addWidget(self.filter_mode)
        buttons.addWidget(QLabel("Linear speed"))
        buttons.addWidget(self.linear_speed)
        buttons.addWidget(QLabel("Angular speed"))
        buttons.addWidget(self.angular_speed)
        buttons.addWidget(save)
        buttons.addWidget(load)
        buttons.addWidget(reset)
        buttons.addWidget(shot)
        buttons.addStretch(1)
        controls_row.addLayout(buttons, 1)
        layout.addLayout(controls_row)
        self.setCentralWidget(root)
        self.set_values(self.defaults)
        if self.config_path.exists():
            self.load_config()

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.poll_ros)
        self.ros_timer.start(10)
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.take_latest_frame)
        self.render_timer.start(50)
        self.drive_timer = QTimer(self)
        self.drive_timer.timeout.connect(self.publish_drive)
        self.drive_timer.start(50)
        self.resize(1400, 720)

    def values(self):
        return {
            **{key: pair[1].value() for key, pair in self.controls.items()},
            "filter_mode": self.filter_mode.currentData(),
        }

    def set_values(self, values):
        for key, default in self.defaults.items():
            self.controls[key][1].setValue(int(values.get(key, default)))
        mode = str(values.get("filter_mode", "HSV"))
        mode_index = self.filter_mode.findData(mode)
        self.filter_mode.setCurrentIndex(max(0, mode_index))
        self.render()

    def poll_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish_drive(self):
        active = bool(self.pressed & self.MOVEMENT_KEYS)
        if not active:
            if self.drive_was_active:
                self.node.stop(3)
                self.drive_was_active = False
            return
        linear = self.linear_speed.value()
        angular = self.angular_speed.value()
        self.node.drive(
            linear * (
                int(Qt.Key_W in self.pressed) - int(Qt.Key_S in self.pressed)
            ),
            linear * (
                int(Qt.Key_A in self.pressed) - int(Qt.Key_D in self.pressed)
            ),
            angular * (
                int(Qt.Key_Q in self.pressed) - int(Qt.Key_E in self.pressed)
            ),
        )
        self.drive_was_active = True

    def emergency_stop(self):
        self.pressed.clear()
        self.drive_was_active = False
        self.node.stop(5)

    def eventFilter(self, watched, event):
        if event.type() == QEvent.WindowDeactivate:
            self.emergency_stop()
        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Space:
                self.emergency_stop()
                return True
            if event.key() in self.MOVEMENT_KEYS:
                self.pressed.add(event.key())
                self.publish_drive()
                return True
        if (
            event.type() == QEvent.KeyRelease
            and event.key() in self.MOVEMENT_KEYS
            and not event.isAutoRepeat()
        ):
            self.pressed.discard(event.key())
            self.publish_drive()
            return True
        return super().eventFilter(watched, event)

    def focusOutEvent(self, event):
        if self.pressed:
            self.emergency_stop()
        super().focusOutEvent(event)

    def take_latest_frame(self):
        if self.node.frame is None:
            return
        self.latest_frame = self.node.frame.copy()
        self.render()

    def render(self, *_args):
        if self.latest_frame is None:
            return
        values = self.values()
        if (
            values["s_min"] > values["s_max"]
            or values["v_min"] > values["v_max"]
            or values["r_min"] > values["r_max"]
            or values["g_min"] > values["g_max"]
            or values["b_min"] > values["b_max"]
        ):
            self.status.setText(
                "Invalid range (H may wrap; other min values must be <= max)"
            )
            return
        hsv, mask = filtered_mask(self.latest_frame, values)
        masked = cv2.bitwise_and(self.latest_frame, self.latest_frame, mask=mask)
        self.original.set_bgr(self.latest_frame)
        self.mask.set_mask(mask)
        self.masked.set_bgr(masked)
        selected = cv2.countNonZero(mask)
        total = max(mask.size, 1)
        center_hsv = hsv[hsv.shape[0] // 2, hsv.shape[1] // 2].tolist()
        center_bgr = self.latest_frame[
            self.latest_frame.shape[0] // 2,
            self.latest_frame.shape[1] // 2,
        ].tolist()
        center_rgb = center_bgr[::-1]
        age = time.monotonic() - self.node.frame_stamp
        self.status.setText(
            f"topic: {self.args.topic}\n"
            f"selected: {selected:,} px ({100.0 * selected / total:.2f}%)\n"
            f"center HSV: {center_hsv} · RGB: {center_rgb}\n"
            f"mode: {values['filter_mode']} · frame age: {age:.3f}s\n"
            f"config: {self.config_path}"
        )

    def save_config(self):
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "vehicle": self.args.vehicle,
            "topic": self.args.topic,
            **self.values(),
        }
        self.config_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self.status.setText(f"Saved {self.config_path}")

    def load_config(self):
        try:
            payload = json.loads(self.config_path.read_text(encoding="utf-8"))
            self.set_values(payload)
            self.status.setText(f"Loaded {self.config_path}")
        except Exception as exc:
            self.status.setText(f"Load failed: {exc}")

    def save_snapshot(self):
        if self.latest_frame is None:
            self.status.setText("No camera frame to save")
            return
        _hsv, mask = filtered_mask(self.latest_frame, self.values())
        output = Path(self.args.output_dir).expanduser()
        output.mkdir(parents=True, exist_ok=True)
        stem = time.strftime(
            ("dock_end_hsv_" if self.args.target == "dock-end" else
             "warning_tape_hsv_") + "%Y%m%d_%H%M%S"
        )
        raw_path = output / f"{stem}_raw.jpg"
        mask_path = output / f"{stem}_mask.png"
        cv2.imwrite(str(raw_path), self.latest_frame)
        cv2.imwrite(str(mask_path), mask)
        self.status.setText(f"Saved {raw_path.name} + {mask_path.name}")

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.render_timer.stop()
        self.drive_timer.stop()
        self.emergency_stop()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        super().closeEvent(event)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--target", choices=("warning-tape", "dock-end"),
        default="warning-tape",
    )
    parser.add_argument("--vehicle", type=int, choices=(1, 2), default=2)
    parser.add_argument(
        "--topic", default="/ascamera/camera_publisher/rgb0/image"
    )
    parser.add_argument("--cmd-vel-topic", default="/controller/cmd_vel")
    parser.add_argument("--linear-speed", type=float, default=0.12)
    parser.add_argument("--angular-speed", type=float, default=0.35)
    parser.add_argument("--config")
    parser.add_argument("--output-dir")
    args = parser.parse_args()
    if args.config is None:
        args.config = (
            "~/dock_end_hsv.json" if args.target == "dock-end"
            else "~/warning_tape_hsv.json"
        )
    if args.output_dir is None:
        args.output_dir = (
            "~/dock_end_hsv_captures" if args.target == "dock-end"
            else "~/warning_tape_hsv_captures"
        )
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = RawCameraNode(args.topic, args.cmd_vel_topic)
    app = QApplication(sys.argv)
    window = HsvTuner(node, args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
