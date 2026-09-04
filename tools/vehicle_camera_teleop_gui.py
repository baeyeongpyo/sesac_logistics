#!/usr/bin/env python3
"""Qt dual-camera viewer with dead-man teleop and local recording."""

import argparse
import json
import math
import os
import queue
import shlex
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

# Select the vehicle DDS domain before rclpy is imported.
_bootstrap_parser = argparse.ArgumentParser(add_help=False)
_bootstrap_parser.add_argument("--vehicle", type=int, choices=(1, 2))
_bootstrap_parser.add_argument("--ros-domain-id", type=int, default=None)
_bootstrap_args, _ = _bootstrap_parser.parse_known_args()
_bootstrap_domain = _bootstrap_args.ros_domain_id
if _bootstrap_domain is None and _bootstrap_args.vehicle in (1, 2):
    _bootstrap_domain = 214 + _bootstrap_args.vehicle
if _bootstrap_domain is not None:
    os.environ["ROS_DOMAIN_ID"] = str(_bootstrap_domain)

# The vehicle container defaults to the POSIX locale, which makes Qt render
# Korean UI text and notes incorrectly even though metadata is written as UTF-8.
for locale_variable in ("LANG", "LC_ALL"):
    if not os.environ.get(locale_variable):
        os.environ[locale_variable] = "C.UTF-8"

# Match the vehicle runtime before importing rclpy, which selects its RMW
# implementation during import on some ROS 2 distributions.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

# Allow this single file to be launched from a plain Ubuntu terminal without
# making the operator source ROS manually first.
if not os.environ.get("VEHICLE_GUI_ROS_READY"):
    ros_setup = next(
        (path for path in (Path("/opt/ros/humble/setup.bash"), Path("/opt/ros/jazzy/setup.bash")) if path.exists()),
        None,
    )
    if ros_setup is not None:
        workspace_setup = Path("/home/ubuntu/ros2_ws/install/setup.bash")
        setup_commands = [f"source {shlex.quote(str(ros_setup))}"]
        if workspace_setup.exists():
            setup_commands.append(f"source {shlex.quote(str(workspace_setup))}")
        environment = os.environ.copy()
        environment["VEHICLE_GUI_ROS_READY"] = "1"
        if _bootstrap_domain is not None:
            environment["ROS_DOMAIN_ID"] = str(_bootstrap_domain)
        # Conda's Python and libstdc++ are incompatible with the system ROS 2
        # extension modules.  Prefer the OS Python whenever the GUI is started
        # from an activated Conda shell.
        python_executable = "/usr/bin/python3" if environment.get("CONDA_PREFIX") else sys.executable
        if python_executable == "/usr/bin/python3":
            environment.pop("LD_LIBRARY_PATH", None)
        os.execve(
            "/bin/bash",
            [
                "bash", "-lc",
                f'{" && ".join(setup_commands)} && exec "$@"',
                "vehicle-gui", python_executable, str(Path(__file__).resolve()), *sys.argv[1:],
            ],
            environment,
        )

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from python_qt_binding.QtCore import QEvent, QPointF, QRect, Qt, QTimer
from python_qt_binding.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from python_qt_binding.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QGridLayout, QGroupBox, QHBoxLayout,
    QDoubleSpinBox, QLabel, QListWidget, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSlider,
    QVBoxLayout, QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_robot_controller_msgs.msg import MotorsState
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String, UInt16
from auto_dock.auto_dock_node import detect_y_slot_x


VEHICLE_HOSTS = {1: "192.168.100.38", 2: "192.168.100.35"}
DETECTION_COLORS = {
    "star": (30, 220, 255),
    "diamond": (255, 120, 30),
    "pallet": (40, 210, 40),
    "spade": (230, 80, 230),
    "clover": (50, 180, 50),
    "heart": (40, 40, 240),
}


def detect_warning_tape_debug(
    frame, minimum_yellow_pixels=600, minimum_center_y_ratio=None,
    filter_config=None,
):
    """Run the auto-dock tape detector and retain its rejection evidence."""
    height, width = frame.shape[:2]
    values = filter_config if isinstance(filter_config, dict) else {}
    if minimum_center_y_ratio is None:
        minimum_center_y_ratio = values.get("roi_top_ratio")
    roi_top = 0 if minimum_center_y_ratio is None else int(round(
        max(0.0, min(0.95, float(minimum_center_y_ratio))) * height
    ))
    roi = frame[roi_top:, :]
    roi_height = roi.shape[0]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    raw_mask = cv2.inRange(
        hsv,
        np.asarray((
            int(values.get("h_min", 15)),
            int(values.get("s_min", 90)),
            int(values.get("v_min", 70)),
        ), dtype=np.uint8),
        np.asarray((
            int(values.get("h_max", 42)),
            int(values.get("s_max", 255)),
            int(values.get("v_max", 255)),
        ), dtype=np.uint8),
    )
    yellow = raw_mask
    for operation, key, default in (
        (cv2.MORPH_OPEN, "open_kernel", 3),
        (cv2.MORPH_CLOSE, "close_kernel", 1),
    ):
        kernel_size = int(values.get(key, default))
        if kernel_size > 1:
            if kernel_size % 2 == 0:
                kernel_size += 1
            yellow = cv2.morphologyEx(
                yellow, operation,
                np.ones((kernel_size, kernel_size), dtype=np.uint8),
            )
    black = cv2.inRange(
        hsv, np.asarray((0, 0, 0), dtype=np.uint8),
        np.asarray((179, 255, 75), dtype=np.uint8),
    )
    black = cv2.morphologyEx(
        black, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
    )
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        yellow, connectivity=8
    )
    candidates = []
    minimum_component_pixels = int(values.get(
        "min_component_pixels", 200 if filter_config is not None else 80
    ))
    for label in range(1, count):
        x, y, component_width, component_height, area = stats[label]
        if area < minimum_component_pixels or component_width < 12:
            continue
        side_width = max(8, int(round(component_width * 0.75)))
        y0 = max(0, y)
        y1 = min(roi_height, y + component_height)
        left_x0 = max(0, x - side_width)
        left_x1 = min(width, x + 2)
        right_x0 = max(0, x + component_width - 2)
        right_x1 = min(width, x + component_width + side_width)
        side_areas = []
        for x0, x1 in ((left_x0, left_x1), (right_x0, right_x1)):
            if x1 <= x0 or y1 <= y0:
                continue
            region = black[y0:y1, x0:x1]
            side_areas.append(
                cv2.countNonZero(region) / max(float(region.size), 1.0)
            )
        center_x, center_y = centroids[label]
        candidates.append({
            "label": label,
            "center_x": float(center_x),
            "center_y": float(center_y),
            "height": int(component_height),
            "area": int(area),
            "x0": int(x),
            "x1": int(x + component_width),
            "black_adjacent": bool(
                side_areas and max(side_areas) >= 0.15
            ),
        })
    best_group = []
    best_score = -1.0
    for first_index, first in enumerate(candidates):
        for second in candidates[first_index + 1:]:
            dx = second["center_x"] - first["center_x"]
            dy = second["center_y"] - first["center_y"]
            if abs(dx) < 12.0:
                continue
            angle_deg = math.degrees(math.atan2(dy, dx))
            if angle_deg >= 90.0:
                angle_deg -= 180.0
            if angle_deg < -90.0:
                angle_deg += 180.0
            if abs(angle_deg) > 35.0:
                continue
            norm = math.hypot(dx, dy)
            group = []
            for candidate in candidates:
                distance = abs(
                    dy * (candidate["center_x"] - first["center_x"])
                    - dx * (candidate["center_y"] - first["center_y"])
                ) / norm
                tolerance = max(14.0, min(30.0, candidate["height"] * 0.5))
                if distance <= tolerance:
                    group.append(candidate)
            ordered = sorted(group, key=lambda item: item["x0"])
            widths = [item["x1"] - item["x0"] for item in ordered]
            maximum_gap = max(
                (right["x0"] - left["x1"] for left, right in zip(
                    ordered, ordered[1:]
                )),
                default=0,
            )
            allowed_gap = max(100.0, float(np.median(widths)) * 1.5)
            if maximum_gap > allowed_gap:
                continue
            span = max(item["x1"] for item in group) - min(
                item["x0"] for item in group
            )
            area = sum(item["area"] for item in group)
            center_y = float(np.median([
                item["center_y"] for item in group
            ]))
            score = (
                span * (1.0 + 0.25 * max(0, len(group) - 2))
                + min(area, 5000) * 0.01
                + center_y * 3.0
            )
            if score > best_score:
                best_score = score
                best_group = group
    accepted = np.zeros_like(yellow)
    for candidate in best_group:
        accepted[labels == candidate["label"]] = 255
    accepted_components = len(best_group)
    black_adjacent_components = sum(
        int(candidate["black_adjacent"]) for candidate in best_group
    )
    ys, xs = np.nonzero(accepted)
    debug = {
        "detected": False,
        "reason": "",
        "roi_top": roi_top,
        "raw_yellow_pixels": int(cv2.countNonZero(raw_mask)),
        "yellow_pixels": int(len(xs)),
        "component_count": int(accepted_components),
        "black_adjacent_components": int(black_adjacent_components),
        "black_adjacent_candidates": int(len(candidates)),
        "span_px": 0 if not len(xs) else int(xs.max()) - int(xs.min()),
        "x_min_px": None if not len(xs) else int(xs.min()),
        "x_max_px": None if not len(xs) else int(xs.max()),
        "center_x_px": None if not len(xs) else float(
            0.5 * (int(xs.min()) + int(xs.max()))
        ),
        "accepted_mask": accepted,
    }
    if accepted_components < 2:
        debug["reason"] = "yellow_with_black_neighbor<2"
        return debug
    if black_adjacent_components < max(1, math.ceil(accepted_components / 2)):
        debug["reason"] = "black_neighbor_majority_failed"
        return debug
    if len(xs) < int(minimum_yellow_pixels):
        debug["reason"] = f"pixels<{int(minimum_yellow_pixels)}"
        return debug
    minimum_span = int(width * 0.25)
    if debug["span_px"] < minimum_span:
        debug["reason"] = f"span<{minimum_span}px"
        return debug
    points = np.column_stack((xs, ys)).astype(np.float32)
    vx, vy, line_x, line_y = (
        float(value) for value in cv2.fitLine(
            points, cv2.DIST_L2, 0.0, 0.01, 0.01
        ).reshape(-1)
    )
    if abs(vx) < 1e-6:
        debug["reason"] = "vertical_fit"
        return debug
    angle_deg = math.degrees(math.atan2(vy, vx))
    if angle_deg >= 90.0:
        angle_deg -= 180.0
    if angle_deg < -90.0:
        angle_deg += 180.0
    debug["angle_deg"] = float(angle_deg)
    if abs(angle_deg) > 35.0:
        debug["reason"] = "angle>35deg"
        return debug
    center_y = line_y + (vy / vx) * (width * 0.5 - line_x) + roi_top
    normal_distance = np.abs(-vy * (xs - line_x) + vx * (ys - line_y))
    band_width_px = float(np.percentile(normal_distance, 90)) * 2.0
    debug.update({
        "center_y_ratio": float(center_y / height),
        "band_width_px": round(band_width_px, 1),
        "line": (vx, vy, line_x, line_y),
    })
    if band_width_px > height * 0.18:
        debug["reason"] = f"band>{height * 0.18:.0f}px"
        return debug
    debug["detected"] = True
    debug["reason"] = "ok"
    return debug


def detect_y_slot_x_debug(frame, minimum_yellow_pixels=600, filter_config=None):
    """Run the exact AutoDock Y-slot X detector and retain overlay evidence."""
    height, width = frame.shape[:2]
    values = filter_config if isinstance(filter_config, dict) else {}
    roi_top = int(round(max(0.0, min(
        0.95, float(values.get("roi_top_ratio", 0.70))
    )) * height))
    hsv = cv2.cvtColor(frame[roi_top:, :], cv2.COLOR_BGR2HSV)
    x_s_min = int(values.get("y_slot_x_s_min", 40))
    raw_mask = cv2.inRange(
        hsv,
        np.asarray((
            int(values.get("h_min", 15)), x_s_min,
            int(values.get("v_min", 70)),
        ), dtype=np.uint8),
        np.asarray((
            int(values.get("h_max", 42)), int(values.get("s_max", 255)),
            int(values.get("v_max", 255)),
        ), dtype=np.uint8),
    )
    result = detect_y_slot_x(
        frame, minimum_yellow_pixels=minimum_yellow_pixels,
        filter_config=filter_config,
    )
    accepted = np.zeros_like(raw_mask)
    if result is not None:
        x0 = max(0, int(result["x_min_px"]))
        x1 = min(width, int(result["x_max_px"]))
        accepted[:, x0:x1] = raw_mask[:, x0:x1]
    debug = {
        "detected": result is not None,
        "reason": "ok" if result is not None else "yellow_x_shape_not_found",
        "roi_top": roi_top,
        "raw_yellow_pixels": int(cv2.countNonZero(raw_mask)),
        "yellow_pixels": 0 if result is None else int(result["yellow_pixels"]),
        "component_count": 0 if result is None else 1,
        "black_adjacent_components": 0,
        "black_adjacent_candidates": 0,
        "span_px": 0 if result is None else int(
            result["x_max_px"] - result["x_min_px"]
        ),
        "x_min_px": None if result is None else int(result["x_min_px"]),
        "x_max_px": None if result is None else int(result["x_max_px"]),
        "center_x_px": None if result is None else float(result["center_x_px"]),
        "accepted_mask": accepted,
        "line": None,
    }
    if result is not None:
        debug.update({
            "center_y_ratio": float(result["center_y_ratio"]),
            "angle_deg": 0.0,
            "x_lines": result.get("x_lines"),
            "border_angle_deg": result.get("border_angle_deg"),
            "border_line_count": int(result.get("border_line_count", 0)),
            "border_lines": result.get("border_lines"),
        })
    return debug


class DevControlClientNode(Node):
    """ROS I/O client for the optional development GUI/UI."""

    def __init__(self, args):
        super().__init__(getattr(args, "node_name", "dev_control_client"))
        self.viewer_only = args.viewer_only
        self.bridge = CvBridge()
        self.frame = None
        self.frame_sequence = 0
        self.last_frame_monotonic = 0.0
        self.tape_frame = None
        self.tape_frame_monotonic = 0.0
        self.secondary_frame = None
        self.secondary_frame_sequence = 0
        self.secondary_last_frame_monotonic = 0.0
        self.tertiary_frame = None
        self.tertiary_frame_sequence = 0
        self.tertiary_last_frame_monotonic = 0.0
        self.secondary_stream_stop = threading.Event()
        self.external_stream_threads = []
        self.control_address = (args.control_host, args.control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_base_url = args.primary_video_url.rsplit("/stream", 1)[0]
        self.front_range = math.inf
        self.closest_obstacle_angle_rad = None
        self.last_motor_active_monotonic = 0.0
        self.battery_voltage_mv = None
        self.battery_samples = []
        self.last_battery_monotonic = 0.0
        self.safety_min_valid_range = args.safety_min_valid_range
        self.lidar_self_filter_distance_m = getattr(
            args, "lidar_self_filter_distance_m", 0.25
        )
        self.last_scan_monotonic = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = None
        if getattr(args, "disable_primary_camera", False):
            pass
        elif args.primary_video_url:
            self.start_mjpeg_stream(args.primary_video_url, "primary")
        else:
            self.image_sub = self.create_subscription(Image, args.image_topic, self.on_image, qos)
        self.tape_image_sub = self.create_subscription(
            Image, args.tape_image_topic, self.on_tape_image, qos
        )
        self.secondary_image_sub = None
        if args.secondary_image_topic:
            self.secondary_image_sub = self.create_subscription(
                Image, args.secondary_image_topic, self.on_secondary_image, qos
            )
        elif not args.disable_external_webcams:
            self.start_mjpeg_stream(args.webcam_1_video_url, "secondary")
        if not args.disable_external_webcams:
            self.start_mjpeg_stream(args.webcam_2_video_url, "tertiary")
        self.scan_sub = self.create_subscription(LaserScan, args.scan_topic, self.on_scan, 10)
        self.motor_sub = self.create_subscription(
            MotorsState, args.motor_command_topic, self.on_motor_command, 50
        )
        self.battery_sub = self.create_subscription(
            UInt16, args.battery_topic, self.on_battery, 10
        )
        self.latest_detection = None
        self.latest_detection_monotonic = 0.0
        self.detection_sub = self.create_subscription(
            String, args.detection_topic, self.on_detection, 10
        )
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.fork_pub = self.create_publisher(String, args.fork_command_topic, 10)
        self.arrival_pub = self.create_publisher(String, args.arrival_topic, 10)
        self.auto_dock_stop_pub = self.create_publisher(
            Empty, args.auto_dock_stop_topic, 10
        )
        self.dock_inventory_reset_pub = self.create_publisher(
            Empty, args.dock_inventory_reset_topic, 10
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.auto_dock_status = {"state": "unknown", "reason": "no_status"}
        self.auto_dock_status_sub = self.create_subscription(
            String, args.auto_dock_status_topic, self.on_auto_dock_status, status_qos
        )
        self.latest_dock_inventory = None
        self.latest_dock_inventory_monotonic = 0.0
        self.dock_inventory_sub = self.create_subscription(
            String, args.dock_inventory_topic, self.on_dock_inventory, status_qos
        )
        self.fork_state = {"state": "UNKNOWN", "error": ""}
        self.drive_ready_count = 0
        self.fork_state_sub = self.create_subscription(
            String,
            getattr(args, "fork_state_topic", "/fork/state"),
            self.on_fork_state, 10,
        )
        self.drive_ready_sub = self.create_subscription(
            Empty,
            getattr(
                args, "drive_ready_topic",
                "/auto_dock/drive_ready",
            ),
            self.on_drive_ready, 10,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_image = None
        self.map_metadata = None
        self.map_sequence = 0
        self.entity_map = {"state": "waiting", "entities": []}
        self.entity_map_sequence = 0
        self.map_sub = self.create_subscription(
            OccupancyGrid, args.map_topic, self.on_map, map_qos
        )
        self.entity_map_sub = self.create_subscription(
            String, args.entity_map_topic, self.on_entity_map, map_qos
        )

    def publish_arrival(
        self, location="DOCK_1", operation="PICK",
        product_type="NORMAL", insertion_distance_cm=None,
        legacy_recognition=False,
    ):
        location = str(location).strip().upper()
        operation = str(operation).strip().upper()
        product_type = str(product_type).strip().upper()
        target = (
            {
                "type": "NEAREST",
                "recognition_mode": (
                    "LEGACY" if legacy_recognition else "CURRENT"
                ),
            }
            if location == "DOCK_1" and operation == "PICK"
            else {"type": "NONE"}
        )
        payload = {
            "status": "SUCCEEDED",
            "location": location,
            "operation": operation,
            "product_type": product_type,
            "target": target,
        }
        if insertion_distance_cm is not None:
            payload["insertion_distance_cm"] = float(insertion_distance_cm)
        self.arrival_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )))
        return payload

    def publish_nearest_arrival(self, product_type, legacy_recognition=False):
        payload = {
            "status": "SUCCEEDED",
            "location": "DOCK_1",
            "operation": "PICK",
            "product_type": str(product_type).strip().upper(),
            "target": {
                "type": "NEAREST",
                "recognition_mode": (
                    "LEGACY" if legacy_recognition else "CURRENT"
                ),
            },
        }
        self.arrival_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )))
        return payload

    def publish_auto_dock_stop(self):
        self.auto_dock_stop_pub.publish(Empty())

    def publish_dock_inventory_reset(self):
        self.dock_inventory_reset_pub.publish(Empty())

    def on_auto_dock_status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.auto_dock_status = status
        except (TypeError, ValueError):
            self.get_logger().warning("invalid auto_dock status JSON received")

    def on_dock_inventory(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.latest_dock_inventory = payload
                self.latest_dock_inventory_monotonic = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid dock inventory JSON received")

    def on_fork_state(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.fork_state = payload
        except (TypeError, ValueError):
            self.fork_state = {"state": "INVALID", "error": msg.data}

    def on_drive_ready(self, _msg):
        self.drive_ready_count += 1

    def on_map(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return
        occupancy = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
        grayscale = np.full((height, width), 110, dtype=np.uint8)
        known = occupancy >= 0
        grayscale[known] = np.clip(
            254.0 - occupancy[known].astype(np.float32) * 2.34, 20, 254
        ).astype(np.uint8)
        grayscale = np.flipud(grayscale)
        orientation = msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.map_image = QImage(
            grayscale.tobytes(), width, height, width, QImage.Format_Grayscale8
        ).copy()
        self.map_metadata = {
            "width": width,
            "height": height,
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "origin_yaw": origin_yaw,
            "frame_id": str(msg.header.frame_id),
        }
        self.map_sequence += 1

    def on_entity_map(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.entity_map = payload
                self.entity_map_sequence += 1
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid tag entity map JSON received")

    def on_detection(self, msg):
        try:
            self.latest_detection = json.loads(msg.data)
            self.latest_detection_monotonic = time.monotonic()
        except (TypeError, ValueError):
            self.get_logger().warning("invalid detection JSON received")

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frame_sequence += 1
        self.last_frame_monotonic = time.monotonic()

    def on_tape_image(self, msg):
        self.tape_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.tape_frame_monotonic = time.monotonic()

    def on_secondary_image(self, msg):
        self.secondary_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.secondary_frame_sequence += 1
        self.secondary_last_frame_monotonic = time.monotonic()

    def start_mjpeg_stream(self, url, stream_name):
        thread = threading.Thread(target=self.read_mjpeg, args=(url, stream_name), daemon=True)
        thread.start()
        self.external_stream_threads.append(thread)

    def read_mjpeg(self, url, stream_name):
        frame_attr = "frame" if stream_name == "primary" else f"{stream_name}_frame"
        sequence_attr = "frame_sequence" if stream_name == "primary" else f"{stream_name}_frame_sequence"
        age_attr = "last_frame_monotonic" if stream_name == "primary" else f"{stream_name}_last_frame_monotonic"
        while not self.secondary_stream_stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=5) as response:
                    buffer = bytearray()
                    while not self.secondary_stream_stop.is_set():
                        chunk = response.read(16384)
                        if not chunk:
                            break
                        buffer.extend(chunk)
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                        if start >= 0 and end >= 0:
                            jpg = np.frombuffer(buffer[start:end + 2], dtype=np.uint8)
                            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                            del buffer[:end + 2]
                            if frame is not None:
                                setattr(self, frame_attr, frame)
                                setattr(self, sequence_attr, getattr(self, sequence_attr) + 1)
                                setattr(self, age_attr, time.monotonic())
                        elif len(buffer) > 2_000_000:
                            del buffer[:-2]
            except Exception as exc:
                self.get_logger().warning(f"external webcam {stream_name} unavailable: {exc}")
                self.secondary_stream_stop.wait(1.0)

    def on_scan(self, msg):
        self.last_scan_monotonic = time.monotonic()
        values = []
        for index, value in enumerate(msg.ranges):
            if math.isfinite(value) and max(
                msg.range_min,
                self.safety_min_valid_range,
                self.lidar_self_filter_distance_m,
            ) <= value <= msg.range_max:
                angle = msg.angle_min + index * msg.angle_increment
                values.append((value, math.atan2(math.sin(angle), math.cos(angle))))
        if values:
            self.front_range, self.closest_obstacle_angle_rad = min(values)
        else:
            self.front_range = math.inf
            self.closest_obstacle_angle_rad = None

    def on_motor_command(self, msg):
        if any(abs(float(motor.rps)) > 1e-4 for motor in msg.data):
            self.last_motor_active_monotonic = time.monotonic()

    def on_battery(self, msg):
        now = time.monotonic()
        self.battery_voltage_mv = int(msg.data)
        self.last_battery_monotonic = now
        self.battery_samples.append((now, self.battery_voltage_mv))
        cutoff = now - 600.0
        while self.battery_samples and self.battery_samples[0][0] < cutoff:
            del self.battery_samples[0]

    def publish(self, linear_x, linear_y, angular_z):
        if self.viewer_only:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop(self, repeats=1):
        if self.viewer_only:
            return
        for _ in range(repeats):
            self.cmd_pub.publish(Twist())

    def publish_fork(self, command):
        if self.viewer_only:
            return
        self.fork_pub.publish(String(data=command))

    def start_telemetry(self, session):
        payload = {"type": "recording", "action": "start", "session": session}
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)

    def finish_telemetry(self, session, destination):
        payload = {"type": "recording", "action": "stop", "session": session}
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)
        for _ in range(10):
            try:
                time.sleep(0.1)
                urllib.request.urlretrieve(
                    f"{self.telemetry_base_url}/telemetry/{session}.jsonl", str(destination)
                )
                return True
            except Exception:
                continue
        return False


# Compatibility for the archived 1.1 headless runner. New GUI/UI code uses the
# role-specific name above.
TeleopNode = DevControlClientNode


class HttpViewerSource:
    """Three MJPEG inputs with the same frame interface as DevControlClientNode."""

    def __init__(self, args):
        self.frame = None
        self.frame_sequence = 0
        self.last_frame_monotonic = 0.0
        self.secondary_frame = None
        self.secondary_frame_sequence = 0
        self.secondary_last_frame_monotonic = 0.0
        self.tertiary_frame = None
        self.tertiary_frame_sequence = 0
        self.tertiary_last_frame_monotonic = 0.0
        self.front_range = math.inf
        self.last_scan_monotonic = 0.0
        self.last_motor_active_monotonic = 0.0
        self.battery_voltage_mv = None
        self.battery_samples = []
        self.last_battery_monotonic = 0.0
        self.latest_detection = None
        self.latest_detection_monotonic = 0.0
        self.latest_dock_inventory = None
        self.latest_dock_inventory_monotonic = 0.0
        self.auto_dock_status = {"state": "unknown", "reason": "http_viewer_only"}
        self.secondary_stream_stop = threading.Event()
        self.external_stream_threads = []
        self.child_processes = []
        self.control_address = (args.control_host, args.control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_url = args.control_url
        self.control_command = args.control_command
        self.control_process = None
        self.control_queue = queue.Queue(maxsize=32)
        self.telemetry_base_url = args.primary_video_url.rsplit("/stream", 1)[0]
        if args.primary_video_command:
            self.start_mjpeg_command(args.primary_video_command)
        else:
            self.start_mjpeg_stream(args.primary_video_url, "primary")
        if not args.disable_external_webcams:
            self.start_mjpeg_stream(args.webcam_1_video_url, "secondary")
            self.start_mjpeg_stream(args.webcam_2_video_url, "tertiary")
        if self.control_url or self.control_command:
            thread = threading.Thread(target=self.send_control_http, daemon=True)
            thread.start()
            self.external_stream_threads.append(thread)

    def start_mjpeg_command(self, command):
        thread = threading.Thread(target=self.read_mjpeg_command, args=(command,), daemon=True)
        thread.start()
        self.external_stream_threads.append(thread)

    def read_mjpeg_command(self, command):
        while not self.secondary_stream_stop.is_set():
            process = subprocess.Popen(
                shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
            self.child_processes.append(process)
            buffer = bytearray()
            try:
                while not self.secondary_stream_stop.is_set():
                    chunk = process.stdout.read(16384)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    start = buffer.find(b"\xff\xd8")
                    end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                    if start >= 0 and end >= 0:
                        jpg = np.frombuffer(buffer[start:end + 2], dtype=np.uint8)
                        frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                        del buffer[:end + 2]
                        if frame is not None:
                            self.frame = frame
                            self.frame_sequence += 1
                            self.last_frame_monotonic = time.monotonic()
                    elif len(buffer) > 2_000_000:
                        del buffer[:-2]
            finally:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                if process in self.child_processes:
                    self.child_processes.remove(process)
            self.secondary_stream_stop.wait(1.0)

    def start_mjpeg_stream(self, url, stream_name):
        thread = threading.Thread(target=self.read_mjpeg, args=(url, stream_name), daemon=True)
        thread.start()
        self.external_stream_threads.append(thread)

    def read_mjpeg(self, url, stream_name):
        frame_attr = "frame" if stream_name == "primary" else f"{stream_name}_frame"
        sequence_attr = "frame_sequence" if stream_name == "primary" else f"{stream_name}_frame_sequence"
        age_attr = "last_frame_monotonic" if stream_name == "primary" else f"{stream_name}_last_frame_monotonic"
        while not self.secondary_stream_stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=5) as response:
                    buffer = bytearray()
                    while not self.secondary_stream_stop.is_set():
                        chunk = response.read(16384)
                        if not chunk:
                            break
                        buffer.extend(chunk)
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                        if start >= 0 and end >= 0:
                            jpg = np.frombuffer(buffer[start:end + 2], dtype=np.uint8)
                            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                            del buffer[:end + 2]
                            if frame is not None:
                                setattr(self, frame_attr, frame)
                                setattr(self, sequence_attr, getattr(self, sequence_attr) + 1)
                                setattr(self, age_attr, time.monotonic())
                        elif len(buffer) > 2_000_000:
                            del buffer[:-2]
            except Exception:
                self.secondary_stream_stop.wait(1.0)

    def send_control(self, payload):
        if not self.control_url and not self.control_command:
            self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)
            return
        try:
            self.control_queue.put_nowait(payload)
        except queue.Full:
            try:
                self.control_queue.get_nowait()
            except queue.Empty:
                pass
            self.control_queue.put_nowait(payload)

    def send_control_http(self):
        while not self.secondary_stream_stop.is_set():
            try:
                payload = self.control_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if self.control_command:
                    if self.control_process is None or self.control_process.poll() is not None:
                        self.control_process = subprocess.Popen(
                            shlex.split(self.control_command),
                            stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                        self.child_processes.append(self.control_process)
                    self.control_process.stdin.write(
                        json.dumps(payload).encode("utf-8") + b"\n"
                    )
                    self.control_process.stdin.flush()
                else:
                    request = urllib.request.Request(
                        self.control_url,
                        data=json.dumps(payload).encode("utf-8"),
                        headers={"Content-Type": "application/json"},
                        method="POST",
                    )
                    with urllib.request.urlopen(request, timeout=1.0):
                        pass
            except Exception:
                if self.control_process is not None:
                    if self.control_process in self.child_processes:
                        self.child_processes.remove(self.control_process)
                    self.control_process = None
                continue

    def publish(self, linear_x, linear_y, angular_z):
        self.send_control({
            "type": "drive", "linear_x": linear_x,
            "linear_y": linear_y, "angular_z": angular_z,
        })

    def stop(self, repeats=1):
        for _ in range(repeats):
            self.publish(0.0, 0.0, 0.0)

    def publish_fork(self, command):
        self.send_control({"type": "fork", "command": command})

    def start_telemetry(self, session):
        self.send_control({"type": "recording", "action": "start", "session": session})

    def finish_telemetry(self, session, destination):
        self.send_control({"type": "recording", "action": "stop", "session": session})
        for _ in range(10):
            try:
                time.sleep(0.1)
                urllib.request.urlretrieve(
                    f"{self.telemetry_base_url}/telemetry/{session}.jsonl", str(destination)
                )
                return True
            except Exception:
                continue
        return False


class VideoLabel(QLabel):
    """Fixed-size video viewport that does not feed each frame back into layout."""

    def __init__(self, placeholder, width, height):
        super().__init__(placeholder)
        self._pixmap = None
        self._source_size = None
        self._draw_rect = QRect()
        self.setAlignment(Qt.AlignCenter)
        self.setFixedSize(width, height)
        self.setStyleSheet("background: #090909; color: #d9d9d9;")

    def set_frame(self, image):
        self._pixmap = QPixmap.fromImage(image)
        if self._source_size != self._pixmap.size():
            self._source_size = self._pixmap.size()
            self._update_draw_rect()
        self.update()

    def resizeEvent(self, event):
        self._update_draw_rect()
        super().resizeEvent(event)

    def _update_draw_rect(self):
        if self._pixmap is None or self._pixmap.isNull():
            self._draw_rect = QRect()
            return
        source_size = self._pixmap.size()
        target_size = source_size.scaled(self.contentsRect().size(), Qt.KeepAspectRatio)
        self._draw_rect = QRect(
            (self.width() - target_size.width()) // 2,
            (self.height() - target_size.height()) // 2,
            target_size.width(),
            target_size.height(),
        )

    def paintEvent(self, event):
        if self._pixmap is None:
            super().paintEvent(event)
            return
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.palette().window())
        painter.drawPixmap(self._draw_rect, self._pixmap)


class EntityMapView(QWidget):
    """Low-cost map view showing one best-visible face per grouped pallet."""

    SYMBOL_TEXT = {
        "star": "★", "diamond": "◆", "spade": "♠",
        "clover": "♣", "heart": "♥",
    }

    def __init__(self):
        super().__init__()
        self.map_image = None
        self.metadata = None
        self.entity_map = {"state": "waiting", "entities": []}
        self.setFixedSize(560, 220)
        self.setStyleSheet("background: #090909;")

    def set_data(self, map_image, metadata, entity_map):
        self.map_image = map_image
        self.metadata = metadata
        self.entity_map = entity_map or {"state": "waiting", "entities": []}
        self.update()

    def map_pixel(self, pose):
        metadata = self.metadata
        dx = float(pose["x"]) - metadata["origin_x"]
        dy = float(pose["y"]) - metadata["origin_y"]
        c = math.cos(metadata["origin_yaw"])
        s = math.sin(metadata["origin_yaw"])
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return (
            local_x / metadata["resolution"],
            metadata["height"] - 1.0 - local_y / metadata["resolution"],
        )

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#090909"))
        if self.map_image is None or self.metadata is None:
            painter.setPen(QColor("#d9d9d9"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Nav2 /map 수신 대기")
            return
        target_size = self.map_image.size().scaled(
            self.contentsRect().size(), Qt.KeepAspectRatio
        )
        draw_rect = QRect(
            (self.width() - target_size.width()) // 2,
            (self.height() - target_size.height()) // 2,
            target_size.width(), target_size.height(),
        )
        painter.drawImage(draw_rect, self.map_image)
        scale_x = draw_rect.width() / max(self.metadata["width"], 1)
        scale_y = draw_rect.height() / max(self.metadata["height"], 1)
        painter.setPen(QPen(QColor("#00e5ff"), 2))
        painter.setBrush(QColor("#00e5ff"))
        for entity in self.entity_map.get("entities", []):
            pose = entity.get("pose") if isinstance(entity, dict) else None
            if not isinstance(pose, dict):
                continue
            try:
                source_x, source_y = self.map_pixel(pose)
                x = draw_rect.left() + source_x * scale_x
                y = draw_rect.top() + source_y * scale_y
                yaw = float(pose.get("yaw", 0.0)) - self.metadata["origin_yaw"]
            except (KeyError, TypeError, ValueError):
                continue
            center = QPointF(x, y)
            heading = QPointF(x + 18.0 * math.cos(yaw), y - 18.0 * math.sin(yaw))
            painter.drawEllipse(center, 5.0, 5.0)
            painter.drawLine(center, heading)
            matrix = entity.get("matrix") or []
            symbols = [self.SYMBOL_TEXT.get(str(item), str(item)[:1]) for item in matrix]
            label = "".join(symbols[:2]) + ("/" + "".join(symbols[2:4]) if len(symbols) >= 4 else "")
            painter.drawText(QPointF(x + 8.0, y - 7.0), f"{entity.get('id', '?')} {label}")
        painter.setPen(QColor("#f2f2f2"))
        state = str(self.entity_map.get("state", "waiting"))
        count = len(self.entity_map.get("entities", []))
        painter.drawText(draw_rect.adjusted(8, 6, -8, -6), Qt.AlignLeft | Qt.AlignTop,
                         f"{self.metadata['frame_id']} | {state} | pallets {count}")


class TeleopWindow(QMainWindow):
    MOVEMENT_KEYS = {Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D, Qt.Key_Q, Qt.Key_E}
    FORK_KEYS = {Qt.Key_Up, Qt.Key_Down}
    KEY_RELEASE_DEBOUNCE_MS = 150
    LATERAL_ACCEL_M_S2 = 0.025
    LATERAL_STOP_TOLERANCE_M = 0.003
    def __init__(self, node, args):
        super().__init__()
        self.node = node
        self.args = args
        self.pressed = set()
        self.movement_release_timers = {}
        self.last_displayed_sequence = (-1, -1, -1)
        self.last_map_sequence = (-1, -1)
        self.writer = None
        self.record_path = None
        self.frame_log_file = None
        self.frame_log_path = None
        self.telemetry_session = None
        self.recorded_frames = 0
        self.auto_dock_log_file = None
        self.auto_dock_log_path = None
        self.last_logged_auto_dock_status = None
        self.warning_tape_debug = None
        self.warning_tape_track = None
        self.warning_tape_track_at = 0.0
        self.warning_tape_hsv_path = self.resolve_warning_tape_hsv_path()
        self.warning_tape_hsv_cache = None
        self.warning_tape_hsv_cache_mtime_ns = None
        self.dock_grid_revision = None
        self.dock_grid_occupied = {}
        self.dock_grid_last_signature = None

        self.charge_state_path = Path(args.output_dir) / "battery_charge_state.json"
        self.charge_state = "unknown"
        self.charge_started_at = None
        self.last_charge_ended_at = None
        self.last_charge_evaluation = 0.0
        self.battery_slope_mv_min = None
        self.load_charge_state()
        self.setWindowTitle(f"Vehicle {args.vehicle} Camera Teleop")
        self.resize(1180, 760)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #151515; color: #f2f2f2; }
            QGroupBox { border: 1px solid #f2f2f2; margin-top: 8px; padding: 12px 10px 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #f2f2f2; }
            QPushButton { background: #222222; border: 1px solid #f2f2f2; padding: 7px 10px; }
            QPushButton:pressed { background: #555555; }
            QPushButton:checked { background: #f2f2f2; color: #151515; }
            QComboBox { background: #222222; border: 1px solid #f2f2f2; padding: 5px; }
            QPlainTextEdit { background: #222222; border: 1px solid #f2f2f2; padding: 5px; }
            QSlider::groove:horizontal { height: 3px; background: #777777; }
            QSlider::sub-page:horizontal { background: #f2f2f2; }
            QSlider::handle:horizontal { background: #f2f2f2; width: 12px; margin: -5px 0; }
            QLabel#status { color: #d4d4d4; font-family: monospace; font-size: 11px; }
        """)

        self.video = VideoLabel("Waiting for vehicle camera…", 560, 420)
        self.entity_map_view = EntityMapView()
        self.status = QLabel("READY")
        self.status.setObjectName("status")
        self.battery_label = QLabel("배터리: 수신 대기 | 충전 상태 판정 중")
        self.battery_label.setObjectName("status")
        self.y_slot_center_label = QLabel("Y 슬롯 X 중심: arrival 대기")
        self.y_slot_center_label.setObjectName("status")

        self.linear, self.linear_value = self.speed_slider(
            args.linear_speed, 0.01, 0.30, 0.01, " m/s"
        )
        self.angular, self.angular_value = self.speed_slider(
            args.angular_speed, 0.05, 2.0, 0.05, " rad/s"
        )

        controls = QGridLayout()
        controls.setHorizontalSpacing(8)
        controls.setVerticalSpacing(8)
        controls.addWidget(QLabel("전후/횡이동 속도"), 0, 0)
        controls.addWidget(self.linear, 0, 1, 1, 2)
        controls.addWidget(self.linear_value, 0, 3)
        controls.addWidget(QLabel("회전 속도"), 1, 0)
        controls.addWidget(self.angular, 1, 1, 1, 2)
        controls.addWidget(self.angular_value, 1, 3)

        self.record = QPushButton("●  Start recording")
        self.record.setCheckable(True)
        self.record.toggled.connect(self.toggle_recording)
        self.record_label = QLabel("Not recording")
        record_row = QHBoxLayout()
        record_row.addWidget(self.record)
        record_row.addWidget(self.record_label, 1)


        self.arrival_location = QComboBox()
        for location in ("DOCK_1", "Y1", "Y2", "Y3", "Y4"):
            self.arrival_location.addItem(location, location)
        self.arrival_operation = QComboBox()
        self.arrival_operation.addItem("PICK", "PICK")
        self.arrival_operation.addItem("PLACE", "PLACE")
        self.arrival_product = QComboBox()
        self.arrival_product.addItem("NORMAL", "NORMAL")
        self.arrival_product.addItem("FRESH", "FRESH")
        self.y_slot_insertion_distance = QDoubleSpinBox()
        self.y_slot_insertion_distance.setRange(1.0, 100.0)
        self.y_slot_insertion_distance.setDecimals(1)
        self.y_slot_insertion_distance.setValue(25.0)
        self.y_slot_insertion_distance.setSuffix(" cm")
        self.y_slot_insertion_distance.setEnabled(False)
        self.arrival_location.currentIndexChanged.connect(
            self.on_arrival_location_changed
        )
        self.arrival_button = QPushButton("선택 Arrival 발행")
        self.arrival_button.clicked.connect(self.publish_arrival_trigger)
        self.nearest_product = QComboBox()
        self.nearest_product.addItem("NORMAL", "NORMAL")
        self.nearest_product.addItem("FRESH", "FRESH")
        self.nearest_arrival_button = QPushButton("최근접 상품 PICK")
        self.nearest_arrival_button.clicked.connect(
            self.publish_nearest_arrival_trigger
        )
        self.legacy_entity_recognition = QCheckBox("이전 버전 엔티티 인식")
        self.legacy_entity_recognition.setChecked(True)
        self.auto_dock_stop_button = QPushButton("AUTO-DOCK STOP 발행")
        self.auto_dock_stop_button.clicked.connect(self.publish_auto_dock_stop)
        self.auto_dock_status_label = QLabel("AUTO-DOCK 상태: 수신 대기")
        self.fork_flow_status_label = QLabel("Fork/Ready 상태: 수신 대기")
        self.operation_label = QLabel("Arrival 명령 대기")
        arrival_layout = QGridLayout()
        arrival_layout.addWidget(self.arrival_location, 0, 0)
        arrival_layout.addWidget(self.arrival_operation, 0, 1)
        arrival_layout.addWidget(self.arrival_product, 0, 2)
        arrival_layout.addWidget(self.arrival_button, 0, 3, 1, 2)
        arrival_layout.addWidget(QLabel("Y 삽입 직진거리"), 1, 0)
        arrival_layout.addWidget(self.y_slot_insertion_distance, 1, 1)
        arrival_layout.addWidget(self.auto_dock_stop_button, 1, 3, 1, 2)
        arrival_layout.addWidget(QLabel("태그 미지정"), 2, 0)
        arrival_layout.addWidget(self.nearest_product, 2, 1)
        arrival_layout.addWidget(self.nearest_arrival_button, 2, 2)
        arrival_layout.addWidget(self.legacy_entity_recognition, 2, 3, 1, 2)
        arrival_layout.addWidget(self.auto_dock_status_label, 3, 0, 1, 5)
        arrival_layout.addWidget(self.fork_flow_status_label, 4, 0, 1, 5)
        arrival_layout.addWidget(self.operation_label, 5, 0, 1, 5)

        self.memo = QPlainTextEdit()
        self.memo.setPlaceholderText("캡처 메모 (이미지와 별도 JSON으로 저장)")
        self.memo.setMaximumHeight(70)
        self.capture = QPushButton("현재 차량 화면 캡처")
        self.capture.clicked.connect(self.capture_snapshot)
        self.capture_label = QLabel("캡처 대기")
        capture_row = QHBoxLayout()
        capture_row.addWidget(self.capture)
        capture_row.addWidget(self.capture_label, 1)


        vehicle_column = QVBoxLayout()
        vehicle_column.addWidget(self.video, 1)
        self.dock_slot_cells = {}
        dock_grid_layout = QGridLayout()
        dock_grid_layout.setSpacing(3)
        dock_grid_layout.addWidget(QLabel("깊이"), 0, 0)
        for row in range(1, 9):
            header = QLabel(f"R{row}")
            header.setAlignment(Qt.AlignCenter)
            dock_grid_layout.addWidget(header, 0, row)
        for display_row, column in enumerate((3, 2, 1), start=1):
            depth_label = QLabel(f"C{column}")
            depth_label.setAlignment(Qt.AlignCenter)
            dock_grid_layout.addWidget(depth_label, display_row, 0)
            for row in range(1, 9):
                slot_id = f"DOCK_R{row}_C{column}"
                cell = QLabel("·")
                cell.setAlignment(Qt.AlignCenter)
                cell.setFixedSize(48, 34)
                cell.setToolTip(slot_id)
                cell.setStyleSheet(
                    "background:#262626;color:#888;border:1px solid #555;"
                    "border-radius:2px;"
                )
                self.dock_slot_cells[slot_id] = cell
                dock_grid_layout.addWidget(cell, display_row, row)
        self.dock_grid_status = QLabel("슬롯 수신 대기 | C1이 주의선에서 가장 가까운 열")
        dock_grid_layout.addWidget(self.dock_grid_status, 4, 0, 1, 7)
        self.dock_grid_reset_button = QPushButton("슬롯 초기화")
        self.dock_grid_reset_button.clicked.connect(self.reset_dock_slot_grid)
        dock_grid_layout.addWidget(self.dock_grid_reset_button, 4, 7, 1, 2)
        dock_grid_panel = QGroupBox("DOCK 슬롯 3×8")
        dock_grid_panel.setLayout(dock_grid_layout)
        vehicle_column.addWidget(dock_grid_panel)
        vehicle_column.addWidget(self.entity_map_view)
        vehicle_panel = QGroupBox("차량 카메라 / 태그 엔티티 지도")
        vehicle_panel.setLayout(vehicle_column)

        layout = QVBoxLayout()
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)
        main_row = QHBoxLayout()
        main_row.setSpacing(16)
        main_row.addWidget(vehicle_panel, 1, Qt.AlignTop)
        control_panel = QGroupBox("설정 및 캡처")
        control_layout = QVBoxLayout()
        self.control_details = QWidget()
        control_details_layout = QVBoxLayout()
        control_details_layout.setContentsMargins(0, 0, 0, 0)
        self.control_details.setLayout(control_details_layout)
        self.control_details.setHidden(True)
        self.control_collapse_button = QPushButton("▼ 상세 설정 펼치기")
        self.control_collapse_button.clicked.connect(self.toggle_control_details)
        control_layout.addWidget(self.control_collapse_button)
        arrival_panel = QGroupBox("Auto Dock")
        arrival_panel.setLayout(arrival_layout)
        control_details_layout.addWidget(arrival_panel)
        self.log_filter = QComboBox()
        for label, value in (
            ("전체", "all"), ("Auto Dock", "autodock"),
            ("녹화", "recording"), ("캡처", "capture"),
        ):
            self.log_filter.addItem(label, value)
        self.log_filter.currentIndexChanged.connect(self.refresh_log_records)
        self.log_list = QListWidget()
        self.log_list.currentRowChanged.connect(self.show_selected_log_record)
        self.log_preview = QLabel("기록을 선택하세요")
        self.log_preview.setAlignment(Qt.AlignCenter)
        self.log_preview.setMinimumSize(520, 320)
        self.log_details = QPlainTextEdit()
        self.log_details.setReadOnly(True)
        self.log_details.setMaximumHeight(220)
        self.log_refresh_button = QPushButton("새로고침")
        self.log_refresh_button.clicked.connect(self.refresh_log_records)
        self.log_discard_button = QPushButton("노이즈로 폐기")
        self.log_discard_button.clicked.connect(self.discard_selected_log_record)
        log_top = QHBoxLayout()
        log_top.addWidget(QLabel("종류"))
        log_top.addWidget(self.log_filter, 1)
        log_top.addWidget(self.log_refresh_button)
        log_top.addWidget(self.log_discard_button)
        log_body = QHBoxLayout()
        log_body.addWidget(self.log_list, 1)
        log_visual = QVBoxLayout()
        log_visual.addWidget(self.log_preview, 1)
        log_visual.addWidget(self.log_details)
        log_body.addLayout(log_visual, 2)
        self.log_window = QDialog(self)
        self.log_window.setWindowTitle("기록 검토 / 노이즈 폐기")
        self.log_window.resize(980, 700)
        log_window_layout = QVBoxLayout()
        log_window_layout.addLayout(log_top)
        log_window_layout.addLayout(log_body, 1)
        self.log_window.setLayout(log_window_layout)
        self.open_log_button = QPushButton("기록 검토 / 폐기 창 열기")
        self.open_log_button.clicked.connect(self.open_log_window)
        control_details_layout.addWidget(self.open_log_button)
        controls_widget = QWidget()
        controls_widget.setLayout(controls)
        control_details_layout.addWidget(controls_widget)
        control_details_layout.addWidget(QLabel(
            "주행: W A S D / 제자리 회전: Q E / 리프트: 방향키 / 비상정지: SPACE"
        ))
        control_layout.addWidget(self.status)
        control_layout.addWidget(self.battery_label)
        control_layout.addWidget(self.y_slot_center_label)
        control_layout.addLayout(record_row)
        control_layout.addWidget(self.memo)
        control_layout.addLayout(capture_row)
        control_layout.addWidget(self.control_details)
        control_panel.setLayout(control_layout)
        main_row.addWidget(control_panel, 1, Qt.AlignTop)
        layout.addLayout(main_row, 1)
        root = QWidget()
        root.setLayout(layout)
        self.setCentralWidget(root)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        # X11 forwarding becomes unresponsive when three full-resolution
        # streams are repainted at 20 FPS.  Teleop commands remain responsive
        # at 10 FPS while substantially reducing remote paint traffic.
        self.timer.start(100)

    def toggle_control_details(self):
        collapsed = not self.control_details.isHidden()
        self.control_details.setHidden(collapsed)
        self.control_collapse_button.setText(
            "▼ 상세 설정 펼치기" if collapsed else "▲ 상세 설정 접기"
        )

    def on_arrival_location_changed(self, _index=None):
        location = str(self.arrival_location.currentData() or "")
        if location in {"Y1", "Y2", "Y3", "Y4"}:
            self.y_slot_insertion_distance.setEnabled(True)
            self.arrival_operation.setCurrentIndex(
                self.arrival_operation.findData("PLACE")
            )
            self.arrival_product.setCurrentIndex(
                self.arrival_product.findData("FRESH")
            )
        elif location == "DOCK_1":
            self.y_slot_insertion_distance.setEnabled(False)
            self.arrival_operation.setCurrentIndex(
                self.arrival_operation.findData("PICK")
            )

    def open_log_window(self):
        self.refresh_log_records()
        self.log_window.show()
        self.log_window.raise_()
        self.log_window.activateWindow()

    def collect_log_records(self):
        root = Path(self.args.output_dir)
        records = []

        def add_record(kind, label, files, metadata=None):
            files = list(dict.fromkeys(path for path in files if path.exists()))
            if not files:
                return
            images = [path for path in files if path.suffix.lower() in {".jpg", ".jpeg", ".png"}]
            videos = [path for path in files if path.suffix.lower() in {".mp4", ".avi"}]
            records.append({
                "kind": kind,
                "label": label,
                "files": files,
                "metadata": metadata if metadata is not None and metadata.exists() else None,
                "preview_files": images or videos,
                "mtime": max(path.stat().st_mtime for path in files),
            })

        recording_stems = set()
        for path in root.glob("teleop_*.*"):
            if path.name.endswith(".frames.jsonl"):
                recording_stems.add(path.name[:-len(".frames.jsonl")])
            elif path.suffix.lower() in {".mp4", ".jsonl"}:
                recording_stems.add(path.stem)
        for stem in recording_stems:
            files = [
                root / f"{stem}.mp4",
                root / f"{stem}.jsonl",
                root / f"{stem}.frames.jsonl",
            ]
            telemetry = root / f"{stem}.jsonl"
            frame_log = root / f"{stem}.frames.jsonl"
            add_record(
                "recording", f"녹화 | {stem}", files,
                telemetry if telemetry.exists() else frame_log,
            )

        for path in root.glob("autodock_*.jsonl"):
            add_record("autodock", f"Auto Dock | {path.stem}", [path], path)

        capture_dir = root / "captures"
        capture_stems = {path.stem for path in capture_dir.glob("capture_*.*")}
        for stem in capture_stems:
            files = [capture_dir / f"{stem}.json", capture_dir / f"{stem}.jpg"]
            add_record("capture", f"캡처 | {stem}", files, capture_dir / f"{stem}.json")


        return sorted(records, key=lambda record: record["mtime"], reverse=True)

    def refresh_log_records(self, _index=None):
        if not hasattr(self, "log_list"):
            return
        selected_filter = self.log_filter.currentData()
        self.log_records = [
            record for record in self.collect_log_records()
            if selected_filter == "all" or record["kind"] == selected_filter
        ]
        self.log_list.clear()
        for record in self.log_records:
            stamp = time.strftime("%m-%d %H:%M:%S", time.localtime(record["mtime"]))
            self.log_list.addItem(f"{stamp}  {record['label']}")
        self.log_preview.setText(
            f"기록 {len(self.log_records)}개" if self.log_records else "기록 없음"
        )
        self.log_preview.setPixmap(QPixmap())
        self.log_details.clear()

    @staticmethod
    def compact_log_json(value, depth=0):
        if depth >= 5:
            if isinstance(value, (dict, list)):
                return f"<{type(value).__name__} {len(value)}개>"
            return value
        if isinstance(value, dict):
            return {
                key: TeleopWindow.compact_log_json(item, depth + 1)
                for key, item in value.items()
            }
        if isinstance(value, list):
            if len(value) > 12:
                return {
                    "count": len(value),
                    "first": [
                        TeleopWindow.compact_log_json(item, depth + 1)
                        for item in value[:3]
                    ],
                }
            return [TeleopWindow.compact_log_json(item, depth + 1) for item in value]
        return value

    def log_record_summary(self, record):
        if "inline_metadata" in record:
            return self.compact_log_json(record["inline_metadata"])
        metadata = record.get("metadata")
        if metadata is None:
            return {"files": [path.name for path in record["files"]]}
        try:
            if metadata.suffix == ".jsonl":
                if record.get("kind") == "autodock":
                    events = []
                    with metadata.open(encoding="utf-8") as source:
                        for line in source:
                            if not line.strip():
                                continue
                            try:
                                events.append(json.loads(line))
                            except ValueError:
                                continue
                    return {
                        "files": [path.name for path in record["files"]],
                        "events": self.compact_log_json(events[-100:]),
                    }
                counts = {}
                first_time = last_time = None
                line_count = 0
                with metadata.open(encoding="utf-8") as source:
                    for line in source:
                        if not line.strip():
                            continue
                        line_count += 1
                        try:
                            item = json.loads(line)
                        except ValueError:
                            continue
                        event_type = item.get("type", "unknown")
                        counts[event_type] = counts.get(event_type, 0) + 1
                        stamp = item.get("monotonic")
                        if stamp is not None:
                            first_time = stamp if first_time is None else first_time
                            last_time = stamp
                return {
                    "files": [path.name for path in record["files"]],
                    "line_count": line_count,
                    "duration_sec": (
                        None if first_time is None or last_time is None
                        else last_time - first_time
                    ),
                    "event_counts": counts,
                }
            return self.compact_log_json(
                json.loads(metadata.read_text(encoding="utf-8"))
            )
        except (OSError, TypeError, ValueError) as exc:
            return {"files": [path.name for path in record["files"]], "read_error": str(exc)}

    def show_selected_log_record(self, row):
        if row < 0 or row >= len(getattr(self, "log_records", [])):
            return
        record = self.log_records[row]
        frames = []
        for path in record["preview_files"][:2]:
            if path.suffix.lower() in {".mp4", ".avi"}:
                capture = cv2.VideoCapture(str(path))
                frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
                if frame_count > 1:
                    capture.set(cv2.CAP_PROP_POS_FRAMES, frame_count // 2)
                ok, frame = capture.read()
                capture.release()
                if ok:
                    frames.append(frame)
            else:
                frame = cv2.imread(str(path))
                if frame is not None:
                    frames.append(frame)
        if frames:
            target_height = min(frame.shape[0] for frame in frames)
            resized = [
                cv2.resize(
                    frame,
                    (max(1, round(frame.shape[1] * target_height / frame.shape[0])), target_height),
                    interpolation=cv2.INTER_AREA,
                )
                for frame in frames
            ]
            combined = np.hstack(resized)
            rgb = cv2.cvtColor(combined, cv2.COLOR_BGR2RGB)
            image = QImage(
                rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888
            ).copy()
            pixmap = QPixmap.fromImage(image).scaled(
                self.log_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.log_preview.setPixmap(pixmap)
        else:
            self.log_preview.setPixmap(QPixmap())
            self.log_preview.setText("미리보기 없음")
        self.log_details.setPlainText(
            json.dumps(self.log_record_summary(record), ensure_ascii=False, indent=2)
        )

    def discard_selected_log_record(self):
        row = self.log_list.currentRow()
        if row < 0 or row >= len(getattr(self, "log_records", [])):
            self.log_details.setPlainText("폐기할 기록을 선택하세요")
            return
        record = self.log_records[row]
        if (
            self.record.isChecked() and self.record_path is not None
            and Path(self.record_path) in record["files"]
        ):
            self.log_details.setPlainText("녹화 중인 항목은 먼저 녹화를 종료하세요")
            return
        answer = QMessageBox.question(
            self.log_window, "노이즈 기록 폐기",
            f"{record['label']}\n"
            f"연결된 파일 {len(record['files'])}개를 trash로 옮길까?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        trash_root = Path(self.args.output_dir) / ".trash" / "log_records"
        record_stem = record["files"][0].stem
        trash_dir = trash_root / (
            time.strftime("%Y%m%d_%H%M%S") + "_" + record["kind"] + "_" +
            record_stem
        )
        trash_dir.mkdir(parents=True, exist_ok=False)
        for path in record["files"]:
            path.replace(trash_dir / path.name)
        self.refresh_log_records()
        self.log_details.setPlainText(f"trash로 옮겨짐: {trash_dir}")

    def load_charge_state(self):
        if not self.charge_state_path.exists():
            return
        try:
            data = json.loads(self.charge_state_path.read_text(encoding="utf-8"))
            state = str(data.get("charge_state", "unknown"))
            self.charge_state = state if state in {"unknown", "charging", "not_charging"} else "unknown"
            self.charge_started_at = data.get("charge_started_at")
            self.last_charge_ended_at = data.get("last_charge_ended_at")
        except (OSError, TypeError, ValueError):
            return

    def persist_charge_state(self):
        self.charge_state_path.parent.mkdir(parents=True, exist_ok=True)
        self.charge_state_path.write_text(
            json.dumps({
                "charge_state": self.charge_state,
                "charge_started_at": self.charge_started_at,
                "last_charge_ended_at": self.last_charge_ended_at,
                "updated_at": time.time(),
                "method": "estimated from battery voltage trend while motors are stopped",
            }, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def format_elapsed(seconds):
        seconds = max(0, int(seconds))
        days, remainder = divmod(seconds, 86400)
        hours, remainder = divmod(remainder, 3600)
        minutes, _seconds = divmod(remainder, 60)
        if days:
            return f"{days}일 {hours}시간"
        if hours:
            return f"{hours}시간 {minutes}분"
        return f"{minutes}분"

    def update_battery_status(self):
        now = time.monotonic()
        if now - self.last_charge_evaluation < 1.0:
            return
        self.last_charge_evaluation = now
        voltage = self.node.battery_voltage_mv
        if voltage is None or now - self.node.last_battery_monotonic > 5.0:
            self.battery_label.setText("배터리: 수신 없음 | 충전 상태 알 수 없음")
            return
        samples = [sample for sample in self.node.battery_samples if sample[0] >= now - 180.0]
        enough = len(samples) >= 20 and samples[-1][0] - samples[0][0] >= 60.0
        if enough:
            origin = samples[0][0]
            times = [sample[0] - origin for sample in samples]
            values = [float(sample[1]) for sample in samples]
            mean_time = sum(times) / len(times)
            mean_value = sum(values) / len(values)
            denominator = sum((value - mean_time) ** 2 for value in times)
            slope = 0.0 if denominator <= 1e-9 else (
                sum(
                    (sample_time - mean_time) * (sample_value - mean_value)
                    for sample_time, sample_value in zip(times, values)
                ) / denominator * 60.0
            )
            self.battery_slope_mv_min = slope
            stationary = now - self.node.last_motor_active_monotonic >= 20.0
            next_state = self.charge_state
            if stationary and slope >= 8.0:
                next_state = "charging"
            elif not stationary or slope <= 2.0:
                next_state = "not_charging"
            if next_state != self.charge_state:
                wall_time = time.time()
                if next_state == "charging":
                    self.charge_started_at = wall_time
                elif self.charge_state == "charging":
                    self.last_charge_ended_at = wall_time
                    self.charge_started_at = None
                self.charge_state = next_state
                try:
                    self.persist_charge_state()
                except OSError:
                    pass
        voltage_text = f"{voltage / 1000.0:.2f} V"
        slope_text = (
            "판정 중"
            if self.battery_slope_mv_min is None
            else f"{self.battery_slope_mv_min:+.1f} mV/분"
        )
        if self.charge_state == "charging":
            elapsed = ""
            if self.charge_started_at is not None:
                elapsed = f" | 충전 시작 후 {self.format_elapsed(time.time() - self.charge_started_at)}"
            state_text = f"충전 중 추정{elapsed}"
        else:
            if self.last_charge_ended_at is None:
                last_text = "마지막 충전 기록 없음"
            else:
                last_text = (
                    "마지막 충전 종료 후 "
                    f"{self.format_elapsed(time.time() - self.last_charge_ended_at)}"
                )
            state_text = (
                "미충전 추정" if self.charge_state == "not_charging" else "충전 판정 중"
            )
            state_text += f" | {last_text}"
        self.battery_label.setText(
            f"배터리 {voltage_text} | {state_text} | 전압추세 {slope_text}"
        )


    @staticmethod
    def speed_slider(value, minimum, maximum, step, suffix):
        scale = round(1.0 / step)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(round(minimum * scale), round(maximum * scale))
        slider.setValue(round(value * scale))
        slider.setMinimumWidth(220)
        label = QLabel()
        label.setMinimumWidth(82)
        update_label = lambda raw: label.setText(f"{raw / scale:.2f}{suffix}")
        slider.valueChanged.connect(update_label)
        update_label(slider.value())
        slider.speed_scale = scale
        return slider, label

    def publish_latched_fork_key(self, key):
        if key == Qt.Key_Up:
            self.node.publish_fork("UP")
        elif key == Qt.Key_Down:
            self.node.publish_fork("DOWN")

    def press_movement_key(self, key):
        timer = self.movement_release_timers.pop(key, None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
        self.pressed.add(key)

    def release_movement_key(self, key):
        previous = self.movement_release_timers.pop(key, None)
        if previous is not None:
            previous.stop()
            previous.deleteLater()
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(
            lambda held_key=key, release_timer=timer:
            self.finish_movement_key_release(held_key, release_timer)
        )
        self.movement_release_timers[key] = timer
        timer.start(self.KEY_RELEASE_DEBOUNCE_MS)

    def finish_movement_key_release(self, key, timer):
        if self.movement_release_timers.get(key) is not timer:
            return
        self.movement_release_timers.pop(key, None)
        timer.deleteLater()
        self.pressed.discard(key)
        if not (self.pressed & self.MOVEMENT_KEYS):
            self.node.stop()

    def cancel_movement_key_releases(self):
        for timer in self.movement_release_timers.values():
            timer.stop()
            timer.deleteLater()
        self.movement_release_timers.clear()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self.emergency_stop()
            return
        if event.key() == Qt.Key_R and not event.isAutoRepeat():
            self.record.toggle()
            return
        if event.key() in self.MOVEMENT_KEYS:
            self.press_movement_key(event.key())
            event.accept()
            return
        if event.key() in self.FORK_KEYS:
            if not event.isAutoRepeat():
                self.publish_latched_fork_key(event.key())
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() in self.MOVEMENT_KEYS and not event.isAutoRepeat():
            self.release_movement_key(event.key())
            event.accept()
            return
        if event.key() in self.FORK_KEYS:
            event.accept()
            return
        super().keyReleaseEvent(event)

    def eventFilter(self, watched, event):
        """Capture drive keys even when a checkbox, button, or spinbox has focus."""
        # Fork arrows use the same latched command contract as the test panel:
        # press publishes UP/DOWN once, release does not publish STOP.  Handle
        # them before the editable-widget exception so focus cannot consume them.
        if event.type() == QEvent.KeyPress and event.key() in self.FORK_KEYS:
            if not event.isAutoRepeat():
                self.publish_latched_fork_key(event.key())
            return True
        if event.type() == QEvent.KeyRelease and event.key() in self.FORK_KEYS:
            return True

        # The memo editor must still receive ordinary letters and spaces
        # instead of treating them as vehicle controls.
        if self.memo.hasFocus():
            return super().eventFilter(watched, event)
        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Space and not event.isAutoRepeat():
                self.emergency_stop()
                return True
            if event.key() == Qt.Key_R and not event.isAutoRepeat():
                self.record.toggle()
                return True
            if event.key() in self.MOVEMENT_KEYS:
                self.press_movement_key(event.key())
                return True
        elif event.type() == QEvent.KeyRelease:
            if event.key() in self.MOVEMENT_KEYS and not event.isAutoRepeat():
                self.release_movement_key(event.key())
                return True
        return super().eventFilter(watched, event)

    def focusOutEvent(self, event):
        self.cancel_movement_key_releases()
        self.pressed.clear()
        self.node.stop(repeats=3)
        self.node.publish_fork("STOP")
        super().focusOutEvent(event)

    def write_auto_dock_log(self, event_type, **payload):
        if self.auto_dock_log_file is None:
            return
        record = {
            "type": event_type,
            "time": time.time(),
            "monotonic": time.monotonic(),
            **payload,
        }
        try:
            self.auto_dock_log_file.write(
                json.dumps(record, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )
            self.auto_dock_log_file.flush()
        except (OSError, TypeError, ValueError) as exc:
            self.auto_dock_log_file.close()
            self.auto_dock_log_file = None
            self.operation_label.setText(f"Auto Dock 로그 저장 실패: {exc}")

    def start_auto_dock_log(self, arrival):
        if self.auto_dock_log_file is not None:
            self.auto_dock_log_file.close()
        output = Path(self.args.output_dir)
        output.mkdir(parents=True, exist_ok=True)
        self.auto_dock_log_path = output / (
            time.strftime("autodock_%Y%m%d_%H%M%S")
            + f"_{time.time_ns() % 1_000_000_000:09d}.jsonl"
        )
        self.auto_dock_log_file = self.auto_dock_log_path.open(
            "w", encoding="utf-8", buffering=1
        )
        self.last_logged_auto_dock_status = None
        self.write_auto_dock_log("arrival", arrival=arrival)

    def log_auto_dock_status(self):
        if self.auto_dock_log_file is None:
            return
        status = self.node.auto_dock_status or {}
        signature = json.dumps(status, ensure_ascii=False, sort_keys=True)
        if signature == self.last_logged_auto_dock_status:
            return
        self.last_logged_auto_dock_status = signature
        self.write_auto_dock_log("status", status=status)

    def publish_arrival_trigger(self):
        """Publish the structured arrival selected in the Control GUI."""
        if self.args.http_viewer_only:
            self.operation_label.setText("HTTP 화면 전용 모드에서는 arrival 발행 불가")
            return
        location = self.arrival_location.currentData()
        operation = self.arrival_operation.currentData()
        product_type = self.arrival_product.currentData()
        arrival = self.node.publish_arrival(
            location=location, operation=operation,
            product_type=product_type,
            insertion_distance_cm=(
                self.y_slot_insertion_distance.value()
                if location in {"Y1", "Y2", "Y3", "Y4"} else None
            ),
            legacy_recognition=self.legacy_entity_recognition.isChecked(),
        )
        self.start_auto_dock_log(arrival)
        target_text = (
            " | NEAREST LEGACY"
            if location == "DOCK_1" and operation == "PICK"
            and self.legacy_entity_recognition.isChecked()
            else " | NEAREST"
            if location == "DOCK_1" and operation == "PICK" else ""
        )
        self.operation_label.setText(
            f"Arrival 발행: {location} {operation} {product_type}{target_text}"
        )

    def publish_nearest_arrival_trigger(self):
        if self.args.http_viewer_only:
            self.operation_label.setText("HTTP 화면 전용 모드에서는 arrival 발행 불가")
            return
        product_type = self.nearest_product.currentData()
        legacy = self.legacy_entity_recognition.isChecked()
        arrival = self.node.publish_nearest_arrival(
            product_type, legacy_recognition=legacy
        )
        self.start_auto_dock_log(arrival)
        self.operation_label.setText(
            f"DOCK 최근접 {product_type} PICK Arrival 발행"
            + (" | 이전 인식" if legacy else " | 현재 인식")
        )

    def publish_auto_dock_stop(self):
        if self.args.http_viewer_only:
            self.operation_label.setText("HTTP 화면 전용 모드에서는 stop 발행 불가")
            return
        self.node.publish_auto_dock_stop()
        self.auto_dock_status_label.setText("AUTO-DOCK stop 요청 전송")


    def emergency_stop(self):
        self.cancel_movement_key_releases()
        self.pressed.clear()
        self.node.stop(repeats=5)
        self.node.publish_fork("STOP")
        if not self.args.http_viewer_only:
            self.node.publish_auto_dock_stop()
        self.status.setText("EMERGENCY STOP")

    def command(self):
        linear_speed = self.linear.value() / self.linear.speed_scale
        angular_speed = self.angular.value() / self.angular.speed_scale
        linear_x = linear_speed * ((Qt.Key_W in self.pressed) - (Qt.Key_S in self.pressed))
        left_right = (Qt.Key_A in self.pressed) - (Qt.Key_D in self.pressed)
        rotate = (Qt.Key_Q in self.pressed) - (Qt.Key_E in self.pressed)
        return linear_x, linear_speed * left_right, angular_speed * rotate

    def tick(self):
        if not self.args.http_viewer_only:
            if not rclpy.ok():
                return
            for _ in range(20):
                rclpy.spin_once(self.node, timeout_sec=0.0)
        camera_age = time.monotonic() - self.node.last_frame_monotonic
        camera_ok = self.node.frame is not None and camera_age <= 0.75
        linear_x, linear_y, angular_z = self.command()
        manual_drive_active = bool(self.pressed & self.MOVEMENT_KEYS)
        blocked = linear_x > 0.0 and self.node.front_range < self.args.stop_distance
        # Manual teleop must remain independent of a delayed/dropped video
        # frame. When no drive key is held, leave the command channel free for
        # the vehicle-side aligner instead of continuously publishing zeros.
        if manual_drive_active and not blocked:
            self.node.publish(linear_x, linear_y, angular_z)
        elif manual_drive_active:
            self.node.stop()
        state = "READY" if camera_ok else "TELEOP READY | CAMERA STALE"
        if not camera_ok:
            state += " (video only)"
        if blocked:
            state += f" | FORWARD BLOCKED: {self.node.front_range:.2f} m"
        else:
            state += f" | front: {self.node.front_range:.2f} m"
        active = manual_drive_active and not blocked
        state += (f" | MECANUM | cmd x={linear_x if active else 0.0:+.2f}, "
                  f"y={linear_y if active else 0.0:+.2f}, z={angular_z if active else 0.0:+.2f}")
        self.status.setText(state)
        if not self.args.http_viewer_only:
            auto_status = self.node.auto_dock_status
            self.log_auto_dock_status()
            self.auto_dock_status_label.setText(
                f"AUTO-DOCK 상태: {auto_status.get('state', '?')} | "
                f"{auto_status.get('reason', '?')}"
            )
            fork_state = self.node.fork_state
            self.fork_flow_status_label.setText(
                f"Fork: {fork_state.get('state', '?')} | "
                f"drive_ready {self.node.drive_ready_count}회"
            )
        self.update_battery_status()
        self.update_entity_map()
        self.update_dock_slot_grid()
        self.update_frame()

    def update_entity_map(self):
        sequences = (
            getattr(self.node, "map_sequence", 0),
            getattr(self.node, "entity_map_sequence", 0),
        )
        if sequences == self.last_map_sequence:
            return
        self.last_map_sequence = sequences
        self.entity_map_view.set_data(
            getattr(self.node, "map_image", None),
            getattr(self.node, "map_metadata", None),
            getattr(self.node, "entity_map", None),
        )

    def update_dock_slot_grid(self):
        inventory = self.node.latest_dock_inventory or {}
        visible = inventory.get("visible_nearest") or []
        signature = (
            inventory.get("revision"),
            tuple(sorted(
                str(slot.get("slot_id")) for slot in visible
                if isinstance(slot, dict)
            )),
        )
        if signature == self.dock_grid_last_signature:
            return
        self.dock_grid_last_signature = signature
        revision = inventory.get("revision")
        if revision != self.dock_grid_revision:
            self.dock_grid_revision = revision
            self.dock_grid_occupied = {}
        current_slots = set()
        for slot in visible:
            if not isinstance(slot, dict):
                continue
            slot_id = str(slot.get("slot_id", "")).upper()
            if slot_id not in self.dock_slot_cells:
                continue
            current_slots.add(slot_id)
            self.dock_grid_occupied[slot_id] = dict(slot)
        for slot_id, cell in self.dock_slot_cells.items():
            slot = self.dock_grid_occupied.get(slot_id)
            if slot is None:
                cell.setText("·")
                cell.setStyleSheet(
                    "background:#262626;color:#888;border:1px solid #555;"
                    "border-radius:2px;"
                )
                continue
            fresh = str(slot.get("product_type", "")).upper() == "FRESH"
            background = "#a86600" if fresh else "#167447"
            border = "#7de8ff" if slot_id in current_slots else "#a0a0a0"
            cell.setText("별" if fresh else "있음")
            cell.setStyleSheet(
                f"background:{background};color:white;border:2px solid {border};"
                "border-radius:2px;font-weight:bold;"
            )
            cell.setToolTip(
                f"{slot_id} | {slot.get('product_type', '?')} | "
                f"{slot.get('distance_cm', '?')} cm"
            )
        age = time.monotonic() - self.node.latest_dock_inventory_monotonic
        self.dock_grid_status.setText(
            f"revision {revision if revision is not None else '-'} | "
            f"확인 {len(self.dock_grid_occupied)}/24 | "
            f"현재 화면 {len(current_slots)} | age {age:.1f}s"
        )

    def reset_dock_slot_grid(self):
        self.node.publish_dock_inventory_reset()
        self.dock_grid_status.setText("슬롯 초기화 요청 전송")

    def update_frame(self):
        sequences = (
            self.node.frame_sequence,
            self.node.secondary_frame_sequence,
            self.node.tertiary_frame_sequence,
        )
        if sequences == self.last_displayed_sequence:
            return
        frame = None if self.node.frame is None else self.node.frame.copy()
        secondary = None if self.node.secondary_frame is None else self.node.secondary_frame.copy()
        tertiary = None if self.node.tertiary_frame is None else self.node.tertiary_frame.copy()
        self.last_displayed_sequence = sequences
        display_frame = self.render_recording_overlay(frame)
        if display_frame is not None and self.writer is not None:
            recording_frame = self.compose_recording_frame(
                display_frame, secondary, tertiary
            )
            self.writer.write(recording_frame)
            self.recorded_frames += 1
            self.write_frame_record()
            self.record_label.setText(f"REC {self.recorded_frames} frames → {self.record_path}")
        self.show_frame(display_frame, self.video)

    def render_recording_overlay(self, frame):
        if frame is None:
            return None
        frame = self.render_warning_tape_overlay(frame)
        inventory = self.node.latest_dock_inventory or {}
        inventory_age = time.monotonic() - self.node.latest_dock_inventory_monotonic
        inventory_fresh = bool(inventory) and inventory_age <= 2.0
        # The standard control_gui source is YOLO's port-8090 MJPEG stream,
        # which already contains its detection boxes.  Only ROS image input is
        # raw and needs the client-side detection overlay.
        if not self.args.primary_video_url:
            # Always show the raw YOLO detections.  Inventory contains only
            # complete four-tag pallet entities, so replacing this overlay
            # with inventory hid valid single-star FRESH detections.
            frame = self.render_detection_overlay(frame)
        if not inventory_fresh:
            return frame
        for side, marker in (inventory.get("markers") or {}).items():
            if not isinstance(marker, dict):
                continue
            line = marker.get("line")
            box = marker.get("box")
            if (
                isinstance(line, (list, tuple)) and len(line) == 2
                and all(isinstance(point, (list, tuple)) and len(point) == 2
                        for point in line)
            ):
                start = tuple(int(round(value)) for value in line[0])
                end = tuple(int(round(value)) for value in line[1])
                cv2.line(frame, start, end, (0, 0, 255), 4)
                cv2.circle(frame, end, 7, (0, 0, 255), -1)
                label_x, label_y = start
            elif isinstance(box, (list, tuple)) and len(box) == 4:
                x1, y1, x2, y2 = (int(round(value)) for value in box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                label_x, label_y = x1, y1
            else:
                continue
            cv2.putText(
                frame, f"DOCK {str(side).upper()} END",
                (label_x, max(18, label_y - 7)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 255), 2,
            )
        entities = {
            item.get("entity_id"): item
            for item in (self.node.latest_detection or {}).get("entities", [])
            if isinstance(item, dict) and item.get("entity_id") is not None
        }
        visible = inventory.get("visible_nearest") or []
        for slot in visible:
            entity = entities.get(slot.get("entity_id")) if isinstance(slot, dict) else None
            box = entity.get("image_pallet_box") if entity else None
            if not isinstance(box, (list, tuple)) or len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(round(value)) for value in box)
            label = (
                f"{slot.get('slot_id', '?')} {slot.get('product_type', '?')} "
                f"{slot.get('distance_cm', '?')}cm"
            )
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 120, 0), 3)
            cv2.putText(
                frame, label, (x1, min(frame.shape[0] - 8, y2 + 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 120, 0), 2,
            )
        summary = (
            f"DOCK inventory | R:{int(bool(inventory.get('right_end_detected')))} "
            f"L:{int(bool(inventory.get('left_end_detected')))} "
            f"slots:{len(visible)} rev:{inventory.get('revision', '?')}"
        )
        cv2.rectangle(frame, (5, 34), (min(frame.shape[1] - 5, 520), 62), (0, 0, 0), -1)
        cv2.putText(
            frame, summary, (10, 55), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 180, 255), 2,
        )
        return frame

    def resolve_warning_tape_hsv_path(self):
        configured = str(self.args.warning_tape_hsv_config or "").strip()
        if configured:
            return Path(configured).expanduser()
        pose_config_path = Path(str(self.args.pose_config or "")).expanduser()
        try:
            pose_config = json.loads(pose_config_path.read_text(encoding="utf-8"))
            configured = str(
                pose_config.get("warning_tape_hsv_config_path", "")
            ).strip()
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            configured = ""
        return Path(configured or "/home/ubuntu/warning_tape_hsv.json").expanduser()

    def warning_tape_filter_values(self):
        """Reload the same GUI-authored HSV config consumed by Auto Dock."""
        try:
            mtime_ns = self.warning_tape_hsv_path.stat().st_mtime_ns
        except OSError:
            return None
        if (
            self.warning_tape_hsv_cache is not None
            and self.warning_tape_hsv_cache_mtime_ns == mtime_ns
        ):
            return self.warning_tape_hsv_cache
        try:
            payload = json.loads(
                self.warning_tape_hsv_path.read_text(encoding="utf-8")
            )
            if not isinstance(payload, dict):
                return None
        except (OSError, ValueError, json.JSONDecodeError):
            return None
        payload.setdefault("min_component_pixels", 200)
        self.warning_tape_hsv_cache = payload
        self.warning_tape_hsv_cache_mtime_ns = mtime_ns
        return self.warning_tape_hsv_cache

    def render_warning_tape_overlay(self, frame):
        """Show exactly which yellow pixels pass the auto-dock tape gates."""
        now = time.monotonic()
        tape_age = now - self.node.tape_frame_monotonic
        if self.node.tape_frame is None or tape_age > 0.75:
            self.warning_tape_track = None
            self.warning_tape_track_at = 0.0
            self.warning_tape_debug = {
                "detected": False,
                "reason": "raw_frame_stale",
                "source_age_sec": None if self.node.tape_frame is None else tape_age,
            }
            self.render_auto_dock_status_banner(frame)
            return frame
        tape_frame = self.node.tape_frame.copy()
        if tape_frame.shape[:2] != frame.shape[:2]:
            tape_frame = cv2.resize(
                tape_frame, (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_AREA,
            )
        location = str(
            (self.node.auto_dock_status or {}).get("location", "")
        ).strip().upper()
        is_y_slot = location in {"Y1", "Y2", "Y3", "Y4"}
        auto_state = str(
            (self.node.auto_dock_status or {}).get("state", "")
        ).strip().upper()
        if is_y_slot and auto_state in {"IDLE", "READY", "ERROR"}:
            self.warning_tape_track = None
            self.warning_tape_track_at = 0.0
        track_age = now - self.warning_tape_track_at
        tracked_minimum = None
        if (
            self.warning_tape_track is not None
            and (is_y_slot or track_age <= 0.75)
        ):
            tracked_minimum = max(
                0.0, float(self.warning_tape_track["center_y_ratio"]) - 0.12
            )
        else:
            self.warning_tape_track = None
        filter_config = self.warning_tape_filter_values()
        if is_y_slot and auto_state == "ALIGNING":
            debug = detect_y_slot_x_debug(
                tape_frame, filter_config=filter_config,
            )
        elif is_y_slot:
            values = filter_config if isinstance(filter_config, dict) else {}
            roi_top = int(round(max(0.0, min(
                0.95, float(values.get("roi_top_ratio", 0.70))
            )) * tape_frame.shape[0]))
            accepted_mask = np.zeros(
                (tape_frame.shape[0] - roi_top, tape_frame.shape[1]),
                dtype=np.uint8,
            )
            if self.warning_tape_track is not None:
                debug = dict(self.warning_tape_track)
                debug.update({
                    "detected": True,
                    "reason": "insertion_latched",
                    "roi_top": roi_top,
                    "accepted_mask": accepted_mask,
                    "line": None,
                    "x_lines": None,
                    "border_lines": None,
                })
            else:
                debug = {
                    "detected": False,
                    "reason": "x_detection_inactive",
                    "roi_top": roi_top,
                    "raw_yellow_pixels": 0,
                    "yellow_pixels": 0,
                    "component_count": 0,
                    "black_adjacent_components": 0,
                    "black_adjacent_candidates": 0,
                    "span_px": 0,
                    "x_min_px": None,
                    "x_max_px": None,
                    "center_x_px": None,
                    "accepted_mask": accepted_mask,
                    "line": None,
                }
        else:
            debug = detect_warning_tape_debug(
                tape_frame, minimum_center_y_ratio=tracked_minimum,
                filter_config=filter_config,
            )
        accepted = debug.pop("accepted_mask")
        line = debug.pop("line", None)
        x_lines = debug.pop("x_lines", None)
        border_lines = debug.pop("border_lines", None)
        if (
            is_y_slot
            and not debug["detected"]
            and self.warning_tape_track is not None
        ):
            debug.update(self.warning_tape_track)
            debug["detected"] = True
            debug["reason"] = "latched"
        if (
            not is_y_slot
            and debug["detected"]
            and self.warning_tape_track is not None
        ):
            angle_delta = math.degrees(math.atan2(
                math.sin(math.radians(
                    float(debug["angle_deg"])
                    - float(self.warning_tape_track["angle_deg"])
                )),
                math.cos(math.radians(
                    float(debug["angle_deg"])
                    - float(self.warning_tape_track["angle_deg"])
                )),
            ))
            center_delta = (
                float(debug["center_y_ratio"])
                - float(self.warning_tape_track["center_y_ratio"])
            )
            if abs(angle_delta) > 8.0 or abs(center_delta) > 0.10:
                debug["detected"] = False
                debug["reason"] = "tracked_pose_jump"
                line = None
        if debug["detected"]:
            self.warning_tape_track = dict(debug)
            self.warning_tape_track_at = now
        debug["source_age_sec"] = tape_age
        debug["source_topic"] = self.args.tape_image_topic
        debug["hsv_config_path"] = str(self.warning_tape_hsv_path)
        debug["hsv_config_loaded"] = filter_config is not None
        self.warning_tape_debug = dict(debug)
        roi_top = int(debug["roi_top"])
        cv2.line(frame, (0, roi_top), (frame.shape[1] - 1, roi_top),
                 (255, 120, 0), 1)
        contours, _hierarchy = cv2.findContours(
            accepted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        shifted = [contour + np.asarray([[[0, roi_top]]]) for contour in contours]
        cv2.drawContours(frame, shifted, -1, (255, 0, 255), 2)
        if debug["detected"]:
            if line is not None:
                vx, vy, line_x, line_y = line
                left_y = int(round(
                    line_y + (vy / vx) * (0.0 - line_x) + roi_top
                ))
                right_x = frame.shape[1] - 1
                right_y = int(round(
                    line_y + (vy / vx) * (right_x - line_x) + roi_top
                ))
                cv2.line(
                    frame, (0, left_y), (right_x, right_y), (0, 255, 255), 3
                )
            if x_lines is not None:
                for x1, y1, x2, y2 in x_lines:
                    cv2.line(
                        frame, (int(x1), int(y1 + roi_top)),
                        (int(x2), int(y2 + roi_top)), (0, 255, 255), 3,
                    )
            if border_lines is not None:
                for x1, y1, x2, y2 in border_lines:
                    cv2.line(
                        frame, (int(x1), int(y1 + roi_top)),
                        (int(x2), int(y2 + roi_top)), (0, 165, 255), 2,
                    )
            center = (
                int(round(float(debug.get(
                    "center_x_px", frame.shape[1] * 0.5
                )))),
                int(round(float(debug["center_y_ratio"]) * frame.shape[0])),
            )
            cv2.circle(frame, center, 7, (0, 255, 255), -1)
        self.render_auto_dock_status_banner(frame)
        if location in {"Y1", "Y2", "Y3", "Y4"}:
            target_center_ratio = float(getattr(
                self.args, "y_slot_target_center_x_ratio", 0.5
            ))
            target_center_x = target_center_ratio * frame.shape[1]
            center_x = debug.get("center_x_px") if debug["detected"] else None
            cv2.line(
                frame, (int(round(target_center_x)), 0),
                (int(round(target_center_x)), frame.shape[0] - 1),
                (255, 255, 0), 1,
            )
            if center_x is None:
                center_text = f"{location} SLOT X: tape detection waiting"
                self.y_slot_center_label.setText(
                    f"{location} 슬롯 X 중심: 주의선 검출 대기"
                )
            else:
                offset_px = float(center_x) - target_center_x
                cv2.line(
                    frame, (int(round(center_x)), 0),
                    (int(round(center_x)), frame.shape[0] - 1),
                    (255, 0, 255), 2,
                )
                center_text = (
                    f"{location} SLOT X {center_x:.1f}px | "
                    f"CENTER DELTA {offset_px:+.1f}px"
                )
                self.y_slot_center_label.setText(
                    f"{location} 슬롯 X 중심: {center_x:.1f}px | "
                    f"차량 직진축 {target_center_x:.1f}px 대비 "
                    f"{offset_px:+.1f}px (오른쪽 +)"
                )
            cv2.rectangle(
                frame, (4, frame.shape[0] - 33),
                (min(frame.shape[1] - 4, 500), frame.shape[0] - 4),
                (0, 0, 0), -1,
            )
            cv2.putText(
                frame, center_text, (9, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 0), 2,
            )
        else:
            self.y_slot_center_label.setText("Y 슬롯 X 중심: arrival 대기")
        return frame

    def render_auto_dock_status_banner(self, frame):
        status = self.node.auto_dock_status or {}
        state = str(status.get("state", "UNKNOWN")).strip().upper() or "UNKNOWN"
        reason = str(status.get("reason", "no_status")).strip() or "no_status"
        text_line = f"AUTO {state} | {reason}"
        color = {
            "SEARCHING": (0, 220, 255),
            "ALIGNING": (255, 200, 0),
            "INSERTING": (0, 220, 0),
            "READY": (0, 220, 0),
            "ERROR": (0, 0, 255),
        }.get(state, (220, 220, 220))
        text_width = min(frame.shape[1] - 6, max(280, len(text_line) * 9))
        cv2.rectangle(frame, (4, 4), (text_width, 31), (0, 0, 0), -1)
        cv2.putText(
            frame, text_line, (9, 24), cv2.FONT_HERSHEY_SIMPLEX,
            0.52, color, 2,
        )

    def render_inventory_entity_overlay(self, frame):
        """Draw only complete pallet groups used by the inventory tracker."""
        detection = self.node.latest_detection or {}
        if time.monotonic() - self.node.latest_detection_monotonic > 1.0:
            return frame
        for entity in detection.get("entities", []):
            if not isinstance(entity, dict):
                continue
            box = entity.get("image_pallet_box")
            if not isinstance(box, (list, tuple)) or len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(round(value)) for value in box)
            matrix = "/".join(str(value)[:2].upper() for value in entity.get("matrix", []))
            measurement = entity.get("depth_yaw") or entity.get("pnp") or {}
            distance = measurement.get("forward_distance_cm")
            distance_text = "" if distance is None else f" {float(distance):.1f}cm"
            label = f"PALLET {entity.get('entity_id', '?')} {matrix}{distance_text}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 220, 0), 2)
            cv2.putText(
                frame, label, (x1, max(18, y1 - 7)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 220, 0), 2,
            )
        return frame

    def write_frame_record(self):
        if self.frame_log_file is None:
            return
        now = time.monotonic()
        record = {
            "type": "video_frame",
            "frame_index": self.recorded_frames,
            "time": time.time(),
            "monotonic": now,
            "primary_frame_sequence": self.node.frame_sequence,
            "primary_frame_age_sec": now - self.node.last_frame_monotonic,
            "detection_age_sec": (
                None if self.node.latest_detection is None
                else now - self.node.latest_detection_monotonic
            ),
            "detection": self.node.latest_detection,
            "dock_inventory_age_sec": (
                None if self.node.latest_dock_inventory is None
                else now - self.node.latest_dock_inventory_monotonic
            ),
            "dock_inventory": self.node.latest_dock_inventory,
            "auto_dock_status": self.node.auto_dock_status,
            "warning_tape": self.warning_tape_debug,
        }
        try:
            self.frame_log_file.write(
                json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
            )
        except (OSError, TypeError, ValueError) as exc:
            self.frame_log_file.close()
            self.frame_log_file = None
            self.record_label.setText(f"REC frame JSON stopped: {exc}")

    def render_detection_overlay(self, frame):
        """Draw the existing detection JSON over the original camera frame."""
        if frame is None:
            return None
        detection = self.node.latest_detection or {}
        if time.monotonic() - self.node.latest_detection_monotonic > 1.0:
            return frame
        for item in detection.get("detections", []):
            box = item.get("box") or []
            if len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(round(value)) for value in box)
            label = str(item.get("class", "?"))
            confidence = float(item.get("confidence", 0.0))
            color = DETECTION_COLORS.get(label, (0, 220, 0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, f"{label} {confidence:.2f}", (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2,
            )
        candidate = detection.get("candidate") or {}
        for point in candidate.get("tag_centers", []):
            if len(point) == 2:
                cv2.circle(
                    frame, (int(round(point[0])), int(round(point[1]))),
                    5, (0, 255, 255), -1,
                )
        return self.render_inventory_entity_overlay(frame)

    @staticmethod
    def show_frame(frame, label):
        if frame is None:
            return
        height, width = frame.shape[:2]
        scale = min(label.width() / width, label.height() / height, 1.0)
        if scale < 1.0:
            frame = cv2.resize(
                frame,
                (max(1, round(width * scale)), max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        image = QImage(rgb.data, width, height, channels * width, QImage.Format_RGB888).copy()
        label.set_frame(image)

    def compose_recording_frame(self, primary, secondary, tertiary):
        if self.args.disable_external_webcams:
            return primary
        height, width = primary.shape[:2]
        frames = [primary]
        for name, external in (("WEBCAM 1", secondary), ("WEBCAM 2", tertiary)):
            if external is None:
                external = np.zeros_like(primary)
                cv2.putText(external, f"{name} OFFLINE", (20, height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                external = cv2.resize(external, (width, height), interpolation=cv2.INTER_AREA)
            frames.append(external)
        return np.hstack(frames)

    def capture_snapshot(self):
        if self.node.frame is None:
            self.capture_label.setText("실패: 차량 카메라 화면 없음")
            return

        capture_dir = Path(self.args.output_dir) / "captures"
        capture_dir.mkdir(parents=True, exist_ok=True)
        captured_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        stem = time.strftime("capture_%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
        image_path = capture_dir / f"{stem}.jpg"
        metadata_path = capture_dir / f"{stem}.json"
        frame = self.node.frame.copy()

        if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            self.capture_label.setText("실패: 이미지 저장 오류")
            return

        metadata = {
            "vehicle": self.args.vehicle,
            "captured_at": captured_at,
            "image": image_path.name,
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "detection": self.node.latest_detection,
            "note": self.memo.toPlainText().strip(),
        }
        try:
            metadata_path.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            self.capture_label.setText(f"이미지만 저장됨: {exc}")
            return

        self.memo.clear()
        self.capture_label.setText(f"저장됨: {image_path.name} + JSON")


    def toggle_recording(self, enabled):
        if not enabled:
            self.finish_recording()
            return
        if self.node.frame is None:
            self.record.setChecked(False)
            self.record_label.setText("Cannot record: no camera frame")
            return
        output = Path(self.args.output_dir)
        output.mkdir(parents=True, exist_ok=True)
        self.record_path = output / time.strftime("teleop_%Y%m%d_%H%M%S.mp4")
        sample = self.compose_recording_frame(
            self.node.frame, self.node.secondary_frame, self.node.tertiary_frame
        )
        height, width = sample.shape[:2]
        self.writer = cv2.VideoWriter(
            str(self.record_path), cv2.VideoWriter_fourcc(*"mp4v"), self.args.record_fps, (width, height)
        )
        if not self.writer.isOpened():
            self.writer = None
            self.record.setChecked(False)
            self.record_label.setText("Failed to open video writer")
            return
        self.frame_log_path = self.record_path.with_suffix(".frames.jsonl")
        try:
            self.frame_log_file = self.frame_log_path.open(
                "w", encoding="utf-8", buffering=1
            )
        except OSError as exc:
            self.writer.release()
            self.writer = None
            self.frame_log_path = None
            self.record.setChecked(False)
            self.record_label.setText(f"Failed to open frame log: {exc}")
            return
        self.recorded_frames = 0
        self.telemetry_session = self.record_path.stem
        self.node.start_telemetry(self.telemetry_session)
        self.record.setText("Stop recording (R)")

    def finish_recording(self):
        telemetry_session = self.telemetry_session
        telemetry_path = None if self.record_path is None else self.record_path.with_suffix(".jsonl")
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.frame_log_file is not None:
            self.frame_log_file.flush()
            self.frame_log_file.close()
            self.frame_log_file = None
        telemetry_ok = False
        if telemetry_session is not None and telemetry_path is not None:
            telemetry_ok = self.node.finish_telemetry(telemetry_session, telemetry_path)
            self.telemetry_session = None
        self.record.setText("Start recording (R)")
        if self.record_path is not None:
            frame_suffix = (
                "" if self.frame_log_path is None else f" + {self.frame_log_path.name}"
            )
            suffix = (
                f" + {telemetry_path.name}" if telemetry_ok
                else " (telemetry download failed)"
            )
            suffix += frame_suffix
            self.record_label.setText(f"Saved: {self.record_path}{suffix}")

    def closeEvent(self, event):
        self.cancel_movement_key_releases()
        self.pressed.clear()
        self.node.stop(repeats=5)
        self.node.publish_fork("STOP")
        self.finish_recording()
        if self.auto_dock_log_file is not None:
            self.auto_dock_log_file.close()
            self.auto_dock_log_file = None
        event.accept()



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", type=int, choices=(1, 2), required=True)
    parser.add_argument(
        "--ros-domain-id", type=int, default=None,
        help="ROS domain to use; defaults to the selected vehicle number.",
    )
    parser.add_argument("--webcam-ip", default="210.220.0.12")
    parser.add_argument("--image-topic", default="")
    parser.add_argument(
        "--tape-image-topic",
        default="/ascamera/camera_publisher/rgb0/image",
    )
    parser.add_argument("--secondary-image-topic", default="")
    parser.add_argument("--secondary-video-url", default="")
    parser.add_argument("--primary-video-url", default="")
    parser.add_argument("--primary-video-command", default="")
    parser.add_argument("--control-host", default="")
    parser.add_argument("--control-port", type=int, default=8091)
    parser.add_argument("--control-url", default="")
    parser.add_argument("--control-command", default="")
    parser.add_argument("--webcam-1-video-url", default="")
    parser.add_argument("--webcam-2-video-url", default="")
    parser.add_argument("--scan-topic", default="")
    parser.add_argument("--odom-topic", help=argparse.SUPPRESS)
    parser.add_argument("--motor-command-topic", default="")
    parser.add_argument("--battery-topic", default="")
    parser.add_argument("--detection-topic", default="")
    parser.add_argument("--cmd-vel-topic", default="")
    parser.add_argument("--fork-command-topic", default="")
    parser.add_argument("--arrival-topic", default="")
    parser.add_argument("--auto-dock-stop-topic", default="")
    parser.add_argument("--auto-dock-status-topic", default="")
    parser.add_argument("--dock-inventory-topic", default="")
    parser.add_argument("--dock-inventory-reset-topic", default="")
    parser.add_argument("--fork-state-topic", default="")
    parser.add_argument("--drive-ready-topic", default="")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--entity-map-topic", default="")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--linear-speed", type=float, default=0.12)
    parser.add_argument("--angular-speed", type=float, default=0.35)
    parser.add_argument("--pose-config", default="")
    parser.add_argument("--warning-tape-hsv-config", default="")
    parser.add_argument("--stop-distance", type=float, default=0.20)
    parser.add_argument("--safety-min-valid-range", type=float, default=0.25)
    parser.add_argument("--record-fps", type=float, default=15.0)
    parser.add_argument("--viewer-only", action="store_true")
    webcam_group = parser.add_mutually_exclusive_group()
    webcam_group.add_argument(
        "--disable-external-webcams",
        action="store_true",
        dest="disable_external_webcams",
    )
    webcam_group.add_argument(
        "--enable-external-webcams",
        action="store_false",
        dest="disable_external_webcams",
    )
    parser.set_defaults(disable_external_webcams=None)
    parser.add_argument("--http-viewer-only", action="store_true")
    args = parser.parse_args()
    explicit_image_topic = bool(args.image_topic)
    if not args.image_topic:
        args.image_topic = "/ascamera/camera_publisher/rgb0/image"
    if not args.scan_topic:
        args.scan_topic = "/scan_raw"
    if not args.motor_command_topic:
        args.motor_command_topic = "/ros_robot_controller/set_motor"
    if not args.battery_topic:
        args.battery_topic = "/ros_robot_controller/battery"
    if not args.detection_topic:
        args.detection_topic = "/symbol_seg/detections"
    if not args.cmd_vel_topic:
        args.cmd_vel_topic = "/controller/cmd_vel"
    if not args.fork_command_topic:
        args.fork_command_topic = "/fork/command"
    if not args.arrival_topic:
        args.arrival_topic = "/nav2/arrival"
    if not args.auto_dock_stop_topic:
        args.auto_dock_stop_topic = "/auto_dock/stop"
    if not args.auto_dock_status_topic:
        args.auto_dock_status_topic = "/auto_dock/status"
    if not args.dock_inventory_topic:
        args.dock_inventory_topic = "/dock/inventory"
    if not args.dock_inventory_reset_topic:
        args.dock_inventory_reset_topic = "/dock/inventory/reset"
    if not args.fork_state_topic:
        args.fork_state_topic = "/fork/state"
    if not args.drive_ready_topic:
        args.drive_ready_topic = "/auto_dock/drive_ready"
    if not args.entity_map_topic:
        args.entity_map_topic = "/tag_entity_map"
    if args.secondary_image_topic and (args.secondary_video_url or args.webcam_1_video_url):
        parser.error("use only one of --secondary-image-topic and --secondary-video-url")
    if args.secondary_video_url and args.webcam_1_video_url:
        parser.error("use only one of --secondary-video-url and --webcam-1-video-url")
    if args.secondary_video_url:
        args.webcam_1_video_url = args.secondary_video_url
    if not args.secondary_image_topic and not args.webcam_1_video_url:
        args.webcam_1_video_url = f"http://{args.webcam_ip}:5000/video/0"
    if not args.webcam_2_video_url:
        args.webcam_2_video_url = f"http://{args.webcam_ip}:5000/video/2"
    if not args.primary_video_url and not explicit_image_topic:
        args.primary_video_url = f"http://{VEHICLE_HOSTS[args.vehicle]}:8090/stream"
    if not args.control_host:
        args.control_host = VEHICLE_HOSTS[args.vehicle]
    if not args.output_dir:
        args.output_dir = str(Path.home() / "recordings" / f"vehicle{args.vehicle}")
    if not args.pose_config:
        candidates = (
            Path("/shared/vehicle_pose_config.json"),
            Path(__file__).resolve().parent.parent / "config" / "vehicle_pose_config.json",
        )
        args.pose_config = str(next(
            (path for path in candidates if path.exists()),
            Path(args.output_dir) / "vehicle_pose_config.json",
        ))
    pose_config_path = Path(args.pose_config)
    if pose_config_path.exists():
        try:
            pose_config = json.loads(pose_config_path.read_text(encoding="utf-8"))
            args.y_slot_target_center_x_ratio = float(
                pose_config.get("y_slot_target_center_x_ratio", 0.5)
            )
            if args.disable_external_webcams is None:
                args.disable_external_webcams = bool(
                    pose_config.get("disable_external_webcams", True)
                )
        except (OSError, TypeError, ValueError):
            parser.error(f"invalid pose config: {pose_config_path}")
    if args.disable_external_webcams is None:
        args.disable_external_webcams = True

    args.ros_domain_id = (
        214 + args.vehicle if args.ros_domain_id is None else args.ros_domain_id
    )
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    # Vehicle bringup currently uses normal Fast DDS discovery. Inheriting an
    # old/dead discovery-server setting isolates this GUI from every topic.
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    os.environ.pop("CYCLONEDDS_URI", None)

    if args.http_viewer_only:
        node = HttpViewerSource(args)
    else:
        rclpy.init()
        node = DevControlClientNode(args)
    app = QApplication([])
    window = TeleopWindow(node, args)
    app.installEventFilter(window)
    window.show()
    try:
        app.exec_()
    finally:
        node.secondary_stream_stop.set()
        for process in getattr(node, "child_processes", []):
            if process.poll() is None:
                process.terminate()
        for thread in node.external_stream_threads:
            thread.join(timeout=1.0)
        node.stop(repeats=5)
        node.publish_fork("STOP")
        if not args.http_viewer_only:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
